/*
 * gps.c
 *
 *  Created on: May 20, 2021
 *      Author: dhruv
 */

#include "gps.h"
#include "stm32f7xx_hal.h"

/** What UART interface the device is connected to */
#define GPS_UART_INTERFACE 1

/** Baudrate the gps communicates over by default */
#define GPS_UART_BAUDRATE 38400

/** How large to make the internal buffer for parsing messages. Should be the size
 of the largest message we'll receive from the gps*/
#define GPS_UART_BUFFER_SIZE 800

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
static const uint8_t* PMTK_SET_NMEA_UPDATE_1HZ = "$PMTK220,1000*1F\r\n";
static const uint8_t* PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C\r\n";
static const uint8_t* PMTK_SET_NMEA_UPDATE_10HZ = "$PMTK220,100*2F\r\n";

// Position fix update rate commands.
static const uint8_t* PMTK_API_SET_FIX_CTL_1HZ = "$PMTK100,1000,0,0,0,0*1C\r\n";
static const uint8_t* PMTK_API_SET_FIX_CTL_5HZ = "$PMTK100,200,0,0,0,0*2F\r\n";
// Can't fix position faster than 5 times a second!

static const uint8_t* PMTK_SET_BAUD_57600 = "$PMTK251,57600*2C\r\n";
static const uint8_t* PMTK_SET_BAUD_9600 = "$PMTK251,9600*17\r\n";

//turn on VTG (velocity info) and GGA (positional info) only
static const uint8_t* PMTK_SET_NMEA_OUTPUT_GGAVTG = "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
// turn on ALL THE DATA
static const uint8_t* PMTK_SET_NMEA_OUTPUT_ALLDATA = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
// turn off output
static const uint8_t* PMTK_SET_NMEA_OUTPUT_OFF = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

// one of the 2 DGPS modes
static const uint8_t* PMTK_ENABLE_SBAS = "$PMTK313,1*2E\r\n";
static const uint8_t* PMTK_ENABLE_WAAS = "$PMTK301,2*2E\r\n";

// ask for the release and version
static const uint8_t* PMTK_Q_RELEASE = "$PMTK605*31\r\n";

// request for updates on antenna status
static const uint8_t* PGCMD_ANTENNA = "$PGCMD,33,1*6C\r\n";
static const uint8_t* PGCMD_NOANTENNA = "$PGCMD,33,0*6D\r\n";

// https://nmeachecksum.eqth.net --> get checksums
//static const uint8_t* PUBX_CONFIG_NMEA =  "$PUBX,41,1,07,03,38400,0*20\r\n"; //Change hex numbers later
static const uint8_t* PUBX_CONFIG_NMEA =  "$PUBX,41,1,07,03,9600,0*10\r\n"; //Change hex numbers later
static const uint8_t* PUBX_SET_GGA = "$PUBX,40,GGA,0,1,0,0,0,0*5B\r\n";
static const uint8_t* PUBX_SET_VTG = "$PUBX,40,VTG,0,1,0,0,0,0*5F\r\n";
static const uint8_t* PUBX_SET_RMC_OFF = "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n";
static const uint8_t* PUBX_SET_GSA_OFF = "$PUBX,40,GSA,0,0,0,0,0,0*4A\r\n";
static const uint8_t* PUBX_SET_GNS_OFF = "$PUBX,40,GNS,0,0,0,0,0,0*41\r\n";
static const uint8_t* PUBX_SET_GLL_OFF = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";

static const uint8_t* GPS_GGA_MESSAGE = "GNGGA";
static const uint8_t* GPS_VTG_MESSAGE = "GNVTG";

GPSData gps_data;

extern UART_HandleTypeDef huart4;

static bool data_available = false;
static bool new_vtg_data = false;
static bool new_gga_data = false;
static bool configured = false; //if the gps module has been initialized and configured

static uint8_t gga_buffer[GPS_UART_BUFFER_SIZE]; //buffer for pasing gga (positional packets)
static uint8_t vtg_buffer[GPS_UART_BUFFER_SIZE]; //buffer for parsing vtg packets (velocity packets)
static uint8_t uart_buffer[GPS_UART_BUFFER_SIZE]; //buffer for parsing vtg packets (velocity packets)
static uint8_t a[GPS_UART_BUFFER_SIZE];

static void parseGGA(uint8_t* data);
static void parseVTG(uint8_t* data);

static uint8_t byteToHexString(unsigned int checkSumHalf) {
    uint8_t uint8_tOut = 0;

    if (checkSumHalf >= 0 && checkSumHalf <= 9){
        uint8_tOut = checkSumHalf + 0x30;
    }
    else if (checkSumHalf >= 0xA && checkSumHalf <= 0xF){
        uint8_tOut = checkSumHalf + 0x37;
    }
    return uint8_tOut;
}

static uint8_t asciiToHex(uint8_t asciiSymbol) {
    uint8_t hexOut = 0;
    if (asciiSymbol == 0x2E)
        hexOut = 0x10;
    else if (asciiSymbol >= 0x30 && asciiSymbol <= 0x39){
        hexOut = asciiSymbol - 0x30;
    }
    else if (asciiSymbol >= 0x41 && asciiSymbol <= 0x46){
        hexOut = asciiSymbol - 0x37; //Letter "F"(ASCII 0x46) becomes 0xF
    }
    return hexOut;
}

/**
 * Given an NMEA string starting after the $, verifies the integrity of the stirng
 * using the checksum
 * @param string
 * @return True if string is a valid gps string, false otherwise
 */
static bool isNMEAChecksumValid(uint8_t* string){
    uint16_t i = 0;
    uint8_t checksum = 0;

    while(string[i] != '*'){
        checksum ^= string[i];
        i++;
    }
    i++;

    return byteToHexString((checksum & 0xF0) >> 4) == string[i] &&  byteToHexString(checksum & 0x0F) == string[i+1];
}

GPSData getData() {

	GPSData toReturn = gps_data;
	gps_data.ggaDataIsNew = false;
	gps_data.vtgDataIsNew = false;
	gps_data.dataIsNew = false;
	gps_data.timeIsNew = false;

	return toReturn;
}

void parseIncomingGPSData(){

	static bool currently_parsing = false;
	static uint16_t buffer_index = 0;
	int b = 0;
	int c = 0;

	for (int i = 0; i < GPS_UART_BUFFER_SIZE; i++) {
		if (a[i] == '$') { //Beginning of Packet
			currently_parsing = true;
			buffer_index = 0;
		} else if (a[i] == '\r') { //End of Packet
			 if (strncmp(GPS_GGA_MESSAGE, uart_buffer, 5) == 0){
				 memcpy(gga_buffer, uart_buffer, GPS_UART_BUFFER_SIZE);
				 new_gga_data = true;
				 b += 10;
			 } else if (strncmp(GPS_VTG_MESSAGE, uart_buffer, 5) == 0){
				memcpy(vtg_buffer, uart_buffer, GPS_UART_BUFFER_SIZE);
				new_vtg_data = true;
				c += 20;
			 } else {
//				 debug("Received NMEA that was neither GPVTG or GPGGA!");
			 }
			 currently_parsing = false;
		} else if (currently_parsing){
			uart_buffer[buffer_index] = a[i];
			buffer_index = (buffer_index + 1) % GPS_UART_BUFFER_SIZE; //make sure we dont cause a memory fault here
		}
	}

    if (!new_gga_data && !new_vtg_data){ //if no data has been copied over
        return;
    }

    if (new_gga_data){
        new_gga_data = false;
        if (isNMEAChecksumValid(gga_buffer)){
            data_available = false;
            parseGGA(gga_buffer);
            data_available = true;
            gps_data.dataIsNew = true;
            gps_data.timeIsNew = true;
            gps_data.ggaDataIsNew = true;
            gps_data.sensorStatus = 0;
        } else {
        	gps_data.sensorStatus = 1;
//            debug("Failed checksum when parsing a GPGGA (positional) packet!");
//            debug(gga_buffer);
        }
    }

    if (new_vtg_data){
        new_vtg_data = false;
        if (isNMEAChecksumValid(vtg_buffer)){
            data_available = false;
            parseVTG(vtg_buffer);
            data_available = true;
            gps_data.dataIsNew = true;
            gps_data.sensorStatus = 0;
            gps_data.vtgDataIsNew = true;
        } else {
        	gps_data.sensorStatus = 1;
//            debug("Failed checksum when parsing a GPVTG (velocity) packet!");
//            debug(vtg_buffer);
        }
    }
}

void init() {

	const uint8_t CFG_NMEA[16] = { 0x17, 0x20, 0b00011000, 0x40, 0x08, 0x01, 0x00, 0x00, 0x00, 0b01110110, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00 };
	HAL_UART_Transmit_DMA(&huart4, CFG_NMEA, sizeof(CFG_NMEA));

//	HAL_UART_Transmit_DMA(&huart4, PUBX_CONFIG_NMEA, sizeof(PUBX_CONFIG_NMEA));

//	const uint8_t CFG_UART[21] = { 0x06, 0x00, 0x01, 0x00, 0b00000000, 0b00000000, 0x00, 0x00, 0b11000000, 0b00001000, 0x00, 0x01, 0b11000010, 0x00, 0x00, 0b00000010, 0x00, 0b00000010, 0x00, 0x00, 0x00 };
//	HAL_UART_Transmit_DMA(&huart4, CFG_UART, sizeof(CFG_UART));

	HAL_UART_Transmit_DMA(&huart4, PMTK_SET_NMEA_UPDATE_10HZ, sizeof(PMTK_SET_NMEA_UPDATE_10HZ));

	HAL_UART_Transmit_DMA(&huart4, PMTK_API_SET_FIX_CTL_5HZ, sizeof(PMTK_API_SET_FIX_CTL_5HZ));

	HAL_UART_Transmit_DMA(&huart4, PMTK_ENABLE_WAAS, sizeof(PMTK_ENABLE_WAAS));

	configured = true;

	HAL_UART_Receive_DMA(&huart4, a, GPS_UART_BUFFER_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	parseIncomingGPSData();

	HAL_UART_Receive_DMA(&huart4, a, GPS_UART_BUFFER_SIZE);
}

bool isNewDataAvailable(){
    if (data_available){
        data_available = false;
        return true;
    }
    return false;
}

static void parseVTG(uint8_t* data){

    //static so that we dont allocate these variables every time
    static uint8_t rawHeading[6] = {0, 0, 0, 0, 0, 0};
    static uint8_t rawGroundSpeed[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    int comma = 0;
    int i = 0;
    int j = 0;

    while (data[j] != '*') {
        uint8_t numData = asciiToHex(data[j]);
        if (data[j] == ',') {
            comma++;
            i = 0;
        }

        if (comma == 1 && (i != 0)) {
            rawHeading[i] = numData;
        } else if (comma == 7 && (i != 0)) {
            rawGroundSpeed[i] = numData;
        }

        i++;
        j++;
    }

    i = 1;
    long int multiplier = 10;
    int decimalPoint = 0;

    gps_data.heading = 0;
    float tHeading = 0;
    for (i = 1; i < 6; i++) //this code first generates an 5 digit decimal number
    {
        if (rawHeading[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            tHeading += (float) (rawHeading[i]*100000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 10000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.heading = (int)(tHeading / multiplier);

    //	//calculate speed - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    multiplier = 10;
    decimalPoint = 0;
    gps_data.groundSpeed = 0;
    for (i = 1; i < 7; i++) //this code first generates an 6 digit decimal number
    {
        if (rawGroundSpeed[i] == 0x10)//check for decimal point
        {
            decimalPoint = i;
        } else {
            gps_data.groundSpeed += (float) (rawGroundSpeed[i]*1000000 / multiplier);
            multiplier = multiplier * 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divdes it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.groundSpeed = gps_data.groundSpeed / multiplier;
}

/**
 * Parses a GGA type NEMA string and modifies the GPS data struct with the applicable
 * fields
 * @param data
 */
static void parseGGA(uint8_t* data){

    int comma = 0; //comma counting so that we know what header we're parsing for
    int i = 0; //index for the current position of the field value
    int j = 0; //7th uint8_tacter is where data will start. index for the byte index in the counter

    //statically initialize placeholders for values. Static to avoid reinitializations every time
    static uint8_t rawTime[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t rawLatitude[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t rawLongitude[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t rawSatellites[3] = {0, 0, 10};
    static uint8_t rawAltitude[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t latitudeNS = 0;
    static uint8_t longitudeEW = 0;
    static uint8_t positionFix = 0;

    while (data[j] != '*') {
        uint8_t numData = asciiToHex(data[j]);

        if (data[j] == ',') {
            comma++;
            i = 0;
        }

        if ((comma == 1) && (i != 0)) {
            rawTime[i] = numData;
        } else if ((comma == 2) && (i != 0)) {
            rawLatitude[i] = numData;

        } else if ((comma == 3) && (i != 0)) {
            latitudeNS = data[j];

        } else if ((comma == 4) && (i != 0)) {
            rawLongitude[i] = numData;

        } else if ((comma == 5) && (i != 0)) {
            longitudeEW = data[j];

        } else if ((comma == 6) && (i != 0)) {
            positionFix = numData;

        } else if ((comma == 7) && (i != 0)) {
            rawSatellites[i] = numData;
        } else  if ((comma == 9) && (i != 0)) {
            rawAltitude[i] = numData;
        }

        i++;
        j++;
    }

    //now we've got all the valid data placed in our buffers. Modify gps data struct to match

    //calculate time
    gps_data.utcTime = (float) rawTime[1] * 100000;
    gps_data.utcTime += (float) rawTime[2] * 10000;
    gps_data.utcTime += (float) rawTime[3] * 1000;
    gps_data.utcTime += (float) rawTime[4] * 100;
    gps_data.utcTime += (float) rawTime[5] * 10;
    gps_data.utcTime += (float) rawTime[6] * 1;
    //Decimal Point
    gps_data.utcTime += (float) rawTime[8] * 0.1;
    gps_data.utcTime += (float) rawTime[9] * 0.01;
    gps_data.utcTime += (float) rawTime[10] * 0.001;

    //calculate latitude
    gps_data.latitude = rawLatitude[3]*10.0;
    gps_data.latitude += rawLatitude[4]*1.0;
    gps_data.latitude += rawLatitude[6]*0.1;
    gps_data.latitude += rawLatitude[7]*0.01;
    gps_data.latitude += rawLatitude[8]*0.001;
    gps_data.latitude += rawLatitude[9]*0.0001;
    gps_data.latitude /= 60;  //Converts from dd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -90 to +90)
    gps_data.latitude += rawLatitude[1]*10.0;
    gps_data.latitude += rawLatitude[2]*1.0;

    if (latitudeNS == 'S'){
        gps_data.latitude *= -1;
    }

    //calculate longitude
    gps_data.longitude = rawLongitude[4]*10.0;
    gps_data.longitude += rawLongitude[5]*1.0;
    gps_data.longitude += rawLongitude[7]*0.1;
    gps_data.longitude += rawLongitude[8]*0.01;
    gps_data.longitude += rawLongitude[9]*0.001;
    gps_data.longitude += rawLongitude[10]*0.0001;
    gps_data.longitude /= 60;  //Converts from ddd.mmmmmm to decimal degrees. (60 minutes in a degree)
    //Then add the degrees (ranges from -180 to +180)
    gps_data.longitude += rawLongitude[1]*100.0;
    gps_data.longitude += rawLongitude[2]*10.0;
    gps_data.longitude += rawLongitude[3]*1.0;

    if (longitudeEW == 'W'){
        gps_data.longitude *= -1;
    }

    //calculate satellites
    if (rawSatellites[2] == 10) gps_data.numSatellites = rawSatellites[1];
    else gps_data.numSatellites = rawSatellites[1]*10 + rawSatellites[2];

    //calculate altitude - tricky because of unknown 1-3 digits preceeding the decimal
    i = 1;
    long int multiplier = 10;
    int decimalPoint = 0;
    gps_data.altitude = 0;
    float tAltitude = 0;
    for (i = 1; i < 8; i++) //this code first generates an 6 digit decimal number
    {
        if (rawAltitude[i] == 0x10) //check for decimal point
        {
            decimalPoint = i;
        } else {
            tAltitude += (float) (rawAltitude[i]*1000000 / multiplier);
            multiplier *= 10;
        }
    }
    decimalPoint = decimalPoint - 2;
    multiplier = 100000;
    while (decimalPoint > 0) //then divides it according to the placement of the decimal
    {
        multiplier = multiplier / 10;
        decimalPoint--;
    }
    gps_data.altitude = (int)(tAltitude / multiplier);
}

