/*
 * gps.h
 *
 *  Created on: May 10, 2021
 *      Author: dhruv
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    long double latitude;  // 8 Bytes
    long double longitude; // 8 Bytes
    float utcTime;     // 4 Bytes. Time in seconds since 00:00 (midnight)
    float groundSpeed; // in m/s
    int altitude; // in m
    int16_t heading; // in degrees. Should be between 0-360 at all times, but using integer just in case
    uint8_t numSatellites;    // 1 Byte
	uint8_t fixStatus; //0 = No GPS, 1 = GPS fix, 2 = DGSP Fix, 3 = Estimated/Dead Recoking Fix

    uint8_t sensorStatus; // 0 = no fix, 1 = gps fix, 2 = differential gps fix (DGPS) (other codes are possible)
    bool dataIsNew; // true if data has been refreshed since the previous time GetResult was called, false otherwise.
	bool timeIsValid;

	//Added these so autopilot knows which data is new
	bool ggaDataIsNew; //Position, altitude, time, and number of satellites
	bool vtgDataIsNew; //Groundspeed and Heading

} GPSData;

void parseIncomingGPSData();

void init();

bool isNewDataAvailable();

GPSData getData();




#endif /* INC_GPS_H_ */
