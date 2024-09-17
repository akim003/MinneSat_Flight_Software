#ifndef DATA_FORMATTER_H
#define DATA_FORMATTER_H

// Arduino Library
#include <Arduino.h> // only need to include this in the header file

// Sensor Libraries
#include "EEPROM.h"

// Custom Libraries
#include "initialization.h"
#include "data_handler.h"

// IMU
extern float tiltX;
extern float tiltY;
extern float rotZ; 

// PITOT
extern float pitotVelocity;
extern float velocity;

// RTC
extern String CanSatTime;

// GPS
extern String gpsTime;
extern float altitude;
extern long latitude;
extern long longitude;
extern uint8_t numSatellites;

// BME
extern float temperatureBME;
extern float pressureBME;
extern float relativeAltitude;

// AUDIO BCN
void monotoneBCN();

// OTHER
extern float canSatVoltage;

// Data Formatting Functions
void formatIMU();
void formatPITOT();
void formatRTC();
void formatGPS();
void formatBME();
void saveSD(String packet);
void updateVoltage();

// Helper Functions
float roundToDecimalPlaces(float value, int decimalPlaces);

#endif