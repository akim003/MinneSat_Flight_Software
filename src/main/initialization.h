#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <Arduino.h> // only need to include this in the header file

// Sensor Libraries
#include "ICM_20948.h"                 // IMU (ICM20948)
#include "ms4525do.h"                  // PITOT (MS4525DO) 
#include "SD.h"                        // SD
#include "SPI.h"                       // SD
#include "DS3231.h"                    // RTC (DS3231)
#include "Buzzer.h"                    // Audio BCN (E40A1-R1)
#include "SparkFun_u-blox_GNSS_v3.h"   // GPS (UBLOX SAM-M1-0Q)
#include "Adafruit_Sensor.h"           // BME (BME280)
#include "Adafruit_BME280.h"           // BME (BME280)
#include "PWMServo.h"                  // SERVO 
#include "XBee.h"                      // XBEE (XBEE 3 PRO)
#include "EEPROM.h"                      

// Custom Libraries

// MASTER SETUP
void setupCanSat();

// MASTER GLOBALS
extern bool prevStarted;
extern bool recoveryStartup;

// EEPROM ADRESSES
extern int StateAddy;           // integer for EEPROM address for state
extern int ApogeeAddy;          // integer for EEPROM address for apogee
extern int LastAltAddy;         // integer for EEPROM address for LastAltAddy
extern int BaseAltAddy;         // integer for EEPROM address for storing the surface pressure
extern int PacketNumAddy;       // integer for packet number address

// IMU FUNCTIONS
void setupIMU();
void printFormattedFloat();
void printScaledAGMT();
void IMUtest();

// IMU GLOBAL VARIABLES
extern ICM_20948_I2C canSatIMU;

// PITOT FUNCTIONS
void setupPITOT();

// PITOT GLOBAL VARIABLES
extern bfs::Ms4525do canSatPITOT;

// SD FUNCTIONS
void setupSD();

// RTC GLOBAL VARIABLES
extern DS3231 canSatRTC;
extern RTClib canSatDataRTC;

// RTC FUNCTIONS
void resetTimeRTC();
void syncTimeRTC(String newTimeRTC);
void parseTimeString(String timeString, int& hours, int& minutes, int& seconds);

// AUDIO BCN FUNCTIONS
void jingleBellsBCN();
void toggleBCN();

// AUDIO BCN GLOBAL VARIABLES
extern Buzzer canSatBCN;

// GPS FUNCTIONS
void setupGPS();

// GPS GLOBAL VARIABLES
extern SFE_UBLOX_GNSS canSatGPS;

// BME FUNCTIONS
void setupBME();
void tareAltitude();

// BME GLOBAL VARIABLES
extern Adafruit_BME280 canSatBME;
extern float initialAltitude;

// SERVO FUNCTIONS
void setupSERVOS();

// SERVO VARIABLES
extern PWMServo heatShieldServo;
extern PWMServo parachuteServo;

// XBEE FUNCTIONS
void setupXBEE();
void XBEEtestSendData();
void XBEEtestReceiveData();
void XBEEtest();

// XBEE GLOBAL VARIABLES
extern XBee canSatXBEE;
extern TxStatusResponse txStatus;
extern XBeeResponse response;
extern Rx16Response rx16;
extern Rx64Response rx64;
extern uint8_t option;
extern uint8_t xbeeData;
const int payloadByteNum = 100; // modify if needed
extern char ReceivedData[payloadByteNum];

#endif