#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

// NOTE: Arduino only has a 128 byte serial buffer so it can easily overflow if two or more packets arrive 
// without a call to readPacket. readPacket only supports reading one packet at a time.


// Arduino Library
#include <Arduino.h> // only need to include this in the header file

// Sensor Libraries
#include "EEPROM.h"

// Custom Libraries
#include "initialization.h"
#include "data_formatter.h"

// Struct Definition
struct ContainerData_t{ 
  uint16_t ID = 2056;             // [1]   2 bytes 
  String missionTimeRTC;          // [2]   9 bytes
  uint16_t packetNum = 0;         // [3]   2 bytes
  char mode = 'F';                // [4]   1 bytes
  uint8_t softwareState = 1;      // [5]   1 bytes (@ME Update Definitions for Each State Char)
  float pressureAlt;              // [6]   4 bytes 
  float airSpeed;                 // [7]   4 bytes
  char heatShieldStatus = 'A';    // [8]   1 bytes (A - Attached, D - Detached)
  char parachuteStatus = 'N';     // [9]   1 bytes (N - No, C - Deployed)
  float tempC;                    // [10]  4 bytes 
  float pressurePa;               // [11]  4 bytes
  float voltage;                  // [12]  4 bytes 
  String timeGPS;                 // [13]  9 bytes 
  float altitudeMetersGPS;        // [14]  4 bytes GPS altitude in meters
  long latitudeGPS;              // [15]  4 bytes GPS latitude in DDMM.MMM (Degrees, Minutes, then seconds in decimal minutes)  
  long longitudeGPS;             // [16]  4 bytes GPS longitude in DDDMM.MMMM     
  uint8_t satNum;                 // [17]  1 bytes unsigned int for sats 
  float tiltX;                    // [18]  4 bytes
  float tiltY;                    // [19]  4 bytes
  float rotZ;                     // [20]  4 bytes
  String cmdEcho;                 // [21]  Dynamic Bytes  (Leave this as a place holder for now) (@ETHAN what does this do??)
};

// Global Variables
extern ContainerData_t ContainerData;
extern String DataOut;
extern float pressureSIMP;

// EEPROM Variables
extern int StateAddy; // integer for EEPROM address for state 
extern int ApogeeAddy; // integer for EEPROM address for apogee 
extern int LastAltAddy; // integer for EEPROM address for LastAltAddy  
extern int SurfacePressureAddy; // Integer for EEPROM address for storing the surface pressure ***CHANGE FOR TEENSY 4.1) 
extern int PacketNumAddy; //Integer for packet number address 

// CANSAT Status
extern bool RX_ON;
extern bool SIM_ENABLED;
extern bool SIM_ON;
extern bool BCN_ON;

// Data Handling Functions
void sendData();
void receiveData();
void packetParser();
void resetPacketCount();

// Container Updating Function
void updateContainerData();

#endif