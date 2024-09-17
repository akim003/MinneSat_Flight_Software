#include "flight_logic.h"

// Variable Declarations
bool newState = true;
int altDelta = 0;
int deltaTracker = 0;
float apogee = -1;

// Stage 3 Variables
float lastAlt;
int lastAltTracker;

// Header Variables
String H1 = "Team #,";
String H2 = "Mission Time,";      // RTC
String H3 = "Packet #,";
String H4 = "Mode,";              // Flight/SIM
String H5 = "Software Stage,";
String H6 = "Altitude,";          // Pressure
String H7 = "Air Speed,";
String H8 = "HS Status,";         // Heat Shield
String H9 = "Parachute Status,";
String H10 = "Temp,";
String H11 = "Pressure,";
String H12 = "Voltage,";
String H13 = "GPS Time,";
String H14 = "Altitude,";
String H15 = "Latitude,";
String H16 = "Longitude,";
String H17 = "Sat #,";
String H18 = "Tilt X,";
String H19 = "Tilt Y,";
String H20 = "Rot Z,";
String H21 = "CMD Echo";

// UNITS
// Altitude (meters)
// Airspeed (meters/s)
// Temp     (celsius)
// Pressure (killopascals)
// Voltage  (volts)


void canSatFlightLogic() {
  /*
    FLIGHT STAGES
    1  - Launch Wait (CX - OFF)
    2  - Launch Wait (CX - ON)
    3  - Ascent
    4  - Descent     (HS - DEPLOYED)
    5  - Descent     (HS - DETACHED)
    6  - Recovery
  */

  // STAGE 1: Launch Wait (CX - OFF)

  // CX OFF and CX idle occur automatically with receiveData() & sendData()
  
  if (ContainerData.softwareState == 1) {

    // Checks for newState
    if (newState) {
      newState = false;

      // Test Prints
      Serial.println("Stage 1: CX OFF");
    }

    // Checks for Stage 2 Conditions
    if (RX_ON) {
      ContainerData.softwareState = 2;
      newState = true;
    }
  }

  // STAGE 2: Launch Wait (CX - ON)

  // CX OFF and CX idle occur automatically with receiveData() & sendData()

  else if (ContainerData.softwareState == 2) {

    // Inital State Actions
    if (newState) {
      // reset RTC time
      resetTimeRTC();

      // tare altitude
      tareAltitude();

      // update newState
      newState = false;

      // resets deltaTracker
      deltaTracker = 0;

      // adds headers to SD save
      DataOut = H1 + H2 + H3 + H4 + H5 + H6 + H7 + H8 + H9 + H10 + H11 + H12 + H13 + H14 + H15 + H16 + H17 + H18 + H19 + H20 + H21;
      sendData();
      saveSD(DataOut);

      // Test Prints
      Serial.println("Stage 2: CX ON");
    }

    // Tracks Deltas
    if (lastAlt < ContainerData.pressureAlt - 2) {
      deltaTracker ++;
    } 
    else {
      deltaTracker = 0;
    }
  
    // Updates Last Alt
    lastAlt = ContainerData.pressureAlt;

    // Checks for Stage 3 Conditions
    if (deltaTracker > 3) {
      ContainerData.softwareState = 3;
      newState = true;
      deltaTracker = 0;
    }
  }
  
  // STAGE 3: ASCENT

  else if (ContainerData.softwareState == 3) {

    // Inital State Actions
    if (newState) {
      lastAlt = ContainerData.pressureAlt;
      newState = false;

      // Test Print
      Serial.println("Stage 3: ASCENT");
    }

    // Updates Pressure Alt & Tracks Deltas
    if (ContainerData.pressureAlt > apogee) {
      apogee = ContainerData.pressureAlt;
      EEPROM.write(ApogeeAddy, apogee);
      deltaTracker = 0;
    }
    else {
      deltaTracker ++;
    }

    // Checks for Stage 4 Conditions
    if (deltaTracker >= 3) {
      ContainerData.softwareState = 4;
      newState = true;
    }   
  }

  // STAGE 4: DESCENT (HS - DEPLOYED)

  else if (ContainerData.softwareState == 4) {

    // Initial Stage Actions
    if (newState) {
      // Aerobrake Deployment [@TODO]
      newState = false;

      heatShieldServo.write(130);
      delay(1000);
      heatShieldServo.write(98);
      //heatShieldServo.write(90);
      //delay(10);

      // Test Print
      Serial.println("Stage 4: DESCENT (HS - DEPLOYED)");
    }

    // Checks for Stage 5 Conditions
    if (ContainerData.pressureAlt < 150) {
      ContainerData.softwareState = 5;
      newState = true;
    }
  }

  // STAGE 5: DESCENT (HS - DETACHED)

  else if (ContainerData.softwareState == 5) {

    // Initial Stage Actions
    if (newState) {
      // Aerobrake DETACH [@TODO]
      newState = false;

      deltaTracker = 0;

      // Update Status
      ContainerData.heatShieldStatus = 'D';
      ContainerData.parachuteStatus = 'C';

      // Release Parachute
      parachuteServo.write(160);
      delay(160);
      parachuteServo.write(90);

      //Detach Heatsheild
      heatShieldServo.write(50);
      delay(1000);
      heatShieldServo.write(82);
      //heatShieldServo.write(90);
      //delay(10);

      // Test Print
      Serial.println("Stage 5: DESCENT (HS - DETACHED)");
    }

    deltaTracker ++;

    // Checks for Stage 6 Conditions
    if (ContainerData.pressureAlt < 75 && deltaTracker >= 5) {
      ContainerData.softwareState = 6;
      newState = true;
    }
  }

  // STAGE 6: RECOVERY

  else if (ContainerData.softwareState == 6) {

    // Inital State Actions
    if (newState) {
      newState = false;
      deltaTracker = 0;

      // Turn off Servo
      heatShieldServo.write(90);

      // Test Print
      Serial.println("Stage 6: RECOVERY");
    }

    // Delays CX OFF for 5 Cycles
    if (deltaTracker > 5) {
      BCN_ON = true;
      RX_ON = false;
    }

    // Tracks Cycles
    deltaTracker ++;

    // Runs AUDIO BCN
    monotoneBCN();
  }
}