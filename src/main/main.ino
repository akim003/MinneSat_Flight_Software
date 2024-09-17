#include "flight_logic.h"

// Timing Variables
unsigned long startTime;
unsigned long loopTime;

void setup() {
  // Sets up sensors and utility
  setupCanSat();

  startTime = millis();
}

void loop() {
  // Start Timer
  loopTime = millis();

  // Recieve Data
  receiveData();
  
  // Sends out Data at Proper Intervals
  if (loopTime - startTime > 950) {
    sendData(); 
    canSatFlightLogic();
    monotoneBCN();
    
    // Update Time
    startTime = loopTime;

    // Test Print
    Serial.println("Software State: " + String(ContainerData.softwareState));
  }
}

// ------
// NOTES
// ------

// - ADD BACK SD FUNCTIONALITY