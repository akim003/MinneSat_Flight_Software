#include "data_handler.h"

// Creates an Instance of ContainerData_t
ContainerData_t ContainerData;
String DataOut;
float pressureSIMP;

// Packet Parser Variables
int commaIndx[16];
String receivedDataStr;
int packetLength = 0;
String canSatCMD = "";
#define TEAM_ID "2056"

// State Variables
bool RX_ON = false;
bool SIM_ENABLED = false;
bool SIM_ON = false;

// Local Variables
String D1;
String D2;
String D3;
String D4;
String D5;
String D6;
String D7;
String D8;
String D9;
String D10;
String D11;
String D12;
String D13;
String D14;
String D15;
String D16;
String D17;
String D18;
String D19;
String D20;
String D21;

// Sends Data to CanSat Ground Station
void sendData() {
  // checks if telemetry is on
  if (RX_ON == false) {
    Serial.println("Waiting");
    return;
  }

  // Updates Data Struct
  updateContainerData();

  // Stores Packet Count to EEPROM for Long Term Storage
  //EEPROM.write(PacketNumAddy, ContainerData.packetNum);
  //EEPROM.write(StateAddy, ContainerData.softwareState);
  //EEPROM.write(LastAltAddy, ContainerData.pressureAlt);

  delay(1);

  // Converts String to Bytes for I2C
  byte Buffer[DataOut.length()+1];
  DataOut.getBytes(Buffer,DataOut.length()+1);

  // Sends Data Out Using the XBEE
  Tx16Request tx = Tx16Request(0x2056, Buffer, sizeof(Buffer)); 
  canSatXBEE.send(tx);

  // Sends Feedback to the Serial Monitor
  Serial.println("Packet Sent");
}

//TODO: SIM,SIMP,STATE
// PARTIAL - DETACH, 
// Parses Packets and Carries out Commands [@TEST]
void packetParser() {
  // PARSE STRING INTO COMPONENTS
  int k = 0;

  // convert to string to do length operation
  receivedDataStr = String(ReceivedData);
  packetLength = String(ReceivedData).length(); 

  // test print
  Serial.println(receivedDataStr);

  // retrieve locations of commas
  for (int i = 0; i < packetLength; i ++) {
    if (ReceivedData[i] == ',') {
      commaIndx[k] = i;
      k ++;
    }
  }

  // HANDLES COMMANDS
  if (receivedDataStr.substring(0, commaIndx[0]).equals("CMD")) {
    Serial.println("Command Received");
    
    // checks for team code
    if (!receivedDataStr.substring(commaIndx[0] + 1, commaIndx[1]).equals(TEAM_ID)) {
      Serial.println("Received External Data");
      return;
    }

    // CX - (ON/OFF) 
    if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("CX")) {
      // Telemetry On
      if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("ON")) {
        // update state variable
        RX_ON = true;

        // success message
        Serial.println("Telemetry ON");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "CX ON";
      }
      // Telemetry OFF
      else if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("OFF")) {
        // update status
        RX_ON = false;

        // success message
        Serial.println("Telemetry OFF");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "CX OFF";
      }
    }

    // ST - SYNC TIME RTC
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("ST")) {
      // TODO: ADD ERROR CHECKING CODE (ENSURE VALID TIME IS RECIEVED)

      // retrieves new time
      String newTimeRTC = receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]);

      // syncs RTC
      syncTimeRTC(newTimeRTC);

      // success message
      Serial.println("RTC Synced");

      // CMD ECHO - STORES LAST CMD
      ContainerData.cmdEcho = "ST RTC";
    }

    // SIM - TOOGLE SIMULATION MODE
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("SIM")) {

      // SIM ENABLED
      if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("ENABLE")) {
        SIM_ENABLED = true;
        ContainerData.mode = 'S';

        // Test Print
        Serial.println("SIM ENABLED");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "SIM ENABLE";
      }

      // SIM ACTIVATED
      else if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("ACTIVATE")) {
        SIM_ON = true;
        ContainerData.mode = 'S';

        // Test Print
        Serial.println("SIM ACTIVATED");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "SIM ACTIVATED";
      }

      // SIM DISABLED
      else if (receivedDataStr.substring(commaIndx[2] + 1).equals("DISABLE")) {
        SIM_ENABLED = false;
        SIM_ON = false;
        ContainerData.mode = 'F';

        // Test Print
        Serial.println("SIM DISABLED");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "SIM DISABLED";
      }
    }

    // SIMP - IMPORT SIMULATED PRESSURE
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("SIMP")) {
      // EXITS EARLY IF SIM IS NOT ENABLED
      if (SIM_ENABLED == false || SIM_ON == false) {
        return;
      }

      // IMPORTS SIMULATED PRESSURE DATA
      pressureSIMP = receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).toFloat();

      // SUCCESS MESSAGE
      Serial.println("SIMP RECIEVED");

      // CMD ECHO - STORES LAST CMD
      ContainerData.cmdEcho = "SIMP DATA";
    }

    // CAL - CALIBRATE PRESSURE ALTITUDE
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("CAL")) {
      tareAltitude();

      // success message
      Serial.println("ALTITUDE CALIBRATED");

      // CMD ECHO - STORES LAST CMD
      ContainerData.cmdEcho = "CAL";
    }

    // BCN - TOGGLE AUDIO BCN
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("BCN")) {
      Serial.println("BCN");
      // BCN ON
      if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("ON")) {
        BCN_ON = true;

        // success message
        Serial.println("BCN ON");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "BCN ON";
      }
      // BCN OFF
      else if (receivedDataStr.substring(commaIndx[2] + 1, commaIndx[3]).equals("OFF")) {
        BCN_ON = false;

        // success message
        Serial.println("BCN OFF");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "BCN OFF";
      }
    }

    // HEATSHIELD - TOGGLE HEATSHIELD
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("DETACH")) {

      // HEATSHIELD ATTACHED
      if (ContainerData.heatShieldStatus == 'A') {
        // update status
        ContainerData.heatShieldStatus = 'D';

        // TODO: call detachHeatShield();
        heatShieldServo.write(125);
        delay(1000);
        heatShieldServo.write(90);
        delay(10);

        heatShieldServo.write(55);
        delay(1000);
        heatShieldServo.write(90);
        delay(10);

        // success message
        Serial.println("HEATSHEILD DETACHED");

        // CMD ECHO - STORES LAST CMD
        ContainerData.cmdEcho = "DETACH";
      }
      // HEATSHIELD DETACHED
      else {
        // success message
        Serial.println("HEATSHEILD ALREADY DETACHED");
      }
    }

    // RESETS FSW
    else if (receivedDataStr.substring(commaIndx[1] + 1, commaIndx[2]).equals("RESET")) {
      // restart sensors
      setupCanSat();

      // reset state variables
      RX_ON = false;
      SIM_ENABLED = false;
      SIM_ON = false;
      BCN_ON = false;
      ContainerData.softwareState = 1;

      // success message
      Serial.println("FSW RESET");
    }

    // INVALID COMMAND
    else {
      Serial.println("ERROR: UNSUPPORTED COMMAND");
    }
  }

}

// Receives Commands & Sim Data from CanSat Ground Station [@TEST]
// GET HELP FROM ETHAN WITH ADRESSES ON CANSAT XBEE AND RECEVING METHODS FSW
void receiveData() {  
  // Xbee Reads Incoming Packet
  canSatXBEE.readPacket();

  // Checks if a Packet was Received
  if (canSatXBEE.getResponse().isAvailable()) {
    // Test Print
    //Serial.println("Available");

    // Checks if Packet is a Valid RX Packet
    if (canSatXBEE.getResponse().getApiId() == RX_16_RESPONSE || canSatXBEE.getResponse().getApiId() == RX_64_RESPONSE) {  
      Serial.println("Response Received");
      canSatXBEE.getResponse().getRx16Response(rx16);

      // Parses Data into an Array
      for(int i = 0; i < rx16.getDataLength(); i++){
        ReceivedData[i] = rx16.getData(i); 
      }

      // Test Prints
      //Serial.println (ReceivedData);

      packetParser(); 
    }
  } 
}

// Updates Fields of the Data Container Struct & Formats them as Strings
void updateContainerData() {
  // Pulls Fresh Sensor Data
  formatRTC();     // updates time
  formatIMU();     // updates x and y tilt
  formatBME();     // updates BME data
  formatPITOT();   // updates velocity
  updateVoltage(); // updates voltage
  formatGPS();     // updates gps

  // Updates Container Fields
  ContainerData.missionTimeRTC = CanSatTime;        // [2]
  ContainerData.packetNum ++;                       // [3]

  ContainerData.pressureAlt = relativeAltitude;     // [6]
  ContainerData.airSpeed = velocity;                // [7]

  ContainerData.tempC = temperatureBME;             // [10]
  ContainerData.pressurePa = pressureBME;           // [11]
  ContainerData.voltage = canSatVoltage;            // [12]

  ContainerData.timeGPS = gpsTime;                  // [13]
  ContainerData.altitudeMetersGPS = altitude;       // [14]
  ContainerData.latitudeGPS = latitude;             // [15]
  ContainerData.longitudeGPS = longitude;           // [16]
  ContainerData.satNum = numSatellites;             // [17]
  ContainerData.tiltX = tiltX;                      // [18]
  ContainerData.tiltY = tiltY;                      // [19]
  ContainerData.rotZ = rotZ;                         // [20]

  // @TODO: Add logic functions to update state variables
  
  // Formats Data into Strings
  D1 = String(ContainerData.ID) + ",";
  D2 = String(ContainerData.missionTimeRTC) + ","; 
  D3 = String(ContainerData.packetNum) + ",";
  D4 = String(ContainerData.mode) + ",";             // Needs work
  D5 = String(ContainerData.softwareState) + ",";
  D6 = String(ContainerData.pressureAlt) + ",";     
  D7 = String(ContainerData.airSpeed) + ",";
  D8 = String(ContainerData.heatShieldStatus) + ","; // Needs work
  D9 = String(ContainerData.parachuteStatus) + ",";  // Needs work
  D10 = String(ContainerData.tempC) + ",";
  D11 = String(ContainerData.pressurePa) + ",";
  D12 = String(ContainerData.voltage) + ",";
  D13 = String(ContainerData.timeGPS) + ",";
  D14 = String(ContainerData.altitudeMetersGPS) + ",";
  D15 = String(ContainerData.latitudeGPS) + ",";
  D16 = String(ContainerData.longitudeGPS) + ",";
  D17 = String(ContainerData.satNum) + ",";
  D18 = String(ContainerData.tiltX) + ",";
  D19 = String(ContainerData.tiltY) + ",";
  D20 = String(ContainerData.rotZ) + ",";
  D21 = String(ContainerData.cmdEcho) + "";

  // Formats Data Into String
  DataOut = D1 + D2 + D3 + D4 + D5 + D6 + D7 + D8 + D9 + D10 + D11 + D12 + D13 + D14 + D15 + D16 + D17 + D18 + D19 + D20 + D21;

  // Saves Packet to the SD
  saveSD(DataOut);
}
