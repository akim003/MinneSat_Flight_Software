#include "initialization.h"

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
#include "XBee.h"                      // XBEE (XBEE 3 PRO)

// SD Variables
const int chipSelect = BUILTIN_SDCARD;

// ----------------
// GLOBAL VARIABLES
// ----------------

// MASTER SETUP
bool prevStarted = false;
bool recoveryStartup = false;
int StateAddy = 200;           // integer for EEPROM address for state
int ApogeeAddy = 205;          // integer for EEPROM address for apogee
int LastAltAddy = 210;         // integer for EEPROM address for LastAltAddy
int BaseAltAddy = 215;         // integer for EEPROM address for storing the surface pressure
int PacketNumAddy = 220;       // integer for packet number address

// IMU Variables
ICM_20948_I2C canSatIMU;
ICM_20948_fss_t canSatFSS;

// PITOT Variables
bfs::Ms4525do canSatPITOT;

// RTC Variables (doesn't need setup function)
DS3231 canSatRTC;
RTClib canSatDataRTC;

// AUDIO BCN VARIABLES (doesn't need setup function)
Buzzer canSatBCN(40);
bool BCN_ON = false;

// GPS VARIABLES
SFE_UBLOX_GNSS canSatGPS;

// BME Variables
Adafruit_BME280 canSatBME;
float initialAltitude;

// SERVO VARIABLES
PWMServo heatShieldServo;
PWMServo parachuteServo;

// XBEE Variables
XBee canSatXBEE;
TxStatusResponse txStatus = TxStatusResponse();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();
uint8_t option = 0;
uint8_t xbeeData = 0; 
char ReceivedData[payloadByteNum];

// ----------------------
// SENSOR SETUP FUNCTIONS
// ----------------------

// MASTER SETUP
void setupCanSat() {
  // setup macros
  #define WIRE_PORT Wire 
  #define SERIAL_PORT Serial // serial to print to serial monitor

  // begins serial to print to
  SERIAL_PORT.begin(9600);

  // initializes I2C
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // starts all of the sensors
  setupIMU();
  setupSD();
  setupPITOT();
  setupGPS();
  setupBME();
  setupSERVOS();
  setupXBEE();

  // sets RTC mode to 24hr
  canSatRTC.setClockMode(false);

  // UNTESTED RECOVERY MODE
  /*if (prevStarted == true) {
    recoveryStartup = true;
  }

  // updates prevStarted boolean
  prevStarted = true;*/
}

// IMU SETUP
void setupIMU() {
  #define AD0_VAL 1
    
  // sensor calibration values
  int32_t biasAX, biasAY, biasAZ;
  int32_t biasGX, biasGY, biasGZ;

  // starts 
  canSatIMU.begin(WIRE_PORT, AD0_VAL);

  canSatIMU.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), canSatFSS);
  canSatFSS.a = gpm8;

  // provides feedback on sensor initialization
  while (canSatIMU.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.println("IMU Initialization Failed. Retrying...\n");
    delay(1000);
  }

  // calibrate IMU acceleration
  canSatIMU.getBiasAccelX(&biasAX);
  canSatIMU.setBiasAccelX(biasAX);
  canSatIMU.getBiasAccelY(&biasAY);
  canSatIMU.setBiasAccelY(biasAY);
  canSatIMU.getBiasAccelZ(&biasAZ);
  canSatIMU.setBiasAccelZ(biasAZ);

  // calibrate IMU gyro
  canSatIMU.getBiasGyroX(&biasGX);
  canSatIMU.setBiasGyroX(biasGX);
  canSatIMU.getBiasGyroY(&biasGY);
  canSatIMU.setBiasGyroY(biasGY);
  canSatIMU.getBiasGyroZ(&biasGZ);
  canSatIMU.setBiasGyroZ(biasGZ);

  // prints success message
  SERIAL_PORT.println("IMU Initialization Success!");
}

// PITOT SETUP
void setupPITOT() {
  // sets preferred startup settings
  canSatPITOT.Config(&Wire, 0x28, 1.0f, -1.0f);

  // ensures proper startup
  while (!canSatPITOT.Begin()) {
    SERIAL_PORT.println("PITOT Initialization Failed. Retrying...");
  }

  // prints success message
  SERIAL_PORT.println("PITOT Initialization Success!");
}

// SD SETUP
void setupSD() {
  while (!SD.begin(chipSelect)) {
    Serial.println("SD ERROR: Failed to Initialize");
    delay(1000);
  }

  // prints success message
  Serial.println("SD Initialization Success");
}

// GPS SETUP
void setupGPS() {
  #define gpsWire Wire
  #define gnssAddress 0x42

  // attempts to start gps
  while(canSatGPS.begin(gpsWire, gnssAddress) == false) {
    Serial.println("GPS Initialization Failed. Retrying...");
    delay(100);
  }

  // filter out noise (from Example4_Custom12C Example )
  canSatGPS.setI2COutput(COM_TYPE_UBX);
  canSatGPS.setNavigationFrequency(10);
  canSatGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // prints success message
  Serial.println("GPS Initialization Success!");

}

// BME SETUP
void setupBME() {
  #define SEALEVELPRESSURE_HPA (1013.25)

  while (!canSatBME.begin(0x77, &Wire)) {
    Serial.println("BME ERROR: Failed to Initialize");
  }

  // calculates inital height
  initialAltitude = canSatBME.readAltitude(SEALEVELPRESSURE_HPA);

  // prints success message
  Serial.println("BME Initialization Success");
}

// SERVO SETUP
void setupSERVOS() {
  heatShieldServo.attach(25);
  parachuteServo.attach(29);
}

// XBEE SETUP
void setupXBEE() {
  #define XBEE_SERIAL Serial1 // designates serial port for printing
  
  // initializes xbee and serial port 5
  canSatXBEE = XBee();
  XBEE_SERIAL.begin(921600);
  canSatXBEE.setSerial(XBEE_SERIAL);

  // prints success message
  Serial.println("XBEE Initialization Success!");
}

// ------------------------------
// IMU DEBUGGING HELPER FUNCTIONS
// ------------------------------

// Taken From Example1_Basics.ino  (#ICM_20948.h)

// Formats Floats
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {

  float aval = abs(val);

  if (val < 0) {
    SERIAL_PORT.print("-");
  }
  else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    }
    else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  }
  else {
    SERIAL_PORT.print(val, decimals);
  }
}

// Formats Data Output
void printScaledAGMT(ICM_20948_I2C *sensor) {
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

// Displays Senor Readings
void IMUtest() {
  if (canSatIMU.dataReady()) {
    canSatIMU.getAGMT();         // The values are only updated when you call 'getAGMT'                      
    printScaledAGMT(&canSatIMU); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    SERIAL_PORT.println("\n");
    delay(500);
  }
  else {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

// ------------------------------
// RTC DEBUGGING HELPER FUNCTIONS
// ------------------------------

// Resets RTC Time to 00:00:00
void resetTimeRTC() {
  canSatRTC.setSecond(0);
  canSatRTC.setMinute(0);
  canSatRTC.setHour(0);
}

// Sets RTC Time to Given String
void syncTimeRTC(String newTimeRTC) {
  // creates holders for time components
  int hours, minutes, seconds;

  // built in Arduino method for parsing time
  parseTimeString(newTimeRTC, hours, minutes, seconds);

  // updates RTC with new time
  canSatRTC.setSecond(seconds);
  canSatRTC.setMinute(minutes);
  canSatRTC.setHour(hours);
}

// (ALEX) ALERT - NEEDS ERROR CHECKING MECHANISM
// Parses Time String Into Its Components
void parseTimeString(String timeString, int& hours, int& minutes, int& seconds) {
  // finds indexes of semicolons
  int firstColon = timeString.indexOf(':');
  int lastColon = timeString.lastIndexOf(':');

  // assigns time values to variables
  hours = timeString.substring(0, firstColon).toInt();
  minutes = timeString.substring(firstColon + 1, lastColon).toInt();
  seconds = timeString.substring(lastColon + 1).toInt();
}

// ------------------------------
// BCN DEBUGGING HELPER FUNCTIONS
// ------------------------------

// Plays Jingle Bells
void jingleBellsBCN() {
  // starts buzzer
  canSatBCN.begin(100);

  // jingle bell score
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_C7, 80);
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_G7, 80);
  canSatBCN.sound(0, 240);
  canSatBCN.sound(NOTE_G6, 80);
  canSatBCN.sound(0, 240);
  canSatBCN.sound(NOTE_C7, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_G6, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_E6, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_A6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_B6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_AS6, 80);
  canSatBCN.sound(NOTE_A6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_G6, 100);
  canSatBCN.sound(NOTE_E7, 100);
  canSatBCN.sound(NOTE_G7, 100);
  canSatBCN.sound(NOTE_A7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_F7, 80);
  canSatBCN.sound(NOTE_G7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_C7, 80);
  canSatBCN.sound(NOTE_D7, 80);
  canSatBCN.sound(NOTE_B6, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_C7, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_G6, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_E6, 80);
  canSatBCN.sound(0, 160);
  canSatBCN.sound(NOTE_A6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_B6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_AS6, 80);
  canSatBCN.sound(NOTE_A6, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_G6, 100);
  canSatBCN.sound(NOTE_E7, 100);
  canSatBCN.sound(NOTE_G7, 100);
  canSatBCN.sound(NOTE_A7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_F7, 80);
  canSatBCN.sound(NOTE_G7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_E7, 80);
  canSatBCN.sound(0, 80);
  canSatBCN.sound(NOTE_C7, 80);
  canSatBCN.sound(NOTE_D7, 80);
  canSatBCN.sound(NOTE_B6, 80);
  canSatBCN.sound(0, 160);

  // ends buzzer
  canSatBCN.end(2000);
}

// Toggles Beacon Status
void toggleBCN() {
  // False = OFF : True = ON
  BCN_ON = !BCN_ON;  

  // needs delay for monotoneBCN to register
  delay(1000);
}

// ------------------------------
// BME DEBUGGING HELPER FUNCTIONS
// ------------------------------

// Updates the Value of InitialAltitude
void tareAltitude() {
  #define SEALEVELPRESSURE_HPA (1013.25)

  initialAltitude = canSatBME.readAltitude(SEALEVELPRESSURE_HPA);

  EEPROM.write(BaseAltAddy, initialAltitude);
}

// -------------------------------
// XBEE DEBUGGING HELPER FUNCTIONS
// -------------------------------

// Taken From CanSat Probe 2021

// send a test packet to receiver xbee
void XBEEtestSendData() {
  // preps & sends the test string
  String testData = "Testing... 1.. 2.. 3.. \n";
  byte Buffer[testData.length()+1]; 
  testData.getBytes(Buffer,testData.length()+1);
  Tx16Request tx = Tx16Request(0x2056, Buffer, sizeof(Buffer)); 
  canSatXBEE.send(tx);

  // TODO: idk what this does or if it works maybe ask @Ethan
  // checks ground station connection
  if (canSatXBEE.getResponse().getApiId() == TX_STATUS_RESPONSE) {
    canSatXBEE.getResponse().getTxStatusResponse(txStatus); 
  } 

  // prints outbound data to the serial monitor
  Serial.println(testData);
}

// receives incoming test packet from GSC xbee
void XBEEtestReceiveData() {
  canSatXBEE.readPacket();
  if (canSatXBEE.getResponse().isAvailable()) { // Got something 
    if (canSatXBEE.getResponse().getApiId() == RX_16_RESPONSE || canSatXBEE.getResponse().getApiId() == RX_64_RESPONSE) { // Received and RX packet 
      canSatXBEE.getResponse().getRx16Response(rx16);
      for (int i = 0; i < rx16.getDataLength(); i++){ //Read in the data in a for loop 
        ReceivedData[i] = rx16.getData(i); 
      }
      Serial.println("CMD Echo: \n"); 
      Serial.flush();
      Serial.println(String(ReceivedData)); 
      Serial.flush();
      Serial.println(); 
      Serial.flush();
    }
  } 
}

void XBEEtest() {
  // attempts to receive and send out test data
  XBEEtestSendData();
  XBEEtestReceiveData(); // @Ethan can i do both at the same time?

  delay(1000);
}
