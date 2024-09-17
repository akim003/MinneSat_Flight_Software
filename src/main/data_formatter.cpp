#include "data_formatter.h"

// IMU Variables
float tiltX;
float tiltY;
float accAngleX;
float accAngleY;
float gyroRateX;
float gyroRateY;
float alpha;
float rotZ;

// PITOT Variables
float pitotVelocity;
float velocity;
float pressurePA;
float tempC;
float density;
int decimalPlaces = 6;

// RTC Variables
String CanSatTime;

// GPS
String gpsTime;
float altitude;
long latitude;
long longitude;
uint8_t numSatellites;
long lastTime = 0;

// BME
float temperatureBME;
float pressureBME;
float relativeAltitude;

// Initializes SD Variables
File dataFile;

// OTHER
float canSatVoltage;

// Calculates X and Y Tilt
void formatIMU() {
  // Refreshes IMU Data
  canSatIMU.getAGMT();

  // Get accelerometer angles
  tiltX = atan2(canSatIMU.accY(), canSatIMU.accZ()) * RAD_TO_DEG;
  tiltY = atan2(-canSatIMU.accX(), sqrt(canSatIMU.accY() * canSatIMU.accY() + canSatIMU.accZ() * canSatIMU.accZ())) * RAD_TO_DEG;

  // Test Serial Print Functions
  //Serial.println("X Tilt: " + String(tiltX));
  //Serial.println("Y Tilt: " + String(tiltY));

  // Get gyroscope data
  rotZ = canSatIMU.gyrZ();

  // Test Serial Print Functions
  //Serial.println("rotZ: " + String(rotZ));
}

// Calculates Pressure & Temp [Update Density Calculations]
void formatPITOT() {
  if (canSatPITOT.Read()) {
    // test print
    //Serial.print(canSatPITOT.pres_pa(), 6);
    //Serial.print("\t");
    //Serial.print(canSatPITOT.die_temp_c(), 6);
    //Serial.print("\n");

    // calculate density using the BME
    density = pressureBME / (287 * (temperatureBME + 273.1));

    // calculate velocity
    pitotVelocity = sqrt(2 * abs(canSatPITOT.pres_pa()) / density) -  1;
    //velocity = 42.8 * log(pitotVelocity) - 109;
    velocity = pitotVelocity;

    // test print
    //Serial.println(velocity);
    //Serial.println(canSatPITOT.pres_pa());
    //Serial.println(pitotVelocity);
  }
}

// Fetches & Updates Time
void formatRTC() {
  // updates times
  DateTime MyTime = canSatDataRTC.now();

  // resets time string
  CanSatTime = "";

  //concatenates time into a string
  if (MyTime.hour() < 10) CanSatTime += "0";        // Add leading zero if hour is less than 10
  CanSatTime += String(MyTime.hour(), DEC) + ":";
  if (MyTime.minute() < 10) CanSatTime += "0";      // Add leading zero if minute is less than 10
  CanSatTime += String(MyTime.minute(), DEC) + ":";
  if (MyTime.second() < 10) CanSatTime += "0";      // Add leading zero if second is less than 10
  CanSatTime += String(MyTime.second(), DEC);

  // test print
  //Serial.println(CanSatTime);
}

// Updates Altitude, Latitude, Longitude, and numSatellites
void formatGPS() {
  // updates data
  canSatGPS.getPVT(120);

  // reset time string
  String gpsTime = "";
  int hoursGPS = canSatGPS.getHour();
  int minutesGPS = canSatGPS.getMinute();
  int secondsGPS = canSatGPS.getSecond();

  // format time
  if (hoursGPS < 10) gpsTime += "0";        // Add leading zero if hour is less than 10
  gpsTime += String(hoursGPS, DEC) + ":";
  if (minutesGPS < 10) gpsTime += "0";      // Add leading zero if minute is less than 10
  gpsTime += String(minutesGPS, DEC) + ":";
  if (secondsGPS < 10) gpsTime += "0";      // Add leading zero if second is less than 10
  gpsTime += String(secondsGPS, DEC);

  // updates position variables
  altitude = (static_cast<float>(canSatGPS.getAltitude()) / 1000);       // meters
  latitude = (static_cast<long>(canSatGPS.getLatitude()));   // degrees
  longitude = (static_cast<long>(canSatGPS.getLongitude())); // degrees

  // updates number of satellites
  numSatellites = canSatGPS.getSIV();

  // test prints
  /*Serial.println("GPS Time");
  Serial.println(gpsTime);

  Serial.println("Real Altitude");
  Serial.println(altitude);

  Serial.println("Latitude");
  Serial.println(latitude);

  Serial.println("Longitude");
  Serial.println(longitude);

  Serial.println("# Satellites");
  Serial.println(numSatellites); 

  Serial.println(canSatGPS.getLatitude());
  Serial.println(canSatGPS.getLongitude());*/
}

// Updates Temp and Pressure,
void formatBME() {
  #define SEALEVELPRESSURE_HPA (1013.25)

  // handles SIM mode
  if (SIM_ON) {
    pressureBME = pressureSIMP;
    relativeAltitude = ((pow(10,((log10((pressureSIMP/100)/SEALEVELPRESSURE_HPA))/(5.2558797))) - 1)/(-6.8755856*pow(10,-6)))*0.3048;
  }
  else {
    pressureBME = canSatBME.readPressure() * 100;
    relativeAltitude = canSatBME.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude;
  }

  // update data
  temperatureBME = canSatBME.readTemperature();

  // test prints
  //Serial.println(temperatureBME);
  //Serial.println(pressureBME);
  //Serial.println(relativeAltitude);
}

// Pulses A5 Tone
/*void monotoneBCN() {
  // checks for BCN status
  if(BCN_ON == true) {
    // starts buzzer 
    canSatBCN.begin(100);

    // plays sound BCN
    canSatBCN.sound(NOTE_A5, 200);
    canSatBCN.sound(0, 50);

    // ends buzzer
    canSatBCN.end(100);
  }
}*/

// Plays Jingle Bells
void monotoneBCN() {
  if(BCN_ON == true) {
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
    canSatBCN.end(100);
  }
}

void saveSD(String packet) {
  // creates a .csv file
  dataFile = SD.open("/data.csv", FILE_WRITE);

  // saves data to SD
  if (dataFile) {
    dataFile.println(packet);
    dataFile.close();
    Serial.println("Data saved");
  } 
  else {
    Serial.println("Error opening data.csv");
  }
}

// Updates Battery Voltage Reading
void updateVoltage() {
  canSatVoltage = (analogRead(24) / 1023.00) * 3.30 * 3.00 * (7.70/9.45); 

  // test print
  //Serial.println(canSatVoltage);
}

//-----------------
// HELPER FUNCTIONS
//-----------------

// GPS HELPER
float roundToDecimalPlaces(float value, int decimalPlaces) {
  float scale = pow(10, decimalPlaces);
  return round(value * scale) / scale;
}