#include <Adafruit_MPL3115A2.h>
#include <wire.h>
#include <SD.h>

// Defining Objects
#define Addr 0x60
Adafruit_MPL3115A2 baro;
File myFile;

// Lockout Timings
unsigned long time;
float SuperSonicMin = 0.0;
float SuperSonicSec = 5.0;

void setup() {

  // Initializing Serial Connection
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Adafruit_MPL3115A2 test!");
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  // Initializing SD Card
  Serial.println("Initializing SD card...");
  pinMode(10, OUTPUT);// change this pin to 53 for mega
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // Locatizing Sensor to Launch Site
  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  //orlando is 1020 hpa
  baro.setSeaPressure(1013.95);

}

void loop() {

  Wire.beginTransmission(Addr); // Start I2C transmission
  Wire.write(0x26); // Select control register
  Wire.write(0x00); // Active mode, OSR = 128, altimeter mode
  Wire.endTransmission(); // Stop I2C transmission

  // Recieving Pressure Altitude and Temperature
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperature = baro.getTemperature();

  // Time Calculations
  time = millis();
  float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*60000.0;
  int MilliSec = time % 10;
  int HundrethSec = (time/10) % 10;
  int TenthSec = (time/100) % 10;
  int OneSec = (time/ 1000) % 10;
  int TenSec = (time/ 10000)%6;

  // Printing Data
  Serial.print("T+");Serial.print(time/60000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(MilliSec);
  Serial.print("| pressure = "); Serial.print(pressure); Serial.print(" hPa");
  Serial.print("| altitude = "); Serial.print(altitude); Serial.print(" m");
  Serial.print("| temperature = "); Serial.print(temperature); Serial.println(" C");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  // Saving Data to SD Card
  myFile = SD.open("Phoenix.txt", FILE_WRITE);

  // If the file opened okay, write to it:
  if (myFile) {

    Serial.println("Writing to Phoenix.txt...");
    myFile.println("pressure = "); myFile.print(pressure); myFile.println(" hPa");
    myFile.println("altitude = "); myFile.print(altitude); myFile.println(" m");
    myFile.println("temperature = "); myFile.print(temperature); myFile.println(" C");
    myFile.println("-----------------");

    // Close The File:
    myFile.close();

  } else {
    // If the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

}
