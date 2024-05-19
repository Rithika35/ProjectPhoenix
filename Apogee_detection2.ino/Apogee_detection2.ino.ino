// QUESTION: Is it Faster to have different print statements or have one big print statement

#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <math.h>

// Timing Variables 
#define NUM_READINGS 10
unsigned long lastReadingTime ;
const unsigned long readingInterval = 10000; // milliseconds

// Objects + Inputs and Output
MPL3115A2 myPressure;
File myFile;
int ledPin = 13;  // LED is attached to digital pin 13
const int led_pin = PB5;
volatile uint32_t count = 0;

// Pressure Variables
float Kelvin = 0 ;
int Altitude_AGL =0;
float pressureReadings[NUM_READINGS];

// Apogee Flag
int apogee = 0;

// Lockout Timing 
float SuperSonicMin = 0.0; 
float SuperSonicSec = 10.0; 
unsigned long time; //Timer

// Altitude Calculation Constants / Variables 
unsigned long  L_b = -0.0065; //Standard temperature lapse rate (K/m)
unsigned long  R = 8.31432; //Universal gas constant (N*m/mol*K)
unsigned long  g_0 = 9.80665; //Acceleration due to gravity (m/s^2)
unsigned long  M = 0.0289644; //Molar mass of Earth s air (kg/mol)
unsigned long  h_0 = 609.6; //Launch site elevation above sea level (2000ft / 609.6m for FAR launch site)/ orlanod is 25 m(82ft)
unsigned long ApogeeCount;
float pressure = 0;
float temperature = 0;
float P = 101250;//ground pressure

void setup() {
  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Timer:");
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(1); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void loop() {

  // Getting The Time
  time = micros();

  // Time Calculations
  float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*60000000.0;
  int Micro = time % 10;
  int TenMicro = (time/10) % 10;
  int HundredMicro = (time/100) % 10;
  int MilliSec = (time/1000) % 10;
  int HundrethSec = (time/10000) % 10;
  int TenthSec = (time/100000) % 10;
  int OneSec = (time/ 1000000) % 10;
  int TenSec = (time/ 10000000)%6;


  // Sensor readings every readingInterval (ex: 10ms)
  if (time - lastReadingTime >= readingInterval) {

    lastReadingTime = time;

    // Lockout Period Elapsed, Start Looking for Apogee
    while(time > SuperSonicCount && apogee==0  ){

      // Getting The Time
      time = micros();

      // Time Calculations 
      float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*60000000.0;
      int MilliSec = (time/1000) % 10;
      int HundrethSec = (time/10000) % 10;
      int TenthSec = (time/100000) % 10;
      int OneSec = (time/ 1000000) % 10;
      int TenSec = (time/ 10000000)%6;

      // Recieving and Printing Sensor Data
      PORTB |= (1 << led_pin);
      if (time - lastReadingTime >= readingInterval) {

        lastReadingTime = time;
        pinMode(ledPin, OUTPUT);
        Serial.print("T+");Serial.print(time/60000000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(MilliSec);
        pressure = myPressure.readPressure();
        Serial.print(" Pressure(Pa):");
        Serial.print(pressure, 2);
        temperature = myPressure.readTempF();
        Serial.print(" Temp(f):");
        Serial.print(temperature, 2);
        Kelvin = (temperature - 32) * 5/9 + 273.15;
        Altitude_AGL = (Kelvin/L_b)*((pow((P/pressure),(-R*L_b)/(g_0*M)))-1)- h_0;
        Serial.print(" Altitude(m):");
        Serial.print(Altitude_AGL, 2);
        Serial.println(" LOCKOUT PERIOD");


        // Checking For Apogee
        for (int i = 0; i < NUM_READINGS; i++) pressureReadings[i] = pressure/100;
        for(int j = 0; j< NUM_READINGS; j++){
          if(pressureReadings[j]>pressureReadings[j+1]){
            float maxPressure = pressureReadings[0];
            for (int i = 1; i < NUM_READINGS; i++) {
              if (pressureReadings[i] < maxPressure) {
                maxPressure = pressureReadings[i];
                PORTB &= ~(1 << led_pin);
                Serial.print(" Apogee Detected");
                apogee = 1;
                goto Apogee;
              }
            }
          }
        }
    }
  }



  // Apogee Detected, Deploy Parachute
  Apogee:
    // Print Pressure
    Serial.print("T+");Serial.print(time/60000000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(MilliSec);Serial.print(HundredMicro);Serial.print(TenMicro);Serial.print(Micro);
    pressure = myPressure.readPressure();
    Serial.print(" Pressure(Pa):");
    Serial.print(pressure, 2);
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.  
    // float temperature = myPressure.readTemp();
    // Serial.print(" Temp(c):");
    // Serial.print(temperature, 2);

    // Print Temperature and Altitute
    temperature = myPressure.readTempF();
    Serial.print(" Temp(f):");
    Serial.print(temperature, 2);
    Kelvin = (temperature - 32) * 5/9 + 273.15;
    Altitude_AGL = (Kelvin/L_b)*((pow((P/pressure),(-R*L_b)/(g_0*M)))-1)- h_0;
    Serial.print(" Altitude(m):");
    Serial.print(Altitude_AGL, 2);
    Serial.println();
  }
  
}
