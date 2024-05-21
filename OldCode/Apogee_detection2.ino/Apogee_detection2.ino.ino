#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <math.h>

// timing variables
#define num_readings 10
unsigned long lastreadingtime ;
const unsigned long readinginterval = 10000; // milliseconds

// objects + inputs and output
mpl3115a2 mypressure;
file myfile;
int ledpin = 13;  // led is attached to digital pin 13
const int led_pin = pb5;
volatile uint32_t count = 0;

// pressure variables
float kelvin = 0 ;
float altitude_agl;
float pressurereadings[num_readings];

// apogee flag
int apogee = 0;

// lockout timing
float supersonicmin = 0.0;
float supersonicsec = 10.0;

//timer
unsigned long time;

// altitude calculation constants and variables
float  l_b = -0.0065; //standard temperature lapse rate (k/m)
float  r = 8.31432; //universal gas constant (n*m/mol*k)
float  g_0 = 9.80665; //acceleration due to gravity (m/s^2)
float  m = 0.0289644; //molar mass of earth s air (kg/mol)
float  h_0 = 28; //launch site elevation above sea level (2000ft / 609.6m for far launch site)/ orlanod is 25 m(82ft)
float apogeecount;
float pressure = 0;
float temperature = 0;
float p_0 = 101325;//ground pressure
float t_0 = 288;

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
   //once rocket reaches lock out period
  // while(time > SuperSonicCount && apogee==0  ){
  //   time = micros();
  //   float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*60000000.0;
  //   int MilliSec = (time/1000) % 10;
  //   int HundrethSec = (time/10000) % 10;
  //   int TenthSec = (time/100000) % 10;
  //   int OneSec = (time/ 1000000) % 10;
  //   int TenSec = (time/ 10000000)%6;
  //   PORTB |= (1 << led_pin);
  //   if (time - lastReadingTime >= readingInterval) {
  //   lastReadingTime = time;
  //   pinMode(ledPin, OUTPUT);
  //   Serial.print("T+");Serial.print(time/60000000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(MilliSec);
  //   pressure = myPressure.readPressure();
  //   Serial.print(" Pressure(Pa):");
  //   Serial.print(pressure, 2);
  //   temperature = myPressure.readTempF();
  //   Serial.print(" Temp(f):");
  //   Serial.print(temperature, 2);
  //   float T_0 = (temperature - 32) * 5/9 + 273.15;
  //   Altitude_AGL = (T_0/L_b)*(pow((101500/pressure),((-R*L_b)/(g_0*M)))-1) - h_0;
  //   Serial.print(" Altitude(m):");
  //   Serial.print(Altitude_AGL, 2);
  //   Serial.println(" LOCKOUT PERIOD");
  //   for (int i = 0; i < NUM_READINGS; i++) {
  //   pressureReadings[i] = pressure/100;
  //   }
  //   for(int j = 0; j< NUM_READINGS; j++){
  //     if(pressureReadings[j]>pressureReadings[j+1]){
  //       float maxPressure = pressureReadings[0];
  //       for (int i = 1; i < NUM_READINGS; i++) {
  //         if (pressureReadings[i] < maxPressure) {
  //           maxPressure = pressureReadings[i];
  //           PORTB &= ~(1 << led_pin);
  //           Serial.print(" Apogee Detected");
  //           apogee = 1;
  //           goto Apogee;
  //           }
  //         }
  //       }
  //     }
  //   }
  // }
  // Apogee:

  // Recieving Temperature and Pressure
  Serial.print("T+");Serial.print(time/60000000);Serial.print(":");Serial.print(TenSec);Serial.print(OneSec);Serial.print(":");Serial.print(TenthSec);Serial.print(HundrethSec);Serial.print(MilliSec);Serial.print(HundredMicro);Serial.print(TenMicro);Serial.print(Micro);
  pressure = myPressure.readPressure();
  Serial.print(" Pressure(Pa):");
  Serial.print(pressure, 2);
   // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // float temperature = myPressure.readTemp();
  // Serial.print(" Temp(c):");
  // Serial.print(temperature, 2);
  temperature = myPressure.readTempF();
  Serial.print(" Temp(f):");
  Serial.print(temperature, 2);


  // Calculating Altitude
  Altitude_AGL =(((T_0)/(L_b))*(pow((pressure/P_0),((-R*L_b)/(g_0*M)))-1)) - h_0; //(Kelvin/L_b)*((pow((pressure/P_0),(-R*L_b)/(g_0*M)))-1)- P_0;
  Serial.print(" Altitude(m):");
  Serial.print(Altitude_AGL, 2);
  Serial.println();
  }

}
