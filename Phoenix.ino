// Apogee
#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <math.h>

// BPS
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

const int SOLENOID1 = 7;  //first set
const int SOLENOID2 = 5;  //second set

void setup() {

  // Apogee
  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Timer:");
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(1); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  // BPS
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
