// 1 sample rates every 6 milli checking every 100 milliseconds
//for card reader
//53 -> cs
//52 -> clk
//51 -> DI
//50 -> DO
#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <math.h>
#define NUMREADINGS 4
MPL3115A2 myPressure;
File myFile;

//Timer
unsigned long time;//once power is connected
unsigned long timePlus;//time after liftoff
unsigned long lastReadingTime ; //last measurement taken
const unsigned long readingInterval = 1000000; // check sesnor every one second
//Initialize temperature, pressure, and constants used to determine altitude
float pressure = 0;
float temperature = 0;
float Kelvin = 0 ;// for temperature conversion
float Altitude_AGL; // altitude above ground level
float  L_b = -0.0065; //Standard temperature lapse rate (K/m)
float  R = 8.31432; //Universal gas constant (N*m/mol*K)
float  g_0 = 9.80665; //Acceleration due to gravity (m/s^2)
float  M = 0.0289644; //Molar mass of Earth s air (kg/mol)
float  h_0 = 11; //Launch site elevation above sea level (2000ft / 609.6m for FAR launch site)/ orlanod is 25 m(82ft)
float P_0 = 101325;//ground pressure in orlando, must check at mojave
float T_0 = 288;//ground temperature in kelvin

//Liftoff
float threshold = 5.0; // Altitude in meters needed to detect lift off
int liftoff = 0;// liftoff = flase
unsigned long LiftOffTime = 0; //timer after liftoff
unsigned long LOTime; //timestamp

// Lockoutperiod 
int lockout = 1; //lockout = true
float SuperSonicMin = 0.0; //numer of minutes of lockout duration
float SuperSonicSec = 3.0; // number of seconds of lockout duration
const int consecutiveReadings = NUMREADINGS; // number of readings needed to exit lockout
float lastAltitude = 0;// Variable to store the last altitude reading to check if lockout is over

// Apogee
int Apogee = 0;//Apogee = false
float ApogeePressure = 0; // Variable to store the last pressure reading
unsigned long ApogeeTime;//time stamp of apogee

//Water Ballast pins
const int SOLENOID1 = 7;  //first set
const int SOLENOID2 = 5;  //second set
unsigned long DrainTime = 5000000;//seconds to drain ballast 45 seconds
int TurnOffValve = 0;//flag to turn off valve
unsigned long AfterApogee;//keep track of time after apogee

//initiator
const int IGNITER = 8;//set initiator to pin 8
bool igniterOn = false;

//Actuator 
int DroneDeploy = 0;//if solenoid valve is closed drone is ready to deploy
float DroneDeployment = 8 ;//altitude of drone deployment in meters
const int Actuator = 9; //Actuator pin
int TouchDown =0;
unsigned long Landing = 4; //altitude to terminate program

//function to check if lockout is over
bool isLockoutOver(float altitude){
  static int consecutiveCount = 0; 
  bool status;
  // Check if altitude is greater than the last reading
  if (altitude > lastAltitude) {
    // Increase consecutive readings count
    consecutiveCount++;  
    // Check if consecutive readings count reaches the threshold
    if (consecutiveCount >= consecutiveReadings) {
      return true; // Altitude is increasing for consecutive readings
    }
  } else {
    // Reset consecutive readings count if altitude is not increasing
    consecutiveCount = 0;
    status = false;
  }
  
  // Update the last altitude reading
  lastAltitude = altitude;
  
  return false; // Altitude is not increasing for consecutive readings
}

//function to check if apogee has been reach using pressure
bool isApogee(float Pressure){
  static int consecutiveCount = 0; 
  bool status;
  // Check if altitude is greater than the last reading
  if (Pressure > ApogeePressure) {
    // Increase consecutive readings count
    consecutiveCount++;  
    // Check if consecutive readings count reaches the threshold
    if (consecutiveCount >= consecutiveReadings) {
      return true; // Altitude is increasing for consecutive readings
    }
  } else {
    // Reset consecutive readings count if altitude is not increasing
    consecutiveCount = 0;
    status = false;
  }
  
  // Update the last altitude reading
  ApogeePressure = Pressure;
  
  return false; // Altitude is not increasing for consecutive readings
}
void setup() {
  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Initializing SD card...");
  pinMode(53, OUTPUT);// change this pin to 53 for mega
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }Serial.println("initialization done.");
  myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card  
    if (myFile) {
      myFile.println("Welcome to Phoenix");
      myFile.close();//Close file
    }
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(128); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  pinMode(SOLENOID1, OUTPUT);//set solenoid 1 pin
  pinMode(SOLENOID2, OUTPUT);//set solenoid 2 pin
  pinMode(IGNITER, OUTPUT);//set initiator pin
  pinMode(Actuator, OUTPUT);//set initiator pin
}
void loop() {
  // put your main code here, to run repeatedly:
  time = micros();
  float SuperSonicCount = (SuperSonicSec/60.0 +SuperSonicMin)*60000000.0;
  int Micro = time % 10;
  int TenMicro = (time/10) % 10;
  int HundredMicro = (time/100) % 10;
  int MilliSec = (time/1000) % 10;
  int HundrethSec = (time/10000) % 10;
  int TenthSec = (time/100000) % 10;
  int OneSec = (time/ 1000000) % 10;
  int TenSec = (time/ 10000000)%6;
  // Read sensor every 10 milliseconds
  if (time - lastReadingTime >= readingInterval) {
    lastReadingTime = time; 
  
    //while on launchpad after pin has been pulled
    if(liftoff == 0){
      temperature = myPressure.readTempF();
      pressure = myPressure.readPressure();
      Altitude_AGL =(((T_0)/(L_b))*(pow((pressure/P_0),((-R*L_b)/(g_0*M)))-1)) - h_0; //Formula to detect altitude 
      myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card  
      if (myFile) {      
        myFile.print("T-");myFile.print(time/60000000);myFile.print(":");myFile.print(TenSec);myFile.print(OneSec);myFile.print(":");myFile.print(TenthSec);myFile.print(HundrethSec);myFile.print(MilliSec);myFile.print(HundredMicro);myFile.print(TenMicro);myFile.print(Micro);
        myFile.print(" Pressure(Pa):");
        myFile.print(pressure, 2);
        myFile.print(" Temp(f):");
        myFile.print(temperature, 2); 
        myFile.print(" Altitude(m):");
        myFile.println(Altitude_AGL, 2);
        myFile.close();//Close file
      }
    }
    //check for liftoff
    if(time>3000000 && Altitude_AGL >= threshold && liftoff == 0 ) {
      myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
      if (myFile) {
        myFile.println("LIFT OFF DETECTED");
        myFile.close();//Close file
      }
      LOTime = time;
      liftoff = 1;
    }
    //Lockout period
    if ((liftoff == 1) && (lockout == 1) ){
      LiftOffTime = time - LOTime;
      int HundrethSec = (LiftOffTime/10000) % 10;
      int TenthSec = (LiftOffTime/100000) % 10;
      int OneSec = (LiftOffTime/ 1000000) % 10;
      int TenSec = (LiftOffTime/ 10000000)%6;
      pressure = myPressure.readPressure();
      temperature = myPressure.readTempF();   
      Altitude_AGL =(((T_0)/(L_b))*(pow((pressure/P_0),((-R*L_b)/(g_0*M)))-1)) - h_0; //Formula to detect altitude 
      myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
      if (myFile) {
        myFile.print("T+");myFile.print(LiftOffTime/60000000);myFile.print(":");myFile.print(TenSec);myFile.print(OneSec);myFile.print(":");myFile.print(TenthSec);myFile.print(HundrethSec);myFile.print(MilliSec);myFile.print(HundredMicro);myFile.print(TenMicro);myFile.print(Micro);
        myFile.print(" Pressure(Pa):");
        myFile.print(pressure, 2);
        myFile.print(" Temp(f):");
        myFile.print(temperature, 2); 
        myFile.print(" Altitude(m):");
        myFile.print(Altitude_AGL, 2);
        myFile.println(" LOCKOUT PERIOD");
        myFile.close();//Close file
        }      
      if( LiftOffTime > SuperSonicCount){//if simulated lockout period is over  
        if (isLockoutOver(Altitude_AGL)) {//begin to check for 10 consectutive increasing values
          myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
          if (myFile) {

            myFile.println("Lockout Period Over");
            myFile.close();//Close file
          }
          lockout = 0;
        }
      }
    }
    
    //if lockout period is over startlooking for apogee
    if(liftoff == 1 && lockout == 0 && Apogee ==0 ){
      LiftOffTime = time - LOTime;
      int HundrethSec = (LiftOffTime/10000) % 10;
      int TenthSec = (LiftOffTime/100000) % 10;
      int OneSec = (LiftOffTime/ 1000000) % 10;
      int TenSec = (LiftOffTime/ 10000000)%6;
      temperature = myPressure.readTempF();
      pressure = myPressure.readPressure();    
      Altitude_AGL =(((T_0)/(L_b))*(pow((pressure/P_0),((-R*L_b)/(g_0*M)))-1)) - h_0; //Formula to detect altitude 
      myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
      if (myFile) {
        myFile.print("T+");myFile.print(LiftOffTime/60000000);myFile.print(":");myFile.print(TenSec);myFile.print(OneSec);myFile.print(":");myFile.print(TenthSec);myFile.print(HundrethSec);myFile.print(MilliSec);myFile.print(HundredMicro);myFile.print(TenMicro);myFile.print(Micro);
        myFile.print(" Pressure(Pa):");
        myFile.print(pressure, 2);
        myFile.print(" Temp(f):");
        myFile.print(temperature, 2); 
        myFile.print(" Altitude(m):");
        myFile.println(Altitude_AGL, 2);
        myFile.close();//Close file
      }
      if(isApogee(pressure)){
        myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
        if (myFile) {
          myFile.println("Apogee Detected");
          myFile.close();//Close file
        }
        Apogee = 1;
        digitalWrite(SOLENOID1, HIGH);
        digitalWrite(SOLENOID2, HIGH);
        TurnOffValve = 1;
        ApogeeTime = LiftOffTime;
        if (!igniterOn) {
          // Turn on igniter
          digitalWrite(IGNITER, HIGH);
          igniterOn = true;
        }  
      }
    }
    
   
    //After apogee wait 45 seconds to turn of solenoid, and open actuator at 400 ft/121.92 meters
    if(Apogee == 1 && liftoff==1 && lockout ==0){ 
      LiftOffTime = time - LOTime;
      int HundrethSec = (LiftOffTime/10000) % 10;
      int TenthSec = (LiftOffTime/100000) % 10;
      int OneSec = (LiftOffTime/ 1000000) % 10;
      int TenSec = (LiftOffTime/ 10000000)%6;  
      temperature = myPressure.readTempF();
      pressure = myPressure.readPressure();
      Altitude_AGL =(((T_0)/(L_b))*(pow((pressure/P_0),((-R*L_b)/(g_0*M)))-1)) - h_0; //Formula to detect altitude 
      
      digitalWrite(IGNITER, LOW);//turn off ignitor pin
      AfterApogee = LiftOffTime - ApogeeTime;
      int solenoidtimeO = (AfterApogee/ 1000000) % 10;
      int solenoidtimeT = (AfterApogee/ 10000000) % 10;
      myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
      if (myFile) {
        myFile.print("T+");myFile.print(LiftOffTime/60000000);myFile.print(":");myFile.print(TenSec);myFile.print(OneSec);myFile.print(":");myFile.print(TenthSec);myFile.print(HundrethSec);myFile.print(MilliSec);myFile.print(HundredMicro);myFile.print(TenMicro);myFile.print(Micro);
        myFile.print(" Pressure(Pa):");
        myFile.print(pressure, 2);
        myFile.print(" Temp(f):");
        myFile.print(temperature, 2); 
        myFile.print(" Altitude(m):");
        myFile.print(Altitude_AGL, 2);
        myFile.print(" | Descending Time:");
        myFile.print(solenoidtimeT);
        myFile.println(solenoidtimeO);
        myFile.close();//Close file
      }
      if((TurnOffValve == 1) && (DrainTime < AfterApogee)){
         myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
        if (myFile) {
          myFile.println("Turn off pumps and close solenoid valve");
          myFile.close();//Close file
        }
        digitalWrite(SOLENOID1, LOW);
        digitalWrite(SOLENOID2, LOW);
        TurnOffValve = 0; // solenoid and pumps have been turned off
        DroneDeploy = 1;//Ready to search drone deployment altitude
      }
      if((Altitude_AGL < DroneDeployment) && (DroneDeploy ==1)){
        myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
        if (myFile) {
          myFile.println("Deploying drone...");
          myFile.close();//Close file
        } 
        digitalWrite(Actuator, HIGH);

        DroneDeploy = 0;
        TouchDown = 1;
      }      
      if((Altitude_AGL < Landing) && (TouchDown ==1)){
        myFile = SD.open("Phoenix.txt", FILE_WRITE); //open file on SD card
        if (myFile) {
          myFile.println("Welcome home Phoenix!");
          myFile.close();//Close file
        } 
        TouchDown = 0;
        while(1){};
      }
    }
  }

}
