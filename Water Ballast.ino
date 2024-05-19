
const int SOLENOID1 = 7;  //first set
const int SOLENOID2 = 5;  //second set

void setup() {
  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  Serial.print("Setting solenoid pin to HIGH");
  digitalWrite(SOLENOID1, HIGH);
  digitalWrite(SOLENOID2, HIGH);

  // Solenoids and Pumps turned on for 30 s. Change if needed.
  delay(30000);
  Serial.print("Setting solenoid pin to LOW");
  digitalWrite(SOLENOID1, LOW);
  digitalWrite(SOLENOID2, LOW);
  delay(1000);

}
