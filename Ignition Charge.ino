const int IGNITER = 8;
bool igniterOn = false;

void setup() {
  pinMode(IGNITER, OUTPUT);
  Serial.begin(9600);
  delay(10000); // Delay for 10 seconds
}

void loop() {
  if (!igniterOn) {
    // Turn on igniter
    digitalWrite(IGNITER, HIGH);
    igniterOn = true;
    Serial.println("Igniter turned on.");
  }
}
