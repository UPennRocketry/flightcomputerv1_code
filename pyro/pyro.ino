void setup() {
  // Set pins 2, 3, 4, 40, and 41 as outputs
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);

  // Turn on LEDs on pins 2, 3, 4
  //digitalWrite(2, HIGH);
  //digitalWrite(3, HIGH);
  //digitalWrite(4, HIGH);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}

void loop() {
  digitalWrite(40, LOW);
  digitalWrite(41, LOW);
  digitalWrite(2, LOW);
  delay(10000);
  digitalWrite(40, HIGH);
  digitalWrite(41, HIGH);
  digitalWrite(2, HIGH);
  delay(10000);
}
