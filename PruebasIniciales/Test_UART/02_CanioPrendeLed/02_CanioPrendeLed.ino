

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    digitalWrite(5, !digitalRead(5));
    byte c = Serial.read();
    Serial.write(c);
  }
}
