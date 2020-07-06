void setup() {
  pinMode(5, OUTPUT);
}

void loop() {
  digitalWrite(5, HIGH);   // Led ON
  delay(1000);              // pausa 1 seg
  digitalWrite(5, LOW);    // Led OFF
  delay(1000);              // pausa 1seg
}
