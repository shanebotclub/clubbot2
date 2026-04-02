void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT OK");
}

void loop() {
  Serial.println("Blink");
  delay(500);
}
