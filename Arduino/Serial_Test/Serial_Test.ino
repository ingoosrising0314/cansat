void setup() {
  Serial.begin(9600);
}

void loop() {
  for (byte n=0; n <256; n++){
    Serial.write(n);
    delay(500);
  }
}
