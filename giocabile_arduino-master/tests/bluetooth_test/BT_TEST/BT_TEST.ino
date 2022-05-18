
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
}

void loop() {
  int x = millis();
  
  while(Serial1.available()) {
    Serial1.print(millis());
  }
  while(!Serial1.available()) {
    Serial.println(" NOT available");
  }
  delay(500);
}
