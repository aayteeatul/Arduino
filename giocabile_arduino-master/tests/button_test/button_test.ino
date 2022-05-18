void setup()
{
  Serial.begin(115200);
  pinMode(A3, INPUT_PULLUP);
}

void loop()
{
  Serial.println(digitalRead(A3));
  delay(10);
}
