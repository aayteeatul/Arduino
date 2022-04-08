//Do not sort the enable pin and Connect HC-05 RXD to TX1 (18) and HC-05 TXD to RX1 (19) of Arduino Due
// It will print the same thing on the serial Monitor thaat we are sending from connected device to the bluetooth.
// Then it will send back the same command to the connected device
// Here, I am using the Serial1 communication and hence it will not work in Arduino UNO.
void setup()
{
  delay(500);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.write("\nHC-05 bluetooth test\n");
}

void loop(){
  while(Serial1.available()){
    char data1 = Serial1.read();
    Serial.write(data1);
    Serial1.print(data1);
  }
}
