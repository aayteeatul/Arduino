#include <Wire.h>


bool msg_rcvd = false;
int x;


void setup()
{
  Serial.begin(9600);
  Wire.begin(0x36);
  Wire.onReceive(receiveEvent);
}


void receiveEvent(int bytes)
{
  x = Wire.read();
  
  Serial.print("Received ");
  Serial.print(bytes);
  Serial.println(" bytes:");
  Serial.println(x);
  msg_rcvd = true;
}


void loop()
{
  if (msg_rcvd) {
    Wire.beginTransmission(9);
    Wire.write(1);
    Wire.endTransmission();
    msg_rcvd = false;
  }
  delay(500);
}
