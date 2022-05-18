#include <Wire.h>

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


bool msg_rcvd = false;
int x;


void receiveEvent(int bytes)
{
  x = Wire.read();
  msg_rcvd = true;
}


void setup()
{
  Serial.begin(115200);
  Wire.begin(0x70);
  Wire.onReceive(receiveEvent);
  TCA9548A(1);
}


void loop()
{
  if (msg_rcvd) {
    Serial.println(x);
    Wire.beginTransmission(0x09);
    Wire.write(1);
    Wire.endTransmission();
    msg_rcvd = false;
  }
  delay(500);
}
