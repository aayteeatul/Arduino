#include <AS5600.h>
AMS_5600 ams5600;

float output;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  //delay(500);
}

void loop()
{
  output = ams5600.getRawAngle()*(float)360/4096;
  Serial.println(output,5);
}
