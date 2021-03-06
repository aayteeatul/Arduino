#include <AS5600.h>
#include <USBHost_t36.h>      // to enable 5V output in teensy 4.1

AMS_5600 ams5600;
USBHost myusb;

float absPos, incrPos;
float prevA = 0;
float currA = 0;
int counter = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();  
  myusb.begin();
  absPos = ams5600.getRawAngle()*(float)360/4096;
  prevA = absPos; 
}

void loop()
{
  absPos = ams5600.getRawAngle()*(float)360/4096;
  currA = absPos;
  if (abs(currA - prevA) >= 345 ) {
    if( (currA-prevA) >= 345) {
      counter = counter -1;  
    }
    if( (currA-prevA) <= -345) {
      counter = counter + 1;  
    }   
  }
  incrPos = counter*360 + absPos;
  Serial.print(absPos,5);
  Serial.print(" , ");
  Serial.println(incrPos,5);
  prevA = currA;
}
