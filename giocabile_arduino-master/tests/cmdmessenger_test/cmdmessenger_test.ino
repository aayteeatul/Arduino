#include <CmdMessenger.h>

CmdMessenger cmdMessenger = CmdMessenger(Serial);

enum
{
  Data,
};

void setup()
{
  Serial.begin(115200); 
}

void loop()
{
  cmdMessenger.sendCmd(Data,"New message");
  delay(1000);
}
