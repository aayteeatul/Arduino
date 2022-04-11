// This is written for the Arduino-DUE, for using this sketch in Arduino Uno/Mega, replace Wire1 with Wire and comment "extern TwoWire Wire"

#include <Wire.h> //include Wire.h library
extern TwoWire Wire1;

void setup()
{
  Wire1.begin(); // Wire communication begin
  Serial.begin(115200); // The baudrate of Serial monitor is set in 115200
  while (!Serial); // Waiting for Serial Monitor
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000); // wait 2 seconds for the next I2C scan
}
