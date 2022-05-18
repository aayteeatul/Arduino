#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void setup()
{
  Wire.begin();
  Serial.begin(115200);
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); 
 
}

uint8_t effect = 1;

void loop()
{
  TCA9548A(3);    // select I2C bus 0 for the DRV2605L

 // set the effect to play
  drv.setWaveform(0,16);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();

  // wait a bit
  //delay(1);

  effect++;
  if (effect > 117 ) effect = 1;




    TCA9548A(6);    // select I2C bus 0 for the DRV2605L

 // set the effect to play
  drv.setWaveform(0,16);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();

  // wait a bit
  //delay(1);

  effect++;
  if (effect > 117 ) effect = 1;




    TCA9548A(2);    // select I2C bus 0 for the DRV2605L

 // set the effect to play
  drv.setWaveform(0,16);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();

  // wait a bit
  //delay(1);

  effect++;
  if (effect > 117 ) effect = 1;

  
}
