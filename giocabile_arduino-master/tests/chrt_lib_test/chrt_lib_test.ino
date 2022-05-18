#include <ChRt.h>

#define SERIAL_PRINT_PERIOD 1
int ii;
THD_WORKING_AREA(waThread, 128);
THD_FUNCTION(Thread, arg)
{
  (void)arg;
  systime_t wakeTime = chVTGetSystemTime();
  while(1) {
    wakeTime += TIME_MS2I(SERIAL_PRINT_PERIOD);
    chThdSleepUntil(wakeTime);
    Serial.println(micros());
  }
}


void chSetup()
{
  chThdCreateStatic(waThread, sizeof(waThread), NORMALPRIO+2, Thread, NULL);
}


void setup()
{
  Serial.begin(115200);
  while(!Serial) {}

  chBegin(chSetup);
  while(1) {}
}


void loop() {
  }
