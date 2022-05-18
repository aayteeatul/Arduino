#include <ChibiOS_AVR.h>
#define SERIAL_PRINT_PERIOD_MS 10

systime_t wakeTime;
static WORKING_AREA(waThSerialPrint, 128);

static msg_t ThSerialPrint(void *arg)
{
  wakeTime = chTimeNow();
  
  while (1) {

    wakeTime += MS2ST(SERIAL_PRINT_PERIOD_MS);
    chThdSleepUntil(wakeTime);
      
    Serial.print(chTimeNow());
    Serial.print(' ');
    Serial.println(wakeTime);
  }
}


void chSetup()
{
  chThdCreateStatic(waThSerialPrint, sizeof(waThSerialPrint), NORMALPRIO+3, ThSerialPrint, NULL);
}


void setup()
{
  Serial.begin(115200);
  
  chBegin(chSetup);
  while(1) {}
}

void loop() {}
