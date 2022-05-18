/*  @file Interrupt_test.ino
    @author Matteo Lavit Nicora (matteo.lavit@stiima.cnr.it)
    @version Revision 0.1
    @brief Sketch used to test interrupts on all Arduino Uno Wifi Rev2 pins.
    @details The application requires an Arduino Uno Wifi Rev2  and an LM393 sensor used to trigger the interrupt for each selected pin.
    @date Tuesday, March 15th, 2021
*/


#define PIN_A 0
#define PIN_B 1
int onRef;


void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_A,INPUT);
  pinMode(PIN_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_A), ISR_PIN_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), ISR_PIN_B, CHANGE);
  Serial.begin(9600);
}


void loop()
{
  if (onRef) {
    digitalWrite(LED_BUILTIN,HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN,LOW);
  }
  Serial.println("Testing interrupts while serial communication in active");
  //delay(2000);
}


void ISR_PIN_A()
{
  onRef = HIGH;
}


void ISR_PIN_B()
{
  onRef = LOW;
}
