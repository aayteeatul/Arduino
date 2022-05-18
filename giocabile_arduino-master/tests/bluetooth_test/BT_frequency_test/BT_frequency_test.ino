#include <SoftwareSerial.h>

#define PIN_X A0
#define PIN_Y A1
#define PIN_BUTTON 2
//NOTE: Bt module pinout is inverted! (TX=RX and RX=TX)
#define PIN_TX 11
#define PIN_RX 10


int x;
int y;
int button;
unsigned long lastPbTime;
unsigned long reqPbTime;

SoftwareSerial bt(PIN_RX, PIN_TX);


void setup() {
  Serial.begin(9600);
  bt.begin(38400);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  reqPbTime = 1000.0/30.0;
  btPublish();
}


void loop() {
  if ((millis()-lastPbTime) > reqPbTime) {
    btPublish();
  }
}


void btPublish() {
  Serial.println(1000.0/(millis()-lastPbTime));
  lastPbTime = millis();
  x = analogRead(PIN_X);
  y = analogRead(PIN_Y);
  button = !digitalRead(PIN_BUTTON);

  String msg = String(x) + String(y) + String(button) + String(" ");  
  bt.print(msg);
}
