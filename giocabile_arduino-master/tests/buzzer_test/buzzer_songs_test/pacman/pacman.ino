#include "Pacman.h"

Pacman pacman;

void setup()
{
  Serial.begin(9600);
  Serial.print("Tempo: ");
  Serial.println(pacman.getTempo());
  Serial.print("Buzzer pin: ");
  Serial.println(pacman.getBuzzerPin());

  pacman.play();
}

void loop()
{
  delay(1000);
}
