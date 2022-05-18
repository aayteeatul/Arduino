#include "SingleVNH5019Motor.h"

#define INA1 11
#define INB1 4
#define PWM1 9
#define EN1DIAG1 6
#define CS1 A0
#define INA2 7
#define INB2 8
#define PWM2 10
#define EN2DIAG2 12
#define CS2 A1


DualVNH5019MotorShield ms(INA1,INB1,EN1DIAG1,CS1,PWM1,INA2,INB2,EN2DIAG2,CS2,PWM2);
SingleVNH5019Motor motor_1(&ms, 1);
SingleVNH5019Motor motor_2(&ms, 2);

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
}

void loop()
{
  motor_1.setSpeed(100);
  motor_2.setSpeed(0);
  delay(2000);
  motor_1.setSpeed(0);
  motor_2.setSpeed(100);
  delay(2000); 
}
