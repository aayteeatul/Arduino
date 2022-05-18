// **************************
// *** Required libraries ***
// **************************

#include <ros.h>
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetFloat.h>
#include <std_msgs/Int32.h>
#include <ChRt.h>
#include <RTIncrementalEncoder.h>
#include "UnitConversion.h"
#include "PINList.h"

#define ENCODER_PERIOD_MS 10


// ********************************
// *** Variables initialization ***
// ********************************

//speed estimator
const double encoder_1_ratio = double(0.003272492347);
const double encoder_2_ratio = double(0.003272492347);

double p = 100;
double i = 50;
double a = 0.4;

double pos_1;
double vel_1;
int pos_2;
int vel_2;


// *******************************
// *** Encoders initialization ***
// *******************************

RTIncrementalEncoder encoder_1(ConvertMsToS(ENCODER_PERIOD_MS),true, encoder_1_ratio, 0, true, true);
RTIncrementalEncoder encoder_2(ConvertMsToS(ENCODER_PERIOD_MS),true, encoder_2_ratio, 0, true, true);


// ******************************
// *** Threads implementation ***
// ******************************


THD_WORKING_AREA(waThdEncoderRead, 512);
THD_FUNCTION(ThdEncoderRead, arg)
{
  (void)arg;
  systime_t encoderTime = chVTGetSystemTime();

  while(1) {

    encoderTime += TIME_MS2I(ENCODER_PERIOD_MS);
    chThdSleepUntil(encoderTime);

    encoder_1.update();
    //encoder_2.update();
    
    pos_1 = encoder_1.getPos();
    vel_1 = encoder_1.getVel();
    /*
    Serial.print(pos_1,10);
    Serial.print(',');
    Serial.println(vel_1,10);
    */
    
    Serial.print(pos_1);
    Serial.print(' ');
    Serial.print(vel_1);
    Serial.print(' ');
    Serial.println();
    
    //pos_2 = encoder_2.getPos();
    //vel_2 = encoder_2.getVel();
  }
}


// ***********************
// *** Setup functions ***
// ***********************

void chSetup()
{
  chThdCreateStatic(waThdEncoderRead, sizeof(waThdEncoderRead), NORMALPRIO+2, ThdEncoderRead, NULL);
}

void setupMotors()
{  
  encoder_1.setLoopTrackerPID(p, i);
  encoder_1.setAlphaFilter(a);
  //encoder_2.setLoopTrackerPID(15, 3);
  //encoder_2.setAlphaFilter(1);
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(ENCODER_1_CHA, INPUT_PULLUP);
  pinMode(ENCODER_1_CHB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_CHA), set_chA_enc_motor_1_state, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_CHB), set_chB_enc_motor_1_state, CHANGE);
  
  //pinMode(ENCODER_2_CHA, INPUT_PULLUP);
  //pinMode(ENCODER_2_CHB, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_2_CHA), set_chA_enc_motor_2_state, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_2_CHB), set_chB_enc_motor_2_state, CHANGE);   
  
  setupMotors();
  
  chBegin(chSetup);
  while(1) {}
}


// ***************************
// *** Interrupt functions ***
// ***************************

void set_chA_enc_motor_1_state()
{
  encoder_1.setChAState(bool(digitalRead(ENCODER_1_CHA)));
}

void set_chB_enc_motor_1_state()
{
  encoder_1.setChBState(bool(digitalRead(ENCODER_1_CHB)));
}

/*
void set_chA_enc_motor_2_state()
{
  encoder_2.setChAState(bool(digitalRead(ENCODER_2_CHA)));
}

void set_chB_enc_motor_2_state()
{
  encoder_2.setChBState(bool(digitalRead(ENCODER_2_CHB)));
}
*/


// **************************
// *** Empty loop funtion ***
// **************************

void loop() {}
