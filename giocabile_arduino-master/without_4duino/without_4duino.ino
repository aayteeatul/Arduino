// **************************
// *** Required libraries ***
// **************************

#include <CmdMessenger.h>
#include <Wire.h>
#include <ChRt.h>

#include <DualVNH5019MotorShield.h>
#include <RTIncrementalEncoder.h>
#include <SingleVNH5019Motor.h>

#include "UnitConversion.h"
#include "Management.h"
#include "PINList.h"
#include "CCMotor.h"
#include "Robot.h"
#include "Notes.h"


// **************************
// *** Thread frequencies ***
// **************************

#define ROBOT_CTRL_PERIOD_MS 10
#define STATE_MACHINE_PERIOD_MS 20


// **********************
// *** Init variables ***
// **********************

#define pi 3.14159265

LLState actual_LL_state = IDLE;
HLState actual_HL_state = INITIALIZING;
LLMode actual_LL_mode = NO_MODE;
HLMode actual_HL_mode = CONFIGURATION_1;
Error actual_error = NO_ERROR;

bool homingAtStartup = false;
bool soundOn = false;
bool homingRequested = false;
bool initializationRequested = false;
bool setupDone = false;
bool powerLost = false;
bool assist = false;
bool timer_begin = false;
double homingVelocity = 1.5;
double currentTarget1 = 0;
double currentTarget2 = 0;
double timer = 0;
double timer1 = 0;
double accn = 0.025;
double equilibriumPos1, equilibriumPos2;
double tunnel_Vel_Error;
double ref_tunnel_velocity = 3.0;
int signalCount = 0;

enum
{
  Data,
};

// ********************************
// *** Robot function prototype ***
// ********************************

DualVNH5019MotorShield ms(INA1,INB1,EN1DIAG1,CS1,PWM1,INA2,INB2,EN2DIAG2,CS2,PWM2);
SingleVNH5019Motor motor_1(&ms, 1);
SingleVNH5019Motor motor_2(&ms, 2);

const double encoder_1_ratio = double(0.003272492347);  //(2*pi)/1920
const double encoder_2_ratio = double(0.003272492347);  //(2*pi)/1920
RTIncrementalEncoder encoder_1(ConvertMsToS(ROBOT_CTRL_PERIOD_MS),true, encoder_1_ratio, 0, true, true);
RTIncrementalEncoder encoder_2(ConvertMsToS(ROBOT_CTRL_PERIOD_MS),true, encoder_2_ratio, 0, true, true);

CCMotorCtrl motor_1_ctrl(&motor_1, ConvertMsToS(ROBOT_CTRL_PERIOD_MS), encoder_1);
CCMotorCtrl motor_2_ctrl(&motor_2, ConvertMsToS(ROBOT_CTRL_PERIOD_MS), encoder_2);
Robot robot(ConvertMsToS(ROBOT_CTRL_PERIOD_MS));

CmdMessenger cmdMessenger = CmdMessenger(Serial);

// ****************************
// *** Motor control thread ***
// ****************************

THD_WORKING_AREA(waThdMotorCtrl,1024);
THD_FUNCTION(ThdMotorCtrl, arg)
{
  (void)arg;
  systime_t motorCtrlTime = chVTGetSystemTime();

  while (1) {
    
    motorCtrlTime += TIME_MS2I(ROBOT_CTRL_PERIOD_MS);
    chThdSleepUntil(motorCtrlTime);

    robot.update();
  }
}


// ****************************
// *** State machine thread ***
// ****************************

THD_WORKING_AREA(waThdStateMachine, 1024);
THD_FUNCTION(ThdStateMachine, arg)
{
  (void)arg;
  systime_t stateMachineTime = chVTGetSystemTime();

  while (1) {
    
    stateMachineTime += TIME_MS2I(STATE_MACHINE_PERIOD_MS);
    chThdSleepUntil(stateMachineTime);

    updateLLStateMachine();
    updateHLStateMachine();
  }
}


// *****************************
// *** Robot control methods ***
// *****************************

void(* resetBoard)(void) = 0;

void updateLLStateMachine()
{
  robot.setControlMode(Actuator::IMPEDANCE);
  if (robot.isOn()) {
    actual_LL_state = ON;
  }
  else {
    actual_LL_state = IDLE;
  }

  //low-level mode
  switch(robot.getControlMode()) {
    case(Actuator::POSITION):
      actual_LL_mode = POSITION;
    break;
    case(Actuator::VELOCITY):
      actual_LL_mode = VELOCITY;
    break;
    case(Actuator::IMPEDANCE):
      actual_LL_mode = IMPEDANCE;
    break;
    case(Actuator::HOMING):
      actual_LL_mode = HOMING;
    break;
    case(Actuator::NO_MODE):
      actual_LL_mode = NO_MODE;
    break;
  }
}


void updateHLStateMachine()
{
  //high-level state
  switch (actual_HL_state) {
    
    case INITIALIZING:
      //line below is the extra one. delete it after the installing back the 4duino.
      initializationRequested = false;
      robot.setControlMode(Actuator::IMPEDANCE);
      if (initializationRequested) {
        double normalizedPos1, normalizedPos2; 
        if (!setupDone) {
          motor_1_ctrl.setMaxVelocity(10e9);             
          motor_1_ctrl.setMaxVelTrackingError(10e9);
          motor_2_ctrl.setMaxVelocity(10e9);             
          motor_2_ctrl.setMaxVelTrackingError(10e9); 
          motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
          motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
          int i1 = 1;
          int i2 = 0;
          if (motor_1_ctrl.getPos() < 0) {
            i1 = -1;
            i2 = 1; 
          }
          normalizedPos1 = abs(motor_1_ctrl.getPos()) - floor(abs(motor_1_ctrl.getPos())/(2*pi))*(2*pi);
          normalizedPos1 = i2*2*pi + i1*normalizedPos1; 
          i1 = 1;
          i2 = 0;
          if (motor_2_ctrl.getPos() < 0) {
            i1 = -1;
            i2 = 1; 
          }    
          normalizedPos2 = abs(motor_2_ctrl.getPos()) - floor(abs(motor_2_ctrl.getPos())/(2*pi))*(2*pi);
          normalizedPos2 = i2*2*pi + i1*normalizedPos2;
          encoder_1.reset(normalizedPos1);
          encoder_2.reset(normalizedPos2);
          currentTarget1 = encoder_1.getPos();
          currentTarget2 = encoder_2.getPos();
          motor_1_ctrl.setTargetImp(currentTarget1); 
          motor_2_ctrl.setTargetImp(currentTarget2);
          setupDone = true;  
        }
        if (motor_1_ctrl.getPos() == encoder_1.getPos() && abs(motor_1_ctrl.getVel()) < 5) {
          if (motor_2_ctrl.getPos() == encoder_2.getPos() && abs(motor_2_ctrl.getVel()) < 5) {
            initializationRequested = false; 
            setupDone = false;   
          }   
        }
      }
      else {
        if (robot.getControlMode() != Actuator::IMPEDANCE) {
          robot.turnOff();
          robot.setControlMode(Actuator::IMPEDANCE);
          robot.turnOn(); 
        }
        else {
          switch (actual_HL_mode) {
            case CONFIGURATION_1: // car wheel
              doInitialization(10,10,10,10,0.0,1.65588116);
            break;
            /*
            case CONFIGURATION_2: // bike wheel
              doInitialization(10,10,10,10,0.0,3.14813780);
            break;   
            case CONFIGURATION_3: // vertical differential
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;
            case CONFIGURATION_4: // horizontal differential
              doInitialization(10,10,10,10,1.59515109,-1.30062675);
            break;
            */
            case CONFIGURATION_5: // left damping right stiffness
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            
            case CONFIGURATION_6: // right damping left stiffness
              doInitialization(10,10,10,10,0.0,0.0);
            break;          
          }
        }
      }  
    break;
    
    case INITIALIZED:
      switch (actual_HL_mode) {
        case CONFIGURATION_1: // car wheel
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_2: // bike wheel
          if (!setupDone) {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true;  
          }
        break;
        case CONFIGURATION_3: // vertical differential
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2);
            setupDone = true; 
          }
        break;
        case CONFIGURATION_4: // horizontal differential
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_5: // left damping right stiffness
          if (!setupDone)  {
            robot.turnOff();
            robot.setControlMode(Actuator::VELOCITY);
            robot.turnOn();
            motor_1_ctrl.setPIDVelocityParameters(70,30,1.2);
            motor_2_ctrl.setPIDVelocityParameters(120,0,1.5);
            //motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            //motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
          if (setupDone == true) {
            float currentSensing = analogRead(CS2);
            if (motor_2_ctrl.getVel() >= 3.5) {
              currentSensing = -currentSensing;
            }
            Serial.print(ref_tunnel_velocity);
            Serial.print(",");
            Serial.print(motor_2_ctrl.getTargetVel());
            Serial.print(",");
            Serial.print(motor_2_ctrl.getVel());
            Serial.print(",");
            Serial.print(motor_2_ctrl.getCurrent());
            Serial.print(",");
            Serial.println(currentSensing);
            
            if (motor_2_ctrl.getVel() < 0.15) {
              motor_2_ctrl.setTargetVel(0.0);
              accn = 0;
              assist = false;
            }
            if (timer_begin == true && motor_2_ctrl.getVel() < (ref_tunnel_velocity - 0.25)) {
              timer = timer + 1;
              if (timer <= 150) {
                motor_2_ctrl.setTargetVel(motor_2_ctrl.getVel());
                accn = motor_2_ctrl.getVel();
              }
              else {
                timer_begin = false;
              }
              /* same function as the else command is doing.... but check for all the possible situations once
              if (timer > 150 && timer <= 300) {
                accn = accn + 0.01;
                if (accn >= (ref_tunnel_velocity - 0.5)) {
                  accn = (ref_tunnel_velocity - 0.5);
                }
                motor_2_ctrl.setTargetVel(accn);
                timer_begin = false;
              }
              */
            }
            if (motor_2_ctrl.getVel() <= (ref_tunnel_velocity - 0.25) && motor_2_ctrl.getVel() >= 0.2 && timer_begin == false) {
              if (assist == false) {
                motor_2_ctrl.setTargetVel(accn);
                accn = accn + 0.01;
              }
              if (motor_2_ctrl.getVel() >= ref_tunnel_velocity - 0.5) {
                motor_2_ctrl.setTargetVel(motor_2_ctrl.getTargetVel());
                assist = true; 
              }
              if ((motor_2_ctrl.getTargetVel() - motor_2_ctrl.getVel()) >= 1.0) {
                double target_temp = motor_2_ctrl.getVel() + 1.0;
                if (target_temp > (ref_tunnel_velocity + 0.5)) {
                  target_temp = (ref_tunnel_velocity + 0.5);
                }
                motor_2_ctrl.setTargetVel(target_temp);
                assist = false;
                accn = motor_2_ctrl.getTargetVel();
              }
              
              if ((motor_2_ctrl.getTargetVel() - motor_2_ctrl.getVel()) <= 0.1) {
                double target_temp = (motor_2_ctrl.getVel()+ 0.1);
                if (target_temp >= (ref_tunnel_velocity-0.25)) {
                  target_temp = (ref_tunnel_velocity - 0.25);
                }
                motor_2_ctrl.setTargetVel(target_temp);
                assist = false;
                accn = motor_2_ctrl.getTargetVel();
              }
            
            }
            if (motor_2_ctrl.getVel() <= (ref_tunnel_velocity + 0.5) && motor_2_ctrl.getVel() > (ref_tunnel_velocity - 0.25)) {
              motor_2_ctrl.setTargetVel(motor_2_ctrl.getVel());
              timer_begin = true;
              timer = 0;           
            }
            if (motor_2_ctrl.getVel() > (ref_tunnel_velocity + 0.5)) {
              motor_2_ctrl.setTargetVel(ref_tunnel_velocity + 0.5);
            }

            
            /*
            if (motor_2_ctrl.getVel() < ref_tunnel_velocity ) {
              if (assist == false) {
                motor_2_ctrl.setTargetVel(accn);
                accn = accn + 0.025;
              }
              if (motor_2_ctrl.getVel() >= ref_tunnel_velocity - 0.25) {
                assist = true; 
              }
              if (abs(motor_2_ctrl.getTargetVel() - motor_2_ctrl.getVel()) >= 1.5) {
                motor_2_ctrl.setTargetVel(motor_2_ctrl.getVel());
              }
              if (abs(motor_2_ctrl.getTargetVel() - motor_2_ctrl.getVel()) <= 0.5) {
                accn = motor_2_ctrl.getVel();
                assist = false;
              }
              if (motor_2_ctrl.getVel() <= 1.75 && assist == true) {
                accn = motor_2_ctrl.getVel();
                assist = false;
              }  
            }
            else { 
              accn = motor_2_ctrl.getVel();
              motor_2_ctrl.setTargetVel(ref_tunnel_velocity); 
            }
            */
          }
        break;
        case CONFIGURATION_6 : // right damping left stiffness
          
          if (!setupDone)  {
            robot.turnOff();
            robot.setControlMode(Actuator::VELOCITY);
            robot.turnOn();
            //motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            //motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setPIDVelocityParameters(70,30,1.2);
            motor_2_ctrl.setPIDVelocityParameters(70,0,1.2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
          if (setupDone == true) {
            /*Serial.print(ref_tunnel_velocity);
            Serial.print(",");
            Serial.print(ref_tunnel_velocity + 0.5);
            Serial.print(",");
            Serial.print(ref_tunnel_velocity - 0.5);
            Serial.print(",");
            Serial.print(0.5);
            Serial.print(",");
            Serial.println(abs(motor_2_ctrl.getVel()));  
            */         
            tunnel_Vel_Error =  (ref_tunnel_velocity - abs(motor_2_ctrl.getVel()));
            if (abs(tunnel_Vel_Error) <= 0.5) {
              timer = 0;
              timer1 = 0;
              motor_2_ctrl.setTargetVel(motor_2_ctrl.getVel());   
            }
            if (abs(motor_2_ctrl.getVel()) > (ref_tunnel_velocity + 0.5)) {
              timer = 0;
              timer1 = timer1 + 1;
              motor_2_ctrl.setTargetVel(ref_tunnel_velocity + 0.5);
              if (timer1 > 100) {
                ref_tunnel_velocity = 4.0;  
              }            
            }
            if (abs(motor_2_ctrl.getVel()) < 0.5) {
              timer = 0;
              timer1 = 0;            
            } 
            if (abs(motor_2_ctrl.getVel()) >= 0.5 && abs(motor_2_ctrl.getVel()) <= (ref_tunnel_velocity - 0.5)) {
              timer = timer + 1;
              timer1 = 0;
              if (timer >= 100 && timer <= 200) {
                motor_2_ctrl.setTargetVel(ref_tunnel_velocity-0.5);   
              }
              else {
                motor_2_ctrl.setTargetVel(motor_2_ctrl.getVel());
              }
            } 
          }
        break;
      }
      digitalWrite(GREEN_LED, HIGH);
    break;
  }
}

void doInitialization(double _max_vel_1, double _max_vel_2, double _max_vel_track_1, double _max_vel_track_2, double _eq_pos_1, double _eq_pos_2) 
{
  if (!setupDone) {
    motor_1_ctrl.setMaxVelocity(_max_vel_1);             
    motor_1_ctrl.setMaxVelTrackingError(_max_vel_track_1);
    motor_2_ctrl.setMaxVelocity(_max_vel_2);             
    motor_2_ctrl.setMaxVelTrackingError(_max_vel_track_2);
    equilibriumPos1 = _eq_pos_1;
    equilibriumPos2 = _eq_pos_2; 
    setupDone = true;
  }
  if (motor_1_ctrl.getPos() < equilibriumPos1 && abs(motor_1_ctrl.getPos() - equilibriumPos1) > 0.35) {
    motor_1_ctrl.setTargetImp(currentTarget1);
    currentTarget1 = currentTarget1 + 0.06;
  }
  else if (motor_1_ctrl.getPos() > equilibriumPos1 && abs(motor_1_ctrl.getPos() - equilibriumPos1) > 0.35) {
    motor_1_ctrl.setTargetImp(currentTarget1);
    currentTarget1 = currentTarget1 - 0.06;
  }
  if (motor_2_ctrl.getPos() < equilibriumPos2 && abs(motor_2_ctrl.getPos() - equilibriumPos2) > 0.35) {
    motor_2_ctrl.setTargetImp(currentTarget2);
    currentTarget2 = currentTarget2 + 0.06;
  }
  else if (motor_2_ctrl.getPos() > equilibriumPos2 && abs(motor_2_ctrl.getPos() - equilibriumPos2) > 0.35) {
    motor_2_ctrl.setTargetImp(currentTarget2);
    currentTarget2 = currentTarget2 - 0.06;
  }
  if (abs(motor_1_ctrl.getPos() - equilibriumPos1) < 0.35 && abs(motor_2_ctrl.getPos() - equilibriumPos2) < 0.35) {
      actual_HL_state = INITIALIZED;
      setupDone = false;
   }                        
 }

// ***********************
// *** Setup functions ***
// ***********************

void chSetup()
{
  chThdCreateStatic(waThdMotorCtrl, sizeof(waThdMotorCtrl), NORMALPRIO+3, ThdMotorCtrl, NULL);
  chThdCreateStatic(waThdStateMachine, sizeof(waThdStateMachine), NORMALPRIO+2, ThdStateMachine, NULL);
}


void setupMotors()
{
  encoder_1.setLoopTrackerPID(70,0);
  encoder_1.setAlphaFilter(0.1);

  motor_1_ctrl.setPIDVelocityDirection(PID::DIRECT);
  motor_1_ctrl.setPIDVelocityParameters(70,30,1.2);
  motor_1_ctrl.setPIDPositionDirection(PID::DIRECT);
  motor_1_ctrl.setPIDPositionParameters(20,0,0.2);
  motor_1_ctrl.setPIDImpedanceDirection(PID::DIRECT);
  motor_1_ctrl.setPIDImpedanceParameters(0,0,0);
  motor_1_ctrl.setEndStrokeErrorEnabled(false);
  motor_1_ctrl.setWarningEnabled(false);
  motor_1_ctrl.setMaxVelocity(10e9);
  motor_1_ctrl.setMaxVelTrackingError(10e9);
  motor_1_ctrl.setMaxPosTrackingError(10e9);
  motor_1_ctrl.setMaxImpTrackingError(10e9);
  motor_1_ctrl.setHomingType(Actuator::MAX_ENDSTROKE_AND_HOLD);
  motor_1_ctrl.setHomingVelocity(0.0);
  motor_1_ctrl.setHomeOffset(0);

  encoder_2.setLoopTrackerPID(70,0);
  encoder_2.setAlphaFilter(0.1);

  motor_2_ctrl.setPIDVelocityDirection(PID::DIRECT);
  motor_2_ctrl.setPIDVelocityParameters(70,30,1.2);
  motor_2_ctrl.setPIDPositionDirection(PID::DIRECT);
  motor_2_ctrl.setPIDPositionParameters(20,0,0.2);
  motor_2_ctrl.setPIDImpedanceDirection(PID::DIRECT);
  motor_2_ctrl.setPIDImpedanceParameters(0,0,0);
  motor_2_ctrl.setEndStrokeErrorEnabled(false);
  motor_2_ctrl.setWarningEnabled(false);
  motor_2_ctrl.setMaxVelocity(10e9);             
  motor_2_ctrl.setMaxVelTrackingError(10e9);
  motor_2_ctrl.setMaxPosTrackingError(10e9);
  motor_2_ctrl.setMaxImpTrackingError(10e9);
  motor_2_ctrl.setHomingType(Actuator::MAX_ENDSTROKE_AND_HOLD);
  motor_2_ctrl.setHomingVelocity(0.0);
  motor_2_ctrl.setHomeOffset(0);

  robot.addActuator(&motor_1_ctrl);
  robot.addActuator(&motor_2_ctrl);
  robot.setDemo(false);

  robot.turnOff();
  robot.setControlMode(Actuator::NO_MODE);
  robot.turnOn();
  
  if (homingAtStartup) {
    homingRequested = true;
  }
}


void setup()
{
  Serial.begin(115200);
  Wire.begin(MULTIPLEXER_ADDRESS);
  Wire.onReceive(receiveEvent);

  pinMode(BUZZER, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_BUTTON, INPUT_PULLUP);
  pinMode(GREEN_BUTTON, INPUT_PULLUP);
  pinMode(ZERO_REF_1, INPUT_PULLUP);
  pinMode(ZERO_REF_2, INPUT_PULLUP);
  pinMode(ENCODER_1_CHA, INPUT_PULLUP);
  pinMode(ENCODER_1_CHB, INPUT_PULLUP);
  pinMode(ENCODER_2_CHA, INPUT_PULLUP);
  pinMode(ENCODER_2_CHB, INPUT_PULLUP);
  pinMode(CS2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_1_CHA), setChAEncMotor1State, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_CHB), setChBEncMotor1State, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_CHA), setChAEncMotor2State, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_CHB), setChBEncMotor2State, CHANGE);
  /*
  int myEraser = 7;
  int myPrescaler = 1; 
  TCCR2B &= ~myEraser;
  TCCR2B |= myPrescaler;
  */
  setupMotors();
  TCA9548A(DISPLAY_CHANNEL);
  if (soundOn) {
    playIntro();  
  }
  
  chBegin(chSetup);
  while(1) {}
}


// ***************************
// *** Interrupt functions ***
// ***************************


void setChAEncMotor1State()
{
  encoder_1.setChAState(bool(digitalRead(ENCODER_1_CHA)));
}

void setChBEncMotor1State()
{
  encoder_1.setChBState(bool(digitalRead(ENCODER_1_CHB)));
}

void setChAEncMotor2State()
{
  encoder_2.setChAState(bool(digitalRead(ENCODER_2_CHA)));
}

void setChBEncMotor2State()
{
  encoder_2.setChBState(bool(digitalRead(ENCODER_2_CHB)));
}


void receiveEvent(int bytes)
{
  I2Cmsg msg = Wire.read();
  switch(msg) {
    case I2C_HOME:
      homingRequested = true;
    break;
    case I2C_CONFIGURATION_1:
      if (actual_HL_mode != CONFIGURATION_1) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_1;
      }    
    break;
    case I2C_CONFIGURATION_2:
    if (actual_HL_mode != CONFIGURATION_2) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_2;
      }      
    break;
    case I2C_CONFIGURATION_3:
    if (actual_HL_mode != CONFIGURATION_3) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_3; 
      }        
    break;
    case I2C_CONFIGURATION_4:
    if (actual_HL_mode != CONFIGURATION_4) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_4;
      }        
    break;
    case I2C_CONFIGURATION_5:
    if (actual_HL_mode != CONFIGURATION_5) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_5;
      }      
    break;
    case I2C_CONFIGURATION_6:
    if (actual_HL_mode != CONFIGURATION_6) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_6; 
      }        
    break;
    default:
      actual_HL_state = IN_ERROR;
    break;
  }
}


// *********************
// *** Other methods ***
// *********************

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << bus);
  Wire.endTransmission();
}


void playIntro()
{
  int melody[] = {
    NOTE_B4, 16, NOTE_B5, 16, NOTE_FS5, 16, NOTE_DS5, 16,
    NOTE_B5, 32, NOTE_FS5, -16, NOTE_DS5, 8, NOTE_C5, 16,
    NOTE_C6, 16, NOTE_G6, 16, NOTE_E6, 16, NOTE_C6, 32, NOTE_G6, -16, NOTE_E6, 8,

    NOTE_B4, 16,  NOTE_B5, 16,  NOTE_FS5, 16,   NOTE_DS5, 16,  NOTE_B5, 32,
    NOTE_FS5, -16, NOTE_DS5, 8,  NOTE_DS5, 32, NOTE_E5, 32,  NOTE_F5, 32,
    NOTE_F5, 32,  NOTE_FS5, 32,  NOTE_G5, 32,  NOTE_G5, 32, NOTE_GS5, 32,  NOTE_A5, 16, NOTE_B5, 8
  };

  int tempo = 105;
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 4) / tempo;
  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } 
    else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }
    tone(BUZZER, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(BUZZER);
  }
}


// **************************
// *** Loop (leave empty) ***
// **************************

void loop() {}
