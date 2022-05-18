/*! 
    @file     PhiCube.ino
    @author   Matteo Lavit Nicora (matteo.lavit@stiima.cnr.it)
    @version  Revision 0.1
    @brief    Main application file to control the PhiCube device.
*/

// **************************
// *** Required libraries ***
// **************************
#include <SoftwareSerial.h>
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

union ByteIntUnion{
  uint8_t byteformat[2];
  int intformat;
};

union ByteUnLongUnion{
  uint8_t byteformat[4];
  unsigned long unLongformat;
};

union ByteFloatUnion{
  uint8_t byteformat[4];
  float floatformat;
};


// **************************
// *** Thread frequencies ***
// **************************

#define ROBOT_CTRL_PERIOD_MS 10
#define STATE_MACHINE_PERIOD_MS 100


// **********************
// *** Init variables ***
// **********************

#define pi 3.14159265
#define PIN_TX 16
#define PIN_RX 17

LLState actual_LL_state = IDLE;
HLState actual_HL_state = NOT_HOMED;
LLMode actual_LL_mode = NO_MODE;
HLMode actual_HL_mode = NO_CONTROL;
Error actual_error = NO_ERROR;

bool homingAtStartup = false;
bool soundOn = false;
bool homingRequested = false;
bool initializationRequested = false;
bool setupDone = false;
bool powerLost = false;
double homingVelocity = 1.5;
double currentTarget1 = 0;
double currentTarget2 = 0;
double equilibriumPos1, equilibriumPos2;
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
SoftwareSerial bt(PIN_RX, PIN_TX);

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

    updateStateButtons();
    updateLLStateMachine();
    updateHLStateMachine();

    btPublish();
    /*
    cmdMessenger.sendCmdStart(Data);
    cmdMessenger.sendCmdArg(millis());
    cmdMessenger.sendCmdArg(actual_HL_mode);
    cmdMessenger.sendCmdArg(motor_1_ctrl.getPos(), 5);
    cmdMessenger.sendCmdArg(motor_2_ctrl.getPos(), 5);
    cmdMessenger.sendCmdArg(motor_1_ctrl.getVel(), 5);
    cmdMessenger.sendCmdArg(motor_2_ctrl.getVel(), 5);
    cmdMessenger.sendCmdEnd();
    */
  }
}


// *****************************
// *** Robot control methods ***
// *****************************

void(* resetBoard)(void) = 0;

void updateStateButtons()
{
  if (digitalRead(POWER_CHECK_1) == LOW || digitalRead(POWER_CHECK_2) == LOW) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      robot.turnOff();
      actual_HL_state = NOT_HOMED;
      actual_HL_mode = NO_CONTROL;
      robot.setControlMode(Actuator::NO_MODE);
      homingRequested = false;
      powerLost = true;  
  }
  else {
    if (powerLost) {
      resetBoard();
    }
  }
  
  if (digitalRead(RED_BUTTON) == LOW ) {
    robot.turnOff();
    robot.setControlMode(Actuator::NO_MODE);
    actual_HL_state = IN_ERROR;
    actual_HL_mode = NO_CONTROL;
  }
  if (digitalRead(GREEN_BUTTON) == LOW) {
    if (motor_1_ctrl.isInitialized() && motor_2_ctrl.isInitialized() && homingRequested && actual_HL_state == IN_ERROR) {
      TCA9548A(DISPLAY_CHANNEL);
      Wire.beginTransmission(DISPLAY_ADDRESS);
      Wire.write(I2C_RESET);
      int error = Wire.endTransmission();
      if (error == 0) {
        robot.resetError();
        robot.turnOff();
        robot.setControlMode(Actuator::NO_MODE);
        actual_HL_state = HOMED;
      }
    }  
  }
}


void updateLLStateMachine()
{
  //low-level state
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

  //error
  if(robot.inError()) {
    actual_LL_state = ERROR;
    actual_HL_state = IN_ERROR;
    switch (robot.getErrorState()) {
      case(Robot::NO_ERROR):
        actual_error = NO_ERROR;
      break;
      case(Robot::EMERGENCY_ERROR):
        actual_error = EMERGENCY;
      break;
      case(Robot::MOTOR_ERROR):
        if(motor_1_ctrl.inError()) {actual_error = MOTOR1_ERROR;}
        else {actual_error = MOTOR2_ERROR;}
      break;
      case(Robot::COHERENCE_ERROR):
        actual_error = INCOHERENCE_ERROR;
      break;
      default:
        actual_error = INCOHERENCE_ERROR;
      break;
    }
  }
  else {actual_error = NO_ERROR;}
}


void updateHLStateMachine()
{
  //high-level state
  switch (actual_HL_state) {
    
    case NOT_HOMED:
      if (homingRequested) {
        actual_HL_state = CURRENTLY_HOMING;
      }
    break;
    
    case CURRENTLY_HOMING:
      if (actual_LL_state == ON && actual_LL_mode == NO_MODE) {
        robot.turnOff();
        robot.setControlMode(Actuator::HOMING);
        robot.turnOn(); 
      }
      else {
        doHoming();
      }
    break; 
    
    case HOMED:
      noTone(BUZZER);
      digitalWrite(RED_LED, LOW);
      if (signalCount < 4) {
        digitalWrite(GREEN_LED, HIGH);
        signalCount++;  
      }
      else if (signalCount >= 4 && signalCount < 8){
        digitalWrite(GREEN_LED, LOW);
        signalCount++;
      }
      else {
        signalCount = 0;
      }
    break;
    
    case INITIALIZING:
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
            // ****************** PLATFORM ****************** 
            case CONFIGURATION_1: // *left stiffness right damping*
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            case CONFIGURATION_2: // *left damping right stiffness*
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;   
            case CONFIGURATION_3: // *both stiffness
              doInitialization(10,10,10,10,0.0,0.21925699); 
            break;
            // ****************** GIUNGLA ******************
            case CONFIGURATION_4: // *car wheel*
              doInitialization(10,10,10,10,0.0,1.65588116);
            break;
            case CONFIGURATION_5: // *bike wheel*
              doInitialization(10,10,10,10,0.0,3.14813780);
            break;
            case CONFIGURATION_6: // *left stiffness right damping (Mn Davanti)* 
              doInitialization(10,10,10,10,0.0,0.0);
            break; 
            case CONFIGURATION_7: // *left stiffness right damping (Mn Sopra)*
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            case CONFIGURATION_8: // *both stiffness*
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;
            // ****************** BOARDING ******************
            case CONFIGURATION_9: // *both stiffness*
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;   
            case CONFIGURATION_10: // *both damping*
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            // ****************** ESCAVATORE ******************
            case CONFIGURATION_11: // *both damping*
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            // ****************** AEROPLANO ******************
            case CONFIGURATION_12: // *car wheel*
              doInitialization(10,10,10,10,0.0,1.65588116);
            break;
            case CONFIGURATION_13: // *bike wheel*
              doInitialization(10,10,10,10,0.0,3.14813780);
            break;
            case CONFIGURATION_14: // *left stiffness right damping (Mn Davanti)*
              doInitialization(10,10,10,10,0.0,0.0);
            break;
            case CONFIGURATION_15: // *left stiffness right damping (Mn Sopra)*
              doInitialization(10,10,10,10,0.0,0.0);
            break; 
            case CONFIGURATION_16: // *both stiffness*
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;  
            // ****************** PESCA ******************
            case CONFIGURATION_17: // not ready yet
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;
            // ****************** SCALATA ******************
            case CONFIGURATION_18: // not ready yet
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;
            // ****************** Diamante ******************
            case CONFIGURATION_19: // *both stiffness*
              doInitialization(10,10,10,10,0.0,0.21925699);
            break;
                    
          }
        }
      }  
    break;
    
    case INITIALIZED:
      switch (actual_HL_mode) {
        // ****************** PLATFORM ******************
        case CONFIGURATION_1: // *left stiffness right damping
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_2: // *left damping right stiffness
          if (!setupDone) {
            motor_1_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true;  
          }
        break;
        case CONFIGURATION_3: // *both stiffness
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2);
            setupDone = true; 
          }
        break;
        // ****************** GIUNGLA ******************
        case CONFIGURATION_4: // *car wheel
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_5: // *bike wheel
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_6 : // *left stiffness right damping (Mn Davanti)
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_7: // *left damping right stiffness (Mn Sopra)
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_8: // *both stiffness
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        // ****************** BOARDING ******************
        case CONFIGURATION_9: // *both stiffness
          if (!setupDone) {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true;  
          }
        break;
        case CONFIGURATION_10: // *both damping
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(0,0,30);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,30);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2);
            setupDone = true; 
          }
        break;
        // ****************** ESCAVATORE ******************
        case CONFIGURATION_11: // *both damping
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(0,0,10);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,10);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        // ****************** AEROPLANO ******************
        case CONFIGURATION_12: // *car wheel
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_13 : // *bike wheel
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_14: // *left stiffness right damping (Mn Davanti)
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        case CONFIGURATION_15: // *left damping right stiffness (Mn Sopra)
          if (!setupDone) {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(0,0,2);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true;  
          }
        break;
        case CONFIGURATION_16: // *left stiffness
          if (!setupDone) {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true;  
          }
        break;
        // ****************** PESCA ******************
        case CONFIGURATION_17: // not defined yet
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2);
            setupDone = true; 
          }
        break;
        // ****************** SCALATA ******************
        case CONFIGURATION_18: // not defined yet
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break;
        // ****************** DIAMANTE ******************
        case CONFIGURATION_19: // *both damping
          if (!setupDone)  {
            motor_1_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_2_ctrl.setPIDImpedanceParameters(200,0,0);
            motor_1_ctrl.setTargetImp(equilibriumPos1);
            motor_2_ctrl.setTargetImp(equilibriumPos2); 
            setupDone = true; 
          }
        break; 
      }
      digitalWrite(GREEN_LED, HIGH);
    break;
    
    case IN_ERROR:
      TCA9548A(DISPLAY_CHANNEL);
      Wire.beginTransmission(DISPLAY_ADDRESS);
      Wire.write(I2C_ERROR);
      Wire.endTransmission();
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      if (soundOn) {
        if (signalCount < 4) {
          tone(BUZZER,1000,2000);
          signalCount++;
        }
        else if (signalCount >= 4 && signalCount < 14) {
          noTone(BUZZER);
          signalCount++;
        }
        else {
          signalCount = 0;
        }  
      }
    break;
  }
}


void doHoming()
{
  if (!motor_1_ctrl.isInitialized()) {
    motor_1_ctrl.setHomingVelocity(homingVelocity);
    motor_1_ctrl.setMaxEndStrokeState(bool(digitalRead(ZERO_REF_1)));
  }
  if (!motor_2_ctrl.isInitialized()) {
    motor_2_ctrl.setHomingVelocity(homingVelocity);
    motor_2_ctrl.setMaxEndStrokeState(bool(digitalRead(ZERO_REF_2)));
  }
  if (motor_1_ctrl.isInitialized() && motor_2_ctrl.isInitialized()) {
    TCA9548A(DISPLAY_CHANNEL);
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.write(I2C_SUCCESS);
    int error = Wire.endTransmission();
    if (error != 0) {
      actual_HL_state = IN_ERROR;
    }
    else {
      actual_HL_state = HOMED;
    }
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
    TCA9548A(DISPLAY_CHANNEL);
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.write(I2C_SUCCESS);
    int error = Wire.endTransmission();
    if (error != 0) {
      actual_HL_state = IN_ERROR;
    }
    else {
      actual_HL_state = INITIALIZED;
      setupDone = false;
    }                        
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

void btPublish() {

  ByteIntUnion intVal;
  intVal.intformat = 258;
  
  ByteFloatUnion floatVal;
  floatVal.floatformat = 258;

  uint8_t devId = 0x02;
  uint8_t dataLength = 0x15;
  uint8_t configDev = actual_HL_mode;
  ByteUnLongUnion timeStampUnion;
  timeStampUnion.unLongformat = millis();
  
  ByteFloatUnion motor_1_Pos_Union;
  motor_1_Pos_Union.floatformat = motor_1_ctrl.getPos();
  
  ByteFloatUnion motor_2_Pos_Union;
  motor_2_Pos_Union.floatformat = motor_2_ctrl.getPos();
  
  ByteFloatUnion motor_1_Vel_Union;
  motor_1_Vel_Union.floatformat = motor_1_ctrl.getVel();
  
  ByteFloatUnion motor_2_Vel_Union;
  motor_2_Vel_Union.floatformat = motor_2_ctrl.getVel();

  uint8_t checkData = 0xFF - (dataLength +
                          configDev +
                          timeStampUnion.byteformat[0]+timeStampUnion.byteformat[1] + timeStampUnion.byteformat[2] + timeStampUnion.byteformat[3] +
                          motor_1_Pos_Union.byteformat[0] + motor_1_Pos_Union.byteformat[1] + motor_1_Pos_Union.byteformat[2]+motor_1_Pos_Union.byteformat[3]+
                          motor_2_Pos_Union.byteformat[0]+motor_2_Pos_Union.byteformat[1]+motor_2_Pos_Union.byteformat[2]+motor_2_Pos_Union.byteformat[3]+
                          motor_1_Vel_Union.byteformat[0]+motor_1_Vel_Union.byteformat[1]+motor_1_Vel_Union.byteformat[2]+motor_1_Vel_Union.byteformat[3]+
                          motor_2_Vel_Union.byteformat[0]+motor_2_Vel_Union.byteformat[1]+motor_2_Vel_Union.byteformat[2]+motor_2_Vel_Union.byteformat[3]);
  uint8_t dataArray[24] = {devId,
                          dataLength,
                          configDev,
                          timeStampUnion.byteformat[0], timeStampUnion.byteformat[1], timeStampUnion.byteformat[2], timeStampUnion.byteformat[3],
                          motor_1_Pos_Union.byteformat[0],motor_1_Pos_Union.byteformat[1],motor_1_Pos_Union.byteformat[2],motor_1_Pos_Union.byteformat[3],
                          motor_2_Pos_Union.byteformat[0],motor_2_Pos_Union.byteformat[1],motor_2_Pos_Union.byteformat[2],motor_2_Pos_Union.byteformat[3],
                          motor_1_Vel_Union.byteformat[0],motor_1_Vel_Union.byteformat[1],motor_1_Vel_Union.byteformat[2],motor_1_Vel_Union.byteformat[3],
                          motor_2_Vel_Union.byteformat[0],motor_2_Vel_Union.byteformat[1],motor_2_Vel_Union.byteformat[2],motor_2_Vel_Union.byteformat[3],
                          checkData};

  bt.write(dataArray, sizeof(dataArray));
  /*
  bt.print(millis());
  bt.print(actual_HL_mode);
  bt.print(motor_1_ctrl.getPos());
  bt.print(motor_2_ctrl.getPos());
  bt.print(motor_1_ctrl.getVel());
  bt.println(motor_2_ctrl.getVel());
  */
}
    
void setup()
{
  Serial.begin(115200);
  bt.begin(9600);
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
    case I2C_CONFIGURATION_7:
    if (actual_HL_mode != CONFIGURATION_7) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_7; 
      }        
    break;
    case I2C_CONFIGURATION_8:
    if (actual_HL_mode != CONFIGURATION_8) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_8; 
      }        
    break;
    case I2C_CONFIGURATION_9:
    if (actual_HL_mode != CONFIGURATION_9) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_9; 
      }        
    break;
    case I2C_CONFIGURATION_10:
    if (actual_HL_mode != CONFIGURATION_10) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_10; 
      }        
    break;
    case I2C_CONFIGURATION_11:
    if (actual_HL_mode != CONFIGURATION_11) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_11; 
      }        
    break;
    case I2C_CONFIGURATION_12:
    if (actual_HL_mode != CONFIGURATION_12) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_12; 
      }        
    break;
    case I2C_CONFIGURATION_13:
    if (actual_HL_mode != CONFIGURATION_13) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_13; 
      }        
    break;
    case I2C_CONFIGURATION_14:
    if (actual_HL_mode != CONFIGURATION_14) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_14; 
      }        
    break;
    case I2C_CONFIGURATION_15:
    if (actual_HL_mode != CONFIGURATION_15) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_15; 
      }        
    break;
    case I2C_CONFIGURATION_16:
    if (actual_HL_mode != CONFIGURATION_16) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_16; 
      }        
    break;
    case I2C_CONFIGURATION_17:
    if (actual_HL_mode != CONFIGURATION_17) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_17; 
      }        
    break;
    case I2C_CONFIGURATION_18:
    if (actual_HL_mode != CONFIGURATION_18) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_18; 
      }        
    break;
    case I2C_CONFIGURATION_19:
    if (actual_HL_mode != CONFIGURATION_19) {
        initializationRequested = true;
        setupDone = false;
        actual_HL_state = INITIALIZING;
        actual_HL_mode = CONFIGURATION_19; 
      }        
    break;
    case I2C_BACK_MODE:
      //robot.setControlMode(Actuator::NO_MODE);
      //actual_HL_state = HOMED;
      robot.turnOff();
      robot.setControlMode(Actuator::NO_MODE);
      robot.turnOn();
      actual_HL_state = HOMED;       
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
