/*! @file CCMotor.h
    @author Alessio Prini (alessio.prini@itia.cnr.it)
    @version Revision 0.1
    @brief Header file actuator class and StepperCtrl class
*/

#ifndef _CC_MOTOR_
#define _CC_MOTOR_

#include <PID.h>
#include <RTEncoder.h>
#include <RTIncrementalEncoder.h>
#include "Actuator.h"
#include <SingleVNH5019Motor.h> 

class CCMotorCtrl: public Actuator {

public:

  SingleVNH5019Motor *motor;
  CCMotorCtrl(SingleVNH5019Motor *singleVNH5019Motor, double _deltaTime, RTEncoder & _encoder);

  void setCurrent(double);
  double getCurrent(); //!< Return filtered motor current
  void updateCurrent();
  double getMotorCommand();

  // machine state control
  void setControlState(Actuator::ControlState _ctrlState); //!< Set control state
  Actuator::ControlState getControlState(); //!< Get current control state
  bool setControlMode(Actuator::ControlMode _ctrlMode); //!< Set control mode
  Actuator::ControlMode getControlMode(); //!< Get current control mode
  void checkError(); //!< Check error
  void checkWarning(); //!< Check warning
  void resetError();

  // homing setup
  void setHomingVelocity(double _homeVel); //!< Set homing velocity
  double getHomingVelocity(); //!< Return homing velocity
  void setHomingAcceleration(double _homeAcc); //!< Set homing acceleration
  double getHomingAcceleration(); //!< Return homing acceleration

  // tracking error function
  void setMaxVelTrackingError(double _velErr);
  double getMaxVelTrackingError();
  void setMaxPosTrackingError(double _posErr);
  double getMaxPosTrackingError();
  void setMaxImpTrackingError(double _impErr);
  double getMaxImpTrackingError();

  // control function
  void update(); //!< Control cycle
  void setPIDVelocityParameters(double _kp, double _ki, double _kd);
  void setPIDPositionParameters(double _kp, double _ki, double _kd);
  void setPIDImpedanceParameters(double _kp, double _ki, double _kd);
  double* getPIDVelocityParameters(); //!< Return an array with the PID velocity control loop parameter
  double* getPIDPositionParameters(); //!< Return an array with the PID position control loop parameter
  double* getPIDImpedanceParameters(); //!< Return an array with the PID position control loop parameter
  void setPIDPositionOutputLimits(double _minLim, double _maxLim);
  void setPIDVelocityOutputLimits(double _minLim, double _maxLim);
  void setPIDImpedanceOutputLimits(double _minLim, double _maxLim);
  void setPIDPositionDirection(PID::Direction);
  void setPIDVelocityDirection(PID::Direction);
  void setPIDImpedanceDirection(PID::Direction);
  
  // limit methods
  void setMaxVelocity(double _maxVel);
  double getMaxVelocity();
  void setMaxAcceleration(double _maxAcc);
  double getMaxAcceleration();
  void setMaxCurrent(double _maxCurrent); //!< Set max 
  double getMaxCurrent();
  void setMaxImpedance(double _maxImpedance); //!< Set max 
  double getMaxImpedance();
  void setMaxCurrentDelay(double _delay); //!< Max number of seconds permitted for max current value
  double getMaxCurrentDelay();//!< Return  max number of seconds permitted for max current value
  void setTargetVelThreshold(double _tresh){targetVelThreshold = _tresh;};
  double getTargetVelThreshold(){return targetVelThreshold;};
  void setTargetHomePos(double);
  
private:
 
  // motor variable
  double current = 0;
  double currentDelay = 0;

  // utilities position and velocity loop function
  double positionLoop(double _targetPos);
  int velocityLoop(double _targetVel);
  int impedanceLoop(double _targetImp);

  //homing 
  double homingVelocity;
  double homingAcceleration;
  double end_stroke_pos;
  
  double kpp, kpi, kpd; //!< PID factor for position control loop
  double kvp, kvi, kvd; //!< PID factor for velocity control loop
  double kip, kii, kid; //!< PID factor for velocity control loop
  double positionPIDInput, positionPIDOutput;
  double velocityPIDInput, velocityPIDOutput;
  double impedancePIDInput, impedancePIDOutput;
  double positionPIDSetpoint, velocityPIDSetpoint, impedancePIDSetpoint; // uscita del PID
  int motorCommand; // variabile che viene poi effettivamente data al motore
  double maxPositionTrackingError, maxVelocityTrackingError, maxImpedanceTrackingError;
  PID velPid; //! For velocity control loop
  PID posPid; //! For position control loop
  PID impPid; //! For position control loop
  double PIDLimit;
  double targetVelThreshold;
  
  int countCycleUpdate = 0;
  int maxCountCycle = 1;
  
  double alphaCurrent = 0;
  double target_home_pos = 0;
  bool end_stroke_reached;
  
};

#endif
