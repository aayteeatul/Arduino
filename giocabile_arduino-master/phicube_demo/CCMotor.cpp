#include "CCMotor.h"


CCMotorCtrl::CCMotorCtrl(SingleVNH5019Motor *singleVNH5019Motor, double _deltaTime, RTEncoder & _encoder) : Actuator(_deltaTime, _encoder),
      velPid(&velocityPIDInput, &velocityPIDOutput, &velocityPIDSetpoint,1,1,1, PID::DIRECT),
      posPid(&positionPIDInput, &positionPIDOutput, &positionPIDSetpoint,1,1,1, PID::DIRECT),
      impPid(&impedancePIDInput, &impedancePIDOutput, &impedancePIDSetpoint,1,1,1, PID::DIRECT)
{
  // motor initialization
  motorCommand = 0;
  velPid.setSampleTime(_deltaTime);
  posPid.setSampleTime(_deltaTime);
  impPid.setSampleTime(_deltaTime);
  velPid.setMode(PID::AUTOMATIC);
  posPid.setMode(PID::AUTOMATIC);
  impPid.setMode(PID::AUTOMATIC);
  
  // setting current filter coefficient
  alphaCurrent = (double)2/(10*51);// si può pensare di metterla come parametro da passare

  // setting homing procedure
  setHomingVelocity(0.0);
  setHomingAcceleration(0.25);
  
  // setting limit
  setMaxAcceleration(1000);
  setMaxCurrent(2);
  setMaxCurrentDelay(3);
  setMaxVelocity(15);
  setMaxPosTrackingError(1);
  setMaxVelTrackingError(0.5);
  setMaxImpTrackingError(1);
  PIDLimit = 400;
  setPIDVelocityOutputLimits(-PIDLimit,PIDLimit);
  setPIDImpedanceOutputLimits(-PIDLimit,PIDLimit);
  
  motor = singleVNH5019Motor;
}


void CCMotorCtrl::setCurrent(double _val) {}

double CCMotorCtrl::getCurrent() {return current;}

void CCMotorCtrl::updateCurrent()
{
  current = alphaCurrent * double(double(motor->getCurrentMilliamps())/double(1000)) + (1-alphaCurrent)*current;
  if (current > maxCurrent && !isDemo()) {
    currentDelay += deltaTime;
  }
  else{
    currentDelay = 0;
  }
}

void CCMotorCtrl::setControlState(ControlState _ctrlState)
{
  ctrlState = _ctrlState;
  if (ctrlState == Actuator::ON) {
    setTargetVel(0);
    velocityLoop(vel);
    setTargetPos(getPos());
    positionLoop(pos);
    setTargetImp(getPos());
    impedanceLoop(imp);
    setWarningEnabled(true);
  }
  else if (ctrlState == Actuator::IDLE) {
    setTargetVel(0);
    velocityLoop(vel);
    setTargetPos(getPos());
    positionLoop(pos);
    setTargetImp(getPos());
    impedanceLoop(imp);
    setWarningEnabled(true);
  }
}

Actuator::ControlState CCMotorCtrl::getControlState() {return ctrlState;}

bool CCMotorCtrl::setControlMode(ControlMode _ctrlMode){
  if (ctrlState == IDLE) {
    ctrlMode = _ctrlMode;
    if (ctrlMode == HOMING) {
      encoder->unset();
      setHomeState(MOTOR_NOT_INITIALIZED);
      end_stroke_reached = false;
    }
    return true;
  }
  else {
    return false;
  }
}


Actuator::ControlMode CCMotorCtrl::getControlMode() {return ctrlMode;}

void CCMotorCtrl::checkError()
{
  if(ctrlMode != HOMING && ctrlState == ON) {
    if (endStrokeErrorFlag  && ctrlMode != HOMING) {
      if (pos > maxSoftEndStroke || pos < minSoftEndStroke) {
	      error = SOFT_END_STROKE_ERROR;
      }
      if ((pos > maxHardEndStroke || pos < minHardEndStroke || maxEndStrokeState || minEndStrokeState) && (!isDemo())) {
	      error = HARD_END_STROKE_ERROR;
      }
    }
    if (abs(pos - positionPIDSetpoint) > maxPositionTrackingError || abs(vel - velocityPIDSetpoint) > maxVelocityTrackingError || abs(imp - impedancePIDSetpoint) > maxImpedanceTrackingError) {
      error = TRACKING_ERROR;
    }
    if (abs(vel) > maxVel*1.5) { // lasciamo un 50 percento di margine rispetto alla massima consentita
      error = EXCESSIVE_VELOCITY_ERROR; // credo che le oscillazioni sopra questo limite siano dovute al controllo che non sta bene dietro al tutto
    }
    if ((currentDelay > maxCurrentDelay) && (!isDemo())) {
      error = EXCESSIVE_CURRENT_ERROR;
    }
  }
}


void CCMotorCtrl::checkWarning()
{
  warningFlag = false;
  if(warningEnabledFlag){
    if(ctrlMode != HOMING && ctrlState == ON){
      //Esiste gia un controllo che tiene conto degli endstroke
      //if (pos > maxHardEndStroke || pos < minHardEndStroke || maxEndStrokeState || minEndStrokeState){
        //warningFlag = true;
      //}
    }
  }
}

void CCMotorCtrl::setHomingVelocity(double _homeVel) {homingVelocity = (_homeVel);}

double CCMotorCtrl::getHomingVelocity() {return homingVelocity;}

void CCMotorCtrl::setHomingAcceleration(double _homeAcc) {homingAcceleration = (_homeAcc);}

double CCMotorCtrl::getHomingAcceleration() {return homingAcceleration;}

void CCMotorCtrl::setMaxPosTrackingError(double _posErr) {maxPositionTrackingError = _posErr;}

void CCMotorCtrl::setMaxVelTrackingError(double _velErr) {maxVelocityTrackingError = _velErr;}

void CCMotorCtrl::setMaxImpTrackingError(double _impErr) {maxImpedanceTrackingError = _impErr;}

double CCMotorCtrl::getMaxPosTrackingError() {return maxPositionTrackingError;}

double CCMotorCtrl::getMaxVelTrackingError() {return maxVelocityTrackingError;}

double CCMotorCtrl::getMaxImpTrackingError() {return maxImpedanceTrackingError;}

void CCMotorCtrl::setPIDPositionParameters(double _kp, double _ki, double _kd)
{
  kpp = _kp;
  kpi = _ki;
  kpd = _kd;
  posPid.setTunings(kpp,kpi,kpd);
}

void CCMotorCtrl::setPIDVelocityParameters(double _kp, double _ki, double _kd)
{
  kvp = _kp;
  kvi = _ki;
  kvd = _kd;
  velPid.setTunings(kvp,kvi,kvd);
}

void CCMotorCtrl::setPIDImpedanceParameters(double _kp, double _ki, double _kd)
{
  kip = _kp;
  kii = _ki;
  kid = _kd;
  impPid.setTunings(kip,kii,kid);
}

double* CCMotorCtrl::getPIDPositionParameters()
{
  double param[] = {kpp,kpi,kpd};
  return param;
}


double* CCMotorCtrl::getPIDVelocityParameters()
{
  double param[] = {kvp,kvi,kvd};
  return param;
}

double* CCMotorCtrl::getPIDImpedanceParameters()
{
  double param[] = {kip,kii,kid};
  return param;
}

void CCMotorCtrl::setPIDPositionOutputLimits(double _minLim, double _maxLim) {posPid.setOutputLimits(_minLim,_maxLim);}

void CCMotorCtrl::setPIDVelocityOutputLimits(double _minLim, double _maxLim) {velPid.setOutputLimits(_minLim,_maxLim);}

void CCMotorCtrl::setPIDImpedanceOutputLimits(double _minLim, double _maxLim) {impPid.setOutputLimits(_minLim,_maxLim);}

void CCMotorCtrl::setMaxVelocity(double _maxVelocity)
{
  maxVel = _maxVelocity;
  setPIDPositionOutputLimits(-_maxVelocity,_maxVelocity);
}

void CCMotorCtrl::setMaxImpedance(double _maxImpedance)
{
  maxImp = _maxImpedance;
  setPIDPositionOutputLimits(-_maxImpedance,_maxImpedance);
}

double CCMotorCtrl::getMaxVelocity() {return maxVel;}

double CCMotorCtrl::getMaxImpedance() {return maxImp;}

void CCMotorCtrl::setMaxAcceleration(double _maxAcceleration) {maxAcc = _maxAcceleration;}

double CCMotorCtrl::getMaxAcceleration() {return maxAcc;}

void CCMotorCtrl::setMaxCurrent(double _maxCurrent) {maxCurrent = _maxCurrent;}

double CCMotorCtrl::getMaxCurrent() {return maxCurrent;}

void CCMotorCtrl::setMaxCurrentDelay(double _maxDelay)
{
  if (_maxDelay < 0.001) { // se fosse 0 sarebbe un bel casino perche andrebbe sempre in errore
    _maxDelay = 0.001;
  }
  maxCurrentDelay = _maxDelay;
}

double CCMotorCtrl::getMaxCurrentDelay() {return maxCurrentDelay;}


void CCMotorCtrl::update()
{
  // update data
  updateCurrent();
  //checkWarning();
  encoder->update();
  checkError();
  
  if (inError()) {
    setControlState(IDLE);
  }
  
  if (!isDemo()) {
    setPos(encoder->getPos());
    setVel(encoder->getVel());
    setImp(encoder->getPos());
    setAcc(0);// potremmo mettere anche una stima acc -- testata con simulink fa cagare al cazzostiffnessMotor.
  }
  else{
    double calcPos = pos + targetVel*deltaTime;
    setPos(calcPos);
    setVel(targetVel);
    setImp(targetImp);
    setAcc(0);// potremmo mettere anche una stima acc  -- testata con simulink fa cagare al cazzo
  }


  switch(ctrlState) {
    
    case ON:
    
      switch(ctrlMode) {
        
        case POSITION:
          targetVel = positionLoop(targetPos);
          motorCommand = velocityLoop(targetVel);
        break;
        
        case VELOCITY:
          motorCommand = velocityLoop(targetVel);
          positionLoop(pos); 
        break;

        case IMPEDANCE:
          motorCommand = impedanceLoop(targetImp);
        break;
    
        case HOMING:
        
          if (homeState != MOTOR_INITIALIZED) {
	          if(!isDemo()) {
	            
	            switch(homingType) {
	    
	              case MIN_ENDSTROKE:
                  if(!end_stroke_reached) {
                    if (!getMinEndStrokeState()) {
                      homeState = MOTOR_NOT_INITIALIZED;
                      if (!encoder->isSet()) {
                        targetVel = -homingVelocity;
                        motorCommand = velocityLoop(targetVel);
                      }
                    }
                    else {
                      targetVel = 0;
                      motorCommand = 0;
                      //homeState = MOTOR_INITIALIZED;
                      end_stroke_reached = true;
                      end_stroke_pos = encoder->getPos();
                      //encoder->reset(0);
                      //encoder->setOffset(homeOffset);
                    }
                  }
                  else {
                    //if(pos < target_home_pos) {
                    if(pos < (end_stroke_pos + target_home_pos - homeOffset)) {
                      if(targetVel < homingVelocity) {
                        targetVel = targetVel + homingAcceleration*deltaTime;
                      }
                      else {
                        targetVel = homingVelocity;
                      }
                      motorCommand = velocityLoop(targetVel);
                    }
                    else {
                      targetVel = 0;
                      motorCommand = 0;
                      encoder->reset(target_home_pos);
                      //encoder->setOffset(0);
                      homeState = MOTOR_INITIALIZED;
                    }
                  }
                break; //MIN_ENDSTROKE
                
	              case MAX_ENDSTROKE:
	                if (!getMaxEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
		                if (!encoder->isSet()) {
		                  targetVel = homingVelocity;
		                  motorCommand = velocityLoop(targetVel);
		                }
                  }
                  else {
		                targetVel = 0;
	                  motorCommand = 0;
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset(0);
		                encoder->setOffset(homeOffset);
	                }
                break; //MAX_ENDSTROKE
                  
	              case MIN_ENDSTROKE_AND_HOLD:
	                if (!getMinEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
		                if (!encoder->isSet()) {
		                  targetVel = -homingVelocity;
		                  motorCommand = velocityLoop(targetVel);
		                }
	                }
	                else {
		                targetVel = 0;
		                motorCommand = 0;
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset(0);
                		encoder->setOffset(homeOffset);
                		setControlState(IDLE);
                		setControlMode(POSITION);
                		setControlState(ON);
                		setTargetPos(homeOffset);
                		setWarningEnabled(false);
	                }
	              break; //MIN_ENDSTROKE_AND_HOLD
               
	              case MAX_ENDSTROKE_AND_HOLD:
	                if (!getMaxEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
	                  if (!encoder->isSet()) {
		                  targetVel = homingVelocity;
		                  motorCommand = velocityLoop(targetVel);
		                }
	                }
	                else {
                		targetVel = 0;
                		motorCommand = 0;
                    encoder->reset(0);
                		homeState = MOTOR_INITIALIZED;
                		//encoder->setOffset(homeOffset);
                		//setControlState(IDLE);
                		//setControlMode(POSITION);
                		//setControlState(ON);
                		//setTargetPos(homeOffset);
                		setWarningEnabled(false);
	                }
                break;
	    
	              case ACTUAL:
                  targetVel = 0;
          	      motorCommand = 0;
          	      homeState = MOTOR_INITIALIZED;
          	      encoder->reset(0);
          	      encoder->setOffset(0);
          	      setControlState(IDLE);
          	      setTargetPos(getPos());
	              break;
                
	            } //homingType switch
	          } //!isDemo
	          else {
	            setPos(0);
	            homeState = MOTOR_INITIALIZED;
	            setControlState(IDLE);
	          }
          } //!MOTOR INITIALIZED
          
        break; //HOMING
      
        case NO_MODE:
          targetVel = vel;
          motorCommand = 0;
        break;
      
      } //ctrlMode switch

      if (!isDemo()) {
        if(!inWarning()) {
          motor->setSpeed(motorCommand);
        }
        else {
          motor->setSpeed(0);
        }
      }
      else {
        motor->setSpeed(0);
      }
    
    break; //ON
    
    case IDLE:
      motorCommand = 0; // switch off motor
      targetVel = 0;
      motor->setSpeed(int(motorCommand));
      positionLoop(pos);
      impedanceLoop(imp);
      velocityLoop(0);
    break;
    
    default:
      motorCommand = 0; // switch off motor
      targetVel = 0;
      motor->setSpeed(int(motorCommand));
      positionLoop(pos);
      impedanceLoop(imp);
      velocityLoop(0);
    break;
  
  } //ctrlState swtich
  
  
  if (inError()) {
    //Serial.println(motor.getErrorCode());
  }
  
  updatedFlag = true;
}


double CCMotorCtrl::positionLoop(double _targetPos)
{
  positionPIDSetpoint = _targetPos;
  positionPIDInput = getPos();
  posPid.compute(); // da qui esce una velocità
  double _targetVel = positionPIDOutput;
  return _targetVel;
}

int CCMotorCtrl::velocityLoop(double _targetVel)
{
  /*
  if(ctrlMode != HOMING){ 
    if (pos > maxSoftEndStroke && _targetVel > 0) {_targetVel = 0;}
    if (pos < minSoftEndStroke && _targetVel < 0) {_targetVel = 0;}
    if (maxEndStrokeState && _targetVel > 0) {_targetVel = 0;};
    if (minEndStrokeState && _targetVel < 0) {_targetVel = 0;};
  }
  */
  velocityPIDSetpoint = _targetVel;
  velocityPIDInput = getVel();
  velPid.compute();
  int _motorCommand = int(velocityPIDOutput);
  return _motorCommand;
}

int CCMotorCtrl::impedanceLoop(double _targetImp)
{
  impedancePIDSetpoint = _targetImp;
  impedancePIDInput = getImp();
  impPid.compute(); // da qui esce una velocità
  int _motorCommand = impedancePIDOutput;
  return _motorCommand;
}

double CCMotorCtrl::getMotorCommand() {return motorCommand;}

void CCMotorCtrl::resetError()
{
  setControlState(IDLE);
  error = NO_ERROR;
  //if(!isDemo()) {
    //motor.resetError();
    //motor.reset();
  //}
  init();
}

void CCMotorCtrl::setPIDPositionDirection(PID::Direction _dir) {posPid.setControllerDirection(_dir);}

void CCMotorCtrl::setPIDVelocityDirection(PID::Direction _dir) {velPid.setControllerDirection(_dir);}

void CCMotorCtrl::setPIDImpedanceDirection(PID::Direction _dir) {impPid.setControllerDirection(_dir);}

void CCMotorCtrl::setTargetHomePos(double _pos) {target_home_pos = _pos;}
