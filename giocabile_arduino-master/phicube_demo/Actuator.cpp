#include "Actuator.h"
#include "UnitConversion.h"


Actuator::Actuator(double _deltaTime, RTEncoder & _encoder):warningFlag(false),
      maxSoftEndStroke(1000000),minSoftEndStroke(-1000000),maxHardEndStroke(1000000),minHardEndStroke(-1000000){

  // encoder inititalization
  encoder = &(_encoder);  
  deltaTime = _deltaTime;
  encoder->setDeltaTime(_deltaTime);
  resetEncoder(0);
  
  // state machine initialization
  setDemo(false);
  resetError();
  setControlState(IDLE);
  setControlMode(NO_MODE);
  
  // homing part
  setHomingType(MIN_ENDSTROKE);
  setHomeOffset(0);
  setEndStrokeErrorEnabled(true);
  homeState = MOTOR_NOT_INITIALIZED;
}



double Actuator::getPos(){return pos;}

void Actuator::setPos(double _pos){pos = _pos;}

double Actuator::getVel(){return vel;}

void Actuator::setVel(double _vel){vel = _vel;}

double Actuator::getImp(){return imp;}

void Actuator::setImp(double _imp){imp = _imp;}

double Actuator::getAcc(){return acc;}

void Actuator::setAcc(double _acc){acc = _acc;}

void Actuator::resetEncoder(double _pos = 0){
  setPos(_pos);
  encoder->reset(_pos);
}

void Actuator::setDemo(bool _demo){ 
  demo = _demo;
}

bool Actuator::isDemo(){return demo;}

Actuator::MotorError Actuator::getErrorState(){return error;}


void Actuator::resetError(){
  setControlState(IDLE);
  error = NO_ERROR;
}

bool Actuator::inError(){
  bool resp;
  if(error == NO_ERROR){
    resp = false;
  }else{
    resp = true;
  }
  return resp;
}

bool Actuator::inWarning(){
  return warningFlag;
}


void Actuator::setHomingType(HomingType _homingType){
  if (ctrlState == IDLE){homingType = _homingType;}
}

Actuator::HomingType Actuator::getHomingType(){return homingType;}

void Actuator::setHomeOffset(double _homeOffset){homeOffset = _homeOffset;}

double Actuator::getHomeOffset(){return homeOffset;}

void Actuator::setDeltaTime(double _delta){deltaTime = _delta;}

double Actuator::getDeltaTime(){return deltaTime;}


void Actuator::setHomeState(Actuator::HomeState _homeState){
  homeState = _homeState;
}

Actuator::HomeState Actuator::getHomeState(){
  return homeState;
}  

bool Actuator::isInitialized(){
  return (getHomeState() == MOTOR_INITIALIZED);
}


bool Actuator::targetReached(){
  if(abs(targetPos - getPos()) < tolerance){
    return true;
  }
  else{
    return false;
  }
}

void Actuator::setTargetReachedToll(double _toll){
  tolerance = _toll;
}

void Actuator::setEndStrokeErrorEnabled(bool _enabled){
  endStrokeErrorFlag =_enabled;
}


bool Actuator::getEndStrokeErrorEnabled(){
  return endStrokeErrorFlag;
}

void Actuator::setWarningEnabled(bool _enabled){
  warningEnabledFlag =_enabled;
}


bool Actuator::getWarningEnabled(){
  return warningEnabledFlag;
}

void Actuator::setTargetPos(double _target){targetPos = _target;}

double Actuator::getTargetPos(){return targetPos;}

void Actuator::setTargetVel(double _target){
  if(targetVel > maxVel){
    targetVel = maxVel;
  }else{
    targetVel = _target;
  }
}

double Actuator::getTargetVel(){return targetVel;}

void Actuator::setTargetImp(double _target) {targetImp = _target;}

double Actuator::getTargetImp(){return targetImp;}

// virtual function
void Actuator::setControlState(Actuator::ControlState _ctrlState){//virtual
  ctrlState = _ctrlState;

  if (ctrlState == ON){
    setTargetVel(0);
    setTargetPos(getPos());
    setWarningEnabled(true);
  }
}

Actuator::ControlState Actuator::getControlState(){return ctrlState;}//virtual


bool Actuator::setControlMode(ControlMode _ctrlMode){//virtual
  if (ctrlState == IDLE){
    ctrlMode = _ctrlMode;
    if (ctrlMode == HOMING){
      encoder->unset();
    }
    return true;
  }else{
    return false;
  }
}

Actuator::ControlMode Actuator::getControlMode(){//virtual
  return ctrlMode;
}

void Actuator::checkError(){//virtual
    if (endStrokeErrorFlag && ctrlMode != HOMING){
      if (pos > maxSoftEndStroke || pos < minSoftEndStroke){
	error = SOFT_END_STROKE_ERROR;
      }
      if ((pos > maxHardEndStroke || pos < minHardEndStroke || maxEndStrokeState || minEndStrokeState) && (!isDemo())){
	error = HARD_END_STROKE_ERROR;
      }
    }
}


void Actuator::checkWarning(){//virtual
  warningFlag = false;
  if(warningEnabledFlag){
    if(ctrlMode != HOMING && ctrlState == ON){
      if (pos > maxHardEndStroke || pos < minHardEndStroke || maxEndStrokeState || minEndStrokeState){
	warningFlag = true;
      }
    }
  }
}


void Actuator::update()
{  
  checkError();
  checkWarning();
  encoder->update();
  
  if (inError()){
    setControlState(IDLE);
  }
  
  if (!isDemo()) {
    setPos(encoder->getPos());
    setVel(encoder->getVel());
    setAcc(0);
  }
  else {
    double calcPos = pos + targetVel*deltaTime;
    setPos(calcPos);
    setVel(targetVel);
    setImp(targetImp);
    setAcc(0);// potremmo mettere anche una stima acc  -- testata con simulink fa cagare al cazzo
  }

  switch(ctrlState) {
    
    case ON:
    
      switch(ctrlMode) {
        
        case HOMING:
        
          if (homeState != MOTOR_INITIALIZED) {
	          if(!isDemo()) {
            
	            switch(homingType) {

	              case MIN_ENDSTROKE:
	                if (!getMinEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
	                }
	                else {
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset();
		                encoder->setOffset(-pos + homeOffset);
	                }
	              break;
                
	              case MAX_ENDSTROKE:
	                if (!getMaxEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
	                } 
	                else {
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset();
		                encoder->setOffset(-pos + homeOffset);
	                }
	              break;
                 
	              case MIN_ENDSTROKE_AND_HOLD:
	                if (!getMinEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
	                }
	                else {
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset();
		                encoder->setOffset(-pos + homeOffset);
		                setControlState(IDLE);
		                setControlMode(POSITION);
		                setControlState(ON);
                    //setTargetPos(homeOffset);
		                setWarningEnabled(false);
	                }
	              break;
                
	              case MAX_ENDSTROKE_AND_HOLD:
	                if (!getMaxEndStrokeState()) {
		                homeState = MOTOR_NOT_INITIALIZED;
	                }
	                else {
		                homeState = MOTOR_INITIALIZED;
		                encoder->reset();
		                encoder->setOffset(-pos + homeOffset);
		                //setControlState(IDLE);
		                //setControlMode(POSITION);
		                //setControlState(ON);
   	    		        //setTargetPos(homeOffset);
		                setWarningEnabled(false);
	                }
	              break;
                
	              case ACTUAL:
	                homeState = MOTOR_INITIALIZED;
	                setControlState(IDLE);
  	              //encoder->reset();
	              break;
              }
	        
	          } //!isDemo()
            else {
	            setPos(0);
	            homeState = MOTOR_INITIALIZED;
	            setControlState(IDLE);
	          }
           
          } //!MOTOR_INITIALIZED
          
        break; //HOMING
        
        case NO_MODE:
        break;
    
      } //ctrlMode switch

    break;//ON
    
    case IDLE:
    break;
   
    default:
    break;
    
  } //ctrlState switch
  
  updatedFlag = true;
};


void Actuator::setMaxEndStrokeState(bool _state){
  if(!demo){
    maxEndStrokeState = _state;
  }else{
    maxEndStrokeState = false;
  }
}

void Actuator::setMinEndStrokeState(bool _state){
  if(!demo){
    minEndStrokeState = _state;
  }else{  
    minEndStrokeState = false;
  }
}

bool Actuator::getMaxEndStrokeState(){return maxEndStrokeState;}

bool Actuator::getMinEndStrokeState(){return minEndStrokeState;}

bool Actuator::updated(){
  return updatedFlag;
}

void Actuator::release(){
  updatedFlag = false;
}
