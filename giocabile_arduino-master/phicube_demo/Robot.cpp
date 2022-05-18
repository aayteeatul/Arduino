#include "Robot.h"


Robot::Robot(double _dt):demo(false)
{
  dt = _dt;
  isInitiitialized = false;
  emergency = false;
  error = false;
  errorState = ErrorState::NO_ERROR;
  oldInitialized = false;
  
  // qui devo mettere l'inizializzazione del motore
}


void Robot::update()
{
  
  checkError();
  
  for(std::vector<AcquisitionModule*>::iterator ii = acquisitionModules.begin(); ii!= acquisitionModules.end(); ii++) {
    (*ii)->update();
  } 
  
  if(inError()) {
    turnOff();
  }

  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    (*ii)->update();
  }

  if(oldInitialized != getIsInitialized()) {
    oldInitialized = getIsInitialized();
    turnOff();
  }
}


void Robot::addActuator(Actuator* _act)
{
  actuators.push_back(_act);
}


void Robot::addAcquisitionModule(AcquisitionModule* _acq)
{
  acquisitionModules.push_back(_acq);
}


void Robot::turnOn()
{
  if(!inError()) {
    for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
      (*ii)->setControlState(Actuator::ON);
    }
  }
}


void Robot::turnOff()
{
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
      (*ii)->setControlState(Actuator::IDLE);
  }
}


bool Robot::isOn()
{
  if(checkCoherence()) {
    return (Actuator::ON == actuators.at(0)->getControlState());
  }
  else {
    return false;
  }
}


Actuator::ControlState Robot::getControlState()
{
  if(checkCoherence()) {
    return actuators.at(0)->getControlState();
  }
}


void Robot::setControlMode(Actuator::ControlMode _mode)
{
  if(checkCoherence()) {
    if(!isOn()) {
      for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
          (*ii)->setControlMode(_mode);
      }
    }
  }
}


Actuator::ControlMode Robot::getControlMode()
{
  Actuator::ControlMode _mode = actuators.at(0)->getControlMode();
  return _mode;
}


void Robot::resetError()
{
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    (*ii)->resetError();
  }
  errorState = ErrorState::NO_ERROR;
}


void Robot::checkError()
{
  bool motorError = !false;
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    motorError = motorError && !(*ii)->inError();
  } 

  if(!motorError) {errorState = ErrorState::MOTOR_ERROR;}

  //if(!checkCoherence()) {errorState = ErrorState::COHERENCE_ERROR;}

  if(emergency) {errorState = ErrorState::EMERGENCY_ERROR;}
  
  if (errorState != ErrorState::NO_ERROR) {
    error = true;
  }
  else {
    error = false;
  }
}


bool Robot::checkCoherence()
{
  Actuator::ControlState _state = actuators.at(0)->getControlState();
  bool coherence = true;
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    coherence = coherence && (_state == (*ii)->getControlState());
  }
  Actuator::ControlMode _mode = actuators.at(0)->getControlMode();
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    coherence = coherence && (_mode == (*ii)->getControlMode());
  }
  return coherence;
}


bool Robot::inError() {return error;}


void Robot::setDemo(bool _demo)
{
  demo = _demo;
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++) {
    (*ii)->setDemo(demo);
  }
}


bool Robot::inDemo()
{
  return demo;
}


bool Robot::getIsInitialized()
{
  bool initialized = true;
  for(std::vector<Actuator*>::iterator ii = actuators.begin(); ii!= actuators.end(); ii++){
      initialized = initialized && (*ii)->isInitialized();
  } 
  return initialized;
}


void Robot::setEmergencyState(bool _em)
{
  if (!demo) {emergency = _em;}
  else {emergency = false;}
}


Robot::ErrorState Robot::getErrorState()
{
  return errorState;
}
