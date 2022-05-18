/*! @file Robot.h
    @author Alessio Prini (alessio.prini@itia.cnr.it)
    @version Revision 0.1
    @brief Header file for Robot class
*/
#ifndef _ROBOT_
#define _ROBOT_
#include "Actuator.h"
#include "AcquisitionModule.h"
#include <ArduinoSTL.h>

class Robot {
  
public:
  
  enum ErrorState {
      NO_ERROR,
      EMERGENCY_ERROR,
      MOTOR_ERROR,
      COHERENCE_ERROR,
  };
  
  Robot(double dt);
  
  void update();
  void addActuator(Actuator*);
  void addAcquisitionModule(AcquisitionModule*);
  void resetError();
  bool inError();
  void checkError();
  bool checkCoherence();

  void turnOn();
  void turnOff();
  bool isOn();
  void setControlMode(Actuator::ControlMode);
  Actuator::ControlMode getControlMode();
  Actuator::ControlState getControlState();

  void setDemo(bool);
  bool inDemo();
  
  void setEmergencyState(bool);
  Robot::ErrorState getErrorState();
  
  bool getIsInitialized();
  bool oldInitialized;
  
private:  

  std::vector<Actuator*> actuators;
  std::vector<AcquisitionModule*> acquisitionModules;
  
  double dt;
  bool demo;
  bool isInitiitialized = false;
  bool emergency = false;
  bool error;
  ErrorState errorState;
  
};


#endif
