/*! @file Actuator.h
    @author Alessio Prini (alessio.prini@itia.cnr.it)
    @version Revision 0.1
    @brief Header file actuator class and MotorCtrl classes
*/

#ifndef _STEP_ACTUATOR_
#define _STEP_ACTUATOR_

#include <RTIncrementalEncoder.h>


class Actuator {
  
  public:
   
  /// Actuator Control States  
  enum ControlState {
    IDLE,	///< 00 IDLE - No movement
    ON		///< 01 ON 
  };

  /// Actuator Control Modes
  enum ControlMode {
    POSITION,	///< 00 Position loop closed on the actuator
    VELOCITY,	///< 01 Velocity loop closed on the actuator
    IMPEDANCE, ///< 02 Impedance loop closed on the actuator
    HOMING,	///< 02 Homing procedure performed by the actuator
    NO_MODE	///< 03 No movement
  };
  
  /// Actuator Errors
  enum MotorError {
    NO_ERROR,			///< 00 No error
    EXCESSIVE_CURRENT_ERROR,	///< 01 Current over the limit for more than 5s
    EXCESSIVE_VELOCITY_ERROR,	///< 02 velocity exceed the 1.2 x Maximum Velocity
    EXCESSIVE_ACCELERATION_ERROR,	///< 03 acceleration exceed the maximum value
    SOFT_END_STROKE_ERROR,	///< 04 Actuator position exceed the limits
    HARD_END_STROKE_ERROR,	///< 05 End stroke contact (if the error is enabled)
    TRACKING_ERROR,		///< 06 PID loop tracking error. The difference between real pos and target esceed the limit
    EMERGENCY_ERROR,		///< 07 Emergency button pressed
    STEPPER_ERROR, 		///< 08 sarebbe bello metterlo per indicare quando qualcosa non va
  };

  /// Homing types
  enum HomingType {
    MAX_ENDSTROKE,	///< 00 Homing at the max endstroke
    MIN_ENDSTROKE,	///< 01 Homing at the min endstroke
    MAX_ENDSTROKE_AND_HOLD,	///< 02 Homing and the max endstroke and hold position
    MIN_ENDSTROKE_AND_HOLD,	///< 03 Homing and the min endstroke and hold position
    ACTUAL,			///< 04 Homing will be executed in the actual postion
  };

  /// Home state list
  enum HomeState {
    MOTOR_NOT_INITIALIZED,	///< 00 Not initiaialized
    MOTOR_INITIALIZED		///< 01 Initialalized
  };
 
  
  RTEncoder* encoder; // da aggiungere il getVel per stima velocita -- cambiare libreria eventualmente
  Actuator(double _deltaTime, RTEncoder & _encoder);
  
  // variable method
  double getPos(); //!< Get current position
  void setPos(double _pos); //!< Set current position
  double getVel(); //!< Get current velocity
  void setVel(double _vel); //!< Set current velocity
  double getImp(); //!< Get current impedance
  void setImp(double _imp); //!< Set current velocity
  double getAcc(); //!< Get current acceleration
  void setAcc(double _acc); //!< Set current acceleration
  void resetEncoder(double pos); //!< Reset current motor position
  
  // machine state control
  virtual void setControlState(Actuator::ControlState _ctrlState); //!< Set control state
  virtual Actuator::ControlState getControlState(); //!< Get current control state
  virtual bool setControlMode(Actuator::ControlMode _ctrlMode); //!< Set control mode
  virtual Actuator::ControlMode getControlMode(); //!< Get current control mode
  void setDemo(bool _demo); //!< Set demo state (true for demo, false no demo)
  bool isDemo(); //!< Return demo flag state
  Actuator::MotorError getErrorState(); //!< Get error state
  virtual void checkError(); //!< Check error
  virtual void checkWarning(); //!< Check warning 
  virtual void resetError(); //!< Reset error state, put motor in IDLE
  bool inError(); //!< Return true if motor is in error
  bool inWarning(); //!< Return true if motor is in warning
  void setHomingType(Actuator::HomingType _homingType);//!< Set homing type
  Actuator::HomingType getHomingType();//!< Return selected homing type
  void setHomeState(Actuator::HomeState _homeState);
  Actuator::HomeState getHomeState();  
  
  // homing parameter
  void setHomeOffset(double _offset); //!< Set home offset, motor position after homing
  double getHomeOffset(); //!< Get current home offset
  bool isInitialized();
  virtual void setHomingVelocity(double _vel){};
  
  virtual void update();
  
  void setDeltaTime(double _delta); //!< Set delta time of control loop in seconds
  double getDeltaTime(); //!< Get control delta time in seconds

    
  void setEndStrokeErrorEnabled(bool _enabled);//!< If set true hitting endstroke cause an error
  bool getEndStrokeErrorEnabled();//!< Return if endstroke hit cause an error
  
  void setWarningEnabled(bool _enabled); //!< A warning lead to null velocity in certain cases(i.e. endstroke hit)
  bool getWarningEnabled();//!< Return if warnings are enabled
  
  // target
  void setTargetPos(double _target);//!< Set target position
  double getTargetPos();//! Retrun target position
  void setTargetVel(double _target);//!< Set target velocity
  double getTargetVel();//!< Return target velocity
  void setTargetImp(double _target);//!< Set target impedance
  double getTargetImp();//!< Return target impedance

  bool targetReached();//!< Return if target is within the tollerance setted
  void setTargetReachedToll(double _toll);//!< Set target tollerance
  
  // endStrokeFunction
  void setMaxEndStrokeState(bool _state);//!< Set max endstroke state -- must be connected to endstroke pin
  bool getMaxEndStrokeState();//!< Get max endstroke state
  void setMinEndStrokeState(bool _state);//!< Set min endstroke state -- must be connected to endstroke pin
  bool getMinEndStrokeState();//!< Set min endstroke state
  
  
  virtual void setMaxVelocity(double _maxVel){maxVel = _maxVel;};
  virtual void setMaxImpedance(double _maxImp){maxImp = _maxImp;};
  
  
  // for the actuator access -- sort of mutex for this shared property.
  // maybe it is possible to use ChibiOS mutex
  bool updated(); //!< Set the updated flag to true. Useful as mutex in case of actuator is shared between dfferent source
  void release(); //!< Realease the updates "mutex"
  
  virtual void init(){};
  virtual void initFast(){};

  void setPositionLimits(double _min, double _max){
      maxSoftEndStroke = _max;
      minSoftEndStroke = _min;
  }

protected:

  // init variables
  double pos = 0;
  double vel = 0;
  double imp = 0;
  double acc = 0;
  
  // limit control 
  double maxSoftEndStroke, minSoftEndStroke;
  double maxHardEndStroke, minHardEndStroke;
  bool maxEndStrokeState, minEndStrokeState;
    
  // machine state variable
  Actuator::ControlState ctrlState;
  Actuator::ControlMode ctrlMode;
  Actuator::MotorError error;

  // flags
  bool demo;
  bool warningFlag;
  bool warningEnabledFlag;

  // home parameter
  double homeOffset;
  HomingType homingType;
  HomeState homeState;
  
  // control variable
  double deltaTime;
  double targetPos;
  double targetVel;
  double targetImp;
  
  //limit control
  double maxVel;
  double maxImp;
  double maxAcc;
  double maxCurrent, maxCurrentDelay;
  
  double tolerance;
  bool endStrokeErrorFlag; 
  bool mutex;
  bool updatedFlag;
  
};



#endif
