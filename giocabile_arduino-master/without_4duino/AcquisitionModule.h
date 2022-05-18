#ifndef ACQUISITION_MODULE
#define ACQUISITION_MODULE

#define ACQUIRE_LENGTH 200

#include "UnitConversion.h"
#include "Actuator.h"
#include <ArduinoSTL.h>

template <typename type>
type sign(type value) {
  return type((value>0)-(value<0));
}



class AcquisitionModule{
public:
  AcquisitionModule(Actuator* _joint){
    joint = _joint;
    acquisitionReady = false;
    acquisitionStarted = false;
    sendingAcquisition = false;
    triggerPosActive = false;
  };
  
  void sendAcquisition(){if(!acquisitionStarted && acquisitionReady){sendingAcquisition = true; acquireEv = 0;}};
  bool activeDataSend(){return sendingAcquisition;}
  void startAcquisition(int dec){
    if(!sendingAcquisition){
      acquisitionStarted = true;
      tick = 0;
      if(dec <= 0){
	decimation = dec;}
    }
  };
  void closeAcquisition(){acquisitionStarted = false; tick = 0; acquireEv = 0;acquisitionReady= true;};
  bool isAcquisitionReady(){return acquisitionReady;};

  
  void update(){
    
    
      if(triggerPosActive){
	if (sign(joint->getPos() - triggerPos) != triggerPosSign){
	  triggerPosActive = false;
	  startAcquisition(triggerDec);
	}
      }

     if(triggerVelActive){
	if (sign(joint->getVel() - triggerVel) != triggerVelSign){
	  triggerVelActive = false;
	  startAcquisition(triggerDec);
	}
      }

      
      
      if(acquisitionStarted){
	if(tick%decimation == 0){
	  acquireTime[acquireEv] = ConvertUsToS(micros()/10);
	  acquirePos[acquireEv] = joint->getPos();
	  acquireVel[acquireEv] = joint->getVel();
	  acquireTargetPos[acquireEv] = joint->getTargetPos();
	  acquireTargetVel[acquireEv] = joint->getTargetVel();
	  acquireEv++;
	}
	tick++;
	if(acquireEv == ACQUIRE_LENGTH){
	  closeAcquisition();
	}
      }
  };  
  
  double* getAcquireData(){
    if(sendingAcquisition && acquisitionReady){

      static double vett[5];
      vett[0] = acquireTime[acquireEv];
      vett[1] = acquirePos[acquireEv];
      vett[2] = acquireVel[acquireEv];
      vett[3] = acquireTargetPos[acquireEv];
      vett[4] = acquireTargetVel[acquireEv];
      acquireEv++;
      if(acquireEv == ACQUIRE_LENGTH){
	sendingAcquisition = false;
	acquisitionReady = false;
	acquireEv = 0;
      }
      return vett;
    }else{
      double vett[] = {0,0,0,0,0};
      return vett;
    }
  };
  
  
  
  void activeTriggerPos(double _pos, int dec){
    triggerPos = _pos;
    triggerPosActive = true;
    triggerPosSign = sign(joint->getPos() - triggerPos);
    triggerDec = dec;
  };


  void activeTriggerVel(double _vel, int dec){
    triggerVel = _vel;
    triggerVelActive = true;
    triggerVelSign = sign(joint->getVel() - triggerVel);
    triggerDec = dec;
  };
  
private:
  Actuator* joint;
  
  bool acquisitionStarted = false;
  bool sendingAcquisition = false;
  bool acquisitionReady = false;
  
  double acquireTime[ACQUIRE_LENGTH];
  double acquirePos[ACQUIRE_LENGTH];
  double acquireVel[ACQUIRE_LENGTH];
  double acquireTargetPos[ACQUIRE_LENGTH];
  double acquireTargetVel[ACQUIRE_LENGTH];
  
  bool triggerPosActive = false;
  int triggerPosSign = 0;
  double triggerPos = 0;
  
  
  bool triggerVelActive = false;
  int triggerVelSign = 0;
  double triggerVel = 0;
  
  
  int triggerDec = 1;
  
  int acquireEv = 0;
  int decimation = 1;
  int tick = 0;
  
};


#endif
