
// Class for controlling each extruder
//
// Adrian Bowyer 14 May 2009

#ifndef EXTRUDER_H
#define EXTRUDER_H

void manageAllExtruders();

void newExtruder(byte e);


/**********************************************************************************************
*
* RepRap Arduino Mega Motherboard
*
*/
//******************************************************************************************************

#define TEMP_PID_INTEGRAL_DRIVE_MAX 110
#define TEMP_PID_PGAIN 5.0
#define TEMP_PID_IGAIN 0.1
#define TEMP_PID_DGAIN 100.0

#define MILLI_CORRECTION 64

class extruder
{
  
public:
   extruder(byte step, byte dir, byte en, byte heat, byte temp, float spm);
   void waitForTemperature();
   
   void setDirection(bool direction);
   void setCooler(byte e_speed);
   void setTemperature(int temp);
   int getTemperature();
   int getTarget();
   void slowManage();
   void manage();
   void sStep();
   void enableStep();
   void disableStep();
   void shutdown();
   float stepsPerMM();
   void  stepsPerMM(float spm);
   void controlTemperature();   
   void valveSet(bool open, int dTime); 

   
void setTemp(int temp);
byte sendTemp(int temp);
int requestTemp();
int recieveTemp();
byte timeoutHigh();
byte timeoutLow();
 
private:

//   int targetTemperature;
   int count;
   int oldT, newT;
   float sPerMM;
   long manageCount;
   
   int sampleTemperature();

   void temperatureError(); 
    int targetTemperature;
    int currentTemperature;

// The pins we control
   byte motor_step_pin, motor_dir_pin, heater_pin,  temp_pin,  motor_en_pin;

   //byte fan_pin;
 
};

void  extruder::stepsPerMM(float spm)
{
  sPerMM = spm;
}

inline void extruder::sStep()
{
	digitalWrite(motor_step_pin, HIGH);
	digitalWrite(motor_step_pin, LOW);  
}

inline void extruder::slowManage()
{
  manageCount = 0;  

  controlTemperature();
}

inline void extruder::setTemperature(int tp)
{
  targetTemperature=tp;
  setTemp(tp);
}

inline int extruder::getTemperature()
{
  //int last=currentTemperature;
  //currentTemperature=requestTemp();
  //if(currentTemperature==0)
  //{
  //  currentTemperature=last;
  //}
  //return currentTemperature;  
  return requestTemp();
}

inline int extruder::getTarget()
{
  return targetTemperature;  
}

inline void extruder::manage()
{
  
}

inline void extruder::setDirection(bool direction)
{
  digitalWrite(motor_dir_pin, direction);  
}

inline void extruder::setCooler(byte e_speed)
{
  //analogWrite(fan_pin, e_speed);   
}




inline void extruder::enableStep()
{
    digitalWrite(motor_en_pin, ENABLE_ON);
}

inline void extruder::disableStep()
{
#if DISABLE_E
    digitalWrite(motor_en_pin, !ENABLE_ON);
#endif
}


inline void extruder::valveSet(bool closed, int dTime)
{
}



//*********************************************************************************************************

extern extruder* ex[ ];
extern byte extruder_in_use;

inline float extruder::stepsPerMM()
{
  return sPerMM;
}

#endif
