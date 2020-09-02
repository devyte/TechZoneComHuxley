/*
 * This controld the heated bed (if any).
 */

#ifndef BED_H
#define BED_H

#if MOTHERBOARD != 2  

class bed
{
  
public:
   bed(byte heat, byte temp);
   void waitForTemperature();
   
   void setTemperature(int temp);
   int getTemperature();
   void slowManage();
   void manage();
   void shutdown();
   #ifdef USE_TECHZONETIPMANAGE
void setTemp(int temp);
byte sendTemp(int temp);
int requestTemp();
int recieveTemp();
byte timeoutHigh();
byte timeoutLow();
 #endif
 
private:

//   int targetTemperature;
   int count;
   int oldT, newT;
   long manageCount;
   #ifdef USE_TECHZONETIPMANAGE
   #else
   PIDcontrol* bedPID;    // Temperature control - extruder...
#endif
   int sampleTemperature();
   void controlTemperature();
   void temperatureError(); 

// The pins we control
   byte heater_pin,  temp_pin;
 
};

inline void bed::slowManage()
{
  manageCount = 0;  

  controlTemperature();
}

   #ifdef USE_TECHZONETIPMANAGE
inline void bed::manage()
{
 
}

// Stop everything

inline void bed::shutdown()
{
  setTemperature(0);
  

}

inline void bed::setTemperature(int tp)
{
  setTemp(tp);
}

inline int bed::getTemperature()
{
 return requestTemp();
}

   #else
inline void bed::manage()
{
  manageCount++;
  if(manageCount > SLOW_CLOCK)
    slowManage();   
}

// Stop everything

inline void bed::shutdown()
{
  setTemperature(0);
  
   #ifdef USE_TECHZONETIPMANAGE
   #else
  bedPID->shutdown();
  #endif
}

inline void bed::setTemperature(int tp)
{
   #ifdef USE_TECHZONETIPMANAGE
   #else
  bedPID->setTarget(tp);
  #endif
}

inline int bed::getTemperature()
{
   #ifdef USE_TECHZONETIPMANAGE
   #else
  return bedPID->temperature();  
  #endif
}

#endif
#endif
#endif
