/*
 * This controld the heated bed (if any).
 */

#ifndef BED_H
#define BED_H


class bed
{
  
public:
   bed(byte heat, byte temp);
   void waitForTemperature();
   
   void setTemperature(int temp);
   int getTemperature();
   int getTarget();
   void slowManage();
   void manage();
   void shutdown();

void setTemp(int temp);
byte sendTemp(int temp);
int requestTemp();
int recieveTemp();
byte timeoutHigh();
byte timeoutLow();
 
private:

   int targetTemperature;
   int count;
   int oldT, newT;
   long manageCount;

   int sampleTemperature();
   void controlTemperature();
   void temperatureError(); 

// The pins we control
   byte heater_pin,  temp_pin;
 
};

inline int bed::getTarget()
{
   return targetTemperature; 
}

inline void bed::slowManage()
{
  manageCount = 0;  

  controlTemperature();
}

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

#endif
