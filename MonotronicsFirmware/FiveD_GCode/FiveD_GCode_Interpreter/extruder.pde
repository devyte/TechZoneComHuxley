

#include "configuration.h"
#include "pins.h"
#include "intercom.h"
#include "extruder.h" 
#include "Temperature.h"



// Select a new extruder

void newExtruder(byte e)
{
  if(e < 0)
    return;
  if(e >= EXTRUDER_COUNT)
    return;

  if(e != extruder_in_use)
  {  
    extruder_in_use = e;
    setUnits(cdda[0]->get_units());
  }
}

//*************************************************************************

// Extruder functions that are the same for all extruders.

void extruder::waitForTemperature()
{
  byte seconds = 0;
  bool warming = true;
  count = 0;
  newT = 0;
  oldT = newT;

  while (true)
  {
    newT += getTemperature();
    count++;
    if(count > 5)
    {
      newT = newT/5;
      if(newT >= getTarget() - HALF_DEAD_ZONE)
      {
        warming = false;
        if(seconds > WAIT_AT_TEMPERATURE)
          return;
        else 
          seconds++;
      } 

      if(warming)
      {
        if(newT > oldT)
          oldT = newT;
        else
        {
          // Temp isn't increasing - extruder hardware error
          temperatureError();
          return;
        }
      }

      newT = 0;
      count = 0;
    }
    for(int i = 0; i < 1000; i++)
    {
      manage();
      delay(1);
    }
  }
}

// This is a fatal error - something is wrong with the heater.

void extruder::temperatureError()
{
  sprintf(talkToHost.string(), "Extruder temperature not rising - hard fault.");
  talkToHost.setFatal();
}



/***************************************************************************************************************************
 * 
 * Arduino Mega motherboard
 */
#if MOTHERBOARD == 3

extruder::extruder(byte stp, byte dir, byte en, byte heat, byte temp, float spm)
{
  motor_step_pin = stp;
  motor_dir_pin = dir;
  motor_en_pin = en;
  heater_pin = heat;
  temp_pin = temp;
  sPerMM = spm;
  manageCount = 0;
  //extruderPID = &ePID;

  //fan_pin = ;

  //setup our pins
  pinMode(motor_step_pin, OUTPUT);
  pinMode(motor_dir_pin, OUTPUT);
  pinMode(motor_en_pin, OUTPUT);
  pinMode(heater_pin, OUTPUT);
  pinMode(temp_pin, INPUT);
  
  disableStep();
 
  //initialize values
  digitalWrite(motor_dir_pin, 1);
  digitalWrite(motor_step_pin, 0);
  
  targetTemperature = 0;
  currentTemperature = 0;
  analogWrite(heater_pin, 0);

  //setTemperature(0);
  

}



void extruder::controlTemperature()
{
  currentTemperature = internalTemperature(); 
    
}



void extruder::setTemp(int temp)
{
  for(int x=0;x<200;x++)
  {
    noInterrupts();
    if(sendTemp(temp)==1)
    {
      interrupts();
      break;
    }
    
    interrupts();
  }
}
byte extruder::sendTemp(int temp)
{
  pinMode(tipManageData,INPUT);
  digitalWrite(tipManageClock,0);
  byte checkTimeout=1;
  for(int timeout=0;timeout<=600;timeout++)
  {
    if(digitalRead(tipManageData)!=1)
    {
      checkTimeout=0;
      break;
    }
  }
  if(checkTimeout==1)
  {
    return 0;
  }
  digitalWrite(tipManageClock,1);
  delayMicroseconds(50);
  if(timeoutHigh()==1)
  {
    return 0;
  }
  digitalWrite(tipManageClock,0);
  pinMode(tipManageData,OUTPUT);
  digitalWrite(tipManageData,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  delayMicroseconds(50);
  
  for(int mask=0x0001;mask;mask<<=1)
  {
    digitalWrite(tipManageClock,0);
    if(temp & mask)
    {
      pinMode(tipManageData,INPUT);
    } else {
      pinMode(tipManageData,OUTPUT);
      digitalWrite(tipManageData,0);
    }
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    delayMicroseconds(50);
  }
  digitalWrite(tipManageClock,0);
  pinMode(tipManageData,INPUT);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  if(timeoutLow()==1)
  {
    return 0;
  }
  digitalWrite(tipManageClock,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  if(timeoutHigh()==1)
  {
    return 0;
  }
  digitalWrite(tipManageClock,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  if(timeoutLow()==1)
  {
    return 0;
  }
  digitalWrite(tipManageClock,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  if(timeoutHigh()==1)
  {
    return 0;
  }
  if(recieveTemp()==temp)
  {
    return 1;
  }
  return 0;
}

int extruder::requestTemp()
{
  pinMode(tipManageClock,OUTPUT);
  pinMode(tipManageData,INPUT);
  digitalWrite(tipManageClock,0);
  byte checkTimeout=1;
  int timeout;
  for(timeout=0;timeout<=2500;timeout++)
  {
    if(digitalRead(tipManageData)!=1)
    {
      checkTimeout=0;
      break;
    }
  }
  if(checkTimeout==1)
  {
    return timeout+10;
  }
  pinMode(tipManageData,INPUT);
  digitalWrite(tipManageClock,1);
  if(timeoutHigh()==1)
  {
    return 1989;
  }
  digitalWrite(tipManageClock,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  delayMicroseconds(50);
  return recieveTemp();
}
int extruder::recieveTemp()
{
  pinMode(tipManageData,INPUT);
  int recieve=0;
  for(int mask=0x0001;mask;mask<<=1)
  {
  digitalWrite(tipManageClock,0);
  delayMicroseconds(50);
  digitalWrite(tipManageClock,1);
  delayMicroseconds(50);
    if(digitalRead(tipManageData)==1)
    {
      recieve=recieve | mask;
    }
  }
  recieve=recieve & 0b0111111111111111;
  return recieve;
}
byte extruder::timeoutHigh()
{
 for(int timeout=0; timeout<=200; timeout++)
  {
    if(digitalRead(tipManageData)==1)
    {
      return 0;
    }
  } 
  return 1;
}
byte extruder::timeoutLow()
{
 for(int timeout=0; timeout<=200; timeout++)
  {
    if(digitalRead(tipManageData)==0)
    {
      return 0;
    }
  } 
  return 1;
}

int internalTemperature()
{

#ifdef AD595_THERMOCOUPLE
  return ( 5.0 * analogRead(TEMP_PIN) * 100.0) / 1024.0; //(int)(((long)500*(long)analogRead(TEMP_PIN))/(long)1024);
#endif  

#ifdef MAX6675_THERMOCOUPLE
  skipCount++;
  if (skipCount>SKIP_CLOCK_COUNT)
  {
    skipCount=0;
  return currentTemperature;
  }
else
{
  int value = 0;
  byte error_tc;


  digitalWrite(TC_0, 0); // Enable device

  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCK,1);
  digitalWrite(SCK,0);

  /* Read bits 14-3 from MAX6675 for the Temp
   	 Loop for each bit reading the value 
   */
  for (int i=11; i>=0; i--)
  {
    digitalWrite(SCK,1);  // Set Clock to HIGH
    value += digitalRead(SO) << i;  // Read data and add it to our variable
    digitalWrite(SCK,0);  // Set Clock to LOW
  }

  /* Read the TC Input inp to check for TC Errors */
  digitalWrite(SCK,1); // Set Clock to HIGH
  error_tc = digitalRead(SO); // Read data
  digitalWrite(SCK,0);  // Set Clock to LOW

  digitalWrite(TC_0, 1); //Disable Device

  if(error_tc)
    return 2000;
  else
    return value/4;

}
#endif
}

// Stop everything

void extruder::shutdown()
{
  // Heater off;
  setTemperature(0);
  // Motor off
  digitalWrite(motor_en_pin, !ENABLE_ON);
  // Close valve
}




#endif

