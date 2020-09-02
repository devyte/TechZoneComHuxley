

#include "configuration.h"
#include "pins.h"
#include "intercom.h"
#include "extruder.h" 
#include "Temperature.h"
#include "OneWire.h"



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
 * Darwin-style motherboard
 */

#if MOTHERBOARD == 1 

extruder::extruder(byte md_pin, byte ms_pin, byte h_pin, byte f_pin, byte t_pin, byte vd_pin, byte ve_pin, byte se_pin, float spm)
{
  motor_dir_pin = md_pin;
  motor_speed_pin = ms_pin;
  heater_pin = h_pin;
  fan_pin = f_pin;
  temp_pin = t_pin;
  valve_dir_pin = vd_pin;
  valve_en_pin = ve_pin;
  step_en_pin = se_pin;
  sPerMM = spm;
  
  //setup our pins
  pinMode(motor_dir_pin, OUTPUT);
  pinMode(motor_speed_pin, OUTPUT);
  pinMode(heater_pin, OUTPUT);

  pinMode(temp_pin, INPUT);
  pinMode(valve_dir_pin, OUTPUT); 
  pinMode(valve_en_pin, OUTPUT);

  //initialize values
  digitalWrite(motor_dir_pin, EXTRUDER_FORWARD);

  analogWrite(heater_pin, 0);
  analogWrite(motor_speed_pin, 0);
  digitalWrite(valve_dir_pin, false);
  digitalWrite(valve_en_pin, 0);

  // The step enable pin and the fan pin are the same...
  // We can have one, or the other, but not both

  if(step_en_pin >= 0)
  {
    pinMode(step_en_pin, OUTPUT);
    disableStep();
  } 
  else
  {
    pinMode(fan_pin, OUTPUT);
    analogWrite(fan_pin, 0);
  }

  //these our the default values for the extruder.
  e_speed = 0;
  targetTemperature = 0;
  max_celsius = 0;
  heater_low = 64;
  heater_high = 255;
  heater_current = 0;
  valve_open = false;

  //this is for doing encoder based extruder control
  //        rpm = 0;
  //        e_delay = 0;
  //        error = 0;
  //        last_extruder_error = 0;
  //        error_delta = 0;
  e_direction = EXTRUDER_FORWARD;

  //default to cool
  setTemperature(targetTemperature);
}

void extruder::shutdown()
{
  analogWrite(heater_pin, 0); 
  digitalWrite(step_en_pin, !ENABLE_ON);
  valveSet(false, 500);
}


void extruder::valveSet(bool open, int dTime)
{
  waitForTemperature();
  valve_open = open;
  digitalWrite(valve_dir_pin, open);
  digitalWrite(valve_en_pin, 1);
  delay(dTime);
  digitalWrite(valve_en_pin, 0);
}


void extruder::setTemperature(int temp)
{
  targetTemperature = temp;
  max_celsius = (temp*11)/10;

  // If we've turned the heat off, we might as well disable the extrude stepper
  // if(target_celsius < 1)
  //  disableStep(); 
}

/**
 *  Samples the temperature and converts it to degrees celsius.
 *  Returns degrees celsius.
 */
 
#ifdef USE_DS2760
void outputDebugInfo(int outNum)
{
  int dataDelay = 20;
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
digitalWrite(OWTroubleshoot,bitRead(outNum,0));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,1));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,2));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,3));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,4));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,5));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,6));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,7));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,8));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,9));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,10));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,11));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,12));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,13));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,14));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,15));
	delayMicroseconds(dataDelay);
}
void outputDebugInfoByte(byte outNum)
{
  int dataDelay = 20;
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
digitalWrite(OWTroubleshoot,bitRead(outNum,0));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,1));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,2));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,3));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,4));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,5));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,6));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,7));
	delayMicroseconds(dataDelay);
}
int readDS2760(int useWire)
{
  
  word vIn;
  word tmpCJ;
  word tCuV;
  byte sign;
  word cjComp;
  word tempC;
  word tblLo;
  word tblHi;
  word eePntr;
  word testVal;
  byte error;
  byte tmpRead;
  int i;
  int tries=0;
  
  OneWire myWire(useWire);
	 
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x18,0);
  tmpRead=myWire.read();
  myWire.reset();
  
  if(bitRead(tmpRead,7))
  {
    tmpCJ=0;
  } else {
    tmpCJ=tmpRead;
  }	
  
  
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x0E,0);
  tmpRead=myWire.read();
  tCuV=tmpRead;
  tCuV=tCuV << 8;
  tmpRead=myWire.read();
  myWire.reset();
  tCuV=tCuV | tmpRead;
  tCuV=tCuV >> 3;
  if(bitRead(tCuV,12))
  {
     tCuV = tCuV | 0xF000;
     tCuV = tCuV ^ 0xFFFF;
  }
   
 
    cjComp=temptable[(int)(tmpCJ/10)][0] + (tmpCJ-temptable[(int)(tmpCJ/10)][1])*(temptable[(int)(tmpCJ/10)+1][0]-temptable[(int)(tmpCJ/10)][0])/10;

    cjComp=cjComp+tCuV;
  if(cjComp>temptable[NUMTEMPS-1][0])
  {
    return temptable[NUMTEMPS-1][1];
  }
  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0]>cjComp)
    {
      return temptable[i-1][1] + (cjComp-temptable[i-1][0])*(temptable[i][1]-temptable[i-1][1])/(temptable[i][0]-temptable[i-1][0]);
      
    }
  }
  return 2000;  //if it fails to get a temperature return 2000 so it will turn off the heat.
}
#endif
int extruder::getTemperature()
{
#ifdef USE_THERMISTOR
  int raw = sampleTemperature();

  int celsius = 0;
  byte i;

  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0] > raw)
    {
      celsius  = temptable[i-1][1] + 
        (raw - temptable[i-1][0]) * 
        (temptable[i][1] - temptable[i-1][1]) /
        (temptable[i][0] - temptable[i-1][0]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == NUMTEMPS) celsius = temptable[i-1][1];
  // Clamp to byte
  if (celsius > 255) celsius = 255; 
  else if (celsius < 0) celsius = 0; 

  return celsius;
#else
#ifdef USE_DS2760
  
  word vIn;
  word tmpCJ;
  word tCuV;
  byte sign;
  word cjComp;
  word tempC;
  word tblLo;
  word tblHi;
  word eePntr;
  word testVal;
  byte error;
  byte tmpRead;
  int i;
  int useWire;
  int tries=0;
  
  useWire=OW;
  OneWire myWire(useWire);
  do {
  OneWire myWire(useWire);
	 
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x18,0);
  tmpRead=myWire.read();
  myWire.reset();
  
  if(bitRead(tmpRead,7))
  {
    tmpCJ=0;
  } else {
    tmpCJ=tmpRead;
  }	
    if(tmpCJ==0)
    {
      tries+=1;
      if(useWire==OW)
      {
         useWire=OWTroubleshoot;
      } else {
        useWire=OW;
      }
    } else {
      tries=10;
    }
	} while (tries<5);
  
 if(tries<10)
 {
   return 2000;
 }
  
  tmpRead=readDS2760(useWire);
  
  if(tmpRead==0)
  {
    return 1999;
  } else {
    return tmpRead;
  }
#else
  return ( 5.0 * sampleTemperature() * 100.0) / 1024.0;
#endif
#endif
}



/*
* This function gives us an averaged sample of the analog temperature pin.
 */
int extruder::sampleTemperature()
{
  int raw = 0;

  //read in a certain number of samples
  for (byte i=0; i<TEMPERATURE_SAMPLES; i++)
    raw += analogRead(temp_pin);

  //average the samples
  raw = raw/TEMPERATURE_SAMPLES;

  //send it back.
  return raw;
}

/*!
 Manages extruder functions to keep temps, speeds etc
 at the set levels.  Should be called only by manage_all_extruders(),
 which should be called in all non-trivial loops.
 o If temp is too low, don't start the motor
 o Adjust the heater power to keep the temperature at the target
 */
void extruder::manage()
{
  //make sure we know what our temp is.
  int current_celsius = getTemperature();
  byte newheat = 0;

  //put the heater into high mode if we're not at our target.
  if (current_celsius < targetTemperature)
    newheat = heater_high;
  //put the heater on low if we're at our target.
  else if (current_celsius < max_celsius)
    newheat = heater_low;

  // Only update heat if it changed
  if (heater_current != newheat) {
    heater_current = newheat;
    analogWrite(heater_pin, heater_current);
  }
}

#endif


/***************************************************************************************************************************
 * 
 * Arduino Mega motherboard
 */
#if MOTHERBOARD == 3
#ifdef monolithic

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
 
#ifdef  PID_CONTROL

   pGain = TEMP_PID_PGAIN;
   iGain = TEMP_PID_IGAIN;
   dGain = TEMP_PID_DGAIN;
   temp_iState = 0;
   temp_dState = 0;
   temp_iState_min = -TEMP_PID_INTEGRAL_DRIVE_MAX/iGain;
   temp_iState_max = TEMP_PID_INTEGRAL_DRIVE_MAX/iGain;
   iState = 0;
   dState = 0;
   previousTime = millis()/MILLI_CORRECTION;

#endif
  //initialize values
  digitalWrite(motor_dir_pin, 1);
  digitalWrite(motor_step_pin, 0);
  
  targetTemperature = 0;
  currentTemperature = 0;
  analogWrite(heater_pin, 0);

  //setTemperature(0);
  
#ifdef PASTE_EXTRUDER
  valve_dir_pin = vd_pin;
  valve_en_pin = ve_pin;
  pinMode(valve_dir_pin, OUTPUT); 
  pinMode(valve_en_pin, OUTPUT);
  digitalWrite(valve_dir_pin, false);
  digitalWrite(valve_en_pin, 0);
  valve_open = false;
#endif
}


#ifdef  PID_CONTROL

// With thanks to Adam at Makerbot and Tim at BotHacker
// see http://blog.makerbot.com/2009/10/01/open-source-ftw/

byte extruder::pidCalculation(int dt)
{
  int output;
  int error;
  float pTerm, iTerm, dTerm;

  error = targetTemperature - currentTemperature;

  pTerm = pGain * error;

  temp_iState += error;
  temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
  iTerm = iGain * temp_iState;

  dTerm = dGain * (currentTemperature - temp_dState);
  temp_dState = currentTemperature;

  output = pTerm + iTerm - dTerm;
  output = constrain(output, 0, 255);

  return output;
}

#endif

void extruder::controlTemperature()
{
  currentTemperature = internalTemperature(); 
    
#ifdef PID_CONTROL

  int dt;
  unsigned long time = millis()/MILLI_CORRECTION;  // Correct for fast clock
  dt = time - previousTime;
  previousTime = time;
  if (dt > 0) // Don't do it when millis() has rolled over
    analogWrite(heater_pin, pidCalculation(dt));

#else
#ifdef USE_TECHZONETIPMANAGE
#else
  // Simple bang-bang temperature control

  if(targetTemperature > currentTemperature)
    digitalWrite(OUTPUT_C, 1);
  else
    digitalWrite(OUTPUT_C, 0);
#endif
#endif 
}

/* 
 Temperature reading function  
 With thanks to: Ryan Mclaughlin - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1230859336
 for the MAX6675 code
 */
#ifdef USE_DS2760
void outputDebugInfo(int outNum)
{
  int dataDelay = 20;
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
digitalWrite(OWTroubleshoot,bitRead(outNum,0));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,1));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,2));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,3));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,4));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,5));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,6));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,7));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,8));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,9));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,10));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,11));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,12));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,13));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,14));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,15));
	delayMicroseconds(dataDelay);
}
void outputDebugInfoByte(byte outNum)
{
  int dataDelay = 20;
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(10);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(3);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(3);
digitalWrite(OWTroubleshoot,0);
digitalWrite(OWTroubleshoot,bitRead(outNum,0));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,1));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,2));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,3));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
        delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,4));
        delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,5));
	delayMicroseconds(dataDelay);
digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
digitalWrite(OWTroubleshoot,bitRead(outNum,6));
	delayMicroseconds(dataDelay);
 digitalWrite(OWTroubleshoot,0);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,1);
	delayMicroseconds(1);
  digitalWrite(OWTroubleshoot,bitRead(outNum,7));
	delayMicroseconds(dataDelay);
}
int readDS2760(int useWire)
{
  
  word vIn;
  word tmpCJ;
  word tCuV;
  byte sign;
  word cjComp;
  word tempC;
  word tblLo;
  word tblHi;
  word eePntr;
  word testVal;
  byte error;
  byte tmpRead;
  int i;
  int tries=0;
  
  OneWire myWire(useWire);
	 
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x18,0);
  tmpRead=myWire.read();
  myWire.reset();
  
  if(bitRead(tmpRead,7))
  {
    tmpCJ=0;
  } else {
    tmpCJ=tmpRead;
  }	
  
  
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x0E,0);
  tmpRead=myWire.read();
  tCuV=tmpRead;
  tCuV=tCuV << 8;
  tmpRead=myWire.read();
  myWire.reset();
  tCuV=tCuV | tmpRead;
  tCuV=tCuV >> 3;
  if(bitRead(tCuV,12))
  {
     tCuV = tCuV | 0xF000;
     tCuV = tCuV ^ 0xFFFF;
  }
   
 
    cjComp=temptable[(int)(tmpCJ/10)][0] + (tmpCJ-temptable[(int)(tmpCJ/10)][1])*(temptable[(int)(tmpCJ/10)+1][0]-temptable[(int)(tmpCJ/10)][0])/10;

    cjComp=cjComp+tCuV;
  if(cjComp>temptable[NUMTEMPS-1][0])
  {
    return temptable[NUMTEMPS-1][1];
  }
  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0]>cjComp)
    {
      return temptable[i-1][1] + (cjComp-temptable[i-1][0])*(temptable[i][1]-temptable[i-1][1])/(temptable[i][0]-temptable[i-1][0]);
      
    }
  }
  return 2000;  //if it fails to get a temperature return 2000 so it will turn off the heat.
}
#endif

#ifdef USE_TECHZONETIPMANAGE

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
#endif
int internalTemperature()
{
#ifdef USE_DS2760
  
  word vIn;
  word tmpCJ;
  word tCuV;
  byte sign;
  word cjComp;
  word tempC;
  word tblLo;
  word tblHi;
  word eePntr;
  word testVal;
  byte error;
  byte tmpRead;
  int i;
  int useWire;
  int tries=0;
  
  useWire=OW;
  OneWire myWire(useWire);
  do {
  OneWire myWire(useWire);
	 
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x18,0);
  tmpRead=myWire.read();
  myWire.reset();
  
  if(bitRead(tmpRead,7))
  {
    tmpCJ=0;
  } else {
    tmpCJ=tmpRead;
  }	
    if(tmpCJ==0)
    {
      tries+=1;
      if(useWire==OW)
      {
         useWire=OWTroubleshoot;
      } else {
        useWire=OW;
      }
    } else {
      tries=10;
    }
	} while (tries<5);
  
 if(tries<10)
 {
   return 2000;
 }
  
  tmpRead=readDS2760(useWire);
  
  if(tmpRead==0)
  {
    return 1999;
  } else {
    return tmpRead;
  }
  
  myWire.reset();
  myWire.write(SkipNet,0);
  myWire.write(RdReg,0);
  myWire.write(0x0E,0);
  tmpRead=myWire.read();
  tCuV=tmpRead;
  tCuV=tCuV << 8;
  tmpRead=myWire.read();
  myWire.reset();
  tCuV=tCuV | tmpRead;
  tCuV=tCuV >> 3;
  if(bitRead(tCuV,12))
  {
     tCuV = tCuV | 0xF000;
     tCuV = tCuV ^ 0xFFFF;
  }
   
 
    cjComp=temptable[(int)(tmpCJ/10)][0] + (tmpCJ-temptable[(int)(tmpCJ/10)][1])*(temptable[(int)(tmpCJ/10)+1][0]-temptable[(int)(tmpCJ/10)][0])/10;

    cjComp=cjComp+tCuV;
  if(cjComp>temptable[NUMTEMPS-1][0])
  {
    return temptable[NUMTEMPS-1][1];
  }
  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0]>cjComp)
    {
      return temptable[i-1][1] + (cjComp-temptable[i-1][0])*(temptable[i][1]-temptable[i-1][1])/(temptable[i][0]-temptable[i-1][0]);
      
    }
  }
  return 2000;  //if it fails to get a temperature return 2000 so it will turn off the heat.
#endif
#ifdef USE_THERMISTOR
  int raw = analogRead(TEMP_PIN);

  int celsius = raw;
  byte i;

  // TODO: This should do a binary chop

  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0] > raw)
    {
      celsius  = temptable[i-1][1] + 
        (raw - temptable[i-1][0]) * 
        (temptable[i][1] - temptable[i-1][1]) /
        (temptable[i][0] - temptable[i-1][0]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == NUMTEMPS) celsius = temptable[i-1][1];
  // Clamp to byte
  if (celsius > 255) celsius = 255; 
  else if (celsius < 0) celsius = 0; 

  return celsius;
#endif

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
   #ifdef USE_TECHZONETIPMANAGE
   #else
  extruderPID->shutdown();
  #endif
  // Motor off
  digitalWrite(motor_en_pin, !ENABLE_ON);
  // Close valve
#ifdef PASTE_EXTRUDER
  valveSet(false, 500);
#endif
}


#ifdef PASTE_EXTRUDER

bool extruder::valveTimeCheck(int millisecs)
{
  if(valveAlreadyRunning)
  {
    if(millis() >= valveEndTime)
    {
      valveAlreadyRunning = false;
      return true;
    }
    return false;
  }

  valveEndTime = millis() + millisecs*MILLI_CORRECTION;
  valveAlreadyRunning = true;
  return false;
}

void extruder::valveTurn(bool close)
{
  if(valveAtEnd)
    return;
    
  byte valveRunningState = VALVE_STARTING;
  if(digitalRead(OPTO_PIN))
  {
    seenHighLow = true;
    valveRunningState = VALVE_RUNNING;
  } else
  {
    if(!seenHighLow)
     valveRunningState = VALVE_STARTING;
    else
     valveRunningState = VALVE_STOPPING; 
  }    
   
  switch(valveRunningState)
  {
  case VALVE_STARTING: 
          if(close)
             digitalWrite(H1D, 1);
          else
             digitalWrite(H1D, 0);
          digitalWrite(H1E, HIGH);
          break;
          
  case VALVE_RUNNING:
          return;
  
  case VALVE_STOPPING:
          if(close)
            digitalWrite(H1D, 0);
          else
            digitalWrite(H1D, 1);
            
          if(!valveTimeCheck(10))
            return;
            
          digitalWrite(H1E, LOW);
          valveState = close;
          valveAtEnd = true;
          seenHighLow = false;
          break;
          
  default:
          break;
  }  
}

void extruder::valveMonitor()
{
  if(valveState == requiredValveState)
    return;
  valveAtEnd = false;
  valveTurn(requiredValveState);
} 

void extruder::kickStartValve()
{
  if(digitalRead(OPTO_PIN))
  {
     if(requiredValveState)
       digitalWrite(H1D, 1);
     else
       digitalWrite(H1D, 0);
     digitalWrite(H1E, HIGH);    
  }
} 
#endif
#else
static PIDcontrol ePID(EXTRUDER_0_HEATER_PIN, EXTRUDER_0_TEMPERATURE_PIN, false);


//*******************************************************************************************

// Motherboard 3 - Arduino Mega

extruder::extruder(byte stp, byte dir, byte en, byte heat, byte temp, float spm)
{
  motor_step_pin = stp;
  motor_dir_pin = dir;
  motor_en_pin = en;
  heater_pin = heat;
  temp_pin = temp;
  sPerMM = spm;
  manageCount = 0;
  extruderPID = &ePID;
  
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
  
  analogWrite(heater_pin, 0);

  setTemperature(0);
  
#ifdef PASTE_EXTRUDER
  valve_dir_pin = vd_pin;
  valve_en_pin = ve_pin;
  pinMode(valve_dir_pin, OUTPUT); 
  pinMode(valve_en_pin, OUTPUT);
  digitalWrite(valve_dir_pin, false);
  digitalWrite(valve_en_pin, 0);
  valve_open = false;
#endif
}


// Stop everything

void extruder::shutdown()
{
  // Heater off;
  setTemperature(0);
  extruderPID->shutdown();
  // Motor off
  digitalWrite(motor_en_pin, !ENABLE_ON);
  // Close valve
#ifdef PASTE_EXTRUDER
  valveSet(false, 500);
#endif
}


#ifdef PASTE_EXTRUDER

bool extruder::valveTimeCheck(int millisecs)
{
  if(valveAlreadyRunning)
  {
    if(millis() >= valveEndTime)
    {
      valveAlreadyRunning = false;
      return true;
    }
    return false;
  }

  valveEndTime = millis() + millisecs*MILLI_CORRECTION;
  valveAlreadyRunning = true;
  return false;
}

void extruder::valveTurn(bool close)
{
  if(valveAtEnd)
    return;
    
  byte valveRunningState = VALVE_STARTING;
  if(digitalRead(OPTO_PIN))
  {
    seenHighLow = true;
    valveRunningState = VALVE_RUNNING;
  } else
  {
    if(!seenHighLow)
     valveRunningState = VALVE_STARTING;
    else
     valveRunningState = VALVE_STOPPING; 
  }    
   
  switch(valveRunningState)
  {
  case VALVE_STARTING: 
          if(close)
             digitalWrite(H1D, 1);
          else
             digitalWrite(H1D, 0);
          digitalWrite(H1E, HIGH);
          break;
          
  case VALVE_RUNNING:
          return;
  
  case VALVE_STOPPING:
          if(close)
            digitalWrite(H1D, 0);
          else
            digitalWrite(H1D, 1);
            
          if(!valveTimeCheck(10))
            return;
            
          digitalWrite(H1E, LOW);
          valveState = close;
          valveAtEnd = true;
          seenHighLow = false;
          break;
          
  default:
          break;
  }  
}

void extruder::valveMonitor()
{
  if(valveState == requiredValveState)
    return;
  valveAtEnd = false;
  valveTurn(requiredValveState);
} 

void extruder::kickStartValve()
{
  if(digitalRead(OPTO_PIN))
  {
     if(requiredValveState)
       digitalWrite(H1D, 1);
     else
       digitalWrite(H1D, 0);
     digitalWrite(H1E, HIGH);    
  }
} 
#endif
#endif


#endif

