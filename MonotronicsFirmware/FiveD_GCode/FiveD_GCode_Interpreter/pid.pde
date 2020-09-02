#ifdef USE_TECHZONETIPMANAGE
#else
#include "pid.h"
#include "OneWire.h"

#if MOTHERBOARD != 2

// Based on the excellent Wikipedia PID control article.
// See http://en.wikipedia.org/wiki/PID_controller

PIDcontrol::PIDcontrol(byte hp, byte tp, bool b)
{
   heat_pin = hp;
   temp_pin = tp;
   doingBed = b;
   if(doingBed)
   {
     pGain = B_TEMP_PID_PGAIN;
     iGain = B_TEMP_PID_IGAIN;
     dGain = B_TEMP_PID_DGAIN;
   } else
   {
     pGain = E_TEMP_PID_PGAIN;
     iGain = E_TEMP_PID_IGAIN;
     dGain = E_TEMP_PID_DGAIN;
   }   
   currentTemperature = 0;
   setTarget(0);
   pinMode(heat_pin, OUTPUT);
   pinMode(temp_pin, INPUT); 
}

/*
 Set the target temperature.  This also
 resets the PID to, for example, remove accumulated integral error from
 a long period when the heater was off and the requested temperature was 0 (which it
 won't go down to, even with the heater off, so the integral error grows).  
*/

void PIDcontrol::setTarget(int t)
{
   targetTemperature = t;
   previousTime = millis();
   previousError = 0;
   integral = 0;  
}

/* 
 Temperature reading function  
 With thanks to: Ryan Mclaughlin - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1230859336
 for the MAX6675 code
 */
/*
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
#endif*/
void PIDcontrol::internalTemperature(short table[][2])
{
#ifdef USE_THERMISTOR
  int raw = 0;
  for(int i = 0; i < 3; i++)
    raw += analogRead(temp_pin);
    
  raw = raw/3;

  byte i;

  // TODO: This should do a binary chop

  for (i=1; i<NUMTEMPS; i++)
  {
    if (table[i][0] > raw)
    {
      currentTemperature  = table[i-1][1] + 
        (raw - table[i-1][0]) * 
        (table[i][1] - table[i-1][1]) /
        (table[i][0] - table[i-1][0]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i >= NUMTEMPS) currentTemperature = table[i-1][1];
  // Clamp to byte
  //if (celsius > 255) celsius = 255; 
  //else if (celsius < 0) celsius = 0; 

#endif

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
    currentTemperature = 2000;
 }
  
  tmpRead=readDS2760(useWire);
  
  if(tmpRead==0)
  {
     currentTemperature = 1999;
  } else {
     currentTemperature = tmpRead;
  }
#endif
#ifdef AD595_THERMOCOUPLE
  currentTemperature = ( 5.0 * analogRead(pin* 100.0) / 1024.0; //(int)(((long)500*(long)analogRead(TEMP_PIN))/(long)1024);
#endif  

#ifdef MAX6675_THERMOCOUPLE
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
    currentTemperature = 2000;
  else
    currentTemperature = value/4;

#endif

}


void PIDcontrol::pidCalculation()
{
  if(doingBed)
    internalTemperature(bedtemptable);
  else
    internalTemperature(temptable);
  time = millis();
  float dt = 0.001*(float)(time - previousTime);
  previousTime = time;
  if (dt <= 0) // Don't do it when millis() has rolled over
    return;
    
  float error = (float)(targetTemperature - currentTemperature);
  integral += error*dt;
  float derivative = (error - previousError)/dt;
  previousError = error;
  int output = (int)(error*pGain + integral*iGain + derivative*dGain);
  if(!doingBed && cdda[tail]->extruding())
    output += EXTRUDING_INCREASE;
  output = constrain(output, 0, 255);
  analogWrite(heat_pin, output);
}


#endif
#endif
