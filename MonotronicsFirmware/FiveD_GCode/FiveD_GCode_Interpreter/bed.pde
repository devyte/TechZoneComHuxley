
/*
 * This controld the heated bed (if any).
 * In a standard Mendel (MOTHERBOARD == 2) this
 * is done by an extruder controller.
 */


#if MOTHERBOARD != 2

   #ifdef USE_TECHZONETIPMANAGE
   
void bed::setTemp(int temp)
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
byte bed::sendTemp(int temp)
{
  pinMode(bedManageData,INPUT);
  digitalWrite(bedManageClock,0);
  byte checkTimeout=1;
  for(int timeout=0;timeout<=600;timeout++)
  {
    if(digitalRead(bedManageData)!=1)
    {
      checkTimeout=0;
      break;
    }
  }
  if(checkTimeout==1)
  {
    return 0;
  }
  digitalWrite(bedManageClock,1);
  delayMicroseconds(50);
  if(timeoutHigh()==1)
  {
    return 0;
  }
  digitalWrite(bedManageClock,0);
  pinMode(bedManageData,OUTPUT);
  digitalWrite(bedManageData,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  delayMicroseconds(50);
  
  for(int mask=0x0001;mask;mask<<=1)
  {
    digitalWrite(bedManageClock,0);
    if(temp & mask)
    {
      pinMode(bedManageData,INPUT);
    } else {
      pinMode(bedManageData,OUTPUT);
      digitalWrite(bedManageData,0);
    }
    delayMicroseconds(50);
    digitalWrite(bedManageClock,1);
    delayMicroseconds(50);
  }
  digitalWrite(bedManageClock,0);
  pinMode(bedManageData,INPUT);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  if(timeoutLow()==1)
  {
    return 0;
  }
  digitalWrite(bedManageClock,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  if(timeoutHigh()==1)
  {
    return 0;
  }
  digitalWrite(bedManageClock,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  if(timeoutLow()==1)
  {
    return 0;
  }
  digitalWrite(bedManageClock,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
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

int bed::requestTemp()
{
  pinMode(bedManageClock,OUTPUT);
  pinMode(bedManageData,INPUT);
  digitalWrite(bedManageClock,0);
  byte checkTimeout=1;
  int timeout;
  for(timeout=0;timeout<=2500;timeout++)
  {
    if(digitalRead(bedManageData)!=1)
    {
      checkTimeout=0;
      break;
    }
  }
  if(checkTimeout==1)
  {
    return timeout+10;
  }
  pinMode(bedManageData,INPUT);
  digitalWrite(bedManageClock,1);
  if(timeoutHigh()==1)
  {
    return 1989;
  }
  digitalWrite(bedManageClock,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  delayMicroseconds(50);
  return recieveTemp();
}
int bed::recieveTemp()
{
  pinMode(bedManageData,INPUT);
  int recieve=0;
  for(int mask=0x0001;mask;mask<<=1)
  {
  digitalWrite(bedManageClock,0);
  delayMicroseconds(50);
  digitalWrite(bedManageClock,1);
  delayMicroseconds(50);
    if(digitalRead(bedManageData)==1)
    {
      recieve=recieve | mask;
    }
  }
  recieve=recieve & 0b0111111111111111;
  return recieve;
}
byte bed::timeoutHigh()
{
 for(int timeout=0; timeout<=200; timeout++)
  {
    if(digitalRead(bedManageData)==1)
    {
      return 0;
    }
  } 
  return 1;
}
byte bed::timeoutLow()
{
 for(int timeout=0; timeout<=200; timeout++)
  {
    if(digitalRead(bedManageData)==0)
    {
      return 0;
    }
  } 
  return 1;
}
   #else
static PIDcontrol bPID(BED_HEATER_PIN, BED_TEMPERATURE_PIN, true);
#endif

bed::bed(byte heat, byte temp)
{
  heater_pin = heat;
  temp_pin = temp;

  manageCount = 0;
  
   #ifdef USE_TECHZONETIPMANAGE
   #else
  bedPID = &bPID;
#endif
  //setup our pins

  pinMode(heater_pin, OUTPUT);
  pinMode(temp_pin, INPUT);
  
  analogWrite(heater_pin, 0);


  setTemperature(0);
}

void bed::controlTemperature()
{   
   #ifdef USE_TECHZONETIPMANAGE
   #else
  bedPID->pidCalculation();
#endif
}


void bed::waitForTemperature()
{
   #ifdef USE_TECHZONETIPMANAGE
   #else
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
      if(newT >= bedPID->getTarget() - HALF_DEAD_ZONE)
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
  #endif
}

// This is a fatal error - something is wrong with the heater.

void bed::temperatureError()
{
  sprintf(talkToHost.string(), "Bed temperature not rising - hard fault.");
  talkToHost.setFatal();
}

#endif
