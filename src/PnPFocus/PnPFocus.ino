// Moonlite-compatible stepper controller
//
// Original code by orly.andico@gmail.com, 13 April 2014
// Modified by Anat Ruangrassamee (aruangra@yahoo.com), 26 September 2017
// Modified by Michael Fulbright (mike.fulrbight@pobox.com>, 27 November 2017


//
// MSF
//
// NOTE on wiring to an Arduino
//
// DS18B20 ->  Connect PWR and GND together and connect to GND on Arduino
//             Connect DATA to A1 on Arduino and pull-up with 4.7k to +5V for parasitic power
//
// PROGRAM PROTECTION ->  Connect 10uF (or greater) cap between GND and RESET - use jumper so you can
//                        remove and program Arduino then put jumper back and you can't accidentally
//                        reprogram the Arduino!  


#include <EEPROM.h>

#include "OneWire.h"
#include "DallasTemperature.h"

// Data wire is plugged into pin A1 on the Arduino
#define ONE_WIRE_BUS A1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#include "ArdumotoStepper.h"

const int stepsPerRevolution = 3600;  // change this to fit the number of steps per revolution for your motor

// initialize the stepper library:
ArdumotoStepper myStepper(stepsPerRevolution);

#define MAXCOMMAND 8

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
int isRunning = 0;
int speed = 32;
volatile bool reverse=false;
int stepdelay = 8;
int eoc = 0;
int idx = 0;
long pos=0;
volatile long distanceToGo = 0;
volatile long currentPosition = 8000;
float tempC = 0;
long timer_millis = 0;


//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

// EEPROM data
//
// V2b-msf -> write a single long
//            Byte 0 = Reverse flag - 0 NORMAL FF REVERSE
//            Byte 1 = Speed
//            Byte 2 = 00 FULL FF HALF stepping
//            Byte 3 == UNDEF

void writeEEPROMData()
{
  unsigned long l;
  unsigned long b;
  
  l = 0x00;
  
  // Byte 0 Reverse Flag
  l = (reverse) ? 0xFF : 0x00;

  // Byte 1 = Speed
  l |= speed << 8;

  // Byte 2 = stepping
  if (myStepper.getStepMode() == FULLSTEP)
    b = 0x00;
  else
    b = 0xFF;
    
    l |= b << 16;

  // Byte 3 = UNDEF
  //l |=  0 << 24;

  EEPROMWritelong(0x0, l);  
}

void readEEPROMData()
{
  unsigned long l;
  unsigned long b;

  l = EEPROMReadlong(0x00);
//  Serial.println(l, HEX);

  // Byte 0 Reverse
  reverse = (l & 0x000000FF);
//  Serial.println(reverse);
  
  // Byte 1 = Speed
  speed = (l & 0x0000FF00) >> 8;
//  Serial.println(speed);
 
  if (speed != 0x01 && speed != 0x02 && speed != 0x04 && speed != 0x08 && speed != 0x10 && speed != 0x20)
    speed = 0x20;

  // Byte 2 = stepping
  b = (l >> 16) && 0xFF;
//  Serial.println(b, HEX);
  if (b)
    myStepper.setStepMode(HALFSTEP);
  else
    myStepper.setStepMode(FULLSTEP);

  // Byte 3 = ignore
}


// given a moonlite "speed" set step delay var
void setStepDelay(int speed)
{
  switch (speed)
  {
    case 32:
        stepdelay = 32;
        break;
    case 16:
        stepdelay = 16;
        break;
    case  8:
        stepdelay = 8;
        break;
    case  4:
        stepdelay = 4;
        break;
    case  2:
        stepdelay = 4;
        break;
    default:
        stepdelay = 4;
        break;
  }
}

void setup()
{  
  Serial.begin(9600);

  readEEPROMData();

  sensors.begin();
  
  memset(line, 0, MAXCOMMAND);

  setStepDelay(speed);

  delay(1000);
  
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);

  timer_millis=millis();

  // connect timer interrupt to Timer0
  // we will NOT change anything about Timer0 config since it is used for millis()!
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A); 
}

volatile bool enableIntr = false;   // true means interrupt will send pulses and is active
volatile uint8_t speedcntIntr = 0;  // number of interrupts per step sent
volatile uint8_t stepcntIntr = 0;   // internal var for interrupt to count steps
volatile int8_t  dirIntr = 0;       // -1 or 1 - determines direction of steps
//volatile uint16_t nstepsIntr = 0;   // number of steps to move - modified by interrupt
//                                   // when it reaches 0 then enableIntr is set false
// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  if (enableIntr)
  {
    //Serial.write("#");
    stepcntIntr++;
    if (!(stepcntIntr % speedcntIntr))
    {

      // are we done?
      if (distanceToGo == 0)
      {
        enableIntr = false;
        //Serial.write("$");
        return;       
      }
            
      myStepper.step((reverse) ? -dirIntr : dirIntr);
      stepcntIntr = 0;
      distanceToGo -= dirIntr;
      currentPosition += dirIntr;

      //Serial.write(".");
      //Serial.print(distanceToGo);

    }
  }
}

void printInfo()
{
  int i;
  char tempString[8];

  Serial.println();
  Serial.println("INFO DUMP");
  Serial.println("PnPFocus V2b-msf");

  // position
  sprintf(tempString, "%05d", currentPosition);
  Serial.print("Current Position: ");
  Serial.println(tempString);

  // speed and stepping
  Serial.print("Current Motor Speed: ");
  Serial.println(speed);

  Serial.print("Current Step Mode: ");
  if (myStepper.getStepMode() == FULLSTEP)
    Serial.println("FULL");
  else
    Serial.println("HALF"); 

  Serial.print("Current Reverse Flag: ");
  if (reverse)
    Serial.println("REVERSE");
  else
    Serial.println("NORMAL"); 
        
  // temperature sensors 
  Serial.print("# of temperature sensors: ");
  i = sensors.getDeviceCount();
  Serial.println(i);

  // FIXME doesnt handle more than 1 temperature sensor
  if (i > 0)
  {
    Serial.print("  Sensor 1 = ");
    Serial.print((int)tempC);
    if ((tempC-(int)tempC) < 0.5)
      Serial.print(".0");
    else
      Serial.print(".5");
    Serial.println(" C");
  }

}

void loop(){
  // update temperature every 10 seconds!
  if ((millis() - timer_millis) > 10000) {
    timer_millis=millis();
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);

    // set 0 to 1 to get output debug message
    if (0) {
      if ((tempC < -50) || (tempC > 50)){
        Serial.print("NA  ");
      } else {
        //Serial.print(String(round(tempC)) + "C");
        Serial.print(tempC);
        Serial.print("C");
      } 
    }
  }
       
#if 0
  if (isRunning) {
    if (distanceToGo > 0) {
      currentPosition = currentPosition + 1;
      if (currentPosition > 65535)
          currentPosition = 0;        
      distanceToGo = distanceToGo - 1;
      myStepper.step( (reverse) ? -1 : 1);
      delay(stepdelay);
    } else {
      if (distanceToGo < 0) {
        currentPosition = currentPosition - 1;
        if (currentPosition < 0)
            currentPosition = 65535;
        distanceToGo = distanceToGo + 1;
        myStepper.step( (reverse) ? 1 : -1);
        delay(stepdelay);
      } else {
         isRunning = 0;
         myStepper.release();
      }
    }
  }
#else
  // did interrupt finish moving
  if (isRunning)
  {
    if (!enableIntr)
    {
      isRunning = false;
      myStepper.release();
    }
  }
#endif

  // heartbeat debug
  //Serial.write(".");

  // read the command until the terminating # character
  while (Serial.available() && !eoc) {
    inChar = Serial.read();
    //Serial1.write(inChar);
    
    if (inChar != '#' && inChar != ':') {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    } 
    else {
      if (inChar == '#') {
        eoc = 1;
      }
    }
  }

  // process the command we got
  if (eoc) {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);
    if (len >= 2) {
      strncpy(cmd, line, 2);
    }

    if (len > 2) {
      strncpy(param, line + 2, len - 2);
    }

    memset(line, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;

    // motor is moving - 01 if moving, 00 otherwise
    if (!strcasecmp(cmd, "GI")) {
      if (isRunning) {
        Serial.print("01#");
      } 
      else {
        Serial.print("00#");
      }
    }

    // initiate a move
    if (!strcasecmp(cmd, "FG")) {
      if (distanceToGo != 0)
      {
        isRunning = 1;
        noInterrupts();
        enableIntr = true;
        speedcntIntr = stepdelay;
        if (distanceToGo < 0)
          dirIntr = -1;
        else
          dirIntr = 1;
        interrupts();
  
//        Serial.println("Move start");
//        Serial.println(enableIntr);
//        Serial.println(speedcntIntr);
//        Serial.println(distanceToGo);
      }      
    }

    // stop a move
    if (!strcasecmp(cmd, "FQ")) {
      isRunning = 0;
      noInterrupts();
      enableIntr = false;
      //nstepsIntr = 0;
      distanceToGo = 0;
      interrupts();
      myStepper.release();
    } 

    // get the current motor position
    if (!strcasecmp(cmd, "GP")) {
      pos = currentPosition;
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // set new motor position
    if (!strcasecmp(cmd, "SN")) {
      pos = hexstr2long(param);
      distanceToGo = pos - currentPosition;
    }

    // set current motor position
    if (!strcasecmp(cmd, "SP")) {
      pos = hexstr2long(param);
      currentPosition = pos;
    }

    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "SD")) {
      speed = hexstr2long(param);
      // the Moonlite speed setting is ignored.
      setStepDelay(speed);
    }

    // get the current temperature
    if (!strcasecmp(cmd, "GT")) {
           if ((tempC < -50) || (tempC > 50)){
                Serial.print("00C6#");
           } else {
                char tempString[6];
                int tpval = (tempC * 2);
                sprintf(tempString, "%04X", (int) tpval);
                Serial.print(tempString);;
                Serial.print("#");
           }
    }

    // get the temperature coefficient.
    if (!strcasecmp(cmd, "GC")) {
      Serial.print("02#");
    }

    // get the new motor position (target)
    if (!strcasecmp(cmd, "GN")) {
      pos = currentPosition + distanceToGo;
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }
    
    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "GD")) {
      char tempString[6];
      sprintf(tempString, "%02X", speed);
      Serial.print(tempString);
      Serial.print("#");
    }
    
    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")) {
      Serial.print("00#");
    }
 
    // whether half-step is enabled or not
    if (!strcasecmp(cmd, "GH")) {
      if (myStepper.getStepMode() == FULLSTEP)
        Serial.print("00#");
      else
        Serial.print("FF#");
    }

    // home the motor
    if (!strcasecmp(cmd, "PH")) { 

    }

    // step mode the motor
    if (!strcasecmp(cmd, "SF")) { 
      myStepper.setStepMode(FULLSTEP);
    }

    // step mode the motor
    if (!strcasecmp(cmd, "SH")) { 
      myStepper.setStepMode(HALFSTEP);
    }

    // firmware value, always return "10"
    if (!strcasecmp(cmd, "GV")) {
      Serial.print("10#");
      //Serial1.write("10#");
    }

    // Custom commands for me
    if (!strcasecmp(cmd, "IN")) {
      printInfo();
    }
    
    // set reverse flag - 00 for normal and FF to reverse direction
    if (!strcasecmp(cmd, "RV")) {
      long i;
      
      i = hexstr2long(param);

      reverse = (i != 0);
    }   

    // write/read eeprom
    if (!strcasecmp(cmd, "WR")) {
      writeEEPROMData();
    }

    // write/read eeprom
    if (!strcasecmp(cmd, "RD")) {
      readEEPROMData();
    }
        
  }
  
} // end loop

long hexstr2long(char *line) {
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}




