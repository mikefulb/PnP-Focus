// Moonlite-compatible stepper controller
//
// Original code by orly.andico@gmail.com, 13 April 2014
// Modified by Anat Ruangrassamee (aruangra@yahoo.com), 26 September 2017


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
int stepdelay = 8;
int eoc = 0;
int idx = 0;
long pos=0;
long distanceToGo = 0;
long currentPosition = 8000;
float tempC = 0;
long timer_millis = 0;

#if 0
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
#endif

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

  Serial.println("PnPFocus V1a-msf");

  Serial.println("Looking for temperature sensors");

  sensors.begin();
  
  memset(line, 0, MAXCOMMAND);
  //currentPosition=EEPROMReadlong(0);

  setStepDelay(speed);

  delay(1000);
  
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);

  timer_millis=millis();
}

void loop(){
  // update temperature every 10 seconds unless moving!
  if (!isRunning)
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
         
//  Serial.print(String(currentPosition)+ "     ");
  
  if (isRunning) {
    if (distanceToGo > 0) {
      currentPosition = currentPosition + 1;
      if (currentPosition > 65535)
          currentPosition = 0;        
      distanceToGo = distanceToGo - 1;
      myStepper.step(1);
      delay(stepdelay);
    } else {
      if (distanceToGo < 0) {
        currentPosition = currentPosition - 1;
        if (currentPosition < 0)
            currentPosition = 65535;
        distanceToGo = distanceToGo + 1;
        myStepper.step(-1);
        delay(stepdelay);
      } else {
         isRunning = 0;
         myStepper.release();
      }
    }
  }

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
        //Serial1.write("01#");
      } 
      else {
        Serial.print("00#");
        //Serial1.write("00#");

        // EEPROM
        //EEPROMWritelong(0, currentPosition);
      }
    }

    // initiate a move
    if (!strcasecmp(cmd, "FG")) {
      isRunning = 1;
    }

    // stop a move
    if (!strcasecmp(cmd, "FQ")) {
      isRunning = 0;
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

      // EEPROM
      //EEPROMWritelong(0, currentPosition);
      
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
                Serial.print("C6#");
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
    
    // whether half-step is enabled or not, always return "00"
    if (!strcasecmp(cmd, "GH")) {
      Serial.print("00#");
    }
 
    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")) {
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
  }
  
} // end loop

long hexstr2long(char *line) {
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}




