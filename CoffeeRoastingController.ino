/****************************************************************************

// https://gist.github.com/soemarko/9ac80d5df13155fb3302
// ORIGINAL RoastLoggerMax6675.ino 
  
  This sketch is for use with MAX 31855 thermocouple interface chips.

  See the "Contributed Libraries" section of http://www.arduino.cc/en/Reference/Libraries
  for details of how to install it.
  
 
Revision history: - of RoastLoggerMax6675
20120201:  Version 1.2 - Made available for download from downloads page
20120303:  Version 1.3 - Added #define CELSIUS as default, user to comment out for Fahrenheit output
20120324:  Version 1.4 - Serial output limited to 1 decimal place
20200115:  Version 1.5 - Updated to include 4 relay PWM outputs and MAX31855 for temperature inputs, removed LCD output and potentiometer control, only control via computer

****************************************************************************/


// ------------------------------------------------------------------------------------------
// Copyright (c) 2011-2012, Tom Coxon (GreenBean TMC)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

/****************************************************************************
 * 
 * Arduino control of a Hottop roaster by T R Coxon
 * 
 * Using an Arduino Duemilanove 328 on Hottop P and B Roasters
 *
 * Sends data and receives instructions as key=value pairs.                 
 * Sends temperatures T1 and T2 at sendTimePeriod intervals as "T1=123.6" 
 * Accepts instructions for power level, set temperature etc.
 * Example: Send to Arduino "power=80"  or "setT=240.6"  
 ****************************************************************************/

/****************************************************************************
 * 
 * Connections to Arduino and Behmor for heater power control:
 * 
 * Please note that the following describes the modifications to my Behmor
 * 1600+.  You should not attempt this unless you are suitably qualified /
 * experienced.  Modifying your roaster will probably void your warranty
 * and may result in damage or injury. You do this at your own risk.
 *
 * http://blog.soemarko.com/post/109402505018/behmor-mod-phase-2-heat-control-with-arduino
 * 
 ****************************************************************************/

/****************************************************************************
 * 
 * Modified by Soemarko Ridwan
 * 25 Jan 2015
 * for modded Behmor 1600+
 *  * Change log.
 *
 * 31 Jan 2015:
 * - Remove PID dependencies
 * - Add support for a potentiometer (5V, Pin A0)
 * - Add I2C LCD
 * - Add support for Artisan
 * 
 ****************************************************************************/

/****************************************************************************
 *
 * Modified by Chris St.Amand
 * 15 Jan 2020
 * for modded BM/HG
 * 
 * 15 Jan 2020:
 * - Normalized Variables
 * - Added support for 4 PWM Relays (fan, heat, cooling fan, motor
 * - Must download MAX31855 library - https://learn.adafruit.com/thermocouple/arduino-code
 * - Control only from Artisan
 *
 ****************************************************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

#define CELSIUS //RoastLogger default is Celsius - To output data in Fahrenheit comment out this one line 
#define DP 1  // No. decimal places for serial output

#define maxLength 30                  // maximum length for strings used

const int arduino = 1;                // controlled via digitalRead on ctrlPin
const int computer = 0;

/****************************************************************************
 * 
 * Arduino pin assignments
 * 
 * The following are the pin assignments used by my setup.
 * 
 * You only need to change the following 5 constants to suit the pin
 * assignments used by your setup.  
 * 
 ****************************************************************************/

bool isArtisan = false;

// PWM Relay Setup
const int PWM1Pin =  10;        // digital pin for pulse width modulation of heater
char PWM1Name[] = "power";      // label to take from Artisan
const int PWM2Pin =  9;         // digital pin for pulse width modulation of fan
char PWM2Name[] = "fan";        // label to take from Artisan
const int PWM3Pin =  6;         // digital pin for drum/motor/cooling fan/drop door/etc
char PWM3Name[] = "motor";      // label to take from Artisan
const int PWM4Pin =  5;         // digital pin for drum/motor/cooling fan/drop door/etc
char PWM4Name[] = "cooling";    // label to take from Artisan

// thermocouple reading Max 6675 pins
const int SCKa = 13;   // SCK on UNO (usually pin 13) to CLK pin on MAX31855
const int TC1CS = 7;    // Thermocouple 1 CS pin on MAX31855
const int TC2CS = 8;    // Thermocouple 2 CS pin on MAX31855
const int SO  = 12;    // SO on UNO (usually pin 12) to DO pin on MAX31855

const int ctrlPin = 2; // if using arduino to adjust -- was 12
// const int RelayCtrlPin1 = 3; // if using arduino to adjust -- was 11
// const int RelayCtrlPin2 = 4; // if using arduino to adjust -- was 11

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple1(SCKa, TC1CS, SO);
Adafruit_MAX31855 thermocouple2(SCKa, TC2CS, SO);

// pots 1
const int Pot1Pin = A0;
const int Pot1Max = 1020;
int Pot1Val = 0;
float Pot1SmoothedVal = 0;
float Pot1LastReadVal = 0;
float Pot1Alpha = 0.5;

// pots 2
const int Pot2Pin = A1;
const int Pot2Max = 1020;
int Pot2Val = 0;
float Pot2SmoothedVal = 0;
float Pot2LastReadVal = 0;
float Pot2Alpha = 0.5;

// LM35
// const int lmPin = A2;
// float lmTemp = 0.0;

// LCD
byte pcChar[8] = {
  0b11111,
  0b00101,
  0b00101,
  0b00010,
  0b00000,
  0b01110,
  0b10001,
  0b10001
};
byte ardChar[8] = {
  0b01110,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b01110,
        0b00000
};
LiquidCrystal_I2C lcd(0x27,16,2);

/****************************************************************************
 *  After setting the above pin assignments you can use the remainder of this
 *   sketch as is or change it, if you wish, to add additional functionality
 * 
 ****************************************************************************/


// time constants
const int PWM1timePeriod = 2000;          // total time period of PWM milliseconds see note on setupPWM1 before changing, maybe 1250/1500
const int PWM2timePeriod = 2000;          // total time period of PWM milliseconds see note on setupPWM2 before changing, maybe 1250/1500
const int PWM3timePeriod = 2000;          // total time period of PWM milliseconds see note on setupPWM1 before changing, maybe 1250/1500
const int PWM4timePeriod = 2000;          // total time period of PWM milliseconds see note on setupPWM2 before changing, maybe 1250/1500
const int SubLoopTimePeriod = 250;         // 250 ms loop to read thermocouples

// thermocouple settings
float TC1Calibrate = 0.0; // Temperature compensation for T1
float TC2Calibrate = 0.0; // Temperature compensation for T2

// set global variables

//temporary values for temperature to be read
float TC1Temp = 0.0;                      // temporary temperature variable
float TC2Temp = 0.0;                      // temporary temperature variable 
float t0 = 0 ;                            // ambient for TC4
float TC1AvgTemp = 0.0;                   // Last average temperature on thermocouple 1, Environment Temp - average of four readings
float TC2AvgTemp = 0.0;                   // Last average temperature on thermocouple 2, Bean Temp - average of four readings
float TC1Cumulative = 0.0;                // cumulative total of temperatures read before averaged
float TC2Cumulative = 0.0;                // cumulative total of temperatures read before averaged
int TC1BadReading = 0;                    // counter of no. of good readings for average calculation 
int TC2BadReading = 0;                    // counter of no. of good readings for average calculation

int inByte = 0;                           // incoming serial byte
String inString = String(maxLength);      // input String


// loop control variables
unsigned long lastTCTimerLoop;            // for timing the thermocouple loop
int tcLoopCount = 0;                      // counter to run serial output once every 4 loops of 250 ms t/c loop

// PWM variables
int  TC1timeOn;                           // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
int  TC2timeOn;                           // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
int  TC3timeOn;                           // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
int  TC4timeOn;                           // millis PWM is on out of total of timePeriod (timeOn = timePeriod for 100% power)
unsigned long TC1lastTimePeriod;          // millis since last turned on pwm
unsigned long TC2lastTimePeriod;          // millis since last turned on pwm
unsigned long TC3lastTimePeriod;          // millis since last turned on pwm
unsigned long TC4lastTimePeriod;          // millis since last turned on pwm
int power1 = 0;                          //use as %, 100 is always on, 0 always off default 100
int power2 = 0;                          //use as %, 100 is always on, 0 always off default 100
int power3 = 0;                          //use as %, 100 is always on, 0 always off default 100
int power4 = 0;                          //use as %, 100 is always on, 0 always off default 100
// int currentMotor = 0;

int controlBy = arduino;                  // default is arduino control. PC sends "pccontrol" to gain control or
                                          // swapped back to Arduino control if PC sends "arduinocontrol"


/****************************************************************************
 * MAIN SETUP
 ****************************************************************************/
void setup()
{
  // start serial port at 115200 baud:
  Serial.begin(115200);

  setupPWM1();
  setupPWM2();
  setupPWM3();
  setupPWM4();
  
  //set up pin modes for Max31855's
  pinMode(TC1CS, OUTPUT);
  pinMode(TC2CS, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(SCKa, OUTPUT);
  
  // deselect both Max31855's
  digitalWrite(TC1CS, HIGH);
  digitalWrite(TC2CS, HIGH);
  
  //pinMode(5, OUTPUT); digitalWrite(5, LOW); // force LCD to be off -- WAS 13
   
  //lcd.init();
  //lcd.backlight();
  //lcd.createChar(0, ardChar);
  //lcd.createChar(1, pcChar);

  //pinMode(ctrlPin, INPUT);
  //pinMode(RelayCtrlPin1, INPUT);
 // pinMode(RelayCtrlPin2, INPUT);

  Serial.println("Reset");            // flag that Arduino has reset used for debugging
}


/****************************************************************************
 * Main loop must not use delay!  PWM heater control relies on loop running
 * at least every 40 ms.  If it takes longer then heater will be on slightly
 * longer than planned. Not a big problem if 1% becomes 1.2%! But keep loop fast.
 * Currently loop takes about 4-5 ms to run so no problem.
 ****************************************************************************/
void loop(){
  controlBy = digitalRead(ctrlPin);

  getSerialInput();// check if any serial data waiting

  // loop to run once every 250 ms to read TC's, update Pots, etc.
  if (millis() - lastTCTimerLoop >= SubLoopTimePeriod)
  {
    lastTCTimerLoop = millis();
    doSubLoop();  
  }

  doPWM1();        // Toggle Heat on/off based on power setting
  doPWM2();        // Toggle Fan  on/off based on power setting
  doPWM3();        // Toggle Heat on/off based on power setting
  doPWM4();        // Toggle Fan  on/off based on power setting
}

/****************************************************************************
 * Set up power pwm control for heater.  Hottop uses a triac that switches
 * only on zero crossing so normal pwm will not work.
 * Minimum time slice is a half cycle or 10 millisecs in UK.  
 * Loop in this prog may need up to 10 ms to complete.
 * I will use 20 millisecs as time slice with 100 power levels that
 * gives 2000 milliseconds total time period.
 * The Hottop heater is very slow responding so 2 sec period is not a problem.
 ****************************************************************************/
void setupPWM1() {
  // set the digital pin as output:
  pinMode(PWM1Pin, OUTPUT);

  //setup PWM
  TC1lastTimePeriod = millis();
  digitalWrite(PWM1Pin, LOW); //set PWM 1 pin off to start
}

void setupPWM2() {
  // set the digital pin as output:
  pinMode(PWM2Pin, OUTPUT);
  
  //setup PWM
  TC2lastTimePeriod = millis();
  digitalWrite(PWM2Pin, LOW); //set PWM 2 pin off to start
}

void setupPWM3() {
  // set the digital pin as output:
  pinMode(PWM3Pin, OUTPUT);
  
  //setup PWM
  TC3lastTimePeriod = millis();
  digitalWrite(PWM3Pin, LOW); //set PWM 3 pin off to start
}

void setupPWM4() {
  // set the digital pin as output:
  pinMode(PWM4Pin, OUTPUT);
  
  //setup PWM
  TC4lastTimePeriod = millis();
  digitalWrite(PWM4Pin, LOW); //set PWM 4 pin off to start
}

/****************************************************************************
 * Toggles the heater on/off based on the current power level.  Power level
 * may be determined by arduino or computer.
 ****************************************************************************/
void doPWM1()
{
  TC1timeOn = PWM1timePeriod * power1 / 100; //recalc the millisecs on to get this power level, user may have changed
 
  if (millis() - TC1lastTimePeriod > PWM1timePeriod) TC1lastTimePeriod = millis();
  if (millis() - TC1lastTimePeriod < TC1timeOn ){
// if (millis() - TC1lastTimePeriod < TC1timeOn && digitalRead(RelayCtrlPin1)){
      digitalWrite(PWM1Pin, HIGH); // turn on
  } else {
      digitalWrite(PWM1Pin, LOW); // turn off
 }
 
}

void doPWM2()
{
  TC2timeOn = PWM2timePeriod * power2 / 100; //recalc the millisecs on to get this power level, user may have changed
  
  if (millis() - TC2lastTimePeriod > PWM2timePeriod) TC2lastTimePeriod = millis();
  if (millis() - TC2lastTimePeriod < TC2timeOn ){
    // if (millis() - TC2lastTimePeriod < TC2timeOn && digitalRead(RelayCtrlPin2)){
    digitalWrite(PWM2Pin, HIGH); // turn on
  } else {
    digitalWrite(PWM2Pin, LOW); // turn off
  }
 
}

void doPWM3()
{
  TC3timeOn = PWM3timePeriod * power3 / 100; //recalc the millisecs on to get this power level, user may have changed
  
  if (millis() - TC3lastTimePeriod > PWM3timePeriod) TC3lastTimePeriod = millis();
  if (millis() - TC3lastTimePeriod < TC3timeOn ){
    digitalWrite(PWM3Pin, HIGH); // turn on
  } else {
    digitalWrite(PWM3Pin, LOW); // turn off
  }
 
}

void doPWM4()
{
  TC4timeOn = PWM4timePeriod * power4 / 100; //recalc the millisecs on to get this power level, user may have changed
  
  if (millis() - TC4lastTimePeriod > PWM4timePeriod) TC4lastTimePeriod = millis();
  if (millis() - TC4lastTimePeriod < TC4timeOn ){
    digitalWrite(PWM4Pin, HIGH); // turn on
  } else {
    digitalWrite(PWM4Pin, LOW); // turn off
  }
 
}

/****************************************************************************
 * Called to set power level. Now always safe as hardware only turns heater on
 * if BOTH the Arduino AND the Hottop control board call for heat.
 ****************************************************************************/
void setPowerLevel1(int p)
{
  if (p > -1 && p < 101) power1 = p;
}

void setPowerLevel2(int p)
{
  if (p > -1 && p < 101) power2 = p;
}

void setPowerLevel3(int p)
{
  if (p > -1 && p < 101) power3 = p;
}

void setPowerLevel4(int p)
{
  if (p > -1 && p < 101) power4 = p;
}

/****************************************************************************
 * Called when an input string is received from computer
 * designed for key=value pairs or simple text commands. 
 * Performs commands and splits key and value 
 * and if key is defined sets value otherwise ignores
 ****************************************************************************/
void doInputCommand()
{
  float v = -1;
  inString.toLowerCase();
  int indx = inString.indexOf('=');

  if (indx < 0) {  //this is a message not a key value pair
  
  if(inString.equals("chan;1200")) {
  isArtisan = true;
  Serial.println("# Active channels set to 2");
    }
    else if(inString.equals("read")) {
  String x = String(TC2AvgTemp) + "," + String(TC1AvgTemp);
  // String x = String(lmTemp) + "," + String(TC2AvgTemp) + "," + String(TC1AvgTemp);
  Serial.println(x);
    }
    Serial.flush();

  } 
  else {  //this is a key value pair for decoding
    
    String key = inString.substring(0, indx);
    String value = inString.substring(indx+1, inString.length());

    //parse string value and return float v
    char buf[value.length()+1];
    value.toCharArray(buf,value.length()+1);
    v = atof (buf); 
 
    //only set value if we have a valid positive number - atof will return 0.0 if invalid
    if (v >= 0)
    {
      if (key.equals(PWM1Name)      && controlBy == computer && v < 101){
        setPowerLevel1((long) v);//convert v to integer for power     
      }
      else if (key.equals(PWM2Name)   && controlBy == computer && v < 101) {
         setPowerLevel2((long) v);//convert v to integer for power 
      }
      else if (key.equals(PWM3Name) && controlBy == computer && v < 101) {
         setPowerLevel3((long) v);//convert v to integer for power 
      }
      else if (key.equals(PWM4Name)  && controlBy == computer && v < 101) {
         setPowerLevel4((long) v);//convert v to integer for power 
      }
    }
  }
}

/****************************************************************************
 * check if serial input is waiting if so add it to inString.  
 * Instructions are terminated with \n \r or 'z' 
 * If this is the end of input line then call doInputCommand to act on it.
 ****************************************************************************/
void getSerialInput()
{
  //check if data is coming in if so deal with it
  if (Serial.available() > 0) {

    // read the incoming data as a char:
    char inChar = Serial.read();
    // if it's a newline or return or z, print the string:
    if ((inChar == '\n') || (inChar == '\r') || (inChar == 'z')) {

      //do whatever is commanded by the input string
      if (inString.length() > 0) doInputCommand();
      inString = "";        //reset for next line of input
    } 
    else {
      // if we are not at the end of the string, append the incoming character
      if (inString.length() < maxLength) {
                inString += inChar; 

      }
      else {
        // empty the string and set it equal to the incoming char:
      //  inString = inChar;
           inString = "";
           inString += inChar;
      }
    }
  }
}

/****************************************************************************
 * Send data to computer once every second.  Data such as temperatures, etc.
 * This allows current settings to be checked by the controlling program
 * and changed if, and only if, necessary.
 * This is quicker that resending data from the controller each second
 * and the Arduino having to read and interpret the results.
 ****************************************************************************/
void doSerialOutput() {
  if (isArtisan) {
    return;
  }

  //send data to logger
 // float tt0;
  float t1;
  float t2;

//  tt0 = t0;
  t1 = TC1AvgTemp;
  t2 = TC2AvgTemp;

 // Serial.print("t0="); // Ambient always 0
 // Serial.println(tt0,DP);
  
  Serial.print("Temp 1: "); // Environment Temp
  Serial.println(t1,DP);
  
  //Comment out the next two lines if using only one thermocouple
  Serial.print("Temp 2: ");
  Serial.println(t2,DP);

 /*
  Serial.print(PWM1Name);
  Serial.print(": ");
  Serial.print(power1);
  Serial.println("%");
  
  Serial.print(PWM2Name);
  Serial.print(": ");
  Serial.print(power2);
  Serial.println("%");
  
  Serial.print(PWM3Name);
  Serial.print(": ");
  Serial.print(power3);
  Serial.println("%");

  Serial.print(PWM4Name);
  Serial.print(": ");
  Serial.print(power4);
  Serial.println("%");
  */

  // only need to send these if Arduino controlling by potentiometer
  if (controlBy == arduino)
  {
    Serial.print("pots1=");
    Serial.println(Pot1Val);
    Serial.print("pots2=");
    Serial.println(Pot2Val);
   // Serial.print("lm35=");
   // Serial.println(lmTemp);
  }
}

/****************************************************************************
 * Read temperatures from Max31855 chips Sets t1 and t2, -1 if an error
 * occurred.  Max31855 needs 240 ms between readings or will return last
 * value again. I am reading it once per second.
 ****************************************************************************/
void getTemperatures()
{
  TC1Temp = readThermocouple(TC1CS, TC1Calibrate); 
  TC2Temp = readThermocouple(TC2CS, TC2Calibrate); 
  
   if (TC1Temp > 0.0) 
   {
    TC1Cumulative = TC1Cumulative + TC1Temp;
    TC1BadReading ++;
   }
   if (TC2Temp > 0.0) 
   {
    TC2Cumulative = TC2Cumulative + TC2Temp;
    TC2BadReading ++;
   }
}

/****************************************************************************
* Purpose to adjust power based on the value from the potentiometer
****************************************************************************/
void updatePot1(){
  Pot1Val = analogRead(Pot1Pin);
  
  Pot1LastReadVal = Pot1Val;
  Pot1SmoothedVal = (Pot1Alpha) * Pot1SmoothedVal + (1-Pot1Alpha) * Pot1LastReadVal;
  Pot1Val = Pot1SmoothedVal;

  int pow = map(Pot1Val, 15, Pot1Max, 25, 100);
  setPowerLevel1(pow);
}

void updatePot2(){
  Pot2Val = analogRead(Pot2Pin);
  
  Pot2LastReadVal = Pot2Val;
  Pot2SmoothedVal = (Pot2Alpha) * Pot2SmoothedVal + (1-Pot2Alpha) * Pot2LastReadVal;
  Pot2Val = Pot2SmoothedVal;

  int pow = map(Pot2Val, 15, Pot2Max, 25, 100);
  setPowerLevel2(pow);
}

/****************************************************************************
* Purpose to gauge the internal temperature of the box
****************************************************************************/
//void updateLM35() {
//  analogRead(lmPin); delay(40);
//  int reading = analogRead(lmPin);
// lmTemp = reading * 5.0*100.0 / 1023.0;
//}

/****************************************************************************
 * Update the values on the LCD
 ****************************************************************************/
void outputLCD() {
  int col2 = 2;
  int col3 = 10;

  if (controlBy == arduino) {
    //lcd.setCursor(0,0); lcd.write(byte(0));
    lcd.setCursor(0,0); lcd.printByte(0);
    lcd.setCursor(0,1); lcd.print(" ");
  }
  else {
    lcd.setCursor(0,0); lcd.print(" ");
    //lcd.setCursor(0,1); lcd.write(byte(1));
    lcd.setCursor(0,1); lcd.printByte(1);
  }

  lcd.setCursor(col2,0);
  lcd.print(TC1AvgTemp,DP);
  lcd.write(B11011111);

  lcd.setCursor(col3,0);
  lcd.print(TC1AvgTemp,DP);
  lcd.write(B11011111);
  
  lcd.setCursor(col2,1); // row 2
  char powChar[3];
//  sprintf(powChar, "%3d", power1 * digitalRead(RelayCtrlPin1));
  lcd.print(powChar);
  lcd.print("%");

 // lcd.setCursor(col3,1);
 // lcd.print(lmTemp, DP);
//  lcd.write(B11011111);
}

/****************************************************************************
 * Called by main loop once every XXX ms
 * Used to read each thermocouple once every XXX ms
 *
 * Once per second averages temperature results, updates potentiometer and outputs data
 * to serial port.
 ****************************************************************************/
void doSubLoop()
{
  getTemperatures();

  if (tcLoopCount > 3)  // once every four loops (1 second) calculate average temp, update Pot and do serial output
  {
    tcLoopCount = 0;

    if (TC1BadReading > 0)  TC1AvgTemp = TC1Cumulative / TC1BadReading; else TC1AvgTemp = -1.0;
    if (TC2BadReading > 0)  TC2AvgTemp = TC2Cumulative / TC2BadReading; else TC2AvgTemp = -1.0;
    TC1BadReading = 0;
    TC2BadReading = 0;
    TC1Cumulative = 0.0;
    TC2Cumulative = 0.0;

    // updateLM35();
    doSerialOutput(); // once per second
  }
  tcLoopCount++;

  // only use Pot if in Arduino control.  If Computer control power is set by computer
  //if (controlBy == arduino){
  //  updatePot1();
  //  updatePot2();
 // }

  // outputLCD();
}


/*****************************************************************
 * Read the Max31855 device 1 or 2.  Returns temp as float or  -1.0
 * if an error reading device.
 * Note at least 240 ms should elapse between readings of a device
 * to allow it to settle to new reading.  If not the last reading 
 * will be returned again.
 *****************************************************************/
float readThermocouple(int CS, float calibrate) //device selected by passing in the relavant CS (chip select)
{
  
  int value = 0;
  int error_tc = 0;
  float temp = 0.0;
  
  digitalWrite(CS,LOW); // Enable device

  // wait for it to settle
  delayMicroseconds(1);
  
  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCKa,HIGH);
  digitalWrite(SCKa,LOW);
  //wait for it to settle
  delayMicroseconds(1);
  
  /* Read each MAX31855 for the Temp 
   storing the value, depending on F or C
   */
  digitalWrite(SCKa,HIGH);  // Set Clock to HIGH
  if(CS == TC1CS) {
    #ifdef CELSIUS // If Celcius is defined then get Celcius for Thermoocouple 1
     value = thermocouple1.readCelsius();
     #else
     value = thermocouple1.readFarenheit();
    #endif  
  }
  if(CS == TC2CS) {
    #ifdef CELSIUS // If Celcius is defined then get Celcius for Thermoocouple 2
     value = thermocouple2.readCelsius();
     #else
     value = thermocouple2.readFarenheit();
    #endif  
  }
  digitalWrite(SCKa,LOW);  // Set Clock to LOW
  
  // Read the TC input to check for error
  //digitalWrite(SCKa,HIGH); // Set Clock to HIGH
 // error_tc = digitalRead(SO); // Read data
 // digitalWrite(SCKa,LOW);  // Set Clock to LOW

 // digitalWrite(CS,HIGH); // Disable device 1

  value = value + calibrate;          // Add the calibration value
  temp = (value*0.25);            // Multiply the value by 0.25 to get temp in ˚C

  // return -1 if an error occurred, otherwise return temp
 // if(error_tc == 0) {
    return temp; 
 // } 
 // else { 
 //   return -1.0; 
 // }
}
