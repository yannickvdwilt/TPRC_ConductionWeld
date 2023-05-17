// include toolboxes
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <MobaTools.h>
#include <EEPROM.h>
#include "Adafruit_MAX31855.h"
#include <movingAvg.h>

// -----------------------------------  functions in this program ----------------------
void readTemp(int startIndices, int endIndices);  // reads the temperatures on certain thermocouples
void readAllTemps();                              // reads the temperatures on all thermocouples
void overTemp(int i);                             // Activated if temperatures are exceeded, shuts down everything
void PIDCalc();                                   // calculate the PID values for the two heaters
void logTempCSV(int plot, bool plotPID);          // print the temperature and PID data to the terminal in a CSV format
void dispTemp();                                  // display the temperatures on an attached I2C LCD
void runMotor();                                  // Check if motor should be moving
void moveMotor(int speed, int dist);              // move motor certain distance with certain speed, also checks if max distance is not exceeded
void controlCooler();                             // determine whether the cooler needs to be activated
void readButtons();                               // used to read the external buttons and do some logic
void setPointTemp();                              // function used to make a temperature profile, not used at the moment
void writeIntIntoEEPROM(int address, int number); // Write int into EEPROM instead of signed 8-bits
int readIntFromEEPROM(int address);               // read int form EEPROM
String n2s(int i, int base);                      // turns number into string with leading zeros, with base indicating the amount of total numbers
void homeStepper(bool messages);                  // homes stepper, this simple code blocks
void setTemps(int polNum, bool manual);           // set the temperature to "Setpoint"
void setEncVals();                                // read the encoder/selectVal's from the EEPROM
void thermocoupleInitialise(bool printSerial);    // initialise and check all the thermocouples
void encButSwitch();                              // do everything with the encoder button selection
void runProgram();                                // called to run the automatic program
int runAverage(int n, int val);                   // Run average, in function because in array would not work :(
void repeatrun();                                 // custom program for doing multipe tests on one sample


/*        Todo
    re-implement temperature curves
    implement different FF for moving
    during movement between sequential welds, make the program not block
    Add something to make backwards and forwards movement easyer (maybe an external switch)
*/

// ------------------------------------- Ajustable values ---------------------------------
 
int noRuns = 1;       // Amount of welds the setup should preform sequentialy. This should only be done for spot-welds (set movement to 0 via control panel). Set to 0 or 1 for no repeats
#define N_POLY 6                                                                   // amount of polymers, needs to be ajusted when "polymers" string is ajusted
const String polymers[N_POLY] = {"LMPAEK", "LMLine", "PEKK  ", "PEEK  ", "PEI   ", "PPS   "}; // (initial) temperature setpoints in degrees celsius (hot top, hot bot, cooler top, cooler bottom)
float polTemps[N_POLY][4] =     {{370, 370, 200, 200},                            // Th1 Th2 Tc1 Tc2
                                  {400, 400, 200, 200},
                                   {395, 395, 250, 250},
                                   {375, 375, 250, 250},
                                   {395, 395, 250, 250},
                                   {325, 325, 220, 220}};
float Setpoint[4] = {0, 0, 0, 0};                 // Array for current setpoints
unsigned long setPointRefRate = 1000;             // how often the temperature setpoint gets refreshed
const int tempSet[6] = {25, 400, 401, 180, 0, 1}; // temperature setpoints in degrees celsius
const float dT[5] = {1.6, 0.1, 0.6, 1.6, 0};      // temperature change in deg/sec

const float kp[4] = {4, 5, 10, 8};
const float ki[4] = {0.1, 0.1, 0.5, 0.5};
const float kd[4] = {3, 3, 10, 10};

//const float ff[2][4] = {{0.004, 0.0012, 0.2, 0.44},  // Open jaw ff values, 
//                        {0.0012, 0.0021, 0.5, 0.5}}; // Closed jaw feed forward variables, 0.0021 was 0.0018


const float ff[2][6] = {{0.0004, 0.0014, 0.6, 0.55, 0.2, 0.44},  // Open jaw ff values, hTop, hBot, offsetTop, offsetBot, difTop, difBot 
                        {0.0012, 0.0021, 0.75, 0.65, 0.48, 0.5}}; // Closed jaw feed forward variables, 0.0021 was 0.0018

const float Imax[4] = {8, 8, 20, 20}; // maximum I value, to prevent I overflow
const int Irange[4] = {9, 9, 10, 10}; // range within which the I value is activated
#define MAX_TEMP 470                  // maximum temperature, after which the machine will be blocked
const int bangBangMargin[4] = {0, 0, 1, 1};
const int boostMargin[4] = {0, 0, 5, 5};

unsigned long slowRefRate = 1000 / 3; // refresh rate for display etc (here useless as reading the temps takes very long
unsigned long fastRefRate = 100;      // refresh rate for the faster processes
const int overTempTime = 1000;        // how long an overtemperature is allowed before triggering alarm. Needed for false readings

// ------------------------------------- Pins and constants -------------------------------

#define ENC_DATA 3
#define ENC_CLOCK 2
#define ENC_BUT 35
#define SDA 20
#define SCL 21
#define CLK 23
#define DO 22
#define STEPPERHOMEPIN 13 //
const byte ENA = 37;
const byte stepPin = 38;
const byte dirPin = 39;
#define START_PROGRAM 43   // Hot down
#define MANUAL_COOLDOWN 41 // Hot select
#define PISTON_DOWN 42     // Hot up
#define HOT_ENABLE 44      // Hot enable
#define AUTOSWITCH 36      // INPUT, choose automatic or manual
#define SECOND_PISTON_DOWN 12
#define ENDSTOP 13
const int CS[10] = {24, 25, 26, 27, 28, 33, 32, 31, 30, 29}; // Oeps
const int PN[3] = {45, 46, 47};                              // Pneumatic output pins
const int LEDPINS[4] = {50, 51, 52, 53};                     // LED output pins
const int PWMOUT[4] = {8, 9, 10, 11};                        // all PWM-pins, should be: HotTop, HotBottom, BoostTop, BoostBottom
#define MAX_MOTOR_SPEED 3000                                 // 200 steps per rotation, this is in steps/min,
#define STEPTOMM 200                                         // amount of stepper motor steps per mm traveled. Steps/rev = 200 , microstepping = 1, mm/rev = 0.75 (pitch 1.5)
#define stepsPerRev  200                                     // Steps per revolution - may need to be adjusted
#define STAGE_LENGTH 150                                     // length of the movement stage in mm
#define D_FILTER_LENGTH 5                                    // running average filter length for derivative filtering
const String offOn[2] = {"OFF", " ON"};
const String upDown[2] = {"  UP", "DOWN"};
const String stopStart[2] = {" STOP", "START"};
const String prgmMove[2] = {" Prgm:", " Move:"};              // display either "prgm: " or "move:"
const String prgStep[6] = {" STOP", "PRE-H", " HEAT", " HOLD", " MOVE", " COOL"}; // all the phases of the welding process


// -----------------------------  Declarations and initialisations -------------------------------

double temp[10]; // array for the temperatures
float error[4], prevError[4], PID_p[4], PID_i[4], PID_d[4], PID_FF[4], elapsedTime;
int PID_value[4] = {0, 0, 0, 0};
bool PNon[3] = {0, 0, 0}; // boolian whether pneumatics should be on from PID

movingAvg dAvg0(D_FILTER_LENGTH);
movingAvg dAvg1(D_FILTER_LENGTH);
movingAvg dAvg2(D_FILTER_LENGTH);
movingAvg dAvg3(D_FILTER_LENGTH);

unsigned long lastUpd = 0;
unsigned long lastSlowTime = 0;
unsigned long lastFastTime = 0;
unsigned long lastTempUpdateTime = 0; // timer for the temperature setpoint
unsigned long lastPID = 0;

int tempPhase = -1; // vaiable for the temperature profile
int heatDir = 1;    // tells arduino whether it should cool or heat, used for the temperature profile

bool motorMoveEnable = 0;
int motorSpeed = 0; // to transmit data about the motor speed between functions
bool max31855Initialised = 0;
bool heatEnable = 0;         // HIGH wHOT_ENABLEn heater is active
bool manualMode = 1;         // whether the system is in manual (1) or automatic (0) mode
int lastMillisDigit = 0;     // for saving the motor speed in EEPROM
int lastSpeedSaved = 0;      // last motorspeed saved
bool pistonDown = 0;         // Determines whether piston shoule be down (0 is up, 1 is down). Can be done manualy via buttons or during program
bool startProgram = 0;       // Whether the program is started (1) or not (0). 
#define MOTORSPEEDEEPROM 0 // address for motor speed EEprom
unsigned long encButPressed = 0;
unsigned long pistButPressed = 0;
unsigned long startButPressed = 0;
bool encButReleased = 0;
bool pistButReleased = 0;
bool startButReleased = 0;
int selectMode = 4; // polymer, weldTime, weldLength, weldSpd, Nothing
int selectVal[5] = {0, 0, 0, 0, 0};
int selectMax[5] = {4, 999, STAGE_LENGTH, MAX_MOTOR_SPEED, 0};
int selectMin[5] = {0, 0, -STAGE_LENGTH, 0, 0};
bool encChangeAllowed = 1;
int lastSelectMode = 4;          // last selected mode, used to determine modeswitch
bool dispBlink = 0;              // bool to indicate whether the display selection indicator should blink
unsigned long startOvertemp = 0; // when the overtemp condition was started
bool wasOvertemp = 1;            // bool to see if overtemp state has changed
bool manualCoolOn = 0;           // Cooling on (1) or off (0)
bool coolButReleased = 0;
unsigned long coolButPressed = 0;
bool progStopped  =0;           // flag that indicates the "runmultiple" that the run has been stopped, so it does not finish its other run after one is stopped

bool motorStarted = 0;
bool programStarted = 0; // flag that actualy determines whether program is started/still running
int programMode = 0;     // mode to switch between program steps, preheat, heat, hold, cooldown, release
int lastProgramMode = 0;
int secToGo = 0; // amount of seconds left in the program
unsigned long endHeating = 0;

unsigned long lastFFTime = 0; // timer for feedfoward values test
int setPWM = 0;               // PWM values for feedforward test

bool startRepeat = 0; // flag to check whether to start the repeating program, will only start if called upon and program is started
int amountRepeat = 0; // amount of repeat runs that have been run
int amountToMove = 0; // amount of distance between each sequential weld, gets calculated at the start of the sequential welds to cover the whole stage range

LiquidCrystal_I2C lcd1(0x27, 20, 4); // set the LCD address to 0x27
LiquidCrystal_I2C lcd2(0x26, 20, 4); // set the LCD address to 0x26

MoToStepper stepper1(stepsPerRev, STEPDIR); // create a stepper instance stepper1.rotate(1);        // start turning, 1=vorward, -1=backwards
Encoder myEnc(ENC_DATA, ENC_CLOCK);         // myEnc.read();
Adafruit_MAX31855 thermocouples[10] =
    {
        Adafruit_MAX31855(CLK, CS[0], DO),
        Adafruit_MAX31855(CLK, CS[1], DO),
        Adafruit_MAX31855(CLK, CS[2], DO),
        Adafruit_MAX31855(CLK, CS[3], DO),
        Adafruit_MAX31855(CLK, CS[4], DO),
        Adafruit_MAX31855(CLK, CS[5], DO),
        Adafruit_MAX31855(CLK, CS[6], DO),
        Adafruit_MAX31855(CLK, CS[7], DO),
        Adafruit_MAX31855(CLK, CS[8], DO),
        Adafruit_MAX31855(CLK, CS[9], DO),
};

void setup() // Setup procedure
/*
To do
- check if electricity is on if not error
*/

{
  digitalWrite(ENA, HIGH); // set stepper enable to HIGH (off)
  digitalWrite(stepPin, LOW);
  pinMode(AUTOSWITCH, INPUT_PULLUP);
  pinMode(HOT_ENABLE, INPUT_PULLUP);
  pinMode(START_PROGRAM, INPUT_PULLUP);
  pinMode(MANUAL_COOLDOWN, INPUT_PULLUP);
  pinMode(PISTON_DOWN, INPUT_PULLUP);
  pinMode(ENC_BUT, INPUT_PULLUP);
  pinMode(SECOND_PISTON_DOWN, INPUT_PULLUP);
  pinMode(ENDSTOP, INPUT_PULLUP);
  lcd1.init();
  lcd2.init();
  lcd1.backlight();
  lcd2.backlight();
  Serial.begin(115200);
  setEncVals();
  setTemps(selectVal[0], digitalRead(AUTOSWITCH));
  thermocoupleInitialise(0);

  for (int i = 0; i < 3; i++)
  {
    pinMode(PN[i], OUTPUT);
  }
  for (int i = 0; i < 4; i++)
  {
    pinMode(LEDPINS[i], OUTPUT);
  }
  for (int i = 0; i < 4; i++)
  {
    pinMode(PWMOUT[i], OUTPUT);
  }

  pinMode(ENC_BUT, INPUT_PULLUP);
  pinMode(PISTON_DOWN, INPUT_PULLUP);
  pinMode(START_PROGRAM, INPUT_PULLUP);
  myEnc.write(readIntFromEEPROM(MOTORSPEEDEEPROM));
  dAvg0.begin();
  dAvg1.begin();
  dAvg2.begin();
  dAvg3.begin();
  stepper1.attach(stepPin, dirPin);
  stepper1.attachEnable(ENA, 200, 0);
  stepper1.setRampLen(stepsPerRev / 8); // Ramp length is 1/2 revolution
  homeStepper(0);
  //stepper1.setSpeed(3000); // 3000 rev/min (if stepsPerRev is set correctly)
}

// -------------------------------------------------        Main            ----------------------------------------------------------------
//  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// ----------------------------------------------------------------------------------------------------------------------------------------

void loop() // Main loop function, loops continuously
{
  readButtons();                                              // do this as often as possible, would have been nice to use interrupts...
  if ((unsigned long)(millis() - lastFastTime) > fastRefRate) // for relatively frequent processes
  {
    lastFastTime = millis();
    readAllTemps();
    PIDCalc();
  }

  if ((unsigned long)(millis() - lastSlowTime) > slowRefRate) // for relatively infrequent processes	  
  {
    lastSlowTime = millis();
    encButSwitch();
    dispTemp();
    logTempCSV(0, 0);
    controlCooler();
    
    if (!manualMode || programStarted) // if running automatic, or program is still running
    {
      runProgram();
      repeatrun();    // uncomment/remove this to run some custom programming variant
    }
    else
    {
      runMotor();
    }
  }
}

// -----------------------------------    Functions ------------------------------
// in loop order
// ----------------------------------- Fast refresh functions

void readButtons()
  /* To do:
  - Alphabetical order

  // Heat button.
  - Show for several seconds on LCD "Heat on!" / "Heat off" as warning
  - ? Program does not start when heat is not pressed.
  
  // Auto or manual button
  - Show for several seconds on LCD "Automatic" / "Manual"
  
  // Cooling down button
    - Show during use "Cooling down" 
    - By pressing the cooling down process is started / heating (up) is not possible.
  
  // Start program button
    - Show for several seconds on LCD "Start Program"
  
  // On off button
    - make it working also when attached to the laptop
  */
{
  // Auto or manual button
  if (digitalRead(AUTOSWITCH) != manualMode) // if changed from manual to automatic, or reverse. If done during program, program/move should stop and all startflags should reset
  {                                          // pressed = 0, so  1 = manual, 0 = automatic
    manualMode = digitalRead(AUTOSWITCH);    
    selectMode = 4;                          // set selectMode to the default off state
    startProgram = 0;
    motorStarted = 0;
    stepper1.stop();
    encChangeAllowed = 1; // enable changing the values via encoder
    programStarted = 0;   // program has stopped
    programMode = 0;      // set to default, where nothing will happen

    for (int i = 0; i < 2; i++)
    {
      digitalWrite(PN[i], 0); // when changing, turn off cooler pneumatics
    }
    setTemps(selectVal[0], manualMode);
  }

  // Cooling down button
  if (!digitalRead(MANUAL_COOLDOWN)) // if any of the two piston buttons has been pressed, but the program/motor has not started
  {
    if ((millis() - coolButPressed > 200) && coolButReleased)
    {
      coolButReleased = 0;
      coolButPressed = millis();
      manualCoolOn = !manualCoolOn;   // in controlcooler --> cooler activated.
    }
  }
  else
  {
    coolButReleased = 1;
  }

  // Heat button.
  if (digitalRead(HOT_ENABLE) && heatEnable)        // if heat is turned off but was on
  { 
    heatEnable = 0;
    // Serial.println("Hot off");
    for (int i = 0; i < 4; i++)
    { // turn all heaters off to be sure
      digitalWrite(PWMOUT[i], LOW);
    }
  }
  else if (!digitalRead(HOT_ENABLE) && !heatEnable) // if heat is turned on but was off (also activates on startup)
  { 
    heatEnable = 1;
    // Serial.println("Hot on"); Do this for a few seconds.
    for (int i = 0; i < 4; i++)
    {               // turn all heaters off to be sure
      PID_i[i] = 0; // set all I values to 0 to remove the possible buildup
    }
  }

  // Do not know which button
  if (!digitalRead(ENC_BUT))
  {
    if ((millis() - encButPressed > 50) && encButReleased)
    {
      encButReleased = 0;
      encButPressed = millis();
      selectMode += 1;
      if (selectMode > 4)
      {
        selectMode = 0;
      }
    }
  }
  else
  {
    encButReleased = 1;
  }
  
  // Do not know which button
  if ((!digitalRead(PISTON_DOWN) || !digitalRead(SECOND_PISTON_DOWN)) && !programStarted && !stepper1.moving()) // if any of the two piston buttons has been pressed, but the program/motor has not started
  {
    if ((millis() - pistButPressed > 200) && pistButReleased)
    {
      pistButReleased = 0;
      pistButPressed = millis();
      pistonDown = !pistonDown;
      digitalWrite(PN[2], pistonDown);
    }
  }
  else
  {
    pistButReleased = 1;
  }

  // Start program button
  if (!digitalRead(START_PROGRAM))
  {
    if ((millis() - startButPressed > 200) && startButReleased)
    {
      startButReleased = 0;
      startButPressed = millis();
      startProgram = !startProgram;
    }
  }
  else
  {
    startButReleased = 1;
  }
}

void readAllTemps()
{
  readTemp(0, 10);
}

void readTemp(int startIndices, int endIndices)
{
  int overTempNo = 0;
  for (int i = startIndices; i < endIndices; i++)
  {
    double tempTemp = thermocouples[i].readCelsius();
    if (isnan(tempTemp))
    {
      tempTemp = 0;
    }
    temp[i] = tempTemp;

    if (temp[i] > MAX_TEMP)
    { // if temp is too high, raise overtemp flag
      overTempNo = i;
    }
  }

  // code for overtemperature check
  if (overTempNo > 0)
  {
    if (wasOvertemp == 0)
    {
      startOvertemp = millis();
      wasOvertemp = 1;
    }
    if ((millis() - startOvertemp) > overTempTime) // if x time has passed during which there was an overtemperature, activate overtemperature
    {
      overTemp(overTempNo);
    }
  }
  else
  {
    wasOvertemp = 0;
  }
}

void overTemp(int i)
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(PWMOUT[i], LOW);
  }
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PN[i], LOW);
  }
  stepper1.stop();
  lcd1.clear();
  lcd2.clear();
  lcd1.setCursor(0, 0);
  lcd1.print("Overtemp ");
  lcd1.setCursor(9, 0);
  lcd1.print(i);
  Serial.print("Overtemp on temperature probe");
  Serial.print(i);
  while (1)
    ;
}

void PIDCalc()
{
  elapsedTime = (float)(millis() - lastPID) / 1000;
  lastPID = millis();
  for (int i = 0; i < 4; i++)
  {
    // we calculate the error between the setpoint and the real value
    error[i] = Setpoint[i] - temp[i];
    // Calculate the P value
    PID_p[i] = kp[i] * error[i];
    // Calculate the I value in a range on +-10
    PID_i[i] = PID_i[i] + error[i] * ki[i];
    if (PID_i[i] > Imax[i])
    { // prevemt I buildup
      PID_i[i] = Imax[i];
    }
    if (PID_i[i] < -Imax[i])
    {
      PID_i[i] = -Imax[i];
    }
    if ((error[i] < Irange[i]) && (error[i] > -Irange[i]))
    { // only add to I within margin, to prevent I buildup and overshoot
      PID_i[i] = PID_i[i] + error[i] * ki[i];
    }

    if (PID_i[i] > Imax[i])
    { // prevemt I buildup, why dubble?
      PID_i[i] = Imax[i];
    }
    else if (PID_i[i] < -Imax[i])
    {
      PID_i[i] = -Imax[i];
    }

    PID_d[i] = float(runAverage(i, 1000 * kd[i] * ((error[i] - prevError[i]) / elapsedTime))) / 1000; // take running average for D to smooth the value. Is multiplied by 1000 so the runAverage can use int's instead of Floats

    if (i < 2)
    {                                          // for the heater PID calcs
      long dT = Setpoint[i] - Setpoint[i + 2]; // Because the heater and cooler system are linked, heater input power is dependant on the cooler temperature
      PID_FF[i] = ff[pistonDown][i] * pow(dT, 2);
    }
    else
    { // fot the cooler PID calcs
      long dT = Setpoint[i - 2] - Setpoint[i];
      //PID_FF[i] = 115 - dT * ff[pistonDown][i];
      PID_FF[i] = Setpoint[i] * ff[pistonDown][i] -  dT * ff[pistonDown][i+2];
    }
    // Final total PID value is the sum of P + I + D and the feedforward
    PID_value[i] = PID_p[i] + PID_i[i] + PID_d[i] + PID_FF[i]; // Add feedforward value
    // We define PWM range between 0 and 255
    if (PID_value[i] < 0)
    {
      PID_value[i] = 0;
    }
    else if (PID_value[i] > 255)
    {
      PID_value[i] = 255;
    }
    prevError[i] = error[i]; // store error for d calculations
    if (heatEnable)
    {
      analogWrite(PWMOUT[i], PID_value[i]);
    }
    else
    {
      analogWrite(PWMOUT[i], 0);
    }
  }
}

int runAverage(int n, int val)
{
  int avg = 0;
  switch (n)
  {
  case 0:
    avg = dAvg0.reading(val);
    break;
  case 1:
    avg = dAvg1.reading(val);
    break;
  case 2:
    avg = dAvg2.reading(val);
    break;
  case 3:
    avg = dAvg3.reading(val);
    break;

  default:
    Serial.println("runAverage, wrong input n");
    break;
  }
  return avg;
}

// ----------------------------------- Slow refresh functions

void encButSwitch()  // Selection with the encoder button
{
  if (encChangeAllowed) // only run this code if no program is running
  {
    if (selectMode != lastSelectMode) // upon change:
    { 
      writeIntIntoEEPROM(lastSelectMode * 8, selectVal[lastSelectMode]);
      if (selectMode == 0) // for the polymers, do something else
      {
        myEnc.write(selectVal[selectMode] * 10 + 5);
      }
      else
      {
        myEnc.write(selectVal[selectMode]);
      }
      lastSelectMode = selectMode;
    }
    int encVal = myEnc.read(); // read encoder
    if (selectMode == 0)       // for the polymer, do something different
    {
      int newPolVal = abs(encVal / 10) % N_POLY;
      if (selectVal[0] != newPolVal) // if there is a change in polymer
      {
        selectVal[0] = newPolVal;
        setTemps(selectVal[0], manualMode);
      }
    }
    else
    {
      if (selectMode == 2 && abs(encVal) <= 2)
      { // if the amount of movement is selected and the value is smaller than 3, say it's 0. This is because 0 is often wanted and the encoder is a hassle
        encVal = 0;
        myEnc.write(encVal);
      }

      if (encVal > selectMax[selectMode])
      {
        encVal = selectMax[selectMode];
        myEnc.write(encVal);
      }
      else if (encVal < selectMin[selectMode])
      {
        encVal = selectMin[selectMode];
        myEnc.write(encVal);
      }
      selectVal[selectMode] = encVal; // write value
    }
  }
}

void setTemps(int polNum, bool manual)
{
  for (int i = 0; i < 4; i++) // set all the setpoint temperatures from the database into "Setpoint"
  {
    if (manual)
    {
      Setpoint[i] = polTemps[polNum][i]; // If manual, set all values to their normal desired temperature
    }
    else
    {
      Setpoint[i] = polTemps[polNum][2]; // If automatic, set all values to the colder setpoint, waiting for the process to start
    }
  }
}

void dispTemp() // int selectMode = 0; // polymer, weldTime, weldLength, weldSpd
{
  String weldTime = n2s(selectVal[1], 3);
  String stepsTodo = n2s(selectVal[2], 4);
  if (programStarted)
  { // if the program is started, do not display the total set time, but the time left during welding
    weldTime = n2s(secToGo, 3);
  }
  if (stepper1.moving())
  { // if the stepper is running, do not display the total movement, but the movement to go
    stepsTodo = n2s(stepper1.stepsToDo() / STEPTOMM, 4);
  }

  String selString[4] = {polymers[selectVal[0]], weldTime, stepsTodo, n2s(selectVal[3], 3)}; // makes a string with all the ajustable variables
  String selStringVoids[4] = {"      ", "   ", "    ", "   "};                               // string to replace variable that needs to blink
  for (int i = 0; i < 4; i++)
  {
    if (i == selectMode && dispBlink) // if that variable is the variable that can be changed, and its time to blink
    {
      selString[i] = selStringVoids[i]; // set variable that can change to blink
    }
  }
  dispBlink = !dispBlink;

  String s00 = "H1:";
  s00 += n2s(temp[0], 3) + "/" + n2s(int(Setpoint[0]), 3) + " 2:" + n2s(temp[1], 3) + "/" + n2s(Setpoint[1], 3);
  lcd1.setCursor(0, 0);
  lcd1.print(s00);

  String s01 = "C1:";
  s01 += n2s(temp[2], 3) + "/" + n2s(Setpoint[2], 3) + " 2:" + n2s(temp[3], 3) + "/" + n2s(Setpoint[3], 3);
  lcd1.setCursor(0, 2); // set to 2
  lcd1.print(s01);

  String s02 = "S1:";
  s02 += n2s(temp[4], 3) + "     2:" + n2s(temp[5], 3) + "    ";
  lcd1.setCursor(0, 1); // set to 1
  lcd1.print(s02);

  String s03 = "Mat:";
  s03 += selString[0] + " Heat: " + offOn[heatEnable];
  lcd1.setCursor(0, 3);
  lcd1.print(s03);
  // cursor 2 temperatures
  String s10 = "R:" + n2s(noRuns,1) + " "; 
  s10 += n2s(temp[6], 3) + " " + n2s(temp[7], 3) + " " + n2s(temp[8], 3) + " " + n2s(temp[9], 3);
  lcd2.setCursor(0, 0);
  lcd2.print(s10);

  String s11 = "Weld:" + selString[1] + "s" + " Clamp:" + upDown[pistonDown];
  lcd2.setCursor(0, 2); // set to 2
  lcd2.print(s11);

  String progAction = stopStart[startProgram];
  if (!manualMode)
  {
    progAction = prgStep[programMode];
  }
  String s12 = "Auto: ";
  s12 += offOn[!manualMode] + prgmMove[manualMode] + progAction;
  lcd2.setCursor(0, 1); // set to 1
  lcd2.print(s12);

  String s13 = "Weld:" + selString[2] + "mm " + selString[3] + "mm/mn";
  lcd2.setCursor(0, 3);
  lcd2.print(s13);
}

void logTempCSV(int plot, bool logPID) // Prints temperatures and setpoints in the serial monitor.
{
  /*
    To do: 
    int plot: whether this is used in the arduino IDE to plot the temperatures,
    Temperature, setpoint, P, I, D, ff, PID
    hot 1 , hot 2, cold 1, cold 2
  */
  if (!plot)
  {
    Serial.print(millis());
    lastUpd = millis();
    Serial.print(",     ");
  }
  for (int i = 0; i < 10; i++)
  {
    Serial.print(temp[i], 1);
    Serial.print(",");
  }
  Serial.print(" ");
  if (logPID && !plot)
  {
    Serial.println();
  }
  for (int i = 0; i < 4; i++)
  {
    Serial.print(Setpoint[i], 1);
    Serial.print(", ");
    Serial.print(temp[i], 1);
    Serial.print(", ");
    if (!plot)
      if (logPID)
      {
        Serial.print(PID_p[i]);
        Serial.print(", ");
        Serial.print(PID_i[i]);
        Serial.print(", ");
        Serial.print(PID_d[i]);
        Serial.print(", ");
        Serial.print(PID_FF[i]);
        Serial.print(", ");
      }
    Serial.print(PID_value[i]);
    Serial.print(",    ");
  }
  Serial.println();
}

void controlCooler() // Determines if the cooling is activated or not
{ // controls whether the cooling elements need to be activated
  for (int i = 2; i < 4; i++)
  {
    if (temp[i] > Setpoint[i] + 2)
    {
      PNon[i - 2] = 1;
    }
    else if (temp[i] < Setpoint[i] + 2)
    {
      PNon[i - 2] = 0;
    }
  }

  digitalWrite(PN[0], (manualCoolOn || PNon[0])); // if the manual button is pressed, or the system wants to cool down
  digitalWrite(PN[1], (manualCoolOn || PNon[1]));
}

void runProgram() // Runs the weld process
/*
To do:
- Add in serial monitor the state of the process with numbers or with "Phase 1: heating up"
*/
{
  if (startProgram && !programStarted) // if the program start button is pressed and no program is running
  {
    programMode = 1;        // Set to starting program mode
    selectMode = 4;         // set the selectMode for the encoder to the default "off" position
    encChangeAllowed = 0;   // disable changing the values via encoder
    programStarted = 1;     // program has started, no other program can be started.
    secToGo = selectVal[1]; // set seconds to go for weld
    motorStarted = 0;
    digitalWrite(PN[2], HIGH);
    pistonDown = 1;
    // Serial.println("Program started");
    // change for git
  }
  else if (!startProgram && programStarted)  // if the program was started, but should stop
  {                       // if the program is running, but should stop
    programMode = 0;      // set to default, where nothing will happen
    encChangeAllowed = 1; // enable changing the values via encoder
    programStarted = 0;   // program has stopped
    selectMode = 0;
    stepper1.stop();
    motorStarted = 0;
    setTemps(selectVal[0], 0); // re-set cooler temps, else the temps will stay at the hot temps. This may not be an advantage
    noRuns = 0;       // Set the amount of repeat runs to 0, else if there were any left in memory, it would do them now
    // Serial.println("Program Manualy stopped");
  }

  switch (programMode)
  {
  case 1:                      // wait for preheat
    if (checkSetTemp(5, 4, 0)) // if temp is high enough
    {
      programMode = 2;
      // Serial.println("Preheat finished, entering heating phase");
      delay(1000);               // wait for jaw to close
      setTemps(selectVal[0], 1); // set temperatures to hot temperature
    }
    break;

  case 2:                      // Heating phase
    if (checkSetTemp(5, 2, 0)) // if the heater has reached the setpoint
    {
      programMode = 3;
      endHeating = millis() + long(selectVal[1]) * 1000;
      // Serial.println("Heating finished, entering holding phase");
    }
    break;

  case 3:                                     // Holding phase
    secToGo = (endHeating - millis()) / 1000; // re-calculate the seconds to go in the weld
    if (secToGo < 0)
    { // if all the time has passed
      secToGo = 0;
      programMode = 4;
      // Serial.println("Holding finished, entering cooling phase");
    }
    break;

  case 4: // Movement phase.
    if (!motorStarted)
    { // if motor not yes started, do this
      motorStarted = 1;
      moveMotor(selectVal[3], selectVal[2]);
    }
    else
    {
      if (!stepper1.moving())
      { // if motor is started, but has also stopped, go to next mode
        programMode = 5;
        motorStarted = 0;
        setTemps(selectVal[0], 0); // set temperatures to cooling temps
        Setpoint[2] = 0;
        Setpoint[3] = 0; // set cooler setpoints to 0 for max cooling
      }
    }
    break;

  case 5:                       // cooling phase
      for(int i = 0; i < 2; i++){
        if(temp[i] - Setpoint[i] < 10){   // if the difference is small in one of the two heaters, no need to keep cooling the cooling element
            Setpoint[2 + i] = polTemps[selectVal[0]][2+i];  // set cooler temperature back to cooling temperature          
        }
      }

    if (checkSetTemp(10, 2, 1)) // if both are cooled down enough
    {
      digitalWrite(PN[2], LOW);
      pistonDown = 0;
      // Serial.println("Cooling finished, end program");
      startProgram = 0; // stop program
      programStarted = 0;
      programMode = 0;
      encChangeAllowed = 1;
      setTemps(selectVal[0], 0); // re-set cooler temps to not 0
      if(-stepper1.readSteps() / STEPTOMM == STAGE_LENGTH)  // if after the final move the carriage is at it's end position, likeliy indicating a long line weld way layed, return it to it's zero position
      {
        moveMotor(1000,-150);
      }
    }
    break;

  default:
    break;
  }
}

bool checkSetTemp(int margin, int noOfTempsToCheck, bool cooling)
{
  bool ready = 1;
  for (int i = 0; i < noOfTempsToCheck; i++)
  {
    if (cooling)
    { // if cooling, temp is higher and setpoint is lower
      if (temp[i] - Setpoint[i] > margin)
      { // if the difference between the setpoint and the actual temperature is larger than the margin
        ready = 0;
      }
    }
    else
    {
      if (Setpoint[i] - temp[i] > margin)
      { // if the difference between the setpoint and the actual temperature is larger than the margin
        ready = 0;
      }
    }
  }
  return ready;
}

void runMotor() // program for running the motor in "manual" mode
{
  if (!motorStarted)
  { // if the motor has not yet started
    if (startProgram && !stepper1.moving())
    {
      moveMotor(selectVal[3], selectVal[2]);
      motorStarted = 1;
      encChangeAllowed = 0;
      selectMode = 0;
    }
  }
  else // if the motor has been started
  { 
    if (!stepper1.moving() || !startProgram) // if done with movement or program is stopped
    { // if the motor has stopped moving, allow for new movement
      stepper1.stop();
      motorStarted = 0;
      startProgram = 0;
      encChangeAllowed = 1;
    }
  }
}

void moveMotor(int speed, int dist) // speed in mm/min, dist in mm
{
  stepper1.setSpeed(speed * STEPTOMM * 3 / 60); // = U/Min, no idea why * 3
    // Set speed in rpm*10. Step time is computed internally based on CYCLETIME and
    // steps per full rotation (stepsRev)
  if ((-stepper1.readSteps() / STEPTOMM) + dist > STAGE_LENGTH)
  {
    stepper1.moveTo(-STAGE_LENGTH * STEPTOMM);
  }
  else if (-stepper1.readSteps() / STEPTOMM + dist < 0)
  {
    Serial.println(-stepper1.readSteps() / STEPTOMM);
    stepper1.moveTo(0);
  }
  else
  {
    stepper1.move(-dist * STEPTOMM);
  }
}

void controlCooler()
{ // controls whether the cooling elements need to be activated
  for (int i = 2; i < 4; i++)
  {
    if (temp[i] > Setpoint[i] + 2)
    {
      PNon[i - 2] = 1;
    }
    else if (temp[i] < Setpoint[i] + 2)
    {
      PNon[i - 2] = 0;
    }
  }

  digitalWrite(PN[0], (manualCoolOn || PNon[0])); // if the manual button is pressed, or the system wants to cool down
  digitalWrite(PN[1], (manualCoolOn || PNon[1]));
}

void setPointTemp()
{
  if (tempPhase == -1)
  { // starts at -1, then the setpoint gets changed to
    // Serial.print("Temperature setpoint inititalised");
    Setpoint[0] = temp[0];
    Setpoint[1] = Setpoint[0];
    tempPhase = 0;

    if (tempSet[tempPhase + 1] > tempSet[tempPhase])
    {
      heatDir = 1;
    }
    else
    {
      heatDir = -1;
    }
  }

  if ((unsigned long)(millis() - lastTempUpdateTime) > setPointRefRate)
  {
    lastTempUpdateTime = millis();

    if ((Setpoint[0] + dT[tempPhase] * heatDir - tempSet[tempPhase + 1]) * heatDir > 0)
    { // if the current set semp + a next step would be beyond the setpoint
      if (abs(Setpoint[0] - temp[0]) < 5 && abs(Setpoint[1] - temp[1]) < 5)
      { // also wait for both temps to be near the setpoint
        tempPhase += 1;
        Setpoint[0] = tempSet[tempPhase];
        if (tempSet[tempPhase + 1] > tempSet[tempPhase])
        {
          heatDir = 1;
        }
        else
        {
          heatDir = -1;
        }
      }
    }
    else
    {
      Setpoint[0] += dT[tempPhase] * (float)(heatDir); // change setpoint
    }
    Setpoint[1] = Setpoint[0];
  }
}

void writeIntIntoEEPROM(int address, int number)
{
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

int readIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

String n2s(int i, int base) // takes int i and adds leading zeros if it is bases larger.
{
  String retString = "";
  int iBase;
  bool neg = (i < 0); // is 1 if i is negative, as then one leading zero needs to be removed
  if (-1 < i && i < 1)
  {
    iBase = 1;
  }
  else
  {
    iBase = floor(log10(abs(i))) + 1 + neg; // 1 =>  1, 10 =>  2, 100 => 3, -100 => 4
    if (iBase > base)
    {
      Serial.println("n2s error, too large number");
    }
  }
  for (int i = 0; i < (base - iBase); i++)
  {
    retString += " ";
  }
  retString += i;
  return retString;
}

void homeStepper(bool message)
{
  if (message)
  {
    Serial.println("start homing stepper");
  }
  lcd1.setCursor(4, 0);
  lcd1.print("Homing stepper");
  lcd1.setCursor(3, 2);
  lcd1.print("To stop homing,");
  lcd1.setCursor(3, 3);
  lcd1.print("press S, C or P");

  unsigned long lastHomePress = millis(); 
  bool homeButtonPressed = 0;     

  while(millis() - lastHomePress < 50 && homeButtonPressed == 0)    // first, check for 50ms that the switch is not pressed
  {
    if(digitalRead(STEPPERHOMEPIN)){    //if the home switch is not pressed
        homeButtonPressed = 1;          //say that the system should home
    } 
  }

  // digitalWrite(ENA, LOW);
  if(homeButtonPressed){    // if the stepper should be homed (during the 50ms the swith showed it was not home), turn the stepper on
    stepper1.setSpeed(2000); // = U/Min
    stepper1.rotate(1);
  }

  while (homeButtonPressed && digitalRead(START_PROGRAM) && digitalRead(MANUAL_COOLDOWN) && digitalRead(PISTON_DOWN))
  {
    if(!digitalRead(STEPPERHOMEPIN)){    //if the home switch is pressed
      if(millis() - lastHomePress > 50){ // if the home switch is pressed for 50ms (this is done because there was interference from statics)
        homeButtonPressed = 0;
      }
    } else {                              // if the home switch is  not pressed, re-set the last time it was seen pressed
      lastHomePress = millis();
    }
  }
  if (message)
  {
    Serial.println("Stepper homed");
  }
  stepper1.rotate(0);
  while(stepper1.moving()){   //wait for the stepper to wind down
  }
  digitalWrite(ENA, HIGH);    //TODO, can this be removed?
  stepper1.setZero();         // zero the starting point of the stepper
}

void thermocoupleInitialise(bool printSerial)
{
  for (int i = 0; i < 10; i++)
  {
    if (!thermocouples[i].begin())
    {
      if (printSerial)
      {
        Serial.print("Thermocouple ");
        Serial.print(i);
        Serial.println(" Not initialised");
        max31855Initialised = 1;
      }
    }
    if (printSerial)
    {
      Serial.print(thermocouples[i].readError(), BIN);
      Serial.print(" ");
    }
  }
  if (!max31855Initialised && printSerial)
  {
    Serial.println("All themocouples initialised");
  }
}

void setEncVals()
{
  for (int i = 0; i < 4; i++)
  {
    selectVal[i] = readIntFromEEPROM(i * 8);
  }
}

void repeatrun(){ 
  // Code to run multiple welding tests sequentialy
  
  if(startProgram && ! startRepeat){
    startRepeat = 1;    // only start repeating after a program has started and finished
    amountToMove = (STAGE_LENGTH)/(noRuns - 1); // distance to move in mm, total movement / (amount of tests - 1)
  }

  if(!startProgram && startRepeat && noRuns > 1){ // if the other program has finished and the amount of repeats is lower then the wanted amount
    startProgram = 1;   // start a new program
    noRuns -= 1;  // nth test that is taken
    moveMotor(selectVal[3], amountToMove);
    while(stepper1.moving()){ // while stepper is moving, do nothing
      delay(1);
    }
    polTemps[selectVal[0]][0] += 10;   // increase setpoint temperature with 10 deg
    polTemps[selectVal[0]][1] += 10;
    setTemps(selectVal[0],0);         // set temperature from memory to setpoints
    // selectVal[1] += 10;            // increase welding time by 10 seconds
  }


}