/*
 * A faire
 * - trouver comment sortir du menu ou ajouter des boutons
 * - stockage en Eprom
 * - capteur de vitesse
 * - menus test et calib
 * - fct led
 * - circuit alim
 * - PCB
 * - batterie a connecter
 */




/* Sketch For a lightweight Air Data Reference Unit (ADIRU) and integrated standby instrument display
   by FSA - Apr 2018
   version 0.21 with display menu , 
   to be used as a standby instrument for indications on flying machines
   The ADIRU computes, broadcast and displays parameters such as
   Airspeed,                  SPD
   barometric altitude,       ALT
   heading, (magnetic)        HDG
   vertical speed,            VS
   pitch,                     PTCH
   roll,                      ROLL
   yaw,                       YAW
   acceleration vertical,     VRTG
   acceleration longitudinal  LONG
   Acceleration lateral       LATG
   Air temperature (local)    TEMP

   it is connected to a display for local indication of parameters
   It receives input from 3 buttons on pins 13,14,15 (Right,Left,Up)
   it receives a rotary encoder input on pins 35/36 - This sets the baro reference (QNH)
   a tri color LED is connected on pins 25/26/27 (RGB) for status indication : greeen OK, BLUE during align/reset, red for FAULT
   an Air Pressure sensor will be connected at some stage to compute airspeed - pin GPIO2
   An additional circuit and program provides full graphic display using a raspberry pi and a 800x640 px display (not described here)
   This sketch uses SDA/SCL on pins 21 / 22
   Interrupt is pin 23
   Note: not fit for flight, as it is, it will not display correctly in inverted flight.
   It runs on an ESP32 connected to the EM7180 sensor hub (onehorse)
   based on software by Kris Winner - devices found here https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/

*/

#include <MadgwickAHRS.h>
#include "data_structure.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <EEPROM.h>

#define EEPROM_SIZE 64 ; // to store a few data : defaultaffichage in position 0 ,
#define SerialDebug false  // set to true to get Serial output for debugging

// Set initial input parameters

// -------------------------------- affichage section -------------------------------------- 
// enumere les modes d affichage possible. Ils seront modifies par action sur les boutons Right (++) et left (--)
// la variable modeaffichage est un int avec des valeurs de 0 a 7
int modeaffichage ; // (si je ne le declare pas, ca ne marche pas), 
enum modeaffichage {PFD, // 0
  Heading,        // 1
  Altitude,       // 2
  Speed,          // 3
  Verticalspeed,  // 4 
  Temp,           // 5
  Chrono,         // 6
  Test,           // 7
  Startup         // 8
    };
int defaultaffichage=0 ; // defaut display mode used at startup (si on veut faire un compas, mettre 1, un alti 2,etc)
int startfault = 0; // used to detect fault during start-0 means no fault, if not fault code
// if startfault = 4 : "unknown error"
// if startfault = 5 : "No I2C devices found"
long fps = 20, oldfps = 1 ; // Valeur utilisee pour compter les frames par seconde, pas de zero a cause de la division 

// --------------------------------- Airspeed computation section -----------------------
int valuepitot ; // value read from pitot press sensor 
float pitotpress ; // raw pitot pressure
float ro = 1.225; // Air density in Kg/m3
float airspeed ; // airspeed
int airspd , newairspd,oldairspd ; // used for filtering
int deltaspd , deltaspd2 , fsa ; // used for filtering
// note : float pitch, yaw, roll,(software), Yaw, Pitch, Roll;(hardware)
int ptch, rol; // integers, to ease computation and display
int affalt,altpix,altpix10; // for altitude bars display
int counter=1;  // for test alt
int counter1=1;  // for test spd
int spdpix=1;
int cnt=0; // for spd test
int vmo = 240; // for vmo symbol, max speed , here 240 km/h
int vstall=65; // for stall symbol, here 65 km/h

// -----------------------------------Altitude computation variables -----------------
float newalt, oldalt ; // pour lissage des valeurs - attention aux floats


// -------------------------------- Partie inputs/debouncing -----------------------------
byte interruptPinR = 14; // Pin button Right
byte interruptPinL = 13; // Pin Button Left
byte interruptPinSET = 15;// Pin Button UP/SET
byte interruptPinCLOCK = 35;// Pin encoder 1/CLOCK
byte interruptPinDATA = 36;// Pin encoder 2/DATA
byte interruptPinPRESS = 12;// Pin encoder PRESS SWITCH  

// Holds the current buttons state.
boolean stateR;  // set by interrupt, must be reset by a function
boolean stateL;
boolean stateSET;
boolean stateCLOCK;
boolean stateDATA;
boolean statePRESS;
volatile int lastEncoded = 0;   // Volatile variables in an interruption
volatile long encodeur = 0; 
long lastEncodeur = 0;          // Value saved for comparison

// Holds the last time debounce was evaluated (in millis).
volatile long lastDebounceTimeR = 0;
volatile long lastDebounceTimeL = 0;
volatile long lastDebounceTimeSET = 0;
volatile long lastDebounceTimeCLOCK = 0;
volatile long lastDebounceTimeDATA = 0;
volatile long lastDebounceTimePRESS = 0;

// The delay threshold for debounce checking.in Ms
int debounceDelay = 80; // mini 50 ms, entre 50 et 100 ms c est bien!
int debounceDelay1 = 30; // utilise pour l encodeur
// ------------------------------- Variables diverses -------------------------------------

float qnh =1013.25 ; // normal value 
float calc1=1.0001; // used for alt calculation
int mov =0; // to check encoder, if positive mov = 1, if neg mov=-1
// ------------------------------- Partie capteurs & inertiel -----------------------------
// parametres divers
float vs = 0; // utilisé pour calculer la vertical speed
int vspd = 0; // idem, en integer

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BMP280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

// Specify BMP280 configuration
uint8_t Posr = P_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_042ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BMP280
int32_t t_fine;
//
// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 23;  // These can be changed,interruption pin for the EM7180
int myLed  = 5;  // GPIO5 : blue LED on the ESP32 onehorse
int ledred = 25;  // using a multicolour RGB led
int ledgreen = 26;
int ledblue = 27;

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280

// MPU9250 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount, rawPressure, rawTemperature;   // pressure, temperature raw count output
float   temperature, pressure, altitude; // Stores the MPU9250 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
uint8_t param[4];                         // used for param transfer
uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = false;  // pas utilise, serait pour un acces direct aux composants du EM7180
// display constructor - I have used a standard 128x64 Oled I2C and found this driver to work for U8glib
// it is fast, uses the Full frame buffer, but need to clear before and send after 
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

//  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX GRAPHIC FUNCTIONS

void draw(void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX DRAW 
  u8g2_prepare();

 
  // note : enum modeaffichage {PFD,Heading,Altitude,Speed,Verticalspeed,Temp,Chrono,Test,Startup};
    switch (modeaffichage) {
    case 0: displayPFD (); break;
    case 1: displayHeading (); break;
    case 2: displayAltitude (); break;
    case 3: displaySpeed (); break;
    case 4: displayVerticalspeed (); break;
    case 5: displayTemp (); break ;
    case 6: displayChrono (); break;
    case 7: displayTest () ; break;
    case 8: displayStartup () ; break;
     };
}   //  see tabs for details
  

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_9x18B_mr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX INPUTS & INTERRUPTS FUNCTIONS
void handleInterruptR() {   // fonction traitant l interruption R
    int reading = digitalRead (interruptPinR); // get the pin reading
    if (reading == stateR) return ; // Ignore dupe readings.
    boolean debounce = false;
    // Check to see if the change is within a debounce delay threshold.
    if ((millis() - lastDebounceTimeR) <= debounceDelay) {debounce = true;}
    // This update to the last debounce check is necessary regardless of debounce state.
    lastDebounceTimeR = millis();
    if(debounce) return; // Ignore reads within a debounce delay threshold.
    stateR = true; // All is good, persist the reading as the state.
   // Work with the value now:
   modeaffichage ++;  // decremente le compteur
   if (modeaffichage == 8) {modeaffichage = 7;} // si on depasse
   // note: la page startup n est pas selectable ici
   stateR = false; // fin du travail
}
void handleInterruptL() {   // fonction traitant l interruption L
    int reading = digitalRead (interruptPinL); // get the pin reading
    if (reading == stateL) return ; // Ignore dupe readings.
    boolean debounce = false;
    // Check to see if the change is within a debounce delay threshold.
    if ((millis() - lastDebounceTimeL) <= debounceDelay) {debounce = true;}
    // This update to the last debounce check is necessary regardless of debounce state.
    lastDebounceTimeL = millis();
    if(debounce) return; // Ignore reads within a debounce delay threshold.
    stateL = true; // All is good, persist the reading as the state.
   // Work with the value now.
   modeaffichage --;  // incremente le compteur
   if (modeaffichage <= 0) {modeaffichage = 0;} // si depassement inferieur, reste en mode 0 
   stateL = false; // fin du travail
}
void handleInterruptSET() {   // fonction traitant l interruption SET (3rd button)
    int reading = digitalRead (interruptPinSET); // get the pin reading
    if (reading == stateSET) return ; // Ignore dupe readings.
    boolean debounce = false;
    // Check to see if the change is within a debounce delay threshold.
    if ((millis() - lastDebounceTimeSET) <= debounceDelay) {debounce = true;}
    // This update to the last debounce check is necessary regardless of debounce state.
    lastDebounceTimeSET = millis();
    if(debounce) return; // Ignore reads within a debounce delay threshold.
    stateSET = true; // All is good, persist the reading as the state.
   // Work with the value now.
   digitalWrite (ledgreen, LOW);digitalWrite (ledblue, LOW);digitalWrite (ledred, LOW);
   Wire.reset();
   
}

void updateEncodeur() //  *******************************************
{
  int MSB = digitalRead(interruptPinDATA); // Le MSB = most significant bit
  int LSB = digitalRead(interruptPinCLOCK); // Le LSB = least significant bit

  int encoded = (MSB << 1) |LSB; // Converts the values on the 2 pins in an integer
  int sum  = (lastEncoded << 2) | encoded; // Adds the old value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encodeur++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encodeur--;

  lastEncoded = encoded; // Saves this value for the next time
}
void setup()  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX SETUP
{
  // Test LEDs at startup
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);// on circuit blue Led
    // frontal Led flash
  pinMode (ledred, OUTPUT);
  pinMode (ledgreen, OUTPUT);
  pinMode (ledblue, OUTPUT);
  digitalWrite (ledred, HIGH);
  delay (50);
  digitalWrite (ledgreen, HIGH);
  delay (50);
  digitalWrite (ledblue, HIGH);
  delay(50);
  digitalWrite (ledred, LOW);
  delay (50);
  digitalWrite (ledgreen, LOW);
  delay (50);
  digitalWrite (ledblue, LOW); // Blue Led should be on during alignment, but got some supply problems
  digitalWrite(myLed, LOW);// on circuit blue Led
  // initialise display routine and sets interface pin for menus
  // pins:   35 encoder A,36  Encoder clock, Encoder switch 12, Button left13 ,Button right 14,Button X : 15 
 // u8g2.begin(/*Select=*/ 15, /*Right/Next=*/ 13, /*Left/Prev=*/ 14);// , /*Up=*/ 4, /*Down=*/ 3, /*Home/Cancel=*/ 12);
   u8g2.begin(/*Select=*/ 12, /*Right/Next=*/ 36, /*Left/Prev=*/ 35);// , /*Up=*/ 13, /*Down=*/ 14, /*Home/Cancel=*/ 15);
    Serial.begin(74880);// reduced from 115200 because if too high seems to trigger ESP32 connection problems
  delay(100); // 
  // picture loop, only once here to display startup page
  modeaffichage = 8 ; // used only during startup
  u8g2.clearBuffer();
  draw();
  // Serial.println (" appel de la fonction draw");
  u8g2.sendBuffer();
    
  delay(1500);  // to let time to processes inside EM7180 to start, can be reduced but lead to anomalies <<<<<<<<<<<<<<<<<<<<<< Start delay

  // EEPROM READING 
  // defaultaffichage = byte(EEPROM.read(0)) ;
  Serial.print ("EPROM READ default affichage = "); Serial.println (defaultaffichage); 
  
  pinMode(interruptPinR, INPUT_PULLUP);// activation des resistances pullup for 5 buttons
  pinMode(interruptPinL, INPUT_PULLUP);
  pinMode(interruptPinSET, INPUT_PULLUP);
  pinMode(interruptPinCLOCK, INPUT_PULLUP);
  pinMode(interruptPinDATA, INPUT_PULLUP);
  pinMode(interruptPinPRESS, INPUT_PULLUP);
  // declare interruptions in case of actions on buttons
  attachInterrupt(digitalPinToInterrupt(interruptPinR), handleInterruptR, CHANGE);  // declarations 
  attachInterrupt(digitalPinToInterrupt(interruptPinL), handleInterruptL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinDATA), updateEncodeur, CHANGE); // Interrupt data
  attachInterrupt(digitalPinToInterrupt(interruptPinCLOCK), updateEncodeur, CHANGE); // Interrupt clock
  

  Wire.begin(21, 22, 400000); //(SDA, SCL) (21,22) on ESP32, 400 kHz I2C clock
  // Wire.setTimeout(500);// problem with wire library
  delay(500);  // a modifier et tester <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);  // interruption for the EM7180 , necesssary ,I dont know !
  
  I2Cscan(); // should detect SENtral at 0x28

  // Read SENtral device information
  uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
  uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
  uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
  Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
  uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
  Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");

  delay(800); // give some time to read the screen

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
  if (featureflag & 0x01)  Serial.println("A barometer is installed");
  if (featureflag & 0x02)  Serial.println("A humidity sensor is installed");
  if (featureflag & 0x04)  Serial.println("A temperature sensor is installed");
  if (featureflag & 0x08)  Serial.println("A custom sensor is installed");
  if (featureflag & 0x10)  Serial.println("A second custom sensor is installed");
  if (featureflag & 0x20)  Serial.println("A third custom sensor is installed");

  // delay(1000); // give some time to read the screen

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
  Serial.print ("STAT = "); Serial.println (STAT);
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("1 EEPROM detected on the sensor bus!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("1 EEPROM uploaded config file!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("1 EEPROM CRC incorrect!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("1 EM7180 in initialized state!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("1 No EEPROM detected!");
  int count = 0;
  Serial.print ("STAT 2 = "); Serial.println (STAT);
  while (!STAT) {
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);
    count++;
    STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    Serial.print ("STAT 3 = "); Serial.println (STAT);
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("2 EEPROM detected on the sensor bus!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("2 EEPROM uploaded config file!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("2 EEPROM CRC incorrect!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("2 EM7180 in initialized state!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("2 No EEPROM detected!");
    if (count > 10) break;
  }

  if (!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
  delay(800); // give some time to read the screen

  // Set up the SENtral as sensor bus in normal operating mode
   if (!passThru) {
    // Enter EM7180 initialized state
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz
    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz
    // writeByte(EM7180_ADDRESS, EM7180_TempRate, 0x19);  // set enable bit and set rate to 25 Hz

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4A)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4B)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    //Disable stillness mode
    EM7180_set_integer_param (0x49, 0x00);

    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
    EM7180_set_gyro_FS (0x7D0); // 2000 dps

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4A)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4B)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    if (runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    if (passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
    if (eventStatus & 0x02) Serial.println(" EM7180 Error");
    if (eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
    if (eventStatus & 0x08) Serial.println(" EM7180 new mag result");
    if (eventStatus & 0x10) Serial.println(" EM7180 new accel result");
    if (eventStatus & 0x20) Serial.println(" EM7180 new gyro result");

    delay(1000); // give some time to read the screen

    // Check sensor status
    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
    Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
    if (sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
    if (sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
    if (sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
    if (sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
    if (sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
    if (sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

    Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz");
    Serial.print("Actual AccelRate = "); Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz");
    Serial.print("Actual GyroRate = "); Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz");
    Serial.print("Actual BaroRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz");
    //  Serial.print("Actual TempRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualTempRate)); Serial.println(" Hz");

    delay(1000); // give some time to read the screen

    modeaffichage = defaultaffichage ; // startup ended, return to default mode

   }
  
}

void loop()   // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX LOOP
{

  
  // Serial.print("modeaffichage = "); Serial.println( modeaffichage);
//  Serial.print("interruptPinDATA = ");Serial.println(interruptPinDATA);
  
  if (!passThru) {

    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

    // Check for errors
    /*if (eventStatus & 0x02) { // error detected, what is it?

      uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
      if (errorStatus != 0x00) { // non-zero value indicates error, what is it?
        Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
        if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
        if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
        if (errorStatus == 0x14) Serial.print("Gyro failure!");
        if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
        if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
        if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
        if (errorStatus == 0x30) Serial.print("Math error!");
        if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
      }

      // Handle errors ToDo
      
    } */
    // if no errors, see if new data is ready
    if (eventStatus & 0x10) { // new acceleration data available
      readSENtralAccelData(accelCount);

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * 0.000488; // get actual g value
      ay = (float)accelCount[1] * 0.000488;
      az = (float)accelCount[2] * 0.000488;
    }

    if (eventStatus & 0x20) { // new gyro data available
      readSENtralGyroData(gyroCount);

      // Now we'll calculate the gyro value into actual dps's
      gx = (float)gyroCount[0] * 0.153; // get actual dps value
      gy = (float)gyroCount[1] * 0.153;
      gz = (float)gyroCount[2] * 0.153;
    }

    if (eventStatus & 0x08) { // new mag data available
      readSENtralMagData(magCount);

      // Now we'll calculate the mag value into actual G's
      mx = (float)magCount[0] * 0.305176; // get actual G value
      my = (float)magCount[1] * 0.305176;
      mz = (float)magCount[2] * 0.305176;
    }

    if (eventStatus & 0x04) { // new quaternion data available
      readSENtralQuatData(Quat);
    }

    
  }

  if (passThru) {
    // If intPin goes high, all data registers have new data
    //  if (digitalRead(intACC2)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the acceleration value into actual g's
    ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes - accelBias[1];
    az = (float)accelCount[2] * aRes - accelBias[2];
    // }
    //  if (digitalRead(intGYRO2)) {  // On interrupt, read data
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
    // }
    //  if (digitalRead(intDRDYM)) {  // On interrupt, read data
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
    //    mx *= magScale[0];
    //    my *= magScale[1];
    //    mz *= magScale[2];
    //   }
  }


  // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // We will assume that +y accel/gyro is North, then x accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the madgwick function Ay, Ax, -Az, Gy, Gx, -Gz, Mx, My, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ay, -Ax, Az, Gy, Gx, -Gz, Mx, My, and Mz into the Madgwick
  // function to get North along the accel +y-axis, East along the accel +x-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(-ay, -ax, az, gy * PI / 180.0f, gx * PI / 180.0f, -gz * PI / 180.0f,  mx,  my, mz);
  //  if(passThru)MahonyQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz);

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  // if (delt_t > 500) { // update LCD once per half-second independent of read rate  <<<< no need in final version
    if (delt_t > 30) {  // to reduce lag , for 25 images/sec => delta_t = 40
    if (SerialDebug) {
      Serial.print("ax = "); Serial.print((int)1000 * ax);
      Serial.print(" ay = "); Serial.print((int)1000 * ay);
      Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
      Serial.print("gx = "); Serial.print( gx, 2);
      Serial.print(" gy = "); Serial.print( gy, 2);
      Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print( (int)mx);
      Serial.print(" my = "); Serial.print( (int)my);
      Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");

      Serial.println("Software quaternions:");
      Serial.print("q0 = "); Serial.print(q[0]);
      Serial.print(" qx = "); Serial.print(q[1]);
      Serial.print(" qy = "); Serial.print(q[2]);
      Serial.print(" qz = "); Serial.println(q[3]);
      Serial.println("Hardware quaternions:");
      Serial.print("Q0 = "); Serial.print(Quat[0]);
      Serial.print(" Qx = "); Serial.print(Quat[1]);
      Serial.print(" Qy = "); Serial.print(Quat[2]);
      Serial.print(" Qz = "); Serial.println(Quat[3]);
    }


    // tempCount = readTempData();  // Read the gyro adc values
    //    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade
    //    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    if (passThru) {
      rawPress =  readBMP280Pressure();
      pressure = (float) bmp280_compensate_P(rawPress) / 25600.; // Pressure in mbar
      rawTemp =   readBMP280Temperature();
      temperature = (float) bmp280_compensate_T(rawTemp) / 100.;

    }


    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
    //Hardware AHRS:
    Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);
    Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    Pitch *= 180.0f / PI;
    Yaw   *= 180.0f / PI;
    // Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    Yaw += 1.0f; // declinaison a Toulouse, france
    if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    Roll  *= 180.0f / PI;

    // Or define output variable according to the Android system, where heading (0 to 360) is defined by the angle between the y-axis
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.
    // get BMP280 pressure
  
      //   Serial.println("new Baro data!");
      rawPressure = readSENtralBaroData();
      pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar

      // get BMP280 temperature
      rawTemperature = readSENtralTempData();
      temperature = (float) rawTemperature * 0.01; // temperature in degrees C
  // Serial.print ("encodeur = "); Serial.println (encodeur); 
 
    // Altitude computation - it need some tricks due to problem with division
    if (modeaffichage==0 || modeaffichage==2)   // This section is only needed if altitude has to be displayed
    {
    // check QNH change
    qnh=1013.25f+((float)encodeur/10);
    if (qnh>1050) {qnh=1050;};
    if (qnh<900) {qnh=900;}
    calc1 = (pressure/qnh);  // 
    altitude = 145366.45f * (1.0f - pow(calc1, 0.190284f));
    
 // Serial.print("     mov = ");Serial.print(mov);
  Serial.print("    qnh = ");Serial.print(qnh);
 // Serial.print("    calc1 = ");Serial.print(calc1);
 //  Serial.print("    pressure = ");Serial.print(pressure);
   Serial.print("    altitude = ");Serial.println(altitude);
}
    if (SerialDebug) {
      Serial.print("Software yaw, pitch, roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);

      Serial.print("Hardware Yaw, Pitch, Roll: ");
      Serial.print(Yaw, 2);
      Serial.print(", ");
      Serial.print(Pitch, 2);
      Serial.print(", ");
      Serial.println(Roll, 2);

      Serial.println("BMP280:");
      Serial.print("Altimeter temperature = ");
      Serial.print( temperature, 2);
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = ");
      Serial.print(9.*temperature / 5. + 32., 2);
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = ");
      Serial.print(pressure, 2);
      Serial.println(" mbar");// pressure in millibar
     
      Serial.print("Altitude = ");
      Serial.print(altitude, 2);
      Serial.println(" feet");
      Serial.println(" ");
    }

   //  Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
    //     Serial.print(millis()/1000.0, 1);Serial.print(",");
    //     Serial.print(yaw); Serial.print(",");Serial.print(pitch); Serial.print(",");Serial.print(roll); Serial.print(",");
    //     Serial.print(Yaw); Serial.print(",");Serial.print(Pitch); Serial.print(",");Serial.println(Roll);


    digitalWrite(myLed, !digitalRead(myLed));
    count = millis();
    sumCount = 0;
    sum = 0;
  }

// ----------------------- This section to compute Airspeed, this will be a Computed Air Speed, compensated for altitude and for temp if a temp sensor is added
 // Let's compute CAS (Computed air speed), first pressure measurement 
   valuepitot = analogRead(2);// value read from sensor , here 3496 units (measured voltage) corresponds to 10 Kpa on the sensor diagram , ADC is 12 bits, max reading 4096 
   // note :pressure sensor hysteresis which is 0.040 Kpa or 0,3V, see chart from sensor MPX5010, but we still need 0 if sensor says 0
   // pressure is converted to a voltage , then divided by 3,; 5 V gives a reading of 3720 , corresponds to 10 Kpascals
  pitotpress = valuepitot /0.3496 ; //  because 3496 units for 10 Kpa  
  // Let s compute air density
  // tc = temperature ; // temp measured inside BMP280 sensor , normally different from Outside Air Temperature
  //ro = 1.292 + (273/(tc + 273)) ; // see wikipedia, actual temp converted to kelvins. Real temp measurement should give better accuracy
  // ro = 1.225 ; // at sea level air density in KG/m3
    ro= 1.225 - (altitude* 0.000032) ; // conversion function approximated, linear for lower atmosphere
    airspeed = 3.6 * (sqrt ((2 * pitotpress)/ ro)) ; // 3.6 to convert M/S into Km/h, for the rest, see wikipedia
    //correction for altitude = +1% per 600 ft above MSL
    airspeed = airspeed + (airspeed *(altitude/600)/100);
    // correction for temperature : add 1% for every 5 deg C above 15 C
    airspeed = airspeed + temperature *((temperature-15)/5) / 100 ; // should be measured by an external T sensor
    
     // special fsa filtering method
  newairspd = int(airspeed); // this will be faster
  deltaspd = (newairspd - oldairspd);
  deltaspd2 = deltaspd * deltaspd;
  fsa = min (deltaspd2,10 * deltaspd); 
  airspd= oldairspd + (fsa/20) ;// this division with integers filters the lowest values
  oldairspd = airspd ; 

// ----------------------- This section is to compute and filter barometric altitude
 // lissage
 if (newalt==0) {newalt = altitude;} // C est pour le demarrage
 
 newalt=oldalt + ((altitude-oldalt)/10);  // lissage au 1/10 eme
 oldalt = newalt ;
 // test
 /*   Serial.print ("altitude = "); Serial.println (altitude);
    Serial.print ("newalt = "); Serial.println (newalt);
    Serial.print ("oldalt = "); Serial.println (oldalt);
   */  


  // picture loop
  u8g2.clearBuffer();
  draw();
  // Serial.println (" appel de la fonction draw");
  u8g2.sendBuffer();

/* if need to display buttons & flags status :
 *   // affiche l etat
   Serial.print ("stateR: "); Serial.print (stateR); 
   Serial.print ("  stateL: ");   Serial.print (stateL);
   Serial.print ("  stateUP: ");  Serial.print (stateUP); 
   Serial.print ("  stateP1: ");  Serial.print (stateP1);
   Serial.print ("  stateP2: ");  Serial.println (stateP2);
   Serial.print (" bton R:"); Serial.print (digitalRead (interruptPinR));
   Serial.print (" bton L:"); Serial.print (digitalRead (interruptPinL));
   Serial.print (" bton UP:"); Serial.print (digitalRead (interruptPinUP));
   Serial.print (" bton P1:"); Serial.print (digitalRead (interruptPinP1));
   Serial.print (" bton P2:"); Serial.println (digitalRead (interruptPinP2));
  if (millis() >= 20000) { stateR=false;stateL=false;stateUP=false;stateP1=false;stateP2=false; }// por resetter apres 20 secondes
     delay(200);
     
 */


}


