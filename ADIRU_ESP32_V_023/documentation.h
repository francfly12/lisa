/*  Documentation and Wiring
 *   
 *  based on an ESP32 and a EM7180 SENtral sensor hub from onehorse 
 *  tested on "onehorse" which is one of the lightest ESP 32 but should easily be adapted to other esp32 boards
   partially derived from software by Kris Winner - 
   devices found here https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/
   
 * The ADIRU should broadcast parameters such as
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
   It receives input from 3 buttons on pins 13,14,15
   it receives a rotary encoder input on pins 35/36 - This determines the baro reference (QNH)
   a tri color LED is connected on pins 25/26/27 (RGB) for status indication : greeen OK, BLUE during align/reset, red for FAULT
   an Air Pressure sensor will be connected at some stage to compute airspeed - pin GPIO2
   This sketch uses SDA/SCL on pins 21 / 22
   Interrupt is pin 23
  
*/
/* extract :
  The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
  in this case the MPU9250 (with embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
  on-board M24512DRC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
  and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
  heading/pitch/roll, if selected.
  This sketch operates the EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor,
  getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
  allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
  Mahony filter algorithms to compare with the hardware sensor fusion results.
  This sketch is for the EM7180 SENtral sensor hub as master,
  the MPU9250 9-axis motion sensor (accel/gyro/mag) as slave, a BMP280 pressure/temperature sensor, and an M24512DRC
  512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
  yet and there is a driver for the BMP280 in the SENtral firmware.
  The BMP280 is a simple but high resolution pressure sensor, which can be used in its high resolution
  mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
  only 1 microAmp. The choice will depend on the application.

  SDA and SCL should normally NOT need external pull-up resistors (to 3.3V).
  4k7 resistors are already on the EM7180+MPU9250+BMP280+M24512DRC

  Hardware setup:
                                _________
            SCL         GPIO22 |    E    | GPIO21          SDA
    Interrupt from 7180 GPIO23 |    S    | GPIO19
    Led red             GPIO25 |    P    | GPIO18
    Led green           GPIO26 |    3    | GPIO17
    Led blue            GPIO27 |    2    | GPIO16
                        GPIO34 |         | GPIO15          Button set
    Encoder switch 1    GPIO35 |         | GPIO14          Button rh
    Encoder switch 2    GPIO36 |         | GPIO13          Button lh
                        GPIO37 |         | GPIO12          encoder switch press
                        GPIO38 |         | GPIO5   Blue led on circuit (myled)
                        GPIO39 |         | GPIO4
                          3V3  |         | GPIO2           resistor bridge 2/3 to Analog press sensor  
                          GND  |         | GPIO0   Boot button
                         VBAT  |_________|  GND

  EM7180 Mini Add-On ------- ESP32
  VDD ---------------------- 3.3V
  SDA ----------------------- 21
  SCL ----------------------- 22
  GND ---------------------- GND
  INT------------------------ 23

  Sensors in this board are I2C sensor and uses the Wire library.
  Because the sensors are not 5V tolerant, we are using a 3.3 V

  disclaimer

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  


  */
