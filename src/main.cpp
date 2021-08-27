/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Accelerometer.ino
* Date: 2014/09/09
* Revision: 3.0 $
*
* Usage:        Example code to stream Accelerometer data
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NineAxesMotion mySensor;                 //Object that for the sensor
bool justStarted = true; // a 'first time' start, which means i can just hit the reboot button to pull a fresh frame

void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();

}

void loop() //This code is looped forever
{
  //mySensor.updateAccel();        //Update the Accelerometer data
  //mySensor.updateLinearAccel();  //Update the Linear Acceleration data
  //mySensor.updateGravAccel();    //Update the Gravity Acceleration data

  /**
  **/
  if(Serial.read() == 1 || justStarted) {
    justStarted = false;
    
    mySensor.updateQuat();
    mySensor.updateMag();
    //mySensor.updateCalibStatus();  //Update the Calibration Status
    int16_t w = mySensor.readQuaternion(W_QUAT);
    int16_t x = mySensor.readQuaternion(X_QUAT);
    int16_t y = mySensor.readQuaternion(Y_QUAT);
    int16_t z = mySensor.readQuaternion(Z_QUAT);
    float mag_x = mySensor.readMagX();
    float mag_y = mySensor.readMagY();
    float mag_z = mySensor.readMagZ();
    
    Serial.write(reinterpret_cast<char*>(&w), sizeof(w));
    Serial.write(reinterpret_cast<char*>(&x), sizeof(x));
    Serial.write(reinterpret_cast<char*>(&y), sizeof(y));
    Serial.write(reinterpret_cast<char*>(&z), sizeof(z));
    Serial.write(reinterpret_cast<char*>(&mag_x), sizeof(mag_x));
    Serial.write(reinterpret_cast<char*>(&mag_y), sizeof(mag_y));
    Serial.write(reinterpret_cast<char*>(&mag_z), sizeof(mag_z));
    Serial.flush();
  }
}