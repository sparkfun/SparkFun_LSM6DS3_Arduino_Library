/******************************************************************************
MinimalistExample.ino

Owen Lyke @ SparkFun Electronics
March 13, 2019
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
Most basic example of use - except now you can see if your settings got changed

Example using the LSM6DS3 with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");

  // Let's choose an unsupported setting...
  myIMU.settings.accelSampleRate = 404; // Typo, 'meant' to type '104'

  // Make a SensorSettings object to remember what you wanted to set everyhting to
  SensorSettings settingsIWanted;
  
  //Call .begin() to configure the IMU - supplying pointer to the SensorSettings structure
  myIMU.begin(&settingsIWanted);

  // Compare the sensor settings structure to know if anything was changed
  compareSettings(settingsIWanted);
  
}


void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 4);
  
  delay(1000);
}

void compareSettings(SensorSettings desiredSettings){
  if(myIMU.settings.accelBandWidth != desiredSettings.accelBandWidth )    { Serial.println("'accelBandWidth' was changed!"); }
  if(myIMU.settings.accelRange != desiredSettings.accelRange )            { Serial.println("'accelRange' was changed!"); }
  if(myIMU.settings.accelSampleRate != desiredSettings.accelSampleRate )  { Serial.println("'accelSampleRate' was changed!"); }
  if(myIMU.settings.gyroRange != desiredSettings.gyroRange )              { Serial.println("'gyroRange' was changed!"); }
  if(myIMU.settings.gyroSampleRate != desiredSettings.gyroSampleRate )    { Serial.println("'gyroSampleRate' was changed!"); }
  // That's all the settings that might get changed in 'begin()'
}
