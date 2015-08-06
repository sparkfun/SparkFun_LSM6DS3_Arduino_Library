/******************************************************************************
MultiI2C.ino

Example using up to two LSM6DS3s on the same I2C channel.  If only one sensor
is attached, this sketch reports failure on that channel and runs with the
single sensor instead.

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and ***3.3v*** power to the IMU.  The sensors are not 5v tolerant.

(Multiple I2C devices use the same pins.  Up to two LSM6DS3s are allowed.  Use
the solder jumper to select address 0x6A or 0x6B)

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.
******************************************************************************/
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

//Create two instances of the driver class
LSM6DS3 SensorOne( I2C_MODE, 0x6A );
LSM6DS3 SensorTwo( I2C_MODE, 0x6B );


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMUs
  if( SensorOne.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor at 0x6A.");
  }
  else
  {
	  Serial.println("Sensor at 0x6A started.");
  }
  if( SensorTwo.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor at 0x6B.");
  }
  else
  {
	  Serial.println("Sensor at 0x6B started.");
  }
  
}


void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  Serial.println(SensorOne.readFloatAccelX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(SensorOne.readFloatAccelY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(SensorOne.readFloatAccelZ(), 4);
  Serial.print(" X2 = ");
  Serial.println(SensorTwo.readFloatAccelX(), 4);
  Serial.print(" Y2 = ");
  Serial.println(SensorTwo.readFloatAccelY(), 4);
  Serial.print(" Z2 = ");
  Serial.println(SensorTwo.readFloatAccelZ(), 4);
  
  Serial.print("\nGyroscope:\n");
  Serial.print(" X1 = ");
  Serial.println(SensorOne.readFloatGyroX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(SensorOne.readFloatGyroY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(SensorOne.readFloatGyroZ(), 4);
  Serial.print(" X2 = ");
  Serial.println(SensorTwo.readFloatGyroX(), 4);
  Serial.print(" Y2 = ");
  Serial.println(SensorTwo.readFloatGyroY(), 4);
  Serial.print(" Z2 = ");
  Serial.println(SensorTwo.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C1 = ");
  Serial.println(SensorOne.readTempC(), 4);
  Serial.print(" Degrees C2 = ");
  Serial.println(SensorTwo.readTempC(), 4);
  Serial.print(" Degrees F1 = ");
  Serial.println(SensorOne.readTempF(), 4);
  Serial.print(" Degrees F2 = ");
  Serial.println(SensorTwo.readTempF(), 4);
  
  Serial.print("\nSensorOne Bus Errors Reported:\n");
  Serial.print(" All '1's = ");
  Serial.println(SensorOne.allOnesCounter);
  Serial.print(" Non-success = ");
  Serial.println(SensorOne.nonSuccessCounter);
  Serial.print("SensorTwo Bus Errors Reported:\n");
  Serial.print(" All '1's = ");
  Serial.println(SensorTwo.allOnesCounter);
  Serial.print(" Non-success = ");
  Serial.println(SensorTwo.nonSuccessCounter);
  delay(1000);
}
