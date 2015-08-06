/******************************************************************************
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout


FifoExample.ino

Example using the FIFO over SPI.

The FIFO is configured to take readings at 50Hz.  When 100 samples have
accumulated (when the "watermark" is reached), the sketch dumps the float values to the serial terminal.

The FIFO can sample much faster but the serial port isn't fast enough to get
that data out before another 100 samples get queued up.  There is a 10ms delay
placed after each line ("1.40,-4.41,-3.22,-0.01,0.01,0.99") so that the
internal serial buffer is guaranteed to empty and not overflow.

Cranking the sample rate up to 800Hz will result in the FIFO dumping routine
never getting the FIFO back down to zero.

Removing the 10ms delay allows the FIFO to be emptied, but then too much data
gets placed in the serial write buffer and stability suffers.


Resources:
Uses Wire.h for I2C operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections
***CAUTION -- SPI pins can not be connected directly to 5V IO***

Connect SDA/SDI line to pin 11 through a level shifter (MOSI)
Connect SCL pin line to pin 13 through a level shifter (SCLK)
Connect SDO/SA0 line to pin 12 through a level shifter (MISO)
Connect CS to a free pin through a level shifter.  This example uses pin 10.
Connect GND and ***3.3v*** power to the IMU.  The sensors are not 5v tolerant.

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.
******************************************************************************/
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU( SPI_MODE, 10 );

void setup( void ) {
  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 833;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
  myIMU.settings.tempEnabled = 1;
  
    //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control settings
  myIMU.settings.fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 50;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.
  //FIFO mode.  Can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (Stop when full)
  //  3 (Continuous during trigger)
  //  4 (Bypass until trigger)
  //  6 (Continous mode)
  

  Serial.begin(57600);  // start serial for output
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMUs
  if( myIMU.begin() != 0 )
  {
	  Serial.println("Problem starting the sensor with CS @ Pin 10.");
  }
  else
  {
	  Serial.println("Sensor with CS @ Pin 10 started.");
  }
  
  Serial.print("Configuring FIFO with no error checking...");
  myIMU.fifoBegin();
  Serial.print("Done!\n");
  
  Serial.print("Clearing out the FIFO...");
  myIMU.fifoClear();
  Serial.print("Done!\n");
  
}


void loop()
{
  float temp;  //This is to hold read data
  uint16_t tempUnsigned;
  
  while( ( myIMU.fifoGetStatus() & 0x8000 ) == 0 ) {};  //Wait for watermark
 
  //Now loop until FIFO is empty.  NOTE:  As the FIFO is only 8 bits wide,
  //the channels must be synchronized to a known position for the data to align
  //properly.  Emptying the fifo is one way of doing this (this example)
  while( ( myIMU.fifoGetStatus() & 0x1000 ) == 0 ) {

  temp = myIMU.calcGyro(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print(",");

  temp = myIMU.calcGyro(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print(",");

  temp = myIMU.calcGyro(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print(",");

  temp = myIMU.calcAccel(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print(",");

  temp = myIMU.calcAccel(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print(",");

  temp = myIMU.calcAccel(myIMU.fifoRead());
  Serial.print(temp);
  Serial.print("\n");
  
  delay(10); //Wait for the serial buffer to clear (~50 bytes worth of time @ 57600baud)
  
  }

  tempUnsigned = myIMU.fifoGetStatus();
  Serial.print("\nFifo Status 1 and 2 (16 bits): 0x");
  Serial.println(tempUnsigned, HEX);
  Serial.print("\n");  

}