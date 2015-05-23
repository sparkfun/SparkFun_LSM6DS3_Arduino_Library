#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU;

void setup( void ) {
  Serial.begin(57600);  // start serial for output

  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 208;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 208;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1
  myIMU.settings.tempEnabled = 1;
  
  //Select interface mode
  myIMU.settings.commInterface = SPI_MODE; //Can be I2C_MODE, SPI_MODE
  myIMU.settings.commMode = 1;  //Can be modes 1, 2 or 3
  //Select address for I2C.  Does nothing for SPI
  myIMU.settings.I2CAddress = 0x6B; //Ignored for SPI_MODE
  //Select CS pin for SPI.  Does nothing for I2C
  myIMU.settings.chipSelectPin = 10;
  
  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control settings
  myIMU.settings.fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 200;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.
  //FIFO mode.  Can be:
  //  0 (Bypass mode, FIFO off)
  //  1 (Stop when full)
  //  3 (Continuous during trigger)
  //  4 (Bypass until trigger)
  //  6 (Continous mode)
  
  uint8_t c = myIMU.begin();  //Can be called again to load new settings
  myIMU.fifoBegin();
  
  Serial.print("begin() ran.  Returns WHO_AM_I of: 0x");
  Serial.println(c, HEX);
  
  Serial.print("Clearing out the FIFO...");
  myIMU.fifoClear();
  Serial.print("Done!\n");
  
}


void loop()
{
  float temp;  //This is to hold read data
  uint16_t tempUnsigned;
  
  while( ( myIMU.fifoGetStatus() & 0x8000 ) == 0 ) {};  //Wait for watermark

  //Get the divisor for the gyro
  uint8_t gyroRangeDivisor = myIMU.settings.gyroRange / 125;
  if ( myIMU.settings.gyroRange == 245 ) {
    gyroRangeDivisor = 2;
  }
  
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
//  tempUnsigned = myIMU.fifoGetStatus();
//  Serial.print("Fifo Status 1 and 2 (16 bits): 0x");
//  Serial.println(tempUnsigned, HEX);
  }
  
  delay(1000);
}