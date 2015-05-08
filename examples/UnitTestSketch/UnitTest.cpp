#include "Arduino.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM6DS3.h"
#include "UnitTest.h"

LSM6DS3 myIMU;

DeviceUnderTest::DeviceUnderTest( void )
{

}

void DeviceUnderTest::begin( void )
{
  myIMU.settings.commInterface = I2C_MODE;
  
  Serial.print("Initializing the IMU...\n");
  status_t tempStatus;
  tempStatus = myIMU.begin();
  if ( tempStatus != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
  }
  Serial.print("Done!\n");
}

void DeviceUnderTest::readAll( void )
{
  float tempFloat = 0;
  status_t tempStatus;

  //Get all parameters, report the errors
  Serial.print("Accelerometer:\n");

  tempFloat = myIMU.readAccelX();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" X = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readAccelY();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Y = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readAccelZ();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Z = ");
    Serial.println(tempFloat, 4);
  }

  Serial.print("Gyroscope:\n");

  tempFloat = myIMU.readGyroX();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" X = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readGyroY();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Y = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readGyroZ();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Z = ");
    Serial.println(tempFloat, 4);
  }

  Serial.print("Magnetometer:\n");

  tempFloat = myIMU.readMagX();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" X = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readMagY();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Y = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readMagZ();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Z = ");
    Serial.println(tempFloat, 4);
  }

  Serial.print("Thermometer:\n");

  tempFloat = myIMU.readTempC();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Degrees C = ");
    Serial.println(tempFloat, 4);
  }

  tempFloat = myIMU.readTempF();
  tempStatus = myIMU.getStatus();
  if ( myIMU.getStatus() != IMU_SUCCESS )
  {
    Serial.print("Not OK.  Error code: 0x0");
    Serial.print(tempStatus, HEX);
    Serial.print("\n");
  }
  else
  {
    Serial.print(" Degrees F = ");
    Serial.println(tempFloat, 4);
  }
}

void DeviceUnderTest::readAllPeak( void )
{

  Serial.print("\nSampling peak.  Press esc to exit.\n");
  uint16_t AccelErrors = 0;
  uint16_t GyroErrors = 0;
  uint16_t MagErrors = 0;
  uint16_t TempErrors = 0;

  float tempFloat = 0;

  float peakAccelX = 0;
  float peakAccelY = 0;
  float peakAccelZ = 0;
  float peakGyroX = 0;
  float peakGyroY = 0;
  float peakGyroZ = 0;
  float peakMagX = 0;
  float peakMagY = 0;
  float peakMagZ = 0;
  float peakTempC = 0;
  float peakTempF = 0;

  uint8_t exitWhile = 0;
  while ( exitWhile == 0 )
  {
    //Read the data
    tempFloat = myIMU.readAccelX();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakAccelX )
      {
        peakAccelX = tempFloat;
      }
    }
    else
    {
      AccelErrors++;
    }
    tempFloat = myIMU.readAccelY();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakAccelY )
      {
        peakAccelY = tempFloat;
      }
    }
    else
    {
      AccelErrors++;
    }
    tempFloat = myIMU.readAccelZ();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakAccelZ )
      {
        peakAccelZ = tempFloat;
      }
    }
    else
    {
      AccelErrors++;
    }

    tempFloat = myIMU.readGyroX();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakGyroX )
      {
        peakGyroX = tempFloat;
      }
    }
    else
    {
      GyroErrors++;
    }
    tempFloat = myIMU.readGyroY();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakGyroY )
      {
        peakGyroY = tempFloat;
      }
    }
    else
    {
      GyroErrors++;
    }
    tempFloat = myIMU.readGyroZ();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakGyroZ )
      {
        peakGyroZ = tempFloat;
      }
    }
    else
    {
      GyroErrors++;
    }

    tempFloat = myIMU.readMagX();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakMagX )
      {
        peakMagX = tempFloat;
      }
    }
    else
    {
      MagErrors++;
    }
    tempFloat = myIMU.readMagY();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakMagY )
      {
        peakMagY = tempFloat;
      }
    }
    else
    {
      MagErrors++;
    }
    tempFloat = myIMU.readMagZ();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakMagZ )
      {
        peakMagZ = tempFloat;
      }
    }
    else
    {
      MagErrors++;
    }

    tempFloat = myIMU.readTempC();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakTempC )
      {
        peakTempC = tempFloat;
      }
    }
    else
    {
      TempErrors++;
    }
    tempFloat = myIMU.readTempF();
    if ( myIMU.getStatus() == IMU_SUCCESS )
    {
      if ( tempFloat > peakTempF )
      {
        peakTempF = tempFloat;
      }
    }
    else
    {
      TempErrors++;
    }

    while (Serial.available())
    {
      if (Serial.read() == 0x1B)
      {
        exitWhile = 1;
      }
    }
  }

  Serial.print("\nAccel op errors: ");
  Serial.print(AccelErrors);
  Serial.print("\nGyro op errors: ");
  Serial.print(GyroErrors);
  Serial.print("\nMag op errors: ");
  Serial.print(MagErrors);
  Serial.print("\nTemp op errors: ");
  Serial.print(TempErrors);
  Serial.print("\n");

  Serial.print("\nAccelerometer");
  Serial.print("\n X = ");
  Serial.print(peakAccelX);
  Serial.print("\n Y = ");
  Serial.print(peakAccelY);
  Serial.print("\n Z = ");
  Serial.print(peakAccelZ);
  Serial.print("\n");
  Serial.print("\nGyroscope");
  Serial.print("\n X = ");
  Serial.print(peakGyroX);
  Serial.print("\n Y = ");
  Serial.print(peakGyroY);
  Serial.print("\n Z = ");
  Serial.print(peakGyroZ);
  Serial.print("\n");
  Serial.print("\nMagnetometer");
  Serial.print("\n X = ");
  Serial.print(peakMagX);
  Serial.print("\n Y = ");
  Serial.print(peakMagY);
  Serial.print("\n Z = ");
  Serial.print(peakMagZ);
  Serial.print("\n");
  Serial.print("\nTemperature\n");
  Serial.print(peakTempC);
  Serial.print(" degrees C\n");
  Serial.print(peakTempF);
  Serial.print(" degrees F\n");

}

void DeviceUnderTest::set1( void )
{
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.tempEnabled = 1;

  //Select interface mode
  myIMU.settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  myIMU.settings.commMode = 1;  //Can be modes 1, 2 or 3
  //Select address for I2C.  Does nothing for SPI
  myIMU.settings.I2CAddress = 0x6B; //Ignored for SPI_MODE
  //Select CS pin for SPI.  Does nothing for I2C
  myIMU.settings.chipSelectPin = 10;

  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control data
  myIMU.settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 10;  //default 10Hz
  myIMU.settings.fifoModeWord = 0;  //Default off
}

void DeviceUnderTest::set2( void )
{
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
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

  //FIFO control data
  myIMU.settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 10;  //default 10Hz
  myIMU.settings.fifoModeWord = 0;  //Default off
}

void DeviceUnderTest::set3( void )
{
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  myIMU.settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  myIMU.settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  myIMU.settings.accelFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.tempEnabled = 1;

  //Select interface mode
  myIMU.settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  myIMU.settings.commMode = 1;  //Can be modes 1, 2 or 3
  //Select address for I2C.  Does nothing for SPI
  myIMU.settings.I2CAddress = 0x6B; //Ignored for SPI_MODE
  //Select CS pin for SPI.  Does nothing for I2C
  myIMU.settings.chipSelectPin = 10;

  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  //FIFO control data
  myIMU.settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 10;  //default 10Hz
  myIMU.settings.fifoModeWord = 0;  //Default off
}