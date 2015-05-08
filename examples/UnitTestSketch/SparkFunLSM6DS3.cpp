#include "SparkFunLSM6DS3.h"
#include "SparkFunIMU.h"
#include "stdint.h"

#include "Wire.h"
#include "SPI.h"

//Constructor
LSM6DS3::LSM6DS3( void )
{

  //Construct with these default settings if nothing is specified

  settings.gyroEnabled = 1;  //Can be 0 or 1
  settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  settings.gyroSampleRate = 416;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
  settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  settings.gyroFifoDecimation = 1;  //set 1 for on /1

  settings.accelEnabled = 1;
  settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  settings.accelSampleRate = 416;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
  settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  settings.accelFifoDecimation = 1;  //set 1 for on /1

  settings.tempEnabled = 1;

  //Select interface mode
  settings.commInterface = I2C_MODE; //Can be I2C_MODE, SPI_MODE
  settings.commMode = 1;  //Can be modes 1, 2 or 3
  //Select address for I2C.  Does nothing for SPI
  settings.I2CAddress = 0x6B; //Ignored for SPI_MODE
  //Select CS pin for SPI.  Does nothing for I2C
  settings.chipSelectPin = 10;

  //Non-basic mode settings
  settings.commMode = 1;

  //FIFO control data
  settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
  settings.fifoSampleRate = 10;  //default 10Hz
  settings.fifoModeWord = 0;  //Default off

  //Clear out data regs
  xAccel = 0;
  yAccel = 0;
  zAccel = 0;
  xGyro = 0;
  yGyro = 0;
  zGyro = 0;
  celsiusTemp = 0;
  fahrenheitTemp = 0;
  
  //Status of the settings
  settings.status = IMU_NOT_CONFIGURED;

}


// Public methods
status_t LSM6DS3::begin()
{
  //Check the settings structure values to determine how to setup the device
  uint8_t dataToWrite = 0;  //Temporary variable
  status_t result;
  
  settings.status = IMU_NOT_CONFIGURED; //Write this when the configuration completes.
  driverStatus = IMU_NOT_CONFIGURED;
  
  switch (settings.commInterface) {

    case I2C_MODE:
      Wire.begin();
      break;

    case SPI_MODE:
      // start the SPI library:
      SPI.begin();
      // Maximum SPI frequency is 10MHz, could divide by 2 here:
      SPI.setClockDivider(SPI_CLOCK_DIV4);
      // Data is read and written MSb first.
      SPI.setBitOrder(MSBFIRST);
      // Data is captured on rising edge of clock (CPHA = 0)
      // Base value of the clock is HIGH (CPOL = 1)
      SPI.setDataMode(SPI_MODE3);
      // initalize the  data ready and chip select pins:
      pinMode(settings.chipSelectPin, OUTPUT);
      digitalWrite(settings.chipSelectPin, HIGH);
      break;

    default:
      break;
  }

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  if ( settings.accelEnabled == 1) {
    //Build config reg
    //First patch in filter bandwidth
    switch (settings.accelBandWidth) {
      case 50:
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
        break;
      case 100:
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
        break;
      case 200:
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
        break;
      default:  //set default case to max passthrough
      case 400:
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
        break;
    }
    //Next, patch in full scale
    switch (settings.accelRange) {
      case 2:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
        break;
      case 4:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
        break;
      case 8:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
        break;
      default:  //set default case to 16(max)
      case 16:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
        break;
    }
    //Lastly, patch in accelerometer ODR
    switch (settings.accelSampleRate) {
      case 13:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
        break;
      case 26:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
        break;
      case 52:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
        break;
      default:  //Set default to 104
      case 104:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
        break;
      case 208:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
        break;
      case 416:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
        break;
      case 833:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
        break;
      case 1660:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
        break;
      case 3330:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
        break;
      case 6660:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
        break;
      case 13330:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
        break;
    }
  }
  else
  {
    //dataToWrite already = 0 (powerdown);
  }

  //Now, write the patched together data
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Setup the gyroscope**********************************************
  dataToWrite = 0; //Start Fresh!
  if ( settings.gyroEnabled == 1) {
    //Build config reg
    //First, patch in full scale
    switch (settings.gyroRange) {
      case 125:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
        break;
      case 245:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
        break;
      case 500:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
        break;
      case 1000:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
        break;
      default:  //Default to full 2000DPS range
      case 2000:
        dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
        break;
    }
    //Lastly, patch in gyro ODR
    switch (settings.gyroSampleRate) {
      case 13:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
        break;
      case 26:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
        break;
      case 52:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
        break;
      default:  //Set default to 104
      case 104:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
        break;
      case 208:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
        break;
      case 416:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
        break;
      case 833:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
        break;
      case 1660:
        dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
        break;
    }
  }
  else
  {
    //dataToWrite already = 0 (powerdown);
  }
  //Write the byte
  writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  //Setup the internal temperature sensor
  if ( settings.tempEnabled == 1) {
  }

  //Test for WHO_AM_I register
  if( readRegister(LSM6DS3_ACC_GYRO_WHO_AM_I_REG) == WHO_AM_I_CORRECT_RESPONSE )
  {
	  result = IMU_SUCCESS;
	  settings.status = IMU_SUCCESS;
	  driverStatus = IMU_SUCCESS;
  }
  else
  {
	  result = IMU_NOT_CONFIGURED;
	  settings.status = IMU_NOT_CONFIGURED;
	  driverStatus = IMU_NOT_CONFIGURED;
  }

  return result;
  
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
float LSM6DS3::readAccelX()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTX_L_XL, 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  xAccel = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 0.061 * (settings.accelRange >> 1) / 1000;

  return xAccel;
}
float LSM6DS3::readAccelY()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTY_L_XL, 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  yAccel = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 0.061 * (settings.accelRange >> 1) / 1000;

  return yAccel;
}
float LSM6DS3::readAccelZ()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTZ_L_XL, 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  zAccel = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 0.061 * (settings.accelRange >> 1) / 1000;

  return zAccel;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
float LSM6DS3::readGyroX()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTX_L_G, 2);  //Does memory transfer

  uint8_t gyroRangeDivisor = settings.gyroRange / 125;
  if ( settings.gyroRange == 245 ) {
    gyroRangeDivisor = 2;
  }
  
  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  xGyro = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 4.375 * (gyroRangeDivisor) / 1000;

  return xAccel;
}
float LSM6DS3::readGyroY()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTY_L_G, 2);  //Does memory transfer

  uint8_t gyroRangeDivisor = settings.gyroRange / 125;
  if ( settings.gyroRange == 245 ) {
    gyroRangeDivisor = 2;
  }
  
  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  yGyro = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 4.375 * (gyroRangeDivisor) / 1000;

  return yAccel;
}
float LSM6DS3::readGyroZ()
{
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUTZ_L_G, 2);  //Does memory transfer

  uint8_t gyroRangeDivisor = settings.gyroRange / 125;
  if ( settings.gyroRange == 245 ) {
    gyroRangeDivisor = 2;
  }
  
  //Do the math to get from raw numbers to useful parts.  Save into class.
  //Do counts * (min unit/count) * full scale
  zGyro = (float)(myBuffer[0] | (myBuffer[1] << 8)) * 4.375 * (gyroRangeDivisor) / 1000;

  return zGyro;
}


//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
float LSM6DS3::readTempC( void ) {
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUT_TEMP_L, 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful int
  int16_t outputVariable = myBuffer[0] | (myBuffer[1] << 8);

  outputVariable = outputVariable >> 4; //divide by 16 to scale
  outputVariable += 25; //Add 25 degrees to remove offset

  //Calibration
  celsiusTemp = outputVariable;
  fahrenheitTemp = (celsiusTemp * 9) / 5 + 32;

  return celsiusTemp;
}

float LSM6DS3::readTempF( void ) {
  uint8_t myBuffer[2];
  readRegisterRegion(myBuffer, LSM6DS3_ACC_GYRO_OUT_TEMP_L, 2);  //Does memory transfer

  //Do the math to get from raw numbers to useful int
  int16_t outputVariable = myBuffer[0] | (myBuffer[1] << 8);

  outputVariable = outputVariable >> 4; //divide by 16 to scale
  outputVariable += 25; //Add 25 degrees to remove offset

  //Calibration
  celsiusTemp = outputVariable;
  fahrenheitTemp = (celsiusTemp * 9) / 5 + 32;

  return fahrenheitTemp;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void LSM6DS3::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length) {
  //define pointer that will point to the external space
  uint8_t i = 0;
  char c = 0;
  uint8_t retryCounter = 0;

  switch (settings.commInterface) {
  
    case I2C_MODE:
      Wire.beginTransmission(settings.I2CAddress);
      Wire.write(offset);
      Wire.endTransmission();
  
      // request 6 bytes from slave device
      Wire.requestFrom(settings.I2CAddress, length);
      while ( (Wire.available()) && (i < length) && (retryCounter < RETRY_LIMIT))  // slave may send less than requested
      {
        c = Wire.read(); // receive a byte as character
        *outputPointer = c;
        outputPointer++;
        i++;
  	retryCounter++;
  
      }
      break;
  
    case SPI_MODE:
      // take the chip select low to select the device:
      digitalWrite(settings.chipSelectPin, LOW);
      // send the device the register you want to read:
      SPI.transfer(offset | 0x80);  //Ored with "read request" bit
      while ( (i < length) && (retryCounter < RETRY_LIMIT)) // slave may send less than requested
      {
  	Serial.println(retryCounter);
        c = SPI.transfer(0x00); // receive a byte as character
        *outputPointer = c;
        outputPointer++;
        i++;
  	retryCounter++;
      }
      // take the chip select high to de-select:
      digitalWrite(settings.chipSelectPin, HIGH);
      break;
  
    default:
      break;
  }
  if( retryCounter >= RETRY_LIMIT)
  {
    driverStatus = IMU_HW_ERROR;
  }
  else
  {
    driverStatus = IMU_SUCCESS;
  }
}



uint8_t LSM6DS3::readRegister(uint8_t offset) {
  //Return value
  uint8_t result;
  uint8_t numBytes = 1;
  switch (settings.commInterface) {

    case I2C_MODE:
      Wire.beginTransmission(settings.I2CAddress);
      Wire.write(offset);
      Wire.endTransmission();

      Wire.requestFrom(settings.I2CAddress, numBytes);
      while ( Wire.available() ) // slave may send less than requested
      {
        result = Wire.read(); // receive a byte as a proper uint8_t
      }
      break;

    case SPI_MODE:
      // take the chip select low to select the device:
      digitalWrite(settings.chipSelectPin, LOW);
      // send the device the register you want to read:
      SPI.transfer(offset | 0x80);  //Ored with "read request" bit
      // send a value of 0 to read the first byte returned:
      result = SPI.transfer(0x00);
      // take the chip select high to de-select:
      digitalWrite(settings.chipSelectPin, HIGH);
      break;

    default:
      break;
  }
  return result;
}

void LSM6DS3::writeRegister(uint8_t offset, uint8_t dataToWrite) {
  switch (settings.commInterface) {
    case I2C_MODE:
      //Write the byte
      Wire.beginTransmission(settings.I2CAddress);
      Wire.write(offset);
      Wire.write(dataToWrite);
      Wire.endTransmission();
      break;

    case SPI_MODE:
      // take the chip select low to select the device:
      digitalWrite(settings.chipSelectPin, LOW);
      // send the device the register you want to read:
      SPI.transfer(offset);
      // send a value of 0 to read the first byte returned:
      SPI.transfer(dataToWrite);
      // decrement the number of bytes left to read:
      // take the chip select high to de-select:
      digitalWrite(settings.chipSelectPin, HIGH);
      break;

    default:
      break;
  }
}

//****************************************************************************//
//
//  Error checking
//
//****************************************************************************//
status_t LSM6DS3::getStatus( void )
{
	//Move the configuration down a level
	if(settings.status == IMU_NOT_CONFIGURED)
	{
		driverStatus = IMU_NOT_CONFIGURED;
	}
	
	return driverStatus;
}