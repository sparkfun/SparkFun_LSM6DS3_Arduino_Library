/******************************************************************************
InterruptDoubleTapConfig.ino

Example using the LSM6DS3 interrupts.

This sketch uses only the LSM6DS3Core methods to demonstrate the configuration
of the interrupts.

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

Optional:
Connect LEDs and current limiting resistors to INT1 and INT2 to detect
interrupts on a hardware level.

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.
******************************************************************************/
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

//uint16_t errorsAndWarnings = 0;

LSM6DS3Core myIMU( I2C_MODE, 0x6B );
//LSM6DS3Core myIMU( SPI_MODE, 10 );

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	delay(1000); //relax...
	Serial.println("Sketch came out of reset.\n");

	//Call .beginCore() to configure the IMU
	if( myIMU.beginCore() != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite = 0;  //Temporary variable

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

	// //Now, write the patched together data
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	errorAccumulator += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

	// Enable tap detection on X, Y, Z axis
	
	//In order to enable the latch feature on the single-tap interrupt signal,
	//both the LIR bit and the INT1_DOUBLE_TAP (or INT2_DOUBLE_TAP) bit of 
	//MD1_CFG (MD2_CFG) have to be set to 1: the interrupt is kept high until
	//the TAP_SRC register is read.
	
	// Enable tap detection on X, Y, Z axis
	// Write 0Fh into TAP_CFG
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0F );
	
	// Set tap threshold
	// Write 0Ch into TAP_THS_6D
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x0C );

	// Set Duration, Quiet and Shock time windows
	// Write 7Fh into INT_DUR2
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F );
	
	// Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
	// Write 80h into WAKE_UP_THS
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80 );
	
	// Single tap interrupt driven to INT1 pin -- and latch
	// Write 08h into MD1_CFG
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x08 );

	if( errorAccumulator )
	{
		Serial.println("Problem configuring the device.");
	}
	else
	{
		Serial.println("Device O.K.");
	}	
}


void loop()
{
	uint8_t readDataByte = 0;
	myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_TAP_SRC);
	if( readDataByte )
	{
		//debounce
		//delay(10);
		Serial.print("Interrupt caught.  TAP_SRC = 0x");
		Serial.println(readDataByte, HEX);
		if( readDataByte & 0x20 )
		{
			Serial.print("Single-tap event\n");
		}
		if( readDataByte & 0x10 )
		{
			Serial.print("Double-tap event\n");
		}
	}
	
}
