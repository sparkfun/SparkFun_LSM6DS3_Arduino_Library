/******************************************************************************
MemoryPagingExample.ino

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
This sketch switches between the base memory page and the embedded page.

The test writes to a base address, switches pages, writes to a embedded location
at the same numerical address, switches back and reads the original value.

This sketch doesn't do any meaningful configuration for the LSM6DS3, just tests.

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

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

uint16_t errorsAndWarnings = 0;

LSM6DS3Core myIMU( I2C_MODE, 0x6B );
//LSM6DS3Core myIMU( SPI_MODE, 10 );

void setup()
{
	Serial.begin(9600);
	delay(1000); //relax...
	Serial.println("Processor came out of reset.\n");

	//Call .beginCore() to configure the IMU
	if( myIMU.beginCore() != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	uint8_t dataVariable = 0;  //Temporary variable


	//Write something to a base page location to make sure it gets preserved
	//Then, read it back
	if( myIMU.writeRegister( LSM6DS3_ACC_GYRO_FIFO_CTRL1, 0xF0 ) != 0 )
	{
		errorsAndWarnings++;
	}
	if( myIMU.readRegister(&dataVariable, LSM6DS3_ACC_GYRO_FIFO_CTRL1) != 0 )
	{
		errorsAndWarnings++;
	}
	Serial.print("FIFO_CTRL1 (should read 0xF0): 0x");
	Serial.println(dataVariable, HEX);


	//Switch to the embedded page
	if( myIMU.embeddedPage() != 0 )
	{
		errorsAndWarnings++;
	}  

	//Write something to a the embedded page at the same address
	//Then, read it back
	if( myIMU.writeRegister( LSM6DS3_ACC_GYRO_SLV1_SUBADD, 0xA5 ) != 0 )
	{
		errorsAndWarnings++;
	}
	//Now read it back and display it
	if( myIMU.readRegister(&dataVariable, LSM6DS3_ACC_GYRO_SLV1_SUBADD) != 0 )
	{
		errorsAndWarnings++;
	}
	Serial.print("SUBADD (should read 0xA5): 0x");
	Serial.println(dataVariable, HEX);  

	//Switch back to the base page
	//Then, read back to see if our value has been preserved
	if( myIMU.basePage() != 0 )
	{
		errorsAndWarnings++;
	}  
	if( myIMU.readRegister(&dataVariable, LSM6DS3_ACC_GYRO_FIFO_CTRL1) != 0 )
	{
		errorsAndWarnings++;
	}
	Serial.print("FIFO_CTRL1 (should read 0xF0): 0x");
	Serial.println(dataVariable, HEX);

	Serial.print("Number of errors: ");
	Serial.println(errorsAndWarnings);
}


void loop()
{
	while(1);
}