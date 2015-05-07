#include "SparkFunIMU.h"
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU;

void setup() {
  Serial.begin(57600);
  myIMU.settings.commInterface = I2C_MODE;
  if ( myIMU.begin() != IMU_SUCCESS )
  {
    Serial.println("Error: Couldn't initialize IMU!");
    while (1);
  }
}

void loop() {
  float tempFloat;
  status_t statusTemp;
  tempFloat = myIMU.readAccelX();

  if ( myIMU.getStatus() == IMU_SUCCESS )
  {
    Serial.print("Accelerometer X data reported: ");
    Serial.println(tempFloat, 4);
  }
  else
  {
    Serial.println("Accelerometer not successfully read.");
  }

  delay(1000);

}
