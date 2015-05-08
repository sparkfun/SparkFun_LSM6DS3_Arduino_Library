// Abstract class for SFE IMU products.  Provides interchangeable interface
// and serves as a template.

#ifndef __SPARKFUNIMU_H__
#define __SPARKFUNIMU_H__

#include "Arduino.h"

// Return values 
typedef enum
{
  IMU_SUCCESS,
  IMU_HW_ERROR,
  IMU_GENERIC_ERROR,
  IMU_OUT_OF_BOUNDS,
  IMU_NOT_CONFIGURED
  //...
} status_t;

class SparkFunIMU
{
  protected:
    status_t driverStatus; // Stores status of last operation

  public:
    // begin() returns up to 16 who_am_i bits.  If there is only one serial
    //   device the MSBs will be zero
    virtual status_t begin(void) = 0; // Must be defined in derived classes
	
    virtual float readGyroX()  { return NAN; }
    virtual float readGyroY()  { return NAN; }
    virtual float readGyroZ()  { return NAN; }
    virtual float readAccelX() { return NAN; }
    virtual float readAccelY() { return NAN; }
    virtual float readAccelZ() { return NAN; }
    virtual float readMagX()   { return NAN; }
    virtual float readMagY()   { return NAN; }
    virtual float readMagZ()   { return NAN; }
    virtual float readTempC()  { return NAN; }
    virtual float readTempF()  { return NAN; }

    status_t getStatus() { return driverStatus; }

    virtual ~SparkFunIMU() { }
};

#endif // End of __SPARKFUNIMU_H__ definition check
