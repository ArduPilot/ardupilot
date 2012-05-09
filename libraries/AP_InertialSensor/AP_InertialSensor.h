/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 */
class AP_InertialSensor
{
  public:
  AP_InertialSensor() {}

  virtual uint16_t init( AP_PeriodicProcess * scheduler ) = 0;

  /* Update the sensor data, so that getters are nonblocking.
   * Returns a bool of whether data was updated or not.
   */
  virtual bool update() = 0;

  // check if the sensors have new data
  virtual bool new_data_available(void) = 0;

  /* Getters for individual gyro axes.
   * Gyros have correct coordinate frame and units (degrees per second).
   */
  virtual float gx() = 0;
  virtual float gy() = 0;
  virtual float gz() = 0;

  /* Same data as above gyro getters, written to array as { gx, gy, gz } */
  virtual void get_gyros( float * ) = 0;

  /* Getters for individual accel axes.
   * Accels have correct coordinate frame ( flat level ax, ay = 0; az = -9.81)
   * and units (meters per second squared).
   */
  virtual float ax() = 0;
  virtual float ay() = 0;
  virtual float az() = 0;

  /* Same data as above accel getters, written to array as { ax, ay, az } */
  virtual void get_accels( float * ) = 0;

  /* Same data as above accel and gyro getters, written to array as
   * { gx, gy, gz, ax, ay, az }
   */
  virtual void get_sensors( float * ) = 0;

  /* Temperature, in degrees celsius, of the gyro. */
  virtual float temperature() = 0;

  /* sample_time returns the delta in microseconds since the
   * last call to reset_sample_time.
   */
  virtual uint32_t sample_time() = 0;
  virtual void reset_sample_time() = 0;

  // return the maximum gyro drift rate in radians/s/s. This
  // depends on what gyro chips are being used
  virtual float get_gyro_drift_rate(void) = 0;

};

#include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_Stub.h"

#endif // __AP_INERTIAL_SENSOR_H__
