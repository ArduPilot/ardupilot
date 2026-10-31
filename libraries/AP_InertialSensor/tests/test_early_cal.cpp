#include <AP_gtest.h>

/*
  regression tests for calibration requests received before
  AP_InertialSensor::init() has created the backends.  Such requests
  used to block forever in wait_for_sample() as there were no backends
  to provide the samples the calibration was waiting for.
 */

#include <AP_InertialSensor/AP_InertialSensor.h>

#if AP_INERTIALSENSOR_ENABLED

// with no backends created there are no gyros or accels to calibrate:
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(INS_Calibration, EarlyGyroCalibration)
{
    EXPECT_EQ(AP::ins().get_gyro_count(), 0);
    EXPECT_EQ(AP::ins().get_accel_count(), 0);
    // must fail immediately, without blocking in wait_for_sample()
    EXPECT_FALSE(AP::ins().calibrate_gyros());
    EXPECT_EQ(AP::ins().simple_accel_cal(), MAV_RESULT_TEMPORARILY_REJECTED);
}

#endif  // AP_INERTIALSENSOR_ENABLED

AP_GTEST_MAIN()
