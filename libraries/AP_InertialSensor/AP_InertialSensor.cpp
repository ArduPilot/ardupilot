#include <assert.h>

#include "AP_InertialSensor.h"

#if AP_INERTIALSENSOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/DSP.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#if !APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AP_Motors/AP_Motors_Class.h>
#endif
#include <GCS_MAVLink/GCS.h>

#include "AP_InertialSensor_BMI160.h"
#include "AP_InertialSensor_BMI270.h"
#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_L3G4200D.h"
#include "AP_InertialSensor_LSM9DS0.h"
#include "AP_InertialSensor_LSM9DS1.h"
#include "AP_InertialSensor_Invensense.h"
#include "AP_InertialSensor_SITL.h"
#include "AP_InertialSensor_RST.h"
#include "AP_InertialSensor_BMI055.h"
#include "AP_InertialSensor_BMI088.h"
#include "AP_InertialSensor_Invensensev2.h"
#include "AP_InertialSensor_ADIS1647x.h"
#include "AP_InertialSensor_ExternalAHRS.h"
#include "AP_InertialSensor_Invensensev3.h"
#include "AP_InertialSensor_NONE.h"

/* Define INS_TIMING_DEBUG to track down scheduling issues with the main loop.
 * Output is on the debug console. */
#ifdef INS_TIMING_DEBUG
#include <stdio.h>
#define timing_printf(fmt, args...)      do { printf("[timing] " fmt, ##args); } while(0)
#else
#define timing_printf(fmt, args...)
#endif

#ifndef HAL_DEFAULT_INS_FAST_SAMPLE
#define HAL_DEFAULT_INS_FAST_SAMPLE 1
#endif

extern const AP_HAL::HAL& hal;



#if APM_BUILD_COPTER_OR_HELI
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#define DEFAULT_STILL_THRESH 2.5f
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
#define DEFAULT_GYRO_FILTER  4
#define DEFAULT_ACCEL_FILTER 10
#define DEFAULT_STILL_THRESH 0.1f
#else
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) && CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // In steady-state level flight on SITL Plane, especially while the motor is off, the INS system
    // returns ins.is_still()==true. Baseline vibes while airborne are unrealistically low: around 0.07.
    // A real aircraft would be experiencing micro turbulence and be rocking around a tiny bit. Therefore,
    // for Plane SIM the vibe threshold needs to be a little lower. Since plane.is_flying() uses
    // ins.is_still() during gps loss to detect if we're flying, we want to make sure we are not "perfectly"
    // still in the air like we are on the ground.
    #define DEFAULT_STILL_THRESH 0.05f
#else
    #define DEFAULT_STILL_THRESH 0.1f
#endif
#endif

#if defined(STM32H7) || defined(STM32F7)
#define MPU_FIFO_FASTSAMPLE_DEFAULT 1
#else
#define MPU_FIFO_FASTSAMPLE_DEFAULT 0
#endif

#define GYRO_INIT_MAX_DIFF_DPS 0.1f

#ifndef HAL_INS_TRIM_LIMIT_DEG
#define HAL_INS_TRIM_LIMIT_DEG 10
#endif

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::var_info[] = {
    // 0 was PRODUCT_ID

    /*
      The following parameter indexes and reserved from previous use
      as accel offsets and scaling from before the 16g change in the
      PX4 backend:

      ACCSCAL : 1
      ACCOFFS : 2
      MPU6K_FILTER: 4
      ACC2SCAL : 5
      ACC2OFFS : 6
      ACC3SCAL : 8
      ACC3OFFS : 9
      CALSENSFRAME : 11
     */

    // @Param: _GYROFFS_X
    // @DisplayName: Gyro offsets of X axis
    // @Description: Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYROFFS_Y
    // @DisplayName: Gyro offsets of Y axis
    // @Description: Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYROFFS_Z
    // @DisplayName: Gyro offsets of Z axis
    // @Description: Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("_GYROFFS",     3, AP_InertialSensor, _gyro_offset_old_param[0],  0),

    // @Param: _GYR2OFFS_X
    // @DisplayName: Gyro2 offsets of X axis
    // @Description: Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYR2OFFS_Y
    // @DisplayName: Gyro2 offsets of Y axis
    // @Description: Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYR2OFFS_Z
    // @DisplayName: Gyro2 offsets of Z axis
    // @Description: Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_GYR2OFFS",    7, AP_InertialSensor, _gyro_offset_old_param[1],   0),
#endif

    // @Param: _GYR3OFFS_X
    // @DisplayName: Gyro3 offsets of X axis
    // @Description: Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYR3OFFS_Y
    // @DisplayName: Gyro3 offsets of Y axis
    // @Description: Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: _GYR3OFFS_Z
    // @DisplayName: Gyro3 offsets of Z axis
    // @Description: Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_GYR3OFFS",   10, AP_InertialSensor, _gyro_offset_old_param[2],   0),
#endif

    // @Param: _ACCSCAL_X
    // @DisplayName: Accelerometer scaling of X axis
    // @Description: Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACCSCAL_Y
    // @DisplayName: Accelerometer scaling of Y axis
    // @Description: Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACCSCAL_Z
    // @DisplayName: Accelerometer scaling of Z axis
    // @Description: Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("_ACCSCAL",     12, AP_InertialSensor, _accel_scale_old_param[0],  1.0),

    // @Param: _ACCOFFS_X
    // @DisplayName: Accelerometer offsets of X axis
    // @Description: Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACCOFFS_Y
    // @DisplayName: Accelerometer offsets of Y axis
    // @Description: Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACCOFFS_Z
    // @DisplayName: Accelerometer offsets of Z axis
    // @Description: Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("_ACCOFFS",     13, AP_InertialSensor, _accel_offset_old_param[0], 0),

    // @Param: _ACC2SCAL_X
    // @DisplayName: Accelerometer2 scaling of X axis
    // @Description: Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC2SCAL_Y
    // @DisplayName: Accelerometer2 scaling of Y axis
    // @Description: Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC2SCAL_Z
    // @DisplayName: Accelerometer2 scaling of Z axis
    // @Description: Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_ACC2SCAL",    14, AP_InertialSensor, _accel_scale_old_param[1],   1.0),
#endif

    // @Param: _ACC2OFFS_X
    // @DisplayName: Accelerometer2 offsets of X axis
    // @Description: Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC2OFFS_Y
    // @DisplayName: Accelerometer2 offsets of Y axis
    // @Description: Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC2OFFS_Z
    // @DisplayName: Accelerometer2 offsets of Z axis
    // @Description: Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_ACC2OFFS",    15, AP_InertialSensor, _accel_offset_old_param[1],  0),
#endif

    // @Param: _ACC3SCAL_X
    // @DisplayName: Accelerometer3 scaling of X axis
    // @Description: Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC3SCAL_Y
    // @DisplayName: Accelerometer3 scaling of Y axis
    // @Description: Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC3SCAL_Z
    // @DisplayName: Accelerometer3 scaling of Z axis
    // @Description: Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_ACC3SCAL",    16, AP_InertialSensor, _accel_scale_old_param[2],   1.0),
#endif

    // @Param: _ACC3OFFS_X
    // @DisplayName: Accelerometer3 offsets of X axis
    // @Description: Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC3OFFS_Y
    // @DisplayName: Accelerometer3 offsets of Y axis
    // @Description: Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: _ACC3OFFS_Z
    // @DisplayName: Accelerometer3 offsets of Z axis
    // @Description: Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_ACC3OFFS",    17, AP_InertialSensor, _accel_offset_old_param[2],  0),
#endif

    // @Param: _GYRO_FILTER
    // @DisplayName: Gyro filter cutoff frequency
    // @Description: Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 256
    // @User: Advanced
    AP_GROUPINFO("_GYRO_FILTER", 18, AP_InertialSensor, _gyro_filter_cutoff,  DEFAULT_GYRO_FILTER),

    // @Param: _ACCEL_FILTER
    // @DisplayName: Accel filter cutoff frequency
    // @Description: Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 256
    // @User: Advanced
    AP_GROUPINFO("_ACCEL_FILTER", 19, AP_InertialSensor, _accel_filter_cutoff,  DEFAULT_ACCEL_FILTER),

    // @Param: _USE
    // @DisplayName: Use first IMU for attitude, velocity and position estimates
    // @Description: Use first IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_USE", 20, AP_InertialSensor, _use_old_param[0],  1),

    // @Param: _USE2
    // @DisplayName: Use second IMU for attitude, velocity and position estimates
    // @Description: Use second IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_USE2", 21, AP_InertialSensor, _use_old_param[1],  1),
#endif

    // @Param: _USE3
    // @DisplayName: Use third IMU for attitude, velocity and position estimates
    // @Description: Use third IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_USE3", 22, AP_InertialSensor, _use_old_param[2],  1),
#endif

    // @Param: _STILL_THRESH
    // @DisplayName: Stillness threshold for detecting if we are moving
    // @Description: Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5
    // @Range: 0.05 50
    // @User: Advanced
    AP_GROUPINFO("_STILL_THRESH", 23, AP_InertialSensor, _still_threshold,  DEFAULT_STILL_THRESH),

    // @Param: _GYR_CAL
    // @DisplayName: Gyro Calibration scheme
    // @Description: Conrols when automatic gyro calibration is performed
    // @Values: 0:Never, 1:Start-up only
    // @User: Advanced
    AP_GROUPINFO("_GYR_CAL", 24, AP_InertialSensor, _gyro_cal_timing, 1),

    // @Param: _TRIM_OPTION
    // @DisplayName: Accel cal trim option
    // @Description: Specifies how the accel cal routine determines the trims
    // @User: Advanced
    // @Values: 0:Don't adjust the trims,1:Assume first orientation was level,2:Assume ACC_BODYFIX is perfectly aligned to the vehicle
    AP_GROUPINFO("_TRIM_OPTION", 25, AP_InertialSensor, _trim_option, 1),

    // @Param: _ACC_BODYFIX
    // @DisplayName: Body-fixed accelerometer
    // @Description: The body-fixed accelerometer to be used for trim calculation
    // @User: Advanced
    // @Values: 1:IMU 1,2:IMU 2,3:IMU 3
    AP_GROUPINFO("_ACC_BODYFIX", 26, AP_InertialSensor, _acc_body_aligned, 2),

    // @Param: _POS1_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS1_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS1_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_POS1", 27, AP_InertialSensor, _accel_pos_old_param[0], 0.0f),

    // @Param: _POS2_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the second IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS2_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the second IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS2_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the second IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_POS2", 28, AP_InertialSensor, _accel_pos_old_param[1], 0.0f),
#endif

    // @Param: _POS3_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the third IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: _POS3_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the third IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS3_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the third IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_POS3", 29, AP_InertialSensor, _accel_pos_old_param[2], 0.0f),
#endif

    // @Param: _GYR_ID
    // @DisplayName: Gyro ID
    // @Description: Gyro sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("_GYR_ID", 30, AP_InertialSensor, _gyro_id_old_param[0], 0),

    // @Param: _GYR2_ID
    // @DisplayName: Gyro2 ID
    // @Description: Gyro2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_GYR2_ID", 31, AP_InertialSensor, _gyro_id_old_param[1], 0),
#endif

    // @Param: _GYR3_ID
    // @DisplayName: Gyro3 ID
    // @Description: Gyro3 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_GYR3_ID", 32, AP_InertialSensor, _gyro_id_old_param[2], 0),
#endif

    // @Param: _ACC_ID
    // @DisplayName: Accelerometer ID
    // @Description: Accelerometer sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("_ACC_ID", 33, AP_InertialSensor, _accel_id_old_param[0], 0),

    // @Param: _ACC2_ID
    // @DisplayName: Accelerometer2 ID
    // @Description: Accelerometer2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced

#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("_ACC2_ID", 34, AP_InertialSensor, _accel_id_old_param[1], 0),
#endif

    // @Param: _ACC3_ID
    // @DisplayName: Accelerometer3 ID
    // @Description: Accelerometer3 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced

#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("_ACC3_ID", 35, AP_InertialSensor, _accel_id_old_param[2], 0),
#endif

    // @Param: _FAST_SAMPLE
    // @DisplayName: Fast sampling mask
    // @Description: Mask of IMUs to enable fast sampling on, if available
    // @User: Advanced
    // @Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU
    AP_GROUPINFO("_FAST_SAMPLE",  36, AP_InertialSensor, _fast_sampling_mask,   HAL_DEFAULT_INS_FAST_SAMPLE),

    // index 37 was NOTCH_

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    // @Group: _LOG_
    // @Path: ../AP_InertialSensor/BatchSampler.cpp
    AP_SUBGROUPINFO(batchsampler, "_LOG_",  39, AP_InertialSensor, AP_InertialSensor::BatchSampler),
#endif

    // @Param: _ENABLE_MASK
    // @DisplayName: IMU enable mask
    // @Description: Bitmask of IMUs to enable. It can be used to prevent startup of specific detected IMUs
    // @User: Advanced
    // @Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU,6:SeventhIMU
    AP_GROUPINFO("_ENABLE_MASK",  40, AP_InertialSensor, _enable_mask, 0x7F),

    // @Group: _HNTCH_
    // @Path: ../Filter/HarmonicNotchFilter.cpp
    AP_SUBGROUPINFO(harmonic_notches[0].params, "_HNTCH_",  41, AP_InertialSensor, HarmonicNotchFilterParams),

#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    // @Group: _HNTC2_
    // @Path: ../Filter/HarmonicNotchFilter.cpp
    AP_SUBGROUPINFO(harmonic_notches[1].params, "_HNTC2_",  53, AP_InertialSensor, HarmonicNotchFilterParams),
#endif

    // @Param: _GYRO_RATE
    // @DisplayName: Gyro rate for IMUs with Fast Sampling enabled
    // @Description: Gyro rate for IMUs with fast sampling enabled. The gyro rate is the sample rate at which the IMU filters operate and needs to be at least double the maximum filter frequency. If the sensor does not support the selected rate the next highest supported rate will be used. For IMUs which do not support fast sampling this setting is ignored and the default gyro rate of 1Khz is used.
    // @User: Advanced
    // @Values: 0:1kHz,1:2kHz,2:4kHz,3:8kHz
    // @RebootRequired: True
    AP_GROUPINFO("_GYRO_RATE",  42, AP_InertialSensor, _fast_sampling_rate, MPU_FIFO_FASTSAMPLE_DEFAULT),


#if HAL_INS_TEMPERATURE_CAL_ENABLE
    // @Group: _TCAL1_
    // @Path: AP_InertialSensor_tempcal.cpp
    AP_SUBGROUPINFO(tcal_old_param[0], "_TCAL1_", 43, AP_InertialSensor, AP_InertialSensor_TCal),

#if INS_MAX_INSTANCES > 1
    // @Group: _TCAL2_
    // @Path: AP_InertialSensor_tempcal.cpp
    AP_SUBGROUPINFO(tcal_old_param[1], "_TCAL2_", 44, AP_InertialSensor, AP_InertialSensor_TCal),
#endif

#if INS_MAX_INSTANCES > 2
    // @Group: _TCAL3_
    // @Path: AP_InertialSensor_tempcal.cpp
    AP_SUBGROUPINFO(tcal_old_param[2], "_TCAL3_", 45, AP_InertialSensor, AP_InertialSensor_TCal),
#endif

    // @Param: _ACC1_CALTEMP
    // @DisplayName: Calibration temperature for 1st accelerometer
    // @Description: Temperature that the 1st accelerometer was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_ACC1_CALTEMP", 46, AP_InertialSensor, caltemp_accel_old_param[0], -300),

    // @Param: _GYR1_CALTEMP
    // @DisplayName: Calibration temperature for 1st gyroscope
    // @Description: Temperature that the 1st gyroscope was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_GYR1_CALTEMP", 47, AP_InertialSensor, caltemp_gyro_old_param[0], -300),

#if INS_MAX_INSTANCES > 1
    // @Param: _ACC2_CALTEMP
    // @DisplayName: Calibration temperature for 2nd accelerometer
    // @Description: Temperature that the 2nd accelerometer was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_ACC2_CALTEMP", 48, AP_InertialSensor, caltemp_accel_old_param[1], -300),

    // @Param: _GYR2_CALTEMP
    // @DisplayName: Calibration temperature for 2nd gyroscope
    // @Description: Temperature that the 2nd gyroscope was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_GYR2_CALTEMP", 49, AP_InertialSensor, caltemp_gyro_old_param[1], -300),
#endif

#if INS_MAX_INSTANCES > 2
    // @Param: _ACC3_CALTEMP
    // @DisplayName: Calibration temperature for 3rd accelerometer
    // @Description: Temperature that the 3rd accelerometer was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_ACC3_CALTEMP", 50, AP_InertialSensor, caltemp_accel_old_param[2], -300),

    // @Param: _GYR3_CALTEMP
    // @DisplayName: Calibration temperature for 3rd gyroscope
    // @Description: Temperature that the 3rd gyroscope was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("_GYR3_CALTEMP", 51, AP_InertialSensor, caltemp_gyro_old_param[2], -300),
#endif

    // @Param: _TCAL_OPTIONS
    // @DisplayName: Options for temperature calibration
    // @Description: This enables optional temperature calibration features. Setting PersistParams will save the accelerometer and temperature calibration parameters in the bootloader sector on the next update of the bootloader.
    // @Bitmask: 0:PersistParams
    // @User: Advanced
    AP_GROUPINFO("_TCAL_OPTIONS", 52, AP_InertialSensor, tcal_options, 0),
    
#endif // HAL_INS_TEMPERATURE_CAL_ENABLE


#if INS_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_InertialSensor_Params.cpp
    AP_SUBGROUPINFO(params[0], "4_", 54, AP_InertialSensor, AP_InertialSensor_Params),
#endif

#if INS_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_InertialSensor_Params.cpp
    AP_SUBGROUPINFO(params[1], "5_", 55, AP_InertialSensor, AP_InertialSensor_Params),
#endif

    /*
      NOTE: parameter indexes have gaps above. When adding new
      parameters check for conflicts carefully
     */
    AP_GROUPEND
};

AP_InertialSensor *AP_InertialSensor::_singleton = nullptr;

AP_InertialSensor::AP_InertialSensor() :
    _board_orientation(ROTATION_NONE),
    _log_raw_bit(-1)
{
    if (_singleton) {
        AP_HAL::panic("Too many inertial sensors");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _gyro_cal_ok[i] = true;
        _accel_max_abs_offsets[i] = 3.5f;
    }
    for (uint8_t i=0; i<INS_VIBRATION_CHECK_INSTANCES; i++) {
        _accel_vibe_floor_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ);
        _accel_vibe_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ);
    }
#if HAL_INS_ACCELCAL_ENABLED
    AP_AccelCal::register_client(this);
#endif
}

/*
 * Get the AP_InertialSensor singleton
 */
AP_InertialSensor *AP_InertialSensor::get_singleton()
{
    if (!_singleton) {
        _singleton = new AP_InertialSensor();
    }
    return _singleton;
}

/*
  register a new gyro instance
 */
bool AP_InertialSensor::register_gyro(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id)
{
    if (_gyro_count == INS_MAX_INSTANCES) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to register gyro id %u", unsigned(id));
        return false;
    }

    _gyro_raw_sample_rates[_gyro_count] = raw_sample_rate_hz;
    _gyro_over_sampling[_gyro_count] = 1;
    _gyro_raw_sampling_multiplier[_gyro_count] = INT16_MAX/radians(2000);

    bool saved = _gyro_id(_gyro_count).load();

    if (saved && (uint32_t)_gyro_id(_gyro_count) != id) {
        // inconsistent gyro id - mark it as needing calibration
        _gyro_cal_ok[_gyro_count] = false;
    }

    _gyro_id(_gyro_count).set((int32_t) id);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!saved) {
        // assume this is the same sensor and save its ID to allow seamless
        // transition from when we didn't have the IDs.
        _gyro_id(_gyro_count).save();
    }
#endif

    instance = _gyro_count++;

    return true;
}

/*
  register a new accel instance
 */
bool AP_InertialSensor::register_accel(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id)
{
    if (_accel_count == INS_MAX_INSTANCES) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to register accel id %u", unsigned(id));
        return false;
    }

    _accel_raw_sample_rates[_accel_count] = raw_sample_rate_hz;
    _accel_over_sampling[_accel_count] = 1;
    _accel_raw_sampling_multiplier[_accel_count] = INT16_MAX/(16*GRAVITY_MSS);

    bool saved = _accel_id(_accel_count).load();

    if (!saved) {
        // inconsistent accel id
        _accel_id_ok[_accel_count] = false;
    } else if ((uint32_t)_accel_id(_accel_count) != id) {
        // inconsistent accel id
        _accel_id_ok[_accel_count] = false;
    } else {
        _accel_id_ok[_accel_count] = true;
    }

    _accel_id(_accel_count).set((int32_t) id);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_SIM_ENABLED)
        // assume this is the same sensor and save its ID to allow seamless
        // transition from when we didn't have the IDs.
        _accel_id_ok[_accel_count] = true;
        _accel_id(_accel_count).save();
#endif

    instance = _accel_count++;
    return true;
}

/*
 * Start all backends for gyro and accel measurements. It automatically calls
 * detect_backends() if it has not been called already.
 */
void AP_InertialSensor::_start_backends()

{
    detect_backends();

    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->start();
    }

    if (_gyro_count == 0 || _accel_count == 0) {
        AP_HAL::panic("INS needs at least 1 gyro and 1 accel");
    }

    // clear IDs for unused sensor instances
    for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
        _accel_id(i).set(0);
    }
    for (uint8_t i=get_gyro_count(); i<INS_MAX_INSTANCES; i++) {
        _gyro_id(i).set(0);
    }
}

/* Find the N instance of the backend that has already been successfully detected */
AP_InertialSensor_Backend *AP_InertialSensor::_find_backend(int16_t backend_id, uint8_t instance)
{
    uint8_t found = 0;

    for (uint8_t i = 0; i < _backend_count; i++) {
        int16_t id = _backends[i]->get_id();

        if (id < 0 || id != backend_id) {
            continue;
        }

        if (instance == found) {
            return _backends[i];
        } else {
            found++;
        }
    }

    return nullptr;
}

bool AP_InertialSensor::set_gyro_window_size(uint16_t size) {
#if HAL_GYROFFT_ENABLED
    _gyro_window_size = size;

    // allocate FFT gyro window
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        for (uint8_t j = 0; j < XYZ_AXIS_COUNT; j++) {
            if (!_gyro_window[i][j].set_size(size)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to allocate window for INS");
                // clean up whatever we have currently allocated
                for (uint8_t ii = 0; ii <= i; ii++) {
                    for (uint8_t jj = 0; jj < j; jj++) {
                        _gyro_window[ii][jj].set_size(0);
                        _gyro_window_size = 0;
                    }
                }
                return false;
            }
        }
    }
#endif
    return true;
}

#if HAL_WITH_DSP
bool AP_InertialSensor::has_fft_notch() const
{
    for (auto &notch : harmonic_notches) {
        if (notch.params.enabled() && notch.params.tracking_mode() == HarmonicNotchDynamicMode::UpdateGyroFFT) {
            return true;
        }
    }
    return false;
}
#endif

void
AP_InertialSensor::init(uint16_t loop_rate)
{
    // remember the sample rate
    _loop_rate = loop_rate;
    _loop_delta_t = 1.0f / loop_rate;

    // we don't allow deltat values greater than 10x the normal loop
    // time to be exposed outside of INS. Large deltat values can
    // cause divergence of state estimators
    _loop_delta_t_max = 10 * _loop_delta_t;

    if (_gyro_count == 0 && _accel_count == 0) {
        _start_backends();
    }

    // calibrate gyros unless gyro calibration has been disabled
    if (gyro_calibration_timing() != GYRO_CAL_NEVER) {
        init_gyro();
    }

    _sample_period_usec = 1000*1000UL / _loop_rate;

    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    // initialise IMU batch logging
    batchsampler.init();
#endif

#if HAL_GYROFFT_ENABLED
    AP_GyroFFT* fft = AP::fft();
    bool fft_enabled = fft != nullptr && fft->enabled();
    if (fft_enabled) {
        _post_filter_fft = fft->using_post_filter_samples();
    }

    // calculate the position that the FFT window needs to be applied
    // Use cases:
    //  Gyro -> FFT window -> FFT Notch1/2 -> Non-FFT Notch2/1 -> LPF -> Filtered Gyro -- Phase 0
    //  Gyro -> FFT window -> Non-FFT Notch1/2 -> LPF -> Filtered Gyro  -- Phase 0
    //  Gyro -> Non-FFT Notch1 -> Filtered FFT Window -> FFT Notch2 -> LPF -> Filtered Gyro -- Phase 1
    //  Gyro -> Non-FFT Notch1/2 -> Non-FFT Notch1/2 -> Filtered FFT Window -> LPF -> Filtered Gyro -- Phase 2
    //  Gyro -> Non-FFT Notch1/2 -> Filtered FFT Window -> LPF -> Filtered Gyro -- Phase 1
    //  Gyro -> Filtered FFT Window -> LPF -> Filtered Gyro -- Phase 0
    //  Gyro -> FFT window -> LPF -> Filtered Gyro -- Phase 0
    //  Gyro -> Notch1/2 -> LPF -> Filtered Gyro

    if (_post_filter_fft) {
        for (auto &notch : harmonic_notches) {
            if (!notch.params.enabled()) {
                continue;
            }
            // window must always come before any FFT notch
            if (notch.params.tracking_mode() == HarmonicNotchDynamicMode::UpdateGyroFFT) {
                break;
            }
            _fft_window_phase++;
        }
    }
#else
    bool fft_enabled = false;
#endif
    // the center frequency of the harmonic notch is always taken from the calculated value so that it can be updated
    // dynamically, the calculated value is always some multiple of the configured center frequency, so start with the
    // configured value
    for (auto &notch : harmonic_notches) {
        if (!notch.params.enabled() && !fft_enabled) {
            continue;
        }
        notch.calculated_notch_freq_hz[0] = notch.params.center_freq_hz();
        notch.num_calculated_notch_frequencies = 1;
        notch.num_dynamic_notches = 1;
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        if (notch.params.hasOption(HarmonicNotchFilterParams::Options::DynamicHarmonic)) {
#if HAL_GYROFFT_ENABLED
            if (notch.params.tracking_mode() == HarmonicNotchDynamicMode::UpdateGyroFFT) {
                notch.num_dynamic_notches = AP_HAL::DSP::MAX_TRACKED_PEAKS; // only 3 peaks supported currently
            } else
#endif
            {
                AP_Motors *motors = AP::motors();
                if (motors != nullptr) {
                    notch.num_dynamic_notches = __builtin_popcount(motors->get_motor_mask());
                }
            }
            // avoid harmonics unless actually configured by the user
            notch.params.set_default_harmonics(1);
        }
#endif
    }
    // count number of used sensors
    uint8_t sensors_used = 0;
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        sensors_used += _use(i);
    }

    uint8_t num_filters = 0;
    for (auto &notch : harmonic_notches) {
        // calculate number of notches we might want to use for harmonic notch
        if (notch.params.enabled() || fft_enabled) {
            const bool double_notch = notch.params.hasOption(HarmonicNotchFilterParams::Options::DoubleNotch);
            const bool triple_notch = notch.params.hasOption(HarmonicNotchFilterParams::Options::TripleNotch);
            const bool all_sensors = notch.params.hasOption(HarmonicNotchFilterParams::Options::EnableOnAllIMUs);
            num_filters += __builtin_popcount(notch.params.harmonics())
                * notch.num_dynamic_notches * (double_notch ? 2 : triple_notch ? 3 : 1)
                * (all_sensors?sensors_used:1);
        }
    }

    if (num_filters > HAL_HNF_MAX_FILTERS) {
        AP_BoardConfig::config_error("Too many notches: %u > %u", num_filters, HAL_HNF_MAX_FILTERS);
    }

    // allocate notches
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        // only allocate notches for IMUs in use
        if (_use(i)) {
            for (auto &notch : harmonic_notches) {
                if (notch.params.enabled() || fft_enabled) {
                    const bool double_notch = notch.params.hasOption(HarmonicNotchFilterParams::Options::DoubleNotch);
                    const bool triple_notch = notch.params.hasOption(HarmonicNotchFilterParams::Options::TripleNotch);
                    notch.filter[i].allocate_filters(notch.num_dynamic_notches,
                                                     notch.params.harmonics(), double_notch ? 2 : triple_notch ? 3 : 1);
                    // initialise default settings, these will be subsequently changed in AP_InertialSensor_Backend::update_gyro()
                    notch.filter[i].init(_gyro_raw_sample_rates[i], notch.calculated_notch_freq_hz[0],
                                         notch.params.bandwidth_hz(), notch.params.attenuation_dB());
                }
            }
        }
    }

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    /*
      see if user has setup for on-boot enable of temperature learning
     */
    if (temperature_cal_running()) {
        tcal_learning = true;
    }
#endif
}

bool AP_InertialSensor::_add_backend(AP_InertialSensor_Backend *backend)
{

    if (!backend) {
        return false;
    }
    if (_backend_count == INS_MAX_BACKENDS) {
        AP_HAL::panic("Too many INS backends");
    }
    _backends[_backend_count++] = backend;
    return true;
}

/*
  detect available backends for this board
 */
void
AP_InertialSensor::detect_backends(void)
{
    if (_backends_detected) {
        return;
    }

    _backends_detected = true;

#if defined(HAL_CHIBIOS_ARCH_CUBE) && INS_MAX_INSTANCES > 2
    // special case for Cubes, where the IMUs on the isolated
    // board could fail on some boards. If the user has INS_USE=1,
    // INS_USE2=1 and INS_USE3=0 then force INS_USE3 to 1. This is
    // done as users loading past parameter files may end up with
    // INS_USE3=0 unintentionally, which is unsafe on these
    // boards. For users who really want limited IMUs they will need
    // to either use the INS_ENABLE_MASK or set INS_USE2=0 which will
    // enable the first IMU without triggering this check
    if (_use(0) == 1 && _use(1) == 1 && _use(2) == 0) {
        _use(2).set(1);
    }
#endif

    uint8_t probe_count __attribute__((unused)) = 0;
    uint8_t enable_mask __attribute__((unused)) = uint8_t(_enable_mask.get());
    uint8_t found_mask __attribute__((unused)) = 0;

    /*
      use ADD_BACKEND() macro to allow for INS_ENABLE_MASK for enabling/disabling INS backends
     */
#define ADD_BACKEND(x) do { \
        if (((1U<<probe_count)&enable_mask) && _add_backend(x)) { \
            found_mask |= (1U<<probe_count); \
        } \
        probe_count++; \
} while (0)

// Can be used by adding INSTANCE:<num> keyword in hwdef.
// This keyword is used to denote the instance number of the sensor
// while probing. Probing is skipped if the instance number doesn't match the
// backend count. Its important the IMUs are listed in order of precedence globally
// (i.e. INSTANCE:0 IMUs are listed before INSTANCE:1 IMUs) and locally (i.e. IMUs
// on the same bus are listed in order of detection precedence)

#define ADD_BACKEND_INSTANCE(x, instance) if (instance == _backend_count) { ADD_BACKEND(x); }

// Can be used by adding AUX:<devid> keyword in hwdef.
// AUX:<devid> keyword is used to check for the presence of the sensor
// in the detected IMUs list. If the IMU with the given devid is found
// then we skip the probe for the sensor the second time. This is useful
// if you have multiple choices for IMU over same instance number, and still 
// want to instantiate the sensor after main IMUs are detected.

#define ADD_BACKEND_AUX(x, devid) do { \
        bool init_aux = true; \
        for (uint8_t i=0; i<_backend_count; i++) { \
            if (((uint32_t)_accel_id(i) == devid) || ((uint32_t)_gyro_id(i) == devid)) { \
                init_aux = false; \
            } \
        } \
        if (init_aux) { \
            ADD_BACKEND(x); \
        } \
} while (0)

// support for adding IMUs conditioned on board type
#define BOARD_MATCH(board_type) AP_BoardConfig::get_board_type()==AP_BoardConfig::board_type
#define ADD_BACKEND_BOARD_MATCH(board_match, x) do { if (board_match) { ADD_BACKEND(x); } } while(0)

// macro for use by HAL_INS_PROBE_LIST
#define GET_I2C_DEVICE(bus, address) hal.i2c_mgr->get_device(bus, address)

#if HAL_EXTERNAL_AHRS_ENABLED
    // if enabled, make the first IMU the external AHRS
    const int8_t serial_port = AP::externalAHRS().get_port(AP_ExternalAHRS::AvailableSensor::IMU);
    if (serial_port >= 0) {
        ADD_BACKEND(new AP_InertialSensor_ExternalAHRS(*this, serial_port));
    }
#endif

#if AP_SIM_INS_ENABLED
    for (uint8_t i=0; i<AP::sitl()->imu_count; i++) {
        ADD_BACKEND(AP_InertialSensor_SITL::detect(*this, i==1?INS_SITL_SENSOR_B:INS_SITL_SENSOR_A));
    }
    return;
#endif

#if defined(HAL_INS_PROBE_LIST)
    // IMUs defined by IMU lines in hwdef.dat
    HAL_INS_PROBE_LIST;
#if defined(HAL_SITL_INVENSENSEV3)
    ADD_BACKEND(AP_InertialSensor_Invensensev3::probe(*this, hal.i2c_mgr->get_device(1, 1), ROTATION_NONE));
#endif
#elif AP_FEATURE_BOARD_DETECT
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_NONE));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_ROLL_180));
        ADD_BACKEND(AP_InertialSensor_LSM9DS0::probe(*this,
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_G_NAME),
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME),
                                                      ROTATION_ROLL_180,
                                                      ROTATION_ROLL_180_YAW_270,
                                                      ROTATION_PITCH_180));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        // older Pixhawk2 boards have the MPU6000 instead of MPU9250
        _fast_sampling_mask.set_default(1);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_EXT_NAME), ROTATION_PITCH_180));
        ADD_BACKEND(AP_InertialSensor_LSM9DS0::probe(*this,
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_EXT_G_NAME),
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_EXT_A_NAME),
                                                      ROTATION_ROLL_180_YAW_270,
                                                      ROTATION_ROLL_180_YAW_90,
                                                      ROTATION_ROLL_180_YAW_90));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_YAW_270));
        // new cubes have ICM20602, ICM20948, ICM20649
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device("icm20602_ext"), ROTATION_ROLL_180_YAW_270));
        ADD_BACKEND(AP_InertialSensor_Invensensev2::probe(*this, hal.spi->get_device("icm20948_ext"), ROTATION_PITCH_180));
        ADD_BACKEND(AP_InertialSensor_Invensensev2::probe(*this, hal.spi->get_device("icm20948"), ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
        _fast_sampling_mask.set_default(1);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device("icm20689"), ROTATION_NONE));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device("icm20602"), ROTATION_NONE));
        // allow for either BMI055 or BMI088
        ADD_BACKEND(AP_InertialSensor_BMI055::probe(*this,
                                                    hal.spi->get_device("bmi055_a"),
                                                    hal.spi->get_device("bmi055_g"),
                                                    ROTATION_ROLL_180_YAW_90));
        ADD_BACKEND(AP_InertialSensor_BMI088::probe(*this,
                                                    hal.spi->get_device("bmi055_a"),
                                                    hal.spi->get_device("bmi055_g"),
                                                    ROTATION_ROLL_180_YAW_90));
        break;
        
    case AP_BoardConfig::PX4_BOARD_SP01:
        _fast_sampling_mask.set_default(1);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_EXT_NAME), ROTATION_NONE));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_NONE));
        break;
        
    case AP_BoardConfig::PX4_BOARD_PIXHAWK_PRO:
        _fast_sampling_mask.set_default(3);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_NAME), ROTATION_ROLL_180_YAW_90));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180_YAW_90));
        break;		

    case AP_BoardConfig::PX4_BOARD_PHMINI:
        // PHMINI uses ICM20608 on the ACCEL_MAG device and a MPU9250 on the old MPU6000 CS line
        _fast_sampling_mask.set_default(3);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_AM_NAME), ROTATION_ROLL_180));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180));
        break;

    case AP_BoardConfig::PX4_BOARD_AUAV21:
        // AUAV2.1 uses ICM20608 on the ACCEL_MAG device and a MPU9250 on the old MPU6000 CS line
        _fast_sampling_mask.set_default(3);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_AM_NAME), ROTATION_ROLL_180_YAW_90));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180_YAW_90));
        break;
        
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
        _fast_sampling_mask.set_default(1);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_AEROFC:
        _fast_sampling_mask.set_default(1);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU6500_NAME), ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_MINDPXV2:
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU6500_NAME), ROTATION_NONE));
        ADD_BACKEND(AP_InertialSensor_LSM9DS0::probe(*this,
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_G_NAME),
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME),
                                                      ROTATION_YAW_90,
                                                      ROTATION_YAW_90,
                                                      ROTATION_YAW_90));
        break;
        
    case AP_BoardConfig::VRX_BOARD_BRAIN54:
        _fast_sampling_mask.set_default(7);
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_YAW_180));
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_EXT_NAME), ROTATION_YAW_180));
#ifdef HAL_INS_MPU60x0_IMU_NAME
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_IMU_NAME), ROTATION_YAW_180));
#endif
        break;

    case AP_BoardConfig::VRX_BOARD_BRAIN51:
    case AP_BoardConfig::VRX_BOARD_BRAIN52:
    case AP_BoardConfig::VRX_BOARD_BRAIN52E:
    case AP_BoardConfig::VRX_BOARD_CORE10:
    case AP_BoardConfig::VRX_BOARD_UBRAIN51:
    case AP_BoardConfig::VRX_BOARD_UBRAIN52:
        ADD_BACKEND(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_YAW_180));
        break;
        
    case AP_BoardConfig::PX4_BOARD_PCNC1:
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_ROLL_180));
        break;

    default:
        break;
    }
#elif HAL_INS_DEFAULT == HAL_INS_NONE
    // no INS device
#else
    #error Unrecognised HAL_INS_TYPE setting
#endif

    if (_backend_count == 0) {

        // no real INS backends avail, lets use an empty substitute to boot ok and get to mavlink
        #if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
        ADD_BACKEND(AP_InertialSensor_NONE::detect(*this, INS_NONE_SENSOR_A));
        #else
        DEV_PRINTF("INS: unable to initialise driver\n");
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "INS: unable to initialise driver");
        AP_BoardConfig::config_error("INS: unable to initialise driver");
        #endif
    }
}

// Armed, Copter, PixHawk:
// ins_periodic: 57500 events, 0 overruns, 208754us elapsed, 3us avg, min 1us max 218us 40.662us rms
void AP_InertialSensor::periodic()
{
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    batchsampler.periodic();
#endif
}


/*
  _calculate_trim - calculates the x and y trim angles. The
  accel_sample must be correctly scaled, offset and oriented for the
  board

  Note that this only changes 2 axes of the trim vector. When in
  ROTATION_NONE view we can calculate the x and y trim. When in
  ROTATION_PITCH_90 for tailsitters we can calculate y and z. This
  allows users to trim for both flight orientations by doing two trim
  operations, one at each orientation

  When doing a full accel cal we pass in a trim vector that has been
  zeroed so the 3rd non-observable axis is reset
*/
bool AP_InertialSensor::_calculate_trim(const Vector3f &accel_sample, Vector3f &trim)
{
    Rotation rotation = ROTATION_NONE;
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AP_AHRS_View *view = AP::ahrs().get_view();
    if (view != nullptr) {
        // Use pitch to guess which axis the user is trying to trim
        // 5 deg buffer to favor normal AHRS and avoid floating point funny business
        if (fabsf(view->pitch) < (fabsf(AP::ahrs().pitch)+radians(5)) ) {
            // user is trying to calibrate view
            rotation = view->get_rotation();
            if (!is_zero(view->get_pitch_trim())) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Cannot calibrate with Q_TRIM_PITCH set");
                return false;
            }
        }
    }
#endif

    Vector3f newtrim = trim;
    switch (rotation) {
    case ROTATION_NONE:
        newtrim.y = atan2f(accel_sample.x, norm(accel_sample.y, accel_sample.z));
        newtrim.x = atan2f(-accel_sample.y, -accel_sample.z);
        break;

    case ROTATION_PITCH_90: {
        newtrim.y = atan2f(accel_sample.z, norm(accel_sample.y, -accel_sample.x));
        newtrim.z = atan2f(-accel_sample.y, accel_sample.x);
        break;
    }
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "unsupported trim rotation");
        return false;
    }
    if (fabsf(newtrim.x) <= radians(HAL_INS_TRIM_LIMIT_DEG) &&
        fabsf(newtrim.y) <= radians(HAL_INS_TRIM_LIMIT_DEG) &&
        fabsf(newtrim.z) <= radians(HAL_INS_TRIM_LIMIT_DEG)) {
        trim = newtrim;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Trim OK: roll=%.2f pitch=%.2f yaw=%.2f",
                        (double)degrees(trim.x),
                        (double)degrees(trim.y),
                        (double)degrees(trim.z));
        return true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "trim over maximum of 10 degrees");
    return false;
}

void
AP_InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_gyro_calibration();
}

// output GCS startup messages
bool AP_InertialSensor::get_output_banner(uint8_t backend_id, char* banner, uint8_t banner_len)
{
    if (backend_id >= INS_MAX_BACKENDS || _backends[backend_id] == nullptr) {
        return false;
    }

    return _backends[backend_id]->get_output_banner(banner, banner_len);
}

// accelerometer clipping reporting
uint32_t AP_InertialSensor::get_accel_clip_count(uint8_t instance) const
{
    if (instance >= get_accel_count()) {
        return 0;
    }
    return _accel_clip_count[instance];
}

// get_gyro_health_all - return true if all gyros are healthy
bool AP_InertialSensor::get_gyro_health_all(void) const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!get_gyro_health(i)) {
            return false;
        }
    }
    // return true if we have at least one gyro
    return (get_gyro_count() > 0);
}

// gyro_calibration_ok_all - returns true if all gyros were calibrated successfully
bool AP_InertialSensor::gyro_calibrated_ok_all() const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!gyro_calibrated_ok(i)) {
            return false;
        }
    }
    for (uint8_t i=get_gyro_count(); i<INS_MAX_INSTANCES; i++) {
        if (_gyro_id(i) != 0) {
            // missing gyro
            return false;
        }
    }
    return (get_gyro_count() > 0);
}

// return true if gyro instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_InertialSensor::use_gyro(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_gyro_health(instance) && _use(instance));
}

// get_accel_health_all - return true if all accels are healthy
bool AP_InertialSensor::get_accel_health_all(void) const
{
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!get_accel_health(i)) {
            return false;
        }
    }
    // return true if we have at least one accel
    return (get_accel_count() > 0);
}


/*
  calculate the trim_roll and trim_pitch. This is used for redoing the
  trim without needing a full accel cal
 */
bool AP_InertialSensor::calibrate_trim(Vector3f &trim_rad)
{
    Vector3f level_sample;

    // exit immediately if calibration is already in progress
    if (calibrating()) {
        return false;
    }

    const uint8_t update_dt_milliseconds = (uint8_t)(1000.0f/get_loop_rate_hz()+0.5f);

    // wait 100ms for ins filter to rise
    for (uint8_t k=0; k<100/update_dt_milliseconds; k++) {
        wait_for_sample();
        update();
        hal.scheduler->delay(update_dt_milliseconds);
    }

    uint32_t num_samples = 0;
    while (num_samples < 400/update_dt_milliseconds) {
        wait_for_sample();
        // read samples from ins
        update();
        // capture sample
        Vector3f samp;
        samp = get_accel(0);
        level_sample += samp;
        if (!get_accel_health(0)) {
            return false;
        }
        hal.scheduler->delay(update_dt_milliseconds);
        num_samples++;
    }
    level_sample /= num_samples;

    if (!_calculate_trim(level_sample, trim_rad)) {
        return false;
    }

    return true;
}

/*
  check if the accelerometers are calibrated in 3D and that current number of accels matched number when calibrated
 */
bool AP_InertialSensor::accel_calibrated_ok_all() const
{
    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!_accel_id_ok[i]) {
            return false;
        }
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset(i).get().is_zero()) {
            return false;
        }
        // zero scaling also indicates not calibrated
        if (_accel_scale(i).get().is_zero()) {
            return false;
        }
    }
    for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
        if (_accel_id(i) != 0) {
            // missing accel
            return false;
        }
    }
    
    // check calibrated accels matches number of accels (no unused accels should have offsets or scaling)
    if (get_accel_count() < INS_MAX_INSTANCES) {
        for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
            const Vector3f &scaling = _accel_scale(i).get();
            bool have_scaling = (!is_zero(scaling.x) && !is_equal(scaling.x,1.0f)) || (!is_zero(scaling.y) && !is_equal(scaling.y,1.0f)) || (!is_zero(scaling.z) && !is_equal(scaling.z,1.0f));
            bool have_offsets = !_accel_offset(i).get().is_zero();
            if (have_scaling || have_offsets) {
                return false;
            }
        }
    }

    // if we got this far the accelerometers must have been calibrated
    return true;
}

// return true if accel instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_InertialSensor::use_accel(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_accel_health(instance) && _use(instance));
}

void
AP_InertialSensor::_init_gyro()
{
    uint8_t num_gyros = MIN(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    Vector3f new_gyro_offset[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    float start_temperature[INS_MAX_INSTANCES] {};
#endif

    // exit immediately if calibration is already in progress
    if (calibrating()) {
        return;
    }

    // record we are calibrating
    _calibrating_gyro = true;

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // cold start
    DEV_PRINTF("Init Gyro");

    /*
      we do the gyro calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset(k).set(Vector3f());
        new_gyro_offset[k].zero();
        best_diff[k] = -1.f;
        last_average[k].zero();
        converged[k] = false;
    }

    for(int8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        update();
    }

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    // get start temperature. gyro cal usually happens when the board
    // has just been powered on, so the temperature may be changing
    // rapidly. We use the average between start and end temperature
    // as the calibration temperature to minimise errors
    for (uint8_t k=0; k<num_gyros; k++) {
        start_temperature[k] = get_temperature(k);
    }
#endif

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 30 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 30*4 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
        Vector3f accel_start;
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        EXPECT_DELAY_MS(1000);

        memset(diff_norm, 0, sizeof(diff_norm));

        DEV_PRINTF("*");

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        accel_start = get_accel(0);
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += get_gyro(k);
            }
            hal.scheduler->delay(5);
        }

        Vector3f accel_diff = get_accel(0) - accel_start;
        if (accel_diff.length() > 0.2f) {
            // the accelerometers changed during the gyro sum. Skip
            // this sample. This copes with doing gyro cal on a
            // steadily moving platform. The value 0.2 corresponds
            // with around 5 degrees/second of rotation.
            continue;
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (best_diff[k] < 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(GYRO_INIT_MAX_DIFF_DPS)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                if (!converged[k] || last_average[k].length() < new_gyro_offset[k].length()) {
                    new_gyro_offset[k] = last_average[k];
                }
                if (!converged[k]) {
                    converged[k] = true;
                    num_converged++;
                }
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    DEV_PRINTF("\n");
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            DEV_PRINTF("gyro[%u] did not converge: diff=%f dps (expected < %f)\n",
                                (unsigned)k,
                                (double)ToDeg(best_diff[k]),
                                (double)GYRO_INIT_MAX_DIFF_DPS);
            _gyro_offset(k).set(best_avg[k]);
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        } else {
            _gyro_cal_ok[k] = true;
            _gyro_offset(k).set(new_gyro_offset[k]);
#if HAL_INS_TEMPERATURE_CAL_ENABLE
            caltemp_gyro(k).set(0.5 * (get_temperature(k) + start_temperature[k]));
#endif
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    // record calibration complete
    _calibrating_gyro = false;

    // stop flashing leds
    AP_Notify::flags.initialising = false;
    AP_Notify::flags.gyro_calibrated = true;
}

// save parameters to eeprom
void AP_InertialSensor::_save_gyro_calibration()
{
    for (uint8_t i=0; i<_gyro_count; i++) {
        _gyro_offset(i).save();
        _gyro_id(i).save();
#if HAL_INS_TEMPERATURE_CAL_ENABLE
        caltemp_gyro(i).save();
#endif
    }
    for (uint8_t i=_gyro_count; i<INS_MAX_INSTANCES; i++) {
        _gyro_offset(i).set_and_save(Vector3f());
        _gyro_id(i).set_and_save(0);
#if HAL_INS_TEMPERATURE_CAL_ENABLE
        caltemp_gyro(i).set_and_save_ifchanged(-300);
#endif
    }
}

/*
  update harmonic notch parameters
 */
void AP_InertialSensor::HarmonicNotch::update_params(uint8_t instance, bool converging, float gyro_rate)
{
    const float center_freq = calculated_notch_freq_hz[0];
    if (!is_equal(last_bandwidth_hz[instance], params.bandwidth_hz()) ||
        !is_equal(last_attenuation_dB[instance], params.attenuation_dB()) ||
        (params.tracking_mode() == HarmonicNotchDynamicMode::Fixed && !is_equal(last_center_freq_hz[instance], center_freq)) ||
        converging) {
        filter[instance].init(gyro_rate,
                              center_freq,
                              params.bandwidth_hz(),
                              params.attenuation_dB());
        last_center_freq_hz[instance] = center_freq;
        last_bandwidth_hz[instance] = params.bandwidth_hz();
        last_attenuation_dB[instance] = params.attenuation_dB();
    } else if (params.tracking_mode() != HarmonicNotchDynamicMode::Fixed) {
        if (num_calculated_notch_frequencies > 1) {
            filter[instance].update(num_calculated_notch_frequencies, calculated_notch_freq_hz);
        } else {
            filter[instance].update(center_freq);
        }
        last_center_freq_hz[instance] = center_freq;
    }
}

/*
  update gyro and accel values from backends
 */
void AP_InertialSensor::update(void)
{
    // during initialisation update() may be called without
    // wait_for_sample(), and a wait is implied
    wait_for_sample();

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            // mark sensors unhealthy and let update() in each backend
            // mark them healthy via _publish_gyro() and
            // _publish_accel()
            _gyro_healthy[i] = false;
            _accel_healthy[i] = false;
            _delta_velocity_valid[i] = false;
            _delta_angle_valid[i] = false;
        }
        for (uint8_t i=0; i<_backend_count; i++) {
            _backends[i]->update();
        }

        if (!_startup_error_counts_set) {
            for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
                _accel_startup_error_count[i] = _accel_error_count[i];
                _gyro_startup_error_count[i] = _gyro_error_count[i];
            }

            if (_startup_ms == 0) {
                _startup_ms = AP_HAL::millis();
            } else if (AP_HAL::millis()-_startup_ms > 2000) {
                _startup_error_counts_set = true;
            }
        }

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_error_count[i] < _accel_startup_error_count[i]) {
                _accel_startup_error_count[i] = _accel_error_count[i];
            }
            if (_gyro_error_count[i] < _gyro_startup_error_count[i]) {
                _gyro_startup_error_count[i] = _gyro_error_count[i];
            }
        }

        // adjust health status if a sensor has a non-zero error count
        // but another sensor doesn't.
        bool have_zero_accel_error_count = false;
        bool have_zero_gyro_error_count = false;
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _accel_error_count[i] <= _accel_startup_error_count[i]) {
                have_zero_accel_error_count = true;
            }
            if (_gyro_healthy[i] && _gyro_error_count[i] <= _gyro_startup_error_count[i]) {
                have_zero_gyro_error_count = true;
            }
        }

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i] && _gyro_error_count[i] > _gyro_startup_error_count[i] && have_zero_gyro_error_count) {
                // we prefer not to use a gyro that has had errors
                _gyro_healthy[i] = false;
            }
            if (_accel_healthy[i] && _accel_error_count[i] > _accel_startup_error_count[i] && have_zero_accel_error_count) {
                // we prefer not to use a accel that has had errors
                _accel_healthy[i] = false;
            }
        }

        // set primary to first healthy accel and gyro
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i] && _use(i)) {
                _primary_gyro = i;
                break;
            }
        }
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _use(i)) {
                _primary_accel = i;
                break;
            }
        }

    _last_update_usec = AP_HAL::micros();
    
    _have_sample = false;

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    if (tcal_learning && !temperature_cal_running()) {
        AP_Notify::flags.temp_cal_running = false;
        AP_Notify::events.temp_cal_saved = 1;
        tcal_learning = false;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TCAL finished all IMUs");
    }
#endif
}

/*
  wait for a sample to be available. This is the function that
  determines the timing of the main loop in ardupilot.

  Ideally this function would return at exactly the rate given by the
  sample_rate argument given to AP_InertialSensor::init().

  The key output of this function is _delta_time, which is the time
  over which the gyro and accel integration will happen for this
  sample. We want that to be a constant time if possible, but if
  delays occur we need to cope with them. The long term sum of
  _delta_time should be exactly equal to the wall clock elapsed time
 */
void AP_InertialSensor::wait_for_sample(void)
{
    if (_have_sample) {
        // the user has called wait_for_sample() again without
        // consuming the sample with update()
        return;
    }

    uint32_t now = AP_HAL::micros();

    if (_next_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }

    // see how long it is till the next sample is due
    if (_next_sample_usec - now <=_sample_period_usec) {
        // we're ahead on time, schedule next sample at expected period
        uint32_t wait_usec = _next_sample_usec - now;
        hal.scheduler->delay_microseconds_boost(wait_usec);
        uint32_t now2 = AP_HAL::micros();
        if (now2+100 < _next_sample_usec) {
            timing_printf("shortsleep %u\n", (unsigned)(_next_sample_usec-now2));
        }
        if (now2 > _next_sample_usec+400) {
            timing_printf("longsleep %u wait_usec=%u\n",
                          (unsigned)(now2-_next_sample_usec),
                          (unsigned)wait_usec);
        }
        _next_sample_usec += _sample_period_usec;
    } else if (now - _next_sample_usec < _sample_period_usec/8) {
        // we've overshot, but only by a small amount, keep on
        // schedule with no delay
        timing_printf("overshoot1 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec += _sample_period_usec;
    } else {
        // we've overshot by a larger amount, re-zero scheduling with
        // no delay
        timing_printf("overshoot2 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec = now + _sample_period_usec;
    }

check_sample:
        // now we wait until we have the gyro and accel samples we need
        uint8_t gyro_available_mask = 0;
        uint8_t accel_available_mask = 0;
        uint32_t wait_counter = 0;
        // allow to wait for up to 1/3 of the loop time for samples from all
        // IMUs to come in
        const uint8_t wait_per_loop = 100;
        const uint8_t wait_counter_limit = uint32_t(_loop_delta_t * 1.0e6) / (3*wait_per_loop);

        while (true) {
            for (uint8_t i=0; i<_backend_count; i++) {
                // this is normally a nop, but can be used by backends
                // that don't accumulate samples on a timer
                _backends[i]->accumulate();
            }

            for (uint8_t i=0; i<_gyro_count; i++) {
                if (_new_gyro_data[i]) {
                    const uint8_t imask = (1U<<i);
                    gyro_available_mask |= imask;
                    if (_use(i)) {
                        _gyro_wait_mask |= imask;
                    } else {
                        _gyro_wait_mask &= ~imask;
                    }
                }
            }
            for (uint8_t i=0; i<_accel_count; i++) {
                if (_new_accel_data[i]) {
                    const uint8_t imask = (1U<<i);
                    accel_available_mask |= imask;
                    if (_use(i)) {
                        _accel_wait_mask |= imask;
                    } else {
                        _accel_wait_mask &= ~imask;
                    }
                }
            }

            // we wait for up to 1/3 of the loop time to get all of the required
            // accel and gyro samples. After that we accept at least
            // one of each
            if (wait_counter < wait_counter_limit) {
                if (gyro_available_mask &&
                    ((gyro_available_mask & _gyro_wait_mask) == _gyro_wait_mask) &&
                    accel_available_mask &&
                    ((accel_available_mask & _accel_wait_mask) == _accel_wait_mask)) {
                    break;
                }
            } else {
                if (gyro_available_mask && accel_available_mask) {
                    // reset the wait mask so we don't keep delaying
                    // for a dead IMU on the next loop. As soon as it
                    // comes back we will start waiting on it again
                    _gyro_wait_mask &= gyro_available_mask;
                    _accel_wait_mask &= accel_available_mask;
                    break;
                }
            }

            hal.scheduler->delay_microseconds_boost(wait_per_loop);
            wait_counter++;
        }

    now = AP_HAL::micros();
    _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    _last_sample_usec = now;

#if 0
    {
        static uint64_t delta_time_sum;
        static uint16_t counter;
        if (delta_time_sum == 0) {
            delta_time_sum = _sample_period_usec;
        }
        delta_time_sum += _delta_time * 1.0e6f;
        if (counter++ == 400) {
            counter = 0;
            hal.console->printf("now=%lu _delta_time_sum=%lu diff=%ld\n",
                                (unsigned long)now,
                                (unsigned long)delta_time_sum,
                                (long)(now - delta_time_sum));
        }
    }
#endif

    _have_sample = true;
}


/*
  get delta angles
 */
bool AP_InertialSensor::get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const
{
    if (_delta_angle_valid[i] && _delta_angle_dt[i] > 0) {
        delta_angle_dt = _delta_angle_dt[i];
    } else {
        delta_angle_dt = get_delta_time();
    }
    delta_angle_dt = MIN(delta_angle_dt, _loop_delta_t_max);

    if (_delta_angle_valid[i]) {
        delta_angle = _delta_angle[i];
        return true;
    } else if (get_gyro_health(i)) {
        // provide delta angle from raw gyro, so we use the same code
        // at higher level
        delta_angle = get_gyro(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
  get delta velocity if available
*/
bool AP_InertialSensor::get_delta_velocity(uint8_t i, Vector3f &delta_velocity, float &delta_velocity_dt) const
{
    if (_delta_velocity_valid[i]) {
        delta_velocity_dt = _delta_velocity_dt[i];
    } else {
        delta_velocity_dt = get_delta_time();
    }
    delta_velocity_dt = MIN(delta_velocity_dt, _loop_delta_t_max);

    if (_delta_velocity_valid[i]) {
        delta_velocity = _delta_velocity[i];
        return true;
    } else if (get_accel_health(i)) {
        delta_velocity = get_accel(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
 * Get an AuxiliaryBus of N @instance of backend identified by @backend_id
 */
AuxiliaryBus *AP_InertialSensor::get_auxiliary_bus(int16_t backend_id, uint8_t instance)
{
    detect_backends();

    AP_InertialSensor_Backend *backend = _find_backend(backend_id, instance);
    if (backend == nullptr) {
        return nullptr;
    }

    return backend->get_auxiliary_bus();
}

// calculate vibration levels and check for accelerometer clipping (called by a backends)
void AP_InertialSensor::calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt)
{
    // check for clipping
    if (_backends[instance] == nullptr) {
        return;
    }
    if (fabsf(accel.x) >  _backends[instance]->get_clip_limit() ||
        fabsf(accel.y) >  _backends[instance]->get_clip_limit() ||
        fabsf(accel.z) > _backends[instance]->get_clip_limit()) {
        _accel_clip_count[instance]++;
    }

    // calculate vibration levels
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        // filter accel at 5hz
        Vector3f accel_filt = _accel_vibe_floor_filter[instance].apply(accel, dt);

        // calc difference from this sample and 5hz filtered value, square and filter at 2hz
        Vector3f accel_diff = (accel - accel_filt);
        accel_diff.x *= accel_diff.x;
        accel_diff.y *= accel_diff.y;
        accel_diff.z *= accel_diff.z;
        _accel_vibe_filter[instance].apply(accel_diff, dt);
    }
}

// peak hold detector for slower mechanisms to detect spikes
void AP_InertialSensor::set_accel_peak_hold(uint8_t instance, const Vector3f &accel)
{
    if (instance != _primary_accel) {
        // we only record for primary accel
        return;
    }
    uint32_t now = AP_HAL::millis();

    // negative x peak(min) hold detector
    if (accel.x < _peak_hold_state.accel_peak_hold_neg_x ||
        _peak_hold_state.accel_peak_hold_neg_x_age <= now) {
        _peak_hold_state.accel_peak_hold_neg_x = accel.x;
        _peak_hold_state.accel_peak_hold_neg_x_age = now + AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS;
    }
}

// retrieve latest calculated vibration levels
Vector3f AP_InertialSensor::get_vibration_levels(uint8_t instance) const
{
    Vector3f vibe;
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        vibe = _accel_vibe_filter[instance].get();
        vibe.x = safe_sqrt(vibe.x);
        vibe.y = safe_sqrt(vibe.y);
        vibe.z = safe_sqrt(vibe.z);
    }
    return vibe;
}

// check for vibration movement. Return true if all axis show nearly zero movement
bool AP_InertialSensor::is_still()
{
    Vector3f vibe = get_vibration_levels();
    return (vibe.x < _still_threshold) &&
           (vibe.y < _still_threshold) &&
           (vibe.z < _still_threshold);
}

// return true if we are in a calibration
bool AP_InertialSensor::calibrating() const
{
    if (_calibrating_accel || _calibrating_gyro) {
        return true;
    }
#if HAL_INS_ACCELCAL_ENABLED
    if (_acal && _acal->running()) {
        return true;
    }
#endif
    return false;
}

/// calibrating - returns true if a temperature calibration is running
bool AP_InertialSensor::temperature_cal_running() const
{
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (tcal(i).enable == AP_InertialSensor_TCal::Enable::LearnCalibration) {
            return true;
        }
    }
#endif
    return false;
}

#if HAL_INS_ACCELCAL_ENABLED
// initialise and register accel calibrator
// called during the startup of accel cal
void AP_InertialSensor::acal_init()
{
    // NOTE: these objects are never deallocated because the pre-arm checks force a reboot
    if (_acal == nullptr) {
        _acal = new AP_AccelCal;
    }
    if (_accel_calibrator == nullptr) {
        _accel_calibrator = new AccelCalibrator[INS_MAX_INSTANCES];
    }
}

// update accel calibrator
void AP_InertialSensor::acal_update()
{
    if(_acal == nullptr) {
        return;
    }

    EXPECT_DELAY_MS(20000);
    _acal->update();

    if (hal.util->get_soft_armed() && _acal->get_status() != ACCEL_CAL_NOT_STARTED) {
        _acal->cancel();
    }
}
#endif

// Update the harmonic notch frequency
void AP_InertialSensor::HarmonicNotch::update_freq_hz(float scaled_freq)
{
    // protect against zero as the scaled frequency
    if (is_positive(scaled_freq)) {
        calculated_notch_freq_hz[0] = scaled_freq;
    }
    num_calculated_notch_frequencies = 1;
}

// Update the harmonic notch frequency
void AP_InertialSensor::HarmonicNotch::update_frequencies_hz(uint8_t num_freqs, const float scaled_freq[]) {
    // protect against zero as the scaled frequency
    for (uint8_t i = 0; i < num_freqs; i++) {
        if (is_positive(scaled_freq[i])) {
            calculated_notch_freq_hz[i] = scaled_freq[i];
        }
    }
    // any uncalculated frequencies will float at the previous value or the initialized freq if none
    num_calculated_notch_frequencies = num_freqs;
}

// setup the notch for throttle based tracking, called from FFT based tuning
bool AP_InertialSensor::setup_throttle_gyro_harmonic_notch(float center_freq_hz, float lower_freq_hz, float ref, uint8_t harmonics)
{
    for (auto &notch : harmonic_notches) {
        if (notch.params.tracking_mode() != HarmonicNotchDynamicMode::UpdateThrottle) {
            continue;
        }
        notch.params.enable();
        notch.params.set_center_freq_hz(center_freq_hz);
        notch.params.set_reference(ref);
        notch.params.set_bandwidth_hz(center_freq_hz / 2.0f);
        notch.params.set_freq_min_ratio(lower_freq_hz / center_freq_hz);
        notch.params.set_harmonics(harmonics);
        notch.params.save_params();
        // only enable the first notch
        return true;
    }

    return false;
}

/*
    set and save accelerometer bias along with trim calculation
*/
void AP_InertialSensor::_acal_save_calibrations()
{
    Vector3f bias, gain;
    for (uint8_t i=0; i<_accel_count; i++) {
        if (_accel_calibrator[i].get_status() == ACCEL_CAL_SUCCESS) {
            _accel_calibrator[i].get_calibration(bias, gain);
            _accel_offset(i).set_and_save(bias);
            _accel_scale(i).set_and_save(gain);
            _accel_id(i).save();
            _accel_id_ok[i] = true;
#if HAL_INS_TEMPERATURE_CAL_ENABLE
            caltemp_accel(i).set_and_save(get_temperature(i));
#endif
        } else {
            _accel_offset(i).set_and_save(Vector3f());
            _accel_scale(i).set_and_save(Vector3f());
#if HAL_INS_TEMPERATURE_CAL_ENABLE
            caltemp_accel(i).set_and_save(-300);
#endif
        }
    }

    // clear any unused accels
    for (uint8_t i=_accel_count; i<INS_MAX_INSTANCES; i++) {
        _accel_id(i).set_and_save(0);
        _accel_offset(i).set_and_save(Vector3f());
        _accel_scale(i).set_and_save(Vector3f());
#if HAL_INS_TEMPERATURE_CAL_ENABLE
        caltemp_accel(i).set_and_save_ifchanged(-300);
#endif
    }
    
    Vector3f aligned_sample;
    Vector3f misaligned_sample;
    switch(_trim_option) {
        case 0:
            break;
        case 1:
            // The first level step of accel cal will be taken as gnd truth,
            // i.e. trim will be set as per the output of primary accel from the level step
            get_primary_accel_cal_sample_avg(0,aligned_sample);
            _trim_rad.zero();
            _calculate_trim(aligned_sample, _trim_rad);
            _new_trim = true;
            break;
        case 2:
            // Reference accel is truth, in this scenario there is a reference accel
            // as mentioned in ACC_BODY_ALIGNED
            if (get_primary_accel_cal_sample_avg(0,misaligned_sample) && get_fixed_mount_accel_cal_sample(0,aligned_sample)) {
                // determine trim from aligned sample vs misaligned sample
                Vector3f cross = (misaligned_sample%aligned_sample);
                float dot = (misaligned_sample*aligned_sample);
                Quaternion q(safe_sqrt(sq(misaligned_sample.length())*sq(aligned_sample.length()))+dot, cross.x, cross.y, cross.z);
                q.normalize();
                _trim_rad.x = q.get_euler_roll();
                _trim_rad.y = q.get_euler_pitch();
                _trim_rad.z = 0;
                _new_trim = true;
            }
            break;
        default:
            _new_trim = false;
            /* no break */
    }

    if (fabsf(_trim_rad.x) > radians(HAL_INS_TRIM_LIMIT_DEG) ||
        fabsf(_trim_rad.y) > radians(HAL_INS_TRIM_LIMIT_DEG) ||
        fabsf(_trim_rad.z) > radians(HAL_INS_TRIM_LIMIT_DEG)) {
        DEV_PRINTF("ERR: Trim over maximum of %.1f degrees!!", float(HAL_INS_TRIM_LIMIT_DEG));
        _new_trim = false;  //we have either got faulty level during acal or highly misaligned accelerometers
    }

    _accel_cal_requires_reboot = true;
}

void AP_InertialSensor::_acal_event_failure()
{
    for (uint8_t i=0; i<_accel_count; i++) {
        _accel_offset(i).set_and_notify(Vector3f(0,0,0));
        _accel_scale(i).set_and_notify(Vector3f(1,1,1));
    }
}

/*
    Returns true if new valid trim values are available and passes them to reference vars
*/
bool AP_InertialSensor::get_new_trim(Vector3f &trim_rad)
{
    if (_new_trim) {
        trim_rad = _trim_rad;
        _new_trim = false;
        return true;
    }
    return false;
}

/*
    Returns body fixed accelerometer level data averaged during accel calibration's first step
*/
bool AP_InertialSensor::get_fixed_mount_accel_cal_sample(uint8_t sample_num, Vector3f& ret) const
{
    if (_accel_count <= (_acc_body_aligned-1) || _accel_calibrator[2].get_status() != ACCEL_CAL_SUCCESS || sample_num>=_accel_calibrator[2].get_num_samples_collected()) {
        return false;
    }
    _accel_calibrator[_acc_body_aligned-1].get_sample_corrected(sample_num, ret);
    ret.rotate(_board_orientation);
    return true;
}

/*
    Returns Primary accelerometer level data averaged during accel calibration's first step
*/
bool AP_InertialSensor::get_primary_accel_cal_sample_avg(uint8_t sample_num, Vector3f& ret) const
{
    uint8_t count = 0;
    Vector3f avg = Vector3f(0,0,0);
    for (uint8_t i=0; i<MIN(_accel_count,2); i++) {
        if (_accel_calibrator[i].get_status() != ACCEL_CAL_SUCCESS || sample_num>=_accel_calibrator[i].get_num_samples_collected()) {
            continue;
        }
        Vector3f sample;
        _accel_calibrator[i].get_sample_corrected(sample_num, sample);
        avg += sample;
        count++;
    }
    if (count == 0) {
        return false;
    }
    avg /= count;
    ret = avg;
    ret.rotate(_board_orientation);
    return true;
}

/*
  perform a simple 1D accel calibration, returning mavlink result code
 */
#if HAL_GCS_ENABLED
MAV_RESULT AP_InertialSensor::simple_accel_cal()
{
    uint8_t num_accels = MIN(get_accel_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES];
    Vector3f new_accel_offset[INS_MAX_INSTANCES];
    Vector3f saved_offsets[INS_MAX_INSTANCES];
    Vector3f saved_scaling[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];
    const float accel_convergence_limit = 0.05;
    Vector3f rotated_gravity(0, 0, -GRAVITY_MSS);
    
    // exit immediately if calibration is already in progress
    if (calibrating()) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    EXPECT_DELAY_MS(20000);
    // record we are calibrating
    _calibrating_accel = true;

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    /*
      we do the accel calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // get the rotated gravity vector which will need to be applied to the offsets
    rotated_gravity.rotate_inverse(saved_orientation);
    
    // save existing accel offsets
    for (uint8_t k=0; k<num_accels; k++) {
        saved_offsets[k] = _accel_offset(k);
        saved_scaling[k] = _accel_scale(k);
    }
    
    // remove existing accel offsets and scaling
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset(k).set(Vector3f());
        _accel_scale(k).set(Vector3f(1,1,1));
        new_accel_offset[k].zero();
        last_average[k].zero();
        converged[k] = false;
    }

    for (uint8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 10 seconds
    // if the accels are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 10*4 && num_converged < num_accels; j++) {
        Vector3f accel_sum[INS_MAX_INSTANCES], accel_avg[INS_MAX_INSTANCES], accel_diff[INS_MAX_INSTANCES];
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

        DEV_PRINTF("*");

        for (uint8_t k=0; k<num_accels; k++) {
            accel_sum[k].zero();
        }
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_accels; k++) {
                accel_sum[k] += get_accel(k);
            }
            hal.scheduler->delay(5);
        }

        for (uint8_t k=0; k<num_accels; k++) {
            accel_avg[k] = accel_sum[k] / i;
            accel_diff[k] = last_average[k] - accel_avg[k];
            diff_norm[k] = accel_diff[k].length();
        }

        for (uint8_t k=0; k<num_accels; k++) {
            if (j > 0 && diff_norm[k] < accel_convergence_limit) {
                last_average[k] = (accel_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                if (!converged[k] || last_average[k].length() < new_accel_offset[k].length()) {
                    new_accel_offset[k] = last_average[k];
                }
                if (!converged[k]) {
                    converged[k] = true;
                    num_converged++;
                }
            } else {
                last_average[k] = accel_avg[k];
            }
        }
    }

    MAV_RESULT result = MAV_RESULT_ACCEPTED;

    // see if we've passed
    for (uint8_t k=0; k<num_accels; k++) {
        if (!converged[k]) {
            result = MAV_RESULT_FAILED;
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    if (result == MAV_RESULT_ACCEPTED) {
        DEV_PRINTF("\nPASSED\n");
        for (uint8_t k=0; k<num_accels; k++) {
            // remove rotated gravity
            new_accel_offset[k] -= rotated_gravity;
            _accel_offset(k).set_and_save(new_accel_offset[k]);
            _accel_scale(k).save();
            _accel_id(k).save();
            _accel_id_ok[k] = true;
#if HAL_INS_TEMPERATURE_CAL_ENABLE
            caltemp_accel(k).set_and_save(get_temperature(k));
#endif
        }

        // force trim to zero
        AP::ahrs().set_trim(Vector3f(0, 0, 0));
    } else {
        DEV_PRINTF("\nFAILED\n");
        // restore old values
        for (uint8_t k=0; k<num_accels; k++) {
            _accel_offset(k).set(saved_offsets[k]);
            _accel_scale(k).set(saved_scaling[k]);
        }
    }

    // record calibration complete
    _calibrating_accel = false;

    // throw away any existing samples that may have the wrong
    // orientation. We do this by throwing samples away for 0.5s,
    // which is enough time for the filters to settle
    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 500) {
        update();
    }

    // and reset state estimators
    AP::ahrs().reset();

    // stop flashing leds
    AP_Notify::flags.initialising = false;

    return result;
}
#endif

/*
  see if gyro calibration should be performed
 */
AP_InertialSensor::Gyro_Calibration_Timing AP_InertialSensor::gyro_calibration_timing()
{
    if (hal.util->was_watchdog_reset()) {
        return GYRO_CAL_NEVER;
    }
    return (Gyro_Calibration_Timing)_gyro_cal_timing.get();
}

#if !HAL_MINIMIZE_FEATURES
/*
  update IMU kill mask, used for testing IMU failover
 */
void AP_InertialSensor::kill_imu(uint8_t imu_idx, bool kill_it)
{
    if (kill_it) {
        uint8_t new_kill_mask = imu_kill_mask | (1U<<imu_idx);
        // don't allow the last IMU to be killed
        bool all_dead = true;
        for (uint8_t i=0; i<MIN(_gyro_count, _accel_count); i++) {
            if (use_gyro(i) && use_accel(i) && !(new_kill_mask & (1U<<i))) {
                // we have at least one healthy IMU left
                all_dead = false;
            }
        }
        if (!all_dead) {
            imu_kill_mask = new_kill_mask;
        }
    } else {
        imu_kill_mask &= ~(1U<<imu_idx);
    }
}
#endif // HAL_MINIMIZE_FEATURES


#if HAL_EXTERNAL_AHRS_ENABLED
void AP_InertialSensor::handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt)
{
    for (uint8_t i = 0; i < _backend_count; i++) {
        _backends[i]->handle_external(pkt);
    }
}
#endif // HAL_EXTERNAL_AHRS_ENABLED

// force save of current calibration as valid
void AP_InertialSensor::force_save_calibration(void)
{
    for (uint8_t i=0; i<_accel_count; i++) {
        if (_accel_id(i) != 0) {
            _accel_id(i).save();
            // we also save the scale as the default of 1.0 may be
            // over a stored value of 0.0
            _accel_scale(i).save();
            _accel_id_ok[i] = true;
        }
    }
}

namespace AP {

AP_InertialSensor &ins()
{
    return *AP_InertialSensor::get_singleton();
}

};

#endif  // AP_INERTIALSENSOR_ENABLED
