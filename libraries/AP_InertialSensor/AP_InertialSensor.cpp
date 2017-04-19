#include <assert.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_BMI160.h"
#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_HIL.h"
#include "AP_InertialSensor_L3G4200D.h"
#include "AP_InertialSensor_LSM9DS0.h"
#include "AP_InertialSensor_Invensense.h"
#include "AP_InertialSensor_PX4.h"
#include "AP_InertialSensor_QURT.h"
#include "AP_InertialSensor_SITL.h"
#include "AP_InertialSensor_qflight.h"

/* Define INS_TIMING_DEBUG to track down scheduling issues with the main loop.
 * Output is on the debug console. */
#ifdef INS_TIMING_DEBUG
#include <stdio.h>
#define timing_printf(fmt, args...)      do { printf("[timing] " fmt, ##args); } while(0)
#else
#define timing_printf(fmt, args...)
#endif

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#define DEFAULT_STILL_THRESH 2.5f
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
#define DEFAULT_GYRO_FILTER  4
#define DEFAULT_ACCEL_FILTER 10
#define DEFAULT_STILL_THRESH 0.1f
#else
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#define DEFAULT_STILL_THRESH 0.1f
#endif

#define SAMPLE_UNIT 1

#define GYRO_INIT_MAX_DIFF_DPS 0.1f

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::var_info[] = {
    // @Param: PRODUCT_ID
    // @DisplayName: IMU Product ID
    // @Description: unused
    // @User: Advanced
    AP_GROUPINFO("PRODUCT_ID",  0, AP_InertialSensor, _old_product_id,   0),

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

    // @Param: GYROFFS_X
    // @DisplayName: Gyro offsets of X axis
    // @Description: Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYROFFS_Y
    // @DisplayName: Gyro offsets of Y axis
    // @Description: Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYROFFS_Z
    // @DisplayName: Gyro offsets of Z axis
    // @Description: Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYROFFS",     3, AP_InertialSensor, _gyro_offset[0],  0),

    // @Param: GYR2OFFS_X
    // @DisplayName: Gyro2 offsets of X axis
    // @Description: Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR2OFFS_Y
    // @DisplayName: Gyro2 offsets of Y axis
    // @Description: Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR2OFFS_Z
    // @DisplayName: Gyro2 offsets of Z axis
    // @Description: Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYR2OFFS",    7, AP_InertialSensor, _gyro_offset[1],   0),

    // @Param: GYR3OFFS_X
    // @DisplayName: Gyro3 offsets of X axis
    // @Description: Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR3OFFS_Y
    // @DisplayName: Gyro3 offsets of Y axis
    // @Description: Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR3OFFS_Z
    // @DisplayName: Gyro3 offsets of Z axis
    // @Description: Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYR3OFFS",   10, AP_InertialSensor, _gyro_offset[2],   0),

    // @Param: ACCSCAL_X
    // @DisplayName: Accelerometer scaling of X axis
    // @Description: Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACCSCAL_Y
    // @DisplayName: Accelerometer scaling of Y axis
    // @Description: Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACCSCAL_Z
    // @DisplayName: Accelerometer scaling of Z axis
    // @Description: Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACCSCAL",     12, AP_InertialSensor, _accel_scale[0],  0),

    // @Param: ACCOFFS_X
    // @DisplayName: Accelerometer offsets of X axis
    // @Description: Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACCOFFS_Y
    // @DisplayName: Accelerometer offsets of Y axis
    // @Description: Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACCOFFS_Z
    // @DisplayName: Accelerometer offsets of Z axis
    // @Description: Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACCOFFS",     13, AP_InertialSensor, _accel_offset[0], 0),

    // @Param: ACC2SCAL_X
    // @DisplayName: Accelerometer2 scaling of X axis
    // @Description: Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC2SCAL_Y
    // @DisplayName: Accelerometer2 scaling of Y axis
    // @Description: Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC2SCAL_Z
    // @DisplayName: Accelerometer2 scaling of Z axis
    // @Description: Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACC2SCAL",    14, AP_InertialSensor, _accel_scale[1],   0),

    // @Param: ACC2OFFS_X
    // @DisplayName: Accelerometer2 offsets of X axis
    // @Description: Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC2OFFS_Y
    // @DisplayName: Accelerometer2 offsets of Y axis
    // @Description: Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC2OFFS_Z
    // @DisplayName: Accelerometer2 offsets of Z axis
    // @Description: Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACC2OFFS",    15, AP_InertialSensor, _accel_offset[1],  0),

    // @Param: ACC3SCAL_X
    // @DisplayName: Accelerometer3 scaling of X axis
    // @Description: Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC3SCAL_Y
    // @DisplayName: Accelerometer3 scaling of Y axis
    // @Description: Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC3SCAL_Z
    // @DisplayName: Accelerometer3 scaling of Z axis
    // @Description: Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACC3SCAL",    16, AP_InertialSensor, _accel_scale[2],   0),

    // @Param: ACC3OFFS_X
    // @DisplayName: Accelerometer3 offsets of X axis
    // @Description: Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC3OFFS_Y
    // @DisplayName: Accelerometer3 offsets of Y axis
    // @Description: Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC3OFFS_Z
    // @DisplayName: Accelerometer3 offsets of Z axis
    // @Description: Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACC3OFFS",    17, AP_InertialSensor, _accel_offset[2],  0),

    // @Param: GYRO_FILTER
    // @DisplayName: Gyro filter cutoff frequency
    // @Description: Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("GYRO_FILTER", 18, AP_InertialSensor, _gyro_filter_cutoff,  DEFAULT_GYRO_FILTER),

    // @Param: ACCEL_FILTER
    // @DisplayName: Accel filter cutoff frequency
    // @Description: Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("ACCEL_FILTER", 19, AP_InertialSensor, _accel_filter_cutoff,  DEFAULT_ACCEL_FILTER),

    // @Param: USE
    // @DisplayName: Use first IMU for attitude, velocity and position estimates
    // @Description: Use first IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE", 20, AP_InertialSensor, _use[0],  1),

    // @Param: USE2
    // @DisplayName: Use second IMU for attitude, velocity and position estimates
    // @Description: Use second IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2", 21, AP_InertialSensor, _use[1],  1),

    // @Param: USE3
    // @DisplayName: Use third IMU for attitude, velocity and position estimates
    // @Description: Use third IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3", 22, AP_InertialSensor, _use[2],  0),

    // @Param: STILL_THRESH
    // @DisplayName: Stillness threshold for detecting if we are moving
    // @Description: Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5
    // @Range: 0.05 50
    // @User: Advanced
    AP_GROUPINFO("STILL_THRESH", 23, AP_InertialSensor, _still_threshold,  DEFAULT_STILL_THRESH),

    // @Param: GYR_CAL
    // @DisplayName: Gyro Calibration scheme
    // @Description: Conrols when automatic gyro calibration is performed
    // @Values: 0:Never, 1:Start-up only
    // @User: Advanced
    AP_GROUPINFO("GYR_CAL", 24, AP_InertialSensor, _gyro_cal_timing, 1),

    // @Param: TRIM_OPTION
    // @DisplayName: Accel cal trim option
    // @Description: Specifies how the accel cal routine determines the trims
    // @User: Advanced
    // @Values: 0:Don't adjust the trims,1:Assume first orientation was level,2:Assume ACC_BODYFIX is perfectly aligned to the vehicle
    AP_GROUPINFO("TRIM_OPTION", 25, AP_InertialSensor, _trim_option, 1),

    // @Param: ACC_BODYFIX
    // @DisplayName: Body-fixed accelerometer
    // @Description: The body-fixed accelerometer to be used for trim calculation
    // @User: Advanced
    // @Values: 1:IMU 1,2:IMU 2,3:IMU 3
    AP_GROUPINFO("ACC_BODYFIX", 26, AP_InertialSensor, _acc_body_aligned, 2),

    // @Param: POS1_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS1_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS1_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS1", 27, AP_InertialSensor, _accel_pos[0], 0.0f),

    // @Param: POS2_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the second IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS2_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the second IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS2_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the second IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS2", 28, AP_InertialSensor, _accel_pos[1], 0.0f),

    // @Param: POS3_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the third IMU accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS3_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the third IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced

    // @Param: POS3_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the third IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS3", 29, AP_InertialSensor, _accel_pos[2], 0.0f),

    // @Param: GYR_ID
    // @DisplayName: Gyro ID
    // @Description: Gyro sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("GYR_ID", 30, AP_InertialSensor, _gyro_id[0], 0),

    // @Param: GYR2_ID
    // @DisplayName: Gyro2 ID
    // @Description: Gyro2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("GYR2_ID", 31, AP_InertialSensor, _gyro_id[1], 0),

    // @Param: GYR3_ID
    // @DisplayName: Gyro3 ID
    // @Description: Gyro3 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("GYR3_ID", 32, AP_InertialSensor, _gyro_id[2], 0),

    // @Param: ACC_ID
    // @DisplayName: Accelerometer ID
    // @Description: Accelerometer sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("ACC_ID", 33, AP_InertialSensor, _accel_id[0], 0),

    // @Param: ACC2_ID
    // @DisplayName: Accelerometer2 ID
    // @Description: Accelerometer2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("ACC2_ID", 34, AP_InertialSensor, _accel_id[1], 0),

    // @Param: ACC3_ID
    // @DisplayName: Accelerometer3 ID
    // @Description: Accelerometer3 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("ACC3_ID", 35, AP_InertialSensor, _accel_id[2], 0),

    // @Param: FAST_SAMPLE
    // @DisplayName: Fast sampling mask
    // @Description: Mask of IMUs to enable fast sampling on, if available
    // @User: Advanced
    AP_GROUPINFO("FAST_SAMPLE",  36, AP_InertialSensor, _fast_sampling_mask,   0),

    /*
      NOTE: parameter indexes have gaps above. When adding new
      parameters check for conflicts carefully
     */
    AP_GROUPEND
};

AP_InertialSensor *AP_InertialSensor::_s_instance = nullptr;

AP_InertialSensor::AP_InertialSensor() :
    _gyro_count(0),
    _accel_count(0),
    _backend_count(0),
    _accel(),
    _gyro(),
    _board_orientation(ROTATION_NONE),
    _primary_gyro(0),
    _primary_accel(0),
    _hil_mode(false),
    _calibrating(false),
    _log_raw_data(false),
    _backends_detected(false),
    _dataflash(nullptr),
    _accel_cal_requires_reboot(false),
    _startup_error_counts_set(false),
    _startup_ms(0)
{
    if (_s_instance) {
        AP_HAL::panic("Too many inertial sensors");
    }
    _s_instance = this;
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i=0; i<INS_MAX_BACKENDS; i++) {
        _backends[i] = nullptr;
    }
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_error_count[i] = 0;
        _gyro_error_count[i] = 0;
        _gyro_cal_ok[i] = true;
        _accel_clip_count[i] = 0;

        _accel_max_abs_offsets[i] = 3.5f;

        _accel_raw_sample_rates[i] = 0;
        _gyro_raw_sample_rates[i] = 0;

        _delta_velocity_acc[i].zero();
        _delta_velocity_acc_dt[i] = 0;

        _delta_angle_acc[i].zero();
        _delta_angle_acc_dt[i] = 0;
        _last_delta_angle[i].zero();
        _last_raw_gyro[i].zero();

        _accel_startup_error_count[i] = 0;
        _gyro_startup_error_count[i] = 0;
    }
    for (uint8_t i=0; i<INS_VIBRATION_CHECK_INSTANCES; i++) {
        _accel_vibe_floor_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ);
        _accel_vibe_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ);
    }
    memset(_delta_velocity_valid,0,sizeof(_delta_velocity_valid));
    memset(_delta_angle_valid,0,sizeof(_delta_angle_valid));

    AP_AccelCal::register_client(this);
}

/*
 * Get the AP_InertialSensor singleton
 */
AP_InertialSensor *AP_InertialSensor::get_instance()
{
    if (!_s_instance) {
        _s_instance = new AP_InertialSensor();
    }
    return _s_instance;
}

/*
  register a new gyro instance
 */
uint8_t AP_InertialSensor::register_gyro(uint16_t raw_sample_rate_hz,
                                         uint32_t id)
{
    if (_gyro_count == INS_MAX_INSTANCES) {
        AP_HAL::panic("Too many gyros");
    }

    _gyro_raw_sample_rates[_gyro_count] = raw_sample_rate_hz;

    bool saved = _gyro_id[_gyro_count].load();

    if (saved && (uint32_t)_gyro_id[_gyro_count] != id) {
        // inconsistent gyro id - mark it as needing calibration
        _gyro_cal_ok[_gyro_count] = false;
    }

    _gyro_id[_gyro_count].set((int32_t) id);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!saved) {
        // assume this is the same sensor and save its ID to allow seamless
        // transition from when we didn't have the IDs.
        _gyro_id[_gyro_count].save();
    }
#endif

    return _gyro_count++;
}

/*
  register a new accel instance
 */
uint8_t AP_InertialSensor::register_accel(uint16_t raw_sample_rate_hz,
                                          uint32_t id)
{
    if (_accel_count == INS_MAX_INSTANCES) {
        AP_HAL::panic("Too many accels");
    }

    _accel_raw_sample_rates[_accel_count] = raw_sample_rate_hz;
    bool saved = _accel_id[_accel_count].load();

    if (!saved) {
        // inconsistent accel id
        _accel_id_ok[_accel_count] = false;
    } else if ((uint32_t)_accel_id[_accel_count] != id) {
        // inconsistent accel id
        _accel_id_ok[_accel_count] = false;
    } else {
        _accel_id_ok[_accel_count] = true;
    }

    _accel_id[_accel_count].set((int32_t) id);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // assume this is the same sensor and save its ID to allow seamless
        // transition from when we didn't have the IDs.
        _accel_id_ok[_accel_count] = true;
        _accel_id[_accel_count].save();
#endif

    return _accel_count++;
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
}

/* Find the N instance of the backend that has already been successfully detected */
AP_InertialSensor_Backend *AP_InertialSensor::_find_backend(int16_t backend_id, uint8_t instance)
{
    assert(_backends_detected);
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

void
AP_InertialSensor::init(uint16_t sample_rate)
{
    // remember the sample rate
    _sample_rate = sample_rate;
    _loop_delta_t = 1.0f / sample_rate;

    if (_gyro_count == 0 && _accel_count == 0) {
        _start_backends();
    }

    // initialise accel scale if need be. This is needed as we can't
    // give non-zero default values for vectors in AP_Param
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (_accel_scale[i].get().is_zero()) {
            _accel_scale[i].set(Vector3f(1,1,1));
        }
    }

    // calibrate gyros unless gyro calibration has been disabled
    if (gyro_calibration_timing() != GYRO_CAL_NEVER) {
        init_gyro();
    }

    _sample_period_usec = 1000*1000UL / _sample_rate;

    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;
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

    if (_hil_mode) {
        _add_backend(AP_InertialSensor_HIL::detect(*this));
        return;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _add_backend(AP_InertialSensor_SITL::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_HIL
    _add_backend(AP_InertialSensor_HIL::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_SPI && defined(HAL_INS_DEFAULT_ROTATION)
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME),
                                                  HAL_INS_DEFAULT_ROTATION));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_SPI
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME)));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_I2C && defined(HAL_INS_DEFAULT_ROTATION)
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.i2c_mgr->get_device(HAL_INS_MPU60x0_I2C_BUS, HAL_INS_MPU60x0_I2C_ADDR),
                                                  HAL_INS_DEFAULT_ROTATION));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_I2C
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.i2c_mgr->get_device(HAL_INS_MPU60x0_I2C_BUS, HAL_INS_MPU60x0_I2C_ADDR)));
#elif HAL_INS_DEFAULT == HAL_INS_BH
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.i2c_mgr->get_device(HAL_INS_MPU60x0_I2C_BUS, HAL_INS_MPU60x0_I2C_ADDR)));
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME)));
#elif HAL_INS_DEFAULT == HAL_INS_PX4 || HAL_INS_DEFAULT == HAL_INS_VRBRAIN

    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME)));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME), ROTATION_ROLL_180));
        _add_backend(AP_InertialSensor_LSM9DS0::probe(*this,
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_G_NAME),
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME),
                                                      ROTATION_ROLL_180,
                                                      ROTATION_ROLL_180_YAW_270,
                                                      ROTATION_PITCH_180));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        // older Pixhawk2 boards have the MPU6000 instead of MPU9250
        _fast_sampling_mask.set_default(1);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_EXT_NAME), ROTATION_PITCH_180));
        _add_backend(AP_InertialSensor_LSM9DS0::probe(*this,
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_EXT_G_NAME),
                                                      hal.spi->get_device(HAL_INS_LSM9DS0_EXT_A_NAME),
                                                      ROTATION_ROLL_180_YAW_270,
                                                      ROTATION_ROLL_180_YAW_90,
                                                      ROTATION_ROLL_180_YAW_90));
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXRACER:
        _fast_sampling_mask.set_default(3);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_NAME), ROTATION_ROLL_180_YAW_90));
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180_YAW_90));
        break;

    case AP_BoardConfig::PX4_BOARD_PHMINI:
        // PHMINI uses ICM20608 on the ACCEL_MAG device and a MPU9250 on the old MPU6000 CS line
        _fast_sampling_mask.set_default(3);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_AM_NAME), ROTATION_ROLL_180));
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180));
        break;

    case AP_BoardConfig::PX4_BOARD_AUAV21:
        // AUAV2.1 uses ICM20608 on the ACCEL_MAG device and a MPU9250 on the old MPU6000 CS line
        _fast_sampling_mask.set_default(3);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_ICM20608_AM_NAME), ROTATION_ROLL_180_YAW_90));
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_ROLL_180_YAW_90));
        break;
        
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
        _fast_sampling_mask.set_default(1);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), ROTATION_YAW_270));
        break;

    case AP_BoardConfig::PX4_BOARD_AEROFC:
        _fast_sampling_mask.set_default(1);
        _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU6500_NAME), ROTATION_YAW_270));
        break;

    default:
        break;
    }
    // also add any PX4 backends (eg. canbus sensors)
    _add_backend(AP_InertialSensor_PX4::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_MPU9250_SPI && defined(HAL_INS_DEFAULT_ROTATION)
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME), HAL_INS_DEFAULT_ROTATION));
#elif HAL_INS_DEFAULT == HAL_INS_MPU9250_SPI
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME)));
#elif HAL_INS_DEFAULT == HAL_INS_LSM9DS0
    _add_backend(AP_InertialSensor_LSM9DS0::probe(*this,
                 hal.spi->get_device(HAL_INS_LSM9DS0_G_NAME),
                 hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME)));
#elif HAL_INS_DEFAULT == HAL_INS_L3G4200D
    _add_backend(AP_InertialSensor_L3G4200D::probe(*this, hal.i2c_mgr->get_device(HAL_INS_L3G4200D_I2C_BUS, HAL_INS_L3G4200D_I2C_ADDR)));
#elif HAL_INS_DEFAULT == HAL_INS_RASPILOT
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU60x0_NAME)));
    _add_backend(AP_InertialSensor_LSM9DS0::probe(*this,
                                                  hal.spi->get_device(HAL_INS_LSM9DS0_G_NAME),
                                                  hal.spi->get_device(HAL_INS_LSM9DS0_A_NAME),
                                                  ROTATION_NONE, ROTATION_YAW_90));
#elif HAL_INS_DEFAULT == HAL_INS_MPU9250_I2C
    _add_backend(AP_InertialSensor_Invensense::probe(*this, hal.i2c_mgr->get_device(HAL_INS_MPU9250_I2C_BUS, HAL_INS_MPU9250_I2C_ADDR)));
#elif HAL_INS_DEFAULT == HAL_INS_QFLIGHT
    _add_backend(AP_InertialSensor_QFLIGHT::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_QURT
    _add_backend(AP_InertialSensor_QURT::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_BBBMINI
    AP_InertialSensor_Backend *backend = AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME));
    if (backend) {
        _add_backend(backend);
        hal.console->printf("MPU9250: Onboard IMU detected\n");
    } else {
        hal.console->printf("MPU9250: Onboard IMU not detected\n");
    }

    backend = AP_InertialSensor_Invensense::probe(*this, hal.spi->get_device(HAL_INS_MPU9250_NAME_EXT));
    if (backend) {
        _add_backend(backend);
        hal.console->printf("MPU9250: External IMU detected\n");
    } else {
        hal.console->printf("MPU9250: External IMU not detected\n");
    }
#elif HAL_INS_DEFAULT == HAL_INS_AERO
    auto *backend = AP_InertialSensor_BMI160::probe(*this,
                                                    hal.spi->get_device("bmi160"));
    if (backend) {
        _add_backend(backend);
    } else {
        hal.console->printf("aero: onboard IMU not detected\n");
    }
#else
    #error Unrecognised HAL_INS_TYPE setting
#endif

    if (_backend_count == 0) {
        AP_HAL::panic("No INS backends available");
    }
}

/*
  _calculate_trim - calculates the x and y trim angles. The
  accel_sample must be correctly scaled, offset and oriented for the
  board
*/
bool AP_InertialSensor::_calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch)
{
    trim_pitch = atan2f(accel_sample.x, norm(accel_sample.y, accel_sample.z));
    trim_roll = atan2f(-accel_sample.y, -accel_sample.z);
    if (fabsf(trim_roll) > radians(10) ||
        fabsf(trim_pitch) > radians(10)) {
        hal.console->printf("trim over maximum of 10 degrees\n");
        return false;
    }
    hal.console->printf("Trim OK: roll=%.2f pitch=%.2f\n",
                          (double)degrees(trim_roll),
                          (double)degrees(trim_pitch));
    return true;
}

void
AP_InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_gyro_calibration();
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
        if (_gyro_id[i] != 0) {
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

    return (get_gyro_health(instance) && _use[instance]);
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
bool AP_InertialSensor::calibrate_trim(float &trim_roll, float &trim_pitch)
{
    Vector3f level_sample;

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return false;
    }

    _calibrating = true;

    const uint8_t update_dt_milliseconds = (uint8_t)(1000.0f/get_sample_rate()+0.5f);

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
            goto failed;
        }
        hal.scheduler->delay(update_dt_milliseconds);
        num_samples++;
    }
    level_sample /= num_samples;

    if (!_calculate_trim(level_sample, trim_roll, trim_pitch)) {
        goto failed;
    }

    _calibrating = false;
    return true;

failed:
    _calibrating = false;
    return false;
}

/*
  check if the accelerometers are calibrated in 3D and that current number of accels matched number when calibrated
 */
bool AP_InertialSensor::accel_calibrated_ok_all() const
{
    // calibration is not applicable for HIL mode
    if (_hil_mode) {
        return true;
    }

    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!_accel_id_ok[i]) {
            return false;
        }
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset[i].get().is_zero()) {
            return false;
        }
        // exactly 1.0 scaling is extremely unlikely
        const Vector3f &scaling = _accel_scale[i].get();
        if (is_equal(scaling.x,1.0f) && is_equal(scaling.y,1.0f) && is_equal(scaling.z,1.0f)) {
            return false;
        }
        // zero scaling also indicates not calibrated
        if (_accel_scale[i].get().is_zero()) {
            return false;
        }
    }
    for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
        if (_accel_id[i] != 0) {
            // missing accel
            return false;
        }
    }
    
    // check calibrated accels matches number of accels (no unused accels should have offsets or scaling)
    if (get_accel_count() < INS_MAX_INSTANCES) {
        for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
            const Vector3f &scaling = _accel_scale[i].get();
            bool have_scaling = (!is_zero(scaling.x) && !is_equal(scaling.x,1.0f)) || (!is_zero(scaling.y) && !is_equal(scaling.y,1.0f)) || (!is_zero(scaling.z) && !is_equal(scaling.z,1.0f));
            bool have_offsets = !_accel_offset[i].get().is_zero();
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

    return (get_accel_health(instance) && _use[instance]);
}

void
AP_InertialSensor::_init_gyro()
{
    uint8_t num_gyros = MIN(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    Vector3f new_gyro_offset[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return;
    }

    // record we are calibrating
    _calibrating = true;

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // cold start
    hal.console->printf("Init Gyro");

    /*
      we do the gyro calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset[k].set(Vector3f());
        new_gyro_offset[k].zero();
        best_diff[k] = -1.f;
        last_average[k].zero();
        converged[k] = false;
    }

    for(int8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        update();
    }

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

        memset(diff_norm, 0, sizeof(diff_norm));

        hal.console->printf("*");

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
    hal.console->printf("\n");
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            hal.console->printf("gyro[%u] did not converge: diff=%f dps (expected < %f)\n",
                                (unsigned)k,
                                (double)ToDeg(best_diff[k]),
                                (double)GYRO_INIT_MAX_DIFF_DPS);
            _gyro_offset[k] = best_avg[k];
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        } else {
            _gyro_cal_ok[k] = true;
            _gyro_offset[k] = new_gyro_offset[k];
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    // record calibration complete
    _calibrating = false;

    // stop flashing leds
    AP_Notify::flags.initialising = false;
}

// save parameters to eeprom
void AP_InertialSensor::_save_gyro_calibration()
{
    for (uint8_t i=0; i<_gyro_count; i++) {
        _gyro_offset[i].save();
        _gyro_id[i].save();
    }
    for (uint8_t i=_gyro_count; i<INS_MAX_INSTANCES; i++) {
        _gyro_offset[i].set_and_save(Vector3f());
        _gyro_id[i].set_and_save(0);
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

    if (!_hil_mode) {
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

        // clear accumulators
        for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
            _delta_velocity_acc[i].zero();
            _delta_velocity_acc_dt[i] = 0;
            _delta_angle_acc[i].zero();
            _delta_angle_acc_dt[i] = 0;
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
            if (_gyro_healthy[i] && _use[i]) {
                _primary_gyro = i;
                break;
            }
        }
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _use[i]) {
                _primary_accel = i;
                break;
            }
        }
    }

    _have_sample = false;
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
    if (!_hil_mode) {
        // we also wait for at least one backend to have a sample of both
        // accel and gyro. This normally completes immediately.
        bool gyro_available = false;
        bool accel_available = false;
        while (true) {
            for (uint8_t i=0; i<_backend_count; i++) {
                _backends[i]->accumulate();
            }

            for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
                gyro_available |= _new_gyro_data[i];
                accel_available |= _new_accel_data[i];
            }

            if (gyro_available && accel_available) {
                break;
            }

            hal.scheduler->delay_microseconds(100);
        }
    }

    now = AP_HAL::micros();
    if (_hil_mode && _hil.delta_time > 0) {
        _delta_time = _hil.delta_time;
        _hil.delta_time = 0;
    } else {
        _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    }
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
bool AP_InertialSensor::get_delta_angle(uint8_t i, Vector3f &delta_angle) const
{
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
bool AP_InertialSensor::get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const
{
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
  return delta_time for the delta_velocity
 */
float AP_InertialSensor::get_delta_velocity_dt(uint8_t i) const
{
    if (_delta_velocity_valid[i]) {
        return _delta_velocity_dt[i];
    }
    return get_delta_time();
}

/*
  return delta_time for the delta_angle
 */
float AP_InertialSensor::get_delta_angle_dt(uint8_t i) const
{
    if (_delta_angle_valid[i] && _delta_angle_dt[i] > 0) {
        return _delta_angle_dt[i];
    }
    return get_delta_time();
}


/*
  support for setting accel and gyro vectors, for use by HIL
 */
void AP_InertialSensor::set_accel(uint8_t instance, const Vector3f &accel)
{
    if (_accel_count == 0) {
        // we haven't initialised yet
        return;
    }
    if (instance < INS_MAX_INSTANCES) {
        _accel[instance] = accel;
        _accel_healthy[instance] = true;
        if (_accel_count <= instance) {
            _accel_count = instance+1;
        }
        if (!_accel_healthy[_primary_accel]) {
            _primary_accel = instance;
        }
    }
}

void AP_InertialSensor::set_gyro(uint8_t instance, const Vector3f &gyro)
{
    if (_gyro_count == 0) {
        // we haven't initialised yet
        return;
    }
    if (instance < INS_MAX_INSTANCES) {
        _gyro[instance] = gyro;
        _gyro_healthy[instance] = true;
        if (_gyro_count <= instance) {
            _gyro_count = instance+1;
            _gyro_cal_ok[instance] = true;
        }
        if (!_accel_healthy[_primary_accel]) {
            _primary_accel = instance;
        }
    }
}

/*
  set delta time for next ins.update()
 */
void AP_InertialSensor::set_delta_time(float delta_time)
{
    _hil.delta_time = delta_time;
}

/*
  set delta velocity for next update
 */
void AP_InertialSensor::set_delta_velocity(uint8_t instance, float deltavt, const Vector3f &deltav)
{
    if (instance < INS_MAX_INSTANCES) {
        _delta_velocity_valid[instance] = true;
        _delta_velocity[instance] = deltav;
        _delta_velocity_dt[instance] = deltavt;
    }
}

/*
  set delta angle for next update
 */
void AP_InertialSensor::set_delta_angle(uint8_t instance, const Vector3f &deltaa, float deltaat)
{
    if (instance < INS_MAX_INSTANCES) {
        _delta_angle_valid[instance] = true;
        _delta_angle[instance] = deltaa;
        _delta_angle_dt[instance] = deltaat;
    }
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
    if (fabsf(accel.x) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.y) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.z) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS) {
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

    _acal->update();

    if (hal.util->get_soft_armed() && _acal->get_status() != ACCEL_CAL_NOT_STARTED) {
        _acal->cancel();
    }
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
            _accel_offset[i].set_and_save(bias);
            _accel_scale[i].set_and_save(gain);
            _accel_id[i].save();
            _accel_id_ok[i] = true;
        } else {
            _accel_offset[i].set_and_save(Vector3f());
            _accel_scale[i].set_and_save(Vector3f());
        }
    }

    // clear any unused accels
    for (uint8_t i=_accel_count; i<INS_MAX_INSTANCES; i++) {
        _accel_id[i].set_and_save(0);
        _accel_offset[i].set_and_save(Vector3f());
        _accel_scale[i].set_and_save(Vector3f());
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
            _trim_pitch = atan2f(aligned_sample.x, norm(aligned_sample.y, aligned_sample.z));
            _trim_roll = atan2f(-aligned_sample.y, -aligned_sample.z);
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
                _trim_roll = q.get_euler_roll();
                _trim_pitch = q.get_euler_pitch();
                _new_trim = true;
            }
            break;
        default:
            _new_trim = false;
            /* no break */
    }

    if (fabsf(_trim_roll) > radians(10) ||
        fabsf(_trim_pitch) > radians(10)) {
        hal.console->printf("ERR: Trim over maximum of 10 degrees!!");
        _new_trim = false;  //we have either got faulty level during acal or highly misaligned accelerometers
    }

    _accel_cal_requires_reboot = true;
}

void AP_InertialSensor::_acal_event_failure()
{
    for (uint8_t i=0; i<_accel_count; i++) {
        _accel_offset[i].set_and_notify(Vector3f(0,0,0));
        _accel_scale[i].set_and_notify(Vector3f(1,1,1));
    }
}

/*
    Returns true if new valid trim values are available and passes them to reference vars
*/
bool AP_InertialSensor::get_new_trim(float& trim_roll, float &trim_pitch)
{
    if (_new_trim) {
        trim_roll = _trim_roll;
        trim_pitch = _trim_pitch;
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
