#include "AP_InertialSensor_Params.h"

const AP_Param::GroupInfo AP_InertialSensor_Params::var_info[] = {

    // @Param: USE
    // @DisplayName: Use first IMU for attitude, velocity and position estimates
    // @Description: Use first IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE", 1, AP_InertialSensor_Params, _use,  1),

    // @Param: ACC_ID
    // @DisplayName: Accelerometer ID
    // @Description: Accelerometer sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("ACC_ID", 2, AP_InertialSensor_Params, _accel_id, 0),


    // @Param: ACCSCAL_X
    // @DisplayName: Accelerometer scaling of X axis
    // @Description: Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACCSCAL_Y
    // @DisplayName: Accelerometer scaling of Y axis
    // @Description: Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACCSCAL_Z
    // @DisplayName: Accelerometer scaling of Z axis
    // @Description: Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("ACCSCAL",     3, AP_InertialSensor_Params, _accel_scale,  1.0),


    // @Param: ACCOFFS_X
    // @DisplayName: Accelerometer offsets of X axis
    // @Description: Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACCOFFS_Y
    // @DisplayName: Accelerometer offsets of Y axis
    // @Description: Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1

    // @Param: ACCOFFS_Z
    // @DisplayName: Accelerometer offsets of Z axis
    // @Description: Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("ACCOFFS",     4, AP_InertialSensor_Params, _accel_offset, 0),

    // @Param: POS_X
    // @DisplayName: IMU accelerometer X position
    // @Description: X position of the first IMU Accelerometer in body frame. Positive X is forward of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: IMU accelerometer Y position
    // @Description: Y position of the first IMU accelerometer in body frame. Positive Y is to the right of the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: IMU accelerometer Z position
    // @Description: Z position of the first IMU accelerometer in body frame. Positive Z is down from the origin. Attention: The IMU should be located as close to the vehicle c.g. as practical so that the value of this parameter is minimised. Failure to do so can result in noisy navigation velocity measurements due to vibration and IMU gyro noise. If the IMU cannot be moved and velocity noise is a problem, a location closer to the IMU can be used as the body frame origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("POS", 5, AP_InertialSensor_Params, _accel_pos, 0.0f),

    // @Param: ACC_CALTEMP
    // @DisplayName: Calibration temperature for accelerometer
    // @Description: Temperature that the accelerometer was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("ACC_CALTEMP", 6, AP_InertialSensor_Params, caltemp_accel, -300),

    // @Param: GYR_ID
    // @DisplayName: Gyro ID
    // @Description: Gyro sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO("GYR_ID", 7, AP_InertialSensor_Params, _gyro_id, 0),


    // @Param: GYROFFS_X
    // @DisplayName: Gyro offsets of X axis
    // @Description: Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYROFFS_Y
    // @DisplayName: Gyro offsets of Y axis
    // @Description: Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1

    // @Param: GYROFFS_Z
    // @DisplayName: Gyro offsets of Z axis
    // @Description: Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    // @Calibration: 1
    AP_GROUPINFO("GYROFFS",     8, AP_InertialSensor_Params, _gyro_offset,  0),


    // @Param: GYR_CALTEMP
    // @DisplayName: Calibration temperature for gyroscope
    // @Description: Temperature that the gyroscope was calibrated at
    // @User: Advanced
    // @Units: degC
    // @Calibration: 1
    AP_GROUPINFO("GYR_CALTEMP", 9, AP_InertialSensor_Params, caltemp_gyro, -300),

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    // @Group: TCAL_
    // @Path: AP_InertialSensor_tempcal.cpp
    AP_SUBGROUPINFO(tcal, "TCAL_", 10, AP_InertialSensor_Params, AP_InertialSensor_TCal),
#endif
    AP_GROUPEND

};


AP_InertialSensor_Params::AP_InertialSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
