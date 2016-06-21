/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2_core.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built.
 */
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Replay)
// copter defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     1.0E-03f
#define MAG_P_NSE_DEFAULT       2.5E-02f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    100

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
// rover defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     1.0E-03f
#define MAG_P_NSE_DEFAULT       2.5E-02f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         2
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    100

#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// plane defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     1.0E-03f
#define MAG_P_NSE_DEFAULT       2.5E-02f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         0
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    150

#else
// build type not specified, use copter defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     1.0E-03f
#define MAG_P_NSE_DEFAULT       2.5E-02f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.25f
#define FLOW_I_GATE_DEFAULT     300
#define CHECK_SCALER_DEFAULT    100

#endif // APM_BUILD_DIRECTORY

extern const AP_HAL::HAL& hal;

// Define tuning parameters
const AP_Param::GroupInfo NavEKF2::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable EKF2
    // @Description: This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, NavEKF2, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // GPS measurement parameters

    // @Param: GPS_TYPE
    // @DisplayName: GPS mode control
    // @Description: This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
    // @Values: 0:GPS 3D Vel and 2D Pos, 1:GPS 2D vel and 2D pos, 2:GPS 2D pos, 3:No GPS use optical flow
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE", 1, NavEKF2, _fusionModeGPS, 0),

    // @Param: VELNE_M_NSE
    // @DisplayName: GPS horizontal velocity measurement noise (m/s)
    // @Description: This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VELNE_M_NSE", 2, NavEKF2, _gpsHorizVelNoise, VELNE_M_NSE_DEFAULT),

    // @Param: VELD_M_NSE
    // @DisplayName: GPS vertical velocity measurement noise (m/s)
    // @Description: This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VELD_M_NSE", 3, NavEKF2, _gpsVertVelNoise, VELD_M_NSE_DEFAULT),

    // @Param: VEL_I_GATE
    // @DisplayName: GPS velocity innovation gate size
    // @Description: This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("VEL_I_GATE", 4, NavEKF2, _gpsVelInnovGate, VEL_I_GATE_DEFAULT),

    // @Param: POSNE_M_NSE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("POSNE_M_NSE", 5, NavEKF2, _gpsHorizPosNoise, POSNE_M_NSE_DEFAULT),

    // @Param: POS_I_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("POS_I_GATE", 6, NavEKF2, _gpsPosInnovGate, POS_I_GATE_DEFAULT),

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.
    // @Range: 10 100
    // @Increment: 5
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("GLITCH_RAD", 7, NavEKF2, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // @Param: GPS_DELAY
    // @DisplayName: GPS measurement delay (msec)
    // @Description: This is the number of msec that the GPS measurements lag behind the inertial measurements.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: msec
    AP_GROUPINFO("GPS_DELAY", 8, NavEKF2, _gpsDelay_ms, 220),

    // Height measurement parameters

    // @Param: ALT_SOURCE
    // @DisplayName: Primary height source
    // @Description: This parameter controls which height sensor is used by the EKF. If the selected optionn cannot be used, it will default to Baro as the primary height source. Setting 0 will use the baro altitude at all times. Setting 1 uses the range finder and is only available in combination with optical flow navigation (EK2_GPS_TYPE = 3). Setting 2 uses GPS.
    // @Values: 0:Use Baro, 1:Use Range Finder, 2:Use GPS
    // @User: Advanced
    AP_GROUPINFO("ALT_SOURCE", 9, NavEKF2, _altSource, 0),

    // @Param: ALT_M_NSE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("ALT_M_NSE", 10, NavEKF2, _baroAltNoise, ALT_M_NSE_DEFAULT),

    // @Param: HGT_I_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("HGT_I_GATE", 11, NavEKF2, _hgtInnovGate, HGT_I_GATE_DEFAULT),

    // @Param: HGT_DELAY
    // @DisplayName: Height measurement delay (msec)
    // @Description: This is the number of msec that the Height measurements lag behind the inertial measurements.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: msec
    AP_GROUPINFO("HGT_DELAY", 12, NavEKF2, _hgtDelay_ms, 60),

    // Magnetometer measurement parameters

    // @Param: MAG_M_NSE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    // @Units: gauss
    AP_GROUPINFO("MAG_M_NSE", 13, NavEKF2, _magNoise, MAG_M_NSE_DEFAULT),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer calibration mode
    // @Description: EKF_MAG_CAL = 0 enables calibration when airborne and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration when manoeuvreing. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition, is recommended if the external magnetic field is varying and is the default for rovers. EKF_MAG_CAL = 3 enables calibration when the first in-air field and yaw reset has completed and is the default for copters. EKF_MAG_CAL = 4 enables calibration all the time. This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states. This model is only suitable for use when the external magnetic field environment is stable.
    // @Values: 0:When flying,1:When manoeuvring,2:Never,3:After first climb yaw reset,4:Always
    // @User: Advanced
    AP_GROUPINFO("MAG_CAL", 14, NavEKF2, _magCal, MAG_CAL_DEFAULT),

    // @Param: MAG_I_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("MAG_I_GATE", 15, NavEKF2, _magInnovGate, MAG_I_GATE_DEFAULT),

    // Airspeed measurement parameters

    // @Param: EAS_M_NSE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("EAS_M_NSE", 16, NavEKF2, _easNoise, 1.4f),

    // @Param: EAS_I_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("EAS_I_GATE", 17, NavEKF2, _tasInnovGate, 400),

    // Rangefinder measurement parameters

    // @Param: RNG_M_NSE
    // @DisplayName: Range finder measurement noise (m)
    // @Description: This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("RNG_M_NSE", 18, NavEKF2, _rngNoise, 0.5f),

    // @Param: RNG_I_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("RNG_I_GATE", 19, NavEKF2, _rngInnovGate, 500),

    // Optical flow measurement parameters

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 4.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("MAX_FLOW", 20, NavEKF2, _maxFlowRate, 2.5f),

    // @Param: FLOW_M_NSE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("FLOW_M_NSE", 21, NavEKF2, _flowNoise, FLOW_M_NSE_DEFAULT),

    // @Param: FLOW_I_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("FLOW_I_GATE", 22, NavEKF2, _flowInnovGate, FLOW_I_GATE_DEFAULT),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: msec
    AP_GROUPINFO("FLOW_DELAY", 23, NavEKF2, _flowDelay_ms, FLOW_MEAS_DELAY),

    // State and Covariance Predition Parameters

    // @Param: GYRO_P_NSE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.0001 0.1
    // @Increment: 0.0001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GYRO_P_NSE", 24, NavEKF2, _gyrNoise, GYRO_P_NSE_DEFAULT),

    // @Param: ACC_P_NSE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.01 1.0
    // @Increment: 0.01
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_P_NSE", 25, NavEKF2, _accNoise, ACC_P_NSE_DEFAULT),

    // @Param: GBIAS_P_NSE
    // @DisplayName: Rate gyro bias stability (rad/s/s)
    // @Description: This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: rad/s/s
    AP_GROUPINFO("GBIAS_P_NSE", 26, NavEKF2, _gyroBiasProcessNoise, GBIAS_P_NSE_DEFAULT),

    // @Param: GSCL_P_NSE
    // @DisplayName: Rate gyro scale factor stability (1/s)
    // @Description: This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
    // @Range: 0.000001 0.001
    // @User: Advanced
    // @Units: 1/s
    AP_GROUPINFO("GSCL_P_NSE", 27, NavEKF2, _gyroScaleProcessNoise, GSCALE_P_NSE_DEFAULT),

    // @Param: ABIAS_P_NSE
    // @DisplayName: Accelerometer bias stability (m/s^3)
    // @Description: This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: m/s/s/s
    AP_GROUPINFO("ABIAS_P_NSE", 28, NavEKF2, _accelBiasProcessNoise, ABIAS_P_NSE_DEFAULT),

    // @Param: MAG_P_NSE
    // @DisplayName: Magnetic field process noise (gauss/s)
    // @Description: This state process noise controls the growth of magnetic field state error estimates. Increasing it makes magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    // @Units: gauss/s
    AP_GROUPINFO("MAG_P_NSE", 29, NavEKF2, _magProcessNoise, MAG_P_NSE_DEFAULT),

    // @Param: WIND_P_NSE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("WIND_P_NSE", 30, NavEKF2, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind procss noise scaler
    // @Description: This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE", 31, NavEKF2, _wndVarHgtRateScale, 0.5f),

    // @Param: GPS_CHECK
    // @DisplayName: GPS preflight check
    // @Description: This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
    // @Bitmask: 0:NSats,1:HDoP,2:speed error,3:horiz pos error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed
    // @User: Advanced
    AP_GROUPINFO("GPS_CHECK",    32, NavEKF2, _gpsCheck, 31),

    // @Param: IMU_MASK
    // @DisplayName: Bitmask of active IMUs
    // @Description: 1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.
    // @Range: 1 127
    // @User: Advanced
    AP_GROUPINFO("IMU_MASK",     33, NavEKF2, _imuMask, 3),
    
    // @Param: CHECK_SCALE
    // @DisplayName: GPS accuracy check scaler (%)
    // @Description: This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.
    // @Range: 50 200
    // @User: Advanced
    // @Units: %
    AP_GROUPINFO("CHECK_SCALE", 34, NavEKF2, _gpsCheckScaler, CHECK_SCALER_DEFAULT),

    // @Param: NOAID_M_NSE
    // @DisplayName: Non-GPS operation position uncertainty (m)
    // @Description: This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.
    // @Range: 0.5 50.0
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("NOAID_M_NSE", 35, NavEKF2, _noaidHorizNoise, 10.0f),

    // @Param: LOG_MASK
    // @DisplayName: EKF sensor logging IMU mask
    // @Description: This sets the IMU mask of sensors to do full logging for
    // @Values: 0:Disabled,1:FirstIMU,3:FirstAndSecondIMU,7:AllIMUs
    // @User: Advanced
    AP_GROUPINFO("LOG_MASK", 36, NavEKF2, _logging_mask, 1),
    
    AP_GROUPEND
};

NavEKF2::NavEKF2(const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng) :
    _ahrs(ahrs),
    _baro(baro),
    _rng(rng),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    magDelay_ms(60),                // Magnetometer measurement delay (msec)
    tasDelay_ms(240),               // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS_ms(10000),   // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS_ms(7000),     // GPS retry time without airspeed measurements (msec)
    gpsFailTimeWithFlow_ms(1000),   // If we have no GPS for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    hgtRetryTimeMode0_ms(10000),    // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12_ms(5000),    // Height retry time without vertical velocity measurement (msec)
    tasRetryTime_ms(5000),          // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.005f),        // scale factor applied to magnetometer variance due to angular rate and measurement timing jitter. Assume timing jitter of 10msec
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    hgtAvg_ms(100),                 // average number of msec between height measurements
    betaAvg_ms(100),                // average number of msec between synthetic sideslip measurements
    covTimeStepMax(0.1f),           // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),      // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f),      // scaler applied to the barometer observation variance when operating in ground effect
    gndGradientSigma(50),           // RMS terrain gradient percentage assumed by the terrain height estimation
    fusionTimeStep_ms(10)           // The minimum number of msec between covariance prediction and fusion operations
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  see if we should log some sensor data
 */
void NavEKF2::check_log_write(void)
{
    if (!have_ekf_logging()) {
        return;
    }
    if (logging.log_compass) {
        DataFlash_Class::instance()->Log_Write_Compass(*_ahrs->get_compass(), imuSampleTime_us);
        logging.log_compass = false;
    }
    if (logging.log_gps) {
        DataFlash_Class::instance()->Log_Write_GPS(_ahrs->get_gps(), 0, imuSampleTime_us);
        logging.log_gps = false;
    }
    if (logging.log_baro) {
        DataFlash_Class::instance()->Log_Write_Baro(_baro, imuSampleTime_us);
        logging.log_baro = false;
    }
    if (logging.log_imu) {
        const AP_InertialSensor &ins = _ahrs->get_ins();
        DataFlash_Class::instance()->Log_Write_IMUDT(ins, imuSampleTime_us, _logging_mask.get());
        logging.log_imu = false;
    }

    // this is an example of an ad-hoc log in EKF
    // DataFlash_Class::instance()->Log_Write("NKA", "TimeUS,X", "Qf", AP_HAL::micros64(), (double)2.4f);
}


// Initialise the filter
bool NavEKF2::InitialiseFilter(void)
{
    if (_enable == 0) {
        return false;
    }

    imuSampleTime_us = AP_HAL::micros64();

    // see if we will be doing logging
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash != nullptr) {
        logging.enabled = dataflash->log_replay();
    }
    
    if (core == nullptr) {

        // don't run multiple filters for 1 IMU
        const AP_InertialSensor &ins = _ahrs->get_ins();
        uint8_t mask = (1U<<ins.get_accel_count())-1;
        _imuMask.set(_imuMask.get() & mask);
        
        // count IMUs from mask
        num_cores = 0;
        for (uint8_t i=0; i<7; i++) {
            if (_imuMask & (1U<<i)) {
                num_cores++;
            }
        }

        if (hal.util->available_memory() < sizeof(NavEKF2_core)*num_cores + 4096) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "NavEKF2: not enough memory");
            _enable.set(0);
            return false;
        }
        
        core = new NavEKF2_core[num_cores];
        if (core == nullptr) {
            _enable.set(0);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "NavEKF2: allocation failed");
            return false;
        }

        // set the IMU index for the cores
        num_cores = 0;
        for (uint8_t i=0; i<7; i++) {
            if (_imuMask & (1U<<i)) {
                if(!core[num_cores].setup_core(this, i, num_cores)) {
                    return false;
                }
                num_cores++;
            }
        }

        // Set the primary initially to be the lowest index
        primary = 0;
    }

    // initialse the cores. We return success only if all cores
    // initialise successfully
    bool ret = true;
    for (uint8_t i=0; i<num_cores; i++) {
        ret &= core[i].InitialiseFilterBootstrap();
    }

    check_log_write();
    return ret;
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2::UpdateFilter(void)
{
    if (!core) {
        return;
    }

    imuSampleTime_us = AP_HAL::micros64();
    
    const AP_InertialSensor &ins = _ahrs->get_ins();

    for (uint8_t i=0; i<num_cores; i++) {
        // if the previous core has only recently finished a new state prediction cycle, then
        // don't start a new cycle to allow time for fusion operations to complete if the update
        // rate is higher than 200Hz
        bool statePredictEnabled;
        if ((i > 0) && (core[i-1].getFramesSincePredict() < 2) && (ins.get_sample_rate() > 200)) {
            statePredictEnabled = false;
        } else {
            statePredictEnabled = true;
        }
        core[i].UpdateFilter(statePredictEnabled);
    }

    // If the current core selected has a bad fault score or is unhealthy, switch to a healthy core with the lowest fault score
    if (core[primary].faultScore() > 0.0f || !core[primary].healthy()) {
        float score = 1e9f;
        for (uint8_t i=0; i<num_cores; i++) {
            if (core[i].healthy()) {
                float tempScore = core[i].faultScore();
                if (tempScore < score) {
                    primary = i;
                    score = tempScore;
                }
            }
        }
    }

    check_log_write();
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF2::healthy(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].healthy();
}

// returns the index of the primary core
// return -1 if no primary core selected
int8_t NavEKF2::getPrimaryCoreIndex(void) const
{
    if (!core) {
        return -1;
    }
    return primary;
}


// Return the last calculated NED position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosNED(int8_t instance, Vector3f &pos)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getPosNED(pos);
}

// return NED velocity in m/s
void NavEKF2::getVelNED(int8_t instance, Vector3f &vel)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getVelNED(vel);
    }
}

// Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
float NavEKF2::getPosDownDerivative(int8_t instance)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    // return the value calculated from a complementary filer applied to the EKF height and vertical acceleration
    if (core) {
        return core[instance].getPosDownDerivative();
    }
    return 0.0f;
}

// This returns the specific forces in the NED frame
void NavEKF2::getAccelNED(Vector3f &accelNED) const
{
    if (core) {
        core[primary].getAccelNED(accelNED);
    }
}

// return body axis gyro bias estimates in rad/sec
void NavEKF2::getGyroBias(int8_t instance, Vector3f &gyroBias)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroBias(gyroBias);
    }
}

// return body axis gyro scale factor error as a percentage
void NavEKF2::getGyroScaleErrorPercentage(int8_t instance, Vector3f &gyroScale)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroScaleErrorPercentage(gyroScale);
    }
}

// return tilt error convergence metric for the specified instance
void NavEKF2::getTiltError(int8_t instance, float &ang)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getTiltError(ang);
    }
}

// reset body axis gyro bias estimates
void NavEKF2::resetGyroBias(void)
{
    if (core) {
        core[primary].resetGyroBias();
    }
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool NavEKF2::resetHeightDatum(void)
{
    if (!core) {
        return false;
    }
    return core[primary].resetHeightDatum();
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming as it will only be actioned when the filter is in static mode
// This command is forgotten by the EKF each time it goes back into static mode (eg the vehicle disarms)
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF2::setInhibitGPS(void)
{
    if (!core) {
        return 0;
    }
    return core[primary].setInhibitGPS();
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF2::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (core) {
        core[primary].getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    }
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF2::getAccelZBias(int8_t instance, float &zbias)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getAccelZBias(zbias);
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF2::getWind(int8_t instance, Vector3f &wind)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getWind(wind);
    }
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF2::getMagNED(int8_t instance, Vector3f &magNED)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF2::getMagXYZ(int8_t instance, Vector3f &magXYZ)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagXYZ(magXYZ);
    }
}

// return the magnetometer in use for the specified instance
uint8_t NavEKF2::getActiveMag(int8_t instance)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        return core[instance].getActiveMag();
    } else {
        return 255;
    }
}

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool NavEKF2::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    if (!core) {
        return false;
    }
    // try the primary first, else loop through all of the cores and return when one has offsets for this mag instance
    if (core[primary].getMagOffsets(mag_idx, magOffsets)) {
        return true;
    }
    for (uint8_t i=0; i<num_cores; i++) {
        if(core[i].getMagOffsets(mag_idx, magOffsets)) {
            return true;
        }
    }
    return false;
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF2::getLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core[primary].getLLH(loc);
}

// return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF2::getOriginLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core[primary].getOriginLLH(loc);
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calcualted by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool NavEKF2::setOriginLLH(struct Location &loc)
{
    if (!core) {
        return false;
    }
    return core[primary].setOriginLLH(loc);
}

// return estimated height above ground level
// return false if ground height is not being estimated.
bool NavEKF2::getHAGL(float &HAGL) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHAGL(HAGL);
}

// return the Euler roll, pitch and yaw angle in radians for the specified instance
void NavEKF2::getEulerAngles(int8_t instance, Vector3f &eulers)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getEulerAngles(eulers);
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF2::getRotationBodyToNED(Matrix3f &mat) const
{
    if (core) {
        core[primary].getRotationBodyToNED(mat);
    }
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF2::getQuaternion(Quaternion &quat) const
{
    if (core) {
        core[primary].getQuaternion(quat);
    }
}

// return the innovations for the specified instance
void NavEKF2::getInnovations(int8_t instance, Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void NavEKF2::getVariances(int8_t instance, float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }
}

// should we use the compass? This is public so it can be used for
// reporting via ahrs.use_compass()
bool NavEKF2::use_compass(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].use_compass();
}

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
// msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
void NavEKF2::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
        }
    }
}

// return data for debugging optical flow fusion
void NavEKF2::getFlowDebug(int8_t instance, float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov,
                           float &HAGL, float &rngInnov, float &range, float &gndOffsetErr)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFlowDebug(varFlow, gndOffset, flowInnovX, flowInnovY, auxInnov, HAGL, rngInnov, range, gndOffsetErr);
    }
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTakeoffExpected(bool val)
{
    if (core) {
        core[primary].setTakeoffExpected(val);
    }
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTouchdownExpected(bool val)
{
    if (core) {
        core[primary].setTouchdownExpected(val);
    }
}

/*
  return the filter fault status as a bitmasked integer
  0 = quaternions are NaN
  1 = velocities are NaN
  2 = badly conditioned X magnetometer fusion
  3 = badly conditioned Y magnetometer fusion
  5 = badly conditioned Z magnetometer fusion
  6 = badly conditioned airspeed fusion
  7 = badly conditioned synthetic sideslip fusion
  7 = filter is not initialised
*/
void NavEKF2::getFilterFaults(int8_t instance, uint16_t &faults)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterFaults(faults);
    } else {
        faults = 0;
    }
}

/*
  return filter timeout status as a bitmasked integer
  0 = position measurement timeout
  1 = velocity measurement timeout
  2 = height measurement timeout
  3 = magnetometer measurement timeout
  5 = unassigned
  6 = unassigned
  7 = unassigned
  7 = unassigned
*/
void NavEKF2::getFilterTimeouts(int8_t instance, uint8_t &timeouts)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterTimeouts(timeouts);
    } else {
        timeouts = 0;
    }
}

/*
  return filter status flags
*/
void NavEKF2::getFilterStatus(int8_t instance, nav_filter_status &status)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

/*
return filter gps quality check status
*/
void  NavEKF2::getFilterGpsStatus(int8_t instance, nav_gps_status &status)
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterGpsStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

// send an EKF_STATUS_REPORT message to GCS
void NavEKF2::send_status_report(mavlink_channel_t chan)
{
    if (core) {
        core[primary].send_status_report(chan);
    }
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF2::getHeightControlLimit(float &height) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHeightControlLimit(height);
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF2::getLastYawResetAngle(float &yawAng) const
{
    if (!core) {
        return 0;
    }
    return core[primary].getLastYawResetAngle(yawAng);
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF2::getLastPosNorthEastReset(Vector2f &pos) const
{
    if (!core) {
        return 0;
    }
    return core[primary].getLastPosNorthEastReset(pos);
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF2::getLastVelNorthEastReset(Vector2f &vel) const
{
    if (!core) {
        return 0;
    }
    return core[primary].getLastVelNorthEastReset(vel);
}

// report the reason for why the backend is refusing to initialise
const char *NavEKF2::prearm_failure_reason(void) const
{
    if (!core) {
        return nullptr;
    }
    return core[primary].prearm_failure_reason();
}

#endif //HAL_CPU_CLASS
