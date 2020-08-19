#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <new>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built.
 */
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Replay)
// copter defaults
#define VELNE_M_NSE_DEFAULT     0.3f
#define VELD_M_NSE_DEFAULT      0.5f
#define POSNE_M_NSE_DEFAULT     1.0f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      3.0E-02f
#define ACC_P_NSE_DEFAULT       6.0E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-04f
#define GSCALE_P_NSE_DEFAULT    5.0E-04f
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
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
#define FLOW_USE_DEFAULT        1

#elif APM_BUILD_TYPE(APM_BUILD_Rover)
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
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
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
#define FLOW_USE_DEFAULT        1

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
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
#define VEL_I_GATE_DEFAULT      500
#define POS_I_GATE_DEFAULT      500
#define HGT_I_GATE_DEFAULT      500
#define MAG_I_GATE_DEFAULT      300
#define MAG_CAL_DEFAULT         0
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_M_NSE_DEFAULT      0.15f
#define FLOW_I_GATE_DEFAULT     500
#define CHECK_SCALER_DEFAULT    150
#define FLOW_USE_DEFAULT        2

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
#define ABIAS_P_NSE_DEFAULT     5.0E-03f
#define MAGB_P_NSE_DEFAULT      1.0E-04f
#define MAGE_P_NSE_DEFAULT      1.0E-03f
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
#define FLOW_USE_DEFAULT        1

#endif // APM_BUILD_DIRECTORY

extern const AP_HAL::HAL& hal;

// Define tuning parameters
const AP_Param::GroupInfo NavEKF2::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable EKF2
    // @Description: This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 0, NavEKF2, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // GPS measurement parameters

    // @Param: GPS_TYPE
    // @DisplayName: GPS mode control
    // @Description: This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = Inhibit GPS use - this can be useful when flying with an optical flow sensor in an environment where GPS quality is poor and subject to large multipath errors.
    // @Values: 0:GPS 3D Vel and 2D Pos, 1:GPS 2D vel and 2D pos, 2:GPS 2D pos, 3:No GPS
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

    // 8 was used for GPS_DELAY

    // Height measurement parameters

    // @Param: ALT_SOURCE
    // @DisplayName: Primary altitude sensor source
    // @Description: Primary height sensor used by the EKF. If a sensor other than Baro is selected and becomes unavailable, then the Baro sensor will be used as a fallback. NOTE: The EK2_RNG_USE_HGT parameter can be used to switch to range-finder when close to the ground in conjunction with EK2_ALT_SOURCE = 0 or 2 (Baro or GPS).
    // @Values: 0:Use Baro, 1:Use Range Finder, 2:Use GPS, 3:Use Range Beacon
    // @User: Advanced
    // @RebootRequired: True
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
    // @Units: ms
    // @RebootRequired: True
    AP_GROUPINFO("HGT_DELAY", 12, NavEKF2, _hgtDelay_ms, 60),

    // Magnetometer measurement parameters

    // @Param: MAG_M_NSE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    // @Units: Gauss
    AP_GROUPINFO("MAG_M_NSE", 13, NavEKF2, _magNoise, MAG_M_NSE_DEFAULT),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer default fusion mode
    // @Description: This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states, when it will use a simpler magnetic heading fusion model that does not use magnetic field states and when it will use an alternative method of yaw determination to the magnetometer. The 3-axis magnetometer fusion is only suitable for use when the external magnetic field environment is stable. EK2_MAG_CAL = 0 uses heading fusion on ground, 3-axis fusion in-flight, and is the default setting for Plane users. EK2_MAG_CAL = 1 uses 3-axis fusion only when manoeuvring. EK2_MAG_CAL = 2 uses heading fusion at all times, is recommended if the external magnetic field is varying and is the default for rovers. EK2_MAG_CAL = 3 uses heading fusion on the ground and 3-axis fusion after the first in-air field and yaw reset has completed, and is the default for copters. EK2_MAG_CAL = 4 uses 3-axis fusion at all times. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK2_MAG_MASK parameter. NOTE: limited operation without a magnetometer or any other yaw sensor is possible by setting all COMPASS_USE, COMPASS_USE2, COMPASS_USE3, etc parameters to 0 with COMPASS_ENABLE set to 1. If this is done, the EK2_GSF_RUN and EK2_GSF_USE masks must be set to the same as EK2_IMU_MASK.
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
    // @Range: 0 127
    // @Increment: 10
    // @User: Advanced
    // @Units: ms
    // @RebootRequired: True
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
    // @Units: Hz
    AP_GROUPINFO("GSCL_P_NSE", 27, NavEKF2, _gyroScaleProcessNoise, GSCALE_P_NSE_DEFAULT),

    // @Param: ABIAS_P_NSE
    // @DisplayName: Accelerometer bias stability (m/s^3)
    // @Description: This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.005
    // @User: Advanced
    // @Units: m/s/s/s
    AP_GROUPINFO("ABIAS_P_NSE", 28, NavEKF2, _accelBiasProcessNoise, ABIAS_P_NSE_DEFAULT),

    // 29 previously used for EK2_MAG_P_NSE parameter that has been replaced with EK2_MAGE_P_NSE and EK2_MAGB_P_NSE

    // @Param: WIND_P_NSE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("WIND_P_NSE", 30, NavEKF2, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind process noise scaler
    // @Description: This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE", 31, NavEKF2, _wndVarHgtRateScale, 0.5f),

    // @Param: GPS_CHECK
    // @DisplayName: GPS preflight check
    // @Description: This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
    // @Bitmask: 0:NSats,1:HDoP,2:speed error,3:position error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed
    // @User: Advanced
    AP_GROUPINFO("GPS_CHECK",    32, NavEKF2, _gpsCheck, 31),

    // @Param: IMU_MASK
    // @DisplayName: Bitmask of active IMUs
    // @Description: 1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.
    // @Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU
    // @User: Advanced
    // @RebootRequired: True
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
    // @Units: m
    AP_GROUPINFO("NOAID_M_NSE", 35, NavEKF2, _noaidHorizNoise, 10.0f),

    // @Param: LOG_MASK
    // @DisplayName: EKF sensor logging IMU mask
    // @Description: This sets the IMU mask of sensors to do full logging for
    // @Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("LOG_MASK", 36, NavEKF2, _logging_mask, 1),

    // control of magentic yaw angle fusion

    // @Param: YAW_M_NSE
    // @DisplayName: Yaw measurement noise (rad)
    // @Description: This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad
    AP_GROUPINFO("YAW_M_NSE", 37, NavEKF2, _yawNoise, 0.5f),

    // @Param: YAW_I_GATE
    // @DisplayName: Yaw measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("YAW_I_GATE", 38, NavEKF2, _yawInnovGate, 300),

    // @Param: TAU_OUTPUT
    // @DisplayName: Output complementary filter time constant (centi-sec)
    // @Description: Sets the time constant of the output complementary filter/predictor in centi-seconds.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    // @Units: cs
    AP_GROUPINFO("TAU_OUTPUT", 39, NavEKF2, _tauVelPosOutput, 25),

    // @Param: MAGE_P_NSE
    // @DisplayName: Earth magnetic field process noise (gauss/s)
    // @Description: This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.
    // @Range: 0.00001 0.01
    // @User: Advanced
    // @Units: Gauss/s
    AP_GROUPINFO("MAGE_P_NSE", 40, NavEKF2, _magEarthProcessNoise, MAGE_P_NSE_DEFAULT),

    // @Param: MAGB_P_NSE
    // @DisplayName: Body magnetic field process noise (gauss/s)
    // @Description: This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.
    // @Range: 0.00001 0.01
    // @User: Advanced
    // @Units: Gauss/s
    AP_GROUPINFO("MAGB_P_NSE", 41, NavEKF2, _magBodyProcessNoise, MAGB_P_NSE_DEFAULT),

    // @Param: RNG_USE_HGT
    // @DisplayName: Range finder switch height percentage
    // @Description: Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFND_MAX_CM). This will not work unless Baro or GPS height is selected as the primary height source vis EK2_ALT_SOURCE = 0 or 2 respectively.  This feature should not be used for terrain following as it is designed  for vertical takeoff and landing with climb above  the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.
    // @Range: -1 70
    // @Increment: 1
    // @User: Advanced
    // @Units: %
    AP_GROUPINFO("RNG_USE_HGT", 42, NavEKF2, _useRngSwHgt, -1),

    // @Param: TERR_GRAD
    // @DisplayName: Maximum terrain gradient
    // @Description: Specifies the maximum gradient of the terrain below the vehicle assumed when it is fusing range finder or optical flow to estimate terrain height.
    // @Range: 0 0.2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TERR_GRAD", 43, NavEKF2, _terrGradMax, 0.1f),

    // @Param: BCN_M_NSE
    // @DisplayName: Range beacon measurement noise (m)
    // @Description: This is the RMS value of noise in the range beacon measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("BCN_M_NSE", 44, NavEKF2, _rngBcnNoise, 1.0f),

    // @Param: BCN_I_GTE
    // @DisplayName: Range beacon measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the range beacon measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("BCN_I_GTE", 45, NavEKF2, _rngBcnInnovGate, 500),

    // @Param: BCN_DELAY
    // @DisplayName: Range beacon measurement delay (msec)
    // @Description: This is the number of msec that the range beacon measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 127
    // @Increment: 10
    // @User: Advanced
    // @Units: ms
    // @RebootRequired: True
    AP_GROUPINFO("BCN_DELAY", 46, NavEKF2, _rngBcnDelay_ms, 50),

    // @Param: RNG_USE_SPD
    // @DisplayName: Range finder max ground speed
    // @Description: The range finder will not be used as the primary height source when the horizontal ground speed is greater than this value.
    // @Range: 2.0 6.0
    // @Increment: 0.5
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("RNG_USE_SPD", 47, NavEKF2, _useRngSwSpd, 2.0f),

    // @Param: MAG_MASK
    // @DisplayName: Bitmask of active EKF cores that will always use heading fusion
    // @Description: 1 byte bitmap of EKF cores that will disable magnetic field states and use simple magnetic heading fusion at all times. This parameter enables specified cores to be used as a backup for flight into an environment with high levels of external magnetic interference which may degrade the EKF attitude estimate when using 3-axis magnetometer fusion. NOTE : Use of a different magnetometer fusion algorithm on different cores makes unwanted EKF core switches due to magnetometer errors more likely.
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAG_MASK", 48, NavEKF2, _magMask, 0),

    // @Param: OGN_HGT_MASK
    // @DisplayName: Bitmask control of EKF reference height correction
    // @Description: When a height sensor other than GPS is used as the primary height source by the EKF, the position of the zero height datum is defined by that sensor and its frame of reference. If a GPS height measurement is also available, then the height of the WGS-84 height datum used by the EKF can be corrected so that the height returned by the getLLH() function is compensated for primary height sensor drift and change in datum over time. The first two bit positions control when the height datum will be corrected. Correction is performed using a Bayes filter and only operates when GPS quality permits. The third bit position controls where the corrections to the GPS reference datum are applied. Corrections can be applied to the local vertical position or to the reported EKF origin height (default).
    // @Bitmask: 0:Correct when using Baro height,1:Correct when using range finder height,2:Apply corrections to local position
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OGN_HGT_MASK", 49, NavEKF2, _originHgtMode, 0),

    // EXTNAV_DELAY was 50

    // @Param: FLOW_USE
    // @DisplayName: Optical flow use bitmask
    // @Description: Controls if the optical flow data is fused into the 24-state navigation estimator OR the 1-state terrain height estimator.
    // @User: Advanced
    // @Values: 0:None,1:Navigation,2:Terrain
    // @RebootRequired: True
    AP_GROUPINFO("FLOW_USE", 51, NavEKF2, _flowUse, FLOW_USE_DEFAULT),

    // @Param: MAG_EF_LIM
    // @DisplayName: EarthField error limit
    // @Description: This limits the difference between the learned earth magnetic field and the earth field from the world magnetic model tables. A value of zero means to disable the use of the WMM tables.
    // @User: Advanced
    // @Range: 0 500
    // @Units: mGauss
    AP_GROUPINFO("MAG_EF_LIM", 52, NavEKF2, _mag_ef_limit, 50),
    
    // @Param: HRT_FILT
    // @DisplayName: Height rate filter crossover frequency
    // @Description: Specifies the crossover frequency of the complementary filter used to calculate the output predictor height rate derivative.
    // @Range: 0.1 30.0
    // @Units: Hz
    // @RebootRequired: False
    AP_GROUPINFO("HRT_FILT", 53, NavEKF2, _hrt_filt_freq, 2.0f),

    // @Param: GSF_RUN_MASK
    // @DisplayName: Bitmask of which EKF-GSF yaw estimators run
    // @Description: 1 byte bitmap of which EKF2 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF2 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK2_GSF_USE, EK2_GSF_DELAY and EK2_GSF_MAXCOUNT parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK2_GSF_USE to 0. 
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_RUN_MASK", 54, NavEKF2, _gsfRunMask, 3),

    // @Param: GSF_USE_MASK
    // @DisplayName: Bitmask of which EKF-GSF yaw estimators are used
    // @Description: 1 byte bitmap of which EKF2 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK2_GSF_RUN parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF2 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF2 instance. Additionally the EKF2 will  initiate a reset internally if navigation is lost for more than EK2_GSF_DELAY milli seconds.
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_USE_MASK", 55, NavEKF2, _gsfUseMask, 3),

    // @Param: GSF_DELAY
    // @DisplayName: Delay from loss of navigation to yaw reset
    // @Description: If the inertial navigation calculation stops following the GPS and other positioning sensors for longer than EK2_GSF_DELAY milli-seconds, then the EKF2 code will generate a reset request internally and reset the yaw to the estimate from the EKF-GSF filter and reset the horizontal velocity and position to the GPS. This reset will not be performed unless the use of the EKF-GSF yaw estimate is enabled via the EK2_GSF_USE parameter.
    // @Range: 500 5000
    // @Increment: 100
    // @Units: ms
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_DELAY", 56, NavEKF2, _gsfResetDelay, 1000),

    // @Param: GSF_RST_MAX
    // @DisplayName: Maximum number of resets to the EKF-GSF yaw estimate allowed
    // @Description: Sets the maximum number of times the EKF2 will be allowed to reset it's yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK2_GSF_USE parameter.
    // @Range: 1 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_RST_MAX", 57, NavEKF2, _gsfResetMaxCount, 2),
    
    AP_GROUPEND
};

NavEKF2::NavEKF2()
{
    AP_Param::setup_object_defaults(this, var_info);
    _ahrs = &AP::ahrs();
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
        AP::logger().Write_Compass(imuSampleTime_us);
        logging.log_compass = false;
    }
    if (logging.log_baro) {
        AP::logger().Write_Baro(imuSampleTime_us);
        logging.log_baro = false;
    }
    if (logging.log_imu) {
        AP::logger().Write_IMUDT(imuSampleTime_us, _logging_mask.get());
        logging.log_imu = false;
    }

    // this is an example of an ad-hoc log in EKF
    // AP::logger().Write("NKA", "TimeUS,X", "Qf", AP_HAL::micros64(), (double)2.4f);
}


// Initialise the filter
bool NavEKF2::InitialiseFilter(void)
{
    // Return immediately if there is insufficient memory
    if (core_malloc_failed) {
        return false;
    }

    initFailure = InitFailures::UNKNOWN;
    if (_enable == 0) {
        if (_ahrs->get_ekf_type() == 2) {
            initFailure = InitFailures::NO_ENABLE;
        }
        return false;
    }
    const AP_InertialSensor &ins = AP::ins();

    imuSampleTime_us = AP_HAL::micros64();

    // remember expected frame time
    _frameTimeUsec = 1e6 / ins.get_loop_rate_hz();

    // expected number of IMU frames per prediction
    _framesPerPrediction = uint8_t((EKF_TARGET_DT / (_frameTimeUsec * 1.0e-6) + 0.5));

    // see if we will be doing logging
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logging.enabled = logger->log_replay();
    }
    
    if (core == nullptr) {

        // don't allow more filters than IMUs
        uint8_t mask = (1U<<ins.get_accel_count())-1;
        _imuMask.set(_imuMask.get() & mask);
        
        // count IMUs from mask
        num_cores = __builtin_popcount(_imuMask);

        // abort if num_cores is zero
        if (num_cores == 0) {
            if (ins.get_accel_count() == 0) {
                initFailure = InitFailures::NO_IMUS;
            } else {
                initFailure = InitFailures::NO_MASK;
            }
            return false;
        }

        // check if there is enough memory to create the EKF cores
        if (hal.util->available_memory() < sizeof(NavEKF2_core)*num_cores + 4096) {
            initFailure = InitFailures::NO_MEM;
            core_malloc_failed = true;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "NavEKF2: not enough memory available");
            return false;
        }

        // try to allocate from CCM RAM, fallback to Normal RAM if not available or full
        core = (NavEKF2_core*)hal.util->malloc_type(sizeof(NavEKF2_core)*num_cores, AP_HAL::Util::MEM_FAST);
        if (core == nullptr) {
            initFailure = InitFailures::NO_MEM;
            core_malloc_failed = true;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "NavEKF2: memory allocation failed");
            return false;
        }

        //Call Constructors on all cores
        for (uint8_t i = 0; i < num_cores; i++) {
            new (&core[i]) NavEKF2_core(this);
        }

        // set the IMU index for the cores
        num_cores = 0;
        for (uint8_t i=0; i<7; i++) {
            if (_imuMask & (1U<<i)) {
                if(!core[num_cores].setup_core(i, num_cores)) {
                    // if any core setup fails, free memory, zero the core pointer and abort
                    hal.util->free_type(core, sizeof(NavEKF2_core)*num_cores, AP_HAL::Util::MEM_FAST);
                    core = nullptr;
                    initFailure = InitFailures::NO_SETUP;
                    gcs().send_text(MAV_SEVERITY_WARNING, "NavEKF2: core %d setup failed", num_cores);
                    return false;
                }
                num_cores++;
            }
        }

        // Set the primary initially to be the lowest index
        primary = 0;
    }

    // invalidate shared origin
    common_origin_valid = false;
    
    // initialise the cores. We return success only if all cores
    // initialise successfully
    bool ret = true;
    for (uint8_t i=0; i<num_cores; i++) {
        ret &= core[i].InitialiseFilterBootstrap();
    }

    // zero the structs used capture reset events
    memset(&yaw_reset_data, 0, sizeof(yaw_reset_data));
    memset((void *)&pos_reset_data, 0, sizeof(pos_reset_data));
    memset(&pos_down_reset_data, 0, sizeof(pos_down_reset_data));

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
    
    const AP_InertialSensor &ins = AP::ins();

    bool statePredictEnabled[num_cores];
    for (uint8_t i=0; i<num_cores; i++) {
        // if we have not overrun by more than 3 IMU frames, and we
        // have already used more than 1/3 of the CPU budget for this
        // loop then suppress the prediction step. This allows
        // multiple EKF instances to cooperate on scheduling
        if (core[i].getFramesSincePredict() < (_framesPerPrediction+3) &&
            (AP_HAL::micros() - ins.get_last_update_usec()) > _frameTimeUsec/3) {
            statePredictEnabled[i] = false;
        } else {
            statePredictEnabled[i] = true;
        }
        core[i].UpdateFilter(statePredictEnabled[i]);
    }

    // If the current core selected has a bad error score or is unhealthy, switch to a healthy core with the lowest fault score
    // Don't start running the check until the primary core has started returned healthy for at least 10 seconds to avoid switching
    // due to initial alignment fluctuations and race conditions
    if (!runCoreSelection) {
        static uint64_t lastUnhealthyTime_us = 0;
        if (!core[primary].healthy() || lastUnhealthyTime_us == 0) {
            lastUnhealthyTime_us = imuSampleTime_us;
        }
        runCoreSelection = (imuSampleTime_us - lastUnhealthyTime_us) > 1E7;
    }
    float primaryErrorScore = core[primary].errorScore();
    if ((primaryErrorScore > 1.0f || !core[primary].healthy()) && runCoreSelection) {
        float lowestErrorScore = 0.67f * primaryErrorScore;
        uint8_t newPrimaryIndex = primary; // index for new primary
        for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {

            if (coreIndex != primary) {
                // an alternative core is available for selection only if healthy and if states have been updated on this time step
                bool altCoreAvailable = core[coreIndex].healthy() && statePredictEnabled[coreIndex];

                // If the primary core is unhealthy and another core is available, then switch now
                // If the primary core is still healthy,then switching is optional and will only be done if
                // a core with a significantly lower error score can be found
                float altErrorScore = core[coreIndex].errorScore();
                if (altCoreAvailable && (!core[newPrimaryIndex].healthy() || altErrorScore < lowestErrorScore)) {
                    newPrimaryIndex = coreIndex;
                    lowestErrorScore = altErrorScore;
                }
            }
        }
        // update the yaw and position reset data to capture changes due to the lane switch
        if (newPrimaryIndex != primary) {
            updateLaneSwitchYawResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosDownResetData(newPrimaryIndex, primary);
            primary = newPrimaryIndex;
            lastLaneSwitch_ms = AP_HAL::millis();
        }
    }

    if (primary != 0 && core[0].healthy() && !hal.util->get_soft_armed()) {
        // when on the ground and disarmed force the first lane. This
        // avoids us ending with with a lottery for which IMU is used
        // in each flight. Otherwise the alignment of the timing of
        // the lane updates with the timing of GPS updates can lead to
        // a lane other than the first one being used as primary for
        // some flights. As different IMUs may have quite different
        // noise characteristics this leads to inconsistent
        // performance
        primary = 0;
    }

    check_log_write();
}

/*
  check if switching lanes will reduce the normalised
  innovations. This is called when the vehicle code is about to
  trigger an EKF failsafe, and it would like to avoid that by
  using a different EKF lane
*/
void NavEKF2::checkLaneSwitch(void)
{
    uint32_t now = AP_HAL::millis();
    if (lastLaneSwitch_ms != 0 && now - lastLaneSwitch_ms < 5000) {
        // don't switch twice in 5 seconds
        return;
    }
    float primaryErrorScore = core[primary].errorScore();
    float lowestErrorScore = primaryErrorScore;
    uint8_t newPrimaryIndex = primary;
    for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {
        if (coreIndex != primary) {
            // an alternative core is available for selection only if healthy and if states have been updated on this time step
            bool altCoreAvailable = core[coreIndex].healthy();
            float altErrorScore = core[coreIndex].errorScore();
            if (altCoreAvailable &&
                altErrorScore < lowestErrorScore &&
                altErrorScore < 0.9) {
                newPrimaryIndex = coreIndex;
                lowestErrorScore = altErrorScore;
            }
        }
    }

    // update the yaw and position reset data to capture changes due to the lane switch
    if (newPrimaryIndex != primary) {
        updateLaneSwitchYawResetData(newPrimaryIndex, primary);
        updateLaneSwitchPosResetData(newPrimaryIndex, primary);
        updateLaneSwitchPosDownResetData(newPrimaryIndex, primary);
        primary = newPrimaryIndex;
        lastLaneSwitch_ms = now;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "NavEKF2: lane switch %u", primary);
    }
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF2::healthy(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].healthy();
}

bool NavEKF2::all_cores_healthy(void) const
{
    if (!core) {
        return false;
    }
    for (uint8_t i = 0; i < num_cores; i++) {
        if (!core[i].healthy()) {
            return false;
        }
    }
    return true;
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

// returns the index of the IMU of the primary core
// return -1 if no primary core selected
int8_t NavEKF2::getPrimaryCoreIMUIndex(void) const
{
    if (!core) {
        return -1;
    }
    return core[primary].getIMUIndex();
}

// Write the last calculated NE position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosNE(int8_t instance, Vector2f &posNE) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getPosNE(posNE);
}

// Write the last calculated D position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosD(int8_t instance, float &posD) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getPosD(posD);
}

// return NED velocity in m/s
void NavEKF2::getVelNED(int8_t instance, Vector3f &vel) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getVelNED(vel);
    }
}

// Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
float NavEKF2::getPosDownDerivative(int8_t instance) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    // return the value calculated from a complementary filter applied to the EKF height and vertical acceleration
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
void NavEKF2::getGyroBias(int8_t instance, Vector3f &gyroBias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroBias(gyroBias);
    }
}

// return body axis gyro scale factor error as a percentage
void NavEKF2::getGyroScaleErrorPercentage(int8_t instance, Vector3f &gyroScale) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroScaleErrorPercentage(gyroScale);
    }
}

// return tilt error convergence metric for the specified instance
void NavEKF2::getTiltError(int8_t instance, float &ang) const
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
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].resetGyroBias();
        }
    }
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool NavEKF2::resetHeightDatum(void)
{
    bool status = true;
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            if (!core[i].resetHeightDatum()) {
                status = false;
            }
        }
    } else {
        status = false;
    }
    return status;
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
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF2::getAccelZBias(int8_t instance, float &zbias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getAccelZBias(zbias);
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF2::getWind(int8_t instance, Vector3f &wind) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getWind(wind);
    }
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF2::getMagNED(int8_t instance, Vector3f &magNED) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF2::getMagXYZ(int8_t instance, Vector3f &magXYZ) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getMagXYZ(magXYZ);
    }
}

// return the magnetometer in use for the specified instance
uint8_t NavEKF2::getActiveMag(int8_t instance) const
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

// Return the latitude and longitude and height used to set the NED origin for the specified instance
// An out of range instance (eg -1) returns data for the primary instance
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF2::getOriginLLH(int8_t instance, struct Location &loc) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (!core) {
        return false;
    }
    return core[instance].getOriginLLH(loc);
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool NavEKF2::setOriginLLH(const Location &loc)
{
    if (!core) {
        return false;
    }
    if (_fusionModeGPS != 3) {
        // we don't allow setting of the EKF origin unless we are
        // flying in non-GPS mode. This is to prevent accidental set
        // of EKF origin with invalid position or height
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF2 refusing set origin");
        return false;
    }
    bool ret = false;
    for (uint8_t i=0; i<num_cores; i++) {
        ret |= core[i].setOriginLLH(loc);
    }
    // return true if any core accepts the new origin
    return ret;
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
void NavEKF2::getEulerAngles(int8_t instance, Vector3f &eulers) const
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
void NavEKF2::getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        Matrix3f mat;
        core[instance].getRotationBodyToNED(mat);
        quat.from_rotation_matrix(mat);
    }
}

// return the quaternions defining the rotation from NED to XYZ (autopilot) axes
void NavEKF2::getQuaternion(int8_t instance, Quaternion &quat) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getQuaternion(quat);
    }
}

// return the innovations for the specified instance
void NavEKF2::getInnovations(int8_t instance, Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }
}

// publish output observer angular, velocity and position tracking error
void NavEKF2::getOutputTrackingError(int8_t instance, Vector3f &error) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getOutputTrackingError(error);
    }
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void NavEKF2::getVariances(int8_t instance, float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
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
// posOffset is the XYZ flow sensor position in the body frame in m
void NavEKF2::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
        }
    }
}

// return data for debugging optical flow fusion
void NavEKF2::getFlowDebug(int8_t instance, float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov,
                           float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFlowDebug(varFlow, gndOffset, flowInnovX, flowInnovY, auxInnov, HAGL, rngInnov, range, gndOffsetErr);
    }
}

// return data for debugging range beacon fusion
bool NavEKF2::getRangeBeaconDebug(int8_t instance, uint8_t &ID, float &rng, float &innov, float &innovVar, float &testRatio, Vector3f &beaconPosNED, float &offsetHigh, float &offsetLow) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        return core[instance].getRangeBeaconDebug(ID, rng, innov, innovVar, testRatio, beaconPosNED, offsetHigh, offsetLow);
    } else {
        return false;
    }
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTakeoffExpected(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTakeoffExpected(val);
        }
    }
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTouchdownExpected(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTouchdownExpected(val);
        }
    }
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference. Use to prevent range finder operation otherwise
// enabled by the combination of EK2_RNG_AID_HGT and EK2_RNG_USE_SPD parameters.
void NavEKF2::setTerrainHgtStable(bool val)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].setTerrainHgtStable(val);
        }
    }

}

/*
  return the filter fault status as a bitmasked integer
  0 = quaternions are NaN
  1 = velocities are NaN
  2 = badly conditioned X magnetometer fusion
  3 = badly conditioned Y magnetometer fusion
  4 = badly conditioned Z magnetometer fusion
  5 = badly conditioned airspeed fusion
  6 = badly conditioned synthetic sideslip fusion
  7 = filter is not initialised
*/
void NavEKF2::getFilterFaults(int8_t instance, uint16_t &faults) const
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
  4 = unassigned
  5 = unassigned
  6 = unassigned
  7 = unassigned
*/
void NavEKF2::getFilterTimeouts(int8_t instance, uint8_t &timeouts) const
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
void NavEKF2::getFilterStatus(int8_t instance, nav_filter_status &status) const
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
void  NavEKF2::getFilterGpsStatus(int8_t instance, nav_gps_status &status) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getFilterGpsStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

// send an EKF_STATUS_REPORT message to GCS
void NavEKF2::send_status_report(mavlink_channel_t chan) const
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

// Returns the amount of yaw angle change (in radians) due to the last yaw angle reset or core selection switch
// Returns the time of the last yaw angle reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastYawResetAngle(float &yawAngDelta)
{
    if (!core) {
        return 0;
    }

    yawAngDelta = 0.0f;

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastYawReset_ms = yaw_reset_data.last_primary_change;

    // There has been a change notification in the primary core that the controller has not consumed
    // or this is a repeated acce
    if (yaw_reset_data.core_changed || yaw_reset_data.last_function_call == now_time_ms) {
        yawAngDelta = yaw_reset_data.core_delta;
        yaw_reset_data.core_changed = false;
    }

    // Record last time controller got the yaw reset
    yaw_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    float temp_yawAng;
    uint32_t lastCoreYawReset_ms = core[primary].getLastYawResetAngle(temp_yawAng);
    if (lastCoreYawReset_ms > lastYawReset_ms) {
        yawAngDelta = wrap_PI(yawAngDelta + temp_yawAng);
        lastYawReset_ms = lastCoreYawReset_ms;
    }

    return lastYawReset_ms;
}

// Returns the amount of NE position change due to the last position reset or core switch in metres
// Returns the time of the last reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastPosNorthEastReset(Vector2f &posDelta)
{
    if (!core) {
        return 0;
    }

    posDelta.zero();

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastPosReset_ms = pos_reset_data.last_primary_change;

    // There has been a change in the primary core that the controller has not consumed
    // allow for multiple consumers on the same frame
    if (pos_reset_data.core_changed || pos_reset_data.last_function_call == now_time_ms) {
        posDelta = pos_reset_data.core_delta;
        pos_reset_data.core_changed = false;
    }

    // Record last time controller got the position reset
    pos_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    Vector2f tempPosDelta;
    uint32_t lastCorePosReset_ms = core[primary].getLastPosNorthEastReset(tempPosDelta);
    if (lastCorePosReset_ms > lastPosReset_ms) {
        posDelta = posDelta + tempPosDelta;
        lastPosReset_ms = lastCorePosReset_ms;
    }

    return lastPosReset_ms;
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
        return initFailureReason[int(initFailure)];
    }
    for (uint8_t i = 0; i < num_cores; i++) {
        const char * failure = core[i].prearm_failure_reason();
        if (failure != nullptr) {
            return failure;
        }
    }
    return nullptr;
}

// Returns the amount of vertical position change due to the last reset or core switch in metres
// Returns the time of the last reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF2::getLastPosDownReset(float &posDelta)
{
    if (!core) {
        return 0;
    }

    posDelta = 0.0f;

    // Do the conversion to msec in one place
    uint32_t now_time_ms = imuSampleTime_us / 1000;

    // The last time we switched to the current primary core is the first reset event
    uint32_t lastPosReset_ms = pos_down_reset_data.last_primary_change;

    // There has been a change in the primary core that the controller has not consumed
    // allow for multiple consumers on the same frame
    if (pos_down_reset_data.core_changed || pos_down_reset_data.last_function_call == now_time_ms) {
        posDelta = pos_down_reset_data.core_delta;
        pos_down_reset_data.core_changed = false;
    }

    // Record last time controller got the position reset
    pos_down_reset_data.last_function_call = now_time_ms;

    // There has been a reset inside the core since we switched so update the time and delta
    float tempPosDelta;
    uint32_t lastCorePosReset_ms = core[primary].getLastPosDownReset(tempPosDelta);
    if (lastCorePosReset_ms > lastPosReset_ms) {
        posDelta += tempPosDelta;
        lastPosReset_ms = lastCorePosReset_ms;
    }

    return lastPosReset_ms;
}

// update the yaw reset data to capture changes due to a lane switch
void NavEKF2::updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary)
{
    Vector3f eulers_old_primary, eulers_new_primary;
    float old_yaw_delta;

    // If core yaw reset data has been consumed reset delta to zero
    if (!yaw_reset_data.core_changed) {
        yaw_reset_data.core_delta = 0;
    }

    // If current primary has reset yaw after controller got it, add it to the delta
    if (core[old_primary].getLastYawResetAngle(old_yaw_delta) > yaw_reset_data.last_function_call) {
        yaw_reset_data.core_delta += old_yaw_delta;
    }

    // Record the yaw delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getEulerAngles(eulers_old_primary);
    core[new_primary].getEulerAngles(eulers_new_primary);
    yaw_reset_data.core_delta = wrap_PI(eulers_new_primary.z - eulers_old_primary.z + yaw_reset_data.core_delta);
    yaw_reset_data.last_primary_change = imuSampleTime_us / 1000;
    yaw_reset_data.core_changed = true;

}

// update the position reset data to capture changes due to a lane switch
void NavEKF2::updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary)
{
    Vector2f pos_old_primary, pos_new_primary, old_pos_delta;

    // If core position reset data has been consumed reset delta to zero
    if (!pos_reset_data.core_changed) {
        pos_reset_data.core_delta.zero();
    }

    // If current primary has reset position after controller got it, add it to the delta
    if (core[old_primary].getLastPosNorthEastReset(old_pos_delta) > pos_reset_data.last_function_call) {
        pos_reset_data.core_delta += old_pos_delta;
    }

    // Record the position delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getPosNE(pos_old_primary);
    core[new_primary].getPosNE(pos_new_primary);
    pos_reset_data.core_delta = pos_new_primary - pos_old_primary + pos_reset_data.core_delta;
    pos_reset_data.last_primary_change = imuSampleTime_us / 1000;
    pos_reset_data.core_changed = true;

}

// Update the vertical position reset data to capture changes due to a core switch
// This should be called after the decision to switch cores has been made, but before the
// new primary EKF update has been run
void NavEKF2::updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary)
{
    float posDownOldPrimary, posDownNewPrimary, oldPosDownDelta;

    // If core position reset data has been consumed reset delta to zero
    if (!pos_down_reset_data.core_changed) {
        pos_down_reset_data.core_delta = 0.0f;
    }

    // If current primary has reset position after controller got it, add it to the delta
    if (core[old_primary].getLastPosDownReset(oldPosDownDelta) > pos_down_reset_data.last_function_call) {
        pos_down_reset_data.core_delta += oldPosDownDelta;
    }

    // Record the position delta between current core and new primary core and the timestamp of the core change
    // Add current delta in case it hasn't been consumed yet
    core[old_primary].getPosD(posDownOldPrimary);
    core[new_primary].getPosD(posDownNewPrimary);
    pos_down_reset_data.core_delta = posDownNewPrimary - posDownOldPrimary + pos_down_reset_data.core_delta;
    pos_down_reset_data.last_primary_change = imuSampleTime_us / 1000;
    pos_down_reset_data.core_changed = true;

}

/*
  get timing statistics structure
*/
void NavEKF2::getTimingStatistics(int8_t instance, struct ekf_timing &timing) const
{
    if (instance < 0 || instance >= num_cores) {
        instance = primary;
    }
    if (core) {
        core[instance].getTimingStatistics(timing);
    } else {
        memset(&timing, 0, sizeof(timing));
    }
}

/*
 * Write position and quaternion data from an external navigation system
 *
 * pos        : XYZ position (m) in a RH navigation frame with the Z axis pointing down and XY axes horizontal. Frame must be aligned with NED if the magnetomer is being used for yaw.
 * quat       : quaternion describing the rotation from navigation frame to body frame
 * posErr     : 1-sigma spherical position error (m)
 * angErr     : 1-sigma spherical angle error (rad)
 * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
 * delay_ms   : average delay of external nav system measurements relative to inertial measurements
 * resetTime_ms : system time of the last position reset request (mSec)
 *
 * Sensor offsets are pulled directly from the AP_VisualOdom library
 *
*/
void NavEKF2::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
        }
    }
}

// check if external navigation is being used for yaw observation
bool NavEKF2::isExtNavUsedForYaw() const
{
    if (core) {
        return core[primary].isExtNavUsedForYaw();
    }
    return false;
}

void NavEKF2::requestYawReset(void)
{
    for (uint8_t i = 0; i < num_cores; i++) {
        core[primary].EKFGSF_requestYawReset();
    }
}

// return data for debugging EKF-GSF yaw estimator
// return false if data not available
bool NavEKF2::getDataEKFGSF(int8_t instance, float &yaw_composite, float &yaw_composite_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        if (core[instance].getDataEKFGSF(yaw_composite, yaw_composite_variance, yaw, innov_VN, innov_VE, weight)) {
            return true;
        }
    }
    return false;
}

// Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
void NavEKF2::writeDefaultAirSpeed(float airspeed)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeDefaultAirSpeed(airspeed);
        }
    }
}

/* Write velocity data from an external navigation system
 * vel : velocity in NED (m)
 * err : velocity error (m/s) 
 * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
 * delay_ms   : average delay of external nav system measurements relative to inertial measurements
 */
void NavEKF2::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
        }
    }
}
