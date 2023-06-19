#include "AP_NavEKF3.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_DAL/AP_DAL.h"

#include <new>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built.
 */
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_Replay)
// copter defaults
#define VELNE_M_NSE_DEFAULT     0.3f
#define VELD_M_NSE_DEFAULT      0.5f
#define POSNE_M_NSE_DEFAULT     0.5f
#define ALT_M_NSE_DEFAULT       2.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      1.5E-02f
#define ACC_P_NSE_DEFAULT       3.5E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-03f
#define ABIAS_P_NSE_DEFAULT     2.0E-02f
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
#define WIND_P_NSE_DEFAULT      0.2

#elif APM_BUILD_TYPE(APM_BUILD_Rover)
// rover defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     0.5f
#define ALT_M_NSE_DEFAULT       2.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      1.5E-02f
#define ACC_P_NSE_DEFAULT       3.5E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-03f
#define ABIAS_P_NSE_DEFAULT     2.0E-02f
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
#define WIND_P_NSE_DEFAULT      0.1

#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// plane defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     0.5f
#define ALT_M_NSE_DEFAULT       3.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      1.5E-02f
#define ACC_P_NSE_DEFAULT       3.5E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-03f
#define ABIAS_P_NSE_DEFAULT     2.0E-02f
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
#define WIND_P_NSE_DEFAULT      0.1

#else
// build type not specified, use copter defaults
#define VELNE_M_NSE_DEFAULT     0.5f
#define VELD_M_NSE_DEFAULT      0.7f
#define POSNE_M_NSE_DEFAULT     0.5f
#define ALT_M_NSE_DEFAULT       2.0f
#define MAG_M_NSE_DEFAULT       0.05f
#define GYRO_P_NSE_DEFAULT      1.5E-02f
#define ACC_P_NSE_DEFAULT       3.5E-01f
#define GBIAS_P_NSE_DEFAULT     1.0E-03f
#define ABIAS_P_NSE_DEFAULT     2.0E-02f
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
#define WIND_P_NSE_DEFAULT      0.1

#endif // APM_BUILD_DIRECTORY

#ifndef EK3_PRIMARY_DEFAULT
#define EK3_PRIMARY_DEFAULT 0
#endif

// This allows boards to default to using a specified number of IMUs and EKF lanes
#ifndef HAL_EKF_IMU_MASK_DEFAULT
#define HAL_EKF_IMU_MASK_DEFAULT 3       // Default to using two IMUs
#endif

// Define tuning parameters
const AP_Param::GroupInfo NavEKF3::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable EKF3
    // @Description: This enables EKF3. Enabling EKF3 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=3. A reboot or restart will need to be performed after changing the value of EK3_ENABLE for it to take effect.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 0, NavEKF3, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // GPS measurement parameters

    // 1 was GPS_TYPE

    // @Param: VELNE_M_NSE
    // @DisplayName: GPS horizontal velocity measurement noise (m/s)
    // @Description: This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VELNE_M_NSE", 2, NavEKF3, _gpsHorizVelNoise, VELNE_M_NSE_DEFAULT),

    // @Param: VELD_M_NSE
    // @DisplayName: GPS vertical velocity measurement noise (m/s)
    // @Description: This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VELD_M_NSE", 3, NavEKF3, _gpsVertVelNoise, VELD_M_NSE_DEFAULT),

    // @Param: VEL_I_GATE
    // @DisplayName: GPS velocity innovation gate size
    // @Description: This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted. If EK3_GLITCH_RAD set to 0 the velocity innovations will be clipped instead of rejected if they exceed the gate size and a smaller value of EK3_VEL_I_GATE not exceeding 300 is recommended to limit the effect of GPS transient errors.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("VEL_I_GATE", 4, NavEKF3, _gpsVelInnovGate, VEL_I_GATE_DEFAULT),

    // @Param: POSNE_M_NSE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("POSNE_M_NSE", 5, NavEKF3, _gpsHorizPosNoise, POSNE_M_NSE_DEFAULT),

    // @Param: POS_I_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted. If EK3_GLITCH_RAD has been set to 0 the horizontal position innovations will be clipped instead of rejected if they exceed the gate size so a smaller value of EK3_POS_I_GATE not exceeding 300 is recommended to limit the effect of GPS transient errors.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("POS_I_GATE", 6, NavEKF3, _gpsPosInnovGate, POS_I_GATE_DEFAULT),

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position. If EK3_GLITCH_RAD set to 0 the GPS innovations will be clipped instead of rejected if they exceed the gate size set by EK3_VEL_I_GATE and EK3_POS_I_GATE which can be useful if poor quality sensor data is causing GPS rejection and loss of navigation but does make the EKF more susceptible to GPS glitches. If setting EK3_GLITCH_RAD to 0 it is recommended to reduce EK3_VEL_I_GATE and EK3_POS_I_GATE to 300.
    // @Range: 10 100
    // @Increment: 5
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("GLITCH_RAD", 7, NavEKF3, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // 8 previously used for EKF3_GPS_DELAY parameter that has been deprecated.
    // The EKF now takes its GPS delay form the GPS library with the default delays
    // specified by the GPS_DELAY and GPS_DELAY2 parameters.

    // Height measurement parameters

    // 9 was ALT_SOURCE

    // @Param: ALT_M_NSE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors. A larger value for EK3_ALT_M_NSE may be required when operating with EK3_SRCx_POSZ = 0. This parameter also sets the noise for the 'synthetic' zero height measurement that is used when EK3_SRCx_POSZ = 0.
    // @Range: 0.1 100.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("ALT_M_NSE", 10, NavEKF3, _baroAltNoise, ALT_M_NSE_DEFAULT),

    // @Param: HGT_I_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.  If EK3_GLITCH_RAD set to 0 the vertical position innovations will be clipped instead of rejected if they exceed the gate size and a smaller value of EK3_HGT_I_GATE not exceeding 300 is recommended to limit the effect of height sensor transient errors.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("HGT_I_GATE", 11, NavEKF3, _hgtInnovGate, HGT_I_GATE_DEFAULT),

    // @Param: HGT_DELAY
    // @DisplayName: Height measurement delay (msec)
    // @Description: This is the number of msec that the Height measurements lag behind the inertial measurements.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: ms
    // @RebootRequired: True
    AP_GROUPINFO("HGT_DELAY", 12, NavEKF3, _hgtDelay_ms, 60),

    // Magnetometer measurement parameters

    // @Param: MAG_M_NSE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    // @Units: Gauss
    AP_GROUPINFO("MAG_M_NSE", 13, NavEKF3, _magNoise, MAG_M_NSE_DEFAULT),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer default fusion mode
    // @Description: This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states and when it will use a simpler magnetic heading fusion model that does not use magnetic field states. The 3-axis magnetometer fusion is only suitable for use when the external magnetic field environment is stable. EK3_MAG_CAL = 0 uses heading fusion on ground, 3-axis fusion in-flight, and is the default setting for Plane users. EK3_MAG_CAL = 1 uses 3-axis fusion only when manoeuvring. EK3_MAG_CAL = 2 uses heading fusion at all times, is recommended if the external magnetic field is varying and is the default for rovers. EK3_MAG_CAL = 3 uses heading fusion on the ground and 3-axis fusion after the first in-air field and yaw reset has completed, and is the default for copters. EK3_MAG_CAL = 4 uses 3-axis fusion at all times. EK3_MAG_CAL = 5 uses an external yaw sensor with simple heading fusion. NOTE : Use of simple heading magnetometer fusion makes vehicle compass calibration and alignment errors harder for the EKF to detect which reduces the sensitivity of the Copter EKF failsafe algorithm. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK3_MAG_MASK parameter. EK3_MAG_CAL = 6 uses an external yaw sensor with fallback to compass when the external sensor is not available if we are flying. NOTE: The fusion mode can be forced to 2 for specific EKF cores using the EK3_MAG_MASK parameter. NOTE: limited operation without a magnetometer or any other yaw sensor is possible by setting all COMPASS_USE, COMPASS_USE2, COMPASS_USE3, etc parameters to 0 and setting COMPASS_ENABLE to 0. If this is done, the EK3_GSF_RUN and EK3_GSF_USE masks must be set to the same as EK3_IMU_MASK. A yaw angle derived from IMU and GPS velocity data using a Gaussian Sum Filter (GSF) will then be used to align the yaw when flight commences and there is sufficient movement.
    // @Values: 0:When flying,1:When manoeuvring,2:Never,3:After first climb yaw reset,4:Always,5:Use external yaw sensor (Deprecated in 4.1+ see EK3_SRCn_YAW),6:External yaw sensor with compass fallback (Deprecated in 4.1+ see EK3_SRCn_YAW)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAG_CAL", 14, NavEKF3, _magCal, MAG_CAL_DEFAULT),

    // @Param: MAG_I_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("MAG_I_GATE", 15, NavEKF3, _magInnovGate, MAG_I_GATE_DEFAULT),

    // Airspeed measurement parameters

    // @Param: EAS_M_NSE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("EAS_M_NSE", 16, NavEKF3, _easNoise, 1.4f),

    // @Param: EAS_I_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("EAS_I_GATE", 17, NavEKF3, _tasInnovGate, 400),

    // Rangefinder measurement parameters

    // @Param: RNG_M_NSE
    // @DisplayName: Range finder measurement noise (m)
    // @Description: This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("RNG_M_NSE", 18, NavEKF3, _rngNoise, 0.5f),

    // @Param: RNG_I_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("RNG_I_GATE", 19, NavEKF3, _rngInnovGate, 500),

    // Optical flow measurement parameters

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 4.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("MAX_FLOW", 20, NavEKF3, _maxFlowRate, 2.5f),

    // @Param: FLOW_M_NSE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("FLOW_M_NSE", 21, NavEKF3, _flowNoise, FLOW_M_NSE_DEFAULT),

    // @Param: FLOW_I_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("FLOW_I_GATE", 22, NavEKF3, _flowInnovGate, FLOW_I_GATE_DEFAULT),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: ms
    // @RebootRequired: True
    AP_GROUPINFO("FLOW_DELAY", 23, NavEKF3, _flowDelay_ms, FLOW_MEAS_DELAY),

    // State and Covariance Predition Parameters

    // @Param: GYRO_P_NSE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.0001 0.1
    // @Increment: 0.0001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GYRO_P_NSE", 24, NavEKF3, _gyrNoise, GYRO_P_NSE_DEFAULT),

    // @Param: ACC_P_NSE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.01 1.0
    // @Increment: 0.01
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_P_NSE", 25, NavEKF3, _accNoise, ACC_P_NSE_DEFAULT),

    // @Param: GBIAS_P_NSE
    // @DisplayName: Rate gyro bias stability (rad/s/s)
    // @Description: This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: rad/s/s
    AP_GROUPINFO("GBIAS_P_NSE", 26, NavEKF3, _gyroBiasProcessNoise, GBIAS_P_NSE_DEFAULT),

    // 27 previously used for EK2_GSCL_P_NSE parameter that has been removed

    // @Param: ABIAS_P_NSE
    // @DisplayName: Accelerometer bias stability (m/s^3)
    // @Description: This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.02
    // @User: Advanced
    // @Units: m/s/s/s
    AP_GROUPINFO("ABIAS_P_NSE", 28, NavEKF3, _accelBiasProcessNoise, ABIAS_P_NSE_DEFAULT),

    // 29 previously used for EK2_MAG_P_NSE parameter that has been replaced with EK3_MAGE_P_NSE and EK3_MAGB_P_NSE

    // @Param: WIND_P_NSE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 2.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("WIND_P_NSE", 30, NavEKF3, _windVelProcessNoise, WIND_P_NSE_DEFAULT),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind process noise scaler
    // @Description: This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
    // @Range: 0.0 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE", 31, NavEKF3, _wndVarHgtRateScale, 1.0f),

    // @Param: GPS_CHECK
    // @DisplayName: GPS preflight check
    // @Description: This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
    // @Bitmask: 0:NSats,1:HDoP,2:speed error,3:position error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed
    // @User: Advanced
    AP_GROUPINFO("GPS_CHECK",    32, NavEKF3, _gpsCheck, 31),

    // @Param: IMU_MASK
    // @DisplayName: Bitmask of active IMUs
    // @Description: 1 byte bitmap of IMUs to use in EKF3. A separate instance of EKF3 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF3 will fail to start.
    // @Bitmask: 0:FirstIMU,1:SecondIMU,2:ThirdIMU,3:FourthIMU,4:FifthIMU,5:SixthIMU
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("IMU_MASK",     33, NavEKF3, _imuMask, HAL_EKF_IMU_MASK_DEFAULT),
    
    // @Param: CHECK_SCALE
    // @DisplayName: GPS accuracy check scaler (%)
    // @Description: This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.
    // @Range: 50 200
    // @User: Advanced
    // @Units: %
    AP_GROUPINFO("CHECK_SCALE", 34, NavEKF3, _gpsCheckScaler, CHECK_SCALER_DEFAULT),

    // @Param: NOAID_M_NSE
    // @DisplayName: Non-GPS operation position uncertainty (m)
    // @Description: This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.
    // @Range: 0.5 50.0
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("NOAID_M_NSE", 35, NavEKF3, _noaidHorizNoise, 10.0f),

    // @Param: BETA_MASK
    // @DisplayName: Bitmask controlling sidelip angle fusion
    // @Description: 1 byte bitmap controlling use of sideslip angle fusion for estimation of non wind states during operation of 'fly forward' vehicle types such as fixed wing planes. By assuming that the angle of sideslip is small, the wind velocity state estimates are corrected  whenever the EKF is not dead reckoning (e.g. has an independent velocity or position sensor such as GPS). This behaviour is on by default and cannot be disabled. When the EKF is dead reckoning, the wind states are used as a reference, enabling use of the small angle of sideslip assumption to correct non wind velocity states (eg attitude, velocity, position, etc) and improve navigation accuracy. This behaviour is on by default and cannot be disabled. The behaviour controlled by this parameter is the use of the small angle of sideslip assumption to correct non wind velocity states when the EKF is NOT dead reckoning. This is primarily of benefit to reduce the buildup of yaw angle errors during straight and level flight without a yaw sensor (e.g. magnetometer or dual antenna GPS yaw) provided aerobatic flight maneuvers with large sideslip angles are not performed. The 'always' option might be used where the yaw sensor is intentionally not fitted or disabled. The 'WhenNoYawSensor' option might be used if a yaw sensor is fitted, but protection against in-flight failure and continual rejection by the EKF is desired. For vehicles operated within visual range of the operator performing frequent turning maneuvers, setting this parameter is unnecessary.
    // @Bitmask: 0:Always,1:WhenNoYawSensor
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BETA_MASK", 36, NavEKF3, _betaMask, 0),

    // control of magnetic yaw angle fusion

    // @Param: YAW_M_NSE
    // @DisplayName: Yaw measurement noise (rad)
    // @Description: This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad
    AP_GROUPINFO("YAW_M_NSE", 37, NavEKF3, _yawNoise, 0.5f),

    // @Param: YAW_I_GATE
    // @DisplayName: Yaw measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("YAW_I_GATE", 38, NavEKF3, _yawInnovGate, 300),

    // @Param: TAU_OUTPUT
    // @DisplayName: Output complementary filter time constant (centi-sec)
    // @Description: Sets the time constant of the output complementary filter/predictor in centi-seconds.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    // @Units: cs
    AP_GROUPINFO("TAU_OUTPUT", 39, NavEKF3, _tauVelPosOutput, 25),

    // @Param: MAGE_P_NSE
    // @DisplayName: Earth magnetic field process noise (gauss/s)
    // @Description: This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.
    // @Range: 0.00001 0.01
    // @User: Advanced
    // @Units: Gauss/s
    AP_GROUPINFO("MAGE_P_NSE", 40, NavEKF3, _magEarthProcessNoise, MAGE_P_NSE_DEFAULT),

    // @Param: MAGB_P_NSE
    // @DisplayName: Body magnetic field process noise (gauss/s)
    // @Description: This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.
    // @Range: 0.00001 0.01
    // @User: Advanced
    // @Units: Gauss/s
    AP_GROUPINFO("MAGB_P_NSE", 41, NavEKF3, _magBodyProcessNoise, MAGB_P_NSE_DEFAULT),

    // @Param: RNG_USE_HGT
    // @DisplayName: Range finder switch height percentage
    // @Description: Range finder can be used as the primary height source when below this percentage of its maximum range (see RNGFNDx_MAX_CM) and the primary height source is Baro or GPS (see EK3_SRCx_POSZ).  This feature should not be used for terrain following as it is designed for vertical takeoff and landing with climb above the range finder use height before commencing the mission, and with horizontal position changes below that height being limited to a flat region around the takeoff and landing point.
    // @Range: -1 70
    // @Increment: 1
    // @User: Advanced
    // @Units: %
    AP_GROUPINFO("RNG_USE_HGT", 42, NavEKF3, _useRngSwHgt, -1),

    // @Param: TERR_GRAD
    // @DisplayName: Maximum terrain gradient
    // @Description: Specifies the maximum gradient of the terrain below the vehicle when it is using range finder as a height reference
    // @Range: 0 0.2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TERR_GRAD", 43, NavEKF3, _terrGradMax, 0.1f),

    // @Param: BCN_M_NSE
    // @DisplayName: Range beacon measurement noise (m)
    // @Description: This is the RMS value of noise in the range beacon measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("BCN_M_NSE", 44, NavEKF3, _rngBcnNoise, 1.0f),

    // @Param: BCN_I_GTE
    // @DisplayName: Range beacon measurement gate size
    // @Description: This sets the percentage number of standard deviations applied to the range beacon measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 100 1000
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("BCN_I_GTE", 45, NavEKF3, _rngBcnInnovGate, 500),

    // @Param: BCN_DELAY
    // @DisplayName: Range beacon measurement delay (msec)
    // @Description: This is the number of msec that the range beacon measurements lag behind the inertial measurements.
    // @Range: 0 250
    // @Increment: 10
    // @User: Advanced
    // @Units: ms
    // @RebootRequired: True
    AP_GROUPINFO("BCN_DELAY", 46, NavEKF3, _rngBcnDelay_ms, 50),

    // @Param: RNG_USE_SPD
    // @DisplayName: Range finder max ground speed
    // @Description: The range finder will not be used as the primary height source when the horizontal ground speed is greater than this value.
    // @Range: 2.0 6.0
    // @Increment: 0.5
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("RNG_USE_SPD", 47, NavEKF3, _useRngSwSpd, 2.0f),

    // @Param: ACC_BIAS_LIM
    // @DisplayName: Accelerometer bias limit
    // @Description: The accelerometer bias state will be limited to +- this value
    // @Range: 0.5 2.5
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_BIAS_LIM", 48, NavEKF3, _accBiasLim, 1.0f),

    // @Param: MAG_MASK
    // @DisplayName: Bitmask of active EKF cores that will always use heading fusion
    // @Description: 1 byte bitmap of EKF cores that will disable magnetic field states and use simple magnetic heading fusion at all times. This parameter enables specified cores to be used as a backup for flight into an environment with high levels of external magnetic interference which may degrade the EKF attitude estimate when using 3-axis magnetometer fusion. NOTE : Use of a different magnetometer fusion algorithm on different cores makes unwanted EKF core switches due to magnetometer errors more likely.
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAG_MASK", 49, NavEKF3, _magMask, 0),

    // @Param: OGN_HGT_MASK
    // @DisplayName: Bitmask control of EKF reference height correction
    // @Description: When a height sensor other than GPS is used as the primary height source by the EKF, the position of the zero height datum is defined by that sensor and its frame of reference. If a GPS height measurement is also available, then the height of the WGS-84 height datum used by the EKF can be corrected so that the height returned by the getLLH() function is compensated for primary height sensor drift and change in datum over time. The first two bit positions control when the height datum will be corrected. Correction is performed using a Bayes filter and only operates when GPS quality permits. The third bit position controls where the corrections to the GPS reference datum are applied. Corrections can be applied to the local vertical position or to the reported EKF origin height (default).
    // @Bitmask: 0:Correct when using Baro height,1:Correct when using range finder height,2:Apply corrections to local position
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OGN_HGT_MASK", 50, NavEKF3, _originHgtMode, 0),

    // @Param: VIS_VERR_MIN
    // @DisplayName: Visual odometry minimum velocity error
    // @Description: This is the 1-STD odometry velocity observation error that will be assumed when maximum quality is reported by the sensor. When quality is between max and min, the error will be calculated using linear interpolation between VIS_VERR_MIN and VIS_VERR_MAX.
    // @Range: 0.05 0.5
    // @Increment: 0.05
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VIS_VERR_MIN", 51, NavEKF3, _visOdmVelErrMin, 0.1f),

    // @Param: VIS_VERR_MAX
    // @DisplayName: Visual odometry maximum velocity error
    // @Description: This is the 1-STD odometry velocity observation error that will be assumed when minimum quality is reported by the sensor. When quality is between max and min, the error will be calculated using linear interpolation between VIS_VERR_MIN and VIS_VERR_MAX.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("VIS_VERR_MAX", 52, NavEKF3, _visOdmVelErrMax, 0.9f),

    // @Param: WENC_VERR
    // @DisplayName: Wheel odometry velocity error
    // @Description: This is the 1-STD odometry velocity observation error that will be assumed when wheel encoder data is being fused.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("WENC_VERR", 53, NavEKF3, _wencOdmVelErr, 0.1f),

    // @Param: FLOW_USE
    // @DisplayName: Optical flow use bitmask
    // @Description: Controls if the optical flow data is fused into the 24-state navigation estimator OR the 1-state terrain height estimator.
    // @User: Advanced
    // @Values: 0:None,1:Navigation,2:Terrain
    // @RebootRequired: True
    AP_GROUPINFO("FLOW_USE", 54, NavEKF3, _flowUse, FLOW_USE_DEFAULT),

    // @Param: HRT_FILT
    // @DisplayName: Height rate filter crossover frequency
    // @Description: Specifies the crossover frequency of the complementary filter used to calculate the output predictor height rate derivative.
    // @Range: 0.1 30.0
    // @Units: Hz
    AP_GROUPINFO("HRT_FILT", 55, NavEKF3, _hrt_filt_freq, 2.0f),

    // @Param: MAG_EF_LIM
    // @DisplayName: EarthField error limit
    // @Description: This limits the difference between the learned earth magnetic field and the earth field from the world magnetic model tables. A value of zero means to disable the use of the WMM tables.
    // @User: Advanced
    // @Range: 0 500
    // @Units: mGauss
    AP_GROUPINFO("MAG_EF_LIM", 56, NavEKF3, _mag_ef_limit, 50),

    // @Param: GSF_RUN_MASK
    // @DisplayName: Bitmask of which EKF-GSF yaw estimators run
    // @Description: 1 byte bitmap of which EKF3 instances run an independant EKF-GSF yaw estimator to provide a backup yaw estimate that doesn't rely on magnetometer data. This estimator uses IMU, GPS and, if available, airspeed data. EKF-GSF yaw estimator data for the primary EKF3 instance will be logged as GSF0 and GSF1 messages. Use of the yaw estimate generated by this algorithm is controlled by the EK3_GSF_USE_MASK and EK3_GSF_RST_MAX parameters. To run the EKF-GSF yaw estimator in ride-along and logging only, set EK3_GSF_USE to 0. 
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_RUN_MASK", 57, NavEKF3, _gsfRunMask, 3),

    // @Param: GSF_USE_MASK
    // @DisplayName: Bitmask of which EKF-GSF yaw estimators are used
    // @Description: A bitmask of which EKF3 instances will use the output from the EKF-GSF yaw estimator that has been turned on by the EK3_GSF_RUN_MASK parameter. If the inertial navigation calculation stops following the GPS, then the vehicle code can request EKF3 to attempt to resolve the issue, either by performing a yaw reset if enabled by this parameter by switching to another EKF3 instance.
    // @Bitmask: 0:FirstEKF,1:SecondEKF,2:ThirdEKF,3:FourthEKF,4:FifthEKF,5:SixthEKF
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_USE_MASK", 58, NavEKF3, _gsfUseMask, 3),

    // 59 was GSF_DELAY which was never released in a stable version

    // @Param: GSF_RST_MAX
    // @DisplayName: Maximum number of resets to the EKF-GSF yaw estimate allowed
    // @Description: Sets the maximum number of times the EKF3 will be allowed to reset its yaw to the estimate from the EKF-GSF yaw estimator. No resets will be allowed unless the use of the EKF-GSF yaw estimate is enabled via the EK3_GSF_USE_MASK parameter.
    // @Range: 1 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GSF_RST_MAX", 60, NavEKF3, _gsfResetMaxCount, 2),

    // @Param: ERR_THRESH
    // @DisplayName: EKF3 Lane Relative Error Sensitivity Threshold
    // @Description: lanes have to be consistently better than the primary by at least this threshold to reduce their overall relativeCoreError, lowering this makes lane switching more sensitive to smaller error differences
    // @Range: 0.05 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("ERR_THRESH", 61, NavEKF3, _err_thresh, 0.2),

    // @Param: AFFINITY
    // @DisplayName: EKF3 Sensor Affinity Options
    // @Description: These options control the affinity between sensor instances and EKF cores
    // @User: Advanced
    // @Bitmask: 0:EnableGPSAffinity,1:EnableBaroAffinity,2:EnableCompassAffinity,3:EnableAirspeedAffinity
    // @RebootRequired: True

    AP_GROUPINFO("AFFINITY", 62, NavEKF3, _affinity, 0),

    AP_SUBGROUPEXTENSION("", 63, NavEKF3, var_info2),

    AP_GROUPEND
};

// second table of parameters. allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo NavEKF3::var_info2[] = {

    // @Group: SRC
    // @Path: ../AP_NavEKF/AP_NavEKF_Source.cpp
    AP_SUBGROUPINFO(sources, "SRC", 1, NavEKF3, AP_NavEKF_Source),

    // @Param: DRAG_BCOEF_X
    // @DisplayName: Ballistic coefficient for X axis drag
    // @Description: Ratio of mass to drag coefficient measured along the X body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the frontal area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.
    // @Range: 0.0 1000.0
    // @Units: kg/m/m
    // @User: Advanced
    AP_GROUPINFO("DRAG_BCOEF_X", 2, NavEKF3, _ballisticCoef_x, 0.0f),

    // @Param: DRAG_BCOEF_Y
    // @DisplayName: Ballistic coefficient for Y axis drag
    // @Description: Ratio of mass to drag coefficient measured along the Y body axis. This parameter enables estimation of wind drift for vehicles with bluff bodies and without propulsion forces in the X and Y direction (eg multicopters). The drag produced by this effect scales with speed squared.  Set to a postive value > 1.0 to enable. A starting value is the mass in Kg divided by the side area. The predicted drag from the rotors is specified separately by the EK3_DRAG_MCOEF parameter.
    // @Range: 50.0 1000.0
    // @Units: kg/m/m
    // @User: Advanced
    AP_GROUPINFO("DRAG_BCOEF_Y", 3, NavEKF3, _ballisticCoef_y, 0.0f),

    // @Param: DRAG_M_NSE
    // @DisplayName: Observation noise for drag acceleration
    // @Description: This sets the amount of noise used when fusing X and Y acceleration as an observation that enables esitmation of wind velocity for multi-rotor vehicles. This feature is enabled by the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters
    // @Range: 0.1 2.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("DRAG_M_NSE", 4, NavEKF3, _dragObsNoise, 0.5f),

    // @Param: DRAG_MCOEF
    // @DisplayName: Momentum coefficient for propeller drag
    // @Description: This parameter is used to predict the drag produced by the rotors when flying a multi-copter, enabling estimation of wind drift. The drag produced by this effect scales with speed not speed squared and is produced because some of the air velocity normal to the rotors axis of rotation is lost when passing through the rotor disc which changes the momentum of the airflow causing drag. For unducted rotors the effect is roughly proportional to the area of the propeller blades when viewed side on and changes with different propellers. It is higher for ducted rotors. For example if flying at 15 m/s at sea level conditions produces a rotor induced drag acceleration of 1.5 m/s/s, then EK3_DRAG_MCOEF would be set to 0.1 = (1.5/15.0). Set EK3_MCOEF to a postive value to enable wind estimation using this drag effect. To account for the drag produced by the body which scales with speed squared, see documentation for the EK3_DRAG_BCOEF_X and EK3_DRAG_BCOEF_Y parameters.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("DRAG_MCOEF", 5, NavEKF3, _momentumDragCoef, 0.0f),

    // @Param: OGNM_TEST_SF
    // @DisplayName: On ground not moving test scale factor
    // @Description: This parameter is adjust the sensitivity of the on ground not moving test which is used to assist with learning the yaw gyro bias and stopping yaw drift before flight when operating without a yaw sensor. Bigger values allow the detection of a not moving condition with noiser IMU data. Check the XKFM data logged when the vehicle is on ground not moving and adjust the value of OGNM_TEST_SF to be slightly higher than the maximum value of the XKFM.ADR, XKFM.ALR, XKFM.GDR and XKFM.GLR test levels.
    // @Range: 1.0 10.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("OGNM_TEST_SF", 6, NavEKF3, _ognmTestScaleFactor, 2.0f),

    // @Param: GND_EFF_DZ
    // @DisplayName: Baro height ground effect dead zone
    // @Description: This parameter sets the size of the dead zone that is applied to negative baro height spikes that can occur when taking off or landing when a vehicle with lift rotors is operating in ground effect ground effect. Set to about 0.5m less than the amount of negative offset in baro height that occurs just prior to takeoff when lift motors are spooling up. Set to 0 if no ground effect is present.
    // @Range: 0.0 10.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("GND_EFF_DZ", 7, NavEKF3, _baroGndEffectDeadZone, 4.0f),

    // @Param: PRIMARY
    // @DisplayName: Primary core number
    // @Description: The core number (index in IMU mask) that will be used as the primary EKF core on startup. While disarmed the EKF will force the use of this core. A value of 0 corresponds to the first IMU in EK3_IMU_MASK.
    // @Range: 0 2
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 8, NavEKF3, _primary_core, EK3_PRIMARY_DEFAULT),

    // @Param: LOG_LEVEL
    // @DisplayName: Logging Level
    // @Description: Determines how verbose the EKF3 streaming logging is. A value of 0 provides full logging(default), a value of 1 only XKF4 scaled innovations are logged, a value of 2 both XKF4 and GSF are logged, and a value of 3 disables all streaming logging of EKF3.
    // @Range: 0 3
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOG_LEVEL", 9, NavEKF3, _log_level, 0),
    
    // @Param: GPS_VACC_MAX
    // @DisplayName: GPS vertical accuracy threshold
    // @Description: Vertical accuracy threshold for GPS as the altitude source. The GPS will not be used as an altitude source if the reported vertical accuracy of the GPS is larger than this threshold, falling back to baro instead. Set to zero to deactivate the threshold check.
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("GPS_VACC_MAX", 10, NavEKF3, _gpsVAccThreshold, 0.0f),

    // @Param: OPTIONS
    // @DisplayName: Optional EKF behaviour
    // @Description: This controls optional EKF behaviour. Setting JammingExpected will change the EKF nehaviour such that if dead reckoning navigation is possible it will require the preflight alignment GPS quality checks controlled by EK3_GPS_CHECK and EK3_CHECK_SCALE to pass before resuming GPS use if GPS lock is lost for more than 2 seconds to prevent bad
    // @Bitmask: 0:JammingExpected
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  11, NavEKF3, _options, 0),

    AP_GROUPEND
};

NavEKF3::NavEKF3() :
    dal{AP::dal()}
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}


// Initialise the filter
bool NavEKF3::InitialiseFilter(void)
{
    if (_enable == 0 || _imuMask == 0) {
        return false;
    }
    const auto &ins = dal.ins();

    dal.start_frame(AP_DAL::FrameType::InitialiseFilterEKF3);

    imuSampleTime_us = dal.micros64();

    // remember expected frame time
    const float loop_rate = ins.get_loop_rate_hz();
    if (!is_positive(loop_rate)) {
        return false;
    }
    _frameTimeUsec = 1e6 / loop_rate;

    // expected number of IMU frames per prediction
    _framesPerPrediction = uint8_t((EKF_TARGET_DT / (_frameTimeUsec * 1.0e-6) + 0.5));

#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    // convert parameters if necessary
    convert_parameters();
#endif

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    if (ins.get_accel_count() == 0) {
        return false;
    }
#endif

    if (core == nullptr) {

        // don't run multiple filters for 1 IMU
        uint8_t mask = (1U<<ins.get_accel_count())-1;
        _imuMask.set_and_default(_imuMask.get() & mask);
        
        // initialise the setup variables
        for (uint8_t i=0; i<MAX_EKF_CORES; i++) {
            coreSetupRequired[i] = false;
            coreImuIndex[i] = 0;
        }
        num_cores = 0;

        // count IMUs from mask
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_imuMask & (1U<<i)) {
                coreSetupRequired[num_cores] = true;
                coreImuIndex[num_cores] = i;
                num_cores++;
            }
        }

        // check if there is enough memory to create the EKF cores
        if (dal.available_memory() < sizeof(NavEKF3_core)*num_cores + 4096) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 not enough memory");
            _enable.set(0);
            num_cores = 0;
            return false;
        }

        //try to allocate from CCM RAM, fallback to Normal RAM if not available or full
        core = (NavEKF3_core*)dal.malloc_type(sizeof(NavEKF3_core)*num_cores, AP_DAL::MemoryType::FAST);
        if (core == nullptr) {
            _enable.set(0);
            num_cores = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 allocation failed");
            return false;
        }

        // Call constructors on all cores
        for (uint8_t i = 0; i < num_cores; i++) {
            new (&core[i]) NavEKF3_core(this, dal);
        }
    }

    // Set up any cores that have been created
    // This specifies the IMU to be used and creates the data buffers
    // If we are unable to set up a core, return false and try again next time the function is called
    bool core_setup_success = true;
    for (uint8_t core_index=0; core_index<num_cores; core_index++) {
        if (coreSetupRequired[core_index]) {
            coreSetupRequired[core_index] = !core[core_index].setup_core(coreImuIndex[core_index], core_index);
            if (coreSetupRequired[core_index]) {
                core_setup_success = false;
            }
        }
    }
    // exit with failure if any cores could not be setup
    if (!core_setup_success) {
        return false;
    }

    // set relative error scores for all cores to 0
    resetCoreErrors();

    // Set the primary initially to be users selected primary
    primary = uint8_t(_primary_core) < num_cores? _primary_core : 0;

    // invalidate shared origin
    common_origin_valid = false;

    // initialise the cores. We return success only if all cores
    // initialise successfully
    bool ret = true;
    for (uint8_t i=0; i<num_cores; i++) {
        ret &= core[i].InitialiseFilterBootstrap();
    }

    // set last time the cores were primary to 0
    memset(coreLastTimePrimary_us, 0, sizeof(coreLastTimePrimary_us));

    // zero the structs used capture reset events
    memset(&yaw_reset_data, 0, sizeof(yaw_reset_data));
    memset((void *)&pos_reset_data, 0, sizeof(pos_reset_data));
    memset(&pos_down_reset_data, 0, sizeof(pos_down_reset_data));

    return ret;
}

/*
  return true if a new core index has a better score than the current
  core
 */
bool NavEKF3::coreBetterScore(uint8_t new_core, uint8_t current_core) const
{
    const NavEKF3_core &oldCore = core[current_core];
    const NavEKF3_core &newCore = core[new_core];
    if (!newCore.healthy()) {
        // never consider a new core that isn't healthy
        return false;
    }
    if (newCore.have_aligned_tilt() != oldCore.have_aligned_tilt()) {
        // tilt alignment is most critical, if one is tilt aligned and
        // the other isn't then use the tilt aligned lane
        return newCore.have_aligned_tilt();
    }
    if (newCore.have_aligned_yaw() != oldCore.have_aligned_yaw()) {
        // yaw alignment is next most critical, if one is yaw aligned
        // and the other isn't then use the yaw aligned lane
        return newCore.have_aligned_yaw();
    }
    // if both cores are aligned then look at relative error scores
    return coreRelativeErrors[new_core] < coreRelativeErrors[current_core];
}

/* 
  Update Filter States - this should be called whenever new IMU data is available
  Execution speed governed by SCHED_LOOP_RATE
*/
void NavEKF3::UpdateFilter(void)
{
    dal.start_frame(AP_DAL::FrameType::UpdateFilterEKF3);

    if (!core) {
        return;
    }

    imuSampleTime_us = dal.micros64();

    for (uint8_t i=0; i<num_cores; i++) {
        // if we have not overrun by more than 3 IMU frames, and we
        // have already used more than 1/3 of the CPU budget for this
        // loop then suppress the prediction step. This allows
        // multiple EKF instances to cooperate on scheduling
        bool allow_state_prediction = true;
        if (core[i].getFramesSincePredict() < (_framesPerPrediction+3) &&
            dal.ekf_low_time_remaining(AP_DAL::EKFType::EKF3, i)) {
            allow_state_prediction = false;
        }
        core[i].UpdateFilter(allow_state_prediction);
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

    const bool armed  = dal.get_armed();

    // core selection is only available after the vehicle is armed, else forced to lane 0 if its healthy
    if (runCoreSelection && armed) {
        // update this instance's error scores for all active cores and get the primary core's error score
        float primaryErrorScore = updateCoreErrorScores();

        // update the accumulated relative error scores for all active cores
        updateCoreRelativeErrors();

        bool betterCore = false;
        bool altCoreAvailable = false;
        uint8_t newPrimaryIndex = primary;

        // loop through all available cores to find if an alternative core is available
        for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {
            if (coreIndex != primary) {
                float altCoreError = coreRelativeErrors[coreIndex];

                // an alternative core is available for selection based on 2 conditions -
                // 1. healthy and states have been updated on this time step
                // 2. has relative error less than primary core error
                // 3. not been the primary core for at least 10 seconds
                altCoreAvailable = coreBetterScore(coreIndex, newPrimaryIndex) &&
                    imuSampleTime_us - coreLastTimePrimary_us[coreIndex] > 1E7;

                if (altCoreAvailable) {
                    // if this core has a significantly lower relative error to the active primary, we consider it as a 
                    // better core and would like to switch to it even if the current primary is healthy
                    betterCore = altCoreError <= -BETTER_THRESH; // a better core if its relative error is below a substantial level than the primary's
                    // handle the case where the secondary core is faster to complete yaw alignment which can happen
                    // in flight when oeprating without a magnetomer
                    const NavEKF3_core &newCore = core[coreIndex];
                    const NavEKF3_core &oldCore = core[primary];
                    betterCore |= newCore.have_aligned_yaw() && !oldCore.have_aligned_yaw();
                    newPrimaryIndex = coreIndex;
                }
            }
        }
        altCoreAvailable = newPrimaryIndex != primary;

        // Switch cores if another core is available and the active primary core meets one of the following conditions - 
        // 1. has a bad error score
        // 2. is unhealthy
        // 3. is healthy, but a better core is available
        // also update the yaw and position reset data to capture changes due to the lane switch
        if (altCoreAvailable && (primaryErrorScore > 1.0f || !core[primary].healthy() || betterCore)) {
            updateLaneSwitchYawResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosResetData(newPrimaryIndex, primary);
            updateLaneSwitchPosDownResetData(newPrimaryIndex, primary);
            resetCoreErrors();
            coreLastTimePrimary_us[primary] = imuSampleTime_us;
            primary = newPrimaryIndex;
            lastLaneSwitch_ms = dal.millis();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 lane switch %u", primary);
        }       
    }

    const uint8_t user_primary = uint8_t(_primary_core) < num_cores? _primary_core : 0;
    if (primary != user_primary && core[user_primary].healthy() && !armed) {
        // when on the ground and disarmed force the selected primary
        // core. This avoids us ending with with a lottery for which
        // IMU is used in each flight. Otherwise the alignment of the
        // timing of the core selection updates with the timing of GPS
        // updates can lead to a core other than the first one being
        // used as primary for some flights. As different IMUs may
        // have quite different noise characteristics this leads to
        // inconsistent performance
        primary = user_primary;
    }

    // align position of inactive sources to ahrs
    sources.align_inactive_sources();
}

/*
  check if switching lanes will reduce the normalised
  innovations. This is called when the vehicle code is about to
  trigger an EKF failsafe, and it would like to avoid that by
  using a different EKF lane
*/
void NavEKF3::checkLaneSwitch(void)
{
    dal.log_event3(AP_DAL::Event::checkLaneSwitch);

    uint32_t now = dal.millis();
    if (lastLaneSwitch_ms != 0 && now - lastLaneSwitch_ms < 5000) {
        // don't switch twice in 5 seconds
        return;
    }

    float primaryErrorScore = core[primary].errorScore();
    float lowestErrorScore = primaryErrorScore;
    uint8_t newPrimaryIndex = primary;
    for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {
        if (coreIndex != primary) {
            const NavEKF3_core &newCore = core[coreIndex];
            // an alternative core is available for selection only if healthy and if states have been updated on this time step
            bool altCoreAvailable = newCore.healthy() && newCore.have_aligned_yaw() && newCore.have_aligned_tilt();
            float altErrorScore = newCore.errorScore();
            if (altCoreAvailable && altErrorScore < lowestErrorScore && altErrorScore < 0.9) {
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
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 lane switch %u", primary);
    }
}

void NavEKF3::requestYawReset(void)
{
    dal.log_event3(AP_DAL::Event::requestYawReset);

    for (uint8_t i = 0; i < num_cores; i++) {
        core[i].EKFGSF_requestYawReset();
    }
}

/*
  Update this instance error score value for all active cores
*/
float NavEKF3::updateCoreErrorScores()
{
    for (uint8_t i = 0; i < num_cores; i++) {
        coreErrorScores[i] = core[i].errorScore();
    }
    return coreErrorScores[primary];
}

/*
  Update the relative error for all alternate available cores with respect to primary core's error.
  A positive relative error for a core means it has been more erroneous than the existing primary.
  A negative relative error indicates a core which can be switched to.
*/
void NavEKF3::updateCoreRelativeErrors()
{
    float error = 0;
    for (uint8_t i = 0; i < num_cores; i++) {
        if (i != primary) {
            error = coreErrorScores[i] - coreErrorScores[primary];
            // reduce error for a core only if its better than the primary lane by at least the Relative Error Threshold, this should prevent unnecessary lane changes
            if (error > 0 || error < -MAX(_err_thresh, 0.05)) {
                coreRelativeErrors[i] += error;
                coreRelativeErrors[i] = constrain_ftype(coreRelativeErrors[i], -CORE_ERR_LIM, CORE_ERR_LIM);
            }
        }
    }
}

// Reset the relative error values
void NavEKF3::resetCoreErrors(void)
{
    for (uint8_t i = 0; i < num_cores; i++) {
        coreRelativeErrors[i] = 0;
    }
}

// set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
void NavEKF3::setPosVelYawSourceSet(uint8_t source_set_idx)
{
    if (source_set_idx < AP_NAKEKF_SOURCE_SET_MAX) {
        dal.log_event3(AP_DAL::Event(uint8_t(AP_DAL::Event::setSourceSet0)+source_set_idx));
    }
    sources.setPosVelYawSourceSet((AP_NavEKF_Source::SourceSetSelection)source_set_idx);
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF3::healthy(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].healthy();
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
// requires_position should be true if horizontal position configuration should be checked
bool NavEKF3::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    // check source configuration
    if (!sources.pre_arm_check(requires_position, failure_msg, failure_msg_len)) {
        return false;
    }

    // check if using compass (i.e. EK3_SRCn_YAW) with deprecated MAG_CAL values (5 was EXTERNAL_YAW, 6 was EXTERNAL_YAW_FALLBACK)
    const int8_t magCalParamVal = _magCal.get();
    const AP_NavEKF_Source::SourceYaw yaw_source = sources.getYawSource();
    if (((magCalParamVal == 5) || (magCalParamVal == 6)) && (yaw_source != AP_NavEKF_Source::SourceYaw::GPS)) {
        // yaw source is configured to use compass but MAG_CAL valid is deprecated
        dal.snprintf(failure_msg, failure_msg_len, "EK3_MAG_CAL and EK3_SRC1_YAW inconsistent");
        return false;
    }

    if (!core) {
        dal.snprintf(failure_msg, failure_msg_len, "no EKF3 cores");
        return false;
    }
    for (uint8_t i = 0; i < num_cores; i++) {
        if (!core[i].healthy()) {
            const char *failure = core[i].prearm_failure_reason();
            if (failure != nullptr) {
                dal.snprintf(failure_msg, failure_msg_len, failure);
            } else {
                dal.snprintf(failure_msg, failure_msg_len, "EKF3 core %d unhealthy", (int)i);
            }
            return false;
        }
    }
    return true;
}

// returns the index of the primary core
// return -1 if no primary core selected
int8_t NavEKF3::getPrimaryCoreIndex(void) const
{
    if (!core) {
        return -1;
    }
    return primary;
}

// returns the index of the IMU of the primary core
// return -1 if no primary core selected
int8_t NavEKF3::getPrimaryCoreIMUIndex(void) const
{
    if (!core) {
        return -1;
    }
    return core[primary].getIMUIndex();
}

// Write the last calculated NE position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF3::getPosNE(Vector2f &posNE) const
{
    if (!core) {
        return false;
    }
    return core[primary].getPosNE(posNE);
}

// Write the last calculated D position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF3::getPosD(float &posD) const
{
    if (!core) {
        return false;
    }
    return core[primary].getPosD(posD);
}

// return NED velocity in m/s
void NavEKF3::getVelNED(Vector3f &vel) const
{
    if (core) {
        core[primary].getVelNED(vel);
    }
}

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool NavEKF3::getAirSpdVec(Vector3f &vel) const
{
    if (core) {
        return core[primary].getAirSpdVec(vel);
    }
    return false;
}

// return the innovation in m/s, innovation variance in (m/s)^2 and age in msec of the last TAS measurement processed
bool NavEKF3::getAirSpdHealthData(float &innovation, float &innovationVariance, uint32_t &age_ms) const
{
    if (core) {
        return core[primary].getAirSpdHealthData(innovation, innovationVariance, age_ms);
    }
    return false;
}

// Return the rate of change of vertical position in the down direction (dPosD/dt) in m/s
float NavEKF3::getPosDownDerivative() const
{
    // return the value calculated from a complementary filter applied to the EKF height and vertical acceleration
    if (core) {
        return core[primary].getPosDownDerivative();
    }
    return 0.0f;
}

// return body axis gyro bias estimates in rad/sec
void NavEKF3::getGyroBias(int8_t instance, Vector3f &gyroBias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getGyroBias(gyroBias);
    }
}

// return accelerometer bias estimate in m/s/s
void NavEKF3::getAccelBias(int8_t instance, Vector3f &accelBias) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        core[instance].getAccelBias(accelBias);
    }
}

// returns active source set used by EKF3
uint8_t NavEKF3::get_active_source_set() const
{
    return sources.get_active_source_set();
}

// reset body axis gyro bias estimates
void NavEKF3::resetGyroBias(void)
{
    dal.log_event3(AP_DAL::Event::resetGyroBias);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].resetGyroBias();
        }
    }
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKF origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool NavEKF3::resetHeightDatum(void)
{
    dal.log_event3(AP_DAL::Event::resetHeightDatum);

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

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF3::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (core) {
        core[primary].getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
// returns true if wind state estimation is active
bool NavEKF3::getWind(Vector3f &wind) const
{
    if (core == nullptr) {
        return false;
    }
    return core[primary].getWind(wind);
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF3::getMagNED(Vector3f &magNED) const
{
    if (core) {
        core[primary].getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF3::getMagXYZ(Vector3f &magXYZ) const
{
    if (core) {
        core[primary].getMagXYZ(magXYZ);
    }
}

// return the airspeed sensor in use
uint8_t NavEKF3::getActiveAirspeed() const
{
    if (core) {
        return core[primary].getActiveAirspeed();
    } else {
        return 255;
    }
}

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool NavEKF3::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
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
bool NavEKF3::getLLH(Location &loc) const
{
    if (!core) {
        return false;
    }
    return core[primary].getLLH(loc);
}

// Return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF3::getOriginLLH(Location &loc) const
{
    if (!core) {
        return false;
    }
    if (common_origin_valid) {
        loc = common_EKF_origin;
        return true;
    }
    return core[primary].getOriginLLH(loc);
}

// set the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter will be relative to this location
// The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
// Returns false if the filter has rejected the attempt to set the origin
bool NavEKF3::setOriginLLH(const Location &loc)
{
    dal.log_SetOriginLLH3(loc);

    if (!core) {
        return false;
    }
    if (common_origin_valid) {
        // we don't allow setting the EKF origin if it has already been set
        // this is to prevent causing upsets from a shifting origin.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF3: origin already set");
        return false;
    }
    bool ret = false;
    for (uint8_t i=0; i<num_cores; i++) {
        ret |= core[i].setOriginLLH(loc);
    }
    // return true if any core accepts the new origin
    return ret;
}

bool NavEKF3::setLatLng(const Location &loc, float posAccuracy, uint32_t timestamp_ms)
{
#if EK3_FEATURE_POSITION_RESET
    dal.log_SetLatLng(loc, posAccuracy, timestamp_ms);

    if (!core) {
        return false;
    }
    bool ret = false;
    for (uint8_t i=0; i<num_cores; i++) {
        ret |= core[i].setLatLng(loc, posAccuracy, timestamp_ms);
    }
    // return true if any core accepts the new origin
    return ret;
#else
    return false;
#endif // EK3_FEATURE_POSITION_RESET
}


// return estimated height above ground level
// return false if ground height is not being estimated.
bool NavEKF3::getHAGL(float &HAGL) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHAGL(HAGL);
}

// return the Euler roll, pitch and yaw angle in radians
void NavEKF3::getEulerAngles(Vector3f &eulers) const
{
    if (core) {
        core[primary].getEulerAngles(eulers);
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF3::getRotationBodyToNED(Matrix3f &mat) const
{
    if (core) {
        core[primary].getRotationBodyToNED(mat);
    }
}

// return the quaternions defining the rotation from XYZ (body) to NED axes
void NavEKF3::getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const
{
    if (instance < 0 || instance >= num_cores) instance = primary;
    if (core) {
        Matrix3f mat;
        core[instance].getRotationBodyToNED(mat);
        quat.from_rotation_matrix(mat);
    }
}

// return the quaternions defining the rotation from NED to XYZ (autopilot) axes
void NavEKF3::getQuaternion(Quaternion &quat) const
{
    if (core) {
        core[primary].getQuaternion(quat);
    }
}

// return the innovations
bool NavEKF3::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    if (core == nullptr) {
        return false;
    }

    return core[primary].getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
bool NavEKF3::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    if (core == nullptr) {
        return false;
    }

    return core[primary].getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
}

// get a source's velocity innovations
// returns true on success and results are placed in innovations and variances arguments
bool NavEKF3::getVelInnovationsAndVariancesForSource(AP_NavEKF_Source::SourceXY source, Vector3f &innovations, Vector3f &variances) const
{
    if (core) {
        return core[primary].getVelInnovationsAndVariancesForSource(source, innovations, variances);
    }
    return false;
}

// should we use the compass? This is public so it can be used for
// reporting via ahrs.use_compass()
bool NavEKF3::use_compass(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].use_compass();
}

// are we using (aka fusing) a non-compass yaw?
bool NavEKF3::using_noncompass_for_yaw(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].using_noncompass_for_yaw();
}

// are we using (aka fusing) external nav for yaw?
bool NavEKF3::using_extnav_for_yaw() const
{
    if (!core) {
        return false;
    }
    return core[primary].using_extnav_for_yaw();
}

// check if configured to use GPS for horizontal position estimation
bool NavEKF3::configuredToUseGPSForPosXY(void) const
{
    // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = do not use GPS
    return  (sources.getPosXYSource() == AP_NavEKF_Source::SourceXY::GPS);
}

// write the raw optical flow measurements
// rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
// rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
// rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
// The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
// msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
// posOffset is the XYZ flow sensor position in the body frame in m
// heightOverride is the fixed height of the sensor above ground in m, when on rover vehicles. 0 if not used
#if EK3_FEATURE_OPTFLOW_FUSION
void NavEKF3::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride)
{
    dal.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride);
        }
    }
}

// retrieve latest corrected optical flow samples (used for calibration)
bool NavEKF3::getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const
{
    // return optical flow samples from primary core
    if (core) {
        return core[primary].getOptFlowSample(timeStamp_ms, flowRate, bodyRate, losPred);
    }
    return false;
}
#endif  // EK3_FEATURE_OPTFLOW_FUSION

// write yaw angle sensor measurements
void NavEKF3::writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type)
{
    dal.log_writeEulerYawAngle(yawAngle, yawAngleErr, timeStamp_ms, type);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeEulerYawAngle(yawAngle, yawAngleErr, timeStamp_ms, type);
        }
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
*/
void NavEKF3::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
    dal.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
        }
    }
}

/* Write velocity data from an external navigation system
 * vel : velocity in NED (m)
 * err : velocity error (m/s)
 * timeStamp_ms : system time the measurement was taken, not the time it was received (mSec)
 * delay_ms : average delay of external nav system measurements relative to inertial measurements
*/
void NavEKF3::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
    dal.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
        }
    }
}

// return data for debugging optical flow fusion
/*
 * Write body frame linear and angular displacement measurements from a visual odometry sensor
 *
 * quality is a normalised confidence value from 0 to 100
 * delPos is the XYZ change in linear position measured in body frame and relative to the inertial reference at timeStamp_ms (m)
 * delAng is the XYZ angular rotation measured in body frame and relative to the inertial reference at timeStamp_ms (rad)
 * delTime is the time interval for the measurement of delPos and delAng (sec)
 * timeStamp_ms is the timestamp of the last image used to calculate delPos and delAng (msec)
 * delay_ms is the average delay of external nav system measurements relative to inertial measurements
 * posOffset is the XYZ body frame position of the camera focal point (m)
*/
void NavEKF3::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset)
{
    dal.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);
        }
    }
}

/*
 * Write odometry data from a wheel encoder. The axis of rotation is assumed to be parallel to the vehicle body axis
 *
 * delAng is the measured change in angular position from the previous measurement where a positive rotation is produced by forward motion of the vehicle (rad)
 * delTime is the time interval for the measurement of delAng (sec)
 * timeStamp_ms is the time when the rotation was last measured (msec)
 * posOffset is the XYZ body frame position of the wheel hub (m)
*/
void NavEKF3::writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius)
{
    dal.writeWheelOdom(delAng, delTime, timeStamp_ms, posOffset, radius);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeWheelOdom(delAng, delTime, timeStamp_ms, posOffset, radius);
        }
    }
}

// parameter conversion of EKF3 parameters
void NavEKF3::convert_parameters()
{
    // exit immediately if param conversion has been done before
    if (sources.configured()) {
        return;
    }

    // find EKF3's top level key
    uint16_t k_param_ekf3;
    if (!AP_Param::find_top_level_key_by_pointer(this, k_param_ekf3)) {
        return;
    }

    // use EK3_GPS_TYPE to set EK3_SRC1_POSXY, EK3_SRC1_VELXY, EK3_SRC1_VELZ
    const AP_Param::ConversionInfo gps_type_info = {k_param_ekf3, 1, AP_PARAM_INT8, "EK3_GPS_TYPE"};
    AP_Int8 gps_type_old;
    const bool found_gps_type = AP_Param::find_old_parameter(&gps_type_info, &gps_type_old);
    if (found_gps_type) {
        switch (gps_type_old.get()) {
        case 0:
            // EK3_GPS_TYPE == 0 (GPS 3D Vel and 2D Pos)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSXY", (int8_t)AP_NavEKF_Source::SourceXY::GPS);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELXY", (int8_t)AP_NavEKF_Source::SourceXY::GPS);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELZ", (int8_t)AP_NavEKF_Source::SourceZ::GPS);
            break;
        case 1:
            // EK3_GPS_TYPE == 1 (GPS 2D Vel and 2D Pos) then EK3_SRC1_POSXY = GPS(1), EK3_SRC1_VELXY = GPS(1), EK3_SRC1_VELZ = NONE(0)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSXY", (int8_t)AP_NavEKF_Source::SourceXY::GPS);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELXY", (int8_t)AP_NavEKF_Source::SourceXY::GPS);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELZ", (int8_t)AP_NavEKF_Source::SourceZ::NONE);
            break;
        case 2:
            // EK3_GPS_TYPE == 2 (GPS 2D Pos) then EK3_SRC1_POSXY = GPS(1), EK3_SRC1_VELXY = None(0), EK3_SRC1_VELZ = NONE(0)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSXY", (int8_t)AP_NavEKF_Source::SourceXY::GPS);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELXY", (int8_t)AP_NavEKF_Source::SourceXY::NONE);
            AP_Param::set_and_save_by_name("EK3_SRC1_VELZ", (int8_t)AP_NavEKF_Source::SourceZ::NONE);
            break;
        case 3:
        default:
            // EK3_GPS_TYPE == 3 (No GPS) we don't know what to do, could be optical flow, beacon or external nav
            sources.mark_configured();
            break;
        }
    } else {
        // mark configured in storage so conversion is only run once
        sources.mark_configured();
    }

    // use EK3_ALT_SOURCE to set EK3_SRC1_POSZ
    const AP_Param::ConversionInfo alt_source_info = {k_param_ekf3, 9, AP_PARAM_INT8, "EK3_ALT_SOURCE"};
    AP_Int8 alt_source_old;
    if (AP_Param::find_old_parameter(&alt_source_info, &alt_source_old)) {
        switch (alt_source_old.get()) {
        case 0:
            // EK3_ALT_SOURCE = BARO, the default so do nothing
            break;
        case 1:
            // EK3_ALT_SOURCE == 1 (RangeFinder)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSZ", (int8_t)AP_NavEKF_Source::SourceZ::RANGEFINDER);
            break;
        case 2:
            // EK3_ALT_SOURCE == 2 (GPS)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSZ", (int8_t)AP_NavEKF_Source::SourceZ::GPS);
            break;
        case 3:
            // EK3_ALT_SOURCE == 3 (Beacon)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSZ", (int8_t)AP_NavEKF_Source::SourceZ::BEACON);
            break;
        case 4:
            // EK3_ALT_SOURCE == 4 (ExtNav)
            AP_Param::set_and_save_by_name("EK3_SRC1_POSZ", (int8_t)AP_NavEKF_Source::SourceZ::EXTNAV);
            break;
        default:
            // do nothing
            break;
        }
    }

    // use EK3_MAG_CAL to set EK3_SRC1_YAW
    switch (_magCal.get()) {
    case 5:
        // EK3_MAG_CAL = 5 (External Yaw sensor).  We rely on effective_magCal to interpret old "5" values as "Never"
        AP_Param::set_and_save_by_name("EK3_SRC1_YAW", (int8_t)AP_NavEKF_Source::SourceYaw::GPS);
        break;
    case 6:
        // EK3_MAG_CAL = 6 (ExtYaw with Compass fallback).  We rely on effective_magCal to interpret old "6" values as "When Flying"
        AP_Param::set_and_save_by_name("EK3_SRC1_YAW", (int8_t)AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK);
        break;
    default:
        // do nothing
        break;
    }

    // if GPS and optical flow enabled set EK3_SRC2_VELXY to optical flow
    // EK3_SRC_OPTIONS should default to 1 meaning both GPS and optical flow velocities will be fused
    if (dal.opticalflow_enabled() && (!found_gps_type || (gps_type_old.get() <= 2))) {
        AP_Param::set_and_save_by_name("EK3_SRC2_VELXY", (int8_t)AP_NavEKF_Source::SourceXY::OPTFLOW);
    }
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference. Use to prevent range finder operation otherwise
// enabled by the combination of EK3_RNG_USE_HGT and EK3_RNG_USE_SPD parameters.
void NavEKF3::setTerrainHgtStable(bool val)
{
    if (val) {
        dal.log_event3(AP_DAL::Event::setTerrainHgtStable);
    } else {
        dal.log_event3(AP_DAL::Event::unsetTerrainHgtStable);
    }

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
void NavEKF3::getFilterFaults(uint16_t &faults) const
{
    if (core) {
        core[primary].getFilterFaults(faults);
    } else {
        faults = 0;
    }
}

/*
  return filter status flags
*/
void NavEKF3::getFilterStatus(nav_filter_status &status) const
{
    if (core) {
        core[primary].getFilterStatus(status);
    } else {
        memset(&status, 0, sizeof(status));
    }
}

// send an EKF_STATUS_REPORT message to GCS
void NavEKF3::send_status_report(GCS_MAVLINK &link) const
{
    if (core) {
        core[primary].send_status_report(link);
    }
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF3::getHeightControlLimit(float &height) const
{
    if (!core) {
        return false;
    }
    return core[primary].getHeightControlLimit(height);
}

// Returns the amount of yaw angle change (in radians) due to the last yaw angle reset or core selection switch
// Returns the time of the last yaw angle reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF3::getLastYawResetAngle(float &yawAngDelta)
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
    // or this is a repeated access
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
uint32_t NavEKF3::getLastPosNorthEastReset(Vector2f &posDelta)
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
uint32_t NavEKF3::getLastVelNorthEastReset(Vector2f &vel) const
{
    if (!core) {
        return 0;
    }
    return core[primary].getLastVelNorthEastReset(vel);
}

// Returns the amount of vertical position change due to the last reset or core switch in metres
// Returns the time of the last reset or 0 if no reset or core switch has ever occurred
// Where there are multiple consumers, they must access this function on the same frame as each other
uint32_t NavEKF3::getLastPosDownReset(float &posDelta)
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
void NavEKF3::updateLaneSwitchYawResetData(uint8_t new_primary, uint8_t old_primary)
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
void NavEKF3::updateLaneSwitchPosResetData(uint8_t new_primary, uint8_t old_primary)
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
void NavEKF3::updateLaneSwitchPosDownResetData(uint8_t new_primary, uint8_t old_primary)
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
    core[old_primary].getPosD_local(posDownOldPrimary);
    core[new_primary].getPosD_local(posDownNewPrimary);
    pos_down_reset_data.core_delta = posDownNewPrimary - posDownOldPrimary + pos_down_reset_data.core_delta;
    pos_down_reset_data.last_primary_change = imuSampleTime_us / 1000;
    pos_down_reset_data.core_changed = true;

}

// Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
void NavEKF3::writeDefaultAirSpeed(float airspeed, float uncertainty)
{
    // ignore any data if the EKF is not started
    if (!core) {
        return;
    }

    dal.log_writeDefaultAirSpeed3(airspeed, uncertainty);

    for (uint8_t i=0; i<num_cores; i++) {
        core[i].writeDefaultAirSpeed(airspeed, uncertainty);
    }
}

// returns true when the yaw angle has been aligned
bool NavEKF3::yawAlignmentComplete(void) const
{
    if (!core) {
        return false;
    }
    return core[primary].have_aligned_yaw();
}

// returns true when the state estimates are significantly degraded by vibration
bool NavEKF3::isVibrationAffected() const
{
    if (core) {
        return core[primary].isVibrationAffected();
    }
    return false;
}

// get a yaw estimator instance
const EKFGSF_yaw *NavEKF3::get_yawEstimator(void) const
{
    if (core) {
        return core[primary].get_yawEstimator();
    }
    return nullptr;
}
