/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

// uncomment this to force the optimisation of this code, note that
// this makes debugging harder
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#pragma GCC optimize("O0")
#else
#pragma GCC optimize("O3")
#endif

#include "AP_NavEKF.h"
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Vehicle.h>

#include <stdio.h>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built. Note that this trick won't work for arduino
  builds on APM2, but NavEKF doesn't run on APM2, so that's OK
 */
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// copter defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0001f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         1
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15
#define FLOW_MEAS_DELAY         10

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
// rover defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       1.0f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0002f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         1
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15
#define FLOW_MEAS_DELAY         25

#else
// generic defaults (and for plane)
#define VELNE_NOISE_DEFAULT     0.3f
#define VELD_NOISE_DEFAULT      0.5f
#define POSNE_NOISE_DEFAULT     0.5f
#define ALT_NOISE_DEFAULT       0.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.015f
#define ACC_PNOISE_DEFAULT      0.5f
#define GBIAS_PNOISE_DEFAULT    1E-06f
#define ABIAS_PNOISE_DEFAULT    0.0002f
#define MAGE_PNOISE_DEFAULT     0.0003f
#define MAGB_PNOISE_DEFAULT     0.0003f
#define VEL_GATE_DEFAULT        6
#define POS_GATE_DEFAULT        10
#define HGT_GATE_DEFAULT        20
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         0
#define GLITCH_ACCEL_DEFAULT    150
#define GLITCH_RADIUS_DEFAULT   15
#define FLOW_MEAS_DELAY         25

#endif // APM_BUILD_DIRECTORY


extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// when the wind estimation first starts with no airspeed sensor,
// assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// initial gyro bias uncertainty (deg/sec)
#define INIT_GYRO_BIAS_UNCERTAINTY 0.1f

// Define tuning parameters
const AP_Param::GroupInfo NavEKF::var_info[] PROGMEM = {

    // @Param: VELNE_NOISE
    // @DisplayName: GPS horizontal velocity measurement noise (m/s)
    // @Description: This is the RMS value of noise in the North and East GPS velocity measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELNE_NOISE",    0, NavEKF, _gpsHorizVelNoise, VELNE_NOISE_DEFAULT),

    // @Param: VELD_NOISE
    // @DisplayName: GPS vertical velocity measurement noise (m/s)
    // @Description: This is the RMS value of noise in the vertical GPS velocity measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELD_NOISE",    1, NavEKF, _gpsVertVelNoise, VELD_NOISE_DEFAULT),

    // @Param: POSNE_NOISE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("POSNE_NOISE",    2, NavEKF, _gpsHorizPosNoise, POSNE_NOISE_DEFAULT),

    // @Param: ALT_NOISE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ALT_NOISE",    3, NavEKF, _baroAltNoise, ALT_NOISE_DEFAULT),

    // @Param: MAG_NOISE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MAG_NOISE",    4, NavEKF, _magNoise, MAG_NOISE_DEFAULT),

    // @Param: EAS_NOISE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("EAS_NOISE",    5, NavEKF, _easNoise, 1.4f),

    // @Param: WIND_PNOISE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PNOISE",    6, NavEKF, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind procss noise scaler
    // @Description: Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE",    7, NavEKF, _wndVarHgtRateScale, 0.5f),

    // @Param: GYRO_PNOISE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.001 0.05
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("GYRO_PNOISE",    8, NavEKF, _gyrNoise, GYRO_PNOISE_DEFAULT),

    // @Param: ACC_PNOISE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.05 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("ACC_PNOISE",    9, NavEKF, _accNoise, ACC_PNOISE_DEFAULT),

    // @Param: GBIAS_PNOISE
    // @DisplayName: Rate gyro bias process noise (rad/s)
    // @Description: This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    AP_GROUPINFO("GBIAS_PNOISE",    10, NavEKF, _gyroBiasProcessNoise, GBIAS_PNOISE_DEFAULT),

    // @Param: ABIAS_PNOISE
    // @DisplayName: Accelerometer bias process noise (m/s^2)
    // @Description: This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    AP_GROUPINFO("ABIAS_PNOISE",    11, NavEKF, _accelBiasProcessNoise, ABIAS_PNOISE_DEFAULT),

    // @Param: MAGE_PNOISE
    // @DisplayName: Earth magnetic field process noise (gauss/s)
    // @Description: This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("MAGE_PNOISE",    12, NavEKF, _magEarthProcessNoise, MAGE_PNOISE_DEFAULT),

    // @Param: MAGB_PNOISE
    // @DisplayName: Body magnetic field process noise (gauss/s)
    // @Description: This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("MAGB_PNOISE",    13, NavEKF, _magBodyProcessNoise, MAGB_PNOISE_DEFAULT),

    // @Param: VEL_DELAY
    // @DisplayName: GPS velocity measurement delay (msec)
    // @Description: This is the number of msec that the GPS velocity measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("VEL_DELAY",    14, NavEKF, _msecVelDelay, 220),

    // @Param: POS_DELAY
    // @DisplayName: GPS position measurement delay (msec)
    // @Description: This is the number of msec that the GPS position measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("POS_DELAY",    15, NavEKF, _msecPosDelay, 220),

    // @Param: GPS_TYPE
    // @DisplayName: GPS velocity mode control
    // @Description: This parameter controls use of GPS velocity measurements : 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity, 3 = use no GPS
    // @Range: 0 3
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE",    16, NavEKF, _fusionModeGPS, 0),

    // @Param: VEL_GATE
    // @DisplayName: GPS velocity measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL_GATE",    17, NavEKF, _gpsVelInnovGate, VEL_GATE_DEFAULT),

    // @Param: POS_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POS_GATE",    18, NavEKF, _gpsPosInnovGate, POS_GATE_DEFAULT),

    // @Param: HGT_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HGT_GATE",    19, NavEKF, _hgtInnovGate, HGT_GATE_DEFAULT),

    // @Param: MAG_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAG_GATE",    20, NavEKF, _magInnovGate, MAG_GATE_DEFAULT),

    // @Param: EAS_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EAS_GATE",    21, NavEKF, _tasInnovGate, 10),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer calibration mode
    // @Description: EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
    // @Values: 0:Speed and Height,1:Acceleration,2:Never
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAG_CAL",    22, NavEKF, _magCal, MAG_CAL_DEFAULT),

    // @Param: GLITCH_ACCEL
    // @DisplayName: GPS glitch accel gate size (cm/s^2)
    // @Description: This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position.
    // @Range: 100 500
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("GLITCH_ACCEL",    23, NavEKF, _gpsGlitchAccelMax, GLITCH_ACCEL_DEFAULT),

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and an offset is applied to the GPS measurement to compensate. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("GLITCH_RAD",    24, NavEKF, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // @Param: GND_GRADIENT
    // @DisplayName: Terrain Gradient % RMS
    // @Description: This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values casue the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
    // @Range: 1 - 50
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("GND_GRADIENT",    25, NavEKF, _gndGradientSigma, 2),

    // @Param: FLOW_NOISE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 - 1.0
    // @Increment: 0.05
    // @User: advanced
    AP_GROUPINFO("FLOW_NOISE",    26, NavEKF, _flowNoise, 0.3f),

    // @Param: FLOW_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("FLOW_GATE",    27, NavEKF, _flowInnovGate, 3),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: advanced
    AP_GROUPINFO("FLOW_DELAY",    28, NavEKF, _msecFLowDelay, FLOW_MEAS_DELAY),

    // @Param: RNG_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("RNG_GATE",    29, NavEKF, _rngInnovGate, 5),

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 - 4.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("MAX_FLOW",    30, NavEKF, _maxFlowRate, 2.5f),

    // @Param: FALLBACK
    // @DisplayName: Fallback strictness
    // @Description: This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
    // @Values: 0:Trust EKF more, 1:Trust DCM more
    // @User: Advanced
    AP_GROUPINFO("FALLBACK",    31, NavEKF, _fallback, 1),

    AP_GROUPEND
};

// constructor
NavEKF::NavEKF(const AP_AHRS *ahrs, AP_Baro &baro) :
    _ahrs(ahrs),
    _baro(baro),
    state(*reinterpret_cast<struct state_elements *>(&states)),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    msecHgtDelay(60),               // Height measurement delay (msec)
    msecMagDelay(40),               // Magnetometer measurement delay (msec)
    msecTasDelay(240),              // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS(10000),      // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS(10000),       // GPS retry time without airspeed measurements (msec)
    hgtRetryTimeMode0(10000),       // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12(5000),       // Height retry time without vertical velocity measurement (msec)
    tasRetryTime(5000),             // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.05f),         // scale factor applied to magnetometer variance due to angular rate
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to gyro bias state process noise when on ground
    msecGpsAvg(200),                // average number of msec between GPS measurements
    msecHgtAvg(100),                // average number of msec between height measurements
    msecMagAvg(100),                // average number of msec between magnetometer measurements
    msecBetaAvg(100),               // average number of msec between synthetic sideslip measurements
    msecBetaMax(200),               // maximum number of msec between synthetic sideslip measurements
    msecFlowAvg(100),               // average number of msec between optical flow measurements
    dtVelPos(0.2f),                 // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    covTimeStepMax(0.07f),          // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    TASmsecMax(200),                // maximum allowed interval between airspeed measurement updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100)         // maximum allowable time between flow fusion events

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_UpdateFilter(perf_alloc(PC_ELAPSED, "EKF_UpdateFilter")),
    _perf_CovariancePrediction(perf_alloc(PC_ELAPSED, "EKF_CovariancePrediction")),
    _perf_FuseVelPosNED(perf_alloc(PC_ELAPSED, "EKF_FuseVelPosNED")),
    _perf_FuseMagnetometer(perf_alloc(PC_ELAPSED, "EKF_FuseMagnetometer")),
    _perf_FuseAirspeed(perf_alloc(PC_ELAPSED, "EKF_FuseAirspeed")),
    _perf_FuseSideslip(perf_alloc(PC_ELAPSED, "EKF_FuseSideslip"))
#endif
{
    AP_Param::setup_object_defaults(this, var_info);

}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF::healthy(void) const
{
    if (!statesInitialised) {
        return false;
    }
    if (state.quat.is_nan()) {
        return false;
    }
    if (state.velocity.is_nan()) {
        return false;
    }
    if (_fallback && velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1) {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }
    // Give the filter 10 seconds to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 10000) {
        return false;
    }

    // all OK
    return true;
}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF::ResetPosition(void)
{
    if (constPosMode || (PV_AidingMode != AID_ABSOLUTE)) {
        state.position.x = 0;
        state.position.y = 0;
    } else if (!gpsNotAvailable) {
        // write to state vector and compensate for GPS latency
        state.position.x = gpsPosNE.x + gpsPosGlitchOffsetNE.x + 0.001f*velNED.x*float(_msecPosDelay);
        state.position.y = gpsPosNE.y + gpsPosGlitchOffsetNE.y + 0.001f*velNED.y*float(_msecPosDelay);
    }
    // stored horizontal position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position[0] = state.position[0];
        storedStates[i].position[1] = state.position[1];
    }
}

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF::ResetVelocity(void)
{
    if (constPosMode || PV_AidingMode != AID_ABSOLUTE) {
         state.velocity.zero();
         state.vel1.zero();
         state.vel2.zero();
    } else if (!gpsNotAvailable) {
        // reset horizontal velocity states, applying an offset to the GPS velocity to prevent the GPS position being rejected when the GPS position offset is being decayed to zero.
        state.velocity.x  = velNED.x + gpsVelGlitchOffset.x; // north velocity from blended accel data
        state.velocity.y  = velNED.y + gpsVelGlitchOffset.y; // east velocity from blended accel data
        state.vel1.x      = velNED.x + gpsVelGlitchOffset.x; // north velocity from IMU1 accel data
        state.vel1.y      = velNED.y + gpsVelGlitchOffset.y; // east velocity from IMU1 accel data
        state.vel2.x      = velNED.x + gpsVelGlitchOffset.x; // north velocity from IMU2 accel data
        state.vel2.y      = velNED.y + gpsVelGlitchOffset.y; // east velocity from IMU2 accel data
        // over write stored horizontal velocity states to prevent subsequent GPS measurements from being rejected
        for (uint8_t i=0; i<=49; i++){
            storedStates[i].velocity.x = velNED.x + gpsVelGlitchOffset.x;
            storedStates[i].velocity.y = velNED.y + gpsVelGlitchOffset.y;
        }
    }
}

// reset the vertical position state using the last height measurement
void NavEKF::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    state.position.z = -hgtMea; // down position from blended accel data
    state.posD1 = -hgtMea; // down position from IMU1 accel data
    state.posD2 = -hgtMea; // down position from IMU2 accel data
    // reset stored vertical position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.z = -hgtMea;
    }
    terrainState = states[9] + 0.1f;
}

// this function is used to initialise the filter whilst moving, using the AHRS DCM solution
// it should NOT be used to re-initialise after a timeout as DCM will also be corrupted
void NavEKF::InitialiseFilterDynamic(void)
{
    // this forces healthy() to be false so that when we ask for ahrs
    // attitude we get the DCM attitude regardless of the state of AHRS_EKF_USE
    statesInitialised = false;

    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    // Set re-used variables to zero
    InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtIMU = constrain_float(_ahrs->get_ins().get_delta_time(),0.001f,1.0f);
    dtIMUinv = 1.0f / dtIMU;

    // If we have a high rate update step of >100 Hz, then there may not be enough time to do all the fusion steps at once, so load levelling is used
    if (dtIMU < 0.009f) {
        inhibitLoadLeveling = false;
    } else {
        inhibitLoadLeveling = true;
    }
    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    gpsUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecGpsAvg);
    gpsUpdateCountMax = uint8_t(1.0f/gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecHgtAvg);
    hgtUpdateCountMax = uint8_t(1.0f/hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecMagAvg);
    magUpdateCountMax = uint8_t(1.0f/magUpdateCountMaxInv);
    flowUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecFlowAvg);
    flowUpdateCountMax = uint8_t(1.0f/flowUpdateCountMaxInv);

    // calculate initial orientation and earth magnetic field states
    state.quat = calcQuatAndFieldStates(_ahrs->roll, _ahrs->pitch);

    // write to state vector
    state.gyro_bias.zero();
    state.accel_zbias1 = 0;
    state.accel_zbias2 = 0;
    state.wind_vel.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readHgtData();
    ResetHeight();

    // set stored states to current state
    StoreStatesReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
void NavEKF::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        statesInitialised = false;
        return;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtIMU = constrain_float(_ahrs->get_ins().get_delta_time(),0.001f,1.0f);
    dtIMUinv = 1.0f / dtIMU;

    // If we have a high rate update step of >100 Hz, then there may not enough time to all the fusion steps at once, so load levelling is used
    if (dtIMU < 0.009f) {
        inhibitLoadLeveling = false;
    } else {
        inhibitLoadLeveling = true;
    }

    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    gpsUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecGpsAvg);
    gpsUpdateCountMax = uint8_t(1.0f/gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecHgtAvg);
    hgtUpdateCountMax = uint8_t(1.0f/hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMU * 1000.0f)/float(msecMagAvg);
    magUpdateCountMax = uint8_t(1.0f/magUpdateCountMaxInv);

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = _ahrs->get_ins().get_accel();

    // read the magnetometer data
    readMagData();

    // normalise the acceleration vector
    float pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinf(initAccVec.x);

        // calculate initial roll angle
        roll = -asinf(initAccVec.y / cosf(pitch));
    }

    // calculate initial orientation and earth magnetic field states
    Quaternion initQuat;
    initQuat = calcQuatAndFieldStates(roll, pitch);

    // check on ground status
    SetFlightAndFusionModes();

    // write to state vector
    state.quat = initQuat;
    state.gyro_bias.zero();
    state.accel_zbias1 = 0;
    state.accel_zbias2 = 0;
    state.wind_vel.zero();
    state.body_magfield.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readHgtData();
    ResetHeight();

    // set stored states to current state
    StoreStatesReset();

    // set to true now we have intialised the states
    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF::UpdateFilter()
{
    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
    perf_begin(_perf_UpdateFilter);

    //get starting time for update step
    imuSampleTime_ms = hal.scheduler->millis();

    // read IMU data and convert to delta angles and velocities
    readIMUData();

    // detect if the filter update has been delayed for too long
    if (dtIMU > 0.2f) {
        // we have stalled for too long - reset states
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        StoreStatesReset();
        //Initialise IMU pre-processing states
        readIMUData();
        // stop the timer used for load measurement
        perf_end(_perf_UpdateFilter);
        return;
    }

    // check if on ground
    SetFlightAndFusionModes();

    // Check arm status and perform required checks and mode changes
    performArmingChecks();

    // run the strapdown INS equations every IMU update
    UpdateStrapdownEquationsNED();

    // store the predicted states for subsequent use by measurement fusion
    StoreStates();

    // sum delta angles and time used by covariance prediction
    summedDelAng = summedDelAng + correctedDelAng;
    summedDelVel = summedDelVel + correctedDelVel1;
    dt += dtIMU;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtIMU)) || (summedDelAng.length() > covDelAngMax))) {
        CovariancePrediction();
        covPredStep = true;
        summedDelAng.zero();
        summedDelVel.zero();
        dt = 0.0;
    } else {
        covPredStep = false;
    }

    // Update states using GPS, altimeter, compass, airspeed and synthetic sideslip observations
    SelectVelPosFusion();
    SelectMagFusion();
    SelectFlowFusion();
    SelectTasFusion();
    SelectBetaFusion();

    // stop the timer used for load measurement
    perf_end(_perf_UpdateFilter);
}

// select fusion of velocity, position and height measurements
void NavEKF::SelectVelPosFusion()
{
    // check for new data, specify which measurements should be used and check data for freshness
    if (PV_AidingMode == AID_ABSOLUTE) {

        // check for and read new GPS data
        readGpsData();

        // Set GPS time-out threshold depending on whether we have an airspeed sensor to constrain drift
        uint16_t gpsRetryTimeout = useAirspeed() ? gpsRetryTimeUseTAS : gpsRetryTimeNoTAS;

        // If we haven't received GPS data for a while, then declare the position and velocity data as being timed out
        if (imuSampleTime_ms - lastFixTime_ms > gpsRetryTimeout) {
            posTimeout = true;
            velTimeout = true;
            //If this happens in flight and we don't have airspeed or sideslip assumption to constrain drift, then go into constant position mode and stay there until disarmed
            if (vehicleArmed && !useAirspeed() && !assume_zero_sideslip()) {
                constVelMode = false; // always clear constant velocity mode if constant velocity mode is active
                constPosMode = true;
                PV_AidingMode = AID_NONE;
                // reset the velocity
                ResetVelocity();
                // store the current position to be used to keep reporting the last known position
                lastKnownPositionNE.x = state.position.x;
                lastKnownPositionNE.y = state.position.y;
                // reset the position
                ResetPosition();
            }
        }

        // command fusion of GPS data and reset states as required
        if (newDataGps) {
            // reset data arrived flag
            newDataGps = false;
            // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
            memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
            gpsUpdateCount = 0;
            // select which of velocity and position measurements will be fused
            if (PV_AidingMode == AID_ABSOLUTE) {
                // use both if GPS use is enabled
                fuseVelData = true;
                fusePosData = true;
                // If a long time since last GPS update, then reset position and velocity and reset stored state history
                if (imuSampleTime_ms - secondLastFixTime_ms > gpsRetryTimeout) {
                    // Apply an offset to the GPS position so that the position can be corrected gradually
                    gpsPosGlitchOffsetNE.x = statesAtPosTime.position.x - gpsPosNE.x;
                    gpsPosGlitchOffsetNE.y = statesAtPosTime.position.y - gpsPosNE.y;
                    // limit the radius of the offset to 100m and decay the offset to zero radially
                    decayGpsOffset();
                    ResetPosition();
                    ResetVelocity();
                }
            } else {
                fuseVelData = false;
                fusePosData = false;
            }
        } else {
            fuseVelData = false;
            fusePosData = false;
        }
    } else if (constPosMode ) {
        // in constant position mode use synthetic position measurements set to zero
        // only fuse synthetic measurements when rate of change of velocity is less than 0.5g to reduce attitude errors due to launch acceleration
        // do not use velocity fusion to reduce the effect of movement on attitude
        if (accNavMag < 4.9f) {
            fusePosData = true;
        } else {
            fusePosData = false;
        }
        fuseVelData = false;
    } else if (constVelMode) {
        // In constant velocity mode we fuse the last valid velocity vector
        // Reset the stored velocity vector when we enter the mode
        if (constVelMode && !lastConstVelMode) {
            heldVelNE.x = state.velocity.x;
            heldVelNE.y = state.velocity.y;
        }
        lastConstVelMode = constVelMode;
        // We do not fuse when manoeuvring to avoid corrupting the attitude
        if (accNavMag < 4.9f) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = false;
    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // check for and read new height data
    readHgtData();

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime = (_fusionModeGPS == 0 && !velTimeout) ? hgtRetryTimeMode0 : hgtRetryTimeMode12;
    if (imuSampleTime_ms - lastHgtMeasTime > hgtRetryTime) {
        hgtTimeout = true;
    }

    // command fusion of height data
    if (newDataHgt)
    {
        // reset data arrived flag
        newDataHgt = false;
        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&hgtIncrStateDelta[0], 0, sizeof(hgtIncrStateDelta));
        hgtUpdateCount = 0;
        // enable fusion
        fuseHgtData = true;
    } else {
        fuseHgtData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
            FuseVelPosNED();
    }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 5 and 10Hz pulsing in the output
    if (gpsUpdateCount < gpsUpdateCountMax) {
        gpsUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += gpsIncrStateDelta[i];
        }
    }
    if (hgtUpdateCount < hgtUpdateCountMax) {
        hgtUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += hgtIncrStateDelta[i];
        }
    }
}

// select fusion of magnetometer data
void NavEKF::SelectMagFusion()
{
    // check for and read new magnetometer measurements
    readMagData();

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    // determine if conditions are right to start a new fusion cycle
    bool dataReady = statesInitialised && use_compass() && newDataMag;
    if (dataReady)
    {
        fuseMagData = true;
        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&magIncrStateDelta[0], 0, sizeof(magIncrStateDelta));
        magUpdateCount = 0;
    }
    else
    {
        fuseMagData = false;
    }

        // delay if covariance prediction is being performed on this prediction cycle unless load levelling is inhibited
        if (!covPredStep || inhibitLoadLeveling) {
            FuseMagnetometer();
        }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 10Hz pulsing in the output
    if (magUpdateCount < magUpdateCountMax) {
        magUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += magIncrStateDelta[i];
        }
    }
}

// select fusion of optical flow measurements
void NavEKF::SelectFlowFusion()
{
    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 200);
    // Check if the fusion has timed out (flow measurements have been rejected for too long)
    bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowUseTime_ms) > 5000);
    // check is the terrain offset estimate is still valid
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000);
    // Perform tilt check
    bool tiltOK = (Tnb_flow.c.z > DCM33FlowMin);
    // if we have waited too long, set a timeout flag which will force fusion to occur regardless of load spreading
    bool flowFuseNow = ((imuSampleTime_ms - flowValidMeaTime_ms) >= flowIntervalMax_ms/2);
    // check if fusion should be delayed to spread load. Setting fuseMeNow to true disables this load spreading feature.
    bool delayFusion = ((covPredStep || magFusePerformed) && !(flowFuseNow || inhibitLoadLeveling));

    // if we don't have valid flow measurements and are relying on them, dead reckon using current velocity vector
    // If the flow measurements have been rejected for too long and we are relying on them, then reset the velocities to an estimate based on the flow and range data
    if (!flowDataValid && PV_AidingMode == AID_RELATIVE) {
        constVelMode = true;
        constPosMode = false; // always clear constant position mode if constant velocity mode is active
    } else if (flowDataValid && flowFusionTimeout && PV_AidingMode == AID_RELATIVE) {
        // we need to reset the velocities to a value estimated from the flow data
        // estimate the range
        float initPredRng = max((terrainState - state.position[2]),0.1f) / Tnb_flow.c.z;
        // multiply the range by the LOS rates to get an estimated XY velocity in body frame
        Vector3f initVel;
        initVel.x = -flowRadXYcomp[1]*initPredRng;
        initVel.y =  flowRadXYcomp[0]*initPredRng;
        // rotate into earth frame
        initVel = Tbn_flow*initVel;
        // set horizontal velocity states
        state.velocity.x = initVel.x;
        state.velocity.y = initVel.y;
        // clear any hold modes
        constVelMode = false;
        lastConstVelMode = false;
    } else if (flowDataValid) {
        // clear the constant velocity mode now we have good data
        constVelMode = false;
        lastConstVelMode = false;
    }
    // if we do have valid flow measurements

    // Fuse data into a 1-state EKF to estimate terrain height
    if ((newDataFlow || newDataRng) && tiltOK) {
        // fuse range data into the terrain estimator if available
        fuseRngData = newDataRng;
        // fuse optical flow data into the terrain estimator if available and if there is no range data (range data is better)
        fuseOptFlowData = (newDataFlow && !fuseRngData);
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset();
        // Indicate we have used the range data
        newDataRng = false;
        // we don't do subsequent fusion of optical flow data into the main filter if GPS is good and terrain offset data is invalid
        // because an invalid height above ground estimate will casue the optical flow measurements to fight the GPS
        if (!gpsNotAvailable && !gndOffsetValid) {
            // turn of fusion permissions
            // reset the measurement axis index
            flow_state.obsIndex = 0;
            // reset the flags to indicate that no new range finder or flow data is available for fusion
            newDataFlow = false;
        }
    }

    // Fuse optical flow data into the main filter
    // if the filter is initialised, we have data to fuse and the vehicle is not excessively tilted, then perform optical flow fusion
    if (flowDataValid && newDataFlow && tiltOK && !delayFusion && !constPosMode)
    {
        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&flowIncrStateDelta[0], 0, sizeof(flowIncrStateDelta));
        flowUpdateCount = 0;
        // Set the flow noise used by the fusion processes
        R_LOS = sq(max(_flowNoise, 0.05f));
        // set the measurement axis index to fuse the X axis data
        flow_state.obsIndex = 0;
        // Fuse the optical flow X axis data into the main filter
        FuseOptFlow();
        // increment the measurement axis index to fuse the Y axis data on the next prediction cycle
        flow_state.obsIndex = 1;
        // reset flag to indicate that no new flow data is available for fusion
        newDataFlow = false;
        // indicate that flow fusion has been performed. This is used for load spreading.
        flowFusePerformed = true;
        // update the time stamp
        prevFlowUseTime_ms = imuSampleTime_ms;
    } else if (flowDataValid && flow_state.obsIndex == 1 && !delayFusion && !constPosMode && tiltOK) {
        // Fuse the optical flow Y axis data into the main filter
        FuseOptFlow();
        // Reset the measurement axis index to prevent further fusion of this data
        flow_state.obsIndex = 0;
        // reset flag to indicate that no new flow data is available for fusion
        newDataFlow = false;
        // indicate that flow fusion has been performed. This is used for load spreading.
        flowFusePerformed = true;
    }

    // Apply corrections to quaternion, position and velocity states across several time steps to reduce 10Hz pulsing in the output
    if (flowUpdateCount < flowUpdateCountMax) {
        flowUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += flowIncrStateDelta[i];
        }
    }
}

// select fusion of true airspeed measurements
void NavEKF::SelectTasFusion()
{
    // get true airspeed measurement
    readAirSpdData();

    // If we haven't received airspeed data for a while, then declare the airspeed data as being timed out
    if (imuSampleTime_ms - lastAirspeedUpdate > tasRetryTime) {
        tasTimeout = true;
    }

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then queue TAS fusion
    tasDataWaiting = (statesInitialised && !inhibitWindStates && (tasDataWaiting || newDataTas));
    // if we have waited too long, set a timeout flag which will force fusion to occur
    bool timeout = ((imuSampleTime_ms - TASmsecPrev) >= TASmsecMax);
    // we don't fuse airspeed measurements if magnetometer fusion has been performed in the same frame, unless timed out or the fuseMeNow option is selected
    // this helps to spreasthe load associated with fusion of different measurements across multiple frames
    // setting fuseMeNow to true disables this load spreading feature
    if (tasDataWaiting && (!(covPredStep || magFusePerformed || flowFusePerformed) || timeout || inhibitLoadLeveling))
    {
        FuseAirspeed();
        TASmsecPrev = imuSampleTime_ms;
        tasDataWaiting = false;
    }
}

// select fusion of synthetic sideslip measurements
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// it requires a stable wind for best results and should not be used for aerobatic flight with manoeuvres that induce large sidslip angles (eg knife-edge, spins, etc)
void NavEKF::SelectBetaFusion()
{
    // set to true if fusion is locked out due to magnetometer fusion on the same time step (done for load levelling)
    bool f_lockedOut = (magFusePerformed && !inhibitLoadLeveling);
    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - BETAmsecPrev) >= msecBetaAvg);
    // set true when use of synthetic sideslip fusion is necessary because we have limited sensor data or are dead reckoning position
    bool f_required = !(use_compass() && useAirspeed() && posHealth);
    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    bool f_feasible = (assume_zero_sideslip() && !inhibitWindStates);
    // use synthetic sideslip fusion if feasible, required, enough time has lapsed since the last fusion and it is not locked out
    if (f_feasible && f_required && f_timeTrigger && !f_lockedOut) {
        FuseSideslip();
        BETAmsecPrev = imuSampleTime_ms;
    }
}

// update the quaternion, velocity and position states using IMU measurements
void NavEKF::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;  // delta velocity vector calculated using a blend of IMU1 and IMU2 data
    Vector3f delVelNav1; // delta velocity vector calculated using IMU1 data
    Vector3f delVelNav2; // delta velocity vector calculated using IMU2 data
    float rotationMag;   // magnitude of rotation vector from previous to current time step
    float rotScaler;     // scaling variable used to calculate delta quaternion from last to current time step
    Quaternion qUpdated; // quaternion at current time step after application of delta quaternion
    Quaternion deltaQuat; // quaternion from last to current time step
    const Vector3f gravityNED(0, 0, GRAVITY_MSS); // NED gravity vector m/s^2

    // remove sensor bias errors
    correctedDelAng = dAngIMU - state.gyro_bias;
    correctedDelVel1 = dVelIMU1;
    correctedDelVel2 = dVelIMU2;
    correctedDelVel1.z -= state.accel_zbias1;
    correctedDelVel2.z -= state.accel_zbias2;

    // use weighted average of both IMU units for delta velocities
    correctedDelVel12 = correctedDelVel1 * IMU1_weighting + correctedDelVel2 * (1.0f - IMU1_weighting);

    // apply corrections for earths rotation rate and coning errors
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*dtIMU + (prevDelAng % correctedDelAng) * 8.333333e-2f;

    // save current measurements
    prevDelAng = correctedDelAng;

    // convert the rotation vector to its equivalent quaternion
    rotationMag = correctedDelAng.length();
    if (rotationMag < 1e-12f)
    {
        deltaQuat[0] = 1;
        deltaQuat[1] = 0;
        deltaQuat[2] = 0;
        deltaQuat[3] = 0;
    }
    else
    {
        deltaQuat[0] = cosf(0.5f * rotationMag);
        rotScaler = (sinf(0.5f * rotationMag)) / rotationMag;
        deltaQuat[1] = correctedDelAng.x * rotScaler;
        deltaQuat[2] = correctedDelAng.y * rotScaler;
        deltaQuat[3] = correctedDelAng.z * rotScaler;
    }

    // update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    qUpdated[0] = states[0]*deltaQuat[0] - states[1]*deltaQuat[1] - states[2]*deltaQuat[2] - states[3]*deltaQuat[3];
    qUpdated[1] = states[0]*deltaQuat[1] + states[1]*deltaQuat[0] + states[2]*deltaQuat[3] - states[3]*deltaQuat[2];
    qUpdated[2] = states[0]*deltaQuat[2] + states[2]*deltaQuat[0] + states[3]*deltaQuat[1] - states[1]*deltaQuat[3];
    qUpdated[3] = states[0]*deltaQuat[3] + states[3]*deltaQuat[0] + states[1]*deltaQuat[2] - states[2]*deltaQuat[1];

    // normalise the quaternions and update the quaternion states
    qUpdated.normalize();
    state.quat = qUpdated;

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    state.quat.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    // blended IMU calc
    delVelNav  = Tbn_temp*correctedDelVel12 + gravityNED*dtIMU;
    // single IMU calcs
    delVelNav1 = Tbn_temp*correctedDelVel1 + gravityNED*dtIMU;
    delVelNav2 = Tbn_temp*correctedDelVel2 + gravityNED*dtIMU;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / dtIMU ;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = pythagorous2(velDotNEDfilt.x , velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = state.velocity;
    Vector3f lastVel1     = state.vel1;
    Vector3f lastVel2     = state.vel2;

    // sum delta velocities to get velocity
    state.velocity += delVelNav;
    state.vel1     += delVelNav1;
    state.vel2     += delVelNav2;

    // apply a trapezoidal integration to velocities to calculate position
    state.position += (state.velocity + lastVelocity) * (dtIMU*0.5f);
    state.posD1    += (state.vel1.z + lastVel1.z) * (dtIMU*0.5f);
    state.posD2    += (state.vel2.z + lastVel2.z) * (dtIMU*0.5f);

    // capture current angular rate to augmented state vector for use by optical flow fusion
    state.omega = correctedDelAng * dtIMUinv;

    // limit states to protect against divergence
    ConstrainStates();
}

// calculate the predicted state covariance matrix
void NavEKF::CovariancePrediction()
{
    perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxCov;       // X axis delta angle variance rad^2
    float dayCov;       // Y axis delta angle variance rad^2
    float dazCov;       // Z axis delta angle variance rad^2
    float dvxCov;       // X axis delta velocity variance (m/s)^2
    float dvyCov;       // Y axis delta velocity variance (m/s)^2
    float dvzCov;       // Z axis delta velocity variance (m/s)^2
    float dvx;          // X axis delta velocity (m/s)
    float dvy;          // Y axis delta velocity (m/s)
    float dvz;          // Z axis delta velocity (m/s)
    float dax;          // X axis delta angle (rad)
    float day;          // Y axis delta angle (rad)
    float daz;          // Z axis delta angle (rad)
    float q0;           // attitude quaternion
    float q1;           // attitude quaternion
    float q2;           // attitude quaternion
    float q3;           // attitude quaternion
    float dax_b;        // X axis delta angle measurement bias (rad)
    float day_b;        // Y axis delta angle measurement bias (rad)
    float daz_b;        // Z axis delta angle measurement bias (rad)
    float dvz_b;        // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - state.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    if (!inhibitWindStates) {
        windVelSigma  = dt * constrain_float(_windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    } else {
        windVelSigma  = 0.0f;
    }
    dAngBiasSigma = dt * constrain_float(_gyroBiasProcessNoise, 1e-7f, 1e-5f);
    dVelBiasSigma = dt * constrain_float(_accelBiasProcessNoise, 1e-5f, 1e-3f);
    if (!inhibitMagStates) {
        magEarthSigma = dt * constrain_float(_magEarthProcessNoise, 1e-4f, 1e-2f);
        magBodySigma  = dt * constrain_float(_magBodyProcessNoise, 1e-4f, 1e-2f);
    } else {
        magEarthSigma = 0.0f;
        magBodySigma  = 0.0f;
    }
    for (uint8_t i= 0; i<=9;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dAngBiasSigma;
    // scale gyro bias noise when disarmed to allow for faster bias estimation
    for (uint8_t i=10; i<=12; i++) {
        processNoise[i] = dAngBiasSigma;
        if (!vehicleArmed) {
            processNoise[i] *= gyroBiasNoiseScaler;
        }
    }
	processNoise[13] = dVelBiasSigma;
    for (uint8_t i=14; i<=15; i++) processNoise[i] = windVelSigma;
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i= 0; i<=21; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = state.quat[0];
    q1 = state.quat[1];
    q2 = state.quat[2];
    q3 = state.quat[3];
    dax_b = state.gyro_bias.x;
    day_b = state.gyro_bias.y;
    daz_b = state.gyro_bias.z;
    dvz_b = IMU1_weighting * state.accel_zbias1 + (1.0f - IMU1_weighting) * state.accel_zbias2;
    _gyrNoise = constrain_float(_gyrNoise, 1e-3f, 5e-2f);
    daxCov = sq(dt*_gyrNoise);
    dayCov = sq(dt*_gyrNoise);
    dazCov = sq(dt*_gyrNoise);
    _accNoise = constrain_float(_accNoise, 5e-2f, 1.0f);
    dvxCov = sq(dt*_accNoise);
    dvyCov = sq(dt*_accNoise);
    dvzCov = sq(dt*_accNoise);

    // calculate the predicted covariance due to inertial sensor error propagation
	SF[0] = dvz - dvz_b;
	SF[1] = 2*q3*SF[0] + 2*dvx*q1 + 2*dvy*q2;
	SF[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
	SF[3] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
	SF[4] = day/2 - day_b/2;
	SF[5] = daz/2 - daz_b/2;
	SF[6] = dax/2 - dax_b/2;
	SF[7] = dax_b/2 - dax/2;
	SF[8] = daz_b/2 - daz/2;
	SF[9] = day_b/2 - day/2;
	SF[10] = 2*q0*SF[0];
	SF[11] = q1/2;
	SF[12] = q2/2;
	SF[13] = q3/2;
	SF[14] = 2*dvy*q1;

	SG[0] = q0/2;
	SG[1] = sq(q3);
	SG[2] = sq(q2);
	SG[3] = sq(q1);
	SG[4] = sq(q0);
	SG[5] = 2*q2*q3;
	SG[6] = 2*q1*q3;
	SG[7] = 2*q1*q2;

	SQ[0] = dvzCov*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyCov*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxCov*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
	SQ[1] = dvzCov*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxCov*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyCov*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
	SQ[2] = dvzCov*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyCov*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxCov*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
	SQ[3] = (dayCov*q1*SG[0])/2 - (dazCov*q1*SG[0])/2 - (daxCov*q2*q3)/4;
	SQ[4] = (dazCov*q2*SG[0])/2 - (daxCov*q2*SG[0])/2 - (dayCov*q1*q3)/4;
	SQ[5] = (daxCov*q3*SG[0])/2 - (dayCov*q3*SG[0])/2 - (dazCov*q1*q2)/4;
	SQ[6] = (daxCov*q1*q2)/4 - (dazCov*q3*SG[0])/2 - (dayCov*q1*q2)/4;
	SQ[7] = (dazCov*q1*q3)/4 - (daxCov*q1*q3)/4 - (dayCov*q2*SG[0])/2;
	SQ[8] = (dayCov*q2*q3)/4 - (daxCov*q1*SG[0])/2 - (dazCov*q2*q3)/4;
	SQ[9] = sq(SG[0]);
	SQ[10] = sq(q1);

	SPP[0] = SF[10] + SF[14] - 2*dvx*q2;
	SPP[1] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
	SPP[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
	SPP[3] = 2*q0*q1 - 2*q2*q3;
	SPP[4] = 2*q0*q2 + 2*q1*q3;
	SPP[5] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
	SPP[6] = SF[13];
	SPP[7] = SF[12];

	nextP[0][0] = P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6] + (daxCov*SQ[10])/4 + SF[7]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[9]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[8]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) + SPP[7]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[6]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) + (dayCov*sq(q2))/4 + (dazCov*sq(q3))/4;
	nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6] + SF[6]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[5]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[9]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[6]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) - SPP[7]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - (q0*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]))/2;
	nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6] + SF[4]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[8]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[6]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - SPP[6]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]))/2;
	nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6] + SF[5]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[4]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[7]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SF[11]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[7]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]))/2;
	nextP[0][4] = P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6] + SF[3]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[0]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SPP[2]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[4]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][5] = P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6] + SF[2]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[3]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[0]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[3]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][6] = P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6] + SF[2]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[1]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[0]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) - SPP[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][7] = P[0][7] + P[1][7]*SF[7] + P[2][7]*SF[9] + P[3][7]*SF[8] + P[10][7]*SF[11] + P[11][7]*SPP[7] + P[12][7]*SPP[6] + dt*(P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6]);
	nextP[0][8] = P[0][8] + P[1][8]*SF[7] + P[2][8]*SF[9] + P[3][8]*SF[8] + P[10][8]*SF[11] + P[11][8]*SPP[7] + P[12][8]*SPP[6] + dt*(P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6]);
	nextP[0][9] = P[0][9] + P[1][9]*SF[7] + P[2][9]*SF[9] + P[3][9]*SF[8] + P[10][9]*SF[11] + P[11][9]*SPP[7] + P[12][9]*SPP[6] + dt*(P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6]);
	nextP[0][10] = P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6];
	nextP[0][11] = P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6];
	nextP[0][12] = P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6];
	nextP[0][13] = P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6];
	nextP[0][14] = P[0][14] + P[1][14]*SF[7] + P[2][14]*SF[9] + P[3][14]*SF[8] + P[10][14]*SF[11] + P[11][14]*SPP[7] + P[12][14]*SPP[6];
	nextP[0][15] = P[0][15] + P[1][15]*SF[7] + P[2][15]*SF[9] + P[3][15]*SF[8] + P[10][15]*SF[11] + P[11][15]*SPP[7] + P[12][15]*SPP[6];
	nextP[0][16] = P[0][16] + P[1][16]*SF[7] + P[2][16]*SF[9] + P[3][16]*SF[8] + P[10][16]*SF[11] + P[11][16]*SPP[7] + P[12][16]*SPP[6];
	nextP[0][17] = P[0][17] + P[1][17]*SF[7] + P[2][17]*SF[9] + P[3][17]*SF[8] + P[10][17]*SF[11] + P[11][17]*SPP[7] + P[12][17]*SPP[6];
	nextP[0][18] = P[0][18] + P[1][18]*SF[7] + P[2][18]*SF[9] + P[3][18]*SF[8] + P[10][18]*SF[11] + P[11][18]*SPP[7] + P[12][18]*SPP[6];
	nextP[0][19] = P[0][19] + P[1][19]*SF[7] + P[2][19]*SF[9] + P[3][19]*SF[8] + P[10][19]*SF[11] + P[11][19]*SPP[7] + P[12][19]*SPP[6];
	nextP[0][20] = P[0][20] + P[1][20]*SF[7] + P[2][20]*SF[9] + P[3][20]*SF[8] + P[10][20]*SF[11] + P[11][20]*SPP[7] + P[12][20]*SPP[6];
	nextP[0][21] = P[0][21] + P[1][21]*SF[7] + P[2][21]*SF[9] + P[3][21]*SF[8] + P[10][21]*SF[11] + P[11][21]*SPP[7] + P[12][21]*SPP[6];
	nextP[1][0] = P[1][0] + SQ[8] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2 + SF[7]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) + SPP[7]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[6]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2);
	nextP[1][1] = P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] + daxCov*SQ[9] - (P[10][1]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[5]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[9]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[6]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) - SPP[7]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) + (dayCov*sq(q3))/4 + (dazCov*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2))/2;
	nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[8]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[6]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) - SPP[6]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2))/2;
	nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[4]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SF[11]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[7]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2))/2;
	nextP[1][4] = P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2 + SF[3]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SPP[2]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[4]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][5] = P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2 + SF[2]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][6] = P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2 + SF[2]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[1]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) - SPP[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][7] = P[1][7] + P[0][7]*SF[6] + P[2][7]*SF[5] + P[3][7]*SF[9] + P[11][7]*SPP[6] - P[12][7]*SPP[7] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2);
	nextP[1][8] = P[1][8] + P[0][8]*SF[6] + P[2][8]*SF[5] + P[3][8]*SF[9] + P[11][8]*SPP[6] - P[12][8]*SPP[7] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2);
	nextP[1][9] = P[1][9] + P[0][9]*SF[6] + P[2][9]*SF[5] + P[3][9]*SF[9] + P[11][9]*SPP[6] - P[12][9]*SPP[7] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2);
	nextP[1][10] = P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2;
	nextP[1][11] = P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2;
	nextP[1][12] = P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2;
	nextP[1][13] = P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2;
	nextP[1][14] = P[1][14] + P[0][14]*SF[6] + P[2][14]*SF[5] + P[3][14]*SF[9] + P[11][14]*SPP[6] - P[12][14]*SPP[7] - (P[10][14]*q0)/2;
	nextP[1][15] = P[1][15] + P[0][15]*SF[6] + P[2][15]*SF[5] + P[3][15]*SF[9] + P[11][15]*SPP[6] - P[12][15]*SPP[7] - (P[10][15]*q0)/2;
	nextP[1][16] = P[1][16] + P[0][16]*SF[6] + P[2][16]*SF[5] + P[3][16]*SF[9] + P[11][16]*SPP[6] - P[12][16]*SPP[7] - (P[10][16]*q0)/2;
	nextP[1][17] = P[1][17] + P[0][17]*SF[6] + P[2][17]*SF[5] + P[3][17]*SF[9] + P[11][17]*SPP[6] - P[12][17]*SPP[7] - (P[10][17]*q0)/2;
	nextP[1][18] = P[1][18] + P[0][18]*SF[6] + P[2][18]*SF[5] + P[3][18]*SF[9] + P[11][18]*SPP[6] - P[12][18]*SPP[7] - (P[10][18]*q0)/2;
	nextP[1][19] = P[1][19] + P[0][19]*SF[6] + P[2][19]*SF[5] + P[3][19]*SF[9] + P[11][19]*SPP[6] - P[12][19]*SPP[7] - (P[10][19]*q0)/2;
	nextP[1][20] = P[1][20] + P[0][20]*SF[6] + P[2][20]*SF[5] + P[3][20]*SF[9] + P[11][20]*SPP[6] - P[12][20]*SPP[7] - (P[10][20]*q0)/2;
	nextP[1][21] = P[1][21] + P[0][21]*SF[6] + P[2][21]*SF[5] + P[3][21]*SF[9] + P[11][21]*SPP[6] - P[12][21]*SPP[7] - (P[10][21]*q0)/2;
	nextP[2][0] = P[2][0] + SQ[7] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2 + SF[7]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + SPP[7]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[6]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2);
	nextP[2][1] = P[2][1] + SQ[5] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[5]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[9]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[6]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) - SPP[7]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - (q0*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2))/2;
	nextP[2][2] = P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] + dayCov*SQ[9] + (dazCov*SQ[10])/4 - (P[11][2]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[8]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[6]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - SPP[6]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + (daxCov*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2))/2;
	nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[4]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[7]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SF[11]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[7]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2))/2;
	nextP[2][4] = P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2 + SF[3]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SPP[2]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[4]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][5] = P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2 + SF[2]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][6] = P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2 + SF[2]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[1]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) - SPP[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][7] = P[2][7] + P[0][7]*SF[4] + P[1][7]*SF[8] + P[3][7]*SF[6] + P[12][7]*SF[11] - P[10][7]*SPP[6] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2);
	nextP[2][8] = P[2][8] + P[0][8]*SF[4] + P[1][8]*SF[8] + P[3][8]*SF[6] + P[12][8]*SF[11] - P[10][8]*SPP[6] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2);
	nextP[2][9] = P[2][9] + P[0][9]*SF[4] + P[1][9]*SF[8] + P[3][9]*SF[6] + P[12][9]*SF[11] - P[10][9]*SPP[6] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2);
	nextP[2][10] = P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2;
	nextP[2][11] = P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2;
	nextP[2][12] = P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2;
	nextP[2][13] = P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2;
	nextP[2][14] = P[2][14] + P[0][14]*SF[4] + P[1][14]*SF[8] + P[3][14]*SF[6] + P[12][14]*SF[11] - P[10][14]*SPP[6] - (P[11][14]*q0)/2;
	nextP[2][15] = P[2][15] + P[0][15]*SF[4] + P[1][15]*SF[8] + P[3][15]*SF[6] + P[12][15]*SF[11] - P[10][15]*SPP[6] - (P[11][15]*q0)/2;
	nextP[2][16] = P[2][16] + P[0][16]*SF[4] + P[1][16]*SF[8] + P[3][16]*SF[6] + P[12][16]*SF[11] - P[10][16]*SPP[6] - (P[11][16]*q0)/2;
	nextP[2][17] = P[2][17] + P[0][17]*SF[4] + P[1][17]*SF[8] + P[3][17]*SF[6] + P[12][17]*SF[11] - P[10][17]*SPP[6] - (P[11][17]*q0)/2;
	nextP[2][18] = P[2][18] + P[0][18]*SF[4] + P[1][18]*SF[8] + P[3][18]*SF[6] + P[12][18]*SF[11] - P[10][18]*SPP[6] - (P[11][18]*q0)/2;
	nextP[2][19] = P[2][19] + P[0][19]*SF[4] + P[1][19]*SF[8] + P[3][19]*SF[6] + P[12][19]*SF[11] - P[10][19]*SPP[6] - (P[11][19]*q0)/2;
	nextP[2][20] = P[2][20] + P[0][20]*SF[4] + P[1][20]*SF[8] + P[3][20]*SF[6] + P[12][20]*SF[11] - P[10][20]*SPP[6] - (P[11][20]*q0)/2;
	nextP[2][21] = P[2][21] + P[0][21]*SF[4] + P[1][21]*SF[8] + P[3][21]*SF[6] + P[12][21]*SF[11] - P[10][21]*SPP[6] - (P[11][21]*q0)/2;
	nextP[3][0] = P[3][0] + SQ[6] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2 + SF[7]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[8]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + SPP[7]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[6]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2);
	nextP[3][1] = P[3][1] + SQ[4] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2 + SF[6]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[5]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[9]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[6]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) - SPP[7]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - (q0*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2))/2;
	nextP[3][2] = P[3][2] + SQ[3] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[8]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[6]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - SPP[6]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) - (q0*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2))/2;
	nextP[3][3] = P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] + (dayCov*SQ[10])/4 + dazCov*SQ[9] - (P[12][3]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[4]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[7]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SF[11]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[7]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + (daxCov*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2))/2;
	nextP[3][4] = P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2 + SF[3]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SPP[2]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[4]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][5] = P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2 + SF[2]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][6] = P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2 + SF[2]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[1]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) - SPP[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][7] = P[3][7] + P[0][7]*SF[5] + P[1][7]*SF[4] + P[2][7]*SF[7] - P[11][7]*SF[11] + P[10][7]*SPP[7] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2);
	nextP[3][8] = P[3][8] + P[0][8]*SF[5] + P[1][8]*SF[4] + P[2][8]*SF[7] - P[11][8]*SF[11] + P[10][8]*SPP[7] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2);
	nextP[3][9] = P[3][9] + P[0][9]*SF[5] + P[1][9]*SF[4] + P[2][9]*SF[7] - P[11][9]*SF[11] + P[10][9]*SPP[7] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2);
	nextP[3][10] = P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2;
	nextP[3][11] = P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2;
	nextP[3][12] = P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2;
	nextP[3][13] = P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2;
	nextP[3][14] = P[3][14] + P[0][14]*SF[5] + P[1][14]*SF[4] + P[2][14]*SF[7] - P[11][14]*SF[11] + P[10][14]*SPP[7] - (P[12][14]*q0)/2;
	nextP[3][15] = P[3][15] + P[0][15]*SF[5] + P[1][15]*SF[4] + P[2][15]*SF[7] - P[11][15]*SF[11] + P[10][15]*SPP[7] - (P[12][15]*q0)/2;
	nextP[3][16] = P[3][16] + P[0][16]*SF[5] + P[1][16]*SF[4] + P[2][16]*SF[7] - P[11][16]*SF[11] + P[10][16]*SPP[7] - (P[12][16]*q0)/2;
	nextP[3][17] = P[3][17] + P[0][17]*SF[5] + P[1][17]*SF[4] + P[2][17]*SF[7] - P[11][17]*SF[11] + P[10][17]*SPP[7] - (P[12][17]*q0)/2;
	nextP[3][18] = P[3][18] + P[0][18]*SF[5] + P[1][18]*SF[4] + P[2][18]*SF[7] - P[11][18]*SF[11] + P[10][18]*SPP[7] - (P[12][18]*q0)/2;
	nextP[3][19] = P[3][19] + P[0][19]*SF[5] + P[1][19]*SF[4] + P[2][19]*SF[7] - P[11][19]*SF[11] + P[10][19]*SPP[7] - (P[12][19]*q0)/2;
	nextP[3][20] = P[3][20] + P[0][20]*SF[5] + P[1][20]*SF[4] + P[2][20]*SF[7] - P[11][20]*SF[11] + P[10][20]*SPP[7] - (P[12][20]*q0)/2;
	nextP[3][21] = P[3][21] + P[0][21]*SF[5] + P[1][21]*SF[4] + P[2][21]*SF[7] - P[11][21]*SF[11] + P[10][21]*SPP[7] - (P[12][21]*q0)/2;
	nextP[4][0] = P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4] + SF[7]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[9]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[8]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) + SPP[7]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[6]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]);
	nextP[4][1] = P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4] + SF[6]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[5]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[9]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[6]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) - SPP[7]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - (q0*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]))/2;
	nextP[4][2] = P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4] + SF[4]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[8]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[6]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - SPP[6]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]))/2;
	nextP[4][3] = P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4] + SF[5]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[4]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[7]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SF[11]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[7]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]))/2;
	nextP[4][4] = P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4] + dvyCov*sq(SG[7] - 2*q0*q3) + dvzCov*sq(SG[6] + 2*q0*q2) + SF[3]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[0]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SPP[2]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[4]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]) + dvxCov*sq(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4] + SF[2]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[3]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[0]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[3]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
	nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4] + SF[2]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[1]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[0]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) - SPP[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
	nextP[4][7] = P[4][7] + P[0][7]*SF[3] + P[1][7]*SF[1] + P[2][7]*SPP[0] - P[3][7]*SPP[2] - P[13][7]*SPP[4] + dt*(P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4]);
	nextP[4][8] = P[4][8] + P[0][8]*SF[3] + P[1][8]*SF[1] + P[2][8]*SPP[0] - P[3][8]*SPP[2] - P[13][8]*SPP[4] + dt*(P[4][5] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4]);
	nextP[4][9] = P[4][9] + P[0][9]*SF[3] + P[1][9]*SF[1] + P[2][9]*SPP[0] - P[3][9]*SPP[2] - P[13][9]*SPP[4] + dt*(P[4][6] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4]);
	nextP[4][10] = P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4];
	nextP[4][11] = P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4];
	nextP[4][12] = P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4];
	nextP[4][13] = P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4];
	nextP[4][14] = P[4][14] + P[0][14]*SF[3] + P[1][14]*SF[1] + P[2][14]*SPP[0] - P[3][14]*SPP[2] - P[13][14]*SPP[4];
	nextP[4][15] = P[4][15] + P[0][15]*SF[3] + P[1][15]*SF[1] + P[2][15]*SPP[0] - P[3][15]*SPP[2] - P[13][15]*SPP[4];
	nextP[4][16] = P[4][16] + P[0][16]*SF[3] + P[1][16]*SF[1] + P[2][16]*SPP[0] - P[3][16]*SPP[2] - P[13][16]*SPP[4];
	nextP[4][17] = P[4][17] + P[0][17]*SF[3] + P[1][17]*SF[1] + P[2][17]*SPP[0] - P[3][17]*SPP[2] - P[13][17]*SPP[4];
	nextP[4][18] = P[4][18] + P[0][18]*SF[3] + P[1][18]*SF[1] + P[2][18]*SPP[0] - P[3][18]*SPP[2] - P[13][18]*SPP[4];
	nextP[4][19] = P[4][19] + P[0][19]*SF[3] + P[1][19]*SF[1] + P[2][19]*SPP[0] - P[3][19]*SPP[2] - P[13][19]*SPP[4];
	nextP[4][20] = P[4][20] + P[0][20]*SF[3] + P[1][20]*SF[1] + P[2][20]*SPP[0] - P[3][20]*SPP[2] - P[13][20]*SPP[4];
	nextP[4][21] = P[4][21] + P[0][21]*SF[3] + P[1][21]*SF[1] + P[2][21]*SPP[0] - P[3][21]*SPP[2] - P[13][21]*SPP[4];
	nextP[5][0] = P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3] + SF[7]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[9]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[8]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) + SPP[7]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[6]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]);
	nextP[5][1] = P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3] + SF[6]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[5]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[9]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[6]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) - SPP[7]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - (q0*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]))/2;
	nextP[5][2] = P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3] + SF[4]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[8]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[6]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - SPP[6]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]))/2;
	nextP[5][3] = P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3] + SF[5]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[4]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[7]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SF[11]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[7]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]))/2;
	nextP[5][4] = P[5][4] + SQ[2] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3] + SF[3]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[0]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SPP[2]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[4]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
	nextP[5][5] = P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3] + dvxCov*sq(SG[7] + 2*q0*q3) + dvzCov*sq(SG[5] - 2*q0*q1) + SF[2]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[3]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[0]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[3]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]) + dvyCov*sq(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3] + SF[2]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[1]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[0]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) - SPP[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
	nextP[5][7] = P[5][7] + P[0][7]*SF[2] + P[2][7]*SF[1] + P[3][7]*SF[3] - P[1][7]*SPP[0] + P[13][7]*SPP[3] + dt*(P[5][4] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3]);
	nextP[5][8] = P[5][8] + P[0][8]*SF[2] + P[2][8]*SF[1] + P[3][8]*SF[3] - P[1][8]*SPP[0] + P[13][8]*SPP[3] + dt*(P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3]);
	nextP[5][9] = P[5][9] + P[0][9]*SF[2] + P[2][9]*SF[1] + P[3][9]*SF[3] - P[1][9]*SPP[0] + P[13][9]*SPP[3] + dt*(P[5][6] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3]);
	nextP[5][10] = P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3];
	nextP[5][11] = P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3];
	nextP[5][12] = P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3];
	nextP[5][13] = P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3];
	nextP[5][14] = P[5][14] + P[0][14]*SF[2] + P[2][14]*SF[1] + P[3][14]*SF[3] - P[1][14]*SPP[0] + P[13][14]*SPP[3];
	nextP[5][15] = P[5][15] + P[0][15]*SF[2] + P[2][15]*SF[1] + P[3][15]*SF[3] - P[1][15]*SPP[0] + P[13][15]*SPP[3];
	nextP[5][16] = P[5][16] + P[0][16]*SF[2] + P[2][16]*SF[1] + P[3][16]*SF[3] - P[1][16]*SPP[0] + P[13][16]*SPP[3];
	nextP[5][17] = P[5][17] + P[0][17]*SF[2] + P[2][17]*SF[1] + P[3][17]*SF[3] - P[1][17]*SPP[0] + P[13][17]*SPP[3];
	nextP[5][18] = P[5][18] + P[0][18]*SF[2] + P[2][18]*SF[1] + P[3][18]*SF[3] - P[1][18]*SPP[0] + P[13][18]*SPP[3];
	nextP[5][19] = P[5][19] + P[0][19]*SF[2] + P[2][19]*SF[1] + P[3][19]*SF[3] - P[1][19]*SPP[0] + P[13][19]*SPP[3];
	nextP[5][20] = P[5][20] + P[0][20]*SF[2] + P[2][20]*SF[1] + P[3][20]*SF[3] - P[1][20]*SPP[0] + P[13][20]*SPP[3];
	nextP[5][21] = P[5][21] + P[0][21]*SF[2] + P[2][21]*SF[1] + P[3][21]*SF[3] - P[1][21]*SPP[0] + P[13][21]*SPP[3];
	nextP[6][0] = P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[7]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][1] = P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[6]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[5]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[7]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][2] = P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[4]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[6]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[6]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][3] = P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[5]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[4]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[7]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SF[11]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][4] = P[6][4] + SQ[1] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[3]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[2]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[4]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][5] = P[6][5] + SQ[0] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[2]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[3]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[0]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[3]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][6] = P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + dvxCov*sq(SG[6] - 2*q0*q2) + dvyCov*sq(SG[5] + 2*q0*q1) - SPP[5]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5]) + SF[2]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + dvzCov*sq(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[6][7] = P[6][7] + P[1][7]*SF[2] + P[3][7]*SF[1] + P[0][7]*SPP[0] - P[2][7]*SPP[1] - P[13][7]*SPP[5] + dt*(P[6][4] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*SPP[5]);
	nextP[6][8] = P[6][8] + P[1][8]*SF[2] + P[3][8]*SF[1] + P[0][8]*SPP[0] - P[2][8]*SPP[1] - P[13][8]*SPP[5] + dt*(P[6][5] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*SPP[5]);
	nextP[6][9] = P[6][9] + P[1][9]*SF[2] + P[3][9]*SF[1] + P[0][9]*SPP[0] - P[2][9]*SPP[1] - P[13][9]*SPP[5] + dt*(P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*SPP[5]);
	nextP[6][10] = P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*SPP[5];
	nextP[6][11] = P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*SPP[5];
	nextP[6][12] = P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*SPP[5];
	nextP[6][13] = P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5];
	nextP[6][14] = P[6][14] + P[1][14]*SF[2] + P[3][14]*SF[1] + P[0][14]*SPP[0] - P[2][14]*SPP[1] - P[13][14]*SPP[5];
	nextP[6][15] = P[6][15] + P[1][15]*SF[2] + P[3][15]*SF[1] + P[0][15]*SPP[0] - P[2][15]*SPP[1] - P[13][15]*SPP[5];
	nextP[6][16] = P[6][16] + P[1][16]*SF[2] + P[3][16]*SF[1] + P[0][16]*SPP[0] - P[2][16]*SPP[1] - P[13][16]*SPP[5];
	nextP[6][17] = P[6][17] + P[1][17]*SF[2] + P[3][17]*SF[1] + P[0][17]*SPP[0] - P[2][17]*SPP[1] - P[13][17]*SPP[5];
	nextP[6][18] = P[6][18] + P[1][18]*SF[2] + P[3][18]*SF[1] + P[0][18]*SPP[0] - P[2][18]*SPP[1] - P[13][18]*SPP[5];
	nextP[6][19] = P[6][19] + P[1][19]*SF[2] + P[3][19]*SF[1] + P[0][19]*SPP[0] - P[2][19]*SPP[1] - P[13][19]*SPP[5];
	nextP[6][20] = P[6][20] + P[1][20]*SF[2] + P[3][20]*SF[1] + P[0][20]*SPP[0] - P[2][20]*SPP[1] - P[13][20]*SPP[5];
	nextP[6][21] = P[6][21] + P[1][21]*SF[2] + P[3][21]*SF[1] + P[0][21]*SPP[0] - P[2][21]*SPP[1] - P[13][21]*SPP[5];
	nextP[7][0] = P[7][0] + P[4][0]*dt + SF[7]*(P[7][1] + P[4][1]*dt) + SF[9]*(P[7][2] + P[4][2]*dt) + SF[8]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][10] + P[4][10]*dt) + SPP[7]*(P[7][11] + P[4][11]*dt) + SPP[6]*(P[7][12] + P[4][12]*dt);
	nextP[7][1] = P[7][1] + P[4][1]*dt + SF[6]*(P[7][0] + P[4][0]*dt) + SF[5]*(P[7][2] + P[4][2]*dt) + SF[9]*(P[7][3] + P[4][3]*dt) + SPP[6]*(P[7][11] + P[4][11]*dt) - SPP[7]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][10] + P[4][10]*dt))/2;
	nextP[7][2] = P[7][2] + P[4][2]*dt + SF[4]*(P[7][0] + P[4][0]*dt) + SF[8]*(P[7][1] + P[4][1]*dt) + SF[6]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][12] + P[4][12]*dt) - SPP[6]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][11] + P[4][11]*dt))/2;
	nextP[7][3] = P[7][3] + P[4][3]*dt + SF[5]*(P[7][0] + P[4][0]*dt) + SF[4]*(P[7][1] + P[4][1]*dt) + SF[7]*(P[7][2] + P[4][2]*dt) - SF[11]*(P[7][11] + P[4][11]*dt) + SPP[7]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][12] + P[4][12]*dt))/2;
	nextP[7][4] = P[7][4] + P[4][4]*dt + SF[1]*(P[7][1] + P[4][1]*dt) + SF[3]*(P[7][0] + P[4][0]*dt) + SPP[0]*(P[7][2] + P[4][2]*dt) - SPP[2]*(P[7][3] + P[4][3]*dt) - SPP[4]*(P[7][13] + P[4][13]*dt);
	nextP[7][5] = P[7][5] + P[4][5]*dt + SF[2]*(P[7][0] + P[4][0]*dt) + SF[1]*(P[7][2] + P[4][2]*dt) + SF[3]*(P[7][3] + P[4][3]*dt) - SPP[0]*(P[7][1] + P[4][1]*dt) + SPP[3]*(P[7][13] + P[4][13]*dt);
	nextP[7][6] = P[7][6] + P[4][6]*dt + SF[2]*(P[7][1] + P[4][1]*dt) + SF[1]*(P[7][3] + P[4][3]*dt) + SPP[0]*(P[7][0] + P[4][0]*dt) - SPP[1]*(P[7][2] + P[4][2]*dt) - SPP[5]*(P[7][13] + P[4][13]*dt);
	nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
	nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
	nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
	nextP[7][10] = P[7][10] + P[4][10]*dt;
	nextP[7][11] = P[7][11] + P[4][11]*dt;
	nextP[7][12] = P[7][12] + P[4][12]*dt;
	nextP[7][13] = P[7][13] + P[4][13]*dt;
	nextP[7][14] = P[7][14] + P[4][14]*dt;
	nextP[7][15] = P[7][15] + P[4][15]*dt;
	nextP[7][16] = P[7][16] + P[4][16]*dt;
	nextP[7][17] = P[7][17] + P[4][17]*dt;
	nextP[7][18] = P[7][18] + P[4][18]*dt;
	nextP[7][19] = P[7][19] + P[4][19]*dt;
	nextP[7][20] = P[7][20] + P[4][20]*dt;
	nextP[7][21] = P[7][21] + P[4][21]*dt;
	nextP[8][0] = P[8][0] + P[5][0]*dt + SF[7]*(P[8][1] + P[5][1]*dt) + SF[9]*(P[8][2] + P[5][2]*dt) + SF[8]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][10] + P[5][10]*dt) + SPP[7]*(P[8][11] + P[5][11]*dt) + SPP[6]*(P[8][12] + P[5][12]*dt);
	nextP[8][1] = P[8][1] + P[5][1]*dt + SF[6]*(P[8][0] + P[5][0]*dt) + SF[5]*(P[8][2] + P[5][2]*dt) + SF[9]*(P[8][3] + P[5][3]*dt) + SPP[6]*(P[8][11] + P[5][11]*dt) - SPP[7]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][10] + P[5][10]*dt))/2;
	nextP[8][2] = P[8][2] + P[5][2]*dt + SF[4]*(P[8][0] + P[5][0]*dt) + SF[8]*(P[8][1] + P[5][1]*dt) + SF[6]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][12] + P[5][12]*dt) - SPP[6]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][11] + P[5][11]*dt))/2;
	nextP[8][3] = P[8][3] + P[5][3]*dt + SF[5]*(P[8][0] + P[5][0]*dt) + SF[4]*(P[8][1] + P[5][1]*dt) + SF[7]*(P[8][2] + P[5][2]*dt) - SF[11]*(P[8][11] + P[5][11]*dt) + SPP[7]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][12] + P[5][12]*dt))/2;
	nextP[8][4] = P[8][4] + P[5][4]*dt + SF[1]*(P[8][1] + P[5][1]*dt) + SF[3]*(P[8][0] + P[5][0]*dt) + SPP[0]*(P[8][2] + P[5][2]*dt) - SPP[2]*(P[8][3] + P[5][3]*dt) - SPP[4]*(P[8][13] + P[5][13]*dt);
	nextP[8][5] = P[8][5] + P[5][5]*dt + SF[2]*(P[8][0] + P[5][0]*dt) + SF[1]*(P[8][2] + P[5][2]*dt) + SF[3]*(P[8][3] + P[5][3]*dt) - SPP[0]*(P[8][1] + P[5][1]*dt) + SPP[3]*(P[8][13] + P[5][13]*dt);
	nextP[8][6] = P[8][6] + P[5][6]*dt + SF[2]*(P[8][1] + P[5][1]*dt) + SF[1]*(P[8][3] + P[5][3]*dt) + SPP[0]*(P[8][0] + P[5][0]*dt) - SPP[1]*(P[8][2] + P[5][2]*dt) - SPP[5]*(P[8][13] + P[5][13]*dt);
	nextP[8][7] = P[8][7] + P[5][7]*dt + dt*(P[8][4] + P[5][4]*dt);
	nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
	nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
	nextP[8][10] = P[8][10] + P[5][10]*dt;
	nextP[8][11] = P[8][11] + P[5][11]*dt;
	nextP[8][12] = P[8][12] + P[5][12]*dt;
	nextP[8][13] = P[8][13] + P[5][13]*dt;
	nextP[8][14] = P[8][14] + P[5][14]*dt;
	nextP[8][15] = P[8][15] + P[5][15]*dt;
	nextP[8][16] = P[8][16] + P[5][16]*dt;
	nextP[8][17] = P[8][17] + P[5][17]*dt;
	nextP[8][18] = P[8][18] + P[5][18]*dt;
	nextP[8][19] = P[8][19] + P[5][19]*dt;
	nextP[8][20] = P[8][20] + P[5][20]*dt;
	nextP[8][21] = P[8][21] + P[5][21]*dt;
	nextP[9][0] = P[9][0] + P[6][0]*dt + SF[7]*(P[9][1] + P[6][1]*dt) + SF[9]*(P[9][2] + P[6][2]*dt) + SF[8]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][10] + P[6][10]*dt) + SPP[7]*(P[9][11] + P[6][11]*dt) + SPP[6]*(P[9][12] + P[6][12]*dt);
	nextP[9][1] = P[9][1] + P[6][1]*dt + SF[6]*(P[9][0] + P[6][0]*dt) + SF[5]*(P[9][2] + P[6][2]*dt) + SF[9]*(P[9][3] + P[6][3]*dt) + SPP[6]*(P[9][11] + P[6][11]*dt) - SPP[7]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][10] + P[6][10]*dt))/2;
	nextP[9][2] = P[9][2] + P[6][2]*dt + SF[4]*(P[9][0] + P[6][0]*dt) + SF[8]*(P[9][1] + P[6][1]*dt) + SF[6]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][12] + P[6][12]*dt) - SPP[6]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][11] + P[6][11]*dt))/2;
	nextP[9][3] = P[9][3] + P[6][3]*dt + SF[5]*(P[9][0] + P[6][0]*dt) + SF[4]*(P[9][1] + P[6][1]*dt) + SF[7]*(P[9][2] + P[6][2]*dt) - SF[11]*(P[9][11] + P[6][11]*dt) + SPP[7]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][12] + P[6][12]*dt))/2;
	nextP[9][4] = P[9][4] + P[6][4]*dt + SF[1]*(P[9][1] + P[6][1]*dt) + SF[3]*(P[9][0] + P[6][0]*dt) + SPP[0]*(P[9][2] + P[6][2]*dt) - SPP[2]*(P[9][3] + P[6][3]*dt) - SPP[4]*(P[9][13] + P[6][13]*dt);
	nextP[9][5] = P[9][5] + P[6][5]*dt + SF[2]*(P[9][0] + P[6][0]*dt) + SF[1]*(P[9][2] + P[6][2]*dt) + SF[3]*(P[9][3] + P[6][3]*dt) - SPP[0]*(P[9][1] + P[6][1]*dt) + SPP[3]*(P[9][13] + P[6][13]*dt);
	nextP[9][6] = P[9][6] + P[6][6]*dt + SF[2]*(P[9][1] + P[6][1]*dt) + SF[1]*(P[9][3] + P[6][3]*dt) + SPP[0]*(P[9][0] + P[6][0]*dt) - SPP[1]*(P[9][2] + P[6][2]*dt) - SPP[5]*(P[9][13] + P[6][13]*dt);
	nextP[9][7] = P[9][7] + P[6][7]*dt + dt*(P[9][4] + P[6][4]*dt);
	nextP[9][8] = P[9][8] + P[6][8]*dt + dt*(P[9][5] + P[6][5]*dt);
	nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
	nextP[9][10] = P[9][10] + P[6][10]*dt;
	nextP[9][11] = P[9][11] + P[6][11]*dt;
	nextP[9][12] = P[9][12] + P[6][12]*dt;
	nextP[9][13] = P[9][13] + P[6][13]*dt;
	nextP[9][14] = P[9][14] + P[6][14]*dt;
	nextP[9][15] = P[9][15] + P[6][15]*dt;
    nextP[9][16] = P[9][16] + P[6][16]*dt;
    nextP[9][17] = P[9][17] + P[6][17]*dt;
	nextP[9][18] = P[9][18] + P[6][18]*dt;
	nextP[9][19] = P[9][19] + P[6][19]*dt;
	nextP[9][20] = P[9][20] + P[6][20]*dt;
	nextP[9][21] = P[9][21] + P[6][21]*dt;
	nextP[10][0] = P[10][0] + P[10][1]*SF[7] + P[10][2]*SF[9] + P[10][3]*SF[8] + P[10][10]*SF[11] + P[10][11]*SPP[7] + P[10][12]*SPP[6];
	nextP[10][1] = P[10][1] + P[10][0]*SF[6] + P[10][2]*SF[5] + P[10][3]*SF[9] + P[10][11]*SPP[6] - P[10][12]*SPP[7] - (P[10][10]*q0)/2;
	nextP[10][2] = P[10][2] + P[10][0]*SF[4] + P[10][1]*SF[8] + P[10][3]*SF[6] + P[10][12]*SF[11] - P[10][10]*SPP[6] - (P[10][11]*q0)/2;
	nextP[10][3] = P[10][3] + P[10][0]*SF[5] + P[10][1]*SF[4] + P[10][2]*SF[7] - P[10][11]*SF[11] + P[10][10]*SPP[7] - (P[10][12]*q0)/2;
	nextP[10][4] = P[10][4] + P[10][1]*SF[1] + P[10][0]*SF[3] + P[10][2]*SPP[0] - P[10][3]*SPP[2] - P[10][13]*SPP[4];
	nextP[10][5] = P[10][5] + P[10][0]*SF[2] + P[10][2]*SF[1] + P[10][3]*SF[3] - P[10][1]*SPP[0] + P[10][13]*SPP[3];
	nextP[10][6] = P[10][6] + P[10][1]*SF[2] + P[10][3]*SF[1] + P[10][0]*SPP[0] - P[10][2]*SPP[1] - P[10][13]*SPP[5];
	nextP[10][7] = P[10][7] + P[10][4]*dt;
	nextP[10][8] = P[10][8] + P[10][5]*dt;
	nextP[10][9] = P[10][9] + P[10][6]*dt;
	nextP[10][10] = P[10][10];
	nextP[10][11] = P[10][11];
	nextP[10][12] = P[10][12];
	nextP[10][13] = P[10][13];
	nextP[10][14] = P[10][14];
	nextP[10][15] = P[10][15];
	nextP[10][16] = P[10][16];
	nextP[10][17] = P[10][17];
	nextP[10][18] = P[10][18];
	nextP[10][19] = P[10][19];
	nextP[10][20] = P[10][20];
	nextP[10][21] = P[10][21];
	nextP[11][0] = P[11][0] + P[11][1]*SF[7] + P[11][2]*SF[9] + P[11][3]*SF[8] + P[11][10]*SF[11] + P[11][11]*SPP[7] + P[11][12]*SPP[6];
	nextP[11][1] = P[11][1] + P[11][0]*SF[6] + P[11][2]*SF[5] + P[11][3]*SF[9] + P[11][11]*SPP[6] - P[11][12]*SPP[7] - (P[11][10]*q0)/2;
	nextP[11][2] = P[11][2] + P[11][0]*SF[4] + P[11][1]*SF[8] + P[11][3]*SF[6] + P[11][12]*SF[11] - P[11][10]*SPP[6] - (P[11][11]*q0)/2;
	nextP[11][3] = P[11][3] + P[11][0]*SF[5] + P[11][1]*SF[4] + P[11][2]*SF[7] - P[11][11]*SF[11] + P[11][10]*SPP[7] - (P[11][12]*q0)/2;
	nextP[11][4] = P[11][4] + P[11][1]*SF[1] + P[11][0]*SF[3] + P[11][2]*SPP[0] - P[11][3]*SPP[2] - P[11][13]*SPP[4];
	nextP[11][5] = P[11][5] + P[11][0]*SF[2] + P[11][2]*SF[1] + P[11][3]*SF[3] - P[11][1]*SPP[0] + P[11][13]*SPP[3];
	nextP[11][6] = P[11][6] + P[11][1]*SF[2] + P[11][3]*SF[1] + P[11][0]*SPP[0] - P[11][2]*SPP[1] - P[11][13]*SPP[5];
	nextP[11][7] = P[11][7] + P[11][4]*dt;
	nextP[11][8] = P[11][8] + P[11][5]*dt;
	nextP[11][9] = P[11][9] + P[11][6]*dt;
	nextP[11][10] = P[11][10];
	nextP[11][11] = P[11][11];
	nextP[11][12] = P[11][12];
	nextP[11][13] = P[11][13];
	nextP[11][14] = P[11][14];
	nextP[11][15] = P[11][15];
	nextP[11][16] = P[11][16];
	nextP[11][17] = P[11][17];
	nextP[11][18] = P[11][18];
	nextP[11][19] = P[11][19];
	nextP[11][20] = P[11][20];
	nextP[11][21] = P[11][21];
	nextP[12][0] = P[12][0] + P[12][1]*SF[7] + P[12][2]*SF[9] + P[12][3]*SF[8] + P[12][10]*SF[11] + P[12][11]*SPP[7] + P[12][12]*SPP[6];
	nextP[12][1] = P[12][1] + P[12][0]*SF[6] + P[12][2]*SF[5] + P[12][3]*SF[9] + P[12][11]*SPP[6] - P[12][12]*SPP[7] - (P[12][10]*q0)/2;
	nextP[12][2] = P[12][2] + P[12][0]*SF[4] + P[12][1]*SF[8] + P[12][3]*SF[6] + P[12][12]*SF[11] - P[12][10]*SPP[6] - (P[12][11]*q0)/2;
	nextP[12][3] = P[12][3] + P[12][0]*SF[5] + P[12][1]*SF[4] + P[12][2]*SF[7] - P[12][11]*SF[11] + P[12][10]*SPP[7] - (P[12][12]*q0)/2;
	nextP[12][4] = P[12][4] + P[12][1]*SF[1] + P[12][0]*SF[3] + P[12][2]*SPP[0] - P[12][3]*SPP[2] - P[12][13]*SPP[4];
	nextP[12][5] = P[12][5] + P[12][0]*SF[2] + P[12][2]*SF[1] + P[12][3]*SF[3] - P[12][1]*SPP[0] + P[12][13]*SPP[3];
	nextP[12][6] = P[12][6] + P[12][1]*SF[2] + P[12][3]*SF[1] + P[12][0]*SPP[0] - P[12][2]*SPP[1] - P[12][13]*SPP[5];
	nextP[12][7] = P[12][7] + P[12][4]*dt;
	nextP[12][8] = P[12][8] + P[12][5]*dt;
	nextP[12][9] = P[12][9] + P[12][6]*dt;
	nextP[12][10] = P[12][10];
	nextP[12][11] = P[12][11];
	nextP[12][12] = P[12][12];
	nextP[12][13] = P[12][13];
	nextP[12][14] = P[12][14];
	nextP[12][15] = P[12][15];
	nextP[12][16] = P[12][16];
	nextP[12][17] = P[12][17];
	nextP[12][18] = P[12][18];
	nextP[12][19] = P[12][19];
	nextP[12][20] = P[12][20];
	nextP[12][21] = P[12][21];
	nextP[13][0] = P[13][0] + P[13][1]*SF[7] + P[13][2]*SF[9] + P[13][3]*SF[8] + P[13][10]*SF[11] + P[13][11]*SPP[7] + P[13][12]*SPP[6];
	nextP[13][1] = P[13][1] + P[13][0]*SF[6] + P[13][2]*SF[5] + P[13][3]*SF[9] + P[13][11]*SPP[6] - P[13][12]*SPP[7] - (P[13][10]*q0)/2;
	nextP[13][2] = P[13][2] + P[13][0]*SF[4] + P[13][1]*SF[8] + P[13][3]*SF[6] + P[13][12]*SF[11] - P[13][10]*SPP[6] - (P[13][11]*q0)/2;
	nextP[13][3] = P[13][3] + P[13][0]*SF[5] + P[13][1]*SF[4] + P[13][2]*SF[7] - P[13][11]*SF[11] + P[13][10]*SPP[7] - (P[13][12]*q0)/2;
	nextP[13][4] = P[13][4] + P[13][1]*SF[1] + P[13][0]*SF[3] + P[13][2]*SPP[0] - P[13][3]*SPP[2] - P[13][13]*SPP[4];
	nextP[13][5] = P[13][5] + P[13][0]*SF[2] + P[13][2]*SF[1] + P[13][3]*SF[3] - P[13][1]*SPP[0] + P[13][13]*SPP[3];
	nextP[13][6] = P[13][6] + P[13][1]*SF[2] + P[13][3]*SF[1] + P[13][0]*SPP[0] - P[13][2]*SPP[1] - P[13][13]*SPP[5];
	nextP[13][7] = P[13][7] + P[13][4]*dt;
	nextP[13][8] = P[13][8] + P[13][5]*dt;
	nextP[13][9] = P[13][9] + P[13][6]*dt;
	nextP[13][10] = P[13][10];
	nextP[13][11] = P[13][11];
	nextP[13][12] = P[13][12];
	nextP[13][13] = P[13][13];
	nextP[13][14] = P[13][14];
	nextP[13][15] = P[13][15];
	nextP[13][16] = P[13][16];
	nextP[13][17] = P[13][17];
	nextP[13][18] = P[13][18];
	nextP[13][19] = P[13][19];
	nextP[13][20] = P[13][20];
	nextP[13][21] = P[13][21];
	nextP[14][0] = P[14][0] + P[14][1]*SF[7] + P[14][2]*SF[9] + P[14][3]*SF[8] + P[14][10]*SF[11] + P[14][11]*SPP[7] + P[14][12]*SPP[6];
	nextP[14][1] = P[14][1] + P[14][0]*SF[6] + P[14][2]*SF[5] + P[14][3]*SF[9] + P[14][11]*SPP[6] - P[14][12]*SPP[7] - (P[14][10]*q0)/2;
	nextP[14][2] = P[14][2] + P[14][0]*SF[4] + P[14][1]*SF[8] + P[14][3]*SF[6] + P[14][12]*SF[11] - P[14][10]*SPP[6] - (P[14][11]*q0)/2;
	nextP[14][3] = P[14][3] + P[14][0]*SF[5] + P[14][1]*SF[4] + P[14][2]*SF[7] - P[14][11]*SF[11] + P[14][10]*SPP[7] - (P[14][12]*q0)/2;
	nextP[14][4] = P[14][4] + P[14][1]*SF[1] + P[14][0]*SF[3] + P[14][2]*SPP[0] - P[14][3]*SPP[2] - P[14][13]*SPP[4];
	nextP[14][5] = P[14][5] + P[14][0]*SF[2] + P[14][2]*SF[1] + P[14][3]*SF[3] - P[14][1]*SPP[0] + P[14][13]*SPP[3];
	nextP[14][6] = P[14][6] + P[14][1]*SF[2] + P[14][3]*SF[1] + P[14][0]*SPP[0] - P[14][2]*SPP[1] - P[14][13]*SPP[5];
	nextP[14][7] = P[14][7] + P[14][4]*dt;
	nextP[14][8] = P[14][8] + P[14][5]*dt;
	nextP[14][9] = P[14][9] + P[14][6]*dt;
	nextP[14][10] = P[14][10];
	nextP[14][11] = P[14][11];
	nextP[14][12] = P[14][12];
	nextP[14][13] = P[14][13];
	nextP[14][14] = P[14][14];
	nextP[14][15] = P[14][15];
	nextP[14][16] = P[14][16];
	nextP[14][17] = P[14][17];
	nextP[14][18] = P[14][18];
	nextP[14][19] = P[14][19];
	nextP[14][20] = P[14][20];
	nextP[14][21] = P[14][21];
	nextP[15][0] = P[15][0] + P[15][1]*SF[7] + P[15][2]*SF[9] + P[15][3]*SF[8] + P[15][10]*SF[11] + P[15][11]*SPP[7] + P[15][12]*SPP[6];
	nextP[15][1] = P[15][1] + P[15][0]*SF[6] + P[15][2]*SF[5] + P[15][3]*SF[9] + P[15][11]*SPP[6] - P[15][12]*SPP[7] - (P[15][10]*q0)/2;
	nextP[15][2] = P[15][2] + P[15][0]*SF[4] + P[15][1]*SF[8] + P[15][3]*SF[6] + P[15][12]*SF[11] - P[15][10]*SPP[6] - (P[15][11]*q0)/2;
	nextP[15][3] = P[15][3] + P[15][0]*SF[5] + P[15][1]*SF[4] + P[15][2]*SF[7] - P[15][11]*SF[11] + P[15][10]*SPP[7] - (P[15][12]*q0)/2;
	nextP[15][4] = P[15][4] + P[15][1]*SF[1] + P[15][0]*SF[3] + P[15][2]*SPP[0] - P[15][3]*SPP[2] - P[15][13]*SPP[4];
	nextP[15][5] = P[15][5] + P[15][0]*SF[2] + P[15][2]*SF[1] + P[15][3]*SF[3] - P[15][1]*SPP[0] + P[15][13]*SPP[3];
	nextP[15][6] = P[15][6] + P[15][1]*SF[2] + P[15][3]*SF[1] + P[15][0]*SPP[0] - P[15][2]*SPP[1] - P[15][13]*SPP[5];
	nextP[15][7] = P[15][7] + P[15][4]*dt;
	nextP[15][8] = P[15][8] + P[15][5]*dt;
	nextP[15][9] = P[15][9] + P[15][6]*dt;
	nextP[15][10] = P[15][10];
	nextP[15][11] = P[15][11];
	nextP[15][12] = P[15][12];
	nextP[15][13] = P[15][13];
	nextP[15][14] = P[15][14];
	nextP[15][15] = P[15][15];
	nextP[15][16] = P[15][16];
	nextP[15][17] = P[15][17];
	nextP[15][18] = P[15][18];
	nextP[15][19] = P[15][19];
	nextP[15][20] = P[15][20];
	nextP[15][21] = P[15][21];
	nextP[16][0] = P[16][0] + P[16][1]*SF[7] + P[16][2]*SF[9] + P[16][3]*SF[8] + P[16][10]*SF[11] + P[16][11]*SPP[7] + P[16][12]*SPP[6];
	nextP[16][1] = P[16][1] + P[16][0]*SF[6] + P[16][2]*SF[5] + P[16][3]*SF[9] + P[16][11]*SPP[6] - P[16][12]*SPP[7] - (P[16][10]*q0)/2;
	nextP[16][2] = P[16][2] + P[16][0]*SF[4] + P[16][1]*SF[8] + P[16][3]*SF[6] + P[16][12]*SF[11] - P[16][10]*SPP[6] - (P[16][11]*q0)/2;
	nextP[16][3] = P[16][3] + P[16][0]*SF[5] + P[16][1]*SF[4] + P[16][2]*SF[7] - P[16][11]*SF[11] + P[16][10]*SPP[7] - (P[16][12]*q0)/2;
	nextP[16][4] = P[16][4] + P[16][1]*SF[1] + P[16][0]*SF[3] + P[16][2]*SPP[0] - P[16][3]*SPP[2] - P[16][13]*SPP[4];
	nextP[16][5] = P[16][5] + P[16][0]*SF[2] + P[16][2]*SF[1] + P[16][3]*SF[3] - P[16][1]*SPP[0] + P[16][13]*SPP[3];
	nextP[16][6] = P[16][6] + P[16][1]*SF[2] + P[16][3]*SF[1] + P[16][0]*SPP[0] - P[16][2]*SPP[1] - P[16][13]*SPP[5];
	nextP[16][7] = P[16][7] + P[16][4]*dt;
	nextP[16][8] = P[16][8] + P[16][5]*dt;
	nextP[16][9] = P[16][9] + P[16][6]*dt;
	nextP[16][10] = P[16][10];
	nextP[16][11] = P[16][11];
	nextP[16][12] = P[16][12];
	nextP[16][13] = P[16][13];
	nextP[16][14] = P[16][14];
	nextP[16][15] = P[16][15];
	nextP[16][16] = P[16][16];
	nextP[16][17] = P[16][17];
	nextP[16][18] = P[16][18];
	nextP[16][19] = P[16][19];
	nextP[16][20] = P[16][20];
	nextP[16][21] = P[16][21];
	nextP[17][0] = P[17][0] + P[17][1]*SF[7] + P[17][2]*SF[9] + P[17][3]*SF[8] + P[17][10]*SF[11] + P[17][11]*SPP[7] + P[17][12]*SPP[6];
	nextP[17][1] = P[17][1] + P[17][0]*SF[6] + P[17][2]*SF[5] + P[17][3]*SF[9] + P[17][11]*SPP[6] - P[17][12]*SPP[7] - (P[17][10]*q0)/2;
	nextP[17][2] = P[17][2] + P[17][0]*SF[4] + P[17][1]*SF[8] + P[17][3]*SF[6] + P[17][12]*SF[11] - P[17][10]*SPP[6] - (P[17][11]*q0)/2;
	nextP[17][3] = P[17][3] + P[17][0]*SF[5] + P[17][1]*SF[4] + P[17][2]*SF[7] - P[17][11]*SF[11] + P[17][10]*SPP[7] - (P[17][12]*q0)/2;
	nextP[17][4] = P[17][4] + P[17][1]*SF[1] + P[17][0]*SF[3] + P[17][2]*SPP[0] - P[17][3]*SPP[2] - P[17][13]*SPP[4];
	nextP[17][5] = P[17][5] + P[17][0]*SF[2] + P[17][2]*SF[1] + P[17][3]*SF[3] - P[17][1]*SPP[0] + P[17][13]*SPP[3];
	nextP[17][6] = P[17][6] + P[17][1]*SF[2] + P[17][3]*SF[1] + P[17][0]*SPP[0] - P[17][2]*SPP[1] - P[17][13]*SPP[5];
	nextP[17][7] = P[17][7] + P[17][4]*dt;
	nextP[17][8] = P[17][8] + P[17][5]*dt;
	nextP[17][9] = P[17][9] + P[17][6]*dt;
	nextP[17][10] = P[17][10];
	nextP[17][11] = P[17][11];
	nextP[17][12] = P[17][12];
	nextP[17][13] = P[17][13];
	nextP[17][14] = P[17][14];
	nextP[17][15] = P[17][15];
	nextP[17][16] = P[17][16];
	nextP[17][17] = P[17][17];
	nextP[17][18] = P[17][18];
	nextP[17][19] = P[17][19];
	nextP[17][20] = P[17][20];
	nextP[17][21] = P[17][21];
	nextP[18][0] = P[18][0] + P[18][1]*SF[7] + P[18][2]*SF[9] + P[18][3]*SF[8] + P[18][10]*SF[11] + P[18][11]*SPP[7] + P[18][12]*SPP[6];
	nextP[18][1] = P[18][1] + P[18][0]*SF[6] + P[18][2]*SF[5] + P[18][3]*SF[9] + P[18][11]*SPP[6] - P[18][12]*SPP[7] - (P[18][10]*q0)/2;
	nextP[18][2] = P[18][2] + P[18][0]*SF[4] + P[18][1]*SF[8] + P[18][3]*SF[6] + P[18][12]*SF[11] - P[18][10]*SPP[6] - (P[18][11]*q0)/2;
	nextP[18][3] = P[18][3] + P[18][0]*SF[5] + P[18][1]*SF[4] + P[18][2]*SF[7] - P[18][11]*SF[11] + P[18][10]*SPP[7] - (P[18][12]*q0)/2;
	nextP[18][4] = P[18][4] + P[18][1]*SF[1] + P[18][0]*SF[3] + P[18][2]*SPP[0] - P[18][3]*SPP[2] - P[18][13]*SPP[4];
	nextP[18][5] = P[18][5] + P[18][0]*SF[2] + P[18][2]*SF[1] + P[18][3]*SF[3] - P[18][1]*SPP[0] + P[18][13]*SPP[3];
	nextP[18][6] = P[18][6] + P[18][1]*SF[2] + P[18][3]*SF[1] + P[18][0]*SPP[0] - P[18][2]*SPP[1] - P[18][13]*SPP[5];
	nextP[18][7] = P[18][7] + P[18][4]*dt;
	nextP[18][8] = P[18][8] + P[18][5]*dt;
	nextP[18][9] = P[18][9] + P[18][6]*dt;
	nextP[18][10] = P[18][10];
	nextP[18][11] = P[18][11];
	nextP[18][12] = P[18][12];
	nextP[18][13] = P[18][13];
	nextP[18][14] = P[18][14];
	nextP[18][15] = P[18][15];
	nextP[18][16] = P[18][16];
	nextP[18][17] = P[18][17];
	nextP[18][18] = P[18][18];
	nextP[18][19] = P[18][19];
	nextP[18][20] = P[18][20];
	nextP[18][21] = P[18][21];
	nextP[19][0] = P[19][0] + P[19][1]*SF[7] + P[19][2]*SF[9] + P[19][3]*SF[8] + P[19][10]*SF[11] + P[19][11]*SPP[7] + P[19][12]*SPP[6];
	nextP[19][1] = P[19][1] + P[19][0]*SF[6] + P[19][2]*SF[5] + P[19][3]*SF[9] + P[19][11]*SPP[6] - P[19][12]*SPP[7] - (P[19][10]*q0)/2;
	nextP[19][2] = P[19][2] + P[19][0]*SF[4] + P[19][1]*SF[8] + P[19][3]*SF[6] + P[19][12]*SF[11] - P[19][10]*SPP[6] - (P[19][11]*q0)/2;
	nextP[19][3] = P[19][3] + P[19][0]*SF[5] + P[19][1]*SF[4] + P[19][2]*SF[7] - P[19][11]*SF[11] + P[19][10]*SPP[7] - (P[19][12]*q0)/2;
	nextP[19][4] = P[19][4] + P[19][1]*SF[1] + P[19][0]*SF[3] + P[19][2]*SPP[0] - P[19][3]*SPP[2] - P[19][13]*SPP[4];
	nextP[19][5] = P[19][5] + P[19][0]*SF[2] + P[19][2]*SF[1] + P[19][3]*SF[3] - P[19][1]*SPP[0] + P[19][13]*SPP[3];
	nextP[19][6] = P[19][6] + P[19][1]*SF[2] + P[19][3]*SF[1] + P[19][0]*SPP[0] - P[19][2]*SPP[1] - P[19][13]*SPP[5];
	nextP[19][7] = P[19][7] + P[19][4]*dt;
	nextP[19][8] = P[19][8] + P[19][5]*dt;
	nextP[19][9] = P[19][9] + P[19][6]*dt;
	nextP[19][10] = P[19][10];
	nextP[19][11] = P[19][11];
	nextP[19][12] = P[19][12];
	nextP[19][13] = P[19][13];
	nextP[19][14] = P[19][14];
	nextP[19][15] = P[19][15];
	nextP[19][16] = P[19][16];
	nextP[19][17] = P[19][17];
	nextP[19][18] = P[19][18];
	nextP[19][19] = P[19][19];
	nextP[19][20] = P[19][20];
	nextP[19][21] = P[19][21];
	nextP[20][0] = P[20][0] + P[20][1]*SF[7] + P[20][2]*SF[9] + P[20][3]*SF[8] + P[20][10]*SF[11] + P[20][11]*SPP[7] + P[20][12]*SPP[6];
	nextP[20][1] = P[20][1] + P[20][0]*SF[6] + P[20][2]*SF[5] + P[20][3]*SF[9] + P[20][11]*SPP[6] - P[20][12]*SPP[7] - (P[20][10]*q0)/2;
	nextP[20][2] = P[20][2] + P[20][0]*SF[4] + P[20][1]*SF[8] + P[20][3]*SF[6] + P[20][12]*SF[11] - P[20][10]*SPP[6] - (P[20][11]*q0)/2;
	nextP[20][3] = P[20][3] + P[20][0]*SF[5] + P[20][1]*SF[4] + P[20][2]*SF[7] - P[20][11]*SF[11] + P[20][10]*SPP[7] - (P[20][12]*q0)/2;
	nextP[20][4] = P[20][4] + P[20][1]*SF[1] + P[20][0]*SF[3] + P[20][2]*SPP[0] - P[20][3]*SPP[2] - P[20][13]*SPP[4];
	nextP[20][5] = P[20][5] + P[20][0]*SF[2] + P[20][2]*SF[1] + P[20][3]*SF[3] - P[20][1]*SPP[0] + P[20][13]*SPP[3];
	nextP[20][6] = P[20][6] + P[20][1]*SF[2] + P[20][3]*SF[1] + P[20][0]*SPP[0] - P[20][2]*SPP[1] - P[20][13]*SPP[5];
	nextP[20][7] = P[20][7] + P[20][4]*dt;
	nextP[20][8] = P[20][8] + P[20][5]*dt;
	nextP[20][9] = P[20][9] + P[20][6]*dt;
	nextP[20][10] = P[20][10];
	nextP[20][11] = P[20][11];
	nextP[20][12] = P[20][12];
	nextP[20][13] = P[20][13];
	nextP[20][14] = P[20][14];
	nextP[20][15] = P[20][15];
	nextP[20][16] = P[20][16];
	nextP[20][17] = P[20][17];
	nextP[20][18] = P[20][18];
	nextP[20][19] = P[20][19];
	nextP[20][20] = P[20][20];
	nextP[20][21] = P[20][21];
	nextP[21][0] = P[21][0] + P[21][1]*SF[7] + P[21][2]*SF[9] + P[21][3]*SF[8] + P[21][10]*SF[11] + P[21][11]*SPP[7] + P[21][12]*SPP[6];
	nextP[21][1] = P[21][1] + P[21][0]*SF[6] + P[21][2]*SF[5] + P[21][3]*SF[9] + P[21][11]*SPP[6] - P[21][12]*SPP[7] - (P[21][10]*q0)/2;
	nextP[21][2] = P[21][2] + P[21][0]*SF[4] + P[21][1]*SF[8] + P[21][3]*SF[6] + P[21][12]*SF[11] - P[21][10]*SPP[6] - (P[21][11]*q0)/2;
	nextP[21][3] = P[21][3] + P[21][0]*SF[5] + P[21][1]*SF[4] + P[21][2]*SF[7] - P[21][11]*SF[11] + P[21][10]*SPP[7] - (P[21][12]*q0)/2;
	nextP[21][4] = P[21][4] + P[21][1]*SF[1] + P[21][0]*SF[3] + P[21][2]*SPP[0] - P[21][3]*SPP[2] - P[21][13]*SPP[4];
	nextP[21][5] = P[21][5] + P[21][0]*SF[2] + P[21][2]*SF[1] + P[21][3]*SF[3] - P[21][1]*SPP[0] + P[21][13]*SPP[3];
	nextP[21][6] = P[21][6] + P[21][1]*SF[2] + P[21][3]*SF[1] + P[21][0]*SPP[0] - P[21][2]*SPP[1] - P[21][13]*SPP[5];
	nextP[21][7] = P[21][7] + P[21][4]*dt;
	nextP[21][8] = P[21][8] + P[21][5]*dt;
	nextP[21][9] = P[21][9] + P[21][6]*dt;
	nextP[21][10] = P[21][10];
	nextP[21][11] = P[21][11];
	nextP[21][12] = P[21][12];
	nextP[21][13] = P[21][13];
	nextP[21][14] = P[21][14];
	nextP[21][15] = P[21][15];
	nextP[21][16] = P[21][16];
	nextP[21][17] = P[21][17];
	nextP[21][18] = P[21][18];
	nextP[21][19] = P[21][19];
	nextP[21][20] = P[21][20];
	nextP[21][21] = P[21][21];

    // add the general state process noise variances
    for (uint8_t i=0; i<= 21; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1e4f)
    {
        for (uint8_t i=7; i<=8; i++)
        {
            for (uint8_t j=0; j<=21; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output and fix numerical errors
    CopyAndFixCovariances();

    // constrain diagonals to prevent ill-conditioning
    ConstrainVariances();

    perf_end(_perf_CovariancePrediction);
}

// fuse selected position, velocity and height measurements
void NavEKF::FuseVelPosNED()
{
    // start performance timer
    perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;
    Vector3f velInnov1;
    Vector3f velInnov2;
    Vector2 posInnov;
    float hgtInnov = 0;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    float NEvelErr;
    float DvelErr;
    float posErr;
    Vector6 R_OBS;
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {

        // if constant position or constant velocity mode use the current states to calculate the predicted
        // measurement rather than use states from a previous time. We need to do this
        // because there may be no stored states due to lack of real measurements.
        if (constPosMode) {
            statesAtPosTime = state;
        } else if (constVelMode) {
            statesAtVelTime = state;
        }

        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed()) gpsRetryTime = gpsRetryTimeUseTAS;
        else gpsRetryTime = gpsRetryTimeNoTAS;

        // form the observation vector and zero velocity and horizontal position observations if in constant position mode
        // If in constant velocity mode, hold the last known horizontal velocity vector
        if (!constPosMode && !constVelMode) {
            observation[0] = velNED.x + gpsVelGlitchOffset.x;
            observation[1] = velNED.y + gpsVelGlitchOffset.y;
            observation[2] = velNED.z;
            observation[3] = gpsPosNE.x + gpsPosGlitchOffsetNE.x;
            observation[4] = gpsPosNE.y + gpsPosGlitchOffsetNE.y;
        } else if (constPosMode){
            for (uint8_t i=0; i<=4; i++) observation[i] = 0.0f;
        } else if (constVelMode) {
            observation[0] = heldVelNE.x;
            observation[1] = heldVelNE.y;
            for (uint8_t i=2; i<=4; i++) observation[i] = 0.0f;
        }
        observation[5] = -hgtMea;

        // calculate additional error in GPS velocity caused by manoeuvring
        NEvelErr = gpsNEVelVarAccScale * accNavMag;
        DvelErr  = gpsDVelVarAccScale * accNavMag;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        R_OBS[0] = sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(NEvelErr);
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = sq(constrain_float(_gpsVertVelNoise, 0.05f, 5.0f)) + sq(DvelErr);
        R_OBS[3] = sq(constrain_float(_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrain_float(_baroAltNoise, 0.1f, 10.0f));

        // if vertical GPS velocity data is being used, check to see if the GPS vertical velocity and barometer
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (_fusionModeGPS == 0 && fuseVelData && (imuSampleTime_ms - lastHgtTime_ms) <  (2 * msecHgtAvg)) {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr  = statesAtHgtTime.position.z - observation[5];
            float velDErr = statesAtVelTime.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[9][9] + R_OBS[5])) && (sq(velDErr) > 9.0f * (P[6][6] + R_OBS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        if (fusePosData) {
            // test horizontal position measurements
            posInnov[0] = statesAtPosTime.position.x - observation[3];
            posInnov[1] = statesAtPosTime.position.y - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate max valid position innovation squared based on a maximum horizontal inertial nav accel error and GPS noise parameter
            // max inertial nav error is scaled with horizontal g to allow for increased errors when manoeuvring
            float accelScale =  (1.0f + 0.1f * accNavMag);
            float maxPosInnov2 = sq(_gpsPosInnovGate * _gpsHorizPosNoise + 0.005f * accelScale * float(_gpsGlitchAccelMax) * sq(0.001f * float(imuSampleTime_ms - posFailTime)));
            posTestRatio = (sq(posInnov[0]) + sq(posInnov[1])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data
            posTimeout = ((imuSampleTime_ms - posFailTime) > gpsRetryTime);
            // use position data if healthy, timed out, or in constant position mode
            if (posHealth || posTimeout || constPosMode) {
                posHealth = true;
                // We don't reset the failed time if we are in constant position mode
                if (!constPosMode) {
                    posFailTime = imuSampleTime_ms;
                }
                // if timed out or outside the specified glitch radius, increment the offset applied to GPS data to compensate for large GPS position jumps
                if (posTimeout || (maxPosInnov2 > sq(float(_gpsGlitchRadiusMax)))) {
                    gpsPosGlitchOffsetNE.x += posInnov[0];
                    gpsPosGlitchOffsetNE.y += posInnov[1];
                    // limit the radius of the offset to 100m and decay the offset to zero radially
                    decayGpsOffset();
                    // reset the position to the current GPS position which will include the glitch correction offset
                    ResetPosition();
                    // don't fuse data on this time step
                    fusePosData = false;
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            if (_fusionModeGPS == 1 || constVelMode) {
                imax = 1;
            }
            float K1 = 0; // innovation to error ratio for IMU1
            float K2 = 0; // innovation to error ratio for IMU2
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 4
                stateIndex   = i + 4;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = statesAtVelTime.velocity[i] - observation[i]; // blended
                velInnov1[i] = statesAtVelTime.vel1[i] - observation[i]; // IMU1
                velInnov2[i] = statesAtVelTime.vel2[i] - observation[i]; // IMU2
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
                // calculate error weightings for single IMU velocity states using
                // observation error to normalise
                float R_hgt;
                if (i == 2) {
                    R_hgt = sq(constrain_float(_gpsVertVelNoise, 0.05f, 5.0f));
                } else {
                    R_hgt = sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f));
                }
                K1 += R_hgt / (R_hgt + sq(velInnov1[i]));
                K2 += R_hgt / (R_hgt + sq(velInnov2[i]));
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // calculate weighting used by fuseVelPosNED to do IMU accel data blending
            // this is used to detect and compensate for aliasing errors with the accelerometers
            // provide for a first order lowpass filter to reduce noise on the weighting if required
            IMU1_weighting = 1.0f * (K1 / (K1 + K2)) + 0.0f * IMU1_weighting; // filter currently inactive
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(_gpsVelInnovGate));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // declare a timeout if we have not fused velocity data for too long or in constant velocity mode
            velTimeout = ((imuSampleTime_ms - velFailTime) > gpsRetryTime) || constVelMode;
            // if data is healthy  or in constant velocity mode we fuse it
            if (velHealth || constVelMode) {
                velHealth = true;
                velFailTime = imuSampleTime_ms;
            } else if (velTimeout && !posHealth) {
                // if data is not healthy and timed out and position is unhealthy we reset the velocity, but do not fuse data on this time step
                ResetVelocity();
                fuseVelData =  false;
            } else {
                // if data is unhealthy and position is healthy, we do not fuse it
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // calculate height innovations
            hgtInnov = statesAtHgtTime.position.z - observation[5];
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(hgtInnov) / (sq(_hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - hgtFailTime) > hgtRetryTime;
            // Fuse height data if healthy or timed out or in constant position mode
            if (hgtHealth || hgtTimeout || constPosMode) {
                hgtHealth = true;
                hgtFailTime = imuSampleTime_ms;
                // if timed out, reset the height, but do not fuse data on this time step
                if (hgtTimeout) {
                    ResetHeight();
                    fuseHgtData = false;
                }
            }
            else {
                hgtHealth = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData && _fusionModeGPS == 0 && velHealth && !constPosMode && PV_AidingMode == AID_ABSOLUTE) {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && _fusionModeGPS == 1 && velHealth && !constPosMode && PV_AidingMode == AID_ABSOLUTE) {
            fuseData[0] = true;
            fuseData[1] = true;
        }
        if ((fusePosData && posHealth && PV_AidingMode == AID_ABSOLUTE) || constPosMode) {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if ((fuseHgtData && hgtHealth) || constPosMode) {
            fuseData[5] = true;
        }
        if (constVelMode) {
            fuseData[0] = true;
            fuseData[1] = true;
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime.velocity[obsIndex] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = statesAtPosTime.position[obsIndex-3] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else {
                    innovVelPos[obsIndex] = statesAtHgtTime.position[obsIndex-3] - observation[obsIndex];
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=12; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }
                // Only height observations are used to update z accel bias estimate
                // Protect Kalman gain from ill-conditioning
                // Don't update Z accel bias if off-level by greater than 60 degrees to avoid scale factor error effects
                if (obsIndex == 5 && prevTnb.c.z > 0.5f) {
                    Kfusion[13] = constrain_float(P[13][stateIndex]*SK,-1.0f,0.0f);
                } else {
                    Kfusion[13] = 0.0f;
                }
                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[14] = P[14][stateIndex]*SK;
                    Kfusion[15] = P[15][stateIndex]*SK;
                } else {
                    Kfusion[14] = 0.0f;
                    Kfusion[15] = 0.0f;
                }
                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }
                // Set the Kalman gain values for the single IMU states
                Kfusion[22] = Kfusion[13]; // IMU2 Z accel bias
                Kfusion[26] = Kfusion[9];  // IMU1 posD
                Kfusion[30] = Kfusion[9];  // IMU2 posD
                for (uint8_t i = 0; i<=2; i++) {
                    Kfusion[i+23] = Kfusion[i+4]; // IMU1 velNED
                    Kfusion[i+27] = Kfusion[i+4]; // IMU2 velNED
                }

                // Correct states that have been predicted using single (not blended) IMU data
                if (obsIndex == 5){
                    // Calculate height measurement innovations using single IMU states
                    float hgtInnov1 = statesAtHgtTime.posD1 - observation[obsIndex];
                    float hgtInnov2 = statesAtHgtTime.posD2 - observation[obsIndex];
                    // Correct single IMU prediction states using height measurement, limiting rate of change of bias to 0.002 m/s3
                    float correctionLimit = 0.002f * dtIMU * dtVelPos;
                    state.accel_zbias1 -= constrain_float(Kfusion[13] * hgtInnov1, -correctionLimit, correctionLimit); // IMU1 Z accel bias
                    state.accel_zbias2 -= constrain_float(Kfusion[22] * hgtInnov2, -correctionLimit, correctionLimit); // IMU2 Z accel bias
                    for (uint8_t i = 23; i<=26; i++) {
                        states[i] = states[i] - Kfusion[i] * hgtInnov1; // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i<=30; i++) {
                        states[i] = states[i] - Kfusion[i] * hgtInnov2; // IMU2 velNED,posD
                    }
                } else if (obsIndex == 0 || obsIndex == 1 || obsIndex == 2) {
                    // Correct single IMU prediction states using velocity measurements
                    for (uint8_t i = 23; i<=26; i++) {
                        states[i] = states[i] - Kfusion[i] * velInnov1[obsIndex]; // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i<=30; i++) {
                        states[i] = states[i] - Kfusion[i] * velInnov2[obsIndex]; // IMU2 velNED,posD
                    }
                }

                // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                // attitude, velocity and position corrections are spread across multiple prediction cycles between now
                // and the anticipated time for the next measurement.
                // Don't spread quaternion corrections if total angle change across predicted interval is going to exceed 0.1 rad
                bool highRates = ((gpsUpdateCountMax * correctedDelAng.length()) > 0.1f);
                for (uint8_t i = 0; i<=21; i++) {
                    if ((i <= 3 && highRates) || i >= 10 || constPosMode || constVelMode) {
                        states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                    } else {
                        if (obsIndex == 5) {
                            hgtIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * hgtUpdateCountMaxInv;
                        } else {
                            gpsIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * gpsUpdateCountMaxInv;
                        }
                    }
                }
                state.quat.normalize();

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=21; i++) {
                    for (uint8_t j= 0; j<=21; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=21; i++) {
                    for (uint8_t j= 0; j<=21; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    perf_end(_perf_FuseVelPosNED);
}

// fuse magnetometer measurements and apply innovation consistency checks
// fuse each axis on consecutive time steps to spread computional load
void NavEKF::FuseMagnetometer()
{
    // start performance timer
    perf_begin(_perf_FuseMagnetometer);

    // declarations
    ftype &q0 = mag_state.q0;
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    uint8_t &obsIndex = mag_state.obsIndex;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector22 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseMagData || obsIndex == 1 || obsIndex == 2)
    {
        // calculate observation jacobians and Kalman gains
        if (fuseMagData)
        {
            // copy required states to local variable names
            q0       = statesAtMagMeasTime.quat[0];
            q1       = statesAtMagMeasTime.quat[1];
            q2       = statesAtMagMeasTime.quat[2];
            q3       = statesAtMagMeasTime.quat[3];
            magN     = statesAtMagMeasTime.earth_magfield[0];
            magE     = statesAtMagMeasTime.earth_magfield[1];
            magD     = statesAtMagMeasTime.earth_magfield[2];
            magXbias = statesAtMagMeasTime.body_magfield[0];
            magYbias = statesAtMagMeasTime.body_magfield[1];
            magZbias = statesAtMagMeasTime.body_magfield[2];

            // rotate predicted earth components into body axes and calculate
            // predicted measurements
            DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            DCM[0][1] = 2*(q1*q2 + q0*q3);
            DCM[0][2] = 2*(q1*q3-q0*q2);
            DCM[1][0] = 2*(q1*q2 - q0*q3);
            DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            DCM[1][2] = 2*(q2*q3 + q0*q1);
            DCM[2][0] = 2*(q1*q3 + q0*q2);
            DCM[2][1] = 2*(q2*q3 - q0*q1);
            DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
            MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
            MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

            // scale magnetometer observation error with total angular rate
            R_MAG = sq(constrain_float(_magNoise, 0.01f, 0.5f)) + sq(magVarRateScale*dAngIMU.length() / dtIMU);

            // calculate observation jacobians
			SH_MAG[0] = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
			SH_MAG[1] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
			SH_MAG[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
			SH_MAG[3] = sq(q3);
			SH_MAG[4] = sq(q2);
			SH_MAG[5] = sq(q1);
			SH_MAG[6] = sq(q0);
			SH_MAG[7] = 2*magN*q0;
			SH_MAG[8] = 2*magE*q3;
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
			H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			H_MAG[1] = SH_MAG[0];
			H_MAG[2] = 2*magE*q1 - 2*magD*q0 - 2*magN*q2;
			H_MAG[3] = SH_MAG[2];
			H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
			H_MAG[17] = 2*q0*q3 + 2*q1*q2;
			H_MAG[18] = 2*q1*q3 - 2*q0*q2;
			H_MAG[19] = 1;

            // calculate Kalman gain
            float temp = (P[19][19] + R_MAG + P[1][19]*SH_MAG[0] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) - (2*magD*q0 - 2*magE*q1 + 2*magN*q2)*(P[19][2] + P[1][2]*SH_MAG[0] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2*q0*q3 + 2*q1*q2) - P[18][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2*q0*q3 + 2*q1*q2) - P[18][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2*q0*q3 + 2*q1*q2) - P[18][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2*q0*q3 + 2*q1*q2) - P[18][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2*q0*q3 + 2*q1*q2) - P[18][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[17][19]*(2*q0*q3 + 2*q1*q2) - P[18][19]*(2*q0*q2 - 2*q1*q3) - P[2][19]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + (2*q0*q3 + 2*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2*q0*q3 + 2*q1*q2) - P[18][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q2 - 2*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2*q0*q3 + 2*q1*q2) - P[18][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MX[0] = 1.0f / temp;
                faultStatus.bad_xmag = false;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[19][19] += 0.1f*R_MAG;
                obsIndex = 1;
                faultStatus.bad_xmag = true;
                return;
            }
            SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
			SK_MX[2] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
			SK_MX[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			SK_MX[4] = 2*q0*q2 - 2*q1*q3;
			SK_MX[5] = 2*q0*q3 + 2*q1*q2;
            Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][1]*SH_MAG[0] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[3] - P[0][2]*SK_MX[2] - P[0][16]*SK_MX[1] + P[0][17]*SK_MX[5] - P[0][18]*SK_MX[4]);
			Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][1]*SH_MAG[0] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[3] - P[1][2]*SK_MX[2] - P[1][16]*SK_MX[1] + P[1][17]*SK_MX[5] - P[1][18]*SK_MX[4]);
			Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][1]*SH_MAG[0] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[3] - P[2][2]*SK_MX[2] - P[2][16]*SK_MX[1] + P[2][17]*SK_MX[5] - P[2][18]*SK_MX[4]);
			Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][1]*SH_MAG[0] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[3] - P[3][2]*SK_MX[2] - P[3][16]*SK_MX[1] + P[3][17]*SK_MX[5] - P[3][18]*SK_MX[4]);
			Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][1]*SH_MAG[0] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[3] - P[4][2]*SK_MX[2] - P[4][16]*SK_MX[1] + P[4][17]*SK_MX[5] - P[4][18]*SK_MX[4]);
			Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][1]*SH_MAG[0] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[3] - P[5][2]*SK_MX[2] - P[5][16]*SK_MX[1] + P[5][17]*SK_MX[5] - P[5][18]*SK_MX[4]);
			Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][1]*SH_MAG[0] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[3] - P[6][2]*SK_MX[2] - P[6][16]*SK_MX[1] + P[6][17]*SK_MX[5] - P[6][18]*SK_MX[4]);
			Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][1]*SH_MAG[0] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[3] - P[7][2]*SK_MX[2] - P[7][16]*SK_MX[1] + P[7][17]*SK_MX[5] - P[7][18]*SK_MX[4]);
			Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][1]*SH_MAG[0] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[3] - P[8][2]*SK_MX[2] - P[8][16]*SK_MX[1] + P[8][17]*SK_MX[5] - P[8][18]*SK_MX[4]);
			Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][1]*SH_MAG[0] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[3] - P[9][2]*SK_MX[2] - P[9][16]*SK_MX[1] + P[9][17]*SK_MX[5] - P[9][18]*SK_MX[4]);
			Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][1]*SH_MAG[0] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[3] - P[10][2]*SK_MX[2] - P[10][16]*SK_MX[1] + P[10][17]*SK_MX[5] - P[10][18]*SK_MX[4]);
			Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][1]*SH_MAG[0] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[3] - P[11][2]*SK_MX[2] - P[11][16]*SK_MX[1] + P[11][17]*SK_MX[5] - P[11][18]*SK_MX[4]);
			Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][1]*SH_MAG[0] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[3] - P[12][2]*SK_MX[2] - P[12][16]*SK_MX[1] + P[12][17]*SK_MX[5] - P[12][18]*SK_MX[4]);
            // this term has been zeroed to improve stability of the Z accel bias
            Kfusion[13] = 0.0f;//SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] - P[13][2]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[5] - P[13][18]*SK_MX[4]);
            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[3] - P[14][2]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[5] - P[14][18]*SK_MX[4]);
                Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[3] - P[15][2]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[5] - P[15][18]*SK_MX[4]);
            } else {
                Kfusion[14] = 0.0;
                Kfusion[15] = 0.0;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[3] - P[16][2]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[5] - P[16][18]*SK_MX[4]);
                Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[3] - P[17][2]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[5] - P[17][18]*SK_MX[4]);
                Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[3] - P[18][2]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[5] - P[18][18]*SK_MX[4]);
                Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[3] - P[19][2]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[5] - P[19][18]*SK_MX[4]);
                Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[3] - P[20][2]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[5] - P[20][18]*SK_MX[4]);
                Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[3] - P[21][2]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[5] - P[21][18]*SK_MX[4]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // calculate the observation innovation variance
            varInnovMag[0] = 1.0f/SK_MX[0];

            // reset the observation index to 0 (we start by fusing the X measurement)
            obsIndex = 0;

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // calculate observation jacobians
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
			H_MAG[0] = SH_MAG[2];
			H_MAG[1] = SH_MAG[1];
			H_MAG[2] = SH_MAG[0];
			H_MAG[3] = 2*magD*q2 - SH_MAG[8] - SH_MAG[7];
			H_MAG[16] = 2*q1*q2 - 2*q0*q3;
			H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
			H_MAG[18] = 2*q0*q1 + 2*q2*q3;
			H_MAG[20] = 1;

            // calculate Kalman gain
            float temp = (P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2*q0*q3 - 2*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2*q0*q3 - 2*q1*q2) + P[18][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (2*q0*q1 + 2*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2*q0*q3 - 2*q1*q2) + P[18][18]*(2*q0*q1 + 2*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2*q0*q3 - 2*q1*q2) + P[18][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[16][20]*(2*q0*q3 - 2*q1*q2) + P[18][20]*(2*q0*q1 + 2*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2*q0*q3 - 2*q1*q2) + P[18][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2*q0*q3 - 2*q1*q2) + P[18][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2*q0*q3 - 2*q1*q2) + P[18][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2*q0*q3 - 2*q1*q2) + P[18][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MY[0] = 1.0f / temp;
                faultStatus.bad_ymag = false;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[20][20] += 0.1f*R_MAG;
                obsIndex = 2;
                faultStatus.bad_ymag = true;
                return;
            }
            SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
			SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MY[3] = 2*q0*q3 - 2*q1*q2;
			SK_MY[4] = 2*q0*q1 + 2*q2*q3;
            Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][17]*SK_MY[1] - P[0][16]*SK_MY[3] + P[0][18]*SK_MY[4]);
			Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][17]*SK_MY[1] - P[1][16]*SK_MY[3] + P[1][18]*SK_MY[4]);
			Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][17]*SK_MY[1] - P[2][16]*SK_MY[3] + P[2][18]*SK_MY[4]);
			Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][17]*SK_MY[1] - P[3][16]*SK_MY[3] + P[3][18]*SK_MY[4]);
			Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][17]*SK_MY[1] - P[4][16]*SK_MY[3] + P[4][18]*SK_MY[4]);
			Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][17]*SK_MY[1] - P[5][16]*SK_MY[3] + P[5][18]*SK_MY[4]);
			Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][17]*SK_MY[1] - P[6][16]*SK_MY[3] + P[6][18]*SK_MY[4]);
			Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][17]*SK_MY[1] - P[7][16]*SK_MY[3] + P[7][18]*SK_MY[4]);
			Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][17]*SK_MY[1] - P[8][16]*SK_MY[3] + P[8][18]*SK_MY[4]);
			Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][17]*SK_MY[1] - P[9][16]*SK_MY[3] + P[9][18]*SK_MY[4]);
			Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][17]*SK_MY[1] - P[10][16]*SK_MY[3] + P[10][18]*SK_MY[4]);
			Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][17]*SK_MY[1] - P[11][16]*SK_MY[3] + P[11][18]*SK_MY[4]);
			Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][17]*SK_MY[1] - P[12][16]*SK_MY[3] + P[12][18]*SK_MY[4]);
            // this term has been zeroed to improve stability of the Z accel bias
            Kfusion[13] = 0.0f;//SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
                Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
            } else {
                Kfusion[14] = 0.0;
                Kfusion[15] = 0.0;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
                Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
                Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
                Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
                Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
                Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // calculate the observation innovation variance
            varInnovMag[1] = 1.0f/SK_MY[0];

            // set flags to indicate to other processes that fusion has been performede and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // calculate observation jacobians
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = 2*magN*q3 - 2*magE*q0 - 2*magD*q1;
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[16] = 2*q0*q2 + 2*q1*q3;
            H_MAG[17] = 2*q2*q3 - 2*q0*q1;
            H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[21] = 1;

            // calculate Kalman gain
            float temp = (P[21][21] + R_MAG + P[0][21]*SH_MAG[1] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) - (2*magD*q1 + 2*magE*q0 - 2*magN*q3)*(P[21][1] + P[0][1]*SH_MAG[1] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2*q0*q2 + 2*q1*q3) - P[17][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2*q0*q2 + 2*q1*q3) - P[17][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2*q0*q2 + 2*q1*q3) - P[17][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2*q0*q2 + 2*q1*q3) - P[17][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2*q0*q2 + 2*q1*q3) - P[17][18]*(2*q0*q1 - 2*q2*q3) - P[1][18]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[16][21]*(2*q0*q2 + 2*q1*q3) - P[17][21]*(2*q0*q1 - 2*q2*q3) - P[1][21]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + (2*q0*q2 + 2*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2*q0*q2 + 2*q1*q3) - P[17][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2*q0*q2 + 2*q1*q3) - P[17][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            if (temp >= R_MAG) {
                SK_MZ[0] = 1.0f / temp;
                faultStatus.bad_zmag = false;
            } else {
                // the calculation is badly conditioned, so we cannot perform fusion on this step
                // we increase the state variances and try again next time
                P[21][21] += 0.1f*R_MAG;
                obsIndex = 3;
                faultStatus.bad_zmag = true;
                return;
            }
            SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            SK_MZ[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
            SK_MZ[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MZ[4] = 2*q0*q1 - 2*q2*q3;
            SK_MZ[5] = 2*q0*q2 + 2*q1*q3;
            Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][0]*SH_MAG[1] + P[0][3]*SH_MAG[0] - P[0][1]*SK_MZ[2] + P[0][2]*SK_MZ[3] + P[0][18]*SK_MZ[1] + P[0][16]*SK_MZ[5] - P[0][17]*SK_MZ[4]);
			Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][0]*SH_MAG[1] + P[1][3]*SH_MAG[0] - P[1][1]*SK_MZ[2] + P[1][2]*SK_MZ[3] + P[1][18]*SK_MZ[1] + P[1][16]*SK_MZ[5] - P[1][17]*SK_MZ[4]);
			Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][0]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[2][1]*SK_MZ[2] + P[2][2]*SK_MZ[3] + P[2][18]*SK_MZ[1] + P[2][16]*SK_MZ[5] - P[2][17]*SK_MZ[4]);
			Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][0]*SH_MAG[1] + P[3][3]*SH_MAG[0] - P[3][1]*SK_MZ[2] + P[3][2]*SK_MZ[3] + P[3][18]*SK_MZ[1] + P[3][16]*SK_MZ[5] - P[3][17]*SK_MZ[4]);
			Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][0]*SH_MAG[1] + P[4][3]*SH_MAG[0] - P[4][1]*SK_MZ[2] + P[4][2]*SK_MZ[3] + P[4][18]*SK_MZ[1] + P[4][16]*SK_MZ[5] - P[4][17]*SK_MZ[4]);
			Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][0]*SH_MAG[1] + P[5][3]*SH_MAG[0] - P[5][1]*SK_MZ[2] + P[5][2]*SK_MZ[3] + P[5][18]*SK_MZ[1] + P[5][16]*SK_MZ[5] - P[5][17]*SK_MZ[4]);
			Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][0]*SH_MAG[1] + P[6][3]*SH_MAG[0] - P[6][1]*SK_MZ[2] + P[6][2]*SK_MZ[3] + P[6][18]*SK_MZ[1] + P[6][16]*SK_MZ[5] - P[6][17]*SK_MZ[4]);
			Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][0]*SH_MAG[1] + P[7][3]*SH_MAG[0] - P[7][1]*SK_MZ[2] + P[7][2]*SK_MZ[3] + P[7][18]*SK_MZ[1] + P[7][16]*SK_MZ[5] - P[7][17]*SK_MZ[4]);
			Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][0]*SH_MAG[1] + P[8][3]*SH_MAG[0] - P[8][1]*SK_MZ[2] + P[8][2]*SK_MZ[3] + P[8][18]*SK_MZ[1] + P[8][16]*SK_MZ[5] - P[8][17]*SK_MZ[4]);
			Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][0]*SH_MAG[1] + P[9][3]*SH_MAG[0] - P[9][1]*SK_MZ[2] + P[9][2]*SK_MZ[3] + P[9][18]*SK_MZ[1] + P[9][16]*SK_MZ[5] - P[9][17]*SK_MZ[4]);
			Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][0]*SH_MAG[1] + P[10][3]*SH_MAG[0] - P[10][1]*SK_MZ[2] + P[10][2]*SK_MZ[3] + P[10][18]*SK_MZ[1] + P[10][16]*SK_MZ[5] - P[10][17]*SK_MZ[4]);
			Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][0]*SH_MAG[1] + P[11][3]*SH_MAG[0] - P[11][1]*SK_MZ[2] + P[11][2]*SK_MZ[3] + P[11][18]*SK_MZ[1] + P[11][16]*SK_MZ[5] - P[11][17]*SK_MZ[4]);
			Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][0]*SH_MAG[1] + P[12][3]*SH_MAG[0] - P[12][1]*SK_MZ[2] + P[12][2]*SK_MZ[3] + P[12][18]*SK_MZ[1] + P[12][16]*SK_MZ[5] - P[12][17]*SK_MZ[4]);
            // this term has been zeroed to improve stability of the Z accel bias
            Kfusion[13] = 0.0f;//SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] + P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] + P[13][2]*SK_MZ[3] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[5] - P[13][17]*SK_MZ[4]);
            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] + P[14][3]*SH_MAG[0] - P[14][1]*SK_MZ[2] + P[14][2]*SK_MZ[3] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[5] - P[14][17]*SK_MZ[4]);
                Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] + P[15][3]*SH_MAG[0] - P[15][1]*SK_MZ[2] + P[15][2]*SK_MZ[3] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[5] - P[15][17]*SK_MZ[4]);
            } else {
                Kfusion[14] = 0.0;
                Kfusion[15] = 0.0;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] + P[16][3]*SH_MAG[0] - P[16][1]*SK_MZ[2] + P[16][2]*SK_MZ[3] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[5] - P[16][17]*SK_MZ[4]);
                Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] + P[17][3]*SH_MAG[0] - P[17][1]*SK_MZ[2] + P[17][2]*SK_MZ[3] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[5] - P[17][17]*SK_MZ[4]);
                Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] + P[18][3]*SH_MAG[0] - P[18][1]*SK_MZ[2] + P[18][2]*SK_MZ[3] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[5] - P[18][17]*SK_MZ[4]);
                Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] + P[19][3]*SH_MAG[0] - P[19][1]*SK_MZ[2] + P[19][2]*SK_MZ[3] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[5] - P[19][17]*SK_MZ[4]);
                Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] + P[20][3]*SH_MAG[0] - P[20][1]*SK_MZ[2] + P[20][2]*SK_MZ[3] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[5] - P[20][17]*SK_MZ[4]);
                Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] + P[21][3]*SH_MAG[0] - P[21][1]*SK_MZ[2] + P[21][2]*SK_MZ[3] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[5] - P[21][17]*SK_MZ[4]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // calculate the observation innovation variance
            varInnovMag[2] = 1.0f/SK_MZ[0];

            // set flags to indicate to other processes that fusion has been performede and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = false;
        }
        // calculate the measurement innovation
        innovMag[obsIndex] = MagPred[obsIndex] - magData[obsIndex];
        // calculate the innovation test ratio
        magTestRatio[obsIndex] = sq(innovMag[obsIndex]) / (sq(_magInnovGate) * varInnovMag[obsIndex]);
        // check the last values from all components and set magnetometer health accordingly
        magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);
        // Don't fuse unless all componenets pass. The exception is if the bad health has timed out and we are not a fly forward vehicle
        // In this case we might as well try using the magnetometer, but with a reduced weighting
        if (magHealth || ((magTestRatio[obsIndex] < 1.0f) && !assume_zero_sideslip() && magTimeout)) {
            // Attitude, velocity and position corrections are averaged across multiple prediction cycles between now and the anticipated time for the next measurement.
            // Don't do averaging of quaternion state corrections if total angle change across predicted interval is going to exceed 0.1 rad
            bool highRates = ((magUpdateCountMax * correctedDelAng.length()) > 0.1f);
            // Calculate the number of averaging frames left to go. This is required becasue magnetometer fusion is applied across three consecutive prediction cycles
            // There is no point averaging if the number of cycles left is less than 2
            float minorFramesToGo = float(magUpdateCountMax) - float(magUpdateCount);
            // correct the state vector or store corrections to be applied incrementally
            for (uint8_t j= 0; j<=21; j++) {
                // If we are forced to use a bad compass, we reduce the weighting by a factor of 4
                if (!magHealth) {
                    Kfusion[j] *= 0.25f;
                }
                if ((j <= 3 && highRates) || j >= 10 || constPosMode || minorFramesToGo < 1.5f ) {
                    states[j] = states[j] - Kfusion[j] * innovMag[obsIndex];
                } else {
                    // scale the correction based on the number of averaging frames left to go
                    magIncrStateDelta[j] -= Kfusion[j] * innovMag[obsIndex] * (magUpdateCountMaxInv * float(magUpdateCountMax) / minorFramesToGo);
                }
            }
            // normalise the quaternion states
            state.quat.normalize();
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=21; i++) {
                for (uint8_t j = 0; j<=3; j++) {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
                for (uint8_t j = 4; j<=15; j++) {
                    KH[i][j] = 0.0f;
                }
                if (!inhibitMagStates) {
                    for (uint8_t j = 16; j<=21; j++) {
                        KH[i][j] = Kfusion[i] * H_MAG[j];
                    }
                } else {
                    for (uint8_t j = 16; j<=21; j++) {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i<=21; i++) {
                for (uint8_t j = 0; j<=21; j++) {
                    KHP[i][j] = 0;
                    for (uint8_t k = 0; k<=3; k++) {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!inhibitMagStates) {
                        for (uint8_t k = 16; k<=21; k++) {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
            for (uint8_t i = 0; i<=21; i++) {
                for (uint8_t j = 0; j<=21; j++) {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
        obsIndex = obsIndex + 1;
    }
    else
    {
    // set flags to indicate to other processes that fusion has not been performed and is not required on the next time step
    magFusePerformed = false;
    magFuseRequired = false;
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    perf_end(_perf_FuseMagnetometer);
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optiocal flow rates and range finder measurements
*/
void NavEKF::EstimateTerrainOffset()
{
    // start performance timer
    perf_begin(_perf_OpticalFlowEKF);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(state.velocity.x) + sq(state.velocity.y);
    float losRateSq = velHorizSq / sq(terrainState - state.position[2]);

    // don't update terrain offset state if there is no range finder and not generating enough LOS rate, or without GPS, as it is poorly observable
    if (!fuseRngData && (gpsNotAvailable || PV_AidingMode == AID_RELATIVE || velHorizSq < 25.0f || losRateSq < 0.01f || onGround)) {
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
        // record the time we last updated the terrain offset state
        gndHgtValidTime_ms = imuSampleTime_ms;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
        float distanceTravelledSq = sq(statesAtRngTime.position[0] - prevPosN) + sq(statesAtRngTime.position[1] - prevPosE);
        distanceTravelledSq = min(distanceTravelledSq, 100.0f);
        prevPosN = statesAtRngTime.position[0];
        prevPosE = statesAtRngTime.position[1];

        // in addition to a terrain gradient error model, we also have a time based error growth that is scaled using the gradient parameter
        float timeLapsed = min(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(0.01f*float(_gndGradientSigma))) + sq(float(_gndGradientSigma) * timeLapsed);
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (fuseRngData) {
            // predict range
            float predRngMeas = max((terrainState - statesAtRngTime.position[2]),0.1f) / Tnb_flow.c.z;

            // Copy required states to local variable names
            float q0 = statesAtRngTime.quat[0]; // quaternion at optical flow measurement time
            float q1 = statesAtRngTime.quat[1]; // quaternion at optical flow measurement time
            float q2 = statesAtRngTime.quat[2]; // quaternion at optical flow measurement time
            float q3 = statesAtRngTime.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = 0.5;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, statesAtRngTime.position[2] + 0.1f);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rngMea;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(_rngInnovGate) * varInnovRng);

            // Check the innovation for consistency and don't fuse if > 5Sigma
            if ((sq(innovRng)*SK_RNG) < 25.0f)
            {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = max(terrainState, statesAtRngTime.position[2] + 0.1f);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = max(Popt,0.0f);

            }
        }

        if (fuseOptFlowData) {

            Vector3f vel; // velocity of sensor relative to ground in NED axes
            Vector3f relVelSensor; // velocity of sensor relative to ground in sensor axes
            float losPred; // predicted optical flow angular rate measurement
            float q0 = statesAtFlowTime.quat[0]; // quaternion at optical flow measurement time
            float q1 = statesAtFlowTime.quat[1]; // quaternion at optical flow measurement time
            float q2 = statesAtFlowTime.quat[2]; // quaternion at optical flow measurement time
            float q3 = statesAtFlowTime.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;

            // Correct velocities for GPS glitch recovery offset
            vel.x          = statesAtFlowTime.velocity[0] - gpsVelGlitchOffset.x;
            vel.y          = statesAtFlowTime.velocity[1] - gpsVelGlitchOffset.y;
            vel.z          = statesAtFlowTime.velocity[2];

            // predict range to centre of image
            float flowRngPred = max((terrainState - statesAtFlowTime.position[2]),0.1f) / Tnb_flow.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, statesAtFlowTime.position[2] + 0.1f);

            // calculate relative velocity in sensor frame
            relVelSensor = Tnb_flow*vel;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred =   relVelSensor.length()/flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - sqrtf(sq(flowRadXYcomp[0]) + sq(flowRadXYcomp[1]));

            // calculate observation jacobian
            float t3 = sq(q0);
            float t4 = sq(q1);
            float t5 = sq(q2);
            float t6 = sq(q3);
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t14 = t3+t4-t5-t6;
            float t15 = t14*vel.x;
            float t16 = t10+t11;
            float t17 = t16*vel.y;
            float t18 = q0*q2*2.0f;
            float t19 = q1*q3*2.0f;
            float t20 = t18-t19;
            float t21 = t20*vel.z;
            float t2 = t15+t17-t21;
            float t7 = t3-t4-t5+t6;
            float t8 = statesAtFlowTime.position[2]-terrainState;
            float t9 = 1.0f/sq(t8);
            float t24 = t3-t4+t5-t6;
            float t25 = t24*vel.y;
            float t26 = t10-t11;
            float t27 = t26*vel.x;
            float t28 = q0*q1*2.0f;
            float t29 = q2*q3*2.0f;
            float t30 = t28+t29;
            float t31 = t30*vel.z;
            float t12 = t25-t27+t31;
            float t13 = sq(t7);
            float t22 = sq(t2);
            float t23 = 1.0f/(t8*t8*t8);
            float t32 = sq(t12);
            H_OPT = 0.5f*(t13*t22*t23*2.0f+t13*t23*t32*2.0f)/sqrtf(t9*t13*t22+t9*t13*t32);

            // calculate innovation variances
            auxFlowObsInnovVar = H_OPT*Popt*H_OPT + R_LOS;

            // calculate Kalman gain
            K_OPT = Popt*H_OPT/auxFlowObsInnovVar;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio = sq(auxFlowObsInnov) / (sq(_flowInnovGate) * auxFlowObsInnovVar);

            // don't fuse if optical flow data is outside valid range
            if (max(flowRadXY[0],flowRadXY[1]) < _maxFlowRate) {

            // correct the state
            terrainState -= K_OPT * auxFlowObsInnov;

            // constrain the state
            terrainState = max(terrainState, statesAtFlowTime.position[2] + 0.1f);

            // correct the covariance
            Popt = Popt - K_OPT * H_OPT * Popt;

            // prevent the state variances from becoming negative
            Popt = max(Popt,0.0f);
            }
        }
    }

    // stop the performance timer
    perf_end(_perf_OpticalFlowEKF);
}

void NavEKF::FuseOptFlow()
{
    // start performance timer
    perf_begin(_perf_FuseOptFlow);

    Vector22 H_LOS;
    Vector9 tempVar;
    Vector3f velNED_local;
    Vector3f relVelSensor;

    uint8_t &obsIndex = flow_state.obsIndex;
    ftype &q0 = flow_state.q0;
    ftype &q1 = flow_state.q1;
    ftype &q2 = flow_state.q2;
    ftype &q3 = flow_state.q3;
    ftype *SH_LOS = &flow_state.SH_LOS[0];
    ftype *SK_LOS = &flow_state.SK_LOS[0];
    ftype &vn = flow_state.vn;
    ftype &ve = flow_state.ve;
    ftype &vd = flow_state.vd;
    ftype &pd = flow_state.pd;
    ftype *losPred = &flow_state.losPred[0];

    // Copy required states to local variable names
    q0       = statesAtFlowTime.quat[0];
    q1       = statesAtFlowTime.quat[1];
    q2       = statesAtFlowTime.quat[2];
    q3       = statesAtFlowTime.quat[3];
    vn       = statesAtFlowTime.velocity[0];
    ve       = statesAtFlowTime.velocity[1];
    vd       = statesAtFlowTime.velocity[2];
    pd       = statesAtFlowTime.position[2];
    // Correct velocities for GPS glitch recovery offset
    velNED_local.x = vn - gpsVelGlitchOffset.x;
    velNED_local.y = ve - gpsVelGlitchOffset.y;
    velNED_local.z = vd;

    // constrain terrain to be below vehicle
    terrainState = max(terrainState, pd + 0.1f);
    float heightAboveGndEst = terrainState - pd;
    // Calculate observation jacobians and Kalman gains
    if (obsIndex == 0) {
        // calculate range from ground plain to centre of sensor fov assuming flat earth
        float range = constrain_float((heightAboveGndEst/Tnb_flow.c.z),0.1f,1000.0f);

        // calculate relative velocity in sensor frame
        relVelSensor = Tnb_flow*velNED_local;

        // divide velocity by range  to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;


        // Calculate common expressions for observation jacobians
        SH_LOS[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        SH_LOS[1] = vn*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + ve*(2*q0*q3 + 2*q1*q2);
        SH_LOS[2] = ve*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - vn*(2*q0*q3 - 2*q1*q2);
        SH_LOS[3] = 1/(pd - terrainState);
        SH_LOS[4] = sq(SH_LOS[3]);

        // Calculate common expressions for Kalman gains
        // calculate innovation variance for Y axis observation
        varInnovOptFlow[1] = (R_LOS + (SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3])*(P[0][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][0]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][0]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][0]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][0]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3])*(P[0][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][1]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][1]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][1]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][1]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3])*(P[0][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][2]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][2]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][2]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][2]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3])*(P[0][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][3]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][3]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][3]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][3]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[1]*SH_LOS[4]*(P[0][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][9]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][9]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][9]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][9]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))*(P[0][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][4]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][4]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][4]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][4]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2)*(P[0][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][5]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][5]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][5]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][5]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3)*(P[0][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][6]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][6]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - P[9][6]*SH_LOS[0]*SH_LOS[1]*SH_LOS[4] + P[4][6]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))));
        if (fabsf(varInnovOptFlow[1]) > R_LOS) {
            SK_LOS[0] = 1.0f/varInnovOptFlow[1];
        } else {
            SK_LOS[0] = 1.0f/R_LOS;
        }
        // calculate innovation variance for X axis observation
        varInnovOptFlow[0] = (R_LOS + (SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3])*(P[0][0]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][0]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][0]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][0]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3])*(P[0][1]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][1]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][1]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][1]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3])*(P[0][2]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][2]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][2]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][2]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3])*(P[0][3]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][3]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][3]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][3]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[2]*SH_LOS[4]*(P[0][9]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][9]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][9]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][9]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))*(P[0][5]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][5]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][5]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][5]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2)*(P[0][4]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][4]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][4]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][4]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3)*(P[0][6]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][6]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - P[9][6]*SH_LOS[0]*SH_LOS[2]*SH_LOS[4] + P[5][6]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))));
        if (fabsf(varInnovOptFlow[0]) > R_LOS) {
            SK_LOS[1] = 1.0f/varInnovOptFlow[0];
        } else {
            SK_LOS[1] = 1.0f/R_LOS;
        }
        SK_LOS[2] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn);
        SK_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
        SK_LOS[4] = SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn);
        SK_LOS[5] = SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
        SK_LOS[6] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SK_LOS[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SK_LOS[8] = SH_LOS[3];

        // Calculate common intermediate terms
        tempVar[0] = SH_LOS[0]*SK_LOS[6]*SK_LOS[8];
        tempVar[1] = SH_LOS[0]*SH_LOS[2]*SH_LOS[4];
        tempVar[2] = 2.0f*SH_LOS[2]*SK_LOS[8];
        tempVar[3] = SH_LOS[0]*SK_LOS[8]*(2.0f*q0*q1 + 2.0f*q2*q3);
        tempVar[4] = SH_LOS[0]*SK_LOS[8]*(2.0f*q0*q3 - 2.0f*q1*q2);
        tempVar[5] = (SK_LOS[5] - q2*tempVar[2]);
        tempVar[6] = (SK_LOS[2] - q3*tempVar[2]);
        tempVar[7] = (SK_LOS[3] - q1*tempVar[2]);
        tempVar[8] = (SK_LOS[4] + q0*tempVar[2]);

        // calculate observation jacobians for X LOS rate
        for (uint8_t i = 0; i < 22; i++) H_LOS[i] = 0;
        H_LOS[0] = - SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) - 2*q0*SH_LOS[2]*SH_LOS[3];
        H_LOS[1] = 2*q1*SH_LOS[2]*SH_LOS[3] - SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
        H_LOS[2] = 2*q2*SH_LOS[2]*SH_LOS[3] - SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
        H_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3];
        H_LOS[4] = SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2);
        H_LOS[5] = -SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        H_LOS[6] = -SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3);
        H_LOS[9] = tempVar[1];

        // calculate Kalman gains for X LOS rate
        Kfusion[0] = -(P[0][0]*tempVar[8] + P[0][1]*tempVar[7] - P[0][3]*tempVar[6] + P[0][2]*tempVar[5] - P[0][4]*tempVar[4] + P[0][6]*tempVar[3] - P[0][9]*tempVar[1] + P[0][5]*tempVar[0])/(R_LOS + tempVar[8]*(P[0][0]*tempVar[8] + P[1][0]*tempVar[7] + P[2][0]*tempVar[5] - P[3][0]*tempVar[6] - P[4][0]*tempVar[4] + P[6][0]*tempVar[3] - P[9][0]*tempVar[1] + P[5][0]*tempVar[0]) + tempVar[7]*(P[0][1]*tempVar[8] + P[1][1]*tempVar[7] + P[2][1]*tempVar[5] - P[3][1]*tempVar[6] - P[4][1]*tempVar[4] + P[6][1]*tempVar[3] - P[9][1]*tempVar[1] + P[5][1]*tempVar[0]) + tempVar[5]*(P[0][2]*tempVar[8] + P[1][2]*tempVar[7] + P[2][2]*tempVar[5] - P[3][2]*tempVar[6] - P[4][2]*tempVar[4] + P[6][2]*tempVar[3] - P[9][2]*tempVar[1] + P[5][2]*tempVar[0]) - tempVar[6]*(P[0][3]*tempVar[8] + P[1][3]*tempVar[7] + P[2][3]*tempVar[5] - P[3][3]*tempVar[6] - P[4][3]*tempVar[4] + P[6][3]*tempVar[3] - P[9][3]*tempVar[1] + P[5][3]*tempVar[0]) - tempVar[4]*(P[0][4]*tempVar[8] + P[1][4]*tempVar[7] + P[2][4]*tempVar[5] - P[3][4]*tempVar[6] - P[4][4]*tempVar[4] + P[6][4]*tempVar[3] - P[9][4]*tempVar[1] + P[5][4]*tempVar[0]) + tempVar[3]*(P[0][6]*tempVar[8] + P[1][6]*tempVar[7] + P[2][6]*tempVar[5] - P[3][6]*tempVar[6] - P[4][6]*tempVar[4] + P[6][6]*tempVar[3] - P[9][6]*tempVar[1] + P[5][6]*tempVar[0]) - tempVar[1]*(P[0][9]*tempVar[8] + P[1][9]*tempVar[7] + P[2][9]*tempVar[5] - P[3][9]*tempVar[6] - P[4][9]*tempVar[4] + P[6][9]*tempVar[3] - P[9][9]*tempVar[1] + P[5][9]*tempVar[0]) + tempVar[0]*(P[0][5]*tempVar[8] + P[1][5]*tempVar[7] + P[2][5]*tempVar[5] - P[3][5]*tempVar[6] - P[4][5]*tempVar[4] + P[6][5]*tempVar[3] - P[9][5]*tempVar[1] + P[5][5]*tempVar[0]));
        Kfusion[1] = -SK_LOS[1]*(P[1][0]*tempVar[8] + P[1][1]*tempVar[7] - P[1][3]*tempVar[6] + P[1][2]*tempVar[5] - P[1][4]*tempVar[4] + P[1][6]*tempVar[3] - P[1][9]*tempVar[1] + P[1][5]*tempVar[0]);
        Kfusion[2] = -SK_LOS[1]*(P[2][0]*tempVar[8] + P[2][1]*tempVar[7] - P[2][3]*tempVar[6] + P[2][2]*tempVar[5] - P[2][4]*tempVar[4] + P[2][6]*tempVar[3] - P[2][9]*tempVar[1] + P[2][5]*tempVar[0]);
        Kfusion[3] = -SK_LOS[1]*(P[3][0]*tempVar[8] + P[3][1]*tempVar[7] - P[3][3]*tempVar[6] + P[3][2]*tempVar[5] - P[3][4]*tempVar[4] + P[3][6]*tempVar[3] - P[3][9]*tempVar[1] + P[3][5]*tempVar[0]);
        Kfusion[4] = -SK_LOS[1]*(P[4][0]*tempVar[8] + P[4][1]*tempVar[7] - P[4][3]*tempVar[6] + P[4][2]*tempVar[5] - P[4][4]*tempVar[4] + P[4][6]*tempVar[3] - P[4][9]*tempVar[1] + P[4][5]*tempVar[0]);
        Kfusion[5] = -SK_LOS[1]*(P[5][0]*tempVar[8] + P[5][1]*tempVar[7] - P[5][3]*tempVar[6] + P[5][2]*tempVar[5] - P[5][4]*tempVar[4] + P[5][6]*tempVar[3] - P[5][9]*tempVar[1] + P[5][5]*tempVar[0]);
        Kfusion[6] = -SK_LOS[1]*(P[6][0]*tempVar[8] + P[6][1]*tempVar[7] - P[6][3]*tempVar[6] + P[6][2]*tempVar[5] - P[6][4]*tempVar[4] + P[6][6]*tempVar[3] - P[6][9]*tempVar[1] + P[6][5]*tempVar[0]);
        Kfusion[7] = -SK_LOS[1]*(P[7][0]*tempVar[8] + P[7][1]*tempVar[7] - P[7][3]*tempVar[6] + P[7][2]*tempVar[5] - P[7][4]*tempVar[4] + P[7][6]*tempVar[3] - P[7][9]*tempVar[1] + P[7][5]*tempVar[0]);
        Kfusion[8] = -SK_LOS[1]*(P[8][0]*tempVar[8] + P[8][1]*tempVar[7] - P[8][3]*tempVar[6] + P[8][2]*tempVar[5] - P[8][4]*tempVar[4] + P[8][6]*tempVar[3] - P[8][9]*tempVar[1] + P[8][5]*tempVar[0]);
        Kfusion[9] = -SK_LOS[1]*(P[9][0]*tempVar[8] + P[9][1]*tempVar[7] - P[9][3]*tempVar[6] + P[9][2]*tempVar[5] - P[9][4]*tempVar[4] + P[9][6]*tempVar[3] - P[9][9]*tempVar[1] + P[9][5]*tempVar[0]);
        Kfusion[10] = -SK_LOS[1]*(P[10][0]*tempVar[8] + P[10][1]*tempVar[7] - P[10][3]*tempVar[6] + P[10][2]*tempVar[5] - P[10][4]*tempVar[4] + P[10][6]*tempVar[3] - P[10][9]*tempVar[1] + P[10][5]*tempVar[0]);
        Kfusion[11] = -SK_LOS[1]*(P[11][0]*tempVar[8] + P[11][1]*tempVar[7] - P[11][3]*tempVar[6] + P[11][2]*tempVar[5] - P[11][4]*tempVar[4] + P[11][6]*tempVar[3] - P[11][9]*tempVar[1] + P[11][5]*tempVar[0]);
        Kfusion[12] = -SK_LOS[1]*(P[12][0]*tempVar[8] + P[12][1]*tempVar[7] - P[12][3]*tempVar[6] + P[12][2]*tempVar[5] - P[12][4]*tempVar[4] + P[12][6]*tempVar[3] - P[12][9]*tempVar[1] + P[12][5]*tempVar[0]);
        // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
        Kfusion[13] = 0.0f;//-SK_LOS[1]*(P[13][0]*tempVar[8] + P[13][1]*tempVar[7] - P[13][3]*tempVar[6] + P[13][2]*tempVar[5] - P[13][4]*tempVar[4] + P[13][6]*tempVar[3] - P[13][9]*tempVar[1] + P[13][5]*tempVar[0]);
        if (inhibitWindStates) {
            Kfusion[14] = -SK_LOS[1]*(P[14][0]*tempVar[8] + P[14][1]*tempVar[7] - P[14][3]*tempVar[6] + P[14][2]*tempVar[5] - P[14][4]*tempVar[4] + P[14][6]*tempVar[3] - P[14][9]*tempVar[1] + P[14][5]*tempVar[0]);
            Kfusion[15] = -SK_LOS[1]*(P[15][0]*tempVar[8] + P[15][1]*tempVar[7] - P[15][3]*tempVar[6] + P[15][2]*tempVar[5] - P[15][4]*tempVar[4] + P[15][6]*tempVar[3] - P[15][9]*tempVar[1] + P[15][5]*tempVar[0]);
        } else {
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
        }
        if (inhibitMagStates) {
            Kfusion[16] = -SK_LOS[1]*(P[16][0]*tempVar[8] + P[16][1]*tempVar[7] - P[16][3]*tempVar[6] + P[16][2]*tempVar[5] - P[16][4]*tempVar[4] + P[16][6]*tempVar[3] - P[16][9]*tempVar[1] + P[16][5]*tempVar[0]);
            Kfusion[17] = -SK_LOS[1]*(P[17][0]*tempVar[8] + P[17][1]*tempVar[7] - P[17][3]*tempVar[6] + P[17][2]*tempVar[5] - P[17][4]*tempVar[4] + P[17][6]*tempVar[3] - P[17][9]*tempVar[1] + P[17][5]*tempVar[0]);
            Kfusion[18] = -SK_LOS[1]*(P[18][0]*tempVar[8] + P[18][1]*tempVar[7] - P[18][3]*tempVar[6] + P[18][2]*tempVar[5] - P[18][4]*tempVar[4] + P[18][6]*tempVar[3] - P[18][9]*tempVar[1] + P[18][5]*tempVar[0]);
            Kfusion[19] = -SK_LOS[1]*(P[19][0]*tempVar[8] + P[19][1]*tempVar[7] - P[19][3]*tempVar[6] + P[19][2]*tempVar[5] - P[19][4]*tempVar[4] + P[19][6]*tempVar[3] - P[19][9]*tempVar[1] + P[19][5]*tempVar[0]);
            Kfusion[20] = -SK_LOS[1]*(P[20][0]*tempVar[8] + P[20][1]*tempVar[7] - P[20][3]*tempVar[6] + P[20][2]*tempVar[5] - P[20][4]*tempVar[4] + P[20][6]*tempVar[3] - P[20][9]*tempVar[1] + P[20][5]*tempVar[0]);
            Kfusion[21] = -SK_LOS[1]*(P[21][0]*tempVar[8] + P[21][1]*tempVar[7] - P[21][3]*tempVar[6] + P[21][2]*tempVar[5] - P[21][4]*tempVar[4] + P[21][6]*tempVar[3] - P[21][9]*tempVar[1] + P[21][5]*tempVar[0]);
        } else {
            for (uint8_t i = 16; i <= 21; i++) {
                Kfusion[i] = 0.0f;
            }
        }
        // calculate innovation for X axis observation
        innovOptFlow[0] = losPred[0] - flowRadXYcomp[0];

    } else if (obsIndex == 1) {

        // calculate intermediate common variables
        tempVar[0] = 2.0f*SH_LOS[1]*SK_LOS[8];
        tempVar[1] = (SK_LOS[2] + q0*tempVar[0]);
        tempVar[2] = (SK_LOS[5] - q1*tempVar[0]);
        tempVar[3] = (SK_LOS[3] + q2*tempVar[0]);
        tempVar[4] = (SK_LOS[4] + q3*tempVar[0]);
        tempVar[5] = SH_LOS[0]*SK_LOS[8]*(2*q0*q3 + 2*q1*q2);
        tempVar[6] = SH_LOS[0]*SK_LOS[8]*(2*q0*q2 - 2*q1*q3);
        tempVar[7] = SH_LOS[0]*SH_LOS[1]*SH_LOS[4];
        tempVar[8] = SH_LOS[0]*SK_LOS[7]*SK_LOS[8];

        // Calculate observation jacobians for Y LOS rate
        for (uint8_t i = 0; i < 22; i++) H_LOS[i] = 0;
        H_LOS[0] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3];
        H_LOS[1] = SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3];
        H_LOS[2] = - SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q2*SH_LOS[1]*SH_LOS[3];
        H_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3];
        H_LOS[4] = SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3));
        H_LOS[5] = SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2);
        H_LOS[6] = -SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3);
        H_LOS[9] = -tempVar[7];

        // Calculate Kalman gains for Y LOS rate
        Kfusion[0] = SK_LOS[0]*(P[0][0]*tempVar[1] + P[0][1]*tempVar[2] - P[0][2]*tempVar[3] + P[0][3]*tempVar[4] + P[0][5]*tempVar[5] - P[0][6]*tempVar[6] - P[0][9]*tempVar[7] + P[0][4]*tempVar[8]);
        Kfusion[1] = SK_LOS[0]*(P[1][0]*tempVar[1] + P[1][1]*tempVar[2] - P[1][2]*tempVar[3] + P[1][3]*tempVar[4] + P[1][5]*tempVar[5] - P[1][6]*tempVar[6] - P[1][9]*tempVar[7] + P[1][4]*tempVar[8]);
        Kfusion[2] = SK_LOS[0]*(P[2][0]*tempVar[1] + P[2][1]*tempVar[2] - P[2][2]*tempVar[3] + P[2][3]*tempVar[4] + P[2][5]*tempVar[5] - P[2][6]*tempVar[6] - P[2][9]*tempVar[7] + P[2][4]*tempVar[8]);
        Kfusion[3] = SK_LOS[0]*(P[3][0]*tempVar[1] + P[3][1]*tempVar[2] - P[3][2]*tempVar[3] + P[3][3]*tempVar[4] + P[3][5]*tempVar[5] - P[3][6]*tempVar[6] - P[3][9]*tempVar[7] + P[3][4]*tempVar[8]);
        Kfusion[4] = SK_LOS[0]*(P[4][0]*tempVar[1] + P[4][1]*tempVar[2] - P[4][2]*tempVar[3] + P[4][3]*tempVar[4] + P[4][5]*tempVar[5] - P[4][6]*tempVar[6] - P[4][9]*tempVar[7] + P[4][4]*tempVar[8]);
        Kfusion[5] = SK_LOS[0]*(P[5][0]*tempVar[1] + P[5][1]*tempVar[2] - P[5][2]*tempVar[3] + P[5][3]*tempVar[4] + P[5][5]*tempVar[5] - P[5][6]*tempVar[6] - P[5][9]*tempVar[7] + P[5][4]*tempVar[8]);
        Kfusion[6] = SK_LOS[0]*(P[6][0]*tempVar[1] + P[6][1]*tempVar[2] - P[6][2]*tempVar[3] + P[6][3]*tempVar[4] + P[6][5]*tempVar[5] - P[6][6]*tempVar[6] - P[6][9]*tempVar[7] + P[6][4]*tempVar[8]);
        Kfusion[7] = SK_LOS[0]*(P[7][0]*tempVar[1] + P[7][1]*tempVar[2] - P[7][2]*tempVar[3] + P[7][3]*tempVar[4] + P[7][5]*tempVar[5] - P[7][6]*tempVar[6] - P[7][9]*tempVar[7] + P[7][4]*tempVar[8]);
        Kfusion[8] = SK_LOS[0]*(P[8][0]*tempVar[1] + P[8][1]*tempVar[2] - P[8][2]*tempVar[3] + P[8][3]*tempVar[4] + P[8][5]*tempVar[5] - P[8][6]*tempVar[6] - P[8][9]*tempVar[7] + P[8][4]*tempVar[8]);
        Kfusion[9] = SK_LOS[0]*(P[9][0]*tempVar[1] + P[9][1]*tempVar[2] - P[9][2]*tempVar[3] + P[9][3]*tempVar[4] + P[9][5]*tempVar[5] - P[9][6]*tempVar[6] - P[9][9]*tempVar[7] + P[9][4]*tempVar[8]);
        Kfusion[10] = SK_LOS[0]*(P[10][0]*tempVar[1] + P[10][1]*tempVar[2] - P[10][2]*tempVar[3] + P[10][3]*tempVar[4] + P[10][5]*tempVar[5] - P[10][6]*tempVar[6] - P[10][9]*tempVar[7] + P[10][4]*tempVar[8]);
        Kfusion[11] = SK_LOS[0]*(P[11][0]*tempVar[1] + P[11][1]*tempVar[2] - P[11][2]*tempVar[3] + P[11][3]*tempVar[4] + P[11][5]*tempVar[5] - P[11][6]*tempVar[6] - P[11][9]*tempVar[7] + P[11][4]*tempVar[8]);
        Kfusion[12] = SK_LOS[0]*(P[12][0]*tempVar[1] + P[12][1]*tempVar[2] - P[12][2]*tempVar[3] + P[12][3]*tempVar[4] + P[12][5]*tempVar[5] - P[12][6]*tempVar[6] - P[12][9]*tempVar[7] + P[12][4]*tempVar[8]);
        // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
        Kfusion[13] = 0.0f;//SK_LOS[0]*(P[13][0]*tempVar[1] + P[13][1]*tempVar[2] - P[13][2]*tempVar[3] + P[13][3]*tempVar[4] + P[13][5]*tempVar[5] - P[13][6]*tempVar[6] - P[13][9]*tempVar[7] + P[13][4]*tempVar[8]);
        if (inhibitWindStates) {
            Kfusion[14] = SK_LOS[0]*(P[14][0]*tempVar[1] + P[14][1]*tempVar[2] - P[14][2]*tempVar[3] + P[14][3]*tempVar[4] + P[14][5]*tempVar[5] - P[14][6]*tempVar[6] - P[14][9]*tempVar[7] + P[14][4]*tempVar[8]);
            Kfusion[15] = SK_LOS[0]*(P[15][0]*tempVar[1] + P[15][1]*tempVar[2] - P[15][2]*tempVar[3] + P[15][3]*tempVar[4] + P[15][5]*tempVar[5] - P[15][6]*tempVar[6] - P[15][9]*tempVar[7] + P[15][4]*tempVar[8]);
        } else {
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
        }
        if (inhibitMagStates) {
        Kfusion[16] = SK_LOS[0]*(P[16][0]*tempVar[1] + P[16][1]*tempVar[2] - P[16][2]*tempVar[3] + P[16][3]*tempVar[4] + P[16][5]*tempVar[5] - P[16][6]*tempVar[6] - P[16][9]*tempVar[7] + P[16][4]*tempVar[8]);
        Kfusion[17] = SK_LOS[0]*(P[17][0]*tempVar[1] + P[17][1]*tempVar[2] - P[17][2]*tempVar[3] + P[17][3]*tempVar[4] + P[17][5]*tempVar[5] - P[17][6]*tempVar[6] - P[17][9]*tempVar[7] + P[17][4]*tempVar[8]);
        Kfusion[18] = SK_LOS[0]*(P[18][0]*tempVar[1] + P[18][1]*tempVar[2] - P[18][2]*tempVar[3] + P[18][3]*tempVar[4] + P[18][5]*tempVar[5] - P[18][6]*tempVar[6] - P[18][9]*tempVar[7] + P[18][4]*tempVar[8]);
        Kfusion[19] = SK_LOS[0]*(P[19][0]*tempVar[1] + P[19][1]*tempVar[2] - P[19][2]*tempVar[3] + P[19][3]*tempVar[4] + P[19][5]*tempVar[5] - P[19][6]*tempVar[6] - P[19][9]*tempVar[7] + P[19][4]*tempVar[8]);
        Kfusion[20] = SK_LOS[0]*(P[20][0]*tempVar[1] + P[20][1]*tempVar[2] - P[20][2]*tempVar[3] + P[20][3]*tempVar[4] + P[20][5]*tempVar[5] - P[20][6]*tempVar[6] - P[20][9]*tempVar[7] + P[20][4]*tempVar[8]);
        Kfusion[21] = SK_LOS[0]*(P[21][0]*tempVar[1] + P[21][1]*tempVar[2] - P[21][2]*tempVar[3] + P[21][3]*tempVar[4] + P[21][5]*tempVar[5] - P[21][6]*tempVar[6] - P[21][9]*tempVar[7] + P[21][4]*tempVar[8]);
        } else {
            memset(&H_LOS[0], 0, sizeof(H_LOS));
            memset(&Kfusion[0], 0, sizeof(Kfusion));
        }
        // calculate innovation for Y observation
        innovOptFlow[1] = losPred[1] - flowRadXYcomp[1];

    }

    // calculate the innovation consistency test ratio
    flowTestRatio[obsIndex] = sq(innovOptFlow[obsIndex]) / (sq(_flowInnovGate) * varInnovOptFlow[obsIndex]);

    // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
    if ((flowTestRatio[obsIndex]) < 1.0f && (flowRadXY[obsIndex] < _maxFlowRate)) {
        // record the last time both X and Y observations were accepted for fusion
        if (obsIndex == 0) {
            flowXfailed = false;
        } else if (!flowXfailed) {
            prevFlowFuseTime_ms = imuSampleTime_ms;
        }
        // Attitude, velocity and position corrections are averaged across multiple prediction cycles between now and the anticipated time for the next measurement.
        // Don't do averaging of quaternion state corrections if total angle change across predicted interval is going to exceed 0.1 rad
        bool highRates = ((flowUpdateCountMax * correctedDelAng.length()) > 0.1f);
        // Calculate the number of averaging frames left to go. This is required because flow fusion is applied across two consecutive prediction cycles
        // There is no point averaging if the number of cycles left is less than 2
        float minorFramesToGo = float(flowUpdateCountMax) - float(flowUpdateCount);
        for (uint8_t i = 0; i<=21; i++) {
            if ((i <= 3 && highRates) || i >= 10 || minorFramesToGo < 1.5f) {
                states[i] = states[i] - Kfusion[i] * innovOptFlow[obsIndex];
            } else {
                    flowIncrStateDelta[i] -= Kfusion[i] * innovOptFlow[obsIndex] * (flowUpdateCountMaxInv * float(flowUpdateCountMax) / minorFramesToGo);
            }
        }
        // normalise the quaternion states
        state.quat.normalize();
        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 6; j++)
            {
                KH[i][j] = Kfusion[i] * H_LOS[j];
            }
            for (uint8_t j = 7; j <= 8; j++)
            {
                KH[i][j] = 0.0f;
            }
            KH[i][9] = Kfusion[i] * H_LOS[9];
            for (uint8_t j = 10; j <= 21; j++)
            {
                KH[i][j] = 0.0f;
            }
        }
        for (uint8_t i = 0; i <= 21; i++)
        {
            for (uint8_t j = 0; j <= 21; j++)
            {
                KHP[i][j] = 0.0f;
                for (uint8_t k = 0; k <= 6; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
                KHP[i][j] = KHP[i][j] + KH[i][9] * P[9][j];
            }
        }
        for (uint8_t i = 0; i <=  21; i++)
        {
            for (uint8_t j = 0; j <=  21; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    } else if (obsIndex == 0) {
        // store the fact we have failed the X conponent so that a combined X and Y axis pass/fail can be calculated next time round
        flowXfailed = true;
    }

    ForceSymmetry();
    ConstrainVariances();

    // stop the performance timer
    perf_end(_perf_FuseOptFlow);
}

// fuse true airspeed measurements
void NavEKF::FuseAirspeed()
{
    // start performance timer
    perf_begin(_perf_FuseAirspeed);

    // declarations
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float EAS2TAS = _ahrs->get_EAS2TAS();
    const float R_TAS = sq(constrain_float(_easNoise, 0.5f, 5.0f) * constrain_float(EAS2TAS, 0.9f, 10.0f));
    Vector3f SH_TAS;
    float SK_TAS;
    Vector22 H_TAS;
    float VtasPred;

    // health is set bad until test passed
    tasHealth = false;

    // copy required states to local variable names
    vn = statesAtVtasMeasTime.velocity.x;
    ve = statesAtVtasMeasTime.velocity.y;
    vd = statesAtVtasMeasTime.velocity.z;
    vwn = statesAtVtasMeasTime.wind_vel.x;
    vwe = statesAtVtasMeasTime.wind_vel.y;

    // calculate the predicted airspeed, compensating for bias in GPS velocity when we are pulling a glitch offset back in
    VtasPred = pythagorous3((ve - gpsVelGlitchOffset.y - vwe) , (vn - gpsVelGlitchOffset.x - vwn) , vd);
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // calculate observation jacobians
        SH_TAS[0] = 1.0f/VtasPred;
		SH_TAS[1] = (SH_TAS[0]*(2*ve - 2*vwe))/2;
		SH_TAS[2] = (SH_TAS[0]*(2*vn - 2*vwn))/2;
        for (uint8_t i=0; i<=21; i++) H_TAS[i] = 0.0f;
		H_TAS[4] = SH_TAS[2];
		H_TAS[5] = SH_TAS[1];
		H_TAS[6] = vd*SH_TAS[0];
		H_TAS[14] = -SH_TAS[2];
		H_TAS[15] = -SH_TAS[1];

        // calculate Kalman gains
        float temp = (R_TAS + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[14][4]*SH_TAS[2] - P[15][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[14][5]*SH_TAS[2] - P[15][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][14]*SH_TAS[2] + P[5][14]*SH_TAS[1] - P[14][14]*SH_TAS[2] - P[15][14]*SH_TAS[1] + P[6][14]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][15]*SH_TAS[2] + P[5][15]*SH_TAS[1] - P[14][15]*SH_TAS[2] - P[15][15]*SH_TAS[1] + P[6][15]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[14][6]*SH_TAS[2] - P[15][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
        if (temp >= R_TAS) {
            SK_TAS = 1.0f / temp;
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we increase the wind state variances and try again next time
            P[14][14] += 0.05f*R_TAS;
            P[15][15] += 0.05f*R_TAS;
            faultStatus.bad_airspeed = true;
            return;
        }
        Kfusion[0] = SK_TAS*(P[0][4]*SH_TAS[2] - P[0][14]*SH_TAS[2] + P[0][5]*SH_TAS[1] - P[0][15]*SH_TAS[1] + P[0][6]*vd*SH_TAS[0]);
		Kfusion[1] = SK_TAS*(P[1][4]*SH_TAS[2] - P[1][14]*SH_TAS[2] + P[1][5]*SH_TAS[1] - P[1][15]*SH_TAS[1] + P[1][6]*vd*SH_TAS[0]);
		Kfusion[2] = SK_TAS*(P[2][4]*SH_TAS[2] - P[2][14]*SH_TAS[2] + P[2][5]*SH_TAS[1] - P[2][15]*SH_TAS[1] + P[2][6]*vd*SH_TAS[0]);
		Kfusion[3] = SK_TAS*(P[3][4]*SH_TAS[2] - P[3][14]*SH_TAS[2] + P[3][5]*SH_TAS[1] - P[3][15]*SH_TAS[1] + P[3][6]*vd*SH_TAS[0]);
		Kfusion[4] = SK_TAS*(P[4][4]*SH_TAS[2] - P[4][14]*SH_TAS[2] + P[4][5]*SH_TAS[1] - P[4][15]*SH_TAS[1] + P[4][6]*vd*SH_TAS[0]);
		Kfusion[5] = SK_TAS*(P[5][4]*SH_TAS[2] - P[5][14]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[5][15]*SH_TAS[1] + P[5][6]*vd*SH_TAS[0]);
		Kfusion[6] = SK_TAS*(P[6][4]*SH_TAS[2] - P[6][14]*SH_TAS[2] + P[6][5]*SH_TAS[1] - P[6][15]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]);
		Kfusion[7] = SK_TAS*(P[7][4]*SH_TAS[2] - P[7][14]*SH_TAS[2] + P[7][5]*SH_TAS[1] - P[7][15]*SH_TAS[1] + P[7][6]*vd*SH_TAS[0]);
		Kfusion[8] = SK_TAS*(P[8][4]*SH_TAS[2] - P[8][14]*SH_TAS[2] + P[8][5]*SH_TAS[1] - P[8][15]*SH_TAS[1] + P[8][6]*vd*SH_TAS[0]);
		Kfusion[9] = SK_TAS*(P[9][4]*SH_TAS[2] - P[9][14]*SH_TAS[2] + P[9][5]*SH_TAS[1] - P[9][15]*SH_TAS[1] + P[9][6]*vd*SH_TAS[0]);
		Kfusion[10] = SK_TAS*(P[10][4]*SH_TAS[2] - P[10][14]*SH_TAS[2] + P[10][5]*SH_TAS[1] - P[10][15]*SH_TAS[1] + P[10][6]*vd*SH_TAS[0]);
		Kfusion[11] = SK_TAS*(P[11][4]*SH_TAS[2] - P[11][14]*SH_TAS[2] + P[11][5]*SH_TAS[1] - P[11][15]*SH_TAS[1] + P[11][6]*vd*SH_TAS[0]);
		Kfusion[12] = SK_TAS*(P[12][4]*SH_TAS[2] - P[12][14]*SH_TAS[2] + P[12][5]*SH_TAS[1] - P[12][15]*SH_TAS[1] + P[12][6]*vd*SH_TAS[0]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[13] = 0.0f;//SK_TAS*(P[13][4]*SH_TAS[2] - P[13][14]*SH_TAS[2] + P[13][5]*SH_TAS[1] - P[13][15]*SH_TAS[1] + P[13][6]*vd*SH_TAS[0]);
		Kfusion[14] = SK_TAS*(P[14][4]*SH_TAS[2] - P[14][14]*SH_TAS[2] + P[14][5]*SH_TAS[1] - P[14][15]*SH_TAS[1] + P[14][6]*vd*SH_TAS[0]);
		Kfusion[15] = SK_TAS*(P[15][4]*SH_TAS[2] - P[15][14]*SH_TAS[2] + P[15][5]*SH_TAS[1] - P[15][15]*SH_TAS[1] + P[15][6]*vd*SH_TAS[0]);
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_TAS*(P[16][4]*SH_TAS[2] - P[16][14]*SH_TAS[2] + P[16][5]*SH_TAS[1] - P[16][15]*SH_TAS[1] + P[16][6]*vd*SH_TAS[0]);
            Kfusion[17] = SK_TAS*(P[17][4]*SH_TAS[2] - P[17][14]*SH_TAS[2] + P[17][5]*SH_TAS[1] - P[17][15]*SH_TAS[1] + P[17][6]*vd*SH_TAS[0]);
            Kfusion[18] = SK_TAS*(P[18][4]*SH_TAS[2] - P[18][14]*SH_TAS[2] + P[18][5]*SH_TAS[1] - P[18][15]*SH_TAS[1] + P[18][6]*vd*SH_TAS[0]);
            Kfusion[19] = SK_TAS*(P[19][4]*SH_TAS[2] - P[19][14]*SH_TAS[2] + P[19][5]*SH_TAS[1] - P[19][15]*SH_TAS[1] + P[19][6]*vd*SH_TAS[0]);
            Kfusion[20] = SK_TAS*(P[20][4]*SH_TAS[2] - P[20][14]*SH_TAS[2] + P[20][5]*SH_TAS[1] - P[20][15]*SH_TAS[1] + P[20][6]*vd*SH_TAS[0]);
            Kfusion[21] = SK_TAS*(P[21][4]*SH_TAS[2] - P[21][14]*SH_TAS[2] + P[21][5]*SH_TAS[1] - P[21][15]*SH_TAS[1] + P[21][6]*vd*SH_TAS[0]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate measurement innovation variance
        varInnovVtas = 1.0f/SK_TAS;

        // calculate measurement innovation
        innovVtas = VtasPred - VtasMeas;

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(_tasInnovGate) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        tasHealth = ((tasTestRatio < 1.0f) || badIMUdata);
        tasTimeout = (imuSampleTime_ms - tasFailTime) > tasRetryTime;

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasHealth || (tasTimeout && posTimeout))
        {

            // restart the counter
            tasFailTime = imuSampleTime_ms;

            // correct the state vector
            for (uint8_t j=0; j<=21; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }

            state.quat.normalize();

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in H to reduce the number of operations
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=3; j++) KH[i][j] = 0.0;
                for (uint8_t j = 4; j<=6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 7; j<=13; j++) KH[i][j] = 0.0;
                for (uint8_t j = 14; j<=15; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 16; j<=21; j++) KH[i][j] = 0.0;
            }
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=21; j++)
                {
                    KHP[i][j] = 0;
                    for (uint8_t k = 4; k<=6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 14; k<=15; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=21; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    perf_end(_perf_FuseAirspeed);
}

// fuse sythetic sideslip measurement of zero
void NavEKF::FuseSideslip()
{
    // start performance timer
    perf_begin(_perf_FuseSideslip);

    // declarations
    float q0;
    float q1;
    float q2;
    float q3;
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    const float R_BETA = 0.03f; // assume a sideslip angle RMS of ~10 deg
    Vector13 SH_BETA;
    Vector8 SK_BETA;
    Vector3f vel_rel_wind;
    Vector22 H_BETA;
    float innovBeta;

    // copy required states to local variable names
    q0 = state.quat[0];
    q1 = state.quat[1];
    q2 = state.quat[2];
    q3 = state.quat[3];
    vn = state.velocity.x;
    ve = state.velocity.y;
    vd = state.velocity.z;
    vwn = state.wind_vel.x;
    vwe = state.wind_vel.y;

    // calculate predicted wind relative velocity in NED, compensating for offset in velcity when we are pulling a GPS glitch offset back in
    vel_rel_wind.x = vn - vwn - gpsVelGlitchOffset.x;
    vel_rel_wind.y = ve - vwe - gpsVelGlitchOffset.y;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        if (fabsf(SH_BETA[0]) <= 1e-9f) {
            faultStatus.bad_sideslip = true;
            return;
        } else {
            faultStatus.bad_sideslip = false;
        }
        SH_BETA[1] = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - (vn - vwn)*(2*q0*q3 - 2*q1*q2);
        SH_BETA[2] = vn - vwn;
        SH_BETA[3] = ve - vwe;
        SH_BETA[4] = 1/sq(SH_BETA[0]);
        SH_BETA[5] = 1/SH_BETA[0];
        SH_BETA[6] = SH_BETA[5]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[8] = 2*q0*SH_BETA[3] - 2*q3*SH_BETA[2] + 2*q1*vd;
        SH_BETA[9] = 2*q0*SH_BETA[2] + 2*q3*SH_BETA[3] - 2*q2*vd;
        SH_BETA[10] = 2*q2*SH_BETA[2] - 2*q1*SH_BETA[3] + 2*q0*vd;
        SH_BETA[11] = 2*q1*SH_BETA[2] + 2*q2*SH_BETA[3] + 2*q3*vd;
        SH_BETA[12] = 2*q0*q3;
        for (uint8_t i=0; i<=21; i++) {
            H_BETA[i] = 0.0f;
        }
        H_BETA[0] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        H_BETA[1] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        H_BETA[2] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        H_BETA[3] = - SH_BETA[5]*SH_BETA[9] - SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
        H_BETA[4] = - SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) - SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[5] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        H_BETA[6] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        H_BETA[14] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA[15] = SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2) - SH_BETA[6];

        // Calculate Kalman gains
        float temp = (R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[14][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][4]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][4]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][4]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][4]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][4]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[14][14]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][14]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][14]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][14]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][14]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][14]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][14]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][14]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][14]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[14][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][5]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][5]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][5]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][5]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][5]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[14][15]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][15]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][15]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][15]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][15]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][15]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][15]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][15]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][15]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P[14][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][0]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][0]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][0]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][0]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][0]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P[14][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][1]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][1]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][1]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][1]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][1]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P[14][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][2]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][2]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][2]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][2]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][2]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P[14][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][3]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][3]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][3]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][3]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][3]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))*(P[14][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][6]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][6]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][6]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][6]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][6]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))));
        if (temp >= R_BETA) {
            SK_BETA[0] = 1.0f / temp;
            faultStatus.bad_sideslip = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            faultStatus.bad_sideslip = true;
            return;
        }
        SK_BETA[1] = SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        SK_BETA[2] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2);
        SK_BETA[3] = SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3);
        SK_BETA[4] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        SK_BETA[5] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        SK_BETA[6] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        SK_BETA[7] = SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
        Kfusion[0] = SK_BETA[0]*(P[0][0]*SK_BETA[5] + P[0][1]*SK_BETA[4] - P[0][4]*SK_BETA[1] + P[0][5]*SK_BETA[2] + P[0][2]*SK_BETA[6] + P[0][6]*SK_BETA[3] - P[0][3]*SK_BETA[7] + P[0][14]*SK_BETA[1] - P[0][15]*SK_BETA[2]);
        Kfusion[1] = SK_BETA[0]*(P[1][0]*SK_BETA[5] + P[1][1]*SK_BETA[4] - P[1][4]*SK_BETA[1] + P[1][5]*SK_BETA[2] + P[1][2]*SK_BETA[6] + P[1][6]*SK_BETA[3] - P[1][3]*SK_BETA[7] + P[1][14]*SK_BETA[1] - P[1][15]*SK_BETA[2]);
        Kfusion[2] = SK_BETA[0]*(P[2][0]*SK_BETA[5] + P[2][1]*SK_BETA[4] - P[2][4]*SK_BETA[1] + P[2][5]*SK_BETA[2] + P[2][2]*SK_BETA[6] + P[2][6]*SK_BETA[3] - P[2][3]*SK_BETA[7] + P[2][14]*SK_BETA[1] - P[2][15]*SK_BETA[2]);
        Kfusion[3] = SK_BETA[0]*(P[3][0]*SK_BETA[5] + P[3][1]*SK_BETA[4] - P[3][4]*SK_BETA[1] + P[3][5]*SK_BETA[2] + P[3][2]*SK_BETA[6] + P[3][6]*SK_BETA[3] - P[3][3]*SK_BETA[7] + P[3][14]*SK_BETA[1] - P[3][15]*SK_BETA[2]);
        Kfusion[4] = SK_BETA[0]*(P[4][0]*SK_BETA[5] + P[4][1]*SK_BETA[4] - P[4][4]*SK_BETA[1] + P[4][5]*SK_BETA[2] + P[4][2]*SK_BETA[6] + P[4][6]*SK_BETA[3] - P[4][3]*SK_BETA[7] + P[4][14]*SK_BETA[1] - P[4][15]*SK_BETA[2]);
        Kfusion[5] = SK_BETA[0]*(P[5][0]*SK_BETA[5] + P[5][1]*SK_BETA[4] - P[5][4]*SK_BETA[1] + P[5][5]*SK_BETA[2] + P[5][2]*SK_BETA[6] + P[5][6]*SK_BETA[3] - P[5][3]*SK_BETA[7] + P[5][14]*SK_BETA[1] - P[5][15]*SK_BETA[2]);
        Kfusion[6] = SK_BETA[0]*(P[6][0]*SK_BETA[5] + P[6][1]*SK_BETA[4] - P[6][4]*SK_BETA[1] + P[6][5]*SK_BETA[2] + P[6][2]*SK_BETA[6] + P[6][6]*SK_BETA[3] - P[6][3]*SK_BETA[7] + P[6][14]*SK_BETA[1] - P[6][15]*SK_BETA[2]);
        Kfusion[7] = SK_BETA[0]*(P[7][0]*SK_BETA[5] + P[7][1]*SK_BETA[4] - P[7][4]*SK_BETA[1] + P[7][5]*SK_BETA[2] + P[7][2]*SK_BETA[6] + P[7][6]*SK_BETA[3] - P[7][3]*SK_BETA[7] + P[7][14]*SK_BETA[1] - P[7][15]*SK_BETA[2]);
        Kfusion[8] = SK_BETA[0]*(P[8][0]*SK_BETA[5] + P[8][1]*SK_BETA[4] - P[8][4]*SK_BETA[1] + P[8][5]*SK_BETA[2] + P[8][2]*SK_BETA[6] + P[8][6]*SK_BETA[3] - P[8][3]*SK_BETA[7] + P[8][14]*SK_BETA[1] - P[8][15]*SK_BETA[2]);
        Kfusion[9] = SK_BETA[0]*(P[9][0]*SK_BETA[5] + P[9][1]*SK_BETA[4] - P[9][4]*SK_BETA[1] + P[9][5]*SK_BETA[2] + P[9][2]*SK_BETA[6] + P[9][6]*SK_BETA[3] - P[9][3]*SK_BETA[7] + P[9][14]*SK_BETA[1] - P[9][15]*SK_BETA[2]);
        Kfusion[10] = SK_BETA[0]*(P[10][0]*SK_BETA[5] + P[10][1]*SK_BETA[4] - P[10][4]*SK_BETA[1] + P[10][5]*SK_BETA[2] + P[10][2]*SK_BETA[6] + P[10][6]*SK_BETA[3] - P[10][3]*SK_BETA[7] + P[10][14]*SK_BETA[1] - P[10][15]*SK_BETA[2]);
        Kfusion[11] = SK_BETA[0]*(P[11][0]*SK_BETA[5] + P[11][1]*SK_BETA[4] - P[11][4]*SK_BETA[1] + P[11][5]*SK_BETA[2] + P[11][2]*SK_BETA[6] + P[11][6]*SK_BETA[3] - P[11][3]*SK_BETA[7] + P[11][14]*SK_BETA[1] - P[11][15]*SK_BETA[2]);
        Kfusion[12] = SK_BETA[0]*(P[12][0]*SK_BETA[5] + P[12][1]*SK_BETA[4] - P[12][4]*SK_BETA[1] + P[12][5]*SK_BETA[2] + P[12][2]*SK_BETA[6] + P[12][6]*SK_BETA[3] - P[12][3]*SK_BETA[7] + P[12][14]*SK_BETA[1] - P[12][15]*SK_BETA[2]);
        // this term has been zeroed to improve stability of the Z accel bias
        Kfusion[13] = 0.0f;//SK_BETA[0]*(P[13][0]*SK_BETA[5] + P[13][1]*SK_BETA[4] - P[13][4]*SK_BETA[1] + P[13][5]*SK_BETA[2] + P[13][2]*SK_BETA[6] + P[13][6]*SK_BETA[3] - P[13][3]*SK_BETA[7] + P[13][14]*SK_BETA[1] - P[13][15]*SK_BETA[2]);
        Kfusion[14] = SK_BETA[0]*(P[14][0]*SK_BETA[5] + P[14][1]*SK_BETA[4] - P[14][4]*SK_BETA[1] + P[14][5]*SK_BETA[2] + P[14][2]*SK_BETA[6] + P[14][6]*SK_BETA[3] - P[14][3]*SK_BETA[7] + P[14][14]*SK_BETA[1] - P[14][15]*SK_BETA[2]);
        Kfusion[15] = SK_BETA[0]*(P[15][0]*SK_BETA[5] + P[15][1]*SK_BETA[4] - P[15][4]*SK_BETA[1] + P[15][5]*SK_BETA[2] + P[15][2]*SK_BETA[6] + P[15][6]*SK_BETA[3] - P[15][3]*SK_BETA[7] + P[15][14]*SK_BETA[1] - P[15][15]*SK_BETA[2]);
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_BETA[0]*(P[16][0]*SK_BETA[5] + P[16][1]*SK_BETA[4] - P[16][4]*SK_BETA[1] + P[16][5]*SK_BETA[2] + P[16][2]*SK_BETA[6] + P[16][6]*SK_BETA[3] - P[16][3]*SK_BETA[7] + P[16][14]*SK_BETA[1] - P[16][15]*SK_BETA[2]);
            Kfusion[17] = SK_BETA[0]*(P[17][0]*SK_BETA[5] + P[17][1]*SK_BETA[4] - P[17][4]*SK_BETA[1] + P[17][5]*SK_BETA[2] + P[17][2]*SK_BETA[6] + P[17][6]*SK_BETA[3] - P[17][3]*SK_BETA[7] + P[17][14]*SK_BETA[1] - P[17][15]*SK_BETA[2]);
            Kfusion[18] = SK_BETA[0]*(P[18][0]*SK_BETA[5] + P[18][1]*SK_BETA[4] - P[18][4]*SK_BETA[1] + P[18][5]*SK_BETA[2] + P[18][2]*SK_BETA[6] + P[18][6]*SK_BETA[3] - P[18][3]*SK_BETA[7] + P[18][14]*SK_BETA[1] - P[18][15]*SK_BETA[2]);
            Kfusion[19] = SK_BETA[0]*(P[19][0]*SK_BETA[5] + P[19][1]*SK_BETA[4] - P[19][4]*SK_BETA[1] + P[19][5]*SK_BETA[2] + P[19][2]*SK_BETA[6] + P[19][6]*SK_BETA[3] - P[19][3]*SK_BETA[7] + P[19][14]*SK_BETA[1] - P[19][15]*SK_BETA[2]);
            Kfusion[20] = SK_BETA[0]*(P[20][0]*SK_BETA[5] + P[20][1]*SK_BETA[4] - P[20][4]*SK_BETA[1] + P[20][5]*SK_BETA[2] + P[20][2]*SK_BETA[6] + P[20][6]*SK_BETA[3] - P[20][3]*SK_BETA[7] + P[20][14]*SK_BETA[1] - P[20][15]*SK_BETA[2]);
            Kfusion[21] = SK_BETA[0]*(P[21][0]*SK_BETA[5] + P[21][1]*SK_BETA[4] - P[21][4]*SK_BETA[1] + P[21][5]*SK_BETA[2] + P[21][2]*SK_BETA[6] + P[21][6]*SK_BETA[3] - P[21][3]*SK_BETA[7] + P[21][14]*SK_BETA[1] - P[21][15]*SK_BETA[2]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = vel_rel_wind.y / vel_rel_wind.x;

        // reject measurement if greater than 3-sigma inconsistency
        if (innovBeta > 0.5f) {
            return;
        }

        // correct the state vector
        for (uint8_t j=0; j<=21; j++)
        {
            states[j] = states[j] - Kfusion[j] * innovBeta;
        }

        state.quat.normalize();

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in H to reduce the
        // number of operations
        for (uint8_t i = 0; i<=21; i++)
        {
            for (uint8_t j = 0; j<=6; j++)
            {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (uint8_t j = 7; j<=13; j++) KH[i][j] = 0.0;
            for (uint8_t j = 14; j<=15; j++)
            {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (uint8_t j = 16; j<=21; j++) KH[i][j] = 0.0;
        }
        for (uint8_t i = 0; i<=21; i++)
        {
            for (uint8_t j = 0; j<=21; j++)
            {
                KHP[i][j] = 0;
                for (uint8_t k = 0; k<=6; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
                for (uint8_t k = 14; k<=15; k++)
                {
                    KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                }
            }
        }
        for (uint8_t i = 0; i<=21; i++)
        {
            for (uint8_t j = 0; j<=21; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop the performance timer
    perf_end(_perf_FuseSideslip);
}

// zero specified range of rows in the state covariance matrix
void NavEKF::zeroRows(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*22);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF::zeroCols(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=21; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// store states in a history array along with time stamp
void NavEKF::StoreStates()
{
    // Don't need to store states more often than every 10 msec
    if (imuSampleTime_ms - lastStateStoreTime_ms >= 10) {
        lastStateStoreTime_ms = imuSampleTime_ms;
        if (storeIndex > 49) {
            storeIndex = 0;
        }
        storedStates[storeIndex] = state;
        statetimeStamp[storeIndex] = lastStateStoreTime_ms;
        storeIndex = storeIndex + 1;
    }
}

// reset the stored state history and store the current state
void NavEKF::StoreStatesReset()
{
    // clear stored state history
    memset(&storedStates[0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    // store current state vector in first column
    storeIndex = 0;
    storedStates[storeIndex] = state;
    statetimeStamp[storeIndex] = imuSampleTime_ms;
    storeIndex = storeIndex + 1;
}

// recall state vector stored at closest time to the one specified by msec
void NavEKF::RecallStates(state_elements &statesForFusion, uint32_t msec)
{
    uint32_t timeDelta;
    uint32_t bestTimeDelta = 200;
    uint8_t bestStoreIndex = 0;
    for (uint8_t i=0; i<=49; i++)
    {
        timeDelta = msec - statetimeStamp[i];
        if (timeDelta < bestTimeDelta)
        {
            bestStoreIndex = i;
            bestTimeDelta = timeDelta;
        }
    }
    if (bestTimeDelta < 200) // only output stored state if < 200 msec retrieval error
    {
        statesForFusion = storedStates[bestStoreIndex];
    }
    else // otherwise output current state
    {
        statesForFusion = state;
    }
}

// recall omega (angular rate vector) average across the time interval from msecStart to msecEnd
void NavEKF::RecallOmega(Vector3f &omegaAvg, uint32_t msecStart, uint32_t msecEnd)
{
    // calculate average angular rate vector over the time interval from msecStart to msecEnd
    // if no values are inside the time window, return the current angular rate
    omegaAvg.zero();
    uint8_t numAvg = 0;
    for (uint8_t i=0; i<=49; i++)
    {
        if (msecStart <= statetimeStamp[i] && msecEnd >= statetimeStamp[i])
        {
            omegaAvg += storedStates[i].omega;
            numAvg += 1;
        }
    }
    if (numAvg >= 1)
    {
        omegaAvg = omegaAvg / float(numAvg);
    } else {
        omegaAvg = correctedDelAng * (1.0f * dtIMUinv);
    }
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// return the Euler roll, pitch and yaw angle in radians
void NavEKF::getEulerAngles(Vector3f &euler) const
{
    state.quat.to_euler(euler.x, euler.y, euler.z);
    euler = euler - _ahrs->get_trim();
}

// This returns the specific forces in the NED frame
void NavEKF::getAccelNED(Vector3f &accelNED) const {
    accelNED = velDotNED;
    accelNED.z -= GRAVITY_MSS;
}

// return NED velocity in m/s
//
void NavEKF::getVelNED(Vector3f &vel) const
{
    vel = state.velocity;
}

// return the last calculated NED position relative to the reference point (m).
// return false if no position is available
bool NavEKF::getPosNED(Vector3f &pos) const
{
    if (constPosMode) {
        pos.x = lastKnownPositionNE.x;
        pos.y = lastKnownPositionNE.y;
    } else {
        pos.x = state.position.x;
        pos.y = state.position.y;
    }
    // If relying on optical flow, then output ground relative position so that the vehicle does terain following
    if (_fusionModeGPS == 3) {
        pos.z = state.position.z - terrainState;
    } else {
        pos.z = state.position.z;
    }
    return true;
}

// return body axis gyro bias estimates in rad/sec
void NavEKF::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMU < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = state.gyro_bias / dtIMU;
}

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
void NavEKF::resetGyroBias(void)
{
    state.gyro_bias.zero();
    zeroRows(P,10,12);
    zeroCols(P,10,12);
    P[10][10] = sq(radians(INIT_GYRO_BIAS_UNCERTAINTY * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];

}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming as it will only be actioned when the filter is in constant position mode
// This command is forgotten by the EKF each time it goes back into constant position mode (eg the vehicle disarms)
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF::setInhibitGPS(void)
{
    if(!constPosMode) {
        return 0;
    }
    if (optFlowDataPresent()) {
        PV_AidingMode = AID_RELATIVE;
        return 2;
    } else {
        PV_AidingMode = AID_NONE;
        return 1;
    }
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (flowDataValid || PV_AidingMode == AID_RELATIVE) {
        // allow 1.0 rad/sec margin for angular motion
        ekfGndSpdLimit = max((_maxFlowRate - 1.0f), 0.0f) * max((terrainState - state.position[2]), 0.1f);
        // use standard gains up to 5.0 metres height and reduce above that
        ekfNavVelGainScaler = 4.0f / max((terrainState - state.position[2]),4.0f);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// return weighting of first IMU in blending function
void NavEKF::getIMU1Weighting(float &ret) const
{
    ret = IMU1_weighting;
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF::getAccelZBias(float &zbias1, float &zbias2) const {
    if (dtIMU < 1e-6f) {
        zbias1 = 0;
        zbias2 = 0;
    } else {
        zbias1 = state.accel_zbias1 * dtIMUinv;
        zbias2 = state.accel_zbias2 * dtIMUinv;
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF::getWind(Vector3f &wind) const
{
    wind.x = state.wind_vel.x;
    wind.y = state.wind_vel.y;
    wind.z = 0.0f; // currently don't estimate this
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF::getMagNED(Vector3f &magNED) const
{
    magNED = state.earth_magfield * 1000.0f;
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ = state.body_magfield*1000.0f;
}

// return the last calculated latitude, longitude and height
bool NavEKF::getLLH(struct Location &loc) const
{
    if(validOrigin) {
        loc.lat = EKF_origin.lat;
        loc.lng = EKF_origin.lng;
        loc.alt = EKF_origin.alt - state.position.z*100;
        loc.flags.relative_alt = 0;
        loc.flags.terrain_alt = 0;
        if (constPosMode) {
            location_offset(loc, lastKnownPositionNE.x, lastKnownPositionNE.y);
        } else {
            location_offset(loc, state.position.x, state.position.y);
        }
        return true;
    } else {
        return false;
    }
}

// return the estimated height above ground level
bool NavEKF::getHAGL(float &HAGL) const
{
    HAGL = terrainState - state.position.z;
    return !inhibitGndState;
}

// return data for debugging optical flow fusion
void NavEKF::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    varFlow = max(flowTestRatio[0],flowTestRatio[1]);
    gndOffset = terrainState;
    flowInnovX = innovOptFlow[0];
    flowInnovY = innovOptFlow[1];
    auxInnov = auxFlowObsInnov;
    HAGL = terrainState - state.position.z;
    rngInnov = innovRng;
    range = rngMea;
    gndOffsetErr = sqrtf(Popt); // note Popt is constrained to be non-negative in EstimateTerrainOffset()
}

// calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
void NavEKF::SetFlightAndFusionModes()
{
    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f){
        manoeuvring = true;
    } else {
        manoeuvring = false;
    }
    // if we are a fly forward type vehicle, then in-air mode can be determined through a combination of speed and height criteria
    if (assume_zero_sideslip()) {
        // Evaluate a numerical score that defines the likelihood we are in the air
        const AP_Airspeed *airspeed = _ahrs->get_airspeed();
        float gndSpdSq = sq(velNED[0]) + sq(velNED[1]);
        uint8_t inAirSum = 0;
        bool highGndSpdStage2 = false;

        // trigger at 8 m/s airspeed
        if (airspeed && airspeed->use() && airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 8.0f) {
            inAirSum++;
        }

        // this will trigger during change in baro height
        if (fabsf(_baro.get_climb_rate()) > 0.5f) {

            inAirSum++;
        }

        // trigger at 3 m/s GPS velocity
        if (gndSpdSq > 9.0f) {
            inAirSum++;
        }

        // trigger at 6 m/s GPS velocity
        if (gndSpdSq > 36.0f) {
            highGndSpdStage2 = true;
            inAirSum++;
        }

        // trigger at 9 m/s GPS velocity
        if (gndSpdSq > 81.0f) {
            inAirSum++;
        }

        // trigger if more than 15m away from initial height
        if (fabsf(hgtMea) > 15.0f) {
            inAirSum++;
        }

        // this will trigger due to air turbulence during flight
        if (accNavMag > 0.5f) {
            inAirSum++;
        }

        // if we on-ground then 4 or more out of 7 criteria are required to transition to the
        // in-air mode and we also need enough GPS velocity to be able to calculate a reliable ground track heading
        if (onGround && (inAirSum >= 4) && highGndSpdStage2) {
            onGround = false;
        }
        // if is possible we are in flight, set the time this condition was last detected
        if (inAirSum >= 1) {
            airborneDetectTime_ms = imuSampleTime_ms;
        }
        // after 5 seconds of not detecting a possible flight condition, we transition to on-ground mode
        if(!onGround && ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
        }
        // perform a yaw alignment check against GPS if exiting on-ground mode
        // this is done to protect against unrecoverable heading alignment errors due to compass faults
        if (!onGround && prevOnGround) {
            alignYawGPS();
        }
        // If we aren't using an airspeed sensor we set the wind velocity to the reciprocal
        // of the velocity vector and scale states so that the wind speed is equal to 3m/s. This helps prevent gains
        // being too high at the start of flight if launching into a headwind until the first turn when the EKF can form
        // a wind speed estimate and also corrects bad initial wind estimates due to heading errors
        if (!onGround && prevOnGround && !useAirspeed()) {
            setWindVelStates();
        }
    }
    // store current on-ground status for next time
    prevOnGround = onGround;
    // If we are on ground, or in constant position mode, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    inhibitWindStates = ((!useAirspeed() && !assume_zero_sideslip()) || onGround || constPosMode);
    // request mag calibration for both in-air and manoeuvre threshold options
    bool magCalRequested = ((_magCal == 0) && !onGround) || ((_magCal == 1) && manoeuvring);
    // deny mag calibration request if we aren't using the compass, are in the pre-arm constant position mode or it has been inhibited by the user
    bool magCalDenied = (!use_compass() || constPosMode || (_magCal == 2));
    // inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || magCalDenied);
}

// initialise the covariance matrix
void NavEKF::CovarianceInit()
{
    // zero the matrix
    for (uint8_t i=1; i<=21; i++)
    {
        for (uint8_t j=0; j<=21; j++)
        {
            P[i][j] = 0.0f;
        }
    }
    // quaternions - TODO better maths for initial quaternion covariances that uses roll, pitch and yaw
    P[0][0]   = 1.0e-9f;
    P[1][1]   = 0.25f*sq(radians(1.0f));
    P[2][2]   = 0.25f*sq(radians(1.0f));
    P[3][3]   = 0.25f*sq(radians(1.0f));
    // velocities
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7f);
    // positions
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(5.0f);
    // delta angle biases
    P[10][10] = sq(radians(INIT_GYRO_BIAS_UNCERTAINTY * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // Z delta velocity bias
    P[13][13] = 0.0f;
    // wind velocities
    P[14][14] = 0.0f;
    P[15][15]  = P[14][14];
    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];

    // optical flow ground height covariance
    Popt = 0.25f;

}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF::ForceSymmetry()
{
    for (uint8_t i=1; i<=21; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation and fix numerical errors
void NavEKF::CopyAndFixCovariances()
{
    // copy predicted variances
    for (uint8_t i=0; i<=21; i++) {
        P[i][i] = nextP[i][i];
    }
    // copy predicted covariances and force symmetry
    for (uint8_t i=1; i<=21; i++) {
        for (uint8_t j=0; j<=i-1; j++)
        {
            P[i][j] = 0.5f*(nextP[i][j] + nextP[j][i]);
            P[j][i] = P[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void NavEKF::ConstrainVariances()
{
    for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // quaternions
    for (uint8_t i=4; i<=6; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=7; i<=9; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f); // positions
    for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtIMU)); // delta angle biases
    P[13][13] = constrain_float(P[13][13],0.0f,sq(10.0f * dtIMU)); // delta velocity bias
    for (uint8_t i=14; i<=15; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // earth magnetic field
    for (uint8_t i=16; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // body magnetic field
}

// constrain states to prevent ill-conditioning
void NavEKF::ConstrainStates()
{
    // quaternions are limited between +-1
    for (uint8_t i=0; i<=3; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=4; i<=6; i++) states[i] = constrain_float(states[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i=7; i<=8; i++) states[i] = constrain_float(states[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    states[9] = constrain_float(states[9],-4.0e4f,1.0e4f);
    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers specs)
    for (uint8_t i=10; i<=12; i++) states[i] = constrain_float(states[i],-0.1f*dtIMU,0.1f*dtIMU);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    states[13] = constrain_float(states[13],-1.0f*dtIMU,1.0f*dtIMU);
    states[22] = constrain_float(states[22],-1.0f*dtIMU,1.0f*dtIMU);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=14; i<=15; i++) states[i] = constrain_float(states[i],-100.0f,100.0f);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) states[i] = constrain_float(states[i],-0.5f,0.5f);
}

// update IMU delta angle and delta velocity measurements
void NavEKF::readIMUData()
{
    Vector3f angRate;   // angular rate vector in XYZ body axes measured by the IMU (rad/s)
    Vector3f accel1;    // acceleration vector in XYZ body axes measured by IMU1 (m/s^2)
    Vector3f accel2;    // acceleration vector in XYZ body axes measured by IMU2 (m/s^2)

    // the imu sample time is sued as a common time reference throughout the filter
    imuSampleTime_ms = hal.scheduler->millis();

    // limit IMU delta time to prevent numerical problems elsewhere
    dtIMU = constrain_float(_ahrs->get_ins().get_delta_time(), 0.001f, 1.0f);

    // get accel and gyro data from dual sensors if healthy
    if (_ahrs->get_ins().get_accel_health(0) && _ahrs->get_ins().get_accel_health(1)) {
        accel1 = _ahrs->get_ins().get_accel(0);
        accel2 = _ahrs->get_ins().get_accel(1);
    } else {
        accel1 = _ahrs->get_ins().get_accel();
        accel2 = accel1;
    }

    // average the available gyro sensors
    angRate.zero();
    uint8_t gyro_count = 0;
    for (uint8_t i = 0; i<_ahrs->get_ins().get_gyro_count(); i++) {
        if (_ahrs->get_ins().get_gyro_health(i)) {
            angRate += _ahrs->get_ins().get_gyro(i);
            gyro_count++;
        }
    }
    if (gyro_count != 0) {
        angRate /= gyro_count;
    }

    // trapezoidal integration
    dAngIMU     = (angRate + lastAngRate) * dtIMU * 0.5f;
    lastAngRate = angRate;
    dVelIMU1    = (accel1 + lastAccel1) * dtIMU * 0.5f;
    lastAccel1  = accel1;
    dVelIMU2    = (accel2 + lastAccel2) * dtIMU * 0.5f;
    lastAccel2  = accel2;
}

// check for new valid GPS data and update stored measurement if available
void NavEKF::readGpsData()
{
    // check for new GPS data
    if ((_ahrs->get_gps().last_message_time_ms() != lastFixTime_ms) &&
            (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D))
    {
        // store fix time from previous read
        secondLastFixTime_ms = lastFixTime_ms;

        // get current fix time
        lastFixTime_ms = _ahrs->get_gps().last_message_time_ms();

        // set flag that lets other functions know that new GPS data has arrived
        newDataGps = true;

        // get state vectors that were stored at the time that is closest to when the the GPS measurement
        // time after accounting for measurement delays
        RecallStates(statesAtVelTime, (imuSampleTime_ms - constrain_int16(_msecVelDelay, 0, 500)));
        RecallStates(statesAtPosTime, (imuSampleTime_ms - constrain_int16(_msecPosDelay, 0, 500)));

        // read the NED velocity from the GPS
        velNED = _ahrs->get_gps().velocity();

        // check if we have enough GPS satellites and increase the gps noise scaler if we don't
        if (_ahrs->get_gps().num_sats() >= 6) {
            gpsNoiseScaler = 1.0f;
        } else if (_ahrs->get_gps().num_sats() == 5) {
            gpsNoiseScaler = 1.4f;
        } else { // <= 4 satellites
            gpsNoiseScaler = 2.0f;
        }

        // Check if GPS can output vertical velocity and set GPS fusion mode accordingly
        if (!_ahrs->get_gps().have_vertical_velocity()) {
            // vertical velocity should not be fused
            if (_fusionModeGPS == 0) {
                _fusionModeGPS = 1;
            }
        }

        // read latitutde and longitude from GPS and convert to local NE position relative to the stored origin
        // If we don't have an origin, then set it to the current GPS coordinates
        if (validOrigin) {
            const struct Location &gpsloc = _ahrs->get_gps().location();
            gpsPosNE = location_diff(EKF_origin, gpsloc);
        } else {
            setOrigin();
            gpsPosNE.zero();
        }
        // calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
        decayGpsOffset();
    }
    // If too long since last fix time or we are not allowed to use GPS, we declare no GPS available for use
    if ((imuSampleTime_ms - lastFixTime_ms < 1000) || _fusionModeGPS == 3) {
        gpsNotAvailable = false;
    } else {
        gpsNotAvailable = true;
    }
}

// check for new altitude measurement data and update stored measurement if available
void NavEKF::readHgtData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    if (_baro.get_last_update() != lastHgtMeasTime) {
        // time stamp used to check for new measurement
        lastHgtMeasTime = _baro.get_last_update();

        // time stamp used to check for timeout
        lastHgtTime_ms = imuSampleTime_ms;

        // get measurement and set flag to let other functions know new data has arrived
        hgtMea = _baro.get_altitude();
        newDataHgt = true;

        // get states that wer stored at the time closest to the measurement time, taking measurement delay into account
        RecallStates(statesAtHgtTime, (imuSampleTime_ms - msecHgtDelay));
    } else {
        newDataHgt = false;
    }
}

// check for new magnetometer data and update store measurements if available
void NavEKF::readMagData()
{
    if (use_compass() && _ahrs->get_compass()->last_update != lastMagUpdate) {
        // store time of last measurement update
        lastMagUpdate = _ahrs->get_compass()->last_update;

        // read compass data and scale to improve numerical conditioning
        magData = _ahrs->get_compass()->get_field() * 0.001f;

        // get states stored at time closest to measurement time after allowance for measurement delay
        RecallStates(statesAtMagMeasTime, (imuSampleTime_ms - msecMagDelay));

        // let other processes know that new compass data has arrived
        newDataMag = true;
    } else {
        newDataMag = false;
    }
}

// check for new airspeed data and update stored measurements if available
void NavEKF::readAirSpdData()
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available
    const AP_Airspeed *aspeed = _ahrs->get_airspeed();
    if (aspeed &&
        aspeed->use() &&
        aspeed->last_update_ms() != lastAirspeedUpdate) {
        VtasMeas = aspeed->get_airspeed() * aspeed->get_EAS2TAS();
        lastAirspeedUpdate = aspeed->last_update_ms();
        newDataTas = true;
        RecallStates(statesAtVtasMeasTime, (imuSampleTime_ms - msecTasDelay));
    } else {
        newDataTas = false;
    }
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, uint8_t &rangeHealth, float &rawSonarRange)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    flowQuality = rawFlowQuality;
    // recall angular rates averaged across flow observation period allowing for processing, transmission and intersample delays
    RecallOmega(omegaAcrossFlowTime, imuSampleTime_ms - flowTimeDeltaAvg_ms - _msecFLowDelay, imuSampleTime_ms - _msecFLowDelay);
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - omegaAcrossFlowTime.x),-0.1f,0.1f);
    flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - omegaAcrossFlowTime.y),-0.1f,0.1f);
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sesnor data)
    if (rawFlowQuality > 50 && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // recall vehicle states at mid sample time for flow observations allowing for delays
        RecallStates(statesAtFlowTime, imuSampleTime_ms - _msecFLowDelay - flowTimeDeltaAvg_ms/2);
        // calculate rotation matrices at mid sample time for flow observations
        statesAtFlowTime.quat.rotation_matrix(Tbn_flow);
        Tnb_flow = Tbn_flow.transposed();
        // correct flow sensor rates for bias
        
        omegaAcrossFlowTime.x = rawGyroRates.x - flowGyroBias.x;
        omegaAcrossFlowTime.y = rawGyroRates.y - flowGyroBias.y;
        // write uncorrected flow rate measurements that will be used by the focal length scale factor estimator
        // note correction for different axis and sign conventions used by the px4flow sensor
        flowRadXY[0] = - rawFlowRates.x; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        flowRadXY[1] = - rawFlowRates.y; // raw (non motion compensated) optical flow angular rate about the Y axis (rad/sec)
        // write flow rate measurements corrected for body rates
        flowRadXYcomp[0] = flowRadXY[0] + omegaAcrossFlowTime.x;
        flowRadXYcomp[1] = flowRadXY[1] + omegaAcrossFlowTime.y;
        // set flag that will trigger observations
        newDataFlow = true;
        flowValidMeaTime_ms = imuSampleTime_ms;
    } else {
        newDataFlow = false;
    }
    // Use range finder if 3 or more consecutive good samples. This reduces likelihood of using bad data.
    if (rangeHealth >= 3) {
        statesAtRngTime = statesAtFlowTime;
        rngMea = rawSonarRange;
        newDataRng = true;
    } else {
        newDataRng = false;
    }
}

// calculate the NED earth spin vector in rad/sec
void NavEKF::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
// if no magnetometer data, do not update magnetic field states and assume zero yaw angle
Quaternion NavEKF::calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    Quaternion initQuat;

    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = Tbn * magData;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;
        // calculate initial filter quaternion states using yaw from magnetometer if mag heading healthy
        // otherwise use existing heading
        if (!badMag) {
            initQuat.from_euler(roll, pitch, yaw);
        } else {
            initQuat = state.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        initMagNED = Tbn * magData;

        // write to earth magnetic field state vector
        state.earth_magfield = initMagNED;

        // clear bad magnetometer status
        badMag = false;
    } else {
        initQuat.from_euler(roll, pitch, 0.0f);
        yawAligned = false;
    }

    // return attitude quaternion
    return initQuat;
}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF::alignYawGPS()
{
    if ((sq(velNED[0]) + sq(velNED[1])) > 25.0f) {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        state.quat.to_euler(roll, pitch, oldYaw);
        // calculate course yaw angle
        oldYaw = atan2f(state.velocity.y,state.velocity.x);
        // calculate yaw angle from GPS velocity
        newYaw = atan2f(velNED[1],velNED[0]);
        // estimate the yaw error
        yawErr = wrap_PI(newYaw - oldYaw);
        // If the inertial course angle disagrees with the GPS by more than 45 degrees, we declare the compass as bad
        badMag = (fabsf(yawErr) > 0.7854f);
        // correct yaw angle using GPS ground course compass failed or if not previously aligned
        if (badMag || !yawAligned) {
            // correct the yaw angle
            newYaw = oldYaw + yawErr;
            // calculate new filter quaternion states from Euler angles
            state.quat.from_euler(roll, pitch, newYaw);
            // the yaw angle is now aligned so update its status
            yawAligned =  true;
            // reset the position and velocity states
            ResetPosition();
            ResetVelocity();
            // reset the covariance for the quaternion, velocity and position states
            // zero the matrix entries
            zeroRows(P,0,9);
            zeroCols(P,0,9);
            // quaternions - TODO maths that sets them based on different roll, yaw and pitch uncertainties
            P[0][0]   = 1.0e-9f;
            P[1][1]   = 0.25f*sq(radians(1.0f));
            P[2][2]   = 0.25f*sq(radians(1.0f));
            P[3][3]   = 0.25f*sq(radians(1.0f));
            // velocities - we could have a big error coming out of constant position mode due to GPS lag
            P[4][4]   = 400.0f;
            P[5][5]   = P[4][4];
            P[6][6]   = sq(0.7f);
            // positions - we could have a big error coming out of constant position mode due to GPS lag
            P[7][7]   = 400.0f;
            P[8][8]   = P[7][7];
            P[9][9]   = sq(5.0f);
        }
        // Update magnetic field states if the magnetometer is bad
        if (badMag) {
            calcQuatAndFieldStates(_ahrs->roll, _ahrs->pitch);
        }
    }
}

// This function is used to do a forced alignment of the wind velocity
// states so that they are set to the reciprocal of the ground speed
// and scaled to STARTUP_WIND_SPEED m/s. This is used when launching a
// fly-forward vehicle without an airspeed sensor on the assumption
// that launch will be into wind and STARTUP_WIND_SPEED is
// representative of typical launch wind
void NavEKF::setWindVelStates()
{
    float gndSpd = pythagorous2(state.velocity.x, state.velocity.y);
    if (gndSpd > 4.0f) {
        // set the wind states to be the reciprocal of the velocity and scale
        float scaleFactor = STARTUP_WIND_SPEED / gndSpd;
        state.wind_vel.x = - state.velocity.x * scaleFactor;
        state.wind_vel.y = - state.velocity.y * scaleFactor;
        // reinitialise the wind state covariances
        zeroRows(P,14,15);
        zeroCols(P,14,15);
        P[14][14] = 64.0f;
        P[15][15] = P[14][14];
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF::getRotationBodyToNED(Matrix3f &mat) const
{
    Vector3f trim = _ahrs->get_trim();
    state.quat.rotation_matrix(mat);
    mat.rotateXYinv(trim);
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void  NavEKF::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const
{
    velInnov.x = innovVelPos[0];
    velInnov.y = innovVelPos[1];
    velInnov.z = innovVelPos[2];
    posInnov.x = innovVelPos[3];
    posInnov.y = innovVelPos[4];
    posInnov.z = innovVelPos[5];
    magInnov.x = 1e3f*innovMag[0]; // Convert back to sensor units
    magInnov.y = 1e3f*innovMag[1]; // Convert back to sensor units
    magInnov.z = 1e3f*innovMag[2]; // Convert back to sensor units
    tasInnov   = innovVtas;
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
// this indicates the amount of margin available when tuning the various error traps
// also return the current offsets applied to the GPS position measurements
void  NavEKF::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    velVar   = sqrtf(velTestRatio);
    posVar   = sqrtf(posTestRatio);
    hgtVar   = sqrtf(hgtTestRatio);
    magVar.x = sqrtf(magTestRatio.x);
    magVar.y = sqrtf(magTestRatio.y);
    magVar.z = sqrtf(magTestRatio.z);
    tasVar   = sqrtf(tasTestRatio);
    offset   = gpsPosGlitchOffsetNE;
}

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF::InitialiseVariables()
{
    // initialise time stamps
    imuSampleTime_ms = hal.scheduler->millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    TASmsecPrev = imuSampleTime_ms;
    BETAmsecPrev = imuSampleTime_ms;
    lastMagUpdate = 0;
    lastHgtMeasTime = imuSampleTime_ms;
    lastHgtTime_ms = 0;
    lastAirspeedUpdate = 0;
    velFailTime = imuSampleTime_ms;
    posFailTime = imuSampleTime_ms;
    hgtFailTime = imuSampleTime_ms;
    tasFailTime = imuSampleTime_ms;
    lastStateStoreTime_ms = imuSampleTime_ms;
    lastFixTime_ms = 0;
    secondLastFixTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowUseTime_ms = imuSampleTime_ms;
    prevFlowFuseTime_ms = imuSampleTime_ms;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    velTimeout = true;
    posTimeout = true;
    hgtTimeout = true;
    magTimeout = true;
    tasTimeout = true;
    badMag = false;
    badIMUdata = false;
    firstArmComplete = false;
    finalMagYawInit = false;
    storeIndex = 0;
    dtIMU = 0;
    dt = 0;
    hgtMea = 0;
    firstArmPosD = 0.0f;
    storeIndex = 0;
    lastGyroBias.zero();
	prevDelAng.zero();
    lastAngRate.zero();
    lastAccel1.zero();
    lastAccel2.zero();
    velDotNEDfilt.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    velNED.zero();
    gpsPosGlitchOffsetNE.zero();
    lastKnownPositionNE.zero();
    gpsPosNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    memset(&storedStates[0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
    memset(&hgtIncrStateDelta[0], 0, sizeof(hgtIncrStateDelta));
    memset(&magIncrStateDelta[0], 0, sizeof(magIncrStateDelta));
    memset(&flowIncrStateDelta[0], 0, sizeof(flowIncrStateDelta));
    newDataFlow = false;
    flowDataValid = false;
    newDataRng  = false;
    flowFusePerformed = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = gpsPosNE.x;
    prevPosE = gpsPosNE.y;
    fuseRngData = false;
    inhibitGndState = true;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    constVelMode = false;
    lastConstVelMode = false;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    gpsVelGlitchOffset.zero();
    vehicleArmed = false;
    prevVehicleArmed = false;
    constPosMode = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    IMU1_weighting = 0.5f;
    onGround = true;
    manoeuvring = false;
    yawAligned = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    flowXfailed = false;
    validOrigin = false;
}

// return true if we should use the airspeed sensor
bool NavEKF::useAirspeed(void) const
{
    if (_ahrs->get_airspeed() == NULL) {
        return false;
    }
    return _ahrs->get_airspeed()->use();
}

// return true if we should use the range finder sensor
bool NavEKF::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

// return true if optical flow data is available
bool NavEKF::optFlowDataPresent(void) const
{
    if (imuSampleTime_ms - flowMeaTime_ms < 5000) {
        return true;
    } else {
        return false;
    }
}

// return true if the vehicle is requesting the filter to be ready for flight
bool NavEKF::getVehicleArmStatus(void) const
{
    return _ahrs->get_armed() || _ahrs->get_correct_centrifugal();
}

// return true if we should use the compass
bool NavEKF::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

// decay GPS horizontal position offset to close to zero at a rate of 1 m/s for copters and 5 m/s for planes
// limit radius to a maximum of 100m
void NavEKF::decayGpsOffset()
{
    float offsetDecaySpd;
    if (assume_zero_sideslip()) {
        offsetDecaySpd = 5.0f;
    } else {
        offsetDecaySpd = 1.0f;
    }
    float lapsedTime = 0.001f*float(imuSampleTime_ms - lastDecayTime_ms);
    lastDecayTime_ms = imuSampleTime_ms;
    float offsetRadius = pythagorous2(gpsPosGlitchOffsetNE.x, gpsPosGlitchOffsetNE.y);
    // decay radius if larger than offset decay speed multiplied by lapsed time (plus a margin to prevent divide by zero)
    if (offsetRadius > (offsetDecaySpd * lapsedTime + 0.1f)) {
        // Calculate the GPS velocity offset required. This is necessary to prevent the position measurement being rejected for inconsistency when the radius is being pulled back in.
        gpsVelGlitchOffset = -gpsPosGlitchOffsetNE*offsetDecaySpd/offsetRadius;
        // calculate scale factor to be applied to both offset components
        float scaleFactor = constrain_float((offsetRadius - offsetDecaySpd * lapsedTime), 0.0f, 100.0f) / offsetRadius;
        gpsPosGlitchOffsetNE.x *= scaleFactor;
        gpsPosGlitchOffsetNE.y *= scaleFactor;
    } else {
        gpsVelGlitchOffset.zero();
        gpsPosGlitchOffsetNE.zero();
    }
}

/*
  should we assume zero sideslip?
 */
bool NavEKF::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return _ahrs->get_fly_forward() && _ahrs->get_vehicle_class() != AHRS_VEHICLE_GROUND;
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
void  NavEKF::getFilterFaults(uint8_t &faults) const
{
    faults = (state.quat.is_nan()<<0 |
              state.velocity.is_nan()<<1 |
              faultStatus.bad_xmag<<2 |
              faultStatus.bad_ymag<<3 |
              faultStatus.bad_zmag<<4 |
              faultStatus.bad_airspeed<<5 |
              faultStatus.bad_sideslip<<6 |
              !statesInitialised<<7);
}

/*
return filter timeout status as a bitmasked integer
 0 = position measurement timeout
 1 = velocity measurement timeout
 2 = height measurement timeout
 3 = magnetometer measurement timeout
 4 = true airspeed measurement timeout
 5 = unassigned
 6 = unassigned
 7 = unassigned
*/
void  NavEKF::getFilterTimeouts(uint8_t &timeouts) const
{
    timeouts = (posTimeout<<0 |
                velTimeout<<1 |
                hgtTimeout<<2 |
                magTimeout<<3 |
                tasTimeout<<4);
}

/*
return filter function status as a bitmasked integer
 0 = attitude estimate valid
 1 = horizontal velocity estimate valid
 2 = vertical velocity estimate valid
 3 = relative horizontal position estimate valid
 4 = absolute horizontal position estimate valid
 5 = vertical position estimate valid
 6 = terrain height estimate valid
 7 = constant position mode
*/
void  NavEKF::getFilterStatus(nav_filter_status &status) const
{
    // init return value
    status.value = 0;

    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool notDeadReckoning = !constVelMode && !constPosMode;
    bool someVertRefData = (!velTimeout && (_fusionModeGPS == 0)) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && (_fusionModeGPS == 3);
    bool gpsNavPossible = !gpsNotAvailable && (_fusionModeGPS <= 2);

    // set individual flags
    status.flags.attitude = !state.quat.is_nan();   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && notDeadReckoning;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData;        // vertical velocity estimate valid
    status.flags.horiz_pos_rel = (doingFlowNav || doingWindRelNav || doingNormalGpsNav) && notDeadReckoning;   // relative horizontal position estimate valid
    status.flags.horiz_pos_abs = doingNormalGpsNav && notDeadReckoning; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout;            // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid;		// terrain height estimate valid
    status.flags.const_pos_mode = constPosMode;     // constant position mode
    status.flags.pred_horiz_pos_rel = optFlowNavPossible || gpsNavPossible; // we should be able to estimate a relative position when we enter flight mode
    status.flags.pred_horiz_pos_abs = gpsNavPossible; // we should be able to estimate an absolute position when we enter flight mode
}

// Check arm status and perform required checks and mode changes
void NavEKF::performArmingChecks()
{
    // determine vehicle arm status and don't allow filter to arm until it has been running for long enough to stabilise
    prevVehicleArmed = vehicleArmed;
    vehicleArmed = (getVehicleArmStatus() && (imuSampleTime_ms - ekfStartTime_ms) > 10000);

    // check to see if arm status has changed and reset states if it has
    if (vehicleArmed != prevVehicleArmed) {
        // only reset the magnetic field and heading on the first arm. This prevents in-flight learning being forgotten for vehicles that do multiple short flights and disarm in-between.
        if (vehicleArmed && !firstArmComplete) {
            firstArmComplete = true;
            state.quat = calcQuatAndFieldStates(_ahrs->roll, _ahrs->pitch);
            firstArmPosD = state.position.z;
        }
        // zero stored velocities used to do dead-reckoning
        heldVelNE.zero();
        // zero last known position used to deadreckon if GPS is lost when arming
        // keep position during disarm to provide continuing indication of last known position
        if (vehicleArmed) {
            lastKnownPositionNE.zero();
        }
        // set various  useage modes based on the condition at arming. These are then held until the vehicle is disarmed.
        if (!vehicleArmed) {
            PV_AidingMode = AID_NONE; // When dis-armed, we only estimate orientation & height using the constant position mode
            constPosMode = true;
            constVelMode = false; // always clear constant velocity mode if constant position mode is active
            lastConstVelMode = false;
        } else if (_fusionModeGPS == 3) { // arming when GPS useage has been prohibited
            if (optFlowDataPresent()) {
                PV_AidingMode = AID_RELATIVE; // we have optical flow data and can estimate all vehicle states
                posTimeout = true;
                velTimeout = true;
            } else {
                PV_AidingMode = AID_NONE; // we don't have optical flow data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
            }
        } else { // arming when GPS useage is allowed
            if (gpsNotAvailable) {
                PV_AidingMode = AID_NONE; // we don't have have GPS data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
            } else {
                PV_AidingMode = AID_ABSOLUTE; // we have GPS data and can estimate all vehicle states
            }
        }
        // Reset filter positon, height and velocity states on arming or disarming
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        StoreStatesReset();

    } else if (vehicleArmed && !finalMagYawInit && firstArmPosD - state.position.z > 1.5f && !assume_zero_sideslip()) {
        // Do a final yaw and earth mag field initialisation when the vehicle has gained 1.5m of altitude after arming if it is a non-fly forward vehicle (vertical takeoff)
        state.quat = calcQuatAndFieldStates(_ahrs->roll, _ahrs->pitch);
        finalMagYawInit = true;
    }

    // set constant position mode if gps is inhibited and we are not trying to use optical flow data
    if (PV_AidingMode == AID_NONE) {
        constPosMode = true;
        constVelMode = false; // always clear constant velocity mode if constant position mode is active
        lastConstVelMode = false;
    } else {
        constPosMode = false;
    }

}

// Set the NED origin to be used until the next filter reset
void NavEKF::setOrigin()
{
    EKF_origin = _ahrs->get_gps().location();
    validOrigin = true;
}

// return the LLH location of the filters NED origin
bool NavEKF::getOriginLLH(struct Location &loc) const
{
    if (validOrigin) {
        loc = EKF_origin;
    }
    return validOrigin;
}

// set the LLH location of the filters NED origin
bool NavEKF::setOriginLLH(struct Location &loc)
{
    if (vehicleArmed) {
        return false;
    }
    EKF_origin = loc;
    validOrigin = true;
    return true;
}

#endif // HAL_CPU_CLASS
