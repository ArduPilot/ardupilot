/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2_core.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

/*
  parameter defaults for different types of vehicle. The
  APM_BUILD_DIRECTORY is taken from the main vehicle directory name
  where the code is built. Note that this trick won't work for arduino
  builds on APM2, but NavEKF2 doesn't run on APM2, so that's OK
 */
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// copter defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     1.0f
#define ALT_NOISE_DEFAULT       1.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.005f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    7.0E-05f
#define ABIAS_PNOISE_DEFAULT    1.0E-04f
#define MAG_PNOISE_DEFAULT      2.0E-03f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        5
#define HGT_GATE_DEFAULT        5
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_NOISE_DEFAULT      0.25f
#define FLOW_GATE_DEFAULT       3
#define GSCALE_PNOISE_DEFAULT   1.5E-03f

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
// rover defaults
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     1.0f
#define ALT_NOISE_DEFAULT       1.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.005f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    7.0E-05f
#define ABIAS_PNOISE_DEFAULT    1.0E-04f
#define MAG_PNOISE_DEFAULT      2.0E-03f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        5
#define HGT_GATE_DEFAULT        5
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_NOISE_DEFAULT      0.25f
#define FLOW_GATE_DEFAULT       3
#define GSCALE_PNOISE_DEFAULT   1.5E-03f

#else
// generic defaults (and for plane)
#define VELNE_NOISE_DEFAULT     0.5f
#define VELD_NOISE_DEFAULT      0.7f
#define POSNE_NOISE_DEFAULT     1.0f
#define ALT_NOISE_DEFAULT       1.5f
#define MAG_NOISE_DEFAULT       0.05f
#define GYRO_PNOISE_DEFAULT     0.005f
#define ACC_PNOISE_DEFAULT      0.25f
#define GBIAS_PNOISE_DEFAULT    7.0E-05f
#define ABIAS_PNOISE_DEFAULT    1.0E-04f
#define MAG_PNOISE_DEFAULT      2.0E-03f
#define VEL_GATE_DEFAULT        5
#define POS_GATE_DEFAULT        5
#define HGT_GATE_DEFAULT        10
#define MAG_GATE_DEFAULT        3
#define MAG_CAL_DEFAULT         3
#define GLITCH_RADIUS_DEFAULT   25
#define FLOW_MEAS_DELAY         10
#define FLOW_NOISE_DEFAULT      0.25f
#define FLOW_GATE_DEFAULT       3
#define GSCALE_PNOISE_DEFAULT   1.5E-03f

#endif // APM_BUILD_DIRECTORY

// Define tuning parameters
const AP_Param::GroupInfo NavEKF2::var_info[] PROGMEM = {

    // @Param: ENABLE
    // @DisplayName: Enable EKF2
    // @Description: This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_USE=3
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, NavEKF2, _enable, 0),

    // GPS measurement parameters

    // @Param: GPS_TYPE
    // @DisplayName: GPS mode control
    // @Description: This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
    // @Values: 0:GPS 3D Vel and 2D Pos, 1:GPS 2D vel and 2D pos, 2:GPS 2D pos, 3:No GPS use optical flow
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE", 1, NavEKF2, _fusionModeGPS, 0),

    // @Param: VELNE_NOISE
    // @DisplayName: GPS horizontal velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELNE_NOISE", 2, NavEKF2, _gpsHorizVelNoise, VELNE_NOISE_DEFAULT),

    // @Param: VELD_NOISE
    // @DisplayName: GPS vertical velocity measurement noise scaler
    // @Description: This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed acuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
    // @Range: 0.05 5.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("VELD_NOISE", 3, NavEKF2, _gpsVertVelNoise, VELD_NOISE_DEFAULT),

    // @Param: VEL_GATE
    // @DisplayName: GPS velocity measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL_GATE", 4, NavEKF2, _gpsVelInnovGate, VEL_GATE_DEFAULT),

    // @Param: POSNE_NOISE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("POSNE_NOISE", 5, NavEKF2, _gpsHorizPosNoise, POSNE_NOISE_DEFAULT),

    // @Param: POS_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POS_GATE", 6, NavEKF2, _gpsPosInnovGate, POS_GATE_DEFAULT),

    // @Param: GLITCH_RAD
    // @DisplayName: GPS glitch radius gate size (m)
    // @Description: This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and an offset is applied to the GPS measurement to compensate. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
    // @Range: 10 50
    // @Increment: 5
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("GLITCH_RAD", 7, NavEKF2, _gpsGlitchRadiusMax, GLITCH_RADIUS_DEFAULT),

    // @Param: GPS_DELAY
    // @DisplayName: GPS measurement delay (msec)
    // @Description: This is the number of msec that the GPS measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("VEL_DELAY", 8, NavEKF2, _gpsDelay_ms, 220),

    // Height measurement parameters

    // @Param: ALT_SOURCE
    // @DisplayName: Primary height source
    // @Description: This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will casue it to use range finder if available.
    // @Values: 0:Use Baro, 1:Use Range Finder
    // @User: Advanced
    AP_GROUPINFO("ALT_SOURCE", 9, NavEKF2, _altSource, 1),

    // @Param: ALT_NOISE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("ALT_NOISE", 10, NavEKF2, _baroAltNoise, ALT_NOISE_DEFAULT),

    // @Param: HGT_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("HGT_GATE", 11, NavEKF2, _hgtInnovGate, HGT_GATE_DEFAULT),

    // @Param: HGT_DELAY
    // @DisplayName: Height measurement delay (msec)
    // @Description: This is the number of msec that the Height measurements lag behind the inertial measurements.
    // @Range: 0 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("HGT_DELAY", 12, NavEKF2, _hgtDelay_ms, 60),

    // Magnetometer measurement parameters

    // @Param: MAG_NOISE
    // @DisplayName: Magnetometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("MAG_NOISE", 13, NavEKF2, _magNoise, MAG_NOISE_DEFAULT),

    // @Param: MAG_CAL
    // @DisplayName: Magnetometer calibration mode
    // @Description: EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
    // @Values: 0:Speed and Height,1:Acceleration,2:Never,3:Always
    // @User: Advanced
    AP_GROUPINFO("MAG_CAL", 14, NavEKF2, _magCal, MAG_CAL_DEFAULT),

    // @Param: MAG_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAG_GATE", 15, NavEKF2, _magInnovGate, MAG_GATE_DEFAULT),

    // Airspeed measurement parameters

    // @Param: EAS_NOISE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: m/s
    AP_GROUPINFO("EAS_NOISE", 16, NavEKF2, _easNoise, 1.4f),

    // @Param: EAS_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EAS_GATE", 17, NavEKF2, _tasInnovGate, 10),

    // Rangefinder measurement parameters

    // @Param: RNG_NOISE
    // @DisplayName: Range finder measurement noise (m)
    // @Description: This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("RNG_NOISE", 18, NavEKF2, _rngNoise, 0.5f),

    // @Param: RNG_GATE
    // @DisplayName: Range finder measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RNG_GATE", 19, NavEKF2, _rngInnovGate, 5),

    // @Param: MAX_FLOW
    // @DisplayName: Maximum valid optical flow rate
    // @Description: This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
    // @Range: 1.0 - 4.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MAX_FLOW", 20, NavEKF2, _maxFlowRate, 2.5f),

    // Optical flow measurement parameters

    // @Param: FLOW_NOISE
    // @DisplayName: Optical flow measurement noise (rad/s)
    // @Description: This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 - 1.0
    // @Increment: 0.05
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("FLOW_NOISE", 21, NavEKF2, _flowNoise, FLOW_NOISE_DEFAULT),

    // @Param: FLOW_GATE
    // @DisplayName: Optical Flow measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLOW_GATE", 22, NavEKF2, _flowInnovGate, FLOW_GATE_DEFAULT),

    // @Param: FLOW_DELAY
    // @DisplayName: Optical Flow measurement delay (msec)
    // @Description: This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: Advanced
    // @Units: milliseconds
    AP_GROUPINFO("FLOW_DELAY", 23, NavEKF2, _flowDelay_ms, FLOW_MEAS_DELAY),

    // State and Covariance Predition Parameters

    // @Param: GYRO_PNOISE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.001 0.05
    // @Increment: 0.001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GYRO_PNOISE", 24, NavEKF2, _gyrNoise, GYRO_PNOISE_DEFAULT),

    // @Param: ACC_PNOISE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.05 1.0
    // @Increment: 0.01
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ACC_PNOISE", 25, NavEKF2, _accNoise, ACC_PNOISE_DEFAULT),

    // @Param: GBIAS_PNOISE
    // @DisplayName: Rate gyro bias process noise (rad/s)
    // @Description: This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: rad/s
    AP_GROUPINFO("GBIAS_PNOISE", 26, NavEKF2, _gyroBiasProcessNoise, GBIAS_PNOISE_DEFAULT),

    // @Param: GSCL_PNOISE
    // @DisplayName: Rate gyro scale factor process noise (1/s)
    // @Description: This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
    // @Range: 0.0000001 0.00001
    // @User: Advanced
    // @Units: 1/s
    AP_GROUPINFO("GSCL_PNOISE", 27, NavEKF2, _gyroScaleProcessNoise, GSCALE_PNOISE_DEFAULT),

    // @Param: ABIAS_PNOISE
    // @DisplayName: Accelerometer bias process noise (m/s^2)
    // @Description: This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.00001 0.001
    // @User: Advanced
    // @Units: m/s/s
    AP_GROUPINFO("ABIAS_PNOISE", 28, NavEKF2, _accelBiasProcessNoise, ABIAS_PNOISE_DEFAULT),

    // @Param: MAG_PNOISE
    // @DisplayName: Magnetic field process noise (gauss/s)
    // @Description: This noise controls the growth of magnetic field state error estimates. Increasing it makes magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 0.01
    // @User: Advanced
    // @Units: gauss/s
    AP_GROUPINFO("MAG_PNOISE", 29, NavEKF2, _magProcessNoise, MAG_PNOISE_DEFAULT),

    // @Param: WIND_PNOISE
    // @DisplayName: Wind velocity process noise (m/s^2)
    // @Description: This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PNOISE", 30, NavEKF2, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Height rate to wind procss noise scaler
    // @Description: Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
    // @Range: 0.0 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("WIND_PSCALE", 31, NavEKF2, _wndVarHgtRateScale, 0.5f),

    // @Param: GPS_CHECK
    // @DisplayName: GPS preflight check
    // @Description: 1 byte bitmap of GPS preflight checks to perform. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
    // @Bitmask: 0:NSats,1:HDoP,2:speed error,3:horiz pos error,4:yaw error,5:pos drift,6:vert speed,7:horiz speed
    // @User: Advanced
    AP_GROUPINFO("GPS_CHECK",    32, NavEKF2, _gpsCheck, 31),
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
    magVarRateScale(0.05f),         // scale factor applied to magnetometer variance due to angular rate
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    hgtAvg_ms(100),                 // average number of msec between height measurements
    betaAvg_ms(100),                // average number of msec between synthetic sideslip measurements
    covTimeStepMax(0.07f),          // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),      // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f),      // scaler applied to the barometer observation variance when operating in ground effect
    gndGradientSigma(2)             // RMS terrain gradient percentage assumed by the terrain height estimation
{
    AP_Param::setup_object_defaults(this, var_info);
}


// Initialise the filter
bool NavEKF2::InitialiseFilter(void)
{
    if (_enable == 0) {
        return false;
    }
    if (core == nullptr) {
        core = new NavEKF2_core(*this, _ahrs, _baro, _rng);
        if (core == nullptr) {
            _enable.set(0);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, PSTR("NavEKF2: allocation failed"));
            return false;
        }
    }
    return core->InitialiseFilterBootstrap();
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2::UpdateFilter(void)
{
    if (core) {
        core->UpdateFilter();
    }
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF2::healthy(void) const
{
    if (!core) {
        return false;
    }
    return core->healthy();
}

// Return the last calculated NED position relative to the reference point (m).
// If a calculated solution is not available, use the best available data and return false
// If false returned, do not use for flight control
bool NavEKF2::getPosNED(Vector3f &pos) const
{
    if (!core) {
        return false;
    }
    return core->getPosNED(pos);
}

// return NED velocity in m/s
void NavEKF2::getVelNED(Vector3f &vel) const
{
    if (core) {
        core->getVelNED(vel);
    }
}

// This returns the specific forces in the NED frame
void NavEKF2::getAccelNED(Vector3f &accelNED) const
{
    if (core) {
        core->getAccelNED(accelNED);
    }
}

// return body axis gyro bias estimates in rad/sec
void NavEKF2::getGyroBias(Vector3f &gyroBias) const
{
    if (core) {
        core->getGyroBias(gyroBias);
    }
}

// return body axis gyro scale factor error as a percentage
void NavEKF2::getGyroScaleErrorPercentage(Vector3f &gyroScale) const
{
    if (core) {
        core->getGyroScaleErrorPercentage(gyroScale);
    }
}

// return tilt error convergence metric
void NavEKF2::getTiltError(float &ang) const
{
    if (core) {
        core->getTiltError(ang);
    }
}

// reset body axis gyro bias estimates
void NavEKF2::resetGyroBias(void)
{
    if (core) {
        core->resetGyroBias();
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
    return core->resetHeightDatum();
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
    return core->setInhibitGPS();
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF2::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (core) {
        core->getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    }
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF2::getAccelZBias(float &zbias) const
{
    if (core) {
        core->getAccelZBias(zbias);
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF2::getWind(Vector3f &wind) const
{
    if (core) {
        core->getWind(wind);
    }
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF2::getMagNED(Vector3f &magNED) const
{
    if (core) {
        core->getMagNED(magNED);
    }
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF2::getMagXYZ(Vector3f &magXYZ) const
{
    if (core) {
        core->getMagXYZ(magXYZ);
    }
}

// Return estimated magnetometer offsets
// Return true if magnetometer offsets are valid
bool NavEKF2::getMagOffsets(Vector3f &magOffsets) const
{
    if (!core) {
        return false;
    }
    return core->getMagOffsets(magOffsets);
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
    return core->getLLH(loc);
}

// return the latitude and longitude and height used to set the NED origin
// All NED positions calculated by the filter are relative to this location
// Returns false if the origin has not been set
bool NavEKF2::getOriginLLH(struct Location &loc) const
{
    if (!core) {
        return false;
    }
    return core->getOriginLLH(loc);
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
    return core->setOriginLLH(loc);
}

// return estimated height above ground level
// return false if ground height is not being estimated.
bool NavEKF2::getHAGL(float &HAGL) const
{
    if (!core) {
        return false;
    }
    return core->getHAGL(HAGL);
}

// return the Euler roll, pitch and yaw angle in radians
void NavEKF2::getEulerAngles(Vector3f &eulers) const
{
    if (core) {
        core->getEulerAngles(eulers);
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF2::getRotationBodyToNED(Matrix3f &mat) const
{
    if (core) {
        core->getRotationBodyToNED(mat);
    }
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF2::getQuaternion(Quaternion &quat) const
{
    if (core) {
        core->getQuaternion(quat);
    }
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void NavEKF2::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    if (core) {
        core->getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
void NavEKF2::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    if (core) {
        core->getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }
}

// should we use the compass? This is public so it can be used for
// reporting via ahrs.use_compass()
bool NavEKF2::use_compass(void) const
{
    if (!core) {
        return false;
    }
    return core->use_compass();
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
        core->writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    }
}

// return data for debugging optical flow fusion
void NavEKF2::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov,
                           float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    if (core) {
        core->getFlowDebug(varFlow, gndOffset, flowInnovX, flowInnovY, auxInnov, HAGL, rngInnov, range, gndOffsetErr);
    }
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTakeoffExpected(bool val)
{
    if (core) {
        core->setTakeoffExpected(val);
    }
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2::setTouchdownExpected(bool val)
{
    if (core) {
        core->setTouchdownExpected(val);
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
void NavEKF2::getFilterFaults(uint8_t &faults) const
{
    if (core) {
        core->getFilterFaults(faults);
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
void NavEKF2::getFilterTimeouts(uint8_t &timeouts) const
{
    if (core) {
        core->getFilterTimeouts(timeouts);
    }
}

/*
  return filter status flags
*/
void NavEKF2::getFilterStatus(nav_filter_status &status) const
{
    if (core) {
        core->getFilterStatus(status);
    }
}

/*
return filter gps quality check status
*/
void  NavEKF2::getFilterGpsStatus(nav_gps_status &status) const
{
    if (core) {
        core->getFilterGpsStatus(status);
    }
}

// send an EKF_STATUS_REPORT message to GCS
void NavEKF2::send_status_report(mavlink_channel_t chan)
{
    if (core) {
        core->send_status_report(chan);
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
    return core->getHeightControlLimit(height);
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF2::getLastYawResetAngle(float &yawAng)
{
    if (!core) {
        return false;
    }
    return core->getLastYawResetAngle(yawAng);
}

#endif //HAL_CPU_CLASS
