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

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// Define tuning parameters
const AP_Param::GroupInfo NavEKF::var_info[] PROGMEM = {

    // @Param: VELNE_NOISE
    // @DisplayName: GPS horizontal velocity measurement noise (m/s)
    // @Description: This is the RMS value of noise in the North and East GPS velocity measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.05 - 5.0
    // @Increment: 0.05
    // @User: advanced
    AP_GROUPINFO("VELNE_NOISE",    0, NavEKF, _gpsHorizVelNoise, 0.30f),

    // @Param: VELD_NOISE
    // @DisplayName: GPS vertical velocity measurement noise (m/s)
    // @Description: This is the RMS value of noise in the vertical GPS velocity measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.05 - 5.0
    // @Increment: 0.05
    // @User: advanced
    AP_GROUPINFO("VELD_NOISE",    1, NavEKF, _gpsVertVelNoise, 0.30f),

    // @Param: POSNE_NOISE
    // @DisplayName: GPS horizontal position measurement noise (m)
    // @Description: This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.1 - 10.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("POSNE_NOISE",    2, NavEKF, _gpsHorizPosNoise, 0.50f),

    // @Param: ALT_NOISE
    // @DisplayName: Altitude measurement noise (m)
    // @Description: This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
    // @Range: 0.1 - 10.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("ALT_NOISE",    3, NavEKF, _baroAltNoise, 0.50f),

    // @Param: MAG_NOISE
    // @DisplayName: Magntometer measurement noise (Gauss)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.01 - 0.5
    // @Increment: 0.01
    // @User: advanced
    AP_GROUPINFO("MAG_NOISE",    4, NavEKF, _magNoise, 0.05f),

    // @Param: EAS_NOISE
    // @DisplayName: Equivalent airspeed measurement noise (m/s)
    // @Description: This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
    // @Range: 0.5 - 5.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("EAS_NOISE",    5, NavEKF, _easNoise, 1.4f),

    // @Param: WIND_PNOISE
    // @DisplayName: Wind velocity states process noise (m/s^2)
    // @Description: This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
    // @Range: 0.01 - 1.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("WIND_PNOISE",    6, NavEKF, _windVelProcessNoise, 0.1f),

    // @Param: WIND_PSCALE
    // @DisplayName: Scale factor applied to wind states process noise from height rate
    // @Description: Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
    // @Range: 0.0 - 1.0
    // @Increment: 0.1
    // @User: advanced
    AP_GROUPINFO("WIND_PSCALE",    7, NavEKF, _wndVarHgtRateScale, 0.5f),

    // @Param: GYRO_PNOISE
    // @DisplayName: Rate gyro noise (rad/s)
    // @Description: This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
    // @Range: 0.001 - 0.05
    // @Increment: 0.001
    // @User: advanced
    AP_GROUPINFO("GYRO_PNOISE",    8, NavEKF, _gyrNoise, 0.015f),

    // @Param: ACC_PNOISE
    // @DisplayName: Accelerometer noise (m/s^2)
    // @Description: This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
    // @Range: 0.05 - 1.0    AP_Float _gpsNEVelVarAccScale;  // scale factor applied to NE velocity measurement variance due to Vdot
    // @Increment: 0.01
    // @User: advanced
    AP_GROUPINFO("ACC_PNOISE",    9, NavEKF, _accNoise, 0.25f),

    // @Param: GBIAS_PNOISE
    // @DisplayName: Rate gyro bias state process noise (rad/s)
    // @Description: This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
    // @Range: 0.0000001 - 0.00001
    // @User: advanced
    AP_GROUPINFO("GBIAS_PNOISE",    10, NavEKF, _gyroBiasProcessNoise, 1.0e-7f),

    // @Param: ABIAS_PNOISE
    // @DisplayName: Accelerometer bias state process noise (m/s^2)
    // @Description: This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
    // @Range: 0.0001 - 0.01
    // @User: advanced
    AP_GROUPINFO("ABIAS_PNOISE",    11, NavEKF, _accelBiasProcessNoise, 1.0e-4f),

    // @Param: MAGE_PNOISE
    // @DisplayName: Earth magnetic field states process noise (gauss/s)
    // @Description: This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier.
    // @Range: 0.0001 - 0.01
    // @User: advanced
    AP_GROUPINFO("MAGE_PNOISE",    12, NavEKF, _magEarthProcessNoise, 3.0e-4f),

    // @Param: MAGB_PNOISE
    // @DisplayName: Body magnetic field states process noise (gauss/s)
    // @Description: This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier.
    // @Range: 0.0001 - 0.01
    // @User: advanced
    AP_GROUPINFO("MAGB_PNOISE",    13, NavEKF, _magBodyProcessNoise, 3.0e-4f),

    // @Param: VEL_DELAY
    // @DisplayName: GPS velocity measurement delay (msec)
    // @Description: This is the number of msec that the GPS velocity measurements lag behind the inertial measurements.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: advanced
    AP_GROUPINFO("VEL_DELAY",    14, NavEKF, _msecVelDelay, 220),

    // @Param: POS_DELAY
    // @DisplayName: GPS position measurement delay (msec)
    // @Description: This is the number of msec that the GPS position measurements lag behind the inertial measurements.
    // @Range: 0 - 500
    // @Increment: 10
    // @User: advancedScale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    AP_GROUPINFO("POS_DELAY",    15, NavEKF, _msecPosDelay, 220),

    // @Param: GPS_TYPE
    // @DisplayName: GPS velocity mode control
    // @Description: This parameter controls use of GPS velocity measurements : 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    // @Range: 0 - 3
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("GPS_TYPE",    16, NavEKF, _fusionModeGPS, 0),

    // @Param: VEL_GATE
    // @DisplayName: GPS velocity measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("VEL_GATE",    17, NavEKF, _gpsVelInnovGate, 5),

    // @Param: POS_GATE
    // @DisplayName: GPS position measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("POS_GATE",    18, NavEKF, _gpsPosInnovGate, 10),

    // @Param: HGT_GATE
    // @DisplayName: Height measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("HGT_GATE",    19, NavEKF, _hgtInnovGate, 10),

    // @Param: MAG_GATE
    // @DisplayName: Magnetometer measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("MAG_GATE",    20, NavEKF, _magInnovGate, 5),

    // @Param: EAS_GATE
    // @DisplayName: Airspeed measurement gate size
    // @Description: This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
    // @Range: 1 - 100
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("EAS_GATE",    21, NavEKF, _tasInnovGate, 10),

    // @Param: MAG_CAL
    // @DisplayName: Turns on magnetometer calibration mode
    // @Description: Setting this parameter to 1 forces magnetic field state calibration to be active all the time the vehicle is manoeuvring regardless of its speed and altitude. This parameter should be set to 0 for aircraft use. This parameter can be set to 1 to enable in-flight compass calibration on Copter and Rover vehicles.
    // @Range: 0 - 1
    // @Increment: 1
    // @User: advanced
    AP_GROUPINFO("MAG_CAL",    22, NavEKF, _magCal, 0),

    AP_GROUPEND
};


// constructor
NavEKF::NavEKF(const AP_AHRS *ahrs, AP_Baro &baro) :
    _ahrs(ahrs),
    _baro(baro),
    state(*reinterpret_cast<struct state_elements *>(&states)),
    covTimeStepMax(0.07f),      // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),        // maximum delta angle between covariance prediction updates
    TASmsecMax(200),            // maximum allowed interval between airspeed measurement updates
    fuseMeNow(false),           // forces airspeed and sythetic sideslip fusion to occur on the IMU frame that data arrives
    staticMode(true),           // staticMode forces position and velocity fusion with zero values
    prevStaticMode(true),       // staticMode from previous filter update
    yawAligned(false)           // set true when heading or yaw angle has been aligned

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    ,_perf_UpdateFilter(perf_alloc(PC_ELAPSED, "EKF_UpdateFilter")),
    _perf_CovariancePrediction(perf_alloc(PC_ELAPSED, "EKF_CovariancePrediction")),
    _perf_FuseVelPosNED(perf_alloc(PC_ELAPSED, "EKF_FuseVelPosNED")),
    _perf_FuseMagnetometer(perf_alloc(PC_ELAPSED, "EKF_FuseMagnetometer")),
    _perf_FuseAirspeed(perf_alloc(PC_ELAPSED, "EKF_FuseAirspeed")),
    _perf_FuseSideslip(perf_alloc(PC_ELAPSED, "EKF_FuseSideslip"))
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
    // Tuning parameters
    _gpsNEVelVarAccScale    = 0.05f;     // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    _gpsDVelVarAccScale     = 0.07f;    // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    _gpsPosVarAccScale      = 0.05f;     // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    _msecHgtDelay           = 60;       // Height measurement delay (msec)
    _msecMagDelay           = 40;       // Magnetometer measurement delay (msec)
    _msecTasDelay           = 240;      // Airspeed measurement delay (msec)
    _gpsRetryTimeUseTAS     = 10000;    // GPS retry time with airspeed measurements (msec)
    _gpsRetryTimeNoTAS      = 5000;     // GPS retry time without airspeed measurements (msec)
    _hgtRetryTimeMode0      = 10000;    // Height retry time with vertical velocity measurement (msec)
    _hgtRetryTimeMode12     = 5000;     // Height retry time without vertical velocity measurement (msec)
    _magVarRateScale        = 0.05f;    // scale factor applied to magnetometer variance due to angular rate
    _gyroBiasNoiseScaler    = 3.0f;     // scale factor applied to gyro bias state process variance when on ground
    _msecGpsAvg             = 200;      // average number of msec between GPS measurements
    _msecHgtAvg             = 100;      // average number of msec between height measurements
    _msecBetaAvg            = 100;      // average number of msec between synthetic sideslip measurements
    dtVelPos                = 0.02;     // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    // Misc initial conditions
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    IMU1_weighting = 0.5f;
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
    // If measurements have failed innovation consistency checks for long enough to time-out
    // and force fusion then the nav solution can be conidered to be unhealthy
    // This will only be set as a transient
    if (posTimeout || velTimeout || hgtTimeout) {
        return false;
    }

    // all OK
    return true;
}

// return true if filter is dead-reckoning height
bool NavEKF::HeightDrifting(void) const
{
    // Set to true if height measurements are failing the innovation consistency check
    return !hgtHealth;
}

// return true if filter is dead-reckoning position
bool NavEKF::PositionDrifting(void) const
{
    // Set to true if position measurements are failing the innovation consistency check
    return !posHealth;
}

// resets position states to last GPS measurement or to zero if in static mode
void NavEKF::ResetPosition(void)
{
    if (staticMode) {
        states[7] = 0;
        states[8] = 0;
    } else if (_ahrs->get_gps()->status() >= GPS::GPS_OK_FIX_3D) {

        // read the GPS
        readGpsData();
        // write to state vector
        states[7] = posNE[0];
        states[8] = posNE[1];
    }
}

// resets velocity states to last GPS measurement or to zero if in static mode
void NavEKF::ResetVelocity(void)
{
    if (staticMode) {
         state.velocity.zero();
         state.vel1.zero();
         state.vel2.zero();
    } else if (_ahrs->get_gps()->status() >= GPS::GPS_OK_FIX_3D) {
        // read the GPS
        readGpsData();
        // reset horizontal velocity states
        if (_fusionModeGPS <= 1) {
            states[4]  = velNED[0]; // north velocity from blended accel data
            states[5]  = velNED[1]; // east velocity from blended accel data
            states[23] = velNED[0]; // north velocity from IMU1 accel data
            states[24] = velNED[1]; // east velocity from IMU1 accel data
            states[27] = velNED[0]; // north velocity from IMU2 accel data
            states[28] = velNED[1]; // east velocity from IMU2 accel data
        }
        // reset vertical velocity states
        if (_fusionModeGPS <= 0) {
            states[6]  = velNED[2]; // down velocity from blended accel data
            states[25] = velNED[2]; // down velocity from IMU1 accel data
            states[29] = velNED[2]; // down velocity from IMU2 accel data
        }
    }
}

// reset the vertical position state using the last height measurement
void NavEKF::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    states[9]   = -hgtMea; // down position from blended accel data
    state.posD1 = -hgtMea; // down position from IMU1 accel data
    state.posD2 = -hgtMea; // down position from IMU2 accel data
}

// this function is used to initialise the filter whilst moving, using the AHRS DCM solution
// it should NOT be used to re-initialise after a timeout as DCM will also be corrupted
void NavEKF::InitialiseFilterDynamic(void)
{
    // this forces healthy() to be false so that when we ask for ahrs
    // attitude we get the DCM attitude regardless of the state of AHRS_EKF_USE
    statesInitialised = false;

    // Set re-used variables to zero
    ZeroVariables();

    // get initial time deltat between IMU measurements (sec)
    dtIMU = _ahrs->get_ins().get_delta_time();

    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    Quaternion initQuat;

    // calculate initial yaw angle using declination and magnetic field if available
    // otherwise set yaw to zero
    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(_ahrs->roll, _ahrs->pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = Tbn*magData;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;

        // calculate initial filter quaternion states
        initQuat.from_euler(_ahrs->roll, _ahrs->pitch, yaw);

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        initMagNED = Tbn * magData;
    } else {
        // calculate initial filter quaternion states
        initQuat.from_euler(_ahrs->roll, _ahrs->pitch, 0.0f);
        yawAligned = false;
    }

    // write to state vector
    state.quat = initQuat;
    state.gyro_bias.zero();
    state.accel_zbias1 = 0;
    state.accel_zbias2 = 0;
    state.wind_vel.zero();
    ResetVelocity();
    ResetPosition();
    ResetHeight();
    state.earth_magfield = initMagNED;
    state.body_magfield  = magBias;

    // set to true now that states have be initialised
    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit(_ahrs->roll, _ahrs->pitch, _ahrs->yaw);

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
void NavEKF::InitialiseFilterBootstrap(void)
{
    // set re-used variables to zero
    ZeroVariables();

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = _ahrs->get_ins().get_accel();

    // read the magnetometer data
    readMagData();

    // normalise the acceleration vector
    initAccVec.normalize();

    // calculate initial pitch angle
    float pitch = asinf(initAccVec.x);

    // calculate initial roll angle
    float roll = -asinf(initAccVec.y / cosf(pitch));

    // calculate initial yaw angle
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // rotate the magnetic field into NED axesn
        initMagNED = Tbn*magData;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
        yawAligned = true;
    } else {
        yaw = 0.0f;
        yawAligned = false;
    }

    // calculate initial filter quaternion states
    Quaternion initQuat;
    initQuat.from_euler(roll, pitch, yaw);

    // calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    initQuat.rotation_matrix(Tbn);
    initMagNED = Tbn * magData;

    // read the GPS
    readGpsData();

    // read the barometer
    readHgtData();

    // check on ground status
    OnGroundCheck();

    // write to state vector
    state.quat = initQuat;
    state.gyro_bias.zero();
    state.accel_zbias1 = 0;
    state.accel_zbias2 = 0;
    state.wind_vel.zero();
    state.earth_magfield = initMagNED;
    state.body_magfield = magBias;

    // set to true now we have intialised the states
    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit(roll, pitch, yaw);

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
    OnGroundCheck();

    // define rules used to set staticMode
    // staticMode enables ground operation without GPS by fusing zeros for position and height measurements
    if (static_mode_demanded()) {
        staticMode = true;
    } else {
        staticMode = false;
    }

    // check to see if static mode has changed and reset states if it has
    if (prevStaticMode != staticMode) {
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        StoreStatesReset();
        prevStaticMode = staticMode;
    }

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
    SelectTasFusion();
    SelectBetaFusion();

    // stop the timer used for load measurement
    perf_end(_perf_UpdateFilter);
}

// select fusion of velocity, position and height measurements
void NavEKF::SelectVelPosFusion()
{
    // calculate ratio of VelPos fusion to state prediction steps
    uint8_t velPosFuseStepRatio = floor(dtVelPos/dtIMU + 0.5f);

    // calculate the scale factor to be applied to GPS measurement variance to account for
    // the fact we repeat fusion of the same measurement to provide a smoother output
    gpsVarScaler = _msecGpsAvg/(1000.0f*dtVelPos);

    // calculate the scale factor to be applied to height measurement variance to account for
    // the fact we repeat fusion of the same measurement to provide a smoother output
    hgtVarScaler = _msecHgtAvg/(1000.0f*dtVelPos);

    // check for new data, specify which measurements should be used and check data for freshness
    if (!staticMode) {

        // check for and read new GPS data
        readGpsData();

        // command fusion of GPS data and reset states as required
        if (newDataGps) {
            // reset data arrived flag
            newDataGps = false;
            // enable fusion
            fuseVelData = true;
            fusePosData = true;
            // reset the counter used to schedule updates so that we always fuse data on the frame GPS data arrives
            skipCounter = velPosFuseStepRatio;
            // If a long time since last GPS update, then reset position and velocity and reset stored state history
            if ((hal.scheduler->millis() > secondLastFixTime_ms + _gpsRetryTimeUseTAS && useAirspeed()) || (hal.scheduler->millis() > secondLastFixTime_ms + _gpsRetryTimeNoTAS && !useAirspeed())) {
            ResetPosition();
            ResetVelocity();
            StoreStatesReset();
            }
        } else if (hal.scheduler->millis() > lastFixTime_ms + _msecGpsAvg + 40) {
            // Timeout fusion of GPS data if stale. Needed because we repeatedly fuse the same
            // measurement until the next one arrives to provide a smoother output
            fuseVelData = false;
            fusePosData = false;
        }

        // check for and read new height data
        readHgtData();

        // command fusion of height data
        if (newDataHgt)
        {
            // reset data arrived flag
            newDataHgt = false;
            // enable fusion
            fuseHgtData = true;
        } else if (hal.scheduler->millis() > lastHgtTime_ms + _msecHgtAvg + 40) {
            // timeout fusion of height data if stale. Needed because we repeatedly fuse the same
            // measurement until the next one arrives to provide a smoother output
            fuseHgtData = false;
        }

    } else {
        // in static mode we only fuse position and height to improve long term numerical stability
        // and only when the rate of change of velocity is less than 0.5g. This prevents attitude errors
        // due to launch acceleration
        fuseVelData = false;
        if (accNavMag < 4.9f) {
            fusePosData = true;
            fuseHgtData = true;
        } else {
            fusePosData = false;
            fuseHgtData = false;
        }
    }

    // perform fusion as commanded, and in accordance with specified time intervals
    if (fuseVelData || fusePosData || fuseHgtData) {
        // skip fusion as required to maintain ~dtVelPos time interval between corrections
        if (skipCounter >= velPosFuseStepRatio) {
            FuseVelPosNED();
            // reset counter used to skip update frames
            skipCounter = 1;
        } else {
            // increment counter used to skip update frames
            skipCounter += 1;
        }
    }

}

// select fusion of magnetometer data
void NavEKF::SelectMagFusion()
{
    // check for and read new magnetometer measurements
    readMagData();

    // determine if conditions are right to start a new fusion cycle
    bool dataReady = statesInitialised && use_compass() && newDataMag;
    if (dataReady)
    {
        MAGmsecPrev = IMUmsec;
        fuseMagData = true;
    }
    else
    {
        fuseMagData = false;
    }

    // call the function that performs fusion of magnetometer data
    FuseMagnetometer();

}

// select fusion of true airspeed measurements
void NavEKF::SelectTasFusion()
{
    // get true airspeed measurement
    readAirSpdData();

    // if the filter is initialised and we are using airspeed measurements and we are flying and
    // we either have new data or are waiting to fuse old data, then perform fusion
    tasDataWaiting = (statesInitialised && useAirspeed() && !onGround && (tasDataWaiting || newDataTas));

    // if we have waited too long, set a timeout flag which will force fusion to occur
    bool timeout = ((IMUmsec - TASmsecPrev) >= TASmsecMax);

    // we don't fuse airspeed measurements if magnetometer fusion has been performed in the same frame, unless timed out or the fuseMeNow option is selected
    // this helps to spreasthe load associated with fusion of different measurements across multiple frames
    // setting fuseMeNow to true disables this load spreading feature
    if (tasDataWaiting && (!magFusePerformed || timeout || fuseMeNow))
    {
        FuseAirspeed();
        TASmsecPrev = IMUmsec;
        tasDataWaiting = false;
    }
}

// select fusion of synthetic sideslip measurements
void NavEKF::SelectBetaFusion()
{
    // Determine if synthetic sidelsip data should be fused
    // synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
    // it requires a stable wind estimate for best results and should not be used for aerobatic flight
    // we fuse synthetic sideslip measurements if:
    // (we are not using a compass OR (we are dead-reckoning position AND using airspeed)) AND not on the ground AND enough time has lapsed since our last fusion AND
    // (we have not fused magnetometer data on this time step OR the immediate fusion flag is set)
    if ((!use_compass() || (!posHealth && useAirspeed())) && !onGround  && ((IMUmsec - BETAmsecPrev) >= _msecBetaAvg) && (!magFusePerformed || fuseMeNow)) {
        FuseSideslip();
        BETAmsecPrev = IMUmsec;
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

    // save current measurements
    prevDelAng = correctedDelAng;

    // apply corrections for earths rotation rate and coning errors
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*dtIMU + (prevDelAng % correctedDelAng) * 8.333333e-2f;

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
    Quaternion q(states[0],states[1],states[2],states[3]);
    Matrix3f Tbn_temp;
    q.rotation_matrix(Tbn_temp);
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
    hgtRate = hgtRate * (1.0f - alpha) - states[6] * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(_windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = dt * constrain_float(_gyroBiasProcessNoise, 1e-7f, 1e-5f);
    dVelBiasSigma = dt * constrain_float(_accelBiasProcessNoise, 1e-4f, 1e-2f);
    magEarthSigma = dt * constrain_float(_magEarthProcessNoise, 1e-4f, 1e-2f);
    magBodySigma  = dt * constrain_float(_magBodyProcessNoise, 1e-4f, 1e-2f);
    for (uint8_t i= 0; i<=9;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dAngBiasSigma;
    // scale gyro bias noise when on ground to allow for faster bias estimation
    for (uint8_t i=10; i<=12; i++) {
        processNoise[i] = dAngBiasSigma;
        if (onGround) {
            processNoise[i] *= _gyroBiasNoiseScaler;
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
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    dax_b = states[10];
    day_b = states[11];
    daz_b = states[12];
    dvz_b = IMU1_weighting * states[13] + (1.0f - IMU1_weighting) * states[22];
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

    // if the total position variance exceeds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1e6f)
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

// fuse selected position, velocity and height measurements, checking dat for consistency
// provide a static mode that allows maintenance of the attitude reference without GPS provided the vehicle is not accelerating
// check innovation consistency of velocity states calculated using IMU1 and IMU2 and calculate the optimal weighting of accel data
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
    uint8_t indexLimit; // used to prevent access to wind and magnetic field states and variances when on ground

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
    if (fuseVelData || fusePosData || fuseHgtData)
    {

        // if static mode is active use the current states to calculate the predicted
        // measurement rather than use states from a previous time. We need to do this
        // because there may be no stored states due to lack of real measurements.
        // in static mode, only position and height fusion is used
        if (staticMode) {
            for (uint8_t i=7; i<=9; i++) {
                statesAtPosTime[i] = states[i];
                statesAtHgtTime[i] = states[i];
            }
        }

        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed()) gpsRetryTime = _gpsRetryTimeUseTAS;
        else gpsRetryTime = _gpsRetryTimeNoTAS;

        // form the observation vector and zero observations if in static mode
        if (~staticMode) {
            for (uint8_t i=0; i<=2; i++) observation[i] = velNED[i];
            for (uint8_t i=3; i<=4; i++) observation[i] = posNE[i-3];
        observation[5] = -hgtMea;
        } else {
            for (uint8_t i=0; i<=5; i++) observation[i] = 0.0f;
        }

        // calculate additional error in GPS velocity caused by manoeuvring
        NEvelErr = _gpsNEVelVarAccScale * accNavMag;
        DvelErr  = _gpsDVelVarAccScale * accNavMag;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = _gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        R_OBS[0] = gpsVarScaler*(sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(NEvelErr));
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = gpsVarScaler*(sq(constrain_float(_gpsVertVelNoise, 0.05f, 5.0f)) + sq(DvelErr));
        R_OBS[3] = gpsVarScaler*(sq(constrain_float(_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr));
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = hgtVarScaler*sq(constrain_float(_baroAltNoise, 0.1f, 10.0f));

        // if vertical GPS velocity data is being used, check to see if the GPS vertical velocity and barometer
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        bool badIMUdata = false;
            if (_fusionModeGPS == 0 && fuseVelData && fuseHgtData) {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr  = statesAtVelTime[9] - observation[5];
            float velDErr = statesAtVelTime[6] - observation[2];
            // check if they are the same sign and both more than 2-sigma out of bounds
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 4.0f * (P[9][9] + R_OBS[5])) && (sq(velDErr) > 4.0f * (P[6][6] + R_OBS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }


        // calculate innovations and check GPS data validity using an innovation consistency check
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            if (_fusionModeGPS == 1) {
                imax = 1;
            }
            float K1 = 0;
            float K2 = 0;
            for (uint8_t i = 0; i<=imax; i++)
            {
                // velocity states start at index 4
                stateIndex   = i + 4;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = statesAtVelTime[stateIndex] - observation[i]; // blended
                velInnov1[i] = statesAtVelTime[23 + i] - observation[i]; // IMU1
                velInnov2[i] = statesAtVelTime[27 + i] - observation[i]; // IMU2
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
                // calculate error weightings for singloe IMU velocity states using
                // observation error to normalise
                float R_hgt;
                if (i == 2) {
                    R_hgt = sq(constrain_float(_gpsVertVelNoise, 0.05f, 5.0f));
                } else {
                    R_hgt = sq(constrain_float(_gpsHorizVelNoise, 0.05f, 5.0f));
                }
                K1 += R_hgt / (R_hgt + sq(velInnov1[i]));
                K2 += R_hgt / (R_hgt + sq(velInnov2[i]));
            }
            // calculate weighting used by fuseVelPosNED to do IMU accel data blending
            // this is used to detect and compensate for aliasing errors with the accelerometers
            // provide for a first order lowpass filter to reduce noise on the weighting if required
            IMU1_weighting = 1.0f * (K1 / (K1 + K2)) + 0.0f * IMU1_weighting; // filter currently inactive
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            velHealth = !((sq(velInnov[0]) + sq(velInnov[1]) + sq(velInnov[2])) > (sq(_gpsVelInnovGate) * (varInnovVelPos[0] + varInnovVelPos[1] + varInnovVelPos[2]))  && !badIMUdata);
            velTimeout = (hal.scheduler->millis() - velFailTime) > gpsRetryTime;
            if (velHealth || velTimeout || staticMode)
            {
                velHealth = true;
                velFailTime = hal.scheduler->millis();
                // if timed out, reset the velocity, but do not fuse data on this time step
                if (velTimeout)
                {
                    ResetVelocity();
                    StoreStatesReset();
                    fuseVelData =  false;
                }
            }
            else
            {
                velHealth = false;
            }
        }
        if (fusePosData)
        {
            // test horizontal position measurements
            posInnov[0] = statesAtPosTime[7] - observation[3];
            posInnov[1] = statesAtPosTime[8] - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            posHealth = !((sq(posInnov[0]) + sq(posInnov[1])) > (sq(_gpsPosInnovGate) * (varInnovVelPos[3] + varInnovVelPos[4])) && !badIMUdata);
            posTimeout = (hal.scheduler->millis() - posFailTime) > gpsRetryTime;
            // Fuse position data if healthy or timed out or in static mode
            if (posHealth || posTimeout || staticMode)
            {
                posHealth = true;
                posFailTime = hal.scheduler->millis();
                // if timed out, reset the position, but do not fuse data on this time step
                if (posTimeout)
                {
                    ResetPosition();
                    StoreStatesReset();
                    fusePosData = false;
                }
            }
            else
            {
                posHealth = false;
            }
        }
        // test height measurements
        if (fuseHgtData)
        {
            // set the height data timeout depending on whether vertical velocity data is being used
            uint32_t hgtRetryTime;
            if (_fusionModeGPS == 0) hgtRetryTime = _hgtRetryTimeMode0;
            else hgtRetryTime = _hgtRetryTimeMode12;
            // calculate height innovations
            hgtInnov = statesAtHgtTime[9] - observation[5];
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            hgtHealth = !(sq(hgtInnov) > (sq(_hgtInnovGate) * varInnovVelPos[5]) && !badIMUdata);
            hgtTimeout = (hal.scheduler->millis() - hgtFailTime) > hgtRetryTime;
            // Fuse height data if healthy or timed out or in static mode
            if (hgtHealth || hgtTimeout || staticMode)
            {
                hgtHealth = true;
                hgtFailTime = hal.scheduler->millis();
                // if timed out, reset the height, but do not fuse data on this time step
                if (hgtTimeout)
                {
                    ResetHeight();
                    StoreStatesReset();
                    fuseHgtData = false;
                }
            }
            else
            {
                hgtHealth = false;
            }
        }
        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData && _fusionModeGPS == 0 && velHealth && !staticMode)
        {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && _fusionModeGPS == 1 && velHealth && !staticMode)
        {
            fuseData[0] = true;
            fuseData[1] = true;
        }
        if ((fusePosData && _fusionModeGPS <= 2 && posHealth) || staticMode)
        {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if ((fuseHgtData && hgtHealth) || staticMode)
        {
            fuseData[5] = true;
        }
        // Limit access to first 14 states when on ground or in static mode.
        if (!onGround || staticMode)
        {
            indexLimit = 21;
        }
        else
        {
            indexLimit = 13;
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++)
        {
            if (fuseData[obsIndex])
            {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 3 || obsIndex == 4)
                {
                    innovVelPos[obsIndex] = statesAtPosTime[stateIndex] - observation[obsIndex];
                }
                else
                {
                    innovVelPos[obsIndex] = statesAtHgtTime[stateIndex] - observation[obsIndex];
                }
                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    Kfusion[i] = P[i][stateIndex]*SK;
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
                    float hgtInnov1 = statesAtHgtTime[26] - observation[obsIndex];
                    float hgtInnov2 = statesAtHgtTime[30] - observation[obsIndex];
                    // Correct single IMU prediction states using height measurement
                    states[13] = states[13] - Kfusion[13] * hgtInnov1; // IMU1 Z accel bias
                    states[22] = states[22] - Kfusion[22] * hgtInnov2; // IMU2 Z accel bias
                    for (uint8_t i = 23; i<=26; i++)
                    {
                        states[i] = states[i] - Kfusion[i] * hgtInnov1; // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i<=30; i++)
                    {
                        states[i] = states[i] - Kfusion[i] * hgtInnov2; // IMU2 velNED,posD
                    }
                } else if (obsIndex == 0 || obsIndex == 1 || obsIndex == 2){
                    // don't modify the Z accel bias states when fusing GPS velocity measurements
                    Kfusion[13] = 0.0f;
                    Kfusion[22] = 0.0f;
                    // Correct single IMU prediction states using velocity measurements
                    for (uint8_t i = 23; i<=26; i++)
                    {
                        states[i] = states[i] - Kfusion[i] * velInnov1[obsIndex]; // IMU1 velNED,posD
                    }
                    for (uint8_t i = 27; i<=30; i++)
                    {
                        states[i] = states[i] - Kfusion[i] * velInnov2[obsIndex]; // IMU2 velNED,posD
                    }
                } else {
                    // don't modify the Z accel bias states for IMU1 and IMU2 when fusing GPS horizontal position measurements
                    Kfusion[13] = 0.0f;
                    Kfusion[22] = 0.0f;
                }
                // calculate state corrections and re-normalise the quaternions for blended IMU data predicted states
                // don't update the Zacc bias state because it has already been updated
                for (uint8_t i = 0; i<=indexLimit; i++)
                {
                    if (i != 13) {
                        states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }
                }
                state.quat.normalize();
                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
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
    uint8_t indexLimit; // used to prevent access to wind and magnetic field states and variances when on ground

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseMagData || obsIndex == 1 || obsIndex == 2)
    {
        // prevent access last 9 states when on ground (acc bias, wind and magnetometer states).
        if (!onGround)
        {
            indexLimit = 21;
        }
        else
        {
            indexLimit = 12;
        }
        // calculate observation jacobians and Kalman gains
        if (fuseMagData)
        {
            // copy required states to local variable names
            q0       = statesAtMagMeasTime[0];
            q1       = statesAtMagMeasTime[1];
            q2       = statesAtMagMeasTime[2];
            q3       = statesAtMagMeasTime[3];
            magN     = statesAtMagMeasTime[16];
            magE     = statesAtMagMeasTime[17];
            magD     = statesAtMagMeasTime[18];
            magXbias = statesAtMagMeasTime[19];
            magYbias = statesAtMagMeasTime[20];
            magZbias = statesAtMagMeasTime[21];

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
            R_MAG = sq(constrain_float(_magNoise, 0.01f, 0.5f)) + sq(_magVarRateScale*dAngIMU.length() / dtIMU);

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
			SK_MX[0] = 1/(P[19][19] + R_MAG + P[1][19]*SH_MAG[0] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) - (2*magD*q0 - 2*magE*q1 + 2*magN*q2)*(P[19][2] + P[1][2]*SH_MAG[0] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2*q0*q3 + 2*q1*q2) - P[18][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2*q0*q3 + 2*q1*q2) - P[18][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2*q0*q3 + 2*q1*q2) - P[18][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2*q0*q3 + 2*q1*q2) - P[18][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2*q0*q3 + 2*q1*q2) - P[18][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[17][19]*(2*q0*q3 + 2*q1*q2) - P[18][19]*(2*q0*q2 - 2*q1*q3) - P[2][19]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + (2*q0*q3 + 2*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2*q0*q3 + 2*q1*q2) - P[18][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q2 - 2*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2*q0*q3 + 2*q1*q2) - P[18][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
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
			Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[3] - P[14][2]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[5] - P[14][18]*SK_MX[4]);
			Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[3] - P[15][2]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[5] - P[15][18]*SK_MX[4]);
			Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[3] - P[16][2]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[5] - P[16][18]*SK_MX[4]);
			Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[3] - P[17][2]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[5] - P[17][18]*SK_MX[4]);
			Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[3] - P[18][2]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[5] - P[18][18]*SK_MX[4]);
			Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[3] - P[19][2]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[5] - P[19][18]*SK_MX[4]);
			Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[3] - P[20][2]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[5] - P[20][18]*SK_MX[4]);
			Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[3] - P[21][2]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[5] - P[21][18]*SK_MX[4]);

            // calculate the observation innovation variance
            varInnovMag[0] = 1.0f/SK_MX[0];

            // reset the observation index to 0 (we start by fusing the X measurement)
            obsIndex = 0;

            // set flags to indicate to other processes that fusion has been performede and is required on the next frame
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
			SK_MY[0] = 1/(P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2*q0*q3 - 2*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2*q0*q3 - 2*q1*q2) + P[18][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (2*q0*q1 + 2*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2*q0*q3 - 2*q1*q2) + P[18][18]*(2*q0*q1 + 2*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2*q0*q3 - 2*q1*q2) + P[18][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[16][20]*(2*q0*q3 - 2*q1*q2) + P[18][20]*(2*q0*q1 + 2*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2*q0*q3 - 2*q1*q2) + P[18][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2*q0*q3 - 2*q1*q2) + P[18][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2*q0*q3 - 2*q1*q2) + P[18][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2*q0*q3 - 2*q1*q2) + P[18][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
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
			Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
			Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
			Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
			Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
			Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
			Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
			Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
			Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);

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
			SK_MZ[0] = 1/(P[21][21] + R_MAG + P[0][21]*SH_MAG[1] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) - (2*magD*q1 + 2*magE*q0 - 2*magN*q3)*(P[21][1] + P[0][1]*SH_MAG[1] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2*q0*q2 + 2*q1*q3) - P[17][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2*q0*q2 + 2*q1*q3) - P[17][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2*q0*q2 + 2*q1*q3) - P[17][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2*q0*q2 + 2*q1*q3) - P[17][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2*q0*q2 + 2*q1*q3) - P[17][18]*(2*q0*q1 - 2*q2*q3) - P[1][18]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[16][21]*(2*q0*q2 + 2*q1*q3) - P[17][21]*(2*q0*q1 - 2*q2*q3) - P[1][21]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + (2*q0*q2 + 2*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2*q0*q2 + 2*q1*q3) - P[17][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2*q0*q2 + 2*q1*q3) - P[17][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
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
			Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] + P[14][3]*SH_MAG[0] - P[14][1]*SK_MZ[2] + P[14][2]*SK_MZ[3] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[5] - P[14][17]*SK_MZ[4]);
			Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] + P[15][3]*SH_MAG[0] - P[15][1]*SK_MZ[2] + P[15][2]*SK_MZ[3] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[5] - P[15][17]*SK_MZ[4]);
			Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] + P[16][3]*SH_MAG[0] - P[16][1]*SK_MZ[2] + P[16][2]*SK_MZ[3] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[5] - P[16][17]*SK_MZ[4]);
			Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] + P[17][3]*SH_MAG[0] - P[17][1]*SK_MZ[2] + P[17][2]*SK_MZ[3] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[5] - P[17][17]*SK_MZ[4]);
			Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] + P[18][3]*SH_MAG[0] - P[18][1]*SK_MZ[2] + P[18][2]*SK_MZ[3] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[5] - P[18][17]*SK_MZ[4]);
			Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] + P[19][3]*SH_MAG[0] - P[19][1]*SK_MZ[2] + P[19][2]*SK_MZ[3] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[5] - P[19][17]*SK_MZ[4]);
			Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] + P[20][3]*SH_MAG[0] - P[20][1]*SK_MZ[2] + P[20][2]*SK_MZ[3] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[5] - P[20][17]*SK_MZ[4]);
			Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] + P[21][3]*SH_MAG[0] - P[21][1]*SK_MZ[2] + P[21][2]*SK_MZ[3] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[5] - P[21][17]*SK_MZ[4]);

            // calculate the observation innovation variance
            varInnovMag[2] = 1.0f/SK_MZ[0];

            // set flags to indicate to other processes that fusion has been performede and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = false;
        }
        // calculate the measurement innovation
        innovMag[obsIndex] = MagPred[obsIndex] - magData[obsIndex];
        // apply and innovation consistency check
        if ((innovMag[obsIndex]*innovMag[obsIndex]/varInnovMag[obsIndex]) < sq(_magInnovGate))
        {
            // correct the state vector
            for (uint8_t j= 0; j<=indexLimit; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovMag[obsIndex];
            }
            // normalise the quaternion states
            state.quat.normalize();
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=3; j++)
                {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
                for (uint8_t j = 4; j<=15; j++) KH[i][j] = 0.0f;
                if (!onGround)
                {
                    for (uint8_t j = 16; j<=21; j++)
                    {
                        KH[i][j] = Kfusion[i] * H_MAG[j];
                    }
                }
                else
                {
                    for (uint8_t j = 16; j<=21; j++)
                    {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=indexLimit; j++)
                {
                    KHP[i][j] = 0;
                    for (uint8_t k = 0; k<=3; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!onGround)
                    {
                        for (uint8_t k = 16; k<=21; k++)
                        {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=indexLimit; j++)
                {
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

    // copy required states to local variable names
    vn = statesAtVtasMeasTime[4];
    ve = statesAtVtasMeasTime[5];
    vd = statesAtVtasMeasTime[6];
    vwn = statesAtVtasMeasTime[14];
    vwe = statesAtVtasMeasTime[15];

    // calculate the predicted airspeed
    VtasPred = pythagorous3((ve - vwe) , (vn - vwn) , vd);
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
		SK_TAS = 1.0f/(R_TAS + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[14][4]*SH_TAS[2] - P[15][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[14][5]*SH_TAS[2] - P[15][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][14]*SH_TAS[2] + P[5][14]*SH_TAS[1] - P[14][14]*SH_TAS[2] - P[15][14]*SH_TAS[1] + P[6][14]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][15]*SH_TAS[2] + P[5][15]*SH_TAS[1] - P[14][15]*SH_TAS[2] - P[15][15]*SH_TAS[1] + P[6][15]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[14][6]*SH_TAS[2] - P[15][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
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
		Kfusion[16] = SK_TAS*(P[16][4]*SH_TAS[2] - P[16][14]*SH_TAS[2] + P[16][5]*SH_TAS[1] - P[16][15]*SH_TAS[1] + P[16][6]*vd*SH_TAS[0]);
		Kfusion[17] = SK_TAS*(P[17][4]*SH_TAS[2] - P[17][14]*SH_TAS[2] + P[17][5]*SH_TAS[1] - P[17][15]*SH_TAS[1] + P[17][6]*vd*SH_TAS[0]);
		Kfusion[18] = SK_TAS*(P[18][4]*SH_TAS[2] - P[18][14]*SH_TAS[2] + P[18][5]*SH_TAS[1] - P[18][15]*SH_TAS[1] + P[18][6]*vd*SH_TAS[0]);
		Kfusion[19] = SK_TAS*(P[19][4]*SH_TAS[2] - P[19][14]*SH_TAS[2] + P[19][5]*SH_TAS[1] - P[19][15]*SH_TAS[1] + P[19][6]*vd*SH_TAS[0]);
		Kfusion[20] = SK_TAS*(P[20][4]*SH_TAS[2] - P[20][14]*SH_TAS[2] + P[20][5]*SH_TAS[1] - P[20][15]*SH_TAS[1] + P[20][6]*vd*SH_TAS[0]);
		Kfusion[21] = SK_TAS*(P[21][4]*SH_TAS[2] - P[21][14]*SH_TAS[2] + P[21][5]*SH_TAS[1] - P[21][15]*SH_TAS[1] + P[21][6]*vd*SH_TAS[0]);

        // calculate measurement innovation variance
        varInnovVtas = 1.0f/SK_TAS;

        // calculate measurement innovation
        innovVtas = VtasPred - VtasMeas;

        // apply a innovation consistency check
        if ((innovVtas*innovVtas*SK_TAS) < sq(_tasInnovGate))
        {
            // correct the state vector
            for (uint8_t j=0; j<=21; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }

            Quaternion q(states[0], states[1], states[2], states[3]);
            q.normalize();
            for (uint8_t i = 0; i<=3; i++) {
                states[i] = q[i];
            }
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
    float SH_BETA[13];
    float SK_BETA[8];
    Vector3f vel_rel_wind;
    Vector22 H_BETA;
    float innovBeta;

    // copy required states to local variable names
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    vn = states[4];
    ve = states[5];
    vd = states[6];
    vwn = states[14];
    vwe = states[15];

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
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
        SK_BETA[0] = 1/(R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[14][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][4]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][4]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][4]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][4]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][4]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][4]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][4]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P[14][14]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][14]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][14]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][14]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][14]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][14]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][14]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][14]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][14]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[14][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][5]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][5]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][5]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][5]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][5]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][5]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][5]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2))*(P[14][15]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][15]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][15]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][15]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][15]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][15]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][15]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][15]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][15]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P[14][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][0]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][0]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][0]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][0]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][0]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][0]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][0]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P[14][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][1]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][1]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][1]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][1]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][1]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][1]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][1]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P[14][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][2]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][2]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][2]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][2]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][2]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][2]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][2]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P[14][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][3]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][3]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][3]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][3]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][3]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][3]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][3]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))) + (SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))*(P[14][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P[4][6]*(SH_BETA[5]*(SH_BETA[12] - 2*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P[5][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) - P[15][6]*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2*q1*q2)) + P[0][6]*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P[1][6]*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P[2][6]*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P[3][6]*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P[6][6]*(SH_BETA[5]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2*q0*q2 - 2*q1*q3))));
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
        Kfusion[16] = SK_BETA[0]*(P[16][0]*SK_BETA[5] + P[16][1]*SK_BETA[4] - P[16][4]*SK_BETA[1] + P[16][5]*SK_BETA[2] + P[16][2]*SK_BETA[6] + P[16][6]*SK_BETA[3] - P[16][3]*SK_BETA[7] + P[16][14]*SK_BETA[1] - P[16][15]*SK_BETA[2]);
        Kfusion[17] = SK_BETA[0]*(P[17][0]*SK_BETA[5] + P[17][1]*SK_BETA[4] - P[17][4]*SK_BETA[1] + P[17][5]*SK_BETA[2] + P[17][2]*SK_BETA[6] + P[17][6]*SK_BETA[3] - P[17][3]*SK_BETA[7] + P[17][14]*SK_BETA[1] - P[17][15]*SK_BETA[2]);
        Kfusion[18] = SK_BETA[0]*(P[18][0]*SK_BETA[5] + P[18][1]*SK_BETA[4] - P[18][4]*SK_BETA[1] + P[18][5]*SK_BETA[2] + P[18][2]*SK_BETA[6] + P[18][6]*SK_BETA[3] - P[18][3]*SK_BETA[7] + P[18][14]*SK_BETA[1] - P[18][15]*SK_BETA[2]);
        Kfusion[19] = SK_BETA[0]*(P[19][0]*SK_BETA[5] + P[19][1]*SK_BETA[4] - P[19][4]*SK_BETA[1] + P[19][5]*SK_BETA[2] + P[19][2]*SK_BETA[6] + P[19][6]*SK_BETA[3] - P[19][3]*SK_BETA[7] + P[19][14]*SK_BETA[1] - P[19][15]*SK_BETA[2]);
        Kfusion[20] = SK_BETA[0]*(P[20][0]*SK_BETA[5] + P[20][1]*SK_BETA[4] - P[20][4]*SK_BETA[1] + P[20][5]*SK_BETA[2] + P[20][2]*SK_BETA[6] + P[20][6]*SK_BETA[3] - P[20][3]*SK_BETA[7] + P[20][14]*SK_BETA[1] - P[20][15]*SK_BETA[2]);
        Kfusion[21] = SK_BETA[0]*(P[21][0]*SK_BETA[5] + P[21][1]*SK_BETA[4] - P[21][4]*SK_BETA[1] + P[21][5]*SK_BETA[2] + P[21][2]*SK_BETA[6] + P[21][6]*SK_BETA[3] - P[21][3]*SK_BETA[7] + P[21][14]*SK_BETA[1] - P[21][15]*SK_BETA[2]);

        // calculate predicted sideslip angle and innovation using small angle approximation and assuming zero sideslip
        innovBeta = vel_rel_wind.y / vel_rel_wind.x;

        // correct the state vector
        for (uint8_t j=0; j<=21; j++)
        {
            states[j] = states[j] - Kfusion[j] * innovBeta;
        }

        Quaternion q(states[0], states[1], states[2], states[3]);
        q.normalize();
        for (uint8_t i = 0; i<=3; i++) {
            states[i] = q[i];
        }
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
    if (storeIndex > 49) storeIndex = 0;
    for (uint8_t i=0; i<=30; i++) storedStates[i][storeIndex] = states[i];
    statetimeStamp[storeIndex] = hal.scheduler->millis();
    storeIndex = storeIndex + 1;
}

// reset the stored state history and store the current state
void NavEKF::StoreStatesReset()
{
    // clear stored state history
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    // store current state vector in first column
    storeIndex = 0;
    for (uint8_t i=0; i<=30; i++) storedStates[i][storeIndex] = states[i];
    statetimeStamp[storeIndex] = hal.scheduler->millis();
    storeIndex = storeIndex + 1;
}

// recall state vector stored at closest time to the one specified by msec
void NavEKF::RecallStates(Vector31 &statesForFusion, uint32_t msec)
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
        for (uint8_t i=0; i<=30; i++) {
            statesForFusion[i] = storedStates[i][bestStoreIndex];
        }
    }
    else // otherwise output current state
    {
        for (uint8_t i=0; i<=30; i++) {
            statesForFusion[i] = states[i];
        }
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
    Quaternion q(states[0], states[1], states[2], states[3]);
    q.to_euler(&euler.x, &euler.y, &euler.z);
    euler = euler - _ahrs->get_trim();
}

// return NED velocity in m/s
void NavEKF::getVelNED(Vector3f &vel) const
{
    vel.x = states[4];
    vel.y = states[5];
    vel.z = states[6];
}

// return the last calculated NED position relative to the reference point (m).
// return false if no position is available
bool NavEKF::getPosNED(Vector3f &pos) const
{
    pos.x = states[7];
    pos.y = states[8];
    pos.z = states[9];
    return true;
}

// return body axis gyro bias estimates in rad/sec
void NavEKF::getGyroBias(Vector3f &gyroBias) const
{
    gyroBias.x = states[10] / dtIMU;
    gyroBias.y = states[11] / dtIMU;
    gyroBias.z = states[12] / dtIMU;
}

// return weighting of first IMU in blending function and the individual Z-accel bias estimates in m/s^2
void NavEKF::getAccelBias(Vector3f &accelBias) const
{
    accelBias.x = IMU1_weighting;
    accelBias.y = states[22] / dtIMU;
    accelBias.z = states[13] / dtIMU;
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF::getWind(Vector3f &wind) const
{
    wind.x = states[14];
    wind.y = states[15];
    wind.z = 0.0f; // curently don't estimate this
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF::getMagNED(Vector3f &magNED) const
{
    magNED.x = states[16]*1000.0f;
    magNED.y = states[17]*1000.0f;
    magNED.z = states[18]*1000.0f;
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ.x = states[19]*1000.0f;
    magXYZ.y = states[20]*1000.0f;
    magXYZ.z = states[21]*1000.0f;
}

// return the last calculated latitude, longitude and height
bool NavEKF::getLLH(struct Location &loc) const
{
    loc.lat = _ahrs->get_home().lat;
    loc.lng = _ahrs->get_home().lng;
    loc.alt = _ahrs->get_home().alt - states[9]*100;
    location_offset(loc, states[7], states[8]);
    return true;
}

// calculate whether the flight vehicle is on the ground or flying from height, airspeed and GPS speed
void NavEKF::OnGroundCheck()
{
    const AP_Airspeed *airspeed = _ahrs->get_airspeed();
    uint8_t highAirSpd = (airspeed && airspeed->use() && airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 8.0f);
    float gndSpdSq = sq(velNED[0]) + sq(velNED[1]);
    uint8_t highGndSpdStage1 = (uint8_t)(gndSpdSq > 9.0f);
    uint8_t highGndSpdStage2 = (uint8_t)(gndSpdSq > 36.0f);
    uint8_t highGndSpdStage3 = (uint8_t)(gndSpdSq > 81.0f);
    uint8_t largeHgt = (uint8_t)(fabsf(hgtMea) > 15.0f);
    uint8_t inAirSum = highAirSpd + highGndSpdStage1 + highGndSpdStage2 + highGndSpdStage3 + largeHgt;
    // inhibit onGround mode if magnetometer calibration is enabled, movement is detected and static mode isn't demanded
    if ((_magCal == 1) && (accNavMagHoriz > 0.5f) && !static_mode_demanded() && use_compass()) {
        onGround = false;
    } else {
        // detect on-ground to in-air transition
        // if we are already on the ground then 3 or more out of 5 criteria are required
        // if we are in the air then only 2 or more are required
        // this prevents rapid tansitions
        if ((onGround && (inAirSum >= 3)) || (!onGround && (inAirSum >= 2))) {
            onGround = false;
        } else {
            onGround = true;
        }
        // force a yaw alignment if exiting onGround without a compass
        if (!onGround && prevOnGround && !use_compass()) {
            ForceYawAlignment();
        }
    }
    prevOnGround = onGround;
}

// initialise the covariance matrix
void NavEKF::CovarianceInit(float roll, float pitch, float yaw)
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
    P[10][10] = sq(radians(0.1f * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // Z delta velocity bias
	P[13][13] = sq(radians(0.5f * dtIMU));
    // wind velocities
    P[14][14] = sq(8.0f);
    P[15][15]  = P[14][14];
    // earth magnetic field
    P[16][16] = sq(0.02f);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = sq(0.02f);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
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
    // if we are on-ground or in static mode, we want all the off-diagonals for the wind
    // and magnetic field states to remain zero and want to keep the old variances
    // for these states
    if (onGround || staticMode) {
        // copy calculated variances we want to propagate
        for (uint8_t i=0; i<=13; i++) {
            P[i][i] = nextP[i][i];
        }
        // force covariances to be either zero or symetrical
        for (uint8_t i=1; i<=21; i++)
        {
            for (uint8_t j=0; j<=i-1; j++)
            {
                if ((i >= 14) || (j >= 14)) {
                    P[i][j] = 0.0f;
                } else {
                    P[i][j] = 0.5f*(nextP[i][j] + nextP[j][i]);
                }
                P[j][i] = P[i][j];
            }
        }
    }
    // if we flying and are not using airspeed and are not using synthetic sideslip measurements, we want all the off-diagonals for the wind
    // states to remain zero and want to keep the old variances for these states
    else if (!useAirspeed() && use_compass()) {
        // copy calculated variances we want to propagate
        for (uint8_t i=0; i<=13; i++) {
            P[i][i] = nextP[i][i];
        }
        for (uint8_t i=16; i<=21; i++) {
            P[i][i] = nextP[i][i];
        }
        // force covariances to be either zero or symetrical
        for (uint8_t i=1; i<=21; i++)
        {
            for (uint8_t j=0; j<=i-1; j++)
            {
                if (i == 14 || i == 15 || j == 14 || j == 15) {
                    P[i][j] = 0.0f;
                } else {
                    P[i][j] = 0.5f*(nextP[i][j] + nextP[j][i]);
                }
                P[j][i] = P[i][j];
            }
        }
    }
    // if we are flying with all sensors then all covariance terms are active
    else {
        // copy calculated variances we want to propagate
        for (uint8_t i=0; i<=21; i++) {
            P[i][i] = nextP[i][i];
        }
        for (uint8_t i=1; i<=21; i++)
        {
            for (uint8_t j=0; j<=i-1; j++)
            {
                P[i][j] = 0.5f*(nextP[i][j] + nextP[j][i]);
                P[j][i] = P[i][j];
            }
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

    // get the time the IMU data was read
    IMUmsec = hal.scheduler->millis();

    // limit IMU delta time to prevent numerical problems elsewhere
    dtIMU = constrain_float(_ahrs->get_ins().get_delta_time(), 0.001f, 1.0f);

    // get accels and gyro data from dual sensors if healthy
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
    if ((_ahrs->get_gps()->last_message_time_ms() != lastFixTime_ms) &&
            (_ahrs->get_gps()->status() >= GPS::GPS_OK_FIX_3D))
    {
        // store fix time from previous read
        secondLastFixTime_ms = lastFixTime_ms;

        // get current fix time
        lastFixTime_ms = _ahrs->get_gps()->last_message_time_ms();

        // set flag that lets other functions know that new GPS data has arrived
        newDataGps = true;

        // get state vectors that were stored at the time that is closest to when the the GPS measurement
        // time after accounting for measurement delays
        RecallStates(statesAtVelTime, (IMUmsec - constrain_int16(_msecVelDelay, 0, 500)));
        RecallStates(statesAtPosTime, (IMUmsec - constrain_int16(_msecPosDelay, 0, 500)));

        // read the NED velocity from the GPS
        velNED[0] = _ahrs->get_gps()->velocity_north();
        velNED[1] = _ahrs->get_gps()->velocity_east();

        // Check if GPS can output vertical velocity and set value and GPS fusion mode accordingly
        if (_ahrs->get_gps()->have_vertical_velocity()) {
            velNED[2] = _ahrs->get_gps()->velocity_down();
        } else {
            velNED[2] = 0;
            // vertical velocity should not be fused
            if (_fusionModeGPS == 0) {
                _fusionModeGPS = 1;
            }
        }

        // read latitutde and longitude from GPS and convert to NE position
        struct Location gpsloc;
        gpsloc.lat = _ahrs->get_gps()->latitude;
        gpsloc.lng = _ahrs->get_gps()->longitude;
        Vector2f posdiff = location_diff(_ahrs->get_home(), gpsloc);
        posNE[0] = posdiff.x;
        posNE[1] = posdiff.y;

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
        lastHgtTime_ms = hal.scheduler->millis();

        // get measurement and set flag to let other functions know new data has arrived
        hgtMea = _baro.get_altitude();
        newDataHgt = true;

        // get states that wer stored at the time closest to the measurement time, taking measurement delay into account
        RecallStates(statesAtHgtTime, (IMUmsec - _msecHgtDelay));
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

        // read compass data and assign to bias and uncorrected measurement
        // body fixed magnetic bias is opposite sign to APM compass offsets
        // we scale compass data to improve numerical conditioning
        magBias = -_ahrs->get_compass()->get_offsets() * 0.001f;
        magData = _ahrs->get_compass()->get_field() * 0.001f + magBias;

        // get states stored at time closest to measurement time after allowance for measurement delay
        RecallStates(statesAtMagMeasTime, (IMUmsec - _msecMagDelay));

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
        RecallStates(statesAtVtasMeasTime, (IMUmsec - _msecTasDelay));
    } else {
        newDataTas = false;
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

// this function is used to do a forced alignment of the yaw angle to aligwith the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff without a magnetometer.
void NavEKF::ForceYawAlignment()
{
    if ((sq(velNED[0]) + sq(velNED[1])) > 16.0f) {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        Quaternion initQuat;
        Quaternion newQuat;
        for (uint8_t i=0; i<=3; i++) initQuat[i] = states[i];
        initQuat.to_euler(&roll, &pitch, &oldYaw);
        // calculate yaw angle from GPS velocity
        newYaw = atan2f(velNED[1],velNED[0]);
        // modify yaw angle using GPS ground course if more than 45 degrees away or if not previously aligned
        yawErr = fabsf(newYaw - oldYaw);
        if (((yawErr > 0.7854f) && (yawErr < 5.4978f)) || !yawAligned) {
            // calculate new filter quaternion states from Euler angles
            newQuat.from_euler(roll, pitch, newYaw);
            for (uint8_t i=0; i<=3; i++) states[i] = newQuat[i];
            // the yaw angle is now aligned so update its status
            yawAligned =  true;
            // set the velocity states
            if (_fusionModeGPS < 2) {
                states[4] = velNED[0];
                states[5] = velNED[1];
            }
            // reinitialise the quaternion, velocity and position covariances
            // zero the matrix entries
            zeroRows(P,0,9);
            zeroCols(P,0,9);
            // quaternions - TODO maths that sets them based on different roll, yaw and pitch uncertainties
            P[0][0]   = 1.0e-9f;
            P[1][1]   = 0.25f*sq(radians(1.0f));
            P[2][2]   = 0.25f*sq(radians(1.0f));
            P[3][3]   = 0.25f*sq(radians(1.0f));
            // velocities - we could have a big error coming out of static mode due to GPS lag
            P[4][4]   = 400.0f;
            P[5][5]   = P[4][4];
            P[6][6]   = sq(0.7f);
            // positions - we could have a big error coming out of static mode due to GPS lag
            P[7][7]   = 400.0f;
            P[8][8]   = P[7][7];
            P[9][9]   = sq(5.0f);
        }
    }
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF::getRotationBodyToNED(Matrix3f &mat) const
{
    Vector3f trim = _ahrs->get_trim();
    Quaternion q(states[0], states[1], states[2], states[3]);
    q.rotation_matrix(mat);
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

// return the innovation variances for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void  NavEKF::getVariances(Vector3f &velVar, Vector3f &posVar, Vector3f &magVar, float &tasVar) const
{
    velVar.x = varInnovVelPos[0];
    velVar.y = varInnovVelPos[1];
    velVar.z = varInnovVelPos[2];
    posVar.x = varInnovVelPos[3];
    posVar.y = varInnovVelPos[4];
    posVar.z = varInnovVelPos[5];
    magVar.x = 1e6f*varInnovMag[0]; // Convert back to sensor units
    magVar.y = 1e6f*varInnovMag[1]; // Convert back to sensor units
    magVar.z = 1e6f*varInnovMag[2]; // Convert back to sensor units
    tasVar   = varInnovVtas;
}

// zero stored variables - this needs to be called before a full filter initialisation
void NavEKF::ZeroVariables()
{
    velTimeout = false;
    posTimeout = false;
    hgtTimeout = false;
    lastFixTime_ms = 0;
    secondLastFixTime_ms = 0;
    lastMagUpdate = 0;
    lastAirspeedUpdate = 0;
    velFailTime = 0;
    posFailTime = 0;
    hgtFailTime = 0;
    storeIndex = 0;
    TASmsecPrev = 0;
    BETAmsecPrev = 0;
    MAGmsecPrev = 0;
    HGTmsecPrev = 0;
    lastMagUpdate = 0;
    lastAirspeedUpdate = 0;
    lastHgtMeasTime = 0;
    dtIMU = 0;
    dt = 0;
    hgtMea = 0;
    storeIndex = 0;
	prevDelAng.zero();
    lastAngRate.zero();
    lastAccel1.zero();
    lastAccel2.zero();
    velDotNEDfilt.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    velNED.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
    memset(&posNE[0], 0, sizeof(posNE));
}

// return true if we should use the airspeed sensor
bool NavEKF::useAirspeed(void) const
{
    if (_ahrs->get_airspeed() == NULL) {
        return false;
    }
    return _ahrs->get_airspeed()->use();
}

// return true if the vehicle code has requested use of static mode
// in static mode, position and height are constrained to zero, allowing an attitude
// reference to be initialised and maintained when on the ground and without GPS lock
bool NavEKF::static_mode_demanded(void) const
{
    return !_ahrs->get_armed() || !_ahrs->get_correct_centrifugal();
}

// return true if we should use the compass
bool NavEKF::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

#endif // HAL_CPU_CLASS
