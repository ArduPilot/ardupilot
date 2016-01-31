/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// when the wind estimation first starts with no airspeed sensor,
// assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// maximum gyro bias in rad/sec that can be compensated for
#define MAX_GYRO_BIAS 0.1745f

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// copter defaults
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.1f

#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
// rover defaults
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.3f

#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// generic defaults (and for plane)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.3f

#else
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.1f

#endif // APM_BUILD_DIRECTORY


// constructor
NavEKF_core::NavEKF_core(NavEKF &_frontend, const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng) :
    frontend(_frontend),
    _ahrs(ahrs),
    _baro(baro),
    _rng(rng),
    state(*reinterpret_cast<struct state_elements *>(&states)),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    msecHgtDelay(60),               // Height measurement delay (msec)
    msecMagDelay(40),               // Magnetometer measurement delay (msec)
    msecTasDelay(240),              // Airspeed measurement delay (msec)
    gpsRetryTimeUseTAS(10000),      // GPS retry time with airspeed measurements (msec)
    gpsRetryTimeNoTAS(7000),        // GPS retry time without airspeed measurements (msec)
    gpsFailTimeWithFlow(5000),      // If we have no GPS for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    hgtRetryTimeMode0(10000),       // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12(5000),       // Height retry time without vertical velocity measurement (msec)
    tasRetryTime(5000),             // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.05f),         // scale factor applied to magnetometer variance due to angular rate
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    accelBiasNoiseScaler(1.0f),     // scale factor applied to imu accel bias learning before the vehicle is armed
    msecGpsAvg(200),                // average number of msec between GPS measurements
    msecHgtAvg(100),                // average number of msec between height measurements
    msecMagAvg(100),                // average number of msec between magnetometer measurements
    msecBetaAvg(100),               // average number of msec between synthetic sideslip measurements
    msecBetaMax(200),               // maximum number of msec between synthetic sideslip measurements
    msecFlowAvg(100),               // average number of msec between optical flow measurements
    dtVelPos(0.2f),                 // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    covTimeStepMax(0.02f),          // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    TASmsecMax(200),                // maximum allowed interval between airspeed measurement updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),          // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f)      // scaler applied to the barometer observation variance when operating in ground effect
{
}

// Check basic filter health metrics and return a consolidated health status
bool NavEKF_core::healthy(void) const
{
    uint8_t faultInt;
    getFilterFaults(faultInt);
    if (faultInt > 0) {
        return false;
    }
    if (frontend._fallback && velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1) {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }
    // Give the filter a second to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 1000 ) {
        return false;
    }
    // barometer and position innovations must be within limits when on-ground
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (!vehicleArmed && (!hgtHealth || fabsf(hgtInnovFiltState) > 1.0f || horizErrSq > 2.0f)) {
        return false;
    }

    // all OK
    return true;
}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = state.position.x;
    posResetNE.y = state.position.y;

    if (constPosMode || (PV_AidingMode != AID_ABSOLUTE)) {
        state.position.x = 0;
        state.position.y = 0;
    } else if (!gpsNotAvailable) {
        // write to state vector and compensate for GPS latency
        state.position.x = gpsPosNE.x + 0.001f*velNED.x*float(frontend._msecPosDelay);
        state.position.y = gpsPosNE.y + 0.001f*velNED.y*float(frontend._msecPosDelay);
        // the estimated states at the last GPS measurement are set equal to the GPS measurement to prevent transients on the first fusion
        statesAtPosTime.position.x = gpsPosNE.x;
        statesAtPosTime.position.y = gpsPosNE.y;
    }
    // stored horizontal position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.x = state.position.x;
        storedStates[i].position.y = state.position.y;
    }

    // Calculate the position jump due to the reset
    posResetNE.x = state.position.x - posResetNE.x;
    posResetNE.y = state.position.y - posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;
}

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF_core::ResetVelocity(void)
{
    // Store the velocity before the reset so that we can record the reset delta
    velResetNE.x = state.velocity.x;
    velResetNE.y = state.velocity.y;

    if (constPosMode || PV_AidingMode != AID_ABSOLUTE) {
         state.velocity.zero();
         state.vel1.zero();
         state.vel2.zero();
         posDownDerivative = 0.0f;
    } else if (!gpsNotAvailable) {
        // reset horizontal velocity states
        state.velocity.x  = velNED.x; // north velocity from blended accel data
        state.velocity.y  = velNED.y; // east velocity from blended accel data
        state.vel1.x      = velNED.x; // north velocity from IMU1 accel data
        state.vel1.y      = velNED.y; // east velocity from IMU1 accel data
        state.vel2.x      = velNED.x; // north velocity from IMU2 accel data
        state.vel2.y      = velNED.y; // east velocity from IMU2 accel data
        // over write stored horizontal velocity states to prevent subsequent GPS measurements from being rejected
        for (uint8_t i=0; i<=49; i++){
            storedStates[i].velocity.x = velNED.x;
            storedStates[i].velocity.y = velNED.y;
        }
    }

    // Calculate the velocity jump due to the reset
    velResetNE.x = state.velocity.x - velResetNE.x;
    velResetNE.y = state.velocity.y - velResetNE.y;

    // store the time of the reset
    lastVelReset_ms = imuSampleTime_ms;
}

// reset the vertical position state using the last height measurement
void NavEKF_core::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    state.position.z = -hgtMea; // down position from blended accel data
    state.posD1 = -hgtMea; // down position from IMU1 accel data
    state.posD2 = -hgtMea; // down position from IMU2 accel data
    terrainState = state.position.z + rngOnGnd;
    // Reset the vertical velocity state using GPS vertical velocity if we are airborne (use arm status as a surrogate)
    // Check that GPS vertical velocity data is available and can be used
    if (vehicleArmed && !gpsNotAvailable && frontend._fusionModeGPS == 0) {
        state.velocity.z =  velNED.z;
    }
    // reset stored vertical position states to prevent subsequent GPS measurements from being rejected
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.z = -hgtMea;
    }
    // reset the height state for the complementary filter used to provide a vertical position dervative
    posDown = state.position.z;
}

// this function is used to initialise the filter whilst moving, using the AHRS DCM solution
// it should NOT be used to re-initialise after a timeout as DCM will also be corrupted
bool NavEKF_core::InitialiseFilterDynamic(void)
{
    // Don't start if the user has disabled
    if (frontend._enable == 0) {
        return false;
    }

    // this forces healthy() to be false so that when we ask for ahrs
    // attitude we get the DCM attitude regardless of the state of AHRS_EKF_USE
    statesInitialised = false;

    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return false;
    }

    // If the DCM solution has not converged, then don't initialise
    // If an extended time has passed we apply a looser check criteria because movement or GPS noise can be increase the errorRP value from DCM
    if ((_ahrs->get_error_rp() > 0.05f && _ahrs->uptime_ms() < 30000U) || _ahrs->get_error_rp() > 0.1f) {
        return false;
    }

    // Set re-used variables to zero
    InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtDelAng = dtIMUavg = _ahrs->get_ins().get_loop_delta_t();

    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    gpsUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecGpsAvg);
    gpsUpdateCountMax = uint8_t(1.0f/gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecHgtAvg);
    hgtUpdateCountMax = uint8_t(1.0f/hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecMagAvg);
    magUpdateCountMax = uint8_t(1.0f/magUpdateCountMaxInv);
    flowUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecFlowAvg);
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

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();

    // initialise the covariance matrix
    CovarianceInit();

    return true;
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF_core::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        statesInitialised = false;
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // get initial time deltat between IMU measurements (sec)
    dtDelAng = dtIMUavg = _ahrs->get_ins().get_loop_delta_t();

    // set number of updates over which gps and baro measurements are applied to the velocity and position states
    gpsUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecGpsAvg);
    gpsUpdateCountMax = uint8_t(1.0f/gpsUpdateCountMaxInv);
    hgtUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecHgtAvg);
    hgtUpdateCountMax = uint8_t(1.0f/hgtUpdateCountMaxInv);
    magUpdateCountMaxInv = (dtIMUavg * 1000.0f)/float(msecMagAvg);
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

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise IMU pre-processing states
    readIMUData();

    // initialise the covariance matrix
    CovarianceInit();

    return true;
}

// Update Filter States - this should be called whenever new IMU data is available
void NavEKF_core::UpdateFilter()
{
    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    correctedDelAngQuat.initialise();

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
#if EKF_DISABLE_INTERRUPTS
    irqstate_t istate = irqsave();
#endif
    hal.util->perf_begin(_perf_UpdateFilter);

    //get starting time for update step
    imuSampleTime_ms = AP_HAL::millis();

    // read IMU data and convert to delta angles and velocities
    readIMUData();

    static bool prev_armed = false;
    bool armed = getVehicleArmStatus();

    // the vehicle was previously disarmed and time has slipped
    // gyro auto-zero has likely just been done - skip this timestep
    if (!prev_armed && dtDelAng > dtIMUavg*5.0f) {
        // stop the timer used for load measurement
        prev_armed = armed;
        goto end;
    }
    prev_armed = armed;

    // detect if the filter update has been delayed for too long
    if (dtDelAng > 0.2f) {
        // we have stalled for too long - reset states
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        StoreStatesReset();
        //Initialise IMU pre-processing states
        readIMUData();
        // stop the timer used for load measurement
        goto end;
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
    summedDelVel = summedDelVel + correctedDelVel12;
    dt += dtDelAng;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    if (((dt >= (covTimeStepMax - dtDelAng)) || (summedDelAng.length() > covDelAngMax))) {
        CovariancePrediction();
    } else {
        covPredStep = false;
    }

    // Read range finder data which is used by both position and optical flow fusion
    readRangeFinder();

    // Update states using GPS, altimeter, compass, airspeed and synthetic sideslip observations
    SelectVelPosFusion();
    SelectMagFusion();
    SelectFlowFusion();
    SelectTasFusion();
    SelectBetaFusion();

end:
    // stop the timer used for load measurement
    hal.util->perf_end(_perf_UpdateFilter);
#if EKF_DISABLE_INTERRUPTS
    irqrestore(istate);
#endif
}

// select fusion of velocity, position and height measurements
void NavEKF_core::SelectVelPosFusion()
{
    // check for and read new height data
    readHgtData();

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime = (useGpsVertVel && !velTimeout) ? hgtRetryTimeMode0 : hgtRetryTimeMode12;
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

    // check for and read new GPS data
    readGpsData();

    // Specify which measurements should be used and check data for freshness
    if (PV_AidingMode == AID_ABSOLUTE) {

        // check if we can use opticalflow as a backup
        bool optFlowBackup = (flowDataValid && !hgtTimeout);

        // Set GPS time-out threshold depending on whether we have an airspeed sensor to constrain drift
        uint16_t gpsRetryTimeout = useAirspeed() ? gpsRetryTimeUseTAS : gpsRetryTimeNoTAS;

        // Set the time that copters will fly without a GPS lock before failing the GPS and switching to a non GPS mode
        uint16_t gpsFailTimeout = optFlowBackup ? gpsFailTimeWithFlow : gpsRetryTimeout;

        // If we haven't received GPS data for a while, then declare the position and velocity data as being timed out
        if (imuSampleTime_ms - lastFixTime_ms > gpsFailTimeout) {
            posTimeout = true;
            velTimeout = true;
            // If this happens in flight and we don't have airspeed or sideslip assumption or optical flow to constrain drift, then go into constant position mode.
            // Stay in that mode until the vehicle is re-armed.
            // If we can do optical flow nav (valid flow data and hieght above ground estimate, then go into flow nav mode.
            // Stay in that mode until the vehicle is dis-armed.
            if (vehicleArmed && !useAirspeed() && !assume_zero_sideslip()) {
                if (optFlowBackup) {
                    // we can do optical flow only nav
                    frontend._fusionModeGPS = 3;
                    PV_AidingMode = AID_RELATIVE;
                    constPosMode = false;
                } else {
                    constPosMode = true;
                    PV_AidingMode = AID_NONE;
                    posTimeout = true;
                    velTimeout = true;
                    // reset the velocity
                    ResetVelocity();
                    // store the current position to be used to keep reporting the last known position
                    lastKnownPositionNE.x = state.position.x;
                    lastKnownPositionNE.y = state.position.y;
                    // reset the position
                    ResetPosition();
                }
                // set the position and velocity timeouts to indicate we are not using GPS data
                posTimeout = true;
                velTimeout = true;
            }
        }

        // command fusion of GPS data and reset states as required
        if (newDataGps && (PV_AidingMode == AID_ABSOLUTE)) {
            // reset data arrived flag
            newDataGps = false;
            // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
            memset(&gpsIncrStateDelta[0], 0, sizeof(gpsIncrStateDelta));
            gpsUpdateCount = 0;
            // use both if GPS use is enabled
            fuseVelData = true;
            fusePosData = true;
            // If a long time since last GPS update, then reset position and velocity and reset stored state history
            if (imuSampleTime_ms - secondLastFixTime_ms > gpsRetryTimeout) {
                ResetPosition();
                ResetVelocity();
                // record the fail time
                lastPosFailTime = imuSampleTime_ms;
                // Reset the normalised innovation to avoid false failing the bad position fusion test
                posTestRatio = 0.0f;
            }
        } else {
            fuseVelData = false;
            fusePosData = false;
        }
    } else if (constPosMode && (fuseHgtData || ((imuSampleTime_ms - lastConstPosFuseTime_ms) > 200))) {
        // In constant position mode use synthetic position and velocity measurements set to zero whenever we are fusing a height measurement
        // If no height has been received for 200 msec, then fuse anyway so we have a guaranteed minimum aiding rate equivalent to GPS
        // only fuse synthetic measurements when rate of change of velocity is less than 0.5g to reduce attitude errors due to launch acceleration
        // do not use velocity fusion to reduce the effect of movement on attitude
        if (!vehicleArmed) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        if (accNavMag < 4.9f) {
            fusePosData = true;
        } else {
            fusePosData = false;
        }
        // record the fusion time - used to control fusion rate when there is no baro data
        lastConstPosFuseTime_ms = imuSampleTime_ms;
    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
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

    // Detect and declare bad GPS aiding status for minimum 10 seconds if a GPS rejection occurs after
    // rejection of GPS and reset to GPS position. This addresses failure case where errors cause ongoing rejection
    // of GPS and severe loss of position accuracy.
    uint32_t gpsRetryTime;
    if (useAirspeed()) {
        gpsRetryTime = gpsRetryTimeUseTAS;
    } else {
        gpsRetryTime = gpsRetryTimeNoTAS;
    }
    if ((posTestRatio > 2.0f) && ((imuSampleTime_ms - lastPosFailTime) < gpsRetryTime) && ((imuSampleTime_ms - lastPosFailTime) > gpsRetryTime/2) && fusePosData) {
        lastGpsAidBadTime_ms = imuSampleTime_ms;
        gpsAidingBad = true;
    }
    gpsAidingBad = gpsAidingBad && ((imuSampleTime_ms - lastGpsAidBadTime_ms) < 10000);
}

// select fusion of magnetometer data
void NavEKF_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

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
    if (dataReady) {
        // Calculate change in angle since last magetoemter fusion - used to check if in-flight alignment can be performed
        // Use a quaternion division to calcualte the delta quaternion between the rotation at the current and last time
        Quaternion deltaQuat = state.quat / prevQuatMagReset;
        prevQuatMagReset = state.quat;
        // convert the quaternion to a rotation vector and find its length
        Vector3f deltaRotVec;
        deltaQuat.to_axis_angle(deltaRotVec);
        float deltaRot = deltaRotVec.length();

        // Check if the magnetic field states should be reset
        if (vehicleArmed && !firstMagYawInit && (state.position.z  - posDownAtArming) < -1.5f && !assume_zero_sideslip() && deltaRot < 0.1745f) {
            // Do the first in-air yaw and earth mag field initialisation when the vehicle has gained 1.5m of altitude after arming if it is a non-fly forward vehicle (vertical takeoff)
            // This is done to prevent magnetic field distoration from steel roofs and adjacent structures causing bad earth field and initial yaw values
            // Do not do this alignment if the vehicle is rotating rapidly as timing erors in the mag data will cause significant errors
            Vector3f eulerAngles;
            statesAtMagMeasTime.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            state.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
            firstMagYawInit = true;
        } else if (vehicleArmed && !secondMagYawInit && (state.position.z - posDownAtArming) < -5.0f && !assume_zero_sideslip() && deltaRot < 0.1745f) {
            // Do the second and final yaw and earth mag field initialisation when the vehicle has gained 5.0m of altitude after arming if it is a non-fly forward vehicle (vertical takeoff)
            // This second and final correction is needed for flight from large metal structures where the magnetic field distortion can extend up to 5m
            // Do not do this alignment if the vehicle is rotating rapidly as timing erors in the mag data will cause significant errors
            Vector3f eulerAngles;
            statesAtMagMeasTime.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            state.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
            secondMagYawInit = true;
        }

        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&magIncrStateDelta[0], 0, sizeof(magIncrStateDelta));
        magUpdateCount = 0;
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        // fuse the three magnetometer componenents sequentially
        for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) FuseMagnetometer();
    }

    // Fuse corrections to quaternion, position and velocity states across several time steps to reduce 10Hz pulsing in the output
    if (magUpdateCount < magUpdateCountMax) {
        magUpdateCount ++;
        for (uint8_t i = 0; i <= 9; i++) {
            states[i] += magIncrStateDelta[i];
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

// select fusion of optical flow measurements
void NavEKF_core::SelectFlowFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseOptFlow);
    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // Check if the optical flow sensor has timed out
    bool flowSensorTimeout = ((imuSampleTime_ms - flowValidMeaTime_ms) > 5000);
    // Check if the fusion has timed out (flow measurements have been rejected for too long)
    bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
    // check is the terrain offset estimate is still valid
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000);
    // Perform tilt check
    bool tiltOK = (Tnb_flow.c.z > DCM33FlowMin);
    // Constrain measurements to zero if we are using optical flow and are on the ground
    if (frontend._fusionModeGPS == 3 && !takeOffDetected && vehicleArmed) {
        flowRadXYcomp[0]    = 0.0f;
        flowRadXYcomp[1]    = 0.0f;
        flowRadXY[0]        = 0.0f;
        flowRadXY[1]        = 0.0f;
        omegaAcrossFlowTime.zero();
        flowDataValid = true;
    }
    // If the flow measurements have been rejected for too long and we are relying on them, then revert to constant position mode
    if ((flowSensorTimeout || flowFusionTimeout) && PV_AidingMode == AID_RELATIVE) {
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
    // if we do have valid flow measurements, fuse data into a 1-state EKF to estimate terrain height
    // we don't do terrain height estimation in optical flow only mode as the ground becomes our zero height reference
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
        // because an invalid height above ground estimate will cause the optical flow measurements to fight the GPS
        if (!gpsNotAvailable && !gndOffsetValid) {
            // turn off fusion permissions
            // reset the flags to indicate that no new range finder or flow data is available for fusion
            newDataFlow = false;
        }
    }

    // Fuse optical flow data into the main filter
    // if the filter is initialised, we have data to fuse and the vehicle is not excessively tilted, then perform optical flow fusion
    if (flowDataValid && newDataFlow && tiltOK && !constPosMode)
    {
        // reset state updates and counter used to spread fusion updates across several frames to reduce 10Hz pulsing
        memset(&flowIncrStateDelta[0], 0, sizeof(flowIncrStateDelta));
        flowUpdateCount = 0;
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend._flowNoise, 0.05f));
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        for (flow_state.obsIndex = 0; flow_state.obsIndex <= 1; flow_state.obsIndex++) FuseOptFlow();
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
    // stop the performance timer
    hal.util->perf_end(_perf_FuseOptFlow);
}

// select fusion of true airspeed measurements
void NavEKF_core::SelectTasFusion()
{
    // get true airspeed measurement
    readAirSpdData();

    // If we haven't received airspeed data for a while, then declare the airspeed data as being timed out
    if (imuSampleTime_ms - lastAirspeedUpdate > tasRetryTime) {
        tasTimeout = true;
    }

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion
    tasDataWaiting = (statesInitialised && !inhibitWindStates && newDataTas);
    if (tasDataWaiting)
    {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        FuseAirspeed();
        TASmsecPrev = imuSampleTime_ms;
        tasDataWaiting = false;
        newDataTas = false;
    }
}

// select fusion of synthetic sideslip measurements
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// it requires a stable wind for best results and should not be used for aerobatic flight with manoeuvres that induce large sidslip angles (eg knife-edge, spins, etc)
void NavEKF_core::SelectBetaFusion()
{
    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - BETAmsecPrev) >= msecBetaAvg);
    // set true when use of synthetic sideslip fusion is necessary because we have limited sensor data or are dead reckoning position
    bool f_required = !(use_compass() && useAirspeed() && posHealth);
    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    bool f_feasible = (assume_zero_sideslip() && !inhibitWindStates);
    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_feasible && f_required && f_timeTrigger) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        FuseSideslip();
        BETAmsecPrev = imuSampleTime_ms;
    }
}

// update the quaternion, velocity and position states using IMU measurements
void NavEKF_core::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;  // delta velocity vector calculated using a blend of IMU1 and IMU2 data
    Vector3f delVelNav1; // delta velocity vector calculated using IMU1 data
    Vector3f delVelNav2; // delta velocity vector calculated using IMU2 data

    // remove sensor bias errors
    correctedDelAng = dAngIMU - state.gyro_bias;
    correctedDelVel1 = dVelIMU1;
    correctedDelVel2 = dVelIMU2;
    correctedDelVel1.z -= state.accel_zbias1;
    correctedDelVel2.z -= state.accel_zbias2;

    // use weighted average of both IMU units for delta velocities
    // Over-ride accelerometer blend weighting using a hard switch based on the IMU consistency and vibration monitoring checks
    if (lastImuSwitchState == IMUSWITCH_IMU0) {
        IMU1_weighting = 1.0f;
    } else if (lastImuSwitchState == IMUSWITCH_IMU1) {
        IMU1_weighting = 0.0f;
    }
    correctedDelVel12 = correctedDelVel1 * IMU1_weighting + correctedDelVel2 * (1.0f - IMU1_weighting);
    float dtDelVel = dtDelVel1 * IMU1_weighting + dtDelVel2 * (1.0f - IMU1_weighting);

    // apply correction for earths rotation rate
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*dtDelAng;

    // convert the rotation vector to its equivalent quaternion
    correctedDelAngQuat.from_axis_angle(correctedDelAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    state.quat *= correctedDelAngQuat;
    state.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    state.quat.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // calculate earth frame delta velocity due to gravity
    float delVelGravity1_z = GRAVITY_MSS*dtDelVel1;
    float delVelGravity2_z = GRAVITY_MSS*dtDelVel2;
    float delVelGravity_z = delVelGravity1_z * IMU1_weighting + delVelGravity2_z * (1.0f - IMU1_weighting);

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded

    // blended IMU calc
    delVelNav  = Tbn_temp*correctedDelVel12;
    delVelNav.z += delVelGravity_z;

    // single IMU calcs
    delVelNav1 = Tbn_temp*correctedDelVel1;
    delVelNav1.z += delVelGravity1_z;

    delVelNav2 = Tbn_temp*correctedDelVel2;
    delVelNav2.z += delVelGravity2_z;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / dtDelVel;

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
    state.position += (state.velocity + lastVelocity) * (dtDelVel*0.5f);
    state.posD1    += (state.vel1.z + lastVel1.z) * (dtDelVel1*0.5f);
    state.posD2    += (state.vel2.z + lastVel2.z) * (dtDelVel2*0.5f);

    // capture current angular rate to augmented state vector for use by optical flow fusion
    state.omega = correctedDelAng / dtDelAng;

    // LPF the yaw rate using a 1 second time constant yaw rate and determine if we are doing continual
    // fast rotations that can cause problems due to gyro scale factor errors.
    float alphaLPF = constrain_float(dtDelAng, 0.0f, 1.0f);
    yawRateFilt += (state.omega.z - yawRateFilt)*alphaLPF;
    if (fabsf(yawRateFilt) > 1.0f) {
        highYawRate = true;
    } else {
        highYawRate = false;
    }

    // limit states to protect against divergence
    ConstrainStates();

    // update vertical velocity and position states used to provide a vertical position derivative output
    // using a simple complementary filter
    float lastPosDownDerivative = posDownDerivative;
    posDownDerivative = 2.0f * (state.position.z - posDown);
    posDown += (posDownDerivative + lastPosDownDerivative + 2.0f*delVelNav.z) * (dtDelVel*0.5f);
}

// calculate the predicted state covariance matrix
void NavEKF_core::CovariancePrediction() OPT_MATHS
{
    hal.util->perf_begin(_perf_CovariancePrediction);
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
        windVelSigma  = dt * constrain_float(frontend._windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(frontend._wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    } else {
        windVelSigma  = 0.0f;
    }
    dAngBiasSigma = dt * constrain_float(frontend._gyroBiasProcessNoise, 1e-7f, 1e-5f);
    dVelBiasSigma = dt * constrain_float(frontend._accelBiasProcessNoise, 1e-5f, 1e-3f);
    if (!inhibitMagStates) {
        magEarthSigma = dt * constrain_float(frontend._magEarthProcessNoise, 1e-4f, 1e-2f);
        magBodySigma  = dt * constrain_float(frontend._magBodyProcessNoise, 1e-4f, 1e-2f);
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
    // if we are yawing rapidly, inhibit yaw gyro bias learning to prevent gyro scale factor errors from corrupting the bias estimate
    if (highYawRate) {
        processNoise[12] = 0.0f;
        P[12][12] = 0.0f;
    }
    // scale accel bias noise when disarmed to allow for faster bias estimation
    // inhibit bias estimation during takeoff with ground effect to prevent bad bias learning
    if (expectGndEffectTakeoff) {
        processNoise[13] = 0.0f;
    } else if (!vehicleArmed) {
        processNoise[13] = dVelBiasSigma * accelBiasNoiseScaler;
    } else {
        processNoise[13] = dVelBiasSigma;
    }
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
    frontend._gyrNoise = constrain_float(frontend._gyrNoise, 1e-3f, 5e-2f);
    daxCov = sq(dt*frontend._gyrNoise);
    dayCov = sq(dt*frontend._gyrNoise);
    // Account for 3% scale factor error on Z angular rate. This reduces chance of continuous fast rotations causing loss of yaw reference.
    dazCov = sq(dt*frontend._gyrNoise) + sq(dt*0.03f*yawRateFilt);
    frontend._accNoise = constrain_float(frontend._accNoise, 5e-2f, 1.0f);
    dvxCov = sq(dt*frontend._accNoise);
    dvyCov = sq(dt*frontend._accNoise);
    dvzCov = sq(dt*frontend._accNoise);

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

    // set the flag to indicate that covariance prediction has been performed and reset the increments used by the covariance prediction
    covPredStep = true;
    summedDelAng.zero();
    summedDelVel.zero();
    dt = 0.0f;

    hal.util->perf_end(_perf_CovariancePrediction);
}

// fuse selected position, velocity and height measurements
void NavEKF_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;
    Vector3f velInnov1;
    Vector3f velInnov2;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    float posErr;
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {

        // if constant position mode use the current states to calculate the predicted
        // measurement rather than use states from a previous time. We need to do this
        // because there may be no stored states due to lack of real measurements.
        if (constPosMode) {
            statesAtPosTime = state;
        }

        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed()) gpsRetryTime = gpsRetryTimeUseTAS;
        else gpsRetryTime = gpsRetryTimeNoTAS;

        // form the observation vector and zero velocity and horizontal position observations if in constant position mode
        if (!constPosMode) {
            observation[0] = velNED.x;
            observation[1] = velNED.y;
            observation[2] = velNED.z;
            observation[3] = gpsPosNE.x;
            observation[4] = gpsPosNE.y;
        } else if (constPosMode){
            for (uint8_t i=0; i<=4; i++) observation[i] = 0.0f;
        }
        observation[5] = -hgtMea;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // if the GPS is able to report a speed error, we use it to adjust the observation noise for GPS velocity
        // otherwise we scale it using manoeuvre acceleration
        if (gpsSpdAccuracy > 0.0f) {
            // use GPS receivers reported speed accuracy - floor at value set by gps noise parameter
            R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend._gpsHorizVelNoise, 50.0f));
            R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend._gpsVertVelNoise, 50.0f));
        } else {
            // calculate additional error in GPS velocity caused by manoeuvring
            R_OBS[0] = sq(constrain_float(frontend._gpsHorizVelNoise, 0.05f, 5.0f)) + sq(gpsNEVelVarAccScale * accNavMag);
            R_OBS[2] = sq(constrain_float(frontend._gpsVertVelNoise,  0.05f, 5.0f)) + sq(gpsDVelVarAccScale  * accNavMag);
        }
        R_OBS[1] = R_OBS[0];
        R_OBS[3] = sq(constrain_float(frontend._gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrain_float(frontend._baroAltNoise, 0.1f, 10.0f));

        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if ((getTakeoffExpected() || getTouchdownExpected()) && vehicleArmed) {
            R_OBS[5] *= gndEffectBaroScaler;
        }

        // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
        // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
        // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
        for (uint8_t i=0; i<=1; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend._gpsHorizVelNoise, 0.05f, 5.0f)) + sq(gpsNEVelVarAccScale * accNavMag);
        for (uint8_t i=2; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];


        // if vertical GPS velocity data is being used, check to see if the GPS vertical velocity and barometer
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (useGpsVertVel && fuseVelData && (imuSampleTime_ms - lastHgtMeasTime) <  (2 * msecHgtAvg)) {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr  = statesAtHgtTime.position.z - observation[5];
            float velDErr = statesAtVelTime.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[9][9] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[6][6] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        if (fusePosData) {
            // test horizontal position measurements
            innovVelPos[3] = statesAtPosTime.position.x - observation[3];
            innovVelPos[4] = statesAtPosTime.position.y - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate max valid position innovation squared based on a maximum horizontal inertial nav accel error and GPS noise parameter
            // max inertial nav error is scaled with horizontal g to allow for increased errors when manoeuvring
            float accelScale =  (1.0f + 0.1f * accNavMag);
            float maxPosInnov2 = sq(frontend._gpsPosInnovGate * frontend._gpsHorizPosNoise) + sq(0.005f * accelScale * float(frontend._gpsGlitchAccelMax) * sq(0.001f * float(imuSampleTime_ms - lastPosPassTime)));
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data or not aiding
            posTimeout = (((imuSampleTime_ms - lastPosPassTime) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // use position data if healthy, timed out, or in constant position mode
            if (posHealth || posTimeout || constPosMode) {
                posHealth = true;
                // only reset the failed time and do glitch timeout checks if we are doing full aiding
                if (PV_AidingMode == AID_ABSOLUTE) {
                    lastPosPassTime = imuSampleTime_ms;
                    // if timed out or outside the specified glitch radius, reset to the GPS position
                    if (posTimeout || (maxPosInnov2 > sq(float(frontend._gpsGlitchRadiusMax)))) {
                        // reset the position to the current GPS position
                        ResetPosition();
                        // reset the velocity to the GPS velocity
                        ResetVelocity();
                        // don't fuse data on this time step
                        fusePosData = false;
                        // record the fail time
                        lastPosFailTime = imuSampleTime_ms;
                        // Reset the normalised innovation to avoid false failing the bad position fusion test
                        posTestRatio = 0.0f;
                    }
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            if (frontend._fusionModeGPS == 1) {
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
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // calculate error weightings for single IMU velocity states using
                // observation error to normalise
                float R_hgt;
                if (i == 2) {
                    R_hgt = sq(constrain_float(frontend._gpsVertVelNoise, 0.05f, 5.0f));
                } else {
                    R_hgt = sq(constrain_float(frontend._gpsHorizVelNoise, 0.05f, 5.0f));
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
            // set weighting to 0.5 when on ground to allow more rapid learning of bias errors without 'ringing' in bias estimates
            // NOTE: this weighting can be overwritten in UpdateStrapdownEquationsNED
            if (vehicleArmed) {
                IMU1_weighting = 1.0f * (K1 / (K1 + K2)) + 0.0f * IMU1_weighting; // filter currently inactive
            } else {
                IMU1_weighting = 0.5f;
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(frontend._gpsVelInnovGate));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // declare a timeout if we have not fused velocity data for too long or not aiding
            velTimeout = (((imuSampleTime_ms - lastVelPassTime) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // if data is healthy we fuse it
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count
                lastVelPassTime = imuSampleTime_ms;
            } else if (velTimeout && !posHealth && PV_AidingMode == AID_ABSOLUTE) {
                // if data is not healthy and timed out and position is unhealthy and we are using aiding, we reset the velocity, but do not fuse data on this time step
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
            innovVelPos[5] = statesAtHgtTime.position.z - observation[5];
            // calculate the innovation variance
            varInnovVelPos[5] = P[9][9] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(frontend._hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - lastHgtPassTime_ms) > hgtRetryTime;
            // Fuse height data if healthy
            // Force a reset if timed out to prevent the possibility of inertial errors causing persistent loss of height reference
            // Force fusion in constant position mode on the ground to allow large accelerometer biases to be learned without rejecting barometer
            if (hgtHealth || hgtTimeout || (constPosMode && !vehicleArmed)) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (!vehicleArmed) {
                    float dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms)*1.0e-3f;
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrain_float(dtBaro/(dtBaro+hgtInnovFiltTC),0.0f,1.0f);
                    hgtInnovFiltState += (innovVelPos[5]-hgtInnovFiltState)*alpha;
                }
                // declare height healthy and able to be fused
                lastHgtPassTime_ms = imuSampleTime_ms;
                hgtHealth = true;
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
        if (fuseVelData && useGpsVertVel && velHealth && !constPosMode && PV_AidingMode == AID_ABSOLUTE) {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && frontend._fusionModeGPS == 1 && velHealth && !constPosMode && PV_AidingMode == AID_ABSOLUTE) {
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
                    if (obsIndex == 5) {
                        const float gndMaxBaroErr = 4.0f;
                        const float gndBaroInnovFloor = -0.5f;

                        if(getTouchdownExpected()) {
                            // when a touchdown is expected, floor the barometer innovation at gndBaroInnovFloor
                            // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                            // this function looks like this:
                            //         |/
                            //---------|---------
                            //    ____/|
                            //   /     |
                            //  /      |
                            innovVelPos[5] += constrain_float(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                        }
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=12; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }
                // Only height and height rate observations are used to update z accel bias estimate
                // Protect Kalman gain from ill-conditioning
                // Don't update Z accel bias if off-level by greater than 60 degrees to avoid scale factor error effects
                // Don't update if we are taking off with ground effect
                if ((obsIndex == 5 || obsIndex == 2) && prevTnb.c.z > 0.5f && !getTakeoffExpected()) {
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
                Kfusion[26] = Kfusion[9];  // IMU1 posD
                Kfusion[30] = Kfusion[9];  // IMU2 posD
                for (uint8_t i = 0; i<=2; i++) {
                    Kfusion[i+23] = Kfusion[i+4]; // IMU1 velNED
                    Kfusion[i+27] = Kfusion[i+4]; // IMU2 velNED
                }
                // Don't update Z accel bias values for an acceleraometer we have hard switched away from
                if ((IMU1_weighting >= 0.1f) && (IMU1_weighting <= 0.9f)) {
                    // both IMU's OK
                    Kfusion[22] = Kfusion[13];
                } else if (IMU1_weighting < 0.1f) {
                    // IMU1 bad
                    Kfusion[22] = Kfusion[13];
                    Kfusion[13] = 0.0f;
                } else {
                    // IMU2 bad
                    Kfusion[22] = 0.0f;
                }

                // Correct states that have been predicted using single (not blended) IMU data
                if (obsIndex == 5){
                    // Calculate height measurement innovations using single IMU states
                    float hgtInnov1 = statesAtHgtTime.posD1 - observation[obsIndex];
                    float hgtInnov2 = statesAtHgtTime.posD2 - observation[obsIndex];

                    if (vehicleArmed) {
                        // Correct single IMU prediction states using height measurement, limiting rate of change of bias to 0.005 m/s3
                        float correctionLimit = 0.005f * dtIMUavg * dtVelPos;
                        state.accel_zbias1 -= constrain_float(Kfusion[13] * hgtInnov1, -correctionLimit, correctionLimit); // IMU1 Z accel bias
                        state.accel_zbias2 -= constrain_float(Kfusion[22] * hgtInnov2, -correctionLimit, correctionLimit); // IMU2 Z accel bias
                    } else {
                        // When disarmed, do not rate limit accel bias learning
                        state.accel_zbias1 -= Kfusion[13] * hgtInnov1; // IMU1 Z accel bias
                        state.accel_zbias2 -= Kfusion[22] * hgtInnov2; // IMU2 Z accel bias
                    }

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
                // Don't apply corrections to Z bias state as this has been done already as part of the single IMU calculations
                bool highRates = ((gpsUpdateCountMax * correctedDelAng.length()) > 0.1f);
                for (uint8_t i = 0; i<=21; i++) {
                    if (i != 13) {
                        if ((i <= 3 && highRates) || i >= 10 || constPosMode) {
                            states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                        } else {
                            if (obsIndex == 5) {
                                hgtIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * hgtUpdateCountMaxInv;
                            } else {
                                gpsIncrStateDelta[i] -= Kfusion[i] * innovVelPos[obsIndex] * gpsUpdateCountMaxInv;
                            }
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
    hal.util->perf_end(_perf_FuseVelPosNED);
}

// fuse magnetometer measurements and apply innovation consistency checks
// fuse each axis on consecutive time steps to spread computional load
void NavEKF_core::FuseMagnetometer()
{
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
    // calculate observation jacobians and Kalman gains
    if (obsIndex == 0)
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
        R_MAG = sq(constrain_float(frontend._magNoise, 0.01f, 0.5f)) + sq(magVarRateScale*dAngIMU.length() / dtIMUavg);

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
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
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
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
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
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
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
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
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
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
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
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
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
    magTestRatio[obsIndex] = sq(innovMag[obsIndex]) / (sq(frontend._magInnovGate) * varInnovMag[obsIndex]);
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
            // If we are forced to use a bad compass in flight, we reduce the weighting by a factor of 4
            if (!magHealth && !constPosMode) {
                Kfusion[j] *= 0.25f;
            }
            // If in the air and there is no other form of heading reference or we are yawing rapidly which creates larger inertial yaw errors,
            // we strengthen the magnetometer attitude correction
            if (vehicleArmed && (constPosMode || highYawRate) && j <= 3) {
                Kfusion[j] *= 4.0f;
            }
            // We don't need to spread corrections for non-dynamic states or if we are in a  constant postion mode
            // We can't spread corrections if there is not enough time remaining
            // We don't spread corrections to attitude states if we are rotating rapidly
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

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optiocal flow rates and range finder measurements
*/
void NavEKF_core::EstimateTerrainOffset()
{
    // start performance timer
    hal.util->perf_begin(_perf_OpticalFlowEKF);

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = MAX((terrainState - state.position.z), rngOnGnd);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(state.velocity.x) + sq(state.velocity.y);
    float losRateSq = velHorizSq / sq(heightAboveGndEst);

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
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = statesAtRngTime.position[0];
        prevPosE = statesAtRngTime.position[1];

        // in addition to a terrain gradient error model, we also have a time based error growth that is scaled using the gradient parameter
        float timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(0.01f*float(frontend._gndGradientSigma))) + sq(float(frontend._gndGradientSigma) * timeLapsed);
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (fuseRngData) {
            // predict range
            float predRngMeas = MAX((terrainState - statesAtRngTime.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // Copy required states to local variable names
            float q0 = statesAtRngTime.quat[0]; // quaternion at optical flow measurement time
            float q1 = statesAtRngTime.quat[1]; // quaternion at optical flow measurement time
            float q2 = statesAtRngTime.quat[2]; // quaternion at optical flow measurement time
            float q3 = statesAtRngTime.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = 0.5f;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, statesAtRngTime.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rngMea;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(frontend._rngInnovGate) * varInnovRng);

            // Check the innovation for consistency and don't fuse if > 5Sigma
            if ((sq(innovRng)*SK_RNG) < 25.0f)
            {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = MAX(terrainState, statesAtRngTime.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = MAX(Popt,0.0f);

            }
        }

        if (fuseOptFlowData) {

            Vector3f relVelSensor; // velocity of sensor relative to ground in sensor axes
            float losPred; // predicted optical flow angular rate measurement
            float q0 = statesAtFlowTime.quat[0]; // quaternion at optical flow measurement time
            float q1 = statesAtFlowTime.quat[1]; // quaternion at optical flow measurement time
            float q2 = statesAtFlowTime.quat[2]; // quaternion at optical flow measurement time
            float q3 = statesAtFlowTime.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;

            // predict range to centre of image
            float flowRngPred = MAX((terrainState - statesAtFlowTime.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, statesAtFlowTime.position[2] + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = Tnb_flow*statesAtFlowTime.velocity;

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
            float t15 = t14*statesAtFlowTime.velocity.x;
            float t16 = t10+t11;
            float t17 = t16*statesAtFlowTime.velocity.y;
            float t18 = q0*q2*2.0f;
            float t19 = q1*q3*2.0f;
            float t20 = t18-t19;
            float t21 = t20*statesAtFlowTime.velocity.z;
            float t2 = t15+t17-t21;
            float t7 = t3-t4-t5+t6;
            float t8 = statesAtFlowTime.position[2]-terrainState;
            float t9 = 1.0f/sq(t8);
            float t24 = t3-t4+t5-t6;
            float t25 = t24*statesAtFlowTime.velocity.y;
            float t26 = t10-t11;
            float t27 = t26*statesAtFlowTime.velocity.x;
            float t28 = q0*q1*2.0f;
            float t29 = q2*q3*2.0f;
            float t30 = t28+t29;
            float t31 = t30*statesAtFlowTime.velocity.z;
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
            auxFlowTestRatio = sq(auxFlowObsInnov) / (sq(frontend._flowInnovGate) * auxFlowObsInnovVar);

            // don't fuse if optical flow data is outside valid range
            if (MAX(flowRadXY[0],flowRadXY[1]) < frontend._maxFlowRate) {

            // correct the state
            terrainState -= K_OPT * auxFlowObsInnov;

            // constrain the state
            terrainState = MAX(terrainState, statesAtFlowTime.position[2] + rngOnGnd);

            // correct the covariance
            Popt = Popt - K_OPT * H_OPT * Popt;

            // prevent the state variances from becoming negative
            Popt = MAX(Popt,0.0f);
            }
        }
    }

    // stop the performance timer
    hal.util->perf_end(_perf_OpticalFlowEKF);
}

void NavEKF_core::FuseOptFlow()
{
    Vector22 H_LOS;
    Vector8 tempVar;
    Vector3f relVelSensor;

    uint8_t obsIndex = flow_state.obsIndex;
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

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);
    // Calculate observation jacobians and Kalman gains
    if (obsIndex == 0) {
        // calculate range from ground plain to centre of sensor fov assuming flat earth
        float range = constrain_float((heightAboveGndEst/Tnb_flow.c.z),rngOnGnd,1000.0f);

        // calculate relative velocity in sensor frame
        relVelSensor = Tnb_flow*statesAtFlowTime.velocity;

        // divide velocity by range  to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;


        // Calculate common expressions for observation jacobians
        SH_LOS[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        SH_LOS[1] = vn*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + ve*(2*q0*q3 + 2*q1*q2);
        SH_LOS[2] = ve*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - vn*(2*q0*q3 - 2*q1*q2);
        SH_LOS[3] = -1.0f/heightAboveGndEst;

        // Calculate common expressions for Kalman gains
        // calculate innovation variance for Y axis observation
        varInnovOptFlow[1] = (R_LOS + (SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3])*(P[0][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][0]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][0]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][0]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][0]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3])*(P[0][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][1]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][1]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][1]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][1]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3])*(P[0][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][2]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][2]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][2]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][2]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3])*(P[0][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][3]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][3]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][3]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][3]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))*(P[0][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][4]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][4]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][4]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][4]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2)*(P[0][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][5]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][5]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][5]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][5]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3)*(P[0][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][6]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][6]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][6]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][6]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[1]*(P[0][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3]) + P[1][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3]) - P[2][9]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) + 2*q2*SH_LOS[1]*SH_LOS[3]) + P[3][9]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3]) + P[5][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2) - P[6][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3) - (P[9][9]*SH_LOS[0]*SH_LOS[1])/sq(pd - terrainState) + P[4][9]*SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3))))/sq(pd - terrainState));
        if (varInnovOptFlow[1] > R_LOS) {
            SK_LOS[0] = 1.0f/varInnovOptFlow[1];
        } else {
            SK_LOS[0] = 1.0f/R_LOS;
        }
        // calculate innovation variance for X axis observation
        varInnovOptFlow[0] = (R_LOS + (SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3])*(P[0][0]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][0]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][0]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][0]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][0]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][0]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3])*(P[0][1]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][1]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][1]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][1]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][1]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][1]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + (SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3])*(P[0][2]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][2]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][2]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][2]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][2]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][2]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3])*(P[0][3]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][3]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][3]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][3]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][3]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][3]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))*(P[0][5]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][5]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][5]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][5]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][5]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][5]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2)*(P[0][4]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][4]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][4]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][4]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][4]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][4]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) + SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3)*(P[0][6]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][6]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][6]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][6]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][6]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][6]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))) - (SH_LOS[0]*SH_LOS[2]*(P[0][9]*(SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q0*SH_LOS[2]*SH_LOS[3]) + P[1][9]*(SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q1*SH_LOS[2]*SH_LOS[3]) + P[2][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q2*SH_LOS[2]*SH_LOS[3]) - P[3][9]*(SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3]) - P[4][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2) + P[6][9]*SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3) - (P[9][9]*SH_LOS[0]*SH_LOS[2])/sq(pd - terrainState) + P[5][9]*SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3))))/sq(pd - terrainState));
        if (varInnovOptFlow[0] > R_LOS) {
            SK_LOS[1] = 1.0f/varInnovOptFlow[0];
        } else {
            SK_LOS[1] = 1.0f/R_LOS;
        }
        SK_LOS[2] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn);
        SK_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
        SK_LOS[4] = SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn);
        SK_LOS[5] = SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
        SK_LOS[6] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SK_LOS[7] = 1.0f/sq(heightAboveGndEst);
        SK_LOS[8] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SK_LOS[9] = SH_LOS[3];

        // Calculate common intermediate terms
        tempVar[0] = SK_LOS[4] + 2*q0*SH_LOS[2]*SK_LOS[9];
        tempVar[1] = SK_LOS[3] - 2*q1*SH_LOS[2]*SK_LOS[9];
        tempVar[2] = SK_LOS[2] - 2*q3*SH_LOS[2]*SK_LOS[9];
        tempVar[3] = SH_LOS[0]*SK_LOS[9]*(2*q0*q3 - 2*q1*q2);
        tempVar[4] = SH_LOS[0]*SK_LOS[9]*(2*q0*q1 + 2*q2*q3);
        tempVar[5] = SH_LOS[0]*SH_LOS[2]*SK_LOS[7];
        tempVar[6] = SH_LOS[0]*SK_LOS[6]*SK_LOS[9];
        tempVar[7] = SK_LOS[5] - 2*q2*SH_LOS[2]*SK_LOS[9];

        // calculate observation jacobians for X LOS rate
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        H_LOS[0] = - SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) - 2*q0*SH_LOS[2]*SH_LOS[3];
        H_LOS[1] = 2*q1*SH_LOS[2]*SH_LOS[3] - SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn);
        H_LOS[2] = 2*q2*SH_LOS[2]*SH_LOS[3] - SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn);
        H_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) - 2*q3*SH_LOS[2]*SH_LOS[3];
        H_LOS[4] = SH_LOS[0]*SH_LOS[3]*(2*q0*q3 - 2*q1*q2);
        H_LOS[5] = -SH_LOS[0]*SH_LOS[3]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        H_LOS[6] = -SH_LOS[0]*SH_LOS[3]*(2*q0*q1 + 2*q2*q3);
        H_LOS[9] = (SH_LOS[0]*SH_LOS[2])/sq(heightAboveGndEst);

        // calculate Kalman gains for X LOS rate
        Kfusion[0] = -SK_LOS[1]*(P[0][0]*tempVar[0] + P[0][1]*tempVar[1] - P[0][3]*tempVar[2] + P[0][2]*tempVar[7] - P[0][4]*tempVar[3] + P[0][6]*tempVar[4] - P[0][9]*tempVar[5] + P[0][5]*tempVar[6]);
        Kfusion[1] = -SK_LOS[1]*(P[1][0]*tempVar[0] + P[1][1]*tempVar[1] - P[1][3]*tempVar[2] + P[1][2]*tempVar[7] - P[1][4]*tempVar[3] + P[1][6]*tempVar[4] - P[1][9]*tempVar[5] + P[1][5]*tempVar[6]);
        Kfusion[2] = -SK_LOS[1]*(P[2][0]*tempVar[0] + P[2][1]*tempVar[1] - P[2][3]*tempVar[2] + P[2][2]*tempVar[7] - P[2][4]*tempVar[3] + P[2][6]*tempVar[4] - P[2][9]*tempVar[5] + P[2][5]*tempVar[6]);
        Kfusion[3] = -SK_LOS[1]*(P[3][0]*tempVar[0] + P[3][1]*tempVar[1] - P[3][3]*tempVar[2] + P[3][2]*tempVar[7] - P[3][4]*tempVar[3] + P[3][6]*tempVar[4] - P[3][9]*tempVar[5] + P[3][5]*tempVar[6]);
        Kfusion[4] = -SK_LOS[1]*(P[4][0]*tempVar[0] + P[4][1]*tempVar[1] - P[4][3]*tempVar[2] + P[4][2]*tempVar[7] - P[4][4]*tempVar[3] + P[4][6]*tempVar[4] - P[4][9]*tempVar[5] + P[4][5]*tempVar[6]);
        Kfusion[5] = -SK_LOS[1]*(P[5][0]*tempVar[0] + P[5][1]*tempVar[1] - P[5][3]*tempVar[2] + P[5][2]*tempVar[7] - P[5][4]*tempVar[3] + P[5][6]*tempVar[4] - P[5][9]*tempVar[5] + P[5][5]*tempVar[6]);
        // Don't allow optical flow measurements to modify vertical velocity as it can produce height offsets
        Kfusion[6] = 0.0f;//-SK_LOS[1]*(P[6][0]*tempVar[0] + P[6][1]*tempVar[1] - P[6][3]*tempVar[2] + P[6][2]*tempVar[7] - P[6][4]*tempVar[3] + P[6][6]*tempVar[4] - P[6][9]*tempVar[5] + P[6][5]*tempVar[6]);
        Kfusion[7] = -SK_LOS[1]*(P[7][0]*tempVar[0] + P[7][1]*tempVar[1] - P[7][3]*tempVar[2] + P[7][2]*tempVar[7] - P[7][4]*tempVar[3] + P[7][6]*tempVar[4] - P[7][9]*tempVar[5] + P[7][5]*tempVar[6]);
        Kfusion[8] = -SK_LOS[1]*(P[8][0]*tempVar[0] + P[8][1]*tempVar[1] - P[8][3]*tempVar[2] + P[8][2]*tempVar[7] - P[8][4]*tempVar[3] + P[8][6]*tempVar[4] - P[8][9]*tempVar[5] + P[8][5]*tempVar[6]);
        // Don't allow optical flow measurements to modify vertical position as it can produce height offsets
        Kfusion[9] = 0.0f;//-SK_LOS[1]*(P[9][0]*tempVar[0] + P[9][1]*tempVar[1] - P[9][3]*tempVar[2] + P[9][2]*tempVar[7] - P[9][4]*tempVar[3] + P[9][6]*tempVar[4] - P[9][9]*tempVar[5] + P[9][5]*tempVar[6]);
        Kfusion[10] = -SK_LOS[1]*(P[10][0]*tempVar[0] + P[10][1]*tempVar[1] - P[10][3]*tempVar[2] + P[10][2]*tempVar[7] - P[10][4]*tempVar[3] + P[10][6]*tempVar[4] - P[10][9]*tempVar[5] + P[10][5]*tempVar[6]);
        Kfusion[11] = -SK_LOS[1]*(P[11][0]*tempVar[0] + P[11][1]*tempVar[1] - P[11][3]*tempVar[2] + P[11][2]*tempVar[7] - P[11][4]*tempVar[3] + P[11][6]*tempVar[4] - P[11][9]*tempVar[5] + P[11][5]*tempVar[6]);
        Kfusion[12] = -SK_LOS[1]*(P[12][0]*tempVar[0] + P[12][1]*tempVar[1] - P[12][3]*tempVar[2] + P[12][2]*tempVar[7] - P[12][4]*tempVar[3] + P[12][6]*tempVar[4] - P[12][9]*tempVar[5] + P[12][5]*tempVar[6]);
        // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
        Kfusion[13] = 0.0f;//Kfusion[13] = -SK_LOS[1]*(P[13][0]*tempVar[0] + P[13][1]*tempVar[1] - P[13][3]*tempVar[2] + P[13][2]*tempVar[7] - P[13][4]*tempVar[3] + P[13][6]*tempVar[4] - P[13][9]*tempVar[5] + P[13][5]*tempVar[6]);
        if (inhibitWindStates) {
            Kfusion[14] = -SK_LOS[1]*(P[14][0]*tempVar[0] + P[14][1]*tempVar[1] - P[14][3]*tempVar[2] + P[14][2]*tempVar[7] - P[14][4]*tempVar[3] + P[14][6]*tempVar[4] - P[14][9]*tempVar[5] + P[14][5]*tempVar[6]);
            Kfusion[15] = -SK_LOS[1]*(P[15][0]*tempVar[0] + P[15][1]*tempVar[1] - P[15][3]*tempVar[2] + P[15][2]*tempVar[7] - P[15][4]*tempVar[3] + P[15][6]*tempVar[4] - P[15][9]*tempVar[5] + P[15][5]*tempVar[6]);
        } else {
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
        }
        if (inhibitMagStates) {
            Kfusion[16] = -SK_LOS[1]*(P[16][0]*tempVar[0] + P[16][1]*tempVar[1] - P[16][3]*tempVar[2] + P[16][2]*tempVar[7] - P[16][4]*tempVar[3] + P[16][6]*tempVar[4] - P[16][9]*tempVar[5] + P[16][5]*tempVar[6]);
            Kfusion[17] = -SK_LOS[1]*(P[17][0]*tempVar[0] + P[17][1]*tempVar[1] - P[17][3]*tempVar[2] + P[17][2]*tempVar[7] - P[17][4]*tempVar[3] + P[17][6]*tempVar[4] - P[17][9]*tempVar[5] + P[17][5]*tempVar[6]);
            Kfusion[18] = -SK_LOS[1]*(P[18][0]*tempVar[0] + P[18][1]*tempVar[1] - P[18][3]*tempVar[2] + P[18][2]*tempVar[7] - P[18][4]*tempVar[3] + P[18][6]*tempVar[4] - P[18][9]*tempVar[5] + P[18][5]*tempVar[6]);
            Kfusion[19] = -SK_LOS[1]*(P[19][0]*tempVar[0] + P[19][1]*tempVar[1] - P[19][3]*tempVar[2] + P[19][2]*tempVar[7] - P[19][4]*tempVar[3] + P[19][6]*tempVar[4] - P[19][9]*tempVar[5] + P[19][5]*tempVar[6]);
            Kfusion[20] = -SK_LOS[1]*(P[20][0]*tempVar[0] + P[20][1]*tempVar[1] - P[20][3]*tempVar[2] + P[20][2]*tempVar[7] - P[20][4]*tempVar[3] + P[20][6]*tempVar[4] - P[20][9]*tempVar[5] + P[20][5]*tempVar[6]);
            Kfusion[21] = -SK_LOS[1]*(P[21][0]*tempVar[0] + P[21][1]*tempVar[1] - P[21][3]*tempVar[2] + P[21][2]*tempVar[7] - P[21][4]*tempVar[3] + P[21][6]*tempVar[4] - P[21][9]*tempVar[5] + P[21][5]*tempVar[6]);
        } else {
            for (uint8_t i = 16; i <= 21; i++) {
                Kfusion[i] = 0.0f;
            }
        }
        // calculate innovation for X axis observation
        innovOptFlow[0] = losPred[0] - flowRadXYcomp[0];

    } else if (obsIndex == 1) {

        // calculate intermediate common variables
        tempVar[0] = SK_LOS[2] + 2*q0*SH_LOS[1]*SK_LOS[9];
        tempVar[1] = SK_LOS[5] - 2*q1*SH_LOS[1]*SK_LOS[9];
        tempVar[2] = SK_LOS[3] + 2*q2*SH_LOS[1]*SK_LOS[9];
        tempVar[3] = SK_LOS[4] + 2*q3*SH_LOS[1]*SK_LOS[9];
        tempVar[4] = SH_LOS[0]*SK_LOS[9]*(2*q0*q3 + 2*q1*q2);
        tempVar[5] = SH_LOS[0]*SK_LOS[9]*(2*q0*q2 - 2*q1*q3);
        tempVar[6] = SH_LOS[0]*SH_LOS[1]*SK_LOS[7];
        tempVar[7] = SH_LOS[0]*SK_LOS[8]*SK_LOS[9];

        // Calculate observation jacobians for Y LOS rate
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        H_LOS[0] = SH_LOS[0]*SH_LOS[3]*(2*q3*ve - 2*q2*vd + 2*q0*vn) + 2*q0*SH_LOS[1]*SH_LOS[3];
        H_LOS[1] = SH_LOS[0]*SH_LOS[3]*(2*q3*vd + 2*q2*ve + 2*q1*vn) - 2*q1*SH_LOS[1]*SH_LOS[3];
        H_LOS[2] = - SH_LOS[0]*SH_LOS[3]*(2*q0*vd - 2*q1*ve + 2*q2*vn) - 2*q2*SH_LOS[1]*SH_LOS[3];
        H_LOS[3] = SH_LOS[0]*SH_LOS[3]*(2*q1*vd + 2*q0*ve - 2*q3*vn) + 2*q3*SH_LOS[1]*SH_LOS[3];
        H_LOS[4] = SH_LOS[0]*SH_LOS[3]*(sq(q0) + sq(q1) - sq(q2) - sq(q3));
        H_LOS[5] = SH_LOS[0]*SH_LOS[3]*(2*q0*q3 + 2*q1*q2);
        H_LOS[6] = -SH_LOS[0]*SH_LOS[3]*(2*q0*q2 - 2*q1*q3);
        H_LOS[9] = -(SH_LOS[0]*SH_LOS[1])/sq(heightAboveGndEst);

        // Calculate Kalman gains for Y LOS rate
        Kfusion[0] = SK_LOS[0]*(P[0][0]*tempVar[0] + P[0][1]*tempVar[1] - P[0][2]*tempVar[2] + P[0][3]*tempVar[3] + P[0][5]*tempVar[4] - P[0][6]*tempVar[5] - P[0][9]*tempVar[6] + P[0][4]*tempVar[7]);
        Kfusion[1] = SK_LOS[0]*(P[1][0]*tempVar[0] + P[1][1]*tempVar[1] - P[1][2]*tempVar[2] + P[1][3]*tempVar[3] + P[1][5]*tempVar[4] - P[1][6]*tempVar[5] - P[1][9]*tempVar[6] + P[1][4]*tempVar[7]);
        Kfusion[2] = SK_LOS[0]*(P[2][0]*tempVar[0] + P[2][1]*tempVar[1] - P[2][2]*tempVar[2] + P[2][3]*tempVar[3] + P[2][5]*tempVar[4] - P[2][6]*tempVar[5] - P[2][9]*tempVar[6] + P[2][4]*tempVar[7]);
        Kfusion[3] = SK_LOS[0]*(P[3][0]*tempVar[0] + P[3][1]*tempVar[1] - P[3][2]*tempVar[2] + P[3][3]*tempVar[3] + P[3][5]*tempVar[4] - P[3][6]*tempVar[5] - P[3][9]*tempVar[6] + P[3][4]*tempVar[7]);
        Kfusion[4] = SK_LOS[0]*(P[4][0]*tempVar[0] + P[4][1]*tempVar[1] - P[4][2]*tempVar[2] + P[4][3]*tempVar[3] + P[4][5]*tempVar[4] - P[4][6]*tempVar[5] - P[4][9]*tempVar[6] + P[4][4]*tempVar[7]);
        Kfusion[5] = SK_LOS[0]*(P[5][0]*tempVar[0] + P[5][1]*tempVar[1] - P[5][2]*tempVar[2] + P[5][3]*tempVar[3] + P[5][5]*tempVar[4] - P[5][6]*tempVar[5] - P[5][9]*tempVar[6] + P[5][4]*tempVar[7]);
        // Don't allow optical flow measurements to modify vertical velocity as it can produce height offsets
        Kfusion[6] = 0.0f;//SK_LOS[0]*(P[6][0]*tempVar[0] + P[6][1]*tempVar[1] - P[6][2]*tempVar[2] + P[6][3]*tempVar[3] + P[6][5]*tempVar[4] - P[6][6]*tempVar[5] - P[6][9]*tempVar[6] + P[6][4]*tempVar[7]);
        Kfusion[7] = SK_LOS[0]*(P[7][0]*tempVar[0] + P[7][1]*tempVar[1] - P[7][2]*tempVar[2] + P[7][3]*tempVar[3] + P[7][5]*tempVar[4] - P[7][6]*tempVar[5] - P[7][9]*tempVar[6] + P[7][4]*tempVar[7]);
        Kfusion[8] = SK_LOS[0]*(P[8][0]*tempVar[0] + P[8][1]*tempVar[1] - P[8][2]*tempVar[2] + P[8][3]*tempVar[3] + P[8][5]*tempVar[4] - P[8][6]*tempVar[5] - P[8][9]*tempVar[6] + P[8][4]*tempVar[7]);
        // Don't allow optical flow measurements to modify vertical position as it can produce height offsets
        Kfusion[9] = 0.0f;//SK_LOS[0]*(P[9][0]*tempVar[0] + P[9][1]*tempVar[1] - P[9][2]*tempVar[2] + P[9][3]*tempVar[3] + P[9][5]*tempVar[4] - P[9][6]*tempVar[5] - P[9][9]*tempVar[6] + P[9][4]*tempVar[7]);
        Kfusion[10] = SK_LOS[0]*(P[10][0]*tempVar[0] + P[10][1]*tempVar[1] - P[10][2]*tempVar[2] + P[10][3]*tempVar[3] + P[10][5]*tempVar[4] - P[10][6]*tempVar[5] - P[10][9]*tempVar[6] + P[10][4]*tempVar[7]);
        Kfusion[11] = SK_LOS[0]*(P[11][0]*tempVar[0] + P[11][1]*tempVar[1] - P[11][2]*tempVar[2] + P[11][3]*tempVar[3] + P[11][5]*tempVar[4] - P[11][6]*tempVar[5] - P[11][9]*tempVar[6] + P[11][4]*tempVar[7]);
        Kfusion[12] = SK_LOS[0]*(P[12][0]*tempVar[0] + P[12][1]*tempVar[1] - P[12][2]*tempVar[2] + P[12][3]*tempVar[3] + P[12][5]*tempVar[4] - P[12][6]*tempVar[5] - P[12][9]*tempVar[6] + P[12][4]*tempVar[7]);
        // only height measurements are allowed to modify the Z bias state to improve the stability of the estimate
        Kfusion[13] = 0.0f;//SK_LOS[0]*(P[13][0]*tempVar[0] + P[13][1]*tempVar[1] - P[13][2]*tempVar[2] + P[13][3]*tempVar[3] + P[13][5]*tempVar[4] - P[13][6]*tempVar[5] - P[13][9]*tempVar[6] + P[13][4]*tempVar[7]);
        if (inhibitWindStates) {
            Kfusion[14] = SK_LOS[0]*(P[14][0]*tempVar[0] + P[14][1]*tempVar[1] - P[14][2]*tempVar[2] + P[14][3]*tempVar[3] + P[14][5]*tempVar[4] - P[14][6]*tempVar[5] - P[14][9]*tempVar[6] + P[14][4]*tempVar[7]);
            Kfusion[15] = SK_LOS[0]*(P[15][0]*tempVar[0] + P[15][1]*tempVar[1] - P[15][2]*tempVar[2] + P[15][3]*tempVar[3] + P[15][5]*tempVar[4] - P[15][6]*tempVar[5] - P[15][9]*tempVar[6] + P[15][4]*tempVar[7]);
        } else {
            Kfusion[14] = 0.0f;
            Kfusion[15] = 0.0f;
        }
        if (inhibitMagStates) {
            Kfusion[16] = SK_LOS[0]*(P[16][0]*tempVar[0] + P[16][1]*tempVar[1] - P[16][2]*tempVar[2] + P[16][3]*tempVar[3] + P[16][5]*tempVar[4] - P[16][6]*tempVar[5] - P[16][9]*tempVar[6] + P[16][4]*tempVar[7]);
            Kfusion[17] = SK_LOS[0]*(P[17][0]*tempVar[0] + P[17][1]*tempVar[1] - P[17][2]*tempVar[2] + P[17][3]*tempVar[3] + P[17][5]*tempVar[4] - P[17][6]*tempVar[5] - P[17][9]*tempVar[6] + P[17][4]*tempVar[7]);
            Kfusion[18] = SK_LOS[0]*(P[18][0]*tempVar[0] + P[18][1]*tempVar[1] - P[18][2]*tempVar[2] + P[18][3]*tempVar[3] + P[18][5]*tempVar[4] - P[18][6]*tempVar[5] - P[18][9]*tempVar[6] + P[18][4]*tempVar[7]);
            Kfusion[19] = SK_LOS[0]*(P[19][0]*tempVar[0] + P[19][1]*tempVar[1] - P[19][2]*tempVar[2] + P[19][3]*tempVar[3] + P[19][5]*tempVar[4] - P[19][6]*tempVar[5] - P[19][9]*tempVar[6] + P[19][4]*tempVar[7]);
            Kfusion[20] = SK_LOS[0]*(P[20][0]*tempVar[0] + P[20][1]*tempVar[1] - P[20][2]*tempVar[2] + P[20][3]*tempVar[3] + P[20][5]*tempVar[4] - P[20][6]*tempVar[5] - P[20][9]*tempVar[6] + P[20][4]*tempVar[7]);
            Kfusion[21] = SK_LOS[0]*(P[21][0]*tempVar[0] + P[21][1]*tempVar[1] - P[21][2]*tempVar[2] + P[21][3]*tempVar[3] + P[21][5]*tempVar[4] - P[21][6]*tempVar[5] - P[21][9]*tempVar[6] + P[21][4]*tempVar[7]);
        } else {
            for (uint8_t i = 16; i <= 21; i++) {
                Kfusion[i] = 0.0f;
            }
        }
        // calculate innovation for Y observation
        innovOptFlow[1] = losPred[1] - flowRadXYcomp[1];

    }

    // calculate the innovation consistency test ratio
    flowTestRatio[obsIndex] = sq(innovOptFlow[obsIndex]) / (sq(frontend._flowInnovGate) * varInnovOptFlow[obsIndex]);

    // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
    if ((flowTestRatio[obsIndex]) < 1.0f && (flowRadXY[obsIndex] < frontend._maxFlowRate)) {
        // record the last time both X and Y observations were accepted for fusion
        if (obsIndex == 0) {
            flowXfailed = false;
        } else if (!flowXfailed) {
            prevFlowFuseTime_ms = imuSampleTime_ms;
        }
        // Attitude, velocity and position corrections are averaged across multiple prediction cycles between now and the anticipated time for the next measurement.
        // Don't do averaging of quaternion state corrections if total angle change across predicted interval is going to exceed 0.1 rad
        bool highRates = ((flowUpdateCountMax * correctedDelAng.length()) > 0.1f);
        // Calculate the number of averaging frames left to go.
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

}

// fuse true airspeed measurements
void NavEKF_core::FuseAirspeed()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseAirspeed);

    // declarations
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float EAS2TAS = _ahrs->get_EAS2TAS();
    const float R_TAS = sq(constrain_float(frontend._easNoise, 0.5f, 5.0f) * constrain_float(EAS2TAS, 0.9f, 10.0f));
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
        tasTestRatio = sq(innovVtas) / (sq(frontend._tasInnovGate) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        tasHealth = ((tasTestRatio < 1.0f) || badIMUdata);
        tasTimeout = (imuSampleTime_ms - lastTasPassTime) > tasRetryTime;

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasHealth || (tasTimeout && posTimeout))
        {

            // restart the counter
            lastTasPassTime = imuSampleTime_ms;

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
                for (uint8_t j = 0; j<=3; j++) KH[i][j] = 0.0f;
                for (uint8_t j = 4; j<=6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 7; j<=13; j++) KH[i][j] = 0.0f;
                for (uint8_t j = 14; j<=15; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 16; j<=21; j++) KH[i][j] = 0.0f;
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
    hal.util->perf_end(_perf_FuseAirspeed);
}

// fuse sythetic sideslip measurement of zero
void NavEKF_core::FuseSideslip()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseSideslip);

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

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn ;
    vel_rel_wind.y = ve - vwe;
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
            for (uint8_t j = 7; j<=13; j++) KH[i][j] = 0.0f;
            for (uint8_t j = 14; j<=15; j++)
            {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (uint8_t j = 16; j<=21; j++) KH[i][j] = 0.0f;
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
    hal.util->perf_end(_perf_FuseSideslip);
}

// zero specified range of rows in the state covariance matrix
void NavEKF_core::zeroRows(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*22);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF_core::zeroCols(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=21; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// store states in a history array along with time stamp
void NavEKF_core::StoreStates()
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
void NavEKF_core::StoreStatesReset()
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
void NavEKF_core::RecallStates(state_elements &statesForFusion, uint32_t msec)
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
void NavEKF_core::RecallOmega(Vector3f &omegaAvg, uint32_t msecStart, uint32_t msecEnd)
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
    } else if (dtDelAng > 0) {
        omegaAvg = correctedDelAng / dtDelAng;
    } else {
        omegaAvg.zero();
    }
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF_core::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// return the Euler roll, pitch and yaw angle in radians
void NavEKF_core::getEulerAngles(Vector3f &euler) const
{
    state.quat.to_euler(euler.x, euler.y, euler.z);
    euler = euler - _ahrs->get_trim();
}

// This returns the specific forces in the NED frame
void NavEKF_core::getAccelNED(Vector3f &accelNED) const {
    accelNED = velDotNED;
    accelNED.z -= GRAVITY_MSS;
}

// return NED velocity in m/s
//
void NavEKF_core::getVelNED(Vector3f &vel) const
{
    vel = state.velocity;
}

// Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
float NavEKF_core::getPosDownDerivative(void) const
{
    // return the value calculated from a complmentary filer applied to the EKF height and vertical acceleration
    return posDownDerivative;
}

// Return the last calculated NED position relative to the reference point (m).
// if a calculated solution is not available, use the best available data and return false
bool NavEKF_core::getPosNED(Vector3f &pos) const
{
    // The EKF always has a height estimate regardless of mode of operation
    pos.z = state.position.z;
    // There are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no position estimate available)
    nav_filter_status status;
    getFilterStatus(status);
    if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
        // This is the normal mode of operation where we can use the EKF position states
        pos.x = state.position.x;
        pos.y = state.position.y;
        return true;
    } else {
        // In constant position mode the EKF position states are at the origin, so we cannot use them as a position estimate
        if(validOrigin) {
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // If the origin has been set and we have GPS, then return the GPS position relative to the origin
                const struct Location &gpsloc = _ahrs->get_gps().location();
                Vector2f tempPosNE = location_diff(EKF_origin, gpsloc);
                pos.x = tempPosNE.x;
                pos.y = tempPosNE.y;
                return false;
            } else {
                // If no GPS fix is available, all we can do is provide the last known position
                pos.x = state.position.x + lastKnownPositionNE.x;
                pos.y = state.position.y + lastKnownPositionNE.y;
                return false;
            }
        } else {
            // If the origin has not been set, then we have no means of providing a relative position
            pos.x = 0.0f;
            pos.y = 0.0f;
            return false;
        }
    }
    return false;
}

// return body axis gyro bias estimates in rad/sec
void NavEKF_core::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMUavg < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = state.gyro_bias / dtIMUavg;
}

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
void NavEKF_core::resetGyroBias(void)
{
    state.gyro_bias.zero();
    zeroRows(P,10,12);
    zeroCols(P,10,12);
    P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];

}

// Reset the baro so that it reads zero at the current height
// Reset the EKF height to zero
// Adjust the EKf origin height so that the EKF height + origin height is the same as before
// Return true if the height datum reset has been performed
// If using a range finder for height do not reset and return false
bool NavEKF_core::resetHeightDatum(void)
{
    // if we are using a range finder for height, return false
    if (frontend._altSource == 1) {
        return false;
    }
    // record the old height estimate
    float oldHgt = -state.position.z;
    // reset the barometer so that it reads zero at the current height
    _baro.update_calibration();
    // reset the height state
    state.position.z = 0.0f;
    // reset the stored height states from previous time steps
    for (uint8_t i=0; i<=49; i++){
        storedStates[i].position.z = state.position.z;
    }
    // adjust the height of the EKF origin so that the origin plus baro height before and afer the reset is the same
    if (validOrigin) {
        EKF_origin.alt += oldHgt*100;
    }
    return true;
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming
// This command is forgotten by the EKF each time the vehicle disarms
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF_core::setInhibitGPS(void)
{
    if(!vehicleArmed) {
        return 0;
    }
    if (optFlowDataPresent()) {
        frontend._fusionModeGPS = 3;
        return 2;
    } else {
        return 1;
    }
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF_core::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (PV_AidingMode == AID_RELATIVE) {
        // allow 1.0 rad/sec margin for angular motion
        ekfGndSpdLimit = MAX((frontend._maxFlowRate - 1.0f), 0.0f) * MAX((terrainState - state.position[2]), rngOnGnd);
        // use standard gains up to 5.0 metres height and reduce above that
        ekfNavVelGainScaler = 4.0f / MAX((terrainState - state.position[2]),4.0f);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// return weighting of first IMU in blending function
void NavEKF_core::getIMU1Weighting(float &ret) const
{
    ret = IMU1_weighting;
}

// return the individual Z-accel bias estimates in m/s^2
void NavEKF_core::getAccelZBias(float &zbias1, float &zbias2) const {
    if (dtIMUavg > 0) {
        zbias1 = state.accel_zbias1 / dtIMUavg;
        zbias2 = state.accel_zbias2 / dtIMUavg;
    } else {
        zbias1 = 0;
        zbias2 = 0;
    }
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF_core::getWind(Vector3f &wind) const
{
    wind.x = state.wind_vel.x;
    wind.y = state.wind_vel.y;
    wind.z = 0.0f; // currently don't estimate this
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF_core::getMagNED(Vector3f &magNED) const
{
    magNED = state.earth_magfield * 1000.0f;
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF_core::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ = state.body_magfield*1000.0f;
}

// return magnetometer offsets
// return true if offsets are valid
bool NavEKF_core::getMagOffsets(Vector3f &magOffsets) const
{
    // compass offsets are valid if we have finalised magnetic field initialisation and magnetic field learning is not prohibited and primary compass is valid
    if (secondMagYawInit && (frontend._magCal != 2) && _ahrs->get_compass()->healthy()) {
        magOffsets = _ahrs->get_compass()->get_offsets() - state.body_magfield*1000.0f;
        return true;
    } else {
        magOffsets = _ahrs->get_compass()->get_offsets();
        return false;
    }
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF_core::getLLH(struct Location &loc) const
{
    if(validOrigin) {
        // Altitude returned is an absolute altitude relative to the WGS-84 spherioid
        loc.alt = EKF_origin.alt - state.position.z*100;
        loc.flags.relative_alt = 0;
        loc.flags.terrain_alt = 0;

        // there are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no aiding)
        nav_filter_status status;
        getFilterStatus(status);
        if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
            loc.lat = EKF_origin.lat;
            loc.lng = EKF_origin.lng;
            location_offset(loc, state.position.x, state.position.y);
            return true;
        } else {
            // we could be in constant position mode  becasue the vehicle has taken off without GPS, or has lost GPS
            // in this mode we cannot use the EKF states to estimate position so will return the best available data
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // we have a GPS position fix to return
                const struct Location &gpsloc = _ahrs->get_gps().location();
                loc.lat = gpsloc.lat;
                loc.lng = gpsloc.lng;
                return true;
            } else {
                // if no GPS fix, provide last known position before entering the mode
                location_offset(loc, lastKnownPositionNE.x, lastKnownPositionNE.y);
                return false;
            }
        }
    } else {
        // If no origin has been defined for the EKF, then we cannot use its position states so return a raw
        // GPS reading if available and return false
        if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D)) {
            const struct Location &gpsloc = _ahrs->get_gps().location();
            loc = gpsloc;
            loc.flags.relative_alt = 0;
            loc.flags.terrain_alt = 0;
        }
        return false;
    }
}

// return the estimated height above ground level
bool NavEKF_core::getHAGL(float &HAGL) const
{
    HAGL = terrainState - state.position.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && healthy();
}

// return data for debugging optical flow fusion
void NavEKF_core::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    varFlow = MAX(flowTestRatio[0],flowTestRatio[1]);
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
void NavEKF_core::SetFlightAndFusionModes()
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
        float gndSpdSq = sq(velNED[0]) + sq(velNED[1]);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        if (_ahrs->airspeed_sensor_enabled()) {
            const AP_Airspeed *airspeed = _ahrs->get_airspeed();
            if (airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(hgtMea) > 10.0f) {
            largeHgtChange = true;
        }

        // to go to in-air mode we also need enough GPS velocity to be able to calculate a reliable ground track heading and either a lerge height or airspeed change
        if (onGround && highGndSpd && (highAirSpd || largeHgtChange)) {
            onGround = false;
        }
        // if is possible we are in flight, set the time this condition was last detected
        if (highGndSpd || highAirSpd || largeHgtChange) {
            airborneDetectTime_ms = imuSampleTime_ms;
        }
        // after 5 seconds of not detecting a possible flight condition, we transition to on-ground mode
        if(!onGround && ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
        }
        // perform a yaw alignment check against GPS if exiting on-ground mode, bu tonly if we have enough ground speed
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
    bool magCalRequested = ((frontend._magCal == 0) && !onGround) || ((frontend._magCal == 1) && manoeuvring)  || (frontend._magCal == 3);
    // deny mag calibration request if we aren't using the compass, are in the pre-arm constant position mode or it has been inhibited by the user
    bool magCalDenied = !use_compass() || constPosMode || (frontend._magCal == 2);
    // inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || magCalDenied);
}

// initialise the covariance matrix
void NavEKF_core::CovarianceInit()
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
    P[1][1]   = 0.25f*sq(radians(10.0f));
    P[2][2]   = 0.25f*sq(radians(10.0f));
    P[3][3]   = 0.25f*sq(radians(10.0f));
    // velocities
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7f);
    // positions
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(frontend._baroAltNoise);
    // delta angle biases
    P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // Z delta velocity bias
    P[13][13] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtIMUavg);
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
void NavEKF_core::ForceSymmetry()
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
void NavEKF_core::CopyAndFixCovariances()
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
void NavEKF_core::ConstrainVariances()
{
    for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // quaternions
    for (uint8_t i=4; i<=6; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=7; i<=9; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f); // positions
    for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtIMUavg)); // delta angle biases
    P[13][13] = constrain_float(P[13][13],0.0f,sq(10.0f * dtIMUavg)); // delta velocity bias
    for (uint8_t i=14; i<=15; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // earth magnetic field
    for (uint8_t i=16; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // body magnetic field
}

// constrain states to prevent ill-conditioning
void NavEKF_core::ConstrainStates()
{
    // quaternions are limited between +-1
    for (uint8_t i=0; i<=3; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=4; i<=6; i++) states[i] = constrain_float(states[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i=7; i<=8; i++) states[i] = constrain_float(states[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    state.position.z = constrain_float(state.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers specs)
    for (uint8_t i=10; i<=12; i++) states[i] = constrain_float(states[i],-0.1f*dtIMUavg,0.1f*dtIMUavg);
    // when the vehicle arms we adjust the limits so that in flight the bias can change by the same amount in either direction
    float delAngBiasLim = MAX_GYRO_BIAS*dtIMUavg;
    state.gyro_bias.x = constrain_float(state.gyro_bias.x, (delAngBiasAtArming.x - delAngBiasLim), (delAngBiasAtArming.x + delAngBiasLim));
    state.gyro_bias.y = constrain_float(state.gyro_bias.y, (delAngBiasAtArming.y - delAngBiasLim), (delAngBiasAtArming.y + delAngBiasLim));
    state.gyro_bias.z = constrain_float(state.gyro_bias.z, (delAngBiasAtArming.z - delAngBiasLim), (delAngBiasAtArming.z + delAngBiasLim));
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    states[13] = constrain_float(states[13],-1.0f*dtIMUavg,1.0f*dtIMUavg);
    states[22] = constrain_float(states[22],-1.0f*dtIMUavg,1.0f*dtIMUavg);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=14; i<=15; i++) states[i] = constrain_float(states[i],-100.0f,100.0f);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) states[i] = constrain_float(states[i],-0.5f,0.5f);
    // constrain the terrain offset state
    terrainState = MAX(terrainState, state.position.z + rngOnGnd);
}

bool NavEKF_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);
        dVel_dt = ins.get_delta_velocity_dt(ins_index);
        // catch invalid delta time
        if (dVel_dt <= 0.0f) {
            dVel_dt = dtIMUavg;
        }
        return true;
    }
    return false;
}

bool NavEKF_core::readDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);
        return true;
    }
    return false;
}

// update IMU delta angle and delta velocity measurements
void NavEKF_core::readIMUData()
{
    const AP_InertialSensor &ins = _ahrs->get_ins();

    // calculate the average time between IMU updates
    dtIMUavg = ins.get_loop_delta_t();

    // calculate the most recent time between gyro delta angle updates
    if (ins.get_delta_time() > 0.0f) {
        dtDelAng = ins.get_delta_time();
    } else {
        dtDelAng = dtIMUavg;
    }

    // the imu sample time is used as a common time reference throughout the filter
    imuSampleTime_ms = AP_HAL::millis();

    // dual accel mode - require both IMU's to be able to provide delta velocity outputs
    if (ins.use_accel(0) && ins.use_accel(1) && readDeltaVelocity(0, dVelIMU1, dtDelVel1) && readDeltaVelocity(1, dVelIMU2, dtDelVel2)) {

        // apply a peak hold 0.2 second time constant decaying envelope filter to the noise length on IMU1
        float alpha = 1.0f - 5.0f*dtDelVel1;
        imuNoiseFiltState1 = MAX(ins.get_vibration_levels(0).length(), alpha*imuNoiseFiltState1);

        // apply a peak hold 0.2 second time constant decaying envelope filter to the noise length on IMU2
        alpha = 1.0f - 5.0f*dtDelVel2;
        imuNoiseFiltState2 = MAX(ins.get_vibration_levels(1).length(), alpha*imuNoiseFiltState2);

        // calculate the filtered difference between acceleration vectors from IMU1 and 2
        // apply a LPF filter with a 1.0 second time constant
        alpha = constrain_float(0.5f*(dtDelVel1 + dtDelVel2),0.0f,1.0f);
        accelDiffFilt = (ins.get_accel(0) - ins.get_accel(1)) * alpha + accelDiffFilt * (1.0f - alpha);
        float accelDiffLength = accelDiffFilt.length();

        // Check the difference for excessive error and use the IMU with less noise
        // Apply hysteresis to prevent rapid switching
        if (accelDiffLength > 1.8f || (accelDiffLength > 1.2f && lastImuSwitchState != IMUSWITCH_MIXED)) {
            if (lastImuSwitchState == IMUSWITCH_MIXED) {
                // no previous fail so switch to the IMU with least noise
                if (imuNoiseFiltState1 < imuNoiseFiltState2) {
                    lastImuSwitchState = IMUSWITCH_IMU0;
                } else {
                    lastImuSwitchState = IMUSWITCH_IMU1;
                }
            } else if (lastImuSwitchState == IMUSWITCH_IMU0) {
                // IMU1 previously failed so require 5 m/s/s less noise on IMU2 to switch across
                if (imuNoiseFiltState1 - imuNoiseFiltState2 > 5.0f) {
                    // IMU2 is significantly less noisy, so switch
                    lastImuSwitchState = IMUSWITCH_IMU1;
                }
            } else {
                // IMU2 previously failed so require 5 m/s/s less noise on IMU1 to switch across
                if (imuNoiseFiltState2 - imuNoiseFiltState1 > 5.0f) {
                    // IMU1 is significantly less noisy, so switch
                    lastImuSwitchState = IMUSWITCH_IMU0;
                }
            }
        } else {
            lastImuSwitchState = IMUSWITCH_MIXED;
        }

    } else {
        // single accel mode - one of the first two accelerometers are unhealthy, not available or de-selected by the user
        // read good accelerometer into dVelIMU1 and copy to dVelIMU2
        // set the switch state based on the IMU we are using to make the data source selection visible
        if (ins.use_accel(0) && readDeltaVelocity(0, dVelIMU1, dtDelVel1)) {
            lastImuSwitchState = IMUSWITCH_IMU0;
        } else if (ins.use_accel(1) && readDeltaVelocity(1, dVelIMU1, dtDelVel1)) {
            lastImuSwitchState = IMUSWITCH_IMU1;
        } else {
            readDeltaVelocity(ins.get_primary_accel(), dVelIMU1, dtDelVel1);
            switch (ins.get_primary_accel()) {
                case 0:
                    lastImuSwitchState = IMUSWITCH_IMU0;
                    break;
                case 1:
                    lastImuSwitchState = IMUSWITCH_IMU1;
                    break;
                default:
                    // we must be using IMU2 which can't be properly represented so we set to "mixed"
                    lastImuSwitchState = IMUSWITCH_MIXED;
                    break;
            }
        }
        dtDelVel2 = dtDelVel1;
        dVelIMU2 = dVelIMU1;
    }

    // Default is to use the average of two gyros if available
    // This reduces rate offset due to temperature variation
    Vector3f dAng0,dAng1;
    if (ins.use_gyro(0) && ins.use_gyro(1) && readDeltaAngle(0, dAng0) && readDeltaAngle(1, dAng1)) {
        dAngIMU = (dAng0+dAng1);
        dAngIMU *= 0.5f;
    } else {
        // single gyro mode - one of the first two gyros are unhealthy or don't exist
        // just read primary gyro
        readDeltaAngle(ins.get_primary_gyro(), dAngIMU);
    }
}

// check for new valid GPS data and update stored measurement if available
void NavEKF_core::readGpsData()
{
    // check for new GPS data
    if (_ahrs->get_gps().last_message_time_ms() != lastFixTime_ms) {
         if (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
             // report GPS fix status
             gpsCheckStatus.bad_fix = false;

             // store fix time from previous read
            secondLastFixTime_ms = lastFixTime_ms;

            // get current fix time
            lastFixTime_ms = _ahrs->get_gps().last_message_time_ms();

            // set flag that lets other functions know that new GPS data has arrived
            newDataGps = true;

            // get state vectors that were stored at the time that is closest to when the the GPS measurement
            // time after accounting for measurement delays
            RecallStates(statesAtVelTime, (imuSampleTime_ms - constrain_int16(frontend._msecVelDelay, 0, 500)));
            RecallStates(statesAtPosTime, (imuSampleTime_ms - constrain_int16(frontend._msecPosDelay, 0, 500)));

            // read the NED velocity from the GPS
            velNED = _ahrs->get_gps().velocity();

            // Use the speed accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw speed accuracy data
            float alpha = constrain_float(0.0002f * (lastFixTime_ms - secondLastFixTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!_ahrs->get_gps().speed_accuracy(gpsSpdAccRaw)) {
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            if (_ahrs->get_gps().num_sats() >= 6 && !constPosMode) {
                gpsNoiseScaler = 1.0f;
            } else if (_ahrs->get_gps().num_sats() == 5 && !constPosMode) {
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity and set GPS fusion mode accordingly
            if (_ahrs->get_gps().have_vertical_velocity() && frontend._fusionModeGPS == 0) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data for alignment
            gpsGoodToAlign = calcGpsGoodToAlign();

            // Monitor qulaity of GPS data inflight
            calcGpsGoodForFlight();

            // Read the GPS locaton in WGS-84 lat,long,height coordinates
            const struct Location &gpsloc = _ahrs->get_gps().location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            if (!validOrigin && gpsGoodToAlign) {
                setOrigin();
                // Now we know the location we have an estimate for the magnetic field declination and adjust the earth field accordingly
                alignMagStateDeclination();
                // Set the height of the NED origin to ‘height of baro height datum relative to GPS height datum'
                EKF_origin.alt = gpsloc.alt - hgtMea;
            }

            // Commence GPS aiding when able to
            if ((frontend._fusionModeGPS != 3) && (PV_AidingMode != AID_ABSOLUTE) && vehicleArmed && gpsGoodToAlign) {
                PV_AidingMode = AID_ABSOLUTE;
                constPosMode = false;
                // Initialise EKF position and velocity states to last GPS measurement
                ResetPosition();
                ResetVelocity();
            }

            // Convert to local coordinates if we have an origin.
            if (validOrigin) {
                gpsPosNE = location_diff(EKF_origin, gpsloc);
            }
        } else {
             // report GPS fix status
             gpsCheckStatus.bad_fix = true;
        }
    }


    // If no previous GPS lock or told not to use it, or EKF origin not set, we declare the  GPS unavailable for use
    if ((_ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) || frontend._fusionModeGPS == 3 || !validOrigin) {
        gpsNotAvailable = true;
    } else {
        gpsNotAvailable = false;
    }
}

// check for new altitude measurement data and update stored measurement if available
void NavEKF_core::readHgtData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    if (_baro.healthy() && _baro.get_last_update() != lastHgtMeasTime) {
        // Don't use Baro height if operating in optical flow mode as we use range finder instead
        if (frontend._fusionModeGPS == 3 && frontend._altSource == 1) {
            if ((imuSampleTime_ms - rngValidMeaTime_ms) < 2000) {
                // adjust range finder measurement to allow for effect of vehicle tilt and height of sensor
                hgtMea = MAX(rngMea * Tnb_flow.c.z, rngOnGnd);
                // get states that were stored at the time closest to the measurement time, taking measurement delay into account
                statesAtHgtTime = statesAtFlowTime;
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + state.position.z) + 0.9f * baroHgtOffset;
            } else if (vehicleArmed && takeOffDetected) {
                // use baro measurement and correct for baro offset - failsafe use only as baro will drift
                hgtMea = MAX(_baro.get_altitude() - baroHgtOffset, rngOnGnd);
                // get states that were stored at the time closest to the measurement time, taking measurement delay into account
                RecallStates(statesAtHgtTime, (imuSampleTime_ms - msecHgtDelay));
            } else {
                // If we are on ground and have no range finder reading, assume the nominal on-ground height
                hgtMea = rngOnGnd;
                // get states that were stored at the time closest to the measurement time, taking measurement delay into account
                statesAtHgtTime = state;
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + state.position.z) + 0.9f * baroHgtOffset;
            }
        } else {
            // use baro measurement and correct for baro offset
            hgtMea = _baro.get_altitude();
            // get states that were stored at the time closest to the measurement time, taking measurement delay into account
            RecallStates(statesAtHgtTime, (imuSampleTime_ms - msecHgtDelay));
        }

        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = msecHgtAvg*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (hgtMea-meaHgtAtTakeOff)*alpha;
        } else if (vehicleArmed && getTakeoffExpected()) {
            // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
            // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
            hgtMea = MAX(hgtMea, meaHgtAtTakeOff);
        }

        // set flag to let other functions know new data has arrived
        newDataHgt = true;
        // time stamp used to check for new measurement
        lastHgtMeasTime = _baro.get_last_update();
    } else {
        newDataHgt = false;
    }
}

// check for new magnetometer data and update store measurements if available
void NavEKF_core::readMagData()
{
    if (use_compass() && _ahrs->get_compass()->last_update_usec() != lastMagUpdate) {
        // store time of last measurement update
        lastMagUpdate = _ahrs->get_compass()->last_update_usec();

        // read compass data and scale to improve numerical conditioning
        magData = _ahrs->get_compass()->get_field() * 0.001f;

        // check for consistent data between magnetometers
        consistentMagData = _ahrs->get_compass()->consistent();

        // get states stored at time closest to measurement time after allowance for measurement delay
        RecallStates(statesAtMagMeasTime, (imuSampleTime_ms - msecMagDelay));

        // let other processes know that new compass data has arrived
        newDataMag = true;

        // check if compass offsets have ben changed and adjust EKF bias states to maintain consistent innovations
        if (_ahrs->get_compass()->healthy()) {
            Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets();
            bool changeDetected = (!is_equal(nowMagOffsets.x,lastMagOffsets.x) || !is_equal(nowMagOffsets.y,lastMagOffsets.y) || !is_equal(nowMagOffsets.z,lastMagOffsets.z));
            // Ignore bias changes before final mag field and yaw initialisation, as there may have been a compass calibration
            if (changeDetected && secondMagYawInit) {
                state.body_magfield.x += (nowMagOffsets.x - lastMagOffsets.x) * 0.001f;
                state.body_magfield.y += (nowMagOffsets.y - lastMagOffsets.y) * 0.001f;
                state.body_magfield.z += (nowMagOffsets.z - lastMagOffsets.z) * 0.001f;
            }
        lastMagOffsets = nowMagOffsets;
        }
    } else {
        newDataMag = false;
    }
}

// check for new airspeed data and update stored measurements if available
void NavEKF_core::readAirSpdData()
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
void NavEKF_core::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
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
    RecallOmega(omegaAcrossFlowTime, imuSampleTime_ms - flowTimeDeltaAvg_ms - frontend._msecFLowDelay, imuSampleTime_ms - frontend._msecFLowDelay);
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - omegaAcrossFlowTime.x),-0.1f,0.1f);
    flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - omegaAcrossFlowTime.y),-0.1f,0.1f);
    // check for takeoff if relying on optical flow and zero measurements until takeoff detected
    // if we haven't taken off - constrain position and velocity states
    if (frontend._fusionModeGPS == 3) {
        detectOptFlowTakeoff();
    }
    // recall vehicle states at mid sample time for flow observations allowing for delays
    RecallStates(statesAtFlowTime, imuSampleTime_ms - frontend._msecFLowDelay - flowTimeDeltaAvg_ms/2);
    // calculate rotation matrices at mid sample time for flow observations
    statesAtFlowTime.quat.rotation_matrix(Tbn_flow);
    Tnb_flow = Tbn_flow.transposed();
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
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
}

// calculate the NED earth spin vector in rad/sec
void NavEKF_core::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
// if no magnetometer data, do not update magnetic field states and assume zero yaw angle
Quaternion NavEKF_core::calcQuatAndFieldStates(float roll, float pitch)
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
            // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
            Vector3f tempEuler;
            state.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
            // this check ensures we accumulate the resets that occur within a single iteration of the EKF
            if (imuSampleTime_ms != lastYawReset_ms) {
                yawResetAngle = 0.0f;
            }
            yawResetAngle += wrap_PI(yaw - tempEuler.z);
            lastYawReset_ms = imuSampleTime_ms;
            // calculate an initial quaternion using the new yaw value
            initQuat.from_euler(roll, pitch, yaw);
        } else {
            initQuat = state.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        state.earth_magfield = Tbn * magData;

        // align the NE earth magnetic field states with the published declination
        alignMagStateDeclination();

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
void NavEKF_core::alignYawGPS()
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
            Vector3f eulerAngles;
            getEulerAngles(eulerAngles);
            calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        }
    }
}

// This function is used to do a forced alignment of the wind velocity
// states so that they are set to the reciprocal of the ground speed
// and scaled to STARTUP_WIND_SPEED m/s. This is used when launching a
// fly-forward vehicle without an airspeed sensor on the assumption
// that launch will be into wind and STARTUP_WIND_SPEED is
// representative of typical launch wind
void NavEKF_core::setWindVelStates()
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
void NavEKF_core::getRotationBodyToNED(Matrix3f &mat) const
{
    Vector3f trim = _ahrs->get_trim();
    state.quat.rotation_matrix(mat);
    mat.rotateXYinv(trim);
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void  NavEKF_core::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const
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
// also return the amount of NE position shift due to the last position reset
void  NavEKF_core::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    velVar   = sqrtf(velTestRatio);
    posVar   = sqrtf(posTestRatio);
    hgtVar   = sqrtf(hgtTestRatio);
    magVar.x = sqrtf(magTestRatio.x);
    magVar.y = sqrtf(magTestRatio.y);
    magVar.z = sqrtf(magTestRatio.z);
    tasVar   = sqrtf(tasTestRatio);
    offset   = posResetNE;
}

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF_core::InitialiseVariables()
{
    if (_perf_UpdateFilter == nullptr) {
        // only allocate perf variables once
        _perf_UpdateFilter = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_UpdateFilter");
        _perf_CovariancePrediction = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_CovariancePrediction");
        _perf_FuseVelPosNED = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_FuseVelPosNED");
        _perf_FuseMagnetometer = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_FuseMagnetometer");
        _perf_FuseAirspeed = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_FuseAirspeed");
        _perf_FuseSideslip = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_FuseSideslip");
        _perf_OpticalFlowEKF = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_OpticalFlowEKF");
        _perf_FuseOptFlow = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EKF_FuseOptFlow");
    }

    // initialise time stamps
    imuSampleTime_ms = AP_HAL::millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    TASmsecPrev = imuSampleTime_ms;
    BETAmsecPrev = imuSampleTime_ms;
    lastMagUpdate = 0;
    lastHgtMeasTime = imuSampleTime_ms;
    lastAirspeedUpdate = 0;
    lastVelPassTime = imuSampleTime_ms;
    lastPosPassTime = imuSampleTime_ms;
    lastPosFailTime = 0;
    lastHgtPassTime_ms = imuSampleTime_ms;
    lastTasPassTime = imuSampleTime_ms;
    lastStateStoreTime_ms = imuSampleTime_ms;
    lastFixTime_ms = 0;
    secondLastFixTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = imuSampleTime_ms;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    timeAtDisarm_ms = 0;
    lastConstPosFuseTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = true;
    tasTimeout = true;
    badMag = false;
    badIMUdata = false;
    firstArmComplete = false;
    firstMagYawInit = false;
    secondMagYawInit = false;
    storeIndex = 0;
    dtIMUavg = 0.0025f;
    dtDelAng = 0.0025f;
    dt = 0;
    hgtMea = 0;
    storeIndex = 0;
    lastGyroBias.zero();
    lastAngRate.zero();
    lastAccel1.zero();
    lastAccel2.zero();
    velDotNEDfilt.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    velNED.zero();
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
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    posTimeout = true;
    velTimeout = true;
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
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    gpsAidingBad = false;
    highYawRate = false;
    yawRateFilt = 0.0f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    imuNoiseFiltState1 = 0.0f;
    imuNoiseFiltState2 = 0.0f;
    lastImuSwitchState = IMUSWITCH_MIXED;
    gpsAccuracyGood = false;
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    posDownDerivative = 0.0f;
    posDown = 0.0f;
    delAngBiasAtArming.zero();
    posResetNE.zero();
    velResetNE.zero();
    hgtInnovFiltState = 0.0f;
}

// return true if we should use the airspeed sensor
bool NavEKF_core::useAirspeed(void) const
{
    return _ahrs->airspeed_sensor_enabled();
}

// return true if we should use the range finder sensor
bool NavEKF_core::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

// return true if optical flow data is available
bool NavEKF_core::optFlowDataPresent(void) const
{
    if (imuSampleTime_ms - flowMeaTime_ms < 5000) {
        return true;
    } else {
        return false;
    }
}

// return true if the vehicle is requesting the filter to be ready for flight
bool NavEKF_core::getVehicleArmStatus(void) const
{
    return hal.util->get_soft_armed() || _ahrs->get_correct_centrifugal();
}

// return true if we should use the compass
bool NavEKF_core::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

/*
  should we assume zero sideslip?
 */
bool NavEKF_core::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return _ahrs->get_fly_forward() && _ahrs->get_vehicle_class() != AHRS_VEHICLE_GROUND;
}


/* 
   vehicle specific initial gyro bias uncertainty
 */
float NavEKF_core::InitialGyroBiasUncertainty(void) const
{
    // this is the assumed uncertainty in gyro bias in rad/sec used to initialise the covariance matrix.
    return 0.035f;
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
void  NavEKF_core::getFilterFaults(uint8_t &faults) const
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
void  NavEKF_core::getFilterTimeouts(uint8_t &timeouts) const
{
    timeouts = (posTimeout<<0 |
                velTimeout<<1 |
                hgtTimeout<<2 |
                magTimeout<<3 |
                tasTimeout<<4);
}

/*
return filter gps quality check status
*/
void  NavEKF_core::getFilterGpsStatus(nav_gps_status &faults) const
{
    // init return value
    faults.value = 0;

    // set individual flags
    faults.flags.bad_sAcc           = gpsCheckStatus.bad_sAcc; // reported speed accuracy is insufficient
    faults.flags.bad_hAcc           = gpsCheckStatus.bad_hAcc; // reported horizontal position accuracy is insufficient
    faults.flags.bad_yaw            = gpsCheckStatus.bad_yaw; // EKF heading accuracy is too large for GPS use
    faults.flags.bad_sats           = gpsCheckStatus.bad_sats; // reported number of satellites is insufficient
    faults.flags.bad_VZ             = gpsCheckStatus.bad_VZ; // GPS vertical velocity is inconsistent with the IMU and Baro measurements
    faults.flags.bad_horiz_drift    = gpsCheckStatus.bad_horiz_drift; // GPS horizontal drift is too large to start using GPS (check assumes vehicle is static)
    faults.flags.bad_hdop           = gpsCheckStatus.bad_hdop; // reported HDoP is too large to start using GPS
    faults.flags.bad_vert_vel       = gpsCheckStatus.bad_vert_vel; // GPS vertical speed is too large to start using GPS (check assumes vehicle is static)
    faults.flags.bad_fix            = gpsCheckStatus.bad_fix; // The GPS cannot provide the 3D fix required
    faults.flags.bad_horiz_vel      = gpsCheckStatus.bad_horiz_vel; // The GPS horizontal speed is excessive (check assumes the vehicle is static)
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
void  NavEKF_core::getFilterStatus(nav_filter_status &status) const
{
    // init return value
    status.value = 0;

    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool notDeadReckoning = !constPosMode;
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && (frontend._fusionModeGPS == 3);
    bool gpsNavPossible = !gpsNotAvailable && (frontend._fusionModeGPS <= 2) && gpsGoodToAlign;
    bool filterHealthy = healthy();
    bool gyroHealthy = checkGyroHealthPreFlight();

    // set individual flags
    status.flags.attitude = !state.quat.is_nan() && filterHealthy && gyroHealthy;   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && notDeadReckoning && filterHealthy;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    status.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && notDeadReckoning && filterHealthy;   // relative horizontal position estimate valid
    status.flags.horiz_pos_abs = !gpsAidingBad && doingNormalGpsNav && notDeadReckoning && filterHealthy; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout && filterHealthy;            // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    status.flags.const_pos_mode = constPosMode && filterHealthy;     // constant position mode
    status.flags.pred_horiz_pos_rel = (optFlowNavPossible || gpsNavPossible) && filterHealthy && gyroHealthy; // we should be able to estimate a relative position when we enter flight mode
    status.flags.pred_horiz_pos_abs = gpsNavPossible && filterHealthy && gyroHealthy; // we should be able to estimate an absolute position when we enter flight mode
    status.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    status.flags.takeoff = expectGndEffectTakeoff; // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    status.flags.touchdown = expectGndEffectTouchdown; // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    status.flags.using_gps = (imuSampleTime_ms - lastPosPassTime) < 4000;
    status.flags.gps_glitching = !gpsAccuracyGood; // The GPS is glitching
}

// send an EKF_STATUS message to GCS
void NavEKF_core::send_status_report(mavlink_channel_t chan)
{
    // get filter status
    nav_filter_status filt_state;
    getFilterStatus(filt_state);

    // prepare flags
    uint16_t flags = 0;
    if (filt_state.flags.attitude) { flags |= EKF_ATTITUDE; }
    if (filt_state.flags.horiz_vel) { flags |= EKF_VELOCITY_HORIZ; }
    if (filt_state.flags.vert_vel) { flags |= EKF_VELOCITY_VERT; }
    if (filt_state.flags.horiz_pos_rel) { flags |= EKF_POS_HORIZ_REL; }
    if (filt_state.flags.horiz_pos_abs) { flags |= EKF_POS_HORIZ_ABS; }
    if (filt_state.flags.vert_pos) { flags |= EKF_POS_VERT_ABS; }
    if (filt_state.flags.terrain_alt) { flags |= EKF_POS_VERT_AGL; }
    if (filt_state.flags.const_pos_mode) { flags |= EKF_CONST_POS_MODE; }
    if (filt_state.flags.pred_horiz_pos_rel) { flags |= EKF_PRED_POS_HORIZ_REL; }
    if (filt_state.flags.pred_horiz_pos_abs) { flags |= EKF_PRED_POS_HORIZ_ABS; }

    // get variances
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);

    // send message
    mavlink_msg_ekf_status_report_send(chan, flags, velVar, posVar, hgtVar, magVar.length(), sqrtf(auxRngTestRatio));

}

// Check arm status and perform required checks and mode changes
void NavEKF_core::performArmingChecks()
{
    // determine vehicle arm status and don't allow filter to arm until it has been running for long enough to stabilise
    prevVehicleArmed = vehicleArmed;
    vehicleArmed = (getVehicleArmStatus() && (imuSampleTime_ms - ekfStartTime_ms) > 1000);

    // check to see if arm status has changed and reset states if it has
    if (vehicleArmed != prevVehicleArmed) {
        // only reset the magnetic field and heading on the first arm. This prevents in-flight learning being forgotten for vehicles that do multiple short flights and disarm in-between.
        if (vehicleArmed && !firstArmComplete) {
            firstArmComplete = true;
            Vector3f eulerAngles;
            getEulerAngles(eulerAngles);
            state.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        }
        // store vertical position at arming to use as a reference for ground relative cehcks
        if (vehicleArmed) {
            posDownAtArming = state.position.z;
            // save the gyro bias so that the in-flight gyro bias state limits can be adjusted to provide the same amount of offset change in either direction
            delAngBiasAtArming = state.gyro_bias;
        }
        // zero stored velocities used to do dead-reckoning
        heldVelNE.zero();
        // reset the flag that indicates takeoff for use by optical flow navigation
        takeOffDetected = false;
        // set various  useage modes based on the condition at arming. These are then held until the vehicle is disarmed.
        if (!vehicleArmed) {
            PV_AidingMode = AID_NONE; // When dis-armed, we only estimate orientation & height using the constant position mode
            posTimeout = true;
            velTimeout = true;
            constPosMode = true;
            // store the current position to be used to keep reporting the last known position when disarmed
            lastKnownPositionNE.x = state.position.x;
            lastKnownPositionNE.y = state.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the filter to settle before the estimate can be used
            meaHgtAtTakeOff = hgtMea;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            state.position.z = -hgtMea;
            // record the time we disarmed
            timeAtDisarm_ms = imuSampleTime_ms;
            // if the GPS is not glitching when we land, we reset the timer used to check GPS quality
            // timer is not set to zero to avoid triggering an automatic fail
            if (gpsAccuracyGood) {
                lastGpsVelFail_ms = 1;
                gpsGoodToAlign = true;
            }
            // we reset the GPS drift checks when disarming as the vehicle has been moving during flight
            gpsDriftNE = 0.0f;
            gpsVertVelFilt = 0.0f;
            gpsHorizVelFilt = 0.0f;
        } else if (frontend._fusionModeGPS == 3) { // arming when GPS useage has been prohibited
            if (optFlowDataPresent()) {
                PV_AidingMode = AID_RELATIVE; // we have optical flow data and can estimate all vehicle states
                posTimeout = true;
                velTimeout = true;
                constPosMode = false;
            } else {
                PV_AidingMode = AID_NONE; // we don't have optical flow data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
                constPosMode = true;
            }
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
            // this avoids issues casued by the time delay associated with arming that can trigger short timeouts
            rngValidMeaTime_ms = imuSampleTime_ms;
            // store the range finder measurement which will be used as a reference to detect when we have taken off
            rangeAtArming = rngMea;
            // set the time at which we arm to assist with takeoff detection
            timeAtArming_ms =  imuSampleTime_ms;
        } else { // arming when GPS useage is allowed
            if (gpsNotAvailable) {
                PV_AidingMode = AID_NONE; // we don't have have GPS data and will only be able to estimate orientation and height
                posTimeout = true;
                velTimeout = true;
                constPosMode = true;
            } else {
                PV_AidingMode = AID_ABSOLUTE; // we have GPS data and can estimate all vehicle states
                posTimeout = false;
                velTimeout = false;
                constPosMode = false;
                // we need to reset the GPS timers to prevent GPS timeout logic being invoked on entry into GPS aiding
                // this is becasue the EKF can be interrupted for an arbitrary amount of time during vehicle arming checks
                lastFixTime_ms = imuSampleTime_ms;
                secondLastFixTime_ms = imuSampleTime_ms;
                // reset the last valid position fix time to prevent unwanted activation of GPS glitch logic
                lastPosPassTime = imuSampleTime_ms;
                // reset the fail time to prevent premature reporting of loss of position accruacy
                lastPosFailTime = 0;
            }
        }
        if (vehicleArmed) {
            // Reset filter position to GPS when transitioning into flight mode
            // We need to do this becasue the vehicle may have moved since the EKF origin was set
            ResetPosition();
            StoreStatesReset();
        } else {
            // Reset all position and velocity states when transitioning out of flight mode
            // We need to do this becasue we are going into a mode that assumes zero position and velocity
            ResetVelocity();
            ResetPosition();
            StoreStatesReset();
        }

    }

    // Always turn aiding off when the vehicle is disarmed
    if (!vehicleArmed) {
        PV_AidingMode = AID_NONE;
        posTimeout = true;
        velTimeout = true;
        // set constant position mode if aiding is inhibited
        constPosMode = true;
    }

}

// Set the NED origin to be used until the next filter reset
void NavEKF_core::setOrigin()
{
    EKF_origin = _ahrs->get_gps().location();
    validOrigin = true;
}

// return the LLH location of the filters NED origin
bool NavEKF_core::getOriginLLH(struct Location &loc) const
{
    if (validOrigin) {
        loc = EKF_origin;
    }
    return validOrigin;
}

// set the LLH location of the filters NED origin
bool NavEKF_core::setOriginLLH(struct Location &loc)
{
    if (vehicleArmed) {
        return false;
    }
    EKF_origin = loc;
    validOrigin = true;
    return true;
}

// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF_core::getTakeoffExpected()
{
    if (expectGndEffectTakeoff && imuSampleTime_ms - takeoffExpectedSet_ms > gndEffectTimeout_ms) {
        expectGndEffectTakeoff = false;
    }

    return expectGndEffectTakeoff;
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF_core::setTakeoffExpected(bool val)
{
    takeoffExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTakeoff = val;
}


// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF_core::getTouchdownExpected()
{
    if (expectGndEffectTouchdown && imuSampleTime_ms - touchdownExpectedSet_ms > gndEffectTimeout_ms) {
        expectGndEffectTouchdown = false;
    }

    return expectGndEffectTouchdown;
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF_core::setTouchdownExpected(bool val)
{
    touchdownExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTouchdown = val;
}

/*
   Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds
   Once we have set the origin and are operating in GPS mode the status is set to true to avoid a race conditon with remote useage
   If we have landed with good GPS, then the status is assumed good for 5 seconds to allow transients to settle

   We also record the failure reason so that prearm_failure_reason() can give a good report to the user on why arming is failing
*/
bool NavEKF_core::calcGpsGoodToAlign(void)
{
    static struct Location gpsloc_prev;    // LLH location of previous GPS measurement

    // calculate absolute difference between GPS vert vel and inertial vert vel
    float velDiffAbs;
    if (_ahrs->get_gps().have_vertical_velocity()) {
        velDiffAbs = fabsf(velNED.z - state.velocity.z);
    } else {
        velDiffAbs = 0.0f;
    }

    // fail if velocity difference or reported speed accuracy greater than threshold
    bool gpsVelFail = ((velDiffAbs > 1.0f) || (gpsSpdAccuracy > 1.0f)) && (frontend._gpsCheck & MASK_GPS_SPD_ERR);

    if (velDiffAbs > 1.0f) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vert vel error %.1f", (double)velDiffAbs);
        gpsCheckStatus.bad_VZ = true;
    } else {
        gpsCheckStatus.bad_VZ = false;
    }
    if (gpsSpdAccuracy > 1.0f) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS speed error %.1f", (double)gpsSpdAccuracy);
        gpsCheckStatus.bad_sAcc = true;
    } else {
        gpsCheckStatus.bad_sAcc = false;
    }

    // fail if not enough sats
    bool numSatsFail = (_ahrs->get_gps().num_sats() < 6) && (frontend._gpsCheck & MASK_GPS_NSATS);
    if (numSatsFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS numsats %u (needs 6)", _ahrs->get_gps().num_sats());
        gpsCheckStatus.bad_sats = true;
    } else {
        gpsCheckStatus.bad_sats = false;
    }

    // fail if satellite geometry is poor
    bool hdopFail = (_ahrs->get_gps().get_hdop() > 250)  && (frontend._gpsCheck & MASK_GPS_HDOP);
    if (hdopFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS HDOP %.1f (needs 2.5)", (double)(0.01f * _ahrs->get_gps().get_hdop()));
        gpsCheckStatus.bad_hdop = true;
    } else {
        gpsCheckStatus.bad_hdop = false;
    }

    // fail if horiziontal position accuracy not sufficient
    float hAcc = 0.0f;
    bool hAccFail;
    if (_ahrs->get_gps().horizontal_accuracy(hAcc)) {
        hAccFail = (hAcc > 5.0f)  && (frontend._gpsCheck & MASK_GPS_POS_ERR);
    } else {
        hAccFail =  false;
    }
    if (hAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horiz error %.1f", (double)hAcc);
        gpsCheckStatus.bad_hAcc = true;
    } else {
        gpsCheckStatus.bad_hAcc = false;
    }

    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f) || !consistentMagData) {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if (imuSampleTime_ms - magYawResetTimer_ms > 5000) {
        // reset heading and field states
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        state.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if ((magTestRatio.x > 1.0f || magTestRatio.y > 1.0f) && (frontend._gpsCheck & MASK_GPS_YAW_ERR)) {
        yawFail = true;
    } else {
        yawFail = false;
    }
    if (yawFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Mag yaw error x=%.1f y=%.1f",
                           (double)magTestRatio.x,
                           (double)magTestRatio.y);
        gpsCheckStatus.bad_yaw = true;
    } else {
        gpsCheckStatus.bad_yaw = false;
    }

    // Check for significant change in GPS position if disarmed which indicates bad GPS
    // Note: this assumes we are not flying from a moving vehicle, eg boat
    const struct Location &gpsloc = _ahrs->get_gps().location(); // Current location
    const float posFiltTimeConst = 10.0f; // time constant used to decay position drift
    // calculate time lapsesd since last GPS fix and limit to prevent numerical errors
    float deltaTime = constrain_float(float(lastFixTime_ms - secondLastFixTime_ms)*0.001f,0.01f,posFiltTimeConst);
    // Sum distance moved
    gpsDriftNE += location_diff(gpsloc_prev, gpsloc).length();
    gpsloc_prev = gpsloc;
    // Decay distance moved exponentially to zero
    gpsDriftNE *= (1.0f - deltaTime/posFiltTimeConst);
    // Clamp the fiter state to prevent excessive persistence of large transients
    gpsDriftNE = MIN(gpsDriftNE,10.0f);
    // Fail if more than 3 metres drift after filtering whilst pre-armed when the vehicle is supposed to be stationary
    // This corresponds to a maximum acceptable average drift rate of 0.3 m/s or single glitch event of 3m
    bool gpsDriftFail = (gpsDriftNE > 3.0f) && !vehicleArmed && (frontend._gpsCheck & MASK_GPS_POS_DRIFT);
    if (gpsDriftFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS drift %.1fm (needs 3.0)", (double)gpsDriftNE);
        gpsCheckStatus.bad_horiz_drift = true;
    } else {
        gpsCheckStatus.bad_horiz_drift = false;
    }

    // Check that the vertical GPS vertical velocity is reasonable after noise filtering
    bool gpsVertVelFail;
    if (_ahrs->get_gps().have_vertical_velocity() && !vehicleArmed) {
        // check that the average vertical GPS velocity is close to zero
        gpsVertVelFilt = 0.1f * velNED.z + 0.9f * gpsVertVelFilt;
        gpsVertVelFilt = constrain_float(gpsVertVelFilt,-10.0f,10.0f);
        gpsVertVelFail = (fabsf(gpsVertVelFilt) > 0.3f) && (frontend._gpsCheck & MASK_GPS_VERT_SPD);
    } else if ((frontend._fusionModeGPS == 0) && !_ahrs->get_gps().have_vertical_velocity()) {
        // If the EKF settings require vertical GPS velocity and the receiver is not outputting it, then fail
        gpsVertVelFail = true;
    } else {
        gpsVertVelFail = false;
    }
    if (gpsVertVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vertical speed %.2fm/s (needs 0.30)", (double)fabsf(gpsVertVelFilt));
        gpsCheckStatus.bad_vert_vel = true;
    } else {
        gpsCheckStatus.bad_vert_vel = false;
    }

    // Check that the horizontal GPS vertical velocity is reasonable after noise filtering
    bool gpsHorizVelFail;
    if (!vehicleArmed) {
        gpsHorizVelFilt = 0.1f * pythagorous2(velNED.x,velNED.y) + 0.9f * gpsHorizVelFilt;
        gpsHorizVelFilt = constrain_float(gpsHorizVelFilt,-10.0f,10.0f);
        gpsHorizVelFail = (fabsf(gpsHorizVelFilt) > 0.3f) && (frontend._gpsCheck & MASK_GPS_HORIZ_SPD);
    } else {
        gpsHorizVelFail = false;
    }
    if (gpsHorizVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horizontal speed %.2fm/s (needs 0.30)", (double)gpsDriftNE);
        gpsCheckStatus.bad_horiz_vel = true;
    } else {
        gpsCheckStatus.bad_horiz_vel = false;
    }

    // return healthy if we already have an origin and are inflight to prevent a race condition when checking the status on the ground after landing
    // return healthy for a few seconds after landing so that filter disturbances don't fail the GPS
    static bool usingInFlight = false;
    usingInFlight = (vehicleArmed && validOrigin && !constPosMode) || (!vehicleArmed && usingInFlight && (imuSampleTime_ms - timeAtDisarm_ms) < 5000 && gpsAccuracyGood);

    if (usingInFlight) {
        return true;
    }

    if (lastGpsVelFail_ms == 0) {
        // first time through, start with a failure
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF warmup");
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // record time of fail
    if (gpsVelFail || numSatsFail || hdopFail || hAccFail || yawFail || gpsDriftFail || gpsVertVelFail || gpsHorizVelFail) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // continuous period without fail required to return healthy
    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        return true;
    }
    return false;
}

// report the reason for why the backend is refusing to initialise
const char *NavEKF_core::prearm_failure_reason(void) const
{
    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        // we are not failing
        return nullptr;
    }
    return prearm_fail_string;
}

// Read the range finder and take new measurements if available
// Read at 20Hz and apply a median filter
void NavEKF_core::readRangeFinder(void)
{
    static float storedRngMeas[3];
    static uint32_t storedRngMeasTime_ms[3];
    static uint32_t lastRngMeasTime_ms = 0;
    static uint8_t rngMeasIndex = 0;
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;
    // get theoretical correct range when the vehicle is on the ground
    rngOnGnd = _rng.ground_clearance_cm() * 0.01f;
    if (_rng.status() == RangeFinder::RangeFinder_Good && (imuSampleTime_ms - lastRngMeasTime_ms) > 50) {
        // store samples and sample time into a ring buffer
        rngMeasIndex ++;
        if (rngMeasIndex > 2) {
            rngMeasIndex = 0;
        }
        storedRngMeasTime_ms[rngMeasIndex] = imuSampleTime_ms;
        storedRngMeas[rngMeasIndex] = _rng.distance_cm() * 0.01f;
        // check for three fresh samples and take median
        bool sampleFresh[3];
        for (uint8_t index = 0; index <= 2; index++) {
            sampleFresh[index] = (imuSampleTime_ms - storedRngMeasTime_ms[index]) < 500;
        }
        if (sampleFresh[0] && sampleFresh[1] && sampleFresh[2]) {
            if (storedRngMeas[0] > storedRngMeas[1]) {
                minIndex = 1;
                maxIndex = 0;
            } else {
                maxIndex = 0;
                minIndex = 1;
            }
            if (storedRngMeas[2] > storedRngMeas[maxIndex]) {
                midIndex = maxIndex;
            } else if (storedRngMeas[2] < storedRngMeas[minIndex]) {
                midIndex = minIndex;
            } else {
                midIndex = 2;
            }
            rngMea = MAX(storedRngMeas[midIndex],rngOnGnd);
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
            // recall vehicle states at mid sample time for range finder
            RecallStates(statesAtRngTime, storedRngMeasTime_ms[midIndex] - 25);
        } else if (!vehicleArmed) {
            // if not armed and no return, we assume on ground range
            rngMea = rngOnGnd;
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
            // assume synthetic measurement is at current time (no delay)
            statesAtRngTime = state;
        } else {
            newDataRng = false;
        }
        lastRngMeasTime_ms =  imuSampleTime_ms;
    }
}

// Detect takeoff for optical flow navigation
void NavEKF_core::detectOptFlowTakeoff(void)
{
    if (vehicleArmed && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        takeOffDetected = (takeOffDetected || (rngMea > (rangeAtArming + 0.1f)));
    }
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF_core::getHeightControlLimit(float &height) const
{
    // only ask for limiting if we are doing optical flow navigation
    if (frontend._fusionModeGPS == 3) {
        // If are doing optical flow nav, ensure the height above ground is within range finder limits after accounting for vehicle tilt and control errors
        height = MAX(float(_rng.max_distance_cm()) * 0.007f - 1.0f, 1.0f);
        return true;
    } else {
        return false;
    }
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF_core::getQuaternion(Quaternion& ret) const
{
    ret = state.quat;
}

// align the NE earth magnetic field states with the published declination
void NavEKF_core::alignMagStateDeclination()
{
    // get the magnetic declination
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = state.earth_magfield;
    float magLengthNE = pythagorous2(initMagNED.x,initMagNED.y);
    state.earth_magfield.x = magLengthNE * cosf(magDecAng);
    state.earth_magfield.y = magLengthNE * sinf(magDecAng);
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF_core::getLastYawResetAngle(float &yawAng) const
{
    yawAng = yawResetAngle;
    return lastYawReset_ms;
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF_core::getLastPosNorthEastReset(Vector2f &pos) const
{
    pos = posResetNE;
    return lastPosReset_ms;
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF_core::getLastVelNorthEastReset(Vector2f &vel) const
{
    vel = velResetNE;
    return lastVelReset_ms;
}

// Check for signs of bad gyro health before flight
bool NavEKF_core::checkGyroHealthPreFlight(void) const
{
    bool retVar;
    if (hal.util->get_soft_armed()) {
        // Always return true if we are flying (use arm status as a surrogate for flying)
        retVar = true;
    } else if
    (state.gyro_bias.x < 0.5f*MAX_GYRO_BIAS*dtIMUavg &&
            state.gyro_bias.y < 0.5f*MAX_GYRO_BIAS*dtIMUavg &&
            state.gyro_bias.z < 0.5f*MAX_GYRO_BIAS*dtIMUavg &&
            posTestRatio < 0.1f) {
        // If the synthetic position innovations are too high or the estimated gyro bias exceeds 50% of the available adjustment we declare the gyro as unhealthy
        // this condition is likely caused by excessive gyro bias and the operator should be prompted to perform a gyro calibration and reset.
        retVar = true;
    } else {
        retVar = false;
    }
    return retVar;
}

// returns true of the EKF thinks the GPS is glitching or unavailable
bool NavEKF_core::getGpsGlitchStatus(void) const
{
    return !gpsAccuracyGood;
}

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void NavEKF_core::calcGpsGoodForFlight(void)
{
    // use a simple criteria based on the GPS receivers claimed speed accuracy and the EKF innovation consistency checks
    static bool gpsSpdAccPass = false;
    static bool ekfInnovationsPass = false;

    // set up varaibles and constants used by filter that is applied to GPS speed accuracy
    const float alpha1 = 0.2f; // coefficient for first stage LPF applied to raw speed accuracy data
    const float tau = 10.0f; // time constant (sec) of peak hold decay
    static float lpfFilterState = 0.0f; // first stage LPF filter state
    static float peakHoldFilterState = 0.0f; // peak hold with exponential decay filter state
    static uint32_t lastTime_ms = 0;
    if (lastTime_ms == 0) {
        lastTime_ms =  imuSampleTime_ms;
    }
    float dtLPF = (imuSampleTime_ms - lastTime_ms) * 1e-3f;
    lastTime_ms = imuSampleTime_ms;
    float alpha2 = constrain_float(dtLPF/tau,0.0f,1.0f);

    // get the receivers reported speed accuracy
    float gpsSpdAccRaw;
    if (!_ahrs->get_gps().speed_accuracy(gpsSpdAccRaw)) {
        gpsSpdAccRaw = 0.0f;
    }

    // filter the raw speed accuracy using a LPF
    lpfFilterState = constrain_float((alpha1 * gpsSpdAccRaw + (1.0f - alpha1) * lpfFilterState),0.0f,10.0f);

    // apply a peak hold filter to the LPF output
    peakHoldFilterState = MAX(lpfFilterState,((1.0f - alpha2) * peakHoldFilterState));

    // Apply a threshold test with hysteresis to the filtered GPS speed accuracy data
    if (peakHoldFilterState > 1.5f ) {
        gpsSpdAccPass = false;
    } else if(peakHoldFilterState < 1.0f) {
        gpsSpdAccPass = true;
    }

    // Apply a threshold test with hysteresis to the normalised position and velocity innovations
    // Require a fail for one second and a pass for 10 seconds to transition
    static uint32_t lastInnovPassTime_ms = 0;
    static uint32_t lastInnovFailTime_ms = 0;
    if (lastInnovFailTime_ms == 0) {
        lastInnovFailTime_ms = imuSampleTime_ms;
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    if (velTestRatio < 1.0f && posTestRatio < 1.0f) {
        lastInnovPassTime_ms = imuSampleTime_ms;
    } else if (velTestRatio > 0.7f || posTestRatio > 0.7f) {
        lastInnovFailTime_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - lastInnovPassTime_ms) > 1000) {
        ekfInnovationsPass = false;
    } else if ((imuSampleTime_ms - lastInnovFailTime_ms) > 10000) {
        ekfInnovationsPass = true;
    }

    // both GPS speed accuracy and EKF innovations must pass
    gpsAccuracyGood = gpsSpdAccPass && ekfInnovationsPass;
}
