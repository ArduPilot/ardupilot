#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.5f

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// constructor
NavEKF2_core::NavEKF2_core(void) :
    stateStruct(*reinterpret_cast<struct state_elements *>(&statesArray)),

    _perf_UpdateFilter(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_UpdateFilter")),
    _perf_CovariancePrediction(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_CovariancePrediction")),
    _perf_FuseVelPosNED(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseVelPosNED")),
    _perf_FuseMagnetometer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseMagnetometer")),
    _perf_FuseAirspeed(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseAirspeed")),
    _perf_FuseSideslip(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseSideslip")),
    _perf_TerrainOffset(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_TerrainOffset")),
    _perf_FuseOptFlow(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseOptFlow"))
{
    _perf_test[0] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test0");
    _perf_test[1] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test1");
    _perf_test[2] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test2");
    _perf_test[3] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test3");
    _perf_test[4] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test4");
    _perf_test[5] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test5");
    _perf_test[6] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test6");
    _perf_test[7] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test7");
    _perf_test[8] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test8");
    _perf_test[9] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test9");
}

// setup this core backend
bool NavEKF2_core::setup_core(NavEKF2 *_frontend, uint8_t _imu_index, uint8_t _core_index)
{
    frontend = _frontend;
    imu_index = _imu_index;
    core_index = _core_index;
    _ahrs = frontend->_ahrs;

    /*
      the imu_buffer_length needs to cope with a 260ms delay at a
      maximum fusion rate of 100Hz. Non-imu data coming in at faster
      than 100Hz is downsampled. For 50Hz main loop rate we need a
      shorter buffer.
     */
    if (AP::ins().get_sample_rate() < 100) {
        imu_buffer_length = 13;
    } else {
        // maximum 260 msec delay at 100 Hz fusion rate
        imu_buffer_length = 26;
    }
    if(!storedGPS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedMag.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedBaro.init(OBS_BUFFER_LENGTH)) {
        return false;
    } 
    if(!storedTAS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedOF.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: the use of dual range finders potentially doubles the amount of to be stored
    if(!storedRange.init(2*OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: range beacon data is read one beacon at a time and can arrive at a high rate
    if(!storedRangeBeacon.init(imu_buffer_length)) {
        return false;
    }
    if(!storedExtNav.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF2_core::InitialiseVariables()
{
    // calculate the nominal filter update rate
    const AP_InertialSensor &ins = AP::ins();
    localFilterTimeStep_ms = (uint8_t)(1000*ins.get_loop_delta_t());
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms,10);

    // initialise time stamps
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;
    lastHealthyMagTime_ms = imuSampleTime_ms;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaStep_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = 0;
    lastPosPassTime_ms = 0;
    lastHgtPassTime_ms = 0;
    lastTasPassTime_ms = 0;
    lastYawTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    secondLastGpsTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = 0;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    timeTasReceived_ms = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;
    lastPosResetD_ms = 0;
    lastRngMeasTime_ms = 0;
    terrainHgtStableSet_ms = 0;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = false;
    allMagSensorsFailed = false;
    tasTimeout = true;
    badMagYaw = false;
    badIMUdata = false;
    finalInflightYawInit = false;
    finalInflightMagInit = false;
    dtIMUavg = 0.0025f;
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    flowDataValid = false;
    rangeDataToFuse  = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    inhibitGndState = false;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    PV_AidingModePrev = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    gpsPosAccuracy = 0.0f;
    gpsHgtAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    yawAlignComplete = false;
    stateIndexLim = 23;
    baroStoreIndex = 0;
    rangeStoreIndex = 0;
    magStoreIndex = 0;
    gpsStoreIndex = 0;
    tasStoreIndex = 0;
    ofStoreIndex = 0;
    delAngCorrection.zero();
    velErrintegral.zero();
    posErrintegral.zero();
    gpsGoodToAlign = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    innovationIncrement = 0;
    lastInnovation = 0;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastInnovPassTime_ms = 0;
    lastInnovFailTime_ms = 0;
    gpsAccuracyGood = false;
    memset(&gpsloc_prev, 0, sizeof(gpsloc_prev));
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    memset(&statesArray, 0, sizeof(statesArray));
    posDownDerivative = 0.0f;
    posDown = 0.0f;
    posVelFusionDelayed = false;
    optFlowFusionDelayed = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    posResetNE.zero();
    velResetNE.zero();
    posResetD = 0.0f;
    hgtInnovFiltState = 0.0f;
    if (_ahrs->get_compass()) {
        magSelectIndex = _ahrs->get_compass()->get_primary();
    }
    imuDataDownSampledNew.delAng.zero();
    imuDataDownSampledNew.delVel.zero();
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    runUpdates = false;
    framesSincePredict = 0;
    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;
    gpsYawResetRequest = false;
    posDownAtLastMagReset = stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    quatAtLastMagReset = stateStruct.quat;
    magFieldLearned = false;
    delAngBiasLearned = false;
    memset(&filterStatus, 0, sizeof(filterStatus));
    gpsInhibit = false;
    activeHgtSource = 0;
    memset(&rngMeasIndex, 0, sizeof(rngMeasIndex));
    memset(&storedRngMeasTime_ms, 0, sizeof(storedRngMeasTime_ms));
    memset(&storedRngMeas, 0, sizeof(storedRngMeas));
    terrainHgtStable = true;
    ekfOriginHgtVar = 0.0f;
    ekfGpsRefHgt = 0.0;
    velOffsetNED.zero();
    posOffsetNED.zero();
    memset(&velPosObs, 0, sizeof(velPosObs));

    // range beacon fusion variables
    memset(&rngBcnDataNew, 0, sizeof(rngBcnDataNew));
    memset(&rngBcnDataDelayed, 0, sizeof(rngBcnDataDelayed));
    rngBcnStoreIndex = 0;
    lastRngBcnPassTime_ms = 0;
    rngBcnTestRatio = 0.0f;
    rngBcnHealth = false;
    rngBcnTimeout = true;
    varInnovRngBcn = 0.0f;
    innovRngBcn = 0.0f;
    memset(&lastTimeRngBcn_ms, 0, sizeof(lastTimeRngBcn_ms));
    rngBcnDataToFuse = false;
    beaconVehiclePosNED.zero();
    beaconVehiclePosErr = 1.0f;
    rngBcnLast3DmeasTime_ms = 0;
    rngBcnGoodToAlign = false;
    lastRngBcnChecked = 0;
    receiverPos.zero();
    memset(&receiverPosCov, 0, sizeof(receiverPosCov));
    rngBcnAlignmentStarted =  false;
    rngBcnAlignmentCompleted = false;
    lastBeaconIndex = 0;
    rngBcnPosSum.zero();
    numBcnMeas = 0;
    rngSum = 0.0f;
    N_beacons = 0;
    maxBcnPosD = 0.0f;
    minBcnPosD = 0.0f;
    bcnPosOffset = 0.0f;
    bcnPosOffsetMax = 0.0f;
    bcnPosOffsetMaxVar = 0.0f;
    OffsetMaxInnovFilt = 0.0f;
    bcnPosOffsetMin = 0.0f;
    bcnPosOffsetMinVar = 0.0f;
    OffsetMinInnovFilt = 0.0f;
    rngBcnFuseDataReportIndex = 0;
    memset(&rngBcnFusionReport, 0, sizeof(rngBcnFusionReport));
    last_gps_idx = 0;

    // external nav data fusion
    memset(&extNavDataNew, 0, sizeof(extNavDataNew));
    memset(&extNavDataDelayed, 0, sizeof(extNavDataDelayed));
    extNavDataToFuse = false;
    extNavMeasTime_ms = 0;
    extNavLastPosResetTime_ms = 0;
    extNavUsedForYaw = false;
    extNavUsedForPos = false;
    extNavYawResetRequest = false;

    // zero data buffers
    storedIMU.reset();
    storedGPS.reset();
    storedMag.reset();
    storedBaro.reset();
    storedTAS.reset();
    storedRange.reset();
    storedOutput.reset();
    storedRangeBeacon.reset();
    storedExtNav.reset();
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF2_core::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "EKF2 init failure: No GPS lock");
        statesInitialised = false;
        return false;
    }

    if (statesInitialised) {
        // we are initialised, but we don't return true until the IMU
        // buffer has been filled. This prevents a timing
        // vulnerability with a pause in IMU data during filter startup
        readIMUData();
        readMagData();
        readGpsData();
        readBaroData();
        return storedIMU.is_filled();
    }

    // set re-used variables to zero
    InitialiseVariables();

    const AP_InertialSensor &ins = AP::ins();

    // Initialise IMU data
    dtIMUavg = ins.get_loop_delta_t();
    readIMUData();
    storedIMU.reset_history(imuDataNew);
    imuDataDelayed = imuDataNew;

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = ins.get_accel(imu_index);

    // read the magnetometer data
    readMagData();

    // normalise the acceleration vector
    float pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinf(initAccVec.x);

        // calculate initial roll angle
        roll = atan2f(-initAccVec.y , -initAccVec.z);
    }

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise dynamic states
    stateStruct.velocity.zero();
    stateStruct.position.zero();
    stateStruct.angErr.zero();

    // initialise static process model states
    stateStruct.gyro_bias.zero();
    stateStruct.gyro_scale.x = 1.0f;
    stateStruct.gyro_scale.y = 1.0f;
    stateStruct.gyro_scale.z = 1.0f;
    stateStruct.accel_zbias = 0.0f;
    stateStruct.wind_vel.zero();
    stateStruct.earth_magfield.zero();
    stateStruct.body_magfield.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readBaroData();
    ResetHeight();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset output states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // we initially return false to wait for the IMU buffer to fill
    return false;
}

// initialise the covariance matrix
void NavEKF2_core::CovarianceInit()
{
    // zero the matrix
    memset(P, 0, sizeof(P));

    // attitude error
    P[0][0]   = 0.1f;
    P[1][1]   = 0.1f;
    P[2][2]   = 0.1f;
    // velocities
    P[3][3]   = sq(frontend->_gpsHorizVelNoise);
    P[4][4]   = P[3][3];
    P[5][5]   = sq(frontend->_gpsVertVelNoise);
    // positions
    P[6][6]   = sq(frontend->_gpsHorizPosNoise);
    P[7][7]   = P[6][6];
    P[8][8]   = sq(frontend->_baroAltNoise);
    // gyro delta angle biases
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
    // gyro scale factor biases
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];
    // Z delta velocity bias
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtEkfAvg);
    // earth magnetic field
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities
    P[22][22] = 0.0f;
    P[23][23]  = P[22][22];

    // optical flow ground height covariance
    Popt = 0.25f;
}

/********************************************************
*                 UPDATE FUNCTIONS                      *
********************************************************/
// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2_core::UpdateFilter(bool predict)
{
    // Set the flag to indicate to the filter that the front-end has given permission for a new state prediction cycle to be started
    startPredictEnabled = predict;

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
#if EK2_DISABLE_INTERRUPTS
    irqstate_t istate = irqsave();
#endif
    hal.util->perf_begin(_perf_UpdateFilter);

    // TODO - in-flight restart method

    //get starting time for update step
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData();

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    if (runUpdates) {
        // Predict states using IMU data from the delayed time horizon
        UpdateStrapdownEquationsNED();

        // Predict the covariance growth
        CovariancePrediction();

        // Update states using  magnetometer data
        SelectMagFusion();

        // Update states using GPS and altimeter data
        SelectVelPosFusion();

        // Update states using range beacon data
        SelectRngBcnFusion();

        // Update states using optical flow data
        SelectFlowFusion();

        // Update states using airspeed data
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles
        SelectBetaFusion();

        // Update the filter status
        updateFilterStatus();
    }

    // Wind output forward from the fusion to output time horizon
    calcOutputStates();

    // stop the timer used for load measurement
    hal.util->perf_end(_perf_UpdateFilter);
#if EK2_DISABLE_INTERRUPTS
    irqrestore(istate);
#endif
}

void NavEKF2_core::correctDeltaAngle(Vector3f &delAng, float delAngDT)
{
    delAng.x = delAng.x * stateStruct.gyro_scale.x;
    delAng.y = delAng.y * stateStruct.gyro_scale.y;
    delAng.z = delAng.z * stateStruct.gyro_scale.z;
    delAng -= stateStruct.gyro_bias * (delAngDT / dtEkfAvg);
}

void NavEKF2_core::correctDeltaVelocity(Vector3f &delVel, float delVelDT)
{
    delVel.z -= stateStruct.accel_zbias * (delVelDT / dtEkfAvg);
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observtion is fused. This attitude error is then used to correct
 * the quaternion.
*/
void NavEKF2_core::UpdateStrapdownEquationsNED()
{
    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    // apply correction for earth's rotation rate
    // % * - and + operators have been overloaded
    stateStruct.quat.rotate(delAngCorrected - prevTnb * earthRateNED*imuDataDelayed.delAngDT);
    stateStruct.quat.normalize();

    // transform body delta velocities to delta velocities in the nav frame
    // use the nav frame from previous time step as the delta velocities
    // have been rotated into that frame
    // * and + operators have been overloaded
    Vector3f delVelNav;  // delta velocity vector in earth axes
    delVelNav  = prevTnb.mul_transpose(delVelCorrected);
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the body to nav cosine matrix
    stateStruct.quat.inverse().rotation_matrix(prevTnb);

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = norm(velDotNEDfilt.x , velDotNEDfilt.y);

    // if we are not aiding, then limit the horizontal magnitude of acceleration
    // to prevent large manoeuvre transients disturbing the attitude
    if ((PV_AidingMode == AID_NONE) && (accNavMagHoriz > 5.0f)) {
        float gain = 5.0f/accNavMagHoriz;
        delVelNav.x *= gain;
        delVelNav.y *= gain;
    }

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3f lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF += delAngCorrected;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();
}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void NavEKF2_core::calcOutputStates()
{
    // apply corrections to the IMU data
    Vector3f delAngNewCorrected = imuDataNew.delAng;
    Vector3f delVelNewCorrected = imuDataNew.delVel;
    correctDeltaAngle(delAngNewCorrected, imuDataNew.delAngDT);
    correctDeltaVelocity(delVelNewCorrected, imuDataNew.delVelDT);

    // apply corections to track EKF solution
    Vector3f delAng = delAngNewCorrected + delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    Quaternion deltaQuat;
    deltaQuat.from_axis_angle(delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    outputDataNew.quat *= deltaQuat;
    outputDataNew.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    Vector3f delVelNav  = Tbn_temp*delVelNewCorrected;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3f lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f);

    // If the IMU accelerometer is offset from the body frame origin, then calculate corrections
    // that can be added to the EKF velocity and position outputs so that they represent the velocity
    // and position of the body frame origin.
    // Note the * operator has been overloaded to operate as a dot product
    if (!accelPosOffset.is_zero()) {
        // calculate the average angular rate across the last IMU update
        // note delAngDT is prevented from being zero in readIMUData()
        Vector3f angRate = imuDataNew.delAng * (1.0f/imuDataNew.delAngDT);

        // Calculate the velocity of the body frame origin relative to the IMU in body frame
        // and rotate into earth frame. Note % operator has been overloaded to perform a cross product
        Vector3f velBodyRelIMU = angRate % (- accelPosOffset);
        velOffsetNED = Tbn_temp * velBodyRelIMU;

        // calculate the earth frame position of the body frame origin relative to the IMU
        posOffsetNED = Tbn_temp * (- accelPosOffset);
    } else {
        velOffsetNED.zero();
        posOffsetNED.zero();
    }

    // store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
    if (runUpdates) {
        // store the states at the output time horizon
        storedOutput[storedIMU.get_youngest_index()] = outputDataNew;

        // recall the states from the fusion time horizon
        outputDataDelayed = storedOutput[storedIMU.get_oldest_index()];

        // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction

        // divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = stateStruct.quat / outputDataDelayed.quat;

        // Convert to a delta rotation using a small angle approximation
        quatErr.normalize();
        Vector3f deltaAngErr;
        float scaler;
        if (quatErr[0] >= 0.0f) {
            scaler = 2.0f;
        } else {
            scaler = -2.0f;
        }
        deltaAngErr.x = scaler * quatErr[1];
        deltaAngErr.y = scaler * quatErr[2];
        deltaAngErr.z = scaler * quatErr[3];

        // calculate a gain that provides tight tracking of the estimator states and
        // adjust for changes in time delay to maintain consistent damping ratio of ~0.7
        float timeDelay = 1e-3f * (float)(imuDataNew.time_ms - imuDataDelayed.time_ms);
        timeDelay = fmaxf(timeDelay, dtIMUavg);
        float errorGain = 0.5f / timeDelay;

        // calculate a corrrection to the delta angle
        // that will cause the INS to track the EKF quaternions
        delAngCorrection = deltaAngErr * errorGain * dtIMUavg;

        // calculate velocity and position tracking errors
        Vector3f velErr = (stateStruct.velocity - outputDataDelayed.velocity);
        Vector3f posErr = (stateStruct.position - outputDataDelayed.position);

        // collect magnitude tracking error for diagnostics
        outputTrackError.x = deltaAngErr.length();
        outputTrackError.y = velErr.length();
        outputTrackError.z = posErr.length();

        // convert user specified time constant from centi-seconds to seconds
        float tauPosVel = constrain_float(0.01f*(float)frontend->_tauVelPosOutput, 0.1f, 0.5f);

        // calculate a gain to track the EKF position states with the specified time constant
        float velPosGain = dtEkfAvg / constrain_float(tauPosVel, dtEkfAvg, 10.0f);

        // use a PI feedback to calculate a correction that will be applied to the output state history
        posErrintegral += posErr;
        velErrintegral += velErr;
        Vector3f velCorrection = velErr * velPosGain + velErrintegral * sq(velPosGain) * 0.1f;
        Vector3f posCorrection = posErr * velPosGain + posErrintegral * sq(velPosGain) * 0.1f;

        // loop through the output filter state history and apply the corrections to the velocity and position states
        // this method is too expensive to use for the attitude states due to the quaternion operations required
        // but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
        // to be used
        output_elements outputStates;
        for (unsigned index=0; index < imu_buffer_length; index++) {
            outputStates = storedOutput[index];

            // a constant  velocity correction is applied
            outputStates.velocity += velCorrection;

            // a constant position correction is applied
            outputStates.position += posCorrection;

            // push the updated data to the buffer
            storedOutput[index] = outputStates;
        }

        // update output state to corrected values
        outputDataNew = storedOutput[storedIMU.get_youngest_index()];

    }
}

/*
 * Calculate the predicted state covariance matrix using algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::CovariancePrediction()
{
    hal.util->perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma;// delta angle scale factor 1-Sigma process noise
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxNoise;     // X axis delta angle noise variance rad^2
    float dayNoise;     // Y axis delta angle noise variance rad^2
    float dazNoise;     // Z axis delta angle noise variance rad^2
    float dvxNoise;     // X axis delta velocity variance noise (m/s)^2
    float dvyNoise;     // Y axis delta velocity variance noise (m/s)^2
    float dvzNoise;     // Z axis delta velocity variance noise (m/s)^2
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
    float dax_s;        // X axis delta angle measurement scale factor
    float day_s;        // Y axis delta angle measurement scale factor
    float daz_s;        // Z axis delta angle measurement scale factor
    float dvz_b;        // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    dt = imuDataDelayed.delAngDT;
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(frontend->_windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrain_float(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = sq(dt) * constrain_float(frontend->_gyroBiasProcessNoise, 0.0f, 1.0f);
    dVelBiasSigma = sq(dt) * constrain_float(frontend->_accelBiasProcessNoise, 0.0f, 1.0f);
    dAngScaleSigma = dt * constrain_float(frontend->_gyroScaleProcessNoise, 0.0f, 1.0f);
    magEarthSigma = dt * constrain_float(frontend->_magEarthProcessNoise, 0.0f, 1.0f);
    magBodySigma  = dt * constrain_float(frontend->_magBodyProcessNoise, 0.0f, 1.0f);
    for (uint8_t i= 0; i<=8;  i++) processNoise[i] = 0.0f;
    for (uint8_t i=9; i<=11; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=12; i<=14; i++) processNoise[i] = dAngScaleSigma;
    if (expectGndEffectTakeoff) {
        processNoise[15] = 0.0f;
    } else {
        processNoise[15] = dVelBiasSigma;
    }
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i=22; i<=23; i++) processNoise[i] = windVelSigma;

    for (uint8_t i= 0; i<=stateIndexLim; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = imuDataDelayed.delVel.x;
    dvy = imuDataDelayed.delVel.y;
    dvz = imuDataDelayed.delVel.z;
    dax = imuDataDelayed.delAng.x;
    day = imuDataDelayed.delAng.y;
    daz = imuDataDelayed.delAng.z;
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    dax_b = stateStruct.gyro_bias.x;
    day_b = stateStruct.gyro_bias.y;
    daz_b = stateStruct.gyro_bias.z;
    dax_s = stateStruct.gyro_scale.x;
    day_s = stateStruct.gyro_scale.y;
    daz_s = stateStruct.gyro_scale.z;
    dvz_b = stateStruct.accel_zbias;
    float _gyrNoise = constrain_float(frontend->_gyrNoise, 0.0f, 1.0f);
    daxNoise = dayNoise = dazNoise = sq(dt*_gyrNoise);
    float _accNoise = constrain_float(frontend->_accNoise, 0.0f, 10.0f);
    dvxNoise = dvyNoise = dvzNoise = sq(dt*_accNoise);

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    SF[0] = daz_b/2 - (daz*daz_s)/2;
    SF[1] = day_b/2 - (day*day_s)/2;
    SF[2] = dax_b/2 - (dax*dax_s)/2;
    SF[3] = q3/2 - (q0*SF[0])/2 + (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[4] = q0/2 - (q1*SF[2])/2 - (q2*SF[1])/2 + (q3*SF[0])/2;
    SF[5] = q1/2 + (q0*SF[2])/2 - (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[6] = q3/2 + (q0*SF[0])/2 - (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[7] = q0/2 - (q1*SF[2])/2 + (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[8] = q0/2 + (q1*SF[2])/2 - (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[9] = q2/2 + (q0*SF[1])/2 + (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[10] = q2/2 - (q0*SF[1])/2 - (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[11] = q2/2 + (q0*SF[1])/2 - (q1*SF[0])/2 - (q3*SF[2])/2;
    SF[12] = q1/2 + (q0*SF[2])/2 + (q2*SF[0])/2 + (q3*SF[1])/2;
    SF[13] = q1/2 - (q0*SF[2])/2 + (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[14] = q3/2 + (q0*SF[0])/2 + (q1*SF[1])/2 + (q2*SF[2])/2;
    SF[15] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SF[16] = dvz_b - dvz;
    SF[17] = dvx;
    SF[18] = dvy;
    SF[19] = sq(q2);
    SF[20] = SF[19] - sq(q0) + sq(q1) - sq(q3);
    SF[21] = SF[19] + sq(q0) - sq(q1) - sq(q3);
    SF[22] = 2*q0*q1 - 2*q2*q3;
    SF[23] = SF[19] - sq(q0) - sq(q1) + sq(q3);
    SF[24] = 2*q1*q2;

    SG[0] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);

    SQ[0] = - dvyNoise*(2*q0*q1 + 2*q2*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxNoise*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2);
    SQ[1] = dvxNoise*(2*q0*q2 - 2*q1*q3)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvzNoise*(2*q0*q2 + 2*q1*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyNoise*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2);
    SQ[2] = dvyNoise*(2*q0*q3 - 2*q1*q2)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxNoise*(2*q0*q3 + 2*q1*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3);
    SQ[3] = sq(SG[0]);
    SQ[4] = 2*q2*q3;
    SQ[5] = 2*q1*q3;
    SQ[6] = 2*q1*q2;
    SQ[7] = SG[4];

    SPP[0] = SF[17]*(2*q0*q1 + 2*q2*q3) + SF[18]*(2*q0*q2 - 2*q1*q3);
    SPP[1] = SF[18]*(2*q0*q2 + 2*q1*q3) + SF[16]*(SF[24] - 2*q0*q3);
    SPP[2] = 2*q3*SF[8] + 2*q1*SF[11] - 2*q0*SF[14] - 2*q2*SF[13];
    SPP[3] = 2*q1*SF[7] + 2*q2*SF[6] - 2*q0*SF[12] - 2*q3*SF[10];
    SPP[4] = 2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12];
    SPP[5] = 2*q0*SF[8] + 2*q2*SF[11] + 2*q1*SF[13] + 2*q3*SF[14];
    SPP[6] = 2*q0*SF[7] + 2*q3*SF[6] + 2*q2*SF[10] + 2*q1*SF[12];
    SPP[7] = SF[18]*SF[20] - SF[16]*(2*q0*q1 + 2*q2*q3);
    SPP[8] = 2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9];
    SPP[9] = 2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9];
    SPP[10] = SF[17]*SF[20] + SF[16]*(2*q0*q2 - 2*q1*q3);
    SPP[11] = SF[17]*SF[21] - SF[18]*(SF[24] + 2*q0*q3);
    SPP[12] = SF[17]*SF[22] - SF[16]*(SF[24] + 2*q0*q3);
    SPP[13] = 2*q0*SF[4] + 2*q1*SF[5] + 2*q3*SF[3] + 2*q2*SF[9];
    SPP[14] = 2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13];
    SPP[15] = SF[18]*SF[23] + SF[17]*(SF[24] - 2*q0*q3);
    SPP[16] = daz*SF[19] + daz*sq(q0) + daz*sq(q1) + daz*sq(q3);
    SPP[17] = day*SF[19] + day*sq(q0) + day*sq(q1) + day*sq(q3);
    SPP[18] = dax*SF[19] + dax*sq(q0) + dax*sq(q1) + dax*sq(q3);
    SPP[19] = SF[16]*SF[23] - SF[17]*(2*q0*q2 + 2*q1*q3);
    SPP[20] = SF[16]*SF[21] - SF[18]*SF[22];
    SPP[21] = 2*q0*q2 + 2*q1*q3;
    SPP[22] = SF[15];

    if (inhibitMagStates) {
        zeroRows(P,16,21);
        zeroCols(P,16,21);
    } else if (inhibitWindStates) {
        zeroRows(P,22,23);
        zeroCols(P,22,23);
    }

    nextP[0][0] = daxNoise*SQ[3] + SPP[5]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[9][0]*SPP[22] + P[12][0]*SPP[18] + P[2][0]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) - SPP[4]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[9][1]*SPP[22] + P[12][1]*SPP[18] + P[2][1]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[8]*(P[0][2]*SPP[5] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18] - P[1][2]*(2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12])) + SPP[22]*(P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[9][9]*SPP[22] + P[12][9]*SPP[18] + P[2][9]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[18]*(P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[9][12]*SPP[22] + P[12][12]*SPP[18] + P[2][12]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9]));
    nextP[0][1] = SPP[6]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) - SPP[2]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[22]*(P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18]) + SPP[17]*(P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18]) - (2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9])*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][1] = dayNoise*SQ[3] - SPP[2]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[6]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) - SPP[9]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[22]*(P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17]) + SPP[17]*(P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17]);
    nextP[0][2] = SPP[13]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[3]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[22]*(P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18]) + SPP[16]*(P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]);
    nextP[1][2] = SPP[13]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[3]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[22]*(P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17]) + SPP[16]*(P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]);
    nextP[2][2] = dazNoise*SQ[3] - SPP[3]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[14]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[13]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[22]*(P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16]) + SPP[16]*(P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16]);
    nextP[0][3] = P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18] + SPP[1]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[15]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[21]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][3] = P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17] + SPP[1]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[15]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[21]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][3] = P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16] + SPP[1]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[15]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) - SPP[21]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][3] = P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21] + dvyNoise*sq(SQ[6] - 2*q0*q3) + dvzNoise*sq(SQ[5] + 2*q0*q2) + SPP[1]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[19]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[15]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) - SPP[21]*(P[3][15] + P[0][15]*SPP[1] + P[2][15]*SPP[15] - P[15][15]*SPP[21] + P[1][15]*(SF[16]*SF[23] - SF[17]*SPP[21])) + dvxNoise*sq(SG[1] + SG[2] - SG[3] - SQ[7]);
    nextP[0][4] = P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18] + SF[22]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + SPP[12]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[20]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[11]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][4] = P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17] + SF[22]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + SPP[12]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[20]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[11]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]);
    nextP[2][4] = P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16] + SF[22]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + SPP[12]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[20]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[11]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]);
    nextP[3][4] = P[3][4] + SQ[2] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21] + SF[22]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) + SPP[12]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[20]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[11]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]);
    nextP[4][4] = P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11] + dvxNoise*sq(SQ[6] + 2*q0*q3) + dvzNoise*sq(SQ[4] - 2*q0*q1) + SF[22]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) + SPP[12]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]) + SPP[20]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[11]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + dvyNoise*sq(SG[1] - SG[2] + SG[3] - SQ[7]);
    nextP[0][5] = P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18] + SF[20]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) - SPP[7]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[0]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) + SPP[10]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][5] = P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17] + SF[20]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) - SPP[7]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[0]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[10]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][5] = P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16] + SF[20]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) - SPP[7]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[0]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[10]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][5] = P[3][5] + SQ[1] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21] + SF[20]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) - SPP[7]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[0]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) + SPP[10]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]);
    nextP[4][5] = P[4][5] + SQ[0] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11] + SF[20]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) - SPP[7]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[0]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + SPP[10]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]);
    nextP[5][5] = P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0] + dvxNoise*sq(SQ[5] - 2*q0*q2) + dvyNoise*sq(SQ[4] + 2*q0*q1) + SF[20]*(P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0]) - SPP[7]*(P[5][0] + P[15][0]*SF[20] - P[0][0]*SPP[7] + P[1][0]*SPP[10] + P[2][0]*SPP[0]) + SPP[0]*(P[5][2] + P[15][2]*SF[20] - P[0][2]*SPP[7] + P[1][2]*SPP[10] + P[2][2]*SPP[0]) + SPP[10]*(P[5][1] + P[15][1]*SF[20] - P[0][1]*SPP[7] + P[1][1]*SPP[10] + P[2][1]*SPP[0]) + dvzNoise*sq(SG[1] - SG[2] - SG[3] + SQ[7]);
    nextP[0][6] = P[0][6]*SPP[5] - P[1][6]*SPP[4] + P[2][6]*SPP[8] + P[9][6]*SPP[22] + P[12][6]*SPP[18] + dt*(P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18]);
    nextP[1][6] = P[1][6]*SPP[6] - P[0][6]*SPP[2] - P[2][6]*SPP[9] + P[10][6]*SPP[22] + P[13][6]*SPP[17] + dt*(P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17]);
    nextP[2][6] = P[0][6]*SPP[14] - P[1][6]*SPP[3] + P[2][6]*SPP[13] + P[11][6]*SPP[22] + P[14][6]*SPP[16] + dt*(P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16]);
    nextP[3][6] = P[3][6] + P[0][6]*SPP[1] + P[1][6]*SPP[19] + P[2][6]*SPP[15] - P[15][6]*SPP[21] + dt*(P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21]);
    nextP[4][6] = P[4][6] + P[15][6]*SF[22] + P[0][6]*SPP[20] + P[1][6]*SPP[12] + P[2][6]*SPP[11] + dt*(P[4][3] + P[15][3]*SF[22] + P[0][3]*SPP[20] + P[1][3]*SPP[12] + P[2][3]*SPP[11]);
    nextP[5][6] = P[5][6] + P[15][6]*SF[20] - P[0][6]*SPP[7] + P[1][6]*SPP[10] + P[2][6]*SPP[0] + dt*(P[5][3] + P[15][3]*SF[20] - P[0][3]*SPP[7] + P[1][3]*SPP[10] + P[2][3]*SPP[0]);
    nextP[6][6] = P[6][6] + P[3][6]*dt + dt*(P[6][3] + P[3][3]*dt);
    nextP[0][7] = P[0][7]*SPP[5] - P[1][7]*SPP[4] + P[2][7]*SPP[8] + P[9][7]*SPP[22] + P[12][7]*SPP[18] + dt*(P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18]);
    nextP[1][7] = P[1][7]*SPP[6] - P[0][7]*SPP[2] - P[2][7]*SPP[9] + P[10][7]*SPP[22] + P[13][7]*SPP[17] + dt*(P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17]);
    nextP[2][7] = P[0][7]*SPP[14] - P[1][7]*SPP[3] + P[2][7]*SPP[13] + P[11][7]*SPP[22] + P[14][7]*SPP[16] + dt*(P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16]);
    nextP[3][7] = P[3][7] + P[0][7]*SPP[1] + P[1][7]*SPP[19] + P[2][7]*SPP[15] - P[15][7]*SPP[21] + dt*(P[3][4] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21]);
    nextP[4][7] = P[4][7] + P[15][7]*SF[22] + P[0][7]*SPP[20] + P[1][7]*SPP[12] + P[2][7]*SPP[11] + dt*(P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11]);
    nextP[5][7] = P[5][7] + P[15][7]*SF[20] - P[0][7]*SPP[7] + P[1][7]*SPP[10] + P[2][7]*SPP[0] + dt*(P[5][4] + P[15][4]*SF[20] - P[0][4]*SPP[7] + P[1][4]*SPP[10] + P[2][4]*SPP[0]);
    nextP[6][7] = P[6][7] + P[3][7]*dt + dt*(P[6][4] + P[3][4]*dt);
    nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    nextP[0][8] = P[0][8]*SPP[5] - P[1][8]*SPP[4] + P[2][8]*SPP[8] + P[9][8]*SPP[22] + P[12][8]*SPP[18] + dt*(P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18]);
    nextP[1][8] = P[1][8]*SPP[6] - P[0][8]*SPP[2] - P[2][8]*SPP[9] + P[10][8]*SPP[22] + P[13][8]*SPP[17] + dt*(P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17]);
    nextP[2][8] = P[0][8]*SPP[14] - P[1][8]*SPP[3] + P[2][8]*SPP[13] + P[11][8]*SPP[22] + P[14][8]*SPP[16] + dt*(P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16]);
    nextP[3][8] = P[3][8] + P[0][8]*SPP[1] + P[1][8]*SPP[19] + P[2][8]*SPP[15] - P[15][8]*SPP[21] + dt*(P[3][5] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21]);
    nextP[4][8] = P[4][8] + P[15][8]*SF[22] + P[0][8]*SPP[20] + P[1][8]*SPP[12] + P[2][8]*SPP[11] + dt*(P[4][5] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11]);
    nextP[5][8] = P[5][8] + P[15][8]*SF[20] - P[0][8]*SPP[7] + P[1][8]*SPP[10] + P[2][8]*SPP[0] + dt*(P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0]);
    nextP[6][8] = P[6][8] + P[3][8]*dt + dt*(P[6][5] + P[3][5]*dt);
    nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    nextP[0][9] = P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[2][9]*SPP[8] + P[9][9]*SPP[22] + P[12][9]*SPP[18];
    nextP[1][9] = P[1][9]*SPP[6] - P[0][9]*SPP[2] - P[2][9]*SPP[9] + P[10][9]*SPP[22] + P[13][9]*SPP[17];
    nextP[2][9] = P[0][9]*SPP[14] - P[1][9]*SPP[3] + P[2][9]*SPP[13] + P[11][9]*SPP[22] + P[14][9]*SPP[16];
    nextP[3][9] = P[3][9] + P[0][9]*SPP[1] + P[1][9]*SPP[19] + P[2][9]*SPP[15] - P[15][9]*SPP[21];
    nextP[4][9] = P[4][9] + P[15][9]*SF[22] + P[0][9]*SPP[20] + P[1][9]*SPP[12] + P[2][9]*SPP[11];
    nextP[5][9] = P[5][9] + P[15][9]*SF[20] - P[0][9]*SPP[7] + P[1][9]*SPP[10] + P[2][9]*SPP[0];
    nextP[6][9] = P[6][9] + P[3][9]*dt;
    nextP[7][9] = P[7][9] + P[4][9]*dt;
    nextP[8][9] = P[8][9] + P[5][9]*dt;
    nextP[9][9] = P[9][9];
    nextP[0][10] = P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18];
    nextP[1][10] = P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17];
    nextP[2][10] = P[0][10]*SPP[14] - P[1][10]*SPP[3] + P[2][10]*SPP[13] + P[11][10]*SPP[22] + P[14][10]*SPP[16];
    nextP[3][10] = P[3][10] + P[0][10]*SPP[1] + P[1][10]*SPP[19] + P[2][10]*SPP[15] - P[15][10]*SPP[21];
    nextP[4][10] = P[4][10] + P[15][10]*SF[22] + P[0][10]*SPP[20] + P[1][10]*SPP[12] + P[2][10]*SPP[11];
    nextP[5][10] = P[5][10] + P[15][10]*SF[20] - P[0][10]*SPP[7] + P[1][10]*SPP[10] + P[2][10]*SPP[0];
    nextP[6][10] = P[6][10] + P[3][10]*dt;
    nextP[7][10] = P[7][10] + P[4][10]*dt;
    nextP[8][10] = P[8][10] + P[5][10]*dt;
    nextP[9][10] = P[9][10];
    nextP[10][10] = P[10][10];
    nextP[0][11] = P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18];
    nextP[1][11] = P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17];
    nextP[2][11] = P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16];
    nextP[3][11] = P[3][11] + P[0][11]*SPP[1] + P[1][11]*SPP[19] + P[2][11]*SPP[15] - P[15][11]*SPP[21];
    nextP[4][11] = P[4][11] + P[15][11]*SF[22] + P[0][11]*SPP[20] + P[1][11]*SPP[12] + P[2][11]*SPP[11];
    nextP[5][11] = P[5][11] + P[15][11]*SF[20] - P[0][11]*SPP[7] + P[1][11]*SPP[10] + P[2][11]*SPP[0];
    nextP[6][11] = P[6][11] + P[3][11]*dt;
    nextP[7][11] = P[7][11] + P[4][11]*dt;
    nextP[8][11] = P[8][11] + P[5][11]*dt;
    nextP[9][11] = P[9][11];
    nextP[10][11] = P[10][11];
    nextP[11][11] = P[11][11];
    nextP[0][12] = P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[2][12]*SPP[8] + P[9][12]*SPP[22] + P[12][12]*SPP[18];
    nextP[1][12] = P[1][12]*SPP[6] - P[0][12]*SPP[2] - P[2][12]*SPP[9] + P[10][12]*SPP[22] + P[13][12]*SPP[17];
    nextP[2][12] = P[0][12]*SPP[14] - P[1][12]*SPP[3] + P[2][12]*SPP[13] + P[11][12]*SPP[22] + P[14][12]*SPP[16];
    nextP[3][12] = P[3][12] + P[0][12]*SPP[1] + P[1][12]*SPP[19] + P[2][12]*SPP[15] - P[15][12]*SPP[21];
    nextP[4][12] = P[4][12] + P[15][12]*SF[22] + P[0][12]*SPP[20] + P[1][12]*SPP[12] + P[2][12]*SPP[11];
    nextP[5][12] = P[5][12] + P[15][12]*SF[20] - P[0][12]*SPP[7] + P[1][12]*SPP[10] + P[2][12]*SPP[0];
    nextP[6][12] = P[6][12] + P[3][12]*dt;
    nextP[7][12] = P[7][12] + P[4][12]*dt;
    nextP[8][12] = P[8][12] + P[5][12]*dt;
    nextP[9][12] = P[9][12];
    nextP[10][12] = P[10][12];
    nextP[11][12] = P[11][12];
    nextP[12][12] = P[12][12];
    nextP[0][13] = P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18];
    nextP[1][13] = P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17];
    nextP[2][13] = P[0][13]*SPP[14] - P[1][13]*SPP[3] + P[2][13]*SPP[13] + P[11][13]*SPP[22] + P[14][13]*SPP[16];
    nextP[3][13] = P[3][13] + P[0][13]*SPP[1] + P[1][13]*SPP[19] + P[2][13]*SPP[15] - P[15][13]*SPP[21];
    nextP[4][13] = P[4][13] + P[15][13]*SF[22] + P[0][13]*SPP[20] + P[1][13]*SPP[12] + P[2][13]*SPP[11];
    nextP[5][13] = P[5][13] + P[15][13]*SF[20] - P[0][13]*SPP[7] + P[1][13]*SPP[10] + P[2][13]*SPP[0];
    nextP[6][13] = P[6][13] + P[3][13]*dt;
    nextP[7][13] = P[7][13] + P[4][13]*dt;
    nextP[8][13] = P[8][13] + P[5][13]*dt;
    nextP[9][13] = P[9][13];
    nextP[10][13] = P[10][13];
    nextP[11][13] = P[11][13];
    nextP[12][13] = P[12][13];
    nextP[13][13] = P[13][13];
    nextP[0][14] = P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18];
    nextP[1][14] = P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17];
    nextP[2][14] = P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16];
    nextP[3][14] = P[3][14] + P[0][14]*SPP[1] + P[1][14]*SPP[19] + P[2][14]*SPP[15] - P[15][14]*SPP[21];
    nextP[4][14] = P[4][14] + P[15][14]*SF[22] + P[0][14]*SPP[20] + P[1][14]*SPP[12] + P[2][14]*SPP[11];
    nextP[5][14] = P[5][14] + P[15][14]*SF[20] - P[0][14]*SPP[7] + P[1][14]*SPP[10] + P[2][14]*SPP[0];
    nextP[6][14] = P[6][14] + P[3][14]*dt;
    nextP[7][14] = P[7][14] + P[4][14]*dt;
    nextP[8][14] = P[8][14] + P[5][14]*dt;
    nextP[9][14] = P[9][14];
    nextP[10][14] = P[10][14];
    nextP[11][14] = P[11][14];
    nextP[12][14] = P[12][14];
    nextP[13][14] = P[13][14];
    nextP[14][14] = P[14][14];
    nextP[0][15] = P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18];
    nextP[1][15] = P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17];
    nextP[2][15] = P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16];
    nextP[3][15] = P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21];
    nextP[4][15] = P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11];
    nextP[5][15] = P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0];
    nextP[6][15] = P[6][15] + P[3][15]*dt;
    nextP[7][15] = P[7][15] + P[4][15]*dt;
    nextP[8][15] = P[8][15] + P[5][15]*dt;
    nextP[9][15] = P[9][15];
    nextP[10][15] = P[10][15];
    nextP[11][15] = P[11][15];
    nextP[12][15] = P[12][15];
    nextP[13][15] = P[13][15];
    nextP[14][15] = P[14][15];
    nextP[15][15] = P[15][15];

    if (stateIndexLim > 15) {
        nextP[0][16] = P[0][16]*SPP[5] - P[1][16]*SPP[4] + P[2][16]*SPP[8] + P[9][16]*SPP[22] + P[12][16]*SPP[18];
        nextP[1][16] = P[1][16]*SPP[6] - P[0][16]*SPP[2] - P[2][16]*SPP[9] + P[10][16]*SPP[22] + P[13][16]*SPP[17];
        nextP[2][16] = P[0][16]*SPP[14] - P[1][16]*SPP[3] + P[2][16]*SPP[13] + P[11][16]*SPP[22] + P[14][16]*SPP[16];
        nextP[3][16] = P[3][16] + P[0][16]*SPP[1] + P[1][16]*SPP[19] + P[2][16]*SPP[15] - P[15][16]*SPP[21];
        nextP[4][16] = P[4][16] + P[15][16]*SF[22] + P[0][16]*SPP[20] + P[1][16]*SPP[12] + P[2][16]*SPP[11];
        nextP[5][16] = P[5][16] + P[15][16]*SF[20] - P[0][16]*SPP[7] + P[1][16]*SPP[10] + P[2][16]*SPP[0];
        nextP[6][16] = P[6][16] + P[3][16]*dt;
        nextP[7][16] = P[7][16] + P[4][16]*dt;
        nextP[8][16] = P[8][16] + P[5][16]*dt;
        nextP[9][16] = P[9][16];
        nextP[10][16] = P[10][16];
        nextP[11][16] = P[11][16];
        nextP[12][16] = P[12][16];
        nextP[13][16] = P[13][16];
        nextP[14][16] = P[14][16];
        nextP[15][16] = P[15][16];
        nextP[16][16] = P[16][16];
        nextP[0][17] = P[0][17]*SPP[5] - P[1][17]*SPP[4] + P[2][17]*SPP[8] + P[9][17]*SPP[22] + P[12][17]*SPP[18];
        nextP[1][17] = P[1][17]*SPP[6] - P[0][17]*SPP[2] - P[2][17]*SPP[9] + P[10][17]*SPP[22] + P[13][17]*SPP[17];
        nextP[2][17] = P[0][17]*SPP[14] - P[1][17]*SPP[3] + P[2][17]*SPP[13] + P[11][17]*SPP[22] + P[14][17]*SPP[16];
        nextP[3][17] = P[3][17] + P[0][17]*SPP[1] + P[1][17]*SPP[19] + P[2][17]*SPP[15] - P[15][17]*SPP[21];
        nextP[4][17] = P[4][17] + P[15][17]*SF[22] + P[0][17]*SPP[20] + P[1][17]*SPP[12] + P[2][17]*SPP[11];
        nextP[5][17] = P[5][17] + P[15][17]*SF[20] - P[0][17]*SPP[7] + P[1][17]*SPP[10] + P[2][17]*SPP[0];
        nextP[6][17] = P[6][17] + P[3][17]*dt;
        nextP[7][17] = P[7][17] + P[4][17]*dt;
        nextP[8][17] = P[8][17] + P[5][17]*dt;
        nextP[9][17] = P[9][17];
        nextP[10][17] = P[10][17];
        nextP[11][17] = P[11][17];
        nextP[12][17] = P[12][17];
        nextP[13][17] = P[13][17];
        nextP[14][17] = P[14][17];
        nextP[15][17] = P[15][17];
        nextP[16][17] = P[16][17];
        nextP[17][17] = P[17][17];
        nextP[0][18] = P[0][18]*SPP[5] - P[1][18]*SPP[4] + P[2][18]*SPP[8] + P[9][18]*SPP[22] + P[12][18]*SPP[18];
        nextP[1][18] = P[1][18]*SPP[6] - P[0][18]*SPP[2] - P[2][18]*SPP[9] + P[10][18]*SPP[22] + P[13][18]*SPP[17];
        nextP[2][18] = P[0][18]*SPP[14] - P[1][18]*SPP[3] + P[2][18]*SPP[13] + P[11][18]*SPP[22] + P[14][18]*SPP[16];
        nextP[3][18] = P[3][18] + P[0][18]*SPP[1] + P[1][18]*SPP[19] + P[2][18]*SPP[15] - P[15][18]*SPP[21];
        nextP[4][18] = P[4][18] + P[15][18]*SF[22] + P[0][18]*SPP[20] + P[1][18]*SPP[12] + P[2][18]*SPP[11];
        nextP[5][18] = P[5][18] + P[15][18]*SF[20] - P[0][18]*SPP[7] + P[1][18]*SPP[10] + P[2][18]*SPP[0];
        nextP[6][18] = P[6][18] + P[3][18]*dt;
        nextP[7][18] = P[7][18] + P[4][18]*dt;
        nextP[8][18] = P[8][18] + P[5][18]*dt;
        nextP[9][18] = P[9][18];
        nextP[10][18] = P[10][18];
        nextP[11][18] = P[11][18];
        nextP[12][18] = P[12][18];
        nextP[13][18] = P[13][18];
        nextP[14][18] = P[14][18];
        nextP[15][18] = P[15][18];
        nextP[16][18] = P[16][18];
        nextP[17][18] = P[17][18];
        nextP[18][18] = P[18][18];
        nextP[0][19] = P[0][19]*SPP[5] - P[1][19]*SPP[4] + P[2][19]*SPP[8] + P[9][19]*SPP[22] + P[12][19]*SPP[18];
        nextP[1][19] = P[1][19]*SPP[6] - P[0][19]*SPP[2] - P[2][19]*SPP[9] + P[10][19]*SPP[22] + P[13][19]*SPP[17];
        nextP[2][19] = P[0][19]*SPP[14] - P[1][19]*SPP[3] + P[2][19]*SPP[13] + P[11][19]*SPP[22] + P[14][19]*SPP[16];
        nextP[3][19] = P[3][19] + P[0][19]*SPP[1] + P[1][19]*SPP[19] + P[2][19]*SPP[15] - P[15][19]*SPP[21];
        nextP[4][19] = P[4][19] + P[15][19]*SF[22] + P[0][19]*SPP[20] + P[1][19]*SPP[12] + P[2][19]*SPP[11];
        nextP[5][19] = P[5][19] + P[15][19]*SF[20] - P[0][19]*SPP[7] + P[1][19]*SPP[10] + P[2][19]*SPP[0];
        nextP[6][19] = P[6][19] + P[3][19]*dt;
        nextP[7][19] = P[7][19] + P[4][19]*dt;
        nextP[8][19] = P[8][19] + P[5][19]*dt;
        nextP[9][19] = P[9][19];
        nextP[10][19] = P[10][19];
        nextP[11][19] = P[11][19];
        nextP[12][19] = P[12][19];
        nextP[13][19] = P[13][19];
        nextP[14][19] = P[14][19];
        nextP[15][19] = P[15][19];
        nextP[16][19] = P[16][19];
        nextP[17][19] = P[17][19];
        nextP[18][19] = P[18][19];
        nextP[19][19] = P[19][19];
        nextP[0][20] = P[0][20]*SPP[5] - P[1][20]*SPP[4] + P[2][20]*SPP[8] + P[9][20]*SPP[22] + P[12][20]*SPP[18];
        nextP[1][20] = P[1][20]*SPP[6] - P[0][20]*SPP[2] - P[2][20]*SPP[9] + P[10][20]*SPP[22] + P[13][20]*SPP[17];
        nextP[2][20] = P[0][20]*SPP[14] - P[1][20]*SPP[3] + P[2][20]*SPP[13] + P[11][20]*SPP[22] + P[14][20]*SPP[16];
        nextP[3][20] = P[3][20] + P[0][20]*SPP[1] + P[1][20]*SPP[19] + P[2][20]*SPP[15] - P[15][20]*SPP[21];
        nextP[4][20] = P[4][20] + P[15][20]*SF[22] + P[0][20]*SPP[20] + P[1][20]*SPP[12] + P[2][20]*SPP[11];
        nextP[5][20] = P[5][20] + P[15][20]*SF[20] - P[0][20]*SPP[7] + P[1][20]*SPP[10] + P[2][20]*SPP[0];
        nextP[6][20] = P[6][20] + P[3][20]*dt;
        nextP[7][20] = P[7][20] + P[4][20]*dt;
        nextP[8][20] = P[8][20] + P[5][20]*dt;
        nextP[9][20] = P[9][20];
        nextP[10][20] = P[10][20];
        nextP[11][20] = P[11][20];
        nextP[12][20] = P[12][20];
        nextP[13][20] = P[13][20];
        nextP[14][20] = P[14][20];
        nextP[15][20] = P[15][20];
        nextP[16][20] = P[16][20];
        nextP[17][20] = P[17][20];
        nextP[18][20] = P[18][20];
        nextP[19][20] = P[19][20];
        nextP[20][20] = P[20][20];
        nextP[0][21] = P[0][21]*SPP[5] - P[1][21]*SPP[4] + P[2][21]*SPP[8] + P[9][21]*SPP[22] + P[12][21]*SPP[18];
        nextP[1][21] = P[1][21]*SPP[6] - P[0][21]*SPP[2] - P[2][21]*SPP[9] + P[10][21]*SPP[22] + P[13][21]*SPP[17];
        nextP[2][21] = P[0][21]*SPP[14] - P[1][21]*SPP[3] + P[2][21]*SPP[13] + P[11][21]*SPP[22] + P[14][21]*SPP[16];
        nextP[3][21] = P[3][21] + P[0][21]*SPP[1] + P[1][21]*SPP[19] + P[2][21]*SPP[15] - P[15][21]*SPP[21];
        nextP[4][21] = P[4][21] + P[15][21]*SF[22] + P[0][21]*SPP[20] + P[1][21]*SPP[12] + P[2][21]*SPP[11];
        nextP[5][21] = P[5][21] + P[15][21]*SF[20] - P[0][21]*SPP[7] + P[1][21]*SPP[10] + P[2][21]*SPP[0];
        nextP[6][21] = P[6][21] + P[3][21]*dt;
        nextP[7][21] = P[7][21] + P[4][21]*dt;
        nextP[8][21] = P[8][21] + P[5][21]*dt;
        nextP[9][21] = P[9][21];
        nextP[10][21] = P[10][21];
        nextP[11][21] = P[11][21];
        nextP[12][21] = P[12][21];
        nextP[13][21] = P[13][21];
        nextP[14][21] = P[14][21];
        nextP[15][21] = P[15][21];
        nextP[16][21] = P[16][21];
        nextP[17][21] = P[17][21];
        nextP[18][21] = P[18][21];
        nextP[19][21] = P[19][21];
        nextP[20][21] = P[20][21];
        nextP[21][21] = P[21][21];

        if (stateIndexLim > 21) {
            nextP[0][22] = P[0][22]*SPP[5] - P[1][22]*SPP[4] + P[2][22]*SPP[8] + P[9][22]*SPP[22] + P[12][22]*SPP[18];
            nextP[1][22] = P[1][22]*SPP[6] - P[0][22]*SPP[2] - P[2][22]*SPP[9] + P[10][22]*SPP[22] + P[13][22]*SPP[17];
            nextP[2][22] = P[0][22]*SPP[14] - P[1][22]*SPP[3] + P[2][22]*SPP[13] + P[11][22]*SPP[22] + P[14][22]*SPP[16];
            nextP[3][22] = P[3][22] + P[0][22]*SPP[1] + P[1][22]*SPP[19] + P[2][22]*SPP[15] - P[15][22]*SPP[21];
            nextP[4][22] = P[4][22] + P[15][22]*SF[22] + P[0][22]*SPP[20] + P[1][22]*SPP[12] + P[2][22]*SPP[11];
            nextP[5][22] = P[5][22] + P[15][22]*SF[20] - P[0][22]*SPP[7] + P[1][22]*SPP[10] + P[2][22]*SPP[0];
            nextP[6][22] = P[6][22] + P[3][22]*dt;
            nextP[7][22] = P[7][22] + P[4][22]*dt;
            nextP[8][22] = P[8][22] + P[5][22]*dt;
            nextP[9][22] = P[9][22];
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = P[0][23]*SPP[5] - P[1][23]*SPP[4] + P[2][23]*SPP[8] + P[9][23]*SPP[22] + P[12][23]*SPP[18];
            nextP[1][23] = P[1][23]*SPP[6] - P[0][23]*SPP[2] - P[2][23]*SPP[9] + P[10][23]*SPP[22] + P[13][23]*SPP[17];
            nextP[2][23] = P[0][23]*SPP[14] - P[1][23]*SPP[3] + P[2][23]*SPP[13] + P[11][23]*SPP[22] + P[14][23]*SPP[16];
            nextP[3][23] = P[3][23] + P[0][23]*SPP[1] + P[1][23]*SPP[19] + P[2][23]*SPP[15] - P[15][23]*SPP[21];
            nextP[4][23] = P[4][23] + P[15][23]*SF[22] + P[0][23]*SPP[20] + P[1][23]*SPP[12] + P[2][23]*SPP[11];
            nextP[5][23] = P[5][23] + P[15][23]*SF[20] - P[0][23]*SPP[7] + P[1][23]*SPP[10] + P[2][23]*SPP[0];
            nextP[6][23] = P[6][23] + P[3][23]*dt;
            nextP[7][23] = P[7][23] + P[4][23]*dt;
            nextP[8][23] = P[8][23] + P[5][23]*dt;
            nextP[9][23] = P[9][23];
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];
        }
    }

    // Copy upper diagonal to lower diagonal taking advantage of symmetry
    for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++)
    {
        for (uint8_t rowIndex=0; rowIndex<colIndex; rowIndex++)
        {
            nextP[colIndex][rowIndex] = nextP[rowIndex][colIndex];
        }
    }

    // add the general state process noise variances
    for (uint8_t i=0; i<=stateIndexLim; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[6][6] + P[7][7]) > 1e4f)
    {
        for (uint8_t i=6; i<=7; i++)
        {
            for (uint8_t j=0; j<=stateIndexLim; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output
    CopyCovariances();

    // constrain diagonals to prevent ill-conditioning
    ConstrainVariances();

    hal.util->perf_end(_perf_CovariancePrediction);
}

// zero specified range of rows in the state covariance matrix
void NavEKF2_core::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*24);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF2_core::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// reset the output data to the current EKF state
void NavEKF2_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position dervative output
    posDown = stateStruct.position.z;
    posDownDerivative = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF2_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavEKF2_core::StoreQuatRotate(Quaternion deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF2_core::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF2_core::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation
void NavEKF2_core::CopyCovariances()
{
    // copy predicted covariances
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = nextP[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void NavEKF2_core::ConstrainVariances()
{
    for (uint8_t i=0; i<=2; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // attitude error
    for (uint8_t i=3; i<=5; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=6; i<=7; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f);
    P[8][8] = constrain_float(P[8][8],0.0f,1.0e6f); // vertical position
    for (uint8_t i=9; i<=11; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtEkfAvg)); // delta angle biases
    if (PV_AidingMode != AID_NONE) {
        for (uint8_t i=12; i<=14; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // delta angle scale factors
    } else {
        // we can't reliably estimate scale factors when there is no aiding data due to transient manoeuvre induced innovations
        // so inhibit estimation by keeping covariance elements at zero
        zeroRows(P,12,14);
        zeroCols(P,12,14);
    }
    P[15][15] = constrain_float(P[15][15],0.0f,sq(10.0f * dtEkfAvg)); // delta velocity bias
    for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // earth magnetic field
    for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // body magnetic field
    for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // wind velocity
}

// constrain states to prevent ill-conditioning
void NavEKF2_core::ConstrainStates()
{
    // attitude errors are limited between +-1
    for (uint8_t i=0; i<=2; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=3; i<=5; i++) statesArray[i] = constrain_float(statesArray[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    for (uint8_t i=6; i<=7; i++) statesArray[i] = constrain_float(statesArray[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    stateStruct.position.z = constrain_float(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit (this needs to be set based on manufacturers specs)
    for (uint8_t i=9; i<=11; i++) statesArray[i] = constrain_float(statesArray[i],-GYRO_BIAS_LIMIT*dtEkfAvg,GYRO_BIAS_LIMIT*dtEkfAvg);
    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    for (uint8_t i=12; i<=14; i++) statesArray[i] = constrain_float(statesArray[i],0.95f,1.05f);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    stateStruct.accel_zbias = constrain_float(stateStruct.accel_zbias,-1.0f*dtEkfAvg,1.0f*dtEkfAvg);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_float(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_float(statesArray[i],-100.0f,100.0f);
    // constrain the terrain state to be below the vehicle height unless we are using terrain as the height datum
    if (!inhibitGndState) {
        terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);
    }
}

// calculate the NED earth spin vector in rad/sec
void NavEKF2_core::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
Quaternion NavEKF2_core::calcQuatAndFieldStates(float roll, float pitch)
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
        initMagNED = Tbn * magDataDelayed.mag;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;

        // calculate initial filter quaternion states using yaw from magnetometer
        // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
        Vector3f tempEuler;
        stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
        // this check ensures we accumulate the resets that occur within a single iteration of the EKF
        if (imuSampleTime_ms != lastYawReset_ms) {
            yawResetAngle = 0.0f;
        }
        yawResetAngle += wrap_PI(yaw - tempEuler.z);
        lastYawReset_ms = imuSampleTime_ms;
        // calculate an initial quaternion using the new yaw value
        initQuat.from_euler(roll, pitch, yaw);
        // zero the attitude covariances becasue the corelations will now be invalid
        zeroAttCovOnly();

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        // don't do this if the earth field has already been learned
        if (!magFieldLearned) {
            initQuat.rotation_matrix(Tbn);
            stateStruct.earth_magfield = Tbn * magDataDelayed.mag;

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();

            // set the remaining variances and covariances
            zeroRows(P,18,21);
            zeroCols(P,18,21);
            P[18][18] = sq(frontend->_magNoise);
            P[19][19] = P[18][18];
            P[20][20] = P[18][18];
            P[21][21] = P[18][18];

        }

        // record the fact we have initialised the magnetic field states
        recordMagReset();

        // clear mag state reset request
        magStateResetRequest = false;

    } else {
        // this function should not be called if there is no compass data but if is is, return the
        // current attitude
        initQuat = stateStruct.quat;
    }

    // return attitude quaternion
    return initQuat;
}

// zero the attitude covariances, but preserve the variances
void NavEKF2_core::zeroAttCovOnly()
{
    float varTemp[3];
    for (uint8_t index=0; index<=2; index++) {
        varTemp[index] = P[index][index];
    }
    zeroCols(P,0,2);
    zeroRows(P,0,2);
    for (uint8_t index=0; index<=2; index++) {
        P[index][index] = varTemp[index];
    }
}

#endif // HAL_CPU_CLASS
