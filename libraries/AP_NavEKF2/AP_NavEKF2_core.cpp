/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

/*
  optionally turn down optimisation for debugging
 */
// #pragma GCC optimize("O0")

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// when the wind estimation first starts with no airspeed sensor,
// assume 3m/s to start
#define STARTUP_WIND_SPEED 3.0f

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.3f

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.349066f

// constructor
NavEKF2_core::NavEKF2_core(NavEKF2 &_frontend, const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng) :
    frontend(_frontend),
    _ahrs(ahrs),
    _baro(baro),
    _rng(rng),
    stateStruct(*reinterpret_cast<struct state_elements *>(&statesArray)),

    //variables
    lastRngMeasTime_ms(0),          // time in msec that the last range measurement was taken
    rngMeasIndex(0)                 // index into ringbuffer of current range measurement

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_UpdateFilter(perf_alloc(PC_ELAPSED, "EKF_UpdateFilter")),
    _perf_CovariancePrediction(perf_alloc(PC_ELAPSED, "EKF_CovariancePrediction")),
    _perf_FuseVelPosNED(perf_alloc(PC_ELAPSED, "EKF_FuseVelPosNED")),
    _perf_FuseMagnetometer(perf_alloc(PC_ELAPSED, "EKF_FuseMagnetometer")),
    _perf_FuseAirspeed(perf_alloc(PC_ELAPSED, "EKF_FuseAirspeed")),
    _perf_FuseSideslip(perf_alloc(PC_ELAPSED, "EKF_FuseSideslip"))
#endif
{
}

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF2_core::InitialiseVariables()
{
    // initialise time stamps
    imuSampleTime_ms = hal.scheduler->millis();
    lastHealthyMagTime_ms = imuSampleTime_ms;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaStep_ms = imuSampleTime_ms;
    lastMagUpdate_ms = 0;
    lastHgtReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = imuSampleTime_ms;
    lastPosPassTime_ms = imuSampleTime_ms;
    lastHgtPassTime_ms = imuSampleTime_ms;
    lastTasPassTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    secondLastGpsTime_ms = 0;
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
    hgtMeasTime_ms = imuSampleTime_ms;
    magMeasTime_ms = imuSampleTime_ms;
    timeTasReceived_ms = 0;
    magYawResetTimer_ms = imuSampleTime_ms;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = true;
    tasTimeout = true;
    badMag = false;
    badIMUdata = false;
    firstMagYawInit = false;
    secondMagYawInit = false;
    dtIMUavg = 0.0025f;
    dt = 0;
    velDotNEDfilt.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    gpsPosGlitchOffsetNE.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    flowDataValid = false;
    newDataRng  = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    fuseRngData = false;
    inhibitGndState = true;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    gpsVelGlitchOffset.zero();
    isAiding = false;
    prevIsAiding = false;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    yawAligned = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    highYawRate = false;
    yawRateFilt = 0.0f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    yawAlignComplete = false;
    stateIndexLim = 23;
    imuDataNew.frame = 0;
    baroStoreIndex = 0;
    magStoreIndex = 0;
    gpsStoreIndex = 0;
    tasStoreIndex = 0;
    ofStoreIndex = 0;
    delAngCorrection.zero();
    delVelCorrection.zero();
    velCorrection.zero();
    gpsQualGood = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    innovationIncrement = 0;
    lastInnovation = 0;
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF2_core::InitialiseFilterBootstrap(void)
{
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && _ahrs->get_gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        statesInitialised = false;
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // Initialise IMU data
    dtIMUavg = 1.0f/_ahrs->get_ins().get_sample_rate();
    readIMUData();
    StoreIMU_reset();

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

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise static process state model states
    stateStruct.gyro_bias.zero();
    stateStruct.gyro_scale.x = 1.0f;
    stateStruct.gyro_scale.y = 1.0f;
    stateStruct.gyro_scale.z = 1.0f;
    stateStruct.accel_zbias = 0.0f;
    stateStruct.wind_vel.zero();

    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state
    readHgtData();
    ResetHeight();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset output states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    return true;
}

// initialise the covariance matrix
void NavEKF2_core::CovarianceInit()
{
    // zero the matrix
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = 0.0f;
        }
    }
    // attitude error
    P[0][0]   = 0.1f;
    P[1][1]   = 0.1f;
    P[2][2]   = 0.1f;
    // velocities
    P[3][3]   = sq(0.7f);
    P[4][4]   = P[3][3];
    P[5][5]   = sq(0.7f);
    // positions
    P[6][6]   = sq(15.0f);
    P[7][7]   = P[6][6];
    P[8][8]   = sq(frontend._baroAltNoise);
    // gyro delta angle biases
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
    // gyro scale factor biases
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];
    // Z delta velocity bias
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtIMUavg);
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
void NavEKF2_core::UpdateFilter()
{
    // zero the delta quaternion used by the strapdown navigation because it is published
    // and we need to return a zero rotation of the INS fails to update it
    correctedDelAngQuat.initialise();

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
    perf_begin(_perf_UpdateFilter);

    // TODO - in-flight restart method

    //get starting time for update step
    imuSampleTime_ms = hal.scheduler->millis();

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData();

    // State Prediction Step
    // Run the strapdown INS equations to predict kinematic states forward to the fusion time horizon using buffered IMU data
    UpdateStrapdownEquationsNED();

    // sum delta angles and time used by covariance prediction
    summedDelAng = summedDelAng + correctedDelAng;
    summedDelVel = summedDelVel + correctedDelVel;
    dt += imuDataDelayed.delAngDT;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    if (((dt >= (frontend.covTimeStepMax - dtIMUavg)) || (summedDelAng.length() > frontend.covDelAngMax))) {
        CovariancePrediction();
    } else {
        covPredStep = false;
    }

    // Read range finder data which is used by both position and optical flow fusion
    readRangeFinder();

    // Update states using GPS and altimeter data
    SelectVelPosFusion();

    // Update states using optical flow data
    SelectFlowFusion();

    // Update states using  magnetometer data
    SelectMagFusion();

    // Update states using airspeed data
    SelectTasFusion();

    // Update states using sideslip constraint assumption for fly-forward vehicles
    SelectBetaFusion();

    // Wind output forward from the fusion to output time horizon
    calcOutputStatesFast();

    // stop the timer used for load measurement
    perf_end(_perf_UpdateFilter);
}

// update the quaternion, velocity and position states using delayed IMU measurements
// because the EKF is running on a delayed time horizon
void NavEKF2_core::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;  // delta velocity vector

    // remove gyro scale factor errors
    correctedDelAng.x = imuDataDelayed.delAng.x * stateStruct.gyro_scale.x;
    correctedDelAng.y = imuDataDelayed.delAng.y * stateStruct.gyro_scale.y;
    correctedDelAng.z = imuDataDelayed.delAng.z * stateStruct.gyro_scale.z;

    // remove sensor bias errors
    correctedDelAng -= stateStruct.gyro_bias;
    correctedDelVel = imuDataDelayed.delVel;
    correctedDelVel.z -= stateStruct.accel_zbias;

    // apply correction for earths rotation rate
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*imuDataDelayed.delAngDT;

    // convert the rotation vector to its equivalent quaternion
    correctedDelAngQuat.from_axis_angle(correctedDelAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    stateStruct.quat *= correctedDelAngQuat;
    stateStruct.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    delVelNav  = Tbn_temp*correctedDelVel;
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = pythagorous2(velDotNEDfilt.x , velDotNEDfilt.y);

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF += imuDataNew.delAng - stateStruct.gyro_bias;
    delTimeOF += imuDataNew.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();
}

// Propagate PVA solution forward from the fusion time horizon to the current time horizon
// using buffered IMU data
void  NavEKF2_core::calcOutputStates() {

    // initialise the store access at the fusion time horizon (it will be advanced later)
    uint8_t imuStoreAccessIndex = fifoIndexDelayed;
    imu_elements imuData;

    // Counter used to ensure the while loop always exits
    uint8_t watchdog = 0;

    // initialise to the solution at the fusion time horizon
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;

    // Iterate through the buffered IMU data, using the strapdown equations to wind forward from the fusion time horizon to current time
    // we stop iterating when we have reached the current imu Data
    do {

        // If the loop cannot exit, force exit
        if (watchdog > IMU_BUFFER_LENGTH+2) {
            return;
        }
        watchdog++;

        // advance to the next index
        imuStoreAccessIndex++;

        // if we have got to the end of the array, return to the start
        if (imuStoreAccessIndex >= IMU_BUFFER_LENGTH) {
            imuStoreAccessIndex = 0;
        }
        imuData = storedIMU[imuStoreAccessIndex];

        // remove gyro bias errors
        Vector3f delAng = imuData.delAng - stateStruct.gyro_bias;

        // remove Z accel bias error
        Vector3f delVel = imuData.delVel;
        delVel.z -= stateStruct.accel_zbias;

        // convert the rotation vector to its equivalent quaternion
        Quaternion deltaQuat;
        deltaQuat.from_axis_angle_fast(delAng);

        // update the quaternion states by rotating from the previous attitude through
        // the delta angle rotation quaternion and normalise
        outputDataNew.quat *= deltaQuat;
        outputDataNew.quat.normalize();

        // calculate the body to nav cosine matrix
        Matrix3f Tbn_temp;
        outputDataNew.quat.rotation_matrix(Tbn_temp);

        // transform body delta velocities to delta velocities in the nav frame
        // * and + operators have been overloaded
        Vector3f delVelNav  = Tbn_temp*delVel;
        delVelNav.z += GRAVITY_MSS*imuData.delVelDT;

        // use a simple Euler integration
        outputDataNew.position += outputDataNew.velocity*imuData.delVelDT;
    }
    while (imuStoreAccessIndex != fifoIndexNow);
}

// Propagate PVA solution forward from the fusion time horizon to the current time horizon
// using simple observer. This also applies an LPF to fusion corrections
void  NavEKF2_core::calcOutputStatesFast() {

    // Calculate strapdown solution at the current time horizon

    // remove gyro scale factor errors
    Vector3f delAng;
    delAng.x = imuDataNew.delAng.x * stateStruct.gyro_scale.x;
    delAng.y = imuDataNew.delAng.y * stateStruct.gyro_scale.y;
    delAng.z = imuDataNew.delAng.z * stateStruct.gyro_scale.z;

    // remove sensor bias errors
    delAng -= stateStruct.gyro_bias;
    Vector3f delVel;
    delVel = imuDataNew.delVel;
    delVel.z -= stateStruct.accel_zbias;

    // apply corections to track EKF solution
    delAng += delAngCorrection;

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
    // Add the earth frame correction required to track the EKF states
    // * and + operators have been overloaded
    Vector3f delVelNav  = Tbn_temp*delVel + delVelCorrection;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal intergration for position calcuation
    Vector3f lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position, applying correction required to track EKF solution
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f) + velCorrection * imuDataNew.delVelDT;

    // store the output in the FIFO buffer
    StoreOutput();

    // extract data at the fusion time horizon from the FIFO buffer
    RecallOutput();

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

    // multiply the angle error vector by a gain to calculate the delta angle correction required to track the EKF solution
    const float Kang = 1.0f;
    delAngCorrection = deltaAngErr * imuDataNew.delAngDT * Kang;

    // multiply velocity error by a gain to calculate the delta velocity correction required to track the EKF solution
    const float Kvel = 1.0f;
    delVelCorrection = (stateStruct.velocity - outputDataDelayed.velocity) * imuDataNew.delVelDT * Kvel;

    // multiply position error by a gain to calculate the velocity correction required to track the EKF solution
    const float Kpos = 1.0f;
    velCorrection = (stateStruct.position - outputDataDelayed.position) * Kpos;

}

// calculate the predicted state covariance matrix
void NavEKF2_core::CovariancePrediction()
{
    perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma;// delta angle scale factor 1-Sigma process noise
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxNoise;     // X axis delta angle noise (rad)
    float dayNoise;     // Y axis delta angle noise (rad)
    float dazNoise;     // Z axis delta angle noise (rad)
    float dvxNoise;     // X axis delta velocity noise (m/s)
    float dvyNoise;     // Y axis delta velocity noise (m/s)
    float dvzNoise;     // Z axis delta velocity noise (m/s)
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
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(frontend._windVelProcessNoise, 0.01f, 1.0f) * (1.0f + constrain_float(frontend._wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = dt * constrain_float(frontend._gyroBiasProcessNoise, 0.0f, 1e-4f);
    dVelBiasSigma = dt * constrain_float(frontend._accelBiasProcessNoise, 1e-6f, 1e-2f);
    dAngScaleSigma = dt * constrain_float(frontend._gyroScaleProcessNoise,1e-6f,1e-2f);
    magEarthSigma = dt * constrain_float(frontend._magProcessNoise, 1e-4f, 1e-2f);
    magBodySigma  = dt * constrain_float(frontend._magProcessNoise, 1e-4f, 1e-2f);
    for (uint8_t i= 0; i<=8;  i++) processNoise[i] = 0.0f;
    for (uint8_t i=9; i<=11; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=12; i<=14; i++) processNoise[i] = dAngScaleSigma;
    processNoise[15] = dVelBiasSigma;
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
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
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
    float _gyrNoise = constrain_float(frontend._gyrNoise, 1e-3f, 5e-2f);
    daxNoise = dt*_gyrNoise;
    dayNoise = dt*_gyrNoise;
    // Account for 3% scale factor error on Z angular rate. This reduces chance of continuous fast rotations causing loss of yaw reference.
    dazNoise = dt*(pythagorous2(_gyrNoise,0.03f*yawRateFilt));
    float _accNoise = constrain_float(frontend._accNoise, 5e-2f, 1.0f);
    dvxNoise = dt*_accNoise;
    dvyNoise = dt*_accNoise;
    dvzNoise = dt*_accNoise;

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    SF[0] = daz_b/2 + dazNoise/2 - (daz*daz_s)/2;
    SF[1] = day_b/2 + dayNoise/2 - (day*day_s)/2;
    SF[2] = dax_b/2 + daxNoise/2 - (dax*dax_s)/2;
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
    SF[16] = dvz_b - dvz + dvzNoise;
    SF[17] = dvx - dvxNoise;
    SF[18] = dvy - dvyNoise;
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

    // set the flag to indicate that covariance prediction has been performed and reset the increments used by the covariance prediction
    covPredStep = true;
    summedDelAng.zero();
    summedDelVel.zero();
    dt = 0.0f;

    perf_end(_perf_CovariancePrediction);
}


// update IMU delta angle and delta velocity measurements
void NavEKF2_core::readIMUData()
{
    const AP_InertialSensor &ins = _ahrs->get_ins();

    // average IMU sampling rate
    dtIMUavg = 1.0f/ins.get_sample_rate();

    // the imu sample time is used as a common time reference throughout the filter
    imuSampleTime_ms = hal.scheduler->millis();

    // Get delta velocity data
    readDeltaVelocity(ins.get_primary_accel(), imuDataNew.delVel, imuDataNew.delVelDT);

    // Get delta angle data
    readDeltaAngle(ins.get_primary_gyro(), imuDataNew.delAng);
    imuDataNew.delAngDT = max(ins.get_delta_time(),1.0e-4f);

    // get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // save data in the FIFO buffer
    StoreIMU();

    // extract the oldest available data from the FIFO buffer
    imuDataDelayed = storedIMU[fifoIndexDelayed];

}

// store imu in the FIFO
void NavEKF2_core::StoreIMU()
{
    fifoIndexDelayed = fifoIndexNow;
    fifoIndexNow = fifoIndexNow + 1;
    if (fifoIndexNow >= IMU_BUFFER_LENGTH) {
        fifoIndexNow = 0;
    }
    storedIMU[fifoIndexNow] = imuDataNew;
}

// reset the stored imu history and store the current value
void NavEKF2_core::StoreIMU_reset()
{
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedIMU[i] = imuDataNew;
    }
    imuDataDelayed = imuDataNew;
    fifoIndexDelayed = fifoIndexNow+1;
    if (fifoIndexDelayed >= IMU_BUFFER_LENGTH) {
        fifoIndexDelayed = 0;
    }
}

// recall IMU data from the FIFO
void NavEKF2_core::RecallIMU()
{
    imuDataDelayed = storedIMU[fifoIndexDelayed];
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

// store output data in the FIFO
void NavEKF2_core::StoreOutput()
{
    storedOutput[fifoIndexNow] = outputDataNew;
}

// reset the output data to the current EKF state
void NavEKF2_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF2_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// recall output data from the FIFO
void NavEKF2_core::RecallOutput()
{
    outputDataDelayed = storedOutput[fifoIndexDelayed];
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
    for (uint8_t i=6; i<=8; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f); // positions
    for (uint8_t i=9; i<=11; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtIMUavg)); // delta angle biases
    for (uint8_t i=12; i<=14; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // delta angle scale factors
    P[15][15] = constrain_float(P[15][15],0.0f,sq(10.0f * dtIMUavg)); // delta velocity bias
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
    for (uint8_t i=9; i<=11; i++) statesArray[i] = constrain_float(statesArray[i],-GYRO_BIAS_LIMIT*dtIMUavg,GYRO_BIAS_LIMIT*dtIMUavg);
    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    for (uint8_t i=12; i<=14; i++) statesArray[i] = constrain_float(statesArray[i],0.95f,1.05f);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    stateStruct.accel_zbias = constrain_float(stateStruct.accel_zbias,-1.0f*dtIMUavg,1.0f*dtIMUavg);
    // earth magnetic field limit
    for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_float(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_float(statesArray[i],-100.0f,100.0f);
    // constrain the terrain offset state
    terrainState = max(terrainState, stateStruct.position.z + rngOnGnd);
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
// if no magnetometer data, do not update magnetic field states and assume zero yaw angle
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
        yawAligned = true;
        // calculate initial filter quaternion states using yaw from magnetometer if mag heading healthy
        // otherwise use existing heading
        if (!badMag) {
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
        } else {
            initQuat = stateStruct.quat;
        }

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        initQuat.rotation_matrix(Tbn);
        stateStruct.earth_magfield = Tbn * magDataDelayed.mag;

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

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF2_core::getRotationBodyToNED(Matrix3f &mat) const
{
    Vector3f trim = _ahrs->get_trim();
    outputDataNew.quat.rotation_matrix(mat);
    mat.rotateXYinv(trim);
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF2_core::getQuaternion(Quaternion& ret) const
{
    ret = outputDataNew.quat;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF2_core::getLastYawResetAngle(float &yawAng)
{
    yawAng = yawResetAngle;
    return lastYawReset_ms;
}

#endif // HAL_CPU_CLASS
