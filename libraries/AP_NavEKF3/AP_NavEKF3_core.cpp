#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_DAL/AP_DAL.h>

// constructor
NavEKF3_core::NavEKF3_core(NavEKF3 *_frontend) :
    frontend(_frontend),
    dal(AP::dal()),
    public_origin(frontend->common_EKF_origin)
{
    firstInitTime_ms = 0;
    lastInitFailReport_ms = 0;
}

// setup this core backend
bool NavEKF3_core::setup_core(uint8_t _imu_index, uint8_t _core_index)
{
    imu_index = _imu_index;
    gyro_index_active = imu_index;
    accel_index_active = imu_index;
    core_index = _core_index;

    /*
      The imu_buffer_length needs to cope with the worst case sensor delay at the
      target EKF state prediction rate. Non-IMU data coming in faster is downsampled.
     */

    // Calculate the expected EKF time step
    if (dal.ins().get_loop_rate_hz() > 0) {
        dtEkfAvg = 1.0f / dal.ins().get_loop_rate_hz();
        dtEkfAvg = MAX(dtEkfAvg,EKF_TARGET_DT);
    } else {
        return false;
    }

    // find the maximum time delay for all potential sensors
    uint16_t maxTimeDelay_ms = MAX(frontend->_hgtDelay_ms ,
            MAX(frontend->_flowDelay_ms ,
                MAX(frontend->_rngBcnDelay_ms ,
                    MAX(frontend->magDelay_ms ,
                        (uint16_t)(EKF_TARGET_DT_MS)
                                  ))));

    // GPS sensing can have large delays and should not be included if disabled
    if (frontend->sources.usingGPS()) {
        // Wait for the configuration of all GPS units to be confirmed. Until this has occurred the GPS driver cannot provide a correct time delay
        float gps_delay_sec = 0;
        if (!dal.gps().get_lag(selected_gps, gps_delay_sec)) {
#ifndef HAL_NO_GCS
            const uint32_t now = dal.millis();
            if (now - lastInitFailReport_ms > 10000) {
                lastInitFailReport_ms = now;
                // provide an escalating series of messages
                MAV_SEVERITY severity = MAV_SEVERITY_INFO;
                if (now > 30000) {
                    severity = MAV_SEVERITY_ERROR;
                } else if (now > 15000) {
                    severity = MAV_SEVERITY_WARNING;
                }
                GCS_SEND_TEXT(severity, "EKF3 waiting for GPS config data");
            }
#endif
            return false;
        }
        // limit the time delay value from the GPS library to a max of 250 msec which is the max value the EKF has been tested for.
        maxTimeDelay_ms = MAX(maxTimeDelay_ms , MIN((uint16_t)(gps_delay_sec * 1000.0f),250));
    }

    // airspeed sensing can have large delays and should not be included if disabled
    if (dal.airspeed_sensor_enabled()) {
        maxTimeDelay_ms = MAX(maxTimeDelay_ms , frontend->tasDelay_ms);
    }

#if HAL_VISUALODOM_ENABLED
    // include delay from visual odometry if enabled
    const auto *visual_odom = dal.visualodom();
    if ((visual_odom != nullptr) && visual_odom->enabled()) {
        maxTimeDelay_ms = MAX(maxTimeDelay_ms, MIN(visual_odom->get_delay_ms(), 250));
    }
#endif

    // calculate the IMU buffer length required to accommodate the maximum delay with some allowance for jitter
    imu_buffer_length = (maxTimeDelay_ms / (uint16_t)(EKF_TARGET_DT_MS)) + 1;

    // set the observation buffer length to handle the minimum time of arrival between observations in combination
    // with the worst case delay from current time to ekf fusion time
    // allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
    uint16_t ekf_delay_ms = maxTimeDelay_ms + (int)(ceilF((ftype)maxTimeDelay_ms * 0.5f));
    obs_buffer_length = (ekf_delay_ms / frontend->sensorIntervalMin_ms) + 1;

    // limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
    obs_buffer_length = MIN(obs_buffer_length,imu_buffer_length);

    // calculate buffer size for optical flow data
    const uint8_t flow_buffer_length = MIN((ekf_delay_ms / frontend->flowIntervalMin_ms) + 1, imu_buffer_length);

    // calculate buffer size for external nav data
    const uint8_t extnav_buffer_length = MIN((ekf_delay_ms / frontend->extNavIntervalMin_ms) + 1, imu_buffer_length);

    if(!storedGPS.init(obs_buffer_length)) {
        return false;
    }
    if(!storedMag.init(obs_buffer_length)) {
        return false;
    }
    if(!storedBaro.init(obs_buffer_length)) {
        return false;
    }
    if(dal.airspeed() && !storedTAS.init(obs_buffer_length)) {
        return false;
    }
    if(dal.opticalflow_enabled() && !storedOF.init(flow_buffer_length)) {
        return false;
    }
#if EK3_FEATURE_BODY_ODOM
    if(frontend->sources.ext_nav_enabled() && !storedBodyOdm.init(obs_buffer_length)) {
        return false;
    }
    if(frontend->sources.wheel_encoder_enabled() && !storedWheelOdm.init(imu_buffer_length)) {
        // initialise to same length of IMU to allow for multiple wheel sensors
        return false;
    }
#endif // EK3_FEATURE_BODY_ODOM
    if(frontend->sources.gps_yaw_enabled() && !storedYawAng.init(obs_buffer_length)) {
        return false;
    }
    // Note: the use of dual range finders potentially doubles the amount of data to be stored
    if(dal.rangefinder() && !storedRange.init(MIN(2*obs_buffer_length , imu_buffer_length))) {
        return false;
    }
    // Note: range beacon data is read one beacon at a time and can arrive at a high rate
    if(dal.beacon() && !storedRangeBeacon.init(imu_buffer_length+1)) {
        return false;
    }
#if EK3_FEATURE_EXTERNAL_NAV
    if (frontend->sources.ext_nav_enabled() && !storedExtNav.init(extnav_buffer_length)) {
        return false;
    }
    if (frontend->sources.ext_nav_enabled() && !storedExtNavVel.init(extnav_buffer_length)) {
        return false;
    }
    if(frontend->sources.ext_nav_enabled() && !storedExtNavYawAng.init(extnav_buffer_length)) {
        return false;
    }
#endif // EK3_FEATURE_EXTERNAL_NAV
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }
#if EK3_FEATURE_DRAG_FUSION
    if (!storedDrag.init(obs_buffer_length)) {
        return false;
    }
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u buffs IMU=%u OBS=%u OF=%u EN:%u dt=%.4f",
                    (unsigned)imu_index,
                    (unsigned)imu_buffer_length,
                    (unsigned)obs_buffer_length,
                    (unsigned)flow_buffer_length,
                    (unsigned)extnav_buffer_length,
                    (double)dtEkfAvg);

    if ((yawEstimator == nullptr) && (frontend->_gsfRunMask & (1U<<core_index))) {
        // check if there is enough memory to create the EKF-GSF object
        if (dal.available_memory() < sizeof(EKFGSF_yaw) + 1024) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 IMU%u GSF: not enough memory",(unsigned)imu_index);
            return false;
        }

        // try to instantiate
        yawEstimator = new EKFGSF_yaw();
        if (yawEstimator == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 IMU%uGSF: allocation failed",(unsigned)imu_index);
            return false;
        }
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF3_core::InitialiseVariables()
{
    // calculate the nominal filter update rate
    const auto &ins = dal.ins();
    localFilterTimeStep_ms = (uint8_t)(1000*ins.get_loop_delta_t());
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms, (uint8_t)EKF_TARGET_DT_MS);

    // initialise time stamps
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaDragStep_ms = imuSampleTime_ms;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = 0;
    lastPosPassTime_ms = 0;
    lastHgtPassTime_ms = 0;
    lastTasPassTime_ms = 0;
    lastSynthYawTime_ms = 0;
    lastTimeGpsReceived_ms = 0;
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
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;
    lastPosResetD_ms = 0;
    lastRngMeasTime_ms = 0;

    // initialise other variables
    memset(&dvelBiasAxisInhibit, 0, sizeof(dvelBiasAxisInhibit));
	dvelBiasAxisVarPrev.zero();
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    tasTimeout = true;
    badIMUdata = false;
    vertVelVarClipCounter = 0;
    finalInflightYawInit = false;
    dtIMUavg = ins.get_loop_delta_t();
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&KH[0][0], 0, sizeof(KH));
    memset(&KHP[0][0], 0, sizeof(KHP));
    memset(&nextP[0][0], 0, sizeof(nextP));
    flowDataValid = false;
    rangeDataToFuse  = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    inhibitGndState = false;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    PV_AidingMode = AID_NONE;
    PV_AidingModePrev = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    inhibitWindStates = true;
    inhibitDelVelBiasStates = true;
    inhibitDelAngBiasStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    gpsSpdAccuracy = 0.0f;
    gpsPosAccuracy = 0.0f;
    gpsHgtAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    rngOnGnd = 0.05f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrorVariance = sq(M_2PI);
    tiltAlignComplete = false;
    yawAlignComplete = false;
    have_table_earth_field = false;
    stateIndexLim = 23;
    last_gps_idx = 0;
    delAngCorrection.zero();
    velErrintegral.zero();
    posErrintegral.zero();
    gpsGoodToAlign = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastInnovPassTime_ms = 0;
    lastInnovFailTime_ms = 0;
    gpsAccuracyGood = false;
    gpsloc_prev = {};
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    ZERO_FARRAY(statesArray);
    memset(&vertCompFiltState, 0, sizeof(vertCompFiltState));
    posVelFusionDelayed = false;
    optFlowFusionDelayed = false;
    flowFusionActive = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    airDataFusionWindOnly = false;
    posResetNE.zero();
    velResetNE.zero();
    posResetD = 0.0f;
    hgtInnovFiltState = 0.0f;
    imuDataDownSampledNew.delAng.zero();
    imuDataDownSampledNew.delVel.zero();
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    imuDataDownSampledNew.gyro_index = gyro_index_active;
    imuDataDownSampledNew.accel_index = accel_index_active;
    runUpdates = false;
    framesSincePredict = 0;
    gpsYawResetRequest = false;
    delAngBiasLearned = false;
    memset(&filterStatus, 0, sizeof(filterStatus));
    activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    prevHgtSource = activeHgtSource;
    memset(&rngMeasIndex, 0, sizeof(rngMeasIndex));
    memset(&storedRngMeasTime_ms, 0, sizeof(storedRngMeasTime_ms));
    memset(&storedRngMeas, 0, sizeof(storedRngMeas));
    terrainHgtStable = true;
    ekfOriginHgtVar = 0.0f;
    ekfGpsRefHgt = 0.0;
    velOffsetNED.zero();
    posOffsetNED.zero();
    ZERO_FARRAY(velPosObs);

    // range beacon fusion variables
    memset((void *)&rngBcnDataDelayed, 0, sizeof(rngBcnDataDelayed));
    lastRngBcnPassTime_ms = 0;
    rngBcnTestRatio = 0.0f;
    rngBcnHealth = false;
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
    bcnPosDownOffsetMax = 0.0f;
    bcnPosOffsetMaxVar = 0.0f;
    maxOffsetStateChangeFilt = 0.0f;
    bcnPosDownOffsetMin = 0.0f;
    bcnPosOffsetMinVar = 0.0f;
    minOffsetStateChangeFilt = 0.0f;
    rngBcnFuseDataReportIndex = 0;
    if (dal.beacon()) {
        if (rngBcnFusionReport == nullptr) {
            rngBcnFusionReport = new rngBcnFusionReport_t[dal.beacon()->count()];
        }
    }
    bcnPosOffsetNED.zero();
    bcnOriginEstInit = false;

#if EK3_FEATURE_BODY_ODOM
    // body frame displacement fusion
    memset((void *)&bodyOdmDataNew, 0, sizeof(bodyOdmDataNew));
    memset((void *)&bodyOdmDataDelayed, 0, sizeof(bodyOdmDataDelayed));
#endif
    lastbodyVelPassTime_ms = 0;
    ZERO_FARRAY(bodyVelTestRatio);
    ZERO_FARRAY(varInnovBodyVel);
    ZERO_FARRAY(innovBodyVel);
    prevBodyVelFuseTime_ms = 0;
    bodyOdmMeasTime_ms = 0;
    bodyVelFusionDelayed = false;
    bodyVelFusionActive = false;

    // yaw sensor fusion
    yawMeasTime_ms = 0;
    memset(&yawAngDataNew, 0, sizeof(yawAngDataNew));
    memset(&yawAngDataDelayed, 0, sizeof(yawAngDataDelayed));

#if EK3_FEATURE_EXTERNAL_NAV
    // external nav data fusion
    extNavDataDelayed = {};
    extNavMeasTime_ms = 0;
    extNavLastPosResetTime_ms = 0;
    extNavDataToFuse = false;
    extNavUsedForPos = false;
    extNavVelDelayed = {};
    extNavVelToFuse = false;
    useExtNavVel = false;
    extNavVelMeasTime_ms = 0;
#endif

    // zero data buffers
    storedIMU.reset();
    storedGPS.reset();
    storedBaro.reset();
    storedTAS.reset();
    storedRange.reset();
    storedOutput.reset();
    storedRangeBeacon.reset();
#if EK3_FEATURE_BODY_ODOM
    storedBodyOdm.reset();
    storedWheelOdm.reset();
#endif
#if EK3_FEATURE_EXTERNAL_NAV
    storedExtNav.reset();
    storedExtNavVel.reset();
#endif

    // initialise pre-arm message
    dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF3 still initialising");

    InitialiseVariablesMag();

    // emergency reset of yaw to EKFGSF estimate
    EKFGSF_yaw_reset_ms = 0;
    EKFGSF_yaw_reset_request_ms = 0;
    EKFGSF_yaw_reset_count = 0;
    EKFGSF_run_filterbank = false;
    EKFGSF_yaw_valid_count = 0;

    effectiveMagCal = effective_magCal();
}


// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF3_core::InitialiseVariablesMag()
{
    lastHealthyMagTime_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    magTimeout = false;
    allMagSensorsFailed = false;
    finalInflightMagInit = false;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    inhibitMagStates = true;
    magSelectIndex = dal.compass().get_first_usable();
    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;
    posDownAtLastMagReset = stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    quatAtLastMagReset = stateStruct.quat;
    magFieldLearned = false;
    storedMag.reset();
    storedYawAng.reset();
#if EK3_FEATURE_EXTERNAL_NAV
    storedExtNavYawAng.reset();
#endif
    needMagBodyVarReset = false;
    needEarthBodyVarReset = false;
}

/*
Initialise the states from accelerometer data. This assumes measured acceleration 
is dominated by gravity. If this assumption is not true then the EKF will require
timee to reduce the resulting tilt error. Yaw alignment is not performed by this
function, but is perfomred later and initiated the SelectMagFusion() function
after the tilt has stabilised.
*/

bool NavEKF3_core::InitialiseFilterBootstrap(void)
{
    // update sensor selection (for affinity)
    update_sensor_selection();

    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && dal.gps().status(preferred_gps) < AP_DAL_GPS::GPS_OK_FIX_3D) {
        dal.snprintf(prearm_fail_string,
                     sizeof(prearm_fail_string),
                     "EKF3 init failure: No GPS lock");
        statesInitialised = false;
        return false;
    }

    // read all the sensors required to start the EKF the states
    readIMUData();
    readMagData();
    readGpsData();
    readBaroData();

    if (statesInitialised) {
        // we are initialised, but we don't return true until the IMU
        // buffer has been filled. This prevents a timing
        // vulnerability with a pause in IMU data during filter startup
        return storedIMU.is_filled();
    }

    // accumulate enough sensor data to fill the buffers
    if (firstInitTime_ms == 0) {
        firstInitTime_ms = imuSampleTime_ms;
        return false;
    } else if (imuSampleTime_ms - firstInitTime_ms < 1000) {
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3F initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = dal.ins().get_accel(accel_index_active).toftype();

    // normalise the acceleration vector
    ftype pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinF(initAccVec.x);

        // calculate initial roll angle
        roll = atan2F(-initAccVec.y , -initAccVec.z);
    }

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise dynamic states
    stateStruct.velocity.zero();
    stateStruct.position.zero();

    // initialise static process model states
    stateStruct.gyro_bias.zero();
    stateStruct.accel_bias.zero();
    stateStruct.wind_vel.zero();
    stateStruct.earth_magfield.zero();
    stateStruct.body_magfield.zero();

    // set the position, velocity and height
    ResetVelocity(resetDataSource::DEFAULT);
    ResetPosition(resetDataSource::DEFAULT);
    ResetHeight();

    // initialise sources
    posxy_source_last = frontend->sources.getPosXYSource();
    yaw_source_last = frontend->sources.getYawSource();

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, dal.get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset the output predictor states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // reset inactive biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        inactiveBias[i].gyro_bias.zero();
        inactiveBias[i].accel_bias.zero();
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initialised",(unsigned)imu_index);

    // we initially return false to wait for the IMU buffer to fill
    return false;
}

// initialise the covariance matrix
void NavEKF3_core::CovarianceInit()
{
    // zero the matrix
    memset(&P[0][0], 0, sizeof(P));

    // define the initial angle uncertainty as variances for a rotation vector
    Vector3F rot_vec_var;
    rot_vec_var.x = rot_vec_var.y = rot_vec_var.z = sq(0.1f);

    // reset the quaternion state covariances
    CovariancePrediction(&rot_vec_var);

    // velocities
    P[4][4]   = sq(frontend->_gpsHorizVelNoise);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(frontend->_gpsVertVelNoise);
    // positions
    P[7][7]   = sq(frontend->_gpsHorizPosNoise);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(frontend->_baroAltNoise);
    // gyro delta angle biases
    P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // delta velocity biases
    P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim * dtEkfAvg);
    P[14][14] = P[13][13];
    P[15][15] = P[13][13];
    // earth magnetic field
    P[16][16] = sq(frontend->_magNoise);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = sq(frontend->_magNoise);
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
void NavEKF3_core::UpdateFilter(bool predict)
{
    // Set the flag to indicate to the filter that the front-end has given permission for a new state prediction cycle to be started
    startPredictEnabled = predict;

    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    fill_scratch_variables();

    // update sensor selection (for affinity)
    update_sensor_selection();

    // TODO - in-flight restart method

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData();

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    if (runUpdates) {
        // Predict states using IMU data from the delayed time horizon
        UpdateStrapdownEquationsNED();

        // Predict the covariance growth
        CovariancePrediction(nullptr);

        // Run the IMU prediction step for the GSF yaw estimator algorithm
        // using IMU and optionally true airspeed data.
        // Must be run before SelectMagFusion() to provide an up to date yaw estimate
        runYawEstimatorPrediction();

        // Update states using  magnetometer or external yaw sensor data
        SelectMagFusion();

        // Update states using GPS and altimeter data
        SelectVelPosFusion();

        // Run the GPS velocity correction step for the GSF yaw estimator algorithm
        // and use the yaw estimate to reset the main EKF yaw if requested
        // Muat be run after SelectVelPosFusion() so that fresh GPS data is available
        runYawEstimatorCorrection();

        // Update states using range beacon data
        SelectRngBcnFusion();

        // Update states using optical flow data
        SelectFlowFusion();

#if EK3_FEATURE_BODY_ODOM
        // Update states using body frame odometry data
        SelectBodyOdomFusion();
#endif

        // Update states using airspeed data
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles or body drag for multicopters
        SelectBetaDragFusion();

        // Update the filter status
        updateFilterStatus();

        if (imuSampleTime_ms - last_oneHz_ms >= 1000) {
            // 1Hz tasks
            last_oneHz_ms = imuSampleTime_ms;
            moveEKFOrigin();
            checkUpdateEarthField();
        }
    }

    // Wind output forward from the fusion to output time horizon
    calcOutputStates();

    /*
      this is a check to cope with a vehicle sitting idle on the
      ground and getting over-confident of the state. The symptoms
      would be "gyros still settling" when the user tries to arm. In
      that state the EKF can't recover, so we do a hard reset and let
      it try again.
     */
    if (filterStatus.value != 0) {
        last_filter_ok_ms = dal.millis();
    }
    if (filterStatus.value == 0 &&
        last_filter_ok_ms != 0 &&
        dal.millis() - last_filter_ok_ms > 5000 &&
        !dal.get_armed()) {
        // we've been unhealthy for 5 seconds after being healthy, reset the filter
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF3 IMU%u forced reset",(unsigned)imu_index);
        last_filter_ok_ms = 0;
        statesInitialised = false;
        InitialiseFilterBootstrap();
    }
}

void NavEKF3_core::correctDeltaAngle(Vector3F &delAng, ftype delAngDT, uint8_t gyro_index)
{
    delAng -= inactiveBias[gyro_index].gyro_bias * (delAngDT / dtEkfAvg);
}

void NavEKF3_core::correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index)
{
    delVel -= inactiveBias[accel_index].accel_bias * (delVelDT / dtEkfAvg);
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observation is fused. This attitude error is then used to correct
 * the quaternion.
*/
void NavEKF3_core::UpdateStrapdownEquationsNED()
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
    Vector3F delVelNav;  // delta velocity vector in earth axes
    delVelNav  = prevTnb.mul_transpose(delVelCorrected);
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the nav to body cosine matrix
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
        ftype gain = 5.0f/accNavMagHoriz;
        delVelNav.x *= gain;
        delVelNav.y *= gain;
    }

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3F lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF += delAngCorrected;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();

    // If main filter velocity states are valid, update the range beacon receiver position states
    if (filterStatus.flags.horiz_vel) {
        receiverPos += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);
    }
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
void NavEKF3_core::calcOutputStates()
{
    // apply corrections to the IMU data
    Vector3F delAngNewCorrected = imuDataNew.delAng;
    Vector3F delVelNewCorrected = imuDataNew.delVel;
    correctDeltaAngle(delAngNewCorrected, imuDataNew.delAngDT, imuDataNew.gyro_index);
    correctDeltaVelocity(delVelNewCorrected, imuDataNew.delVelDT, imuDataNew.accel_index);

    // apply corrections to track EKF solution
    Vector3F delAng = delAngNewCorrected + delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    QuaternionF deltaQuat;
    deltaQuat.from_axis_angle(delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    outputDataNew.quat *= deltaQuat;
    outputDataNew.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3F Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    Vector3F delVelNav  = Tbn_temp*delVelNewCorrected;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3F lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // Implement third order complementary filter for height and height rate
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    // Perform filter calculation using backwards Euler integration
    // Coefficients selected to place all three filter poles at omega
    const ftype CompFiltOmega = M_2PI * constrain_ftype(frontend->_hrt_filt_freq, 0.1f, 30.0f);
    ftype omega2 = CompFiltOmega * CompFiltOmega;
    ftype pos_err = constrain_ftype(outputDataNew.position.z - vertCompFiltState.pos, -1e5, 1e5);
    ftype integ1_input = pos_err * omega2 * CompFiltOmega * imuDataNew.delVelDT;
    vertCompFiltState.acc += integ1_input;
    ftype integ2_input = delVelNav.z + (vertCompFiltState.acc + pos_err * omega2 * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.vel += integ2_input;
    ftype integ3_input = (vertCompFiltState.vel + pos_err * CompFiltOmega * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.pos += integ3_input; 

    // apply a trapezoidal integration to velocities to calculate position
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f);

    // If the IMU accelerometer is offset from the body frame origin, then calculate corrections
    // that can be added to the EKF velocity and position outputs so that they represent the velocity
    // and position of the body frame origin.
    // Note the * operator has been overloaded to operate as a dot product
    if (!accelPosOffset.is_zero()) {
        // calculate the average angular rate across the last IMU update
        // note delAngDT is prevented from being zero in readIMUData()
        Vector3F angRate = imuDataNew.delAng * (1.0f/imuDataNew.delAngDT);

        // Calculate the velocity of the body frame origin relative to the IMU in body frame
        // and rotate into earth frame. Note % operator has been overloaded to perform a cross product
        Vector3F velBodyRelIMU = angRate % (- accelPosOffset);
        velOffsetNED = Tbn_temp * velBodyRelIMU;

        // calculate the earth frame position of the body frame origin relative to the IMU
        posOffsetNED = Tbn_temp * (- accelPosOffset);
    } else {
        velOffsetNED.zero();
        posOffsetNED.zero();
    }

    // Detect fixed wing launch acceleration using latest data from IMU to enable early startup of filter functions
    // that use launch acceleration to detect start of flight
    if (!inFlight && !dal.get_takeoff_expected() && assume_zero_sideslip()) {
        const ftype launchDelVel = imuDataNew.delVel.x + GRAVITY_MSS * imuDataNew.delVelDT * Tbn_temp.c.x;
        if (launchDelVel > GRAVITY_MSS * imuDataNew.delVelDT) {
            dal.set_takeoff_expected();
        }
    }

    // store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
    if (runUpdates) {
        // store the states at the output time horizon
        storedOutput[storedIMU.get_youngest_index()] = outputDataNew;

        // recall the states from the fusion time horizon
        outputDataDelayed = storedOutput[storedIMU.get_oldest_index()];

        // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction

        // divide the demanded quaternion by the estimated to get the error
        QuaternionF quatErr = stateStruct.quat / outputDataDelayed.quat;

        // Convert to a delta rotation using a small angle approximation
        quatErr.normalize();
        Vector3F deltaAngErr;
        ftype scaler;
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
        ftype timeDelay = 1e-3f * (ftype)(imuDataNew.time_ms - imuDataDelayed.time_ms);
        timeDelay = MAX(timeDelay, dtIMUavg);
        ftype errorGain = 0.5f / timeDelay;

        // calculate a correction to the delta angle
        // that will cause the INS to track the EKF quaternions
        delAngCorrection = deltaAngErr * errorGain * dtIMUavg;

        // calculate velocity and position tracking errors
        Vector3F velErr = (stateStruct.velocity - outputDataDelayed.velocity);
        Vector3F posErr = (stateStruct.position - outputDataDelayed.position);

        // collect magnitude tracking error for diagnostics
        outputTrackError.x = deltaAngErr.length();
        outputTrackError.y = velErr.length();
        outputTrackError.z = posErr.length();

        // convert user specified time constant from centi-seconds to seconds
        ftype tauPosVel = constrain_ftype(0.01f*(ftype)frontend->_tauVelPosOutput, 0.1f, 0.5f);

        // calculate a gain to track the EKF position states with the specified time constant
        ftype velPosGain = dtEkfAvg / constrain_ftype(tauPosVel, dtEkfAvg, 10.0f);

        // use a PI feedback to calculate a correction that will be applied to the output state history
        posErrintegral += posErr;
        velErrintegral += velErr;
        Vector3F velCorrection = velErr * velPosGain + velErrintegral * sq(velPosGain) * 0.1f;
        Vector3F posCorrection = posErr * velPosGain + posErrintegral * sq(velPosGain) * 0.1f;

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
 * Calculate the predicted state covariance matrix using algebraic equations generated using SymPy
 * See AP_NavEKF3/derivation/main.py for derivation
 * Output for change reference: AP_NavEKF3/derivation/generated/covariance_generated.cpp
 * Argument rotVarVecPtr is pointer to a vector defining the earth frame uncertainty variance of the quaternion states
 * used to perform a reset of the quaternion state covariances only. Set to null for normal operation.
*/
void NavEKF3_core::CovariancePrediction(Vector3F *rotVarVecPtr)
{
    ftype daxVar;       // X axis delta angle noise variance rad^2
    ftype dayVar;       // Y axis delta angle noise variance rad^2
    ftype dazVar;       // Z axis delta angle noise variance rad^2
    ftype dvxVar;       // X axis delta velocity variance noise (m/s)^2
    ftype dvyVar;       // Y axis delta velocity variance noise (m/s)^2
    ftype dvzVar;       // Z axis delta velocity variance noise (m/s)^2
    ftype dvx;          // X axis delta velocity (m/s)
    ftype dvy;          // Y axis delta velocity (m/s)
    ftype dvz;          // Z axis delta velocity (m/s)
    ftype dax;          // X axis delta angle (rad)
    ftype day;          // Y axis delta angle (rad)
    ftype daz;          // Z axis delta angle (rad)
    ftype q0;           // attitude quaternion
    ftype q1;           // attitude quaternion
    ftype q2;           // attitude quaternion
    ftype q3;           // attitude quaternion
    ftype dax_b;        // X axis delta angle measurement bias (rad)
    ftype day_b;        // Y axis delta angle measurement bias (rad)
    ftype daz_b;        // Z axis delta angle measurement bias (rad)
    ftype dvx_b;        // X axis delta velocity measurement bias (rad)
    ftype dvy_b;        // Y axis delta velocity measurement bias (rad)
    ftype dvz_b;        // Z axis delta velocity measurement bias (rad)

    // Calculate the time step used by the covariance prediction as an average of the gyro and accel integration period
    // Constrain to prevent bad timing jitter causing numerical conditioning problems with the covariance prediction
    dt = constrain_ftype(0.5f*(imuDataDelayed.delAngDT+imuDataDelayed.delVelDT),0.5f * dtEkfAvg, 2.0f * dtEkfAvg);

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.Filter height rate using a 10 second time constant filter
    ftype alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // calculate covariance prediction process noise added to diagonals of predicted covariance matrix
    // error growth of first 10 kinematic states is built into auto-code for covariance prediction and driven by IMU noise parameters
    Vector14 processNoiseVariance = {};

    if (!inhibitDelAngBiasStates) {
        ftype dAngBiasVar = sq(sq(dt) * constrain_ftype(frontend->_gyroBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=0; i<=2; i++) processNoiseVariance[i] = dAngBiasVar;
    }

    if (!inhibitDelVelBiasStates) {
        // default process noise (m/s)^2
        ftype dVelBiasVar = sq(sq(dt) * constrain_ftype(frontend->_accelBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=3; i<=5; i++) {
            processNoiseVariance[i] = dVelBiasVar;
        }
    }

    if (!inhibitMagStates && lastInhibitMagStates) {
        // when starting 3D fusion we want to reset mag variances
        needMagBodyVarReset = true;
        needEarthBodyVarReset = true;
    }

    if (needMagBodyVarReset) {
        // reset body mag variances
        needMagBodyVarReset = false;
        zeroCols(P,19,21);
        zeroRows(P,19,21);
        P[19][19] = sq(frontend->_magNoise);
        P[20][20] = P[19][19];
        P[21][21] = P[19][19];
    }

    if (needEarthBodyVarReset) {
        // reset mag earth field variances
        needEarthBodyVarReset = false;
        zeroCols(P,16,18);
        zeroRows(P,16,18);
        P[16][16] = sq(frontend->_magNoise);
        P[17][17] = P[16][16];
        P[18][18] = P[16][16];
        // Fusing the declinaton angle as an observaton with a 20 deg uncertainty helps
        // to stabilise the earth field.
        FuseDeclination(radians(20.0f));
    }

    if (!inhibitMagStates) {
        ftype magEarthVar = sq(dt * constrain_ftype(frontend->_magEarthProcessNoise, 0.0f, 1.0f));
        ftype magBodyVar  = sq(dt * constrain_ftype(frontend->_magBodyProcessNoise, 0.0f, 1.0f));
        for (uint8_t i=6; i<=8; i++) processNoiseVariance[i] = magEarthVar;
        for (uint8_t i=9; i<=11; i++) processNoiseVariance[i] = magBodyVar;
    }
    lastInhibitMagStates = inhibitMagStates;

    if (!inhibitWindStates) {
        ftype windVelVar  = sq(dt * constrain_ftype(frontend->_windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrain_ftype(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsF(hgtRate)));
        for (uint8_t i=12; i<=13; i++) processNoiseVariance[i] = windVelVar;
    }

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
    dvx_b = stateStruct.accel_bias.x;
    dvy_b = stateStruct.accel_bias.y;
    dvz_b = stateStruct.accel_bias.z;

    bool quatCovResetOnly = false;
    if (rotVarVecPtr != nullptr) {
        // Handle special case where we are initialising the quaternion covariances using an earth frame
        // vector defining the variance of the angular alignment uncertainty. Convert he varaince vector
        // to a matrix and rotate into body frame. Use the exisiting gyro error propagation mechanism to
        // propagate the body frame angular uncertainty variances.
        const Vector3F &rotVarVec = *rotVarVecPtr;
        Matrix3F R_ef = Matrix3F (
            rotVarVec.x, 0.0f, 0.0f,
            0.0f, rotVarVec.y, 0.0f,
            0.0f, 0.0f, rotVarVec.z);
        Matrix3F Tnb;
        stateStruct.quat.inverse().rotation_matrix(Tnb);
        Matrix3F R_bf = Tnb * R_ef * Tnb.transposed();
        daxVar = R_bf.a.x;
        dayVar = R_bf.b.y;
        dazVar = R_bf.c.z;
        quatCovResetOnly = true;
        zeroRows(P,0,3);
        zeroCols(P,0,3);
    } else {
        ftype _gyrNoise = constrain_ftype(frontend->_gyrNoise, 0.0f, 1.0f);
        daxVar = dayVar = dazVar = sq(dt*_gyrNoise);
    }
    ftype _accNoise = constrain_ftype(frontend->_accNoise, 0.0f, 10.0f);
    dvxVar = dvyVar = dvzVar = sq(dt*_accNoise);

    if (!inhibitDelVelBiasStates) {
        for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
            const uint8_t index = stateIndex - 13;

            // Don't attempt learning of IMU delta velocty bias if on ground and not aligned with the gravity vector
            const bool is_bias_observable = (fabsF(prevTnb[index][2]) > 0.8f) && onGround;

            if (!is_bias_observable && !dvelBiasAxisInhibit[index]) {
                // store variances to be reinstated wben learning can commence later
                dvelBiasAxisVarPrev[index] = P[stateIndex][stateIndex];
                dvelBiasAxisInhibit[index] = true;
            } else if (is_bias_observable && dvelBiasAxisInhibit[index]) {
                P[stateIndex][stateIndex] = dvelBiasAxisVarPrev[index];
                dvelBiasAxisInhibit[index] = false;
            }
        }
    }

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the lower diagonal and copy to take advantage of symmetry

    // intermediate calculations
    const ftype PS0 = sq(q1);
    const ftype PS1 = 0.25F*daxVar;
    const ftype PS2 = sq(q2);
    const ftype PS3 = 0.25F*dayVar;
    const ftype PS4 = sq(q3);
    const ftype PS5 = 0.25F*dazVar;
    const ftype PS6 = 0.5F*q1;
    const ftype PS7 = 0.5F*q2;
    const ftype PS8 = PS7*P[10][11];
    const ftype PS9 = 0.5F*q3;
    const ftype PS10 = PS9*P[10][12];
    const ftype PS11 = 0.5F*dax - 0.5F*dax_b;
    const ftype PS12 = 0.5F*day - 0.5F*day_b;
    const ftype PS13 = 0.5F*daz - 0.5F*daz_b;
    const ftype PS14 = PS10 - PS11*P[1][10] - PS12*P[2][10] - PS13*P[3][10] + PS6*P[10][10] + PS8 + P[0][10];
    const ftype PS15 = PS6*P[10][11];
    const ftype PS16 = PS9*P[11][12];
    const ftype PS17 = -PS11*P[1][11] - PS12*P[2][11] - PS13*P[3][11] + PS15 + PS16 + PS7*P[11][11] + P[0][11];
    const ftype PS18 = PS6*P[10][12];
    const ftype PS19 = PS7*P[11][12];
    const ftype PS20 = -PS11*P[1][12] - PS12*P[2][12] - PS13*P[3][12] + PS18 + PS19 + PS9*P[12][12] + P[0][12];
    const ftype PS21 = PS12*P[1][2];
    const ftype PS22 = -PS13*P[1][3];
    const ftype PS23 = -PS11*P[1][1] - PS21 + PS22 + PS6*P[1][10] + PS7*P[1][11] + PS9*P[1][12] + P[0][1];
    const ftype PS24 = -PS11*P[1][2];
    const ftype PS25 = PS13*P[2][3];
    const ftype PS26 = -PS12*P[2][2] + PS24 - PS25 + PS6*P[2][10] + PS7*P[2][11] + PS9*P[2][12] + P[0][2];
    const ftype PS27 = PS11*P[1][3];
    const ftype PS28 = -PS12*P[2][3];
    const ftype PS29 = -PS13*P[3][3] - PS27 + PS28 + PS6*P[3][10] + PS7*P[3][11] + PS9*P[3][12] + P[0][3];
    const ftype PS30 = PS11*P[0][1];
    const ftype PS31 = PS12*P[0][2];
    const ftype PS32 = PS13*P[0][3];
    const ftype PS33 = -PS30 - PS31 - PS32 + PS6*P[0][10] + PS7*P[0][11] + PS9*P[0][12] + P[0][0];
    const ftype PS34 = 0.5F*q0;
    const ftype PS35 = q2*q3;
    const ftype PS36 = q0*q1;
    const ftype PS37 = q1*q3;
    const ftype PS38 = q0*q2;
    const ftype PS39 = q1*q2;
    const ftype PS40 = q0*q3;
    const ftype PS41 = -PS2;
    const ftype PS42 = sq(q0);
    const ftype PS43 = -PS4 + PS42;
    const ftype PS44 = PS0 + PS41 + PS43;
    const ftype PS45 = -PS11*P[1][13] - PS12*P[2][13] - PS13*P[3][13] + PS6*P[10][13] + PS7*P[11][13] + PS9*P[12][13] + P[0][13];
    const ftype PS46 = PS37 + PS38;
    const ftype PS47 = -PS11*P[1][15] - PS12*P[2][15] - PS13*P[3][15] + PS6*P[10][15] + PS7*P[11][15] + PS9*P[12][15] + P[0][15];
    const ftype PS48 = 2*PS47;
    const ftype PS49 = dvy - dvy_b;
    const ftype PS50 = dvx - dvx_b;
    const ftype PS51 = dvz - dvz_b;
    const ftype PS52 = PS49*q0 + PS50*q3 - PS51*q1;
    const ftype PS53 = 2*PS29;
    const ftype PS54 = -PS39 + PS40;
    const ftype PS55 = -PS11*P[1][14] - PS12*P[2][14] - PS13*P[3][14] + PS6*P[10][14] + PS7*P[11][14] + PS9*P[12][14] + P[0][14];
    const ftype PS56 = 2*PS55;
    const ftype PS57 = -PS49*q3 + PS50*q0 + PS51*q2;
    const ftype PS58 = 2*PS33;
    const ftype PS59 = PS49*q1 - PS50*q2 + PS51*q0;
    const ftype PS60 = 2*PS59;
    const ftype PS61 = PS49*q2 + PS50*q1 + PS51*q3;
    const ftype PS62 = 2*PS61;
    const ftype PS63 = -PS11*P[1][4] - PS12*P[2][4] - PS13*P[3][4] + PS6*P[4][10] + PS7*P[4][11] + PS9*P[4][12] + P[0][4];
    const ftype PS64 = -PS0;
    const ftype PS65 = PS2 + PS43 + PS64;
    const ftype PS66 = PS39 + PS40;
    const ftype PS67 = 2*PS45;
    const ftype PS68 = -PS35 + PS36;
    const ftype PS69 = -PS11*P[1][5] - PS12*P[2][5] - PS13*P[3][5] + PS6*P[5][10] + PS7*P[5][11] + PS9*P[5][12] + P[0][5];
    const ftype PS70 = PS4 + PS41 + PS42 + PS64;
    const ftype PS71 = PS35 + PS36;
    const ftype PS72 = 2*PS57;
    const ftype PS73 = -PS37 + PS38;
    const ftype PS74 = 2*PS52;
    const ftype PS75 = -PS11*P[1][6] - PS12*P[2][6] - PS13*P[3][6] + PS6*P[6][10] + PS7*P[6][11] + PS9*P[6][12] + P[0][6];
    const ftype PS76 = -PS34*P[10][11];
    const ftype PS77 = PS11*P[0][11] - PS12*P[3][11] + PS13*P[2][11] - PS19 + PS76 + PS9*P[11][11] + P[1][11];
    const ftype PS78 = PS13*P[0][2];
    const ftype PS79 = PS12*P[0][3];
    const ftype PS80 = PS11*P[0][0] - PS34*P[0][10] - PS7*P[0][12] + PS78 - PS79 + PS9*P[0][11] + P[0][1];
    const ftype PS81 = PS11*P[0][2];
    const ftype PS82 = PS13*P[2][2] + PS28 - PS34*P[2][10] - PS7*P[2][12] + PS81 + PS9*P[2][11] + P[1][2];
    const ftype PS83 = PS9*P[10][11];
    const ftype PS84 = PS7*P[10][12];
    const ftype PS85 = PS11*P[0][10] - PS12*P[3][10] + PS13*P[2][10] - PS34*P[10][10] + PS83 - PS84 + P[1][10];
    const ftype PS86 = -PS34*P[10][12];
    const ftype PS87 = PS11*P[0][12] - PS12*P[3][12] + PS13*P[2][12] + PS16 - PS7*P[12][12] + PS86 + P[1][12];
    const ftype PS88 = PS11*P[0][3];
    const ftype PS89 = -PS12*P[3][3] + PS25 - PS34*P[3][10] - PS7*P[3][12] + PS88 + PS9*P[3][11] + P[1][3];
    const ftype PS90 = PS13*P[1][2];
    const ftype PS91 = PS12*P[1][3];
    const ftype PS92 = PS30 - PS34*P[1][10] - PS7*P[1][12] + PS9*P[1][11] + PS90 - PS91 + P[1][1];
    const ftype PS93 = PS11*P[0][13] - PS12*P[3][13] + PS13*P[2][13] - PS34*P[10][13] - PS7*P[12][13] + PS9*P[11][13] + P[1][13];
    const ftype PS94 = PS11*P[0][15] - PS12*P[3][15] + PS13*P[2][15] - PS34*P[10][15] - PS7*P[12][15] + PS9*P[11][15] + P[1][15];
    const ftype PS95 = 2*PS94;
    const ftype PS96 = PS11*P[0][14] - PS12*P[3][14] + PS13*P[2][14] - PS34*P[10][14] - PS7*P[12][14] + PS9*P[11][14] + P[1][14];
    const ftype PS97 = 2*PS96;
    const ftype PS98 = PS11*P[0][4] - PS12*P[3][4] + PS13*P[2][4] - PS34*P[4][10] - PS7*P[4][12] + PS9*P[4][11] + P[1][4];
    const ftype PS99 = 2*PS93;
    const ftype PS100 = PS11*P[0][5] - PS12*P[3][5] + PS13*P[2][5] - PS34*P[5][10] - PS7*P[5][12] + PS9*P[5][11] + P[1][5];
    const ftype PS101 = PS11*P[0][6] - PS12*P[3][6] + PS13*P[2][6] - PS34*P[6][10] - PS7*P[6][12] + PS9*P[6][11] + P[1][6];
    const ftype PS102 = -PS34*P[11][12];
    const ftype PS103 = -PS10 + PS102 + PS11*P[3][12] + PS12*P[0][12] - PS13*P[1][12] + PS6*P[12][12] + P[2][12];
    const ftype PS104 = PS11*P[3][3] + PS22 - PS34*P[3][11] + PS6*P[3][12] + PS79 - PS9*P[3][10] + P[2][3];
    const ftype PS105 = PS13*P[0][1];
    const ftype PS106 = -PS105 + PS12*P[0][0] - PS34*P[0][11] + PS6*P[0][12] + PS88 - PS9*P[0][10] + P[0][2];
    const ftype PS107 = PS6*P[11][12];
    const ftype PS108 = PS107 + PS11*P[3][11] + PS12*P[0][11] - PS13*P[1][11] - PS34*P[11][11] - PS83 + P[2][11];
    const ftype PS109 = PS11*P[3][10] + PS12*P[0][10] - PS13*P[1][10] + PS18 + PS76 - PS9*P[10][10] + P[2][10];
    const ftype PS110 = PS12*P[0][1];
    const ftype PS111 = PS110 - PS13*P[1][1] + PS27 - PS34*P[1][11] + PS6*P[1][12] - PS9*P[1][10] + P[1][2];
    const ftype PS112 = PS11*P[2][3];
    const ftype PS113 = PS112 + PS31 - PS34*P[2][11] + PS6*P[2][12] - PS9*P[2][10] - PS90 + P[2][2];
    const ftype PS114 = PS11*P[3][13] + PS12*P[0][13] - PS13*P[1][13] - PS34*P[11][13] + PS6*P[12][13] - PS9*P[10][13] + P[2][13];
    const ftype PS115 = PS11*P[3][15] + PS12*P[0][15] - PS13*P[1][15] - PS34*P[11][15] + PS6*P[12][15] - PS9*P[10][15] + P[2][15];
    const ftype PS116 = 2*PS115;
    const ftype PS117 = PS11*P[3][14] + PS12*P[0][14] - PS13*P[1][14] - PS34*P[11][14] + PS6*P[12][14] - PS9*P[10][14] + P[2][14];
    const ftype PS118 = 2*PS117;
    const ftype PS119 = PS11*P[3][4] + PS12*P[0][4] - PS13*P[1][4] - PS34*P[4][11] + PS6*P[4][12] - PS9*P[4][10] + P[2][4];
    const ftype PS120 = 2*PS114;
    const ftype PS121 = PS11*P[3][5] + PS12*P[0][5] - PS13*P[1][5] - PS34*P[5][11] + PS6*P[5][12] - PS9*P[5][10] + P[2][5];
    const ftype PS122 = PS11*P[3][6] + PS12*P[0][6] - PS13*P[1][6] - PS34*P[6][11] + PS6*P[6][12] - PS9*P[6][10] + P[2][6];
    const ftype PS123 = -PS11*P[2][10] + PS12*P[1][10] + PS13*P[0][10] - PS15 + PS7*P[10][10] + PS86 + P[3][10];
    const ftype PS124 = PS105 + PS12*P[1][1] + PS24 - PS34*P[1][12] - PS6*P[1][11] + PS7*P[1][10] + P[1][3];
    const ftype PS125 = PS110 + PS13*P[0][0] - PS34*P[0][12] - PS6*P[0][11] + PS7*P[0][10] - PS81 + P[0][3];
    const ftype PS126 = -PS107 - PS11*P[2][12] + PS12*P[1][12] + PS13*P[0][12] - PS34*P[12][12] + PS84 + P[3][12];
    const ftype PS127 = PS102 - PS11*P[2][11] + PS12*P[1][11] + PS13*P[0][11] - PS6*P[11][11] + PS8 + P[3][11];
    const ftype PS128 = -PS11*P[2][2] + PS21 - PS34*P[2][12] - PS6*P[2][11] + PS7*P[2][10] + PS78 + P[2][3];
    const ftype PS129 = -PS112 + PS32 - PS34*P[3][12] - PS6*P[3][11] + PS7*P[3][10] + PS91 + P[3][3];
    const ftype PS130 = -PS11*P[2][13] + PS12*P[1][13] + PS13*P[0][13] - PS34*P[12][13] - PS6*P[11][13] + PS7*P[10][13] + P[3][13];
    const ftype PS131 = -PS11*P[2][15] + PS12*P[1][15] + PS13*P[0][15] - PS34*P[12][15] - PS6*P[11][15] + PS7*P[10][15] + P[3][15];
    const ftype PS132 = 2*PS131;
    const ftype PS133 = -PS11*P[2][14] + PS12*P[1][14] + PS13*P[0][14] - PS34*P[12][14] - PS6*P[11][14] + PS7*P[10][14] + P[3][14];
    const ftype PS134 = 2*PS133;
    const ftype PS135 = -PS11*P[2][4] + PS12*P[1][4] + PS13*P[0][4] - PS34*P[4][12] - PS6*P[4][11] + PS7*P[4][10] + P[3][4];
    const ftype PS136 = 2*PS130;
    const ftype PS137 = -PS11*P[2][5] + PS12*P[1][5] + PS13*P[0][5] - PS34*P[5][12] - PS6*P[5][11] + PS7*P[5][10] + P[3][5];
    const ftype PS138 = -PS11*P[2][6] + PS12*P[1][6] + PS13*P[0][6] - PS34*P[6][12] - PS6*P[6][11] + PS7*P[6][10] + P[3][6];
    const ftype PS139 = 2*PS46;
    const ftype PS140 = 2*PS54;
    const ftype PS141 = -PS139*P[13][15] + PS140*P[13][14] - PS44*P[13][13] + PS60*P[2][13] + PS62*P[1][13] + PS72*P[0][13] - PS74*P[3][13] + P[4][13];
    const ftype PS142 = -PS139*P[15][15] + PS140*P[14][15] - PS44*P[13][15] + PS60*P[2][15] + PS62*P[1][15] + PS72*P[0][15] - PS74*P[3][15] + P[4][15];
    const ftype PS143 = PS62*P[1][3];
    const ftype PS144 = PS72*P[0][3];
    const ftype PS145 = -PS139*P[3][15] + PS140*P[3][14] + PS143 + PS144 - PS44*P[3][13] + PS60*P[2][3] - PS74*P[3][3] + P[3][4];
    const ftype PS146 = -PS139*P[14][15] + PS140*P[14][14] - PS44*P[13][14] + PS60*P[2][14] + PS62*P[1][14] + PS72*P[0][14] - PS74*P[3][14] + P[4][14];
    const ftype PS147 = PS60*P[0][2];
    const ftype PS148 = PS74*P[0][3];
    const ftype PS149 = -PS139*P[0][15] + PS140*P[0][14] + PS147 - PS148 - PS44*P[0][13] + PS62*P[0][1] + PS72*P[0][0] + P[0][4];
    const ftype PS150 = PS62*P[1][2];
    const ftype PS151 = PS72*P[0][2];
    const ftype PS152 = -PS139*P[2][15] + PS140*P[2][14] + PS150 + PS151 - PS44*P[2][13] + PS60*P[2][2] - PS74*P[2][3] + P[2][4];
    const ftype PS153 = PS60*P[1][2];
    const ftype PS154 = PS74*P[1][3];
    const ftype PS155 = -PS139*P[1][15] + PS140*P[1][14] + PS153 - PS154 - PS44*P[1][13] + PS62*P[1][1] + PS72*P[0][1] + P[1][4];
    const ftype PS156 = 4*dvyVar;
    const ftype PS157 = 4*dvzVar;
    const ftype PS158 = -PS139*P[4][15] + PS140*P[4][14] - PS44*P[4][13] + PS60*P[2][4] + PS62*P[1][4] + PS72*P[0][4] - PS74*P[3][4] + P[4][4];
    const ftype PS159 = 2*PS141;
    const ftype PS160 = 2*PS68;
    const ftype PS161 = PS65*dvyVar;
    const ftype PS162 = 2*PS66;
    const ftype PS163 = PS44*dvxVar;
    const ftype PS164 = -PS139*P[5][15] + PS140*P[5][14] - PS44*P[5][13] + PS60*P[2][5] + PS62*P[1][5] + PS72*P[0][5] - PS74*P[3][5] + P[4][5];
    const ftype PS165 = 2*PS71;
    const ftype PS166 = 2*PS73;
    const ftype PS167 = PS70*dvzVar;
    const ftype PS168 = -PS139*P[6][15] + PS140*P[6][14] - PS44*P[6][13] + PS60*P[2][6] + PS62*P[1][6] + PS72*P[0][6] - PS74*P[3][6] + P[4][6];
    const ftype PS169 = PS160*P[14][15] - PS162*P[13][14] - PS60*P[1][14] + PS62*P[2][14] - PS65*P[14][14] + PS72*P[3][14] + PS74*P[0][14] + P[5][14];
    const ftype PS170 = PS160*P[13][15] - PS162*P[13][13] - PS60*P[1][13] + PS62*P[2][13] - PS65*P[13][14] + PS72*P[3][13] + PS74*P[0][13] + P[5][13];
    const ftype PS171 = PS74*P[0][1];
    const ftype PS172 = PS150 + PS160*P[1][15] - PS162*P[1][13] + PS171 - PS60*P[1][1] - PS65*P[1][14] + PS72*P[1][3] + P[1][5];
    const ftype PS173 = PS160*P[15][15] - PS162*P[13][15] - PS60*P[1][15] + PS62*P[2][15] - PS65*P[14][15] + PS72*P[3][15] + PS74*P[0][15] + P[5][15];
    const ftype PS174 = PS62*P[2][3];
    const ftype PS175 = PS148 + PS160*P[3][15] - PS162*P[3][13] + PS174 - PS60*P[1][3] - PS65*P[3][14] + PS72*P[3][3] + P[3][5];
    const ftype PS176 = PS60*P[0][1];
    const ftype PS177 = PS144 + PS160*P[0][15] - PS162*P[0][13] - PS176 + PS62*P[0][2] - PS65*P[0][14] + PS74*P[0][0] + P[0][5];
    const ftype PS178 = PS72*P[2][3];
    const ftype PS179 = -PS153 + PS160*P[2][15] - PS162*P[2][13] + PS178 + PS62*P[2][2] - PS65*P[2][14] + PS74*P[0][2] + P[2][5];
    const ftype PS180 = 4*dvxVar;
    const ftype PS181 = PS160*P[5][15] - PS162*P[5][13] - PS60*P[1][5] + PS62*P[2][5] - PS65*P[5][14] + PS72*P[3][5] + PS74*P[0][5] + P[5][5];
    const ftype PS182 = PS160*P[6][15] - PS162*P[6][13] - PS60*P[1][6] + PS62*P[2][6] - PS65*P[6][14] + PS72*P[3][6] + PS74*P[0][6] + P[5][6];
    const ftype PS183 = -PS165*P[14][15] + PS166*P[13][15] + PS60*P[0][15] + PS62*P[3][15] - PS70*P[15][15] - PS72*P[2][15] + PS74*P[1][15] + P[6][15];
    const ftype PS184 = -PS165*P[14][14] + PS166*P[13][14] + PS60*P[0][14] + PS62*P[3][14] - PS70*P[14][15] - PS72*P[2][14] + PS74*P[1][14] + P[6][14];
    const ftype PS185 = -PS165*P[13][14] + PS166*P[13][13] + PS60*P[0][13] + PS62*P[3][13] - PS70*P[13][15] - PS72*P[2][13] + PS74*P[1][13] + P[6][13];
    const ftype PS186 = -PS165*P[6][14] + PS166*P[6][13] + PS60*P[0][6] + PS62*P[3][6] - PS70*P[6][15] - PS72*P[2][6] + PS74*P[1][6] + P[6][6];

    nextP[0][0] = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
    nextP[0][1] = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
    nextP[1][1] = PS1*PS42 + PS11*PS80 - PS12*PS89 + PS13*PS82 + PS2*PS5 + PS3*PS4 - PS34*PS85 - PS7*PS87 + PS77*PS9 + PS92;
    nextP[0][2] = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
    nextP[1][2] = PS1*PS40 + PS11*PS89 + PS12*PS80 - PS13*PS92 - PS3*PS40 - PS34*PS77 - PS39*PS5 + PS6*PS87 + PS82 - PS85*PS9;
    nextP[2][2] = PS0*PS5 + PS1*PS4 + PS103*PS6 + PS104*PS11 + PS106*PS12 - PS108*PS34 - PS109*PS9 - PS111*PS13 + PS113 + PS3*PS42;
    nextP[0][3] = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
    nextP[1][3] = -PS1*PS38 - PS11*PS82 + PS12*PS92 + PS13*PS80 - PS3*PS37 - PS34*PS87 + PS38*PS5 - PS6*PS77 + PS7*PS85 + PS89;
    nextP[2][3] = -PS1*PS35 - PS103*PS34 + PS104 + PS106*PS13 - PS108*PS6 + PS109*PS7 - PS11*PS113 + PS111*PS12 + PS3*PS36 - PS36*PS5;
    nextP[3][3] = PS0*PS3 + PS1*PS2 - PS11*PS128 + PS12*PS124 + PS123*PS7 + PS125*PS13 - PS126*PS34 - PS127*PS6 + PS129 + PS42*PS5;

    if (quatCovResetOnly) {
        // covariance matrix is symmetrical, so copy diagonals and copy lower half in nextP
        // to lower and upper half in P
        for (uint8_t row = 0; row <= 3; row++) {
            // copy diagonals
            P[row][row] = constrain_ftype(nextP[row][row], 0.0f, 1.0f);
            // copy off diagonals
            for (uint8_t column = 0 ; column < row; column++) {
                P[row][column] = P[column][row] = nextP[column][row];
            }
        }
        calcTiltErrorVariance();
        return;
    }

    nextP[0][4] = PS23*PS62 + PS26*PS60 - PS44*PS45 - PS46*PS48 - PS52*PS53 + PS54*PS56 + PS57*PS58 + PS63;
    nextP[1][4] = -PS44*PS93 - PS46*PS95 + PS54*PS97 + PS60*PS82 + PS62*PS92 + PS72*PS80 - PS74*PS89 + PS98;
    nextP[2][4] = -PS104*PS74 + PS106*PS72 + PS111*PS62 + PS113*PS60 - PS114*PS44 - PS116*PS46 + PS118*PS54 + PS119;
    nextP[3][4] = PS124*PS62 + PS125*PS72 + PS128*PS60 - PS129*PS74 - PS130*PS44 - PS132*PS46 + PS134*PS54 + PS135;
    nextP[4][4] = -PS139*PS142 + PS140*PS146 - PS141*PS44 - PS145*PS74 + PS149*PS72 + PS152*PS60 + PS155*PS62 + PS156*sq(PS54) + PS157*sq(PS46) + PS158 + sq(PS44)*dvxVar;
    nextP[0][5] = -PS23*PS60 + PS26*PS62 + PS48*PS68 + PS52*PS58 + PS53*PS57 - PS55*PS65 - PS66*PS67 + PS69;
    nextP[1][5] = PS100 - PS60*PS92 + PS62*PS82 - PS65*PS96 - PS66*PS99 + PS68*PS95 + PS72*PS89 + PS74*PS80;
    nextP[2][5] = PS104*PS72 + PS106*PS74 - PS111*PS60 + PS113*PS62 + PS116*PS68 - PS117*PS65 - PS120*PS66 + PS121;
    nextP[3][5] = -PS124*PS60 + PS125*PS74 + PS128*PS62 + PS129*PS72 + PS132*PS68 - PS133*PS65 - PS136*PS66 + PS137;
    nextP[4][5] = -PS140*PS161 + PS142*PS160 + PS145*PS72 - PS146*PS65 + PS149*PS74 + PS152*PS62 - PS155*PS60 - PS157*PS46*PS68 - PS159*PS66 + PS162*PS163 + PS164;
    nextP[5][5] = PS157*sq(PS68) + PS160*PS173 - PS162*PS170 - PS169*PS65 - PS172*PS60 + PS175*PS72 + PS177*PS74 + PS179*PS62 + PS180*sq(PS66) + PS181 + sq(PS65)*dvyVar;
    nextP[0][6] = PS23*PS74 - PS26*PS72 - PS47*PS70 + PS53*PS61 - PS56*PS71 + PS58*PS59 + PS67*PS73 + PS75;
    nextP[1][6] = PS101 + PS60*PS80 + PS62*PS89 - PS70*PS94 - PS71*PS97 - PS72*PS82 + PS73*PS99 + PS74*PS92;
    nextP[2][6] = PS104*PS62 + PS106*PS60 + PS111*PS74 - PS113*PS72 - PS115*PS70 - PS118*PS71 + PS120*PS73 + PS122;
    nextP[3][6] = PS124*PS74 + PS125*PS60 - PS128*PS72 + PS129*PS62 - PS131*PS70 - PS134*PS71 + PS136*PS73 + PS138;
    nextP[4][6] = PS139*PS167 - PS142*PS70 + PS145*PS62 - PS146*PS165 + PS149*PS60 - PS152*PS72 + PS155*PS74 - PS156*PS54*PS71 + PS159*PS73 - PS163*PS166 + PS168;
    nextP[5][6] = -PS160*PS167 + PS161*PS165 - PS165*PS169 + PS166*PS170 + PS172*PS74 - PS173*PS70 + PS175*PS62 + PS177*PS60 - PS179*PS72 - PS180*PS66*PS73 + PS182;
    nextP[6][6] = PS156*sq(PS71) - PS165*PS184 + PS166*PS185 + PS180*sq(PS73) - PS183*PS70 + PS186 + PS60*(-PS151 - PS165*P[0][14] + PS166*P[0][13] + PS171 + PS60*P[0][0] + PS62*P[0][3] - PS70*P[0][15] + P[0][6]) + PS62*(PS154 - PS165*P[3][14] + PS166*P[3][13] - PS178 + PS60*P[0][3] + PS62*P[3][3] - PS70*P[3][15] + P[3][6]) + sq(PS70)*dvzVar - PS72*(PS147 - PS165*P[2][14] + PS166*P[2][13] + PS174 - PS70*P[2][15] - PS72*P[2][2] + PS74*P[1][2] + P[2][6]) + PS74*(PS143 - PS165*P[1][14] + PS166*P[1][13] + PS176 - PS70*P[1][15] - PS72*P[1][2] + PS74*P[1][1] + P[1][6]);
    nextP[0][7] = -PS11*P[1][7] - PS12*P[2][7] - PS13*P[3][7] + PS6*P[7][10] + PS63*dt + PS7*P[7][11] + PS9*P[7][12] + P[0][7];
    nextP[1][7] = PS11*P[0][7] - PS12*P[3][7] + PS13*P[2][7] - PS34*P[7][10] - PS7*P[7][12] + PS9*P[7][11] + PS98*dt + P[1][7];
    nextP[2][7] = PS11*P[3][7] + PS119*dt + PS12*P[0][7] - PS13*P[1][7] - PS34*P[7][11] + PS6*P[7][12] - PS9*P[7][10] + P[2][7];
    nextP[3][7] = -PS11*P[2][7] + PS12*P[1][7] + PS13*P[0][7] + PS135*dt - PS34*P[7][12] - PS6*P[7][11] + PS7*P[7][10] + P[3][7];
    nextP[4][7] = -PS139*P[7][15] + PS140*P[7][14] + PS158*dt - PS44*P[7][13] + PS60*P[2][7] + PS62*P[1][7] + PS72*P[0][7] - PS74*P[3][7] + P[4][7];
    nextP[5][7] = PS160*P[7][15] - PS162*P[7][13] - PS60*P[1][7] + PS62*P[2][7] - PS65*P[7][14] + PS72*P[3][7] + PS74*P[0][7] + P[5][7] + dt*(PS160*P[4][15] - PS162*P[4][13] - PS60*P[1][4] + PS62*P[2][4] - PS65*P[4][14] + PS72*P[3][4] + PS74*P[0][4] + P[4][5]);
    nextP[6][7] = -PS165*P[7][14] + PS166*P[7][13] + PS60*P[0][7] + PS62*P[3][7] - PS70*P[7][15] - PS72*P[2][7] + PS74*P[1][7] + P[6][7] + dt*(-PS165*P[4][14] + PS166*P[4][13] + PS60*P[0][4] + PS62*P[3][4] - PS70*P[4][15] - PS72*P[2][4] + PS74*P[1][4] + P[4][6]);
    nextP[7][7] = P[4][7]*dt + P[7][7] + dt*(P[4][4]*dt + P[4][7]);
    nextP[0][8] = -PS11*P[1][8] - PS12*P[2][8] - PS13*P[3][8] + PS6*P[8][10] + PS69*dt + PS7*P[8][11] + PS9*P[8][12] + P[0][8];
    nextP[1][8] = PS100*dt + PS11*P[0][8] - PS12*P[3][8] + PS13*P[2][8] - PS34*P[8][10] - PS7*P[8][12] + PS9*P[8][11] + P[1][8];
    nextP[2][8] = PS11*P[3][8] + PS12*P[0][8] + PS121*dt - PS13*P[1][8] - PS34*P[8][11] + PS6*P[8][12] - PS9*P[8][10] + P[2][8];
    nextP[3][8] = -PS11*P[2][8] + PS12*P[1][8] + PS13*P[0][8] + PS137*dt - PS34*P[8][12] - PS6*P[8][11] + PS7*P[8][10] + P[3][8];
    nextP[4][8] = -PS139*P[8][15] + PS140*P[8][14] + PS164*dt - PS44*P[8][13] + PS60*P[2][8] + PS62*P[1][8] + PS72*P[0][8] - PS74*P[3][8] + P[4][8];
    nextP[5][8] = PS160*P[8][15] - PS162*P[8][13] + PS181*dt - PS60*P[1][8] + PS62*P[2][8] - PS65*P[8][14] + PS72*P[3][8] + PS74*P[0][8] + P[5][8];
    nextP[6][8] = -PS165*P[8][14] + PS166*P[8][13] + PS60*P[0][8] + PS62*P[3][8] - PS70*P[8][15] - PS72*P[2][8] + PS74*P[1][8] + P[6][8] + dt*(-PS165*P[5][14] + PS166*P[5][13] + PS60*P[0][5] + PS62*P[3][5] - PS70*P[5][15] - PS72*P[2][5] + PS74*P[1][5] + P[5][6]);
    nextP[7][8] = P[4][8]*dt + P[7][8] + dt*(P[4][5]*dt + P[5][7]);
    nextP[8][8] = P[5][8]*dt + P[8][8] + dt*(P[5][5]*dt + P[5][8]);
    nextP[0][9] = -PS11*P[1][9] - PS12*P[2][9] - PS13*P[3][9] + PS6*P[9][10] + PS7*P[9][11] + PS75*dt + PS9*P[9][12] + P[0][9];
    nextP[1][9] = PS101*dt + PS11*P[0][9] - PS12*P[3][9] + PS13*P[2][9] - PS34*P[9][10] - PS7*P[9][12] + PS9*P[9][11] + P[1][9];
    nextP[2][9] = PS11*P[3][9] + PS12*P[0][9] + PS122*dt - PS13*P[1][9] - PS34*P[9][11] + PS6*P[9][12] - PS9*P[9][10] + P[2][9];
    nextP[3][9] = -PS11*P[2][9] + PS12*P[1][9] + PS13*P[0][9] + PS138*dt - PS34*P[9][12] - PS6*P[9][11] + PS7*P[9][10] + P[3][9];
    nextP[4][9] = -PS139*P[9][15] + PS140*P[9][14] + PS168*dt - PS44*P[9][13] + PS60*P[2][9] + PS62*P[1][9] + PS72*P[0][9] - PS74*P[3][9] + P[4][9];
    nextP[5][9] = PS160*P[9][15] - PS162*P[9][13] + PS182*dt - PS60*P[1][9] + PS62*P[2][9] - PS65*P[9][14] + PS72*P[3][9] + PS74*P[0][9] + P[5][9];
    nextP[6][9] = -PS165*P[9][14] + PS166*P[9][13] + PS186*dt + PS60*P[0][9] + PS62*P[3][9] - PS70*P[9][15] - PS72*P[2][9] + PS74*P[1][9] + P[6][9];
    nextP[7][9] = P[4][9]*dt + P[7][9] + dt*(P[4][6]*dt + P[6][7]);
    nextP[8][9] = P[5][9]*dt + P[8][9] + dt*(P[5][6]*dt + P[6][8]);
    nextP[9][9] = P[6][9]*dt + P[9][9] + dt*(P[6][6]*dt + P[6][9]);

    if (stateIndexLim > 9) {
        nextP[0][10] = PS14;
        nextP[1][10] = PS85;
        nextP[2][10] = PS109;
        nextP[3][10] = PS123;
        nextP[4][10] = -PS139*P[10][15] + PS140*P[10][14] - PS44*P[10][13] + PS60*P[2][10] + PS62*P[1][10] + PS72*P[0][10] - PS74*P[3][10] + P[4][10];
        nextP[5][10] = PS160*P[10][15] - PS162*P[10][13] - PS60*P[1][10] + PS62*P[2][10] - PS65*P[10][14] + PS72*P[3][10] + PS74*P[0][10] + P[5][10];
        nextP[6][10] = -PS165*P[10][14] + PS166*P[10][13] + PS60*P[0][10] + PS62*P[3][10] - PS70*P[10][15] - PS72*P[2][10] + PS74*P[1][10] + P[6][10];
        nextP[7][10] = P[4][10]*dt + P[7][10];
        nextP[8][10] = P[5][10]*dt + P[8][10];
        nextP[9][10] = P[6][10]*dt + P[9][10];
        nextP[10][10] = P[10][10];
        nextP[0][11] = PS17;
        nextP[1][11] = PS77;
        nextP[2][11] = PS108;
        nextP[3][11] = PS127;
        nextP[4][11] = -PS139*P[11][15] + PS140*P[11][14] - PS44*P[11][13] + PS60*P[2][11] + PS62*P[1][11] + PS72*P[0][11] - PS74*P[3][11] + P[4][11];
        nextP[5][11] = PS160*P[11][15] - PS162*P[11][13] - PS60*P[1][11] + PS62*P[2][11] - PS65*P[11][14] + PS72*P[3][11] + PS74*P[0][11] + P[5][11];
        nextP[6][11] = -PS165*P[11][14] + PS166*P[11][13] + PS60*P[0][11] + PS62*P[3][11] - PS70*P[11][15] - PS72*P[2][11] + PS74*P[1][11] + P[6][11];
        nextP[7][11] = P[4][11]*dt + P[7][11];
        nextP[8][11] = P[5][11]*dt + P[8][11];
        nextP[9][11] = P[6][11]*dt + P[9][11];
        nextP[10][11] = P[10][11];
        nextP[11][11] = P[11][11];
        nextP[0][12] = PS20;
        nextP[1][12] = PS87;
        nextP[2][12] = PS103;
        nextP[3][12] = PS126;
        nextP[4][12] = -PS139*P[12][15] + PS140*P[12][14] - PS44*P[12][13] + PS60*P[2][12] + PS62*P[1][12] + PS72*P[0][12] - PS74*P[3][12] + P[4][12];
        nextP[5][12] = PS160*P[12][15] - PS162*P[12][13] - PS60*P[1][12] + PS62*P[2][12] - PS65*P[12][14] + PS72*P[3][12] + PS74*P[0][12] + P[5][12];
        nextP[6][12] = -PS165*P[12][14] + PS166*P[12][13] + PS60*P[0][12] + PS62*P[3][12] - PS70*P[12][15] - PS72*P[2][12] + PS74*P[1][12] + P[6][12];
        nextP[7][12] = P[4][12]*dt + P[7][12];
        nextP[8][12] = P[5][12]*dt + P[8][12];
        nextP[9][12] = P[6][12]*dt + P[9][12];
        nextP[10][12] = P[10][12];
        nextP[11][12] = P[11][12];
        nextP[12][12] = P[12][12];

        if (stateIndexLim > 12) {
            nextP[0][13] = PS45;
            nextP[1][13] = PS93;
            nextP[2][13] = PS114;
            nextP[3][13] = PS130;
            nextP[4][13] = PS141;
            nextP[5][13] = PS170;
            nextP[6][13] = PS185;
            nextP[7][13] = P[4][13]*dt + P[7][13];
            nextP[8][13] = P[5][13]*dt + P[8][13];
            nextP[9][13] = P[6][13]*dt + P[9][13];
            nextP[10][13] = P[10][13];
            nextP[11][13] = P[11][13];
            nextP[12][13] = P[12][13];
            nextP[13][13] = P[13][13];
            nextP[0][14] = PS55;
            nextP[1][14] = PS96;
            nextP[2][14] = PS117;
            nextP[3][14] = PS133;
            nextP[4][14] = PS146;
            nextP[5][14] = PS169;
            nextP[6][14] = PS184;
            nextP[7][14] = P[4][14]*dt + P[7][14];
            nextP[8][14] = P[5][14]*dt + P[8][14];
            nextP[9][14] = P[6][14]*dt + P[9][14];
            nextP[10][14] = P[10][14];
            nextP[11][14] = P[11][14];
            nextP[12][14] = P[12][14];
            nextP[13][14] = P[13][14];
            nextP[14][14] = P[14][14];
            nextP[0][15] = PS47;
            nextP[1][15] = PS94;
            nextP[2][15] = PS115;
            nextP[3][15] = PS131;
            nextP[4][15] = PS142;
            nextP[5][15] = PS173;
            nextP[6][15] = PS183;
            nextP[7][15] = P[4][15]*dt + P[7][15];
            nextP[8][15] = P[5][15]*dt + P[8][15];
            nextP[9][15] = P[6][15]*dt + P[9][15];
            nextP[10][15] = P[10][15];
            nextP[11][15] = P[11][15];
            nextP[12][15] = P[12][15];
            nextP[13][15] = P[13][15];
            nextP[14][15] = P[14][15];
            nextP[15][15] = P[15][15];

            if (stateIndexLim > 15) {
                nextP[0][16] = -PS11*P[1][16] - PS12*P[2][16] - PS13*P[3][16] + PS6*P[10][16] + PS7*P[11][16] + PS9*P[12][16] + P[0][16];
                nextP[1][16] = PS11*P[0][16] - PS12*P[3][16] + PS13*P[2][16] - PS34*P[10][16] - PS7*P[12][16] + PS9*P[11][16] + P[1][16];
                nextP[2][16] = PS11*P[3][16] + PS12*P[0][16] - PS13*P[1][16] - PS34*P[11][16] + PS6*P[12][16] - PS9*P[10][16] + P[2][16];
                nextP[3][16] = -PS11*P[2][16] + PS12*P[1][16] + PS13*P[0][16] - PS34*P[12][16] - PS6*P[11][16] + PS7*P[10][16] + P[3][16];
                nextP[4][16] = -PS139*P[15][16] + PS140*P[14][16] - PS44*P[13][16] + PS60*P[2][16] + PS62*P[1][16] + PS72*P[0][16] - PS74*P[3][16] + P[4][16];
                nextP[5][16] = PS160*P[15][16] - PS162*P[13][16] - PS60*P[1][16] + PS62*P[2][16] - PS65*P[14][16] + PS72*P[3][16] + PS74*P[0][16] + P[5][16];
                nextP[6][16] = -PS165*P[14][16] + PS166*P[13][16] + PS60*P[0][16] + PS62*P[3][16] - PS70*P[15][16] - PS72*P[2][16] + PS74*P[1][16] + P[6][16];
                nextP[7][16] = P[4][16]*dt + P[7][16];
                nextP[8][16] = P[5][16]*dt + P[8][16];
                nextP[9][16] = P[6][16]*dt + P[9][16];
                nextP[10][16] = P[10][16];
                nextP[11][16] = P[11][16];
                nextP[12][16] = P[12][16];
                nextP[13][16] = P[13][16];
                nextP[14][16] = P[14][16];
                nextP[15][16] = P[15][16];
                nextP[16][16] = P[16][16];
                nextP[0][17] = -PS11*P[1][17] - PS12*P[2][17] - PS13*P[3][17] + PS6*P[10][17] + PS7*P[11][17] + PS9*P[12][17] + P[0][17];
                nextP[1][17] = PS11*P[0][17] - PS12*P[3][17] + PS13*P[2][17] - PS34*P[10][17] - PS7*P[12][17] + PS9*P[11][17] + P[1][17];
                nextP[2][17] = PS11*P[3][17] + PS12*P[0][17] - PS13*P[1][17] - PS34*P[11][17] + PS6*P[12][17] - PS9*P[10][17] + P[2][17];
                nextP[3][17] = -PS11*P[2][17] + PS12*P[1][17] + PS13*P[0][17] - PS34*P[12][17] - PS6*P[11][17] + PS7*P[10][17] + P[3][17];
                nextP[4][17] = -PS139*P[15][17] + PS140*P[14][17] - PS44*P[13][17] + PS60*P[2][17] + PS62*P[1][17] + PS72*P[0][17] - PS74*P[3][17] + P[4][17];
                nextP[5][17] = PS160*P[15][17] - PS162*P[13][17] - PS60*P[1][17] + PS62*P[2][17] - PS65*P[14][17] + PS72*P[3][17] + PS74*P[0][17] + P[5][17];
                nextP[6][17] = -PS165*P[14][17] + PS166*P[13][17] + PS60*P[0][17] + PS62*P[3][17] - PS70*P[15][17] - PS72*P[2][17] + PS74*P[1][17] + P[6][17];
                nextP[7][17] = P[4][17]*dt + P[7][17];
                nextP[8][17] = P[5][17]*dt + P[8][17];
                nextP[9][17] = P[6][17]*dt + P[9][17];
                nextP[10][17] = P[10][17];
                nextP[11][17] = P[11][17];
                nextP[12][17] = P[12][17];
                nextP[13][17] = P[13][17];
                nextP[14][17] = P[14][17];
                nextP[15][17] = P[15][17];
                nextP[16][17] = P[16][17];
                nextP[17][17] = P[17][17];
                nextP[0][18] = -PS11*P[1][18] - PS12*P[2][18] - PS13*P[3][18] + PS6*P[10][18] + PS7*P[11][18] + PS9*P[12][18] + P[0][18];
                nextP[1][18] = PS11*P[0][18] - PS12*P[3][18] + PS13*P[2][18] - PS34*P[10][18] - PS7*P[12][18] + PS9*P[11][18] + P[1][18];
                nextP[2][18] = PS11*P[3][18] + PS12*P[0][18] - PS13*P[1][18] - PS34*P[11][18] + PS6*P[12][18] - PS9*P[10][18] + P[2][18];
                nextP[3][18] = -PS11*P[2][18] + PS12*P[1][18] + PS13*P[0][18] - PS34*P[12][18] - PS6*P[11][18] + PS7*P[10][18] + P[3][18];
                nextP[4][18] = -PS139*P[15][18] + PS140*P[14][18] - PS44*P[13][18] + PS60*P[2][18] + PS62*P[1][18] + PS72*P[0][18] - PS74*P[3][18] + P[4][18];
                nextP[5][18] = PS160*P[15][18] - PS162*P[13][18] - PS60*P[1][18] + PS62*P[2][18] - PS65*P[14][18] + PS72*P[3][18] + PS74*P[0][18] + P[5][18];
                nextP[6][18] = -PS165*P[14][18] + PS166*P[13][18] + PS60*P[0][18] + PS62*P[3][18] - PS70*P[15][18] - PS72*P[2][18] + PS74*P[1][18] + P[6][18];
                nextP[7][18] = P[4][18]*dt + P[7][18];
                nextP[8][18] = P[5][18]*dt + P[8][18];
                nextP[9][18] = P[6][18]*dt + P[9][18];
                nextP[10][18] = P[10][18];
                nextP[11][18] = P[11][18];
                nextP[12][18] = P[12][18];
                nextP[13][18] = P[13][18];
                nextP[14][18] = P[14][18];
                nextP[15][18] = P[15][18];
                nextP[16][18] = P[16][18];
                nextP[17][18] = P[17][18];
                nextP[18][18] = P[18][18];
                nextP[0][19] = -PS11*P[1][19] - PS12*P[2][19] - PS13*P[3][19] + PS6*P[10][19] + PS7*P[11][19] + PS9*P[12][19] + P[0][19];
                nextP[1][19] = PS11*P[0][19] - PS12*P[3][19] + PS13*P[2][19] - PS34*P[10][19] - PS7*P[12][19] + PS9*P[11][19] + P[1][19];
                nextP[2][19] = PS11*P[3][19] + PS12*P[0][19] - PS13*P[1][19] - PS34*P[11][19] + PS6*P[12][19] - PS9*P[10][19] + P[2][19];
                nextP[3][19] = -PS11*P[2][19] + PS12*P[1][19] + PS13*P[0][19] - PS34*P[12][19] - PS6*P[11][19] + PS7*P[10][19] + P[3][19];
                nextP[4][19] = -PS139*P[15][19] + PS140*P[14][19] - PS44*P[13][19] + PS60*P[2][19] + PS62*P[1][19] + PS72*P[0][19] - PS74*P[3][19] + P[4][19];
                nextP[5][19] = PS160*P[15][19] - PS162*P[13][19] - PS60*P[1][19] + PS62*P[2][19] - PS65*P[14][19] + PS72*P[3][19] + PS74*P[0][19] + P[5][19];
                nextP[6][19] = -PS165*P[14][19] + PS166*P[13][19] + PS60*P[0][19] + PS62*P[3][19] - PS70*P[15][19] - PS72*P[2][19] + PS74*P[1][19] + P[6][19];
                nextP[7][19] = P[4][19]*dt + P[7][19];
                nextP[8][19] = P[5][19]*dt + P[8][19];
                nextP[9][19] = P[6][19]*dt + P[9][19];
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
                nextP[0][20] = -PS11*P[1][20] - PS12*P[2][20] - PS13*P[3][20] + PS6*P[10][20] + PS7*P[11][20] + PS9*P[12][20] + P[0][20];
                nextP[1][20] = PS11*P[0][20] - PS12*P[3][20] + PS13*P[2][20] - PS34*P[10][20] - PS7*P[12][20] + PS9*P[11][20] + P[1][20];
                nextP[2][20] = PS11*P[3][20] + PS12*P[0][20] - PS13*P[1][20] - PS34*P[11][20] + PS6*P[12][20] - PS9*P[10][20] + P[2][20];
                nextP[3][20] = -PS11*P[2][20] + PS12*P[1][20] + PS13*P[0][20] - PS34*P[12][20] - PS6*P[11][20] + PS7*P[10][20] + P[3][20];
                nextP[4][20] = -PS139*P[15][20] + PS140*P[14][20] - PS44*P[13][20] + PS60*P[2][20] + PS62*P[1][20] + PS72*P[0][20] - PS74*P[3][20] + P[4][20];
                nextP[5][20] = PS160*P[15][20] - PS162*P[13][20] - PS60*P[1][20] + PS62*P[2][20] - PS65*P[14][20] + PS72*P[3][20] + PS74*P[0][20] + P[5][20];
                nextP[6][20] = -PS165*P[14][20] + PS166*P[13][20] + PS60*P[0][20] + PS62*P[3][20] - PS70*P[15][20] - PS72*P[2][20] + PS74*P[1][20] + P[6][20];
                nextP[7][20] = P[4][20]*dt + P[7][20];
                nextP[8][20] = P[5][20]*dt + P[8][20];
                nextP[9][20] = P[6][20]*dt + P[9][20];
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
                nextP[0][21] = -PS11*P[1][21] - PS12*P[2][21] - PS13*P[3][21] + PS6*P[10][21] + PS7*P[11][21] + PS9*P[12][21] + P[0][21];
                nextP[1][21] = PS11*P[0][21] - PS12*P[3][21] + PS13*P[2][21] - PS34*P[10][21] - PS7*P[12][21] + PS9*P[11][21] + P[1][21];
                nextP[2][21] = PS11*P[3][21] + PS12*P[0][21] - PS13*P[1][21] - PS34*P[11][21] + PS6*P[12][21] - PS9*P[10][21] + P[2][21];
                nextP[3][21] = -PS11*P[2][21] + PS12*P[1][21] + PS13*P[0][21] - PS34*P[12][21] - PS6*P[11][21] + PS7*P[10][21] + P[3][21];
                nextP[4][21] = -PS139*P[15][21] + PS140*P[14][21] - PS44*P[13][21] + PS60*P[2][21] + PS62*P[1][21] + PS72*P[0][21] - PS74*P[3][21] + P[4][21];
                nextP[5][21] = PS160*P[15][21] - PS162*P[13][21] - PS60*P[1][21] + PS62*P[2][21] - PS65*P[14][21] + PS72*P[3][21] + PS74*P[0][21] + P[5][21];
                nextP[6][21] = -PS165*P[14][21] + PS166*P[13][21] + PS60*P[0][21] + PS62*P[3][21] - PS70*P[15][21] - PS72*P[2][21] + PS74*P[1][21] + P[6][21];
                nextP[7][21] = P[4][21]*dt + P[7][21];
                nextP[8][21] = P[5][21]*dt + P[8][21];
                nextP[9][21] = P[6][21]*dt + P[9][21];
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
                    nextP[0][22] = -PS11*P[1][22] - PS12*P[2][22] - PS13*P[3][22] + PS6*P[10][22] + PS7*P[11][22] + PS9*P[12][22] + P[0][22];
                    nextP[1][22] = PS11*P[0][22] - PS12*P[3][22] + PS13*P[2][22] - PS34*P[10][22] - PS7*P[12][22] + PS9*P[11][22] + P[1][22];
                    nextP[2][22] = PS11*P[3][22] + PS12*P[0][22] - PS13*P[1][22] - PS34*P[11][22] + PS6*P[12][22] - PS9*P[10][22] + P[2][22];
                    nextP[3][22] = -PS11*P[2][22] + PS12*P[1][22] + PS13*P[0][22] - PS34*P[12][22] - PS6*P[11][22] + PS7*P[10][22] + P[3][22];
                    nextP[4][22] = -PS139*P[15][22] + PS140*P[14][22] - PS44*P[13][22] + PS60*P[2][22] + PS62*P[1][22] + PS72*P[0][22] - PS74*P[3][22] + P[4][22];
                    nextP[5][22] = PS160*P[15][22] - PS162*P[13][22] - PS60*P[1][22] + PS62*P[2][22] - PS65*P[14][22] + PS72*P[3][22] + PS74*P[0][22] + P[5][22];
                    nextP[6][22] = -PS165*P[14][22] + PS166*P[13][22] + PS60*P[0][22] + PS62*P[3][22] - PS70*P[15][22] - PS72*P[2][22] + PS74*P[1][22] + P[6][22];
                    nextP[7][22] = P[4][22]*dt + P[7][22];
                    nextP[8][22] = P[5][22]*dt + P[8][22];
                    nextP[9][22] = P[6][22]*dt + P[9][22];
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
                    nextP[0][23] = -PS11*P[1][23] - PS12*P[2][23] - PS13*P[3][23] + PS6*P[10][23] + PS7*P[11][23] + PS9*P[12][23] + P[0][23];
                    nextP[1][23] = PS11*P[0][23] - PS12*P[3][23] + PS13*P[2][23] - PS34*P[10][23] - PS7*P[12][23] + PS9*P[11][23] + P[1][23];
                    nextP[2][23] = PS11*P[3][23] + PS12*P[0][23] - PS13*P[1][23] - PS34*P[11][23] + PS6*P[12][23] - PS9*P[10][23] + P[2][23];
                    nextP[3][23] = -PS11*P[2][23] + PS12*P[1][23] + PS13*P[0][23] - PS34*P[12][23] - PS6*P[11][23] + PS7*P[10][23] + P[3][23];
                    nextP[4][23] = -PS139*P[15][23] + PS140*P[14][23] - PS44*P[13][23] + PS60*P[2][23] + PS62*P[1][23] + PS72*P[0][23] - PS74*P[3][23] + P[4][23];
                    nextP[5][23] = PS160*P[15][23] - PS162*P[13][23] - PS60*P[1][23] + PS62*P[2][23] - PS65*P[14][23] + PS72*P[3][23] + PS74*P[0][23] + P[5][23];
                    nextP[6][23] = -PS165*P[14][23] + PS166*P[13][23] + PS60*P[0][23] + PS62*P[3][23] - PS70*P[15][23] - PS72*P[2][23] + PS74*P[1][23] + P[6][23];
                    nextP[7][23] = P[4][23]*dt + P[7][23];
                    nextP[8][23] = P[5][23]*dt + P[8][23];
                    nextP[9][23] = P[6][23]*dt + P[9][23];
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
        }
    }

    // add the general state process noise variances
    if (stateIndexLim > 9) {
        for (uint8_t i=10; i<=stateIndexLim; i++) {
            nextP[i][i] = nextP[i][i] + processNoiseVariance[i-10];
        }
    }

    // inactive delta velocity bias states have all covariances zeroed to prevent
    // interacton with other states
    if (!inhibitDelVelBiasStates) {
        for (uint8_t index=0; index<3; index++) {
            const uint8_t stateIndex = index + 13;
            if (dvelBiasAxisInhibit[index]) {
                zeroCols(nextP,stateIndex,stateIndex);
                nextP[stateIndex][stateIndex] = dvelBiasAxisVarPrev[index];
            }
        }
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1e4f) {
        for (uint8_t i=7; i<=8; i++)
        {
            for (uint8_t j=0; j<=stateIndexLim; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // covariance matrix is symmetrical, so copy diagonals and copy lower half in nextP
    // to lower and upper half in P
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        // copy diagonals
        P[row][row] = nextP[row][row];
        // copy off diagonals
        for (uint8_t column = 0 ; column < row; column++) {
            P[row][column] = P[column][row] = nextP[column][row];
        }
    }

    // constrain values to prevent ill-conditioning
    ConstrainVariances();

    if (vertVelVarClipCounter > 0) {
        vertVelVarClipCounter--;
    }

    calcTiltErrorVariance();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    verifyTiltErrorVariance();
#endif
}

// zero specified range of rows in the state covariance matrix
void NavEKF3_core::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        zero_range(&covMat[row][0], 0, 23);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF3_core::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)
    {
        zero_range(&covMat[row][0], first, last);
    }
}

// reset the output data to the current EKF state
void NavEKF3_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position derivative output
    vertCompFiltState.pos = stateStruct.position.z;
    vertCompFiltState.vel = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF3_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavEKF3_core::StoreQuatRotate(const QuaternionF &deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF3_core::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            ftype temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
// if states are inactive, zero the corresponding off-diagonals
void NavEKF3_core::ConstrainVariances()
{
    for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_ftype(P[i][i],0.0,1.0); // attitude error
    for (uint8_t i=4; i<=5; i++) P[i][i] = constrain_ftype(P[i][i], VEL_STATE_MIN_VARIANCE, 1.0e3); // NE velocity

    // check for collapse of the vertical velocity variance
    if (P[6][6] < VEL_STATE_MIN_VARIANCE) {
        P[6][6] = VEL_STATE_MIN_VARIANCE;
        // this counter is decremented by 1 each prediction cycle in CovariancePrediction
        // resulting in the count from each clip event fading to zero over 1 second which
        // is sufficient to capture collapse from fusion of the lowest update rate sensor
        vertVelVarClipCounter += EKF_TARGET_RATE_HZ;
        if (vertVelVarClipCounter > VERT_VEL_VAR_CLIP_COUNT_LIM) {
            // reset the corresponding covariances
            zeroRows(P,6,6);
            zeroCols(P,6,6);

            // set the variances to the measurement variance
        #if EK3_FEATURE_EXTERNAL_NAV
            if (useExtNavVel) {
                P[6][6] = sq(extNavVelDelayed.err);
            } else
        #endif
            {
                P[6][6] = sq(frontend->_gpsVertVelNoise);
            }
            vertVelVarClipCounter = 0;
        }
    }

    for (uint8_t i=7; i<=9; i++) P[i][i] = constrain_ftype(P[i][i], POS_STATE_MIN_VARIANCE, 1.0e6); // NED position

    if (!inhibitDelAngBiasStates) {
        for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,sq(0.175 * dtEkfAvg));
    } else {
        zeroCols(P,10,12);
        zeroRows(P,10,12);
    }

    const ftype minStateVarTarget = 1E-8;
    if (!inhibitDelVelBiasStates) {

        // Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
        const ftype minSafeStateVar = 1e-9;
        ftype maxStateVar = minSafeStateVar;
        bool resetRequired = false;
        for (uint8_t stateIndex=13; stateIndex<=15; stateIndex++) {
            if (P[stateIndex][stateIndex] > maxStateVar) {
                maxStateVar = P[stateIndex][stateIndex];
            } else if (P[stateIndex][stateIndex] < minSafeStateVar) {
                resetRequired = true;
            }
        }

        // To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
        // not exceed 100 and the minimum variance must not fall below the target minimum
        ftype minAllowedStateVar = fmaxF(0.01f * maxStateVar, minStateVarTarget);
        for (uint8_t stateIndex=13; stateIndex<=15; stateIndex++) {
            P[stateIndex][stateIndex] = constrain_ftype(P[stateIndex][stateIndex], minAllowedStateVar, sq(10.0f * dtEkfAvg));
        }

        // If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
        if (resetRequired) {
            ftype delVelBiasVar[3];
            // store all delta velocity bias variances
            for (uint8_t i=0; i<=2; i++) {
                delVelBiasVar[i] = P[i+13][i+13];
            }
            // reset all delta velocity bias covariances
            zeroCols(P,13,15);
            // restore all delta velocity bias variances
            for (uint8_t i=0; i<=2; i++) {
                P[i+13][i+13] = delVelBiasVar[i];
            }
        }

    } else {
        zeroCols(P,13,15);
        zeroRows(P,13,15);
        for (uint8_t i=0; i<=2; i++) {
            const uint8_t stateIndex = 1 + 13;
            P[stateIndex][stateIndex] = fmaxF(P[stateIndex][stateIndex], minStateVarTarget);
        }
    }

    if (!inhibitMagStates) {
        for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,0.01f); // earth magnetic field
        for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,0.01f); // body magnetic field
    } else {
        zeroCols(P,16,21);
        zeroRows(P,16,21);
    }

    if (!inhibitWindStates) {
        for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,1.0e3f);
    } else {
        zeroCols(P,22,23);
        zeroRows(P,22,23);
    }
}

// constrain states using WMM tables and specified limit
void NavEKF3_core::MagTableConstrain(void)
{
    // constrain to error from table earth field
    ftype limit_ga = frontend->_mag_ef_limit * 0.001f;
    stateStruct.earth_magfield.x = constrain_ftype(stateStruct.earth_magfield.x,
                                                   table_earth_field_ga.x-limit_ga,
                                                   table_earth_field_ga.x+limit_ga);
    stateStruct.earth_magfield.y = constrain_ftype(stateStruct.earth_magfield.y,
                                                   table_earth_field_ga.y-limit_ga,
                                                   table_earth_field_ga.y+limit_ga);
    stateStruct.earth_magfield.z = constrain_ftype(stateStruct.earth_magfield.z,
                                                   table_earth_field_ga.z-limit_ga,
                                                   table_earth_field_ga.z+limit_ga);
}

// constrain states to prevent ill-conditioning
void NavEKF3_core::ConstrainStates()
{
    // quaternions are limited between +-1
    for (uint8_t i=0; i<=3; i++) statesArray[i] = constrain_ftype(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=4; i<=6; i++) statesArray[i] = constrain_ftype(statesArray[i],-5.0e2f,5.0e2f);
    // position limit TODO apply circular limit
    for (uint8_t i=7; i<=8; i++) statesArray[i] = constrain_ftype(statesArray[i],-EK3_POSXY_STATE_LIMIT,EK3_POSXY_STATE_LIMIT);
    // height limit covers home alt on everest through to home alt at SL and balloon drop
    stateStruct.position.z = constrain_ftype(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit (this needs to be set based on manufacturers specs)
    for (uint8_t i=10; i<=12; i++) statesArray[i] = constrain_ftype(statesArray[i],-GYRO_BIAS_LIMIT*dtEkfAvg,GYRO_BIAS_LIMIT*dtEkfAvg);
    // the accelerometer bias limit is controlled by a user adjustable parameter
    for (uint8_t i=13; i<=15; i++) statesArray[i] = constrain_ftype(statesArray[i],-frontend->_accBiasLim*dtEkfAvg,frontend->_accBiasLim*dtEkfAvg);
    // earth magnetic field limit
    if (frontend->_mag_ef_limit <= 0 || !have_table_earth_field) {
        // constrain to +/-1Ga
        for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_ftype(statesArray[i],-1.0f,1.0f);
    } else {
        // use table constrain
        MagTableConstrain();
    }
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_ftype(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_ftype(statesArray[i],-100.0f,100.0f);
    // constrain the terrain state to be below the vehicle height unless we are using terrain as the height datum
    if (!inhibitGndState) {
        terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);
    }
}

// calculate the NED earth spin vector in rad/sec
void NavEKF3_core::calcEarthRateNED(Vector3F &omega, int32_t latitude) const
{
    ftype lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosF(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinF(lat_rad);
}

// set yaw from a single magnetometer sample
void NavEKF3_core::setYawFromMag()
{
    if (!use_compass()) {
        return;
    }

    // read the magnetometer data
    readMagData();

    // use best of either 312 or 321 rotation sequence when calculating yaw
    rotationOrder order;
    bestRotationOrder(order);
    Vector3F eulerAngles;
    Matrix3F Tbn_zeroYaw;
    if (order == rotationOrder::TAIT_BRYAN_321) {
        // rolled more than pitched so use 321 rotation order
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        Tbn_zeroYaw.from_euler(eulerAngles.x, eulerAngles.y, 0.0f);
    } else if (order == rotationOrder::TAIT_BRYAN_312) {
        // pitched more than rolled so use 312 rotation order
        eulerAngles = stateStruct.quat.to_vector312();
        Tbn_zeroYaw.from_euler312(eulerAngles.x, eulerAngles.y, 0.0f);
    } else {
        // rotation order not supported
        return;
    }

    Vector3F magMeasNED = Tbn_zeroYaw * magDataDelayed.mag;
    ftype yawAngMeasured = wrap_PI(-atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination());

    // update quaternion states and covariances
    resetQuatStateYawOnly(yawAngMeasured, sq(MAX(frontend->_yawNoise, 1.0e-2f)), order);
}

// update mag field states and associated variances using magnetomer and declination data
void NavEKF3_core::resetMagFieldStates()
{
    // Rotate Mag measurements into NED to set initial NED magnetic field states

    // update rotation matrix from body to NED frame
    stateStruct.quat.inverse().rotation_matrix(prevTnb);

    if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
        stateStruct.earth_magfield = table_earth_field_ga;
    } else {
        stateStruct.earth_magfield = prevTnb.transposed() * magDataDelayed.mag;
    }

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

    // record the fact we have initialised the magnetic field states
    recordMagReset();
}

// zero the attitude covariances, but preserve the variances
void NavEKF3_core::zeroAttCovOnly()
{
    ftype varTemp[4];
    for (uint8_t index=0; index<=3; index++) {
        varTemp[index] = P[index][index];
    }
    zeroCols(P,0,3);
    zeroRows(P,0,3);
    for (uint8_t index=0; index<=3; index++) {
        P[index][index] = varTemp[index];
    }
}

// calculate the tilt error variance
void NavEKF3_core::calcTiltErrorVariance()
{
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // equations generated by quaternion_error_propagation(): in AP_NavEKF3/derivation/main.py
    // only diagonals have been used
    // dq0 ... dq3  terms have been zeroed
    const ftype PS1 = q0*q1 + q2*q3;
    const ftype PS2 = q1*PS1;
    const ftype PS4 = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    const ftype PS5 = q0*PS4;
    const ftype PS6 = 2*PS2 + PS5;
    const ftype PS8 = PS1*q2;
    const ftype PS10 = PS4*q3;
    const ftype PS11 = PS10 + 2*PS8;
    const ftype PS12 = PS1*q3;
    const ftype PS13 = PS4*q2;
    const ftype PS14 = -2*PS12 + PS13;
    const ftype PS15 = PS1*q0;
    const ftype PS16 = q1*PS4;
    const ftype PS17 = 2*PS15 - PS16;
    const ftype PS18 = q0*q2 - q1*q3;
    const ftype PS19 = PS18*q2;
    const ftype PS20 = 2*PS19 + PS5;
    const ftype PS22 = q1*PS18;
    const ftype PS23 = -PS10 + 2*PS22;
    const ftype PS25 = PS18*q3;
    const ftype PS26 = PS16 + 2*PS25;
    const ftype PS28 = PS18*q0;
    const ftype PS29 = -PS13 + 2*PS28;
    const ftype PS32 = PS12 + PS28;
    const ftype PS33 = PS19 + PS2;
    const ftype PS34 = PS15 - PS25;
    const ftype PS35 = PS22 - PS8;

    tiltErrorVariance  = 4*sq(PS11)*P[2][2] + 4*sq(PS14)*P[3][3] + 4*sq(PS17)*P[0][0] + 4*sq(PS6)*P[1][1];
    tiltErrorVariance += 4*sq(PS20)*P[2][2] + 4*sq(PS23)*P[1][1] + 4*sq(PS26)*P[3][3] + 4*sq(PS29)*P[0][0];
    tiltErrorVariance += 16*sq(PS32)*P[1][1] + 16*sq(PS33)*P[3][3] + 16*sq(PS34)*P[2][2] + 16*sq(PS35)*P[0][0];

    tiltErrorVariance = constrain_ftype(tiltErrorVariance, 0.0f, sq(radians(30.0f)));
}

void NavEKF3_core::bestRotationOrder(rotationOrder &order)
{
    if (fabsF(prevTnb[2][0]) < fabsF(prevTnb[2][1])) {
        // rolled more than pitched so use 321 sequence
        order = rotationOrder::TAIT_BRYAN_321;
    } else {
        // pitched more than rolled so use 312 sequence
        order = rotationOrder::TAIT_BRYAN_312;
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// calculate the tilt error variance using an alternative numerical difference technique
// and log with value generated by NavEKF3_core::calcTiltErrorVariance()
void NavEKF3_core::verifyTiltErrorVariance()
{
    const Vector3f gravity_ef = Vector3f(0.0f,0.0f,1.0f);
    Matrix3f Tnb;
    const float quat_delta = 0.001f;
    float tiltErrorVarianceAlt = 0.0f;
    for (uint8_t index = 0; index<4; index++) {
        QuaternionF quat = stateStruct.quat;

        // Add a positive increment to the quaternion element and calculate the tilt error vector
        quat[index] = stateStruct.quat[index] + quat_delta;
        quat.inverse().rotation_matrix(Tnb);
        const Vector3f gravity_bf_plus = Tnb * gravity_ef;

        // Add a negative increment to the quaternion element and calculate the tilt error vector
        quat[index] = stateStruct.quat[index] - quat_delta;
        quat.inverse().rotation_matrix(Tnb);
        const Vector3f gravity_bf_minus = Tnb * gravity_ef;

        // calculate the angular difference between the two vectors using a small angle assumption
        const Vector3f tilt_diff_vec = gravity_bf_minus % gravity_bf_plus;

        // calculate the partial derivative of angle error wrt the quaternion element
        const float tilt_error_derivative = tilt_diff_vec.length() / (2.0f * quat_delta);

        // sum the contribution of the quaternion elemnent variance to the tilt angle error variance
        tiltErrorVarianceAlt += P[index][index] * sq(tilt_error_derivative);
    }

    tiltErrorVarianceAlt = MIN(tiltErrorVarianceAlt, sq(radians(30.0f)));
    static uint32_t lastLogTime_ms = 0;
    if (imuSampleTime_ms - lastLogTime_ms > 500) {
        lastLogTime_ms = imuSampleTime_ms;
        const struct log_XKTV msg {
            LOG_PACKET_HEADER_INIT(LOG_XKTV_MSG),
            time_us      : dal.micros64(),
            core         : core_index,
            tvs          : float(tiltErrorVariance),
            tvd          : float(tiltErrorVarianceAlt),
        };
        AP::logger().WriteBlock(&msg, sizeof(msg));
    }
}
#endif

/*
  move the EKF origin to the current position at 1Hz. The public_origin doesn't move.
  By moving the EKF origin we keep the distortion due to spherical
  shape of the earth to a minimum.
 */
void NavEKF3_core::moveEKFOrigin(void)
{
    // only move origin when we have a origin and we're using GPS
    if (!frontend->common_origin_valid || !filterStatus.flags.using_gps) {
        return;
    }

    // move the origin to the current state location
    Location loc = EKF_origin;
    loc.offset(stateStruct.position.x, stateStruct.position.y);
    const Vector2F diffNE = loc.get_distance_NE_ftype(EKF_origin);
    EKF_origin = loc;

    // now fix all output states
    stateStruct.position.xy() += diffNE;
    outputDataNew.position.xy() += diffNE;
    outputDataDelayed.position.xy() += diffNE;

    for (unsigned index=0; index < imu_buffer_length; index++) {
        storedOutput[index].position.xy() += diffNE;
    }
}
