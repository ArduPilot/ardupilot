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
#if HAL_GCS_ENABLED
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

#if EK3_FEATURE_EXTERNAL_NAV
    // calculate buffer size for external nav data
    const uint8_t extnav_buffer_length = MIN((ekf_delay_ms / frontend->extNavIntervalMin_ms) + 1, imu_buffer_length);
#endif // EK3_FEATURE_EXTERNAL_NAV

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
#if EK3_FEATURE_BEACON_FUSION
    if(dal.beacon() && !rngBcn.storedRange.init(imu_buffer_length+1)) {
        return false;
    }
#endif
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
    dragTimeout = true;
    badIMUdata = false;
    badIMUdata_ms = 0;
    goodIMUdata_ms = 0;
    vertVelVarClipCounter = 0;
    finalInflightYawInit = false;
    dtIMUavg = ins.get_loop_delta_t();
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    lastKnownPositionD = 0;
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
    windStatesAligned = false;
    inhibitAccelBiasStates = true;
    inhibitGyroBiasStates = true;
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
    yawAlignGpsValidCount = 0;
    have_table_earth_field = false;
    stateIndexLim = 23;
    last_gps_idx = 0;
    delAngCorrection.zero();
    velErrintegral.zero();
    posErrintegral.zero();
    gpsGoodToAlign = false;
    gpsIsInUse = false;
    motorsArmed = false;
    prevMotorsArmed = false;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastGpsInnovPassTime_ms = 0;
    lastGpsInnovFailTime_ms = 0;
    lastGpsVertAccPassTime_ms = 0;
    lastGpsVertAccFailTime_ms = 0;
    gpsAccuracyGood = false;
    gpsAccuracyGoodForAltitude = false;
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
#if EK3_FEATURE_BEACON_FUSION
    memset((void *)&rngBcn.dataDelayed, 0, sizeof(rngBcn.dataDelayed));
    rngBcn.lastPassTime_ms = 0;
    rngBcn.testRatio = 0.0f;
    rngBcn.health = false;
    rngBcn.varInnov = 0.0f;
    rngBcn.innov = 0.0f;
    memset(&rngBcn.lastTime_ms, 0, sizeof(rngBcn.lastTime_ms));
    rngBcn.dataToFuse = false;
    rngBcn.vehiclePosNED.zero();
    rngBcn.vehiclePosErr = 1.0f;
    rngBcn.last3DmeasTime_ms = 0;
    rngBcn.goodToAlign = false;
    rngBcn.lastChecked = 0;
    rngBcn.receiverPos.zero();
    memset(&rngBcn.receiverPosCov, 0, sizeof(rngBcn.receiverPosCov));
    rngBcn.alignmentStarted =  false;
    rngBcn.alignmentCompleted = false;
    rngBcn.lastIndex = 0;
    rngBcn.posSum.zero();
    rngBcn.numMeas = 0;
    rngBcn.sum = 0.0f;
    rngBcn.N = 0;
    rngBcn.maxPosD = 0.0f;
    rngBcn.minPosD = 0.0f;
    rngBcn.posDownOffsetMax = 0.0f;
    rngBcn.posOffsetMaxVar = 0.0f;
    rngBcn.maxOffsetStateChangeFilt = 0.0f;
    rngBcn.posDownOffsetMin = 0.0f;
    rngBcn.posOffsetMinVar = 0.0f;
    rngBcn.minOffsetStateChangeFilt = 0.0f;
    rngBcn.fuseDataReportIndex = 0;
    if (dal.beacon()) {
        if (rngBcn.fusionReport == nullptr) {
            rngBcn.fusionReport = new BeaconFusion::FusionReport[dal.beacon()->count()];
        }
    }
    rngBcn.posOffsetNED.zero();
    rngBcn.originEstInit = false;
#endif  // EK3_FEATURE_BEACON_FUSION

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
#if EK3_FEATURE_BEACON_FUSION
    rngBcn.storedRange.reset();
#endif
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
    inhibitMagStates = true;
    magSelectIndex = dal.compass().get_first_usable();
    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;
    posDownAtLastMagReset = stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    stateStruct.quat.initialise();
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
    readGpsYawData();
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
    P[10][10] = sq(radians(InitialGyroBiasUncertainty()));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // delta velocity biases
    P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim);
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

#if EK3_FEATURE_BEACON_FUSION
        // Update states using range beacon data
        SelectRngBcnFusion();
#endif

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
    delAng -= inactiveBias[gyro_index].gyro_bias * delAngDT;
}

void NavEKF3_core::correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index)
{
    delVel -= inactiveBias[accel_index].accel_bias * delVelDT;
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
    accNavMagHoriz = velDotNEDfilt.xy().length();

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

#if EK3_FEATURE_BEACON_FUSION
    // If main filter velocity states are valid, update the range beacon receiver position states
    if (filterStatus.flags.horiz_vel) {
        rngBcn.receiverPos += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);
    }
#endif
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

        if (badIMUdata) {
            // When IMU accel is bad,  calculate an integral that will be used to drive the difference
            // between the output state and internal EKF state at the delayed time horizon to zero.
            badImuVelErrIntegral += (stateStruct.velocity.z - outputDataNew.velocity.z);
        } else {
            badImuVelErrIntegral = velErrintegral.z;
        }

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
        Vector3F posCorrection = posErr * velPosGain + posErrintegral * sq(velPosGain) * 0.1F;
        Vector3F velCorrection;
        velCorrection.x = velErr.x * velPosGain + velErrintegral.x * sq(velPosGain) * 0.1F;
        velCorrection.y = velErr.y * velPosGain + velErrintegral.y * sq(velPosGain) * 0.1F;
        if (badIMUdata) {
            velCorrection.z = velErr.z * velPosGain + badImuVelErrIntegral * sq(velPosGain) * 0.07F;
            velErrintegral.z = badImuVelErrIntegral;
        } else {
            velCorrection.z = velErr.z * velPosGain + velErrintegral.z * sq(velPosGain) * 0.1F;
        }

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
    ftype gx_b;         // X axis rate gyro measurement bias (rad/s)
    ftype gy_b;         // Y axis rate gyro measurement bias (rad/s)
    ftype gz_b;         // Z axis rate gyro measurement bias (rad/s)
    ftype ax_b;         // X axis accelerometer measurement bias (m/s/s)
    ftype ay_b;         // Y axis accelerometer measurement bias (m/s/s)
    ftype az_b;         // Z axis accelerometer measurement bias (m/s/s)

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

    if (!inhibitGyroBiasStates) {
        ftype gyroBiasVar = sq(dt * constrain_ftype(frontend->_gyroBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=0; i<=2; i++) processNoiseVariance[i] = gyroBiasVar;
    }

    if (!inhibitAccelBiasStates) {
        // default process noise (m/s)^2
        ftype accelBiasVar = sq(dt * constrain_ftype(frontend->_accelBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=3; i<=5; i++) {
            processNoiseVariance[i] = accelBiasVar;
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
        const bool isDragFusionDeadReckoning = filterStatus.flags.dead_reckoning && !dragTimeout;
        if (isDragFusionDeadReckoning) {
            // when dead reckoning using drag fusion stop learning wind states to provide a more stable velocity estimate
            P[23][23] = P[22][22] = 0.0f;
        } else {
	        ftype windVelVar  = sq(dt * constrain_ftype(frontend->_windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrain_ftype(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsF(hgtRate)));
	        if (!tasDataDelayed.allowFusion) {
	            // Allow wind states to recover faster when using sideslip fusion with a failed airspeed sesnor
	            windVelVar *= 10.0f;
	        }
	        for (uint8_t i=12; i<=13; i++) processNoiseVariance[i] = windVelVar;
        }
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
    gx_b = stateStruct.gyro_bias.x;
    gy_b = stateStruct.gyro_bias.y;
    gz_b = stateStruct.gyro_bias.z;
    ax_b = stateStruct.accel_bias.x;
    ay_b = stateStruct.accel_bias.y;
    az_b = stateStruct.accel_bias.z;

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
    ftype _accNoise = badIMUdata ? BAD_IMU_DATA_ACC_P_NSE : constrain_ftype(frontend->_accNoise, 0.0f, BAD_IMU_DATA_ACC_P_NSE);
    dvxVar = dvyVar = dvzVar = sq(dt*_accNoise);

    if (!inhibitAccelBiasStates) {
        for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
            const uint8_t index = stateIndex - 13;

            // Don't attempt learning of IMU delta velocty bias if on ground and not aligned with the gravity vector
            const bool is_bias_observable = (fabsF(prevTnb[index][2]) > 0.8f) || !onGround;

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
    const ftype PS0 = powf(q1, 2);
    const ftype PS1 = 0.25F*daxVar;
    const ftype PS2 = powf(q2, 2);
    const ftype PS3 = 0.25F*dayVar;
    const ftype PS4 = powf(q3, 2);
    const ftype PS5 = 0.25F*dazVar;
    const ftype PS6 = 0.5F*dt;
    const ftype PS7 = PS6*q1;
    const ftype PS8 = PS6*q2;
    const ftype PS9 = PS6*q3;
    const ftype PS10 = 0.5F*dax - 0.5F*dt*gx_b;
    const ftype PS11 = 0.5F*day - 0.5F*dt*gy_b;
    const ftype PS12 = PS11*P[1][2];
    const ftype PS13 = 0.5F*daz - 0.5F*dt*gz_b;
    const ftype PS14 = -PS13*P[1][3];
    const ftype PS15 = -PS10*P[1][1] - PS12 + PS14 + PS7*P[1][10] + PS8*P[1][11] + PS9*P[1][12] + P[0][1];
    const ftype PS16 = -PS10*P[1][2];
    const ftype PS17 = PS13*P[2][3];
    const ftype PS18 = -PS11*P[2][2] + PS16 - PS17 + PS7*P[2][10] + PS8*P[2][11] + PS9*P[2][12] + P[0][2];
    const ftype PS19 = PS10*P[1][3];
    const ftype PS20 = -PS11*P[2][3];
    const ftype PS21 = -PS13*P[3][3] - PS19 + PS20 + PS7*P[3][10] + PS8*P[3][11] + PS9*P[3][12] + P[0][3];
    const ftype PS22 = PS8*P[10][11];
    const ftype PS23 = PS9*P[10][12];
    const ftype PS24 = -PS10*P[1][10] - PS11*P[2][10] - PS13*P[3][10] + PS22 + PS23 + PS7*P[10][10] + P[0][10];
    const ftype PS25 = PS7*P[10][11];
    const ftype PS26 = PS9*P[11][12];
    const ftype PS27 = -PS10*P[1][11] - PS11*P[2][11] - PS13*P[3][11] + PS25 + PS26 + PS8*P[11][11] + P[0][11];
    const ftype PS28 = PS7*P[10][12];
    const ftype PS29 = PS8*P[11][12];
    const ftype PS30 = -PS10*P[1][12] - PS11*P[2][12] - PS13*P[3][12] + PS28 + PS29 + PS9*P[12][12] + P[0][12];
    const ftype PS31 = PS10*P[0][1];
    const ftype PS32 = PS11*P[0][2];
    const ftype PS33 = PS13*P[0][3];
    const ftype PS34 = -PS31 - PS32 - PS33 + PS7*P[0][10] + PS8*P[0][11] + PS9*P[0][12] + P[0][0];
    const ftype PS35 = q2*q3;
    const ftype PS36 = q0*q1;
    const ftype PS37 = PS6*q0;
    const ftype PS38 = q1*q3;
    const ftype PS39 = q0*q2;
    const ftype PS40 = q1*q2;
    const ftype PS41 = q0*q3;
    const ftype PS42 = ay_b*dt;
    const ftype PS43 = PS42 - dvy;
    const ftype PS44 = PS43*q2;
    const ftype PS45 = az_b*dt;
    const ftype PS46 = PS45 - dvz;
    const ftype PS47 = PS46*q3;
    const ftype PS48 = PS44 + PS47;
    const ftype PS49 = 2*PS15;
    const ftype PS50 = PS46*q2;
    const ftype PS51 = PS43*q3;
    const ftype PS52 = PS50 - PS51;
    const ftype PS53 = 2*PS34;
    const ftype PS54 = PS46*q0;
    const ftype PS55 = PS43*q1;
    const ftype PS56 = ax_b*dt;
    const ftype PS57 = PS56 - dvx;
    const ftype PS58 = PS57*q2;
    const ftype PS59 = PS54 + PS55 - 2*PS58;
    const ftype PS60 = 2*PS18;
    const ftype PS61 = PS46*q1;
    const ftype PS62 = PS43*q0;
    const ftype PS63 = PS57*q3;
    const ftype PS64 = PS61 - PS62 - 2*PS63;
    const ftype PS65 = 2*PS21;
    const ftype PS66 = -PS10*P[1][13] - PS11*P[2][13] - PS13*P[3][13] + PS7*P[10][13] + PS8*P[11][13] + PS9*P[12][13] + P[0][13];
    const ftype PS67 = 2*PS2;
    const ftype PS68 = 2*PS4 - 1;
    const ftype PS69 = PS67 + PS68;
    const ftype PS70 = PS69*dt;
    const ftype PS71 = PS38 + PS39;
    const ftype PS72 = -PS10*P[1][15] - PS11*P[2][15] - PS13*P[3][15] + PS7*P[10][15] + PS8*P[11][15] + PS9*P[12][15] + P[0][15];
    const ftype PS73 = 2*dt;
    const ftype PS74 = PS72*PS73;
    const ftype PS75 = -PS40 + PS41;
    const ftype PS76 = -PS10*P[1][14] - PS11*P[2][14] - PS13*P[3][14] + PS7*P[10][14] + PS8*P[11][14] + PS9*P[12][14] + P[0][14];
    const ftype PS77 = PS73*PS76;
    const ftype PS78 = P[4][10]*dt;
    const ftype PS79 = 0.5F*q1;
    const ftype PS80 = P[4][11]*dt;
    const ftype PS81 = 0.5F*q2;
    const ftype PS82 = P[4][12]*dt;
    const ftype PS83 = 0.5F*q3;
    const ftype PS84 = -PS10*P[1][4] - PS11*P[2][4] - PS13*P[3][4] + PS78*PS79 + PS80*PS81 + PS82*PS83 + P[0][4];
    const ftype PS85 = PS57*q1;
    const ftype PS86 = PS47 + PS85;
    const ftype PS87 = -PS61 + PS63;
    const ftype PS88 = PS57*q0;
    const ftype PS89 = 2*PS51;
    const ftype PS90 = PS50 + PS88 - PS89;
    const ftype PS91 = -PS54 - 2*PS55 + PS58;
    const ftype PS92 = 2*PS0;
    const ftype PS93 = PS68 + PS92;
    const ftype PS94 = PS93*dt;
    const ftype PS95 = PS40 + PS41;
    const ftype PS96 = PS66*PS73;
    const ftype PS97 = -PS35 + PS36;
    const ftype PS98 = P[5][10]*dt;
    const ftype PS99 = P[5][11]*dt;
    const ftype PS100 = P[5][12]*dt;
    const ftype PS101 = -PS10*P[1][5] + PS100*PS83 - PS11*P[2][5] - PS13*P[3][5] + PS79*PS98 + PS81*PS99 + P[0][5];
    const ftype PS102 = PS44 + PS85;
    const ftype PS103 = PS55 - PS58;
    const ftype PS104 = -2*PS61 + PS62 + PS63;
    const ftype PS105 = -PS88;
    const ftype PS106 = PS105 - 2*PS50 + PS51;
    const ftype PS107 = PS67 + PS92 - 1;
    const ftype PS108 = PS107*dt;
    const ftype PS109 = PS35 + PS36;
    const ftype PS110 = -PS38 + PS39;
    const ftype PS111 = P[6][10]*dt;
    const ftype PS112 = P[6][11]*dt;
    const ftype PS113 = P[6][12]*dt;
    const ftype PS114 = -PS10*P[1][6] - PS11*P[2][6] + PS111*PS79 + PS112*PS81 + PS113*PS83 - PS13*P[3][6] + P[0][6];
    const ftype PS115 = powf(q0, 2);
    const ftype PS116 = PS13*P[0][2];
    const ftype PS117 = PS11*P[0][3];
    const ftype PS118 = PS10*P[0][0] + PS116 - PS117 - PS37*P[0][10] - PS8*P[0][12] + PS9*P[0][11] + P[0][1];
    const ftype PS119 = PS10*P[0][2];
    const ftype PS120 = PS119 + PS13*P[2][2] + PS20 - PS37*P[2][10] - PS8*P[2][12] + PS9*P[2][11] + P[1][2];
    const ftype PS121 = PS10*P[0][3];
    const ftype PS122 = -PS11*P[3][3] + PS121 + PS17 - PS37*P[3][10] - PS8*P[3][12] + PS9*P[3][11] + P[1][3];
    const ftype PS123 = -PS37*P[10][11];
    const ftype PS124 = PS10*P[0][11] - PS11*P[3][11] + PS123 + PS13*P[2][11] - PS29 + PS9*P[11][11] + P[1][11];
    const ftype PS125 = PS9*P[10][11];
    const ftype PS126 = PS8*P[10][12];
    const ftype PS127 = PS10*P[0][10] - PS11*P[3][10] + PS125 - PS126 + PS13*P[2][10] - PS37*P[10][10] + P[1][10];
    const ftype PS128 = -PS37*P[10][12];
    const ftype PS129 = PS10*P[0][12] - PS11*P[3][12] + PS128 + PS13*P[2][12] + PS26 - PS8*P[12][12] + P[1][12];
    const ftype PS130 = PS13*P[1][2];
    const ftype PS131 = PS11*P[1][3];
    const ftype PS132 = PS130 - PS131 + PS31 - PS37*P[1][10] - PS8*P[1][12] + PS9*P[1][11] + P[1][1];
    const ftype PS133 = 2*PS132;
    const ftype PS134 = 2*PS118;
    const ftype PS135 = 2*PS120;
    const ftype PS136 = 2*PS122;
    const ftype PS137 = PS10*P[0][13] - PS11*P[3][13] + PS13*P[2][13] - PS37*P[10][13] - PS8*P[12][13] + PS9*P[11][13] + P[1][13];
    const ftype PS138 = PS10*P[0][15] - PS11*P[3][15] + PS13*P[2][15] - PS37*P[10][15] - PS8*P[12][15] + PS9*P[11][15] + P[1][15];
    const ftype PS139 = PS138*PS73;
    const ftype PS140 = PS10*P[0][14] - PS11*P[3][14] + PS13*P[2][14] - PS37*P[10][14] - PS8*P[12][14] + PS9*P[11][14] + P[1][14];
    const ftype PS141 = PS140*PS73;
    const ftype PS142 = 0.5F*q0;
    const ftype PS143 = PS10*P[0][4] - PS11*P[3][4] + PS13*P[2][4] - PS142*PS78 + PS80*PS83 - PS81*PS82 + P[1][4];
    const ftype PS144 = PS137*PS73;
    const ftype PS145 = PS10*P[0][5] - PS100*PS81 - PS11*P[3][5] + PS13*P[2][5] - PS142*PS98 + PS83*PS99 + P[1][5];
    const ftype PS146 = PS10*P[0][6] - PS11*P[3][6] - PS111*PS142 + PS112*PS83 - PS113*PS81 + PS13*P[2][6] + P[1][6];
    const ftype PS147 = PS10*P[3][3] + PS117 + PS14 - PS37*P[3][11] + PS7*P[3][12] - PS9*P[3][10] + P[2][3];
    const ftype PS148 = PS13*P[0][1];
    const ftype PS149 = PS11*P[0][0] + PS121 - PS148 - PS37*P[0][11] + PS7*P[0][12] - PS9*P[0][10] + P[0][2];
    const ftype PS150 = PS11*P[0][1];
    const ftype PS151 = -PS13*P[1][1] + PS150 + PS19 - PS37*P[1][11] + PS7*P[1][12] - PS9*P[1][10] + P[1][2];
    const ftype PS152 = -PS37*P[11][12];
    const ftype PS153 = PS10*P[3][12] + PS11*P[0][12] - PS13*P[1][12] + PS152 - PS23 + PS7*P[12][12] + P[2][12];
    const ftype PS154 = PS7*P[11][12];
    const ftype PS155 = PS10*P[3][11] + PS11*P[0][11] - PS125 - PS13*P[1][11] + PS154 - PS37*P[11][11] + P[2][11];
    const ftype PS156 = PS10*P[3][10] + PS11*P[0][10] + PS123 - PS13*P[1][10] + PS28 - PS9*P[10][10] + P[2][10];
    const ftype PS157 = PS10*P[2][3];
    const ftype PS158 = -PS130 + PS157 + PS32 - PS37*P[2][11] + PS7*P[2][12] - PS9*P[2][10] + P[2][2];
    const ftype PS159 = 2*PS151;
    const ftype PS160 = 2*PS149;
    const ftype PS161 = 2*PS158;
    const ftype PS162 = 2*PS147;
    const ftype PS163 = PS10*P[3][13] + PS11*P[0][13] - PS13*P[1][13] - PS37*P[11][13] + PS7*P[12][13] - PS9*P[10][13] + P[2][13];
    const ftype PS164 = PS10*P[3][15] + PS11*P[0][15] - PS13*P[1][15] - PS37*P[11][15] + PS7*P[12][15] - PS9*P[10][15] + P[2][15];
    const ftype PS165 = PS164*PS73;
    const ftype PS166 = PS10*P[3][14] + PS11*P[0][14] - PS13*P[1][14] - PS37*P[11][14] + PS7*P[12][14] - PS9*P[10][14] + P[2][14];
    const ftype PS167 = PS166*PS73;
    const ftype PS168 = PS10*P[3][4] + PS11*P[0][4] - PS13*P[1][4] - PS142*PS80 - PS78*PS83 + PS79*PS82 + P[2][4];
    const ftype PS169 = PS163*PS73;
    const ftype PS170 = PS10*P[3][5] + PS100*PS79 + PS11*P[0][5] - PS13*P[1][5] - PS142*PS99 - PS83*PS98 + P[2][5];
    const ftype PS171 = PS10*P[3][6] + PS11*P[0][6] - PS111*PS83 - PS112*PS142 + PS113*PS79 - PS13*P[1][6] + P[2][6];
    const ftype PS172 = PS11*P[1][1] + PS148 + PS16 - PS37*P[1][12] - PS7*P[1][11] + PS8*P[1][10] + P[1][3];
    const ftype PS173 = -PS119 + PS13*P[0][0] + PS150 - PS37*P[0][12] - PS7*P[0][11] + PS8*P[0][10] + P[0][3];
    const ftype PS174 = -PS10*P[2][2] + PS116 + PS12 - PS37*P[2][12] - PS7*P[2][11] + PS8*P[2][10] + P[2][3];
    const ftype PS175 = -PS10*P[2][10] + PS11*P[1][10] + PS128 + PS13*P[0][10] - PS25 + PS8*P[10][10] + P[3][10];
    const ftype PS176 = -PS10*P[2][12] + PS11*P[1][12] + PS126 + PS13*P[0][12] - PS154 - PS37*P[12][12] + P[3][12];
    const ftype PS177 = -PS10*P[2][11] + PS11*P[1][11] + PS13*P[0][11] + PS152 + PS22 - PS7*P[11][11] + P[3][11];
    const ftype PS178 = PS131 - PS157 + PS33 - PS37*P[3][12] - PS7*P[3][11] + PS8*P[3][10] + P[3][3];
    const ftype PS179 = 2*PS172;
    const ftype PS180 = 2*PS173;
    const ftype PS181 = 2*PS174;
    const ftype PS182 = 2*PS178;
    const ftype PS183 = -PS10*P[2][13] + PS11*P[1][13] + PS13*P[0][13] - PS37*P[12][13] - PS7*P[11][13] + PS8*P[10][13] + P[3][13];
    const ftype PS184 = -PS10*P[2][15] + PS11*P[1][15] + PS13*P[0][15] - PS37*P[12][15] - PS7*P[11][15] + PS8*P[10][15] + P[3][15];
    const ftype PS185 = PS184*PS73;
    const ftype PS186 = -PS10*P[2][14] + PS11*P[1][14] + PS13*P[0][14] - PS37*P[12][14] - PS7*P[11][14] + PS8*P[10][14] + P[3][14];
    const ftype PS187 = PS186*PS73;
    const ftype PS188 = -PS10*P[2][4] + PS11*P[1][4] + PS13*P[0][4] - PS142*PS82 + PS78*PS81 - PS79*PS80 + P[3][4];
    const ftype PS189 = PS183*PS73;
    const ftype PS190 = -PS10*P[2][5] - PS100*PS142 + PS11*P[1][5] + PS13*P[0][5] - PS79*PS99 + PS81*PS98 + P[3][5];
    const ftype PS191 = -PS10*P[2][6] + PS11*P[1][6] + PS111*PS81 - PS112*PS79 - PS113*PS142 + PS13*P[0][6] + P[3][6];
    const ftype PS192 = PS71*PS73;
    const ftype PS193 = PS73*PS75;
    const ftype PS194 = 2*PS48;
    const ftype PS195 = 2*PS52;
    const ftype PS196 = 2*PS59;
    const ftype PS197 = 2*PS64;
    const ftype PS198 = -PS192*P[1][15] + PS193*P[1][14] - PS194*P[1][1] - PS195*P[0][1] - PS196*P[1][2] - PS197*P[1][3] + PS70*P[1][13] + P[1][4];
    const ftype PS199 = -PS192*P[0][15] + PS193*P[0][14] - PS194*P[0][1] - PS195*P[0][0] - PS196*P[0][2] - PS197*P[0][3] + PS70*P[0][13] + P[0][4];
    const ftype PS200 = -PS192*P[2][15] + PS193*P[2][14] - PS194*P[1][2] - PS195*P[0][2] - PS196*P[2][2] - PS197*P[2][3] + PS70*P[2][13] + P[2][4];
    const ftype PS201 = -PS192*P[3][15] + PS193*P[3][14] - PS194*P[1][3] - PS195*P[0][3] - PS196*P[2][3] - PS197*P[3][3] + PS70*P[3][13] + P[3][4];
    const ftype PS202 = 4*dvyVar;
    const ftype PS203 = 4*dvzVar;
    const ftype PS204 = -PS192*P[13][15] + PS193*P[13][14] - PS194*P[1][13] - PS195*P[0][13] - PS196*P[2][13] - PS197*P[3][13] + PS70*P[13][13] + P[4][13];
    const ftype PS205 = -PS192*P[15][15] + PS193*P[14][15] - PS194*P[1][15] - PS195*P[0][15] - PS196*P[2][15] - PS197*P[3][15] + PS70*P[13][15] + P[4][15];
    const ftype PS206 = -PS192*P[14][15] + PS193*P[14][14] - PS194*P[1][14] - PS195*P[0][14] - PS196*P[2][14] - PS197*P[3][14] + PS70*P[13][14] + P[4][14];
    const ftype PS207 = P[4][15]*dt;
    const ftype PS208 = 2*PS71;
    const ftype PS209 = P[4][14]*dt;
    const ftype PS210 = 2*PS75;
    const ftype PS211 = P[4][13]*dt;
    const ftype PS212 = -PS194*P[1][4] - PS195*P[0][4] - PS196*P[2][4] - PS197*P[3][4] - PS207*PS208 + PS209*PS210 + PS211*PS69 + P[4][4];
    const ftype PS213 = 2*PS200;
    const ftype PS214 = 2*PS199;
    const ftype PS215 = 2*PS201;
    const ftype PS216 = 2*PS198;
    const ftype PS217 = PS204*PS73;
    const ftype PS218 = 2*PS95;
    const ftype PS219 = PS69*dvxVar;
    const ftype PS220 = PS73*PS97;
    const ftype PS221 = PS93*dvyVar;
    const ftype PS222 = P[5][15]*dt;
    const ftype PS223 = P[5][14]*dt;
    const ftype PS224 = P[5][13]*dt;
    const ftype PS225 = -PS194*P[1][5] - PS195*P[0][5] - PS196*P[2][5] - PS197*P[3][5] - PS208*PS222 + PS210*PS223 + PS224*PS69 + P[4][5];
    const ftype PS226 = PS109*PS73;
    const ftype PS227 = PS107*dvzVar;
    const ftype PS228 = 2*PS110;
    const ftype PS229 = P[6][15]*dt;
    const ftype PS230 = P[6][14]*dt;
    const ftype PS231 = P[6][13]*dt;
    const ftype PS232 = -PS194*P[1][6] - PS195*P[0][6] - PS196*P[2][6] - PS197*P[3][6] - PS208*PS229 + PS210*PS230 + PS231*PS69 + P[4][6];
    const ftype PS233 = PS218*PS224;
    const ftype PS234 = 4*dvxVar;
    const ftype PS235 = 2*PS97;
    const ftype PS236 = PS222*PS235;
    const ftype PS237 = PS223*PS93;
    const ftype PS238 = 2*PS86;
    const ftype PS239 = PS238*P[2][5];
    const ftype PS240 = 2*PS87;
    const ftype PS241 = 2*PS90;
    const ftype PS242 = 2*PS91;
    const ftype PS243 = PS73*PS95;
    const ftype PS244 = PS243*P[13][13];
    const ftype PS245 = PS220*P[13][15];
    const ftype PS246 = PS94*P[13][14];
    const ftype PS247 = -PS45 + dvz;
    const ftype PS248 = -PS56 + dvx;
    const ftype PS249 = 2*PS247*q1 - 2*PS248*q3;
    const ftype PS250 = PS249*P[0][13];
    const ftype PS251 = PS238*P[2][13];
    const ftype PS252 = 2*PS247*q0 - 2*PS248*q2 + 4*q1*(-PS42 + dvy);
    const ftype PS253 = PS252*P[1][13];
    const ftype PS254 = 2*PS105 - 2*PS50 + 2*PS89;
    const ftype PS255 = PS254*P[3][13];
    const ftype PS256 = PS244 - PS245 - PS246 + PS250 + PS251 + PS253 - PS255 - P[5][13];
    const ftype PS257 = PS243*P[13][15];
    const ftype PS258 = PS220*P[15][15];
    const ftype PS259 = PS94*P[14][15];
    const ftype PS260 = PS249*P[0][15];
    const ftype PS261 = PS238*P[2][15];
    const ftype PS262 = PS252*P[1][15];
    const ftype PS263 = PS254*P[3][15];
    const ftype PS264 = PS257 - PS258 - PS259 + PS260 + PS261 + PS262 - PS263 - P[5][15];
    const ftype PS265 = PS243*P[13][14];
    const ftype PS266 = PS220*P[14][15];
    const ftype PS267 = PS94*P[14][14];
    const ftype PS268 = PS249*P[0][14];
    const ftype PS269 = PS238*P[2][14];
    const ftype PS270 = PS252*P[1][14];
    const ftype PS271 = PS254*P[3][14];
    const ftype PS272 = PS265 - PS266 - PS267 + PS268 + PS269 + PS270 - PS271 - P[5][14];
    const ftype PS273 = -PS220*P[2][15] + PS238*P[2][2] + PS243*P[2][13] + PS249*P[0][2] + PS252*P[1][2] - PS254*P[2][3] - PS94*P[2][14] - P[2][5];
    const ftype PS274 = -PS220*P[0][15] + PS238*P[0][2] + PS243*P[0][13] + PS249*P[0][0] + PS252*P[0][1] - PS254*P[0][3] - PS94*P[0][14] - P[0][5];
    const ftype PS275 = -PS220*P[3][15] + PS238*P[2][3] + PS243*P[3][13] + PS249*P[0][3] + PS252*P[1][3] - PS254*P[3][3] - PS94*P[3][14] - P[3][5];
    const ftype PS276 = -PS220*P[1][15] + PS238*P[1][2] + PS243*P[1][13] + PS249*P[0][1] + PS252*P[1][1] - PS254*P[1][3] - PS94*P[1][14] - P[1][5];
    const ftype PS277 = PS218*PS231;
    const ftype PS278 = PS229*PS235;
    const ftype PS279 = PS230*PS93;
    const ftype PS280 = PS238*P[2][6];
    const ftype PS281 = 2*PS109;
    const ftype PS282 = PS110*PS73;
    const ftype PS283 = 2*PS102;
    const ftype PS284 = 2*PS103;
    const ftype PS285 = 2*PS104;
    const ftype PS286 = 2*PS106;
    const ftype PS287 = PS108*P[15][15] - PS226*P[14][15] + PS282*P[13][15] - PS283*P[3][15] - PS284*P[0][15] - PS285*P[1][15] - PS286*P[2][15] + P[6][15];
    const ftype PS288 = PS108*P[14][15] - PS226*P[14][14] + PS282*P[13][14] - PS283*P[3][14] - PS284*P[0][14] - PS285*P[1][14] - PS286*P[2][14] + P[6][14];
    const ftype PS289 = PS108*P[13][15] - PS226*P[13][14] + PS282*P[13][13] - PS283*P[3][13] - PS284*P[0][13] - PS285*P[1][13] - PS286*P[2][13] + P[6][13];
    const ftype PS290 = PS107*PS229 + PS228*PS231 - PS230*PS281 - PS283*P[3][6] - PS284*P[0][6] - PS285*P[1][6] - PS286*P[2][6] + P[6][6];

    nextP[0][0] = PS0*PS1 - PS10*PS15 - PS11*PS18 - PS13*PS21 + PS2*PS3 + PS24*PS7 + PS27*PS8 + PS30*PS9 + PS34 + PS4*PS5;
    nextP[0][1] = -PS1*PS36 + PS10*PS34 - PS11*PS21 + PS13*PS18 + PS15 - PS24*PS37 + PS27*PS9 + PS3*PS35 - PS30*PS8 - PS35*PS5;
    nextP[1][1] = PS1*PS115 + PS10*PS118 - PS11*PS122 + PS120*PS13 + PS124*PS9 - PS127*PS37 - PS129*PS8 + PS132 + PS2*PS5 + PS3*PS4;
    nextP[0][2] = -PS1*PS38 + PS10*PS21 + PS11*PS34 - PS13*PS15 + PS18 - PS24*PS9 - PS27*PS37 - PS3*PS39 + PS30*PS7 + PS38*PS5;
    nextP[1][2] = PS1*PS41 + PS10*PS122 + PS11*PS118 + PS120 - PS124*PS37 - PS127*PS9 + PS129*PS7 - PS13*PS132 - PS3*PS41 - PS40*PS5;
    nextP[2][2] = PS0*PS5 + PS1*PS4 + PS10*PS147 + PS11*PS149 + PS115*PS3 - PS13*PS151 + PS153*PS7 - PS155*PS37 - PS156*PS9 + PS158;
    nextP[0][3] = PS1*PS40 - PS10*PS18 + PS11*PS15 + PS13*PS34 + PS21 + PS24*PS8 - PS27*PS7 - PS3*PS40 - PS30*PS37 - PS41*PS5;
    nextP[1][3] = -PS1*PS39 - PS10*PS120 + PS11*PS132 + PS118*PS13 + PS122 - PS124*PS7 + PS127*PS8 - PS129*PS37 - PS3*PS38 + PS39*PS5;
    nextP[2][3] = -PS1*PS35 - PS10*PS158 + PS11*PS151 + PS13*PS149 + PS147 - PS153*PS37 - PS155*PS7 + PS156*PS8 + PS3*PS36 - PS36*PS5;
    nextP[3][3] = PS0*PS3 + PS1*PS2 - PS10*PS174 + PS11*PS172 + PS115*PS5 + PS13*PS173 + PS175*PS8 - PS176*PS37 - PS177*PS7 + PS178;

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

    nextP[0][4] = -PS48*PS49 - PS52*PS53 - PS59*PS60 - PS64*PS65 + PS66*PS70 - PS71*PS74 + PS75*PS77 + PS84;
    nextP[1][4] = -PS133*PS48 - PS134*PS52 - PS135*PS59 - PS136*PS64 + PS137*PS70 - PS139*PS71 + PS141*PS75 + PS143;
    nextP[2][4] = -PS159*PS48 - PS160*PS52 - PS161*PS59 - PS162*PS64 + PS163*PS70 - PS165*PS71 + PS167*PS75 + PS168;
    nextP[3][4] = -PS179*PS48 - PS180*PS52 - PS181*PS59 - PS182*PS64 + PS183*PS70 - PS185*PS71 + PS187*PS75 + PS188;
    nextP[4][4] = -PS192*PS205 + PS193*PS206 - PS194*PS198 - PS195*PS199 - PS196*PS200 - PS197*PS201 + PS202*powf(PS75, 2) + PS203*powf(PS71, 2) + PS204*PS70 + PS212 + powf(PS69, 2)*dvxVar;
    nextP[0][5] = PS101 - PS49*PS91 - PS53*PS87 - PS60*PS86 - PS65*PS90 + PS74*PS97 + PS76*PS94 - PS95*PS96;
    nextP[1][5] = -PS133*PS91 - PS134*PS87 - PS135*PS86 - PS136*PS90 + PS139*PS97 + PS140*PS94 - PS144*PS95 + PS145;
    nextP[2][5] = -PS159*PS91 - PS160*PS87 - PS161*PS86 - PS162*PS90 + PS165*PS97 + PS166*PS94 - PS169*PS95 + PS170;
    nextP[3][5] = -PS179*PS91 - PS180*PS87 - PS181*PS86 - PS182*PS90 + PS185*PS97 + PS186*PS94 - PS189*PS95 + PS190;
    nextP[4][5] = -PS203*PS71*PS97 + PS205*PS220 + PS206*PS94 + PS210*PS221 - PS213*PS86 - PS214*PS87 - PS215*PS90 - PS216*PS91 - PS217*PS95 - PS218*PS219 + PS225;
    nextP[5][5] = PS203*powf(PS97, 2) - PS220*PS264 - PS233 + PS234*powf(PS95, 2) + PS236 + PS237 + PS238*PS273 - PS239 + PS240*PS274 - PS240*P[0][5] + PS241*PS275 - PS241*P[3][5] + PS242*PS276 - PS242*P[1][5] + PS243*PS256 - PS272*PS94 + powf(PS93, 2)*dvyVar + P[5][5];
    nextP[0][6] = -PS102*PS65 - PS103*PS53 - PS104*PS49 - PS106*PS60 + PS108*PS72 - PS109*PS77 + PS110*PS96 + PS114;
    nextP[1][6] = -PS102*PS136 - PS103*PS134 - PS104*PS133 - PS106*PS135 + PS108*PS138 - PS109*PS141 + PS110*PS144 + PS146;
    nextP[2][6] = -PS102*PS162 - PS103*PS160 - PS104*PS159 - PS106*PS161 + PS108*PS164 - PS109*PS167 + PS110*PS169 + PS171;
    nextP[3][6] = -PS102*PS182 - PS103*PS180 - PS104*PS179 - PS106*PS181 + PS108*PS184 - PS109*PS187 + PS110*PS189 + PS191;
    nextP[4][6] = -PS102*PS215 - PS103*PS214 - PS104*PS216 - PS106*PS213 + PS108*PS205 - PS109*PS202*PS75 + PS110*PS217 - PS206*PS226 - PS208*PS227 + PS219*PS228 + PS232;
    nextP[5][6] = -PS108*PS264 - PS110*PS234*PS95 - PS221*PS281 + PS226*PS272 + PS227*PS235 - PS240*P[0][6] - PS241*P[3][6] - PS242*P[1][6] - PS256*PS282 + PS273*PS286 + PS274*PS284 + PS275*PS283 + PS276*PS285 - PS277 + PS278 + PS279 - PS280 + P[5][6];
    nextP[6][6] = powf(PS107, 2)*dvzVar + PS108*PS287 + powf(PS109, 2)*PS202 + powf(PS110, 2)*PS234 - PS226*PS288 + PS282*PS289 - PS283*(PS108*P[3][15] - PS226*P[3][14] + PS282*P[3][13] - PS283*P[3][3] - PS284*P[0][3] - PS285*P[1][3] - PS286*P[2][3] + P[3][6]) - PS284*(PS108*P[0][15] - PS226*P[0][14] + PS282*P[0][13] - PS283*P[0][3] - PS284*P[0][0] - PS285*P[0][1] - PS286*P[0][2] + P[0][6]) - PS285*(PS108*P[1][15] - PS226*P[1][14] + PS282*P[1][13] - PS283*P[1][3] - PS284*P[0][1] - PS285*P[1][1] - PS286*P[1][2] + P[1][6]) - PS286*(PS108*P[2][15] - PS226*P[2][14] + PS282*P[2][13] - PS283*P[2][3] - PS284*P[0][2] - PS285*P[1][2] - PS286*P[2][2] + P[2][6]) + PS290;
    nextP[0][7] = -PS10*P[1][7] - PS11*P[2][7] - PS13*P[3][7] + PS7*P[7][10] + PS8*P[7][11] + PS84*dt + PS9*P[7][12] + P[0][7];
    nextP[1][7] = PS10*P[0][7] - PS11*P[3][7] + PS13*P[2][7] + PS143*dt - PS37*P[7][10] - PS8*P[7][12] + PS9*P[7][11] + P[1][7];
    nextP[2][7] = PS10*P[3][7] + PS11*P[0][7] - PS13*P[1][7] + PS168*dt - PS37*P[7][11] + PS7*P[7][12] - PS9*P[7][10] + P[2][7];
    nextP[3][7] = -PS10*P[2][7] + PS11*P[1][7] + PS13*P[0][7] + PS188*dt - PS37*P[7][12] - PS7*P[7][11] + PS8*P[7][10] + P[3][7];
    nextP[4][7] = -PS192*P[7][15] + PS193*P[7][14] - PS194*P[1][7] - PS195*P[0][7] - PS196*P[2][7] - PS197*P[3][7] + PS212*dt + PS70*P[7][13] + P[4][7];
    nextP[5][7] = PS220*P[7][15] - PS238*P[2][7] - PS240*P[0][7] - PS241*P[3][7] - PS242*P[1][7] - PS243*P[7][13] + PS94*P[7][14] + P[5][7] - dt*(-PS207*PS235 - PS209*PS93 + PS211*PS218 + PS238*P[2][4] + PS249*P[0][4] + PS252*P[1][4] - PS254*P[3][4] - P[4][5]);
    nextP[6][7] = PS108*P[7][15] - PS226*P[7][14] + PS282*P[7][13] - PS283*P[3][7] - PS284*P[0][7] - PS285*P[1][7] - PS286*P[2][7] + P[6][7] + dt*(PS107*PS207 - PS209*PS281 + PS211*PS228 - PS283*P[3][4] - PS284*P[0][4] - PS285*P[1][4] - PS286*P[2][4] + P[4][6]);
    nextP[7][7] = P[4][7]*dt + P[7][7] + dt*(P[4][4]*dt + P[4][7]);
    nextP[0][8] = -PS10*P[1][8] + PS101*dt - PS11*P[2][8] - PS13*P[3][8] + PS7*P[8][10] + PS8*P[8][11] + PS9*P[8][12] + P[0][8];
    nextP[1][8] = PS10*P[0][8] - PS11*P[3][8] + PS13*P[2][8] + PS145*dt - PS37*P[8][10] - PS8*P[8][12] + PS9*P[8][11] + P[1][8];
    nextP[2][8] = PS10*P[3][8] + PS11*P[0][8] - PS13*P[1][8] + PS170*dt - PS37*P[8][11] + PS7*P[8][12] - PS9*P[8][10] + P[2][8];
    nextP[3][8] = -PS10*P[2][8] + PS11*P[1][8] + PS13*P[0][8] + PS190*dt - PS37*P[8][12] - PS7*P[8][11] + PS8*P[8][10] + P[3][8];
    nextP[4][8] = -PS192*P[8][15] + PS193*P[8][14] - PS194*P[1][8] - PS195*P[0][8] - PS196*P[2][8] - PS197*P[3][8] + PS225*dt + PS70*P[8][13] + P[4][8];
    nextP[5][8] = PS220*P[8][15] - PS238*P[2][8] - PS240*P[0][8] - PS241*P[3][8] - PS242*P[1][8] - PS243*P[8][13] + PS94*P[8][14] + P[5][8] - dt*(PS233 - PS236 - PS237 + PS239 + PS249*P[0][5] + PS252*P[1][5] - PS254*P[3][5] - P[5][5]);
    nextP[6][8] = PS108*P[8][15] - PS226*P[8][14] + PS282*P[8][13] - PS283*P[3][8] - PS284*P[0][8] - PS285*P[1][8] - PS286*P[2][8] + P[6][8] + dt*(PS107*PS222 - PS223*PS281 + PS224*PS228 - PS283*P[3][5] - PS284*P[0][5] - PS285*P[1][5] - PS286*P[2][5] + P[5][6]);
    nextP[7][8] = P[4][8]*dt + P[7][8] + dt*(P[4][5]*dt + P[5][7]);
    nextP[8][8] = P[5][8]*dt + P[8][8] + dt*(P[5][5]*dt + P[5][8]);
    nextP[0][9] = -PS10*P[1][9] - PS11*P[2][9] + PS114*dt - PS13*P[3][9] + PS7*P[9][10] + PS8*P[9][11] + PS9*P[9][12] + P[0][9];
    nextP[1][9] = PS10*P[0][9] - PS11*P[3][9] + PS13*P[2][9] + PS146*dt - PS37*P[9][10] - PS8*P[9][12] + PS9*P[9][11] + P[1][9];
    nextP[2][9] = PS10*P[3][9] + PS11*P[0][9] - PS13*P[1][9] + PS171*dt - PS37*P[9][11] + PS7*P[9][12] - PS9*P[9][10] + P[2][9];
    nextP[3][9] = -PS10*P[2][9] + PS11*P[1][9] + PS13*P[0][9] + PS191*dt - PS37*P[9][12] - PS7*P[9][11] + PS8*P[9][10] + P[3][9];
    nextP[4][9] = -PS192*P[9][15] + PS193*P[9][14] - PS194*P[1][9] - PS195*P[0][9] - PS196*P[2][9] - PS197*P[3][9] + PS232*dt + PS70*P[9][13] + P[4][9];
    nextP[5][9] = PS220*P[9][15] - PS238*P[2][9] - PS240*P[0][9] - PS241*P[3][9] - PS242*P[1][9] - PS243*P[9][13] + PS94*P[9][14] + P[5][9] - dt*(PS249*P[0][6] + PS252*P[1][6] - PS254*P[3][6] + PS277 - PS278 - PS279 + PS280 - P[5][6]);
    nextP[6][9] = PS108*P[9][15] - PS226*P[9][14] + PS282*P[9][13] - PS283*P[3][9] - PS284*P[0][9] - PS285*P[1][9] - PS286*P[2][9] + PS290*dt + P[6][9];
    nextP[7][9] = P[4][9]*dt + P[7][9] + dt*(P[4][6]*dt + P[6][7]);
    nextP[8][9] = P[5][9]*dt + P[8][9] + dt*(P[5][6]*dt + P[6][8]);
    nextP[9][9] = P[6][9]*dt + P[9][9] + dt*(P[6][6]*dt + P[6][9]);

    if (stateIndexLim > 9) {
        nextP[0][10] = PS24;
        nextP[1][10] = PS127;
        nextP[2][10] = PS156;
        nextP[3][10] = PS175;
        nextP[4][10] = -PS192*P[10][15] + PS193*P[10][14] - PS194*P[1][10] - PS195*P[0][10] - PS196*P[2][10] - PS197*P[3][10] + PS70*P[10][13] + P[4][10];
        nextP[5][10] = PS220*P[10][15] - PS238*P[2][10] - PS243*P[10][13] - PS249*P[0][10] - PS252*P[1][10] + PS254*P[3][10] + PS94*P[10][14] + P[5][10];
        nextP[6][10] = PS108*P[10][15] - PS226*P[10][14] + PS282*P[10][13] - PS283*P[3][10] - PS284*P[0][10] - PS285*P[1][10] - PS286*P[2][10] + P[6][10];
        nextP[7][10] = PS78 + P[7][10];
        nextP[8][10] = PS98 + P[8][10];
        nextP[9][10] = PS111 + P[9][10];
        nextP[10][10] = P[10][10];
        nextP[0][11] = PS27;
        nextP[1][11] = PS124;
        nextP[2][11] = PS155;
        nextP[3][11] = PS177;
        nextP[4][11] = -PS192*P[11][15] + PS193*P[11][14] - PS194*P[1][11] - PS195*P[0][11] - PS196*P[2][11] - PS197*P[3][11] + PS70*P[11][13] + P[4][11];
        nextP[5][11] = PS220*P[11][15] - PS238*P[2][11] - PS243*P[11][13] - PS249*P[0][11] - PS252*P[1][11] + PS254*P[3][11] + PS94*P[11][14] + P[5][11];
        nextP[6][11] = PS108*P[11][15] - PS226*P[11][14] + PS282*P[11][13] - PS283*P[3][11] - PS284*P[0][11] - PS285*P[1][11] - PS286*P[2][11] + P[6][11];
        nextP[7][11] = PS80 + P[7][11];
        nextP[8][11] = PS99 + P[8][11];
        nextP[9][11] = PS112 + P[9][11];
        nextP[10][11] = P[10][11];
        nextP[11][11] = P[11][11];
        nextP[0][12] = PS30;
        nextP[1][12] = PS129;
        nextP[2][12] = PS153;
        nextP[3][12] = PS176;
        nextP[4][12] = -PS192*P[12][15] + PS193*P[12][14] - PS194*P[1][12] - PS195*P[0][12] - PS196*P[2][12] - PS197*P[3][12] + PS70*P[12][13] + P[4][12];
        nextP[5][12] = PS220*P[12][15] - PS238*P[2][12] - PS243*P[12][13] - PS249*P[0][12] - PS252*P[1][12] + PS254*P[3][12] + PS94*P[12][14] + P[5][12];
        nextP[6][12] = PS108*P[12][15] - PS226*P[12][14] + PS282*P[12][13] - PS283*P[3][12] - PS284*P[0][12] - PS285*P[1][12] - PS286*P[2][12] + P[6][12];
        nextP[7][12] = PS82 + P[7][12];
        nextP[8][12] = PS100 + P[8][12];
        nextP[9][12] = PS113 + P[9][12];
        nextP[10][12] = P[10][12];
        nextP[11][12] = P[11][12];
        nextP[12][12] = P[12][12];

        if (stateIndexLim > 12) {
            nextP[0][13] = PS66;
            nextP[1][13] = PS137;
            nextP[2][13] = PS163;
            nextP[3][13] = PS183;
            nextP[4][13] = PS204;
            nextP[5][13] = -PS244 + PS245 + PS246 - PS250 - PS251 - PS253 + PS255 + P[5][13];
            nextP[6][13] = PS289;
            nextP[7][13] = PS211 + P[7][13];
            nextP[8][13] = PS224 + P[8][13];
            nextP[9][13] = PS231 + P[9][13];
            nextP[10][13] = P[10][13];
            nextP[11][13] = P[11][13];
            nextP[12][13] = P[12][13];
            nextP[13][13] = P[13][13];
            nextP[0][14] = PS76;
            nextP[1][14] = PS140;
            nextP[2][14] = PS166;
            nextP[3][14] = PS186;
            nextP[4][14] = PS206;
            nextP[5][14] = -PS265 + PS266 + PS267 - PS268 - PS269 - PS270 + PS271 + P[5][14];
            nextP[6][14] = PS288;
            nextP[7][14] = PS209 + P[7][14];
            nextP[8][14] = PS223 + P[8][14];
            nextP[9][14] = PS230 + P[9][14];
            nextP[10][14] = P[10][14];
            nextP[11][14] = P[11][14];
            nextP[12][14] = P[12][14];
            nextP[13][14] = P[13][14];
            nextP[14][14] = P[14][14];
            nextP[0][15] = PS72;
            nextP[1][15] = PS138;
            nextP[2][15] = PS164;
            nextP[3][15] = PS184;
            nextP[4][15] = PS205;
            nextP[5][15] = -PS257 + PS258 + PS259 - PS260 - PS261 - PS262 + PS263 + P[5][15];
            nextP[6][15] = PS287;
            nextP[7][15] = PS207 + P[7][15];
            nextP[8][15] = PS222 + P[8][15];
            nextP[9][15] = PS229 + P[9][15];
            nextP[10][15] = P[10][15];
            nextP[11][15] = P[11][15];
            nextP[12][15] = P[12][15];
            nextP[13][15] = P[13][15];
            nextP[14][15] = P[14][15];
            nextP[15][15] = P[15][15];

            if (stateIndexLim > 15) {
                nextP[0][16] = -PS10*P[1][16] - PS11*P[2][16] - PS13*P[3][16] + PS7*P[10][16] + PS8*P[11][16] + PS9*P[12][16] + P[0][16];
                nextP[1][16] = PS10*P[0][16] - PS11*P[3][16] + PS13*P[2][16] - PS37*P[10][16] - PS8*P[12][16] + PS9*P[11][16] + P[1][16];
                nextP[2][16] = PS10*P[3][16] + PS11*P[0][16] - PS13*P[1][16] - PS37*P[11][16] + PS7*P[12][16] - PS9*P[10][16] + P[2][16];
                nextP[3][16] = -PS10*P[2][16] + PS11*P[1][16] + PS13*P[0][16] - PS37*P[12][16] - PS7*P[11][16] + PS8*P[10][16] + P[3][16];
                nextP[4][16] = -PS192*P[15][16] + PS193*P[14][16] - PS194*P[1][16] - PS195*P[0][16] - PS196*P[2][16] - PS197*P[3][16] + PS70*P[13][16] + P[4][16];
                nextP[5][16] = PS220*P[15][16] - PS238*P[2][16] - PS243*P[13][16] - PS249*P[0][16] - PS252*P[1][16] + PS254*P[3][16] + PS94*P[14][16] + P[5][16];
                nextP[6][16] = PS108*P[15][16] - PS226*P[14][16] + PS282*P[13][16] - PS283*P[3][16] - PS284*P[0][16] - PS285*P[1][16] - PS286*P[2][16] + P[6][16];
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
                nextP[0][17] = -PS10*P[1][17] - PS11*P[2][17] - PS13*P[3][17] + PS7*P[10][17] + PS8*P[11][17] + PS9*P[12][17] + P[0][17];
                nextP[1][17] = PS10*P[0][17] - PS11*P[3][17] + PS13*P[2][17] - PS37*P[10][17] - PS8*P[12][17] + PS9*P[11][17] + P[1][17];
                nextP[2][17] = PS10*P[3][17] + PS11*P[0][17] - PS13*P[1][17] - PS37*P[11][17] + PS7*P[12][17] - PS9*P[10][17] + P[2][17];
                nextP[3][17] = -PS10*P[2][17] + PS11*P[1][17] + PS13*P[0][17] - PS37*P[12][17] - PS7*P[11][17] + PS8*P[10][17] + P[3][17];
                nextP[4][17] = -PS192*P[15][17] + PS193*P[14][17] - PS194*P[1][17] - PS195*P[0][17] - PS196*P[2][17] - PS197*P[3][17] + PS70*P[13][17] + P[4][17];
                nextP[5][17] = PS220*P[15][17] - PS238*P[2][17] - PS243*P[13][17] - PS249*P[0][17] - PS252*P[1][17] + PS254*P[3][17] + PS94*P[14][17] + P[5][17];
                nextP[6][17] = PS108*P[15][17] - PS226*P[14][17] + PS282*P[13][17] - PS283*P[3][17] - PS284*P[0][17] - PS285*P[1][17] - PS286*P[2][17] + P[6][17];
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
                nextP[0][18] = -PS10*P[1][18] - PS11*P[2][18] - PS13*P[3][18] + PS7*P[10][18] + PS8*P[11][18] + PS9*P[12][18] + P[0][18];
                nextP[1][18] = PS10*P[0][18] - PS11*P[3][18] + PS13*P[2][18] - PS37*P[10][18] - PS8*P[12][18] + PS9*P[11][18] + P[1][18];
                nextP[2][18] = PS10*P[3][18] + PS11*P[0][18] - PS13*P[1][18] - PS37*P[11][18] + PS7*P[12][18] - PS9*P[10][18] + P[2][18];
                nextP[3][18] = -PS10*P[2][18] + PS11*P[1][18] + PS13*P[0][18] - PS37*P[12][18] - PS7*P[11][18] + PS8*P[10][18] + P[3][18];
                nextP[4][18] = -PS192*P[15][18] + PS193*P[14][18] - PS194*P[1][18] - PS195*P[0][18] - PS196*P[2][18] - PS197*P[3][18] + PS70*P[13][18] + P[4][18];
                nextP[5][18] = PS220*P[15][18] - PS238*P[2][18] - PS243*P[13][18] - PS249*P[0][18] - PS252*P[1][18] + PS254*P[3][18] + PS94*P[14][18] + P[5][18];
                nextP[6][18] = PS108*P[15][18] - PS226*P[14][18] + PS282*P[13][18] - PS283*P[3][18] - PS284*P[0][18] - PS285*P[1][18] - PS286*P[2][18] + P[6][18];
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
                nextP[0][19] = -PS10*P[1][19] - PS11*P[2][19] - PS13*P[3][19] + PS7*P[10][19] + PS8*P[11][19] + PS9*P[12][19] + P[0][19];
                nextP[1][19] = PS10*P[0][19] - PS11*P[3][19] + PS13*P[2][19] - PS37*P[10][19] - PS8*P[12][19] + PS9*P[11][19] + P[1][19];
                nextP[2][19] = PS10*P[3][19] + PS11*P[0][19] - PS13*P[1][19] - PS37*P[11][19] + PS7*P[12][19] - PS9*P[10][19] + P[2][19];
                nextP[3][19] = -PS10*P[2][19] + PS11*P[1][19] + PS13*P[0][19] - PS37*P[12][19] - PS7*P[11][19] + PS8*P[10][19] + P[3][19];
                nextP[4][19] = -PS192*P[15][19] + PS193*P[14][19] - PS194*P[1][19] - PS195*P[0][19] - PS196*P[2][19] - PS197*P[3][19] + PS70*P[13][19] + P[4][19];
                nextP[5][19] = PS220*P[15][19] - PS238*P[2][19] - PS243*P[13][19] - PS249*P[0][19] - PS252*P[1][19] + PS254*P[3][19] + PS94*P[14][19] + P[5][19];
                nextP[6][19] = PS108*P[15][19] - PS226*P[14][19] + PS282*P[13][19] - PS283*P[3][19] - PS284*P[0][19] - PS285*P[1][19] - PS286*P[2][19] + P[6][19];
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
                nextP[0][20] = -PS10*P[1][20] - PS11*P[2][20] - PS13*P[3][20] + PS7*P[10][20] + PS8*P[11][20] + PS9*P[12][20] + P[0][20];
                nextP[1][20] = PS10*P[0][20] - PS11*P[3][20] + PS13*P[2][20] - PS37*P[10][20] - PS8*P[12][20] + PS9*P[11][20] + P[1][20];
                nextP[2][20] = PS10*P[3][20] + PS11*P[0][20] - PS13*P[1][20] - PS37*P[11][20] + PS7*P[12][20] - PS9*P[10][20] + P[2][20];
                nextP[3][20] = -PS10*P[2][20] + PS11*P[1][20] + PS13*P[0][20] - PS37*P[12][20] - PS7*P[11][20] + PS8*P[10][20] + P[3][20];
                nextP[4][20] = -PS192*P[15][20] + PS193*P[14][20] - PS194*P[1][20] - PS195*P[0][20] - PS196*P[2][20] - PS197*P[3][20] + PS70*P[13][20] + P[4][20];
                nextP[5][20] = PS220*P[15][20] - PS238*P[2][20] - PS243*P[13][20] - PS249*P[0][20] - PS252*P[1][20] + PS254*P[3][20] + PS94*P[14][20] + P[5][20];
                nextP[6][20] = PS108*P[15][20] - PS226*P[14][20] + PS282*P[13][20] - PS283*P[3][20] - PS284*P[0][20] - PS285*P[1][20] - PS286*P[2][20] + P[6][20];
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
                nextP[0][21] = -PS10*P[1][21] - PS11*P[2][21] - PS13*P[3][21] + PS7*P[10][21] + PS8*P[11][21] + PS9*P[12][21] + P[0][21];
                nextP[1][21] = PS10*P[0][21] - PS11*P[3][21] + PS13*P[2][21] - PS37*P[10][21] - PS8*P[12][21] + PS9*P[11][21] + P[1][21];
                nextP[2][21] = PS10*P[3][21] + PS11*P[0][21] - PS13*P[1][21] - PS37*P[11][21] + PS7*P[12][21] - PS9*P[10][21] + P[2][21];
                nextP[3][21] = -PS10*P[2][21] + PS11*P[1][21] + PS13*P[0][21] - PS37*P[12][21] - PS7*P[11][21] + PS8*P[10][21] + P[3][21];
                nextP[4][21] = -PS192*P[15][21] + PS193*P[14][21] - PS194*P[1][21] - PS195*P[0][21] - PS196*P[2][21] - PS197*P[3][21] + PS70*P[13][21] + P[4][21];
                nextP[5][21] = PS220*P[15][21] - PS238*P[2][21] - PS243*P[13][21] - PS249*P[0][21] - PS252*P[1][21] + PS254*P[3][21] + PS94*P[14][21] + P[5][21];
                nextP[6][21] = PS108*P[15][21] - PS226*P[14][21] + PS282*P[13][21] - PS283*P[3][21] - PS284*P[0][21] - PS285*P[1][21] - PS286*P[2][21] + P[6][21];
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
                    nextP[0][22] = -PS10*P[1][22] - PS11*P[2][22] - PS13*P[3][22] + PS7*P[10][22] + PS8*P[11][22] + PS9*P[12][22] + P[0][22];
                    nextP[1][22] = PS10*P[0][22] - PS11*P[3][22] + PS13*P[2][22] - PS37*P[10][22] - PS8*P[12][22] + PS9*P[11][22] + P[1][22];
                    nextP[2][22] = PS10*P[3][22] + PS11*P[0][22] - PS13*P[1][22] - PS37*P[11][22] + PS7*P[12][22] - PS9*P[10][22] + P[2][22];
                    nextP[3][22] = -PS10*P[2][22] + PS11*P[1][22] + PS13*P[0][22] - PS37*P[12][22] - PS7*P[11][22] + PS8*P[10][22] + P[3][22];
                    nextP[4][22] = -PS192*P[15][22] + PS193*P[14][22] - PS194*P[1][22] - PS195*P[0][22] - PS196*P[2][22] - PS197*P[3][22] + PS70*P[13][22] + P[4][22];
                    nextP[5][22] = PS220*P[15][22] - PS238*P[2][22] - PS243*P[13][22] - PS249*P[0][22] - PS252*P[1][22] + PS254*P[3][22] + PS94*P[14][22] + P[5][22];
                    nextP[6][22] = PS108*P[15][22] - PS226*P[14][22] + PS282*P[13][22] - PS283*P[3][22] - PS284*P[0][22] - PS285*P[1][22] - PS286*P[2][22] + P[6][22];
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
                    nextP[0][23] = -PS10*P[1][23] - PS11*P[2][23] - PS13*P[3][23] + PS7*P[10][23] + PS8*P[11][23] + PS9*P[12][23] + P[0][23];
                    nextP[1][23] = PS10*P[0][23] - PS11*P[3][23] + PS13*P[2][23] - PS37*P[10][23] - PS8*P[12][23] + PS9*P[11][23] + P[1][23];
                    nextP[2][23] = PS10*P[3][23] + PS11*P[0][23] - PS13*P[1][23] - PS37*P[11][23] + PS7*P[12][23] - PS9*P[10][23] + P[2][23];
                    nextP[3][23] = -PS10*P[2][23] + PS11*P[1][23] + PS13*P[0][23] - PS37*P[12][23] - PS7*P[11][23] + PS8*P[10][23] + P[3][23];
                    nextP[4][23] = -PS192*P[15][23] + PS193*P[14][23] - PS194*P[1][23] - PS195*P[0][23] - PS196*P[2][23] - PS197*P[3][23] + PS70*P[13][23] + P[4][23];
                    nextP[5][23] = PS220*P[15][23] - PS238*P[2][23] - PS243*P[13][23] - PS249*P[0][23] - PS252*P[1][23] + PS254*P[3][23] + PS94*P[14][23] + P[5][23];
                    nextP[6][23] = PS108*P[15][23] - PS226*P[14][23] + PS282*P[13][23] - PS283*P[3][23] - PS284*P[0][23] - PS285*P[1][23] - PS286*P[2][23] + P[6][23];
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
    if (!inhibitAccelBiasStates) {
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

    // if vibration affected use sensor observation variances to set a floor on the state variances
    if (badIMUdata) {
        P[6][6] = fmaxF(P[6][6], sq(frontend->_gpsVertVelNoise));
        P[9][9] = fmaxF(P[9][9], sq(frontend->_baroAltNoise));
    } else if (P[6][6] < VEL_STATE_MIN_VARIANCE) {
        // handle collapse of the vertical velocity variance
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

    if (!inhibitGyroBiasStates) {
        for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,sq(0.175));
    } else {
        zeroCols(P,10,12);
        zeroRows(P,10,12);
    }

    const ftype minSafeStateVar = 5E-7;
    if (!inhibitAccelBiasStates) {

        // Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
        ftype maxStateVar = 0.0F;
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
        ftype minAllowedStateVar = fmaxF(0.01f * maxStateVar, minSafeStateVar);
        for (uint8_t stateIndex=13; stateIndex<=15; stateIndex++) {
            P[stateIndex][stateIndex] = constrain_ftype(P[stateIndex][stateIndex], minAllowedStateVar, sq(1.0f));
        }

        // If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
        if (resetRequired) {
            // reset all delta velocity bias covariances
            zeroCols(P,13,15);
            zeroRows(P,13,15);
            // set all delta velocity bias variances to initial values and zero bias states
            P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim);
            P[14][14] = P[13][13];
            P[15][15] = P[13][13];
            stateStruct.accel_bias.zero();
        }

    } else {
        zeroCols(P,13,15);
        zeroRows(P,13,15);
        // set all delta velocity bias variances to a margin above the minimum safe value
        for (uint8_t i=0; i<=2; i++) {
            const uint8_t stateIndex = i + 13;
            P[stateIndex][stateIndex] = fmaxF(P[stateIndex][stateIndex], minSafeStateVar * 10.0F);
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
        for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,WIND_VEL_VARIANCE_MAX);
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
    for (uint8_t i=10; i<=12; i++) statesArray[i] = constrain_ftype(statesArray[i],-GYRO_BIAS_LIMIT,GYRO_BIAS_LIMIT);
    // the accelerometer bias limit is controlled by a user adjustable parameter
    for (uint8_t i=13; i<=15; i++) statesArray[i] = constrain_ftype(statesArray[i],-frontend->_accBiasLim,frontend->_accBiasLim);
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
        stateStruct.quat.to_euler(eulerAngles);
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
