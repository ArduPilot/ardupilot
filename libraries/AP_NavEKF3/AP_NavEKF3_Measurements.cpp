#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_InternalError/AP_InternalError.h>

/********************************************************
*              OPT FLOW AND RANGE FINDER                *
********************************************************/

// Read the range finder and take new measurements if available
// Apply a median filter
void NavEKF3_core::readRangeFinder(void)
{
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;
    // get theoretical correct range when the vehicle is on the ground
    // don't allow range to go below 5cm because this can cause problems with optical flow processing
    const auto *_rng = dal.rangefinder();
    if (_rng == nullptr) {
        return;
    }
    rngOnGnd = MAX(_rng->ground_clearance_cm_orient(ROTATION_PITCH_270) * 0.01f, 0.05f);

    // limit update rate to maximum allowed by data buffers
    if ((imuSampleTime_ms - lastRngMeasTime_ms) > frontend->sensorIntervalMin_ms) {

        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms =  imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        // use data from two range finders if available

        for (uint8_t sensorIndex = 0; sensorIndex < ARRAY_SIZE(rngMeasIndex); sensorIndex++) {
            const auto *sensor = _rng->get_backend(sensorIndex);
            if (sensor == nullptr) {
                continue;
            }
            if ((sensor->orientation() == ROTATION_PITCH_270) && (sensor->status() == AP_DAL_RangeFinder::Status::Good)) {
                rngMeasIndex[sensorIndex] ++;
                if (rngMeasIndex[sensorIndex] > 2) {
                    rngMeasIndex[sensorIndex] = 0;
                }
                storedRngMeasTime_ms[sensorIndex][rngMeasIndex[sensorIndex]] = imuSampleTime_ms - 25;
                storedRngMeas[sensorIndex][rngMeasIndex[sensorIndex]] = sensor->distance_cm() * 0.01f;
            } else {
                continue;
            }

            // check for three fresh samples
            bool sampleFresh[DOWNWARD_RANGEFINDER_MAX_INSTANCES][3] = {};
            for (uint8_t index = 0; index <= 2; index++) {
                sampleFresh[sensorIndex][index] = (imuSampleTime_ms - storedRngMeasTime_ms[sensorIndex][index]) < 500;
            }

            // find the median value if we have three fresh samples
            if (sampleFresh[sensorIndex][0] && sampleFresh[sensorIndex][1] && sampleFresh[sensorIndex][2]) {
                if (storedRngMeas[sensorIndex][0] > storedRngMeas[sensorIndex][1]) {
                    minIndex = 1;
                    maxIndex = 0;
                } else {
                    minIndex = 0;
                    maxIndex = 1;
                }
                if (storedRngMeas[sensorIndex][2] > storedRngMeas[sensorIndex][maxIndex]) {
                    midIndex = maxIndex;
                } else if (storedRngMeas[sensorIndex][2] < storedRngMeas[sensorIndex][minIndex]) {
                    midIndex = minIndex;
                } else {
                    midIndex = 2;
                }

                // don't allow time to go backwards
                if (storedRngMeasTime_ms[sensorIndex][midIndex] > rangeDataNew.time_ms) {
                    rangeDataNew.time_ms = storedRngMeasTime_ms[sensorIndex][midIndex];
                }

                // limit the measured range to be no less than the on-ground range
                rangeDataNew.rng = MAX(storedRngMeas[sensorIndex][midIndex],rngOnGnd);

                // get position in body frame for the current sensor
                rangeDataNew.sensor_idx = sensorIndex;

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            } else if (onGround && ((imuSampleTime_ms - rngValidMeaTime_ms) > 200)) {
                // before takeoff we assume on-ground range value if there is no data
                rangeDataNew.time_ms = imuSampleTime_ms;
                rangeDataNew.rng = rngOnGnd;

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            }
        }
    }
}

void NavEKF3_core::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset)
{
#if EK3_FEATURE_BODY_ODOM
    // protect against NaN
    if (isnan(quality) || delPos.is_nan() || delAng.is_nan() || isnan(delTime) || posOffset.is_nan()) {
        return;
    }

    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if (((timeStamp_ms - bodyOdmMeasTime_ms) < frontend->sensorIntervalMin_ms) || (delTime < dtEkfAvg) || !statesInitialised) {
        return;
    }

    // subtract delay from timestamp
    timeStamp_ms -= delay_ms;

    bodyOdmDataNew.body_offset = posOffset.toftype();
    bodyOdmDataNew.vel = delPos.toftype() * (1.0/delTime);
    bodyOdmDataNew.time_ms = timeStamp_ms;
    bodyOdmDataNew.angRate = (delAng * (1.0/delTime)).toftype();
    bodyOdmMeasTime_ms = timeStamp_ms;

    // simple model of accuracy
    // TODO move this calculation outside of EKF into the sensor driver
    bodyOdmDataNew.velErr = frontend->_visOdmVelErrMin + (frontend->_visOdmVelErrMax - frontend->_visOdmVelErrMin) * (1.0f - 0.01f * quality);

    storedBodyOdm.push(bodyOdmDataNew);
#endif // EK3_FEATURE_BODY_ODOM
}

void NavEKF3_core::writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius)
{
#if EK3_FEATURE_BODY_ODOM
    // This is a simple hack to get wheel encoder data into the EKF and verify the interface sign conventions and units
    // It uses the exisiting body frame velocity fusion.
    // TODO implement a dedicated wheel odometry observation model

    // rate limiting to 50hz should be done by the caller
    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if ((delTime < dtEkfAvg) || !statesInitialised) {
        return;
    }

    wheel_odm_elements wheelOdmDataNew = {};
    wheelOdmDataNew.hub_offset = posOffset.toftype();
    wheelOdmDataNew.delAng = delAng;
    wheelOdmDataNew.radius = radius;
    wheelOdmDataNew.delTime = delTime;

    // because we are currently converting to an equivalent velocity measurement before fusing
    // the measurement time is moved back to the middle of the sampling period
    wheelOdmDataNew.time_ms = timeStamp_ms - (uint32_t)(500.0f * delTime);

    storedWheelOdm.push(wheelOdmDataNew);
#endif // EK3_FEATURE_BODY_ODOM
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF3_core::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride)
{
    // limit update rate to maximum allowed by sensor buffers
    if ((imuSampleTime_ms - flowMeaTime_ms) < frontend->sensorIntervalMin_ms) {
        return;
    }

    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    // reset the accumulated body delta angle and time
    // don't do the calculation if not enough time lapsed for a reliable body rate measurement
    if (delTimeOF > 0.01f) {
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_ftype((rawGyroRates.x - delAngBodyOF.x/delTimeOF),-0.1f,0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_ftype((rawGyroRates.y - delAngBodyOF.y/delTimeOF),-0.1f,0.1f);
        delAngBodyOF.zero();
        delTimeOF = 0.0f;
    }
    // by definition if this function is called, then flow measurements have been provided so we
    // need to run the optical flow takeoff detection
    detectOptFlowTakeoff();

    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor body rates for bias and write
        of_elements ofDataNew {};
        ofDataNew.bodyRadXYZ.x = rawGyroRates.x - flowGyroBias.x;
        ofDataNew.bodyRadXYZ.y = rawGyroRates.y - flowGyroBias.y;
        // the sensor interface doesn't provide a z axis rate so use the rate from the nav sensor instead
        if (delTimeOF > 0.001f) {
            // first preference is to use the rate averaged over the same sampling period as the flow sensor
            ofDataNew.bodyRadXYZ.z = delAngBodyOF.z / delTimeOF;
        } else if (imuDataNew.delAngDT > 0.001f){
            // second preference is to use most recent IMU data
            ofDataNew.bodyRadXYZ.z = imuDataNew.delAng.z / imuDataNew.delAngDT;
        } else {
            // third preference is use zero
            ofDataNew.bodyRadXYZ.z =  0.0f;
        }
        // write uncorrected flow rate measurements
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY = - rawFlowRates.toftype(); // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        // write the flow sensor position in body frame
        ofDataNew.body_offset = posOffset.toftype();
        // write the flow sensor height override
        ofDataNew.heightOverride = heightOverride;
        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + ofDataNew.bodyRadXYZ.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + ofDataNew.bodyRadXYZ.y;
        // record time last observation was received so we can detect loss of data elsewhere
        flowValidMeaTime_ms = imuSampleTime_ms;
        // estimate sample time of the measurement
        ofDataNew.time_ms = imuSampleTime_ms - frontend->_flowDelay_ms - frontend->flowTimeDeltaAvg_ms/2;
        // Correct for the average intersampling delay due to the filter updaterate
        ofDataNew.time_ms -= localFilterTimeStep_ms/2;
        // Prevent time delay exceeding age of oldest IMU data in the buffer
        ofDataNew.time_ms = MAX(ofDataNew.time_ms,imuDataDelayed.time_ms);
        // Save data to buffer
        storedOF.push(ofDataNew);
    }
}


/********************************************************
*                      MAGNETOMETER                     *
********************************************************/

// try changing compass, return true if a new compass is found
void NavEKF3_core::tryChangeCompass(void)
{
    const auto &compass = dal.compass();
    const uint8_t maxCount = compass.get_count();

    // search through the list of magnetometers
    for (uint8_t i=1; i<maxCount; i++) {
        uint8_t tempIndex = magSelectIndex + i;
        // loop back to the start index if we have exceeded the bounds
        if (tempIndex >= maxCount) {
            tempIndex -= maxCount;
        }
        // if the magnetometer is allowed to be used for yaw and has a different index, we start using it
        if (compass.healthy(tempIndex) && compass.use_for_yaw(tempIndex) && tempIndex != magSelectIndex) {
            magSelectIndex = tempIndex;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u switching to compass %u",(unsigned)imu_index,magSelectIndex);
            // reset the timeout flag and timer
            magTimeout = false;
            lastHealthyMagTime_ms = imuSampleTime_ms;
            // zero the learned magnetometer bias states
            stateStruct.body_magfield.zero();
            // clear the measurement buffer
            storedMag.reset();
            // clear the data waiting flag so that we do not use any data pending from the previous sensor
            magDataToFuse = false;
            // request a reset of the magnetic field states
            magStateResetRequest = true;
            // declare the field unlearned so that the reset request will be obeyed
            magFieldLearned = false;
            // reset body mag variances on next CovariancePrediction
            needMagBodyVarReset = true;
            return;
        }
    }
}

// check for new magnetometer data and update store measurements if available
void NavEKF3_core::readMagData()
{
    const auto &compass = dal.compass();

    if (!compass.available()) {
        allMagSensorsFailed = true;
        return;        
    }

    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    const uint8_t maxCount = compass.get_count();
    if (allMagSensorsFailed || (magTimeout && assume_zero_sideslip() && magSelectIndex >= maxCount-1 && inFlight)) {
        allMagSensorsFailed = true;
        return;
    }

    if (compass.learn_offsets_enabled()) {
        // while learning offsets keep all mag states reset
        InitialiseVariablesMag();
        wasLearningCompass_ms = imuSampleTime_ms;
    } else if (wasLearningCompass_ms != 0 && imuSampleTime_ms - wasLearningCompass_ms > 1000) {
        wasLearningCompass_ms = 0;
        // force a new yaw alignment 1s after learning completes. The
        // delay is to ensure any buffered mag samples are discarded
        yawAlignComplete = false;
        InitialiseVariablesMag();
    }

    // If the magnetometer has timed out (been rejected for too long), we find another magnetometer to use if available
    // Don't do this if we are on the ground because there can be magnetic interference and we need to know if there is a problem
    // before taking off. Don't do this within the first 30 seconds from startup because the yaw error could be due to large yaw gyro bias affsets
    // if the timeout is due to a sensor failure, then declare a timeout regardless of onground status
    if (maxCount > 1) {
        bool fusionTimeout = magTimeout && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000 && !(frontend->_affinity & EKF_AFFINITY_MAG);
        bool sensorTimeout = !compass.healthy(magSelectIndex) && imuSampleTime_ms - lastMagRead_ms > frontend->magFailTimeLimit_ms;
        if (fusionTimeout || sensorTimeout) {
            tryChangeCompass();
        }
    }

    // limit compass update rate to prevent high processor loading because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    if (use_compass() &&
        compass.healthy(magSelectIndex) &&
        ((compass.last_update_usec(magSelectIndex) - lastMagUpdate_us) > 1000 * frontend->sensorIntervalMin_ms)) {

        // detect changes to magnetometer offset parameters and reset states
        Vector3F nowMagOffsets = compass.get_offsets(magSelectIndex).toftype();
        bool changeDetected = lastMagOffsetsValid && (nowMagOffsets != lastMagOffsets);
        if (changeDetected) {
            // zero the learned magnetometer bias states
            stateStruct.body_magfield.zero();
            // clear the measurement buffer
            storedMag.reset();

            // reset body mag variances on next
            // CovariancePrediction. This copes with possible errors
            // in the new offsets
            needMagBodyVarReset = true;
        }
        lastMagOffsets = nowMagOffsets;
        lastMagOffsetsValid = true;

        // store time of last measurement update
        lastMagUpdate_us = compass.last_update_usec(magSelectIndex);

        // Magnetometer data at the current time horizon
        mag_elements magDataNew;

        // estimate of time magnetometer measurement was taken, allowing for delays
        magDataNew.time_ms = imuSampleTime_ms - frontend->magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        magDataNew.time_ms -= localFilterTimeStep_ms/2;

        // read compass data and scale to improve numerical conditioning
        magDataNew.mag = (compass.get_field(magSelectIndex) * 0.001f).toftype();

        // check for consistent data between magnetometers
        consistentMagData = compass.consistent();

        // save magnetometer measurement to buffer to be fused later
        storedMag.push(magDataNew);

        // remember time we read compass, to detect compass sensor failure
        lastMagRead_ms = imuSampleTime_ms;
    }
}

/********************************************************
*                Inertial Measurements                  *
********************************************************/

/*
 *  Read IMU delta angle and delta velocity measurements and downsample to 100Hz
 *  for storage in the data buffers used by the EKF. If the IMU data arrives at
 *  lower rate than 100Hz, then no downsampling or upsampling will be performed.
 *  Downsampling is done using a method that does not introduce coning or sculling
 *  errors.
 */
void NavEKF3_core::readIMUData()
{
    const auto &ins = dal.ins();

    // calculate an averaged IMU update rate using a spike and lowpass filter combination
    dtIMUavg = 0.02f * constrain_ftype(ins.get_loop_delta_t(),0.5f * dtIMUavg, 2.0f * dtIMUavg) + 0.98f * dtIMUavg;

    // the imu sample time is used as a common time reference throughout the filter
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;

    uint8_t accel_active, gyro_active;

    if (ins.use_accel(imu_index)) {
        accel_active = imu_index;
    } else {
        accel_active = ins.get_primary_accel();
    }

    if (ins.use_gyro(imu_index)) {
        gyro_active = imu_index;
    } else {
        gyro_active = ins.get_primary_gyro();
    }

    if (gyro_active != gyro_index_active) {
        // we are switching active gyro at runtime. Copy over the
        // bias we have learned from the previously inactive
        // gyro. We don't re-init the bias uncertainty as it should
        // have the same uncertainty as the previously active gyro
        stateStruct.gyro_bias = inactiveBias[gyro_active].gyro_bias;
        gyro_index_active = gyro_active;
    }

    if (accel_active != accel_index_active) {
        // switch to the learned accel bias for this IMU
        stateStruct.accel_bias = inactiveBias[accel_active].accel_bias;
        accel_index_active = accel_active;
    }

    // update the inactive bias states
    learnInactiveBiases();

    // run movement check using IMU data
    updateMovementCheck();

    readDeltaVelocity(accel_index_active, imuDataNew.delVel, imuDataNew.delVelDT);
    accelPosOffset = ins.get_imu_pos_offset(accel_index_active).toftype();
    imuDataNew.accel_index = accel_index_active;
    
    // Get delta angle data from primary gyro or primary if not available
    readDeltaAngle(gyro_index_active, imuDataNew.delAng, imuDataNew.delAngDT);
    imuDataNew.delAngDT = MAX(imuDataNew.delAngDT, 1.0e-4f);
    imuDataNew.gyro_index = gyro_index_active;

    // Get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // Accumulate the measurement time interval for the delta velocity and angle data
    imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    // use the most recent IMU index for the downsampled IMU
    // data. This isn't strictly correct if we switch IMUs between
    // samples
    imuDataDownSampledNew.gyro_index = imuDataNew.gyro_index;
    imuDataDownSampledNew.accel_index = imuDataNew.accel_index;

    // Rotate quaternon atitude from previous to new and normalise.
    // Accumulation using quaternions prevents introduction of coning errors due to downsampling
    imuQuatDownSampleNew.rotate(imuDataNew.delAng);
    imuQuatDownSampleNew.normalize();

    // Rotate the latest delta velocity into body frame at the start of accumulation
    Matrix3F deltaRotMat;
    imuQuatDownSampleNew.rotation_matrix(deltaRotMat);

    // Apply the delta velocity to the delta velocity accumulator
    imuDataDownSampledNew.delVel += deltaRotMat*imuDataNew.delVel;

    // Keep track of the number of IMU frames since the last state prediction
    framesSincePredict++;

    /*
     * If the target EKF time step has been accumulated, and the frontend has allowed start of a new predict cycle,
     * then store the accumulated IMU data to be used by the state prediction, ignoring the frontend permission if more
     * than twice the target time has lapsed. Adjust the target EKF step time threshold to allow for timing jitter in the
     * IMU data.
     */
    if ((imuDataDownSampledNew.delAngDT >= (EKF_TARGET_DT-(dtIMUavg*0.5f)) && startPredictEnabled) ||
        (imuDataDownSampledNew.delAngDT >= 2.0f*EKF_TARGET_DT)) {

        // convert the accumulated quaternion to an equivalent delta angle
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);

        // Time stamp the data
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // Write data to the FIFO IMU buffer
        storedIMU.push_youngest_element(imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF using a combination spike and LPF
        ftype dtNow = constrain_ftype(0.5f*(imuDataDownSampledNew.delAngDT+imuDataDownSampledNew.delVelDT),0.5f * dtEkfAvg, 2.0f * dtEkfAvg);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // do an addtional down sampling for data used to sample XY body frame drag specific forces
        SampleDragData(imuDataDownSampledNew);

        // zero the accumulated IMU data and quaternion
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuQuatDownSampleNew[0] = 1.0f;
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        imuDataDelayed = storedIMU.get_oldest_element();

        // protect against delta time going to zero
        ftype minDT = 0.1f * dtEkfAvg;
        imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT,minDT);
        imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT,minDT);

        updateTimingStatistics();
        
        // correct the extracted IMU data for sensor errors
        delAngCorrected = imuDataDelayed.delAng;
        delVelCorrected = imuDataDelayed.delVel;
        correctDeltaAngle(delAngCorrected, imuDataDelayed.delAngDT, imuDataDelayed.gyro_index);
        correctDeltaVelocity(delVelCorrected, imuDataDelayed.delVelDT, imuDataDelayed.accel_index);

    } else {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        runUpdates = false;
    }
}

// read the delta velocity and corresponding time interval from the IMU
// return false if data is not available
bool NavEKF3_core::readDeltaVelocity(uint8_t ins_index, Vector3F &dVel, ftype &dVel_dt) {
    const auto &ins = dal.ins();

    if (ins_index < ins.get_accel_count()) {
        Vector3f dVelF;
        float dVel_dtF;
        ins.get_delta_velocity(ins_index, dVelF, dVel_dtF);
        dVel = dVelF.toftype();
        dVel_dt = dVel_dtF;
        dVel_dt = MAX(dVel_dt,1.0e-4);
        return true;
    }
    return false;
}

/********************************************************
*             Global Position Measurement               *
********************************************************/

// check for new valid GPS data and update stored measurement if available
void NavEKF3_core::readGpsData()
{
    // check for new GPS data
    const auto &gps = dal.gps();

    // limit update rate to avoid overflowing the FIFO buffer
    if (gps.last_message_time_ms(selected_gps) - lastTimeGpsReceived_ms <= frontend->sensorIntervalMin_ms) {
        return;
    }

    if (gps.status(selected_gps) < AP_DAL_GPS::GPS_OK_FIX_3D) {
        // report GPS fix status
        gpsCheckStatus.bad_fix = true;
        dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string), "Waiting for 3D fix");
        return;
    }

    // report GPS fix status
    gpsCheckStatus.bad_fix = false;

    // store fix time from previous read
    const uint32_t secondLastGpsTime_ms = lastTimeGpsReceived_ms;

    // get current fix time
    lastTimeGpsReceived_ms = gps.last_message_time_ms(selected_gps);

    // estimate when the GPS fix was valid, allowing for GPS processing and other delays
    // ideally we should be using a timing signal from the GPS receiver to set this time
    // Use the driver specified delay
    float gps_delay_sec = 0;
    gps.get_lag(selected_gps, gps_delay_sec);
    gpsDataNew.time_ms = lastTimeGpsReceived_ms - (uint32_t)(gps_delay_sec * 1000.0f);

    // Correct for the average intersampling delay due to the filter updaterate
    gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

    // Prevent the time stamp falling outside the oldest and newest IMU data in the buffer
    gpsDataNew.time_ms = MIN(MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms),imuDataDownSampledNew.time_ms);

    // Get which GPS we are using for position information
    gpsDataNew.sensor_idx = selected_gps;

    // read the NED velocity from the GPS
    gpsDataNew.vel = gps.velocity(selected_gps).toftype();
    gpsDataNew.have_vz = gps.have_vertical_velocity(selected_gps);

    // position and velocity are not yet corrected for sensor position
    gpsDataNew.corrected = false;

    // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
    // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
    ftype alpha = constrain_ftype(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
    gpsSpdAccuracy *= (1.0f - alpha);
    float gpsSpdAccRaw;
    if (!gps.speed_accuracy(selected_gps, gpsSpdAccRaw)) {
        gpsSpdAccuracy = 0.0f;
    } else {
        gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
        gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
        gpsSpdAccuracy = MAX(gpsSpdAccuracy,frontend->_gpsHorizVelNoise);
    }
    gpsPosAccuracy *= (1.0f - alpha);
    float gpsPosAccRaw;
    if (!gps.horizontal_accuracy(selected_gps, gpsPosAccRaw)) {
        gpsPosAccuracy = 0.0f;
    } else {
        gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
        gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
        gpsPosAccuracy = MAX(gpsPosAccuracy, frontend->_gpsHorizPosNoise);
    }
    gpsHgtAccuracy *= (1.0f - alpha);
    float gpsHgtAccRaw;
    if (!gps.vertical_accuracy(selected_gps, gpsHgtAccRaw)) {
        gpsHgtAccuracy = 0.0f;
    } else {
        gpsHgtAccuracy = MAX(gpsHgtAccuracy,gpsHgtAccRaw);
        gpsHgtAccuracy = MIN(gpsHgtAccuracy,100.0f);
        gpsHgtAccuracy = MAX(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise);
    }

    // check if we have enough GPS satellites and increase the gps noise scaler if we don't
    if (gps.num_sats(selected_gps) >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {
        gpsNoiseScaler = 1.0f;
    } else if (gps.num_sats(selected_gps) == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
        gpsNoiseScaler = 1.4f;
    } else { // <= 4 satellites or in constant position mode
        gpsNoiseScaler = 2.0f;
    }

    // Check if GPS can output vertical velocity, vertical velocity use is permitted and set GPS fusion mode accordingly
    if (gpsDataNew.have_vz && frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::GPS)) {
        useGpsVertVel = true;
    } else {
        useGpsVertVel = false;
    }

    // Monitor quality of the GPS velocity data before and after alignment
    calcGpsGoodToAlign();

    // Post-alignment checks
    calcGpsGoodForFlight();

    // Read the GPS location in WGS-84 lat,long,height coordinates
    const Location &gpsloc = gps.location(selected_gps);

    // Set the EKF origin and magnetic field declination if not previously set and GPS checks have passed
    if (gpsGoodToAlign && !validOrigin) {
        Location gpsloc_fieldelevation = gpsloc; 
        // if flying, correct for height change from takeoff so that the origin is at field elevation
        if (inFlight) {
            gpsloc_fieldelevation.alt += (int32_t)(100.0f * stateStruct.position.z);
        }

        if (!setOrigin(gpsloc_fieldelevation)) {
            // set an error as an attempt was made to set the origin more than once
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }

        // set the NE earth magnetic field states using the published declination
        // and set the corresponding variances and covariances
        alignMagStateDeclination();

        // Set the height of the NED origin
        ekfGpsRefHgt = (double)0.01 * (double)gpsloc.alt + (double)outputDataNew.position.z;

        // Set the uncertainty of the GPS origin height
        ekfOriginHgtVar = sq(gpsHgtAccuracy);

    }

    if (gpsGoodToAlign && !have_table_earth_field) {
        const auto &compass = dal.compass();
        if (compass.have_scale_factor(magSelectIndex) &&
            compass.auto_declination_enabled()) {
            getEarthFieldTable(gpsloc);
            if (frontend->_mag_ef_limit > 0) {
                // initialise earth field from tables
                stateStruct.earth_magfield = table_earth_field_ga;
            }
        }
    }

    // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
    if (validOrigin) {
        gpsDataNew.lat = gpsloc.lat;
        gpsDataNew.lng = gpsloc.lng;
        if ((frontend->_originHgtMode & (1<<2)) == 0) {
            gpsDataNew.hgt = (ftype)((double)0.01 * (double)gpsloc.alt - ekfGpsRefHgt);
        } else {
            gpsDataNew.hgt = 0.01 * (gpsloc.alt - EKF_origin.alt);
        }
        storedGPS.push(gpsDataNew);
        // declare GPS in use
        gpsIsInUse = true;
    }
}

// check for new valid GPS yaw data
void NavEKF3_core::readGpsYawData()
{
    const auto &gps = dal.gps();

    // if the GPS has yaw data then fuse it as an Euler yaw angle
    float yaw_deg, yaw_accuracy_deg;
    uint32_t yaw_time_ms;
    if (gps.status(selected_gps) >= AP_DAL_GPS::GPS_OK_FIX_3D &&
        dal.gps().gps_yaw_deg(selected_gps, yaw_deg, yaw_accuracy_deg, yaw_time_ms) &&
        yaw_time_ms != yawMeasTime_ms) {
        // GPS modules are rather too optimistic about their
        // accuracy. Set to min of 5 degrees here to prevent
        // the user constantly receiving warnings about high
        // normalised yaw innovations
        const ftype min_yaw_accuracy_deg = 5.0f;
        yaw_accuracy_deg = MAX(yaw_accuracy_deg, min_yaw_accuracy_deg);
        writeEulerYawAngle(radians(yaw_deg), radians(yaw_accuracy_deg), yaw_time_ms, 2);
    }
}

// read the delta angle and corresponding time interval from the IMU
// return false if data is not available
bool NavEKF3_core::readDeltaAngle(uint8_t ins_index, Vector3F &dAng, ftype &dAngDT) {
    const auto &ins = dal.ins();

    if (ins_index < ins.get_gyro_count()) {
        Vector3f dAngF;
        float dAngDTF;
        ins.get_delta_angle(ins_index, dAngF, dAngDTF);
        dAng = dAngF.toftype();
        dAngDT = dAngDTF;
        return true;
    }
    return false;
}


/********************************************************
*                  Height Measurements                  *
********************************************************/

// check for new pressure altitude measurement data and update stored measurement if available
void NavEKF3_core::readBaroData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    // limit update rate to avoid overflowing the FIFO buffer
    const auto &baro = dal.baro();
    if (baro.get_last_update(selected_baro) - lastBaroReceived_ms > frontend->sensorIntervalMin_ms) {

        baroDataNew.hgt = baro.get_altitude(selected_baro);

        // time stamp used to check for new measurement
        lastBaroReceived_ms = baro.get_last_update(selected_baro);

        // estimate of time height measurement was taken, allowing for delays
        baroDataNew.time_ms = lastBaroReceived_ms - frontend->_hgtDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        baroDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Prevent time delay exceeding age of oldest IMU data in the buffer
        baroDataNew.time_ms = MAX(baroDataNew.time_ms,imuDataDelayed.time_ms);

        // save baro measurement to buffer to be fused later
        storedBaro.push(baroDataNew);
    }
}

// calculate filtered offset between baro height measurement and EKF height estimate
// offset should be subtracted from baro measurement to match filter estimate
// offset is used to enable reversion to baro from alternate height data source
void NavEKF3_core::calcFiltBaroOffset()
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrain_ftype(baroDataDelayed.hgt + stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
}

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void NavEKF3_core::correctEkfOriginHeight()
{
    // Estimate the WGS-84 height of the EKF's origin using a Bayes filter

    // calculate the variance of our a-priori estimate of the ekf origin height
    ftype deltaTime = constrain_ftype(0.001f * (imuDataDelayed.time_ms - lastOriginHgtTime_ms), 0.0, 1.0);
    if (activeHgtSource == AP_NavEKF_Source::SourceZ::BARO) {
        // Use the baro drift rate
        const ftype baroDriftRate = 0.05;
        ekfOriginHgtVar += sq(baroDriftRate * deltaTime);
    } else if (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER) {
        // use the worse case expected terrain gradient and vehicle horizontal speed
        const ftype maxTerrGrad = 0.25;
        ekfOriginHgtVar += sq(maxTerrGrad * stateStruct.velocity.xy().length() * deltaTime);
    } else {
        // by definition our height source is absolute so cannot run this filter
        return;
    }
    lastOriginHgtTime_ms = imuDataDelayed.time_ms;

    // calculate the observation variance assuming EKF error relative to datum is independent of GPS observation error
    // when not using GPS as height source
    ftype originHgtObsVar = sq(gpsHgtAccuracy) + P[9][9];

    // calculate the correction gain
    ftype gain = ekfOriginHgtVar / (ekfOriginHgtVar + originHgtObsVar);

    // calculate the innovation
    ftype innovation = - stateStruct.position.z - gpsDataDelayed.hgt;

    // check the innovation variance ratio
    ftype ratio = sq(innovation) / (ekfOriginHgtVar + originHgtObsVar);

    // correct the EKF origin and variance estimate if the innovation is less than 5-sigma
    if (ratio < 25.0f && gpsAccuracyGood) {
        ekfGpsRefHgt -= (double)(gain * innovation);
        ekfOriginHgtVar -= MAX(gain * ekfOriginHgtVar , 0.0f);
    }
}

/********************************************************
*                Air Speed Measurements                 *
********************************************************/

// check for new airspeed data and update stored measurements if available
void NavEKF3_core::readAirSpdData()
{
    const float EAS2TAS = dal.get_EAS2TAS();
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available

    const auto *airspeed = dal.airspeed();
    if (airspeed &&
        (airspeed->last_update_ms(selected_airspeed) - timeTasReceived_ms) > frontend->sensorIntervalMin_ms) {
        tasDataNew.tas = airspeed->get_airspeed(selected_airspeed) * EAS2TAS;
        timeTasReceived_ms = airspeed->last_update_ms(selected_airspeed);
        tasDataNew.time_ms = timeTasReceived_ms - frontend->tasDelay_ms;
        tasDataNew.tasVariance = sq(MAX(frontend->_easNoise * EAS2TAS, 0.5f));
        tasDataNew.allowFusion = airspeed->healthy(selected_airspeed) && airspeed->use(selected_airspeed);

        // Correct for the average intersampling delay due to the filter update rate
        tasDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Save data into the buffer to be fused when the fusion time horizon catches up with it
        storedTAS.push(tasDataNew);
    }

    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    tasDataToFuse = storedTAS.recall(tasDataDelayed,imuDataDelayed.time_ms);

    float easErrVar = sq(MAX(frontend->_easNoise, 0.5f));
    // Allow use of a default value if enabled
    if (!useAirspeed() &&
        imuDataDelayed.time_ms - tasDataDelayed.time_ms > 200 &&
        is_positive(defaultAirSpeed)) {
        tasDataDelayed.tas = defaultAirSpeed * EAS2TAS;
        tasDataDelayed.tasVariance = sq(MAX(defaultAirSpeedVariance, easErrVar));
        tasDataDelayed.allowFusion = true;
        tasDataDelayed.time_ms = 0;
        usingDefaultAirspeed = true;
    } else {
        usingDefaultAirspeed = false;
    }
}

#if EK3_FEATURE_BEACON_FUSION
/********************************************************
*              Range Beacon Measurements                *
********************************************************/

// check for new range beacon data and push to data buffer if available
void NavEKF3_core::readRngBcnData()
{
    // check that arrays are large enough
    static_assert(ARRAY_SIZE(lastTimeRngBcn_ms) >= AP_BEACON_MAX_BEACONS, "lastTimeRngBcn_ms should have at least AP_BEACON_MAX_BEACONS elements");

    // get the location of the beacon data
    const AP_DAL_Beacon *beacon = dal.beacon();

    // exit immediately if no beacon object
    if (beacon == nullptr) {
        return;
    }

    // get the number of beacons in use
    N_beacons = MIN(beacon->count(), ARRAY_SIZE(lastTimeRngBcn_ms));

    // search through all the beacons for new data and if we find it stop searching and push the data into the observation buffer
    bool newDataPushed = false;
    uint8_t numRngBcnsChecked = 0;
    // start the search one index up from where we left it last time
    uint8_t index = lastRngBcnChecked;
    while (!newDataPushed && (numRngBcnsChecked < N_beacons)) {
        // track the number of beacons checked
        numRngBcnsChecked++;

        // move to next beacon, wrap index if necessary
        index++;
        if (index >= N_beacons) {
            index = 0;
        }

        // check that the beacon is healthy and has new data
        if (beacon->beacon_healthy(index) && beacon->beacon_last_update_ms(index) != lastTimeRngBcn_ms[index]) {
            rng_bcn_elements rngBcnDataNew = {};

            // set the timestamp, correcting for measurement delay and average intersampling delay due to the filter update rate
            lastTimeRngBcn_ms[index] = beacon->beacon_last_update_ms(index);
            rngBcnDataNew.time_ms = lastTimeRngBcn_ms[index] - frontend->_rngBcnDelay_ms - localFilterTimeStep_ms/2;

            // set the range noise
            // TODO the range library should provide the noise/accuracy estimate for each beacon
            rngBcnDataNew.rngErr = frontend->_rngBcnNoise;

            // set the range measurement
            rngBcnDataNew.rng = beacon->beacon_distance(index);

            // set the beacon position
            rngBcnDataNew.beacon_posNED = beacon->beacon_position(index).toftype();

            // identify the beacon identifier
            rngBcnDataNew.beacon_ID = index;

            // indicate we have new data to push to the buffer
            newDataPushed = true;

            // update the last checked index
            lastRngBcnChecked = index;

            // Save data into the buffer to be fused when the fusion time horizon catches up with it
            storedRangeBeacon.push(rngBcnDataNew);
        }
    }

    // Check if the beacon system has returned a 3D fix
    Vector3f bp;
    float bperr;
    if (beacon->get_vehicle_position_ned(bp, bperr)) {
        rngBcnLast3DmeasTime_ms = imuSampleTime_ms;
    }
    beaconVehiclePosNED = bp.toftype();
    beaconVehiclePosErr = bperr;

    // Check if the range beacon data can be used to align the vehicle position
    if ((imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) && (beaconVehiclePosErr < 1.0f) && rngBcnAlignmentCompleted) {
        // check for consistency between the position reported by the beacon and the position from the 3-State alignment filter
        const ftype posDiffSq = sq(receiverPos.x - beaconVehiclePosNED.x) + sq(receiverPos.y - beaconVehiclePosNED.y);
        const ftype posDiffVar = sq(beaconVehiclePosErr) + receiverPosCov[0][0] + receiverPosCov[1][1];
        if (posDiffSq < 9.0f * posDiffVar) {
            rngBcnGoodToAlign = true;
            // Set the EKF origin and magnetic field declination if not previously set
            if (!validOrigin && (PV_AidingMode != AID_ABSOLUTE)) {
                // get origin from beacon system
                Location origin_loc;
                if (beacon->get_origin(origin_loc)) {
                    setOriginLLH(origin_loc);

                    // set the NE earth magnetic field states using the published declination
                    // and set the corresponding variances and covariances
                    alignMagStateDeclination();

                    // Set the uncertainty of the origin height
                    ekfOriginHgtVar = sq(beaconVehiclePosErr);
                }
            }
        } else {
            rngBcnGoodToAlign = false;
        }
    } else {
        rngBcnGoodToAlign = false;
    }

    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    rngBcnDataToFuse = storedRangeBeacon.recall(rngBcnDataDelayed, imuDataDelayed.time_ms);

    // Correct the range beacon earth frame origin for estimated offset relative to the EKF earth frame origin
    if (rngBcnDataToFuse) {
        rngBcnDataDelayed.beacon_posNED.x += bcnPosOffsetNED.x;
        rngBcnDataDelayed.beacon_posNED.y += bcnPosOffsetNED.y;
    }

}
#endif  // EK3_FEATURE_BEACON_FUSION

/********************************************************
*              Independant yaw sensor measurements      *
********************************************************/

void NavEKF3_core::writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type)
{
    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if (((timeStamp_ms - yawMeasTime_ms) < frontend->sensorIntervalMin_ms) || !statesInitialised) {
        return;
    }

    yawAngDataNew.yawAng = yawAngle;
    yawAngDataNew.yawAngErr = yawAngleErr;
    if (type == 2) {
        yawAngDataNew.order = rotationOrder::TAIT_BRYAN_321;
    } else if (type == 1) {
        yawAngDataNew.order = rotationOrder::TAIT_BRYAN_312;
    } else {
        return;
    }
    yawAngDataNew.time_ms = timeStamp_ms;

    storedYawAng.push(yawAngDataNew);

    yawMeasTime_ms = timeStamp_ms;
}

// Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
void NavEKF3_core::writeDefaultAirSpeed(float airspeed, float uncertainty)
{
    defaultAirSpeed = airspeed;
    defaultAirSpeedVariance = sq(uncertainty);
}

/********************************************************
*            External Navigation Measurements           *
********************************************************/

void NavEKF3_core::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
#if EK3_FEATURE_EXTERNAL_NAV
    // protect against NaN
    if (pos.is_nan() || isnan(posErr)) {
        return;
    }

    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if (((timeStamp_ms - extNavMeasTime_ms) < frontend->extNavIntervalMin_ms) || !statesInitialised) {
        return;
    } else {
        extNavMeasTime_ms = timeStamp_ms;
    }

    ext_nav_elements extNavDataNew {};

    if (resetTime_ms != extNavLastPosResetTime_ms) {
        extNavDataNew.posReset = true;
        extNavLastPosResetTime_ms = resetTime_ms;
    } else {
        extNavDataNew.posReset = false;
    }

    extNavDataNew.pos = pos.toftype();
    extNavDataNew.posErr = posErr;

    // calculate timestamp
    timeStamp_ms = timeStamp_ms - delay_ms;
    // Correct for the average intersampling delay due to the filter update rate
    timeStamp_ms -= localFilterTimeStep_ms/2;
    // Prevent time delay exceeding age of oldest IMU data in the buffer
    timeStamp_ms = MAX(timeStamp_ms, imuDataDelayed.time_ms);
    extNavDataNew.time_ms = timeStamp_ms;

    // store position data to buffer
    storedExtNav.push(extNavDataNew);

    // protect against attitude or angle being NaN
    if (!quat.is_nan() && !isnan(angErr)) {
        // extract yaw from the attitude
        ftype roll_rad, pitch_rad, yaw_rad;
        quat.to_euler(roll_rad, pitch_rad, yaw_rad);
        yaw_elements extNavYawAngDataNew;
        extNavYawAngDataNew.yawAng = yaw_rad;
        extNavYawAngDataNew.yawAngErr = MAX(angErr, radians(5.0f)); // ensure yaw accuracy is no better than 5 degrees (some callers may send zero)
        extNavYawAngDataNew.order = rotationOrder::TAIT_BRYAN_321; // Euler rotation order is 321 (ZYX)
        extNavYawAngDataNew.time_ms = timeStamp_ms;
        storedExtNavYawAng.push(extNavYawAngDataNew);
    }
#endif // EK3_FEATURE_EXTERNAL_NAV
}

void NavEKF3_core::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
#if EK3_FEATURE_EXTERNAL_NAV
    // sanity check for NaNs
    if (vel.is_nan() || isnan(err)) {
        return;
    }

    if ((timeStamp_ms - extNavVelMeasTime_ms) < frontend->extNavIntervalMin_ms) {
        return;
    }

    extNavVelMeasTime_ms = timeStamp_ms;
    useExtNavVel = true;
    // calculate timestamp
    timeStamp_ms = timeStamp_ms - delay_ms;
    // Correct for the average intersampling delay due to the filter updaterate
    timeStamp_ms -= localFilterTimeStep_ms/2;
    // Prevent time delay exceeding age of oldest IMU data in the buffer
    timeStamp_ms = MAX(timeStamp_ms,imuDataDelayed.time_ms);

    ext_nav_vel_elements extNavVelNew;
    extNavVelNew.time_ms = timeStamp_ms;
    extNavVelNew.vel = vel.toftype();
    extNavVelNew.err = err;
    extNavVelNew.corrected = false;

    storedExtNavVel.push(extNavVelNew);
#endif // EK3_FEATURE_EXTERNAL_NAV
}

/*
  update the GPS selection
 */
void NavEKF3_core::update_gps_selection(void)
{
    const auto &gps = dal.gps();

    // in normal operation use the primary GPS
    selected_gps = gps.primary_sensor();
    preferred_gps = selected_gps;

    if (frontend->_affinity & EKF_AFFINITY_GPS) {
        if (core_index < gps.num_sensors() ) {
            // always prefer our core_index, unless we don't have that
            // many GPS sensors available
            preferred_gps = core_index;
        }
        if (gps.status(preferred_gps) >= AP_DAL_GPS::GPS_OK_FIX_3D) {
            // select our preferred_gps if it has a 3D fix, otherwise
            // use the primary GPS
            selected_gps = preferred_gps;
        }
    }
}

/*
  update the mag selection
 */
void NavEKF3_core::update_mag_selection(void)
{
    const auto &compass = dal.compass();

    if (frontend->_affinity & EKF_AFFINITY_MAG) {
        if (core_index < compass.get_count() &&
            compass.healthy(core_index) &&
            compass.use_for_yaw(core_index)) {
            // use core_index compass if it is healthy
            magSelectIndex = core_index;
        }
    }
}

/*
  update the baro selection
 */
void NavEKF3_core::update_baro_selection(void)
{
    auto &baro = dal.baro();

    // in normal operation use the primary baro
    selected_baro = baro.get_primary();

    if (frontend->_affinity & EKF_AFFINITY_BARO) {
        if (core_index < baro.num_instances() &&
            baro.healthy(core_index)) {
            // use core_index baro if it is healthy
            selected_baro = core_index;
        }
    }
}

/*
  update the airspeed selection
 */
void NavEKF3_core::update_airspeed_selection(void)
{
    const auto *arsp = dal.airspeed();
    if (arsp == nullptr) {
        return;
    }

    // in normal operation use the primary airspeed sensor
    selected_airspeed = arsp->get_primary();

    if (frontend->_affinity & EKF_AFFINITY_ARSP) {
        if (core_index < arsp->get_num_sensors() &&
            arsp->healthy(core_index) &&
            arsp->use(core_index)) {
            // use core_index airspeed if it is healthy
            selected_airspeed = core_index;
        }
    }
}

/*
  update sensor selections
 */
void NavEKF3_core::update_sensor_selection(void)
{
    update_gps_selection();
    update_mag_selection();
    update_baro_selection();
    update_airspeed_selection();
}

/*
  update timing statistics structure
 */
void NavEKF3_core::updateTimingStatistics(void)
{
    if (timing.count == 0) {
        timing.dtIMUavg_max = dtIMUavg;
        timing.dtIMUavg_min = dtIMUavg;
        timing.dtEKFavg_max = dtEkfAvg;
        timing.dtEKFavg_min = dtEkfAvg;
        timing.delAngDT_max = imuDataDelayed.delAngDT;
        timing.delAngDT_min = imuDataDelayed.delAngDT;
        timing.delVelDT_max = imuDataDelayed.delVelDT;
        timing.delVelDT_min = imuDataDelayed.delVelDT;
    } else {
        timing.dtIMUavg_max = MAX(timing.dtIMUavg_max, dtIMUavg);
        timing.dtIMUavg_min = MIN(timing.dtIMUavg_min, dtIMUavg);
        timing.dtEKFavg_max = MAX(timing.dtEKFavg_max, dtEkfAvg);
        timing.dtEKFavg_min = MIN(timing.dtEKFavg_min, dtEkfAvg);
        timing.delAngDT_max = MAX(timing.delAngDT_max, imuDataDelayed.delAngDT);
        timing.delAngDT_min = MIN(timing.delAngDT_min, imuDataDelayed.delAngDT);
        timing.delVelDT_max = MAX(timing.delVelDT_max, imuDataDelayed.delVelDT);
        timing.delVelDT_min = MIN(timing.delVelDT_min, imuDataDelayed.delVelDT);
    }
    timing.count++;
}

/*
  update estimates of inactive bias states. This keeps inactive IMUs
  as hot-spares so we can switch to them without causing a jump in the
  error
 */
void NavEKF3_core::learnInactiveBiases(void)
{
#if INS_MAX_INSTANCES == 1
    inactiveBias[0].gyro_bias = stateStruct.gyro_bias;
    inactiveBias[0].accel_bias = stateStruct.accel_bias;
#else
    const auto &ins = dal.ins();

    // learn gyro biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (!ins.use_gyro(i)) {
            // can't use this gyro
            continue;
        }
        if (gyro_index_active == i) {
            // use current estimates from main filter of gyro bias
            inactiveBias[i].gyro_bias = stateStruct.gyro_bias;
        } else {
            // get filtered gyro and use the difference between the
            // corrected gyro on the active IMU and the inactive IMU
            // to move the inactive bias towards the right value
            Vector3F filtered_gyro_active = ins.get_gyro(gyro_index_active).toftype() - (stateStruct.gyro_bias/dtEkfAvg);
            Vector3F filtered_gyro_inactive = ins.get_gyro(i).toftype() - (inactiveBias[i].gyro_bias/dtEkfAvg);
            Vector3F error = filtered_gyro_active - filtered_gyro_inactive;

            // prevent a single large error from contaminating bias estimate
            const ftype bias_limit = radians(5);
            error.x = constrain_ftype(error.x, -bias_limit, bias_limit);
            error.y = constrain_ftype(error.y, -bias_limit, bias_limit);
            error.z = constrain_ftype(error.z, -bias_limit, bias_limit);

            // slowly bring the inactive gyro in line with the active gyro. This corrects a 5 deg/sec
            // gyro bias error in around 1 minute
            inactiveBias[i].gyro_bias -= error * (1.0e-4f * dtEkfAvg);
        }
    }

    // learn accel biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (!ins.use_accel(i)) {
            // can't use this accel
            continue;
        }
        if (accel_index_active == i) {
            // use current estimates from main filter of accel bias
            inactiveBias[i].accel_bias = stateStruct.accel_bias;
        } else {
            // get filtered accel and use the difference between the
            // corrected accel on the active IMU and the inactive IMU
            // to move the inactive bias towards the right value
            Vector3F filtered_accel_active = ins.get_accel(accel_index_active).toftype() - (stateStruct.accel_bias/dtEkfAvg);
            Vector3F filtered_accel_inactive = ins.get_accel(i).toftype() - (inactiveBias[i].accel_bias/dtEkfAvg);
            Vector3F error = filtered_accel_active - filtered_accel_inactive;

            // prevent a single large error from contaminating bias estimate
            const ftype bias_limit = 1.0; // m/s/s
            error.x = constrain_ftype(error.x, -bias_limit, bias_limit);
            error.y = constrain_ftype(error.y, -bias_limit, bias_limit);
            error.z = constrain_ftype(error.z, -bias_limit, bias_limit);

            // slowly bring the inactive accel in line with the active
            // accel. This corrects a 0.5 m/s/s accel bias error in
            // around 1 minute
            inactiveBias[i].accel_bias -= error * (1.0e-4f * dtEkfAvg);
        }
    }
#endif
}

/*
  return declination in radians
*/
ftype NavEKF3_core::MagDeclination(void) const
{
    // if we are using the WMM tables then use the table declination
    // to ensure consistency with the table mag field. Otherwise use
    // the declination from the compass library
    if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
        return table_declination;
    }
    if (!use_compass()) {
        return 0;
    }
    return dal.compass().get_declination();
}

/*
  Update the on ground and not moving check.
  Should be called once per IMU update.
  Only updates when on ground and when operating without a magnetometer
*/
void NavEKF3_core::updateMovementCheck(void)
{
    const AP_NavEKF_Source::SourceYaw yaw_source = frontend->sources.getYawSource();
    const bool runCheck = onGround && (yaw_source == AP_NavEKF_Source::SourceYaw::GPS || yaw_source == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK ||
                                       yaw_source == AP_NavEKF_Source::SourceYaw::EXTNAV || yaw_source == AP_NavEKF_Source::SourceYaw::GSF || !use_compass());
    if (!runCheck)
    {
        onGroundNotMoving = false;
        return;
    }

    const ftype gyro_limit = radians(3.0f);
    const ftype gyro_diff_limit = 0.2f;
    const ftype accel_limit = 1.0f;
    const ftype accel_diff_limit = 5.0f;
    const ftype hysteresis_ratio = 0.7f;
    const ftype dtEkfAvgInv = 1.0f / dtEkfAvg;

    // get latest bias corrected gyro and accelerometer data
    const auto &ins = dal.ins();
    Vector3F gyro = ins.get_gyro(gyro_index_active).toftype() - stateStruct.gyro_bias * dtEkfAvgInv;
    Vector3F accel = ins.get_accel(accel_index_active).toftype() - stateStruct.accel_bias * dtEkfAvgInv;

    if (!prevOnGround) {
        gyro_prev = gyro;
        accel_prev = accel;
        onGroundNotMoving = false;
        gyro_diff = gyro_diff_limit;
        accel_diff = accel_diff_limit;
        return;
    }

    // calculate a gyro rate of change metric
    Vector3F temp = (gyro - gyro_prev) * dtEkfAvgInv;
    gyro_prev = gyro;
    gyro_diff = 0.99f * gyro_diff + 0.01f * temp.length();

    // calculate a acceleration rate of change metric
    temp = (accel - accel_prev) * dtEkfAvgInv;
    accel_prev = accel;
    accel_diff = 0.99f * accel_diff + 0.01f * temp.length();

    const ftype gyro_length_ratio = gyro.length() / gyro_limit;
    const ftype accel_length_ratio = (accel.length() - GRAVITY_MSS) / accel_limit;
    const ftype gyro_diff_ratio = gyro_diff / gyro_diff_limit;
    const ftype accel_diff_ratio = accel_diff / accel_diff_limit;
    bool logStatusChange = false;
    if (onGroundNotMoving) {
        if (gyro_length_ratio > frontend->_ognmTestScaleFactor ||
            fabsF(accel_length_ratio) > frontend->_ognmTestScaleFactor ||
            gyro_diff_ratio > frontend->_ognmTestScaleFactor ||
            accel_diff_ratio > frontend->_ognmTestScaleFactor)
        {
            onGroundNotMoving = false;
            logStatusChange = true;
        }
    } else if (gyro_length_ratio < frontend->_ognmTestScaleFactor * hysteresis_ratio &&
            fabsF(accel_length_ratio) < frontend->_ognmTestScaleFactor * hysteresis_ratio &&
            gyro_diff_ratio < frontend->_ognmTestScaleFactor * hysteresis_ratio &&
            accel_diff_ratio < frontend->_ognmTestScaleFactor * hysteresis_ratio)
    {
        onGroundNotMoving = true;
        logStatusChange = true;
    }

    if (logStatusChange || imuSampleTime_ms - lastMoveCheckLogTime_ms > 200) {
        lastMoveCheckLogTime_ms = imuSampleTime_ms;
        const struct log_XKFM pkt{
            LOG_PACKET_HEADER_INIT(LOG_XKFM_MSG),
            time_us            : dal.micros64(),
            core               : core_index,
            ongroundnotmoving  : onGroundNotMoving,
            gyro_length_ratio  : float(gyro_length_ratio),
            accel_length_ratio : float(accel_length_ratio),
            gyro_diff_ratio    : float(gyro_diff_ratio),
            accel_diff_ratio   : float(accel_diff_ratio),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

void NavEKF3_core::SampleDragData(const imu_elements &imu)
{
#if EK3_FEATURE_DRAG_FUSION
    // Average and down sample to 5Hz
    const ftype bcoef_x = frontend->_ballisticCoef_x;
    const ftype bcoef_y = frontend->_ballisticCoef_y;
    const ftype mcoef = frontend->_momentumDragCoef.get();
    const bool using_bcoef_x = bcoef_x > 1.0f;
    const bool using_bcoef_y = bcoef_y > 1.0f;
    const bool using_mcoef = mcoef > 0.001f;
    if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
        // nothing to do
        dragFusionEnabled = false;
        return;
    }

    dragFusionEnabled = true;

    // down-sample the drag specific force data by accumulating and calculating the mean when
    // sufficient samples have been collected

    dragSampleCount ++;

    // note acceleration is accumulated as a delta velocity
    dragDownSampled.accelXY.x += imu.delVel.x;
    dragDownSampled.accelXY.y += imu.delVel.y;
    dragDownSampled.time_ms += imu.time_ms;
    dragSampleTimeDelta += imu.delVelDT;

    // calculate and store means from accumulated values
    if (dragSampleTimeDelta > 0.2f - 0.5f * EKF_TARGET_DT) {
        // note conversion from accumulated delta velocity to acceleration
        dragDownSampled.accelXY.x /= dragSampleTimeDelta;
        dragDownSampled.accelXY.y /= dragSampleTimeDelta;
        dragDownSampled.time_ms /= dragSampleCount;

        // write to buffer
        storedDrag.push(dragDownSampled);

        // reset accumulators
        dragSampleCount = 0;
        dragDownSampled.accelXY.zero();
        dragDownSampled.time_ms = 0;
        dragSampleTimeDelta = 0.0f;
    }
#endif // EK3_FEATURE_DRAG_FUSION
}

/*
  get the earth mag field
 */
void NavEKF3_core::getEarthFieldTable(const Location &loc)
{
    table_earth_field_ga = AP_Declination::get_earth_field_ga(loc).toftype();
    table_declination = radians(AP_Declination::get_declination(loc.lat*1.0e-7,
                                                                loc.lng*1.0e-7));
    have_table_earth_field = true;
}

/*
  update earth field, called at 1Hz
 */
void NavEKF3_core::checkUpdateEarthField(void)
{
    if (have_table_earth_field && filterStatus.flags.using_gps) {
        Location loc = EKF_origin;
        loc.offset(stateStruct.position.x, stateStruct.position.y);
        getEarthFieldTable(loc);
    }
}
