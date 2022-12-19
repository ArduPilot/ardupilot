#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;


/********************************************************
*              OPT FLOW AND RANGE FINDER                *
********************************************************/

// Read the range finder and take new measurements if available
// Apply a median filter
void NavEKF2_core::readRangeFinder(void)
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

    // read range finder at 20Hz
    // TODO better way of knowing if it has new data
    if (_rng && (imuSampleTime_ms - lastRngMeasTime_ms) > 50) {

        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms =  imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        // use data from two range finders if available

        for (uint8_t sensorIndex = 0; sensorIndex < ARRAY_SIZE(rngMeasIndex); sensorIndex++) {
            auto *sensor = _rng->get_backend(sensorIndex);
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

            } else if (!takeOffDetected && ((imuSampleTime_ms - rngValidMeaTime_ms) > 200)) {
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

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF2_core::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride)
{
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

    // calculate rotation matrices at mid sample time for flow observations
    stateStruct.quat.rotation_matrix(Tbn_flow);
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor body rates for bias and write
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
        ofDataNew.flowRadXY = (-rawFlowRates).toftype(); // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
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
        // Check for data at the fusion time horizon
        flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);
    }
}


/********************************************************
*                      MAGNETOMETER                     *
********************************************************/

// try changing to another compass
void NavEKF2_core::tryChangeCompass(void)
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
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u switching to compass %u",(unsigned)imu_index,magSelectIndex);
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
void NavEKF2_core::readMagData()
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
        bool fusionTimeout = magTimeout && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000;
        bool sensorTimeout = !compass.healthy(magSelectIndex) && imuSampleTime_ms - lastMagRead_ms > frontend->magFailTimeLimit_ms;
        if (fusionTimeout || sensorTimeout) {
            tryChangeCompass();
        }
    }

    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    if (use_compass() &&
        compass.healthy(magSelectIndex) &&
        compass.last_update_usec(magSelectIndex) - lastMagUpdate_us > 70000) {

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
        magDataNew.mag = compass.get_field(magSelectIndex).toftype() * 0.001f;

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
void NavEKF2_core::readIMUData()
{
    const auto &ins = dal.ins();

    // average IMU sampling rate
    dtIMUavg = ins.get_loop_delta_t();

    // use the nominated imu or primary if not available
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
        // biases we have learned from the previously inactive
        // gyro. We don't re-init the bias uncertainty as it should
        // have the same uncertainty as the previously active gyro
        stateStruct.gyro_bias = inactiveBias[gyro_active].gyro_bias;
        gyro_index_active = gyro_active;

        // use the gyro scale factor we have previously used on this
        // IMU (if any). We don't reset the variances as we don't want
        // errors after switching to be mis-assigned to the gyro scale
        // factor
        stateStruct.gyro_scale = inactiveBias[gyro_active].gyro_scale;
    }

    if (accel_active != accel_index_active) {
        // switch to the learned accel bias for this IMU
        stateStruct.accel_zbias = inactiveBias[accel_active].accel_zbias;
        accel_index_active = accel_active;
    }

    // update the inactive bias states
    learnInactiveBiases();

    readDeltaVelocity(accel_index_active, imuDataNew.delVel, imuDataNew.delVelDT);
    accelPosOffset = ins.get_imu_pos_offset(accel_index_active).toftype();
    imuDataNew.accel_index = accel_index_active;

    // Get delta angle data from primary gyro or primary if not available
    readDeltaAngle(gyro_index_active, imuDataNew.delAng, imuDataNew.delAngDT);
    imuDataNew.gyro_index = gyro_index_active;

    // Get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // use the most recent IMU index for the downsampled IMU
    // data. This isn't strictly correct if we switch IMUs between
    // samples
    imuDataDownSampledNew.gyro_index = imuDataNew.gyro_index;
    imuDataDownSampledNew.accel_index = imuDataNew.accel_index;

    // Accumulate the measurement time interval for the delta velocity and angle data
    imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

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
    if ((dtIMUavg*(float)framesSincePredict >= (EKF_TARGET_DT-(dtIMUavg*0.5)) &&
         startPredictEnabled) || (dtIMUavg*(float)framesSincePredict >= 2.0f*EKF_TARGET_DT)) {

        // convert the accumulated quaternion to an equivalent delta angle
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);

        // Time stamp the data
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // Write data to the FIFO IMU buffer
        storedIMU.push_youngest_element(imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF
        ftype dtNow = constrain_ftype(0.5f*(imuDataDownSampledNew.delAngDT+imuDataDownSampledNew.delVelDT),0.0f,10.0f*EKF_TARGET_DT);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // zero the accumulated IMU data and quaternion
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuDataDownSampledNew.gyro_index = gyro_index_active;
        imuDataDownSampledNew.accel_index = accel_index_active;
        imuQuatDownSampleNew[0] = 1.0f;
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        imuDataDelayed = storedIMU.get_oldest_element();

        // protect against delta time going to zero
        // TODO - check if calculations can tolerate 0
        ftype minDT = 0.1f*dtEkfAvg;
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
bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3F &dVel, ftype &dVel_dt) {
    const auto &ins = dal.ins();

    if (ins_index < ins.get_accel_count()) {
        Vector3f dVelF;
        float dVel_dtF;
        ins.get_delta_velocity(ins_index, dVelF, dVel_dtF);
        dVel = dVelF.toftype();
        dVel_dt = dVel_dtF;
        dVel_dt = MAX(dVel_dt,1.0e-4f);
        dVel_dt = MIN(dVel_dt,1.0e-1f);
        return true;
    }
    return false;
}

/********************************************************
*             Global Position Measurement               *
********************************************************/

// check for new valid GPS data and update stored measurement if available
void NavEKF2_core::readGpsData()
{
    if (frontend->_fusionModeGPS == 3) {
        // don't read GPS data if GPS usage disabled
        return;
    }

    // check for new GPS data
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    const auto &gps = dal.gps();
    if (gps.last_message_time_ms(gps.primary_sensor()) - lastTimeGpsReceived_ms > 70) {
        if (gps.status() >= AP_DAL_GPS::GPS_OK_FIX_3D) {
            // report GPS fix status
            gpsCheckStatus.bad_fix = false;

            // store fix time from previous read
            const uint32_t secondLastGpsTime_ms = lastTimeGpsReceived_ms;

            // get current fix time
            lastTimeGpsReceived_ms = gps.last_message_time_ms(gps.primary_sensor());


            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            float gps_delay = 0.0;
            gps.get_lag(gps_delay); // ignore the return value
            gpsDataNew.time_ms = lastTimeGpsReceived_ms - (uint32_t)(1e3f * gps_delay);

            // Correct for the average intersampling delay due to the filter updaterate
            gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            gpsDataNew.time_ms = MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms);

            // Get which GPS we are using for position information
            gpsDataNew.sensor_idx = gps.primary_sensor();

            // read the NED velocity from the GPS
            gpsDataNew.vel = gps.velocity().toftype();

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            ftype alpha = constrain_ftype(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!gps.speed_accuracy(gpsSpdAccRaw)) {
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,frontend->_gpsHorizVelNoise);
            }
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw;
            if (!gps.horizontal_accuracy(gpsPosAccRaw)) {
                gpsPosAccuracy = 0.0f;
            } else {
                gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
                gpsPosAccuracy = MAX(gpsPosAccuracy, frontend->_gpsHorizPosNoise);
            }
            gpsHgtAccuracy *= (1.0f - alpha);
            float gpsHgtAccRaw;
            if (!gps.vertical_accuracy(gpsHgtAccRaw)) {
                gpsHgtAccuracy = 0.0f;
            } else {
                gpsHgtAccuracy = MAX(gpsHgtAccuracy,gpsHgtAccRaw);
                gpsHgtAccuracy = MIN(gpsHgtAccuracy,100.0f);
                gpsHgtAccuracy = MAX(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            if (gps.num_sats() >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.0f;
            } else if (gps.num_sats() == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity, if it is allowed to be used, and set GPS fusion mode accordingly
            if (gps.have_vertical_velocity() && frontend->_fusionModeGPS == 0) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data both before and after alignment. This updates
            // GpsGoodToAlign class variable
            calcGpsGoodToAlign();

            // Post-alignment checks
            calcGpsGoodForFlight();

            // see if we can get origin from frontend
            if (!validOrigin && frontend->common_origin_valid) {

                if (!setOrigin(frontend->common_EKF_origin)) {
                    // set an error as an attempt was made to set the origin more than once
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return;
                }

            }

            // Read the GPS location in WGS-84 lat,long,height coordinates
            const struct Location &gpsloc = gps.location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
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
                    table_earth_field_ga = AP_Declination::get_earth_field_ga(gpsloc).toftype();
                    table_declination = radians(AP_Declination::get_declination(gpsloc.lat*1.0e-7,
                                                                                gpsloc.lng*1.0e-7));
                    have_table_earth_field = true;
                    if (frontend->_mag_ef_limit > 0) {
                        // initialise earth field from tables
                        stateStruct.earth_magfield = table_earth_field_ga;
                    }
                }
            }
            
            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            if (validOrigin) {
                gpsDataNew.pos = EKF_origin.get_distance_NE_ftype(gpsloc);
                if ((frontend->_originHgtMode & (1<<2)) == 0) {
                    gpsDataNew.hgt = (float)((double)0.01 * (double)gpsloc.alt - ekfGpsRefHgt);
                } else {
                    gpsDataNew.hgt = 0.01 * (gpsloc.alt - EKF_origin.alt);
                }
                storedGPS.push(gpsDataNew);
                // declare GPS available for use
                gpsNotAvailable = false;
            }

        } else {
            // report GPS fix status
            gpsCheckStatus.bad_fix = true;
            dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string), "Waiting for 3D fix");
        }
    }
}

// read the delta angle and corresponding time interval from the IMU
// return false if data is not available
bool NavEKF2_core::readDeltaAngle(uint8_t ins_index, Vector3F &dAng, ftype &dAng_dt) {
    const auto &ins = dal.ins();

    if (ins_index < ins.get_gyro_count()) {
        Vector3f dAngF;
        float dAng_dtF;
        ins.get_delta_angle(ins_index, dAngF, dAng_dtF);
        dAng = dAngF.toftype();
        dAng_dt = dAng_dtF;
        dAng_dt = MAX(dAng_dt,1.0e-4f);
        dAng_dt = MIN(dAng_dt,1.0e-1f);
        return true;
    }
    return false;
}


/********************************************************
*                  Height Measurements                  *
********************************************************/

// check for new pressure altitude measurement data and update stored measurement if available
void NavEKF2_core::readBaroData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    const auto &baro = dal.baro();
    if (baro.get_last_update() - lastBaroReceived_ms > 70) {

        baroDataNew.hgt = baro.get_altitude();

        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (dal.get_takeoff_expected() && !assume_zero_sideslip()) {
            baroDataNew.hgt = MAX(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastBaroReceived_ms = baro.get_last_update();

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
void NavEKF2_core::calcFiltBaroOffset()
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrain_ftype(baroDataDelayed.hgt + stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
}

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void NavEKF2_core::correctEkfOriginHeight()
{
    // Estimate the WGS-84 height of the EKF's origin using a Bayes filter

    // calculate the variance of our a-priori estimate of the ekf origin height
    ftype deltaTime = constrain_ftype(0.001f * (imuDataDelayed.time_ms - lastOriginHgtTime_ms), 0.0f, 1.0f);
    if (activeHgtSource == HGT_SOURCE_BARO) {
        // Use the baro drift rate
        const ftype baroDriftRate = 0.05f;
        ekfOriginHgtVar += sq(baroDriftRate * deltaTime);
    } else if (activeHgtSource == HGT_SOURCE_RNG) {
        // use the worse case expected terrain gradient and vehicle horizontal speed
        const ftype maxTerrGrad = 0.25f;
        ekfOriginHgtVar += sq(maxTerrGrad * stateStruct.velocity.xy().length() * deltaTime);
    } else {
        // by definition our height source is absolute so cannot run this filter
        return;
    }
    lastOriginHgtTime_ms = imuDataDelayed.time_ms;

    // calculate the observation variance assuming EKF error relative to datum is independent of GPS observation error
    // when not using GPS as height source
    ftype originHgtObsVar = sq(gpsHgtAccuracy) + P[8][8];

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
void NavEKF2_core::readAirSpdData()
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available
    const auto *aspeed = dal.airspeed();
    if (aspeed &&
        aspeed->use() &&
        aspeed->healthy() &&
        aspeed->last_update_ms() != timeTasReceived_ms) {
        tasDataNew.tas = aspeed->get_airspeed() * dal.get_EAS2TAS();
        timeTasReceived_ms = aspeed->last_update_ms();
        tasDataNew.time_ms = timeTasReceived_ms - frontend->tasDelay_ms;

        // Correct for the average intersampling delay due to the filter update rate
        tasDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Save data into the buffer to be fused when the fusion time horizon catches up with it
        storedTAS.push(tasDataNew);
    }
    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    tasDataToFuse = storedTAS.recall(tasDataDelayed,imuDataDelayed.time_ms);
}

/********************************************************
*              Range Beacon Measurements                *
********************************************************/

#if AP_BEACON_ENABLED
// check for new range beacon data and push to data buffer if available
void NavEKF2_core::readRngBcnData()
{
    // get the location of the beacon data
    const auto *beacon = dal.beacon();

    // exit immediately if no beacon object
    if (beacon == nullptr) {
        return;
    }

    // get the number of beacons in use
    N_beacons = beacon->count();

    // search through all the beacons for new data and if we find it stop searching and push the data into the observation buffer
    bool newDataToPush = false;
    uint8_t numRngBcnsChecked = 0;
    // start the search one index up from where we left it last time
    uint8_t index = lastRngBcnChecked;
    while (!newDataToPush && numRngBcnsChecked < N_beacons) {
        // track the number of beacons checked
        numRngBcnsChecked++;

        // move to next beacon, wrap index if necessary
        index++;
        if (index >= N_beacons) {
            index = 0;
        }

        // check that the beacon is healthy and has new data
        if (beacon->beacon_healthy(index) &&
                beacon->beacon_last_update_ms(index) != lastTimeRngBcn_ms[index])
        {
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
            newDataToPush = true;

            // update the last checked index
            lastRngBcnChecked = index;
        }
    }

    // Check if the beacon system has returned a 3D fix
    Vector3f beaconVehiclePosNEDF;
    float beaconVehiclePosErrF;
    if (beacon->get_vehicle_position_ned(beaconVehiclePosNEDF, beaconVehiclePosErrF)) {
        rngBcnLast3DmeasTime_ms = imuSampleTime_ms;
    }
    beaconVehiclePosNED = beaconVehiclePosNEDF.toftype();
    beaconVehiclePosErr = beaconVehiclePosErrF;


    // Check if the range beacon data can be used to align the vehicle position
    if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250 && beaconVehiclePosErr < 1.0f && rngBcnAlignmentCompleted) {
        // check for consistency between the position reported by the beacon and the position from the 3-State alignment filter
        ftype posDiffSq = sq(receiverPos.x - beaconVehiclePosNED.x) + sq(receiverPos.y - beaconVehiclePosNED.y);
        ftype posDiffVar = sq(beaconVehiclePosErr) + receiverPosCov[0][0] + receiverPosCov[1][1];
        if (posDiffSq < 9.0f*posDiffVar) {
            rngBcnGoodToAlign = true;
            // Set the EKF origin and magnetic field declination if not previously set
            if (!validOrigin && PV_AidingMode != AID_ABSOLUTE) {
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

    // Save data into the buffer to be fused when the fusion time horizon catches up with it
    if (newDataToPush) {
        storedRangeBeacon.push(rngBcnDataNew);
    }

    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    rngBcnDataToFuse = storedRangeBeacon.recall(rngBcnDataDelayed,imuDataDelayed.time_ms);

}
#endif  // AP_BEACON_ENABLED

/*
  update timing statistics structure
 */
void NavEKF2_core::updateTimingStatistics(void)
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

void NavEKF2_core::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
    // protect against NaN
    if (pos.is_nan() || isnan(posErr) || quat.is_nan() || isnan(angErr)) {
        return;
    }

    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if ((timeStamp_ms - extNavMeasTime_ms) < 20) {
        return;
    } else {
        extNavMeasTime_ms = timeStamp_ms;
    }

    if (resetTime_ms != extNavLastPosResetTime_ms) {
        extNavDataNew.posReset = true;
        extNavLastPosResetTime_ms = resetTime_ms;
    } else {
        extNavDataNew.posReset = false;
    }

    extNavDataNew.pos = pos.toftype();
    extNavDataNew.quat = quat.toftype();
    extNavDataNew.posErr = posErr;
    extNavDataNew.angErr = angErr;
    timeStamp_ms = timeStamp_ms - delay_ms;
    // Correct for the average intersampling delay due to the filter updaterate
    timeStamp_ms -= localFilterTimeStep_ms/2;
    // Prevent time delay exceeding age of oldest IMU data in the buffer
    timeStamp_ms = MAX(timeStamp_ms,imuDataDelayed.time_ms);
    extNavDataNew.time_ms = timeStamp_ms;

    storedExtNav.push(extNavDataNew);
}

/*
  return declination in radians
*/
ftype NavEKF2_core::MagDeclination(void) const
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
  update estimates of inactive bias states. This keeps inactive IMUs
  as hot-spares so we can switch to them without causing a jump in the
  error
 */
void NavEKF2_core::learnInactiveBiases(void)
{
#if INS_MAX_INSTANCES == 1
    inactiveBias[0].gyro_bias = stateStruct.gyro_bias;
    inactiveBias[0].gyro_scale = stateStruct.gyro_scale;
    inactiveBias[0].accel_zbias = stateStruct.accel_zbias;
#else
    const auto &ins = dal.ins();

    // learn gyro biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        if (!ins.use_gyro(i)) {
            // can't use this gyro
            continue;
        }
        if (gyro_index_active == i) {
            // use current estimates from main filter of gyro bias and scale
            inactiveBias[i].gyro_bias = stateStruct.gyro_bias;
            inactiveBias[i].gyro_scale = stateStruct.gyro_scale;
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
            // use current estimate from main filter
            inactiveBias[i].accel_zbias = stateStruct.accel_zbias;
        } else {
            // get filtered accel and use the difference between the
            // corrected accel on the active IMU and the inactive IMU
            // to move the inactive bias towards the right value
            ftype filtered_accel_active = ins.get_accel(accel_index_active).z - (stateStruct.accel_zbias/dtEkfAvg);
            ftype filtered_accel_inactive = ins.get_accel(i).z - (inactiveBias[i].accel_zbias/dtEkfAvg);
            ftype error = filtered_accel_active - filtered_accel_inactive;

            // prevent a single large error from contaminating bias estimate
            const ftype bias_limit = 1; // m/s/s
            error = constrain_ftype(error, -bias_limit, bias_limit);

            // slowly bring the inactive accel in line with the active accel
            // this learns 0.5m/s/s bias in about 1 minute
            inactiveBias[i].accel_zbias -= error * (1.0e-4f * dtEkfAvg);
        }
    }
#endif
}

// Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
void NavEKF2_core::writeDefaultAirSpeed(float airspeed)
{
    defaultAirSpeed = airspeed;
}

void NavEKF2_core::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
    // protect against NaN
    if (vel.is_nan() || isnan(err)) {
        return;
    }

    if ((timeStamp_ms - extNavVelMeasTime_ms) < 20) {
        return;
    }

    extNavVelMeasTime_ms = timeStamp_ms;
    useExtNavVel = true;
    extNavVelNew.vel = vel.toftype();
    extNavVelNew.err = err;
    timeStamp_ms = timeStamp_ms - delay_ms;
    // Correct for the average intersampling delay due to the filter updaterate
    timeStamp_ms -= localFilterTimeStep_ms/2;
    // Prevent time delay exceeding age of oldest IMU data in the buffer
    timeStamp_ms = MAX(timeStamp_ms,imuDataDelayed.time_ms);
    extNavVelNew.time_ms = timeStamp_ms;
    storedExtNavVel.push(extNavVelNew);
}
