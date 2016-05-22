/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

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
    rngOnGnd = frontend->_rng.ground_clearance_cm() * 0.01f;

    // read range finder at 20Hz
    // TODO better way of knowing if it has new data
    if ((imuSampleTime_ms - lastRngMeasTime_ms) > 50) {

        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms =  imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        if (frontend->_rng.status() == RangeFinder::RangeFinder_Good) {
            rngMeasIndex ++;
            if (rngMeasIndex > 2) {
                rngMeasIndex = 0;
            }
            storedRngMeasTime_ms[rngMeasIndex] = imuSampleTime_ms - 25;
            storedRngMeas[rngMeasIndex] = frontend->_rng.distance_cm() * 0.01f;
        }

        // check for three fresh samples
        bool sampleFresh[3];
        for (uint8_t index = 0; index <= 2; index++) {
            sampleFresh[index] = (imuSampleTime_ms - storedRngMeasTime_ms[index]) < 500;
        }

        // find the median value if we have three fresh samples
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
            rangeDataNew.time_ms = storedRngMeasTime_ms[midIndex];
            // limit the measured range to be no less than the on-ground range
            rangeDataNew.rng = MAX(storedRngMeas[midIndex],rngOnGnd);
            rngValidMeaTime_ms = imuSampleTime_ms;
            // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
            storedRange.push(rangeDataNew);
        } else if (!takeOffDetected) {
            // before takeoff we assume on-ground range value if there is no data
            rangeDataNew.time_ms = imuSampleTime_ms;
            rangeDataNew.rng = rngOnGnd;
            rngValidMeaTime_ms = imuSampleTime_ms;
            // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
            storedRange.push(rangeDataNew);
        }
    }
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF2_core::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
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
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - delAngBodyOF.x/delTimeOF),-0.1f,0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - delAngBodyOF.y/delTimeOF),-0.1f,0.1f);
        delAngBodyOF.zero();
        delTimeOF = 0.0f;
    }
    // check for takeoff if relying on optical flow and zero measurements until takeoff detected
    // if we haven't taken off - constrain position and velocity states
    if (frontend->_fusionModeGPS == 3) {
        detectOptFlowTakeoff();
    }
    // calculate rotation matrices at mid sample time for flow observations
    stateStruct.quat.rotation_matrix(Tbn_flow);
    Tnb_flow = Tbn_flow.transposed();
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor rates for bias
        omegaAcrossFlowTime.x = rawGyroRates.x - flowGyroBias.x;
        omegaAcrossFlowTime.y = rawGyroRates.y - flowGyroBias.y;
        // write uncorrected flow rate measurements that will be used by the focal length scale factor estimator
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY = - rawFlowRates; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + omegaAcrossFlowTime.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + omegaAcrossFlowTime.y;
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

// check for new magnetometer data and update store measurements if available
void NavEKF2_core::readMagData()
{
    if (!_ahrs->get_compass()) {
        allMagSensorsFailed = true;
        return;        
    }
    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    uint8_t maxCount = _ahrs->get_compass()->get_count();
    if (allMagSensorsFailed || (magTimeout && assume_zero_sideslip() && magSelectIndex >= maxCount-1 && inFlight)) {
        allMagSensorsFailed = true;
        return;
    }

    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    if (use_compass() && _ahrs->get_compass()->last_update_usec() - lastMagUpdate_us > 70000) {
        frontend->logging.log_compass = true;

        // If the magnetometer has timed out (been rejected too long) we find another magnetometer to use if available
        // Don't do this if we are on the ground because there can be magnetic interference and we need to know if there is a problem
        // before taking off. Don't do this within the first 30 seconds from startup because the yaw error could be due to large yaw gyro bias affsets
        if (magTimeout && (maxCount > 1) && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000) {

            // search through the list of magnetometers
            for (uint8_t i=1; i<maxCount; i++) {
                uint8_t tempIndex = magSelectIndex + i;
                // loop back to the start index if we have exceeded the bounds
                if (tempIndex >= maxCount) {
                    tempIndex -= maxCount;
                }
                // if the magnetometer is allowed to be used for yaw and has a different index, we start using it
                if (_ahrs->get_compass()->use_for_yaw(tempIndex) && tempIndex != magSelectIndex) {
                    magSelectIndex = tempIndex;
                    hal.console->printf("EKF2 IMU%u switching to compass %u\n",(unsigned)imu_index,magSelectIndex);
                    // reset the timeout flag and timer
                    magTimeout = false;
                    lastHealthyMagTime_ms = imuSampleTime_ms;
                    // zero the learned magnetometer bias states
                    stateStruct.body_magfield.zero();
                    // clear the measurement buffer
                    storedMag.reset();
                    }
            }
        }

        // detect changes to magnetometer offset parameters and reset states
        Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets(magSelectIndex);
        bool changeDetected = lastMagOffsetsValid && (nowMagOffsets != lastMagOffsets);
        if (changeDetected) {
            // zero the learned magnetometer bias states
            stateStruct.body_magfield.zero();
            // clear the measurement buffer
            storedMag.reset();
        }
        lastMagOffsets = nowMagOffsets;
        lastMagOffsetsValid = true;

        // store time of last measurement update
        lastMagUpdate_us = _ahrs->get_compass()->last_update_usec(magSelectIndex);

        // estimate of time magnetometer measurement was taken, allowing for delays
        magDataNew.time_ms = imuSampleTime_ms - frontend->magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        magDataNew.time_ms -= localFilterTimeStep_ms/2;

        // read compass data and scale to improve numerical conditioning
        magDataNew.mag = _ahrs->get_compass()->get_field(magSelectIndex) * 0.001f;

        // check for consistent data between magnetometers
        consistentMagData = _ahrs->get_compass()->consistent();

        // save magnetometer measurement to buffer to be fused later
        storedMag.push(magDataNew);
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
    const AP_InertialSensor &ins = _ahrs->get_ins();

    // average IMU sampling rate
    dtIMUavg = ins.get_loop_delta_t();

    // the imu sample time is used as a common time reference throughout the filter
    imuSampleTime_ms = AP_HAL::millis();

    // use the nominated imu or primary if not available
    if (ins.use_accel(imu_index)) {
        readDeltaVelocity(imu_index, imuDataNew.delVel, imuDataNew.delVelDT);
    } else {
        readDeltaVelocity(ins.get_primary_accel(), imuDataNew.delVel, imuDataNew.delVelDT);
    }

    // Get delta angle data from primary gyro or primary if not available
    if (ins.use_gyro(imu_index)) {
        readDeltaAngle(imu_index, imuDataNew.delAng);
    } else {
        readDeltaAngle(ins.get_primary_gyro(), imuDataNew.delAng);
    }
    imuDataNew.delAngDT = MAX(ins.get_delta_angle_dt(imu_index),1.0e-4f);

    // Get current time stamp
    imuDataNew.time_ms = imuSampleTime_ms;

    // remove gyro scale factor errors
    imuDataNew.delAng.x = imuDataNew.delAng.x * stateStruct.gyro_scale.x;
    imuDataNew.delAng.y = imuDataNew.delAng.y * stateStruct.gyro_scale.y;
    imuDataNew.delAng.z = imuDataNew.delAng.z * stateStruct.gyro_scale.z;

    // remove sensor bias errors
    imuDataNew.delAng -= stateStruct.gyro_bias * (imuDataNew.delAngDT / dtEkfAvg);
    imuDataNew.delVel.z -= stateStruct.accel_zbias * (imuDataNew.delVelDT / dtEkfAvg);

    // Accumulate the measurement time interval for the delta velocity and angle data
    imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    // Rotate quaternon atitude from previous to new and normalise.
    // Accumulation using quaternions prevents introduction of coning errors due to downsampling
    Quaternion deltaQuat;
    deltaQuat.rotate(imuDataNew.delAng);
    imuQuatDownSampleNew = imuQuatDownSampleNew*deltaQuat;
    imuQuatDownSampleNew.normalize();

    // Rotate the accumulated delta velocity into the new frame of reference created by the latest delta angle
    // This prevents introduction of sculling errors due to downsampling
    Matrix3f deltaRotMat;
    deltaQuat.inverse().rotation_matrix(deltaRotMat);
    imuDataDownSampledNew.delVel = deltaRotMat*imuDataDownSampledNew.delVel;

    // accumulate the latest delta velocity
    imuDataDownSampledNew.delVel += imuDataNew.delVel;

    // Keep track of the number of IMU frames since the last state prediction
    framesSincePredict++;

    // If 10msec has elapsed, and the frontend has allowed us to start a new predict cycle, then store the accumulated IMU data
    // to be used by the state prediction, ignoring the frontend permission if more than 20msec has lapsed
    if ((dtIMUavg*(float)framesSincePredict >= 0.01f && startPredictEnabled) || (dtIMUavg*(float)framesSincePredict >= 0.02f)) {
        // convert the accumulated quaternion to an equivalent delta angle
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);
        // Time stamp the data
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;
        // Write data to the FIFO IMU buffer
        storedIMU.push_youngest_element(imuDataDownSampledNew);
        // zero the accumulated IMU data and quaternion
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuQuatDownSampleNew[0] = 1.0f;
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;
        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        framesSincePredict = 0;
        // set the flag to let the filter know it has new IMU data nad needs to run
        runUpdates = true;
    } else {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        runUpdates = false;
    }

    // extract the oldest available data from the FIFO buffer
    imuDataDelayed = storedIMU.pop_oldest_element();
    float minDT = 0.1f*dtEkfAvg;
    imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT,minDT);
    imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT,minDT);

}

// read the delta velocity and corresponding time interval from the IMU
// return false if data is not available
bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);
        dVel_dt = MAX(ins.get_delta_velocity_dt(ins_index),1.0e-4f);
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
    // check for new GPS data
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    if (_ahrs->get_gps().last_message_time_ms() - lastTimeGpsReceived_ms > 70) {
        if (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            // report GPS fix status
            gpsCheckStatus.bad_fix = false;

            // store fix time from previous read
            secondLastGpsTime_ms = lastTimeGpsReceived_ms;

            // get current fix time
            lastTimeGpsReceived_ms = _ahrs->get_gps().last_message_time_ms();

            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            gpsDataNew.time_ms = lastTimeGpsReceived_ms - frontend->_gpsDelay_ms;

            // Correct for the average intersampling delay due to the filter updaterate
            gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            gpsDataNew.time_ms = MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms);

            // read the NED velocity from the GPS
            gpsDataNew.vel = _ahrs->get_gps().velocity();

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!_ahrs->get_gps().speed_accuracy(gpsSpdAccRaw)) {
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
            }
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw;
            if (!_ahrs->get_gps().horizontal_accuracy(gpsPosAccRaw)) {
                gpsPosAccuracy = 0.0f;
            } else {
                gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            if (_ahrs->get_gps().num_sats() >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.0f;
            } else if (_ahrs->get_gps().num_sats() == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity and set GPS fusion mode accordingly
            if (_ahrs->get_gps().have_vertical_velocity() && frontend->_fusionModeGPS == 0) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data before and after alignment using separate checks
            if (PV_AidingMode != AID_ABSOLUTE) {
                // Pre-alignment checks
                gpsGoodToAlign = calcGpsGoodToAlign();
            } else {
                // Post-alignment checks
                calcGpsGoodForFlight();
            }

            // Read the GPS locaton in WGS-84 lat,long,height coordinates
            const struct Location &gpsloc = _ahrs->get_gps().location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            if (gpsGoodToAlign && !validOrigin) {
                setOrigin();
                // Now we know the location we have an estimate for the magnetic field declination and adjust the earth field accordingly
                alignMagStateDeclination();
                // Set the height of the NED origin to ‘height of baro height datum relative to GPS height datum'
                EKF_origin.alt = gpsloc.alt - baroDataNew.hgt;
            }

            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            if (validOrigin) {
                gpsDataNew.pos = location_diff(EKF_origin, gpsloc);
                gpsDataNew.hgt = 0.01f * (gpsloc.alt - EKF_origin.alt);
                storedGPS.push(gpsDataNew);
                // declare GPS available for use
                gpsNotAvailable = false;
            }

            frontend->logging.log_gps = true;

        } else {
            // report GPS fix status
            gpsCheckStatus.bad_fix = true;
        }
    }

    // We need to handle the case where GPS is lost for a period of time that is too long to dead-reckon
    // If that happens we need to put the filter into a constant position mode, reset the velocity states to zero
    // and use the last estimated position as a synthetic GPS position

    // check if we can use opticalflow as a backup
    bool optFlowBackupAvailable = (flowDataValid && !hgtTimeout);

    // Set GPS time-out threshold depending on whether we have an airspeed sensor to constrain drift
    uint16_t gpsRetryTimeout_ms = useAirspeed() ? frontend->gpsRetryTimeUseTAS_ms : frontend->gpsRetryTimeNoTAS_ms;

    // Set the time that copters will fly without a GPS lock before failing the GPS and switching to a non GPS mode
    uint16_t gpsFailTimeout_ms = optFlowBackupAvailable ? frontend->gpsFailTimeWithFlow_ms : gpsRetryTimeout_ms;

    // If we haven't received GPS data for a while and we are using it for aiding, then declare the position and velocity data as being timed out
    if (imuSampleTime_ms - lastTimeGpsReceived_ms > gpsFailTimeout_ms) {

        // Let other processes know that GPS is not available and that a timeout has occurred
        posTimeout = true;
        velTimeout = true;
        gpsNotAvailable = true;

        // If we are totally reliant on GPS for navigation, then we need to switch to a non-GPS mode of operation
        // If we don't have airspeed or sideslip assumption or optical flow to constrain drift, then go into constant position mode.
        // If we can do optical flow nav (valid flow data and height above ground estimate), then go into flow nav mode.
        if (PV_AidingMode == AID_ABSOLUTE && !useAirspeed() && !assume_zero_sideslip()) {
            if (optFlowBackupAvailable) {
                // we can do optical flow only nav
                frontend->_fusionModeGPS = 3;
                PV_AidingMode = AID_RELATIVE;
            } else {
                // store the current position
                lastKnownPositionNE.x = stateStruct.position.x;
                lastKnownPositionNE.y = stateStruct.position.y;

                // put the filter into constant position mode
                PV_AidingMode = AID_NONE;

                // Reset the velocity and position states
                ResetVelocity();
                ResetPosition();

                // Reset the normalised innovation to avoid false failing bad fusion tests
                velTestRatio = 0.0f;
                posTestRatio = 0.0f;
            }
        }
    }
}

// read the delta angle and corresponding time interval from the IMU
// return false if data is not available
bool NavEKF2_core::readDeltaAngle(uint8_t ins_index, Vector3f &dAng) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);
        frontend->logging.log_imu = true;
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
    if (frontend->_baro.get_last_update() - lastBaroReceived_ms > 70) {
        frontend->logging.log_baro = true;

        baroDataNew.hgt = frontend->_baro.get_altitude();

        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (isAiding && getTakeoffExpected()) {
            baroDataNew.hgt = MAX(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastBaroReceived_ms = frontend->_baro.get_last_update();

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
// offset is used to enable reversion to baro if alternate height data sources fail
void NavEKF2_core::calcFiltBaroOffset()
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrain_float(baroDataDelayed.hgt + stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
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
    const AP_Airspeed *aspeed = _ahrs->get_airspeed();
    if (aspeed &&
            aspeed->use() &&
            aspeed->last_update_ms() != timeTasReceived_ms) {
        tasDataNew.tas = aspeed->get_airspeed() * aspeed->get_EAS2TAS();
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

#endif // HAL_CPU_CLASS
