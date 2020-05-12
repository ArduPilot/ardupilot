#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF3_core::ResetVelocity(void)
{
    // Store the velocity before the reset so that we can record the reset delta
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    zeroRows(P,4,5);
    zeroCols(P,4,5);

    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        P[5][5] = P[4][4] = sq(frontend->_gpsHorizVelNoise);
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250 && velResetSource == DEFAULT) || velResetSource == GPS) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            stateStruct.velocity.x  = gps_corrected.vel.x;
            stateStruct.velocity.y  = gps_corrected.vel.y;
            // set the variances using the reported GPS speed accuracy
            P[5][5] = P[4][4] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[5][5] = P[4][4] = sq(25.0f);
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

    // Calculate the velocity jump due to the reset
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    lastVelReset_ms = imuSampleTime_ms;

    // clear reset data source preference
    velResetSource = DEFAULT;

}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF3_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    zeroRows(P,7,8);
    zeroCols(P,7,8);

    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        P[7][7] = P[8][8] = sq(frontend->_gpsHorizPosNoise);
    } else  {
        // Use GPS data as first preference if fresh data is available
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250 && posResetSource == DEFAULT) || posResetSource == GPS) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            // record the ID of the GPS for the data we are using for the reset
            last_gps_idx = gps_corrected.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            stateStruct.position.x = gps_corrected.pos.x  + 0.001f*gps_corrected.vel.x*(float(imuDataDelayed.time_ms) - float(gps_corrected.time_ms));
            stateStruct.position.y = gps_corrected.pos.y  + 0.001f*gps_corrected.vel.y*(float(imuDataDelayed.time_ms) - float(gps_corrected.time_ms));
            // set the variances using the position measurement noise parameter
            P[7][7] = P[8][8] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;
        } else if ((imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250 && posResetSource == DEFAULT) || posResetSource == RNGBCN) {
            // use the range beacon data as a second preference
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[7][7] = receiverPosCov[0][0];
            P[8][8] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        } else if ((imuSampleTime_ms - extNavDataDelayed.time_ms < 250 && posResetSource == DEFAULT) || posResetSource == EXTNAV) {
            // use external nav data as the third preference
            ext_nav_elements extNavCorrected = extNavDataDelayed;
            CorrectExtNavForSensorOffset(extNavCorrected.pos);
            stateStruct.position.x = extNavCorrected.pos.x;
            stateStruct.position.y = extNavCorrected.pos.y;
            // set the variances as received from external nav system data
            P[7][7] = P[8][8] = sq(extNavDataDelayed.posErr);
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;

    // Calculate the position jump due to the reset
    posResetNE.x = stateStruct.position.x - posResetNE.x;
    posResetNE.y = stateStruct.position.y - posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;

    // clear reset source preference
    posResetSource = DEFAULT;

}

// reset the stateStruct's NE position to the specified position
//    posResetNE is updated to hold the change in position
//    storedOutput, outputDataNew and outputDataDelayed are updated with the change in position
//    lastPosReset_ms is updated with the time of the reset
void NavEKF3_core::ResetPositionNE(float posN, float posE)
{
    // Store the position before the reset so that we can record the reset delta
    const Vector3f posOrig = stateStruct.position;

    // Set the position states to the new position
    stateStruct.position.x = posN;
    stateStruct.position.y = posE;

    // Calculate the position offset due to the reset
    posResetNE.x = stateStruct.position.x - posOrig.x;
    posResetNE.y = stateStruct.position.y - posOrig.y;

    // Add the offset to the output observer states
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x += posResetNE.x;
        storedOutput[i].position.y += posResetNE.y;
    }
    outputDataNew.position.x += posResetNE.x;
    outputDataNew.position.y += posResetNE.y;
    outputDataDelayed.position.x += posResetNE.x;
    outputDataDelayed.position.y += posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;
}

// reset the stateStruct's D position
//    posResetD is updated to hold the change in position
//    storedOutput, outputDataNew and outputDataDelayed are updated with the change in position
//    lastPosResetD_ms is updated with the time of the reset
void NavEKF3_core::ResetPositionD(float posD)
{
    // Store the position before the reset so that we can record the reset delta
    const float posDOrig = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = posD;

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posDOrig;

    // Add the offset to the output observer states
    outputDataNew.position.z += posResetD;
    vertCompFiltState.pos = outputDataNew.position.z;
    outputDataDelayed.position.z += posResetD;
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z += posResetD;
    }

    // store the time of the reset
    lastPosResetD_ms = imuSampleTime_ms;
}

// reset the vertical position state using the last height measurement
void NavEKF3_core::ResetHeight(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetD = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = -hgtMea;
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;

    // reset the terrain state height
    if (onGround) {
        // assume vehicle is sitting on the ground
        terrainState = stateStruct.position.z + rngOnGnd;
    } else {
        // can make no assumption other than vehicle is not below ground level
        terrainState = MAX(stateStruct.position.z + rngOnGnd , terrainState);
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }
    vertCompFiltState.pos = stateStruct.position.z;

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posResetD;

    // store the time of the reset
    lastPosResetD_ms = imuSampleTime_ms;

    // clear the timeout flags and counters
    hgtTimeout = false;
    lastHgtPassTime_ms = imuSampleTime_ms;

    // reset the corresponding covariances
    zeroRows(P,9,9);
    zeroCols(P,9,9);

    // set the variances to the measurement variance
    P[9][9] = posDownObsNoise;

    // Reset the vertical velocity state using GPS vertical velocity if we are airborne
    // Check that GPS vertical velocity data is available and can be used
    if (inFlight && !gpsNotAvailable && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
    } else if (onGround) {
        stateStruct.velocity.z = 0.0f;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.velocity.z = stateStruct.velocity.z;
    outputDataDelayed.velocity.z = stateStruct.velocity.z;
    vertCompFiltState.vel = outputDataNew.velocity.z;

    // reset the corresponding covariances
    zeroRows(P,6,6);
    zeroCols(P,6,6);

    // set the variances to the measurement variance
    P[6][6] = sq(frontend->_gpsVertVelNoise);

}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF3_core::resetHeightDatum(void)
{
    if (activeHgtSource == HGT_SOURCE_RNG || !onGround) {
        // only allow resets when on the ground.
        // If using using rangefinder for height then never perform a
        // reset of the height datum
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    AP::baro().update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same
    if (validOrigin) {
        if (!gpsGoodToAlign) {
            // if we don't have GPS lock then we shouldn't be doing a
            // resetHeightDatum, but if we do then the best option is
            // to maintain the old error
            EKF_origin.alt += (int32_t)(100.0f * oldHgt);
        } else {
            // if we have a good GPS lock then reset to the GPS
            // altitude. This ensures the reported AMSL alt from
            // getLLH() is equal to GPS altitude, while also ensuring
            // that the relative alt is zero
            EKF_origin.alt = AP::gps().location().alt;
        }
        ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    }

    // set the terrain state to zero (on ground). The adjustment for
    // frame height will get added in the later constraints
    terrainState = 0;

    return true;
}

/*
  correct GPS data for position offset of antenna phase centre relative to the IMU
 */
void NavEKF3_core::CorrectGPSForAntennaOffset(gps_elements &gps_data) const
{
    const Vector3f &posOffsetBody = AP::gps().get_antenna_offset(gps_data.sensor_idx) - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    if (fuseVelData) {
        // TODO use a filtered angular rate with a group delay that matches the GPS delay
        Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
        Vector3f velOffsetBody = angRate % posOffsetBody;
        Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
        gps_data.vel -= velOffsetEarth;
    }
    Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    gps_data.pos.x -= posOffsetEarth.x;
    gps_data.pos.y -= posOffsetEarth.y;
    gps_data.hgt += posOffsetEarth.z;
}

// correct external navigation earth-frame position using sensor body-frame offset
void NavEKF3_core::CorrectExtNavForSensorOffset(Vector3f &ext_position)
{
#if HAL_VISUALODOM_ENABLED
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    const Vector3f &posOffsetBody = visual_odom->get_pos_offset() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    ext_position.x -= posOffsetEarth.x;
    ext_position.y -= posOffsetEarth.y;
    ext_position.z -= posOffsetEarth.z;
#endif
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
void NavEKF3_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

    // Check for data at the fusion time horizon
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);

    // Read GPS data from the sensor
    readGpsData();

    // get data that has now fallen behind the fusion time horizon
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);

    // Determine if we need to fuse position and velocity data on this time step
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (frontend->_fusionModeGPS != 3)) {

        // Don't fuse velocity data if GPS doesn't support it
        if (frontend->_fusionModeGPS <= 1) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;
        extNavUsedForPos = false;

        CorrectGPSForAntennaOffset(gpsDataDelayed);

        // copy corrected GPS data to observation vector
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        velPosObs[3] = gpsDataDelayed.pos.x;
        velPosObs[4] = gpsDataDelayed.pos.y;
    } else if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (frontend->_fusionModeGPS == 3)) {
        // use external nav system for position
        extNavUsedForPos = true;
        activeHgtSource = HGT_SOURCE_EXTNAV;
        fuseVelData = false;
        fuseHgtData = true;
        fusePosData = true;

        // correct for external navigation sensor position
        CorrectExtNavForSensorOffset(extNavDataDelayed.pos);

        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
        velPosObs[5] = extNavDataDelayed.pos.z;
    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    if (gpsYawResetRequest) {
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources
    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (frontend->_fusionModeGPS != 3) && (gpsDataDelayed.sensor_idx != last_gps_idx)) {
        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // Store the position before the reset so that we can record the reset delta
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;

        // Set the position states to the position from the new GPS
        stateStruct.position.x = gpsDataDelayed.pos.x;
        stateStruct.position.y = gpsDataDelayed.pos.y;

        // Calculate the position offset due to the reset
        posResetNE.x = stateStruct.position.x - posResetNE.x;
        posResetNE.y = stateStruct.position.y - posResetNE.y;

        // Add the offset to the output observer states
        for (uint8_t i=0; i<imu_buffer_length; i++) {
            storedOutput[i].position.x += posResetNE.x;
            storedOutput[i].position.y += posResetNE.y;
        }
        outputDataNew.position.x += posResetNE.x;
        outputDataNew.position.y += posResetNE.y;
        outputDataDelayed.position.x += posResetNE.x;
        outputDataDelayed.position.y += posResetNE.y;

        // store the time of the reset
        lastPosReset_ms = imuSampleTime_ms;

        // If we are also using GPS as the height reference, reset the height
        if (activeHgtSource == HGT_SOURCE_GPS) {
            // Store the position before the reset so that we can record the reset delta
            posResetD = stateStruct.position.z;

            // write to the state vector
            stateStruct.position.z = -hgtMea;

            // Calculate the position jump due to the reset
            posResetD = stateStruct.position.z - posResetD;

            // Add the offset to the output observer states
            outputDataNew.position.z += posResetD;
            vertCompFiltState.pos = outputDataNew.position.z;
            outputDataDelayed.position.z += posResetD;
            for (uint8_t i=0; i<imu_buffer_length; i++) {
                storedOutput[i].position.z += posResetD;
            }

            // store the time of the reset
            lastPosResetD_ms = imuSampleTime_ms;
        }
    }

    // check for external nav position reset
    if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (frontend->_fusionModeGPS == 3) && extNavDataDelayed.posReset) {
        ResetPositionNE(extNavDataDelayed.pos.x, extNavDataDelayed.pos.y);
        if (activeHgtSource == HGT_SOURCE_EXTNAV) {
            ResetPositionD(-hgtMea);
        }
    }

    // If we are operating without any aiding, fuse in the last known position
    // to constrain tilt drift. This assumes a non-manoeuvring vehicle
    // Do this to coincide with the height fusion
    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        velPosObs[3] = lastKnownPositionNE.x;
        velPosObs[4] = lastKnownPositionNE.y;

        fusePosData = true;
        fuseVelData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        FuseVelPosNED();
        // clear the flags to prevent repeated fusion of the same data
        fuseVelData = false;
        fuseHgtData = false;
        fusePosData = false;
    }
}

// fuse selected position, velocity and height measurements
void NavEKF3_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // calculate additional error in GPS position caused by manoeuvring
        float posErr = frontend->gpsPosVarAccScale * accNavMag;

        // To-Do: this posErr should come from external nav when fusing external nav position

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {
                // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
                R_OBS[0] = sq(constrain_float(frontend->_noaidHorizNoise, 0.5f, 50.0f));
            } else {
                // Use a smaller value to give faster initial alignment
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else {
            if (gpsSpdAccuracy > 0.0f) {
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                R_OBS[0] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_float(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_float(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else if (extNavUsedForPos) {
                R_OBS[3] = sq(constrain_float(extNavDataDelayed.posErr, 0.01f, 10.0f));
            } else {
                R_OBS[3] = sq(constrain_float(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPS velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPS performance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
        }
        R_OBS[5] = posDownObsNoise;
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (useGpsVertVel && fuseVelData && (frontend->_altSource != 2)) {
            // calculate innovations for height and vertical GPS vel measurements
            const float hgtErr  = stateStruct.position.z - velPosObs[5];
            const float velDErr = stateStruct.velocity.z - velPosObs[2];
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
            innovVelPos[3] = stateStruct.position.x - velPosObs[3];
            innovVelPos[4] = stateStruct.position.y - velPosObs[4];
            varInnovVelPos[3] = P[7][7] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            float maxPosInnov2 = sq(MAX(0.01f * (float)frontend->_gpsPosInnovGate, 1.0f))*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // use position data if healthy or timed out
            if (PV_AidingMode == AID_NONE) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
            } else if (posHealth || posTimeout) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
                // if timed out or outside the specified uncertainty radius, reset to the GPS
                if (posTimeout || ((P[8][8] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current GPS position
                    ResetPosition();
                    // reset the velocity to the GPS velocity
                    ResetVelocity();
                    // don't fuse GPS data on this time step
                    fusePosData = false;
                    fuseVelData = false;
                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    zeroRows(P,7,8);
                    zeroCols(P,7,8);
                    P[7][7] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[8][8] = P[7][7];
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    posTestRatio = 0.0f;
                    velTestRatio = 0.0f;
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            // Don't fuse vertical velocity observations if inhibited by the user or if we are using synthetic data
            if (frontend->_fusionModeGPS > 0 || PV_AidingMode != AID_ABSOLUTE || frontend->inhibitGpsVertVelUse) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 4
                stateIndex   = i + 4;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = stateStruct.velocity[i] - velPosObs[i]; // blended
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01f * (float)frontend->_gpsVelInnovGate, 1.0f)));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // use velocity data if healthy, timed out, or in constant position mode
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count
                lastVelPassTime_ms = imuSampleTime_ms;
                // If we are doing full aiding and velocity fusion times out, reset to the GPS velocity
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    // reset the velocity to the GPS velocity
                    ResetVelocity();
                    // don't fuse GPS velocity data on this time step
                    fuseVelData = false;
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    velTestRatio = 0.0f;
                }
            } else {
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // calculate height innovations
            innovVelPos[5] = stateStruct.position.z - velPosObs[5];
            varInnovVelPos[5] = P[9][9] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01f * (float)frontend->_hgtInnovGate, 1.0f)) * varInnovVelPos[5]);

            // when on ground we accept a larger test ratio to allow
            // the filter to handle large switch on IMU bias errors
            // without rejecting the height sensor
            const float maxTestRatio = (PV_AidingMode == AID_NONE && onGround)? 3.0 : 1.0;

            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < maxTestRatio) || badIMUdata);

            // Fuse height data if healthy or timed out or in constant position mode
            if (hgtHealth || hgtTimeout) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (onGround) {
                    float dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms)*1.0e-3f;
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrain_float(dtBaro/(dtBaro+hgtInnovFiltTC),0.0f,1.0f);
                    hgtInnovFiltState += (innovVelPos[5]-hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                // if timed out, reset the height
                if (hgtTimeout) {
                    ResetHeight();
                }

                // If we have got this far then declare the height data as healthy and reset the timeout counter
                hgtHealth = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData && velHealth) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {
                fuseData[2] = true;
            }
        }
        if (fusePosData && posHealth) {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2) {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 5) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    const float gndMaxBaroErr = 4.0f;
                    const float gndBaroInnovFloor = -0.5f;

                    if(getTouchdownExpected() && activeHgtSource == HGT_SOURCE_BARO) {
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

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=9; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit delta angle bias state estmation by setting Kalman gains to zero
                if (!inhibitDelAngBiasStates) {
                    for (uint8_t i = 10; i<=12; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    // zero indexes 10 to 12 = 3*4 bytes
                    memset(&Kfusion[10], 0, 12);
                }

                // inhibit delta velocity bias state estimation by setting Kalman gains to zero
                if (!inhibitDelVelBiasStates) {
                    for (uint8_t i = 13; i<=15; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    // zero indexes 13 to 15 = 3*4 bytes
                    memset(&Kfusion[13], 0, 12);
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    // zero indexes 16 to 21 = 6*4 bytes
                    memset(&Kfusion[16], 0, 24);
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    // zero indexes 22 to 23 = 2*4 bytes
                    memset(&Kfusion[22], 0, 8);
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                // Check that we are not going to drive any variances negative and skip the update if so
                bool healthyFusion = true;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    if (KHP[i][i] > P[i][i]) {
                        healthyFusion = false;
                    }
                }
                if (healthyFusion) {
                    // update the covariance matrix
                    for (uint8_t i= 0; i<=stateIndexLim; i++) {
                        for (uint8_t j= 0; j<=stateIndexLim; j++) {
                            P[i][j] = P[i][j] - KHP[i][j];
                        }
                    }

                    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
                    ForceSymmetry();
                    ConstrainVariances();

                    // update states and renormalise the quaternions
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }
                    stateStruct.quat.normalize();

                    // record good fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = false;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = false;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = false;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = false;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = false;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = false;
                    }
                } else {
                    // record bad fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = true;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = true;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = true;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = true;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = true;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = true;
                    }
                }
            }
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVelPosNED);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// select the height measurement to be fused from the available baro, range finder and GPS sources
void NavEKF3_core::selectHeightForFusion()
{
    // Read range finder data and check for new data in the buffer
    // This data is used by both height and optical flow fusion processing
    readRangeFinder();
    rangeDataToFuse = storedRange.recall(rangeDataDelayed,imuDataDelayed.time_ms);

    // correct range data for the body frame position offset relative to the IMU
    // the corrected reading is the reading that would have been taken if the sensor was
    // co-located with the IMU
    const RangeFinder *_rng = AP::rangefinder();
    if (_rng && rangeDataToFuse) {
        AP_RangeFinder_Backend *sensor = _rng->get_backend(rangeDataDelayed.sensor_idx);
        if (sensor != nullptr) {
            Vector3f posOffsetBody = sensor->get_pos_offset() - accelPosOffset;
            if (!posOffsetBody.is_zero()) {
                Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
                rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
            }
        }
    }

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    // select height source
    if (extNavUsedForPos && (frontend->_altSource == 4)) {
        // always use external navigation as the height source if using for position.
        activeHgtSource = HGT_SOURCE_EXTNAV;
    } else if (_rng && ((frontend->_useRngSwHgt > 0) && (frontend->_altSource == 1)) && (imuSampleTime_ms - rngValidMeaTime_ms < 500)) {
        if (frontend->_altSource == 1) {
            // always use range finder
            activeHgtSource = HGT_SOURCE_RNG;
        } else {
            // determine if we are above or below the height switch region
            float rangeMaxUse = 1e-4f * (float)_rng->max_distance_cm_orient(ROTATION_PITCH_270) * (float)frontend->_useRngSwHgt;
            bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
            bool belowLowerSwHgt = (terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse;

            // If the terrain height is consistent and we are moving slowly, then it can be
            // used as a height reference in combination with a range finder
            // apply a hysteresis to the speed check to prevent rapid switching
            bool dontTrustTerrain, trustTerrain;
            if (filterStatus.flags.horiz_vel) {
                // We can use the velocity estimate
                float horizSpeed = norm(stateStruct.velocity.x, stateStruct.velocity.y);
                dontTrustTerrain = (horizSpeed > frontend->_useRngSwSpd) || !terrainHgtStable;
                float trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
                trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;
            } else {
                // We can't use the velocity estimate
                dontTrustTerrain = !terrainHgtStable;
                trustTerrain = terrainHgtStable;
            }

            /*
             * Switch between range finder and primary height source using height above ground and speed thresholds with
             * hysteresis to avoid rapid switching. Using range finder for height requires a consistent terrain height
             * which cannot be assumed if the vehicle is moving horizontally.
            */
            if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == HGT_SOURCE_RNG)) {
                // cannot trust terrain or range finder so stop using range finder height
                if (frontend->_altSource == 0) {
                    activeHgtSource = HGT_SOURCE_BARO;
                } else if (frontend->_altSource == 2) {
                    activeHgtSource = HGT_SOURCE_GPS;
                }
            } else if (belowLowerSwHgt && trustTerrain && (activeHgtSource != HGT_SOURCE_RNG)) {
                // reliable terrain and range finder so start using range finder height
                activeHgtSource = HGT_SOURCE_RNG;
            }
        }
    } else if ((frontend->_altSource == 2) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = HGT_SOURCE_GPS;
    } else if ((frontend->_altSource == 3) && validOrigin && rngBcnGoodToAlign) {
        activeHgtSource = HGT_SOURCE_BCN;
    } else {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // Use Baro alt as a fallback if we lose range finder, GPS or external nav
    bool lostRngHgt = ((activeHgtSource == HGT_SOURCE_RNG) && ((imuSampleTime_ms - rngValidMeaTime_ms) > 500));
    bool lostGpsHgt = ((activeHgtSource == HGT_SOURCE_GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000));
    bool lostExtNavHgt = ((activeHgtSource == HGT_SOURCE_EXTNAV) && ((imuSampleTime_ms - extNavMeasTime_ms) > 2000));
    if (lostRngHgt || lostGpsHgt || lostExtNavHgt) {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        if  (activeHgtSource != HGT_SOURCE_BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = frontend->hgtAvg_ms*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // If we are not using GPS as the primary height sensor, correct EKF origin height so that
    // combined local NED position height and origin height remains consistent with the GPS altitude
    // This also enables the GPS height to be used as a backup height source
    if (gpsDataToFuse &&
            (((frontend->_originHgtMode & (1 << 0)) && (activeHgtSource == HGT_SOURCE_BARO)) ||
            ((frontend->_originHgtMode & (1 << 1)) && (activeHgtSource == HGT_SOURCE_RNG)))
            ) {
            correctEkfOriginHeight();
    }

    // Select the height measurement source
    if (extNavDataToFuse && (activeHgtSource == HGT_SOURCE_EXTNAV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        posDownObsNoise = sq(constrain_float(extNavDataDelayed.posErr, 0.1f, 10.0f));
    } else if (rangeDataToFuse && (activeHgtSource == HGT_SOURCE_RNG)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            velPosObs[5] = -hgtMea;
            // enable fusion
            fuseHgtData = true;
            // set the observation noise
            posDownObsNoise = sq(constrain_float(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if  (gpsDataToFuse && (activeHgtSource == HGT_SOURCE_GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        velPosObs[5] = -hgtMea;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_float(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_float(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == HGT_SOURCE_BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise
        posDownObsNoise = sq(constrain_float(frontend->_baroAltNoise, 0.1f, 10.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if (getTakeoffExpected() || getTouchdownExpected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (motorsArmed && getTakeoffExpected()) {
            hgtMea = MAX(hgtMea, meaHgtAtTakeOff);
        }
        velPosObs[5] = -hgtMea;
    } else {
        fuseHgtData = false;
    }

    // If we haven't fused height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = (useGpsVertVel && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

/*
 * Fuse body frame velocity measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseBodyVel()
{
    Vector24 H_VEL;
    Vector3f bodyVelPred;

    // Copy required states to local variable names
    float q0  = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];
    float vn = stateStruct.velocity.x;
    float ve = stateStruct.velocity.y;
    float vd = stateStruct.velocity.z;

    // Fuse X, Y and Z axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=2; obsIndex++) {

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        bodyVelPred = (prevTnb * stateStruct.velocity);

        // correct sensor offset body frame position offset relative to IMU
        Vector3f posOffsetBody = (*bodyOdmDataDelayed.body_offset) - accelPosOffset;

        // correct prediction for relative motion due to rotation
        // note - % operator overloaded for cross product
        if (imuDataDelayed.delAngDT > 0.001f) {
            bodyVelPred += (imuDataDelayed.delAng * (1.0f / imuDataDelayed.delAngDT)) % posOffsetBody;
        }

        // calculate observation jacobians and Kalman gains
        if (obsIndex == 0) {
            // calculate X axis observation Jacobian
            H_VEL[0] = q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f;
            H_VEL[1] = q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f;
            H_VEL[2] = q0*vd*-2.0f+q1*ve*2.0f-q2*vn*2.0f;
            H_VEL[3] = q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f;
            H_VEL[4] = q0*q0+q1*q1-q2*q2-q3*q3;
            H_VEL[5] = q0*q3*2.0f+q1*q2*2.0f;
            H_VEL[6] = q0*q2*-2.0f+q1*q3*2.0f;
            for (uint8_t index = 7; index < 24; index++) {
                H_VEL[index] = 0.0f;
            }

            // calculate intermediate expressions for X axis Kalman gains
            float R_VEL = sq(bodyOdmDataDelayed.velErr);
            float t2 = q0*q3*2.0f;
            float t3 = q1*q2*2.0f;
            float t4 = t2+t3;
            float t5 = q0*q0;
            float t6 = q1*q1;
            float t7 = q2*q2;
            float t8 = q3*q3;
            float t9 = t5+t6-t7-t8;
            float t10 = q0*q2*2.0f;
            float t25 = q1*q3*2.0f;
            float t11 = t10-t25;
            float t12 = q3*ve*2.0f;
            float t13 = q0*vn*2.0f;
            float t26 = q2*vd*2.0f;
            float t14 = t12+t13-t26;
            float t15 = q3*vd*2.0f;
            float t16 = q2*ve*2.0f;
            float t17 = q1*vn*2.0f;
            float t18 = t15+t16+t17;
            float t19 = q0*vd*2.0f;
            float t20 = q2*vn*2.0f;
            float t27 = q1*ve*2.0f;
            float t21 = t19+t20-t27;
            float t22 = q1*vd*2.0f;
            float t23 = q0*ve*2.0f;
            float t28 = q3*vn*2.0f;
            float t24 = t22+t23-t28;
            float t29 = P[0][0]*t14;
            float t30 = P[1][1]*t18;
            float t31 = P[4][5]*t9;
            float t32 = P[5][5]*t4;
            float t33 = P[0][5]*t14;
            float t34 = P[1][5]*t18;
            float t35 = P[3][5]*t24;
            float t79 = P[6][5]*t11;
            float t80 = P[2][5]*t21;
            float t36 = t31+t32+t33+t34+t35-t79-t80;
            float t37 = t4*t36;
            float t38 = P[4][6]*t9;
            float t39 = P[5][6]*t4;
            float t40 = P[0][6]*t14;
            float t41 = P[1][6]*t18;
            float t42 = P[3][6]*t24;
            float t81 = P[6][6]*t11;
            float t82 = P[2][6]*t21;
            float t43 = t38+t39+t40+t41+t42-t81-t82;
            float t44 = P[4][0]*t9;
            float t45 = P[5][0]*t4;
            float t46 = P[1][0]*t18;
            float t47 = P[3][0]*t24;
            float t84 = P[6][0]*t11;
            float t85 = P[2][0]*t21;
            float t48 = t29+t44+t45+t46+t47-t84-t85;
            float t49 = t14*t48;
            float t50 = P[4][1]*t9;
            float t51 = P[5][1]*t4;
            float t52 = P[0][1]*t14;
            float t53 = P[3][1]*t24;
            float t86 = P[6][1]*t11;
            float t87 = P[2][1]*t21;
            float t54 = t30+t50+t51+t52+t53-t86-t87;
            float t55 = t18*t54;
            float t56 = P[4][2]*t9;
            float t57 = P[5][2]*t4;
            float t58 = P[0][2]*t14;
            float t59 = P[1][2]*t18;
            float t60 = P[3][2]*t24;
            float t78 = P[2][2]*t21;
            float t88 = P[6][2]*t11;
            float t61 = t56+t57+t58+t59+t60-t78-t88;
            float t62 = P[4][3]*t9;
            float t63 = P[5][3]*t4;
            float t64 = P[0][3]*t14;
            float t65 = P[1][3]*t18;
            float t66 = P[3][3]*t24;
            float t90 = P[6][3]*t11;
            float t91 = P[2][3]*t21;
            float t67 = t62+t63+t64+t65+t66-t90-t91;
            float t68 = t24*t67;
            float t69 = P[4][4]*t9;
            float t70 = P[5][4]*t4;
            float t71 = P[0][4]*t14;
            float t72 = P[1][4]*t18;
            float t73 = P[3][4]*t24;
            float t92 = P[6][4]*t11;
            float t93 = P[2][4]*t21;
            float t74 = t69+t70+t71+t72+t73-t92-t93;
            float t75 = t9*t74;
            float t83 = t11*t43;
            float t89 = t21*t61;
            float t76 = R_VEL+t37+t49+t55+t68+t75-t83-t89;
            float t77;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t76 > R_VEL) {
                t77 = 1.0f/t76;
                faultStatus.bad_xvel = false;
            } else {
                t76 = R_VEL;
                t77 = 1.0f/R_VEL;
                faultStatus.bad_xvel = true;
                return;
            }
            varInnovBodyVel[0] = t76;

            // calculate innovation for X axis observation
            innovBodyVel[0] = bodyVelPred.x - bodyOdmDataDelayed.vel.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t77*(t29+P[0][5]*t4+P[0][4]*t9-P[0][6]*t11+P[0][1]*t18-P[0][2]*t21+P[0][3]*t24);
            Kfusion[1] = t77*(t30+P[1][5]*t4+P[1][4]*t9+P[1][0]*t14-P[1][6]*t11-P[1][2]*t21+P[1][3]*t24);
            Kfusion[2] = t77*(-t78+P[2][5]*t4+P[2][4]*t9+P[2][0]*t14-P[2][6]*t11+P[2][1]*t18+P[2][3]*t24);
            Kfusion[3] = t77*(t66+P[3][5]*t4+P[3][4]*t9+P[3][0]*t14-P[3][6]*t11+P[3][1]*t18-P[3][2]*t21);
            Kfusion[4] = t77*(t69+P[4][5]*t4+P[4][0]*t14-P[4][6]*t11+P[4][1]*t18-P[4][2]*t21+P[4][3]*t24);
            Kfusion[5] = t77*(t32+P[5][4]*t9+P[5][0]*t14-P[5][6]*t11+P[5][1]*t18-P[5][2]*t21+P[5][3]*t24);
            Kfusion[6] = t77*(-t81+P[6][5]*t4+P[6][4]*t9+P[6][0]*t14+P[6][1]*t18-P[6][2]*t21+P[6][3]*t24);
            Kfusion[7] = t77*(P[7][5]*t4+P[7][4]*t9+P[7][0]*t14-P[7][6]*t11+P[7][1]*t18-P[7][2]*t21+P[7][3]*t24);
            Kfusion[8] = t77*(P[8][5]*t4+P[8][4]*t9+P[8][0]*t14-P[8][6]*t11+P[8][1]*t18-P[8][2]*t21+P[8][3]*t24);
            Kfusion[9] = t77*(P[9][5]*t4+P[9][4]*t9+P[9][0]*t14-P[9][6]*t11+P[9][1]*t18-P[9][2]*t21+P[9][3]*t24);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = t77*(P[10][5]*t4+P[10][4]*t9+P[10][0]*t14-P[10][6]*t11+P[10][1]*t18-P[10][2]*t21+P[10][3]*t24);
                Kfusion[11] = t77*(P[11][5]*t4+P[11][4]*t9+P[11][0]*t14-P[11][6]*t11+P[11][1]*t18-P[11][2]*t21+P[11][3]*t24);
                Kfusion[12] = t77*(P[12][5]*t4+P[12][4]*t9+P[12][0]*t14-P[12][6]*t11+P[12][1]*t18-P[12][2]*t21+P[12][3]*t24);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = t77*(P[13][5]*t4+P[13][4]*t9+P[13][0]*t14-P[13][6]*t11+P[13][1]*t18-P[13][2]*t21+P[13][3]*t24);
                Kfusion[14] = t77*(P[14][5]*t4+P[14][4]*t9+P[14][0]*t14-P[14][6]*t11+P[14][1]*t18-P[14][2]*t21+P[14][3]*t24);
                Kfusion[15] = t77*(P[15][5]*t4+P[15][4]*t9+P[15][0]*t14-P[15][6]*t11+P[15][1]*t18-P[15][2]*t21+P[15][3]*t24);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(P[16][5]*t4+P[16][4]*t9+P[16][0]*t14-P[16][6]*t11+P[16][1]*t18-P[16][2]*t21+P[16][3]*t24);
                Kfusion[17] = t77*(P[17][5]*t4+P[17][4]*t9+P[17][0]*t14-P[17][6]*t11+P[17][1]*t18-P[17][2]*t21+P[17][3]*t24);
                Kfusion[18] = t77*(P[18][5]*t4+P[18][4]*t9+P[18][0]*t14-P[18][6]*t11+P[18][1]*t18-P[18][2]*t21+P[18][3]*t24);
                Kfusion[19] = t77*(P[19][5]*t4+P[19][4]*t9+P[19][0]*t14-P[19][6]*t11+P[19][1]*t18-P[19][2]*t21+P[19][3]*t24);
                Kfusion[20] = t77*(P[20][5]*t4+P[20][4]*t9+P[20][0]*t14-P[20][6]*t11+P[20][1]*t18-P[20][2]*t21+P[20][3]*t24);
                Kfusion[21] = t77*(P[21][5]*t4+P[21][4]*t9+P[21][0]*t14-P[21][6]*t11+P[21][1]*t18-P[21][2]*t21+P[21][3]*t24);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(P[22][5]*t4+P[22][4]*t9+P[22][0]*t14-P[22][6]*t11+P[22][1]*t18-P[22][2]*t21+P[22][3]*t24);
                Kfusion[23] = t77*(P[23][5]*t4+P[23][4]*t9+P[23][0]*t14-P[23][6]*t11+P[23][1]*t18-P[23][2]*t21+P[23][3]*t24);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }
        } else if (obsIndex == 1) {
            // calculate Y axis observation Jacobian
            H_VEL[0] = q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f;
            H_VEL[1] = q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f;
            H_VEL[2] = q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f;
            H_VEL[3] = q2*vd*2.0f-q3*ve*2.0f-q0*vn*2.0f;
            H_VEL[4] = q0*q3*-2.0f+q1*q2*2.0f;
            H_VEL[5] = q0*q0-q1*q1+q2*q2-q3*q3;
            H_VEL[6] = q0*q1*2.0f+q2*q3*2.0f;
            for (uint8_t index = 7; index < 24; index++) {
                H_VEL[index] = 0.0f;
            }

            // calculate intermediate expressions for Y axis Kalman gains
            float R_VEL = sq(bodyOdmDataDelayed.velErr);
            float t2 = q0*q3*2.0f;
            float t9 = q1*q2*2.0f;
            float t3 = t2-t9;
            float t4 = q0*q0;
            float t5 = q1*q1;
            float t6 = q2*q2;
            float t7 = q3*q3;
            float t8 = t4-t5+t6-t7;
            float t10 = q0*q1*2.0f;
            float t11 = q2*q3*2.0f;
            float t12 = t10+t11;
            float t13 = q1*vd*2.0f;
            float t14 = q0*ve*2.0f;
            float t26 = q3*vn*2.0f;
            float t15 = t13+t14-t26;
            float t16 = q0*vd*2.0f;
            float t17 = q2*vn*2.0f;
            float t27 = q1*ve*2.0f;
            float t18 = t16+t17-t27;
            float t19 = q3*vd*2.0f;
            float t20 = q2*ve*2.0f;
            float t21 = q1*vn*2.0f;
            float t22 = t19+t20+t21;
            float t23 = q3*ve*2.0f;
            float t24 = q0*vn*2.0f;
            float t28 = q2*vd*2.0f;
            float t25 = t23+t24-t28;
            float t29 = P[0][0]*t15;
            float t30 = P[1][1]*t18;
            float t31 = P[5][4]*t8;
            float t32 = P[6][4]*t12;
            float t33 = P[0][4]*t15;
            float t34 = P[1][4]*t18;
            float t35 = P[2][4]*t22;
            float t78 = P[4][4]*t3;
            float t79 = P[3][4]*t25;
            float t36 = t31+t32+t33+t34+t35-t78-t79;
            float t37 = P[5][6]*t8;
            float t38 = P[6][6]*t12;
            float t39 = P[0][6]*t15;
            float t40 = P[1][6]*t18;
            float t41 = P[2][6]*t22;
            float t81 = P[4][6]*t3;
            float t82 = P[3][6]*t25;
            float t42 = t37+t38+t39+t40+t41-t81-t82;
            float t43 = t12*t42;
            float t44 = P[5][0]*t8;
            float t45 = P[6][0]*t12;
            float t46 = P[1][0]*t18;
            float t47 = P[2][0]*t22;
            float t83 = P[4][0]*t3;
            float t84 = P[3][0]*t25;
            float t48 = t29+t44+t45+t46+t47-t83-t84;
            float t49 = t15*t48;
            float t50 = P[5][1]*t8;
            float t51 = P[6][1]*t12;
            float t52 = P[0][1]*t15;
            float t53 = P[2][1]*t22;
            float t85 = P[4][1]*t3;
            float t86 = P[3][1]*t25;
            float t54 = t30+t50+t51+t52+t53-t85-t86;
            float t55 = t18*t54;
            float t56 = P[5][2]*t8;
            float t57 = P[6][2]*t12;
            float t58 = P[0][2]*t15;
            float t59 = P[1][2]*t18;
            float t60 = P[2][2]*t22;
            float t87 = P[4][2]*t3;
            float t88 = P[3][2]*t25;
            float t61 = t56+t57+t58+t59+t60-t87-t88;
            float t62 = t22*t61;
            float t63 = P[5][3]*t8;
            float t64 = P[6][3]*t12;
            float t65 = P[0][3]*t15;
            float t66 = P[1][3]*t18;
            float t67 = P[2][3]*t22;
            float t89 = P[4][3]*t3;
            float t90 = P[3][3]*t25;
            float t68 = t63+t64+t65+t66+t67-t89-t90;
            float t69 = P[5][5]*t8;
            float t70 = P[6][5]*t12;
            float t71 = P[0][5]*t15;
            float t72 = P[1][5]*t18;
            float t73 = P[2][5]*t22;
            float t92 = P[4][5]*t3;
            float t93 = P[3][5]*t25;
            float t74 = t69+t70+t71+t72+t73-t92-t93;
            float t75 = t8*t74;
            float t80 = t3*t36;
            float t91 = t25*t68;
            float t76 = R_VEL+t43+t49+t55+t62+t75-t80-t91;
            float t77;

            // calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
            if (t76 > R_VEL) {
                t77 = 1.0f/t76;
                faultStatus.bad_yvel = false;
            } else {
                t76 = R_VEL;
                t77 = 1.0f/R_VEL;
                faultStatus.bad_yvel = true;
                return;
            }
            varInnovBodyVel[1] = t76;

            // calculate innovation for Y axis observation
            innovBodyVel[1] = bodyVelPred.y - bodyOdmDataDelayed.vel.y;

            // calculate Kalman gains for Y-axis observation
            Kfusion[0] = t77*(t29-P[0][4]*t3+P[0][5]*t8+P[0][6]*t12+P[0][1]*t18+P[0][2]*t22-P[0][3]*t25);
            Kfusion[1] = t77*(t30-P[1][4]*t3+P[1][5]*t8+P[1][0]*t15+P[1][6]*t12+P[1][2]*t22-P[1][3]*t25);
            Kfusion[2] = t77*(t60-P[2][4]*t3+P[2][5]*t8+P[2][0]*t15+P[2][6]*t12+P[2][1]*t18-P[2][3]*t25);
            Kfusion[3] = t77*(-t90-P[3][4]*t3+P[3][5]*t8+P[3][0]*t15+P[3][6]*t12+P[3][1]*t18+P[3][2]*t22);
            Kfusion[4] = t77*(-t78+P[4][5]*t8+P[4][0]*t15+P[4][6]*t12+P[4][1]*t18+P[4][2]*t22-P[4][3]*t25);
            Kfusion[5] = t77*(t69-P[5][4]*t3+P[5][0]*t15+P[5][6]*t12+P[5][1]*t18+P[5][2]*t22-P[5][3]*t25);
            Kfusion[6] = t77*(t38-P[6][4]*t3+P[6][5]*t8+P[6][0]*t15+P[6][1]*t18+P[6][2]*t22-P[6][3]*t25);
            Kfusion[7] = t77*(-P[7][4]*t3+P[7][5]*t8+P[7][0]*t15+P[7][6]*t12+P[7][1]*t18+P[7][2]*t22-P[7][3]*t25);
            Kfusion[8] = t77*(-P[8][4]*t3+P[8][5]*t8+P[8][0]*t15+P[8][6]*t12+P[8][1]*t18+P[8][2]*t22-P[8][3]*t25);
            Kfusion[9] = t77*(-P[9][4]*t3+P[9][5]*t8+P[9][0]*t15+P[9][6]*t12+P[9][1]*t18+P[9][2]*t22-P[9][3]*t25);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = t77*(-P[10][4]*t3+P[10][5]*t8+P[10][0]*t15+P[10][6]*t12+P[10][1]*t18+P[10][2]*t22-P[10][3]*t25);
                Kfusion[11] = t77*(-P[11][4]*t3+P[11][5]*t8+P[11][0]*t15+P[11][6]*t12+P[11][1]*t18+P[11][2]*t22-P[11][3]*t25);
                Kfusion[12] = t77*(-P[12][4]*t3+P[12][5]*t8+P[12][0]*t15+P[12][6]*t12+P[12][1]*t18+P[12][2]*t22-P[12][3]*t25);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = t77*(-P[13][4]*t3+P[13][5]*t8+P[13][0]*t15+P[13][6]*t12+P[13][1]*t18+P[13][2]*t22-P[13][3]*t25);
                Kfusion[14] = t77*(-P[14][4]*t3+P[14][5]*t8+P[14][0]*t15+P[14][6]*t12+P[14][1]*t18+P[14][2]*t22-P[14][3]*t25);
                Kfusion[15] = t77*(-P[15][4]*t3+P[15][5]*t8+P[15][0]*t15+P[15][6]*t12+P[15][1]*t18+P[15][2]*t22-P[15][3]*t25);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(-P[16][4]*t3+P[16][5]*t8+P[16][0]*t15+P[16][6]*t12+P[16][1]*t18+P[16][2]*t22-P[16][3]*t25);
                Kfusion[17] = t77*(-P[17][4]*t3+P[17][5]*t8+P[17][0]*t15+P[17][6]*t12+P[17][1]*t18+P[17][2]*t22-P[17][3]*t25);
                Kfusion[18] = t77*(-P[18][4]*t3+P[18][5]*t8+P[18][0]*t15+P[18][6]*t12+P[18][1]*t18+P[18][2]*t22-P[18][3]*t25);
                Kfusion[19] = t77*(-P[19][4]*t3+P[19][5]*t8+P[19][0]*t15+P[19][6]*t12+P[19][1]*t18+P[19][2]*t22-P[19][3]*t25);
                Kfusion[20] = t77*(-P[20][4]*t3+P[20][5]*t8+P[20][0]*t15+P[20][6]*t12+P[20][1]*t18+P[20][2]*t22-P[20][3]*t25);
                Kfusion[21] = t77*(-P[21][4]*t3+P[21][5]*t8+P[21][0]*t15+P[21][6]*t12+P[21][1]*t18+P[21][2]*t22-P[21][3]*t25);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(-P[22][4]*t3+P[22][5]*t8+P[22][0]*t15+P[22][6]*t12+P[22][1]*t18+P[22][2]*t22-P[22][3]*t25);
                Kfusion[23] = t77*(-P[23][4]*t3+P[23][5]*t8+P[23][0]*t15+P[23][6]*t12+P[23][1]*t18+P[23][2]*t22-P[23][3]*t25);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }
        } else if (obsIndex == 2) {
            // calculate Z axis observation Jacobian
            H_VEL[0] = q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f;
            H_VEL[1] = q1*vd*-2.0f-q0*ve*2.0f+q3*vn*2.0f;
            H_VEL[2] = q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f;
            H_VEL[3] = q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f;
            H_VEL[4] = q0*q2*2.0f+q1*q3*2.0f;
            H_VEL[5] = q0*q1*-2.0f+q2*q3*2.0f;
            H_VEL[6] = q0*q0-q1*q1-q2*q2+q3*q3;
            for (uint8_t index = 7; index < 24; index++) {
                H_VEL[index] = 0.0f;
            }

            // calculate intermediate expressions for Z axis Kalman gains
            float R_VEL = sq(bodyOdmDataDelayed.velErr);
            float t2 = q0*q2*2.0f;
            float t3 = q1*q3*2.0f;
            float t4 = t2+t3;
            float t5 = q0*q0;
            float t6 = q1*q1;
            float t7 = q2*q2;
            float t8 = q3*q3;
            float t9 = t5-t6-t7+t8;
            float t10 = q0*q1*2.0f;
            float t25 = q2*q3*2.0f;
            float t11 = t10-t25;
            float t12 = q0*vd*2.0f;
            float t13 = q2*vn*2.0f;
            float t26 = q1*ve*2.0f;
            float t14 = t12+t13-t26;
            float t15 = q1*vd*2.0f;
            float t16 = q0*ve*2.0f;
            float t27 = q3*vn*2.0f;
            float t17 = t15+t16-t27;
            float t18 = q3*ve*2.0f;
            float t19 = q0*vn*2.0f;
            float t28 = q2*vd*2.0f;
            float t20 = t18+t19-t28;
            float t21 = q3*vd*2.0f;
            float t22 = q2*ve*2.0f;
            float t23 = q1*vn*2.0f;
            float t24 = t21+t22+t23;
            float t29 = P[0][0]*t14;
            float t30 = P[6][4]*t9;
            float t31 = P[4][4]*t4;
            float t32 = P[0][4]*t14;
            float t33 = P[2][4]*t20;
            float t34 = P[3][4]*t24;
            float t78 = P[5][4]*t11;
            float t79 = P[1][4]*t17;
            float t35 = t30+t31+t32+t33+t34-t78-t79;
            float t36 = t4*t35;
            float t37 = P[6][5]*t9;
            float t38 = P[4][5]*t4;
            float t39 = P[0][5]*t14;
            float t40 = P[2][5]*t20;
            float t41 = P[3][5]*t24;
            float t80 = P[5][5]*t11;
            float t81 = P[1][5]*t17;
            float t42 = t37+t38+t39+t40+t41-t80-t81;
            float t43 = P[6][0]*t9;
            float t44 = P[4][0]*t4;
            float t45 = P[2][0]*t20;
            float t46 = P[3][0]*t24;
            float t83 = P[5][0]*t11;
            float t84 = P[1][0]*t17;
            float t47 = t29+t43+t44+t45+t46-t83-t84;
            float t48 = t14*t47;
            float t49 = P[6][1]*t9;
            float t50 = P[4][1]*t4;
            float t51 = P[0][1]*t14;
            float t52 = P[2][1]*t20;
            float t53 = P[3][1]*t24;
            float t85 = P[5][1]*t11;
            float t86 = P[1][1]*t17;
            float t54 = t49+t50+t51+t52+t53-t85-t86;
            float t55 = P[6][2]*t9;
            float t56 = P[4][2]*t4;
            float t57 = P[0][2]*t14;
            float t58 = P[2][2]*t20;
            float t59 = P[3][2]*t24;
            float t88 = P[5][2]*t11;
            float t89 = P[1][2]*t17;
            float t60 = t55+t56+t57+t58+t59-t88-t89;
            float t61 = t20*t60;
            float t62 = P[6][3]*t9;
            float t63 = P[4][3]*t4;
            float t64 = P[0][3]*t14;
            float t65 = P[2][3]*t20;
            float t66 = P[3][3]*t24;
            float t90 = P[5][3]*t11;
            float t91 = P[1][3]*t17;
            float t67 = t62+t63+t64+t65+t66-t90-t91;
            float t68 = t24*t67;
            float t69 = P[6][6]*t9;
            float t70 = P[4][6]*t4;
            float t71 = P[0][6]*t14;
            float t72 = P[2][6]*t20;
            float t73 = P[3][6]*t24;
            float t92 = P[5][6]*t11;
            float t93 = P[1][6]*t17;
            float t74 = t69+t70+t71+t72+t73-t92-t93;
            float t75 = t9*t74;
            float t82 = t11*t42;
            float t87 = t17*t54;
            float t76 = R_VEL+t36+t48+t61+t68+t75-t82-t87;
            float t77;

            // calculate innovation variance for Z axis observation and protect against a badly conditioned calculation
            if (t76 > R_VEL) {
                t77 = 1.0f/t76;
                faultStatus.bad_zvel = false;
            } else {
                t76 = R_VEL;
                t77 = 1.0f/R_VEL;
                faultStatus.bad_zvel = true;
                return;
            }
            varInnovBodyVel[2] = t76;

            // calculate innovation for Z axis observation
            innovBodyVel[2] = bodyVelPred.z - bodyOdmDataDelayed.vel.z;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t77*(t29+P[0][4]*t4+P[0][6]*t9-P[0][5]*t11-P[0][1]*t17+P[0][2]*t20+P[0][3]*t24);
            Kfusion[1] = t77*(P[1][4]*t4+P[1][0]*t14+P[1][6]*t9-P[1][5]*t11-P[1][1]*t17+P[1][2]*t20+P[1][3]*t24);
            Kfusion[2] = t77*(t58+P[2][4]*t4+P[2][0]*t14+P[2][6]*t9-P[2][5]*t11-P[2][1]*t17+P[2][3]*t24);
            Kfusion[3] = t77*(t66+P[3][4]*t4+P[3][0]*t14+P[3][6]*t9-P[3][5]*t11-P[3][1]*t17+P[3][2]*t20);
            Kfusion[4] = t77*(t31+P[4][0]*t14+P[4][6]*t9-P[4][5]*t11-P[4][1]*t17+P[4][2]*t20+P[4][3]*t24);
            Kfusion[5] = t77*(-t80+P[5][4]*t4+P[5][0]*t14+P[5][6]*t9-P[5][1]*t17+P[5][2]*t20+P[5][3]*t24);
            Kfusion[6] = t77*(t69+P[6][4]*t4+P[6][0]*t14-P[6][5]*t11-P[6][1]*t17+P[6][2]*t20+P[6][3]*t24);
            Kfusion[7] = t77*(P[7][4]*t4+P[7][0]*t14+P[7][6]*t9-P[7][5]*t11-P[7][1]*t17+P[7][2]*t20+P[7][3]*t24);
            Kfusion[8] = t77*(P[8][4]*t4+P[8][0]*t14+P[8][6]*t9-P[8][5]*t11-P[8][1]*t17+P[8][2]*t20+P[8][3]*t24);
            Kfusion[9] = t77*(P[9][4]*t4+P[9][0]*t14+P[9][6]*t9-P[9][5]*t11-P[9][1]*t17+P[9][2]*t20+P[9][3]*t24);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = t77*(P[10][4]*t4+P[10][0]*t14+P[10][6]*t9-P[10][5]*t11-P[10][1]*t17+P[10][2]*t20+P[10][3]*t24);
                Kfusion[11] = t77*(P[11][4]*t4+P[11][0]*t14+P[11][6]*t9-P[11][5]*t11-P[11][1]*t17+P[11][2]*t20+P[11][3]*t24);
                Kfusion[12] = t77*(P[12][4]*t4+P[12][0]*t14+P[12][6]*t9-P[12][5]*t11-P[12][1]*t17+P[12][2]*t20+P[12][3]*t24);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);

            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = t77*(P[13][4]*t4+P[13][0]*t14+P[13][6]*t9-P[13][5]*t11-P[13][1]*t17+P[13][2]*t20+P[13][3]*t24);
                Kfusion[14] = t77*(P[14][4]*t4+P[14][0]*t14+P[14][6]*t9-P[14][5]*t11-P[14][1]*t17+P[14][2]*t20+P[14][3]*t24);
                Kfusion[15] = t77*(P[15][4]*t4+P[15][0]*t14+P[15][6]*t9-P[15][5]*t11-P[15][1]*t17+P[15][2]*t20+P[15][3]*t24);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(P[16][4]*t4+P[16][0]*t14+P[16][6]*t9-P[16][5]*t11-P[16][1]*t17+P[16][2]*t20+P[16][3]*t24);
                Kfusion[17] = t77*(P[17][4]*t4+P[17][0]*t14+P[17][6]*t9-P[17][5]*t11-P[17][1]*t17+P[17][2]*t20+P[17][3]*t24);
                Kfusion[18] = t77*(P[18][4]*t4+P[18][0]*t14+P[18][6]*t9-P[18][5]*t11-P[18][1]*t17+P[18][2]*t20+P[18][3]*t24);
                Kfusion[19] = t77*(P[19][4]*t4+P[19][0]*t14+P[19][6]*t9-P[19][5]*t11-P[19][1]*t17+P[19][2]*t20+P[19][3]*t24);
                Kfusion[20] = t77*(P[20][4]*t4+P[20][0]*t14+P[20][6]*t9-P[20][5]*t11-P[20][1]*t17+P[20][2]*t20+P[20][3]*t24);
                Kfusion[21] = t77*(P[21][4]*t4+P[21][0]*t14+P[21][6]*t9-P[21][5]*t11-P[21][1]*t17+P[21][2]*t20+P[21][3]*t24);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(P[22][4]*t4+P[22][0]*t14+P[22][6]*t9-P[22][5]*t11-P[22][1]*t17+P[22][2]*t20+P[22][3]*t24);
                Kfusion[23] = t77*(P[23][4]*t4+P[23][0]*t14+P[23][6]*t9-P[23][5]*t11-P[23][1]*t17+P[23][2]*t20+P[23][3]*t24);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }
        } else {
            return;
        }

        // calculate the innovation consistency test ratio
        // TODO add tuning parameter for gate
        bodyVelTestRatio[obsIndex] = sq(innovBodyVel[obsIndex]) / (sq(5.0f) * varInnovBodyVel[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds
        // TODO also apply angular velocity magnitude check
        if ((bodyVelTestRatio[obsIndex]) < 1.0f) {
            // record the last time observations were accepted for fusion
            prevBodyVelFuseTime_ms = imuSampleTime_ms;
            // notify first time only
            if (!bodyVelFusionActive) {
                bodyVelFusionActive = true;
                gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u fusing odometry",(unsigned)imu_index);
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * H_VEL[j];
                }
                for (unsigned j = 7; j<=stateIndexLim; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][0] * P[0][j];
                    res += KH[i][1] * P[1][j];
                    res += KH[i][2] * P[2][j];
                    res += KH[i][3] * P[3][j];
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][6] * P[6][j];
                    KHP[i][j] = res;
                }
            }

            // Check that we are not going to drive any variances negative and skip the update if so
            bool healthyFusion = true;
            for (uint8_t i= 0; i<=stateIndexLim; i++) {
                if (KHP[i][i] > P[i][i]) {
                    healthyFusion = false;
                }
            }

            if (healthyFusion) {
                // update the covariance matrix
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }

                // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
                ForceSymmetry();
                ConstrainVariances();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovBodyVel[obsIndex];
                }
                stateStruct.quat.normalize();

            } else {
                // record bad axis
                if (obsIndex == 0) {
                    faultStatus.bad_xvel = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yvel = true;
                } else if (obsIndex == 2) {
                    faultStatus.bad_zvel = true;
                }

            }
        }
    }
}

// select fusion of body odometry measurements
void NavEKF3_core::SelectBodyOdomFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && (dtIMUavg < 0.005f) && !bodyVelFusionDelayed) {
        bodyVelFusionDelayed = true;
        return;
    } else {
        bodyVelFusionDelayed = false;
    }

    // Check for data at the fusion time horizon
    if (storedBodyOdm.recall(bodyOdmDataDelayed, imuDataDelayed.time_ms)) {

        // start performance timer
        hal.util->perf_begin(_perf_FuseBodyOdom);

        usingWheelSensors = false;

        // Fuse data into the main filter
        FuseBodyVel();

        // stop the performance timer
        hal.util->perf_end(_perf_FuseBodyOdom);

    } else if (storedWheelOdm.recall(wheelOdmDataDelayed, imuDataDelayed.time_ms)) {

        // check if the delta time is too small to calculate a velocity
        if (wheelOdmDataDelayed.delTime > EKF_TARGET_DT) {

            // get the forward velocity
            float fwdSpd = wheelOdmDataDelayed.delAng * wheelOdmDataDelayed.radius * (1.0f / wheelOdmDataDelayed.delTime);

            // get the unit vector from the projection of the X axis onto the horizontal
            Vector3f unitVec;
            unitVec.x = prevTnb.a.x;
            unitVec.y = prevTnb.a.y;
            unitVec.z = 0.0f;
            unitVec.normalize();

            // multiply by forward speed to get velocity vector measured by wheel encoders
            Vector3f velNED = unitVec * fwdSpd;

            // This is a hack to enable use of the existing body frame velocity fusion method
            // TODO write a dedicated observation model for wheel encoders
            usingWheelSensors = true;
            bodyOdmDataDelayed.vel = prevTnb * velNED;
            bodyOdmDataDelayed.body_offset = wheelOdmDataDelayed.hub_offset;
            bodyOdmDataDelayed.velErr = frontend->_wencOdmVelErr;

            // Fuse data into the main filter
            FuseBodyVel();

        }

    }
}

