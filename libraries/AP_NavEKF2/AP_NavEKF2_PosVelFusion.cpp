#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF2_core::ResetVelocity(void)
{
    // Store the position before the reset so that we can record the reset delta
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    zeroRows(P,3,4);
    zeroCols(P,3,4);

    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        P[4][4] = P[3][3] = sq(frontend->_gpsHorizVelNoise);
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            stateStruct.velocity.x  = gpsDataNew.vel.x;
            stateStruct.velocity.y  = gpsDataNew.vel.y;
            // set the variances using the reported GPS speed accuracy
            P[4][4] = P[3][3] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[4][4] = P[3][3] = sq(25.0f);
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

    // Calculate the position jump due to the reset
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    lastVelReset_ms = imuSampleTime_ms;


}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF2_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    zeroRows(P,6,7);
    zeroCols(P,6,7);

    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        P[6][6] = P[7][7] = sq(frontend->_gpsHorizPosNoise);
    } else  {
        // Use GPS data as first preference if fresh data is available
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            // record the ID of the GPS for the data we are using for the reset
            last_gps_idx = gpsDataNew.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            stateStruct.position.x = gpsDataNew.pos.x  + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            stateStruct.position.y = gpsDataNew.pos.y  + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            // set the variances using the position measurement noise parameter
            P[6][6] = P[7][7] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) {
            // use the range beacon data as a second preference
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[6][6] = receiverPosCov[0][0];
            P[7][7] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - extNavDataDelayed.time_ms < 250) {
            // use the range beacon data as a second preference
            stateStruct.position.x = extNavDataDelayed.pos.x;
            stateStruct.position.y = extNavDataDelayed.pos.y;
            // set the variances from the beacon alignment filter
            P[7][7] = P[6][6] = sq(extNavDataDelayed.posErr);
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

}

// reset the vertical position state using the last height measurement
void NavEKF2_core::ResetHeight(void)
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
    zeroRows(P,8,8);
    zeroCols(P,8,8);

    // set the variances to the measurement variance
    P[8][8] = posDownObsNoise;

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
    zeroRows(P,5,5);
    zeroCols(P,5,5);

    // set the variances to the measurement variance
    P[5][5] = sq(frontend->_gpsVertVelNoise);

}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF2_core::resetHeightDatum(void)
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

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
void NavEKF2_core::SelectVelPosFusion()
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

    // read GPS data from the sensor and check for new data in the buffer
    readGpsData();
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    // Determine if we need to fuse position and velocity data on this time step
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // set fusion request flags
        if (frontend->_fusionModeGPS <= 1) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;
        extNavUsedForPos = false;

        // correct GPS data for position offset of antenna phase centre relative to the IMU
        Vector3f posOffsetBody = AP::gps().get_antenna_offset(gpsDataDelayed.sensor_idx) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {
            // Don't fuse velocity data if GPS doesn't support it
            if (fuseVelData) {
                // TODO use a filtered angular rate with a group delay that matches the GPS delay
                Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
                Vector3f velOffsetBody = angRate % posOffsetBody;
                Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
                gpsDataDelayed.vel.x -= velOffsetEarth.x;
                gpsDataDelayed.vel.y -= velOffsetEarth.y;
                gpsDataDelayed.vel.z -= velOffsetEarth.z;
            }

            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            gpsDataDelayed.pos.x -= posOffsetEarth.x;
            gpsDataDelayed.pos.y -= posOffsetEarth.y;
            gpsDataDelayed.hgt += posOffsetEarth.z;
        }

        // copy corrected GPS data to observation vector
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        velPosObs[3] = gpsDataDelayed.pos.x;
        velPosObs[4] = gpsDataDelayed.pos.y;

    } else if (extNavDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // This is a special case that uses and external nav system for position
        extNavUsedForPos = true;
        activeHgtSource = HGT_SOURCE_EV;
        fuseVelData = false;
        fuseHgtData = true;
        fusePosData = true;
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
        velPosObs[5] = extNavDataDelayed.pos.z;

        // if compass is disabled, also use it for yaw
        if (!use_compass()) {
            extNavUsedForYaw = true;
            if (!yawAlignComplete) {
                extNavYawResetRequest = true;
                magYawResetRequest = false;
                gpsYawResetRequest = false;
                controlMagYawReset();
                finalInflightYawInit = true;
            } else {
                fuseEulerYaw();
            }
        } else {
            extNavUsedForYaw = false;
        }

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
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE && gpsDataDelayed.sensor_idx != last_gps_idx) {
        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // Store the position before the reset so that we can record the reset delta
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;

        // Set the position states to the position from the new GPS
        stateStruct.position.x = gpsDataNew.pos.x;
        stateStruct.position.y = gpsDataNew.pos.y;

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
void NavEKF2_core::FuseVelPosNED()
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
            // For horizontal GPS velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPS perfomrance
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
            float hgtErr  = stateStruct.position.z - velPosObs[5];
            float velDErr = stateStruct.velocity.z - velPosObs[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
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
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
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
                if (posTimeout || ((P[6][6] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current GPS position
                    ResetPosition();
                    // reset the velocity to the GPS velocity
                    ResetVelocity();
                    // don't fuse GPS data on this time step
                    fusePosData = false;
                    fuseVelData = false;
                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    zeroRows(P,6,7);
                    zeroCols(P,6,7);
                    P[6][6] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[7][7] = P[6][6];
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
                // velocity states start at index 3
                stateIndex   = i + 3;
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
            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01f * (float)frontend->_hgtInnovGate, 1.0f)) * varInnovVelPos[5]);

            // when on ground we accept a larger test ratio to allow
            // the filter to handle large switch on IMU bias errors
            // without rejecting the height sensor
            const float maxTestRatio = (PV_AidingMode == AID_NONE && onGround)? 3.0 : 1.0;

            // fail if the ratio is > maxTestRatio, but don't fail if bad IMU data
            hgtHealth = (hgtTestRatio < maxTestRatio) || badIMUdata;

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
            tiltErrVec.zero();
        }
        if (fusePosData && posHealth) {
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
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
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
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

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
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

                    // update the states
                    // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
                    stateStruct.angErr.zero();

                    // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }

                    // the first 3 states represent the angular misalignment vector. This is
                    // is used to correct the estimated quaternion
                    stateStruct.quat.rotate(stateStruct.angErr);

                    // sum the attitude error from velocity and position fusion only
                    // used as a metric for convergence monitoring
                    if (obsIndex != 5) {
                        tiltErrVec += stateStruct.angErr;
                    }
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
void NavEKF2_core::selectHeightForFusion()
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
        const AP_RangeFinder_Backend *sensor = _rng->get_backend(rangeDataDelayed.sensor_idx);
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
    if (extNavUsedForPos) {
        // always use external vision as the height source if using for position.
        activeHgtSource = HGT_SOURCE_EV;
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
            float horizSpeed = norm(stateStruct.velocity.x, stateStruct.velocity.y);
            bool dontTrustTerrain = ((horizSpeed > frontend->_useRngSwSpd) && filterStatus.flags.horiz_vel) || !terrainHgtStable;
            float trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
            bool trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;

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
    bool lostExtNavHgt = ((activeHgtSource == HGT_SOURCE_EV) && ((imuSampleTime_ms - extNavMeasTime_ms) > 2000));
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
    if (extNavDataToFuse && (activeHgtSource == HGT_SOURCE_EV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        posDownObsNoise = sq(constrain_float(extNavDataDelayed.posErr, 0.01f, 10.0f));
    } else if (rangeDataToFuse && (activeHgtSource == HGT_SOURCE_RNG)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            // enable fusion
            fuseHgtData = true;
            velPosObs[5] = -hgtMea;
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
        // enable fusion
        velPosObs[5] = -hgtMea;
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
        velPosObs[5] = -hgtMea;
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

