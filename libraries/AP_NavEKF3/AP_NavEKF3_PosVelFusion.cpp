#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF3_core::ResetVelocity(resetDataSource velResetSource)
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
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250 && velResetSource == resetDataSource::DEFAULT) || velResetSource == resetDataSource::GPS) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            stateStruct.velocity.x  = gps_corrected.vel.x;
            stateStruct.velocity.y  = gps_corrected.vel.y;
            // set the variances using the reported GPS speed accuracy
            P[5][5] = P[4][4] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
#if EK3_FEATURE_EXTERNAL_NAV
        } else if ((imuSampleTime_ms - extNavVelMeasTime_ms < 250 && velResetSource == resetDataSource::DEFAULT) || velResetSource == resetDataSource::EXTNAV) {
            // use external nav data as the 2nd preference
            // already corrected for sensor position
            stateStruct.velocity.x = extNavVelDelayed.vel.x;
            stateStruct.velocity.y = extNavVelDelayed.vel.y;
            P[5][5] = P[4][4] = sq(extNavVelDelayed.err);
#endif // EK3_FEATURE_EXTERNAL_NAV
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[5][5] = P[4][4] = sq(25.0f);
        }
        // clear the timeout flags and counters
        velTimeout = false;
        lastVelPassTime_ms = imuSampleTime_ms;
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
}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF3_core::ResetPosition(resetDataSource posResetSource)
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
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250 && posResetSource == resetDataSource::DEFAULT) || posResetSource == resetDataSource::GPS) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            // record the ID of the GPS for the data we are using for the reset
            last_gps_idx = gps_corrected.sensor_idx;
            // calculate position
            const Location gpsloc{gps_corrected.lat, gps_corrected.lng, 0, Location::AltFrame::ABSOLUTE};
            stateStruct.position.xy() = EKF_origin.get_distance_NE_ftype(gpsloc);
            // compensate for offset  between last GPS measurement and the EKF time horizon. Note that this is an unusual
            // time delta in that it can be both -ve and +ve
            const int32_t tdiff = imuDataDelayed.time_ms - gps_corrected.time_ms;
            stateStruct.position.xy() += gps_corrected.vel.xy()*0.001*tdiff;
            // set the variances using the position measurement noise parameter
            P[7][7] = P[8][8] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
        } else if ((imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250 && posResetSource == resetDataSource::DEFAULT) || posResetSource == resetDataSource::RNGBCN) {
            // use the range beacon data as a second preference
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[7][7] = receiverPosCov[0][0];
            P[8][8] = receiverPosCov[1][1];
#if EK3_FEATURE_EXTERNAL_NAV
        } else if ((imuSampleTime_ms - extNavDataDelayed.time_ms < 250 && posResetSource == resetDataSource::DEFAULT) || posResetSource == resetDataSource::EXTNAV) {
            // use external nav data as the third preference
            stateStruct.position.x = extNavDataDelayed.pos.x;
            stateStruct.position.y = extNavDataDelayed.pos.y;
            // set the variances as received from external nav system data
            P[7][7] = P[8][8] = sq(extNavDataDelayed.posErr);
#endif // EK3_FEATURE_EXTERNAL_NAV
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

    // clear the timeout flags and counters
    posTimeout = false;
    lastPosPassTime_ms = imuSampleTime_ms;
}

// reset the stateStruct's NE position to the specified position
//    posResetNE is updated to hold the change in position
//    storedOutput, outputDataNew and outputDataDelayed are updated with the change in position
//    lastPosReset_ms is updated with the time of the reset
void NavEKF3_core::ResetPositionNE(ftype posN, ftype posE)
{
    // Store the position before the reset so that we can record the reset delta
    const Vector3F posOrig = stateStruct.position;

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
void NavEKF3_core::ResetPositionD(ftype posD)
{
    // Store the position before the reset so that we can record the reset delta
    const ftype posDOrig = stateStruct.position.z;

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
    if (inFlight &&
        (gpsIsInUse || badIMUdata) &&
        frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::GPS) &&
        gpsDataNew.have_vz &&
        (imuSampleTime_ms - gpsDataDelayed.time_ms < 500)) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
#if EK3_FEATURE_EXTERNAL_NAV
    } else if (inFlight && useExtNavVel && (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV)) {
        stateStruct.velocity.z = extNavVelDelayed.vel.z;
#endif
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

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF3_core::resetHeightDatum(void)
{
    if (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER || !onGround) {
        // only allow resets when on the ground.
        // If using using rangefinder for height then never perform a
        // reset of the height datum
        return false;
    }
    // record the old height estimate
    ftype oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    dal.baro().update_calibration();
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
            EKF_origin.alt = dal.gps().location().alt;
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
    // return immediately if already corrected
    if (gps_data.corrected) {
        return;
    }
    gps_data.corrected = true;

    const Vector3F posOffsetBody = dal.gps().get_antenna_offset(gps_data.sensor_idx).toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }

    // TODO use a filtered angular rate with a group delay that matches the GPS delay
    Vector3F angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
    Vector3F velOffsetBody = angRate % posOffsetBody;
    Vector3F velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
    gps_data.vel -= velOffsetEarth;

    Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    Location::offset_latlng(gps_data.lat, gps_data.lng, -posOffsetEarth.x, -posOffsetEarth.y);
    gps_data.hgt += posOffsetEarth.z;
}

// correct external navigation earth-frame position using sensor body-frame offset
void NavEKF3_core::CorrectExtNavForSensorOffset(ext_nav_elements &ext_nav_data)
{
    // return immediately if already corrected
    if (ext_nav_data.corrected) {
        return;
    }
    ext_nav_data.corrected = true;

    // external nav data is against the public_origin, so convert to offset from EKF_origin
    ext_nav_data.pos.xy() += EKF_origin.get_distance_NE_ftype(public_origin);

#if HAL_VISUALODOM_ENABLED
    const auto *visual_odom = dal.visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    const Vector3F posOffsetBody = visual_odom->get_pos_offset().toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    ext_nav_data.pos.x -= posOffsetEarth.x;
    ext_nav_data.pos.y -= posOffsetEarth.y;
    ext_nav_data.pos.z -= posOffsetEarth.z;
#endif
}

// correct external navigation earth-frame velocity using sensor body-frame offset
void NavEKF3_core::CorrectExtNavVelForSensorOffset(ext_nav_vel_elements &ext_nav_vel_data) const
{
    // return immediately if already corrected
    if (ext_nav_vel_data.corrected) {
        return;
    }
    ext_nav_vel_data.corrected = true;

#if HAL_VISUALODOM_ENABLED
    const auto *visual_odom = dal.visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    const Vector3F posOffsetBody = visual_odom->get_pos_offset().toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    // TODO use a filtered angular rate with a group delay that matches the sensor delay
    const Vector3F angRate = imuDataDelayed.delAng * (1.0/imuDataDelayed.delAngDT);
    ext_nav_vel_data.vel += get_vel_correction_for_sensor_offset(posOffsetBody, prevTnb, angRate);
#endif
}

// calculate velocity variance helper function
void NavEKF3_core::CalculateVelInnovationsAndVariances(const Vector3F &velocity, ftype noise, ftype accel_scale, Vector3F &innovations, Vector3F &variances) const
{
    // innovations are latest estimate - latest observation
    innovations = stateStruct.velocity - velocity;

    const ftype obs_data_chk = sq(constrain_ftype(noise, 0.05, 5.0)) + sq(accel_scale * accNavMag);

    // calculate innovation variance.  velocity states start at index 4
    variances.x = P[4][4] + obs_data_chk;
    variances.y = P[5][5] + obs_data_chk;
    variances.z = P[6][6] + obs_data_chk;
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

#if EK3_FEATURE_EXTERNAL_NAV
    // Check for data at the fusion time horizon
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);
    if (extNavDataToFuse) {
        CorrectExtNavForSensorOffset(extNavDataDelayed);
    }
    extNavVelToFuse = storedExtNavVel.recall(extNavVelDelayed, imuDataDelayed.time_ms);
    if (extNavVelToFuse) {
        CorrectExtNavVelForSensorOffset(extNavVelDelayed);

        // calculate innovations and variances for reporting purposes only
        CalculateVelInnovationsAndVariances(extNavVelDelayed.vel, extNavVelDelayed.err, frontend->extNavVelVarAccScale, extNavVelInnov, extNavVelVarInnov);

        // record time innovations were calculated (for timeout checks)
        extNavVelInnovTime_ms = dal.millis();
    }
#endif // EK3_FEATURE_EXTERNAL_NAV

    // Read GPS data from the sensor
    readGpsData();
    readGpsYawData();

    // get data that has now fallen behind the fusion time horizon
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    if (gpsDataToFuse) {
        CorrectGPSForAntennaOffset(gpsDataDelayed);
        // calculate innovations and variances for reporting purposes only
        CalculateVelInnovationsAndVariances(gpsDataDelayed.vel, frontend->_gpsHorizVelNoise, frontend->gpsNEVelVarAccScale, gpsVelInnov, gpsVelVarInnov);
        // record time innovations were calculated (for timeout checks)
        gpsVelInnovTime_ms = dal.millis();
    }

    // detect position source changes.  Trigger position reset if position source is valid
    const AP_NavEKF_Source::SourceXY posxy_source = frontend->sources.getPosXYSource();
    if (posxy_source != posxy_source_last) {
        posxy_source_reset = (posxy_source != AP_NavEKF_Source::SourceXY::NONE);
        posxy_source_last = posxy_source;
    }

    // initialise all possible data we may fuse
    fusePosData = false;
    fuseVelData = false;

    // Determine if we need to fuse position and velocity data on this time step
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::GPS)) {

        // Don't fuse velocity data if GPS doesn't support it
        fuseVelData = frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS);
        fusePosData = true;
#if EK3_FEATURE_EXTERNAL_NAV
        extNavUsedForPos = false;
#endif

        // copy corrected GPS data to observation vector
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        const Location gpsloc{gpsDataDelayed.lat, gpsDataDelayed.lng, 0, Location::AltFrame::ABSOLUTE};
        const Vector2F posxy = EKF_origin.get_distance_NE_ftype(gpsloc);
        velPosObs[3] = posxy.x;
        velPosObs[4] = posxy.y;
#if EK3_FEATURE_EXTERNAL_NAV
    } else if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::EXTNAV)) {
        // use external nav system for horizontal position
        extNavUsedForPos = true;
        fusePosData = true;
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
#endif // EK3_FEATURE_EXTERNAL_NAV
    }

#if EK3_FEATURE_EXTERNAL_NAV
    // fuse external navigation velocity data if available
    // extNavVelDelayed is already corrected for sensor position
    if (extNavVelToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV)) {
        fuseVelData = true;
        velPosObs[0] = extNavVelDelayed.vel.x;
        velPosObs[1] = extNavVelDelayed.vel.y;
        velPosObs[2] = extNavVelDelayed.vel.z;
    }
#endif

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    if (gpsYawResetRequest) {
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources
    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::GPS) && (gpsDataDelayed.sensor_idx != last_gps_idx || posxy_source_reset)) {
        // mark a source reset as consumed
        posxy_source_reset = false;

        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // reset the position to the GPS position
        const Location gpsloc{gpsDataDelayed.lat, gpsDataDelayed.lng, 0, Location::AltFrame::ABSOLUTE};
        const Vector2F posxy = EKF_origin.get_distance_NE_ftype(gpsloc);
        ResetPositionNE(posxy.x, posxy.y);

        // If we are also using GPS as the height reference, reset the height
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::GPS) {
            ResetPositionD(-hgtMea);
        }
    }

#if EK3_FEATURE_EXTERNAL_NAV
    // check for external nav position reset
    if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::EXTNAV) && (extNavDataDelayed.posReset || posxy_source_reset)) {
        // mark a source reset as consumed
        posxy_source_reset = false;
        ResetPositionNE(extNavDataDelayed.pos.x, extNavDataDelayed.pos.y);
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV) {
            ResetPositionD(-hgtMea);
        }
    }
#endif // EK3_FEATURE_EXTERNAL_NAV

    // If we are operating without any aiding, fuse in constant position of constant
    // velocity measurements to constrain tilt drift. This assumes a non-manoeuvring
    // vehicle. Do this to coincide with the height fusion.
    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        if (assume_zero_sideslip() && tiltAlignComplete && motorsArmed) {
            // handle special case where we are launching a FW aircraft without magnetometer
            fusePosData = false;
            velPosObs[0] = 0.0f;
            velPosObs[1] = 0.0f;
            velPosObs[2] = stateStruct.velocity.z;
            bool resetVelNE = !prevMotorsArmed;
            // reset states to stop launch accel causing tilt error
            if  (imuDataDelayed.delVel.x > 1.1f * GRAVITY_MSS * imuDataDelayed.delVelDT) {
                lastLaunchAccelTime_ms = imuSampleTime_ms;
                fuseVelData = false;
                resetVelNE = true;
            } else if (lastLaunchAccelTime_ms != 0 && (imuSampleTime_ms - lastLaunchAccelTime_ms) < 10000) {
                fuseVelData = false;
                resetVelNE = true;
            } else {
                fuseVelData = true;
            }
            if (resetVelNE) {
                stateStruct.velocity.x = 0.0f;
                stateStruct.velocity.y = 0.0f;
            }
        } else {
            fusePosData = true;
            fuseVelData = false;
            velPosObs[3] = lastKnownPositionNE.x;
            velPosObs[4] = lastKnownPositionNE.y;
        }
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
    // health is set bad until test passed
    bool velCheckPassed = false; // boolean true if velocity measurements have passed innovation consistency checks
    bool posCheckPassed = false; // boolean true if position measurements have passed innovation consistency check
    bool hgtCheckPassed = false; // boolean true if height measurements have passed innovation consistency check

    // declare variables used to control access to arrays
    bool fuseData[6] {};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    ftype SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // calculate additional error in GPS position caused by manoeuvring
        ftype posErr = frontend->gpsPosVarAccScale * accNavMag;

        // To-Do: this posErr should come from external nav when fusing external nav position

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {
                // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
                R_OBS[0] = sq(constrain_ftype(frontend->_noaidHorizNoise, 0.5f, 50.0f));
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
                R_OBS[0] = sq(constrain_ftype(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_ftype(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
#if EK3_FEATURE_EXTERNAL_NAV
            } else if (extNavVelToFuse) {
                R_OBS[2] = R_OBS[0] = sq(constrain_ftype(extNavVelDelayed.err, 0.05f, 5.0f));
#endif
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                R_OBS[0] = sq(constrain_ftype(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_ftype(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_ftype(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
#if EK3_FEATURE_EXTERNAL_NAV
            } else if (extNavUsedForPos) {
                R_OBS[3] = sq(constrain_ftype(extNavDataDelayed.posErr, 0.01f, 10.0f));
#endif
            } else {
                R_OBS[3] = sq(constrain_ftype(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPS velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPS performance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            ftype obs_data_chk;
#if EK3_FEATURE_EXTERNAL_NAV
            if (extNavVelToFuse) {
                obs_data_chk = sq(constrain_ftype(extNavVelDelayed.err, 0.05f, 5.0f)) + sq(frontend->extNavVelVarAccScale * accNavMag);
            } else
#endif
            {
                obs_data_chk = sq(constrain_ftype(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
            }
            R_OBS_DATA_CHECKS[0] = R_OBS_DATA_CHECKS[1] = R_OBS_DATA_CHECKS[2] = obs_data_chk;
        }
        R_OBS[5] = posDownObsNoise;
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (gpsDataDelayed.have_vz && fuseVelData && (frontend->sources.getPosZSource() != AP_NavEKF_Source::SourceZ::GPS)) {
            // calculate innovations for height and vertical GPS vel measurements
            const ftype hgtErr  = stateStruct.position.z - velPosObs[5];
            const ftype velDErr = stateStruct.velocity.z - velPosObs[2];
            // Check if they are the same sign and both more than 3-sigma out of bounds
            // Step the test threshold up in stages from 1 to 2 to 3 sigma after exiting
            // from a previous bad IMU event so that a subsequent error is caught more quickly.
            const uint32_t timeSinceLastBadIMU_ms = imuSampleTime_ms - badIMUdata_ms;
            float R_gain;
            if (timeSinceLastBadIMU_ms > (BAD_IMU_DATA_HOLD_MS * 2)) {
                R_gain = 9.0F;
            } else if  (timeSinceLastBadIMU_ms > ((BAD_IMU_DATA_HOLD_MS * 3) / 2)) {
                R_gain = 4.0F;
            } else {
                R_gain = 1.0F;
            }
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > R_gain * R_OBS[5]) && (sq(velDErr) >R_gain * R_OBS[2])) {
                badIMUdata_ms = imuSampleTime_ms;
            } else {
                goodIMUdata_ms = imuSampleTime_ms;
            }
            if (timeSinceLastBadIMU_ms < BAD_IMU_DATA_HOLD_MS) {
                badIMUdata = true;
                stateStruct.velocity.z = gpsDataDelayed.vel.z;
            } else {
                badIMUdata = false;
            }
        }

        // Test horizontal position measurements
        if (fusePosData) {
            innovVelPos[3] = stateStruct.position.x - velPosObs[3];
            innovVelPos[4] = stateStruct.position.y - velPosObs[4];
            varInnovVelPos[3] = P[7][7] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS_DATA_CHECKS[4];

            // Apply an innovation consistency threshold test
            // Don't allow test to fail if not navigating and using a constant position
            // assumption to constrain tilt errors because innovations can become large
            // due to vehicle motion.
            ftype maxPosInnov2 = sq(MAX(0.01 * (ftype)frontend->_gpsPosInnovGate, 1.0))*(varInnovVelPos[3] + varInnovVelPos[4]);

            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            if (posTestRatio < 1.0f || (PV_AidingMode == AID_NONE)) {
                posCheckPassed = true;
                lastPosPassTime_ms = imuSampleTime_ms;
            }

            // Use position data if healthy or timed out or bad IMU data
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (posCheckPassed || posTimeout || badIMUdata) {
                // if timed out or outside the specified uncertainty radius, reset to the external sensor
                if (posTimeout || ((P[8][8] + P[7][7]) > sq(ftype(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current external sensor position
                    ResetPosition(resetDataSource::DEFAULT);

                    // Don't fuse the same data we have used to reset states.
                    fusePosData = false;

                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    zeroRows(P,7,8);
                    zeroCols(P,7,8);
                    P[7][7] = sq(ftype(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[8][8] = P[7][7];

                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    posTestRatio = 0.0f;

                    // Reset velocity if it has timed out
                    if (velTimeout) {
                        ResetVelocity(resetDataSource::DEFAULT);

                        // Don't fuse the same data we have used to reset states.
                        fuseVelData = false;

                        // Reset the normalised innovation to avoid failing the bad fusion tests
                        velTestRatio = 0.0f;
                    }
                }
            } else {
                fusePosData = false;
            }
        }

        // Test velocity measurements
        if (fuseVelData) {
            uint8_t imax = 2;
            // Don't fuse vertical velocity observations if disabled in sources or not available
            if ((!frontend->sources.haveVelZSource() || PV_AidingMode != AID_ABSOLUTE ||
                 !gpsDataDelayed.have_vz) && !useExtNavVel) {
                imax = 1;
            }

            // Apply an innovation consistency threshold test
            ftype innovVelSumSq = 0; // sum of squares of velocity innovations
            ftype varVelSum = 0; // sum of velocity innovation variances

            for (uint8_t i = 0; i<=imax; i++) {
                stateIndex   = i + 4;
                const float innovation = stateStruct.velocity[i] - velPosObs[i];
                innovVelSumSq += sq(innovation);
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                varVelSum += varInnovVelPos[i];
            }
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01 * (ftype)frontend->_gpsVelInnovGate, 1.0)));
            if (velTestRatio < 1.0) {
                velCheckPassed = true;
                lastVelPassTime_ms = imuSampleTime_ms;
            }

            // Use velocity data if healthy, timed out or when IMU fault has been detected
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (velCheckPassed || velTimeout || badIMUdata) {
                // If we are doing full aiding and velocity fusion times out, reset to the external sensor velocity
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    ResetVelocity(resetDataSource::DEFAULT);

                    // Don't fuse the same data we have used to reset states.
                    fuseVelData = false;

                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    velTestRatio = 0.0f;
                }
            } else {
                fuseVelData = false;
            }
        }

        // Test height measurements
        if (fuseHgtData) {
            // Calculate height innovations
            innovVelPos[5] = stateStruct.position.z - velPosObs[5];
            varInnovVelPos[5] = P[9][9] + R_OBS_DATA_CHECKS[5];

            // Calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01 * (ftype)frontend->_hgtInnovGate, 1.0)) * varInnovVelPos[5]);

            // When on ground we accept a larger test ratio to allow the filter to handle large switch on IMU
            // bias errors without rejecting the height sensor.
            const float maxTestRatio = (PV_AidingMode == AID_NONE && onGround)? 3.0f : 1.0f;
            if (hgtTestRatio < maxTestRatio) {
                hgtCheckPassed = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }

            // Use height data if innovation check passed or timed out or if bad IMU data
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (hgtCheckPassed || hgtTimeout || badIMUdata) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (onGround) {
                    ftype dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms) * 1.0e-3;
                    const ftype hgtInnovFiltTC = 2.0;
                    ftype alpha = constrain_ftype(dtBaro/(dtBaro+hgtInnovFiltTC), 0.0, 1.0);
                    hgtInnovFiltState += (innovVelPos[5] - hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                if (hgtTimeout) {
                    ResetHeight();

                    // Don't fuse the same data we have used to reset states.
                    fuseHgtData = false;
                }

            } else {
                fuseHgtData = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel || useExtNavVel) {
                fuseData[2] = true;
            }
        }
        if (fusePosData) {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if (fuseHgtData) {
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
                    const ftype gndMaxBaroErr = MAX(frontend->_baroGndEffectDeadZone, 0.0);
                    const ftype gndBaroInnovFloor = -0.5;

                    if ((dal.get_touchdown_expected() || dal.get_takeoff_expected()) && activeHgtSource == AP_NavEKF_Source::SourceZ::BARO) {
                        // when baro positive pressure error due to ground effect is expected,
                        // floor the barometer innovation at gndBaroInnovFloor
                        // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                        // this function looks like this:
                        //         |/
                        //---------|---------
                        //    ____/|
                        //   /     |
                        //  /      |
                        innovVelPos[5] += constrain_ftype(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=9; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit delta angle bias state estimation by setting Kalman gains to zero
                if (!inhibitDelAngBiasStates) {
                    for (uint8_t i = 10; i<=12; i++) {
                        // Don't try to learn gyro bias if not aiding and the axis is
                        // less than 45 degrees from vertical because the bias is poorly observable
                        bool poorObservability = false;
                        if (PV_AidingMode == AID_NONE) {
                            const uint8_t axisIndex = i - 10;
                            if (axisIndex == 0) {
                                poorObservability = fabsF(prevTnb.a.z) > M_SQRT1_2;
                            } else if (axisIndex == 1) {
                                poorObservability = fabsF(prevTnb.b.z) > M_SQRT1_2;
                            } else {
                                poorObservability = fabsF(prevTnb.c.z) > M_SQRT1_2;
                            }
                        }
                        if (poorObservability) {
                            Kfusion[i] = 0.0;
                        } else {
                            Kfusion[i] = P[i][stateIndex]*SK;
                        }
                    }
                } else {
                    // zero indexes 10 to 12
                    zero_range(&Kfusion[0], 10, 12);
                }

                // Inhibit delta velocity bias state estimation by setting Kalman gains to zero
                // Don't use 'fake' horizontal measurements used to constrain attitude drift during
                // periods of non-aiding to learn bias as these can give incorrect esitmates.
                const bool horizInhibit = PV_AidingMode == AID_NONE && obsIndex != 2 && obsIndex != 5;
                if (!horizInhibit && !inhibitDelVelBiasStates && !badIMUdata) {
                    for (uint8_t i = 13; i<=15; i++) {
                        if (!dvelBiasAxisInhibit[i-13]) {
                            Kfusion[i] = P[i][stateIndex]*SK;
                        } else {
                            Kfusion[i] = 0.0f;
                        }
                    }
                } else {
                    // zero indexes 13 to 15
                    zero_range(&Kfusion[0], 13, 15);
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    // zero indexes 16 to 21
                    zero_range(&Kfusion[0], 16, 21);
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    // zero indexes 22 to 23
                    zero_range(&Kfusion[0], 22, 23);
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
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
    const auto *_rng = dal.rangefinder();
    if (_rng && rangeDataToFuse) {
        auto *sensor = _rng->get_backend(rangeDataDelayed.sensor_idx);
        if (sensor != nullptr) {
            Vector3F posOffsetBody = sensor->get_pos_offset().toftype() - accelPosOffset;
            if (!posOffsetBody.is_zero()) {
                Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
                rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
            }
        }
    }

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    bool rangeFinderDataIsFresh = (imuSampleTime_ms - rngValidMeaTime_ms < 500);
#if EK3_FEATURE_EXTERNAL_NAV
    const bool extNavDataIsFresh = (imuSampleTime_ms - extNavMeasTime_ms < 500);
#endif
    // select height source
    if ((frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::RANGEFINDER) && _rng && rangeFinderDataIsFresh) {
        // user has specified the range finder as a primary height source
        activeHgtSource = AP_NavEKF_Source::SourceZ::RANGEFINDER;
    } else if ((frontend->_useRngSwHgt > 0) && ((frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::BARO) || (frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::GPS)) && _rng && rangeFinderDataIsFresh) {
        // determine if we are above or below the height switch region
        ftype rangeMaxUse = 1e-4 * (ftype)_rng->max_distance_cm_orient(ROTATION_PITCH_270) * (ftype)frontend->_useRngSwHgt;
        bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
        bool belowLowerSwHgt = (terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse;

        // If the terrain height is consistent and we are moving slowly, then it can be
        // used as a height reference in combination with a range finder
        // apply a hysteresis to the speed check to prevent rapid switching
        bool dontTrustTerrain, trustTerrain;
        if (filterStatus.flags.horiz_vel) {
            // We can use the velocity estimate
            ftype horizSpeed = stateStruct.velocity.xy().length();
            dontTrustTerrain = (horizSpeed > frontend->_useRngSwSpd) || !terrainHgtStable;
            ftype trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
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
        if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
            // cannot trust terrain or range finder so stop using range finder height
            if (frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::BARO) {
                activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
            } else if (frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::GPS) {
                activeHgtSource = AP_NavEKF_Source::SourceZ::GPS;
            }
        } else if (belowLowerSwHgt && trustTerrain && (prevTnb.c.z >= 0.7f)) {
            // reliable terrain and range finder so start using range finder height
            activeHgtSource = AP_NavEKF_Source::SourceZ::RANGEFINDER;
        }
    } else if (frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::BARO) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    } else if ((frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::GPS;
    } else if ((frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::BEACON) && validOrigin && rngBcnGoodToAlign) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BEACON;
#if EK3_FEATURE_EXTERNAL_NAV
    } else if ((frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::EXTNAV) && extNavDataIsFresh) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::EXTNAV;
#endif
    }

    // Use Baro alt as a fallback if we lose range finder, GPS, external nav or Beacon
    bool lostRngHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER) && !rangeFinderDataIsFresh);
    bool lostGpsHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000 || !gpsAccuracyGoodForAltitude));
    bool lostRngBcnHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::BEACON) && ((imuSampleTime_ms - rngBcnDataDelayed.time_ms) > 2000));
    bool fallback_to_baro = lostRngHgt || lostGpsHgt || lostRngBcnHgt;
#if EK3_FEATURE_EXTERNAL_NAV
    bool lostExtNavHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV) && !extNavDataIsFresh);
    fallback_to_baro |= lostExtNavHgt;
#endif
    if (fallback_to_baro) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        if (activeHgtSource != AP_NavEKF_Source::SourceZ::BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!dal.get_takeoff_expected()) {
            const ftype gndHgtFiltTC = 0.5;
            const ftype dtBaro = frontend->hgtAvg_ms*1.0e-3;
            ftype alpha = constrain_ftype(dtBaro / (dtBaro+gndHgtFiltTC),0.0,1.0);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // If we are not using GPS as the primary height sensor, correct EKF origin height so that
    // combined local NED position height and origin height remains consistent with the GPS altitude
    // This also enables the GPS height to be used as a backup height source
    if (gpsDataToFuse &&
            (((frontend->_originHgtMode & (1 << 0)) && (activeHgtSource == AP_NavEKF_Source::SourceZ::BARO)) ||
            ((frontend->_originHgtMode & (1 << 1)) && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)))
            ) {
            correctEkfOriginHeight();
    }

    // Select the height measurement source
#if EK3_FEATURE_EXTERNAL_NAV
    if (extNavDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        velPosObs[5] = -hgtMea;
        posDownObsNoise = sq(constrain_ftype(extNavDataDelayed.posErr, 0.1f, 10.0f));
        fuseHgtData = true;
    } else
#endif // EK3_FEATURE_EXTERNAL_NAV
        if (rangeDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
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
            posDownObsNoise = sq(constrain_ftype(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if (gpsDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        velPosObs[5] = -hgtMea;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_ftype(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_ftype(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise
        posDownObsNoise = sq(constrain_ftype(frontend->_baroAltNoise, 0.1f, 10.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be experiencing rotor wash ground interaction
        if (dal.get_takeoff_expected() || dal.get_touchdown_expected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        velPosObs[5] = -hgtMea;
    } else {
        fuseHgtData = false;
    }

    // detect changes in source and reset height
    if ((activeHgtSource != prevHgtSource) && fuseHgtData) {
        prevHgtSource = activeHgtSource;
        ResetPositionD(-hgtMea);
    }

    // If we haven't fused height data for a while or have bad IMU data, then declare the height data as being timed out
    // set height timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = ((useGpsVertVel || useExtNavVel) && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms ||
        (badIMUdata &&
        (imuSampleTime_ms - goodIMUdata_ms > BAD_IMU_DATA_TIMEOUT_MS))) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

#if EK3_FEATURE_BODY_ODOM
/*
 * Fuse body frame velocity measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseBodyVel()
{
    Vector24 H_VEL;
    Vector3F bodyVelPred;

    // Copy required states to local variable names
    ftype q0  = stateStruct.quat[0];
    ftype q1 = stateStruct.quat[1];
    ftype q2 = stateStruct.quat[2];
    ftype q3 = stateStruct.quat[3];
    ftype vn = stateStruct.velocity.x;
    ftype ve = stateStruct.velocity.y;
    ftype vd = stateStruct.velocity.z;

    // Fuse X, Y and Z axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=2; obsIndex++) {

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        bodyVelPred = (prevTnb * stateStruct.velocity);

        // correct sensor offset body frame position offset relative to IMU
        Vector3F posOffsetBody = bodyOdmDataDelayed.body_offset - accelPosOffset;

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
            ftype R_VEL = sq(bodyOdmDataDelayed.velErr);
            ftype t2 = q0*q3*2.0f;
            ftype t3 = q1*q2*2.0f;
            ftype t4 = t2+t3;
            ftype t5 = q0*q0;
            ftype t6 = q1*q1;
            ftype t7 = q2*q2;
            ftype t8 = q3*q3;
            ftype t9 = t5+t6-t7-t8;
            ftype t10 = q0*q2*2.0f;
            ftype t25 = q1*q3*2.0f;
            ftype t11 = t10-t25;
            ftype t12 = q3*ve*2.0f;
            ftype t13 = q0*vn*2.0f;
            ftype t26 = q2*vd*2.0f;
            ftype t14 = t12+t13-t26;
            ftype t15 = q3*vd*2.0f;
            ftype t16 = q2*ve*2.0f;
            ftype t17 = q1*vn*2.0f;
            ftype t18 = t15+t16+t17;
            ftype t19 = q0*vd*2.0f;
            ftype t20 = q2*vn*2.0f;
            ftype t27 = q1*ve*2.0f;
            ftype t21 = t19+t20-t27;
            ftype t22 = q1*vd*2.0f;
            ftype t23 = q0*ve*2.0f;
            ftype t28 = q3*vn*2.0f;
            ftype t24 = t22+t23-t28;
            ftype t29 = P[0][0]*t14;
            ftype t30 = P[1][1]*t18;
            ftype t31 = P[4][5]*t9;
            ftype t32 = P[5][5]*t4;
            ftype t33 = P[0][5]*t14;
            ftype t34 = P[1][5]*t18;
            ftype t35 = P[3][5]*t24;
            ftype t79 = P[6][5]*t11;
            ftype t80 = P[2][5]*t21;
            ftype t36 = t31+t32+t33+t34+t35-t79-t80;
            ftype t37 = t4*t36;
            ftype t38 = P[4][6]*t9;
            ftype t39 = P[5][6]*t4;
            ftype t40 = P[0][6]*t14;
            ftype t41 = P[1][6]*t18;
            ftype t42 = P[3][6]*t24;
            ftype t81 = P[6][6]*t11;
            ftype t82 = P[2][6]*t21;
            ftype t43 = t38+t39+t40+t41+t42-t81-t82;
            ftype t44 = P[4][0]*t9;
            ftype t45 = P[5][0]*t4;
            ftype t46 = P[1][0]*t18;
            ftype t47 = P[3][0]*t24;
            ftype t84 = P[6][0]*t11;
            ftype t85 = P[2][0]*t21;
            ftype t48 = t29+t44+t45+t46+t47-t84-t85;
            ftype t49 = t14*t48;
            ftype t50 = P[4][1]*t9;
            ftype t51 = P[5][1]*t4;
            ftype t52 = P[0][1]*t14;
            ftype t53 = P[3][1]*t24;
            ftype t86 = P[6][1]*t11;
            ftype t87 = P[2][1]*t21;
            ftype t54 = t30+t50+t51+t52+t53-t86-t87;
            ftype t55 = t18*t54;
            ftype t56 = P[4][2]*t9;
            ftype t57 = P[5][2]*t4;
            ftype t58 = P[0][2]*t14;
            ftype t59 = P[1][2]*t18;
            ftype t60 = P[3][2]*t24;
            ftype t78 = P[2][2]*t21;
            ftype t88 = P[6][2]*t11;
            ftype t61 = t56+t57+t58+t59+t60-t78-t88;
            ftype t62 = P[4][3]*t9;
            ftype t63 = P[5][3]*t4;
            ftype t64 = P[0][3]*t14;
            ftype t65 = P[1][3]*t18;
            ftype t66 = P[3][3]*t24;
            ftype t90 = P[6][3]*t11;
            ftype t91 = P[2][3]*t21;
            ftype t67 = t62+t63+t64+t65+t66-t90-t91;
            ftype t68 = t24*t67;
            ftype t69 = P[4][4]*t9;
            ftype t70 = P[5][4]*t4;
            ftype t71 = P[0][4]*t14;
            ftype t72 = P[1][4]*t18;
            ftype t73 = P[3][4]*t24;
            ftype t92 = P[6][4]*t11;
            ftype t93 = P[2][4]*t21;
            ftype t74 = t69+t70+t71+t72+t73-t92-t93;
            ftype t75 = t9*t74;
            ftype t83 = t11*t43;
            ftype t89 = t21*t61;
            ftype t76 = R_VEL+t37+t49+t55+t68+t75-t83-t89;
            ftype t77;

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
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);
            }

            if (!inhibitDelVelBiasStates && !badIMUdata) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = t77*(P[stateIndex][5]*t4+P[stateIndex][4]*t9+P[stateIndex][0]*t14-P[stateIndex][6]*t11+P[stateIndex][1]*t18-P[stateIndex][2]*t21+P[stateIndex][3]*t24);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15 = 3
                zero_range(&Kfusion[0], 13, 15);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(P[16][5]*t4+P[16][4]*t9+P[16][0]*t14-P[16][6]*t11+P[16][1]*t18-P[16][2]*t21+P[16][3]*t24);
                Kfusion[17] = t77*(P[17][5]*t4+P[17][4]*t9+P[17][0]*t14-P[17][6]*t11+P[17][1]*t18-P[17][2]*t21+P[17][3]*t24);
                Kfusion[18] = t77*(P[18][5]*t4+P[18][4]*t9+P[18][0]*t14-P[18][6]*t11+P[18][1]*t18-P[18][2]*t21+P[18][3]*t24);
                Kfusion[19] = t77*(P[19][5]*t4+P[19][4]*t9+P[19][0]*t14-P[19][6]*t11+P[19][1]*t18-P[19][2]*t21+P[19][3]*t24);
                Kfusion[20] = t77*(P[20][5]*t4+P[20][4]*t9+P[20][0]*t14-P[20][6]*t11+P[20][1]*t18-P[20][2]*t21+P[20][3]*t24);
                Kfusion[21] = t77*(P[21][5]*t4+P[21][4]*t9+P[21][0]*t14-P[21][6]*t11+P[21][1]*t18-P[21][2]*t21+P[21][3]*t24);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(P[22][5]*t4+P[22][4]*t9+P[22][0]*t14-P[22][6]*t11+P[22][1]*t18-P[22][2]*t21+P[22][3]*t24);
                Kfusion[23] = t77*(P[23][5]*t4+P[23][4]*t9+P[23][0]*t14-P[23][6]*t11+P[23][1]*t18-P[23][2]*t21+P[23][3]*t24);
            } else {
                // zero indexes 22 to 23
                zero_range(&Kfusion[0], 22, 23);
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
            ftype R_VEL = sq(bodyOdmDataDelayed.velErr);
            ftype t2 = q0*q3*2.0f;
            ftype t9 = q1*q2*2.0f;
            ftype t3 = t2-t9;
            ftype t4 = q0*q0;
            ftype t5 = q1*q1;
            ftype t6 = q2*q2;
            ftype t7 = q3*q3;
            ftype t8 = t4-t5+t6-t7;
            ftype t10 = q0*q1*2.0f;
            ftype t11 = q2*q3*2.0f;
            ftype t12 = t10+t11;
            ftype t13 = q1*vd*2.0f;
            ftype t14 = q0*ve*2.0f;
            ftype t26 = q3*vn*2.0f;
            ftype t15 = t13+t14-t26;
            ftype t16 = q0*vd*2.0f;
            ftype t17 = q2*vn*2.0f;
            ftype t27 = q1*ve*2.0f;
            ftype t18 = t16+t17-t27;
            ftype t19 = q3*vd*2.0f;
            ftype t20 = q2*ve*2.0f;
            ftype t21 = q1*vn*2.0f;
            ftype t22 = t19+t20+t21;
            ftype t23 = q3*ve*2.0f;
            ftype t24 = q0*vn*2.0f;
            ftype t28 = q2*vd*2.0f;
            ftype t25 = t23+t24-t28;
            ftype t29 = P[0][0]*t15;
            ftype t30 = P[1][1]*t18;
            ftype t31 = P[5][4]*t8;
            ftype t32 = P[6][4]*t12;
            ftype t33 = P[0][4]*t15;
            ftype t34 = P[1][4]*t18;
            ftype t35 = P[2][4]*t22;
            ftype t78 = P[4][4]*t3;
            ftype t79 = P[3][4]*t25;
            ftype t36 = t31+t32+t33+t34+t35-t78-t79;
            ftype t37 = P[5][6]*t8;
            ftype t38 = P[6][6]*t12;
            ftype t39 = P[0][6]*t15;
            ftype t40 = P[1][6]*t18;
            ftype t41 = P[2][6]*t22;
            ftype t81 = P[4][6]*t3;
            ftype t82 = P[3][6]*t25;
            ftype t42 = t37+t38+t39+t40+t41-t81-t82;
            ftype t43 = t12*t42;
            ftype t44 = P[5][0]*t8;
            ftype t45 = P[6][0]*t12;
            ftype t46 = P[1][0]*t18;
            ftype t47 = P[2][0]*t22;
            ftype t83 = P[4][0]*t3;
            ftype t84 = P[3][0]*t25;
            ftype t48 = t29+t44+t45+t46+t47-t83-t84;
            ftype t49 = t15*t48;
            ftype t50 = P[5][1]*t8;
            ftype t51 = P[6][1]*t12;
            ftype t52 = P[0][1]*t15;
            ftype t53 = P[2][1]*t22;
            ftype t85 = P[4][1]*t3;
            ftype t86 = P[3][1]*t25;
            ftype t54 = t30+t50+t51+t52+t53-t85-t86;
            ftype t55 = t18*t54;
            ftype t56 = P[5][2]*t8;
            ftype t57 = P[6][2]*t12;
            ftype t58 = P[0][2]*t15;
            ftype t59 = P[1][2]*t18;
            ftype t60 = P[2][2]*t22;
            ftype t87 = P[4][2]*t3;
            ftype t88 = P[3][2]*t25;
            ftype t61 = t56+t57+t58+t59+t60-t87-t88;
            ftype t62 = t22*t61;
            ftype t63 = P[5][3]*t8;
            ftype t64 = P[6][3]*t12;
            ftype t65 = P[0][3]*t15;
            ftype t66 = P[1][3]*t18;
            ftype t67 = P[2][3]*t22;
            ftype t89 = P[4][3]*t3;
            ftype t90 = P[3][3]*t25;
            ftype t68 = t63+t64+t65+t66+t67-t89-t90;
            ftype t69 = P[5][5]*t8;
            ftype t70 = P[6][5]*t12;
            ftype t71 = P[0][5]*t15;
            ftype t72 = P[1][5]*t18;
            ftype t73 = P[2][5]*t22;
            ftype t92 = P[4][5]*t3;
            ftype t93 = P[3][5]*t25;
            ftype t74 = t69+t70+t71+t72+t73-t92-t93;
            ftype t75 = t8*t74;
            ftype t80 = t3*t36;
            ftype t91 = t25*t68;
            ftype t76 = R_VEL+t43+t49+t55+t62+t75-t80-t91;
            ftype t77;

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
                // zero indexes 10 to 12 = 3
                zero_range(&Kfusion[0], 10, 12);
            }

            if (!inhibitDelVelBiasStates && !badIMUdata) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = t77*(-P[stateIndex][4]*t3+P[stateIndex][5]*t8+P[stateIndex][0]*t15+P[stateIndex][6]*t12+P[stateIndex][1]*t18+P[stateIndex][2]*t22-P[stateIndex][3]*t25);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(-P[16][4]*t3+P[16][5]*t8+P[16][0]*t15+P[16][6]*t12+P[16][1]*t18+P[16][2]*t22-P[16][3]*t25);
                Kfusion[17] = t77*(-P[17][4]*t3+P[17][5]*t8+P[17][0]*t15+P[17][6]*t12+P[17][1]*t18+P[17][2]*t22-P[17][3]*t25);
                Kfusion[18] = t77*(-P[18][4]*t3+P[18][5]*t8+P[18][0]*t15+P[18][6]*t12+P[18][1]*t18+P[18][2]*t22-P[18][3]*t25);
                Kfusion[19] = t77*(-P[19][4]*t3+P[19][5]*t8+P[19][0]*t15+P[19][6]*t12+P[19][1]*t18+P[19][2]*t22-P[19][3]*t25);
                Kfusion[20] = t77*(-P[20][4]*t3+P[20][5]*t8+P[20][0]*t15+P[20][6]*t12+P[20][1]*t18+P[20][2]*t22-P[20][3]*t25);
                Kfusion[21] = t77*(-P[21][4]*t3+P[21][5]*t8+P[21][0]*t15+P[21][6]*t12+P[21][1]*t18+P[21][2]*t22-P[21][3]*t25);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(-P[22][4]*t3+P[22][5]*t8+P[22][0]*t15+P[22][6]*t12+P[22][1]*t18+P[22][2]*t22-P[22][3]*t25);
                Kfusion[23] = t77*(-P[23][4]*t3+P[23][5]*t8+P[23][0]*t15+P[23][6]*t12+P[23][1]*t18+P[23][2]*t22-P[23][3]*t25);
            } else {
                // zero indexes 22 to 23
                zero_range(&Kfusion[0], 22, 23);
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
            ftype R_VEL = sq(bodyOdmDataDelayed.velErr);
            ftype t2 = q0*q2*2.0f;
            ftype t3 = q1*q3*2.0f;
            ftype t4 = t2+t3;
            ftype t5 = q0*q0;
            ftype t6 = q1*q1;
            ftype t7 = q2*q2;
            ftype t8 = q3*q3;
            ftype t9 = t5-t6-t7+t8;
            ftype t10 = q0*q1*2.0f;
            ftype t25 = q2*q3*2.0f;
            ftype t11 = t10-t25;
            ftype t12 = q0*vd*2.0f;
            ftype t13 = q2*vn*2.0f;
            ftype t26 = q1*ve*2.0f;
            ftype t14 = t12+t13-t26;
            ftype t15 = q1*vd*2.0f;
            ftype t16 = q0*ve*2.0f;
            ftype t27 = q3*vn*2.0f;
            ftype t17 = t15+t16-t27;
            ftype t18 = q3*ve*2.0f;
            ftype t19 = q0*vn*2.0f;
            ftype t28 = q2*vd*2.0f;
            ftype t20 = t18+t19-t28;
            ftype t21 = q3*vd*2.0f;
            ftype t22 = q2*ve*2.0f;
            ftype t23 = q1*vn*2.0f;
            ftype t24 = t21+t22+t23;
            ftype t29 = P[0][0]*t14;
            ftype t30 = P[6][4]*t9;
            ftype t31 = P[4][4]*t4;
            ftype t32 = P[0][4]*t14;
            ftype t33 = P[2][4]*t20;
            ftype t34 = P[3][4]*t24;
            ftype t78 = P[5][4]*t11;
            ftype t79 = P[1][4]*t17;
            ftype t35 = t30+t31+t32+t33+t34-t78-t79;
            ftype t36 = t4*t35;
            ftype t37 = P[6][5]*t9;
            ftype t38 = P[4][5]*t4;
            ftype t39 = P[0][5]*t14;
            ftype t40 = P[2][5]*t20;
            ftype t41 = P[3][5]*t24;
            ftype t80 = P[5][5]*t11;
            ftype t81 = P[1][5]*t17;
            ftype t42 = t37+t38+t39+t40+t41-t80-t81;
            ftype t43 = P[6][0]*t9;
            ftype t44 = P[4][0]*t4;
            ftype t45 = P[2][0]*t20;
            ftype t46 = P[3][0]*t24;
            ftype t83 = P[5][0]*t11;
            ftype t84 = P[1][0]*t17;
            ftype t47 = t29+t43+t44+t45+t46-t83-t84;
            ftype t48 = t14*t47;
            ftype t49 = P[6][1]*t9;
            ftype t50 = P[4][1]*t4;
            ftype t51 = P[0][1]*t14;
            ftype t52 = P[2][1]*t20;
            ftype t53 = P[3][1]*t24;
            ftype t85 = P[5][1]*t11;
            ftype t86 = P[1][1]*t17;
            ftype t54 = t49+t50+t51+t52+t53-t85-t86;
            ftype t55 = P[6][2]*t9;
            ftype t56 = P[4][2]*t4;
            ftype t57 = P[0][2]*t14;
            ftype t58 = P[2][2]*t20;
            ftype t59 = P[3][2]*t24;
            ftype t88 = P[5][2]*t11;
            ftype t89 = P[1][2]*t17;
            ftype t60 = t55+t56+t57+t58+t59-t88-t89;
            ftype t61 = t20*t60;
            ftype t62 = P[6][3]*t9;
            ftype t63 = P[4][3]*t4;
            ftype t64 = P[0][3]*t14;
            ftype t65 = P[2][3]*t20;
            ftype t66 = P[3][3]*t24;
            ftype t90 = P[5][3]*t11;
            ftype t91 = P[1][3]*t17;
            ftype t67 = t62+t63+t64+t65+t66-t90-t91;
            ftype t68 = t24*t67;
            ftype t69 = P[6][6]*t9;
            ftype t70 = P[4][6]*t4;
            ftype t71 = P[0][6]*t14;
            ftype t72 = P[2][6]*t20;
            ftype t73 = P[3][6]*t24;
            ftype t92 = P[5][6]*t11;
            ftype t93 = P[1][6]*t17;
            ftype t74 = t69+t70+t71+t72+t73-t92-t93;
            ftype t75 = t9*t74;
            ftype t82 = t11*t42;
            ftype t87 = t17*t54;
            ftype t76 = R_VEL+t36+t48+t61+t68+t75-t82-t87;
            ftype t77;

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
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);

            }

            if (!inhibitDelVelBiasStates && !badIMUdata) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = t77*(P[stateIndex][4]*t4+P[stateIndex][0]*t14+P[stateIndex][6]*t9-P[stateIndex][5]*t11-P[stateIndex][1]*t17+P[stateIndex][2]*t20+P[stateIndex][3]*t24);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t77*(P[16][4]*t4+P[16][0]*t14+P[16][6]*t9-P[16][5]*t11-P[16][1]*t17+P[16][2]*t20+P[16][3]*t24);
                Kfusion[17] = t77*(P[17][4]*t4+P[17][0]*t14+P[17][6]*t9-P[17][5]*t11-P[17][1]*t17+P[17][2]*t20+P[17][3]*t24);
                Kfusion[18] = t77*(P[18][4]*t4+P[18][0]*t14+P[18][6]*t9-P[18][5]*t11-P[18][1]*t17+P[18][2]*t20+P[18][3]*t24);
                Kfusion[19] = t77*(P[19][4]*t4+P[19][0]*t14+P[19][6]*t9-P[19][5]*t11-P[19][1]*t17+P[19][2]*t20+P[19][3]*t24);
                Kfusion[20] = t77*(P[20][4]*t4+P[20][0]*t14+P[20][6]*t9-P[20][5]*t11-P[20][1]*t17+P[20][2]*t20+P[20][3]*t24);
                Kfusion[21] = t77*(P[21][4]*t4+P[21][0]*t14+P[21][6]*t9-P[21][5]*t11-P[21][1]*t17+P[21][2]*t20+P[21][3]*t24);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t77*(P[22][4]*t4+P[22][0]*t14+P[22][6]*t9-P[22][5]*t11-P[22][1]*t17+P[22][2]*t20+P[22][3]*t24);
                Kfusion[23] = t77*(P[23][4]*t4+P[23][0]*t14+P[23][6]*t9-P[23][5]*t11-P[23][1]*t17+P[23][2]*t20+P[23][3]*t24);
            } else {
                // zero indexes 22 to 23
                zero_range(&Kfusion[0], 22, 23);
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
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u fusing odometry",(unsigned)imu_index);
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
#endif // EK3_FEATURE_BODY_ODOM

#if EK3_FEATURE_BODY_ODOM
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

    // Check for body odometry data (aka visual position delta) at the fusion time horizon
    const bool bodyOdomDataToFuse = storedBodyOdm.recall(bodyOdmDataDelayed, imuDataDelayed.time_ms);
    if (bodyOdomDataToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV)) {

        // Fuse data into the main filter
        FuseBodyVel();
    }

    // Check for wheel encoder data at the fusion time horizon
    const bool wheelOdomDataToFuse = storedWheelOdm.recall(wheelOdmDataDelayed, imuDataDelayed.time_ms);
    if (wheelOdomDataToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::WHEEL_ENCODER)) {

        // check if the delta time is too small to calculate a velocity
        if (wheelOdmDataDelayed.delTime > EKF_TARGET_DT) {

            // get the forward velocity
            ftype fwdSpd = wheelOdmDataDelayed.delAng * wheelOdmDataDelayed.radius * (1.0f / wheelOdmDataDelayed.delTime);

            // get the unit vector from the projection of the X axis onto the horizontal
            Vector3F unitVec;
            unitVec.x = prevTnb.a.x;
            unitVec.y = prevTnb.a.y;
            unitVec.z = 0.0f;
            unitVec.normalize();

            // multiply by forward speed to get velocity vector measured by wheel encoders
            Vector3F velNED = unitVec * fwdSpd;

            // This is a hack to enable use of the existing body frame velocity fusion method
            // TODO write a dedicated observation model for wheel encoders
            bodyOdmDataDelayed.vel = prevTnb * velNED;
            bodyOdmDataDelayed.body_offset = wheelOdmDataDelayed.hub_offset;
            bodyOdmDataDelayed.velErr = frontend->_wencOdmVelErr;

            // Fuse data into the main filter
            FuseBodyVel();
        }
    }
}
#endif // EK3_FEATURE_BODY_ODOM
