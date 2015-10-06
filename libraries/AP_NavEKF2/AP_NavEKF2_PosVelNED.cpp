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

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF2_core::ResetVelocity(void)
{
    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.zero();
    } else if (!gpsNotAvailable) {
        // reset horizontal velocity states, applying an offset to the GPS velocity to prevent the GPS position being rejected when the GPS position offset is being decayed to zero.
        stateStruct.velocity.x  = gpsDataNew.vel.x + gpsVelGlitchOffset.x; // north velocity from blended accel data
        stateStruct.velocity.y  = gpsDataNew.vel.y + gpsVelGlitchOffset.y; // east velocity from blended accel data
    }
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;
}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF2_core::ResetPosition(void)
{
    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
    } else if (!gpsNotAvailable) {
        // write to state vector and compensate for offset  between last GPs measurement and the EKF time horizon
        stateStruct.position.x = gpsDataNew.pos.x + gpsPosGlitchOffsetNE.x + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(lastTimeGpsReceived_ms));
        stateStruct.position.y = gpsDataNew.pos.y + gpsPosGlitchOffsetNE.y + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(lastTimeGpsReceived_ms));
    }
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;
}

// reset the vertical position state using the last height measurement
void NavEKF2_core::ResetHeight(void)
{
    // read the altimeter
    readHgtData();
    // write to the state vector
    stateStruct.position.z = -baroDataNew.hgt; // down position from blended accel data
    terrainState = stateStruct.position.z + rngOnGnd;
    for (uint8_t i=0; i<IMU_BUFFER_LENGTH; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;
}

// Reset the baro so that it reads zero at the current height
// Reset the EKF height to zero
// Adjust the EKf origin height so that the EKF height + origin height is the same as before
// Return true if the height datum reset has been performed
// If using a range finder for height do not reset and return false
bool NavEKF2_core::resetHeightDatum(void)
{
    // if we are using a range finder for height, return false
    if (frontend._altSource == 1) {
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    _baro.update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and afer the reset is the same
    if (validOrigin) {
        EKF_origin.alt += oldHgt*100;
    }
    return true;
}




/********************************************************
*            GET STATES/PARAMS FUNCTIONS                *
********************************************************/

// return NED velocity in m/s
//
void NavEKF2_core::getVelNED(Vector3f &vel) const
{
    vel = outputDataNew.velocity;
}

// This returns the specific forces in the NED frame
void NavEKF2_core::getAccelNED(Vector3f &accelNED) const {
    accelNED = velDotNED;
    accelNED.z -= GRAVITY_MSS;
}

// return the Z-accel bias estimate in m/s^2
void NavEKF2_core::getAccelZBias(float &zbias) const {
    if (dtIMUavg > 0) {
        zbias = stateStruct.accel_zbias / dtIMUavg;
    } else {
        zbias = 0;
    }
}

// Return the last calculated NED position relative to the reference point (m).
// if a calculated solution is not available, use the best available data and return false
bool NavEKF2_core::getPosNED(Vector3f &pos) const
{
    // The EKF always has a height estimate regardless of mode of operation
    pos.z = outputDataNew.position.z;
    // There are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no position estimate available)
    nav_filter_status status;
    getFilterStatus(status);
    if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
        // This is the normal mode of operation where we can use the EKF position states
        pos.x = outputDataNew.position.x;
        pos.y = outputDataNew.position.y;
        return true;
    } else {
        // In constant position mode the EKF position states are at the origin, so we cannot use them as a position estimate
        if(validOrigin) {
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // If the origin has been set and we have GPS, then return the GPS position relative to the origin
                const struct Location &gpsloc = _ahrs->get_gps().location();
                Vector2f tempPosNE = location_diff(EKF_origin, gpsloc);
                pos.x = tempPosNE.x;
                pos.y = tempPosNE.y;
                return false;
            } else {
                // If no GPS fix is available, all we can do is provide the last known position
                pos.x = outputDataNew.position.x;
                pos.y = outputDataNew.position.y;
                return false;
            }
        } else {
            // If the origin has not been set, then we have no means of providing a relative position
            pos.x = 0.0f;
            pos.y = 0.0f;
            return false;
        }
    }
    return false;
}

// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF2_core::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (PV_AidingMode == AID_RELATIVE) {
        // allow 1.0 rad/sec margin for angular motion
        ekfGndSpdLimit = max((frontend._maxFlowRate - 1.0f), 0.0f) * max((terrainState - stateStruct.position[2]), rngOnGnd);
        // use standard gains up to 5.0 metres height and reduce above that
        ekfNavVelGainScaler = 4.0f / max((terrainState - stateStruct.position[2]),4.0f);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
    }
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF2_core::getLLH(struct Location &loc) const
{
    if(validOrigin) {
        // Altitude returned is an absolute altitude relative to the WGS-84 spherioid
        loc.alt = EKF_origin.alt - outputDataNew.position.z*100;
        loc.flags.relative_alt = 0;
        loc.flags.terrain_alt = 0;

        // there are three modes of operation, absolute position (GPS fusion), relative position (optical flow fusion) and constant position (no aiding)
        nav_filter_status status;
        getFilterStatus(status);
        if (status.flags.horiz_pos_abs || status.flags.horiz_pos_rel) {
            loc.lat = EKF_origin.lat;
            loc.lng = EKF_origin.lng;
            location_offset(loc, outputDataNew.position.x, outputDataNew.position.y);
            return true;
        } else {
            // we could be in constant position mode  becasue the vehicle has taken off without GPS, or has lost GPS
            // in this mode we cannot use the EKF states to estimate position so will return the best available data
            if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
                // we have a GPS position fix to return
                const struct Location &gpsloc = _ahrs->get_gps().location();
                loc.lat = gpsloc.lat;
                loc.lng = gpsloc.lng;
                return true;
            } else {
                // if no GPS fix, provide last known position before entering the mode
                location_offset(loc, lastKnownPositionNE.x, lastKnownPositionNE.y);
                return false;
            }
        }
    } else {
        // If no origin has been defined for the EKF, then we cannot use its position states so return a raw
        // GPS reading if available and return false
        if ((_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D)) {
            const struct Location &gpsloc = _ahrs->get_gps().location();
            loc = gpsloc;
            loc.flags.relative_alt = 0;
            loc.flags.terrain_alt = 0;
        }
        return false;
    }
}

// return the LLH location of the filters NED origin
bool NavEKF2_core::getOriginLLH(struct Location &loc) const
{
    if (validOrigin) {
        loc = EKF_origin;
    }
    return validOrigin;
}

// return the estimated height above ground level
bool NavEKF2_core::getHAGL(float &HAGL) const
{
    HAGL = terrainState - outputDataNew.position.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && healthy();
}

// return data for debugging optical flow fusion
void NavEKF2_core::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    varFlow = max(flowTestRatio[0],flowTestRatio[1]);
    gndOffset = terrainState;
    flowInnovX = innovOptFlow[0];
    flowInnovY = innovOptFlow[1];
    auxInnov = auxFlowObsInnov;
    HAGL = terrainState - stateStruct.position.z;
    rngInnov = innovRng;
    range = rngMea;
    gndOffsetErr = sqrtf(Popt); // note Popt is constrained to be non-negative in EstimateTerrainOffset()
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF2_core::getHeightControlLimit(float &height) const
{
    // only ask for limiting if we are doing optical flow navigation
    if (frontend._fusionModeGPS == 3) {
        // If are doing optical flow nav, ensure the height above ground is within range finder limits after accounting for vehicle tilt and control errors
        height = max(float(_rng.max_distance_cm()) * 0.007f - 1.0f, 1.0f);
        return true;
    } else {
        return false;
    }
}



/********************************************************
*            SET STATES/PARAMS FUNCTIONS                *
********************************************************/

// set the LLH location of the filters NED origin
bool NavEKF2_core::setOriginLLH(struct Location &loc)
{
    if (isAiding) {
        return false;
    }
    EKF_origin = loc;
    validOrigin = true;
    return true;
}

// Set the NED origin to be used until the next filter reset
void NavEKF2_core::setOrigin()
{
    // assume origin at current GPS location (no averaging)
    EKF_origin = _ahrs->get_gps().location();
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);
    validOrigin = true;
    hal.console->printf("EKF Origin Set\n");
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming
// This command is forgotten by the EKF each time the vehicle disarms
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF2_core::setInhibitGPS(void)
{
    if(!isAiding) {
        return 0;
    }
    if (optFlowDataPresent()) {
        frontend._fusionModeGPS = 3;
//#error writing to a tuning parameter
        return 2;
    } else {
        return 1;
    }
}


/********************************************************
*                      READ SENSORS                     *
********************************************************/

bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = _ahrs->get_ins();

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);
        dVel_dt = max(ins.get_delta_velocity_dt(ins_index),1.0e-4f);
        return true;
    }
    return false;
}

// check for new valid GPS data and update stored measurement if available
void NavEKF2_core::readGpsData()
{
    // check for new GPS data
    if ((_ahrs->get_gps().last_message_time_ms() != lastTimeGpsReceived_ms) &&
            (_ahrs->get_gps().status() >= AP_GPS::GPS_OK_FIX_3D))
    {
        // store fix time from previous read
        secondLastGpsTime_ms = lastTimeGpsReceived_ms;

        // get current fix time
        lastTimeGpsReceived_ms = _ahrs->get_gps().last_message_time_ms();

        // estimate when the GPS fix was valid, allowing for GPS processing and other delays
        // ideally we should be using a timing signal from the GPS receiver to set this time
        gpsDataNew.time_ms = lastTimeGpsReceived_ms - frontend._gpsDelay_ms;

        // read the NED velocity from the GPS
        gpsDataNew.vel = _ahrs->get_gps().velocity();

        // Use the speed accuracy from the GPS if available, otherwise set it to zero.
        // Apply a decaying envelope filter with a 5 second time constant to the raw speed accuracy data
        float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
        gpsSpdAccuracy *= (1.0f - alpha);
        float gpsSpdAccRaw;
        if (!_ahrs->get_gps().speed_accuracy(gpsSpdAccRaw)) {
            gpsSpdAccuracy = 0.0f;
        } else {
            gpsSpdAccuracy = max(gpsSpdAccuracy,gpsSpdAccRaw);
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
        if (_ahrs->get_gps().have_vertical_velocity() && frontend._fusionModeGPS == 0) {
            useGpsVertVel = true;
        } else {
            useGpsVertVel = false;
        }

        // Monitor quality of the GPS velocity data for alignment
        if (PV_AidingMode != AID_ABSOLUTE) {
            gpsQualGood = calcGpsGoodToAlign();
        }

        // read latitutde and longitude from GPS and convert to local NE position relative to the stored origin
        // If we don't have an origin, then set it to the current GPS coordinates
        const struct Location &gpsloc = _ahrs->get_gps().location();
        if (validOrigin) {
            gpsDataNew.pos = location_diff(EKF_origin, gpsloc);
        } else if (gpsQualGood) {
            // Set the NE origin to the current GPS position
            setOrigin();
            // Now we know the location we have an estimate for the magnetic field declination and adjust the earth field accordingly
            alignMagStateDeclination();
            // Set the height of the NED origin to ‘height of baro height datum relative to GPS height datum'
            EKF_origin.alt = gpsloc.alt - baroDataNew.hgt;
            // We are by definition at the origin at the instant of alignment so set NE position to zero
            gpsDataNew.pos.zero();
            // If GPS useage isn't explicitly prohibited, we switch to absolute position mode
            if (isAiding && frontend._fusionModeGPS != 3) {
                PV_AidingMode = AID_ABSOLUTE;
                // Initialise EKF position and velocity states
                ResetPosition();
                ResetVelocity();
            }
        }

        // calculate a position offset which is applied to NE position and velocity wherever it is used throughout code to allow GPS position jumps to be accommodated gradually
        decayGpsOffset();

        // save measurement to buffer to be fused later
        StoreGPS();

        // declare GPS available for use
        gpsNotAvailable = false;
    }

    // We need to handle the case where GPS is lost for a period of time that is too long to dead-reckon
    // If that happens we need to put the filter into a constant position mode, reset the velocity states to zero
    // and use the last estimated position as a synthetic GPS position

    // check if we can use opticalflow as a backup
    bool optFlowBackupAvailable = (flowDataValid && !hgtTimeout);

    // Set GPS time-out threshold depending on whether we have an airspeed sensor to constrain drift
    uint16_t gpsRetryTimeout_ms = useAirspeed() ? frontend.gpsRetryTimeUseTAS_ms : frontend.gpsRetryTimeNoTAS_ms;

    // Set the time that copters will fly without a GPS lock before failing the GPS and switching to a non GPS mode
    uint16_t gpsFailTimeout_ms = optFlowBackupAvailable ? frontend.gpsFailTimeWithFlow_ms : gpsRetryTimeout_ms;

    // If we haven't received GPS data for a while and we are using it for aiding, then declare the position and velocity data as being timed out
    if (imuSampleTime_ms - lastTimeGpsReceived_ms > gpsFailTimeout_ms) {

        // Let other processes know that GPS i snota vailable and that a timeout has occurred
        posTimeout = true;
        velTimeout = true;
        gpsNotAvailable = true;

        // If we are currently reliying on GPS for navigation, then we need to switch to a non-GPS mode of operation
        if (PV_AidingMode == AID_ABSOLUTE) {

            // If we don't have airspeed or sideslip assumption or optical flow to constrain drift, then go into constant position mode.
            // If we can do optical flow nav (valid flow data and height above ground estimate), then go into flow nav mode.
            if (!useAirspeed() && !assume_zero_sideslip()) {
                if (optFlowBackupAvailable) {
                    // we can do optical flow only nav
                    frontend._fusionModeGPS = 3;
                    PV_AidingMode = AID_RELATIVE;
                } else {
                    // store the current position
                    lastKnownPositionNE.x = stateStruct.position.x;
                    lastKnownPositionNE.y = stateStruct.position.y;

                    // put the filter into constant position mode
                    PV_AidingMode = AID_NONE;

                    // reset all glitch states
                    gpsPosGlitchOffsetNE.zero();
                    gpsVelGlitchOffset.zero();

                    // Reset the velocity and position states
                    ResetVelocity();
                    ResetPosition();

                    // Reset the normalised innovation to avoid false failing the bad position fusion test
                    velTestRatio = 0.0f;
                    posTestRatio = 0.0f;
                }
            }
        }
    }

    // If not aiding we synthesise the GPS measurements at the last known position
    if (PV_AidingMode == AID_NONE) {
        if (imuSampleTime_ms - gpsDataNew.time_ms > 200) {
            gpsDataNew.pos.x = lastKnownPositionNE.x;
            gpsDataNew.pos.y = lastKnownPositionNE.y;
            gpsDataNew.time_ms = imuSampleTime_ms-frontend._gpsDelay_ms;
            // save measurement to buffer to be fused later
            StoreGPS();
        }
    }

}


// store GPS data in a history array
void NavEKF2_core::StoreGPS()
{
    if (gpsStoreIndex >= OBS_BUFFER_LENGTH) {
        gpsStoreIndex = 0;
    }
    storedGPS[gpsStoreIndex] = gpsDataNew;
    gpsStoreIndex += 1;
}

// return newest un-used GPS data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF2_core::RecallGPS()
{
    gps_elements dataTemp;
    gps_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedGPS[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedGPS[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                gpsDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}

// check for new altitude measurement data and update stored measurement if available
void NavEKF2_core::readHgtData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    if (_baro.get_last_update() != lastHgtReceived_ms) {
        // Don't use Baro height if operating in optical flow mode as we use range finder instead
        if (frontend._fusionModeGPS == 3 && frontend._altSource == 1) {
            if ((imuSampleTime_ms - rngValidMeaTime_ms) < 2000) {
                // adjust range finder measurement to allow for effect of vehicle tilt and height of sensor
                baroDataNew.hgt = max(rngMea * Tnb_flow.c.z, rngOnGnd);
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + stateStruct.position.z) + 0.9f * baroHgtOffset;
            } else if (isAiding && takeOffDetected) {
                // use baro measurement and correct for baro offset - failsafe use only as baro will drift
                baroDataNew.hgt = max(_baro.get_altitude() - baroHgtOffset, rngOnGnd);
            } else {
                // If we are on ground and have no range finder reading, assume the nominal on-ground height
                baroDataNew.hgt = rngOnGnd;
                // calculate offset to baro data that enables baro to be used as a backup
                // filter offset to reduce effect of baro noise and other transient errors on estimate
                baroHgtOffset = 0.1f * (_baro.get_altitude() + stateStruct.position.z) + 0.9f * baroHgtOffset;
            }
        } else {
            // use baro measurement and correct for baro offset
            baroDataNew.hgt = _baro.get_altitude();
        }

        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = frontend.hgtAvg_ms*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        } else if (isAiding && getTakeoffExpected()) {
            // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
            // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
            baroDataNew.hgt = max(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastHgtReceived_ms = _baro.get_last_update();

        // estimate of time height measurement was taken, allowing for delays
        hgtMeasTime_ms = lastHgtReceived_ms - frontend._hgtDelay_ms;

        // save baro measurement to buffer to be fused later
        baroDataNew.time_ms = hgtMeasTime_ms;
        StoreBaro();
    }
}

// store baro in a history array
void NavEKF2_core::StoreBaro()
{
    if (baroStoreIndex >= OBS_BUFFER_LENGTH) {
        baroStoreIndex = 0;
    }
    storedBaro[baroStoreIndex] = baroDataNew;
    baroStoreIndex += 1;
}

// return newest un-used baro data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF2_core::RecallBaro()
{
    baro_elements dataTemp;
    baro_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedBaro[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedBaro[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                baroDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}


// Read the range finder and take new measurements if available
// Read at 20Hz and apply a median filter
void NavEKF2_core::readRangeFinder(void)
{
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;
    // get theoretical correct range when the vehicle is on the ground
    rngOnGnd = _rng.ground_clearance_cm() * 0.01f;
    if (_rng.status() == RangeFinder::RangeFinder_Good && (imuSampleTime_ms - lastRngMeasTime_ms) > 50) {
        // store samples and sample time into a ring buffer
        rngMeasIndex ++;
        if (rngMeasIndex > 2) {
            rngMeasIndex = 0;
        }
        storedRngMeasTime_ms[rngMeasIndex] = imuSampleTime_ms;
        storedRngMeas[rngMeasIndex] = _rng.distance_cm() * 0.01f;
        // check for three fresh samples and take median
        bool sampleFresh[3];
        for (uint8_t index = 0; index <= 2; index++) {
            sampleFresh[index] = (imuSampleTime_ms - storedRngMeasTime_ms[index]) < 500;
        }
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
            rngMea = max(storedRngMeas[midIndex],rngOnGnd);
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
        } else if (onGround) {
            // if on ground and no return, we assume on ground range
            rngMea = rngOnGnd;
            newDataRng = true;
            rngValidMeaTime_ms = imuSampleTime_ms;
        } else {
            newDataRng = false;
        }
        lastRngMeasTime_ms =  imuSampleTime_ms;
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
    if (frontend._fusionModeGPS == 3) {
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
        ofDataNew.time_ms = imuSampleTime_ms - frontend._flowDelay_ms - frontend.flowTimeDeltaAvg_ms/2;
        // Save data to buffer
        StoreOF();
        // Check for data at the fusion time horizon
        newDataFlow = RecallOF();
    }
}

// store OF data in a history array
void NavEKF2_core::StoreOF()
{
    if (ofStoreIndex >= OBS_BUFFER_LENGTH) {
        ofStoreIndex = 0;
    }
    storedOF[ofStoreIndex] = ofDataNew;
    ofStoreIndex += 1;
}

// return newest un-used optical flow data that has fallen behind the fusion time horizon
// if no un-used data is available behind the fusion horizon, return false
bool NavEKF2_core::RecallOF()
{
    of_elements dataTemp;
    of_elements dataTempZero;
    dataTempZero.time_ms = 0;
    uint32_t temp_ms = 0;
    for (uint8_t i=0; i<OBS_BUFFER_LENGTH; i++) {
        dataTemp = storedOF[i];
        // find a measurement older than the fusion time horizon that we haven't checked before
        if (dataTemp.time_ms != 0 && dataTemp.time_ms <= imuDataDelayed.time_ms) {
            // zero the time stamp so we won't use it again
            storedOF[i]=dataTempZero;
            // Find the most recent non-stale measurement that meets the time horizon criteria
            if (((imuDataDelayed.time_ms - dataTemp.time_ms) < 500) && dataTemp.time_ms > temp_ms) {
                ofDataDelayed = dataTemp;
                temp_ms = dataTemp.time_ms;
            }
        }
    }
    if (temp_ms != 0) {
        return true;
    } else {
        return false;
    }
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
void NavEKF2_core::SelectVelPosFusion()
{
    // check for and read new GPS data
    readGpsData();

    // Determine if we need to fuse position and velocity data on this time step
    if (RecallGPS() && PV_AidingMode != AID_RELATIVE) {
        // Don't fuse velocity data if GPS doesn't support it
        // If no aiding is avaialble, then we use zeroed GPS position and  elocity data to constrain
        // tilt errors assuming that the vehicle is not accelerating
        if (frontend._fusionModeGPS <= 1 || PV_AidingMode == AID_NONE) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;
    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // check for and read new height data
    readHgtData();

    // If we haven't received height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = (useGpsVertVel && !velTimeout) ? frontend.hgtRetryTimeMode0_ms : frontend.hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtReceived_ms > hgtRetryTime_ms) {
        hgtTimeout = true;
    }

    // command fusion of height data
    // wait until the EKF time horizon catches up with the measurement
    if (RecallBaro()) {
        // enable fusion
        fuseHgtData = true;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        FuseVelPosNED();
    }
}

// fuse selected position, velocity and height measurements
void NavEKF2_core::FuseVelPosNED()
{
    // start performance timer
    perf_begin(_perf_FuseVelPosNED);

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
    float posErr;
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData) {

        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed()) gpsRetryTime = frontend.gpsRetryTimeUseTAS_ms;
        else gpsRetryTime = frontend.gpsRetryTimeNoTAS_ms;

        // form the observation vector and zero velocity and horizontal position observations if in constant position mode
        // If in constant velocity mode, hold the last known horizontal velocity vector
        if (PV_AidingMode == AID_ABSOLUTE) {
            observation[0] = gpsDataDelayed.vel.x + gpsVelGlitchOffset.x;
            observation[1] = gpsDataDelayed.vel.y + gpsVelGlitchOffset.y;
            observation[2] = gpsDataDelayed.vel.z;
            observation[3] = gpsDataDelayed.pos.x + gpsPosGlitchOffsetNE.x;
            observation[4] = gpsDataDelayed.pos.y + gpsPosGlitchOffsetNE.y;
        } else if (PV_AidingMode == AID_NONE) {
            for (uint8_t i=0; i<=4; i++) observation[i] = 0.0f;
        }
        observation[5] = -baroDataDelayed.hgt;

        // calculate additional error in GPS position caused by manoeuvring
        posErr = frontend.gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // if the GPS is able to report a speed error, we use it to adjust the observation noise for GPS velocity
        // otherwise we scale it using manoeuvre acceleration
        if (gpsSpdAccuracy > 0.0f) {
            // use GPS receivers reported speed accuracy - floor at value set by gps noise parameter
            R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend._gpsHorizVelNoise, 50.0f));
            R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend._gpsVertVelNoise, 50.0f));
        } else {
            // calculate additional error in GPS velocity caused by manoeuvring
            R_OBS[0] = sq(constrain_float(frontend._gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend.gpsNEVelVarAccScale * accNavMag);
            R_OBS[2] = sq(constrain_float(frontend._gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend.gpsDVelVarAccScale  * accNavMag);
        }
        R_OBS[1] = R_OBS[0];
        R_OBS[3] = sq(constrain_float(frontend._gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = sq(constrain_float(frontend._baroAltNoise, 0.1f, 10.0f));

        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if (getTakeoffExpected() || getTouchdownExpected()) {
            R_OBS[5] *= frontend.gndEffectBaroScaler;
        }

        // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
        // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
        // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
        for (uint8_t i=0; i<=1; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend._gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend.gpsNEVelVarAccScale * accNavMag);
        for (uint8_t i=2; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];


        // if vertical GPS velocity data is being used, check to see if the GPS vertical velocity and barometer
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        if (useGpsVertVel && fuseVelData && (imuSampleTime_ms - lastHgtReceived_ms) <  (2 * frontend.hgtAvg_ms)) {
            // calculate innovations for height and vertical GPS vel measurements
            float hgtErr  = stateStruct.position.z - observation[5];
            float velDErr = stateStruct.velocity.z - observation[2];
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
            innovVelPos[3] = stateStruct.position.x - observation[3];
            innovVelPos[4] = stateStruct.position.y - observation[4];
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            float maxPosInnov2 = sq(frontend._gpsPosInnovGate)*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // declare a timeout condition if we have been too long without data or not aiding
            posTimeout = (((imuSampleTime_ms - lastPosPassTime_ms) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // use position data if healthy, timed out, or in constant position mode
            if (posHealth || posTimeout || (PV_AidingMode == AID_NONE)) {
                posHealth = true;
                // only reset the failed time and do glitch timeout checks if we are doing full aiding
                if (PV_AidingMode == AID_ABSOLUTE) {
                    lastPosPassTime_ms = imuSampleTime_ms;
                    // if timed out or outside the specified uncertainty radius, increment the offset applied to GPS data to compensate for large GPS position jumps
                    if (posTimeout || ((varInnovVelPos[3] + varInnovVelPos[4]) > sq(float(frontend._gpsGlitchRadiusMax)))) {
                        gpsPosGlitchOffsetNE.x += innovVelPos[3];
                        gpsPosGlitchOffsetNE.y += innovVelPos[4];
                        // limit the radius of the offset and decay the offset to zero radially
                        decayGpsOffset();
                        // reset the position to the current GPS position which will include the glitch correction offset
                        ResetPosition();
                        // reset the velocity to the GPS velocity
                        ResetVelocity();
                        // don't fuse data on this time step
                        fusePosData = false;
                        // Reset the normalised innovation to avoid false failing the bad position fusion test
                        posTestRatio = 0.0f;
                        velTestRatio = 0.0f;
                    }
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            if (frontend._fusionModeGPS == 1) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3
                stateIndex   = i + 3;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = stateStruct.velocity[i] - observation[i]; // blended
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(frontend._gpsVelInnovGate));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // declare a timeout if we have not fused velocity data for too long or not aiding
            velTimeout = (((imuSampleTime_ms - lastVelPassTime_ms) > gpsRetryTime) || PV_AidingMode == AID_NONE);
            // if data is healthy  or in constant velocity mode we fuse it
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count
                lastVelPassTime_ms = imuSampleTime_ms;
            } else if (velTimeout && !posHealth && PV_AidingMode == AID_ABSOLUTE) {
                // if data is not healthy and timed out and position is unhealthy and we are using aiding, we reset the velocity, but do not fuse data on this time step
                ResetVelocity();
                fuseVelData =  false;
                // Reset the normalised innovation to avoid false failing the bad position fusion test
                velTestRatio = 0.0f;
            } else {
                // if data is unhealthy and position is healthy, we do not fuse it
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // calculate height innovations
            innovVelPos[5] = stateStruct.position.z - observation[5];

            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(frontend._hgtInnovGate) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            hgtTimeout = (imuSampleTime_ms - lastHgtPassTime_ms) > hgtRetryTime_ms;
            // Fuse height data if healthy or timed out or in constant position mode
            if (hgtHealth || hgtTimeout || (PV_AidingMode == AID_NONE)) {
                hgtHealth = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
                // if timed out, reset the height, but do not fuse data on this time step
                if (hgtTimeout) {
                    ResetHeight();
                    fuseHgtData = false;
                }
            }
            else {
                hgtHealth = false;
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
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    if (obsIndex == 5) {
                        const float gndMaxBaroErr = 4.0f;
                        const float gndBaroInnovFloor = -0.5f;

                        if(getTouchdownExpected()) {
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

                // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                stateStruct.angErr.zero();

                // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                // Don't apply corrections to Z bias state as this has been done already as part of the single IMU calculations
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

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    perf_end(_perf_FuseVelPosNED);
}

// select fusion of optical flow measurements
void NavEKF2_core::SelectFlowFusion()
{
    // start performance timer
    perf_begin(_perf_FuseOptFlow);
    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // Check if the optical flow sensor has timed out
    bool flowSensorTimeout = ((imuSampleTime_ms - flowValidMeaTime_ms) > 5000);
    // Check if the fusion has timed out (flow measurements have been rejected for too long)
    bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
    // check is the terrain offset estimate is still valid
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000);
    // Perform tilt check
    bool tiltOK = (Tnb_flow.c.z > frontend.DCM33FlowMin);
    // Constrain measurements to zero if we are using optical flow and are on the ground
    if (frontend._fusionModeGPS == 3 && !takeOffDetected && isAiding) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // If the flow measurements have been rejected for too long and we are relying on them, then revert to constant position mode
    if ((flowSensorTimeout || flowFusionTimeout) && PV_AidingMode == AID_RELATIVE) {
        PV_AidingMode = AID_NONE;
        // reset the velocity
        ResetVelocity();
        // store the current position to be used to as a sythetic position measurement
        lastKnownPositionNE.x = stateStruct.position.x;
        lastKnownPositionNE.y = stateStruct.position.y;
        // reset the position
        ResetPosition();
    }

    // if we do have valid flow measurements, fuse data into a 1-state EKF to estimate terrain height
    // we don't do terrain height estimation in optical flow only mode as the ground becomes our zero height reference
    if ((newDataFlow || newDataRng) && tiltOK) {
        // fuse range data into the terrain estimator if available
        fuseRngData = newDataRng;
        // fuse optical flow data into the terrain estimator if available and if there is no range data (range data is better)
        fuseOptFlowData = (newDataFlow && !fuseRngData);
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset();
        // Indicate we have used the range data
        newDataRng = false;
    }

    // Fuse optical flow data into the main filter if not excessively tilted and we are in the correct mode
    if (newDataFlow && tiltOK && PV_AidingMode == AID_RELATIVE)
    {
        // Set the flow noise used by the fusion processes
        R_LOS = sq(max(frontend._flowNoise, 0.05f));
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        FuseOptFlow();
        // reset flag to indicate that no new flow data is available for fusion
        newDataFlow = false;
    }

    // stop the performance timer
    perf_end(_perf_FuseOptFlow);
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optiocal flow rates and range finder measurements
*/
void NavEKF2_core::EstimateTerrainOffset()
{
    // start performance timer
    perf_begin(_perf_OpticalFlowEKF);

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = max((terrainState - stateStruct.position.z), rngOnGnd);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);
    float losRateSq = velHorizSq / sq(heightAboveGndEst);

    // don't update terrain offset state if there is no range finder and not generating enough LOS rate, or without GPS, as it is poorly observable
    if (!fuseRngData && (gpsNotAvailable || PV_AidingMode == AID_RELATIVE || velHorizSq < 25.0f || losRateSq < 0.01f)) {
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
        // record the time we last updated the terrain offset state
        gndHgtValidTime_ms = imuSampleTime_ms;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
        float distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = min(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have a time based error growth that is scaled using the gradient parameter
        float timeLapsed = min(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(0.01f*float(frontend.gndGradientSigma))) + sq(float(frontend.gndGradientSigma) * timeLapsed);
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (fuseRngData) {
            // predict range
            float predRngMeas = max((terrainState - stateStruct.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // Copy required states to local variable names
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = frontend._rngNoise;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rngMea;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(frontend._rngInnovGate) * varInnovRng);

            // Check the innovation for consistency and don't fuse if > 5Sigma
            if ((sq(innovRng)*SK_RNG) < 25.0f)
            {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = max(Popt,0.0f);

            }
        }

        if (fuseOptFlowData) {

            Vector3f vel; // velocity of sensor relative to ground in NED axes
            Vector3f relVelSensor; // velocity of sensor relative to ground in sensor axes
            float losPred; // predicted optical flow angular rate measurement
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;

            // Correct velocities for GPS glitch recovery offset
            vel.x          = stateStruct.velocity[0] - gpsVelGlitchOffset.x;
            vel.y          = stateStruct.velocity[1] - gpsVelGlitchOffset.y;
            vel.z          = stateStruct.velocity[2];

            // predict range to centre of image
            float flowRngPred = max((terrainState - stateStruct.position[2]),rngOnGnd) / Tnb_flow.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = Tnb_flow*vel;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred =   relVelSensor.length()/flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - sqrtf(sq(flowRadXYcomp[0]) + sq(flowRadXYcomp[1]));

            // calculate observation jacobian
            float t3 = sq(q0);
            float t4 = sq(q1);
            float t5 = sq(q2);
            float t6 = sq(q3);
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t14 = t3+t4-t5-t6;
            float t15 = t14*vel.x;
            float t16 = t10+t11;
            float t17 = t16*vel.y;
            float t18 = q0*q2*2.0f;
            float t19 = q1*q3*2.0f;
            float t20 = t18-t19;
            float t21 = t20*vel.z;
            float t2 = t15+t17-t21;
            float t7 = t3-t4-t5+t6;
            float t8 = stateStruct.position[2]-terrainState;
            float t9 = 1.0f/sq(t8);
            float t24 = t3-t4+t5-t6;
            float t25 = t24*vel.y;
            float t26 = t10-t11;
            float t27 = t26*vel.x;
            float t28 = q0*q1*2.0f;
            float t29 = q2*q3*2.0f;
            float t30 = t28+t29;
            float t31 = t30*vel.z;
            float t12 = t25-t27+t31;
            float t13 = sq(t7);
            float t22 = sq(t2);
            float t23 = 1.0f/(t8*t8*t8);
            float t32 = sq(t12);
            H_OPT = 0.5f*(t13*t22*t23*2.0f+t13*t23*t32*2.0f)/sqrtf(t9*t13*t22+t9*t13*t32);

            // calculate innovation variances
            auxFlowObsInnovVar = H_OPT*Popt*H_OPT + R_LOS;

            // calculate Kalman gain
            K_OPT = Popt*H_OPT/auxFlowObsInnovVar;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio = sq(auxFlowObsInnov) / (sq(frontend._flowInnovGate) * auxFlowObsInnovVar);

            // don't fuse if optical flow data is outside valid range
            if (max(flowRadXY[0],flowRadXY[1]) < frontend._maxFlowRate) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov;

                // constrain the state
                terrainState = max(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming negative
                Popt = max(Popt,0.0f);
            }
        }
    }

    // stop the performance timer
    perf_end(_perf_OpticalFlowEKF);
}

/*
Fuse angular motion compensated optical flow rates into the main filter.
Requires a valid terrain height estimate.
*/
void NavEKF2_core::FuseOptFlow()
{
    Vector24 H_LOS;
    Vector3f velNED_local;
    Vector3f relVelSensor;
    Vector14 SH_LOS;
    Vector2 losPred;

    // Copy required states to local variable names
    float q0  = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];
    float vn = stateStruct.velocity.x;
    float ve = stateStruct.velocity.y;
    float vd = stateStruct.velocity.z;
    float pd = stateStruct.position.z;

    // Correct velocities for GPS glitch recovery offset
    velNED_local.x = vn - gpsVelGlitchOffset.x;
    velNED_local.y = ve - gpsVelGlitchOffset.y;
    velNED_local.z = vd;

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = max((terrainState - pd), rngOnGnd);
    float ptd = pd + heightAboveGndEst;

    // Calculate common expressions for observation jacobians
    SH_LOS[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    SH_LOS[1] = vn*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + ve*(2*q0*q3 + 2*q1*q2);
    SH_LOS[2] = ve*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - vn*(2*q0*q3 - 2*q1*q2);
    SH_LOS[3] = 1/(pd - ptd);
    SH_LOS[4] = vd*SH_LOS[0] - ve*(2*q0*q1 - 2*q2*q3) + vn*(2*q0*q2 + 2*q1*q3);
    SH_LOS[5] = 2.0f*q0*q2 - 2.0f*q1*q3;
    SH_LOS[6] = 2.0f*q0*q1 + 2.0f*q2*q3;
    SH_LOS[7] = q0*q0;
    SH_LOS[8] = q1*q1;
    SH_LOS[9] = q2*q2;
    SH_LOS[10] = q3*q3;
    SH_LOS[11] = q0*q3*2.0f;
    SH_LOS[12] = pd-ptd;
    SH_LOS[13] = 1.0f/(SH_LOS[12]*SH_LOS[12]);

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first
        // calculate range from ground plain to centre of sensor fov assuming flat earth
        float range = constrain_float((heightAboveGndEst/Tnb_flow.c.z),rngOnGnd,1000.0f);

        // calculate relative velocity in sensor frame
        relVelSensor = Tnb_flow*velNED_local;

        // divide velocity by range  to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;

        // calculate observation jacobians and Kalman gains
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        if (obsIndex == 0) {
            H_LOS[0] = SH_LOS[3]*SH_LOS[2]*SH_LOS[6]-SH_LOS[3]*SH_LOS[0]*SH_LOS[4];
            H_LOS[1] = SH_LOS[3]*SH_LOS[2]*SH_LOS[5];
            H_LOS[2] = SH_LOS[3]*SH_LOS[0]*SH_LOS[1];
            H_LOS[3] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[11]-q1*q2*2.0f);
            H_LOS[4] = -SH_LOS[3]*SH_LOS[0]*(SH_LOS[7]-SH_LOS[8]+SH_LOS[9]-SH_LOS[10]);
            H_LOS[5] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[6];
            H_LOS[8] = SH_LOS[2]*SH_LOS[0]*SH_LOS[13];

            float t2 = SH_LOS[3];
            float t3 = SH_LOS[0];
            float t4 = SH_LOS[2];
            float t5 = SH_LOS[6];
            float t100 = t2 * t3 * t5;
            float t6 = SH_LOS[4];
            float t7 = t2*t3*t6;
            float t9 = t2*t4*t5;
            float t8 = t7-t9;
            float t10 = q0*q3*2.0f;
            float t21 = q1*q2*2.0f;
            float t11 = t10-t21;
            float t101 = t2 * t3 * t11;
            float t12 = pd-ptd;
            float t13 = 1.0f/(t12*t12);
            float t104 = t3 * t4 * t13;
            float t14 = SH_LOS[5];
            float t102 = t2 * t4 * t14;
            float t15 = SH_LOS[1];
            float t103 = t2 * t3 * t15;
            float t16 = q0*q0;
            float t17 = q1*q1;
            float t18 = q2*q2;
            float t19 = q3*q3;
            float t20 = t16-t17+t18-t19;
            float t105 = t2 * t3 * t20;
            float t22 = P[1][1]*t102;
            float t23 = P[3][0]*t101;
            float t24 = P[8][0]*t104;
            float t25 = P[1][0]*t102;
            float t26 = P[2][0]*t103;
            float t63 = P[0][0]*t8;
            float t64 = P[5][0]*t100;
            float t65 = P[4][0]*t105;
            float t27 = t23+t24+t25+t26-t63-t64-t65;
            float t28 = P[3][3]*t101;
            float t29 = P[8][3]*t104;
            float t30 = P[1][3]*t102;
            float t31 = P[2][3]*t103;
            float t67 = P[0][3]*t8;
            float t68 = P[5][3]*t100;
            float t69 = P[4][3]*t105;
            float t32 = t28+t29+t30+t31-t67-t68-t69;
            float t33 = t101*t32;
            float t34 = P[3][8]*t101;
            float t35 = P[8][8]*t104;
            float t36 = P[1][8]*t102;
            float t37 = P[2][8]*t103;
            float t70 = P[0][8]*t8;
            float t71 = P[5][8]*t100;
            float t72 = P[4][8]*t105;
            float t38 = t34+t35+t36+t37-t70-t71-t72;
            float t39 = t104*t38;
            float t40 = P[3][1]*t101;
            float t41 = P[8][1]*t104;
            float t42 = P[2][1]*t103;
            float t73 = P[0][1]*t8;
            float t74 = P[5][1]*t100;
            float t75 = P[4][1]*t105;
            float t43 = t22+t40+t41+t42-t73-t74-t75;
            float t44 = t102*t43;
            float t45 = P[3][2]*t101;
            float t46 = P[8][2]*t104;
            float t47 = P[1][2]*t102;
            float t48 = P[2][2]*t103;
            float t76 = P[0][2]*t8;
            float t77 = P[5][2]*t100;
            float t78 = P[4][2]*t105;
            float t49 = t45+t46+t47+t48-t76-t77-t78;
            float t50 = t103*t49;
            float t51 = P[3][5]*t101;
            float t52 = P[8][5]*t104;
            float t53 = P[1][5]*t102;
            float t54 = P[2][5]*t103;
            float t79 = P[0][5]*t8;
            float t80 = P[5][5]*t100;
            float t81 = P[4][5]*t105;
            float t55 = t51+t52+t53+t54-t79-t80-t81;
            float t56 = P[3][4]*t101;
            float t57 = P[8][4]*t104;
            float t58 = P[1][4]*t102;
            float t59 = P[2][4]*t103;
            float t83 = P[0][4]*t8;
            float t84 = P[5][4]*t100;
            float t85 = P[4][4]*t105;
            float t60 = t56+t57+t58+t59-t83-t84-t85;
            float t66 = t8*t27;
            float t82 = t100*t55;
            float t86 = t105*t60;
            float t61 = R_LOS+t33+t39+t44+t50-t66-t82-t86;
            float t62 = 1.0f/t61;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t61 > R_LOS) {
                t62 = 1.0f/t61;
            } else {
                t61 = 0.0f;
                t62 = 1.0f/R_LOS;
            }
            varInnovOptFlow[0] = t61;

            // calculate innovation for X axis observation
            innovOptFlow[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t62*(-P[0][0]*t8-P[0][5]*t100+P[0][3]*t101+P[0][1]*t102+P[0][2]*t103+P[0][8]*t104-P[0][4]*t105);
            Kfusion[1] = t62*(t22-P[1][0]*t8-P[1][5]*t100+P[1][3]*t101+P[1][2]*t103+P[1][8]*t104-P[1][4]*t105);
            Kfusion[2] = t62*(t48-P[2][0]*t8-P[2][5]*t100+P[2][3]*t101+P[2][1]*t102+P[2][8]*t104-P[2][4]*t105);
            Kfusion[3] = t62*(t28-P[3][0]*t8-P[3][5]*t100+P[3][1]*t102+P[3][2]*t103+P[3][8]*t104-P[3][4]*t105);
            Kfusion[4] = t62*(-t85-P[4][0]*t8-P[4][5]*t100+P[4][3]*t101+P[4][1]*t102+P[4][2]*t103+P[4][8]*t104);
            Kfusion[5] = t62*(-t80-P[5][0]*t8+P[5][3]*t101+P[5][1]*t102+P[5][2]*t103+P[5][8]*t104-P[5][4]*t105);
            Kfusion[6] = t62*(-P[6][0]*t8-P[6][5]*t100+P[6][3]*t101+P[6][1]*t102+P[6][2]*t103+P[6][8]*t104-P[6][4]*t105);
            Kfusion[7] = t62*(-P[7][0]*t8-P[7][5]*t100+P[7][3]*t101+P[7][1]*t102+P[7][2]*t103+P[7][8]*t104-P[7][4]*t105);
            Kfusion[8] = t62*(t35-P[8][0]*t8-P[8][5]*t100+P[8][3]*t101+P[8][1]*t102+P[8][2]*t103-P[8][4]*t105);
            Kfusion[9] = t62*(-P[9][0]*t8-P[9][5]*t100+P[9][3]*t101+P[9][1]*t102+P[9][2]*t103+P[9][8]*t104-P[9][4]*t105);
            Kfusion[10] = t62*(-P[10][0]*t8-P[10][5]*t100+P[10][3]*t101+P[10][1]*t102+P[10][2]*t103+P[10][8]*t104-P[10][4]*t105);
            Kfusion[11] = t62*(-P[11][0]*t8-P[11][5]*t100+P[11][3]*t101+P[11][1]*t102+P[11][2]*t103+P[11][8]*t104-P[11][4]*t105);
            Kfusion[12] = t62*(-P[12][0]*t8-P[12][5]*t100+P[12][3]*t101+P[12][1]*t102+P[12][2]*t103+P[12][8]*t104-P[12][4]*t105);
            Kfusion[13] = t62*(-P[13][0]*t8-P[13][5]*t100+P[13][3]*t101+P[13][1]*t102+P[13][2]*t103+P[13][8]*t104-P[13][4]*t105);
            Kfusion[14] = t62*(-P[14][0]*t8-P[14][5]*t100+P[14][3]*t101+P[14][1]*t102+P[14][2]*t103+P[14][8]*t104-P[14][4]*t105);
            Kfusion[15] = t62*(-P[15][0]*t8-P[15][5]*t100+P[15][3]*t101+P[15][1]*t102+P[15][2]*t103+P[15][8]*t104-P[15][4]*t105);
            if (!inhibitWindStates) {
                Kfusion[22] = t62*(-P[22][0]*t8-P[22][5]*t100+P[22][3]*t101+P[22][1]*t102+P[22][2]*t103+P[22][8]*t104-P[22][4]*t105);
                Kfusion[23] = t62*(-P[23][0]*t8-P[23][5]*t100+P[23][3]*t101+P[23][1]*t102+P[23][2]*t103+P[23][8]*t104-P[23][4]*t105);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = t62*(-P[16][0]*t8-P[16][5]*t100+P[16][3]*t101+P[16][1]*t102+P[16][2]*t103+P[16][8]*t104-P[16][4]*t105);
                Kfusion[17] = t62*(-P[17][0]*t8-P[17][5]*t100+P[17][3]*t101+P[17][1]*t102+P[17][2]*t103+P[17][8]*t104-P[17][4]*t105);
                Kfusion[18] = t62*(-P[18][0]*t8-P[18][5]*t100+P[18][3]*t101+P[18][1]*t102+P[18][2]*t103+P[18][8]*t104-P[18][4]*t105);
                Kfusion[19] = t62*(-P[19][0]*t8-P[19][5]*t100+P[19][3]*t101+P[19][1]*t102+P[19][2]*t103+P[19][8]*t104-P[19][4]*t105);
                Kfusion[20] = t62*(-P[20][0]*t8-P[20][5]*t100+P[20][3]*t101+P[20][1]*t102+P[20][2]*t103+P[20][8]*t104-P[20][4]*t105);
                Kfusion[21] = t62*(-P[21][0]*t8-P[21][5]*t100+P[21][3]*t101+P[21][1]*t102+P[21][2]*t103+P[21][8]*t104-P[21][4]*t105);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

        } else {

            H_LOS[0] = -SH_LOS[3]*SH_LOS[6]*SH_LOS[1];
            H_LOS[1] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[4]-SH_LOS[3]*SH_LOS[1]*SH_LOS[5];
            H_LOS[2] = SH_LOS[3]*SH_LOS[2]*SH_LOS[0];
            H_LOS[3] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[7]+SH_LOS[8]-SH_LOS[9]-SH_LOS[10]);
            H_LOS[4] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[11]+q1*q2*2.0f);
            H_LOS[5] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[5];
            H_LOS[8] = -SH_LOS[0]*SH_LOS[1]*SH_LOS[13];

            float t2 = SH_LOS[3];
            float t3 = SH_LOS[0];
            float t4 = SH_LOS[1];
            float t5 = SH_LOS[5];
            float t100 = t2 * t3 * t5;
            float t6 = SH_LOS[4];
            float t7 = t2*t3*t6;
            float t8 = t2*t4*t5;
            float t9 = t7+t8;
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t12 = t10+t11;
            float t101 = t2 * t3 * t12;
            float t13 = pd-ptd;
            float t14 = 1.0f/(t13*t13);
            float t104 = t3 * t4 * t14;
            float t15 = SH_LOS[6];
            float t105 = t2 * t4 * t15;
            float t16 = SH_LOS[2];
            float t102 = t2 * t3 * t16;
            float t17 = q0*q0;
            float t18 = q1*q1;
            float t19 = q2*q2;
            float t20 = q3*q3;
            float t21 = t17+t18-t19-t20;
            float t103 = t2 * t3 * t21;
            float t22 = P[0][0]*t105;
            float t23 = P[1][1]*t9;
            float t24 = P[8][1]*t104;
            float t25 = P[0][1]*t105;
            float t26 = P[5][1]*t100;
            float t64 = P[4][1]*t101;
            float t65 = P[2][1]*t102;
            float t66 = P[3][1]*t103;
            float t27 = t23+t24+t25+t26-t64-t65-t66;
            float t28 = t9*t27;
            float t29 = P[1][4]*t9;
            float t30 = P[8][4]*t104;
            float t31 = P[0][4]*t105;
            float t32 = P[5][4]*t100;
            float t67 = P[4][4]*t101;
            float t68 = P[2][4]*t102;
            float t69 = P[3][4]*t103;
            float t33 = t29+t30+t31+t32-t67-t68-t69;
            float t34 = P[1][8]*t9;
            float t35 = P[8][8]*t104;
            float t36 = P[0][8]*t105;
            float t37 = P[5][8]*t100;
            float t71 = P[4][8]*t101;
            float t72 = P[2][8]*t102;
            float t73 = P[3][8]*t103;
            float t38 = t34+t35+t36+t37-t71-t72-t73;
            float t39 = t104*t38;
            float t40 = P[1][0]*t9;
            float t41 = P[8][0]*t104;
            float t42 = P[5][0]*t100;
            float t74 = P[4][0]*t101;
            float t75 = P[2][0]*t102;
            float t76 = P[3][0]*t103;
            float t43 = t22+t40+t41+t42-t74-t75-t76;
            float t44 = t105*t43;
            float t45 = P[1][2]*t9;
            float t46 = P[8][2]*t104;
            float t47 = P[0][2]*t105;
            float t48 = P[5][2]*t100;
            float t63 = P[2][2]*t102;
            float t77 = P[4][2]*t101;
            float t78 = P[3][2]*t103;
            float t49 = t45+t46+t47+t48-t63-t77-t78;
            float t50 = P[1][5]*t9;
            float t51 = P[8][5]*t104;
            float t52 = P[0][5]*t105;
            float t53 = P[5][5]*t100;
            float t80 = P[4][5]*t101;
            float t81 = P[2][5]*t102;
            float t82 = P[3][5]*t103;
            float t54 = t50+t51+t52+t53-t80-t81-t82;
            float t55 = t100*t54;
            float t56 = P[1][3]*t9;
            float t57 = P[8][3]*t104;
            float t58 = P[0][3]*t105;
            float t59 = P[5][3]*t100;
            float t83 = P[4][3]*t101;
            float t84 = P[2][3]*t102;
            float t85 = P[3][3]*t103;
            float t60 = t56+t57+t58+t59-t83-t84-t85;
            float t70 = t101*t33;
            float t79 = t102*t49;
            float t86 = t103*t60;
            float t61 = R_LOS+t28+t39+t44+t55-t70-t79-t86;
            float t62 = 1.0f/t61;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t61 > R_LOS) {
                t62 = 1.0f/t61;
            } else {
                t61 = 0.0f;
                t62 = 1.0f/R_LOS;
            }
            varInnovOptFlow[1] = t61;

            // calculate innovation for Y observation
            innovOptFlow[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t62*(t22+P[0][1]*t9+P[0][5]*t100-P[0][4]*t101-P[0][2]*t102-P[0][3]*t103+P[0][8]*t104);
            Kfusion[1] = -t62*(t23+P[1][5]*t100+P[1][0]*t105-P[1][4]*t101-P[1][2]*t102-P[1][3]*t103+P[1][8]*t104);
            Kfusion[2] = -t62*(-t63+P[2][1]*t9+P[2][5]*t100+P[2][0]*t105-P[2][4]*t101-P[2][3]*t103+P[2][8]*t104);
            Kfusion[3] = -t62*(-t85+P[3][1]*t9+P[3][5]*t100+P[3][0]*t105-P[3][4]*t101-P[3][2]*t102+P[3][8]*t104);
            Kfusion[4] = -t62*(-t67+P[4][1]*t9+P[4][5]*t100+P[4][0]*t105-P[4][2]*t102-P[4][3]*t103+P[4][8]*t104);
            Kfusion[5] = -t62*(t53+P[5][1]*t9+P[5][0]*t105-P[5][4]*t101-P[5][2]*t102-P[5][3]*t103+P[5][8]*t104);
            Kfusion[6] = -t62*(P[6][1]*t9+P[6][5]*t100+P[6][0]*t105-P[6][4]*t101-P[6][2]*t102-P[6][3]*t103+P[6][8]*t104);
            Kfusion[7] = -t62*(P[7][1]*t9+P[7][5]*t100+P[7][0]*t105-P[7][4]*t101-P[7][2]*t102-P[7][3]*t103+P[7][8]*t104);
            Kfusion[8] = -t62*(t35+P[8][1]*t9+P[8][5]*t100+P[8][0]*t105-P[8][4]*t101-P[8][2]*t102-P[8][3]*t103);
            Kfusion[9] = -t62*(P[9][1]*t9+P[9][5]*t100+P[9][0]*t105-P[9][4]*t101-P[9][2]*t102-P[9][3]*t103+P[9][8]*t104);
            Kfusion[10] = -t62*(P[10][1]*t9+P[10][5]*t100+P[10][0]*t105-P[10][4]*t101-P[10][2]*t102-P[10][3]*t103+P[10][8]*t104);
            Kfusion[11] = -t62*(P[11][1]*t9+P[11][5]*t100+P[11][0]*t105-P[11][4]*t101-P[11][2]*t102-P[11][3]*t103+P[11][8]*t104);
            Kfusion[12] = -t62*(P[12][1]*t9+P[12][5]*t100+P[12][0]*t105-P[12][4]*t101-P[12][2]*t102-P[12][3]*t103+P[12][8]*t104);
            Kfusion[13] = -t62*(P[13][1]*t9+P[13][5]*t100+P[13][0]*t105-P[13][4]*t101-P[13][2]*t102-P[13][3]*t103+P[13][8]*t104);
            Kfusion[14] = -t62*(P[14][1]*t9+P[14][5]*t100+P[14][0]*t105-P[14][4]*t101-P[14][2]*t102-P[14][3]*t103+P[14][8]*t104);
            Kfusion[15] = -t62*(P[15][1]*t9+P[15][5]*t100+P[15][0]*t105-P[15][4]*t101-P[15][2]*t102-P[15][3]*t103+P[15][8]*t104);
            if (!inhibitWindStates) {
                Kfusion[22] = -t62*(P[22][1]*t9+P[22][5]*t100+P[22][0]*t105-P[22][4]*t101-P[22][2]*t102-P[22][3]*t103+P[22][8]*t104);
                Kfusion[23] = -t62*(P[23][1]*t9+P[23][5]*t100+P[23][0]*t105-P[23][4]*t101-P[23][2]*t102-P[23][3]*t103+P[23][8]*t104);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = -t62*(P[16][1]*t9+P[16][5]*t100+P[16][0]*t105-P[16][4]*t101-P[16][2]*t102-P[16][3]*t103+P[16][8]*t104);
                Kfusion[17] = -t62*(P[17][1]*t9+P[17][5]*t100+P[17][0]*t105-P[17][4]*t101-P[17][2]*t102-P[17][3]*t103+P[17][8]*t104);
                Kfusion[18] = -t62*(P[18][1]*t9+P[18][5]*t100+P[18][0]*t105-P[18][4]*t101-P[18][2]*t102-P[18][3]*t103+P[18][8]*t104);
                Kfusion[19] = -t62*(P[19][1]*t9+P[19][5]*t100+P[19][0]*t105-P[19][4]*t101-P[19][2]*t102-P[19][3]*t103+P[19][8]*t104);
                Kfusion[20] = -t62*(P[20][1]*t9+P[20][5]*t100+P[20][0]*t105-P[20][4]*t101-P[20][2]*t102-P[20][3]*t103+P[20][8]*t104);
                Kfusion[21] = -t62*(P[21][1]*t9+P[21][5]*t100+P[21][0]*t105-P[21][4]*t101-P[21][2]*t102-P[21][3]*t103+P[21][8]*t104);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }
        }

        // calculate the innovation consistency test ratio
        flowTestRatio[obsIndex] = sq(innovOptFlow[obsIndex]) / (sq(frontend._flowInnovGate) * varInnovOptFlow[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
        if ((flowTestRatio[obsIndex]) < 1.0f && (ofDataDelayed.flowRadXY.x < frontend._maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend._maxFlowRate)) {
            // record the last time observations were accepted for fusion
            prevFlowFuseTime_ms = imuSampleTime_ms;

            // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
            stateStruct.angErr.zero();

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovOptFlow[obsIndex];
            }

            // the first 3 states represent the angular misalignment vector. This is
            // is used to correct the estimated quaternion on the current time step
            stateStruct.quat.rotate(stateStruct.angErr);

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=stateIndexLim; i++) {
                for (uint8_t j = 0; j<=5; j++) {
                    KH[i][j] = Kfusion[i] * H_LOS[j];
                }
                for (uint8_t j = 6; j<=7; j++) {
                    KH[i][j] = 0.0f;
                }
                KH[i][8] = Kfusion[i] * H_LOS[8];
                for (uint8_t j = 9; j<=23; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (uint8_t i = 0; i<=stateIndexLim; i++) {
                for (uint8_t j = 0; j<=stateIndexLim; j++) {
                    KHP[i][j] = 0;
                    for (uint8_t k = 0; k<=5; k++) {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    KHP[i][j] = KHP[i][j] + KH[i][8] * P[8][j];
                }
            }
            for (uint8_t i = 0; i<=stateIndexLim; i++) {
                for (uint8_t j = 0; j<=stateIndexLim; j++) {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }

        // fix basic numerical errors
        ForceSymmetry();
        ConstrainVariances();

    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// decay GPS horizontal position offset to close to zero at a rate of 1 m/s for copters and 5 m/s for planes
// limit radius to a maximum of 50m
void NavEKF2_core::decayGpsOffset()
{
    float offsetDecaySpd;
    if (assume_zero_sideslip()) {
        offsetDecaySpd = 5.0f;
    } else {
        offsetDecaySpd = 1.0f;
    }
    float lapsedTime = 0.001f*float(imuSampleTime_ms - lastDecayTime_ms);
    lastDecayTime_ms = imuSampleTime_ms;
    float offsetRadius = pythagorous2(gpsPosGlitchOffsetNE.x, gpsPosGlitchOffsetNE.y);
    // decay radius if larger than offset decay speed multiplied by lapsed time (plus a margin to prevent divide by zero)
    if (offsetRadius > (offsetDecaySpd * lapsedTime + 0.1f)) {
        // Calculate the GPS velocity offset required. This is necessary to prevent the position measurement being rejected for inconsistency when the radius is being pulled back in.
        gpsVelGlitchOffset = -gpsPosGlitchOffsetNE*offsetDecaySpd/offsetRadius;
        // calculate scale factor to be applied to both offset components
        float scaleFactor = constrain_float((offsetRadius - offsetDecaySpd * lapsedTime), 0.0f, 50.0f) / offsetRadius;
        gpsPosGlitchOffsetNE.x *= scaleFactor;
        gpsPosGlitchOffsetNE.y *= scaleFactor;
    } else {
        gpsVelGlitchOffset.zero();
        gpsPosGlitchOffsetNE.zero();
    }
}


#endif // HAL_CPU_CLASS