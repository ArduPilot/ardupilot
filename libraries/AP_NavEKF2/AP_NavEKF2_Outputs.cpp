/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


// Check basic filter health metrics and return a consolidated health status
bool NavEKF2_core::healthy(void) const
{
    uint16_t faultInt;
    getFilterFaults(faultInt);
    if (faultInt > 0) {
        return false;
    }
    if (velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1) {
        // all three metrics being above 1 means the filter is
        // extremely unhealthy.
        return false;
    }
    // Give the filter a second to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 1000 ) {
        return false;
    }
    // position and height innovations must be within limits when on-ground and in a static mode of operation
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (onGround && (PV_AidingMode == AID_NONE) && ((horizErrSq > 1.0f) || (fabsf(hgtInnovFiltState) > 1.0f))) {
        return false;
    }

    // all OK
    return true;
}

// Return a consolidated fault score where higher numbers are less healthy
// Intended to be used by the front-end to determine which is the primary EKF
float NavEKF2_core::faultScore(void) const
{
    float score = 0.0f;
    // If velocity, position or height measurements are failing consistency checks, this adds to the score
    if (velTestRatio > 1.0f) {
        score += velTestRatio-1.0f;
    }
    if (posTestRatio > 1.0f) {
        score += posTestRatio-1.0f;
    }
    if (hgtTestRatio > 1.0f) {
        score += hgtTestRatio-1.0f;
    }
    // If the tilt error is excessive this adds to the score
    const float tiltErrThreshold = 0.05f;
    if (tiltAlignComplete && yawAlignComplete && tiltErrFilt > tiltErrThreshold) {
        score += tiltErrFilt / tiltErrThreshold;
    }
    return score;
}

// return data for debugging optical flow fusion
void NavEKF2_core::getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const
{
    varFlow = MAX(flowTestRatio[0],flowTestRatio[1]);
    gndOffset = terrainState;
    flowInnovX = innovOptFlow[0];
    flowInnovY = innovOptFlow[1];
    auxInnov = auxFlowObsInnov;
    HAGL = terrainState - stateStruct.position.z;
    rngInnov = innovRng;
    range = rangeDataDelayed.rng;
    gndOffsetErr = sqrtf(Popt); // note Popt is constrained to be non-negative in EstimateTerrainOffset()
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
// this is needed to ensure the vehicle does not fly too high when using optical flow navigation
bool NavEKF2_core::getHeightControlLimit(float &height) const
{
    // only ask for limiting if we are doing optical flow navigation
    if (frontend->_fusionModeGPS == 3) {
        // If are doing optical flow nav, ensure the height above ground is within range finder limits after accounting for vehicle tilt and control errors
        height = MAX(float(frontend->_rng.max_distance_cm()) * 0.007f - 1.0f, 1.0f);
        // If we are are not using the range finder as the height reference, then compensate for the difference between terrain and EKF origin
        if (frontend->_altSource != 1) {
            height -= terrainState;
        }
        return true;
    } else {
        return false;
    }
}


// return the Euler roll, pitch and yaw angle in radians
void NavEKF2_core::getEulerAngles(Vector3f &euler) const
{
    outputDataNew.quat.to_euler(euler.x, euler.y, euler.z);
    euler = euler - _ahrs->get_trim();
}

// return body axis gyro bias estimates in rad/sec
void NavEKF2_core::getGyroBias(Vector3f &gyroBias) const
{
    if (dtEkfAvg < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = stateStruct.gyro_bias / dtEkfAvg;
}

// return body axis gyro scale factor error as a percentage
void NavEKF2_core::getGyroScaleErrorPercentage(Vector3f &gyroScale) const
{
    if (!statesInitialised) {
        gyroScale.x = gyroScale.y = gyroScale.z = 0;
        return;
    }
    gyroScale.x = 100.0f/stateStruct.gyro_scale.x - 100.0f;
    gyroScale.y = 100.0f/stateStruct.gyro_scale.y - 100.0f;
    gyroScale.z = 100.0f/stateStruct.gyro_scale.z - 100.0f;
}

// return tilt error convergence metric
void NavEKF2_core::getTiltError(float &ang) const
{
    ang = tiltErrFilt;
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
uint32_t NavEKF2_core::getLastYawResetAngle(float &yawAng) const
{
    yawAng = yawResetAngle;
    return lastYawReset_ms;
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF2_core::getLastPosNorthEastReset(Vector2f &pos) const
{
    pos = posResetNE;
    return lastPosReset_ms;
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF2_core::getLastVelNorthEastReset(Vector2f &vel) const
{
    vel = velResetNE;
    return lastVelReset_ms;
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
void NavEKF2_core::getWind(Vector3f &wind) const
{
    wind.x = stateStruct.wind_vel.x;
    wind.y = stateStruct.wind_vel.y;
    wind.z = 0.0f; // currently don't estimate this
}


// return NED velocity in m/s
//
void NavEKF2_core::getVelNED(Vector3f &vel) const
{
    vel = outputDataNew.velocity;
}

// Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
float NavEKF2_core::getPosDownDerivative(void) const
{
    // return the value calculated from a complmentary filer applied to the EKF height and vertical acceleration
    return posDownDerivative;
}

// This returns the specific forces in the NED frame
void NavEKF2_core::getAccelNED(Vector3f &accelNED) const {
    accelNED = velDotNED;
    accelNED.z -= GRAVITY_MSS;
}

// return the Z-accel bias estimate in m/s^2
void NavEKF2_core::getAccelZBias(float &zbias) const {
    if (dtEkfAvg > 0) {
        zbias = stateStruct.accel_zbias / dtEkfAvg;
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
    if (PV_AidingMode != AID_NONE) {
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


// return the estimated height above ground level
bool NavEKF2_core::getHAGL(float &HAGL) const
{
    HAGL = terrainState - outputDataNew.position.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && healthy();
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
            // we could be in constant position mode  because the vehicle has taken off without GPS, or has lost GPS
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


// return the horizontal speed limit in m/s set by optical flow sensor limits
// return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
void NavEKF2_core::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    if (PV_AidingMode == AID_RELATIVE) {
        // allow 1.0 rad/sec margin for angular motion
        ekfGndSpdLimit = MAX((frontend->_maxFlowRate - 1.0f), 0.0f) * MAX((terrainState - stateStruct.position[2]), rngOnGnd);
        // use standard gains up to 5.0 metres height and reduce above that
        ekfNavVelGainScaler = 4.0f / MAX((terrainState - stateStruct.position[2]),4.0f);
    } else {
        ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
        ekfNavVelGainScaler = 1.0f;
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

// return earth magnetic field estimates in measurement units / 1000
void NavEKF2_core::getMagNED(Vector3f &magNED) const
{
    magNED = stateStruct.earth_magfield * 1000.0f;
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF2_core::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ = stateStruct.body_magfield*1000.0f;
}

// return magnetometer offsets
// return true if offsets are valid
bool NavEKF2_core::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    // compass offsets are valid if we have finalised magnetic field initialisation and magnetic field learning is not prohibited and primary compass is valid
    if (mag_idx == magSelectIndex && firstInflightMagInit && (effective_magCal() != 2) && _ahrs->get_compass()->healthy(magSelectIndex)) {
        magOffsets = _ahrs->get_compass()->get_offsets(magSelectIndex) - stateStruct.body_magfield*1000.0f;
        return true;
    } else {
        magOffsets = _ahrs->get_compass()->get_offsets(magSelectIndex);
        return false;
    }
}

// return the index for the active magnetometer
uint8_t NavEKF2_core::getActiveMag() const
{
    return (uint8_t)magSelectIndex;
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
void  NavEKF2_core::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    velInnov.x = innovVelPos[0];
    velInnov.y = innovVelPos[1];
    velInnov.z = innovVelPos[2];
    posInnov.x = innovVelPos[3];
    posInnov.y = innovVelPos[4];
    posInnov.z = innovVelPos[5];
    magInnov.x = 1e3f*innovMag[0]; // Convert back to sensor units
    magInnov.y = 1e3f*innovMag[1]; // Convert back to sensor units
    magInnov.z = 1e3f*innovMag[2]; // Convert back to sensor units
    tasInnov   = innovVtas;
    yawInnov   = innovYaw;
}

// return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
// this indicates the amount of margin available when tuning the various error traps
// also return the delta in position due to the last position reset
void  NavEKF2_core::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    velVar   = sqrtf(velTestRatio);
    posVar   = sqrtf(posTestRatio);
    hgtVar   = sqrtf(hgtTestRatio);
    // If we are using simple compass yaw fusion, populate all three components with the yaw test ratio to provide an equivalent output
    magVar.x = sqrtf(MAX(magTestRatio.x,yawTestRatio));
    magVar.y = sqrtf(MAX(magTestRatio.y,yawTestRatio));
    magVar.z = sqrtf(MAX(magTestRatio.z,yawTestRatio));
    tasVar   = sqrtf(tasTestRatio);
    offset   = posResetNE;
}


/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 5 = badly conditioned Z magnetometer fusion
 6 = badly conditioned airspeed fusion
 7 = badly conditioned synthetic sideslip fusion
 7 = filter is not initialised
*/
void  NavEKF2_core::getFilterFaults(uint16_t &faults) const
{
    faults = (stateStruct.quat.is_nan()<<0 |
              stateStruct.velocity.is_nan()<<1 |
              faultStatus.bad_xmag<<2 |
              faultStatus.bad_ymag<<3 |
              faultStatus.bad_zmag<<4 |
              faultStatus.bad_airspeed<<5 |
              faultStatus.bad_sideslip<<6 |
              !statesInitialised<<7);
}

/*
return filter timeout status as a bitmasked integer
 0 = position measurement timeout
 1 = velocity measurement timeout
 2 = height measurement timeout
 3 = magnetometer measurement timeout
 4 = true airspeed measurement timeout
 5 = unassigned
 6 = unassigned
 7 = unassigned
*/
void  NavEKF2_core::getFilterTimeouts(uint8_t &timeouts) const
{
    timeouts = (posTimeout<<0 |
                velTimeout<<1 |
                hgtTimeout<<2 |
                magTimeout<<3 |
                tasTimeout<<4);
}

/*
Return a filter function status that indicates:
    Which outputs are valid
    If the filter has detected takeoff
    If the filter has activated the mode that mitigates against ground effect static pressure errors
    If GPS data is being used
*/
void  NavEKF2_core::getFilterStatus(nav_filter_status &status) const
{
    // init return value
    status.value = 0;

    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && (frontend->_fusionModeGPS == 3);
    bool gpsNavPossible = !gpsNotAvailable && gpsGoodToAlign;
    bool filterHealthy = healthy() && tiltAlignComplete && (yawAlignComplete || (!use_compass() && (PV_AidingMode == AID_NONE)));
    // If GPS height usage is specified, height is considered to be inaccurate until the GPS passes all checks
    bool hgtNotAccurate = (frontend->_altSource == 2) && !validOrigin;

    // set individual flags
    status.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && filterHealthy;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    status.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && filterHealthy;   // relative horizontal position estimate valid
    status.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout && filterHealthy && !hgtNotAccurate; // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;     // constant position mode
    status.flags.pred_horiz_pos_rel = (optFlowNavPossible || gpsNavPossible) && filterHealthy; // we should be able to estimate a relative position when we enter flight mode
    status.flags.pred_horiz_pos_abs = gpsNavPossible && filterHealthy; // we should be able to estimate an absolute position when we enter flight mode
    status.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    status.flags.takeoff = expectGndEffectTakeoff; // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    status.flags.touchdown = expectGndEffectTouchdown; // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    status.flags.using_gps = ((imuSampleTime_ms - lastPosPassTime_ms) < 4000) && (PV_AidingMode == AID_ABSOLUTE);
    status.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode == AID_ABSOLUTE); // The GPS is glitching
}

/*
return filter gps quality check status
*/
void  NavEKF2_core::getFilterGpsStatus(nav_gps_status &faults) const
{
    // init return value
    faults.value = 0;

    // set individual flags
    faults.flags.bad_sAcc           = gpsCheckStatus.bad_sAcc; // reported speed accuracy is insufficient
    faults.flags.bad_hAcc           = gpsCheckStatus.bad_hAcc; // reported horizontal position accuracy is insufficient
    faults.flags.bad_yaw            = gpsCheckStatus.bad_yaw; // EKF heading accuracy is too large for GPS use
    faults.flags.bad_sats           = gpsCheckStatus.bad_sats; // reported number of satellites is insufficient
    faults.flags.bad_horiz_drift    = gpsCheckStatus.bad_horiz_drift; // GPS horizontal drift is too large to start using GPS (check assumes vehicle is static)
    faults.flags.bad_hdop           = gpsCheckStatus.bad_hdop; // reported HDoP is too large to start using GPS
    faults.flags.bad_vert_vel       = gpsCheckStatus.bad_vert_vel; // GPS vertical speed is too large to start using GPS (check assumes vehicle is static)
    faults.flags.bad_fix            = gpsCheckStatus.bad_fix; // The GPS cannot provide the 3D fix required
    faults.flags.bad_horiz_vel      = gpsCheckStatus.bad_horiz_vel; // The GPS horizontal speed is excessive (check assumes the vehicle is static)
}

// send an EKF_STATUS message to GCS
void NavEKF2_core::send_status_report(mavlink_channel_t chan)
{
    // get filter status
    nav_filter_status filt_state;
    getFilterStatus(filt_state);

    // prepare flags
    uint16_t flags = 0;
    if (filt_state.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filt_state.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filt_state.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filt_state.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filt_state.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filt_state.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filt_state.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filt_state.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filt_state.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filt_state.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }

    // get variances
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);

    // send message
    mavlink_msg_ekf_status_report_send(chan, flags, velVar, posVar, hgtVar, magVar.length(), sqrtf(auxRngTestRatio));

}

// report the reason for why the backend is refusing to initialise
const char *NavEKF2_core::prearm_failure_reason(void) const
{
    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        // we are not failing
        return nullptr;
    }
    return prearm_fail_string;
}


// report the number of frames lapsed since the last state prediction
// this is used by other instances to level load
uint8_t NavEKF2_core::getFramesSincePredict(void) const
{
    return framesSincePredict;
}

#endif // HAL_CPU_CLASS
