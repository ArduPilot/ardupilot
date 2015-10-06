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

// Check basic filter health metrics and return a consolidated health status
bool NavEKF2_core::healthy(void) const
{
    uint8_t faultInt;
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
    // barometer and position innovations must be within limits when on-ground
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (onGround && (fabsf(innovVelPos[5]) > 1.0f || horizErrSq > 1.0f)) {
        return false;
    }

    // all OK
    return true;
}

// Control filter mode transitions
void NavEKF2_core::controlFilterModes()
{
    // Determine motor arm status
    prevMotorsArmed = motorsArmed;
    motorsArmed = hal.util->get_soft_armed();
    if (motorsArmed && !prevMotorsArmed) {
        // set the time at which we arm to assist with checks
        timeAtArming_ms =  imuSampleTime_ms;
    }

    // Detect if we are in flight on or ground
    detectFlight();

    // Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
    // avoid unnecessary operations
    setWindMagStateLearningMode();

    // Check the alignmnent status of the tilt and yaw attitude
    // Used during initial bootstrap alignment of the filter
    checkAttitudeAlignmentStatus();

    // Control reset of yaw and magnetic field states
    controlMagYawReset();

    // Set the type of inertial navigation aiding used
    setAidingMode();

}

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
// avoid unnecessary operations
void NavEKF2_core::setWindMagStateLearningMode()
{
    // If we are on ground, or in constant position mode, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    inhibitWindStates = ((!useAirspeed() && !assume_zero_sideslip()) || onGround || (PV_AidingMode == AID_NONE));

    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f) {
        manoeuvring = true;
    } else {
        manoeuvring = false;
    }

    // Determine if learning of magnetic field states has been requested by the user
    bool magCalRequested = ((frontend._magCal == 0) && !onGround) || ((frontend._magCal == 1) && manoeuvring)  || (frontend._magCal == 3);

    // Deny mag calibration request if we aren't using the compass, are in the pre-arm constant position mode or it has been inhibited by the user
    bool magCalDenied = !use_compass() || (PV_AidingMode == AID_NONE) || (frontend._magCal == 2);

    // Inhibit the magnetic field calibration if not requested or denied
    inhibitMagStates = (!magCalRequested || magCalDenied);

    // Adjust the indexing limits used to address the covariance, states and other EKF arrays to avoid unnecessary operations
    // if we are not using those states
    if (inhibitMagStates && inhibitWindStates) {
        stateIndexLim = 15;
    } else if (inhibitWindStates) {
        stateIndexLim = 21;
    } else {
        stateIndexLim = 23;
    }
}

// Set inertial navigation aiding mode
void NavEKF2_core::setAidingMode()
{
    // Determine when to commence aiding for inertial navigation
    // Save the previous status so we can detect when it has changed
    prevIsAiding = isAiding;
    // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
    bool filterIsStable = tiltAlignComplete && yawAlignComplete;
    // If GPS useage has been prohiited then we use flow aiding provided optical flow data is present
    bool useFlowAiding = (frontend._fusionModeGPS == 3) && optFlowDataPresent();
    // Start aiding if we have a source of aiding data and the filter attitude algnment is complete
    // Latch to on. Aiding can be turned off by setting both
    isAiding = ((readyToUseGPS() || useFlowAiding) && filterIsStable) || isAiding;

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (isAiding != prevIsAiding) {
        // We have transitioned either into or out of aiding
        // zero stored velocities used to do dead-reckoning
        heldVelNE.zero();
        // reset the flag that indicates takeoff for use by optical flow navigation
        takeOffDetected = false;
        // set various  useage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        if (!isAiding) {
            // We have ceased aiding
            // When not aiding, estimate orientation & height fusing synthetic constant position and zero velocity measurement to constrain tilt errors
            PV_AidingMode = AID_NONE;
            posTimeout = true;
            velTimeout = true;
             // store the current position to be used to keep reporting the last known position
            lastKnownPositionNE.x = stateStruct.position.x;
            lastKnownPositionNE.y = stateStruct.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the baro noise filter to settle before the filtered baro data can be used
            meaHgtAtTakeOff = baroDataDelayed.hgt;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            stateStruct.position.z = -meaHgtAtTakeOff;
        } else if (frontend._fusionModeGPS == 3) {
            // We have commenced aiding, but GPS useage has been prohibited so use optical flow only
            hal.console->printf("EKF is using optical flow\n");
            PV_AidingMode = AID_RELATIVE; // we have optical flow data and can estimate all vehicle states
            posTimeout = true;
            velTimeout = true;
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
        } else {
            // We have commenced aiding and GPS useage is allowed
            hal.console->printf("EKF is using GPS\n");
            PV_AidingMode = AID_ABSOLUTE; // we have GPS data and can estimate all vehicle states
            posTimeout = false;
            velTimeout = false;
            // we need to reset the GPS timers to prevent GPS timeout logic being invoked on entry into GPS aiding
            // this is becasue the EKF can be interrupted for an arbitrary amount of time during vehicle arming checks
            lastTimeGpsReceived_ms = imuSampleTime_ms;
            secondLastGpsTime_ms = imuSampleTime_ms;
            // reset the last valid position fix time to prevent unwanted activation of GPS glitch logic
            lastPosPassTime_ms = imuSampleTime_ms;
        }
        // Reset all position, velocity and covariance
        ResetVelocity();
        ResetPosition();
        CovarianceInit();

    }

    // Always turn aiding off when the vehicle is disarmed
    if (!isAiding) {
        PV_AidingMode = AID_NONE;
        posTimeout = true;
        velTimeout = true;
    }
}

// Check the alignmnent status of the tilt and yaw attitude
// Used during initial bootstrap alignment of the filter
void NavEKF2_core::checkAttitudeAlignmentStatus()
{
    // Check for tilt convergence - used during initial alignment
    float alpha = 1.0f*dtIMUavg;
    float temp=tiltErrVec.length();
    tiltErrFilt = alpha*temp + (1.0f-alpha)*tiltErrFilt;
    if (tiltErrFilt < 0.005f && !tiltAlignComplete) {
        tiltAlignComplete = true;
        hal.console->printf("EKF tilt alignment complete\n");
    }

    // Once tilt has converged, align yaw using magnetic field measurements
    if (tiltAlignComplete && !yawAlignComplete) {
        Vector3f eulerAngles;
        getEulerAngles(eulerAngles);
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        StoreQuatReset();
        yawAlignComplete = true;
        hal.console->printf("EKF yaw alignment complete\n");
    }
}

// return true if we should use the airspeed sensor
bool NavEKF2_core::useAirspeed(void) const
{
    return _ahrs->airspeed_sensor_enabled();
}

// return true if we should use the range finder sensor
bool NavEKF2_core::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

// return true if optical flow data is available
bool NavEKF2_core::optFlowDataPresent(void) const
{
    return (imuSampleTime_ms - flowMeaTime_ms < 200);
}

// return true if the filter to be ready to use gps
bool NavEKF2_core::readyToUseGPS(void) const
{
    return validOrigin && tiltAlignComplete && yawAlignComplete && gpsQualGood;
}

// return true if we should use the compass
bool NavEKF2_core::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw();
}

/*
  should we assume zero sideslip?
 */
bool NavEKF2_core::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return _ahrs->get_fly_forward() && _ahrs->get_vehicle_class() != AHRS_VEHICLE_GROUND;
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
// also return the current offsets applied to the GPS position measurements
void  NavEKF2_core::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    velVar   = sqrtf(velTestRatio);
    posVar   = sqrtf(posTestRatio);
    hgtVar   = sqrtf(hgtTestRatio);
    magVar.x = sqrtf(magTestRatio.x);
    magVar.y = sqrtf(magTestRatio.y);
    magVar.z = sqrtf(magTestRatio.z);
    tasVar   = sqrtf(tasTestRatio);
    offset   = gpsPosGlitchOffsetNE;
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
void  NavEKF2_core::getFilterFaults(uint8_t &faults) const
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
return filter function status as a bitmasked integer
 0 = attitude estimate valid
 1 = horizontal velocity estimate valid
 2 = vertical velocity estimate valid
 3 = relative horizontal position estimate valid
 4 = absolute horizontal position estimate valid
 5 = vertical position estimate valid
 6 = terrain height estimate valid
 7 = constant position mode
*/
void  NavEKF2_core::getFilterStatus(nav_filter_status &status) const
{
    // init return value
    status.value = 0;

    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool notDeadReckoning = (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && (frontend._fusionModeGPS == 3);
    bool gpsNavPossible = !gpsNotAvailable && (frontend._fusionModeGPS <= 2);
    bool filterHealthy = healthy() && tiltAlignComplete && yawAlignComplete;

    // set individual flags
    status.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && notDeadReckoning && filterHealthy;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    status.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && notDeadReckoning && filterHealthy;   // relative horizontal position estimate valid
    status.flags.horiz_pos_abs = doingNormalGpsNav && notDeadReckoning && filterHealthy; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout && filterHealthy;            // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;     // constant position mode
    status.flags.pred_horiz_pos_rel = (optFlowNavPossible || gpsNavPossible) && filterHealthy; // we should be able to estimate a relative position when we enter flight mode
    status.flags.pred_horiz_pos_abs = gpsNavPossible && filterHealthy; // we should be able to estimate an absolute position when we enter flight mode
    status.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    status.flags.takeoff = expectGndEffectTakeoff; // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    status.flags.touchdown = expectGndEffectTouchdown; // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    status.flags.using_gps = (imuSampleTime_ms - lastPosPassTime_ms) < 4000;
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
    mavlink_msg_ekf_status_report_send(chan, flags, velVar, posVar, hgtVar, magVar.length(), tasVar);

}


#endif // HAL_CPU_CLASS