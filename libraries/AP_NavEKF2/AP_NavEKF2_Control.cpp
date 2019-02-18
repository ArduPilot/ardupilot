#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


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

    // Set the type of inertial navigation aiding used
    setAidingMode();

}

/*
  return effective value for _magCal for this core
 */
uint8_t NavEKF2_core::effective_magCal(void) const
{
    // force use of simple magnetic heading fusion for specified cores
    if (frontend->_magMask & core_index) {
        return 2;
    } else {
        return frontend->_magCal;
    }
}

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
// avoid unnecessary operations
void NavEKF2_core::setWindMagStateLearningMode()
{
    // If we are on ground, or in constant position mode, or don't have the right vehicle and sensing to estimate wind, inhibit wind states
    bool setWindInhibit = (!useAirspeed() && !assume_zero_sideslip()) || onGround || (PV_AidingMode == AID_NONE);
    if (!inhibitWindStates && setWindInhibit) {
        inhibitWindStates = true;
    } else if (inhibitWindStates && !setWindInhibit) {
        inhibitWindStates = false;
        // set states and variances
        if (yawAlignComplete && useAirspeed()) {
            // if we have airspeed and a valid heading, set the wind states to the reciprocal of the vehicle heading
            // which assumes the vehicle has launched into the wind
             Vector3f tempEuler;
            stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
            float windSpeed =  sqrtf(sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y)) - tasDataDelayed.tas;
            stateStruct.wind_vel.x = windSpeed * cosf(tempEuler.z);
            stateStruct.wind_vel.y = windSpeed * sinf(tempEuler.z);

            // set the wind sate variances to the measurement uncertainty
            for (uint8_t index=22; index<=23; index++) {
                P[index][index] = sq(constrain_float(frontend->_easNoise, 0.5f, 5.0f) * constrain_float(_ahrs->get_EAS2TAS(), 0.9f, 10.0f));
            }
        } else {
            // set the variances using a typical wind speed
            for (uint8_t index=22; index<=23; index++) {
                P[index][index] = sq(5.0f);
            }
        }
    }

    // determine if the vehicle is manoevring
    if (accNavMagHoriz > 0.5f) {
        manoeuvring = true;
    } else {
        manoeuvring = false;
    }

    // Determine if learning of magnetic field states has been requested by the user
    uint8_t magCal = effective_magCal();
    bool magCalRequested =
            ((magCal == 0) && inFlight) || // when flying
            ((magCal == 1) && manoeuvring)  || // when manoeuvring
            ((magCal == 3) && finalInflightYawInit && finalInflightMagInit) || // when initial in-air yaw and mag field reset is complete
            (magCal == 4); // all the time

    // Deny mag calibration request if we aren't using the compass, it has been inhibited by the user,
    // we do not have an absolute position reference or are on the ground (unless explicitly requested by the user)
    bool magCalDenied = !use_compass() || (magCal == 2) || (onGround && magCal != 4);

    // Inhibit the magnetic field calibration if not requested or denied
    bool setMagInhibit = !magCalRequested || magCalDenied;
    if (!inhibitMagStates && setMagInhibit) {
        inhibitMagStates = true;
    } else if (inhibitMagStates && !setMagInhibit) {
        inhibitMagStates = false;
        if (magFieldLearned) {
            // if we have already learned the field states, then retain the learned variances
            P[16][16] = earthMagFieldVar.x;
            P[17][17] = earthMagFieldVar.y;
            P[18][18] = earthMagFieldVar.z;
            P[19][19] = bodyMagFieldVar.x;
            P[20][20] = bodyMagFieldVar.y;
            P[21][21] = bodyMagFieldVar.z;
        } else {
            // set the variances equal to the observation variances
            for (uint8_t index=18; index<=21; index++) {
                P[index][index] = sq(frontend->_magNoise);
            }

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();

        }
        // request a reset of the yaw and magnetic field states if not done before
        if (!magStateInitComplete || (!finalInflightMagInit && inFlight)) {
            magYawResetRequest = true;
        }
    }

    // If on ground we clear the flag indicating that the magnetic field in-flight initialisation has been completed
    // because we want it re-done for each takeoff
    if (onGround) {
        finalInflightYawInit = false;
        finalInflightMagInit = false;
    }

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
    // Save the previous status so we can detect when it has changed
    PV_AidingModePrev = PV_AidingMode;

    // Determine if we should change aiding mode
    switch (PV_AidingMode) {
    case AID_NONE: {
        // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
        // and IMU gyro bias estimates have stabilised
        bool filterIsStable = tiltAlignComplete && yawAlignComplete && checkGyroCalStatus();
        // If GPS usage has been prohiited then we use flow aiding provided optical flow data is present
        // GPS aiding is the preferred option unless excluded by the user
        bool canUseGPS = ((frontend->_fusionModeGPS) != 3 && readyToUseGPS() && filterIsStable && !gpsInhibit);
        bool canUseRangeBeacon = readyToUseRangeBeacon() && filterIsStable;
        bool canUseExtNav = readyToUseExtNav();
        if(canUseGPS || canUseRangeBeacon || canUseExtNav) {
            PV_AidingMode = AID_ABSOLUTE;
        } else if (optFlowDataPresent() && filterIsStable) {
            PV_AidingMode = AID_RELATIVE;
        }
        }
        break;

    case AID_RELATIVE: {
        // Check if the optical flow sensor has timed out
        bool flowSensorTimeout = ((imuSampleTime_ms - flowValidMeaTime_ms) > 5000);
        // Check if the fusion has timed out (flow measurements have been rejected for too long)
        bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
        // Enable switch to absolute position mode if GPS is available
        // If GPS is not available and flow fusion has timed out, then fall-back to no-aiding
        if((frontend->_fusionModeGPS) != 3 && readyToUseGPS() && !gpsInhibit) {
            PV_AidingMode = AID_ABSOLUTE;
        } else if (flowSensorTimeout || flowFusionTimeout) {
            PV_AidingMode = AID_NONE;
        }
        }
        break;

    case AID_ABSOLUTE: {
        // Find the minimum time without data required to trigger any check
        uint16_t minTestTime_ms = MIN(frontend->tiltDriftTimeMax_ms, MIN(frontend->posRetryTimeNoVel_ms,frontend->posRetryTimeUseVel_ms));

        // Check if optical flow data is being used
        bool optFlowUsed = (imuSampleTime_ms - prevFlowFuseTime_ms <= minTestTime_ms);

        // Check if airspeed data is being used
        bool airSpdUsed = (imuSampleTime_ms - lastTasPassTime_ms <= minTestTime_ms);

        // Check if range beacon data is being used
        bool rngBcnUsed = (imuSampleTime_ms - lastRngBcnPassTime_ms <= minTestTime_ms);

        // Check if GPS is being used
        bool posUsed = (imuSampleTime_ms - lastPosPassTime_ms <= minTestTime_ms);
        bool gpsVelUsed = (imuSampleTime_ms - lastVelPassTime_ms <= minTestTime_ms);

        // Check if attitude drift has been constrained by a measurement source
        bool attAiding = posUsed || gpsVelUsed || optFlowUsed || airSpdUsed || rngBcnUsed;

        // check if velocity drift has been constrained by a measurement source
        bool velAiding = gpsVelUsed || airSpdUsed || optFlowUsed;

        // check if position drift has been constrained by a measurement source
        bool posAiding = posUsed || rngBcnUsed;

        // Check if the loss of attitude aiding has become critical
        bool attAidLossCritical = false;
        if (!attAiding) {
            attAidLossCritical = (imuSampleTime_ms - prevFlowFuseTime_ms > frontend->tiltDriftTimeMax_ms) &&
                   (imuSampleTime_ms - lastTasPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
                   (imuSampleTime_ms - lastRngBcnPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
                   (imuSampleTime_ms - lastPosPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
                   (imuSampleTime_ms - lastVelPassTime_ms > frontend->tiltDriftTimeMax_ms);
        }

        // Check if the loss of position accuracy has become critical
        bool posAidLossCritical = false;
        if (!posAiding ) {
            uint16_t maxLossTime_ms;
            if (!velAiding) {
                maxLossTime_ms = frontend->posRetryTimeNoVel_ms;
            } else {
                maxLossTime_ms = frontend->posRetryTimeUseVel_ms;
            }
            posAidLossCritical = (imuSampleTime_ms - lastRngBcnPassTime_ms > maxLossTime_ms) &&
                   (imuSampleTime_ms - lastPosPassTime_ms > maxLossTime_ms);
        }

        if (attAidLossCritical) {
            // if the loss of attitude data is critical, then put the filter into a constant position mode
            PV_AidingMode = AID_NONE;
            posTimeout = true;
            velTimeout = true;
            rngBcnTimeout = true;
            tasTimeout = true;
            gpsNotAvailable = true;
        } else if (posAidLossCritical) {
            // if the loss of position is critical, declare all sources of position aiding as being timed out
            posTimeout = true;
            velTimeout = true;
            rngBcnTimeout = true;
            gpsNotAvailable = true;
        }
        break;
    }
    }

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (PV_AidingMode != PV_AidingModePrev) {
        // set various  usage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        switch (PV_AidingMode) {
        case AID_NONE:
            // We have ceased aiding
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF2 IMU%u has stopped aiding",(unsigned)imu_index);
            // When not aiding, estimate orientation & height fusing synthetic constant position and zero velocity measurement to constrain tilt errors
            posTimeout = true;
            velTimeout = true;            
            // Reset the normalised innovation to avoid false failing bad fusion tests
            velTestRatio = 0.0f;
            posTestRatio = 0.0f;
             // store the current position to be used to keep reporting the last known position
            lastKnownPositionNE.x = stateStruct.position.x;
            lastKnownPositionNE.y = stateStruct.position.y;
            // initialise filtered altitude used to provide a takeoff reference to current baro on disarm
            // this reduces the time required for the baro noise filter to settle before the filtered baro data can be used
            meaHgtAtTakeOff = baroDataDelayed.hgt;
            // reset the vertical position state to faster recover from baro errors experienced during touchdown
            stateStruct.position.z = -meaHgtAtTakeOff;
            break;

        case AID_RELATIVE:
            // We have commenced aiding, but GPS usage has been prohibited so use optical flow only
            gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u is using optical flow",(unsigned)imu_index);
            posTimeout = true;
            velTimeout = true;
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
            break;

        case AID_ABSOLUTE: {
            bool canUseGPS = ((frontend->_fusionModeGPS) != 3 && readyToUseGPS() && !gpsInhibit);
            bool canUseRangeBeacon = readyToUseRangeBeacon();
            bool canUseExtNav = readyToUseExtNav();
            // We have commenced aiding and GPS usage is allowed
            if (canUseGPS) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u is using GPS",(unsigned)imu_index);
            }
            posTimeout = false;
            velTimeout = false;
            // We have commenced aiding and range beacon usage is allowed
            if (canUseRangeBeacon) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u is using range beacons",(unsigned)imu_index);
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u initial pos NE = %3.1f,%3.1f (m)",(unsigned)imu_index,(double)receiverPos.x,(double)receiverPos.y);
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u initial beacon pos D offset = %3.1f (m)",(unsigned)imu_index,(double)bcnPosOffset);
            }
            // We have commenced aiding and external nav usage is allowed
            if (canUseExtNav) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u is using external nav data",(unsigned)imu_index);
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u initial pos NED = %3.1f,%3.1f,%3.1f (m)",(unsigned)imu_index,(double)extNavDataDelayed.pos.x,(double)extNavDataDelayed.pos.y,(double)extNavDataDelayed.pos.z);
                // handle yaw reset as special case
                extNavYawResetRequest = true;
                controlMagYawReset();
                // handle height reset as special case
                hgtMea = -extNavDataDelayed.pos.z;
                posDownObsNoise = sq(constrain_float(extNavDataDelayed.posErr, 0.1f, 10.0f));
                ResetHeight();
            }
            // reset the last fusion accepted times to prevent unwanted activation of timeout logic
            lastPosPassTime_ms = imuSampleTime_ms;
            lastVelPassTime_ms = imuSampleTime_ms;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
            }
            break;
        }

        // Always reset the position and velocity when changing mode
        ResetVelocity();
        ResetPosition();
    }
}

// Check the tilt and yaw alignmnent status
// Used during initial bootstrap alignment of the filter
void NavEKF2_core::checkAttitudeAlignmentStatus()
{
    // Check for tilt convergence - used during initial alignment
    float alpha = 1.0f*imuDataDelayed.delAngDT;
    float temp=tiltErrVec.length();
    tiltErrFilt = alpha*temp + (1.0f-alpha)*tiltErrFilt;
    if (tiltErrFilt < 0.005f && !tiltAlignComplete) {
        tiltAlignComplete = true;
        gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u tilt alignment complete",(unsigned)imu_index);
    }

    // submit yaw and magnetic field reset requests depending on whether we have compass data
    if (tiltAlignComplete && !yawAlignComplete) {
        if (use_compass()) {
            magYawResetRequest = true;
            gpsYawResetRequest = false;
        } else {
            magYawResetRequest = false;
            gpsYawResetRequest = true;
        }
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
    return validOrigin && tiltAlignComplete && yawAlignComplete && gpsGoodToAlign && (frontend->_fusionModeGPS != 3) && gpsDataToFuse;
}

// return true if the filter to be ready to use the beacon range measurements
bool NavEKF2_core::readyToUseRangeBeacon(void) const
{
    return tiltAlignComplete && yawAlignComplete && rngBcnGoodToAlign && rngBcnDataToFuse;
}

// return true if the filter to be ready to use external nav data
bool NavEKF2_core::readyToUseExtNav(void) const
{
    return tiltAlignComplete && extNavDataToFuse;
}

// return true if we should use the compass
bool NavEKF2_core::use_compass(void) const
{
    return _ahrs->get_compass() && _ahrs->get_compass()->use_for_yaw(magSelectIndex) && !allMagSensorsFailed;
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

// set the LLH location of the filters NED origin
bool NavEKF2_core::setOriginLLH(const Location &loc)
{
    if (PV_AidingMode == AID_ABSOLUTE && !extNavUsedForPos) {
        return false;
    }
    EKF_origin = loc;
    ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);
    validOrigin = true;
    return true;
}

// Set the NED origin to be used until the next filter reset
void NavEKF2_core::setOrigin()
{
    // assume origin at current GPS location (no averaging)
    EKF_origin = AP::gps().location();
    // if flying, correct for height change from takeoff so that the origin is at field elevation
    if (inFlight) {
        EKF_origin.alt += (int32_t)(100.0f * stateStruct.position.z);
    }
    ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);
    validOrigin = true;
    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u Origin set to GPS",(unsigned)imu_index);
}

// record a yaw reset event
void NavEKF2_core::recordYawReset()
{
    yawAlignComplete = true;
    if (inFlight) {
        finalInflightYawInit = true;
    }
}

// return true and set the class variable true if the delta angle bias has been learned
bool NavEKF2_core::checkGyroCalStatus(void)
{
    // check delta angle bias variances
    const float delAngBiasVarMax = sq(radians(0.15f * dtEkfAvg));
    delAngBiasLearned =  (P[9][9] <= delAngBiasVarMax) &&
                            (P[10][10] <= delAngBiasVarMax) &&
                            (P[11][11] <= delAngBiasVarMax);
    return delAngBiasLearned;
}

// Commands the EKF to not use GPS.
// This command must be sent prior to arming
// This command is forgotten by the EKF each time the vehicle disarms
// Returns 0 if command rejected
// Returns 1 if attitude, vertical velocity and vertical position will be provided
// Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
uint8_t NavEKF2_core::setInhibitGPS(void)
{
    if((PV_AidingMode == AID_ABSOLUTE) && motorsArmed) {
        return 0;
    } else {
        gpsInhibit = true;
        return 1;
    }
    // option 2 is not yet implemented as it requires a deeper integration of optical flow and GPS operation
}

// Update the filter status
void  NavEKF2_core::updateFilterStatus(void)
{
    // init return value
    filterStatus.value = 0;
    bool doingFlowNav = (PV_AidingMode == AID_RELATIVE) && flowDataValid;
    bool doingWindRelNav = !tasTimeout && assume_zero_sideslip();
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && useGpsVertVel) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout) || doingFlowNav;
    bool optFlowNavPossible = flowDataValid && delAngBiasLearned;
    bool gpsNavPossible = !gpsNotAvailable && gpsGoodToAlign && delAngBiasLearned;
    bool filterHealthy = healthy() && tiltAlignComplete && (yawAlignComplete || (!use_compass() && (PV_AidingMode == AID_NONE)));
    // If GPS height usage is specified, height is considered to be inaccurate until the GPS passes all checks
    bool hgtNotAccurate = (frontend->_altSource == 2) && !validOrigin;

    // set individual flags
    filterStatus.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    filterStatus.flags.horiz_vel = someHorizRefData && filterHealthy;      // horizontal velocity estimate valid
    filterStatus.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    filterStatus.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav) && filterHealthy;   // relative horizontal position estimate valid
    filterStatus.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy; // absolute horizontal position estimate valid
    filterStatus.flags.vert_pos = !hgtTimeout && filterHealthy && !hgtNotAccurate; // vertical position estimate valid
    filterStatus.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    filterStatus.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;     // constant position mode
    filterStatus.flags.pred_horiz_pos_rel = ((optFlowNavPossible || gpsNavPossible) && filterHealthy) || filterStatus.flags.horiz_pos_rel; // we should be able to estimate a relative position when we enter flight mode
    filterStatus.flags.pred_horiz_pos_abs = (gpsNavPossible && filterHealthy) || filterStatus.flags.horiz_pos_abs; // we should be able to estimate an absolute position when we enter flight mode
    filterStatus.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    filterStatus.flags.takeoff = expectGndEffectTakeoff; // The EKF has been told to expect takeoff and is in a ground effect mitigation mode
    filterStatus.flags.touchdown = expectGndEffectTouchdown; // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    filterStatus.flags.using_gps = ((imuSampleTime_ms - lastPosPassTime_ms) < 4000) && (PV_AidingMode == AID_ABSOLUTE);
    filterStatus.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode == AID_ABSOLUTE) && !extNavUsedForPos; // GPS glitching is affecting navigation accuracy
    filterStatus.flags.gps_quality_good = gpsGoodToAlign;
}

