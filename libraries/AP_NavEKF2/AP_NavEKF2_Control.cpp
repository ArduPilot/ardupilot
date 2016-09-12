/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

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
    // if we are on the 2nd core and _magCal is 3 then treat it as
    // 2. This is a workaround for a mag fusion problem
    if (frontend->_magCal ==3 && imu_index == 1) {
        return 2;
    }
    return frontend->_magCal;
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
     if (PV_AidingMode == AID_NONE) {
        // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
        // and IMU gyro bias estimates have stabilised
        bool filterIsStable = tiltAlignComplete && yawAlignComplete && checkGyroCalStatus();
        // If GPS usage has been prohiited then we use flow aiding provided optical flow data is present
        // GPS aiding is the perferred option unless excluded by the user
        if((frontend->_fusionModeGPS) != 3 && readyToUseGPS() && filterIsStable && !gpsInhibit) {
            PV_AidingMode = AID_ABSOLUTE;
        } else if ((frontend->_fusionModeGPS == 3) && optFlowDataPresent()) {
            PV_AidingMode = AID_RELATIVE;
        }
    } else if (PV_AidingMode == AID_RELATIVE) {
         // Check if the optical flow sensor has timed out
         bool flowSensorTimeout = ((imuSampleTime_ms - flowValidMeaTime_ms) > 5000);
         // Check if the fusion has timed out (flow measurements have been rejected for too long)
         bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
         if (flowSensorTimeout || flowFusionTimeout) {
             PV_AidingMode = AID_NONE;
         }
     } else if (PV_AidingMode == AID_ABSOLUTE) {
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
             if (!useAirspeed() && !assume_zero_sideslip()) {
                 if (optFlowBackupAvailable) {
                     // attempt optical flow navigation
                     PV_AidingMode = AID_RELATIVE;
                 } else {
                     // put the filter into constant position mode
                     PV_AidingMode = AID_NONE;
                 }
             }
         } else if (gpsInhibit) {
             // put the filter into constant position mode in response to an exernal request
             PV_AidingMode = AID_NONE;
         }
     }

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (PV_AidingMode != PV_AidingModePrev) {
        // set various  usage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        if (PV_AidingMode == AID_NONE) {
            // We have ceased aiding
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "EKF2 IMU%u has stopped aiding",(unsigned)imu_index);
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
        } else if (PV_AidingMode == AID_RELATIVE) {
            // We have commenced aiding, but GPS usage has been prohibited so use optical flow only
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "EKF2 IMU%u is using optical flow",(unsigned)imu_index);
            posTimeout = true;
            velTimeout = true;
            // Reset the last valid flow measurement time
            flowValidMeaTime_ms = imuSampleTime_ms;
            // Reset the last valid flow fusion time
            prevFlowFuseTime_ms = imuSampleTime_ms;
        } else if (PV_AidingMode == AID_ABSOLUTE) {
            // We have commenced aiding and GPS usage is allowed
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "EKF2 IMU%u is using GPS",(unsigned)imu_index);
            posTimeout = false;
            velTimeout = false;
            // we need to reset the GPS timers to prevent GPS timeout logic being invoked on entry into GPS aiding
            // this is because the EKF can be interrupted for an arbitrary amount of time during vehicle arming checks
            lastTimeGpsReceived_ms = imuSampleTime_ms;
            secondLastGpsTime_ms = imuSampleTime_ms;
            // reset the last valid position fix time to prevent unwanted activation of GPS glitch logic
            lastPosPassTime_ms = imuSampleTime_ms;
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
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "EKF2 IMU%u tilt alignment complete",(unsigned)imu_index);
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
bool NavEKF2_core::setOriginLLH(struct Location &loc)
{
    if (PV_AidingMode == AID_ABSOLUTE) {
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
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "EKF2 IMU%u Origin Set",(unsigned)imu_index);
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
    bool optFlowNavPossible = flowDataValid && (frontend->_fusionModeGPS == 3) && delAngBiasLearned;
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
    filterStatus.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode == AID_ABSOLUTE); // The GPS is glitching
}

#endif // HAL_CPU_CLASS
