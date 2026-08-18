#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>

#include "AP_DAL/AP_DAL.h"

static const int32_t GPS_POSXY_AIDING_MAX_AGE_MS = 400;
// Maximum measurement ages accepted while checking configured sources before arming.
static constexpr uint32_t GPS_PREARM_MAX_AGE_MS = 500;
static constexpr uint32_t EXTNAV_VEL_PREARM_MAX_AGE_MS = 250;
static constexpr uint32_t BARO_PREARM_MAX_AGE_MS = 500;

static bool lane_source_is_recent(
    uint32_t current_time_ms,
    uint32_t last_update_time_ms,
    int32_t max_age_ms)
{
    if (last_update_time_ms == 0) {
        return false;
    }

    const int32_t age_ms = int32_t(current_time_ms) - int32_t(last_update_time_ms);
    return age_ms >= 0 && age_ms < max_age_ms;
}

// Control filter mode transitions
void NavEKF3_core::controlFilterModes()
{
    // Determine motor arm status
    prevMotorsArmed = motorsArmed;
    motorsArmed = dal.get_armed();
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
NavEKF3_core::MagCal NavEKF3_core::effective_magCal(void) const
{
    // force use of simple magnetic heading fusion for specified cores
    if (frontend->_magMask & core_index) {
        return MagCal::NEVER;
    }

    // handle deprecated MagCal::EXTERNAL_YAW and MagCal::EXTERNAL_YAW_FALLBACK values
    const int8_t magCalParamVal = frontend->_magCal.get();
    if (magCalParamVal == 5) {
        return MagCal::NEVER;
    }
    if (magCalParamVal == 6) {
        return MagCal::WHEN_FLYING;
    }

    return MagCal(magCalParamVal);
}

// Determine if learning of wind and magnetic field will be enabled and set corresponding indexing limits to
// avoid unnecessary operations
void NavEKF3_core::setWindMagStateLearningMode()
{
    const bool canEstimateWind = ((finalInflightYawInit && dragFusionEnabled) || assume_zero_sideslip()) &&
                                 !onGround &&
                                 PV_AidingMode != AID_NONE;
    if (!inhibitWindStates && !canEstimateWind) {
        inhibitWindStates = true;
        lastAspdEstIsValid = false;
        updateStateIndexLim();
    } else if (inhibitWindStates && canEstimateWind &&
               (sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y) > sq(5.0f) || dragFusionEnabled)) {
        inhibitWindStates = false;
        updateStateIndexLim();
        // set states and variances
        if (yawAlignComplete && assume_zero_sideslip()) {
            // if we have a valid heading, set the wind states to the reciprocal of the vehicle heading
            // which assumes the vehicle has launched into the wind
            // use airspeed if if recent data available
            Vector3F tempEuler;
            stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
            ftype trueAirspeedVariance;
            const bool haveAirspeedMeasurement = (tasDataDelayed.allowFusion && (imuDataDelayed.time_ms - tasDataDelayed.time_ms < 500) && useAirspeed());
            if (haveAirspeedMeasurement) {
                trueAirspeedVariance = constrain_ftype(tasDataDelayed.tasVariance, WIND_VEL_VARIANCE_MIN, WIND_VEL_VARIANCE_MAX);
                const ftype windSpeed =  sqrtF(sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y)) - tasDataDelayed.tas;
                stateStruct.wind_vel.x = windSpeed * cosF(tempEuler.z);
                stateStruct.wind_vel.y = windSpeed * sinF(tempEuler.z);
            } else {
                trueAirspeedVariance = sq(WIND_VEL_VARIANCE_MAX); // use 2-sigma for faster initial convergence
            }

            // set the wind state variances to the measurement uncertainty
            zeroCols(P, 22, 23);
            zeroRows(P, 22, 23);
            P[22][22] = P[23][23] = trueAirspeedVariance;

            windStatesAligned = true;

        } else {
            // set the variances using a typical max wind speed for small UAV operation
            zeroCols(P, 22, 23);
            zeroRows(P, 22, 23);
            for (uint8_t index=22; index<=23; index++) {
                P[index][index] = sq(WIND_VEL_VARIANCE_MAX);
            }
        }
    }

    // determine if the vehicle is manoeuvring
    manoeuvring = accNavMagHoriz > 0.5f;

    // Determine if learning of magnetic field states has been requested by the user
    bool magCalRequested =
        ((effectiveMagCal == MagCal::WHEN_FLYING) && inFlight) || // when flying
        ((effectiveMagCal == MagCal::WHEN_MANOEUVRING) && manoeuvring)  || // when manoeuvring
        ((effectiveMagCal == MagCal::AFTER_FIRST_CLIMB) && finalInflightYawInit && finalInflightMagInit) || // when initial in-air yaw and mag field reset is complete
        (effectiveMagCal == MagCal::ALWAYS); // all the time

    // Deny mag calibration request if we aren't using the compass, it has been inhibited by the user,
    // we do not have an absolute position reference or are on the ground (unless explicitly requested by the user)
    bool magCalDenied = !use_compass() || (effectiveMagCal == MagCal::NEVER) || (onGround && effectiveMagCal != MagCal::ALWAYS);

    // Inhibit the magnetic field calibration if not requested or denied
    bool setMagInhibit = !magCalRequested || magCalDenied;
    if (!inhibitMagStates && setMagInhibit) {
        inhibitMagStates = true;
        updateStateIndexLim();
        // variances will be reset in CovariancePrediction
    } else if (inhibitMagStates && !setMagInhibit) {
        inhibitMagStates = false;
        updateStateIndexLim();
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
            for (uint8_t index=16; index<=21; index++) {
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

    // inhibit delta velocity bias learning if we have not yet aligned the tilt
    if (tiltAlignComplete && inhibitDelVelBiasStates) {
        // activate the states
        inhibitDelVelBiasStates = false;
        updateStateIndexLim();

        // set the initial covariance values
        P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim * dtEkfAvg);
        P[14][14] = P[13][13];
        P[15][15] = P[13][13];
    }

    if (tiltAlignComplete && inhibitDelAngBiasStates) {
        // activate the states
        inhibitDelAngBiasStates = false;
        updateStateIndexLim();

        // set the initial covariance values
        P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
        P[11][11] = P[10][10];
        P[12][12] = P[10][10];
    }

    // If on ground we clear the flag indicating that the magnetic field in-flight initialisation has been completed
    // because we want it re-done for each takeoff
    if (onGround) {
        finalInflightYawInit = false;
        finalInflightMagInit = false;
        magFieldLearned = false;
    }

    updateStateIndexLim();
}

// Adjust the indexing limits used to address the covariance, states and other EKF arrays to avoid unnecessary operations
// if we are not using those states
void NavEKF3_core::updateStateIndexLim()
{
    if (inhibitWindStates) {
        if (inhibitMagStates) {
            if (inhibitDelVelBiasStates) {
                if (inhibitDelAngBiasStates) {
                    stateIndexLim = 9;
                } else {
                    stateIndexLim = 12;
                }
            } else {
                stateIndexLim = 15;
            }
        } else {
            stateIndexLim = 21;
        }
    } else {
        stateIndexLim = 23;
    }
}

// set the default yaw source
void NavEKF3_core::setYawSource()
{
    AP_NavEKF_Source::SourceYaw yaw_source_new = yaw_source();
    if (wasLearningCompass_ms > 0) {
        // can't use compass while it is being calibrated
        if (yaw_source_new == AP_NavEKF_Source::SourceYaw::COMPASS) {
            yaw_source_new = AP_NavEKF_Source::SourceYaw::NONE;
        } else if (yaw_source_new == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK) {
            yaw_source_new = AP_NavEKF_Source::SourceYaw::GPS;
        }
    }
    if (yaw_source_new != yaw_source_last) {
        yaw_source_last = yaw_source_new;
        yaw_source_reset = true;
    }
}

// Set inertial navigation aiding mode
void NavEKF3_core::setAidingMode()
{
    resetDataSource posResetSource = resetDataSource::DEFAULT;
    resetDataSource velResetSource = resetDataSource::DEFAULT;

    // Save the previous status so we can detect when it has changed
    PV_AidingModePrev = PV_AidingMode;

    setYawSource();

    // Check that the gyro bias variance has converged
    checkGyroCalStatus();

    // Upstream has code here related to disabling ext-nav in specific cases, e.g. when compass
    // is disabled, but it is unwanted by us.

    // Determine if we should change aiding mode
    switch (PV_AidingMode) {
        case AID_NONE: {
            // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
            // and IMU gyro bias estimates have stabilised
            // If GPS usage has been prohiited then we use flow aiding provided optical flow data is present
            // GPS aiding is the preferred option unless excluded by the user
            if (readyToUseGPS() || readyToUseRangeBeacon() || readyToUseExtNav()) {
                PV_AidingMode = AID_ABSOLUTE;
            } else if (
#if EK3_FEATURE_OPTFLOW_FUSION
                readyToUseOptFlow() ||
#endif
                readyToUseBodyOdm()) {
                PV_AidingMode = AID_RELATIVE;
            }
            break;
        }
        case AID_RELATIVE: {
            // Check if the fusion has timed out (flow measurements have been rejected for too long)
            bool flowFusionTimeout = ((imuSampleTime_ms - prevFlowFuseTime_ms) > 5000);
            // Check if the fusion has timed out (body odometry measurements have been rejected for too long)
            bool bodyOdmFusionTimeout = ((imuSampleTime_ms - prevBodyVelFuseTime_ms) > 5000);
            // Enable switch to absolute position mode if GPS or range beacon data is available
            // If GPS or range beacons data is not available and flow fusion has timed out, then fall-back to no-aiding
            if (readyToUseGPS() || readyToUseRangeBeacon() || readyToUseExtNav()) {
                PV_AidingMode = AID_ABSOLUTE;
            } else if (flowFusionTimeout && bodyOdmFusionTimeout) {
                PV_AidingMode = AID_NONE;
            }
            break;
        }
        case AID_ABSOLUTE: {
            // Find the minimum time without data required to trigger any check
            uint16_t minTestTime_ms = MIN(frontend->tiltDriftTimeMax_ms, MIN(frontend->posRetryTimeNoVel_ms,frontend->posRetryTimeUseVel_ms));

            // Check if optical flow data is being used
            bool optFlowUsed = (imuSampleTime_ms - prevFlowFuseTime_ms <= minTestTime_ms);

            // Check if body odometry data is being used
            bool bodyOdmUsed = (imuSampleTime_ms - prevBodyVelFuseTime_ms <= minTestTime_ms);

            // Check if airspeed data is being used
            bool airSpdUsed = (imuSampleTime_ms - lastTasPassTime_ms <= minTestTime_ms);

            // check if drag data is being used
            bool dragUsed = (imuSampleTime_ms - lastDragPassTime_ms <= minTestTime_ms);

#if EK3_FEATURE_BEACON_FUSION
            // Check if range beacon data is being used
            const bool rngBcnUsed = (imuSampleTime_ms - rngBcn.lastPassTime_ms <= minTestTime_ms);
#else
            const bool rngBcnUsed = false;
#endif

            // Check if GPS or external nav is being used
            bool posUsed = (imuSampleTime_ms - lastGpsPosPassTime_ms <= minTestTime_ms);
            bool gpsVelUsed = (imuSampleTime_ms - lastVelPassTime_ms <= minTestTime_ms);

            // Check if attitude drift has been constrained by a measurement source
            bool attAiding = posUsed || gpsVelUsed || optFlowUsed || airSpdUsed || dragUsed || rngBcnUsed || bodyOdmUsed;

            // Check if velocity drift has been constrained by a measurement source
            // Currently these are all the same source as will stabilise attitude because we do not currently have
            // a sensor that only observes attitude
            velAiding = posUsed || gpsVelUsed || optFlowUsed || airSpdUsed || dragUsed || rngBcnUsed || bodyOdmUsed;

            // Store the last valid airspeed estimate
            windStateIsObservable = !inhibitWindStates && (posUsed || gpsVelUsed || optFlowUsed || rngBcnUsed || bodyOdmUsed);
            if (windStateIsObservable) {
                lastAirspeedEstimate = (stateStruct.velocity - Vector3F(stateStruct.wind_vel.x, stateStruct.wind_vel.y, 0.0F)).length();
                lastAspdEstIsValid = true;
            }

            // check if position drift has been constrained by a measurement source
            bool posAiding = posUsed || rngBcnUsed;

            // Check if the loss of attitude aiding has become critical
            bool attAidLossCritical = false;
            if (!attAiding) {
            	attAidLossCritical = (imuSampleTime_ms - prevFlowFuseTime_ms > frontend->tiltDriftTimeMax_ms) &&
                		(imuSampleTime_ms - lastTasPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
#if EK3_FEATURE_BEACON_FUSION
                        (imuSampleTime_ms - rngBcn.lastPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
#endif
                        (imuSampleTime_ms - lastGpsPosPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
                        (imuSampleTime_ms - lastVelPassTime_ms > frontend->tiltDriftTimeMax_ms);
            }

            // Check if the loss of position accuracy has become critical
            bool posAidLossCritical = false;
            if (!posAiding) {
                uint16_t maxLossTime_ms;
                if (!velAiding) {
                    maxLossTime_ms = frontend->posRetryTimeNoVel_ms;
                } else {
                    maxLossTime_ms = frontend->posRetryTimeUseVel_ms;
                }
                posAidLossCritical =
#if EK3_FEATURE_BEACON_FUSION
                    (imuSampleTime_ms - rngBcn.lastPassTime_ms > maxLossTime_ms) &&
#endif
                    (imuSampleTime_ms - lastGpsPosPassTime_ms > maxLossTime_ms);
            }

            if (attAidLossCritical) {
                // if the loss of attitude data is critical, then put the filter into a constant position mode
                PV_AidingMode = AID_NONE;
                posTimeout = true;
                velTimeout = true;
                tasTimeout = true;
                dragTimeout = true;
                gpsIsInUse = false;
             } else if (posAidLossCritical) {
                // if the loss of position is critical, declare all sources of position aiding as being timed out
                posTimeout = true;
                velTimeout = !optFlowUsed && !gpsVelUsed && !bodyOdmUsed;
                gpsIsInUse = false;

            }
            break;
        }
    }

    // check to see if we are starting or stopping aiding and set states and modes as required
    if (PV_AidingMode != PV_AidingModePrev) {
        // set various usage modes based on the condition when we start aiding. These are then held until aiding is stopped.
        switch (PV_AidingMode) {
        case AID_NONE:
            // We have ceased aiding
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF3 IMU%u stopped aiding",(unsigned)imu_index);
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
            // store the current height to be used to keep reporting
            // the last known position
            lastKnownPositionD = stateStruct.position.z;
            // reset relative aiding sensor fusion activity status
            flowFusionActive = false;
            bodyVelFusionActive = false;
            break;

        case AID_RELATIVE:
            // We are doing relative position navigation where velocity errors are constrained, but position drift will occur
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u started relative aiding",(unsigned)imu_index);
#if EK3_FEATURE_OPTFLOW_FUSION
            if (readyToUseOptFlow()) {
                // Reset time stamps
                flowValidMeaTime_ms = imuSampleTime_ms;
                prevFlowFuseTime_ms = imuSampleTime_ms;
            } else
#endif
                if (readyToUseBodyOdm()) {
                 // Reset time stamps
                lastbodyVelPassTime_ms = imuSampleTime_ms;
                prevBodyVelFuseTime_ms = imuSampleTime_ms;
            }
            posTimeout = true;
            velTimeout = true;
            break;

        case AID_ABSOLUTE:
            if (readyToUseGPS()) {
                // We are commencing aiding using GPS - this is the preferred method
                posResetSource = resetDataSource::GPS;
                velResetSource = resetDataSource::GPS;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u is using GPS",(unsigned)imu_index);
#if EK3_FEATURE_BEACON_FUSION
            } else if (readyToUseRangeBeacon()) {
                // We are commencing aiding using range beacons
                posResetSource = resetDataSource::RNGBCN;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u is using range beacons",(unsigned)imu_index);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial pos NE = %3.1f,%3.1f (m)",(unsigned)imu_index,(double)rngBcn.receiverPos.x,(double)rngBcn.receiverPos.y);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial beacon pos D offset = %3.1f (m)",(unsigned)imu_index,(double)rngBcn.posOffsetNED.z);
#endif  // EK3_FEATURE_BEACON_FUSION
#if EK3_FEATURE_EXTERNAL_NAV
            } else if (readyToUseExtNav()) {
                // we are commencing aiding using external nav
                posResetSource = resetDataSource::EXTNAV;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u is using external nav data",(unsigned)imu_index);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial pos NED = %3.1f,%3.1f,%3.1f (m)",(unsigned)imu_index,(double)extNavDataDelayed.pos.x,(double)extNavDataDelayed.pos.y,(double)extNavDataDelayed.pos.z);
                if (useExtNavVel) {
                    velResetSource = resetDataSource::EXTNAV;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial vel NED = %3.1f,%3.1f,%3.1f (m/s)",(unsigned)imu_index,(double)extNavVelDelayed.vel.x,(double)extNavVelDelayed.vel.y,(double)extNavVelDelayed.vel.z);
                }
                // handle height reset as special case
                if (frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::EXTNAV) {
                    hgtMea = -extNavDataDelayed.pos.z;
                    posDownObsNoise = sq(constrain_ftype(extNavDataDelayed.posErr, 0.1f, 10.0f));
                    ResetHeight();
                }
#endif // EK3_FEATURE_EXTERNAL_NAV
            }

            // clear timeout flags as a precaution to avoid triggering any additional transitions
            posTimeout = false;
            velTimeout = false;

            // reset the last fusion accepted times to prevent unwanted activation of timeout logic
            lastGpsPosPassTime_ms = imuSampleTime_ms;
            lastVelPassTime_ms = imuSampleTime_ms;
#if EK3_FEATURE_BEACON_FUSION
            rngBcn.lastPassTime_ms = imuSampleTime_ms;
#endif
            break;
        }

        // Always reset the position and velocity when changing mode
        ResetVelocity(velResetSource);
        ResetPosition(posResetSource);
    }

}

// Check the tilt and yaw alignmnent status
// Used during initial bootstrap alignment of the filter
void NavEKF3_core::checkAttitudeAlignmentStatus()
{
    // Check for tilt convergence - used during initial alignment
    // Once the tilt variances have reduced, re-set the yaw and magnetic field states
    // and declare the tilt alignment complete
    if (!tiltAlignComplete) {
        if (tiltErrorVariance < sq(radians(5.0))) {
            tiltAlignComplete = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u tilt alignment complete",(unsigned)imu_index);
        }
    }

    // submit yaw and magnetic field reset request
    if (!yawAlignComplete && tiltAlignComplete && use_compass()) {
        magYawResetRequest = true;
    }

}

// return true if we should use the airspeed sensor
bool NavEKF3_core::useAirspeed(void) const
{
    return dal.airspeed_sensor_enabled();
}

// return true if we should use the range finder sensor
bool NavEKF3_core::useRngFinder(void) const
{
    // TO-DO add code to set this based in setting of optical flow use parameter and presence of sensor
    return true;
}

#if EK3_FEATURE_OPTFLOW_FUSION
// return true if the filter is ready to start using optical flow measurements
bool NavEKF3_core::readyToUseOptFlow(void) const
{
    // ensure flow is used for navigation and not terrain alt estimation
    if (frontend->_flowUse != FLOW_USE_NAV) {
        return false;
    }

    if (!uses_velxy_source(AP_NavEKF_Source::SourceXY::OPTFLOW)) {
        return false;
    }

    // We need stable roll/pitch angles and gyro bias estimates but do not need the yaw angle aligned to use optical flow
    return (imuSampleTime_ms - flowMeaTime_ms < 200) && tiltAlignComplete && delAngBiasLearned;
}
#endif  // EK3_FEATURE_OPTFLOW_FUSION

// return true if the filter is ready to start using body frame odometry measurements
bool NavEKF3_core::readyToUseBodyOdm(void) const
{
#if EK3_FEATURE_BODY_ODOM
    if (!uses_velxy_source(AP_NavEKF_Source::SourceXY::EXTNAV) &&
        !uses_velxy_source(AP_NavEKF_Source::SourceXY::WHEEL_ENCODER)) {
        // exit immediately if sources not configured to fuse external nav or wheel encoders
        return false;
    }

    // Check for fresh visual odometry data that meets the accuracy required for alignment
    bool visoDataGood = (imuSampleTime_ms - bodyOdmMeasTime_ms < 200) && (bodyOdmDataNew.velErr < 1.0f);

    // Check for fresh wheel encoder data
    bool wencDataGood = (imuDataDelayed.time_ms - wheelOdmDataDelayed.time_ms < 200);

    // We require stable roll/pitch angles and gyro bias estimates but do not need the yaw angle aligned to use odometry measurements
    // because they are in a body frame of reference
    return (visoDataGood || wencDataGood)
            && tiltAlignComplete
            && delAngBiasLearned;
#else
    return false;
#endif // EK3_FEATURE_BODY_ODOM
}

// return true if the filter to be ready to use gps
bool NavEKF3_core::readyToUseGPS(void) const
{
    if (!uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS)) {
        return false;
    }

    return validOrigin && tiltAlignComplete && yawAlignComplete && (delAngBiasLearned || assume_zero_sideslip()) && gpsGoodToAlign && gpsDataToFuse;
}

// return true if the filter to be ready to use the beacon range measurements
bool NavEKF3_core::readyToUseRangeBeacon(void) const
{
#if EK3_FEATURE_BEACON_FUSION
    if (!uses_posxy_source(AP_NavEKF_Source::SourceXY::BEACON)) {
        return false;
    }

    return tiltAlignComplete && yawAlignComplete && delAngBiasLearned && rngBcn.alignmentCompleted && rngBcn.dataToFuse;
#else
    return false;
#endif  // EK3_FEATURE_BEACON_FUSION
}

// return true if the filter is ready to use external nav data
bool NavEKF3_core::readyToUseExtNav(void) const
{
#if EK3_FEATURE_EXTERNAL_NAV
    if (!uses_posxy_source(AP_NavEKF_Source::SourceXY::EXTNAV)) {
        return false;
    }

    return tiltAlignComplete && extNavPosRecent();
#else
    return false;
#endif // EK3_FEATURE_EXTERNAL_NAV
}

bool NavEKF3_core::extNavPosRecent(void) const
{
#if EK3_FEATURE_EXTERNAL_NAV
    return lane_source_is_recent(
        imuSampleTime_ms,
        lastExtNavPosReceived_ms,
        3000);
#else
    return false;
#endif // EK3_FEATURE_EXTERNAL_NAV
}

bool NavEKF3_core::has_required_posxy_aiding(void) const
{
    return posxy_aiding_failure_reason() == PosXYAidingFailureReason::NONE;
}

NavEKF3_core::PosXYAidingFailureReason NavEKF3_core::posxy_aiding_failure_reason(void) const
{
    switch (posxy_source()) {
    case AP_NavEKF_Source::SourceXY::NONE:
        return PosXYAidingFailureReason::NONE;
    case AP_NavEKF_Source::SourceXY::GPS:
        if (!validOrigin) {
            return PosXYAidingFailureReason::GPS_NO_ORIGIN;
        }
        if (!(delAngBiasLearned || assume_zero_sideslip())) {
            return PosXYAidingFailureReason::GPS_BIAS_LEARNING;
        }
        if (!gpsGoodToAlign) {
            return PosXYAidingFailureReason::GPS_QUALITY_LOW;
        }
        if (!lane_source_is_recent(
                imuSampleTime_ms,
                lastTimeGpsReceived_ms,
                GPS_POSXY_AIDING_MAX_AGE_MS)) {
            return PosXYAidingFailureReason::GPS_STALE;
        }
        return PosXYAidingFailureReason::NONE;
    case AP_NavEKF_Source::SourceXY::BEACON:
        return readyToUseRangeBeacon() ?
            PosXYAidingFailureReason::NONE :
            PosXYAidingFailureReason::RANGE_BEACON_UNAVAILABLE;
    case AP_NavEKF_Source::SourceXY::EXTNAV:
        return extNavPosRecent() ? PosXYAidingFailureReason::NONE : PosXYAidingFailureReason::EXTNAV_STALE;
    case AP_NavEKF_Source::SourceXY::OPTFLOW:
#if EK3_FEATURE_OPTFLOW_FUSION
        return readyToUseOptFlow() ? PosXYAidingFailureReason::NONE : PosXYAidingFailureReason::OPTFLOW_UNAVAILABLE;
#else
        return PosXYAidingFailureReason::OPTFLOW_UNAVAILABLE;
#endif
    case AP_NavEKF_Source::SourceXY::WHEEL_ENCODER:
        return PosXYAidingFailureReason::NONE;
    }

    return PosXYAidingFailureReason::NONE;
}

const char *NavEKF3_core::posxy_aiding_failure_reason_string(PosXYAidingFailureReason reason)
{
    switch (reason) {
    case PosXYAidingFailureReason::NONE:
        return "pos aiding unavailable";
    case PosXYAidingFailureReason::GPS_NO_ORIGIN:
        return "gps no origin";
    case PosXYAidingFailureReason::GPS_BIAS_LEARNING:
        return "gps bias learning";
    case PosXYAidingFailureReason::GPS_QUALITY_LOW:
        return "gps quality low";
    case PosXYAidingFailureReason::GPS_STALE:
        return "gps stale";
    case PosXYAidingFailureReason::RANGE_BEACON_UNAVAILABLE:
        return "range beacon unavailable";
    case PosXYAidingFailureReason::EXTNAV_STALE:
        return "extnav stale";
    case PosXYAidingFailureReason::OPTFLOW_UNAVAILABLE:
        return "optflow unavailable";
    }

    return "pos aiding unavailable";
}

NavEKF3_core::YawAidingFailureReason NavEKF3_core::yaw_aiding_failure_reason(void) const
{
    switch (yaw_source()) {
    case AP_NavEKF_Source::SourceYaw::NONE:
        return YawAidingFailureReason::NONE;
    case AP_NavEKF_Source::SourceYaw::COMPASS:
        if (magTimeout) {
            return YawAidingFailureReason::COMPASS_STALE;
        }
        if (!use_compass()) {
            return YawAidingFailureReason::COMPASS_UNAVAILABLE;
        }
        return YawAidingFailureReason::NONE;
    case AP_NavEKF_Source::SourceYaw::GPS:
        return using_noncompass_for_yaw() ? YawAidingFailureReason::NONE : YawAidingFailureReason::GPS_YAW_STALE;
    case AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK:
        if (using_noncompass_for_yaw()) {
            return YawAidingFailureReason::NONE;
        }
        if (!gps_yaw_mag_fallback_active) {
            return YawAidingFailureReason::GPS_YAW_STALE;
        }
        if (magTimeout) {
            return YawAidingFailureReason::COMPASS_STALE;
        }
        if (!use_compass()) {
            return YawAidingFailureReason::COMPASS_UNAVAILABLE;
        }
        return YawAidingFailureReason::NONE;
    case AP_NavEKF_Source::SourceYaw::EXTNAV:
        return using_extnav_for_yaw() ? YawAidingFailureReason::NONE : YawAidingFailureReason::EXTNAV_YAW_STALE;
    case AP_NavEKF_Source::SourceYaw::GSF:
        return using_noncompass_for_yaw() ? YawAidingFailureReason::NONE : YawAidingFailureReason::GSF_YAW_UNAVAILABLE;
    }

    return YawAidingFailureReason::NONE;
}

bool NavEKF3_core::has_required_yaw_aiding(void) const
{
    return yaw_aiding_failure_reason() == YawAidingFailureReason::NONE;
}

const char *NavEKF3_core::yaw_aiding_failure_reason_string(YawAidingFailureReason reason)
{
    switch (reason) {
    case YawAidingFailureReason::NONE:
        return "yaw unavailable";
    case YawAidingFailureReason::COMPASS_UNAVAILABLE:
        return "compass unavailable";
    case YawAidingFailureReason::COMPASS_STALE:
        return "compass stale";
    case YawAidingFailureReason::GPS_YAW_STALE:
        return "gps yaw stale";
    case YawAidingFailureReason::EXTNAV_YAW_STALE:
        return "extnav yaw stale";
    case YawAidingFailureReason::GSF_YAW_UNAVAILABLE:
        return "gsf yaw unavailable";
    }

    return "yaw unavailable";
}

bool NavEKF3_core::has_acceptable_yaw_variance(void) const
{
    bool compass_yaw_variance_ok = !magTimeout &&
               yawTestRatio < 1.0f &&
               magTestRatio.x < 1.0f &&
               magTestRatio.y < 1.0f &&
               magTestRatio.z < 1.0f;

    switch (yaw_source()) {
    case AP_NavEKF_Source::SourceYaw::NONE:
        return true;
    case AP_NavEKF_Source::SourceYaw::COMPASS:
        return compass_yaw_variance_ok;
    case AP_NavEKF_Source::SourceYaw::GPS:
        return true;
    case AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK:
        return !gps_yaw_mag_fallback_active || compass_yaw_variance_ok;
    case AP_NavEKF_Source::SourceYaw::EXTNAV:
        return true;
    case AP_NavEKF_Source::SourceYaw::GSF:
        return true;
    }

    return true;
}

bool NavEKF3_core::has_acceptable_posxy_variance(void) const
{
    if (PV_AidingMode == AID_NONE) {
        return true;
    }
    return get_pos_variance_NE() < lane_pos_var_threshold();
}

float NavEKF3_core::get_pos_variance_NE(void) const
{
    return P[7][7] + P[8][8];
}

// Scale the eligibility threshold with this lane's own altitude above its EKF
// origin, matching how a vision-based ext-nav source's position error grows
// with height (see header for derivation). Uses this core's own position
// state, not a shared/global altitude, so each lane is judged against its own
// height estimate. Never goes below the base value, so low-altitude and
// non-altitude-dependent aiding sources keep the originally calibrated margin.
float NavEKF3_core::lane_pos_var_threshold(void) const
{
    const float alt_m = MAX(-stateStruct.position.z, 0.0f);
    const float scale = sq(alt_m / LANE_POS_VAR_THRESHOLD_REF_ALT_M);
    return MAX(LANE_POS_VAR_THRESHOLD_BASE, LANE_POS_VAR_THRESHOLD_BASE * scale);
}

bool NavEKF3_core::configured_sources_ready(char *failure_msg, uint8_t failure_msg_len) const
{
    const auto __fail = [&](const char *field_name) -> bool {
        dal.snprintf(failure_msg, failure_msg_len, "EKF3 core %d %s not ready", (int)core_index, field_name);
        return false;
    };
    const auto __gps_recent_for_prearm = [&]() -> bool {
        return lane_source_is_recent(imuSampleTime_ms, lastTimeGpsReceived_ms, GPS_PREARM_MAX_AGE_MS);
    };
    const auto __extnav_recent_for_prearm = [&]() -> bool {
        return extNavPosRecent();
    };
    const auto __gps_ready_for_prearm = [&]() -> bool {
        return validOrigin &&
               tiltAlignComplete &&
               yawAlignComplete &&
               (delAngBiasLearned || assume_zero_sideslip()) &&
               gpsGoodToAlign &&
               __gps_recent_for_prearm();
    };
    const auto __extnav_posxy_ready_for_prearm = [&]() -> bool {
        return tiltAlignComplete && __extnav_recent_for_prearm();
    };
    const auto __extnav_vel_ready_for_prearm = [&]() -> bool {
#if EK3_FEATURE_EXTERNAL_NAV
        return (imuSampleTime_ms - extNavVelMeasTime_ms) < EXTNAV_VEL_PREARM_MAX_AGE_MS && useExtNavVel;
#else
        return false;
#endif
    };
    // All current GPS source checks require yaw alignment. Add a require_yaw
    // argument if a source that can be ready without yaw alignment is added.
    const auto __fail_gps_source = [&](const char *field_name, const bool require_vz) -> bool {
        const bool bias_ok = delAngBiasLearned || assume_zero_sideslip();
        const char *reason = "gps unavailable";
        if (!validOrigin) {
            reason = "gps no origin";
        } else if (!tiltAlignComplete) {
            reason = "tilt unaligned";
        } else if (!yawAlignComplete) {
            reason = "yaw unaligned";
        } else if (!bias_ok) {
            reason = "gps bias learning";
        } else if (!gpsGoodToAlign) {
            reason = "gps quality low";
        } else if (!__gps_recent_for_prearm()) {
            reason = "gps stale";
        } else if (require_vz && !gpsDataNew.have_vz) {
            reason = "gps vertical velocity unavailable";
        }
        dal.snprintf(
            failure_msg,
            failure_msg_len,
            "EKF3 core %d %s GPS: %s",
            (int)core_index,
            field_name,
            reason);
        return false;
    };
    const auto __fail_posxy_extnav = [&]() -> bool {
        dal.snprintf(
            failure_msg,
            failure_msg_len,
            "EKF3 core %d POSXY EXTNAV: %s",
            (int)core_index,
            tiltAlignComplete ? "stale" : "tilt unaligned");
        return false;
    };

    switch (posxy_source()) {
    case AP_NavEKF_Source::SourceXY::GPS:
        if (!__gps_ready_for_prearm()) {
            return __fail_gps_source("POSXY", false);
        }
        break;
    case AP_NavEKF_Source::SourceXY::BEACON:
        if (!readyToUseRangeBeacon()) {
            return __fail("POSXY");
        }
        break;
    case AP_NavEKF_Source::SourceXY::EXTNAV:
        if (!__extnav_posxy_ready_for_prearm()) {
            return __fail_posxy_extnav();
        }
        break;
    case AP_NavEKF_Source::SourceXY::NONE:
    case AP_NavEKF_Source::SourceXY::OPTFLOW:
    case AP_NavEKF_Source::SourceXY::WHEEL_ENCODER:
        break;
    }

    switch (velxy_source()) {
    case AP_NavEKF_Source::SourceXY::GPS:
        if (!__gps_ready_for_prearm()) {
            return __fail_gps_source("VELXY", false);
        }
        break;
    case AP_NavEKF_Source::SourceXY::OPTFLOW:
#if EK3_FEATURE_OPTFLOW_FUSION
        if (!readyToUseOptFlow()) {
            return __fail("VELXY");
        }
#else
        return __fail("VELXY");
#endif
        break;
    case AP_NavEKF_Source::SourceXY::EXTNAV:
        if (!(__extnav_vel_ready_for_prearm() || readyToUseBodyOdm())) {
            return __fail("VELXY");
        }
        break;
    case AP_NavEKF_Source::SourceXY::WHEEL_ENCODER:
        if (!readyToUseBodyOdm()) {
            return __fail("VELXY");
        }
        break;
    case AP_NavEKF_Source::SourceXY::NONE:
    case AP_NavEKF_Source::SourceXY::BEACON:
        break;
    }

    switch (posz_source()) {
    case AP_NavEKF_Source::SourceZ::BARO:
        if (!(dal.baro().healthy(selected_baro) &&
              (imuSampleTime_ms - lastBaroReceived_ms < BARO_PREARM_MAX_AGE_MS))) {
            return __fail("POSZ");
        }
        break;
    case AP_NavEKF_Source::SourceZ::RANGEFINDER:
        if (!rangeDataToFuse) {
            return __fail("POSZ");
        }
        break;
    case AP_NavEKF_Source::SourceZ::GPS:
        if (!__gps_ready_for_prearm()) {
            return __fail_gps_source("POSZ", false);
        }
        break;
    case AP_NavEKF_Source::SourceZ::BEACON:
#if EK3_FEATURE_BEACON_FUSION
        if (!(tiltAlignComplete && yawAlignComplete && delAngBiasLearned &&
              rngBcn.alignmentCompleted && rngBcn.dataToFuse)) {
            return __fail("POSZ");
        }
#else
        return __fail("POSZ");
#endif
        break;
    case AP_NavEKF_Source::SourceZ::EXTNAV:
        if (!(tiltAlignComplete && extNavPosRecent())) {
            return __fail("POSZ");
        }
        break;
    case AP_NavEKF_Source::SourceZ::NONE:
        break;
    }

    switch (velz_source()) {
    case AP_NavEKF_Source::SourceZ::GPS:
        if (!__gps_ready_for_prearm() || !gpsDataNew.have_vz) {
            return __fail_gps_source("VELZ", true);
        }
        break;
    case AP_NavEKF_Source::SourceZ::EXTNAV:
        if (!(__extnav_vel_ready_for_prearm() || readyToUseBodyOdm())) {
            return __fail("VELZ");
        }
        break;
    case AP_NavEKF_Source::SourceZ::NONE:
    case AP_NavEKF_Source::SourceZ::BARO:
    case AP_NavEKF_Source::SourceZ::RANGEFINDER:
    case AP_NavEKF_Source::SourceZ::BEACON:
        break;
    }

    switch (yaw_source()) {
    case AP_NavEKF_Source::SourceYaw::NONE:
        break;
    case AP_NavEKF_Source::SourceYaw::COMPASS:
        if (!have_aligned_yaw()) {
            return __fail("YAW");
        }
        break;
    case AP_NavEKF_Source::SourceYaw::GPS:
        if (!using_noncompass_for_yaw()) {
            return __fail("YAW");
        }
        break;
    case AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK:
        if (!have_aligned_yaw()) {
            return __fail("YAW");
        }
        break;
    case AP_NavEKF_Source::SourceYaw::EXTNAV:
        if (!using_extnav_for_yaw()) {
            return __fail("YAW");
        }
        break;
    case AP_NavEKF_Source::SourceYaw::GSF:
        if (!using_noncompass_for_yaw()) {
            return __fail("YAW");
        }
        break;
    }

    return true;
}

// return true if we should use the compass
bool NavEKF3_core::use_compass(void) const
{
    if ((yaw_source_last != AP_NavEKF_Source::SourceYaw::COMPASS) &&
        (yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK)) {
        // not using compass as a yaw source
        return false;
    }

    const auto &compass = dal.compass();
    return compass.use_for_yaw(magSelectIndex) &&
           !allMagSensorsFailed;
}

// are we using (aka fusing) a non-compass yaw?
bool NavEKF3_core::using_noncompass_for_yaw(void) const
{
#if EK3_FEATURE_EXTERNAL_NAV
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::EXTNAV) {
        return ((imuSampleTime_ms - last_extnav_yaw_fusion_ms < 5000) || (imuSampleTime_ms - lastSynthYawTime_ms < 5000));
    }
#endif
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS || yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK ||
        yaw_source_last == AP_NavEKF_Source::SourceYaw::GSF || !use_compass()) {
        return imuSampleTime_ms - last_gps_yaw_ms < 5000 || imuSampleTime_ms - lastSynthYawTime_ms < 5000;
    }
    return false;
}

// are we using (aka fusing) external nav for yaw?
bool NavEKF3_core::using_extnav_for_yaw() const
{
#if EK3_FEATURE_EXTERNAL_NAV
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::EXTNAV) {
        return ((imuSampleTime_ms - last_extnav_yaw_fusion_ms < 5000) || (imuSampleTime_ms - lastSynthYawTime_ms < 5000));
    }
#endif
    return false;
}

// are we using a gps
bool NavEKF3_core::using_gps() const
{
    return frontend->sources.usingGPS(core_index);
}

/*
  should we assume zero sideslip?
 */
bool NavEKF3_core::assume_zero_sideslip(void) const
{
    // we don't assume zero sideslip for ground vehicles as EKF could
    // be quite sensitive to a rapid spin of the ground vehicle if
    // traction is lost
    return dal.get_fly_forward() && dal.get_vehicle_class() != AP_DAL::VehicleClass::GROUND;
}

// sets the local NED origin using a LLH location (latitude, longitude, height)
// returns false if the origin is already set
bool NavEKF3_core::setOriginLLH(const Location &loc)
{
    // reject external origin setting until the filter has finished
    // bootstrap initialisation.  InitialiseVariables() resets
    // validOrigin, so an origin set before that point is lost.
    // Callers (e.g. AHRS use_recorded_origin_maybe) will retry.
    if (!statesInitialised) {
        return false;
    }
    return setOrigin(loc);
}

bool NavEKF3_core::accepts_external_origin(void) const
{
    return !uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS);
}

// populates the Earth magnetic field table using the given location
void NavEKF3_core::setEarthFieldFromLocation(const Location &loc)
{
    const auto &compass = dal.compass();
    if (compass.have_scale_factor(magSelectIndex) &&
        compass.auto_declination_enabled()) {
        getEarthFieldTable(loc);
    if (frontend->_mag_ef_limit > 0) {
            // initialise earth field from tables
            stateStruct.earth_magfield = table_earth_field_ga;
        }
    }
}

// sets the local NED origin using a LLH location (latitude, longitude, height)
// returns false is the origin has already been set
bool NavEKF3_core::setOrigin(const Location &loc)
{
    // if the origin is valid reject setting a new origin
    if (validOrigin) {
        return false;
    }

    EKF_origin = loc;
    ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    // define Earth rotation vector in the NED navigation frame at the origin
    calcEarthRateNED(earthRateNED, EKF_origin.lat);
    validOrigin = true;

    // but we do want to populate the WMM table even if we don't have a GPS at all
    if (!stateStruct.quat.is_zero()) {
        alignMagStateDeclination();
        setEarthFieldFromLocation(EKF_origin);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u origin set",(unsigned)imu_index);

    return true;
}

// record all requested yaw resets completed
void NavEKF3_core::recordYawResetsCompleted()
{
    gpsYawResetRequest = false;
    magYawResetRequest = false;
    yawAlignComplete = true;
    if (inFlight) {
        finalInflightYawInit = true;
    }
}

// set the class variable true if the delta angle bias variances are sufficiently small
void NavEKF3_core::checkGyroCalStatus(void)
{
    // check delta angle bias variances
    const ftype delAngBiasVarMax = sq(radians(0.15 * dtEkfAvg));
    if (!use_compass() && (yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS) && (yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK) &&
        (yaw_source_last != AP_NavEKF_Source::SourceYaw::EXTNAV)) {
        // rotate the variances into earth frame and evaluate horizontal terms only as yaw component is poorly observable without a yaw reference
        // which can make this check fail
        const Vector3F delAngBiasVarVec { P[10][10], P[11][11], P[12][12] };
        const Vector3F temp = prevTnb * delAngBiasVarVec;
        delAngBiasLearned = (fabsF(temp.x) < delAngBiasVarMax) &&
                            (fabsF(temp.y) < delAngBiasVarMax);
    } else {
        delAngBiasLearned = (P[10][10] <= delAngBiasVarMax) &&
                            (P[11][11] <= delAngBiasVarMax) &&
                            (P[12][12] <= delAngBiasVarMax);
    }
}

// Update the filter status
void  NavEKF3_core::updateFilterStatus(void)
{
    // init return value
    nav_filter_status status;
    status.value = 0;
    bool doingBodyVelNav = (PV_AidingMode != AID_NONE) && (imuSampleTime_ms - prevBodyVelFuseTime_ms < 5000);
    bool doingFlowNav = (PV_AidingMode != AID_NONE) && flowDataValid;
    bool doingWindRelNav = (!tasTimeout && assume_zero_sideslip()) || !dragTimeout;
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && (useGpsVertVel || useExtNavVel)) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout && dragTimeout) || doingFlowNav || doingBodyVelNav;
    bool filterHealthy = healthy() && tiltAlignComplete && (yawAlignComplete || (!use_compass() && (PV_AidingMode != AID_ABSOLUTE)));

    // If GPS height usage is specified, height is considered to be inaccurate until the GPS passes all checks
    bool hgtNotAccurate = uses_posz_source(AP_NavEKF_Source::SourceZ::GPS) && !validOrigin;

    // set individual flags
    status.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    status.flags.horiz_vel = someHorizRefData && filterHealthy;      // horizontal velocity estimate valid
    status.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid

#if EK3_FEATURE_OPTFLOW_SRTM
    const bool optflow_gnd_offset = gndOffsetValid || terrain_srtm_alt_valid;
#else
    const bool optflow_gnd_offset = gndOffsetValid;
#endif
    status.flags.horiz_pos_rel = ((doingFlowNav && optflow_gnd_offset) || doingWindRelNav || doingNormalGpsNav || doingBodyVelNav) && filterHealthy;   // relative horizontal position estimate valid

    status.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy; // absolute horizontal position estimate valid
    status.flags.vert_pos = !hgtTimeout && filterHealthy && !hgtNotAccurate; // vertical position estimate valid
    status.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;     // constant position mode
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_rel; // EKF3 enters the required mode before flight
    status.flags.pred_horiz_pos_abs = status.flags.horiz_pos_abs; // EKF3 enters the required mode before flight
    status.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    status.flags.takeoff = dal.get_takeoff_expected(); // The EKF has been told to expect takeoff is in a ground effect mitigation mode and has started the EKF-GSF yaw estimator
    status.flags.touchdown = dal.get_touchdown_expected(); // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    status.flags.using_gps = uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS) &&
                             ((imuSampleTime_ms - lastGpsPosPassTime_ms) < 4000) &&
                             (PV_AidingMode == AID_ABSOLUTE);
    status.flags.gps_glitching = !gpsAccuracyGood &&
                                 (PV_AidingMode == AID_ABSOLUTE) &&
                                 uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS); // GPS glitching is affecting navigation accuracy
    status.flags.gps_quality_good = gpsGoodToAlign;
    // for reporting purposes we report rejecting airspeed after 3s of not fusing when we want to fuse the data
    status.flags.rejecting_airspeed = lastTasFailTime_ms != 0 &&
                                            (imuSampleTime_ms - lastTasFailTime_ms) < 1000 &&
                                            (imuSampleTime_ms - lastTasPassTime_ms) > 3000;
    status.flags.initalized = status.flags.initalized || healthy();
    status.flags.dead_reckoning = (PV_AidingMode != AID_NONE) && doingWindRelNav && !((doingFlowNav && gndOffsetValid) || doingNormalGpsNav || doingBodyVelNav);

    filterStatus.value = status.value;
}

void NavEKF3_core::runYawEstimatorPrediction()
{
    // exit immediately if no yaw estimator
    if (yawEstimator == nullptr) {
        return;
    }

    // ensure GPS is used for horizontal position and velocity
    if (!uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS) ||
        !uses_velxy_source(AP_NavEKF_Source::SourceXY::GPS)) {
        return;
    }

    ftype trueAirspeed;
    if (tasDataDelayed.allowFusion && assume_zero_sideslip()) {
        trueAirspeed = MAX(tasDataDelayed.tas, 0.0f);
    } else {
        trueAirspeed = 0.0f;
    }
    yawEstimator->update(imuDataDelayed.delAng, imuDataDelayed.delVel, imuDataDelayed.delAngDT, imuDataDelayed.delVelDT, EKFGSF_run_filterbank, trueAirspeed);
}

void NavEKF3_core::runYawEstimatorCorrection()
{
    // exit immediately if no yaw estimator
    if (yawEstimator == nullptr) {
        return;
    }
    // ensure GPS is used for horizontal position and velocity
    if (!uses_posxy_source(AP_NavEKF_Source::SourceXY::GPS) ||
        !uses_velxy_source(AP_NavEKF_Source::SourceXY::GPS)) {
        return;
    }

    if (EKFGSF_run_filterbank) {
        if (gpsDataToFuse) {
            Vector2F gpsVelNE = Vector2F(gpsDataDelayed.vel.x, gpsDataDelayed.vel.y);
            ftype gpsVelAcc = fmaxF(gpsSpdAccuracy, ftype(frontend->_gpsHorizVelNoise));
            yawEstimator->fuseVelData(gpsVelNE, gpsVelAcc);

            // after velocity data has been fused the yaw variance estimate will have been refreshed and
            // is used maintain a history of validity
            ftype gsfYaw, gsfYawVariance;
            if (EKFGSF_getYaw(gsfYaw, gsfYawVariance)) {
                if (EKFGSF_yaw_valid_count <  GSF_YAW_VALID_HISTORY_THRESHOLD) {
                    EKFGSF_yaw_valid_count++;
                }
            } else {
                EKFGSF_yaw_valid_count = 0;
            }
        }

        // action an external reset request
        if (EKFGSF_yaw_reset_request_ms > 0 && imuSampleTime_ms - EKFGSF_yaw_reset_request_ms < YAW_RESET_TO_GSF_TIMEOUT_MS) {
            EKFGSF_resetMainFilterYaw(true);
        }
    } else {
        EKFGSF_yaw_valid_count = 0;
    }
}

// request a reset the yaw to the GSF estimate
// request times out after YAW_RESET_TO_GSF_TIMEOUT_MS if it cannot be actioned
void NavEKF3_core::EKFGSF_requestYawReset()
{
    EKFGSF_yaw_reset_request_ms = imuSampleTime_ms;
}
