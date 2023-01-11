#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>

#include "AP_DAL/AP_DAL.h"

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
            const bool haveAirspeedMeasurement = usingDefaultAirspeed || (tasDataDelayed.allowFusion && (imuDataDelayed.time_ms - tasDataDelayed.time_ms < 500) && useAirspeed());
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

// Set inertial navigation aiding mode
void NavEKF3_core::setAidingMode()
{
    resetDataSource posResetSource = resetDataSource::DEFAULT;
    resetDataSource velResetSource = resetDataSource::DEFAULT;

    // Save the previous status so we can detect when it has changed
    PV_AidingModePrev = PV_AidingMode;

    // Check that the gyro bias variance has converged
    checkGyroCalStatus();

    // Handle the special case where we are on ground and disarmed without a yaw measurement
    // and navigating. This can occur if not using a magnetometer and yaw was aligned using GPS
    // during the previous flight.
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::NONE &&
        !motorsArmed &&
        onGround &&
        PV_AidingMode != AID_NONE)
    {
        PV_AidingMode = AID_NONE;
        yawAlignComplete = false;
        finalInflightYawInit = false;
        ResetVelocity(resetDataSource::DEFAULT);
        ResetPosition(resetDataSource::DEFAULT);
        ResetHeight();
        // preserve quaternion 4x4 covariances, but zero the other rows and columns
        for (uint8_t row=0; row<4; row++) {
            for (uint8_t col=4; col<24; col++) {
                P[row][col] = 0.0f;
            }
        }
        for (uint8_t col=0; col<4; col++) {
            for (uint8_t row=4; row<24; row++) {
                P[row][col] = 0.0f;
            }
        }
        // keep the IMU bias state variances, but zero the covariances
        ftype oldBiasVariance[6];
        for (uint8_t row=0; row<6; row++) {
            oldBiasVariance[row] = P[row+10][row+10];
        }
        zeroCols(P,10,15);
        zeroRows(P,10,15);
        for (uint8_t row=0; row<6; row++) {
            P[row+10][row+10] = oldBiasVariance[row];
        }
    }

    // Determine if we should change aiding mode
    switch (PV_AidingMode) {
        case AID_NONE: {
            // Don't allow filter to start position or velocity aiding until the tilt and yaw alignment is complete
            // and IMU gyro bias estimates have stabilised
            // If GPS usage has been prohiited then we use flow aiding provided optical flow data is present
            // GPS aiding is the preferred option unless excluded by the user
            if (readyToUseGPS() || readyToUseRangeBeacon() || readyToUseExtNav()) {
                PV_AidingMode = AID_ABSOLUTE;
            } else if (readyToUseOptFlow() || readyToUseBodyOdm()) {
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
            const bool rngBcnUsed = (imuSampleTime_ms - lastRngBcnPassTime_ms <= minTestTime_ms);
#else
            const bool rngBcnUsed = false;
#endif

            // Check if GPS or external nav is being used
            bool posUsed = (imuSampleTime_ms - lastPosPassTime_ms <= minTestTime_ms);
            bool gpsVelUsed = (imuSampleTime_ms - lastVelPassTime_ms <= minTestTime_ms);

            // Check if attitude drift has been constrained by a measurement source
            bool attAiding = posUsed || gpsVelUsed || optFlowUsed || airSpdUsed || dragUsed || rngBcnUsed || bodyOdmUsed;

            // check if velocity drift has been constrained by a measurement source
            bool velAiding = gpsVelUsed || airSpdUsed || dragUsed || optFlowUsed || bodyOdmUsed;

            // check if position drift has been constrained by a measurement source
            bool posAiding = posUsed || rngBcnUsed;

            // Check if the loss of attitude aiding has become critical
            bool attAidLossCritical = false;
            if (!attAiding) {
            	attAidLossCritical = (imuSampleTime_ms - prevFlowFuseTime_ms > frontend->tiltDriftTimeMax_ms) &&
                		(imuSampleTime_ms - lastTasPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
#if EK3_FEATURE_BEACON_FUSION
                        (imuSampleTime_ms - lastRngBcnPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
#endif
                        (imuSampleTime_ms - lastPosPassTime_ms > frontend->tiltDriftTimeMax_ms) &&
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
                    (imuSampleTime_ms - lastRngBcnPassTime_ms > maxLossTime_ms) &&
#endif
                    (imuSampleTime_ms - lastPosPassTime_ms > maxLossTime_ms);
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
            if (readyToUseOptFlow()) {
                // Reset time stamps
                flowValidMeaTime_ms = imuSampleTime_ms;
                prevFlowFuseTime_ms = imuSampleTime_ms;
            } else if (readyToUseBodyOdm()) {
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
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial pos NE = %3.1f,%3.1f (m)",(unsigned)imu_index,(double)receiverPos.x,(double)receiverPos.y);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u initial beacon pos D offset = %3.1f (m)",(unsigned)imu_index,(double)bcnPosOffsetNED.z);
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
                hgtMea = -extNavDataDelayed.pos.z;
                posDownObsNoise = sq(constrain_ftype(extNavDataDelayed.posErr, 0.1f, 10.0f));
                ResetHeight();
#endif // EK3_FEATURE_EXTERNAL_NAV
            }

            // clear timeout flags as a precaution to avoid triggering any additional transitions
            posTimeout = false;
            velTimeout = false;

            // reset the last fusion accepted times to prevent unwanted activation of timeout logic
            lastPosPassTime_ms = imuSampleTime_ms;
            lastVelPassTime_ms = imuSampleTime_ms;
#if EK3_FEATURE_BEACON_FUSION
            lastRngBcnPassTime_ms = imuSampleTime_ms;
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

// return true if the filter is ready to start using optical flow measurements
bool NavEKF3_core::readyToUseOptFlow(void) const
{
    // ensure flow is used for navigation and not terrain alt estimation
    if (frontend->_flowUse != FLOW_USE_NAV) {
        return false;
    }

    if (!frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::OPTFLOW)) {
        return false;
    }

    // We need stable roll/pitch angles and gyro bias estimates but do not need the yaw angle aligned to use optical flow
    return (imuSampleTime_ms - flowMeaTime_ms < 200) && tiltAlignComplete && delAngBiasLearned;
}

// return true if the filter is ready to start using body frame odometry measurements
bool NavEKF3_core::readyToUseBodyOdm(void) const
{
#if EK3_FEATURE_BODY_ODOM
    if (!frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV) &&
        !frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::WHEEL_ENCODER)) {
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
    if (frontend->sources.getPosXYSource() != AP_NavEKF_Source::SourceXY::GPS) {
        return false;
    }

    return validOrigin && tiltAlignComplete && yawAlignComplete && (delAngBiasLearned || assume_zero_sideslip()) && gpsGoodToAlign && gpsDataToFuse;
}

// return true if the filter to be ready to use the beacon range measurements
bool NavEKF3_core::readyToUseRangeBeacon(void) const
{
#if EK3_FEATURE_BEACON_FUSION
    if (frontend->sources.getPosXYSource() != AP_NavEKF_Source::SourceXY::BEACON) {
        return false;
    }

    return tiltAlignComplete && yawAlignComplete && delAngBiasLearned && rngBcnAlignmentCompleted && rngBcnDataToFuse;
#else
    return false;
#endif  // EK3_FEATURE_BEACON_FUSION
}

// return true if the filter is ready to use external nav data
bool NavEKF3_core::readyToUseExtNav(void) const
{
#if EK3_FEATURE_EXTERNAL_NAV
    if (frontend->sources.getPosXYSource() != AP_NavEKF_Source::SourceXY::EXTNAV) {
        return false;
    }

    return tiltAlignComplete && extNavDataToFuse;
#else
    return false;
#endif // EK3_FEATURE_EXTERNAL_NAV
}

// return true if we should use the compass
bool NavEKF3_core::use_compass(void) const
{
    const AP_NavEKF_Source::SourceYaw yaw_source = frontend->sources.getYawSource();
    if ((yaw_source != AP_NavEKF_Source::SourceYaw::COMPASS) &&
        (yaw_source != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK)) {
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
    const AP_NavEKF_Source::SourceYaw yaw_source = frontend->sources.getYawSource();
#if EK3_FEATURE_EXTERNAL_NAV
    if (yaw_source == AP_NavEKF_Source::SourceYaw::EXTNAV) {
        return ((imuSampleTime_ms - last_extnav_yaw_fusion_ms < 5000) || (imuSampleTime_ms - lastSynthYawTime_ms < 5000));
    }
#endif
    if (yaw_source == AP_NavEKF_Source::SourceYaw::GPS || yaw_source == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK ||
        yaw_source == AP_NavEKF_Source::SourceYaw::GSF || !use_compass()) {
        return imuSampleTime_ms - last_gps_yaw_ms < 5000 || imuSampleTime_ms - lastSynthYawTime_ms < 5000;
    }
    return false;
}

// are we using (aka fusing) external nav for yaw?
bool NavEKF3_core::using_extnav_for_yaw() const
{
#if EK3_FEATURE_EXTERNAL_NAV
    if (frontend->sources.getYawSource() == AP_NavEKF_Source::SourceYaw::EXTNAV) {
        return ((imuSampleTime_ms - last_extnav_yaw_fusion_ms < 5000) || (imuSampleTime_ms - lastSynthYawTime_ms < 5000));
    }
#endif
    return false;
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
// returns false if absolute aiding and GPS is being used or if the origin is already set
bool NavEKF3_core::setOriginLLH(const Location &loc)
{
    if ((PV_AidingMode == AID_ABSOLUTE) && (frontend->sources.getPosXYSource() == AP_NavEKF_Source::SourceXY::GPS)) {
        // reject attempts to set the origin if GPS is being used or if the origin is already set
        return false;
    }

    return setOrigin(loc);
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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u origin set",(unsigned)imu_index);

    if (!frontend->common_origin_valid) {
        frontend->common_origin_valid = true;
        // put origin in frontend as well to ensure it stays in sync between lanes
        public_origin = EKF_origin;
    }


    return true;
}

// record a yaw reset event
void NavEKF3_core::recordYawReset()
{
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
    const AP_NavEKF_Source::SourceYaw yaw_source = frontend->sources.getYawSource();
    if (!use_compass() && (yaw_source != AP_NavEKF_Source::SourceYaw::GPS) && (yaw_source != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK) &&
        (yaw_source != AP_NavEKF_Source::SourceYaw::EXTNAV)) {
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
    filterStatus.value = 0;
    bool doingBodyVelNav = (PV_AidingMode != AID_NONE) && (imuSampleTime_ms - prevBodyVelFuseTime_ms < 5000);
    bool doingFlowNav = (PV_AidingMode != AID_NONE) && flowDataValid;
    bool doingWindRelNav = (!tasTimeout && assume_zero_sideslip()) || !dragTimeout;
    bool doingNormalGpsNav = !posTimeout && (PV_AidingMode == AID_ABSOLUTE);
    bool someVertRefData = (!velTimeout && (useGpsVertVel || useExtNavVel)) || !hgtTimeout;
    bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout && dragTimeout) || doingFlowNav || doingBodyVelNav;
    bool filterHealthy = healthy() && tiltAlignComplete && (yawAlignComplete || (!use_compass() && (PV_AidingMode != AID_ABSOLUTE)));

    // If GPS height usage is specified, height is considered to be inaccurate until the GPS passes all checks
    bool hgtNotAccurate = (frontend->sources.getPosZSource() == AP_NavEKF_Source::SourceZ::GPS) && !validOrigin;

    // set individual flags
    filterStatus.flags.attitude = !stateStruct.quat.is_nan() && filterHealthy;   // attitude valid (we need a better check)
    filterStatus.flags.horiz_vel = someHorizRefData && filterHealthy;      // horizontal velocity estimate valid
    filterStatus.flags.vert_vel = someVertRefData && filterHealthy;        // vertical velocity estimate valid
    filterStatus.flags.horiz_pos_rel = ((doingFlowNav && gndOffsetValid) || doingWindRelNav || doingNormalGpsNav || doingBodyVelNav) && filterHealthy;   // relative horizontal position estimate valid
    filterStatus.flags.horiz_pos_abs = doingNormalGpsNav && filterHealthy; // absolute horizontal position estimate valid
    filterStatus.flags.vert_pos = !hgtTimeout && filterHealthy && !hgtNotAccurate; // vertical position estimate valid
    filterStatus.flags.terrain_alt = gndOffsetValid && filterHealthy;		// terrain height estimate valid
    filterStatus.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;     // constant position mode
    filterStatus.flags.pred_horiz_pos_rel = filterStatus.flags.horiz_pos_rel; // EKF3 enters the required mode before flight
    filterStatus.flags.pred_horiz_pos_abs = filterStatus.flags.horiz_pos_abs; // EKF3 enters the required mode before flight
    filterStatus.flags.takeoff_detected = takeOffDetected; // takeoff for optical flow navigation has been detected
    filterStatus.flags.takeoff = dal.get_takeoff_expected(); // The EKF has been told to expect takeoff is in a ground effect mitigation mode and has started the EKF-GSF yaw estimator
    filterStatus.flags.touchdown = dal.get_touchdown_expected(); // The EKF has been told to detect touchdown and is in a ground effect mitigation mode
    filterStatus.flags.using_gps = ((imuSampleTime_ms - lastPosPassTime_ms) < 4000) && (PV_AidingMode == AID_ABSOLUTE);
    filterStatus.flags.gps_glitching = !gpsAccuracyGood && (PV_AidingMode == AID_ABSOLUTE) && (frontend->sources.getPosXYSource() == AP_NavEKF_Source::SourceXY::GPS); // GPS glitching is affecting navigation accuracy
    filterStatus.flags.gps_quality_good = gpsGoodToAlign;
    // for reporting purposes we report rejecting airspeed after 3s of not fusing when we want to fuse the data
    filterStatus.flags.rejecting_airspeed = lastTasFailTime_ms != 0 &&
                                            (imuSampleTime_ms - lastTasFailTime_ms) < 1000 &&
                                            (imuSampleTime_ms - lastTasPassTime_ms) > 3000;
    filterStatus.flags.initalized = filterStatus.flags.initalized || healthy();
    filterStatus.flags.dead_reckoning = (PV_AidingMode != AID_NONE) && doingWindRelNav && !((doingFlowNav && gndOffsetValid) || doingNormalGpsNav || doingBodyVelNav);
}

void NavEKF3_core::runYawEstimatorPrediction()
{
    // exit immediately if no yaw estimator
    if (yawEstimator == nullptr) {
        return;
    }

    // ensure GPS is used for horizontal position and velocity
    if (frontend->sources.getPosXYSource() != AP_NavEKF_Source::SourceXY::GPS ||
        !frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS)) {
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
    if (frontend->sources.getPosXYSource() != AP_NavEKF_Source::SourceXY::GPS ||
        !frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS)) {
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
