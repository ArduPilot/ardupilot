#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF2_core::controlMagYawReset()
{

    // Vehicles that can use a zero sideslip assumption (Planes) are a special case
    // They can use the GPS velocity to recover from bad initial compass data
    // This allows recovery for heading alignment errors due to compass faults
    if (assume_zero_sideslip() && !finalInflightYawInit && inFlight ) {
        gpsYawResetRequest = true;
        return;
    } else {
        gpsYawResetRequest = false;
    }

    // QuaternionF and delta rotation vector that are re-used for different calculations
    Vector3F deltaRotVecTemp;
    QuaternionF deltaQuatTemp;

    bool flightResetAllowed = false;
    bool initialResetAllowed = false;
    if (!finalInflightYawInit) {
        // Use a quaternion division to calculate the delta quaternion between the rotation at the current and last time
        deltaQuatTemp = stateStruct.quat / prevQuatMagReset;
        prevQuatMagReset = stateStruct.quat;

        // convert the quaternion to a rotation vector and find its length
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);

        // check if the spin rate is OK - high spin rates can cause angular alignment errors
        bool angRateOK = deltaRotVecTemp.length() < 0.1745f;

        initialResetAllowed = angRateOK;
        flightResetAllowed = angRateOK && !onGround;

    }

    // reset the limit on the number of magnetic anomaly resets for each takeoff
    if (onGround) {
        magYawAnomallyCount = 0;
    }

    // Check if conditions for a interim or final yaw/mag reset are met
    bool finalResetRequest = false;
    bool interimResetRequest = false;
    if (flightResetAllowed && !assume_zero_sideslip()) {
        // check that we have reached a height where ground magnetic interference effects are insignificant
        // and can perform a final reset of the yaw and field states
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) < -EKF2_MAG_FINAL_RESET_ALT;

        // check for increasing height
        bool hgtIncreasing = (posDownAtLastMagReset-stateStruct.position.z) > 0.5f;
        ftype yawInnovIncrease = fabsF(innovYaw) - fabsF(yawInnovAtLastMagReset);

        // check for increasing yaw innovations
        bool yawInnovIncreasing = yawInnovIncrease > 0.25f;

        // check that the yaw innovations haven't been caused by a large change in attitude
        deltaQuatTemp = quatAtLastMagReset / stateStruct.quat;
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);
        bool largeAngleChange = deltaRotVecTemp.length() > yawInnovIncrease;

        // if yaw innovations and height have increased and we haven't rotated much
        // then we are climbing away from a ground based magnetic anomaly and need to reset
        interimResetRequest = !finalInflightYawInit
                                && !finalResetRequest
                                && (magYawAnomallyCount < MAG_ANOMALY_RESET_MAX)
                                && hgtIncreasing
                                && yawInnovIncreasing
                                && !largeAngleChange;
    }

    // an initial reset is required if we have not yet aligned the yaw angle
    bool initialResetRequest = initialResetAllowed && !yawAlignComplete;

    // a combined yaw angle and magnetic field reset can be initiated by:
    magYawResetRequest = magYawResetRequest || // an external request
            initialResetRequest || // an initial alignment performed by all vehicle types using magnetometer
            interimResetRequest || // an interim alignment required to recover from ground based magnetic anomaly
            finalResetRequest; // the final reset when we have acheived enough height to be in stable magnetic field environment

    // Perform a reset of magnetic field states and reset yaw to corrected magnetic heading
    if (magYawResetRequest || magStateResetRequest || extNavYawResetRequest) {

        // if a yaw reset has been requested, apply the updated quaternion to the current state
        if (extNavYawResetRequest) {
            // get the euler angles from the current state estimate
            Vector3F eulerAnglesOld;
            stateStruct.quat.to_euler(eulerAnglesOld.x, eulerAnglesOld.y, eulerAnglesOld.z);

            // previous value used to calculate a reset delta
            QuaternionF prevQuat = stateStruct.quat;

            // Get the Euler angles from the external vision data
            Vector3F eulerAnglesNew;
            extNavDataDelayed.quat.to_euler(eulerAnglesNew.x, eulerAnglesNew.y, eulerAnglesNew.z);

            // the new quaternion uses the old roll/pitch and new yaw angle
            stateStruct.quat.from_euler(eulerAnglesOld.x, eulerAnglesOld.y, eulerAnglesNew.z);

            // calculate the change in the quaternion state and apply it to the output history buffer
            prevQuat = stateStruct.quat/prevQuat;
            StoreQuatRotate(prevQuat);

            // send initial alignment status to console
            if (!yawAlignComplete) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u ext nav yaw alignment complete",(unsigned)imu_index);
            }

            // record the reset as complete and also record the in-flight reset as complete to stop further resets when height is gained
            // in-flight reset is unnecessary because we do not need to consider ground based magnetic anomaly effects
            yawAlignComplete = true;
            finalInflightYawInit = true;

            // clear the yaw reset request flag
            extNavYawResetRequest = false;

        } else if (magYawResetRequest || magStateResetRequest) {
            // get the euler angles from the current state estimate
            Vector3F eulerAngles;
            stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

            // Use the Euler angles and magnetometer measurement to update the magnetic field states
            // and get an updated quaternion
            QuaternionF newQuat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

            if (magYawResetRequest) {
                // previous value used to calculate a reset delta
                QuaternionF prevQuat = stateStruct.quat;

                // update the quaternion states using the new yaw angle
                stateStruct.quat = newQuat;

                // calculate the change in the quaternion state and apply it to the ouput history buffer
                prevQuat = stateStruct.quat/prevQuat;
                StoreQuatRotate(prevQuat);

                // send initial alignment status to console
                if (!yawAlignComplete) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u MAG%u initial yaw alignment complete",(unsigned)imu_index, (unsigned)magSelectIndex);
                }

                // send in-flight yaw alignment status to console
                if (finalResetRequest) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u MAG%u in-flight yaw alignment complete",(unsigned)imu_index, (unsigned)magSelectIndex);
                } else if (interimResetRequest) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF2 IMU%u MAG%u ground mag anomaly, yaw re-aligned",(unsigned)imu_index, (unsigned)magSelectIndex);
                }

                // update the yaw reset completed status
                recordYawReset();

                // clear the yaw reset request flag
                magYawResetRequest = false;

                // clear the complete flags if an interim reset has been performed to allow subsequent
                // and final reset to occur
                if (interimResetRequest) {
                    finalInflightYawInit = false;
                    finalInflightMagInit = false;
                }
            }
        }
    }
}

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF2_core::realignYawGPS()
{
    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        Vector3F eulerAngles;
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

        // calculate course yaw angle
        ftype velYaw = atan2F(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        ftype gpsYaw = atan2F(gpsDataDelayed.vel.y,gpsDataDelayed.vel.x);

        // Check the yaw angles for consistency
        ftype yawErr = MAX(fabsF(wrap_PI(gpsYaw - velYaw)),fabsF(wrap_PI(gpsYaw - eulerAngles.z)));

        // If the angles disagree by more than 45 degrees and GPS innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
        bool badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f) && (PV_AidingMode == AID_ABSOLUTE)) || !yawAlignComplete;

        // correct yaw angle using GPS ground course if compass yaw bad
        if (badMagYaw) {

            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, gpsYaw);
            // reset the velocity and position states as they will be inaccurate due to bad yaw
            ResetVelocity();
            ResetPosition();

            // send yaw alignment information to console
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u yaw aligned to GPS velocity",(unsigned)imu_index);

            // zero the attitude covariances because the correlations will now be invalid
            zeroAttCovOnly();

            // record the yaw reset event
            recordYawReset();

            // clear all pending yaw reset requests
            gpsYawResetRequest = false;
            magYawResetRequest = false;

            if (use_compass()) {
                // request a mag field reset which may enable us to use the magnetometer if the previous fault was due to bad initialisation
                magStateResetRequest = true;
                // clear the all sensors failed status so that the magnetometers sensors get a second chance now that we are flying
                allMagSensorsFailed = false;
            }
        }
    }
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF2_core::SelectMagFusion()
{
    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been performed on that time step
    // used for load levelling
    magFusePerformed = false;

    // Handle case where we are not using a yaw sensor of any type and and attempt to reset the yaw in
    // flight using the output from the GSF yaw estimator.
    if (!use_compass() && tiltAlignComplete) {
        if ((onGround || !assume_zero_sideslip()) && (imuSampleTime_ms - lastYawTime_ms > 140)) {
            fuseEulerYaw();
        }
        if (yawAlignComplete) {
            return;
        }
        yawAlignComplete = EKFGSF_resetMainFilterYaw();
        return;
    }

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    // check for and read new magnetometer measurements
    readMagData();

    // check for availability of magnetometer data to fuse
    magDataToFuse = storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);

    // Control reset of yaw and magnetic field states if we are using compass data
    if (magDataToFuse && use_compass()) {
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // use the simple method of declination to maintain heading if we cannot use the magnetic field states
        if(inhibitMagStates || magStateResetRequest || !magStateInitComplete) {
            fuseEulerYaw();
            // zero the test ratio output from the inactive 3-axis magnetometer fusion
            magTestRatio.zero();
        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            // We also fuse declination if we are using the WMM tables
            if (PV_AidingMode != AID_ABSOLUTE ||
                (frontend->_mag_ef_limit > 0 && have_table_earth_field)) {
                FuseDeclination(0.34f);
            }

            // fuse the three magnetometer componenents using sequential fusion of each axis
            FuseMagnetometer();

            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // If the final yaw reset has been performed and the state variances are sufficiently low
    // record that the earth field has been learned.
    if (!magFieldLearned && finalInflightMagInit) {
        magFieldLearned = (P[16][16] < sq(0.01f)) && (P[17][17] < sq(0.01f)) && (P[18][18] < sq(0.01f));
    }

    // record the last learned field variances
    if (magFieldLearned && !inhibitMagStates) {
        earthMagFieldVar.x = P[16][16];
        earthMagFieldVar.y = P[17][17];
        earthMagFieldVar.z = P[18][18];
        bodyMagFieldVar.x = P[19][19];
        bodyMagFieldVar.y = P[20][20];
        bodyMagFieldVar.z = P[21][21];
    }
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseMagnetometer()
{
    // declarations
    ftype &q0 = mag_state.q0;
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    Matrix3F &DCM = mag_state.DCM;
    Vector3F &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector24 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    // copy required states to local variable names
    q0       = stateStruct.quat[0];
    q1       = stateStruct.quat[1];
    q2       = stateStruct.quat[2];
    q3       = stateStruct.quat[3];
    magN     = stateStruct.earth_magfield[0];
    magE     = stateStruct.earth_magfield[1];
    magD     = stateStruct.earth_magfield[2];
    magXbias = stateStruct.body_magfield[0];
    magYbias = stateStruct.body_magfield[1];
    magZbias = stateStruct.body_magfield[2];

    // rotate predicted earth components into body axes and calculate
    // predicted measurements
    DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    DCM[0][1] = 2.0f*(q1*q2 + q0*q3);
    DCM[0][2] = 2.0f*(q1*q3-q0*q2);
    DCM[1][0] = 2.0f*(q1*q2 - q0*q3);
    DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    DCM[1][2] = 2.0f*(q2*q3 + q0*q1);
    DCM[2][0] = 2.0f*(q1*q3 + q0*q2);
    DCM[2][1] = 2.0f*(q2*q3 - q0*q1);
    DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
    MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
    MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

    // calculate the measurement innovation for each axis
    for (uint8_t i = 0; i<=2; i++) {
        innovMag[i] = MagPred[i] - magDataDelayed.mag[i];
    }

    // scale magnetometer observation error with total angular rate to allow for timing errors
    R_MAG = sq(constrain_ftype(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*delAngCorrected.length() / imuDataDelayed.delAngDT);

    // calculate common expressions used to calculate observation jacobians an innovation variance for each component
    SH_MAG[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
    SH_MAG[1] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
    SH_MAG[2] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    SH_MAG[3] = 2.0f*q0*q1 + 2.0f*q2*q3;
    SH_MAG[4] = 2.0f*q0*q3 + 2.0f*q1*q2;
    SH_MAG[5] = 2.0f*q0*q2 + 2.0f*q1*q3;
    SH_MAG[6] = magE*(2.0f*q0*q1 - 2.0f*q2*q3);
    SH_MAG[7] = 2.0f*q1*q3 - 2.0f*q0*q2;
    SH_MAG[8] = 2.0f*q0*q3;

    // Calculate the innovation variance for each axis
    // X axis
    varInnovMag[0] = (P[19][19] + R_MAG - P[1][19]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][19]*SH_MAG[1] + P[17][19]*SH_MAG[4] + P[18][19]*SH_MAG[7] + P[2][19]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) - (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[19][1] - P[1][1]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][1]*SH_MAG[1] + P[17][1]*SH_MAG[4] + P[18][1]*SH_MAG[7] + P[2][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[1]*(P[19][16] - P[1][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][16]*SH_MAG[1] + P[17][16]*SH_MAG[4] + P[18][16]*SH_MAG[7] + P[2][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[4]*(P[19][17] - P[1][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][17]*SH_MAG[1] + P[17][17]*SH_MAG[4] + P[18][17]*SH_MAG[7] + P[2][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[7]*(P[19][18] - P[1][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][18]*SH_MAG[1] + P[17][18]*SH_MAG[4] + P[18][18]*SH_MAG[7] + P[2][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[19][2] - P[1][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][2]*SH_MAG[1] + P[17][2]*SH_MAG[4] + P[18][2]*SH_MAG[7] + P[2][2]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))));
    if (varInnovMag[0] >= R_MAG) {
        faultStatus.bad_xmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_xmag = true;
        return;
    }

    // Y axis
    varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][20]*SH_MAG[0] + P[18][20]*SH_MAG[3] - (SH_MAG[8] - 2.0f*q1*q2)*(P[20][16] + P[0][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][16]*SH_MAG[0] + P[18][16]*SH_MAG[3] - P[2][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][16]*(SH_MAG[8] - 2.0f*q1*q2)) - P[2][20]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[20][0] + P[0][0]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][0]*SH_MAG[0] + P[18][0]*SH_MAG[3] - P[2][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][0]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[0]*(P[20][17] + P[0][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][17]*SH_MAG[0] + P[18][17]*SH_MAG[3] - P[2][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][17]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[3]*(P[20][18] + P[0][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][18]*SH_MAG[0] + P[18][18]*SH_MAG[3] - P[2][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][18]*(SH_MAG[8] - 2.0f*q1*q2)) - P[16][20]*(SH_MAG[8] - 2.0f*q1*q2) - (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[20][2] + P[0][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][2]*SH_MAG[0] + P[18][2]*SH_MAG[3] - P[2][2]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][2]*(SH_MAG[8] - 2.0f*q1*q2)));
    if (varInnovMag[1] >= R_MAG) {
        faultStatus.bad_ymag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_ymag = true;
        return;
    }

    // Z axis
    varInnovMag[2] = (P[21][21] + R_MAG + P[16][21]*SH_MAG[5] + P[18][21]*SH_MAG[2] - (2.0f*q0*q1 - 2.0f*q2*q3)*(P[21][17] + P[16][17]*SH_MAG[5] + P[18][17]*SH_MAG[2] - P[0][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][17]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[0][21]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][21]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + SH_MAG[5]*(P[21][16] + P[16][16]*SH_MAG[5] + P[18][16]*SH_MAG[2] - P[0][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][16]*(2.0f*q0*q1 - 2.0f*q2*q3)) + SH_MAG[2]*(P[21][18] + P[16][18]*SH_MAG[5] + P[18][18]*SH_MAG[2] - P[0][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][18]*(2.0f*q0*q1 - 2.0f*q2*q3)) - (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[21][0] + P[16][0]*SH_MAG[5] + P[18][0]*SH_MAG[2] - P[0][0]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][0]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[17][21]*(2.0f*q0*q1 - 2.0f*q2*q3) + (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[21][1] + P[16][1]*SH_MAG[5] + P[18][1]*SH_MAG[2] - P[0][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][1]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][1]*(2.0f*q0*q1 - 2.0f*q2*q3)));
    if (varInnovMag[2] >= R_MAG) {
        faultStatus.bad_zmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        faultStatus.bad_zmag = true;
        return;
    }

    // calculate the innovation test ratios
    for (uint8_t i = 0; i<=2; i++) {
        magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (ftype)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);
    }

    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);

    // if the magnetometer is unhealthy, do not proceed further
    if (!magHealth) {
        return;
    }
    
    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains
    for (uint8_t obsIndex = 0; obsIndex <= 2; obsIndex++) {
        if (obsIndex == 0)
        {
            // calculate observation jacobians
            ZERO_FARRAY(H_MAG);
            H_MAG[1] = SH_MAG[6] - magD*SH_MAG[2] - magN*SH_MAG[5];
            H_MAG[2] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
            H_MAG[16] = SH_MAG[1];
            H_MAG[17] = SH_MAG[4];
            H_MAG[18] = SH_MAG[7];
            H_MAG[19] = 1.0f;

            // calculate Kalman gain
            SK_MX[0] = 1.0f / varInnovMag[0];
            SK_MX[1] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
            SK_MX[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
            SK_MX[3] = SH_MAG[7];
            Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][16]*SH_MAG[1] + P[0][17]*SH_MAG[4] - P[0][1]*SK_MX[2] + P[0][2]*SK_MX[1] + P[0][18]*SK_MX[3]);
            Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][16]*SH_MAG[1] + P[1][17]*SH_MAG[4] - P[1][1]*SK_MX[2] + P[1][2]*SK_MX[1] + P[1][18]*SK_MX[3]);
            Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][16]*SH_MAG[1] + P[2][17]*SH_MAG[4] - P[2][1]*SK_MX[2] + P[2][2]*SK_MX[1] + P[2][18]*SK_MX[3]);
            Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][16]*SH_MAG[1] + P[3][17]*SH_MAG[4] - P[3][1]*SK_MX[2] + P[3][2]*SK_MX[1] + P[3][18]*SK_MX[3]);
            Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][16]*SH_MAG[1] + P[4][17]*SH_MAG[4] - P[4][1]*SK_MX[2] + P[4][2]*SK_MX[1] + P[4][18]*SK_MX[3]);
            Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][16]*SH_MAG[1] + P[5][17]*SH_MAG[4] - P[5][1]*SK_MX[2] + P[5][2]*SK_MX[1] + P[5][18]*SK_MX[3]);
            Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][16]*SH_MAG[1] + P[6][17]*SH_MAG[4] - P[6][1]*SK_MX[2] + P[6][2]*SK_MX[1] + P[6][18]*SK_MX[3]);
            Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][16]*SH_MAG[1] + P[7][17]*SH_MAG[4] - P[7][1]*SK_MX[2] + P[7][2]*SK_MX[1] + P[7][18]*SK_MX[3]);
            Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][16]*SH_MAG[1] + P[8][17]*SH_MAG[4] - P[8][1]*SK_MX[2] + P[8][2]*SK_MX[1] + P[8][18]*SK_MX[3]);
            Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][16]*SH_MAG[1] + P[9][17]*SH_MAG[4] - P[9][1]*SK_MX[2] + P[9][2]*SK_MX[1] + P[9][18]*SK_MX[3]);
            Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][16]*SH_MAG[1] + P[10][17]*SH_MAG[4] - P[10][1]*SK_MX[2] + P[10][2]*SK_MX[1] + P[10][18]*SK_MX[3]);
            Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][16]*SH_MAG[1] + P[11][17]*SH_MAG[4] - P[11][1]*SK_MX[2] + P[11][2]*SK_MX[1] + P[11][18]*SK_MX[3]);
            Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][16]*SH_MAG[1] + P[12][17]*SH_MAG[4] - P[12][1]*SK_MX[2] + P[12][2]*SK_MX[1] + P[12][18]*SK_MX[3]);
            Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][16]*SH_MAG[1] + P[13][17]*SH_MAG[4] - P[13][1]*SK_MX[2] + P[13][2]*SK_MX[1] + P[13][18]*SK_MX[3]);
            Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][16]*SH_MAG[1] + P[14][17]*SH_MAG[4] - P[14][1]*SK_MX[2] + P[14][2]*SK_MX[1] + P[14][18]*SK_MX[3]);
            Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][16]*SH_MAG[1] + P[15][17]*SH_MAG[4] - P[15][1]*SK_MX[2] + P[15][2]*SK_MX[1] + P[15][18]*SK_MX[3]);

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MX[0]*(P[22][19] + P[22][16]*SH_MAG[1] + P[22][17]*SH_MAG[4] - P[22][1]*SK_MX[2] + P[22][2]*SK_MX[1] + P[22][18]*SK_MX[3]);
                Kfusion[23] = SK_MX[0]*(P[23][19] + P[23][16]*SH_MAG[1] + P[23][17]*SH_MAG[4] - P[23][1]*SK_MX[2] + P[23][2]*SK_MX[1] + P[23][18]*SK_MX[3]);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][16]*SH_MAG[1] + P[16][17]*SH_MAG[4] - P[16][1]*SK_MX[2] + P[16][2]*SK_MX[1] + P[16][18]*SK_MX[3]);
                Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][16]*SH_MAG[1] + P[17][17]*SH_MAG[4] - P[17][1]*SK_MX[2] + P[17][2]*SK_MX[1] + P[17][18]*SK_MX[3]);
                Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][16]*SH_MAG[1] + P[18][17]*SH_MAG[4] - P[18][1]*SK_MX[2] + P[18][2]*SK_MX[1] + P[18][18]*SK_MX[3]);
                Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][16]*SH_MAG[1] + P[19][17]*SH_MAG[4] - P[19][1]*SK_MX[2] + P[19][2]*SK_MX[1] + P[19][18]*SK_MX[3]);
                Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][16]*SH_MAG[1] + P[20][17]*SH_MAG[4] - P[20][1]*SK_MX[2] + P[20][2]*SK_MX[1] + P[20][18]*SK_MX[3]);
                Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][16]*SH_MAG[1] + P[21][17]*SH_MAG[4] - P[21][1]*SK_MX[2] + P[21][2]*SK_MX[1] + P[21][18]*SK_MX[3]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // set flags to indicate to other processes that fusion has been performed
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;

        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // calculate observation jacobians
            ZERO_FARRAY(H_MAG);
            H_MAG[0] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
            H_MAG[2] = - magE*SH_MAG[4] - magD*SH_MAG[7] - magN*SH_MAG[1];
            H_MAG[16] = 2.0f*q1*q2 - SH_MAG[8];
            H_MAG[17] = SH_MAG[0];
            H_MAG[18] = SH_MAG[3];
            H_MAG[20] = 1.0f;

            // calculate Kalman gain
            SK_MY[0] = 1.0f / varInnovMag[1];
            SK_MY[1] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
            SK_MY[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
            SK_MY[3] = SH_MAG[8] - 2.0f*q1*q2;
            Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][17]*SH_MAG[0] + P[0][18]*SH_MAG[3] + P[0][0]*SK_MY[2] - P[0][2]*SK_MY[1] - P[0][16]*SK_MY[3]);
            Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][17]*SH_MAG[0] + P[1][18]*SH_MAG[3] + P[1][0]*SK_MY[2] - P[1][2]*SK_MY[1] - P[1][16]*SK_MY[3]);
            Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][17]*SH_MAG[0] + P[2][18]*SH_MAG[3] + P[2][0]*SK_MY[2] - P[2][2]*SK_MY[1] - P[2][16]*SK_MY[3]);
            Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][17]*SH_MAG[0] + P[3][18]*SH_MAG[3] + P[3][0]*SK_MY[2] - P[3][2]*SK_MY[1] - P[3][16]*SK_MY[3]);
            Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][17]*SH_MAG[0] + P[4][18]*SH_MAG[3] + P[4][0]*SK_MY[2] - P[4][2]*SK_MY[1] - P[4][16]*SK_MY[3]);
            Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][17]*SH_MAG[0] + P[5][18]*SH_MAG[3] + P[5][0]*SK_MY[2] - P[5][2]*SK_MY[1] - P[5][16]*SK_MY[3]);
            Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][17]*SH_MAG[0] + P[6][18]*SH_MAG[3] + P[6][0]*SK_MY[2] - P[6][2]*SK_MY[1] - P[6][16]*SK_MY[3]);
            Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][17]*SH_MAG[0] + P[7][18]*SH_MAG[3] + P[7][0]*SK_MY[2] - P[7][2]*SK_MY[1] - P[7][16]*SK_MY[3]);
            Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][17]*SH_MAG[0] + P[8][18]*SH_MAG[3] + P[8][0]*SK_MY[2] - P[8][2]*SK_MY[1] - P[8][16]*SK_MY[3]);
            Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][17]*SH_MAG[0] + P[9][18]*SH_MAG[3] + P[9][0]*SK_MY[2] - P[9][2]*SK_MY[1] - P[9][16]*SK_MY[3]);
            Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][17]*SH_MAG[0] + P[10][18]*SH_MAG[3] + P[10][0]*SK_MY[2] - P[10][2]*SK_MY[1] - P[10][16]*SK_MY[3]);
            Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][17]*SH_MAG[0] + P[11][18]*SH_MAG[3] + P[11][0]*SK_MY[2] - P[11][2]*SK_MY[1] - P[11][16]*SK_MY[3]);
            Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][17]*SH_MAG[0] + P[12][18]*SH_MAG[3] + P[12][0]*SK_MY[2] - P[12][2]*SK_MY[1] - P[12][16]*SK_MY[3]);
            Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][17]*SH_MAG[0] + P[13][18]*SH_MAG[3] + P[13][0]*SK_MY[2] - P[13][2]*SK_MY[1] - P[13][16]*SK_MY[3]);
            Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][17]*SH_MAG[0] + P[14][18]*SH_MAG[3] + P[14][0]*SK_MY[2] - P[14][2]*SK_MY[1] - P[14][16]*SK_MY[3]);
            Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][17]*SH_MAG[0] + P[15][18]*SH_MAG[3] + P[15][0]*SK_MY[2] - P[15][2]*SK_MY[1] - P[15][16]*SK_MY[3]);
            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MY[0]*(P[22][20] + P[22][17]*SH_MAG[0] + P[22][18]*SH_MAG[3] + P[22][0]*SK_MY[2] - P[22][2]*SK_MY[1] - P[22][16]*SK_MY[3]);
                Kfusion[23] = SK_MY[0]*(P[23][20] + P[23][17]*SH_MAG[0] + P[23][18]*SH_MAG[3] + P[23][0]*SK_MY[2] - P[23][2]*SK_MY[1] - P[23][16]*SK_MY[3]);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][17]*SH_MAG[0] + P[16][18]*SH_MAG[3] + P[16][0]*SK_MY[2] - P[16][2]*SK_MY[1] - P[16][16]*SK_MY[3]);
                Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][17]*SH_MAG[0] + P[17][18]*SH_MAG[3] + P[17][0]*SK_MY[2] - P[17][2]*SK_MY[1] - P[17][16]*SK_MY[3]);
                Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][17]*SH_MAG[0] + P[18][18]*SH_MAG[3] + P[18][0]*SK_MY[2] - P[18][2]*SK_MY[1] - P[18][16]*SK_MY[3]);
                Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][17]*SH_MAG[0] + P[19][18]*SH_MAG[3] + P[19][0]*SK_MY[2] - P[19][2]*SK_MY[1] - P[19][16]*SK_MY[3]);
                Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][17]*SH_MAG[0] + P[20][18]*SH_MAG[3] + P[20][0]*SK_MY[2] - P[20][2]*SK_MY[1] - P[20][16]*SK_MY[3]);
                Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][17]*SH_MAG[0] + P[21][18]*SH_MAG[3] + P[21][0]*SK_MY[2] - P[21][2]*SK_MY[1] - P[21][16]*SK_MY[3]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // set flags to indicate to other processes that fusion has been performed
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;

        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // calculate observation jacobians
            ZERO_FARRAY(H_MAG);
            H_MAG[0] = magN*(SH_MAG[8] - 2.0f*q1*q2) - magD*SH_MAG[3] - magE*SH_MAG[0];
            H_MAG[1] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
            H_MAG[16] = SH_MAG[5];
            H_MAG[17] = 2.0f*q2*q3 - 2.0f*q0*q1;
            H_MAG[18] = SH_MAG[2];
            H_MAG[21] = 1.0f;

            // calculate Kalman gain
            SK_MZ[0] = 1.0f / varInnovMag[2];
            SK_MZ[1] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
            SK_MZ[2] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
            SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
            Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][18]*SH_MAG[2] + P[0][16]*SH_MAG[5] - P[0][0]*SK_MZ[1] + P[0][1]*SK_MZ[2] - P[0][17]*SK_MZ[3]);
            Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][18]*SH_MAG[2] + P[1][16]*SH_MAG[5] - P[1][0]*SK_MZ[1] + P[1][1]*SK_MZ[2] - P[1][17]*SK_MZ[3]);
            Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][18]*SH_MAG[2] + P[2][16]*SH_MAG[5] - P[2][0]*SK_MZ[1] + P[2][1]*SK_MZ[2] - P[2][17]*SK_MZ[3]);
            Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][18]*SH_MAG[2] + P[3][16]*SH_MAG[5] - P[3][0]*SK_MZ[1] + P[3][1]*SK_MZ[2] - P[3][17]*SK_MZ[3]);
            Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][18]*SH_MAG[2] + P[4][16]*SH_MAG[5] - P[4][0]*SK_MZ[1] + P[4][1]*SK_MZ[2] - P[4][17]*SK_MZ[3]);
            Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][18]*SH_MAG[2] + P[5][16]*SH_MAG[5] - P[5][0]*SK_MZ[1] + P[5][1]*SK_MZ[2] - P[5][17]*SK_MZ[3]);
            Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][18]*SH_MAG[2] + P[6][16]*SH_MAG[5] - P[6][0]*SK_MZ[1] + P[6][1]*SK_MZ[2] - P[6][17]*SK_MZ[3]);
            Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][18]*SH_MAG[2] + P[7][16]*SH_MAG[5] - P[7][0]*SK_MZ[1] + P[7][1]*SK_MZ[2] - P[7][17]*SK_MZ[3]);
            Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][18]*SH_MAG[2] + P[8][16]*SH_MAG[5] - P[8][0]*SK_MZ[1] + P[8][1]*SK_MZ[2] - P[8][17]*SK_MZ[3]);
            Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][18]*SH_MAG[2] + P[9][16]*SH_MAG[5] - P[9][0]*SK_MZ[1] + P[9][1]*SK_MZ[2] - P[9][17]*SK_MZ[3]);
            Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][18]*SH_MAG[2] + P[10][16]*SH_MAG[5] - P[10][0]*SK_MZ[1] + P[10][1]*SK_MZ[2] - P[10][17]*SK_MZ[3]);
            Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][18]*SH_MAG[2] + P[11][16]*SH_MAG[5] - P[11][0]*SK_MZ[1] + P[11][1]*SK_MZ[2] - P[11][17]*SK_MZ[3]);
            Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][18]*SH_MAG[2] + P[12][16]*SH_MAG[5] - P[12][0]*SK_MZ[1] + P[12][1]*SK_MZ[2] - P[12][17]*SK_MZ[3]);
            Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][18]*SH_MAG[2] + P[13][16]*SH_MAG[5] - P[13][0]*SK_MZ[1] + P[13][1]*SK_MZ[2] - P[13][17]*SK_MZ[3]);
            Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][18]*SH_MAG[2] + P[14][16]*SH_MAG[5] - P[14][0]*SK_MZ[1] + P[14][1]*SK_MZ[2] - P[14][17]*SK_MZ[3]);
            Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][18]*SH_MAG[2] + P[15][16]*SH_MAG[5] - P[15][0]*SK_MZ[1] + P[15][1]*SK_MZ[2] - P[15][17]*SK_MZ[3]);
            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MZ[0]*(P[22][21] + P[22][18]*SH_MAG[2] + P[22][16]*SH_MAG[5] - P[22][0]*SK_MZ[1] + P[22][1]*SK_MZ[2] - P[22][17]*SK_MZ[3]);
                Kfusion[23] = SK_MZ[0]*(P[23][21] + P[23][18]*SH_MAG[2] + P[23][16]*SH_MAG[5] - P[23][0]*SK_MZ[1] + P[23][1]*SK_MZ[2] - P[23][17]*SK_MZ[3]);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][18]*SH_MAG[2] + P[16][16]*SH_MAG[5] - P[16][0]*SK_MZ[1] + P[16][1]*SK_MZ[2] - P[16][17]*SK_MZ[3]);
                Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][18]*SH_MAG[2] + P[17][16]*SH_MAG[5] - P[17][0]*SK_MZ[1] + P[17][1]*SK_MZ[2] - P[17][17]*SK_MZ[3]);
                Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][18]*SH_MAG[2] + P[18][16]*SH_MAG[5] - P[18][0]*SK_MZ[1] + P[18][1]*SK_MZ[2] - P[18][17]*SK_MZ[3]);
                Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][18]*SH_MAG[2] + P[19][16]*SH_MAG[5] - P[19][0]*SK_MZ[1] + P[19][1]*SK_MZ[2] - P[19][17]*SK_MZ[3]);
                Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][18]*SH_MAG[2] + P[20][16]*SH_MAG[5] - P[20][0]*SK_MZ[1] + P[20][1]*SK_MZ[2] - P[20][17]*SK_MZ[3]);
                Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][18]*SH_MAG[2] + P[21][16]*SH_MAG[5] - P[21][0]*SK_MZ[1] + P[21][1]*SK_MZ[2] - P[21][17]*SK_MZ[3]);
            } else {
                for (uint8_t i=16; i<=21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

            // set flags to indicate to other processes that fusion has been performed
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;

        }

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=2; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 3; j<=15; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 16; j<=21; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = 0.0f;
            }
        }
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                ftype res = 0;
                res += KH[i][0] * P[0][j];
                res += KH[i][1] * P[1][j];
                res += KH[i][2] * P[2][j];
                res += KH[i][16] * P[16][j];
                res += KH[i][17] * P[17][j];
                res += KH[i][18] * P[18][j];
                res += KH[i][19] * P[19][j];
                res += KH[i][20] * P[20][j];
                res += KH[i][21] * P[21][j];
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

            // update the states
            // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
            stateStruct.angErr.zero();

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
            }

            // add table constraint here for faster convergence
            if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
                MagTableConstrain();
            }

            // the first 3 states represent the angular misalignment vector.
            // This is used to correct the estimated quaternion on the current time step
            stateStruct.quat.rotate(stateStruct.angErr);

        } else {
            // record bad axis
            if (obsIndex == 0) {
                faultStatus.bad_xmag = true;
            } else if (obsIndex == 1) {
                faultStatus.bad_ymag = true;
            } else if (obsIndex == 2) {
                faultStatus.bad_zmag = true;
            }
            CovarianceInit();
            return;
        }
    }
}


/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the magnetic poles)
*/
void NavEKF2_core::fuseEulerYaw()
{
    ftype q0 = stateStruct.quat[0];
    ftype q1 = stateStruct.quat[1];
    ftype q2 = stateStruct.quat[2];
    ftype q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2) set to parameter value as a default
    ftype R_YAW = sq(frontend->_yawNoise);

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    // determine if a 321 or 312 Euler sequence is best
    ftype predicted_yaw;
    ftype measured_yaw;
    ftype H_YAW[3];
    Matrix3F Tbn_zeroYaw;

    if (fabsF(prevTnb[0][2]) < fabsF(prevTnb[1][2])) {
        // calculate observation jacobian when we are observing the first rotation in a 321 sequence
        ftype t2 = q0*q0;
        ftype t3 = q1*q1;
        ftype t4 = q2*q2;
        ftype t5 = q3*q3;
        ftype t6 = t2+t3-t4-t5;
        ftype t7 = q0*q3*2.0f;
        ftype t8 = q1*q2*2.0f;
        ftype t9 = t7+t8;
        ftype t10 = sq(t6);
        if (t10 > 1e-6f) {
            t10 = 1.0f / t10;
        } else {
            return;
        }
        ftype t11 = t9*t9;
        ftype t12 = t10*t11;
        ftype t13 = t12+1.0f;
        ftype t14;
        if (fabsF(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        ftype t15 = 1.0f/t6;
        H_YAW[0] = 0.0f;
        H_YAW[1] = t14*(t15*(q0*q1*2.0f-q2*q3*2.0f)+t9*t10*(q0*q2*2.0f+q1*q3*2.0f));
        H_YAW[2] = t14*(t15*(t2-t3+t4-t5)+t9*t10*(t7-t8));

        // calculate predicted and measured yaw angle
        Vector3F euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        predicted_yaw = euler321.z;
        if (use_compass() && yawAlignComplete && magStateInitComplete) {
            // Use measured mag components rotated into earth frame to measure yaw
            Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);
            Vector3F magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;
            measured_yaw = wrap_PI(-atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination());
        } else if (extNavUsedForYaw) {
            // Get the yaw angle  from the external vision data
            R_YAW = sq(extNavDataDelayed.angErr);
            extNavDataDelayed.quat.to_euler(euler321.x, euler321.y, euler321.z);
            measured_yaw =  euler321.z;
        } else {
            if (imuSampleTime_ms - prevBetaStep_ms > 1000 && yawEstimator != nullptr) {
                ftype gsfYaw, gsfYawVariance, velInnovLength;
                if (yawEstimator->getYawData(gsfYaw, gsfYawVariance) &&
                    is_positive(gsfYawVariance) &&
                    gsfYawVariance < sq(radians(15.0f)) &&
                    (assume_zero_sideslip() || (yawEstimator->getVelInnovLength(velInnovLength) && velInnovLength < frontend->maxYawEstVelInnov))) {
                        measured_yaw = gsfYaw;
                        R_YAW = gsfYawVariance;
                } else {
                    // use predicted to prevent unconstrained variance growth
                    measured_yaw = predicted_yaw;
                }
            } else {
                // use predicted to prevent unconstrained variance growth
                measured_yaw = predicted_yaw;
            }
        }
    } else {
        // calculate observation jacobian when we are observing a rotation in a 312 sequence
        ftype t2 = q0*q0;
        ftype t3 = q1*q1;
        ftype t4 = q2*q2;
        ftype t5 = q3*q3;
        ftype t6 = t2-t3+t4-t5;
        ftype t7 = q0*q3*2.0f;
        ftype t10 = q1*q2*2.0f;
        ftype t8 = t7-t10;
        ftype t9 = sq(t6);
        if (t9 > 1e-6f) {
            t9 = 1.0f/t9;
        } else {
            return;
        }
        ftype t11 = t8*t8;
        ftype t12 = t9*t11;
        ftype t13 = t12+1.0f;
        ftype t14;
        if (fabsF(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        ftype t15 = 1.0f/t6;
        H_YAW[0] = -t14*(t15*(q0*q2*2.0+q1*q3*2.0)-t8*t9*(q0*q1*2.0-q2*q3*2.0));
        H_YAW[1] = 0.0f;
        H_YAW[2] = t14*(t15*(t2+t3-t4-t5)+t8*t9*(t7+t10));

        // calculate predicted and measured yaw angle
        Vector3F euler312 = stateStruct.quat.to_vector312();
        predicted_yaw = euler312.z;
        if (use_compass() && yawAlignComplete && magStateInitComplete) {
            // Use measured mag components rotated into earth frame to measure yaw
            Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
            Vector3F magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;
            measured_yaw = wrap_PI(-atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination());
        } else if (extNavUsedForYaw) {
            // Get the yaw angle  from the external vision data
            R_YAW = sq(extNavDataDelayed.angErr);
            euler312 = extNavDataDelayed.quat.to_vector312();
            measured_yaw =  euler312.z;
        } else {
            if (imuSampleTime_ms - prevBetaStep_ms > 1000 && yawEstimator != nullptr) {
                ftype gsfYaw, gsfYawVariance, velInnovLength;
                if (yawEstimator->getYawData(gsfYaw, gsfYawVariance) &&
                    is_positive(gsfYawVariance) &&
                    gsfYawVariance < sq(radians(15.0f)) &&
                    (assume_zero_sideslip() || (yawEstimator->getVelInnovLength(velInnovLength) && velInnovLength < frontend->maxYawEstVelInnov))) {
                        measured_yaw = gsfYaw;
                        R_YAW = gsfYawVariance;
                } else {
                    // use predicted to prevent unconstrained variance growth
                    measured_yaw = predicted_yaw;
                }
            } else {
                // use predicted to prevent unconstrained variance growth
                measured_yaw = predicted_yaw;
            }
        }
    }

    // Calculate the innovation
    ftype innovation = wrap_PI(predicted_yaw - measured_yaw);

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    ftype PH[3];
    ftype varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    ftype varInnovInv;
    if (varInnov >= R_YAW) {
        varInnovInv = 1.0f / varInnov;
        // output numerical health status
        faultStatus.bad_yaw = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        // output numerical health status
        faultStatus.bad_yaw = true;
        return;
    }

    // calculate Kalman gain
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (ftype)frontend->_yawInnovGate, 1.0f)) * varInnov);

    // Declare the magnetometer unhealthy if the innovation test fails
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        // On the ground a large innovation could be due to large initial gyro bias or magnetic interference from nearby objects
        // If we are flying, then it is more likely due to a magnetometer fault and we should not fuse the data
        if (inFlight) {
            return;
        }
    } else {
        magHealth = true;
    }

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    // calculate K*H*P
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= 2; column++) {
            KH[row][column] = Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            ftype tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            KHP[row][column] = tmp;
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

        // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t i=0; i<=stateIndexLim; i++) {
            statesArray[i] -= Kfusion[i] * innovation;
        }

        // the first 3 states represent the angular misalignment vector.
        // This is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

        // record fusion event
        faultStatus.bad_yaw = false;
        lastYawTime_ms = imuSampleTime_ms;


    } else {
        // record fusion numerical health status
        faultStatus.bad_yaw = true;
    }
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF2_core::FuseDeclination(ftype declErr)
{
    // declination error variance (rad^2)
    const ftype R_DECL = sq(declErr);

    // copy required states to local variables
    ftype magN = stateStruct.earth_magfield.x;
    ftype magE = stateStruct.earth_magfield.y;

    // prevent bad earth field states from causing numerical errors or exceptions
    if (magN < 1e-3f) {
        return;
    }

    // Calculate observation Jacobian and Kalman gains
    ftype t2 = magE*magE;
    ftype t3 = magN*magN;
    ftype t4 = t2+t3;
    ftype t5 = 1.0f/t4;
    ftype t22 = magE*t5;
    ftype t23 = magN*t5;
    ftype t6 = P[16][16]*t22;
    ftype t13 = P[17][16]*t23;
    ftype t7 = t6-t13;
    ftype t8 = t22*t7;
    ftype t9 = P[16][17]*t22;
    ftype t14 = P[17][17]*t23;
    ftype t10 = t9-t14;
    ftype t15 = t23*t10;
    ftype t11 = R_DECL+t8-t15; // innovation variance
    if (t11 < R_DECL) {
        return;
    }
    ftype t12 = 1.0f/t11;

    ftype H_MAG[24];

    H_MAG[16] = -magE*t5;
    H_MAG[17] = magN*t5;

    for (uint8_t i=0; i<=15; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }
    Kfusion[16] = -t12*(t6-P[16][17]*t23);
    Kfusion[17] = t12*(t14-P[17][16]*t22);
    for (uint8_t i=17; i<=23; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }

    // get the magnetic declination
    ftype magDecAng = MagDeclination();

    // Calculate the innovation
    ftype innovation = atan2F(magE , magN) - magDecAng;

    // limit the innovation to protect against data errors
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance P = (I - K*H)*P
    // take advantage of the empty columns in KH to reduce the
    // number of operations
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=15; j++) {
            KH[i][j] = 0.0f;
        }
        KH[i][16] = Kfusion[i] * H_MAG[16];
        KH[i][17] = Kfusion[i] * H_MAG[17];
        for (unsigned j = 18; j<=23; j++) {
            KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j<=stateIndexLim; j++) {
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            KHP[i][j] = KH[i][16] * P[16][j] + KH[i][17] * P[17][j];
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

        // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
        }

        // the first 3 states represent the angular misalignment vector.
        // This is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

        // record fusion health status
        faultStatus.bad_decl = false;
    } else {
        // record fusion health status
        faultStatus.bad_decl = true;
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavEKF2_core::alignMagStateDeclination()
{
    // don't do this if we already have a learned magnetic field
    if (magFieldLearned) {
        return;
    }

    // get the magnetic declination
    ftype magDecAng = MagDeclination();

    // rotate the NE values so that the declination matches the published value
    Vector3F initMagNED = stateStruct.earth_magfield;
    ftype magLengthNE = norm(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosF(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinF(magDecAng);

    if (!inhibitMagStates) {
        // zero the corresponding state covariances if magnetic field state learning is active
        ftype var_16 = P[16][16];
        ftype var_17 = P[17][17];
        zeroRows(P,16,17);
        zeroCols(P,16,17);
        P[16][16] = var_16;
        P[17][17] = var_17;

        // fuse the declination angle to establish covariances and prevent large swings in declination
        // during initial fusion
        FuseDeclination(0.1f);

    }
}

// record a magnetic field state reset event
void NavEKF2_core::recordMagReset()
{
    magStateInitComplete = true;
    if (inFlight) {
        finalInflightMagInit = true;
    }
    // take a snap-shot of the vertical position, quaternion  and yaw innovation to use as a reference
    // for post alignment checks
    posDownAtLastMagReset = stateStruct.position.z;
    quatAtLastMagReset = stateStruct.quat;
    yawInnovAtLastMagReset = innovYaw;
}

// Reset states using yaw from EKF-GSF and velocity and position from GPS
bool NavEKF2_core::EKFGSF_resetMainFilterYaw()
{
    // Don't do a reset unless permitted by the EK2_GSF_USE_MASK and EKF@_GSF_RUN_MASK parameter masks
    if ((yawEstimator == nullptr)
        || !(frontend->_gsfUseMask & (1U<<core_index))
        || EKFGSF_yaw_reset_count >= frontend->_gsfResetMaxCount) {
        return false;
    };

    ftype yawEKFGSF, yawVarianceEKFGSF, velInnovLength;
    if (yawEstimator->getYawData(yawEKFGSF, yawVarianceEKFGSF) &&
        is_positive(yawVarianceEKFGSF) &&
        yawVarianceEKFGSF < sq(radians(15.0f)) &&
        (assume_zero_sideslip() || (yawEstimator->getVelInnovLength(velInnovLength) && velInnovLength < frontend->maxYawEstVelInnov))) {

        // keep roll and pitch and reset yaw
        resetQuatStateYawOnly(yawEKFGSF, yawVarianceEKFGSF, false);

        // record the emergency reset event
        EKFGSF_yaw_reset_request_ms = 0;
        EKFGSF_yaw_reset_ms = imuSampleTime_ms;
        EKFGSF_yaw_reset_count++;

        if (!use_compass()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF2 IMU%u yaw aligned using GPS",(unsigned)imu_index);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF2 IMU%u emergency yaw reset",(unsigned)imu_index);
        }

        // Fail the magnetomer so it doesn't get used and pull the yaw away from the correct value
        allMagSensorsFailed = true;

        // reset velocity and position states to GPS - if yaw is fixed then the filter should start to operate correctly
        ResetVelocity();
        ResetPosition();

        // reset test ratios that are reported to prevent a race condition with the external state machine requesting the reset
        velTestRatio = 0.0f;
        posTestRatio = 0.0f;

        return true;

    }

    return false;

}

void NavEKF2_core::resetQuatStateYawOnly(ftype yaw, ftype yawVariance, bool isDeltaYaw)
{
    QuaternionF quatBeforeReset = stateStruct.quat;

    // check if we should use a 321 or 312 Rotation sequence and update the quaternion
    // states using the preferred yaw definition
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    Vector3F eulerAngles;
    if (fabsF(prevTnb[2][0]) < fabsF(prevTnb[2][1])) {
        // rolled more than pitched so use 321 rotation order
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        if (isDeltaYaw) {
            yaw = wrap_PI(yaw + eulerAngles.z);
        }
        stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, yaw);
    } else {
        // pitched more than rolled so use 312 rotation order
        eulerAngles = stateStruct.quat.to_vector312();
        if (isDeltaYaw) {
            yaw = wrap_PI(yaw + eulerAngles.z);
        }
        stateStruct.quat.from_vector312(eulerAngles.x, eulerAngles.y, yaw);
    }

    // Update the rotation matrix
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    
    ftype deltaYaw = wrap_PI(yaw - eulerAngles.z);

    // calculate the change in the quaternion state and apply it to the output history buffer
    QuaternionF quat_delta = stateStruct.quat / quatBeforeReset;
    StoreQuatRotate(quat_delta);

    // rotate attitude error variances into earth frame
    Vector3F bf_variances = Vector3F(P[0][0], P[1][1], P[2][2]);
    Vector3F ef_variances = prevTnb.transposed() * bf_variances;

    // reset vertical component to supplied value
    ef_variances.z = yawVariance;

    // rotate back into body frame
    bf_variances = prevTnb * ef_variances;

    // Reset all attitude error state covariances
    zeroRows(P, 0, 2);
    zeroCols(P, 0, 2);

    // Initialise variances
    P[0][0] = bf_variances.x;
    P[1][1] = bf_variances.y;
    P[2][2] = bf_variances.z;

    // record the yaw reset event
    yawResetAngle += deltaYaw;
    lastYawReset_ms = imuSampleTime_ms;
    recordYawReset();

    // clear all pending yaw reset requests
    gpsYawResetRequest = false;
    magYawResetRequest = false;
    
}
