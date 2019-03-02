#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF3_core::controlMagYawReset()
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

    // Quaternion and delta rotation vector that are re-used for different calculations
    Vector3f deltaRotVecTemp;
    Quaternion deltaQuatTemp;

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

    // Check if conditions for a interim or final yaw/mag reset are met
    bool finalResetRequest = false;
    bool interimResetRequest = false;
    if (flightResetAllowed && !assume_zero_sideslip()) {
        // check that we have reached a height where ground magnetic interference effects are insignificant
        // and can perform a final reset of the yaw and field states
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) < -EKF3_MAG_FINAL_RESET_ALT;

        // check for increasing height
        bool hgtIncreasing = (posDownAtLastMagReset-stateStruct.position.z) > 0.5f;
        float yawInnovIncrease = fabsf(innovYaw) - fabsf(yawInnovAtLastMagReset);

        // check for increasing yaw innovations
        bool yawInnovIncreasing = yawInnovIncrease > 0.25f;

        // check that the yaw innovations haven't been caused by a large change in attitude
        deltaQuatTemp = quatAtLastMagReset / stateStruct.quat;
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);
        bool largeAngleChange = deltaRotVecTemp.length() > yawInnovIncrease;

        // if yaw innovations and height have increased and we haven't rotated much
        // then we are climbing away from a ground based magnetic anomaly and need to reset
        interimResetRequest = hgtIncreasing && yawInnovIncreasing && !largeAngleChange;
    }

    // an initial reset is required if we have not yet aligned the yaw angle
    bool initialResetRequest = initialResetAllowed && !yawAlignComplete;

    // a combined yaw angle and magnetic field reset can be initiated by:
    magYawResetRequest = magYawResetRequest || // an external request
            initialResetRequest || // an initial alignment performed by all vehicle types using magnetometer
            interimResetRequest || // an interim alignment required to recover from ground based magnetic anomaly
            finalResetRequest; // the final reset when we have achieved enough height to be in stable magnetic field environment

    // Perform a reset of magnetic field states and reset yaw to corrected magnetic heading
    if (magYawResetRequest || magStateResetRequest) {

        // get the euler angles from the current state estimate
        Vector3f eulerAngles;
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

        // Use the Euler angles and magnetometer measurement to update the magnetic field states
        // and get an updated quaternion
        Quaternion newQuat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

        // if a yaw reset has been requested, apply the updated quaternion to the current state
        if (magYawResetRequest) {
            // previous value used to calculate a reset delta
            Quaternion prevQuat = stateStruct.quat;

            // calculate the variance for the rotation estimate expressed as a rotation vector
            // this will be used later to reset the quaternion state covariances
            Vector3f angleErrVarVec = calcRotVecVariances();

            // update the quaternion states using the new yaw angle
            stateStruct.quat = newQuat;

            // update the yaw angle variance using the variance of the measurement
            angleErrVarVec.z = sq(MAX(frontend->_yawNoise, 1.0e-2f));

            // reset the quaternion covariances using the rotation vector variances
            initialiseQuatCovariances(angleErrVarVec);

            // calculate the change in the quaternion state and apply it to the output history buffer
            prevQuat = stateStruct.quat/prevQuat;
            StoreQuatRotate(prevQuat);

            // send initial alignment status to console
            if (!yawAlignComplete) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u initial yaw alignment complete",(unsigned)imu_index);
            }

            // send in-flight yaw alignment status to console
            if (finalResetRequest) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u in-flight yaw alignment complete",(unsigned)imu_index);
            } else if (interimResetRequest) {
                gcs().send_text(MAV_SEVERITY_WARNING, "EKF3 IMU%u ground mag anomaly, yaw re-aligned",(unsigned)imu_index);
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

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF3_core::realignYawGPS()
{
    // get quaternion from existing filter states and calculate roll, pitch and yaw angles
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        // calculate course yaw angle
        float velYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        float gpsYaw = atan2f(gpsDataDelayed.vel.y,gpsDataDelayed.vel.x);

        // Check the yaw angles for consistency
        float yawErr = MAX(fabsf(wrap_PI(gpsYaw - velYaw)),fabsf(wrap_PI(gpsYaw - eulerAngles.z)));

        // If the angles disagree by more than 45 degrees and GPS innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
        badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f) && (PV_AidingMode == AID_ABSOLUTE)) || !yawAlignComplete;

        // correct yaw angle using GPS ground course if compass yaw bad
        if (badMagYaw) {

            // calculate the variance for the rotation estimate expressed as a rotation vector
            // this will be used later to reset the quaternion state covariances
            Vector3f angleErrVarVec = calcRotVecVariances();

            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, gpsYaw);

            // reset the velocity and position states as they will be inaccurate due to bad yaw
            velResetSource = GPS;
            ResetVelocity();
            posResetSource = GPS;
            ResetPosition();

            // set the yaw angle variance to a larger value to reflect the uncertainty in yaw
            angleErrVarVec.z = sq(radians(45.0f));

            // reset the quaternion covariances using the rotation vector variances
            zeroRows(P,0,3);
            zeroCols(P,0,3);
            initialiseQuatCovariances(angleErrVarVec);

            // send yaw alignment information to console
            gcs().send_text(MAV_SEVERITY_INFO, "EKF3 IMU%u yaw aligned to GPS velocity",(unsigned)imu_index);


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
void NavEKF3_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been performed on that time step
    // used for load levelling
    magFusePerformed = false;

    // check for and read new magnetometer measurements
    readMagData();

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

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
            if (PV_AidingMode != AID_ABSOLUTE) {
                FuseDeclination(0.34f);
            }
            // fuse the three magnetometer componenents sequentially
            hal.util->perf_begin(_perf_test[0]);
            for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) {
                FuseMagnetometer();
                // don't continue fusion if unhealthy
                if (!magHealth) {
                    hal.util->perf_end(_perf_test[0]);
                    break;
                }
            }
            hal.util->perf_end(_perf_test[0]);
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // If we have no magnetometer, fuse in a synthetic heading measurement at 7Hz to prevent the filter covariances
    // from becoming badly conditioned. For planes we only do this on-ground because they can align the yaw from GPS when
    // airborne. For other platform types we do this all the time.
    if (!use_compass()) {
        if ((onGround || !assume_zero_sideslip()) && (imuSampleTime_ms - lastSynthYawTime_ms > 140)) {
            fuseEulerYaw();
            magTestRatio.zero();
            yawTestRatio = 0.0f;
            lastSynthYawTime_ms = imuSampleTime_ms;
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

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseMagnetometer()
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
    uint8_t &obsIndex = mag_state.obsIndex;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector24 H_MAG;
    Vector5 SK_MX;
    Vector5 SK_MY;
    Vector5 SK_MZ;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains

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
    R_MAG = sq(constrain_float(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*imuDataDelayed.delAng.length() / imuDataDelayed.delAngDT);

    // calculate common expressions used to calculate observation jacobians an innovation variance for each component
    SH_MAG[0] = 2.0f*magD*q3 + 2.0f*magE*q2 + 2.0f*magN*q1;
    SH_MAG[1] = 2.0f*magD*q0 - 2.0f*magE*q1 + 2.0f*magN*q2;
    SH_MAG[2] = 2.0f*magD*q1 + 2.0f*magE*q0 - 2.0f*magN*q3;
    SH_MAG[3] = sq(q3);
    SH_MAG[4] = sq(q2);
    SH_MAG[5] = sq(q1);
    SH_MAG[6] = sq(q0);
    SH_MAG[7] = 2.0f*magN*q0;
    SH_MAG[8] = 2.0f*magE*q3;

    // Calculate the innovation variance for each axis
    // X axis
    varInnovMag[0] = (P[19][19] + R_MAG + P[1][19]*SH_MAG[0] - P[2][19]*SH_MAG[1] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (2.0f*q0*q3 + 2.0f*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] - P[2][17]*SH_MAG[1] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][17]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q2 - 2.0f*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] - P[2][18]*SH_MAG[1] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][18]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] - P[2][0]*SH_MAG[1] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][0]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[17][19]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][19]*(2.0f*q0*q2 - 2.0f*q1*q3) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] - P[2][1]*SH_MAG[1] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][1]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[1]*(P[19][2] + P[1][2]*SH_MAG[0] - P[2][2]*SH_MAG[1] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][2]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] - P[2][3]*SH_MAG[1] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][3]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] - P[2][16]*SH_MAG[1] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2.0f*q0*q3 + 2.0f*q1*q2) - P[18][16]*(2.0f*q0*q2 - 2.0f*q1*q3) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
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
    varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2.0f*q0*q3 - 2.0f*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][16]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (2.0f*q0*q1 + 2.0f*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][18]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][3]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P[16][20]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][20]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][0]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][1]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][2]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2.0f*q0*q3 - 2.0f*q1*q2) + P[18][17]*(2.0f*q0*q1 + 2.0f*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
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
    varInnovMag[2] = (P[21][21] + R_MAG + P[0][21]*SH_MAG[1] - P[1][21]*SH_MAG[2] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (2.0f*q0*q2 + 2.0f*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] - P[1][16]*SH_MAG[2] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][16]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q1 - 2.0f*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] - P[1][17]*SH_MAG[2] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][17]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] - P[1][2]*SH_MAG[2] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][2]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[16][21]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][21]*(2.0f*q0*q1 - 2.0f*q2*q3) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] - P[1][0]*SH_MAG[2] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][0]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[2]*(P[21][1] + P[0][1]*SH_MAG[1] - P[1][1]*SH_MAG[2] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][1]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] - P[1][3]*SH_MAG[2] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][3]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] - P[1][18]*SH_MAG[2] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2.0f*q0*q2 + 2.0f*q1*q3) - P[17][18]*(2.0f*q0*q1 - 2.0f*q2*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));
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
        magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (float)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);
    }

    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);

    // if the magnetometer is unhealthy, do not proceed further
    if (!magHealth) {
        return;
    }

    for (obsIndex = 0; obsIndex <= 2; obsIndex++) {

        if (obsIndex == 0) {

            for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
            H_MAG[1] = SH_MAG[0];
            H_MAG[2] = -SH_MAG[1];
            H_MAG[3] = SH_MAG[2];
            H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
            H_MAG[17] = 2.0f*q0*q3 + 2.0f*q1*q2;
            H_MAG[18] = 2.0f*q1*q3 - 2.0f*q0*q2;
            H_MAG[19] = 1.0f;

            // calculate Kalman gain
            SK_MX[0] = 1.0f / varInnovMag[0];
            SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
            SK_MX[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
            SK_MX[3] = 2.0f*q0*q2 - 2.0f*q1*q3;
            SK_MX[4] = 2.0f*q0*q3 + 2.0f*q1*q2;

            Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][1]*SH_MAG[0] - P[0][2]*SH_MAG[1] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[2] - P[0][16]*SK_MX[1] + P[0][17]*SK_MX[4] - P[0][18]*SK_MX[3]);
            Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][1]*SH_MAG[0] - P[1][2]*SH_MAG[1] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[2] - P[1][16]*SK_MX[1] + P[1][17]*SK_MX[4] - P[1][18]*SK_MX[3]);
            Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][1]*SH_MAG[0] - P[2][2]*SH_MAG[1] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[2] - P[2][16]*SK_MX[1] + P[2][17]*SK_MX[4] - P[2][18]*SK_MX[3]);
            Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][1]*SH_MAG[0] - P[3][2]*SH_MAG[1] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[2] - P[3][16]*SK_MX[1] + P[3][17]*SK_MX[4] - P[3][18]*SK_MX[3]);
            Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][1]*SH_MAG[0] - P[4][2]*SH_MAG[1] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[2] - P[4][16]*SK_MX[1] + P[4][17]*SK_MX[4] - P[4][18]*SK_MX[3]);
            Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][1]*SH_MAG[0] - P[5][2]*SH_MAG[1] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[2] - P[5][16]*SK_MX[1] + P[5][17]*SK_MX[4] - P[5][18]*SK_MX[3]);
            Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][1]*SH_MAG[0] - P[6][2]*SH_MAG[1] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[2] - P[6][16]*SK_MX[1] + P[6][17]*SK_MX[4] - P[6][18]*SK_MX[3]);
            Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][1]*SH_MAG[0] - P[7][2]*SH_MAG[1] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[2] - P[7][16]*SK_MX[1] + P[7][17]*SK_MX[4] - P[7][18]*SK_MX[3]);
            Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][1]*SH_MAG[0] - P[8][2]*SH_MAG[1] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[2] - P[8][16]*SK_MX[1] + P[8][17]*SK_MX[4] - P[8][18]*SK_MX[3]);
            Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][1]*SH_MAG[0] - P[9][2]*SH_MAG[1] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[2] - P[9][16]*SK_MX[1] + P[9][17]*SK_MX[4] - P[9][18]*SK_MX[3]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][1]*SH_MAG[0] - P[10][2]*SH_MAG[1] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[2] - P[10][16]*SK_MX[1] + P[10][17]*SK_MX[4] - P[10][18]*SK_MX[3]);
                Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][1]*SH_MAG[0] - P[11][2]*SH_MAG[1] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[2] - P[11][16]*SK_MX[1] + P[11][17]*SK_MX[4] - P[11][18]*SK_MX[3]);
                Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][1]*SH_MAG[0] - P[12][2]*SH_MAG[1] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[2] - P[12][16]*SK_MX[1] + P[12][17]*SK_MX[4] - P[12][18]*SK_MX[3]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] - P[13][2]*SH_MAG[1] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[4] - P[13][18]*SK_MX[3]);
                Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] - P[14][2]*SH_MAG[1] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[4] - P[14][18]*SK_MX[3]);
                Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] - P[15][2]*SH_MAG[1] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[4] - P[15][18]*SK_MX[3]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] - P[16][2]*SH_MAG[1] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[4] - P[16][18]*SK_MX[3]);
                Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] - P[17][2]*SH_MAG[1] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[4] - P[17][18]*SK_MX[3]);
                Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] - P[18][2]*SH_MAG[1] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[4] - P[18][18]*SK_MX[3]);
                Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] - P[19][2]*SH_MAG[1] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[4] - P[19][18]*SK_MX[3]);
                Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] - P[20][2]*SH_MAG[1] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[4] - P[20][18]*SK_MX[3]);
                Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] - P[21][2]*SH_MAG[1] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[4] - P[21][18]*SK_MX[3]);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MX[0]*(P[22][19] + P[22][1]*SH_MAG[0] - P[22][2]*SH_MAG[1] + P[22][3]*SH_MAG[2] + P[22][0]*SK_MX[2] - P[22][16]*SK_MX[1] + P[22][17]*SK_MX[4] - P[22][18]*SK_MX[3]);
                Kfusion[23] = SK_MX[0]*(P[23][19] + P[23][1]*SH_MAG[0] - P[23][2]*SH_MAG[1] + P[23][3]*SH_MAG[2] + P[23][0]*SK_MX[2] - P[23][16]*SK_MX[1] + P[23][17]*SK_MX[4] - P[23][18]*SK_MX[3]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;

        } else if (obsIndex == 1) { // Fuse Y axis

            // calculate observation jacobians
            for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[2];
            H_MAG[1] = SH_MAG[1];
            H_MAG[2] = SH_MAG[0];
            H_MAG[3] = 2.0f*magD*q2 - SH_MAG[8] - SH_MAG[7];
            H_MAG[16] = 2.0f*q1*q2 - 2.0f*q0*q3;
            H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
            H_MAG[18] = 2.0f*q0*q1 + 2.0f*q2*q3;
            H_MAG[20] = 1.0f;

            // calculate Kalman gain
            SK_MY[0] = 1.0f / varInnovMag[1];
            SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
            SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
            SK_MY[3] = 2.0f*q0*q3 - 2.0f*q1*q2;
            SK_MY[4] = 2.0f*q0*q1 + 2.0f*q2*q3;

            Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][17]*SK_MY[1] - P[0][16]*SK_MY[3] + P[0][18]*SK_MY[4]);
            Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][17]*SK_MY[1] - P[1][16]*SK_MY[3] + P[1][18]*SK_MY[4]);
            Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][17]*SK_MY[1] - P[2][16]*SK_MY[3] + P[2][18]*SK_MY[4]);
            Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][17]*SK_MY[1] - P[3][16]*SK_MY[3] + P[3][18]*SK_MY[4]);
            Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][17]*SK_MY[1] - P[4][16]*SK_MY[3] + P[4][18]*SK_MY[4]);
            Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][17]*SK_MY[1] - P[5][16]*SK_MY[3] + P[5][18]*SK_MY[4]);
            Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][17]*SK_MY[1] - P[6][16]*SK_MY[3] + P[6][18]*SK_MY[4]);
            Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][17]*SK_MY[1] - P[7][16]*SK_MY[3] + P[7][18]*SK_MY[4]);
            Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][17]*SK_MY[1] - P[8][16]*SK_MY[3] + P[8][18]*SK_MY[4]);
            Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][17]*SK_MY[1] - P[9][16]*SK_MY[3] + P[9][18]*SK_MY[4]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][17]*SK_MY[1] - P[10][16]*SK_MY[3] + P[10][18]*SK_MY[4]);
                Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][17]*SK_MY[1] - P[11][16]*SK_MY[3] + P[11][18]*SK_MY[4]);
                Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][17]*SK_MY[1] - P[12][16]*SK_MY[3] + P[12][18]*SK_MY[4]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
                Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
                Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
                Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
                Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
                Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
                Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
                Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MY[0]*(P[22][20] + P[22][0]*SH_MAG[2] + P[22][1]*SH_MAG[1] + P[22][2]*SH_MAG[0] - P[22][3]*SK_MY[2] - P[22][17]*SK_MY[1] - P[22][16]*SK_MY[3] + P[22][18]*SK_MY[4]);
                Kfusion[23] = SK_MY[0]*(P[23][20] + P[23][0]*SH_MAG[2] + P[23][1]*SH_MAG[1] + P[23][2]*SH_MAG[0] - P[23][3]*SK_MY[2] - P[23][17]*SK_MY[1] - P[23][16]*SK_MY[3] + P[23][18]*SK_MY[4]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // calculate observation jacobians
            for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = -SH_MAG[2];
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[16] = 2.0f*q0*q2 + 2.0f*q1*q3;
            H_MAG[17] = 2.0f*q2*q3 - 2.0f*q0*q1;
            H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[21] = 1.0f;

            // calculate Kalman gain
            SK_MZ[0] = 1.0f / varInnovMag[2];
            SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            SK_MZ[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
            SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
            SK_MZ[4] = 2.0f*q0*q2 + 2.0f*q1*q3;

            Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][0]*SH_MAG[1] - P[0][1]*SH_MAG[2] + P[0][3]*SH_MAG[0] + P[0][2]*SK_MZ[2] + P[0][18]*SK_MZ[1] + P[0][16]*SK_MZ[4] - P[0][17]*SK_MZ[3]);
            Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][0]*SH_MAG[1] - P[1][1]*SH_MAG[2] + P[1][3]*SH_MAG[0] + P[1][2]*SK_MZ[2] + P[1][18]*SK_MZ[1] + P[1][16]*SK_MZ[4] - P[1][17]*SK_MZ[3]);
            Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][0]*SH_MAG[1] - P[2][1]*SH_MAG[2] + P[2][3]*SH_MAG[0] + P[2][2]*SK_MZ[2] + P[2][18]*SK_MZ[1] + P[2][16]*SK_MZ[4] - P[2][17]*SK_MZ[3]);
            Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][0]*SH_MAG[1] - P[3][1]*SH_MAG[2] + P[3][3]*SH_MAG[0] + P[3][2]*SK_MZ[2] + P[3][18]*SK_MZ[1] + P[3][16]*SK_MZ[4] - P[3][17]*SK_MZ[3]);
            Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][0]*SH_MAG[1] - P[4][1]*SH_MAG[2] + P[4][3]*SH_MAG[0] + P[4][2]*SK_MZ[2] + P[4][18]*SK_MZ[1] + P[4][16]*SK_MZ[4] - P[4][17]*SK_MZ[3]);
            Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][0]*SH_MAG[1] - P[5][1]*SH_MAG[2] + P[5][3]*SH_MAG[0] + P[5][2]*SK_MZ[2] + P[5][18]*SK_MZ[1] + P[5][16]*SK_MZ[4] - P[5][17]*SK_MZ[3]);
            Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][0]*SH_MAG[1] - P[6][1]*SH_MAG[2] + P[6][3]*SH_MAG[0] + P[6][2]*SK_MZ[2] + P[6][18]*SK_MZ[1] + P[6][16]*SK_MZ[4] - P[6][17]*SK_MZ[3]);
            Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][0]*SH_MAG[1] - P[7][1]*SH_MAG[2] + P[7][3]*SH_MAG[0] + P[7][2]*SK_MZ[2] + P[7][18]*SK_MZ[1] + P[7][16]*SK_MZ[4] - P[7][17]*SK_MZ[3]);
            Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][0]*SH_MAG[1] - P[8][1]*SH_MAG[2] + P[8][3]*SH_MAG[0] + P[8][2]*SK_MZ[2] + P[8][18]*SK_MZ[1] + P[8][16]*SK_MZ[4] - P[8][17]*SK_MZ[3]);
            Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][0]*SH_MAG[1] - P[9][1]*SH_MAG[2] + P[9][3]*SH_MAG[0] + P[9][2]*SK_MZ[2] + P[9][18]*SK_MZ[1] + P[9][16]*SK_MZ[4] - P[9][17]*SK_MZ[3]);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][0]*SH_MAG[1] - P[10][1]*SH_MAG[2] + P[10][3]*SH_MAG[0] + P[10][2]*SK_MZ[2] + P[10][18]*SK_MZ[1] + P[10][16]*SK_MZ[4] - P[10][17]*SK_MZ[3]);
                Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][0]*SH_MAG[1] - P[11][1]*SH_MAG[2] + P[11][3]*SH_MAG[0] + P[11][2]*SK_MZ[2] + P[11][18]*SK_MZ[1] + P[11][16]*SK_MZ[4] - P[11][17]*SK_MZ[3]);
                Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][0]*SH_MAG[1] - P[12][1]*SH_MAG[2] + P[12][3]*SH_MAG[0] + P[12][2]*SK_MZ[2] + P[12][18]*SK_MZ[1] + P[12][16]*SK_MZ[4] - P[12][17]*SK_MZ[3]);
            } else {
                // zero indexes 10 to 12 = 3*4 bytes
                memset(&Kfusion[10], 0, 12);
            }

            if (!inhibitDelVelBiasStates) {
                Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] - P[13][1]*SH_MAG[2] + P[13][3]*SH_MAG[0] + P[13][2]*SK_MZ[2] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[4] - P[13][17]*SK_MZ[3]);
                Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] - P[14][1]*SH_MAG[2] + P[14][3]*SH_MAG[0] + P[14][2]*SK_MZ[2] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[4] - P[14][17]*SK_MZ[3]);
                Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] - P[15][1]*SH_MAG[2] + P[15][3]*SH_MAG[0] + P[15][2]*SK_MZ[2] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[4] - P[15][17]*SK_MZ[3]);
            } else {
                // zero indexes 13 to 15 = 3*4 bytes
                memset(&Kfusion[13], 0, 12);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!inhibitMagStates) {
                Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] - P[16][1]*SH_MAG[2] + P[16][3]*SH_MAG[0] + P[16][2]*SK_MZ[2] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[4] - P[16][17]*SK_MZ[3]);
                Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] - P[17][1]*SH_MAG[2] + P[17][3]*SH_MAG[0] + P[17][2]*SK_MZ[2] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[4] - P[17][17]*SK_MZ[3]);
                Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] - P[18][1]*SH_MAG[2] + P[18][3]*SH_MAG[0] + P[18][2]*SK_MZ[2] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[4] - P[18][17]*SK_MZ[3]);
                Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] - P[19][1]*SH_MAG[2] + P[19][3]*SH_MAG[0] + P[19][2]*SK_MZ[2] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[4] - P[19][17]*SK_MZ[3]);
                Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] - P[20][1]*SH_MAG[2] + P[20][3]*SH_MAG[0] + P[20][2]*SK_MZ[2] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[4] - P[20][17]*SK_MZ[3]);
                Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] - P[21][1]*SH_MAG[2] + P[21][3]*SH_MAG[0] + P[21][2]*SK_MZ[2] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[4] - P[21][17]*SK_MZ[3]);
            } else {
                // zero indexes 16 to 21 = 6*4 bytes
                memset(&Kfusion[16], 0, 24);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!inhibitWindStates) {
                Kfusion[22] = SK_MZ[0]*(P[22][21] + P[22][0]*SH_MAG[1] - P[22][1]*SH_MAG[2] + P[22][3]*SH_MAG[0] + P[22][2]*SK_MZ[2] + P[22][18]*SK_MZ[1] + P[22][16]*SK_MZ[4] - P[22][17]*SK_MZ[3]);
                Kfusion[23] = SK_MZ[0]*(P[23][21] + P[23][0]*SH_MAG[1] - P[23][1]*SH_MAG[2] + P[23][3]*SH_MAG[0] + P[23][2]*SK_MZ[2] + P[23][18]*SK_MZ[1] + P[23][16]*SK_MZ[4] - P[23][17]*SK_MZ[3]);
            } else {
                // zero indexes 22 to 23 = 2*4 bytes
                memset(&Kfusion[22], 0, 8);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            magFusePerformed = true;
        }
        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=3; j++) {
                KH[i][j] = Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 4; j<=15; j++) {
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
                res += KH[i][3] * P[3][j];
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

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
            }
            stateStruct.quat.normalize();

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
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the magnetic poles)
*/
void NavEKF3_core::fuseEulerYaw()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2)
    const float R_YAW = sq(frontend->_yawNoise);

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    // determine if a 321 or 312 Euler sequence is best
    float predicted_yaw;
    float H_YAW[4];
    Matrix3f Tbn_zeroYaw;
    if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
        // calculate observation jacobian when we are observing the first rotation in a 321 sequence
        float t9 = q0*q3;
        float t10 = q1*q2;
        float t2 = t9+t10;
        float t3 = q0*q0;
        float t4 = q1*q1;
        float t5 = q2*q2;
        float t6 = q3*q3;
        float t7 = t3+t4-t5-t6;
        float t8 = t7*t7;
        if (t8 > 1e-6f) {
            t8 = 1.0f/t8;
        } else {
            return;
        }
        float t11 = t2*t2;
        float t12 = t8*t11*4.0f;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-6f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }

        H_YAW[0] = t8*t14*(q3*t3-q3*t4+q3*t5+q3*t6+q0*q1*q2*2.0f)*-2.0f;
        H_YAW[1] = t8*t14*(-q2*t3+q2*t4+q2*t5+q2*t6+q0*q1*q3*2.0f)*-2.0f;
        H_YAW[2] = t8*t14*(q1*t3+q1*t4+q1*t5-q1*t6+q0*q2*q3*2.0f)*2.0f;
        H_YAW[3] = t8*t14*(q0*t3+q0*t4-q0*t5+q0*t6+q1*q2*q3*2.0f)*2.0f;

        // Get the 321 euler angles
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        predicted_yaw = euler321.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);

    } else {
        // calculate observation jacobian when we are observing a rotation in a 312 sequence
        float t9 = q0*q3;
        float t10 = q1*q2;
        float t2 = t9-t10;
        float t3 = q0*q0;
        float t4 = q1*q1;
        float t5 = q2*q2;
        float t6 = q3*q3;
        float t7 = t3-t4+t5-t6;
        float t8 = t7*t7;
        if (t8 > 1e-6f) {
            t8 = 1.0f/t8;
        } else {
            return;
        }
        float t11 = t2*t2;
        float t12 = t8*t11*4.0f;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-6f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }

        H_YAW[0] = t8*t14*(q3*t3+q3*t4-q3*t5+q3*t6-q0*q1*q2*2.0f)*-2.0f;
        H_YAW[1] = t8*t14*(q2*t3+q2*t4+q2*t5-q2*t6-q0*q1*q3*2.0f)*-2.0f;
        H_YAW[2] = t8*t14*(-q1*t3+q1*t4+q1*t5+q1*t6-q0*q2*q3*2.0f)*2.0f;
        H_YAW[3] = t8*t14*(q0*t3-q0*t4+q0*t5+q0*t6-q1*q2*q3*2.0f)*2.0f;

        // Get the 321 euler angles
        Vector3f euler312 = stateStruct.quat.to_vector312();
        predicted_yaw = euler312.z;

        // set the yaw to zero and calculate the zero yaw rotation from body to earth frame
        Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
    }

    // rotate measured mag components into earth frame
    Vector3f magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;

    // Use the difference between the horizontal projection and declination to give the measured yaw
    // If we can't use compass data, set the  measurement to the predicted
    // to prevent uncontrolled variance growth whilst on ground without magnetometer
    float measured_yaw;
    if (use_compass() && yawAlignComplete) {
        measured_yaw = wrap_PI(-atan2f(magMeasNED.y, magMeasNED.x) + _ahrs->get_compass()->get_declination());
    } else {
        measured_yaw = predicted_yaw;
    }

    // Calculate the innovation
    float innovation = wrap_PI(predicted_yaw - measured_yaw);

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
    float PH[4];
    float varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=3; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=3; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
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
        for (uint8_t colIndex=0; colIndex<=3; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * varInnov);

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
        for (uint8_t column = 0; column <= 3; column++) {
            KH[row][column] = Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            float tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            tmp += KH[row][3] * P[3][column];
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

        // correct the state vector
        for (uint8_t i=0; i<=stateIndexLim; i++) {
            statesArray[i] -= Kfusion[i] * innovation;
        }
        stateStruct.quat.normalize();

        // record fusion numerical health status
        faultStatus.bad_yaw = false;

    } else {
        // record fusion numerical health status
        faultStatus.bad_yaw = true;
    }
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF3_core::FuseDeclination(float declErr)
{
    // declination error variance (rad^2)
    const float R_DECL = sq(declErr);

    // copy required states to local variables
    float magN = stateStruct.earth_magfield.x;
    float magE = stateStruct.earth_magfield.y;

    // prevent bad earth field states from causing numerical errors or exceptions
    if (magN < 1e-3f) {
        return;
    }

    // Calculate observation Jacobian and Kalman gains
    // Calculate intermediate variables
    float t2 = magE*magE;
    float t3 = magN*magN;
    float t4 = t2+t3;
    // if the horizontal magnetic field is too small, this calculation will be badly conditioned
    if (t4 < 1e-4f) {
        return;
    }
    float t5 = P[16][16]*t2;
    float t6 = P[17][17]*t3;
    float t7 = t2*t2;
    float t8 = R_DECL*t7;
    float t9 = t3*t3;
    float t10 = R_DECL*t9;
    float t11 = R_DECL*t2*t3*2.0f;
    float t14 = P[16][17]*magE*magN;
    float t15 = P[17][16]*magE*magN;
    float t12 = t5+t6+t8+t10+t11-t14-t15;
    float t13;
    if (fabsf(t12) > 1e-6f) {
        t13 = 1.0f / t12;
    } else {
        return;
    }
    float t18 = magE*magE;
    float t19 = magN*magN;
    float t20 = t18+t19;
    float t21;
    if (fabsf(t20) > 1e-6f) {
        t21 = 1.0f/t20;
    } else {
        return;
    }

    // Calculate the observation Jacobian
    // Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
    float H_DECL[24] = {};
    H_DECL[16] = -magE*t21;
    H_DECL[17] = magN*t21;

    Kfusion[0] = -t4*t13*(P[0][16]*magE-P[0][17]*magN);
    Kfusion[1] = -t4*t13*(P[1][16]*magE-P[1][17]*magN);
    Kfusion[2] = -t4*t13*(P[2][16]*magE-P[2][17]*magN);
    Kfusion[3] = -t4*t13*(P[3][16]*magE-P[3][17]*magN);
    Kfusion[4] = -t4*t13*(P[4][16]*magE-P[4][17]*magN);
    Kfusion[5] = -t4*t13*(P[5][16]*magE-P[5][17]*magN);
    Kfusion[6] = -t4*t13*(P[6][16]*magE-P[6][17]*magN);
    Kfusion[7] = -t4*t13*(P[7][16]*magE-P[7][17]*magN);
    Kfusion[8] = -t4*t13*(P[8][16]*magE-P[8][17]*magN);
    Kfusion[9] = -t4*t13*(P[9][16]*magE-P[9][17]*magN);

    if (!inhibitDelAngBiasStates) {
        Kfusion[10] = -t4*t13*(P[10][16]*magE-P[10][17]*magN);
        Kfusion[11] = -t4*t13*(P[11][16]*magE-P[11][17]*magN);
        Kfusion[12] = -t4*t13*(P[12][16]*magE-P[12][17]*magN);
    } else {
        // zero indexes 10 to 12 = 3*4 bytes
        memset(&Kfusion[10], 0, 12);
    }

    if (!inhibitDelVelBiasStates) {
        Kfusion[13] = -t4*t13*(P[13][16]*magE-P[13][17]*magN);
        Kfusion[14] = -t4*t13*(P[14][16]*magE-P[14][17]*magN);
        Kfusion[15] = -t4*t13*(P[15][16]*magE-P[15][17]*magN);
    } else {
        // zero indexes 13 to 15 = 3*4 bytes
        memset(&Kfusion[13], 0, 12);
    }

    if (!inhibitMagStates) {
        Kfusion[16] = -t4*t13*(P[16][16]*magE-P[16][17]*magN);
        Kfusion[17] = -t4*t13*(P[17][16]*magE-P[17][17]*magN);
        Kfusion[18] = -t4*t13*(P[18][16]*magE-P[18][17]*magN);
        Kfusion[19] = -t4*t13*(P[19][16]*magE-P[19][17]*magN);
        Kfusion[20] = -t4*t13*(P[20][16]*magE-P[20][17]*magN);
        Kfusion[21] = -t4*t13*(P[21][16]*magE-P[21][17]*magN);
    } else {
        // zero indexes 16 to 21 = 6*4 bytes
        memset(&Kfusion[16], 0, 24);
    }

    if (!inhibitWindStates) {
        Kfusion[22] = -t4*t13*(P[22][16]*magE-P[22][17]*magN);
        Kfusion[23] = -t4*t13*(P[23][16]*magE-P[23][17]*magN);
    } else {
        // zero indexes 22 to 23 = 2*4 bytes
        memset(&Kfusion[22], 0, 8);
    }

    // get the magnetic declination
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // Calculate the innovation
    float innovation = atan2f(magE , magN) - magDecAng;

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
        KH[i][16] = Kfusion[i] * H_DECL[16];
        KH[i][17] = Kfusion[i] * H_DECL[17];
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

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
        }
        stateStruct.quat.normalize();

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
void NavEKF3_core::alignMagStateDeclination()
{
    // don't do this if we already have a learned magnetic field
    if (magFieldLearned) {
        return;
    }

    // get the magnetic declination
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = norm(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);

    if (!inhibitMagStates) {
        // zero the corresponding state covariances if magnetic field state learning is active
        float var_16 = P[16][16];
        float var_17 = P[17][17];
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
void NavEKF3_core::recordMagReset()
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

