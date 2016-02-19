/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF2_core::controlMagYawReset()
{
    // Use a quaternion division to calcualte the delta quaternion between the rotation at the current and last time
    Quaternion deltaQuat = stateStruct.quat / prevQuatMagReset;
    prevQuatMagReset = stateStruct.quat;
    // convert the quaternion to a rotation vector and find its length
    Vector3f deltaRotVec;
    deltaQuat.to_axis_angle(deltaRotVec);
    float deltaRot = deltaRotVec.length();

    // In-Flight reset for vehicle that cannot use a zero sideslip assumption
    // Monitor the gain in height and reset the magnetic field states and heading when initial altitude has been gained
    // This is done to prevent magnetic field distoration from steel roofs and adjacent structures causing bad earth field and initial yaw values
    // Delay if rotated too far since the last check as rapid rotations will produce errors in the magnetic field states
    if (!assume_zero_sideslip() && inFlight && !firstMagYawInit && (stateStruct.position.z  - posDownAtTakeoff) < -5.0f && deltaRot < 0.1745f) {
        firstMagYawInit = true;
        // reset the timer used to prevent magnetometer fusion from affecting attitude until initial field learning is complete
        magFuseTiltInhibit_ms =  imuSampleTime_ms;
        // Update the yaw  angle and earth field states using the magnetic field measurements
        Quaternion tempQuat;
        Vector3f eulerAngles;
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        tempQuat = stateStruct.quat;
        stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        // calculate the change in the quaternion state and apply it to the ouput history buffer
        tempQuat = stateStruct.quat/tempQuat;
        StoreQuatRotate(tempQuat);
    }

    // In-Flight reset for vehicles that can use a zero sideslip assumption (Planes)
    // this is done to protect against unrecoverable heading alignment errors due to compass faults
    if (assume_zero_sideslip() && inFlight && !firstMagYawInit) {
        alignYawGPS();
        firstMagYawInit = true;
    }

    // inhibit the 3-axis mag fusion from modifying the tilt states for the first few seconds after a mag field reset
    // to allow the mag states to converge and prevent disturbances in roll and pitch.
    if (imuSampleTime_ms - magFuseTiltInhibit_ms < 5000) {
        magFuseTiltInhibit = true;
    } else {
        magFuseTiltInhibit = false;
    }

}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF2_core::alignYawGPS()
{
    // get quaternion from existing filter states and calculate roll, pitch and yaw angles
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {

        // calculate course yaw angle
        float velYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        float gpsYaw = atan2f(gpsDataNew.vel.y,gpsDataNew.vel.x);

        // Check the yaw angles for consistency
        float yawErr = MAX(fabsf(wrap_PI(gpsYaw - velYaw)),MAX(fabsf(wrap_PI(gpsYaw - eulerAngles.z)),fabsf(wrap_PI(velYaw - eulerAngles.z))));

        // If the angles disagree by more than 45 degrees and GPS innovations are large, we declare the magnetic yaw as bad
        badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f));

        // correct yaw angle using GPS ground course compass failed or if not previously aligned
        if (badMagYaw) {

            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, gpsYaw);

            // The correlations between attitude errors and positon and velocity errors in the covariance matrix
            // are invalid becasue og the changed yaw angle, so reset the corresponding row and columns
            zeroCols(P,0,2);
            zeroRows(P,0,2);

            // Set the initial attitude error covariances
            P[1][1] = P[0][0] = sq(radians(5.0f));
            P[2][2] = sq(radians(45.0f));

            // reset tposition fusion timer to casue the states to be reset to the GPS on the next GPS fusion cycle
            lastPosPassTime_ms = 0;
        }
    }
    // reset the magnetometer field states - we could have got bad external interference when initialising on-ground
    calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

    // We shoud retry the primary magnetoemter if previously switched or failed
    magSelectIndex = 0;
    allMagSensorsFailed = false;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF2_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been perfomred on that time step
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

    if (magDataToFuse) {
        // Control reset of yaw and magnetic field states
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // If we haven't performed the first airborne magnetic field update or have inhibited magnetic field learning, then we use the simple method of declination to maintain heading
        if(inhibitMagStates) {
            fuseCompass();
            // zero the test ratio output from the inactive 3-axis magneteometer fusion
            magTestRatio.zero();
        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            if (PV_AidingMode != AID_ABSOLUTE || (imuSampleTime_ms - lastPosPassTime_ms) > 4000) {
                FuseDeclination();
            }
            // fuse the three magnetometer componenents sequentially
            for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) {
                hal.util->perf_begin(_perf_test[0]);
                FuseMagnetometer();
                hal.util->perf_end(_perf_test[0]);
                // don't continue fusion if unhealthy
                if (!magHealth) {
                    break;
                }
            }
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseMagnetometer()
{
    hal.util->perf_begin(_perf_test[1]);
    
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
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    hal.util->perf_end(_perf_test[1]);
    
    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains
    if (obsIndex == 0)
    {

        hal.util->perf_begin(_perf_test[2]);

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
            obsIndex = 1;
            faultStatus.bad_xmag = true;

            hal.util->perf_end(_perf_test[2]);

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
            obsIndex = 2;
            faultStatus.bad_ymag = true;

            hal.util->perf_end(_perf_test[2]);

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
            obsIndex = 3;
            faultStatus.bad_zmag = true;

            hal.util->perf_end(_perf_test[2]);

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

        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
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
        // end perf block

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

        // reset the observation index to 0 (we start by fusing the X measurement)
        obsIndex = 0;

        // set flags to indicate to other processes that fusion has been performed and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;

        hal.util->perf_end(_perf_test[2]);

    }
    else if (obsIndex == 1) // we are now fusing the Y measurement
    {

        hal.util->perf_begin(_perf_test[3]);

        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
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

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;

        hal.util->perf_end(_perf_test[3]);

    }
    else if (obsIndex == 2) // we are now fusing the Z measurement
    {

        hal.util->perf_begin(_perf_test[4]);

        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
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

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = false;

        hal.util->perf_end(_perf_test[4]);

    }

    hal.util->perf_begin(_perf_test[5]);

    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
    stateStruct.angErr.zero();

    // correct the state vector
    for (uint8_t j= 0; j<=stateIndexLim; j++) {
        statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
    }

    // Inhibit corrections to tilt if requested. This enables mag states to settle after a reset without causing sudden changes in roll and pitch
    if (magFuseTiltInhibit) {
        stateStruct.angErr.x = 0.0f;
        stateStruct.angErr.y = 0.0f;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    stateStruct.quat.rotate(stateStruct.angErr);

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
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            P[i][j] = P[i][j] - KHP[i][j];
        }
    }
     // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    hal.util->perf_end(_perf_test[5]);

}


/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computatonally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magneometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degreees latitude of the the magnetic poles)
*/
void NavEKF2_core::fuseCompass()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2)
    const float R_MAG = 3e-2f;

    // Calculate observation Jacobian
    float t2 = q0 * q0;
    float t3 = q1 * q1;
    float t4 = q2 * q2;
    float t5 = q3 * q3;
    float t6 = t2 + t3 - t4 - t5;
    float t7 = q0 * q3 * 2.0f;
    float t8 = q1 * q2 * 2.0f;
    float t9 = t7 + t8;
    float t10;
    if (fabsf(t6) > 1e-6f) {
        t10 = 1.0f / (t6 * t6);
    } else  {
        return;
    }
    float t11 = t9 * t9;
    float t12 = t10 * t11;
    float t13 = t12 + 1.0f;
    float t14;
    if (fabsf(t13) > 1e-6f) {
        t14 = 1.0f / t13;
    } else  {
        return;
    }
    float t15 = 1.0f / t6;
    float H_MAG[3];
    H_MAG[0] = 0.0f;
    H_MAG[1] = t14*(t15*(q0*q1*2.0f-q2*q3*2.0f)+t9*t10*(q0*q2*2.0f+q1*q3*2.0f));
    H_MAG[2] = t14*(t15*(t2-t3+t4-t5)+t9*t10*(t7-t8));

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_MAG;
    for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_MAG[colIndex];
        }
        varInnov += H_MAG[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
    if (varInnov >= R_MAG) {
        varInnovInv = 1.0f / varInnov;
        // All three magnetometer components are used in this measurement, so we output health status on three axes
        faultStatus.bad_xmag = false;
        faultStatus.bad_ymag = false;
        faultStatus.bad_zmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        CovarianceInit();
        // All three magnetometer components are used in this measurement, so we output health status on three axes
        faultStatus.bad_xmag = true;
        faultStatus.bad_ymag = true;
        faultStatus.bad_zmag = true;
        return;
    }
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_MAG[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // Calculate the innovation
    float innovation = calcMagHeadingInnov();

    // Copy raw value to output variable used for data logging
    innovYaw = innovation;

    // calculate the innovation test ratio
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (float)frontend->_magInnovGate, 1.0f)) * varInnov);

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

    // correct the state vector
    stateStruct.angErr.zero();
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        statesArray[i] -= Kfusion[i] * innovation;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    stateStruct.quat.rotate(stateStruct.angErr);

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    float HP[24];
    for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++) {
        HP[colIndex] = 0.0f;
        for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
            HP[colIndex] += H_MAG[rowIndex]*P[rowIndex][colIndex];
        }
    }
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++) {
            P[rowIndex][colIndex] -= Kfusion[rowIndex] * HP[colIndex];
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF2_core::FuseDeclination()
{
    // declination error variance (rad^2)
    const float R_DECL = 1e-2f;

    // copy required states to local variables
    float magN = stateStruct.earth_magfield.x;
    float magE = stateStruct.earth_magfield.y;

    // prevent bad earth field states from causing numerical errors or exceptions
    if (magN < 1e-3f) {
        return;
    }

    // Calculate observation Jacobian and Kalman gains
    float t2 = magE*magE;
    float t3 = magN*magN;
    float t4 = t2+t3;
    float t5 = 1.0f/t4;
    float t22 = magE*t5;
    float t23 = magN*t5;
    float t6 = P[16][16]*t22;
    float t13 = P[17][16]*t23;
    float t7 = t6-t13;
    float t8 = t22*t7;
    float t9 = P[16][17]*t22;
    float t14 = P[17][17]*t23;
    float t10 = t9-t14;
    float t15 = t23*t10;
    float t11 = R_DECL+t8-t15; // innovation variance
    float t12 = 1.0f/t11;

    float H_MAG[24];
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
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // Calculate the innovation
    float innovation = atanf(t4) - magDecAng;

    // limit the innovation to protect against data errors
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
    stateStruct.angErr.zero();

    // correct the state vector
    for (uint8_t j= 0; j<=stateIndexLim; j++) {
        statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
    }

    // the first 3 states represent the angular misalignment vector. This is
    // is used to correct the estimated quaternion on the current time step
    stateStruct.quat.rotate(stateStruct.angErr);

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
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            P[i][j] = P[i][j] - KHP[i][j];
        }
    }

    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

}

// Calculate magnetic heading innovation
float NavEKF2_core::calcMagHeadingInnov()
{
    // rotate measured body components into earth axis and compare to declination to give a heading measurement
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    Vector3f magMeasNED = Tbn_temp*magDataDelayed.mag;
    float measHdg = atan2f(magMeasNED.y,magMeasNED.x) - _ahrs->get_compass()->get_declination();

    // wrap the heading so it sits on the range from +-pi
    measHdg = wrap_PI(measHdg);

    // calculate the innovation and wrap between +-pi
    Vector3f eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
    float innovation = wrap_PI(eulerAngles.z - measHdg);

    // Unwrap so that a large yaw gyro bias offset that causes the heading to wrap does not lead to continual uncontrolled heading drift
    if (innovation - lastInnovation > M_PI_F) {
        // Angle has wrapped in the positive direction to subtract an additional 2*Pi
        innovationIncrement -= 2*M_PI_F;
    } else if (innovation -innovationIncrement < -M_PI_F) {
        // Angle has wrapped in the negative direction so add an additional 2*Pi
        innovationIncrement += 2*M_PI_F;
    }
    lastInnovation = innovation;

    return innovation + innovationIncrement;
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavEKF2_core::alignMagStateDeclination()
{
    // get the magnetic declination
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = pythagorous2(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);
}


#endif // HAL_CPU_CLASS
