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
    // Monitor the gain in height and reset the magnetic field states and heading when initial altitude has been gained
    // This is done to prevent magnetic field distoration from steel roofs and adjacent structures causing bad earth field and initial yaw values
    if (inFlight && !firstMagYawInit && (stateStruct.position.z  - posDownAtTakeoff) < -1.5f) {
           // Do the first in-air yaw and earth mag field initialisation when the vehicle has gained 1.5m of altitude after commencement of flight
           Vector3f eulerAngles;
           getEulerAngles(eulerAngles);
           stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
           StoreQuatReset();
           firstMagYawInit = true;
       } else if (inFlight && !secondMagYawInit && (stateStruct.position.z - posDownAtTakeoff) < -5.0f) {
           // Do the second and final yaw and earth mag field initialisation when the vehicle has gained 5.0m of altitude after commencement of flight
           // This second and final correction is needed for flight from large metal structures where the magnetic field distortion can extend up to 5m
           Vector3f eulerAngles;
           getEulerAngles(eulerAngles);
           stateStruct.quat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
           StoreQuatReset();
           secondMagYawInit = true;
       }

    // perform a yaw alignment check against GPS if exiting on-ground mode for fly forward type vehicle (plane)
    // this is done to protect against unrecoverable heading alignment errors due to compass faults
    if (!onGround && prevOnGround && assume_zero_sideslip()) {
        alignYawGPS();
    }
}

// this function is used to do a forced alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF2_core::alignYawGPS()
{
    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        float roll;
        float pitch;
        float oldYaw;
        float newYaw;
        float yawErr;
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        stateStruct.quat.to_euler(roll, pitch, oldYaw);
        // calculate course yaw angle
        oldYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);
        // calculate yaw angle from GPS velocity
        newYaw = atan2f(gpsDataNew.vel.y,gpsDataNew.vel.x);
        // estimate the yaw error
        yawErr = wrap_PI(newYaw - oldYaw);
        // If the inertial course angle disagrees with the GPS by more than 45 degrees, we declare the compass as bad
        badMag = (fabsf(yawErr) > 0.7854f);
        // correct yaw angle using GPS ground course compass failed or if not previously aligned
        if (badMag || !yawAligned) {
            // correct the yaw angle
            newYaw = oldYaw + yawErr;
            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(roll, pitch, newYaw);
            // the yaw angle is now aligned so update its status
            yawAligned =  true;
            // reset the position and velocity states
            ResetPosition();
            ResetVelocity();
            // reset the covariance for the quaternion, velocity and position states
            // zero the matrix entries
            zeroRows(P,0,9);
            zeroCols(P,0,9);
            // velocities - we could have a big error coming out of constant position mode due to GPS lag
            P[3][3]   = 400.0f;
            P[4][4]   = P[3][3];
            P[5][5]   = sq(0.7f);
            // positions - we could have a big error coming out of constant position mode due to GPS lag
            P[6][6]   = 400.0f;
            P[7][7]   = P[6][6];
            P[8][8]   = sq(5.0f);
        }
        // Update magnetic field states if the magnetometer is bad
        if (badMag) {
            Vector3f eulerAngles;
            getEulerAngles(eulerAngles);
            calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);
        }
    }
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
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend.magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    bool temp = RecallMag();

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (temp && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // ensure that the covariance prediction is up to date before fusing data
        if (!covPredStep) CovariancePrediction();
        // If we haven't performed the first airborne magnetic field update or have inhibited magnetic field learning, then we use the simple method of declination to maintain heading
        if(inhibitMagStates) {
            fuseCompass();
            magHealth = true;
            magTimeout = false;
        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            if (PV_AidingMode != AID_ABSOLUTE) {
                FuseDeclination();
            }
            // fuse the three magnetometer componenents sequentially
            for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) {
                hal.util->perf_begin(_perf_test[0]);
                FuseMagnetometer();
                hal.util->perf_end(_perf_test[0]);
            }
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

        // scale magnetometer observation error with total angular rate
        R_MAG = sq(constrain_float(frontend._magNoise, 0.01f, 0.5f)) + sq(frontend.magVarRateScale*imuDataDelayed.delAng.length() / dtIMUavg);

        // calculate observation jacobians
        SH_MAG[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SH_MAG[1] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_MAG[2] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        SH_MAG[3] = 2.0f*q0*q1 + 2.0f*q2*q3;
        SH_MAG[4] = 2.0f*q0*q3 + 2.0f*q1*q2;
        SH_MAG[5] = 2.0f*q0*q2 + 2.0f*q1*q3;
        SH_MAG[6] = magE*(2.0f*q0*q1 - 2.0f*q2*q3);
        SH_MAG[7] = 2.0f*q1*q3 - 2.0f*q0*q2;
        SH_MAG[8] = 2.0f*q0*q3;
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[1] = SH_MAG[6] - magD*SH_MAG[2] - magN*SH_MAG[5];
        H_MAG[2] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
        H_MAG[16] = SH_MAG[1];
        H_MAG[17] = SH_MAG[4];
        H_MAG[18] = SH_MAG[7];
        H_MAG[19] = 1.0f;

        // calculate Kalman gain
        varInnovMag[0] = (P[19][19] + R_MAG - P[1][19]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][19]*SH_MAG[1] + P[17][19]*SH_MAG[4] + P[18][19]*SH_MAG[7] + P[2][19]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) - (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[19][1] - P[1][1]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][1]*SH_MAG[1] + P[17][1]*SH_MAG[4] + P[18][1]*SH_MAG[7] + P[2][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[1]*(P[19][16] - P[1][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][16]*SH_MAG[1] + P[17][16]*SH_MAG[4] + P[18][16]*SH_MAG[7] + P[2][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[4]*(P[19][17] - P[1][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][17]*SH_MAG[1] + P[17][17]*SH_MAG[4] + P[18][17]*SH_MAG[7] + P[2][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[7]*(P[19][18] - P[1][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][18]*SH_MAG[1] + P[17][18]*SH_MAG[4] + P[18][18]*SH_MAG[7] + P[2][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[19][2] - P[1][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][2]*SH_MAG[1] + P[17][2]*SH_MAG[4] + P[18][2]*SH_MAG[7] + P[2][2]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))));
        if (varInnovMag[0] >= R_MAG) {
            SK_MX[0] = 1.0f / varInnovMag[0];
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
        varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][20]*SH_MAG[0] + P[18][20]*SH_MAG[3] - (SH_MAG[8] - 2.0f*q1*q2)*(P[20][16] + P[0][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][16]*SH_MAG[0] + P[18][16]*SH_MAG[3] - P[2][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][16]*(SH_MAG[8] - 2.0f*q1*q2)) - P[2][20]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[20][0] + P[0][0]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][0]*SH_MAG[0] + P[18][0]*SH_MAG[3] - P[2][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][0]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[0]*(P[20][17] + P[0][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][17]*SH_MAG[0] + P[18][17]*SH_MAG[3] - P[2][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][17]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[3]*(P[20][18] + P[0][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][18]*SH_MAG[0] + P[18][18]*SH_MAG[3] - P[2][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][18]*(SH_MAG[8] - 2.0f*q1*q2)) - P[16][20]*(SH_MAG[8] - 2.0f*q1*q2) - (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[20][2] + P[0][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][2]*SH_MAG[0] + P[18][2]*SH_MAG[3] - P[2][2]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][2]*(SH_MAG[8] - 2.0f*q1*q2)));
        if (varInnovMag[1] >= R_MAG) {
            SK_MY[0] = 1.0f / varInnovMag[1];
            faultStatus.bad_ymag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 2;
            faultStatus.bad_ymag = true;
            hal.util->perf_end(_perf_test[3]);
            return;
        }
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
        varInnovMag[2] = (P[21][21] + R_MAG + P[16][21]*SH_MAG[5] + P[18][21]*SH_MAG[2] - (2.0f*q0*q1 - 2.0f*q2*q3)*(P[21][17] + P[16][17]*SH_MAG[5] + P[18][17]*SH_MAG[2] - P[0][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][17]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[0][21]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][21]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + SH_MAG[5]*(P[21][16] + P[16][16]*SH_MAG[5] + P[18][16]*SH_MAG[2] - P[0][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][16]*(2.0f*q0*q1 - 2.0f*q2*q3)) + SH_MAG[2]*(P[21][18] + P[16][18]*SH_MAG[5] + P[18][18]*SH_MAG[2] - P[0][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][18]*(2.0f*q0*q1 - 2.0f*q2*q3)) - (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[21][0] + P[16][0]*SH_MAG[5] + P[18][0]*SH_MAG[2] - P[0][0]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][0]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[17][21]*(2.0f*q0*q1 - 2.0f*q2*q3) + (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[21][1] + P[16][1]*SH_MAG[5] + P[18][1]*SH_MAG[2] - P[0][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][1]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][1]*(2.0f*q0*q1 - 2.0f*q2*q3)));
        if (varInnovMag[2] >= R_MAG) {
            SK_MZ[0] = 1.0f / varInnovMag[2];
            faultStatus.bad_zmag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 3;
            faultStatus.bad_zmag = true;
            hal.util->perf_end(_perf_test[4]);
            return;
        }
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
    // calculate the measurement innovation
    innovMag[obsIndex] = MagPred[obsIndex] - magDataDelayed.mag[obsIndex];
    // calculate the innovation test ratio
    magTestRatio[obsIndex] = sq(innovMag[obsIndex]) / (sq(frontend._magInnovGate) * varInnovMag[obsIndex]);
    // check the last values from all components and set magnetometer health accordingly
    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);
    // Don't fuse unless all componenets pass. The exception is if the bad health has timed out and we are not a fly forward vehicle
    // In this case we might as well try using the magnetometer, but with a reduced weighting
    if (magHealth || ((magTestRatio[obsIndex] < 1.0f) && !assume_zero_sideslip() && magTimeout)) {

        hal.util->perf_begin(_perf_test[8]);
        
        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            // If we are forced to use a bad compass in flight, we reduce the weighting by a factor of 4
            if (!magHealth && (PV_AidingMode == AID_ABSOLUTE)) {
                Kfusion[j] *= 0.25f;
            }
            // If in the air and there is no other form of heading reference or we are yawing rapidly which creates larger inertial yaw errors,
            // we strengthen the magnetometer attitude correction
            if (motorsArmed && ((PV_AidingMode == AID_NONE) || highYawRate) && j <= 3) {
                Kfusion[j] *= 4.0f;
            }
            statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
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
        hal.util->perf_end(_perf_test[8]);
        hal.util->perf_begin(_perf_test[9]);
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
        hal.util->perf_end(_perf_test[9]);
    }

    hal.util->perf_end(_perf_test[5]);
    // force the covariance matrix to be symmetrical and limit the variances to prevent
    // ill-condiioning.
    hal.util->perf_begin(_perf_test[6]);
    ForceSymmetry();
    hal.util->perf_end(_perf_test[6]);
    hal.util->perf_begin(_perf_test[7]);
    ConstrainVariances();
    hal.util->perf_end(_perf_test[7]);
}


/*
 * Fuse compass measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computatonally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magneometer failures.
*/
void NavEKF2_core::fuseCompass()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    float magX = magDataDelayed.mag.x;
    float magY = magDataDelayed.mag.y;
    float magZ = magDataDelayed.mag.z;

    // compass measurement error variance (rad^2)
    const float R_MAG = 3e-2f;

    // Calculate observation Jacobian
    float t2 = q0*q0;
    float t3 = q1*q1;
    float t4 = q2*q2;
    float t5 = q3*q3;
    float t6 = q0*q2*2.0f;
    float t7 = q1*q3*2.0f;
    float t8 = t6+t7;
    float t9 = q0*q3*2.0f;
    float t13 = q1*q2*2.0f;
    float t10 = t9-t13;
    float t11 = t2+t3-t4-t5;
    float t12 = magX*t11;
    float t14 = magZ*t8;
    float t19 = magY*t10;
    float t15 = t12+t14-t19;
    float t16 = t2-t3+t4-t5;
    float t17 = q0*q1*2.0f;
    float t24 = q2*q3*2.0f;
    float t18 = t17-t24;
    float t20 = 1.0f/t15;
    float t21 = magY*t16;
    float t22 = t9+t13;
    float t23 = magX*t22;
    float t28 = magZ*t18;
    float t25 = t21+t23-t28;
    float t29 = t20*t25;
    float t26 = tan(t29);
    float t27 = 1.0f/(t15*t15);
    float t30 = t26*t26;
    float t31 = t30+1.0f;
    float H_MAG[3];
    H_MAG[0] = -t31*(t20*(magZ*t16+magY*t18)+t25*t27*(magY*t8+magZ*t10));
    H_MAG[1] = t31*(t20*(magX*t18+magZ*t22)+t25*t27*(magX*t8-magZ*t11));
    H_MAG[2] = t31*(t20*(magX*t16-magY*t22)+t25*t27*(magX*t10+magY*t11));

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
    float varInnovInv = 1.0f / varInnov;
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
    float t2 = 1.0f/magN;
    float t4 = magE*t2;
    float t3 = tanf(t4);
    float t5 = t3*t3;
    float t6 = t5+1.0f;
    float t7 = 1.0f/(magN*magN);
    float t8 = P[17][17]*t2*t6;
    float t15 = P[16][17]*magE*t6*t7;
    float t9 = t8-t15;
    float t10 = t2*t6*t9;
    float t11 = P[17][16]*t2*t6;
    float t16 = P[16][16]*magE*t6*t7;
    float t12 = t11-t16;
    float t17 = magE*t6*t7*t12;
    float t13 = R_DECL+t10-t17;
    float t14 = 1.0f/t13;
    float t18 = magE;
    float t19 = magN;
    float t21 = 1.0f/t19;
    float t22 = t18*t21;
    float t20 = tanf(t22);
    float t23 = t20*t20;
    float t24 = t23+1.0f;

    float H_MAG[24];
    H_MAG[16] = -t18*1.0f/(t19*t19)*t24;
    H_MAG[17] = t21*t24;

    for (uint8_t i=0; i<=15; i++) {
        Kfusion[i] = t14*(P[i][17]*t2*t6-P[i][16]*magE*t6*t7);
    }
    Kfusion[16] = -t14*(t16-P[16][17]*t2*t6);
    Kfusion[17] = t14*(t8-P[17][16]*magE*t6*t7);
    for (uint8_t i=17; i<=23; i++) {
        Kfusion[i] = t14*(P[i][17]*t2*t6-P[i][16]*magE*t6*t7);
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
    // rotate predicted earth components into body axes and calculate
    // predicted measurements
    Matrix3f Tbn_temp;
    stateStruct.quat.rotation_matrix(Tbn_temp);
    Vector3f magMeasNED = Tbn_temp*magDataDelayed.mag;

    // calculate the innovation where the predicted measurement is the angle wrt magnetic north of the horizontal component of the measured field
    float innovation = atan2f(magMeasNED.y,magMeasNED.x) - _ahrs->get_compass()->get_declination();

    // wrap the innovation so it sits on the range from +-pi
    if (innovation > M_PI_F) {
        innovation = innovation - 2*M_PI_F;
    } else if (innovation < -M_PI_F) {
        innovation = innovation + 2*M_PI_F;
    }

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
