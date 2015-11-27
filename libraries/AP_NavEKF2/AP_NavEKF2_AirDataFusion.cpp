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

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseAirspeed()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseAirspeed);

    // declarations
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float EAS2TAS = _ahrs->get_EAS2TAS();
    const float R_TAS = sq(constrain_float(frontend->_easNoise, 0.5f, 5.0f) * constrain_float(EAS2TAS, 0.9f, 10.0f));
    Vector3 SH_TAS;
    float SK_TAS;
    Vector24 H_TAS;
    float VtasPred;

    // health is set bad until test passed
    tasHealth = false;

    // copy required states to local variable names
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate the predicted airspeed
    VtasPred = pythagorous3((ve - vwe) , (vn - vwn) , vd);
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // calculate observation jacobians
        SH_TAS[0] = 1/(sqrt(sq(ve - vwe) + sq(vn - vwn) + sq(vd)));
        SH_TAS[1] = (SH_TAS[0]*(2*ve - 2*vwe))/2;
        SH_TAS[2] = (SH_TAS[0]*(2*vn - 2*vwn))/2;
        for (uint8_t i=0; i<=2; i++) H_TAS[i] = 0.0f;
        H_TAS[3] = SH_TAS[2];
        H_TAS[4] = SH_TAS[1];
        H_TAS[5] = vd*SH_TAS[0];
        H_TAS[22] = -SH_TAS[2];
        H_TAS[23] = -SH_TAS[1];
        for (uint8_t i=6; i<=21; i++) H_TAS[i] = 0.0f;
        // calculate Kalman gains
        float temp = (R_TAS + SH_TAS[2]*(P[3][3]*SH_TAS[2] + P[4][3]*SH_TAS[1] - P[22][3]*SH_TAS[2] - P[23][3]*SH_TAS[1] + P[5][3]*vd*SH_TAS[0]) + SH_TAS[1]*(P[3][4]*SH_TAS[2] + P[4][4]*SH_TAS[1] - P[22][4]*SH_TAS[2] - P[23][4]*SH_TAS[1] + P[5][4]*vd*SH_TAS[0]) - SH_TAS[2]*(P[3][22]*SH_TAS[2] + P[4][22]*SH_TAS[1] - P[22][22]*SH_TAS[2] - P[23][22]*SH_TAS[1] + P[5][22]*vd*SH_TAS[0]) - SH_TAS[1]*(P[3][23]*SH_TAS[2] + P[4][23]*SH_TAS[1] - P[22][23]*SH_TAS[2] - P[23][23]*SH_TAS[1] + P[5][23]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[3][5]*SH_TAS[2] + P[4][5]*SH_TAS[1] - P[22][5]*SH_TAS[2] - P[23][5]*SH_TAS[1] + P[5][5]*vd*SH_TAS[0]));
        if (temp >= R_TAS) {
            SK_TAS = 1.0f / temp;
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_airspeed = true;
            return;
        }
        Kfusion[0] = SK_TAS*(P[0][3]*SH_TAS[2] - P[0][22]*SH_TAS[2] + P[0][4]*SH_TAS[1] - P[0][23]*SH_TAS[1] + P[0][5]*vd*SH_TAS[0]);
        Kfusion[1] = SK_TAS*(P[1][3]*SH_TAS[2] - P[1][22]*SH_TAS[2] + P[1][4]*SH_TAS[1] - P[1][23]*SH_TAS[1] + P[1][5]*vd*SH_TAS[0]);
        Kfusion[2] = SK_TAS*(P[2][3]*SH_TAS[2] - P[2][22]*SH_TAS[2] + P[2][4]*SH_TAS[1] - P[2][23]*SH_TAS[1] + P[2][5]*vd*SH_TAS[0]);
        Kfusion[3] = SK_TAS*(P[3][3]*SH_TAS[2] - P[3][22]*SH_TAS[2] + P[3][4]*SH_TAS[1] - P[3][23]*SH_TAS[1] + P[3][5]*vd*SH_TAS[0]);
        Kfusion[4] = SK_TAS*(P[4][3]*SH_TAS[2] - P[4][22]*SH_TAS[2] + P[4][4]*SH_TAS[1] - P[4][23]*SH_TAS[1] + P[4][5]*vd*SH_TAS[0]);
        Kfusion[5] = SK_TAS*(P[5][3]*SH_TAS[2] - P[5][22]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[5][23]*SH_TAS[1] + P[5][5]*vd*SH_TAS[0]);
        Kfusion[6] = SK_TAS*(P[6][3]*SH_TAS[2] - P[6][22]*SH_TAS[2] + P[6][4]*SH_TAS[1] - P[6][23]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]);
        Kfusion[7] = SK_TAS*(P[7][3]*SH_TAS[2] - P[7][22]*SH_TAS[2] + P[7][4]*SH_TAS[1] - P[7][23]*SH_TAS[1] + P[7][5]*vd*SH_TAS[0]);
        Kfusion[8] = SK_TAS*(P[8][3]*SH_TAS[2] - P[8][22]*SH_TAS[2] + P[8][4]*SH_TAS[1] - P[8][23]*SH_TAS[1] + P[8][5]*vd*SH_TAS[0]);
        Kfusion[9] = SK_TAS*(P[9][3]*SH_TAS[2] - P[9][22]*SH_TAS[2] + P[9][4]*SH_TAS[1] - P[9][23]*SH_TAS[1] + P[9][5]*vd*SH_TAS[0]);
        Kfusion[10] = SK_TAS*(P[10][3]*SH_TAS[2] - P[10][22]*SH_TAS[2] + P[10][4]*SH_TAS[1] - P[10][23]*SH_TAS[1] + P[10][5]*vd*SH_TAS[0]);
        Kfusion[11] = SK_TAS*(P[11][3]*SH_TAS[2] - P[11][22]*SH_TAS[2] + P[11][4]*SH_TAS[1] - P[11][23]*SH_TAS[1] + P[11][5]*vd*SH_TAS[0]);
        Kfusion[12] = SK_TAS*(P[12][3]*SH_TAS[2] - P[12][22]*SH_TAS[2] + P[12][4]*SH_TAS[1] - P[12][23]*SH_TAS[1] + P[12][5]*vd*SH_TAS[0]);
        Kfusion[13] = SK_TAS*(P[13][3]*SH_TAS[2] - P[13][22]*SH_TAS[2] + P[13][4]*SH_TAS[1] - P[13][23]*SH_TAS[1] + P[13][5]*vd*SH_TAS[0]);
        Kfusion[14] = SK_TAS*(P[14][3]*SH_TAS[2] - P[14][22]*SH_TAS[2] + P[14][4]*SH_TAS[1] - P[14][23]*SH_TAS[1] + P[14][5]*vd*SH_TAS[0]);
        Kfusion[15] = SK_TAS*(P[15][3]*SH_TAS[2] - P[15][22]*SH_TAS[2] + P[15][4]*SH_TAS[1] - P[15][23]*SH_TAS[1] + P[15][5]*vd*SH_TAS[0]);
        Kfusion[22] = SK_TAS*(P[22][3]*SH_TAS[2] - P[22][22]*SH_TAS[2] + P[22][4]*SH_TAS[1] - P[22][23]*SH_TAS[1] + P[22][5]*vd*SH_TAS[0]);
        Kfusion[23] = SK_TAS*(P[23][3]*SH_TAS[2] - P[23][22]*SH_TAS[2] + P[23][4]*SH_TAS[1] - P[23][23]*SH_TAS[1] + P[23][5]*vd*SH_TAS[0]);
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_TAS*(P[16][3]*SH_TAS[2] - P[16][22]*SH_TAS[2] + P[16][4]*SH_TAS[1] - P[16][23]*SH_TAS[1] + P[16][5]*vd*SH_TAS[0]);
            Kfusion[17] = SK_TAS*(P[17][3]*SH_TAS[2] - P[17][22]*SH_TAS[2] + P[17][4]*SH_TAS[1] - P[17][23]*SH_TAS[1] + P[17][5]*vd*SH_TAS[0]);
            Kfusion[18] = SK_TAS*(P[18][3]*SH_TAS[2] - P[18][22]*SH_TAS[2] + P[18][4]*SH_TAS[1] - P[18][23]*SH_TAS[1] + P[18][5]*vd*SH_TAS[0]);
            Kfusion[19] = SK_TAS*(P[19][3]*SH_TAS[2] - P[19][22]*SH_TAS[2] + P[19][4]*SH_TAS[1] - P[19][23]*SH_TAS[1] + P[19][5]*vd*SH_TAS[0]);
            Kfusion[20] = SK_TAS*(P[20][3]*SH_TAS[2] - P[20][22]*SH_TAS[2] + P[20][4]*SH_TAS[1] - P[20][23]*SH_TAS[1] + P[20][5]*vd*SH_TAS[0]);
            Kfusion[21] = SK_TAS*(P[21][3]*SH_TAS[2] - P[21][22]*SH_TAS[2] + P[21][4]*SH_TAS[1] - P[21][23]*SH_TAS[1] + P[21][5]*vd*SH_TAS[0]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate measurement innovation variance
        varInnovVtas = 1.0f/SK_TAS;

        // calculate measurement innovation
        innovVtas = VtasPred - tasDataDelayed.tas;

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (float)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        tasHealth = ((tasTestRatio < 1.0f) || badIMUdata);
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasHealth || (tasTimeout && posTimeout)) {

            // restart the counter
            lastTasPassTime_ms = imuSampleTime_ms;


            // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
            stateStruct.angErr.zero();

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * innovVtas;
            }

            // the first 3 states represent the angular misalignment vector. This is
            // is used to correct the estimated quaternion on the current time step
            stateStruct.quat.rotate(stateStruct.angErr);

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=2; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 3; j<=5; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (unsigned j = 6; j<=21; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 22; j<=23; j++) {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][3] * P[3][j];
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][22] * P[22][j];
                    res += KH[i][23] * P[23][j];
                    KHP[i][j] = res;
                }
            }
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=stateIndexLim; j++) {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop performance timer
    hal.util->perf_end(_perf_FuseAirspeed);
}

// select fusion of true airspeed measurements
void NavEKF2_core::SelectTasFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data locking out fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !airSpdFusionDelayed) {
        airSpdFusionDelayed = true;
        return;
    } else {
        airSpdFusionDelayed = false;
    }

    // get true airspeed measurement
    readAirSpdData();

    // If we haven't received airspeed data for a while, then declare the airspeed data as being timed out
    if (imuSampleTime_ms - tasDataNew.time_ms > frontend->tasRetryTime_ms) {
        tasTimeout = true;
    }

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion
    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        FuseAirspeed();
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// it requires a stable wind for best results and should not be used for aerobatic flight with manoeuvres that induce large sidslip angles (eg knife-edge, spins, etc)
void NavEKF2_core::SelectBetaFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !sideSlipFusionDelayed) {
        sideSlipFusionDelayed = true;
        return;
    } else {
        sideSlipFusionDelayed = false;
    }

    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaStep_ms) >= frontend->betaAvg_ms);
    // set true when use of synthetic sideslip fusion is necessary because we have limited sensor data or are dead reckoning position
    bool f_required = !(use_compass() && useAirspeed() && ((imuSampleTime_ms - lastPosPassTime_ms) < frontend->gpsRetryTimeNoTAS_ms));
    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    bool f_feasible = (assume_zero_sideslip() && !inhibitWindStates);
    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_feasible && f_required && f_timeTrigger) {
        FuseSideslip();
        prevBetaStep_ms = imuSampleTime_ms;
    }
}

/*
 * Fuse sythetic sideslip measurement of zero using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseSideslip()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseSideslip);

    // declarations
    float q0;
    float q1;
    float q2;
    float q3;
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    const float R_BETA = 0.03f; // assume a sideslip angle RMS of ~10 deg
    Vector10 SH_BETA;
    Vector5 SK_BETA;
    Vector3f vel_rel_wind;
    Vector24 H_BETA;
    float innovBeta;

    // copy required states to local variable names
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    vn = stateStruct.velocity.x;
    ve = stateStruct.velocity.y;
    vd = stateStruct.velocity.z;
    vwn = stateStruct.wind_vel.x;
    vwe = stateStruct.wind_vel.y;

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = vn - vwn;
    vel_rel_wind.y = ve - vwe;
    vel_rel_wind.z = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        if (fabsf(SH_BETA[0]) <= 1e-9f) {
            faultStatus.bad_sideslip = true;
            return;
        } else {
            faultStatus.bad_sideslip = false;
        }
        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        SH_BETA[1] = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - (vn - vwn)*(2*q0*q3 - 2*q1*q2);
        SH_BETA[2] = vd*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) - (ve - vwe)*(2*q0*q1 - 2*q2*q3) + (vn - vwn)*(2*q0*q2 + 2*q1*q3);
        SH_BETA[3] = 1/sq(SH_BETA[0]);
        SH_BETA[4] = (sq(q0) - sq(q1) + sq(q2) - sq(q3))/SH_BETA[0];
        SH_BETA[5] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[6] = 1/SH_BETA[0];
        SH_BETA[7] = 2*q0*q3;
        SH_BETA[8] = SH_BETA[7] + 2*q1*q2;
        SH_BETA[9] = SH_BETA[7] - 2*q1*q2;
        H_BETA[0] = SH_BETA[2]*SH_BETA[6];
        H_BETA[1] = SH_BETA[1]*SH_BETA[2]*SH_BETA[3];
        H_BETA[2] = - sq(SH_BETA[1])*SH_BETA[3] - 1;
        H_BETA[3] = - SH_BETA[6]*SH_BETA[9] - SH_BETA[1]*SH_BETA[3]*SH_BETA[5];
        H_BETA[4] = SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8];
        H_BETA[5] = SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3);
        for (uint8_t i=6; i<=21; i++) {
            H_BETA[i] = 0.0f;
        }
        H_BETA[22] = SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5];
        H_BETA[23] = SH_BETA[1]*SH_BETA[3]*SH_BETA[8] - SH_BETA[4];

        // Calculate Kalman gains
        float temp = (R_BETA + (SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8])*(P[22][4]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][4]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][4]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][4]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][4]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][4]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][4]*SH_BETA[2]*SH_BETA[6] + P[1][4]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) - (SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8])*(P[22][23]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][23]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][23]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][23]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][23]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][23]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][23]*SH_BETA[2]*SH_BETA[6] + P[1][23]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) - (SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5])*(P[22][3]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][3]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][3]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][3]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][3]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][3]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][3]*SH_BETA[2]*SH_BETA[6] + P[1][3]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) + (SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5])*(P[22][22]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][22]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][22]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][22]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][22]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][22]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][22]*SH_BETA[2]*SH_BETA[6] + P[1][22]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) - (sq(SH_BETA[1])*SH_BETA[3] + 1)*(P[22][2]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][2]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][2]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][2]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][2]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][2]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][2]*SH_BETA[2]*SH_BETA[6] + P[1][2]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) + (SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3))*(P[22][5]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][5]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][5]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][5]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][5]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][5]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][5]*SH_BETA[2]*SH_BETA[6] + P[1][5]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) + SH_BETA[2]*SH_BETA[6]*(P[22][0]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][0]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][0]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][0]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][0]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][0]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][0]*SH_BETA[2]*SH_BETA[6] + P[1][0]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]) + SH_BETA[1]*SH_BETA[2]*SH_BETA[3]*(P[22][1]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[3][1]*(SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5]) - P[2][1]*(sq(SH_BETA[1])*SH_BETA[3] + 1) + P[5][1]*(SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3)) + P[4][1]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) - P[23][1]*(SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8]) + P[0][1]*SH_BETA[2]*SH_BETA[6] + P[1][1]*SH_BETA[1]*SH_BETA[2]*SH_BETA[3]));
        if (temp >= R_BETA) {
            SK_BETA[0] = 1.0f / temp;
            faultStatus.bad_sideslip = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_sideslip = true;
            return;
        }
        SK_BETA[1] = SH_BETA[6]*(2*q0*q1 + 2*q2*q3) + SH_BETA[1]*SH_BETA[3]*(2*q0*q2 - 2*q1*q3);
        SK_BETA[2] = SH_BETA[6]*SH_BETA[9] + SH_BETA[1]*SH_BETA[3]*SH_BETA[5];
        SK_BETA[3] = SH_BETA[4] - SH_BETA[1]*SH_BETA[3]*SH_BETA[8];
        SK_BETA[4] = sq(SH_BETA[1])*SH_BETA[3] + 1;
        Kfusion[0] = SK_BETA[0]*(P[0][5]*SK_BETA[1] - P[0][2]*SK_BETA[4] - P[0][3]*SK_BETA[2] + P[0][4]*SK_BETA[3] + P[0][22]*SK_BETA[2] - P[0][23]*SK_BETA[3] + P[0][0]*SH_BETA[6]*SH_BETA[2] + P[0][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[1] = SK_BETA[0]*(P[1][5]*SK_BETA[1] - P[1][2]*SK_BETA[4] - P[1][3]*SK_BETA[2] + P[1][4]*SK_BETA[3] + P[1][22]*SK_BETA[2] - P[1][23]*SK_BETA[3] + P[1][0]*SH_BETA[6]*SH_BETA[2] + P[1][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[2] = SK_BETA[0]*(P[2][5]*SK_BETA[1] - P[2][2]*SK_BETA[4] - P[2][3]*SK_BETA[2] + P[2][4]*SK_BETA[3] + P[2][22]*SK_BETA[2] - P[2][23]*SK_BETA[3] + P[2][0]*SH_BETA[6]*SH_BETA[2] + P[2][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[3] = SK_BETA[0]*(P[3][5]*SK_BETA[1] - P[3][2]*SK_BETA[4] - P[3][3]*SK_BETA[2] + P[3][4]*SK_BETA[3] + P[3][22]*SK_BETA[2] - P[3][23]*SK_BETA[3] + P[3][0]*SH_BETA[6]*SH_BETA[2] + P[3][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[4] = SK_BETA[0]*(P[4][5]*SK_BETA[1] - P[4][2]*SK_BETA[4] - P[4][3]*SK_BETA[2] + P[4][4]*SK_BETA[3] + P[4][22]*SK_BETA[2] - P[4][23]*SK_BETA[3] + P[4][0]*SH_BETA[6]*SH_BETA[2] + P[4][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[5] = SK_BETA[0]*(P[5][5]*SK_BETA[1] - P[5][2]*SK_BETA[4] - P[5][3]*SK_BETA[2] + P[5][4]*SK_BETA[3] + P[5][22]*SK_BETA[2] - P[5][23]*SK_BETA[3] + P[5][0]*SH_BETA[6]*SH_BETA[2] + P[5][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[6] = SK_BETA[0]*(P[6][5]*SK_BETA[1] - P[6][2]*SK_BETA[4] - P[6][3]*SK_BETA[2] + P[6][4]*SK_BETA[3] + P[6][22]*SK_BETA[2] - P[6][23]*SK_BETA[3] + P[6][0]*SH_BETA[6]*SH_BETA[2] + P[6][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[7] = SK_BETA[0]*(P[7][5]*SK_BETA[1] - P[7][2]*SK_BETA[4] - P[7][3]*SK_BETA[2] + P[7][4]*SK_BETA[3] + P[7][22]*SK_BETA[2] - P[7][23]*SK_BETA[3] + P[7][0]*SH_BETA[6]*SH_BETA[2] + P[7][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[8] = SK_BETA[0]*(P[8][5]*SK_BETA[1] - P[8][2]*SK_BETA[4] - P[8][3]*SK_BETA[2] + P[8][4]*SK_BETA[3] + P[8][22]*SK_BETA[2] - P[8][23]*SK_BETA[3] + P[8][0]*SH_BETA[6]*SH_BETA[2] + P[8][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[9] = SK_BETA[0]*(P[9][5]*SK_BETA[1] - P[9][2]*SK_BETA[4] - P[9][3]*SK_BETA[2] + P[9][4]*SK_BETA[3] + P[9][22]*SK_BETA[2] - P[9][23]*SK_BETA[3] + P[9][0]*SH_BETA[6]*SH_BETA[2] + P[9][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[10] = SK_BETA[0]*(P[10][5]*SK_BETA[1] - P[10][2]*SK_BETA[4] - P[10][3]*SK_BETA[2] + P[10][4]*SK_BETA[3] + P[10][22]*SK_BETA[2] - P[10][23]*SK_BETA[3] + P[10][0]*SH_BETA[6]*SH_BETA[2] + P[10][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[11] = SK_BETA[0]*(P[11][5]*SK_BETA[1] - P[11][2]*SK_BETA[4] - P[11][3]*SK_BETA[2] + P[11][4]*SK_BETA[3] + P[11][22]*SK_BETA[2] - P[11][23]*SK_BETA[3] + P[11][0]*SH_BETA[6]*SH_BETA[2] + P[11][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[12] = SK_BETA[0]*(P[12][5]*SK_BETA[1] - P[12][2]*SK_BETA[4] - P[12][3]*SK_BETA[2] + P[12][4]*SK_BETA[3] + P[12][22]*SK_BETA[2] - P[12][23]*SK_BETA[3] + P[12][0]*SH_BETA[6]*SH_BETA[2] + P[12][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[13] = SK_BETA[0]*(P[13][5]*SK_BETA[1] - P[13][2]*SK_BETA[4] - P[13][3]*SK_BETA[2] + P[13][4]*SK_BETA[3] + P[13][22]*SK_BETA[2] - P[13][23]*SK_BETA[3] + P[13][0]*SH_BETA[6]*SH_BETA[2] + P[13][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[14] = SK_BETA[0]*(P[14][5]*SK_BETA[1] - P[14][2]*SK_BETA[4] - P[14][3]*SK_BETA[2] + P[14][4]*SK_BETA[3] + P[14][22]*SK_BETA[2] - P[14][23]*SK_BETA[3] + P[14][0]*SH_BETA[6]*SH_BETA[2] + P[14][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[15] = SK_BETA[0]*(P[15][5]*SK_BETA[1] - P[15][2]*SK_BETA[4] - P[15][3]*SK_BETA[2] + P[15][4]*SK_BETA[3] + P[15][22]*SK_BETA[2] - P[15][23]*SK_BETA[3] + P[15][0]*SH_BETA[6]*SH_BETA[2] + P[15][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[22] = SK_BETA[0]*(P[22][5]*SK_BETA[1] - P[22][2]*SK_BETA[4] - P[22][3]*SK_BETA[2] + P[22][4]*SK_BETA[3] + P[22][22]*SK_BETA[2] - P[22][23]*SK_BETA[3] + P[22][0]*SH_BETA[6]*SH_BETA[2] + P[22][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        Kfusion[23] = SK_BETA[0]*(P[23][5]*SK_BETA[1] - P[23][2]*SK_BETA[4] - P[23][3]*SK_BETA[2] + P[23][4]*SK_BETA[3] + P[23][22]*SK_BETA[2] - P[23][23]*SK_BETA[3] + P[23][0]*SH_BETA[6]*SH_BETA[2] + P[23][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_BETA[0]*(P[16][5]*SK_BETA[1] - P[16][2]*SK_BETA[4] - P[16][3]*SK_BETA[2] + P[16][4]*SK_BETA[3] + P[16][22]*SK_BETA[2] - P[16][23]*SK_BETA[3] + P[16][0]*SH_BETA[6]*SH_BETA[2] + P[16][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
            Kfusion[17] = SK_BETA[0]*(P[17][5]*SK_BETA[1] - P[17][2]*SK_BETA[4] - P[17][3]*SK_BETA[2] + P[17][4]*SK_BETA[3] + P[17][22]*SK_BETA[2] - P[17][23]*SK_BETA[3] + P[17][0]*SH_BETA[6]*SH_BETA[2] + P[17][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
            Kfusion[18] = SK_BETA[0]*(P[18][5]*SK_BETA[1] - P[18][2]*SK_BETA[4] - P[18][3]*SK_BETA[2] + P[18][4]*SK_BETA[3] + P[18][22]*SK_BETA[2] - P[18][23]*SK_BETA[3] + P[18][0]*SH_BETA[6]*SH_BETA[2] + P[18][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
            Kfusion[19] = SK_BETA[0]*(P[19][5]*SK_BETA[1] - P[19][2]*SK_BETA[4] - P[19][3]*SK_BETA[2] + P[19][4]*SK_BETA[3] + P[19][22]*SK_BETA[2] - P[19][23]*SK_BETA[3] + P[19][0]*SH_BETA[6]*SH_BETA[2] + P[19][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
            Kfusion[20] = SK_BETA[0]*(P[20][5]*SK_BETA[1] - P[20][2]*SK_BETA[4] - P[20][3]*SK_BETA[2] + P[20][4]*SK_BETA[3] + P[20][22]*SK_BETA[2] - P[20][23]*SK_BETA[3] + P[20][0]*SH_BETA[6]*SH_BETA[2] + P[20][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
            Kfusion[21] = SK_BETA[0]*(P[21][5]*SK_BETA[1] - P[21][2]*SK_BETA[4] - P[21][3]*SK_BETA[2] + P[21][4]*SK_BETA[3] + P[21][22]*SK_BETA[2] - P[21][23]*SK_BETA[3] + P[21][0]*SH_BETA[6]*SH_BETA[2] + P[21][1]*SH_BETA[1]*SH_BETA[3]*SH_BETA[2]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = vel_rel_wind.y / vel_rel_wind.x;

        // reject measurement if greater than 3-sigma inconsistency
        if (innovBeta > 0.5f) {
            return;
        }

        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovBeta;
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

        // correct the covariance P = (I - K*H)*P
        // take advantage of the empty columns in KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=5; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
            for (unsigned j = 6; j<=21; j++) {
                KH[i][j] = 0.0f;
            }
            for (unsigned j = 22; j<=23; j++) {
                KH[i][j] = Kfusion[i] * H_BETA[j];
            }
        }
        for (unsigned j = 0; j<=stateIndexLim; j++) {
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                ftype res = 0;
                res += KH[i][0] * P[0][j];
                res += KH[i][1] * P[1][j];
                res += KH[i][2] * P[2][j];
                res += KH[i][3] * P[3][j];
                res += KH[i][4] * P[4][j];
                res += KH[i][5] * P[5][j];
                res += KH[i][22] * P[22][j];
                res += KH[i][23] * P[23][j];
                KHP[i][j] = res;
            }
        }
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    // stop the performance timer
    hal.util->perf_end(_perf_FuseSideslip);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/


#endif // HAL_CPU_CLASS
