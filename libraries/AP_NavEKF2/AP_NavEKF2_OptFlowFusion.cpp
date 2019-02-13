#include <AP_HAL/AP_HAL.h>

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

// select fusion of optical flow measurements
void NavEKF2_core::SelectFlowFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !optFlowFusionDelayed) {
        optFlowFusionDelayed = true;
        return;
    } else {
        optFlowFusionDelayed = false;
    }

    // start performance timer
    hal.util->perf_begin(_perf_FuseOptFlow);
    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // check is the terrain offset estimate is still valid - if we are using range finder as the main height reference, the ground is assumed to be at 0
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000) || (activeHgtSource == HGT_SOURCE_RNG);
    // Perform tilt check
    bool tiltOK = (prevTnb.c.z > frontend->DCM33FlowMin);
    // Constrain measurements to zero if takeoff is not detected and the height above ground
    // is insuffient to achieve acceptable focus. This allows the vehicle to be picked up
    // and carried to test optical flow operation
    if (!takeOffDetected && ((terrainState - stateStruct.position.z) < 0.5f)) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // if we do have valid flow measurements, fuse data into a 1-state EKF to estimate terrain height
    // we don't do terrain height estimation in optical flow only mode as the ground becomes our zero height reference
    if ((flowDataToFuse || rangeDataToFuse) && tiltOK) {
        // fuse optical flow data into the terrain estimator if available and if there is no range data (range data is better)
        fuseOptFlowData = (flowDataToFuse && !rangeDataToFuse);
        // Estimate the terrain offset (runs a one state EKF)
        if (frontend->_flowUseMask & (1<<1)) {
            EstimateTerrainOffset();
        }
    }

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && tiltOK) {
        if (frontend->_flowUseMask & (1<<0)) {
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
        // Fuse the optical flow X and Y axis data into the main filter sequentially
            FuseOptFlow();
        }
        // reset flag to indicate that no new flow data is available for fusion
        flowDataToFuse = false;
    }

    // stop the performance timer
    hal.util->perf_end(_perf_FuseOptFlow);
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optical flow rates and range finder measurements
Equations generated using https://github.com/PX4/ecl/tree/master/EKF/matlab/scripts/Terrain%20Estimator
*/
void NavEKF2_core::EstimateTerrainOffset()
{
    // start performance timer
    hal.util->perf_begin(_perf_TerrainOffset);

    // calculate a predicted LOS rate squared
    float velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);

    // don't fuse flow data if LOS rate is misaligned, without GPS, or insufficient velocity, as it is poorly observable
    // don't fuse flow data if it exceeds validity limits
    // don't update terrain state if we are using it as a height reference in the main filter
    bool cantFuseFlowData = (gpsNotAvailable || PV_AidingMode == AID_RELATIVE || velHorizSq < 25.0f || (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) > frontend->_maxFlowRate));
    if ((!rangeDataToFuse && cantFuseFlowData) || (activeHgtSource == HGT_SOURCE_RNG)) {
        // skip update
        inhibitGndState = true;
    } else {
        inhibitGndState = false;
        // record the time we last updated the terrain offset state
        gndHgtValidTime_ms = imuSampleTime_ms;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation afer bad gps causing bad numerical conditioning
        float distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have the growth in uncertainty due to the copters vertical velocity
        float timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        float Pincrement = (distanceTravelledSq * sq(frontend->_terrGradMax)) + sq(timeLapsed)*P[5][5];
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (rangeDataToFuse) {
            // predict range
            float predRngMeas = MAX((terrainState - stateStruct.position[2]),rngOnGnd) / prevTnb.c.z;

            // Copy required states to local variable names
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            float R_RNG = frontend->_rngNoise;

            // calculate Kalman gain
            float SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            float K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rangeDataDelayed.rng;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(MAX(0.01f * (float)frontend->_rngInnovGate, 1.0f)) * varInnovRng);

            // Check the innovation test ratio and don't fuse if too large
            if (auxRngTestRatio < 1.0f) {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = MAX(Popt,0.0f);

            }
        }

        if (fuseOptFlowData && !cantFuseFlowData) {

            Vector3f relVelSensor;          // velocity of sensor relative to ground in sensor axes
            Vector2f losPred;               // predicted optical flow angular rate measurement
            float q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            float q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            float q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            float q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            float K_OPT;
            float H_OPT;
            Vector2f auxFlowObsInnovVar;

            // predict range to centre of image
            float flowRngPred = MAX((terrainState - stateStruct.position.z),rngOnGnd) / prevTnb.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = prevTnb*stateStruct.velocity;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred.x =   relVelSensor.y / flowRngPred;
            losPred.y = - relVelSensor.x / flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - ofDataDelayed.flowRadXYcomp;

            // calculate observation jacobians 
            float t2 = q0*q0;
            float t3 = q1*q1;
            float t4 = q2*q2;
            float t5 = q3*q3;
            float t6 = stateStruct.position.z - terrainState;
            float t7 = 1.0f / (t6*t6);
            float t8 = q0*q3*2.0f;
            float t9 = t2-t3-t4+t5;

            // prevent the state variances from becoming badly conditioned
            Popt = MAX(Popt,1E-6f);

            // calculate observation noise variance from parameter
            float flow_noise_variance = sq(MAX(frontend->_flowNoise, 0.05f));

            // Fuse Y axis data

            // Calculate observation partial derivative
            H_OPT = t7*t9*(-stateStruct.velocity.z*(q0*q2*2.0-q1*q3*2.0)+stateStruct.velocity.x*(t2+t3-t4-t5)+stateStruct.velocity.y*(t8+q1*q2*2.0));

            // calculate innovation variance
            auxFlowObsInnovVar.y = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.y;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.y = sq(auxFlowObsInnov.y) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.y);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.y < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.y;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // update intermediate variables used when fusing the X axis
                t6 = stateStruct.position.z - terrainState;
                t7 = 1.0f / (t6*t6);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);
            }

            // fuse X axis data
            H_OPT = -t7*t9*(stateStruct.velocity.z*(q0*q1*2.0+q2*q3*2.0)+stateStruct.velocity.y*(t2-t3+t4-t5)-stateStruct.velocity.x*(t8-q1*q2*2.0));

            // calculate innovation variances
            auxFlowObsInnovVar.x = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.x;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.x = sq(auxFlowObsInnov.x) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.x);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.x < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.x;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);
            }
        }
    }

    // stop the performance timer
    hal.util->perf_end(_perf_TerrainOffset);
}

/*
 * Fuse angular motion compensated optical flow rates using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * Requires a valid terrain height estimate.
*/
void NavEKF2_core::FuseOptFlow()
{
    Vector24 H_LOS;
    Vector3f relVelSensor;
    Vector14 SH_LOS;
    Vector2 losPred;

    // Copy required states to local variable names
    float q0  = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];
    float vn = stateStruct.velocity.x;
    float ve = stateStruct.velocity.y;
    float vd = stateStruct.velocity.z;
    float pd = stateStruct.position.z;

    // constrain height above ground to be above range measured on ground
    float heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);
    float ptd = pd + heightAboveGndEst;

    // Calculate common expressions for observation jacobians
    SH_LOS[0] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    SH_LOS[1] = vn*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + ve*(2*q0*q3 + 2*q1*q2);
    SH_LOS[2] = ve*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - vn*(2*q0*q3 - 2*q1*q2);
    SH_LOS[3] = 1/(pd - ptd);
    SH_LOS[4] = vd*SH_LOS[0] - ve*(2*q0*q1 - 2*q2*q3) + vn*(2*q0*q2 + 2*q1*q3);
    SH_LOS[5] = 2.0f*q0*q2 - 2.0f*q1*q3;
    SH_LOS[6] = 2.0f*q0*q1 + 2.0f*q2*q3;
    SH_LOS[7] = q0*q0;
    SH_LOS[8] = q1*q1;
    SH_LOS[9] = q2*q2;
    SH_LOS[10] = q3*q3;
    SH_LOS[11] = q0*q3*2.0f;
    SH_LOS[12] = pd-ptd;
    SH_LOS[13] = 1.0f/(SH_LOS[12]*SH_LOS[12]);

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first
        // calculate range from ground plain to centre of sensor fov assuming flat earth
        float range = constrain_float((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);

        // correct range for flow sensor offset body frame position offset
        // the corrected value is the predicted range from the sensor focal point to the
        // centre of the image on the ground assuming flat terrain
        Vector3f posOffsetBody = (*ofDataDelayed.body_offset) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {
            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            range -= posOffsetEarth.z / prevTnb.c.z;
        }

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        relVelSensor = prevTnb*stateStruct.velocity + ofDataDelayed.bodyRadXYZ % posOffsetBody;

        // divide velocity by range  to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;

        // calculate observation jacobians and Kalman gains
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        if (obsIndex == 0) {
            H_LOS[0] = SH_LOS[3]*SH_LOS[2]*SH_LOS[6]-SH_LOS[3]*SH_LOS[0]*SH_LOS[4];
            H_LOS[1] = SH_LOS[3]*SH_LOS[2]*SH_LOS[5];
            H_LOS[2] = SH_LOS[3]*SH_LOS[0]*SH_LOS[1];
            H_LOS[3] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[11]-q1*q2*2.0f);
            H_LOS[4] = -SH_LOS[3]*SH_LOS[0]*(SH_LOS[7]-SH_LOS[8]+SH_LOS[9]-SH_LOS[10]);
            H_LOS[5] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[6];
            H_LOS[8] = SH_LOS[2]*SH_LOS[0]*SH_LOS[13];

            float t2 = SH_LOS[3];
            float t3 = SH_LOS[0];
            float t4 = SH_LOS[2];
            float t5 = SH_LOS[6];
            float t100 = t2 * t3 * t5;
            float t6 = SH_LOS[4];
            float t7 = t2*t3*t6;
            float t9 = t2*t4*t5;
            float t8 = t7-t9;
            float t10 = q0*q3*2.0f;
            float t21 = q1*q2*2.0f;
            float t11 = t10-t21;
            float t101 = t2 * t3 * t11;
            float t12 = pd-ptd;
            float t13 = 1.0f/(t12*t12);
            float t104 = t3 * t4 * t13;
            float t14 = SH_LOS[5];
            float t102 = t2 * t4 * t14;
            float t15 = SH_LOS[1];
            float t103 = t2 * t3 * t15;
            float t16 = q0*q0;
            float t17 = q1*q1;
            float t18 = q2*q2;
            float t19 = q3*q3;
            float t20 = t16-t17+t18-t19;
            float t105 = t2 * t3 * t20;
            float t22 = P[1][1]*t102;
            float t23 = P[3][0]*t101;
            float t24 = P[8][0]*t104;
            float t25 = P[1][0]*t102;
            float t26 = P[2][0]*t103;
            float t63 = P[0][0]*t8;
            float t64 = P[5][0]*t100;
            float t65 = P[4][0]*t105;
            float t27 = t23+t24+t25+t26-t63-t64-t65;
            float t28 = P[3][3]*t101;
            float t29 = P[8][3]*t104;
            float t30 = P[1][3]*t102;
            float t31 = P[2][3]*t103;
            float t67 = P[0][3]*t8;
            float t68 = P[5][3]*t100;
            float t69 = P[4][3]*t105;
            float t32 = t28+t29+t30+t31-t67-t68-t69;
            float t33 = t101*t32;
            float t34 = P[3][8]*t101;
            float t35 = P[8][8]*t104;
            float t36 = P[1][8]*t102;
            float t37 = P[2][8]*t103;
            float t70 = P[0][8]*t8;
            float t71 = P[5][8]*t100;
            float t72 = P[4][8]*t105;
            float t38 = t34+t35+t36+t37-t70-t71-t72;
            float t39 = t104*t38;
            float t40 = P[3][1]*t101;
            float t41 = P[8][1]*t104;
            float t42 = P[2][1]*t103;
            float t73 = P[0][1]*t8;
            float t74 = P[5][1]*t100;
            float t75 = P[4][1]*t105;
            float t43 = t22+t40+t41+t42-t73-t74-t75;
            float t44 = t102*t43;
            float t45 = P[3][2]*t101;
            float t46 = P[8][2]*t104;
            float t47 = P[1][2]*t102;
            float t48 = P[2][2]*t103;
            float t76 = P[0][2]*t8;
            float t77 = P[5][2]*t100;
            float t78 = P[4][2]*t105;
            float t49 = t45+t46+t47+t48-t76-t77-t78;
            float t50 = t103*t49;
            float t51 = P[3][5]*t101;
            float t52 = P[8][5]*t104;
            float t53 = P[1][5]*t102;
            float t54 = P[2][5]*t103;
            float t79 = P[0][5]*t8;
            float t80 = P[5][5]*t100;
            float t81 = P[4][5]*t105;
            float t55 = t51+t52+t53+t54-t79-t80-t81;
            float t56 = P[3][4]*t101;
            float t57 = P[8][4]*t104;
            float t58 = P[1][4]*t102;
            float t59 = P[2][4]*t103;
            float t83 = P[0][4]*t8;
            float t84 = P[5][4]*t100;
            float t85 = P[4][4]*t105;
            float t60 = t56+t57+t58+t59-t83-t84-t85;
            float t66 = t8*t27;
            float t82 = t100*t55;
            float t86 = t105*t60;
            float t61 = R_LOS+t33+t39+t44+t50-t66-t82-t86;
            float t62 = 1.0f/t61;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t61 > R_LOS) {
                t62 = 1.0f/t61;
                faultStatus.bad_yflow = false;
            } else {
                t61 = 0.0f;
                t62 = 1.0f/R_LOS;
                faultStatus.bad_yflow = true;
                return;
            }
            varInnovOptFlow[0] = t61;

            // calculate innovation for X axis observation
            innovOptFlow[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t62*(-P[0][0]*t8-P[0][5]*t100+P[0][3]*t101+P[0][1]*t102+P[0][2]*t103+P[0][8]*t104-P[0][4]*t105);
            Kfusion[1] = t62*(t22-P[1][0]*t8-P[1][5]*t100+P[1][3]*t101+P[1][2]*t103+P[1][8]*t104-P[1][4]*t105);
            Kfusion[2] = t62*(t48-P[2][0]*t8-P[2][5]*t100+P[2][3]*t101+P[2][1]*t102+P[2][8]*t104-P[2][4]*t105);
            Kfusion[3] = t62*(t28-P[3][0]*t8-P[3][5]*t100+P[3][1]*t102+P[3][2]*t103+P[3][8]*t104-P[3][4]*t105);
            Kfusion[4] = t62*(-t85-P[4][0]*t8-P[4][5]*t100+P[4][3]*t101+P[4][1]*t102+P[4][2]*t103+P[4][8]*t104);
            Kfusion[5] = t62*(-t80-P[5][0]*t8+P[5][3]*t101+P[5][1]*t102+P[5][2]*t103+P[5][8]*t104-P[5][4]*t105);
            Kfusion[6] = t62*(-P[6][0]*t8-P[6][5]*t100+P[6][3]*t101+P[6][1]*t102+P[6][2]*t103+P[6][8]*t104-P[6][4]*t105);
            Kfusion[7] = t62*(-P[7][0]*t8-P[7][5]*t100+P[7][3]*t101+P[7][1]*t102+P[7][2]*t103+P[7][8]*t104-P[7][4]*t105);
            Kfusion[8] = t62*(t35-P[8][0]*t8-P[8][5]*t100+P[8][3]*t101+P[8][1]*t102+P[8][2]*t103-P[8][4]*t105);
            Kfusion[9] = t62*(-P[9][0]*t8-P[9][5]*t100+P[9][3]*t101+P[9][1]*t102+P[9][2]*t103+P[9][8]*t104-P[9][4]*t105);
            Kfusion[10] = t62*(-P[10][0]*t8-P[10][5]*t100+P[10][3]*t101+P[10][1]*t102+P[10][2]*t103+P[10][8]*t104-P[10][4]*t105);
            Kfusion[11] = t62*(-P[11][0]*t8-P[11][5]*t100+P[11][3]*t101+P[11][1]*t102+P[11][2]*t103+P[11][8]*t104-P[11][4]*t105);
            Kfusion[12] = t62*(-P[12][0]*t8-P[12][5]*t100+P[12][3]*t101+P[12][1]*t102+P[12][2]*t103+P[12][8]*t104-P[12][4]*t105);
            Kfusion[13] = t62*(-P[13][0]*t8-P[13][5]*t100+P[13][3]*t101+P[13][1]*t102+P[13][2]*t103+P[13][8]*t104-P[13][4]*t105);
            Kfusion[14] = t62*(-P[14][0]*t8-P[14][5]*t100+P[14][3]*t101+P[14][1]*t102+P[14][2]*t103+P[14][8]*t104-P[14][4]*t105);
            Kfusion[15] = t62*(-P[15][0]*t8-P[15][5]*t100+P[15][3]*t101+P[15][1]*t102+P[15][2]*t103+P[15][8]*t104-P[15][4]*t105);
            if (!inhibitWindStates) {
                Kfusion[22] = t62*(-P[22][0]*t8-P[22][5]*t100+P[22][3]*t101+P[22][1]*t102+P[22][2]*t103+P[22][8]*t104-P[22][4]*t105);
                Kfusion[23] = t62*(-P[23][0]*t8-P[23][5]*t100+P[23][3]*t101+P[23][1]*t102+P[23][2]*t103+P[23][8]*t104-P[23][4]*t105);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = t62*(-P[16][0]*t8-P[16][5]*t100+P[16][3]*t101+P[16][1]*t102+P[16][2]*t103+P[16][8]*t104-P[16][4]*t105);
                Kfusion[17] = t62*(-P[17][0]*t8-P[17][5]*t100+P[17][3]*t101+P[17][1]*t102+P[17][2]*t103+P[17][8]*t104-P[17][4]*t105);
                Kfusion[18] = t62*(-P[18][0]*t8-P[18][5]*t100+P[18][3]*t101+P[18][1]*t102+P[18][2]*t103+P[18][8]*t104-P[18][4]*t105);
                Kfusion[19] = t62*(-P[19][0]*t8-P[19][5]*t100+P[19][3]*t101+P[19][1]*t102+P[19][2]*t103+P[19][8]*t104-P[19][4]*t105);
                Kfusion[20] = t62*(-P[20][0]*t8-P[20][5]*t100+P[20][3]*t101+P[20][1]*t102+P[20][2]*t103+P[20][8]*t104-P[20][4]*t105);
                Kfusion[21] = t62*(-P[21][0]*t8-P[21][5]*t100+P[21][3]*t101+P[21][1]*t102+P[21][2]*t103+P[21][8]*t104-P[21][4]*t105);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

        } else {

            H_LOS[0] = -SH_LOS[3]*SH_LOS[6]*SH_LOS[1];
            H_LOS[1] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[4]-SH_LOS[3]*SH_LOS[1]*SH_LOS[5];
            H_LOS[2] = SH_LOS[3]*SH_LOS[2]*SH_LOS[0];
            H_LOS[3] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[7]+SH_LOS[8]-SH_LOS[9]-SH_LOS[10]);
            H_LOS[4] = SH_LOS[3]*SH_LOS[0]*(SH_LOS[11]+q1*q2*2.0f);
            H_LOS[5] = -SH_LOS[3]*SH_LOS[0]*SH_LOS[5];
            H_LOS[8] = -SH_LOS[0]*SH_LOS[1]*SH_LOS[13];

            float t2 = SH_LOS[3];
            float t3 = SH_LOS[0];
            float t4 = SH_LOS[1];
            float t5 = SH_LOS[5];
            float t100 = t2 * t3 * t5;
            float t6 = SH_LOS[4];
            float t7 = t2*t3*t6;
            float t8 = t2*t4*t5;
            float t9 = t7+t8;
            float t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t12 = t10+t11;
            float t101 = t2 * t3 * t12;
            float t13 = pd-ptd;
            float t14 = 1.0f/(t13*t13);
            float t104 = t3 * t4 * t14;
            float t15 = SH_LOS[6];
            float t105 = t2 * t4 * t15;
            float t16 = SH_LOS[2];
            float t102 = t2 * t3 * t16;
            float t17 = q0*q0;
            float t18 = q1*q1;
            float t19 = q2*q2;
            float t20 = q3*q3;
            float t21 = t17+t18-t19-t20;
            float t103 = t2 * t3 * t21;
            float t22 = P[0][0]*t105;
            float t23 = P[1][1]*t9;
            float t24 = P[8][1]*t104;
            float t25 = P[0][1]*t105;
            float t26 = P[5][1]*t100;
            float t64 = P[4][1]*t101;
            float t65 = P[2][1]*t102;
            float t66 = P[3][1]*t103;
            float t27 = t23+t24+t25+t26-t64-t65-t66;
            float t28 = t9*t27;
            float t29 = P[1][4]*t9;
            float t30 = P[8][4]*t104;
            float t31 = P[0][4]*t105;
            float t32 = P[5][4]*t100;
            float t67 = P[4][4]*t101;
            float t68 = P[2][4]*t102;
            float t69 = P[3][4]*t103;
            float t33 = t29+t30+t31+t32-t67-t68-t69;
            float t34 = P[1][8]*t9;
            float t35 = P[8][8]*t104;
            float t36 = P[0][8]*t105;
            float t37 = P[5][8]*t100;
            float t71 = P[4][8]*t101;
            float t72 = P[2][8]*t102;
            float t73 = P[3][8]*t103;
            float t38 = t34+t35+t36+t37-t71-t72-t73;
            float t39 = t104*t38;
            float t40 = P[1][0]*t9;
            float t41 = P[8][0]*t104;
            float t42 = P[5][0]*t100;
            float t74 = P[4][0]*t101;
            float t75 = P[2][0]*t102;
            float t76 = P[3][0]*t103;
            float t43 = t22+t40+t41+t42-t74-t75-t76;
            float t44 = t105*t43;
            float t45 = P[1][2]*t9;
            float t46 = P[8][2]*t104;
            float t47 = P[0][2]*t105;
            float t48 = P[5][2]*t100;
            float t63 = P[2][2]*t102;
            float t77 = P[4][2]*t101;
            float t78 = P[3][2]*t103;
            float t49 = t45+t46+t47+t48-t63-t77-t78;
            float t50 = P[1][5]*t9;
            float t51 = P[8][5]*t104;
            float t52 = P[0][5]*t105;
            float t53 = P[5][5]*t100;
            float t80 = P[4][5]*t101;
            float t81 = P[2][5]*t102;
            float t82 = P[3][5]*t103;
            float t54 = t50+t51+t52+t53-t80-t81-t82;
            float t55 = t100*t54;
            float t56 = P[1][3]*t9;
            float t57 = P[8][3]*t104;
            float t58 = P[0][3]*t105;
            float t59 = P[5][3]*t100;
            float t83 = P[4][3]*t101;
            float t84 = P[2][3]*t102;
            float t85 = P[3][3]*t103;
            float t60 = t56+t57+t58+t59-t83-t84-t85;
            float t70 = t101*t33;
            float t79 = t102*t49;
            float t86 = t103*t60;
            float t61 = R_LOS+t28+t39+t44+t55-t70-t79-t86;
            float t62 = 1.0f/t61;

            // calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
            if (t61 > R_LOS) {
                t62 = 1.0f/t61;
                faultStatus.bad_yflow = false;
            } else {
                t61 = 0.0f;
                t62 = 1.0f/R_LOS;
                faultStatus.bad_yflow = true;
                return;
            }
            varInnovOptFlow[1] = t61;

            // calculate innovation for Y observation
            innovOptFlow[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t62*(t22+P[0][1]*t9+P[0][5]*t100-P[0][4]*t101-P[0][2]*t102-P[0][3]*t103+P[0][8]*t104);
            Kfusion[1] = -t62*(t23+P[1][5]*t100+P[1][0]*t105-P[1][4]*t101-P[1][2]*t102-P[1][3]*t103+P[1][8]*t104);
            Kfusion[2] = -t62*(-t63+P[2][1]*t9+P[2][5]*t100+P[2][0]*t105-P[2][4]*t101-P[2][3]*t103+P[2][8]*t104);
            Kfusion[3] = -t62*(-t85+P[3][1]*t9+P[3][5]*t100+P[3][0]*t105-P[3][4]*t101-P[3][2]*t102+P[3][8]*t104);
            Kfusion[4] = -t62*(-t67+P[4][1]*t9+P[4][5]*t100+P[4][0]*t105-P[4][2]*t102-P[4][3]*t103+P[4][8]*t104);
            Kfusion[5] = -t62*(t53+P[5][1]*t9+P[5][0]*t105-P[5][4]*t101-P[5][2]*t102-P[5][3]*t103+P[5][8]*t104);
            Kfusion[6] = -t62*(P[6][1]*t9+P[6][5]*t100+P[6][0]*t105-P[6][4]*t101-P[6][2]*t102-P[6][3]*t103+P[6][8]*t104);
            Kfusion[7] = -t62*(P[7][1]*t9+P[7][5]*t100+P[7][0]*t105-P[7][4]*t101-P[7][2]*t102-P[7][3]*t103+P[7][8]*t104);
            Kfusion[8] = -t62*(t35+P[8][1]*t9+P[8][5]*t100+P[8][0]*t105-P[8][4]*t101-P[8][2]*t102-P[8][3]*t103);
            Kfusion[9] = -t62*(P[9][1]*t9+P[9][5]*t100+P[9][0]*t105-P[9][4]*t101-P[9][2]*t102-P[9][3]*t103+P[9][8]*t104);
            Kfusion[10] = -t62*(P[10][1]*t9+P[10][5]*t100+P[10][0]*t105-P[10][4]*t101-P[10][2]*t102-P[10][3]*t103+P[10][8]*t104);
            Kfusion[11] = -t62*(P[11][1]*t9+P[11][5]*t100+P[11][0]*t105-P[11][4]*t101-P[11][2]*t102-P[11][3]*t103+P[11][8]*t104);
            Kfusion[12] = -t62*(P[12][1]*t9+P[12][5]*t100+P[12][0]*t105-P[12][4]*t101-P[12][2]*t102-P[12][3]*t103+P[12][8]*t104);
            Kfusion[13] = -t62*(P[13][1]*t9+P[13][5]*t100+P[13][0]*t105-P[13][4]*t101-P[13][2]*t102-P[13][3]*t103+P[13][8]*t104);
            Kfusion[14] = -t62*(P[14][1]*t9+P[14][5]*t100+P[14][0]*t105-P[14][4]*t101-P[14][2]*t102-P[14][3]*t103+P[14][8]*t104);
            Kfusion[15] = -t62*(P[15][1]*t9+P[15][5]*t100+P[15][0]*t105-P[15][4]*t101-P[15][2]*t102-P[15][3]*t103+P[15][8]*t104);
            if (!inhibitWindStates) {
                Kfusion[22] = -t62*(P[22][1]*t9+P[22][5]*t100+P[22][0]*t105-P[22][4]*t101-P[22][2]*t102-P[22][3]*t103+P[22][8]*t104);
                Kfusion[23] = -t62*(P[23][1]*t9+P[23][5]*t100+P[23][0]*t105-P[23][4]*t101-P[23][2]*t102-P[23][3]*t103+P[23][8]*t104);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = -t62*(P[16][1]*t9+P[16][5]*t100+P[16][0]*t105-P[16][4]*t101-P[16][2]*t102-P[16][3]*t103+P[16][8]*t104);
                Kfusion[17] = -t62*(P[17][1]*t9+P[17][5]*t100+P[17][0]*t105-P[17][4]*t101-P[17][2]*t102-P[17][3]*t103+P[17][8]*t104);
                Kfusion[18] = -t62*(P[18][1]*t9+P[18][5]*t100+P[18][0]*t105-P[18][4]*t101-P[18][2]*t102-P[18][3]*t103+P[18][8]*t104);
                Kfusion[19] = -t62*(P[19][1]*t9+P[19][5]*t100+P[19][0]*t105-P[19][4]*t101-P[19][2]*t102-P[19][3]*t103+P[19][8]*t104);
                Kfusion[20] = -t62*(P[20][1]*t9+P[20][5]*t100+P[20][0]*t105-P[20][4]*t101-P[20][2]*t102-P[20][3]*t103+P[20][8]*t104);
                Kfusion[21] = -t62*(P[21][1]*t9+P[21][5]*t100+P[21][0]*t105-P[21][4]*t101-P[21][2]*t102-P[21][3]*t103+P[21][8]*t104);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }
        }

        // calculate the innovation consistency test ratio
        flowTestRatio[obsIndex] = sq(innovOptFlow[obsIndex]) / (sq(MAX(0.01f * (float)frontend->_flowInnovGate, 1.0f)) * varInnovOptFlow[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
        if ((flowTestRatio[obsIndex]) < 1.0f && (ofDataDelayed.flowRadXY.x < frontend->_maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend->_maxFlowRate)) {
            // record the last time observations were accepted for fusion
            prevFlowFuseTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=5; j++) {
                    KH[i][j] = Kfusion[i] * H_LOS[j];
                }
                for (unsigned j = 6; j<=7; j++) {
                    KH[i][j] = 0.0f;
                }
                KH[i][8] = Kfusion[i] * H_LOS[8];
                for (unsigned j = 9; j<=23; j++) {
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
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][8] * P[8][j];
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

                // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
                stateStruct.angErr.zero();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovOptFlow[obsIndex];
                }

                // the first 3 states represent the angular misalignment vector. This is
                // is used to correct the estimated quaternion on the current time step
                stateStruct.quat.rotate(stateStruct.angErr);

            } else {
                // record bad axis
                if (obsIndex == 0) {
                    faultStatus.bad_xflow = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yflow = true;
                }

            }
        }
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

