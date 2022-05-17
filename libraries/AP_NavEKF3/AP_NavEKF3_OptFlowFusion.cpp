#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <GCS_MAVLink/GCS.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of optical flow measurements
void NavEKF3_core::SelectFlowFusion()
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

    of_elements ofDataDelayed;      // OF data at the fusion time horizon

    // Check for data at the fusion time horizon
    const bool flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);

    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // check is the terrain offset estimate is still valid - if we are using range finder as the main height reference, the ground is assumed to be at 0
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER);
    // Perform tilt check
    bool tiltOK = (prevTnb.c.z > frontend->DCM33FlowMin);
    // Constrain measurements to zero if takeoff is not detected and the height above ground
    // is insufficient to achieve acceptable focus. This allows the vehicle to be picked up
    // and carried to test optical flow operation
    if (!takeOffDetected && ((terrainState - stateStruct.position.z) < 0.5f)) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // if have valid flow or range measurements, fuse data into a 1-state EKF to estimate terrain height
    if (((flowDataToFuse && (frontend->_flowUse == FLOW_USE_TERRAIN)) || rangeDataToFuse) && tiltOK) {
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset(ofDataDelayed);
    }

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && tiltOK) {
        const bool fuse_optflow = (frontend->_flowUse == FLOW_USE_NAV) && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::OPTFLOW);
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        FuseOptFlow(ofDataDelayed, fuse_optflow);
    }
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optical flow rates and range finder measurements
Equations generated using https://github.com/PX4/ecl/tree/master/EKF/matlab/scripts/Terrain%20Estimator
*/
void NavEKF3_core::EstimateTerrainOffset(const of_elements &ofDataDelayed)
{
    // horizontal velocity squared
    ftype velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);

    // don't fuse flow data if LOS rate is misaligned, without GPS, or insufficient velocity, as it is poorly observable
    // don't fuse flow data if it exceeds validity limits
    // don't update terrain offset if ground is being used as the zero height datum in the main filter
    bool cantFuseFlowData = ((frontend->_flowUse != FLOW_USE_TERRAIN)
    || !gpsIsInUse
    || PV_AidingMode == AID_RELATIVE 
    || velHorizSq < 25.0f 
    || (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) > frontend->_maxFlowRate));

    if ((!rangeDataToFuse && cantFuseFlowData) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
        // skip update
        inhibitGndState = true;
    } else {
        inhibitGndState = false;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation after bad gps causing bad numerical conditioning
        ftype distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have the growth in uncertainty due to the copter's vertical velocity
        ftype timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        ftype Pincrement = (distanceTravelledSq * sq(frontend->_terrGradMax)) + sq(timeLapsed)*P[6][6];
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (rangeDataToFuse) {
            // reset terrain state if rangefinder data not fused for 5 seconds
            if (imuSampleTime_ms - gndHgtValidTime_ms > 5000) {
                terrainState = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd) + stateStruct.position.z;
            }

            // predict range
            ftype predRngMeas = MAX((terrainState - stateStruct.position[2]),rngOnGnd) / prevTnb.c.z;
            // Copy required states to local variable names
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            ftype R_RNG = frontend->_rngNoise;

            // calculate Kalman gain
            ftype SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            ftype K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rangeDataDelayed.rng;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f)) * varInnovRng);

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

                // record the time we last updated the terrain offset state
                gndHgtValidTime_ms = imuSampleTime_ms;
            }
        }

        if (!cantFuseFlowData) {

            Vector3F relVelSensor;          // velocity of sensor relative to ground in sensor axes
            Vector2F losPred;               // predicted optical flow angular rate measurement
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            ftype K_OPT;
            ftype H_OPT;
            Vector2F auxFlowObsInnovVar;

            // predict range to centre of image
            ftype flowRngPred = MAX((terrainState - stateStruct.position.z),rngOnGnd) / prevTnb.c.z;

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
            ftype t2 = q0*q0;
            ftype t3 = q1*q1;
            ftype t4 = q2*q2;
            ftype t5 = q3*q3;
            ftype t6 = stateStruct.position.z - terrainState;
            ftype t7 = 1.0f / (t6*t6);
            ftype t8 = q0*q3*2.0f;
            ftype t9 = t2-t3-t4+t5;

            // prevent the state variances from becoming badly conditioned
            Popt = MAX(Popt,1E-6f);

            // calculate observation noise variance from parameter
            ftype flow_noise_variance = sq(MAX(frontend->_flowNoise, 0.05f));

            // Fuse Y axis data

            // Calculate observation partial derivative
            H_OPT = t7*t9*(-stateStruct.velocity.z*(q0*q2*2.0-q1*q3*2.0)+stateStruct.velocity.x*(t2+t3-t4-t5)+stateStruct.velocity.y*(t8+q1*q2*2.0));

            // calculate innovation variance
            auxFlowObsInnovVar.y = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.y;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.y = sq(auxFlowObsInnov.y) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.y);

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

                // record the time we last updated the terrain offset state
                gndHgtValidTime_ms = imuSampleTime_ms;
            }

            // fuse X axis data
            H_OPT = -t7*t9*(stateStruct.velocity.z*(q0*q1*2.0+q2*q3*2.0)+stateStruct.velocity.y*(t2-t3+t4-t5)-stateStruct.velocity.x*(t8-q1*q2*2.0));

            // calculate innovation variances
            auxFlowObsInnovVar.x = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.x;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.x = sq(auxFlowObsInnov.x) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.x);

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
}

/*
 * Fuse angular motion compensated optical flow rates using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * Requires a valid terrain height estimate.
 *
 * really_fuse should be true to actually fuse into the main filter, false to only calculate variances
*/
void NavEKF3_core::FuseOptFlow(const of_elements &ofDataDelayed, bool really_fuse)
{
    Vector24 H_LOS;
    Vector2 losPred;

    // Copy required states to local variable names
    ftype q0  = stateStruct.quat[0];
    ftype q1 = stateStruct.quat[1];
    ftype q2 = stateStruct.quat[2];
    ftype q3 = stateStruct.quat[3];
    ftype vn = stateStruct.velocity.x;
    ftype ve = stateStruct.velocity.y;
    ftype vd = stateStruct.velocity.z;
    ftype pd = stateStruct.position.z;

    // constrain height above ground to be above range measured on ground
    ftype heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);

    // calculate range from ground plain to centre of sensor fov assuming flat earth
    ftype range = constrain_ftype((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);

    // correct range for flow sensor offset body frame position offset
    // the corrected value is the predicted range from the sensor focal point to the
    // centre of the image on the ground assuming flat terrain
    Vector3F posOffsetBody = ofDataDelayed.body_offset - accelPosOffset;
    if (!posOffsetBody.is_zero()) {
        Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
        range -= posOffsetEarth.z / prevTnb.c.z;
    }

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        const Vector3F relVelSensor = (prevTnb * stateStruct.velocity) + (ofDataDelayed.bodyRadXYZ % posOffsetBody);

        // divide velocity by range to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;

        // calculate observation jacobians and Kalman gains
        memset(&H_LOS[0], 0, sizeof(H_LOS));
        if (obsIndex == 0) {
            // calculate X axis observation Jacobian
            ftype t2 = 1.0f / range;
            H_LOS[0] = t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
            H_LOS[1] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
            H_LOS[2] = t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
            H_LOS[3] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
            H_LOS[4] = -t2*(q0*q3*2.0f-q1*q2*2.0f);
            H_LOS[5] = t2*(q0*q0-q1*q1+q2*q2-q3*q3);
            H_LOS[6] = t2*(q0*q1*2.0f+q2*q3*2.0f);

            // calculate intermediate variables for the X observation innovation variance and Kalman gains
            ftype t3 = q1*vd*2.0f;
            ftype t4 = q0*ve*2.0f;
            ftype t11 = q3*vn*2.0f;
            ftype t5 = t3+t4-t11;
            ftype t6 = q0*q3*2.0f;
            ftype t29 = q1*q2*2.0f;
            ftype t7 = t6-t29;
            ftype t8 = q0*q1*2.0f;
            ftype t9 = q2*q3*2.0f;
            ftype t10 = t8+t9;
            ftype t12 = P[0][0]*t2*t5;
            ftype t13 = q0*vd*2.0f;
            ftype t14 = q2*vn*2.0f;
            ftype t28 = q1*ve*2.0f;
            ftype t15 = t13+t14-t28;
            ftype t16 = q3*vd*2.0f;
            ftype t17 = q2*ve*2.0f;
            ftype t18 = q1*vn*2.0f;
            ftype t19 = t16+t17+t18;
            ftype t20 = q3*ve*2.0f;
            ftype t21 = q0*vn*2.0f;
            ftype t30 = q2*vd*2.0f;
            ftype t22 = t20+t21-t30;
            ftype t23 = q0*q0;
            ftype t24 = q1*q1;
            ftype t25 = q2*q2;
            ftype t26 = q3*q3;
            ftype t27 = t23-t24+t25-t26;
            ftype t31 = P[1][1]*t2*t15;
            ftype t32 = P[6][0]*t2*t10;
            ftype t33 = P[1][0]*t2*t15;
            ftype t34 = P[2][0]*t2*t19;
            ftype t35 = P[5][0]*t2*t27;
            ftype t79 = P[4][0]*t2*t7;
            ftype t80 = P[3][0]*t2*t22;
            ftype t36 = t12+t32+t33+t34+t35-t79-t80;
            ftype t37 = t2*t5*t36;
            ftype t38 = P[6][1]*t2*t10;
            ftype t39 = P[0][1]*t2*t5;
            ftype t40 = P[2][1]*t2*t19;
            ftype t41 = P[5][1]*t2*t27;
            ftype t81 = P[4][1]*t2*t7;
            ftype t82 = P[3][1]*t2*t22;
            ftype t42 = t31+t38+t39+t40+t41-t81-t82;
            ftype t43 = t2*t15*t42;
            ftype t44 = P[6][2]*t2*t10;
            ftype t45 = P[0][2]*t2*t5;
            ftype t46 = P[1][2]*t2*t15;
            ftype t47 = P[2][2]*t2*t19;
            ftype t48 = P[5][2]*t2*t27;
            ftype t83 = P[4][2]*t2*t7;
            ftype t84 = P[3][2]*t2*t22;
            ftype t49 = t44+t45+t46+t47+t48-t83-t84;
            ftype t50 = t2*t19*t49;
            ftype t51 = P[6][3]*t2*t10;
            ftype t52 = P[0][3]*t2*t5;
            ftype t53 = P[1][3]*t2*t15;
            ftype t54 = P[2][3]*t2*t19;
            ftype t55 = P[5][3]*t2*t27;
            ftype t85 = P[4][3]*t2*t7;
            ftype t86 = P[3][3]*t2*t22;
            ftype t56 = t51+t52+t53+t54+t55-t85-t86;
            ftype t57 = P[6][5]*t2*t10;
            ftype t58 = P[0][5]*t2*t5;
            ftype t59 = P[1][5]*t2*t15;
            ftype t60 = P[2][5]*t2*t19;
            ftype t61 = P[5][5]*t2*t27;
            ftype t88 = P[4][5]*t2*t7;
            ftype t89 = P[3][5]*t2*t22;
            ftype t62 = t57+t58+t59+t60+t61-t88-t89;
            ftype t63 = t2*t27*t62;
            ftype t64 = P[6][4]*t2*t10;
            ftype t65 = P[0][4]*t2*t5;
            ftype t66 = P[1][4]*t2*t15;
            ftype t67 = P[2][4]*t2*t19;
            ftype t68 = P[5][4]*t2*t27;
            ftype t90 = P[4][4]*t2*t7;
            ftype t91 = P[3][4]*t2*t22;
            ftype t69 = t64+t65+t66+t67+t68-t90-t91;
            ftype t70 = P[6][6]*t2*t10;
            ftype t71 = P[0][6]*t2*t5;
            ftype t72 = P[1][6]*t2*t15;
            ftype t73 = P[2][6]*t2*t19;
            ftype t74 = P[5][6]*t2*t27;
            ftype t93 = P[4][6]*t2*t7;
            ftype t94 = P[3][6]*t2*t22;
            ftype t75 = t70+t71+t72+t73+t74-t93-t94;
            ftype t76 = t2*t10*t75;
            ftype t87 = t2*t22*t56;
            ftype t92 = t2*t7*t69;
            ftype t77 = R_LOS+t37+t43+t50+t63+t76-t87-t92;
            ftype t78;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t77 > R_LOS) {
                t78 = 1.0f/t77;
                faultStatus.bad_xflow = false;
            } else {
                t77 = R_LOS;
                t78 = 1.0f/R_LOS;
                faultStatus.bad_xflow = true;
                return;
            }
            flowVarInnov[0] = t77;

            // calculate innovation for X axis observation
            // flowInnovTime_ms will be updated when Y-axis innovations are calculated
            flowInnov[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = t78*(t12-P[0][4]*t2*t7+P[0][1]*t2*t15+P[0][6]*t2*t10+P[0][2]*t2*t19-P[0][3]*t2*t22+P[0][5]*t2*t27);
            Kfusion[1] = t78*(t31+P[1][0]*t2*t5-P[1][4]*t2*t7+P[1][6]*t2*t10+P[1][2]*t2*t19-P[1][3]*t2*t22+P[1][5]*t2*t27);
            Kfusion[2] = t78*(t47+P[2][0]*t2*t5-P[2][4]*t2*t7+P[2][1]*t2*t15+P[2][6]*t2*t10-P[2][3]*t2*t22+P[2][5]*t2*t27);
            Kfusion[3] = t78*(-t86+P[3][0]*t2*t5-P[3][4]*t2*t7+P[3][1]*t2*t15+P[3][6]*t2*t10+P[3][2]*t2*t19+P[3][5]*t2*t27);
            Kfusion[4] = t78*(-t90+P[4][0]*t2*t5+P[4][1]*t2*t15+P[4][6]*t2*t10+P[4][2]*t2*t19-P[4][3]*t2*t22+P[4][5]*t2*t27);
            Kfusion[5] = t78*(t61+P[5][0]*t2*t5-P[5][4]*t2*t7+P[5][1]*t2*t15+P[5][6]*t2*t10+P[5][2]*t2*t19-P[5][3]*t2*t22);
            Kfusion[6] = t78*(t70+P[6][0]*t2*t5-P[6][4]*t2*t7+P[6][1]*t2*t15+P[6][2]*t2*t19-P[6][3]*t2*t22+P[6][5]*t2*t27);
            Kfusion[7] = t78*(P[7][0]*t2*t5-P[7][4]*t2*t7+P[7][1]*t2*t15+P[7][6]*t2*t10+P[7][2]*t2*t19-P[7][3]*t2*t22+P[7][5]*t2*t27);
            Kfusion[8] = t78*(P[8][0]*t2*t5-P[8][4]*t2*t7+P[8][1]*t2*t15+P[8][6]*t2*t10+P[8][2]*t2*t19-P[8][3]*t2*t22+P[8][5]*t2*t27);
            Kfusion[9] = t78*(P[9][0]*t2*t5-P[9][4]*t2*t7+P[9][1]*t2*t15+P[9][6]*t2*t10+P[9][2]*t2*t19-P[9][3]*t2*t22+P[9][5]*t2*t27);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = t78*(P[10][0]*t2*t5-P[10][4]*t2*t7+P[10][1]*t2*t15+P[10][6]*t2*t10+P[10][2]*t2*t19-P[10][3]*t2*t22+P[10][5]*t2*t27);
                Kfusion[11] = t78*(P[11][0]*t2*t5-P[11][4]*t2*t7+P[11][1]*t2*t15+P[11][6]*t2*t10+P[11][2]*t2*t19-P[11][3]*t2*t22+P[11][5]*t2*t27);
                Kfusion[12] = t78*(P[12][0]*t2*t5-P[12][4]*t2*t7+P[12][1]*t2*t15+P[12][6]*t2*t10+P[12][2]*t2*t19-P[12][3]*t2*t22+P[12][5]*t2*t27);
            } else {
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);
            }

            if (!inhibitDelVelBiasStates && !badIMUdata) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = t78*(P[stateIndex][0]*t2*t5-P[stateIndex][4]*t2*t7+P[stateIndex][1]*t2*t15+P[stateIndex][6]*t2*t10+P[stateIndex][2]*t2*t19-P[stateIndex][3]*t2*t22+P[stateIndex][5]*t2*t27);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = t78*(P[16][0]*t2*t5-P[16][4]*t2*t7+P[16][1]*t2*t15+P[16][6]*t2*t10+P[16][2]*t2*t19-P[16][3]*t2*t22+P[16][5]*t2*t27);
                Kfusion[17] = t78*(P[17][0]*t2*t5-P[17][4]*t2*t7+P[17][1]*t2*t15+P[17][6]*t2*t10+P[17][2]*t2*t19-P[17][3]*t2*t22+P[17][5]*t2*t27);
                Kfusion[18] = t78*(P[18][0]*t2*t5-P[18][4]*t2*t7+P[18][1]*t2*t15+P[18][6]*t2*t10+P[18][2]*t2*t19-P[18][3]*t2*t22+P[18][5]*t2*t27);
                Kfusion[19] = t78*(P[19][0]*t2*t5-P[19][4]*t2*t7+P[19][1]*t2*t15+P[19][6]*t2*t10+P[19][2]*t2*t19-P[19][3]*t2*t22+P[19][5]*t2*t27);
                Kfusion[20] = t78*(P[20][0]*t2*t5-P[20][4]*t2*t7+P[20][1]*t2*t15+P[20][6]*t2*t10+P[20][2]*t2*t19-P[20][3]*t2*t22+P[20][5]*t2*t27);
                Kfusion[21] = t78*(P[21][0]*t2*t5-P[21][4]*t2*t7+P[21][1]*t2*t15+P[21][6]*t2*t10+P[21][2]*t2*t19-P[21][3]*t2*t22+P[21][5]*t2*t27);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = t78*(P[22][0]*t2*t5-P[22][4]*t2*t7+P[22][1]*t2*t15+P[22][6]*t2*t10+P[22][2]*t2*t19-P[22][3]*t2*t22+P[22][5]*t2*t27);
                Kfusion[23] = t78*(P[23][0]*t2*t5-P[23][4]*t2*t7+P[23][1]*t2*t15+P[23][6]*t2*t10+P[23][2]*t2*t19-P[23][3]*t2*t22+P[23][5]*t2*t27);
            } else {
                // zero indexes 22 to 23
                zero_range(&Kfusion[0], 22, 23);
            }

        } else {

            // calculate Y axis observation Jacobian
            ftype t2 = 1.0f / range;
            H_LOS[0] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
            H_LOS[1] = -t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
            H_LOS[2] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
            H_LOS[3] = -t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
            H_LOS[4] = -t2*(q0*q0+q1*q1-q2*q2-q3*q3);
            H_LOS[5] = -t2*(q0*q3*2.0f+q1*q2*2.0f);
            H_LOS[6] = t2*(q0*q2*2.0f-q1*q3*2.0f);

            // calculate intermediate variables for the Y observation innovation variance and Kalman gains
            ftype t3 = q3*ve*2.0f;
            ftype t4 = q0*vn*2.0f;
            ftype t11 = q2*vd*2.0f;
            ftype t5 = t3+t4-t11;
            ftype t6 = q0*q3*2.0f;
            ftype t7 = q1*q2*2.0f;
            ftype t8 = t6+t7;
            ftype t9 = q0*q2*2.0f;
            ftype t28 = q1*q3*2.0f;
            ftype t10 = t9-t28;
            ftype t12 = P[0][0]*t2*t5;
            ftype t13 = q3*vd*2.0f;
            ftype t14 = q2*ve*2.0f;
            ftype t15 = q1*vn*2.0f;
            ftype t16 = t13+t14+t15;
            ftype t17 = q0*vd*2.0f;
            ftype t18 = q2*vn*2.0f;
            ftype t29 = q1*ve*2.0f;
            ftype t19 = t17+t18-t29;
            ftype t20 = q1*vd*2.0f;
            ftype t21 = q0*ve*2.0f;
            ftype t30 = q3*vn*2.0f;
            ftype t22 = t20+t21-t30;
            ftype t23 = q0*q0;
            ftype t24 = q1*q1;
            ftype t25 = q2*q2;
            ftype t26 = q3*q3;
            ftype t27 = t23+t24-t25-t26;
            ftype t31 = P[1][1]*t2*t16;
            ftype t32 = P[5][0]*t2*t8;
            ftype t33 = P[1][0]*t2*t16;
            ftype t34 = P[3][0]*t2*t22;
            ftype t35 = P[4][0]*t2*t27;
            ftype t80 = P[6][0]*t2*t10;
            ftype t81 = P[2][0]*t2*t19;
            ftype t36 = t12+t32+t33+t34+t35-t80-t81;
            ftype t37 = t2*t5*t36;
            ftype t38 = P[5][1]*t2*t8;
            ftype t39 = P[0][1]*t2*t5;
            ftype t40 = P[3][1]*t2*t22;
            ftype t41 = P[4][1]*t2*t27;
            ftype t82 = P[6][1]*t2*t10;
            ftype t83 = P[2][1]*t2*t19;
            ftype t42 = t31+t38+t39+t40+t41-t82-t83;
            ftype t43 = t2*t16*t42;
            ftype t44 = P[5][2]*t2*t8;
            ftype t45 = P[0][2]*t2*t5;
            ftype t46 = P[1][2]*t2*t16;
            ftype t47 = P[3][2]*t2*t22;
            ftype t48 = P[4][2]*t2*t27;
            ftype t79 = P[2][2]*t2*t19;
            ftype t84 = P[6][2]*t2*t10;
            ftype t49 = t44+t45+t46+t47+t48-t79-t84;
            ftype t50 = P[5][3]*t2*t8;
            ftype t51 = P[0][3]*t2*t5;
            ftype t52 = P[1][3]*t2*t16;
            ftype t53 = P[3][3]*t2*t22;
            ftype t54 = P[4][3]*t2*t27;
            ftype t86 = P[6][3]*t2*t10;
            ftype t87 = P[2][3]*t2*t19;
            ftype t55 = t50+t51+t52+t53+t54-t86-t87;
            ftype t56 = t2*t22*t55;
            ftype t57 = P[5][4]*t2*t8;
            ftype t58 = P[0][4]*t2*t5;
            ftype t59 = P[1][4]*t2*t16;
            ftype t60 = P[3][4]*t2*t22;
            ftype t61 = P[4][4]*t2*t27;
            ftype t88 = P[6][4]*t2*t10;
            ftype t89 = P[2][4]*t2*t19;
            ftype t62 = t57+t58+t59+t60+t61-t88-t89;
            ftype t63 = t2*t27*t62;
            ftype t64 = P[5][5]*t2*t8;
            ftype t65 = P[0][5]*t2*t5;
            ftype t66 = P[1][5]*t2*t16;
            ftype t67 = P[3][5]*t2*t22;
            ftype t68 = P[4][5]*t2*t27;
            ftype t90 = P[6][5]*t2*t10;
            ftype t91 = P[2][5]*t2*t19;
            ftype t69 = t64+t65+t66+t67+t68-t90-t91;
            ftype t70 = t2*t8*t69;
            ftype t71 = P[5][6]*t2*t8;
            ftype t72 = P[0][6]*t2*t5;
            ftype t73 = P[1][6]*t2*t16;
            ftype t74 = P[3][6]*t2*t22;
            ftype t75 = P[4][6]*t2*t27;
            ftype t92 = P[6][6]*t2*t10;
            ftype t93 = P[2][6]*t2*t19;
            ftype t76 = t71+t72+t73+t74+t75-t92-t93;
            ftype t85 = t2*t19*t49;
            ftype t94 = t2*t10*t76;
            ftype t77 = R_LOS+t37+t43+t56+t63+t70-t85-t94;
            ftype t78;

            // calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
            if (t77 > R_LOS) {
                t78 = 1.0f/t77;
                faultStatus.bad_yflow = false;
            } else {
                t77 = R_LOS;
                t78 = 1.0f/R_LOS;
                faultStatus.bad_yflow = true;
                return;
            }
            flowVarInnov[1] = t77;

            // calculate innovation for Y observation
            flowInnov[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;
            flowInnovTime_ms = dal.millis();

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t78*(t12+P[0][5]*t2*t8-P[0][6]*t2*t10+P[0][1]*t2*t16-P[0][2]*t2*t19+P[0][3]*t2*t22+P[0][4]*t2*t27);
            Kfusion[1] = -t78*(t31+P[1][0]*t2*t5+P[1][5]*t2*t8-P[1][6]*t2*t10-P[1][2]*t2*t19+P[1][3]*t2*t22+P[1][4]*t2*t27);
            Kfusion[2] = -t78*(-t79+P[2][0]*t2*t5+P[2][5]*t2*t8-P[2][6]*t2*t10+P[2][1]*t2*t16+P[2][3]*t2*t22+P[2][4]*t2*t27);
            Kfusion[3] = -t78*(t53+P[3][0]*t2*t5+P[3][5]*t2*t8-P[3][6]*t2*t10+P[3][1]*t2*t16-P[3][2]*t2*t19+P[3][4]*t2*t27);
            Kfusion[4] = -t78*(t61+P[4][0]*t2*t5+P[4][5]*t2*t8-P[4][6]*t2*t10+P[4][1]*t2*t16-P[4][2]*t2*t19+P[4][3]*t2*t22);
            Kfusion[5] = -t78*(t64+P[5][0]*t2*t5-P[5][6]*t2*t10+P[5][1]*t2*t16-P[5][2]*t2*t19+P[5][3]*t2*t22+P[5][4]*t2*t27);
            Kfusion[6] = -t78*(-t92+P[6][0]*t2*t5+P[6][5]*t2*t8+P[6][1]*t2*t16-P[6][2]*t2*t19+P[6][3]*t2*t22+P[6][4]*t2*t27);
            Kfusion[7] = -t78*(P[7][0]*t2*t5+P[7][5]*t2*t8-P[7][6]*t2*t10+P[7][1]*t2*t16-P[7][2]*t2*t19+P[7][3]*t2*t22+P[7][4]*t2*t27);
            Kfusion[8] = -t78*(P[8][0]*t2*t5+P[8][5]*t2*t8-P[8][6]*t2*t10+P[8][1]*t2*t16-P[8][2]*t2*t19+P[8][3]*t2*t22+P[8][4]*t2*t27);
            Kfusion[9] = -t78*(P[9][0]*t2*t5+P[9][5]*t2*t8-P[9][6]*t2*t10+P[9][1]*t2*t16-P[9][2]*t2*t19+P[9][3]*t2*t22+P[9][4]*t2*t27);

            if (!inhibitDelAngBiasStates) {
                Kfusion[10] = -t78*(P[10][0]*t2*t5+P[10][5]*t2*t8-P[10][6]*t2*t10+P[10][1]*t2*t16-P[10][2]*t2*t19+P[10][3]*t2*t22+P[10][4]*t2*t27);
                Kfusion[11] = -t78*(P[11][0]*t2*t5+P[11][5]*t2*t8-P[11][6]*t2*t10+P[11][1]*t2*t16-P[11][2]*t2*t19+P[11][3]*t2*t22+P[11][4]*t2*t27);
                Kfusion[12] = -t78*(P[12][0]*t2*t5+P[12][5]*t2*t8-P[12][6]*t2*t10+P[12][1]*t2*t16-P[12][2]*t2*t19+P[12][3]*t2*t22+P[12][4]*t2*t27);
            } else {
                // zero indexes 10 to 12
                zero_range(&Kfusion[0], 10, 12);
            }

            if (!inhibitDelVelBiasStates && !badIMUdata) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!dvelBiasAxisInhibit[index]) {
                        Kfusion[stateIndex] = -t78*(P[stateIndex][0]*t2*t5+P[stateIndex][5]*t2*t8-P[stateIndex][6]*t2*t10+P[stateIndex][1]*t2*t16-P[stateIndex][2]*t2*t19+P[stateIndex][3]*t2*t22+P[stateIndex][4]*t2*t27);
                    } else {
                        Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                zero_range(&Kfusion[0], 13, 15);
            }

            if (!inhibitMagStates) {
                Kfusion[16] = -t78*(P[16][0]*t2*t5+P[16][5]*t2*t8-P[16][6]*t2*t10+P[16][1]*t2*t16-P[16][2]*t2*t19+P[16][3]*t2*t22+P[16][4]*t2*t27);
                Kfusion[17] = -t78*(P[17][0]*t2*t5+P[17][5]*t2*t8-P[17][6]*t2*t10+P[17][1]*t2*t16-P[17][2]*t2*t19+P[17][3]*t2*t22+P[17][4]*t2*t27);
                Kfusion[18] = -t78*(P[18][0]*t2*t5+P[18][5]*t2*t8-P[18][6]*t2*t10+P[18][1]*t2*t16-P[18][2]*t2*t19+P[18][3]*t2*t22+P[18][4]*t2*t27);
                Kfusion[19] = -t78*(P[19][0]*t2*t5+P[19][5]*t2*t8-P[19][6]*t2*t10+P[19][1]*t2*t16-P[19][2]*t2*t19+P[19][3]*t2*t22+P[19][4]*t2*t27);
                Kfusion[20] = -t78*(P[20][0]*t2*t5+P[20][5]*t2*t8-P[20][6]*t2*t10+P[20][1]*t2*t16-P[20][2]*t2*t19+P[20][3]*t2*t22+P[20][4]*t2*t27);
                Kfusion[21] = -t78*(P[21][0]*t2*t5+P[21][5]*t2*t8-P[21][6]*t2*t10+P[21][1]*t2*t16-P[21][2]*t2*t19+P[21][3]*t2*t22+P[21][4]*t2*t27);
            } else {
                // zero indexes 16 to 21
                zero_range(&Kfusion[0], 16, 21);
            }

            if (!inhibitWindStates) {
                Kfusion[22] = -t78*(P[22][0]*t2*t5+P[22][5]*t2*t8-P[22][6]*t2*t10+P[22][1]*t2*t16-P[22][2]*t2*t19+P[22][3]*t2*t22+P[22][4]*t2*t27);
                Kfusion[23] = -t78*(P[23][0]*t2*t5+P[23][5]*t2*t8-P[23][6]*t2*t10+P[23][1]*t2*t16-P[23][2]*t2*t19+P[23][3]*t2*t22+P[23][4]*t2*t27);
            } else {
                // zero indexes 22 to 23
                zero_range(&Kfusion[0], 22, 23);
            }
        }

        // calculate the innovation consistency test ratio
        flowTestRatio[obsIndex] = sq(flowInnov[obsIndex]) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * flowVarInnov[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
        if (really_fuse && (flowTestRatio[obsIndex]) < 1.0f && (ofDataDelayed.flowRadXY.x < frontend->_maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend->_maxFlowRate)) {
            // record the last time observations were accepted for fusion
            prevFlowFuseTime_ms = imuSampleTime_ms;
            // notify first time only
            if (!flowFusionActive) {
                flowFusionActive = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u fusing optical flow",(unsigned)imu_index);
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=stateIndexLim; i++) {
                for (uint8_t j = 0; j<=6; j++) {
                    KH[i][j] = Kfusion[i] * H_LOS[j];
                }
                for (uint8_t j = 7; j<=stateIndexLim; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (uint8_t j = 0; j<=stateIndexLim; j++) {
                for (uint8_t i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][0] * P[0][j];
                    res += KH[i][1] * P[1][j];
                    res += KH[i][2] * P[2][j];
                    res += KH[i][3] * P[3][j];
                    res += KH[i][4] * P[4][j];
                    res += KH[i][5] * P[5][j];
                    res += KH[i][6] * P[6][j];
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
                    statesArray[j] = statesArray[j] - Kfusion[j] * flowInnov[obsIndex];
                }
                stateStruct.quat.normalize();

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

    // store optical flow rates for use in external calibration
    flowCalSample.timestamp_ms = imuSampleTime_ms;
    flowCalSample.flowRate.x = ofDataDelayed.flowRadXY.x;
    flowCalSample.flowRate.y = ofDataDelayed.flowRadXY.y;
    flowCalSample.bodyRate.x = ofDataDelayed.bodyRadXYZ.x;
    flowCalSample.bodyRate.y = ofDataDelayed.bodyRadXYZ.y;
    flowCalSample.losPred.x = losPred[0];
    flowCalSample.losPred.y = losPred[1];
}

// retrieve latest corrected optical flow samples (used for calibration)
bool NavEKF3_core::getOptFlowSample(uint32_t& timestamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const
{
    if (flowCalSample.timestamp_ms != 0) {
        timestamp_ms = flowCalSample.timestamp_ms;
        flowRate = flowCalSample.flowRate;
        bodyRate = flowCalSample.bodyRate;
        losPred = flowCalSample.losPred;
        return true;
    }
    return false;
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

