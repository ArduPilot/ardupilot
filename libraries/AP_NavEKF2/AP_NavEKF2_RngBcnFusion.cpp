#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of range beacon measurements
void NavEKF2_core::SelectRngBcnFusion()
{
    // read range data from the sensor and check for new data in the buffer
    readRngBcnData();

    // Determine if we need to fuse range beacon data on this time step
    if (rngBcnDataToFuse) {
        if (PV_AidingMode == AID_ABSOLUTE) {
            // Normal operating mode is to fuse the range data into the main filter
            FuseRngBcn();
        } else {
            // If we aren't able to use the data in the main filter, use a simple 3-state filter to estimte position only
            FuseRngBcnStatic();
        }
    }
}

void NavEKF2_core::FuseRngBcn()
{
    // declarations
    ftype pn;
    ftype pe;
    ftype pd;
    ftype bcn_pn;
    ftype bcn_pe;
    ftype bcn_pd;
    const ftype R_BCN = sq(MAX(rngBcnDataDelayed.rngErr , 0.1f));
    ftype rngPred;

    // health is set bad until test passed
    rngBcnHealth = false;

    if (activeHgtSource != HGT_SOURCE_BCN) {
        // calculate the vertical offset from EKF datum to beacon datum
        CalcRangeBeaconPosDownOffset(R_BCN, stateStruct.position, false);
    } else {
        bcnPosOffset = 0.0f;
    }

    // copy required states to local variable names
    pn = stateStruct.position.x;
    pe = stateStruct.position.y;
    pd = stateStruct.position.z;
    bcn_pn = rngBcnDataDelayed.beacon_posNED.x;
    bcn_pe = rngBcnDataDelayed.beacon_posNED.y;
    bcn_pd = rngBcnDataDelayed.beacon_posNED.z + bcnPosOffset;

    // predicted range
    Vector3F deltaPosNED = stateStruct.position - rngBcnDataDelayed.beacon_posNED;
    rngPred = deltaPosNED.length();

    // calculate measurement innovation
    innovRngBcn = rngPred - rngBcnDataDelayed.rng;

    // perform fusion of range measurement
    if (rngPred > 0.1f)
    {
        // calculate observation jacobians
        ftype H_BCN[24] = {};
        ftype t2 = bcn_pd-pd;
        ftype t3 = bcn_pe-pe;
        ftype t4 = bcn_pn-pn;
        ftype t5 = t2*t2;
        ftype t6 = t3*t3;
        ftype t7 = t4*t4;
        ftype t8 = t5+t6+t7;
        ftype t9 = 1.0f/sqrtF(t8);
        H_BCN[6] = -t4*t9;
        H_BCN[7] = -t3*t9;
        H_BCN[8] = -t2*t9;

        // calculate Kalman gains
        ftype t10 = P[8][8]*t2*t9;
        ftype t11 = P[7][8]*t3*t9;
        ftype t12 = P[6][8]*t4*t9;
        ftype t13 = t10+t11+t12;
        ftype t14 = t2*t9*t13;
        ftype t15 = P[8][7]*t2*t9;
        ftype t16 = P[7][7]*t3*t9;
        ftype t17 = P[6][7]*t4*t9;
        ftype t18 = t15+t16+t17;
        ftype t19 = t3*t9*t18;
        ftype t20 = P[8][6]*t2*t9;
        ftype t21 = P[7][6]*t3*t9;
        ftype t22 = P[6][6]*t4*t9;
        ftype t23 = t20+t21+t22;
        ftype t24 = t4*t9*t23;
        varInnovRngBcn = R_BCN+t14+t19+t24;
        ftype t26;
        if (varInnovRngBcn >= R_BCN) {
            t26 = 1.0/varInnovRngBcn;
            faultStatus.bad_rngbcn = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_rngbcn = true;
            return;
        }

        Kfusion[0] = -t26*(P[0][6]*t4*t9+P[0][7]*t3*t9+P[0][8]*t2*t9);
        Kfusion[1] = -t26*(P[1][6]*t4*t9+P[1][7]*t3*t9+P[1][8]*t2*t9);
        Kfusion[2] = -t26*(P[2][6]*t4*t9+P[2][7]*t3*t9+P[2][8]*t2*t9);
        Kfusion[3] = -t26*(P[3][6]*t4*t9+P[3][7]*t3*t9+P[3][8]*t2*t9);
        Kfusion[4] = -t26*(P[4][6]*t4*t9+P[4][7]*t3*t9+P[4][8]*t2*t9);
        Kfusion[6] = -t26*(t22+P[6][7]*t3*t9+P[6][8]*t2*t9);
        Kfusion[7] = -t26*(t16+P[7][6]*t4*t9+P[7][8]*t2*t9);
        if (activeHgtSource == HGT_SOURCE_BCN) {
            // We are using the range beacon as the primary height reference, so allow it to modify the EKF's vertical states
            Kfusion[5] = -t26*(P[5][6]*t4*t9+P[5][7]*t3*t9+P[5][8]*t2*t9);
            Kfusion[8] = -t26*(t10+P[8][6]*t4*t9+P[8][7]*t3*t9);
            Kfusion[15] = -t26*(P[15][6]*t4*t9+P[15][7]*t3*t9+P[15][8]*t2*t9);
            bcnPosOffset = 0.0f;

        } else {
            // don't allow the range measurement to affect the vertical states in the main filter
            Kfusion[5] = 0.0f;
            Kfusion[8] = 0.0f;
            Kfusion[15] = 0.0f;

        }
        Kfusion[9] = -t26*(P[9][6]*t4*t9+P[9][7]*t3*t9+P[9][8]*t2*t9);
        Kfusion[10] = -t26*(P[10][6]*t4*t9+P[10][7]*t3*t9+P[10][8]*t2*t9);
        Kfusion[11] = -t26*(P[11][6]*t4*t9+P[11][7]*t3*t9+P[11][8]*t2*t9);
        Kfusion[12] = -t26*(P[12][6]*t4*t9+P[12][7]*t3*t9+P[12][8]*t2*t9);
        Kfusion[13] = -t26*(P[13][6]*t4*t9+P[13][7]*t3*t9+P[13][8]*t2*t9);
        Kfusion[14] = -t26*(P[14][6]*t4*t9+P[14][7]*t3*t9+P[14][8]*t2*t9);
        if (!inhibitMagStates) {
            Kfusion[16] = -t26*(P[16][6]*t4*t9+P[16][7]*t3*t9+P[16][8]*t2*t9);
            Kfusion[17] = -t26*(P[17][6]*t4*t9+P[17][7]*t3*t9+P[17][8]*t2*t9);
            Kfusion[18] = -t26*(P[18][6]*t4*t9+P[18][7]*t3*t9+P[18][8]*t2*t9);
            Kfusion[19] = -t26*(P[19][6]*t4*t9+P[19][7]*t3*t9+P[19][8]*t2*t9);
            Kfusion[20] = -t26*(P[20][6]*t4*t9+P[20][7]*t3*t9+P[20][8]*t2*t9);
            Kfusion[21] = -t26*(P[21][6]*t4*t9+P[21][7]*t3*t9+P[21][8]*t2*t9);
        } else {
            // zero indexes 16 to 21 = 6
            zero_range(&Kfusion[0], 16, 21);
        }
        Kfusion[22] = -t26*(P[22][6]*t4*t9+P[22][7]*t3*t9+P[22][8]*t2*t9);
        Kfusion[23] = -t26*(P[23][6]*t4*t9+P[23][7]*t3*t9+P[23][8]*t2*t9);

        // Calculate innovation using the selected offset value
        Vector3F delta = stateStruct.position - rngBcnDataDelayed.beacon_posNED;
        innovRngBcn = delta.length() - rngBcnDataDelayed.rng;

        // calculate the innovation consistency test ratio
        rngBcnTestRatio = sq(innovRngBcn) / (sq(MAX(0.01f * (ftype)frontend->_rngBcnInnovGate, 1.0f)) * varInnovRngBcn);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        rngBcnHealth = ((rngBcnTestRatio < 1.0f) || badIMUdata);

        // test the ratio before fusing data
        if (rngBcnHealth) {

            // restart the counter
            lastRngBcnPassTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=5; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 6; j<=8; j++) {
                    KH[i][j] = Kfusion[i] * H_BCN[j];
                }
                for (unsigned j = 9; j<=23; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][6] * P[6][j];
                    res += KH[i][7] * P[7][j];
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

                // update the states
                // zero the attitude error state - by definition it is assumed to be zero before each observation fusion
                stateStruct.angErr.zero();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovRngBcn;
                }

                // the first 3 states represent the angular misalignment vector.
                // This is used to correct the estimated quaternion on the current time step
                stateStruct.quat.rotate(stateStruct.angErr);

                // record healthy fusion
                faultStatus.bad_rngbcn = false;

            } else {
                // record bad fusion
                faultStatus.bad_rngbcn = true;

            }
        }

        // Update the fusion report
        rngBcnFusionReport[rngBcnDataDelayed.beacon_ID].beaconPosNED = rngBcnDataDelayed.beacon_posNED;
        rngBcnFusionReport[rngBcnDataDelayed.beacon_ID].innov = innovRngBcn;
        rngBcnFusionReport[rngBcnDataDelayed.beacon_ID].innovVar = varInnovRngBcn;
        rngBcnFusionReport[rngBcnDataDelayed.beacon_ID].rng = rngBcnDataDelayed.rng;
        rngBcnFusionReport[rngBcnDataDelayed.beacon_ID].testRatio = rngBcnTestRatio;
    }
}

/*
Use range beacon measurements to calculate a static position using a 3-state EKF algorithm.
Algorithm based on the following:
https://github.com/priseborough/InertialNav/blob/master/derivations/range_beacon.m
*/
void NavEKF2_core::FuseRngBcnStatic()
{
    // get the estimated range measurement variance
    const ftype R_RNG = sq(MAX(rngBcnDataDelayed.rngErr , 0.1f));

    /*
    The first thing to do is to check if we have started the alignment and if not, initialise the
    states and covariance to a first guess. To do this iterate through the available beacons and then
    initialise the initial position to the mean beacon position. The initial position uncertainty
    is set to the mean range measurement.
    */
    if (!rngBcnAlignmentStarted) {
        if (rngBcnDataDelayed.beacon_ID != lastBeaconIndex) {
            rngBcnPosSum += rngBcnDataDelayed.beacon_posNED;
            lastBeaconIndex = rngBcnDataDelayed.beacon_ID;
            rngSum += rngBcnDataDelayed.rng;
            numBcnMeas++;

            // capture the beacon vertical spread
            if (rngBcnDataDelayed.beacon_posNED.z > maxBcnPosD) {
                maxBcnPosD = rngBcnDataDelayed.beacon_posNED.z;
            } else if(rngBcnDataDelayed.beacon_posNED.z < minBcnPosD) {
                minBcnPosD = rngBcnDataDelayed.beacon_posNED.z;
            }
        }
        if (numBcnMeas >= 100) {
            rngBcnAlignmentStarted = true;
            ftype tempVar = 1.0f / (float)numBcnMeas;
            // initialise the receiver position to the centre of the beacons and at zero height
            receiverPos.x = rngBcnPosSum.x * tempVar;
            receiverPos.y = rngBcnPosSum.y * tempVar;
            receiverPos.z = 0.0f;
            receiverPosCov[2][2] = receiverPosCov[1][1] = receiverPosCov[0][0] = rngSum * tempVar;
            lastBeaconIndex  = 0;
            numBcnMeas = 0;
            rngBcnPosSum.zero();
            rngSum = 0.0f;
        }
    }

    if (rngBcnAlignmentStarted) {
        numBcnMeas++;

        if (numBcnMeas >= 100) {
            // 100 observations is enough for a stable estimate under most conditions
            // TODO monitor stability of the position estimate
            rngBcnAlignmentCompleted = true;

        }

        if (rngBcnAlignmentCompleted) {
            if (activeHgtSource != HGT_SOURCE_BCN) {
                // We are using a different height reference for the main EKF so need to estimate a vertical
                // position offset to be applied to the beacon system that minimises the range innovations
                // The position estimate should be stable after 100 iterations so we use a simple dual
                // hypothesis 1-state EKF to estimate the offset
                Vector3F refPosNED;
                refPosNED.x = receiverPos.x;
                refPosNED.y = receiverPos.y;
                refPosNED.z = stateStruct.position.z;
                CalcRangeBeaconPosDownOffset(R_RNG, refPosNED, true);

            } else {
                // we are using the beacons as the primary height source, so don't modify their vertical position
                bcnPosOffset = 0.0f;

            }
        } else {
            if (activeHgtSource != HGT_SOURCE_BCN) {
                // The position estimate is not yet stable so we cannot run the 1-state EKF to estimate
                // beacon system vertical position offset. Instead we initialise the dual hypothesis offset states
                // using the beacon vertical position, vertical position estimate relative to the beacon origin
                // and the main EKF vertical position

                // Calculate the mid vertical position of all beacons
                ftype bcnMidPosD = 0.5f * (minBcnPosD + maxBcnPosD);

                // calculate the delta to the estimated receiver position
                ftype delta = receiverPos.z - bcnMidPosD;

                // calcuate the two hypothesis for our vertical position
                ftype receverPosDownMax;
                ftype receverPosDownMin;
                if (delta >= 0.0f) {
                    receverPosDownMax = receiverPos.z;
                    receverPosDownMin = receiverPos.z - 2.0f * delta;
                } else {
                    receverPosDownMax = receiverPos.z - 2.0f * delta;
                    receverPosDownMin = receiverPos.z;
                }

                bcnPosOffsetMax = stateStruct.position.z - receverPosDownMin;
                bcnPosOffsetMin = stateStruct.position.z - receverPosDownMax;
            } else {
                // We are using the beacons as the primary height reference, so don't modify their vertical position
                bcnPosOffset = 0.0f;
            }
        }

        // Add some process noise to the states at each time step
        for (uint8_t i= 0; i<=2; i++) {
            receiverPosCov[i][i] += 0.1f;
        }

        // calculate the observation jacobian
        ftype t2 = rngBcnDataDelayed.beacon_posNED.z - receiverPos.z + bcnPosOffset;
        ftype t3 = rngBcnDataDelayed.beacon_posNED.y - receiverPos.y;
        ftype t4 = rngBcnDataDelayed.beacon_posNED.x - receiverPos.x;
        ftype t5 = t2*t2;
        ftype t6 = t3*t3;
        ftype t7 = t4*t4;
        ftype t8 = t5+t6+t7;
        if (t8 < 0.1f) {
            // calculation will be badly conditioned
            return;
        }
        ftype t9 = 1.0f/sqrtF(t8);
        ftype t10 = rngBcnDataDelayed.beacon_posNED.x*2.0f;
        ftype t15 = receiverPos.x*2.0f;
        ftype t11 = t10-t15;
        ftype t12 = rngBcnDataDelayed.beacon_posNED.y*2.0f;
        ftype t14 = receiverPos.y*2.0f;
        ftype t13 = t12-t14;
        ftype t16 = rngBcnDataDelayed.beacon_posNED.z*2.0f;
        ftype t18 = receiverPos.z*2.0f;
        ftype t17 = t16-t18;
        ftype H_RNG[3];
        H_RNG[0] = -t9*t11*0.5f;
        H_RNG[1] = -t9*t13*0.5f;
        H_RNG[2] = -t9*t17*0.5f;

        // calculate the Kalman gains
        ftype t19 = receiverPosCov[0][0]*t9*t11*0.5f;
        ftype t20 = receiverPosCov[1][1]*t9*t13*0.5f;
        ftype t21 = receiverPosCov[0][1]*t9*t11*0.5f;
        ftype t22 = receiverPosCov[2][1]*t9*t17*0.5f;
        ftype t23 = t20+t21+t22;
        ftype t24 = t9*t13*t23*0.5f;
        ftype t25 = receiverPosCov[1][2]*t9*t13*0.5f;
        ftype t26 = receiverPosCov[0][2]*t9*t11*0.5f;
        ftype t27 = receiverPosCov[2][2]*t9*t17*0.5f;
        ftype t28 = t25+t26+t27;
        ftype t29 = t9*t17*t28*0.5f;
        ftype t30 = receiverPosCov[1][0]*t9*t13*0.5f;
        ftype t31 = receiverPosCov[2][0]*t9*t17*0.5f;
        ftype t32 = t19+t30+t31;
        ftype t33 = t9*t11*t32*0.5f;
        varInnovRngBcn = R_RNG+t24+t29+t33;
        ftype t35 = 1.0f/varInnovRngBcn;
        ftype K_RNG[3];
        K_RNG[0] = -t35*(t19+receiverPosCov[0][1]*t9*t13*0.5f+receiverPosCov[0][2]*t9*t17*0.5f);
        K_RNG[1] = -t35*(t20+receiverPosCov[1][0]*t9*t11*0.5f+receiverPosCov[1][2]*t9*t17*0.5f);
        K_RNG[2] = -t35*(t27+receiverPosCov[2][0]*t9*t11*0.5f+receiverPosCov[2][1]*t9*t13*0.5f);

        // calculate range measurement innovation
        Vector3F deltaPosNED = receiverPos - rngBcnDataDelayed.beacon_posNED;
        deltaPosNED.z -= bcnPosOffset;
        innovRngBcn = deltaPosNED.length() - rngBcnDataDelayed.rng;

        // update the state
        receiverPos.x -= K_RNG[0] * innovRngBcn;
        receiverPos.y -= K_RNG[1] * innovRngBcn;
        receiverPos.z -= K_RNG[2] * innovRngBcn;
        receiverPos.z = MAX(receiverPos.z, minBcnPosD + 1.2f);

        // calculate the covariance correction
        for (unsigned i = 0; i<=2; i++) {
            for (unsigned j = 0; j<=2; j++) {
                KH[i][j] = K_RNG[i] * H_RNG[j];
            }
        }
        for (unsigned j = 0; j<=2; j++) {
            for (unsigned i = 0; i<=2; i++) {
                ftype res = 0;
                res += KH[i][0] * receiverPosCov[0][j];
                res += KH[i][1] * receiverPosCov[1][j];
                res += KH[i][2] * receiverPosCov[2][j];
                KHP[i][j] = res;
            }
        }
        // prevent negative variances
        for (uint8_t i= 0; i<=2; i++) {
            if (receiverPosCov[i][i] < 0.0f) {
                receiverPosCov[i][i] = 0.0f;
                KHP[i][i] = 0.0f;
            } else if (KHP[i][i] > receiverPosCov[i][i]) {
                KHP[i][i] = receiverPosCov[i][i];
            }
        }
        // apply the covariance correction
        for (uint8_t i= 0; i<=2; i++) {
            for (uint8_t j= 0; j<=2; j++) {
                receiverPosCov[i][j] -= KHP[i][j];
            }
        }
        // ensure the covariance matrix is symmetric
        for (uint8_t i=1; i<=2; i++) {
            for (uint8_t j=0; j<=i-1; j++) {
                ftype temp = 0.5f*(receiverPosCov[i][j] + receiverPosCov[j][i]);
                receiverPosCov[i][j] = temp;
                receiverPosCov[j][i] = temp;
            }
        }

        if (numBcnMeas >= 100) {
            // 100 observations is enough for a stable estimate under most conditions
            // TODO monitor stability of the position estimate
            rngBcnAlignmentCompleted = true;
        }
    }
}

/*
Run a single state Kalman filter to estimate the vertical position offset of the range beacon constellation
Calculate using a high and low hypothesis and select the hypothesis with the lowest innovation sequence
*/
void NavEKF2_core::CalcRangeBeaconPosDownOffset(ftype obsVar, Vector3F &vehiclePosNED, bool aligning)
{
    // Handle height offsets between the primary height source and the range beacons by estimating
    // the beacon systems global vertical position offset using a single state Kalman filter
    // The estimated offset is used to correct the beacon height when calculating innovations
    // A high and low estimate is calculated to handle the ambiguity in height associated with beacon positions that are co-planar
    // The main filter then uses the offset with the smaller innovations

    ftype innov;    // range measurement innovation (m)
    ftype innovVar; // range measurement innovation variance (m^2)
    ftype gain;     // Kalman gain
    ftype obsDeriv; // derivative of observation relative to state

    const ftype stateNoiseVar = 0.1f; // State process noise variance
    const ftype filtAlpha = 0.01f; // LPF constant
    const ftype innovGateWidth = 5.0f; // width of innovation consistency check gate in std

    // estimate upper value for offset

    // calculate observation derivative
    ftype t2 = rngBcnDataDelayed.beacon_posNED.z - vehiclePosNED.z + bcnPosOffsetMax;
    ftype t3 = rngBcnDataDelayed.beacon_posNED.y - vehiclePosNED.y;
    ftype t4 = rngBcnDataDelayed.beacon_posNED.x - vehiclePosNED.x;
    ftype t5 = t2*t2;
    ftype t6 = t3*t3;
    ftype t7 = t4*t4;
    ftype t8 = t5+t6+t7;
    ftype t9;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - rngBcnDataDelayed.rng;

        // calculate a filtered innovation magnitude to be used to select between the high or low offset
        OffsetMaxInnovFilt = (1.0f - filtAlpha) * bcnPosOffsetMaxVar + filtAlpha * fabsF(innov);

        // covariance prediction
        bcnPosOffsetMaxVar += stateNoiseVar;

        // calculate the innovation variance
        innovVar = obsDeriv * bcnPosOffsetMaxVar * obsDeriv + obsVar;
        innovVar = MAX(innovVar, obsVar);

        // Reject range innovation spikes using a 5-sigma threshold unless aligning
        if ((sq(innov) < sq(innovGateWidth) * innovVar) || aligning) {
            // calculate the Kalman gain
            gain = (bcnPosOffsetMaxVar * obsDeriv) / innovVar;

            // state update
            bcnPosOffsetMax -= innov * gain;

            // covariance update
            bcnPosOffsetMaxVar -= gain * obsDeriv * bcnPosOffsetMaxVar;
            bcnPosOffsetMaxVar = MAX(bcnPosOffsetMaxVar, 0.0f);
        }
    }

    // estimate lower value for offset

    // calculate observation derivative
    t2 = rngBcnDataDelayed.beacon_posNED.z - vehiclePosNED.z + bcnPosOffsetMin;
    t5 = t2*t2;
    t8 = t5+t6+t7;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - rngBcnDataDelayed.rng;

        // calculate a filtered innovation magnitude to be used to select between the high or low offset
        OffsetMinInnovFilt = (1.0f - filtAlpha) * OffsetMinInnovFilt + filtAlpha * fabsF(innov);

        // covariance prediction
        bcnPosOffsetMinVar += stateNoiseVar;

        // calculate the innovation variance
        innovVar = obsDeriv * bcnPosOffsetMinVar * obsDeriv + obsVar;
        innovVar = MAX(innovVar, obsVar);

        // Reject range innovation spikes using a 5-sigma threshold unless aligning
        if ((sq(innov) < sq(innovGateWidth) * innovVar) || aligning) {
            // calculate the Kalman gain
            gain = (bcnPosOffsetMinVar * obsDeriv) / innovVar;

            // state update
            bcnPosOffsetMin -= innov * gain;

            // covariance update
            bcnPosOffsetMinVar -= gain * obsDeriv * bcnPosOffsetMinVar;
            bcnPosOffsetMinVar = MAX(bcnPosOffsetMinVar, 0.0f);
        }
    }

    // calculate the mid vertical position of all beacons
    ftype bcnMidPosD = 0.5f * (minBcnPosD + maxBcnPosD);

    // ensure the two beacon vertical offset hypothesis place the mid point of the beacons below and above the flight vehicle
    bcnPosOffsetMax = MAX(bcnPosOffsetMax, vehiclePosNED.z - bcnMidPosD + 0.5f);
    bcnPosOffsetMin  = MIN(bcnPosOffsetMin,  vehiclePosNED.z - bcnMidPosD - 0.5f);

    // calculate the innovation for the main filter using the offset with the smallest innovation history
    if (OffsetMaxInnovFilt > OffsetMinInnovFilt) {
        bcnPosOffset = bcnPosOffsetMin;
    } else {
        bcnPosOffset = bcnPosOffsetMax;
    }

}

