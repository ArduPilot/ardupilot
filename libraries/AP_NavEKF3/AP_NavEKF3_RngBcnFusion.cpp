#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"

#if EK3_FEATURE_BEACON_FUSION

#include <AP_DAL/AP_DAL.h>

// initialise state:
void NavEKF3_core::BeaconFusion::InitialiseVariables()
{
    memset((void *)&dataDelayed, 0, sizeof(dataDelayed));
    lastPassTime_ms = 0;
    testRatio = 0.0f;
    health = false;
    varInnov = 0.0f;
    innov = 0.0f;
    memset(&lastTime_ms, 0, sizeof(lastTime_ms));
    dataToFuse = false;
    vehiclePosNED.zero();
    vehiclePosErr = 1.0f;
    last3DmeasTime_ms = 0;
    goodToAlign = false;
    lastChecked = 0;
    receiverPos.zero();
    memset(&receiverPosCov, 0, sizeof(receiverPosCov));
    alignmentStarted =  false;
    alignmentCompleted = false;
    lastIndex = 0;
    posSum.zero();
    numMeas = 0;
    sum = 0.0f;
    N = 0;
    maxPosD = 0.0f;
    minPosD = 0.0f;
    posDownOffsetMax = 0.0f;
    posOffsetMaxVar = 0.0f;
    maxOffsetStateChangeFilt = 0.0f;
    posDownOffsetMin = 0.0f;
    posOffsetMinVar = 0.0f;
    minOffsetStateChangeFilt = 0.0f;
    fuseDataReportIndex = 0;
    delete[] fusionReport;
    fusionReport = nullptr;
    numFusionReports = 0;
    auto *beacon = dal.beacon();
    if (beacon != nullptr) {
        const uint8_t count = beacon->count();
        fusionReport = NEW_NOTHROW BeaconFusion::FusionReport[count];
        if (fusionReport != nullptr) {
            numFusionReports = count;
        }
    }
    posOffsetNED.zero();
    originEstInit = false;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of range beacon measurements
void NavEKF3_core::SelectRngBcnFusion()
{
    // read range data from the sensor and check for new data in the buffer
    readRngBcnData();

    // Determine if we need to fuse range beacon data on this time step
    if (rngBcn.dataToFuse) {
        if (PV_AidingMode == AID_ABSOLUTE) {
            if ((frontend->sources.getPosXYSource() == AP_NavEKF_Source::SourceXY::BEACON) && rngBcn.alignmentCompleted) {
                if (!rngBcn.originEstInit) {
                    rngBcn.originEstInit = true;
                    rngBcn.posOffsetNED.x = rngBcn.receiverPos.x - stateStruct.position.x;
                    rngBcn.posOffsetNED.y = rngBcn.receiverPos.y - stateStruct.position.y;
                }
                // beacons are used as the primary means of position reference
                FuseRngBcn();
            } else {
                // If another source (i.e. GPS, ExtNav) is the primary reference, we continue to use the beacon data
                // to calculate an independent position that is used to update the beacon position offset if we need to
                // start using beacon data as the primary reference.
                FuseRngBcnStatic();
                // record that the beacon origin needs to be initialised
                rngBcn.originEstInit = false;
            }
        } else {
            // If we aren't able to use the data in the main filter, use a simple 3-state filter to estimate position only
            FuseRngBcnStatic();
            // record that the beacon origin needs to be initialised
            rngBcn.originEstInit = false;
        }
    }
}

void NavEKF3_core::FuseRngBcn()
{
    // declarations
    ftype pn;
    ftype pe;
    ftype pd;
    ftype bcn_pn;
    ftype bcn_pe;
    ftype bcn_pd;
    const ftype R_BCN = sq(MAX(rngBcn.dataDelayed.rngErr , 0.1f));
    ftype rngPred;

    // health is set bad until test passed
    rngBcn.health = false;

    if (activeHgtSource != AP_NavEKF_Source::SourceZ::BEACON) {
        // calculate the vertical offset from EKF datum to beacon datum
        CalcRangeBeaconPosDownOffset(R_BCN, stateStruct.position, false);
    } else {
        rngBcn.posOffsetNED.z = 0.0f;
    }

    // copy required states to local variable names
    pn = stateStruct.position.x;
    pe = stateStruct.position.y;
    pd = stateStruct.position.z;
    bcn_pn = rngBcn.dataDelayed.beacon_posNED.x;
    bcn_pe = rngBcn.dataDelayed.beacon_posNED.y;
    bcn_pd = rngBcn.dataDelayed.beacon_posNED.z + rngBcn.posOffsetNED.z;

    // predicted range
    Vector3F deltaPosNED = stateStruct.position - rngBcn.dataDelayed.beacon_posNED;
    rngPred = deltaPosNED.length();

    // calculate measurement innovation
    rngBcn.innov = rngPred - rngBcn.dataDelayed.rng;

    // perform fusion of range measurement
    if (rngPred > 0.1f)
    {
        // calculate observation jacobians
        ftype H_BCN[24];
        memset(H_BCN, 0, sizeof(H_BCN));
        ftype t2 = bcn_pd-pd;
        ftype t3 = bcn_pe-pe;
        ftype t4 = bcn_pn-pn;
        ftype t5 = t2*t2;
        ftype t6 = t3*t3;
        ftype t7 = t4*t4;
        ftype t8 = t5+t6+t7;
        ftype t9 = 1.0f/sqrtF(t8);
        H_BCN[7] = -t4*t9;
        H_BCN[8] = -t3*t9;
        // If we are not using the beacons as a height reference, we pretend that the beacons
        // are at the same height as the flight vehicle when calculating the observation derivatives
        // and Kalman gains
        // TODO  - less hacky way of achieving this, preferably using an alternative derivation
        if (activeHgtSource != AP_NavEKF_Source::SourceZ::BEACON) {
            t2 = 0.0f;
        }
        H_BCN[9] = -t2*t9;

        // calculate Kalman gains
        ftype t10 = P[9][9]*t2*t9;
        ftype t11 = P[8][9]*t3*t9;
        ftype t12 = P[7][9]*t4*t9;
        ftype t13 = t10+t11+t12;
        ftype t14 = t2*t9*t13;
        ftype t15 = P[9][8]*t2*t9;
        ftype t16 = P[8][8]*t3*t9;
        ftype t17 = P[7][8]*t4*t9;
        ftype t18 = t15+t16+t17;
        ftype t19 = t3*t9*t18;
        ftype t20 = P[9][7]*t2*t9;
        ftype t21 = P[8][7]*t3*t9;
        ftype t22 = P[7][7]*t4*t9;
        ftype t23 = t20+t21+t22;
        ftype t24 = t4*t9*t23;
        rngBcn.varInnov = R_BCN+t14+t19+t24;
        ftype t26;
        if (rngBcn.varInnov >= R_BCN) {
            t26 = 1.0f/rngBcn.varInnov;
            faultStatus.bad_rngbcn = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_rngbcn = true;
            return;
        }

        Kfusion[0] = -t26*(P[0][7]*t4*t9+P[0][8]*t3*t9+P[0][9]*t2*t9);
        Kfusion[1] = -t26*(P[1][7]*t4*t9+P[1][8]*t3*t9+P[1][9]*t2*t9);
        Kfusion[2] = -t26*(P[2][7]*t4*t9+P[2][8]*t3*t9+P[2][9]*t2*t9);
        Kfusion[3] = -t26*(P[3][7]*t4*t9+P[3][8]*t3*t9+P[3][9]*t2*t9);
        Kfusion[4] = -t26*(P[4][7]*t4*t9+P[4][8]*t3*t9+P[4][9]*t2*t9);
        Kfusion[5] = -t26*(P[5][7]*t4*t9+P[5][8]*t3*t9+P[5][9]*t2*t9);
        Kfusion[7] = -t26*(t22+P[7][8]*t3*t9+P[7][9]*t2*t9);
        Kfusion[8] = -t26*(t16+P[8][7]*t4*t9+P[8][9]*t2*t9);

        if (!inhibitDelAngBiasStates) {
            Kfusion[10] = -t26*(P[10][7]*t4*t9+P[10][8]*t3*t9+P[10][9]*t2*t9);
            Kfusion[11] = -t26*(P[11][7]*t4*t9+P[11][8]*t3*t9+P[11][9]*t2*t9);
            Kfusion[12] = -t26*(P[12][7]*t4*t9+P[12][8]*t3*t9+P[12][9]*t2*t9);
        } else {
            // zero indexes 10 to 12
            zero_range(&Kfusion[0], 10, 12);
        }

        if (!inhibitDelVelBiasStates && !badIMUdata) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    Kfusion[stateIndex] = -t26*(P[stateIndex][7]*t4*t9+P[stateIndex][8]*t3*t9+P[stateIndex][9]*t2*t9);
                } else {
                    Kfusion[stateIndex] = 0.0f;
                }
            }
        } else {
            // zero indexes 13 to 15
            zero_range(&Kfusion[0], 13, 15);
        }

        // only allow the range observations to modify the vertical states if we are using it as a height reference
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::BEACON) {
            Kfusion[6] = -t26*(P[6][7]*t4*t9+P[6][8]*t3*t9+P[6][9]*t2*t9);
            Kfusion[9] = -t26*(t10+P[9][7]*t4*t9+P[9][8]*t3*t9);
        } else {
            Kfusion[6] = 0.0f;
            Kfusion[9] = 0.0f;
        }

        if (!inhibitMagStates) {
            Kfusion[16] = -t26*(P[16][7]*t4*t9+P[16][8]*t3*t9+P[16][9]*t2*t9);
            Kfusion[17] = -t26*(P[17][7]*t4*t9+P[17][8]*t3*t9+P[17][9]*t2*t9);
            Kfusion[18] = -t26*(P[18][7]*t4*t9+P[18][8]*t3*t9+P[18][9]*t2*t9);
            Kfusion[19] = -t26*(P[19][7]*t4*t9+P[19][8]*t3*t9+P[19][9]*t2*t9);
            Kfusion[20] = -t26*(P[20][7]*t4*t9+P[20][8]*t3*t9+P[20][9]*t2*t9);
            Kfusion[21] = -t26*(P[21][7]*t4*t9+P[21][8]*t3*t9+P[21][9]*t2*t9);
        } else {
            // zero indexes 16 to 21
            zero_range(&Kfusion[0], 16, 21);
        }

        if (!inhibitWindStates && !treatWindStatesAsTruth) {
            Kfusion[22] = -t26*(P[22][7]*t4*t9+P[22][8]*t3*t9+P[22][9]*t2*t9);
            Kfusion[23] = -t26*(P[23][7]*t4*t9+P[23][8]*t3*t9+P[23][9]*t2*t9);
        } else {
            // zero indexes 22 to 23
            zero_range(&Kfusion[0], 22, 23);
        }

        // Calculate innovation using the selected offset value
        Vector3F delta = stateStruct.position - rngBcn.dataDelayed.beacon_posNED;
        rngBcn.innov = delta.length() - rngBcn.dataDelayed.rng;

        // calculate the innovation consistency test ratio
        rngBcn.testRatio = sq(rngBcn.innov) / (sq(MAX(0.01f * (ftype)frontend->_rngBcnInnovGate, 1.0f)) * rngBcn.varInnov);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        rngBcn.health = ((rngBcn.testRatio < 1.0f) || badIMUdata);

        // test the ratio before fusing data
        if (rngBcn.health) {

            // restart the counter
            rngBcn.lastPassTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=6; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 7; j<=9; j++) {
                    KH[i][j] = Kfusion[i] * H_BCN[j];
                }
                for (unsigned j = 10; j<=23; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][7] * P[7][j];
                    res += KH[i][8] * P[8][j];
                    res += KH[i][9] * P[9][j];
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
                    statesArray[j] = statesArray[j] - Kfusion[j] * rngBcn.innov;
                }

                // record healthy fusion
                faultStatus.bad_rngbcn = false;

            } else {
                // record bad fusion
                faultStatus.bad_rngbcn = true;

            }
        }

        // Update the fusion report
        if (rngBcn.dataDelayed.beacon_ID < rngBcn.numFusionReports) {
            auto &report = rngBcn.fusionReport[rngBcn.dataDelayed.beacon_ID];
            report.beaconPosNED = rngBcn.dataDelayed.beacon_posNED;
            report.innov = rngBcn.innov;
            report.innovVar = rngBcn.varInnov;
            report.rng = rngBcn.dataDelayed.rng;
            report.testRatio = rngBcn.testRatio;
        }
    }
}

/*
Use range beacon measurements to calculate a static position using a 3-state EKF algorithm.
Algorithm based on the following:
https://github.com/priseborough/InertialNav/blob/master/derivations/range_beacon.m
*/
void NavEKF3_core::FuseRngBcnStatic()
{
    // get the estimated range measurement variance
    const ftype R_RNG = sq(MAX(rngBcn.dataDelayed.rngErr , 0.1f));

    /*
    The first thing to do is to check if we have started the alignment and if not, initialise the
    states and covariance to a first guess. To do this iterate through the available beacons and then
    initialise the initial position to the mean beacon position. The initial position uncertainty
    is set to the mean range measurement.
    */
    if (!rngBcn.alignmentStarted) {
        if (rngBcn.dataDelayed.beacon_ID != rngBcn.lastIndex) {
            rngBcn.posSum += rngBcn.dataDelayed.beacon_posNED;
            rngBcn.lastIndex = rngBcn.dataDelayed.beacon_ID;
            rngBcn.sum += rngBcn.dataDelayed.rng;
            rngBcn.numMeas++;

            // capture the beacon vertical spread
            if (rngBcn.dataDelayed.beacon_posNED.z > rngBcn.maxPosD) {
                rngBcn.maxPosD = rngBcn.dataDelayed.beacon_posNED.z;
            } else if(rngBcn.dataDelayed.beacon_posNED.z < rngBcn.minPosD) {
                rngBcn.minPosD = rngBcn.dataDelayed.beacon_posNED.z;
            }
        }
        if (rngBcn.numMeas >= 100) {
            rngBcn.alignmentStarted = true;
            ftype tempVar = 1.0f / (ftype)rngBcn.numMeas;
            // initialise the receiver position to the centre of the beacons and at zero height
            rngBcn.receiverPos.x = rngBcn.posSum.x * tempVar;
            rngBcn.receiverPos.y = rngBcn.posSum.y * tempVar;
            rngBcn.receiverPos.z = 0.0f;
            rngBcn.receiverPosCov[2][2] = rngBcn.receiverPosCov[1][1] = rngBcn.receiverPosCov[0][0] = rngBcn.sum * tempVar;
            rngBcn.lastIndex  = 0;
            rngBcn.numMeas = 0;
            rngBcn.posSum.zero();
            rngBcn.sum = 0.0f;
        }
    }

    if (rngBcn.alignmentStarted) {
        rngBcn.numMeas++;

        if (rngBcn.numMeas >= 100) {
            // 100 observations is enough for a stable estimate under most conditions
            // TODO monitor stability of the position estimate
            rngBcn.alignmentCompleted = true;

        }

        if (rngBcn.alignmentCompleted) {
            if (activeHgtSource != AP_NavEKF_Source::SourceZ::BEACON) {
                // We are using a different height reference for the main EKF so need to estimate a vertical
                // position offset to be applied to the beacon system that minimises the range innovations
                // The position estimate should be stable after 100 iterations so we use a simple dual
                // hypothesis 1-state EKF to estimate the offset
                Vector3F refPosNED;
                refPosNED.x = rngBcn.receiverPos.x;
                refPosNED.y = rngBcn.receiverPos.y;
                refPosNED.z = stateStruct.position.z;
                CalcRangeBeaconPosDownOffset(R_RNG, refPosNED, true);

            } else {
                // we are using the beacons as the primary height source, so don't modify their vertical position
                rngBcn.posOffsetNED.z = 0.0f;

            }
        } else {
            if (activeHgtSource != AP_NavEKF_Source::SourceZ::BEACON) {
                // The position estimate is not yet stable so we cannot run the 1-state EKF to estimate
                // beacon system vertical position offset. Instead we initialise the dual hypothesis offset states
                // using the beacon vertical position, vertical position estimate relative to the beacon origin
                // and the main EKF vertical position

                // Calculate the mid vertical position of all beacons
                ftype bcnMidPosD = 0.5f * (rngBcn.minPosD + rngBcn.maxPosD);

                // calculate the delta to the estimated receiver position
                ftype delta = rngBcn.receiverPos.z - bcnMidPosD;

                // calculate the two hypothesis for our vertical position
                ftype receiverPosDownMax;
                ftype receiverPosDownMin;
                if (delta >= 0.0f) {
                    receiverPosDownMax = rngBcn.receiverPos.z;
                    receiverPosDownMin = rngBcn.receiverPos.z - 2.0f * delta;
                } else {
                    receiverPosDownMax = rngBcn.receiverPos.z - 2.0f * delta;
                    receiverPosDownMin = rngBcn.receiverPos.z;
                }

                rngBcn.posDownOffsetMax = stateStruct.position.z - receiverPosDownMin;
                rngBcn.posDownOffsetMin = stateStruct.position.z - receiverPosDownMax;
            } else {
                // We are using the beacons as the primary height reference, so don't modify their vertical position
                rngBcn.posOffsetNED.z = 0.0f;
            }
        }

        // Add some process noise to the states at each time step
        for (uint8_t i= 0; i<=2; i++) {
            rngBcn.receiverPosCov[i][i] += 0.1f;
        }

        // calculate the observation jacobian
        ftype t2 = rngBcn.dataDelayed.beacon_posNED.z - rngBcn.receiverPos.z + rngBcn.posOffsetNED.z;
        ftype t3 = rngBcn.dataDelayed.beacon_posNED.y - rngBcn.receiverPos.y;
        ftype t4 = rngBcn.dataDelayed.beacon_posNED.x - rngBcn.receiverPos.x;
        ftype t5 = t2*t2;
        ftype t6 = t3*t3;
        ftype t7 = t4*t4;
        ftype t8 = t5+t6+t7;
        if (t8 < 0.1f) {
            // calculation will be badly conditioned
            return;
        }
        ftype t9 = 1.0f/sqrtF(t8);
        ftype t10 = rngBcn.dataDelayed.beacon_posNED.x*2.0f;
        ftype t15 = rngBcn.receiverPos.x*2.0f;
        ftype t11 = t10-t15;
        ftype t12 = rngBcn.dataDelayed.beacon_posNED.y*2.0f;
        ftype t14 = rngBcn.receiverPos.y*2.0f;
        ftype t13 = t12-t14;
        ftype t16 = rngBcn.dataDelayed.beacon_posNED.z*2.0f;
        ftype t18 = rngBcn.receiverPos.z*2.0f;
        ftype t17 = t16-t18;
        ftype H_RNG[3];
        H_RNG[0] = -t9*t11*0.5f;
        H_RNG[1] = -t9*t13*0.5f;
        H_RNG[2] = -t9*t17*0.5f;

        // calculate the Kalman gains
        ftype t19 = rngBcn.receiverPosCov[0][0]*t9*t11*0.5f;
        ftype t20 = rngBcn.receiverPosCov[1][1]*t9*t13*0.5f;
        ftype t21 = rngBcn.receiverPosCov[0][1]*t9*t11*0.5f;
        ftype t22 = rngBcn.receiverPosCov[2][1]*t9*t17*0.5f;
        ftype t23 = t20+t21+t22;
        ftype t24 = t9*t13*t23*0.5f;
        ftype t25 = rngBcn.receiverPosCov[1][2]*t9*t13*0.5f;
        ftype t26 = rngBcn.receiverPosCov[0][2]*t9*t11*0.5f;
        ftype t27 = rngBcn.receiverPosCov[2][2]*t9*t17*0.5f;
        ftype t28 = t25+t26+t27;
        ftype t29 = t9*t17*t28*0.5f;
        ftype t30 = rngBcn.receiverPosCov[1][0]*t9*t13*0.5f;
        ftype t31 = rngBcn.receiverPosCov[2][0]*t9*t17*0.5f;
        ftype t32 = t19+t30+t31;
        ftype t33 = t9*t11*t32*0.5f;
        rngBcn.varInnov = R_RNG+t24+t29+t33;
        ftype t35 = 1.0f/rngBcn.varInnov;
        ftype K_RNG[3];
        K_RNG[0] = -t35*(t19+rngBcn.receiverPosCov[0][1]*t9*t13*0.5f+rngBcn.receiverPosCov[0][2]*t9*t17*0.5f);
        K_RNG[1] = -t35*(t20+rngBcn.receiverPosCov[1][0]*t9*t11*0.5f+rngBcn.receiverPosCov[1][2]*t9*t17*0.5f);
        K_RNG[2] = -t35*(t27+rngBcn.receiverPosCov[2][0]*t9*t11*0.5f+rngBcn.receiverPosCov[2][1]*t9*t13*0.5f);

        // calculate range measurement innovation
        Vector3F deltaPosNED = rngBcn.receiverPos - rngBcn.dataDelayed.beacon_posNED;
        deltaPosNED.z -= rngBcn.posOffsetNED.z;
        rngBcn.innov = deltaPosNED.length() - rngBcn.dataDelayed.rng;

        // calculate the innovation consistency test ratio
        rngBcn.testRatio = sq(rngBcn.innov) / (sq(MAX(0.01f * (ftype)frontend->_rngBcnInnovGate, 1.0f)) * rngBcn.varInnov);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        rngBcn.health = ((rngBcn.testRatio < 1.0f) || badIMUdata || !rngBcn.alignmentCompleted);

        // test the ratio before fusing data
        if (rngBcn.health) {

            // update the state
            rngBcn.receiverPos.x -= K_RNG[0] * rngBcn.innov;
            rngBcn.receiverPos.y -= K_RNG[1] * rngBcn.innov;
            rngBcn.receiverPos.z -= K_RNG[2] * rngBcn.innov;

            // calculate the covariance correction
            for (unsigned i = 0; i<=2; i++) {
                for (unsigned j = 0; j<=2; j++) {
                    KH[i][j] = K_RNG[i] * H_RNG[j];
                }
            }
            for (unsigned j = 0; j<=2; j++) {
                for (unsigned i = 0; i<=2; i++) {
                    ftype res = 0;
                    res += KH[i][0] * rngBcn.receiverPosCov[0][j];
                    res += KH[i][1] * rngBcn.receiverPosCov[1][j];
                    res += KH[i][2] * rngBcn.receiverPosCov[2][j];
                    KHP[i][j] = res;
                }
            }

            // prevent negative variances
            for (uint8_t i= 0; i<=2; i++) {
                if (rngBcn.receiverPosCov[i][i] < 0.0f) {
                    rngBcn.receiverPosCov[i][i] = 0.0f;
                    KHP[i][i] = 0.0f;
                } else if (KHP[i][i] > rngBcn.receiverPosCov[i][i]) {
                    KHP[i][i] = rngBcn.receiverPosCov[i][i];
                }
            }

            // apply the covariance correction
            for (uint8_t i= 0; i<=2; i++) {
                for (uint8_t j= 0; j<=2; j++) {
                    rngBcn.receiverPosCov[i][j] -= KHP[i][j];
                }
            }

            // ensure the covariance matrix is symmetric
            for (uint8_t i=1; i<=2; i++) {
                for (uint8_t j=0; j<=i-1; j++) {
                    ftype temp = 0.5f*(rngBcn.receiverPosCov[i][j] + rngBcn.receiverPosCov[j][i]);
                    rngBcn.receiverPosCov[i][j] = temp;
                    rngBcn.receiverPosCov[j][i] = temp;
                }
            }

        }

        if (rngBcn.numMeas >= 100) {
            // 100 observations is enough for a stable estimate under most conditions
            // TODO monitor stability of the position estimate
            rngBcn.alignmentCompleted = true;
        }
        // Update the fusion report
        if (rngBcn.dataDelayed.beacon_ID < rngBcn.numFusionReports) {
            auto &report = rngBcn.fusionReport[rngBcn.dataDelayed.beacon_ID];
            report.beaconPosNED = rngBcn.dataDelayed.beacon_posNED;
            report.innov = rngBcn.innov;
            report.innovVar = rngBcn.varInnov;
            report.rng = rngBcn.dataDelayed.rng;
            report.testRatio = rngBcn.testRatio;
        }
    }
}

/*
Run a single state Kalman filter to estimate the vertical position offset of the range beacon constellation
Calculate using a high and low hypothesis and select the hypothesis with the lowest innovation sequence
*/
void NavEKF3_core::CalcRangeBeaconPosDownOffset(ftype obsVar, Vector3F &vehiclePosNED, bool aligning)
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
    const ftype filtAlpha = 0.1f; // LPF constant
    const ftype innovGateWidth = 5.0f; // width of innovation consistency check gate in std

    // estimate upper value for offset

    // calculate observation derivative
    ftype t2 = rngBcn.dataDelayed.beacon_posNED.z - vehiclePosNED.z + rngBcn.posDownOffsetMax;
    ftype t3 = rngBcn.dataDelayed.beacon_posNED.y - vehiclePosNED.y;
    ftype t4 = rngBcn.dataDelayed.beacon_posNED.x - vehiclePosNED.x;
    ftype t5 = t2*t2;
    ftype t6 = t3*t3;
    ftype t7 = t4*t4;
    ftype t8 = t5+t6+t7;
    ftype t9;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - rngBcn.dataDelayed.rng;

        // covariance prediction
        rngBcn.posOffsetMaxVar += stateNoiseVar;

        // calculate the innovation variance
        innovVar = obsDeriv * rngBcn.posOffsetMaxVar * obsDeriv + obsVar;
        innovVar = MAX(innovVar, obsVar);

        // calculate the Kalman gain
        gain = (rngBcn.posOffsetMaxVar * obsDeriv) / innovVar;

        // calculate a filtered state change magnitude to be used to select between the high or low offset
        ftype stateChange = innov * gain;
        rngBcn.maxOffsetStateChangeFilt = (1.0f - filtAlpha) * rngBcn.maxOffsetStateChangeFilt + fminF(fabsF(filtAlpha * stateChange) , 1.0f);

        // Reject range innovation spikes using a 5-sigma threshold unless aligning
        if ((sq(innov) < sq(innovGateWidth) * innovVar) || aligning) {

            // state update
            rngBcn.posDownOffsetMax -= stateChange;

            // covariance update
            rngBcn.posOffsetMaxVar -= gain * obsDeriv * rngBcn.posOffsetMaxVar;
            rngBcn.posOffsetMaxVar = MAX(rngBcn.posOffsetMaxVar, 0.0f);
        }
    }

    // estimate lower value for offset

    // calculate observation derivative
    t2 = rngBcn.dataDelayed.beacon_posNED.z - vehiclePosNED.z + rngBcn.posDownOffsetMin;
    t5 = t2*t2;
    t8 = t5+t6+t7;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - rngBcn.dataDelayed.rng;

        // covariance prediction
        rngBcn.posOffsetMinVar += stateNoiseVar;

        // calculate the innovation variance
        innovVar = obsDeriv * rngBcn.posOffsetMinVar * obsDeriv + obsVar;
        innovVar = MAX(innovVar, obsVar);

        // calculate the Kalman gain
        gain = (rngBcn.posOffsetMinVar * obsDeriv) / innovVar;

        // calculate a filtered state change magnitude to be used to select between the high or low offset
        ftype stateChange = innov * gain;
        rngBcn.minOffsetStateChangeFilt = (1.0f - filtAlpha) * rngBcn.minOffsetStateChangeFilt + fminF(fabsF(filtAlpha * stateChange) , 1.0f);

        // Reject range innovation spikes using a 5-sigma threshold unless aligning
        if ((sq(innov) < sq(innovGateWidth) * innovVar) || aligning) {

            // state update
            rngBcn.posDownOffsetMin -= stateChange;

            // covariance update
            rngBcn.posOffsetMinVar -= gain * obsDeriv * rngBcn.posOffsetMinVar;
            rngBcn.posOffsetMinVar = MAX(rngBcn.posOffsetMinVar, 0.0f);
        }
    }

    // calculate the mid vertical position of all beacons
    ftype bcnMidPosD = 0.5f * (rngBcn.minPosD + rngBcn.maxPosD);

    // ensure the two beacon vertical offset hypothesis place the mid point of the beacons below and above the flight vehicle
    rngBcn.posDownOffsetMax = MAX(rngBcn.posDownOffsetMax, vehiclePosNED.z - bcnMidPosD + 0.5f);
    rngBcn.posDownOffsetMin  = MIN(rngBcn.posDownOffsetMin,  vehiclePosNED.z - bcnMidPosD - 0.5f);

    // calculate the innovation for the main filter using the offset that is most stable
    // apply hysteresis to prevent rapid switching
    if (!rngBcn.usingMinHypothesis && (rngBcn.minOffsetStateChangeFilt < (0.8f * rngBcn.maxOffsetStateChangeFilt))) {
        rngBcn.usingMinHypothesis = true;
    } else if (rngBcn.usingMinHypothesis && (rngBcn.maxOffsetStateChangeFilt < (0.8f * rngBcn.minOffsetStateChangeFilt))) {
        rngBcn.usingMinHypothesis = false;
    }
    if (rngBcn.usingMinHypothesis) {
        rngBcn.posOffsetNED.z = rngBcn.posDownOffsetMin;
    } else {
        rngBcn.posOffsetNED.z = rngBcn.posDownOffsetMax;
    }

    // apply the vertical offset to the beacon positions
    rngBcn.dataDelayed.beacon_posNED.z += rngBcn.posOffsetNED.z;
}

#endif  // EK3_FEATURE_BEACON_FUSION
