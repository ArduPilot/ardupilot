#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_Logger/AP_Logger.h>

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
    auto *beacon = dal.beacon();
    if (beacon != nullptr) {
        const uint8_t count = beacon->count();
        fusionReport = NEW_NOTHROW BeaconFusion::FusionReport[count];
    }
    posOffsetNED.zero();
    originEstInit = false;
#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    verticalOffsetVariance = 0.0f;
    verticalOffset = 0.0f;
#endif
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of range beacon measurements
void NavEKF3_core::SelectRngBcnFusion()
{
#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    // range to location data is being pushed externally to the EKF so we need to check the buffer
    if (rngBcn.usingRangeToLoc) {
        uint32_t new_data_mask = 0;
        for (uint8_t i=0; i<rngBcn.N; i++) {
            auto &dataDelayed = rngBcn.dataDelayed[i];
            // recall data into our dataDelayed array
            if (!rngBcn.storedRange[i].recall(dataDelayed, imuDataDelayed.time_ms)) {
                // no new data
                continue;
            }
            if (dataDelayed.beacon_ID >= AP_BEACON_MAX_BEACONS) {
                // invalid beacon ID
                continue;
            }
            // create dataLast structure by copying and then modifying
            // dataDelayed:
            auto &dataLast = rngBcn.dataLast[dataDelayed.beacon_ID];
            dataLast = dataDelayed;
            dataLast.beacon_posNED = EKF_origin.get_distance_NED_ftype(dataDelayed.beacon_loc);
            rngBcn.receiverPos.zero();
            // correct for time offset and range rate
            Vector3F deltaPosNED = stateStruct.position - dataDelayed.beacon_posNED;
            const auto &deltaxy = deltaPosNED.xy();
            if (deltaxy.length() < MAX(1.0f, dataDelayed.rngErr)) {
                // protect against /0 and badly conditioned range fusion that occurs if uncertainty is large relative to range
                continue;
            }
            const ftype rangeRate = stateStruct.velocity.xy()*(deltaxy.normalized());
            ftype delaySec = 0.001f * (ftype)((double)dataDelayed.delay_ms + (double)imuDataDelayed.time_ms - (double)dataDelayed.time_ms);
            delaySec = constrain_ftype(delaySec, -1.0f, 1.0f);
            const ftype rangeCorrection = delaySec * rangeRate;
            rngBcn.correctedSlantRange[dataDelayed.beacon_ID] = dataDelayed.rng + rangeCorrection;
            // calculate an equivalent horizontal range assuming vehicle and beacon height is accurate
            ftype RHsq = sq(rngBcn.correctedSlantRange[dataDelayed.beacon_ID])-sq(dataDelayed.beacon_posNED.z - stateStruct.position.z);
            rngBcn.horizontalRange[dataDelayed.beacon_ID] = sqrtF(MAX(RHsq, 0.0f));
            new_data_mask |= (1<<i);
        }

        // update covariance prediction for vertical position drift
        rngBcn.verticalOffsetVariance += sq(frontend->rngToLocHgtOfsDriftRate * dtEkfAvg);
        rngBcn.verticalOffsetVariance = MAX(rngBcn.verticalOffsetVariance, 0.0f);

        if (new_data_mask != 0) {
            // a reset using a single observation set is a last resort so wait for the slow timeout on the primary position sensors
            bool noPositionFix = (imuSampleTime_ms - lastGpsPosPassTime_ms > frontend->altPosSwitchTimeout_ms);
            if (noPositionFix && ((imuSampleTime_ms - rngBcn.lastPassTime_ms) > frontend->altPosSwitchTimeout_ms)) {
                if (ResetPosToRngBcn()) {
                    rngBcn.lastPassTime_ms = imuSampleTime_ms;
                }
            } else {
                FuseLowElevationRngBcns(new_data_mask);
            }
        } else if (rngBcn.resetIterCount > 0) {
            // not yet completed a positiion reset using a Gauss-Newton least squares solution for 4 or more beacons
            if (ResetPosToRngBcn()) {
                rngBcn.lastPassTime_ms = imuSampleTime_ms;
            }
        }
        return;
    }
#endif  // EK3_FEATURE_WRITE_RANGE_TO_LOCATION

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

bool NavEKF3_core::ResetPosToRngBcn()
{
    // find beacons that can be used to set position
    ftype correctedHorizRng[AP_BEACON_MAX_BEACONS];
    static rng_bcn_elements dataForReset[AP_BEACON_MAX_BEACONS];

    if (rngBcn.resetIterCount == 0) {
        rngBcn.posResetTime_ms = imuDataDelayed.time_ms;
        rngBcn.posResetNumBcns = 0;
        for (uint8_t index=0; index<AP_BEACON_MAX_BEACONS; index++) {
            // predict range measurement to current time using delay and range rate
            const Vector3F deltaPosNED = stateStruct.position - rngBcn.dataLast[index].beacon_posNED;
            const ftype rangeRate = stateStruct.velocity.xy()*(deltaPosNED.xy().normalized());
            ftype delaySec = 0.001f * (ftype)((double)rngBcn.dataLast[index].delay_ms + (double)imuDataDelayed.time_ms - (double)rngBcn.dataLast[index].time_ms);
            if (fabsF(delaySec) < 2.0f) {
                // limit time separation between measurements or else velocity errors due to turns become too large.
                const ftype correctedSlantRng = rngBcn.dataLast[index].rng + rangeRate * delaySec;
                // calculate an equivalent horizontal range assuming vehicle and beacon height is accurate
                ftype RHsq = sq(correctedSlantRng)-sq(rngBcn.dataLast[index].beacon_posNED.z - stateStruct.position.z);
                if (RHsq > sq(rngBcn.dataLast[index].rngErr)) {
                    correctedHorizRng[rngBcn.posResetNumBcns] = sqrtF(RHsq);
                    dataForReset[rngBcn.posResetNumBcns] = rngBcn.dataLast[index];
                    dataForReset[rngBcn.posResetNumBcns].rng = correctedSlantRng;
                    rngBcn.posResetNumBcns++;
                } else if (rngBcn.dataLast[index].rng < rngBcn.dataLast[index].rngErr) {
                    // we are very close to a beacon so set the vehicle horizontal position to match the beacon
                    ResetPositionNE(rngBcn.dataLast[index].beacon_posNED.x, rngBcn.dataLast[index].beacon_posNED.y);
                    rngBcn.posResetNumBcns = 0;
                    return true;
                }
            }
        }
    }

    bool ret = false;
    bool reachedCountLimit = false;

    switch (rngBcn.posResetNumBcns) {
    case 0:
        break;
    case 1:
        {
            const ftype bearing = atan2F(stateStruct.position.y - dataForReset[0].beacon_posNED.y, stateStruct.position.x - dataForReset[0].beacon_posNED.x);
            rngBcn.posResetNED.xy() = dataForReset[0].beacon_posNED.xy();
            rngBcn.posResetNED.xy() += Vector2F(correctedHorizRng[0] * cosF(bearing), correctedHorizRng[0] * sinF(bearing));
            ResetPositionNE(rngBcn.posResetNED.x, rngBcn.posResetNED.y);
            ret = true;
        }
        break;
    case 2:
        {
            // find the intersection
            const ftype &DR0 = dataForReset[0].rngErr;
            const ftype &DR1 = dataForReset[1].rngErr;
            if (!is_positive(dataForReset[0].rng - DR0) || !is_positive(dataForReset[1].rng - DR1)) {
                break;
            }
            if (DualRangeIntersectNE(rngBcn.posResetNED.xy(), dataForReset[0].rng, dataForReset[1].rng, dataForReset[0].beacon_posNED, dataForReset[1].beacon_posNED, stateStruct.position.z)) {
                ResetPositionNE(rngBcn.posResetNED.x,rngBcn.posResetNED.y);
                ret = true;
            }
        }
        break;
    // case 3: // This case can also be handled by the least squares method
    //     {
    //         // do trilateration
    //         const Vector3F p1 = dataForReset[0].beacon_posNED;
    //         const Vector3F p2 = dataForReset[1].beacon_posNED;
    //         const Vector3F p3 = dataForReset[2].beacon_posNED;
    //         const ftype r1 = dataForReset[0].rng;
    //         const ftype r2 = dataForReset[1].rng;
    //         const ftype r3 = dataForReset[2].rng;

    //         // Calculate vectors
    //         Vector3F ex = Vector3F(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    //         ftype d = ex.x * ex.x + ex.y * ex.y + ex.z * ex.z;
    //         if (!is_positive(d)) {
    //             break;
    //         }
    //         d = sqrtF(d);
    //         ex.x /= d; ex.y /= d; ex.z /= d;

    //         Vector3F temp = Vector3F(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    //         ftype i = ex.x * temp.x + ex.y * temp.y + ex.z * temp.z;

    //         Vector3F ey = Vector3F(temp.x - i * ex.x, temp.y - i * ex.y, temp.z - i * ex.z);
    //         ftype j = ey.x * ey.x + ey.y * ey.y + ey.z * ey.z;
    //         if (!is_positive(j)) {
    //             break;
    //         }
    //         j = sqrtF(j);
    //         ey.x /= j; ey.y /= j; ey.z /= j;

    //         Vector3F ez = Vector3F(ex.y * ey.z - ex.z * ey.y, ex.z * ey.x - ex.x * ey.z, ex.x * ey.y - ex.y * ey.x);

    //         ftype x = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    //         ftype y = (r1 * r1 - r3 * r3 + i * i + j * j) / (2 * j) - (i / j) * x;
    //         ftype z = r1 * r1 - x * x - y * y;
    //         if (!is_positive(z)) {
    //             break;
    //         }
    //         z = sqrtF(z);

    //         // There are two possible solutions for z
    //         Vector3F result1 = Vector3F(p1.x + x * ex.x + y * ey.x + z * ez.x,
    //                         p1.y + x * ex.y + y * ey.y + z * ez.y,
    //                         p1.z + x * ex.z + y * ey.z + z * ez.z);

    //         Vector3F result2 = Vector3F(p1.x + x * ex.x + y * ey.x - z * ez.x,
    //                         p1.y + x * ex.y + y * ey.y - z * ez.y,
    //                         p1.z + x * ex.z + y * ey.z - z * ez.z);

    //         // Use the solution above the beacons (up is negative z)
    //         if (result1.z < result2.z) {
    //             rngBcn.posResetNED.xy() = result1.xy();
    //         } else {
    //             rngBcn.posResetNED.xy() = result2.xy();
    //         }

    //         ResetPositionNE(rngBcn.posResetNED.x,rngBcn.posResetNED.y);
    //         ret = true;
    //     }
    //     break;
    default: // Iterative Gauss Newton least squares method
        {
            const uint8_t Nmeas = rngBcn.posResetNumBcns;

            if (rngBcn.resetIterCount == 0) {
                // start in middle of beacon pattern and 1000m above the highest beacon.
                ftype xmin =  EK3_POSXY_STATE_LIMIT;
                ftype xmax = -EK3_POSXY_STATE_LIMIT;
                ftype ymin =  EK3_POSXY_STATE_LIMIT;
                ftype ymax = -EK3_POSXY_STATE_LIMIT;
                ftype zmin =  1.0e4f;
                for (int i = 0; i < Nmeas; i++) {
                    const auto &bcnPosNED = dataForReset[i].beacon_posNED;
                    if (bcnPosNED.x < xmin) {
                        xmin = bcnPosNED.x;
                    }
                    if (bcnPosNED.x > xmax) {
                        xmax = bcnPosNED.x;
                    }
                    if (bcnPosNED.y < ymin) {
                        ymin = bcnPosNED.y;
                    }
                    if (bcnPosNED.y > ymax) {
                        ymax = bcnPosNED.y;
                    }
                    if (bcnPosNED.z < zmin) {
                        zmin = bcnPosNED.z;
                    }
                }
                rngBcn.posResetNED = Vector3F{0.5F*(xmin+xmax),0.5F*(ymin+ymax),zmin-1000.0F};
            }

            rngBcn.resetIterCount++;

            // Evaluate Jacobian and residual vector
            ftype Jac[Nmeas][3];
            ftype res_vec[AP_BEACON_MAX_BEACONS]; // sized at max possible value for logging purposes
            for (int i = 0; i < Nmeas; i++) {
                const ftype deltaPosN = rngBcn.posResetNED.x - dataForReset[i].beacon_posNED.x;
                const ftype deltaPosE = rngBcn.posResetNED.y - dataForReset[i].beacon_posNED.y;
                const ftype deltaPosD = rngBcn.posResetNED.z - dataForReset[i].beacon_posNED.z;

                const ftype rngPred = sqrtF(sq(deltaPosN) + sq(deltaPosE) + sq(deltaPosD));

                if (rngPred < 1.0F) {
                    rngBcn.resetIterCount = 0;
                    return false;
                }

                Jac[i][0] = deltaPosN / rngPred;
                Jac[i][1] = deltaPosE / rngPred;
                Jac[i][2] = deltaPosD / rngPred;

                res_vec[i] = rngPred - dataForReset[i].rng;
            }

            ftype JacTrans[3][Nmeas];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < Nmeas; j++){
                    JacTrans[i][j] = Jac[j][i];
                }
            }

            Matrix3F JacTransJac;
            for (int i = 0; i<3; i++) {
                for (int j = 0; j<3; j++) {
                    JacTransJac[i][j] = 0;
                    for (int k = 0; k<Nmeas; k++) {
                        JacTransJac[i][j] += JacTrans[i][k]*Jac[k][j];
                    }
                }
            }

            Matrix3F JacTransJacInv;
            bool hasInverse = JacTransJac.inverse(JacTransJacInv);

            if(hasInverse) {
                ftype JacTransRes[3];
                for (int i = 0; i < 3; i++) {
                    JacTransRes[i] = 0;
                    for (int j = 0; j < Nmeas; j++) {
                        JacTransRes[i] += JacTrans[i][j]*res_vec[j];
                    }
                }

                ftype correction[3];
                for (int i = 0; i < 3; i++) {
                    correction[i] = 0;
                    for (int j = 0; j < 3; j++) {
                        correction[i] += JacTransJacInv[i][j] * JacTransRes[j];
                    }
                }

                const ftype tuningGain = 0.7f; // adjust if stability issues are encountered
                rngBcn.posResetNED.x -= correction[0] * tuningGain;
                rngBcn.posResetNED.y -= correction[1] * tuningGain;
                rngBcn.posResetNED.z -= correction[2] * tuningGain;

                // calculate the maximum residual normalised wrt the measurement uncertainty
                ftype sumOfSquares = 0.0F;
                for (int i = 0; i < Nmeas; i++) {
                    sumOfSquares += sq(res_vec[i] / MAX(dataForReset[i].rngErr, 0.1f));
                }

                // continue iterating until RSS residual is small. If run out of iterations don't accept result if > 3 sigma
                reachedCountLimit = rngBcn.resetIterCount >= 20;
                const ftype maxRatio = reachedCountLimit ? 3.0F : 0.5F;
                if (sqrtF(sumOfSquares) / (ftype)Nmeas < maxRatio) {
                    ftype delaySec = 0.001f * (ftype)((double)imuDataDelayed.time_ms - (double)rngBcn.posResetTime_ms);
                    rngBcn.posResetNED += stateStruct.velocity * delaySec;
                    ResetPositionNE(rngBcn.posResetNED.x,rngBcn.posResetNED.y);
                    ret = true;
                }
                AP::logger().WriteStreaming("DBG3",
                                            "TimeUS,C,Error,Count",  // labels
                                            "s#--",    // units
                                            "F---",    // multss
                                            "QBfB",    // fmt
                                            dal.micros64(),
                                            DAL_CORE(core_index),
                                            sqrtF(sumOfSquares) / (ftype)Nmeas,
                                            rngBcn.resetIterCount
                                            );

            } else {
                // badly conditioned so re-start with new data
                reachedCountLimit = true;
            }


        }

        break;
    }

    if (ret) {
        Location loc;
        loc.lat = EKF_origin.lat;
        loc.lng = EKF_origin.lng;
        loc.alt = EKF_origin.alt;
        loc.offset(stateStruct.position.x,stateStruct.position.y);
        loc.alt -= (int32_t)(100.0f * stateStruct.position.z);
        AP::logger().WriteStreaming("BCRS",  // BeaCon ReSet
                                    "TimeUS,C,Lat,Lng,N",  // labels
                                    "s#DU-",    // units
                                    "F----",    // mults
                                    "QBLLB",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(core_index),
                                    loc.lat,
                                    loc.lng,
                                    rngBcn.posResetNumBcns
                                    );
    }

    if (ret || reachedCountLimit) {
        rngBcn.resetIterCount = 0;
    }

    return ret;
}

// PosNE is the NE local position defined by the intersection of slant ranges from two NED locations to the specified vertical position PosD
// return true if solution found
bool NavEKF3_core::DualRangeIntersectNE(Vector2F &PosNE, const ftype R0, const ftype R1, const Vector3F &P0, const Vector3F &P1, const ftype PosD)
{
    // correct slant ranges for vertical position offset
    ftype RH0sq = sq(R0)-sq(P0.z - PosD);
    if (RH0sq < 0.5f * sq(R0)) {
        // elevation angle is too high
        return false;
    }
    ftype RH0 =sqrtF(RH0sq); // horizontal range from beacon 0

    ftype RH1sq = sq(R1)-sq(P1.z - PosD);
    if (RH1sq < 0.5f * sq(R1)) {
        // elevation angle is too high
        return false;
    }
    ftype RH1 = sqrtF(RH1sq); // horizontal range from beacon 1

    // Calculate the distance between the beacons
    ftype dx = P1.x - P0.x;
    ftype dy = P1.y - P0.y;
    ftype dist = sqrtF(dx * dx + dy * dy);
    if (!is_positive(dist)) {
        return false;
    }

    // Check if there are no solutions
    if (dist > (RH0 + RH1) || dist < fabsF(RH0 - RH1)) {
        // No intersection
        return false;
    }

    // Calculate the point of intersection of the line through the circle intersection points
    ftype a = (sq(RH0) - sq(RH1) + sq(dist)) / (2.0f * dist);
    ftype h = sqrtF(RH0 * RH0 - a * a);
    ftype x2 = P0.x + a * (dx / dist);
    ftype y2 = P0.y + a * (dy / dist);

    // Calculate the intersection points
    double rx = -dy * (h / dist);
    double ry =  dx * (h / dist);
    const Vector2F intersection1{x2 + rx, y2 + ry};
    const Vector2F intersection2{x2 - rx, y2 - ry};

    // return the solution closest to the vehicle position
    if ((intersection1-stateStruct.position.xy()).length_squared() < (intersection2-stateStruct.position.xy()).length_squared()) {
        PosNE = intersection1;
    } else {
        PosNE = intersection2;
    }

    return true;
}

/*
  add the range beacon data delayed to the fusion report array, allocating if needed
 */
void NavEKF3_core::PushRngBcn(const rng_bcn_elements &data)
{
    if (data.beacon_ID >= rngBcn.fusionReport_length) {
        auto *old_reports = rngBcn.fusionReport;
        auto *new_reports = new BeaconFusion::FusionReport[data.beacon_ID+1];
        if (new_reports == nullptr) {
            return;
        }
        memcpy(new_reports, old_reports, rngBcn.fusionReport_length * sizeof(BeaconFusion::FusionReport));
        rngBcn.fusionReport = new_reports;
        rngBcn.fusionReport_length = data.beacon_ID+1;
        delete[] old_reports;
    }
    auto &report = rngBcn.fusionReport[data.beacon_ID];
    report.beaconPosNED = data.beacon_posNED;
    report.innov = rngBcn.innov;
    report.innovVar = rngBcn.varInnov;
    report.rng = data.rng;
    report.testRatio = rngBcn.testRatio;
#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    report.vertOffset = rngBcn.verticalOffset;
#endif
    rngBcn.newDataToLog[data.beacon_ID] = true;
}

void NavEKF3_core::FuseRngBcn()
{
    for (uint8_t i=0; i<rngBcn.N; i++) {
        FuseRngBcn(rngBcn.dataDelayed[i]);
    }
}

// FuseLowElevationRngBcns - fuse rangebeacons which are above a
// particular elevation and look good in terms of range errors:
void NavEKF3_core::FuseLowElevationRngBcns(uint32_t new_data_mask)
{
    for (uint8_t i=0; i<rngBcn.N; i++) {
        if ((new_data_mask & (1U<<i)) == 0) {
            continue;
        }
        auto dataDelayed = rngBcn.dataDelayed[i];
        const auto beacon_ID = dataDelayed.beacon_ID;
        if (rngBcn.correctedSlantRange[beacon_ID] <= 3.0F * MAX(dataDelayed.rngErr , 0.1f)) {
            // bounce - too high an error proportion
            continue;
        }
        if (rngBcn.horizontalRange[beacon_ID] <= cosF(radians(45.0f)) * rngBcn.correctedSlantRange[beacon_ID]) {
            // bounce - bad angle
            continue;
        }
        FuseRngBcn(rngBcn.dataDelayed[i]);
    }
}

void NavEKF3_core::FuseRngBcn(rng_bcn_elements &data)
{
    // health is set bad until test passed
    rngBcn.health = false;

    const ftype R_BCN = sq(MAX(data.rngErr , 0.1f));

#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    if (rngBcn.usingRangeToLoc) {
        data.beacon_posNED = EKF_origin.get_distance_NED_ftype(data.beacon_loc);
        // do not offset the beacon positions because they are absolute
        rngBcn.posOffsetNED.zero();
    } else
#endif  // EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    if (activeHgtSource != AP_NavEKF_Source::SourceZ::BEACON) {
        // calculate the vertical offset from EKF datum to beacon datum rngBcn.posOffsetNED.z
        CalcRangeBeaconPosDownOffset(data, R_BCN, stateStruct.position, false);
    } else {
        // the beacons can be offset horizontally, but the vehicle vertical position
        // datum must match the beacon vertical position datum
        rngBcn.posOffsetNED.z = 0.0f;
    }

    // predicted range accounting for vehicle and beacon vertical position offset
    Vector3F deltaPosNED = data.beacon_posNED - stateStruct.position;
    deltaPosNED.z += rngBcn.posOffsetNED.z;
#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    deltaPosNED.z -= rngBcn.verticalOffset;
#endif
    const ftype rngPred = deltaPosNED.length();

    // calculate measurement innovation
    rngBcn.innov = rngPred - rngBcn.correctedSlantRange[data.beacon_ID];

    // perform fusion of range measurement
    if (rngPred > 0.1f)
    {
        // calculate observation jacobians
        Vector24 H_BCN;
        memset(H_BCN, 0, sizeof(H_BCN));
        // If we are not using the beacons as a height reference, we pretend that the beacons
        // are at the same height as the flight vehicle when calculating the observation derivatives
        // and Kalman gains
        // TODO  - less hacky way of achieving this, preferably using an alternative derivation
        const ftype t2 = (activeHgtSource == AP_NavEKF_Source::SourceZ::BEACON) ? deltaPosNED.z : 0.0f;
        const ftype t3 = deltaPosNED.y;
        const ftype t4 = deltaPosNED.x;
        const ftype t5 = t2*t2;
        const ftype t6 = t3*t3;
        const ftype t7 = t4*t4;
        const ftype t8 = t5+t6+t7;
        const ftype t9 = 1.0f/sqrtF(t8);
        H_BCN[7] = -t4*t9;
        H_BCN[8] = -t3*t9;
        H_BCN[9] = -t2*t9;

        // calculate Kalman gains
        const ftype t10 = P[9][9]*t2*t9;
        const ftype t11 = P[8][9]*t3*t9;
        const ftype t12 = P[7][9]*t4*t9;
        const ftype t13 = t10+t11+t12;
        const ftype t14 = t2*t9*t13;
        const ftype t15 = P[9][8]*t2*t9;
        const ftype t16 = P[8][8]*t3*t9;
        const ftype t17 = P[7][8]*t4*t9;
        const ftype t18 = t15+t16+t17;
        const ftype t19 = t3*t9*t18;
        const ftype t20 = P[9][7]*t2*t9;
        const ftype t21 = P[8][7]*t3*t9;
        const ftype t22 = P[7][7]*t4*t9;
        const ftype t23 = t20+t21+t22;
        const ftype t24 = t4*t9*t23;
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

        if (!rngBcn.notUsedForAttitude) {
            Kfusion[0] = -t26*(P[0][7]*t4*t9+P[0][8]*t3*t9+P[0][9]*t2*t9);
            Kfusion[1] = -t26*(P[1][7]*t4*t9+P[1][8]*t3*t9+P[1][9]*t2*t9);
            Kfusion[2] = -t26*(P[2][7]*t4*t9+P[2][8]*t3*t9+P[2][9]*t2*t9);
            Kfusion[3] = -t26*(P[3][7]*t4*t9+P[3][8]*t3*t9+P[3][9]*t2*t9);
        } else {
            // zero indexes 0 to 3
            zero_range(&Kfusion[0], 0, 3);
        }
        Kfusion[4] = -t26*(P[4][7]*t4*t9+P[4][8]*t3*t9+P[4][9]*t2*t9);
        Kfusion[5] = -t26*(P[5][7]*t4*t9+P[5][8]*t3*t9+P[5][9]*t2*t9);
        Kfusion[7] = -t26*(t22+P[7][8]*t3*t9+P[7][9]*t2*t9);
        Kfusion[8] = -t26*(t16+P[8][7]*t4*t9+P[8][9]*t2*t9);

        if (!inhibitDelAngBiasStates && !rngBcn.notUsedForAttitude) {
            Kfusion[10] = -t26*(P[10][7]*t4*t9+P[10][8]*t3*t9+P[10][9]*t2*t9);
            Kfusion[11] = -t26*(P[11][7]*t4*t9+P[11][8]*t3*t9+P[11][9]*t2*t9);
            Kfusion[12] = -t26*(P[12][7]*t4*t9+P[12][8]*t3*t9+P[12][9]*t2*t9);
        } else {
            // zero indexes 10 to 12
            zero_range(&Kfusion[0], 10, 12);
        }

        if (!inhibitDelVelBiasStates && !badIMUdata && !rngBcn.notUsedForAttitude) {
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
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::BEACON && !rngBcn.notUsedForAttitude) {
            Kfusion[6] = -t26*(P[6][7]*t4*t9+P[6][8]*t3*t9+P[6][9]*t2*t9);
            Kfusion[9] = -t26*(t10+P[9][7]*t4*t9+P[9][8]*t3*t9);
        } else {
            Kfusion[6] = 0.0f;
            Kfusion[9] = 0.0f;
        }

        if (!inhibitMagStates && !rngBcn.notUsedForAttitude) {
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

        // calculate the innovation consistency test ratio
        rngBcn.testRatio = sq(rngBcn.innov) / (sq(MAX(0.01f * (ftype)frontend->_rngBcnInnovGate, 1.0f)) * rngBcn.varInnov);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        rngBcn.health = ((rngBcn.testRatio < 1.0f) || badIMUdata);

        // don't fuse unless preferred navigation source has timed out to prevent these measurements fighting the preferred source
        const uint32_t use_delay_ms = velAiding ? frontend->altPosSwitchTimeout_ms : frontend->deadReckonDeclare_ms;
        const bool primary_aiding_lost = (imuSampleTime_ms - lastGpsPosPassTime_ms > use_delay_ms);
        if (rngBcn.health && primary_aiding_lost) {

            // restart the counter
            rngBcn.lastPassTime_ms = imuSampleTime_ms;

            // Efficient implementation of the Joseph stabilised covariance update
            // Based on "G. J. Bierman. Factorization Methods for Discrete Sequential Estimation. Academic Press, Dover Publications, New York, 1977, 2006"
            // P = (I - K * H) * P * (I - K * H).T   + K * R * K.T
            //   =      P_temp     * (I - H.T * K.T) + K * R * K.T
            //   =      P_temp - P_temp * H.T * K.T  + K * R * K.T

            // Step 1: conventional update
            // Compute P_temp and store it in P to avoid allocating more memory
            // P is symmetric, so PH == H.T * P.T == H.T * P. Taking the row is faster as matrices are row-major
            // PH = P * H_BCN where H is stored as a column vector. H is in fact H.T
            Vector24 PH{};
            for (uint8_t row = 0; row < 24; row++) {
                for (uint8_t col = 0; col < 24; col++) {
                    PH[row] += P[row][col] * H_BCN[col];
                }
            }

            for (uint8_t row = 0; row < 24; row++) {
                for (uint8_t col = 0; col < 24; col++) {
                    P[row][col] -= Kfusion[row] * PH[col]; // P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
                }
            }

            // Step 2: stabilized update
            // PH = P * H_BCN where H is stored as a column vector. H is in fact H.T
            for (uint8_t row = 0; row < 24; row++) {
                for (uint8_t col = 0; col < 24; col++) {
                    PH[row] += P[row][col] * H_BCN[col];
                }
            }

            for (uint8_t row = 0; row < 24; row++) {
                for (unsigned col = 0; col <= row; col++) {
                    P[row][col] = P[row][col] - PH[row] * Kfusion[col] + Kfusion[row] * R_BCN * Kfusion[col];
                    P[col][row] = P[row][col];
                }
            }

            // limit the variances to prevent ill-conditioning
            // symmetry is already guaranteed by the Joseph stabilised covariance update
            ConstrainVariances();

            // correct the state vector
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                statesArray[j] = statesArray[j] - Kfusion[j] * rngBcn.innov;
            }

#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
            // In this mode we are using range to beacons that are located at an absolute position
            // but the vehicle height source will be something else, eg baro, so the height inconsistency
            // needs to be handled. This is achieved by estimating an offset that is added to the vehicle
            // vertical position before it is used to calculate the predicted range measurement.
            if ((frontend->_options & (int32_t)NavEKF3::Options::RangeToLocHgtOffset) && rngBcn.usingRangeToLoc) {
                ftype rngPredInv = 1.0f/rngPred;
                const ftype offsetH = - deltaPosNED.z*rngPredInv; // observation Jacobian
                const ftype offsetPH = rngBcn.verticalOffsetVariance * offsetH;
                const ftype offsetHPH = offsetH * offsetPH;
                const ftype offsetS = R_BCN + offsetHPH; //  innovation variance
                if (offsetS >= R_BCN) {
                    // Kalman gain K = (P . H') / S
                    ftype offsetK = offsetPH / offsetS;
                    rngBcn.verticalOffset -= offsetK * rngBcn.innov;
                    // covariance update P = P - K.H.P
                    rngBcn.verticalOffsetVariance -= offsetK * offsetPH;
                }
            } else {
                rngBcn.verticalOffset = 0.0f;
            }
#endif  // EK3_FEATURE_WRITE_RANGE_TO_LOCATION

            // record healthy fusion
            faultStatus.bad_rngbcn = false;
        }

#if EK3_FEATURE_WRITE_RANGE_TO_LOCATION
        // Update the fusion report
        if (rngBcn.usingRangeToLoc) {
            PushRngBcn(data);
        }                
#endif  // EK3_FEATURE_WRITE_RANGE_TO_LOCATION
    }
}

/*
Use range beacon measurements to calculate a static position using a 3-state EKF algorithm.
Algorithm based on the following:
https://github.com/priseborough/InertialNav/blob/master/derivations/range_beacon.m
*/
void NavEKF3_core::FuseRngBcnStatic()
{
    for (uint8_t i=0; i<rngBcn.N; i++) {
        FuseRngBcnStatic(rngBcn.dataDelayed[i]);
    }
}

void NavEKF3_core::FuseRngBcnStatic(rng_bcn_elements &dataDelayed)
{
    // get the estimated range measurement variance
    const ftype R_RNG = sq(MAX(dataDelayed.rngErr , 0.1f));

    /*
    The first thing to do is to check if we have started the alignment and if not, initialise the
    states and covariance to a first guess. To do this iterate through the available beacons and then
    initialise the initial position to the mean beacon position. The initial position uncertainty
    is set to the mean range measurement.
    */
    if (!rngBcn.alignmentStarted) {
        if (dataDelayed.beacon_ID != rngBcn.lastIndex) {
            rngBcn.posSum += dataDelayed.beacon_posNED;
            rngBcn.lastIndex = dataDelayed.beacon_ID;
            rngBcn.sum += dataDelayed.rng;
            rngBcn.numMeas++;

            // capture the beacon vertical spread
            if (dataDelayed.beacon_posNED.z > rngBcn.maxPosD) {
                rngBcn.maxPosD = dataDelayed.beacon_posNED.z;
            } else if(dataDelayed.beacon_posNED.z < rngBcn.minPosD) {
                rngBcn.minPosD = dataDelayed.beacon_posNED.z;
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
                CalcRangeBeaconPosDownOffset(dataDelayed, R_RNG, refPosNED, true);

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
        ftype t2 = dataDelayed.beacon_posNED.z - rngBcn.receiverPos.z + rngBcn.posOffsetNED.z;
        ftype t3 = dataDelayed.beacon_posNED.y - rngBcn.receiverPos.y;
        ftype t4 = dataDelayed.beacon_posNED.x - rngBcn.receiverPos.x;
        ftype t5 = t2*t2;
        ftype t6 = t3*t3;
        ftype t7 = t4*t4;
        ftype t8 = t5+t6+t7;
        if (t8 < 0.1f) {
            // calculation will be badly conditioned
            return;
        }
        ftype t9 = 1.0f/sqrtF(t8);
        ftype t10 = dataDelayed.beacon_posNED.x*2.0f;
        ftype t15 = rngBcn.receiverPos.x*2.0f;
        ftype t11 = t10-t15;
        ftype t12 = dataDelayed.beacon_posNED.y*2.0f;
        ftype t14 = rngBcn.receiverPos.y*2.0f;
        ftype t13 = t12-t14;
        ftype t16 = dataDelayed.beacon_posNED.z*2.0f;
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
        Vector3F deltaPosNED = rngBcn.receiverPos - dataDelayed.beacon_posNED;
        deltaPosNED.z -= rngBcn.posOffsetNED.z;
        rngBcn.innov = deltaPosNED.length() - dataDelayed.rng;

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
        PushRngBcn(dataDelayed);
    }
}

/*
Run a single state Kalman filter to estimate the vertical position offset of the range beacon constellation
Calculate using a high and low hypothesis and select the hypothesis with the lowest innovation sequence
*/
void NavEKF3_core::CalcRangeBeaconPosDownOffset(rng_bcn_elements &dataDelayed, ftype obsVar, Vector3F &vehiclePosNED, bool aligning)
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
    ftype t2 = dataDelayed.beacon_posNED.z - vehiclePosNED.z + rngBcn.posDownOffsetMax;
    ftype t3 = dataDelayed.beacon_posNED.y - vehiclePosNED.y;
    ftype t4 = dataDelayed.beacon_posNED.x - vehiclePosNED.x;
    ftype t5 = t2*t2;
    ftype t6 = t3*t3;
    ftype t7 = t4*t4;
    ftype t8 = t5+t6+t7;
    ftype t9;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - dataDelayed.rng;

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
    t2 = dataDelayed.beacon_posNED.z - vehiclePosNED.z + rngBcn.posDownOffsetMin;
    t5 = t2*t2;
    t8 = t5+t6+t7;
    if (t8 > 0.1f) {
        t9 = 1.0f/sqrtF(t8);
        obsDeriv = t2*t9;

        // Calculate innovation
        innov = sqrtF(t8) - dataDelayed.rng;

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
    dataDelayed.beacon_posNED.z += rngBcn.posOffsetNED.z;
}

#endif  // EK3_FEATURE_BEACON_FUSION
