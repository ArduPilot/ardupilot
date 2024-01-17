#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_DAL/AP_DAL.h>

#pragma GCC diagnostic ignored "-Wnarrowing"

void NavEKF3_core::Log_Write_XKF1(uint64_t time_us) const
{
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    getEulerAngles(euler);
    getVelNED(velNED);
    getPosNE(posNE);
    getPosD(posD);
    getGyroBias(gyroBias);
    posDownDeriv = getPosDownDerivative();
    if (!getOriginLLH(originLLH)) {
        originLLH.alt = 0;
    }
    const struct log_XKF1 pkt{
        LOG_PACKET_HEADER_INIT(LOG_XKF1_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void NavEKF3_core::Log_Write_XKF2(uint64_t time_us) const
{
    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    getAccelBias(accelBias);
    getWind(wind);
    getMagNED(magNED);
    getMagXYZ(magXYZ);
    Vector2f dragInnov;
    float betaInnov = 0;
    getSynthAirDataInnovations(dragInnov, betaInnov);
    const struct log_XKF2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_XKF2_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        accBiasX  : (int16_t)(100*accelBias.x),
        accBiasY  : (int16_t)(100*accelBias.y),
        accBiasZ  : (int16_t)(100*accelBias.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        innovDragX    : dragInnov.x,
        innovDragY    : dragInnov.y,
        innovSideslip : betaInnov
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void NavEKF3_core::Log_Write_XKFS(uint64_t time_us) const
{
    // Write sensor selection EKF packet
    const struct log_XKFS pkt {
        LOG_PACKET_HEADER_INIT(LOG_XKFS_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        mag_index      : magSelectIndex,
        baro_index     : selected_baro,
        gps_index      : selected_gps,
        airspeed_index : getActiveAirspeed(),
        source_set     : frontend->sources.getPosVelYawSourceSet()
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void NavEKF3_core::Log_Write_XKF3(uint64_t time_us) const
{
    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    const struct log_XKF3 pkt3{
        LOG_PACKET_HEADER_INIT(LOG_XKF3_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov),
        rerr : frontend->coreRelativeErrors[core_index],
        errorScore : frontend->coreErrorScores[core_index]
    };
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}

void NavEKF3_core::Log_Write_XKF4(uint64_t time_us) const
{
    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    uint16_t _faultStatus=0;
    Vector2f offset;
    const uint8_t timeoutStatus =
        posTimeout<<0 |
        velTimeout<<1 |
        hgtTimeout<<2 |
        magTimeout<<3 |
        tasTimeout<<4 |
        dragTimeout<<5;

    nav_filter_status solutionStatus {};
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxF(fmaxF(magVar.x,magVar.y),magVar.z);
    getFilterFaults(_faultStatus);
    getFilterStatus(solutionStatus);
    const struct log_XKF4 pkt4{
        LOG_PACKET_HEADER_INIT(LOG_XKF4_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : sqrtF(MAX(tiltErrorVariance,0.0f)),  // estimated 1-sigma tilt error in radians
        offsetNorth : offset.x,
        offsetEast : offset.y,
        faults : _faultStatus,
        timeouts : timeoutStatus,
        solution : solutionStatus.value,
        gps : gpsCheckStatus.value,
        primary : frontend->getPrimaryCoreIndex()
    };
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}


void NavEKF3_core::Log_Write_XKF5(uint64_t time_us) const
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    const struct log_XKF5 pkt5{
        LOG_PACKET_HEADER_INIT(LOG_XKF5_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        normInnov : (uint8_t)(MIN(100*MAX(flowTestRatio[0],flowTestRatio[1]),255)),  // normalised innovation variance ratio for optical flow observations fused by the main nav filter
        FIX : (int16_t)(1000*flowInnov[0]),  // optical flow LOS rate vector innovations from the main nav filter
        FIY : (int16_t)(1000*flowInnov[1]),  // optical flow LOS rate vector innovations from the main nav filter
        AFI : (int16_t)(1000 * auxFlowObsInnov.length()),  // optical flow LOS rate innovation from terrain offset estimator
        HAGL : float_to_int16(100*(terrainState - stateStruct.position.z)),    // height above ground level
        offset : (int16_t)(100*terrainState),           // filter ground offset state error
        RI : (int16_t)(100*innovRng),                   // range finder innovations
        meaRng : (uint16_t)(100*rangeDataDelayed.rng),  // measured range
        errHAGL : (uint16_t)(100*sqrtF(Popt)),          // note Popt is constrained to be non-negative in EstimateTerrainOffset()
        angErr : (float)outputTrackError.x,             // output predictor angle error
        velErr : (float)outputTrackError.y,             // output predictor velocity error
        posErr : (float)outputTrackError.z              // output predictor position tracking error
    };
    AP::logger().WriteBlock(&pkt5, sizeof(pkt5));
}

void NavEKF3_core::Log_Write_Quaternion(uint64_t time_us) const
{
    // log quaternion
    Quaternion quat;
    getQuaternion( quat);
    const struct log_XKQ pktq1{
        LOG_PACKET_HEADER_INIT(LOG_XKQ_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    AP::logger().WriteBlock(&pktq1, sizeof(pktq1));
}

#if EK3_FEATURE_BEACON_FUSION
// logs beacon information, one beacon per call
void NavEKF3_core::Log_Write_Beacon(uint64_t time_us)
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    if (!statesInitialised || rngBcn.N == 0 || rngBcn.fusionReport == nullptr) {
        return;
    }

    // Ensure that beacons are not skipped due to calling this function at a rate lower than the updates
    if (rngBcn.fuseDataReportIndex >= rngBcn.N) {
        rngBcn.fuseDataReportIndex = 0;
    }

    const auto &report = rngBcn.fusionReport[rngBcn.fuseDataReportIndex];

    // write range beacon fusion debug packet if the range value is non-zero
    if (report.rng <= 0.0f) {
        rngBcn.fuseDataReportIndex++;
        return;
    }

    const struct log_XKF0 pkt10{
        LOG_PACKET_HEADER_INIT(LOG_XKF0_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        ID : rngBcn.fuseDataReportIndex,
        rng : (int16_t)(100*report.rng),
        innov : (int16_t)(100*report.innov),
        sqrtInnovVar : (uint16_t)(100*sqrtF(report.innovVar)),
        testRatio : (uint16_t)(100*constrain_ftype(report.testRatio,0.0f,650.0f)),
        beaconPosN : (int16_t)(100*report.beaconPosNED.x),
        beaconPosE : (int16_t)(100*report.beaconPosNED.y),
        beaconPosD : (int16_t)(100*report.beaconPosNED.z),
        offsetHigh : (int16_t)(100*rngBcn.posDownOffsetMax),
        offsetLow : (int16_t)(100*rngBcn.posDownOffsetMin),
        posN : (int16_t)(100*rngBcn.receiverPos.x),
        posE : (int16_t)(100*rngBcn.receiverPos.y),
        posD : (int16_t)(100*rngBcn.receiverPos.z)
    };
    AP::logger().WriteBlock(&pkt10, sizeof(pkt10));
    rngBcn.fuseDataReportIndex++;
}
#endif  // EK3_FEATURE_BEACON_FUSION

#if EK3_FEATURE_BODY_ODOM
void NavEKF3_core::Log_Write_BodyOdom(uint64_t time_us)
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    const uint32_t updateTime_ms = MAX(bodyOdmDataDelayed.time_ms,wheelOdmDataDelayed.time_ms);
    if (updateTime_ms > lastUpdateTime_ms) {
        const struct log_XKFD pkt11{
            LOG_PACKET_HEADER_INIT(LOG_XKFD_MSG),
            time_us : time_us,
            core    : DAL_CORE(core_index),
            velInnovX : innovBodyVel[0],
            velInnovY : innovBodyVel[1],
            velInnovZ : innovBodyVel[2],
            velInnovVarX : varInnovBodyVel[0],
            velInnovVarY : varInnovBodyVel[1],
            velInnovVarZ : varInnovBodyVel[2]
         };
        AP::logger().WriteBlock(&pkt11, sizeof(pkt11));
        lastUpdateTime_ms = updateTime_ms;
    }
}
#endif

void NavEKF3_core::Log_Write_State_Variances(uint64_t time_us)
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    if (AP::dal().millis() - lastEkfStateVarLogTime_ms > 490) {
        lastEkfStateVarLogTime_ms = AP::dal().millis();
        const struct log_XKV pktv1{
            LOG_PACKET_HEADER_INIT(LOG_XKV1_MSG),
            time_us : time_us,
            core    : DAL_CORE(core_index),
            v00 : P[0][0],
            v01 : P[1][1],
            v02 : P[2][2],
            v03 : P[3][3],
            v04 : P[4][4],
            v05 : P[5][5],
            v06 : P[6][6],
            v07 : P[7][7],
            v08 : P[8][8],
            v09 : P[9][9],
            v10 : P[10][10],
            v11 : P[11][11]
        };
        AP::logger().WriteBlock(&pktv1, sizeof(pktv1));
        const struct log_XKV pktv2{
            LOG_PACKET_HEADER_INIT(LOG_XKV2_MSG),
            time_us : time_us,
            core    : DAL_CORE(core_index),
            v00 : P[12][12],
            v01 : P[13][13],
            v02 : P[14][14],
            v03 : P[15][15],
            v04 : P[16][16],
            v05 : P[17][17],
            v06 : P[18][18],
            v07 : P[19][19],
            v08 : P[20][20],
            v09 : P[21][21],
            v10 : P[22][22],
            v11 : P[23][23]
        };
        AP::logger().WriteBlock(&pktv2, sizeof(pktv2));
    }
}

void NavEKF3::Log_Write()
{
    // only log if enabled
    if (activeCores() <= 0) {
        return;
    }
    if (lastLogWrite_us == imuSampleTime_us) {
        // vehicle is doubling up on logging
        return;
    }
    lastLogWrite_us = imuSampleTime_us;

    uint64_t time_us = AP::dal().micros64();

    for (uint8_t i=0; i<activeCores(); i++) {
        core[i].Log_Write(time_us);
    }

    AP::dal().start_frame(AP_DAL::FrameType::LogWriteEKF3);
}

void NavEKF3_core::Log_Write(uint64_t time_us)
{
    const auto level = frontend->_log_level;
    if (level == NavEKF3::LogLevel::NONE) {  // no logging from EK3_LOG_LEVEL param
        return;
    }
    Log_Write_XKF4(time_us);
    if (level == NavEKF3::LogLevel::XKF4) {  // only log XKF4 scaled innovations
        return;
    }
    Log_Write_GSF(time_us);
    if (level == NavEKF3::LogLevel::XKF4_GSF) {  // only log XKF4 scaled innovations and GSF, otherwise log everything
        return;
    }
    // note that several of these functions exit-early if they're not
    // attempting to log the primary core.
    Log_Write_XKF1(time_us);
    Log_Write_XKF2(time_us);
    Log_Write_XKF3(time_us);
    Log_Write_XKF5(time_us);

    Log_Write_XKFS(time_us);
    Log_Write_Quaternion(time_us);


#if EK3_FEATURE_BEACON_FUSION
    // write range beacon fusion debug packet if the range value is non-zero
    Log_Write_Beacon(time_us);
#endif

#if EK3_FEATURE_BODY_ODOM
    // write debug data for body frame odometry fusion
    Log_Write_BodyOdom(time_us);
#endif

    // log state variances every 0.49s
    Log_Write_State_Variances(time_us);

    Log_Write_Timing(time_us);
}

void NavEKF3_core::Log_Write_Timing(uint64_t time_us)
{
    // log EKF timing statistics every 5s
    if (AP::dal().millis() - lastTimingLogTime_ms <= 5000) {
        return;
    }
    lastTimingLogTime_ms = AP::dal().millis();

    const struct log_XKT xkt{
        LOG_PACKET_HEADER_INIT(LOG_XKT_MSG),
        time_us      : time_us,
        core         : core_index,
        timing_count : timing.count,
        dtIMUavg_min : timing.dtIMUavg_min,
        dtIMUavg_max : timing.dtIMUavg_max,
        dtEKFavg_min : timing.dtEKFavg_min,
        dtEKFavg_max : timing.dtEKFavg_max,
        delAngDT_min : timing.delAngDT_min,
        delAngDT_max : timing.delAngDT_max,
        delVelDT_min : timing.delVelDT_min,
        delVelDT_max : timing.delVelDT_max,
    };
    memset(&timing, 0, sizeof(timing));

    AP::logger().WriteBlock(&xkt, sizeof(xkt));
}

void NavEKF3_core::Log_Write_GSF(uint64_t time_us)
{
    if (yawEstimator == nullptr) {
        return;
    }
    yawEstimator->Log_Write(time_us, LOG_XKY0_MSG, LOG_XKY1_MSG, DAL_CORE(core_index));
}

#endif  // HAL_LOGGING_ENABLED
