#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_DAL/AP_DAL.h>

void NavEKF2_core::Log_Write_NKF1(uint64_t time_us) const
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
    const struct log_EKF1 pkt{
        LOG_PACKET_HEADER_INIT(LOG_NKF1_MSG),
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

void NavEKF2_core::Log_Write_NKF2(uint64_t time_us) const
{
    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    getAccelZBias(azbias);
    getWind(wind);
    getMagNED(magNED);
    getMagXYZ(magXYZ);
    getGyroScaleErrorPercentage(gyroScaleFactor);
    const struct log_NKF2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_NKF2_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        AZbias  : (int8_t)(100*azbias),
        scaleX  : (int16_t)(100*gyroScaleFactor.x),
        scaleY  : (int16_t)(100*gyroScaleFactor.y),
        scaleZ  : (int16_t)(100*gyroScaleFactor.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        index   : magSelectIndex
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void NavEKF2_core::Log_Write_NKF3(uint64_t time_us) const
{
    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    const struct log_NKF3 pkt3{
        LOG_PACKET_HEADER_INIT(LOG_NKF3_MSG),
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
        rerr : 0, // TODO : Relative Error based Lane-Switching like EK3
        errorScore : 0 // TODO : Relative Error based Lane-Switching like EK3
    };
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}

void NavEKF2_core::Log_Write_NKF4(uint64_t time_us) const
{
    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t _faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    getFilterFaults(_faultStatus);
    getFilterTimeouts(timeoutStatus);
    getFilterStatus(solutionStatus);
    getFilterGpsStatus(gpsStatus);
    float tiltError;
    getTiltError(tiltError);
    const struct log_NKF4 pkt4{
        LOG_PACKET_HEADER_INIT(LOG_NKF4_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : _faultStatus,
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint32_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : frontend->getPrimaryCoreIndex()
    };
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}

void NavEKF2_core::Log_Write_NKF5(uint64_t time_us) const
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    // Write fifth EKF packet
    const struct log_NKF5 pkt5{
        LOG_PACKET_HEADER_INIT(LOG_NKF5_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        normInnov : (uint8_t)(MIN(100*MAX(flowTestRatio[0],flowTestRatio[1]),255)),  // normalised innovation variance ratio for optical flow observations fused by the main nav filter
        FIX : (int16_t)(1000*innovOptFlow[0]),  // optical flow LOS rate vector innovations from the main nav filter
        FIY : (int16_t)(1000*innovOptFlow[1]),  // optical flow LOS rate vector innovations from the main nav filter
        AFI : (int16_t)(1000*norm(auxFlowObsInnov.x,auxFlowObsInnov.y)),  // optical flow LOS rate innovation from terrain offset estimator
        HAGL : (int16_t)(100*(terrainState - stateStruct.position.z)),  // height above ground level
        offset : (int16_t)(100*terrainState),  // // estimated vertical position of the terrain relative to the nav filter zero datum
        RI : (int16_t)(100*innovRng),  // range finder innovations
        meaRng : (uint16_t)(100*rangeDataDelayed.rng),  // measured range
        errHAGL : (uint16_t)(100*sqrtf(Popt)),  // filter ground offset state error
        angErr : outputTrackError.x,
        velErr : outputTrackError.y,
        posErr : outputTrackError.z
    };
    AP::logger().WriteBlock(&pkt5, sizeof(pkt5));
}

void NavEKF2_core::Log_Write_Quaternion(uint64_t time_us) const
{
    // log quaternion
    Quaternion quat;
    getQuaternion(quat);
    const struct log_Quaternion pktq1{
        LOG_PACKET_HEADER_INIT(LOG_NKQ_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    AP::logger().WriteBlock(&pktq1, sizeof(pktq1));
}

void NavEKF2_core::Log_Write_Beacon(uint64_t time_us)
{
    if (core_index != frontend->primary) {
        // log only primary instance for now
        return;
    }

    if (AP::beacon() == nullptr) {
        return;
    }

    if (!statesInitialised || N_beacons == 0) {
        return;
    }

    // Ensure that beacons are not skipped due to calling this
    // function at a rate lower than the updates
    if (rngBcnFuseDataReportIndex >= N_beacons) {
        rngBcnFuseDataReportIndex = 0;
    }

    const rngBcnFusionReport_t &report = rngBcnFusionReport[rngBcnFuseDataReportIndex];

    if (report.rng <= 0.0f) {
        rngBcnFuseDataReportIndex++;
        return;
    }

    struct log_RngBcnDebug pkt10 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF10_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        ID : rngBcnFuseDataReportIndex,
        rng : (int16_t)(100*report.rng),
        innov : (int16_t)(100*report.innov),
        sqrtInnovVar : (uint16_t)(100*safe_sqrt(report.innovVar)),
        testRatio : (uint16_t)(100*constrain_float(report.testRatio,0.0f,650.0f)),
        beaconPosN : (int16_t)(100*report.beaconPosNED.x),
        beaconPosE : (int16_t)(100*report.beaconPosNED.y),
        beaconPosD : (int16_t)(100*report.beaconPosNED.z),
        offsetHigh : (int16_t)(100*bcnPosOffsetMax),
        offsetLow : (int16_t)(100*bcnPosOffsetMin),
        posN : 0,
        posE : 0,
        posD : 0
    };
    AP::logger().WriteBlock(&pkt10, sizeof(pkt10));
    rngBcnFuseDataReportIndex++;
}

void NavEKF2_core::Log_Write_Timing(uint64_t time_us)
{
    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP::dal().millis() - lastTimingLogTime_ms <= 5000) {
        return;
    }
    lastTimingLogTime_ms = AP::dal().millis();

    const struct log_NKT nkt{
        LOG_PACKET_HEADER_INIT(LOG_NKT_MSG),
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

    AP::logger().WriteBlock(&nkt, sizeof(nkt));
}


void NavEKF2::Log_Write()
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

    const uint64_t time_us = AP::dal().micros64();

    // note that several of these functions exit-early if they're not
    // attempting to log the primary core.
    for (uint8_t i=0; i<activeCores(); i++) {
        core[i].Log_Write(time_us);
    }

    AP::dal().start_frame(AP_DAL::FrameType::LogWriteEKF2);
}

void NavEKF2_core::Log_Write(uint64_t time_us)
{
    // note that several of these functions exit-early if they're not
    // attempting to log the primary core.
    Log_Write_NKF1(time_us);
    Log_Write_NKF2(time_us);
    Log_Write_NKF3(time_us);
    Log_Write_NKF4(time_us);
    Log_Write_NKF5(time_us);

    Log_Write_Quaternion(time_us);
    Log_Write_GSF(time_us);

    // write range beacon fusion debug packet if the range value is non-zero
    Log_Write_Beacon(time_us);

    Log_Write_Timing(time_us);
}

void NavEKF2_core::Log_Write_GSF(uint64_t time_us) const
{
    if (yawEstimator == nullptr) {
        return;
    }

    float yaw_composite;
    float yaw_composite_variance;
    float yaw[N_MODELS_EKFGSF];
    float ivn[N_MODELS_EKFGSF];
    float ive[N_MODELS_EKFGSF];
    float wgt[N_MODELS_EKFGSF];

    if (!yawEstimator->getLogData(yaw_composite, yaw_composite_variance, yaw, ivn, ive, wgt)) {
        return;
    }

    const struct log_NKY0 nky0{
        LOG_PACKET_HEADER_INIT(LOG_NKY0_MSG),
        time_us                 : time_us,
        core                    : DAL_CORE(core_index),
        yaw_composite           : yaw_composite,
        yaw_composite_variance  : sqrtf(MAX(yaw_composite_variance, 0.0f)),
        yaw0                    : yaw[0],
        yaw1                    : yaw[1],
        yaw2                    : yaw[2],
        yaw3                    : yaw[3],
        yaw4                    : yaw[4],
        wgt0                    : wgt[0],
        wgt1                    : wgt[1],
        wgt2                    : wgt[2],
        wgt3                    : wgt[3],
        wgt4                    : wgt[4],
    };
    AP::logger().WriteBlock(&nky0, sizeof(nky0));

    const struct log_NKY1 nky1{
        LOG_PACKET_HEADER_INIT(LOG_NKY1_MSG),
        time_us                 : time_us,
        core                    : DAL_CORE(core_index),
        ivn0                    : ivn[0],
        ivn1                    : ivn[1],
        ivn2                    : ivn[2],
        ivn3                    : ivn[3],
        ivn4                    : ivn[4],
        ive0                    : ive[0],
        ive1                    : ive[1],
        ive2                    : ive[2],
        ive3                    : ive[3],
        ive4                    : ive[4],
    };
    AP::logger().WriteBlock(&nky1, sizeof(nky1));
}
