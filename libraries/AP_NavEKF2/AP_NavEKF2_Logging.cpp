#include "AP_NavEKF2.h"

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

void NavEKF2::Log_Write()
{
    // only log if enabled
    if (activeCores() <= 0) {
        return;
    }

    AP_Logger &logger = AP::logger();

    uint64_t time_us = AP_HAL::micros64();
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    getEulerAngles(0,euler);
    getVelNED(0,velNED);
    getPosNE(0,posNE);
    getPosD(0,posD);
    getGyroBias(0,gyroBias);
    posDownDeriv = getPosDownDerivative(0);
    if (!getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NKF1_MSG),
        time_us : time_us,
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
    logger.WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    uint8_t magIndex = getActiveMag(0);
    getAccelZBias(0,azbias);
    getWind(0,wind);
    getMagNED(0,magNED);
    getMagXYZ(0,magXYZ);
    getGyroScaleErrorPercentage(0,gyroScaleFactor);
    struct log_NKF2 pkt2 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF2_MSG),
        time_us : time_us,
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
        index   : (uint8_t)(magIndex)
    };
    logger.WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF3_MSG),
        time_us : time_us,
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
        innovVT : (int16_t)(100*tasInnov)
    };
    logger.WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    getFilterFaults(0,faultStatus);
    getFilterTimeouts(0,timeoutStatus);
    getFilterStatus(0,solutionStatus);
    getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    getTiltError(0,tiltError);
    int8_t primaryIndex = getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF4_MSG),
        time_us : time_us,
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : (float)tiltError,
        offsetNorth : (int8_t)(offset.x),
        offsetEast : (int8_t)(offset.y),
        faults : (uint16_t)(faultStatus),
        timeouts : (uint8_t)(timeoutStatus),
        solution : (uint16_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    logger.WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {
        LOG_PACKET_HEADER_INIT(LOG_NKF5_MSG),
        time_us : time_us,
        normInnov : (uint8_t)(MIN(100*normInnov,255)),
        FIX : (int16_t)(1000*flowInnovX),
        FIY : (int16_t)(1000*flowInnovY),
        AFI : (int16_t)(1000*auxFlowInnov),
        HAGL : (int16_t)(100*HAGL),
        offset : (int16_t)(100*gndOffset),
        RI : (int16_t)(100*rngInnov),
        meaRng : (uint16_t)(100*range),
        errHAGL : (uint16_t)(100*gndOffsetErr),
        angErr : (float)predictorErrors.x,
        velErr : (float)predictorErrors.y,
        posErr : (float)predictorErrors.z
     };
    logger.WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {
        LOG_PACKET_HEADER_INIT(LOG_NKQ1_MSG),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    logger.WriteBlock(&pktq1, sizeof(pktq1));

    // log innovations for the second IMU if enabled
    if (activeCores() >= 2) {
        // Write 6th EKF packet
        getEulerAngles(1,euler);
        getVelNED(1,velNED);
        getPosNE(1,posNE);
        getPosD(1,posD);
        getGyroBias(1,gyroBias);
        posDownDeriv = getPosDownDerivative(1);
        if (!getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF6_MSG),
            time_us : time_us,
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
        logger.WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        getAccelZBias(1,azbias);
        getWind(1,wind);
        getMagNED(1,magNED);
        getMagXYZ(1,magXYZ);
        getGyroScaleErrorPercentage(1,gyroScaleFactor);
        magIndex = getActiveMag(1);
        struct log_NKF2 pkt7 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF7_MSG),
            time_us : time_us,
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
            index   : (uint8_t)(magIndex)
        };
        logger.WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF8_MSG),
            time_us : time_us,
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
            innovVT : (int16_t)(100*tasInnov)
        };
        logger.WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        getFilterFaults(1,faultStatus);
        getFilterTimeouts(1,timeoutStatus);
        getFilterStatus(1,solutionStatus);
        getFilterGpsStatus(1,gpsStatus);
        getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {
            LOG_PACKET_HEADER_INIT(LOG_NKF9_MSG),
            time_us : time_us,
            sqrtvarV : (int16_t)(100*velVar),
            sqrtvarP : (int16_t)(100*posVar),
            sqrtvarH : (int16_t)(100*hgtVar),
            sqrtvarM : (int16_t)(100*tempVar),
            sqrtvarVT : (int16_t)(100*tasVar),
            tiltErr : (float)tiltError,
            offsetNorth : (int8_t)(offset.x),
            offsetEast : (int8_t)(offset.y),
            faults : (uint16_t)(faultStatus),
            timeouts : (uint8_t)(timeoutStatus),
            solution : (uint16_t)(solutionStatus.value),
            gps : (uint16_t)(gpsStatus.value),
            primary : (int8_t)primaryIndex
        };
        logger.WriteBlock(&pkt9, sizeof(pkt9));

        getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {
            LOG_PACKET_HEADER_INIT(LOG_NKQ2_MSG),
            time_us : time_us,
            q1 : quat.q1,
            q2 : quat.q2,
            q3 : quat.q3,
            q4 : quat.q4
        };
        logger.WriteBlock(&pktq2, sizeof(pktq2));
    }

    // write range beacon fusion debug packet if the range value is non-zero
    if (AP::beacon() != nullptr) {
        uint8_t ID;
        float rng;
        float innovVar;
        float innov;
        float testRatio;
        Vector3f beaconPosNED;
        float bcnPosOffsetHigh;
        float bcnPosOffsetLow;
        if (getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow)) {
            if (rng > 0.0f) {
                struct log_RngBcnDebug pkt10 = {
                    LOG_PACKET_HEADER_INIT(LOG_NKF10_MSG),
                    time_us : time_us,
                    ID : (uint8_t)ID,
                    rng : (int16_t)(100*rng),
                    innov : (int16_t)(100*innov),
                    sqrtInnovVar : (uint16_t)(100*safe_sqrt(innovVar)),
                    testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                    beaconPosN : (int16_t)(100*beaconPosNED.x),
                    beaconPosE : (int16_t)(100*beaconPosNED.y),
                    beaconPosD : (int16_t)(100*beaconPosNED.z),
                    offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                    offsetLow : (int16_t)(100*bcnPosOffsetLow),
                    posN : 0,
                    posE : 0,
                    posD : 0
                };
                logger.WriteBlock(&pkt10, sizeof(pkt10));
            }
        }
    }

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<activeCores(); i++) {
            getTimingStatistics(i, timing);
            logger.Write_EKF_Timing(i==0?"NKT1":"NKT2", time_us, timing);
        }
    }
}
