#include "AP_NavEKF2.h"

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

void NavEKF2::Log_Write_EKF1(uint8_t _core, LogMessages msg_id, uint64_t time_us) const
{
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    getEulerAngles(_core,euler);
    getVelNED(_core,velNED);
    getPosNE(_core,posNE);
    getPosD(_core,posD);
    getGyroBias(_core,gyroBias);
    posDownDeriv = getPosDownDerivative(_core);
    if (!getOriginLLH(_core,originLLH)) {
        originLLH.alt = 0;
    }
    const struct log_EKF1 pkt{
        LOG_PACKET_HEADER_INIT(msg_id),
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
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void NavEKF2::Log_Write_NKF2(uint8_t _core, LogMessages msg_id, uint64_t time_us) const
{
    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    uint8_t magIndex = getActiveMag(_core);
    getAccelZBias(_core,azbias);
    getWind(_core,wind);
    getMagNED(_core,magNED);
    getMagXYZ(_core,magXYZ);
    getGyroScaleErrorPercentage(_core,gyroScaleFactor);
    const struct log_NKF2 pkt2{
        LOG_PACKET_HEADER_INIT(msg_id),
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
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void NavEKF2::Log_Write_NKF3(uint8_t _core, LogMessages msg_id, uint64_t time_us) const
{
    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(_core,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    const struct log_NKF3 pkt3{
        LOG_PACKET_HEADER_INIT(msg_id),
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
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}

void NavEKF2::Log_Write_NKF4(uint8_t _core, LogMessages msg_id, uint64_t time_us) const
{
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
    getVariances(_core,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    getFilterFaults(_core,faultStatus);
    getFilterTimeouts(_core,timeoutStatus);
    getFilterStatus(_core,solutionStatus);
    getFilterGpsStatus(_core,gpsStatus);
    float tiltError;
    getTiltError(_core,tiltError);
    int8_t primaryIndex = getPrimaryCoreIndex();
    const struct log_NKF4 pkt4{
        LOG_PACKET_HEADER_INIT(msg_id),
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
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}

void NavEKF2::Log_Write_NKF5(uint64_t time_us) const
{
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
    const struct log_NKF5 pkt5{
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
    AP::logger().WriteBlock(&pkt5, sizeof(pkt5));
}

void NavEKF2::Log_Write_Quaternion(uint8_t _core, LogMessages msg_id, uint64_t time_us) const
{
    // log quaternion
    Quaternion quat;
    getQuaternion(_core, quat);
    const struct log_Quaternion pktq1{
        LOG_PACKET_HEADER_INIT(msg_id),
        time_us : time_us,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    AP::logger().WriteBlock(&pktq1, sizeof(pktq1));
}

void NavEKF2::Log_Write_Beacon(uint64_t time_us) const
{
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
                AP::logger().WriteBlock(&pkt10, sizeof(pkt10));
            }
        }
    }
}

void NavEKF2::Log_Write()
{
    // only log if enabled
    if (activeCores() <= 0) {
        return;
    }

    const uint64_t time_us = AP_HAL::micros64();

    Log_Write_EKF1(0, LOG_NKF1_MSG, time_us);
    Log_Write_NKF2(0, LOG_NKF2_MSG, time_us);
    Log_Write_NKF3(0, LOG_NKF3_MSG, time_us);
    Log_Write_NKF4(0, LOG_NKF4_MSG, time_us);
    Log_Write_NKF5(time_us);
    Log_Write_Quaternion(0, LOG_NKQ1_MSG, time_us);

    // log EKF state info for the second EFK core if enabled
    if (activeCores() >= 2) {
        Log_Write_EKF1(1, LOG_NKF6_MSG, time_us);
        Log_Write_NKF2(1, LOG_NKF7_MSG, time_us);
        Log_Write_NKF3(1, LOG_NKF8_MSG, time_us);
        Log_Write_NKF4(1, LOG_NKF9_MSG, time_us);
        Log_Write_Quaternion(1, LOG_NKQ2_MSG, time_us);
    }

    // log EKF state info for the third EFK core if enabled
    if (activeCores() >= 3) {
        Log_Write_EKF1(2, LOG_NKF11_MSG, time_us);
        Log_Write_NKF2(2, LOG_NKF12_MSG, time_us);
        Log_Write_NKF3(2, LOG_NKF13_MSG, time_us);
        Log_Write_NKF4(2, LOG_NKF14_MSG, time_us);
        Log_Write_Quaternion(2, LOG_NKQ3_MSG, time_us);
    }

    // write range beacon fusion debug packet if the range value is non-zero
    Log_Write_Beacon(time_us);

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<activeCores(); i++) {
            getTimingStatistics(i, timing);
            if (i == 0) {
                Log_EKF_Timing("NKT1", time_us, timing);
            } else if (i == 1) {
                Log_EKF_Timing("NKT2", time_us, timing);
            } else if (i == 2) {
                Log_EKF_Timing("NKT3", time_us, timing);
            }
        }
    }
}
