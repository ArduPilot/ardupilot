#include "AP_NavEKF3.h"

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

void NavEKF3::Log_Write_XKF1(uint8_t _core, uint64_t time_us) const
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
        LOG_PACKET_HEADER_INIT(LOG_XKF1_MSG),
        time_us : time_us,
        core    : _core,
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

void NavEKF3::Log_Write_XKF2(uint8_t _core, uint64_t time_us) const
{
    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    uint8_t magIndex = getActiveMag(_core);
    getAccelBias(_core,accelBias);
    getWind(_core,wind);
    getMagNED(_core,magNED);
    getMagXYZ(_core,magXYZ);
    const struct log_NKF2a pkt2{
        LOG_PACKET_HEADER_INIT(LOG_XKF2_MSG),
        time_us : time_us,
        core    : _core,
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
        index   : (uint8_t)(magIndex)
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void NavEKF3::Log_Write_XKF3(uint8_t _core, uint64_t time_us) const
{
    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(_core,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    const struct log_NKF3 pkt3{
        LOG_PACKET_HEADER_INIT(LOG_XKF3_MSG),
        time_us : time_us,
        core    : _core,
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

void NavEKF3::Log_Write_XKF4(uint8_t _core, uint64_t time_us) const
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
    uint8_t primaryIndex = getPrimaryCoreIndex();
    const struct log_NKF4 pkt4{
        LOG_PACKET_HEADER_INIT(LOG_XKF4_MSG),
        time_us : time_us,
        core    : _core,
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
        solution : (uint32_t)(solutionStatus.value),
        gps : (uint16_t)(gpsStatus.value),
        primary : (int8_t)primaryIndex
    };
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}


void NavEKF3::Log_Write_XKF5(uint64_t time_us) const
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
        LOG_PACKET_HEADER_INIT(LOG_XKF5_MSG),
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

void NavEKF3::Log_Write_Quaternion(uint8_t _core, uint64_t time_us) const
{
    // log quaternion
    Quaternion quat;
    getQuaternion(_core, quat);
    const struct log_Quaternion pktq1{
        LOG_PACKET_HEADER_INIT(LOG_XKQ_MSG),
        time_us : time_us,
        core    : _core,
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    AP::logger().WriteBlock(&pktq1, sizeof(pktq1));
}

void NavEKF3::Log_Write_Beacon(uint64_t time_us) const
{
    // write range beacon fusion debug packet if the range value is non-zero
    uint8_t ID;
    float rng;
    float innovVar;
    float innov;
    float testRatio;
    Vector3f beaconPosNED;
    float bcnPosOffsetHigh;
    float bcnPosOffsetLow;
    Vector3f posNED;
     if (getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow, posNED)) {
        if (rng > 0.0f) {
            const struct log_RngBcnDebug pkt10{
                LOG_PACKET_HEADER_INIT(LOG_XKF10_MSG),
                time_us : time_us,
                ID : (uint8_t)ID,
                rng : (int16_t)(100*rng),
                innov : (int16_t)(100*innov),
                sqrtInnovVar : (uint16_t)(100*sqrtf(innovVar)),
                testRatio : (uint16_t)(100*constrain_float(testRatio,0.0f,650.0f)),
                beaconPosN : (int16_t)(100*beaconPosNED.x),
                beaconPosE : (int16_t)(100*beaconPosNED.y),
                beaconPosD : (int16_t)(100*beaconPosNED.z),
                offsetHigh : (int16_t)(100*bcnPosOffsetHigh),
                offsetLow : (int16_t)(100*bcnPosOffsetLow),
                posN : (int16_t)(100*posNED.x),
                posE : (int16_t)(100*posNED.y),
                posD : (int16_t)(100*posNED.z)

             };
            AP::logger().WriteBlock(&pkt10, sizeof(pkt10));
        }
    }
}

void NavEKF3::Log_Write_BodyOdom(uint64_t time_us) const
{
    Vector3f velBodyInnov,velBodyInnovVar;
    static uint32_t lastUpdateTime_ms = 0;
    uint32_t updateTime_ms = getBodyFrameOdomDebug(-1, velBodyInnov, velBodyInnovVar);
    if (updateTime_ms > lastUpdateTime_ms) {
        const struct log_ekfBodyOdomDebug pkt11{
            LOG_PACKET_HEADER_INIT(LOG_XKFD_MSG),
            time_us : time_us,
            velInnovX : velBodyInnov.x,
            velInnovY : velBodyInnov.y,
            velInnovZ : velBodyInnov.z,
            velInnovVarX : velBodyInnovVar.x,
            velInnovVarY : velBodyInnovVar.y,
            velInnovVarZ : velBodyInnovVar.z
         };
        AP::logger().WriteBlock(&pkt11, sizeof(pkt11));
        lastUpdateTime_ms = updateTime_ms;
    }
}

void NavEKF3::Log_Write_State_Variances(uint64_t time_us) const
{
    static uint32_t lastEkfStateVarLogTime_ms = 0;
    if (AP_HAL::millis() - lastEkfStateVarLogTime_ms > 490) {
        lastEkfStateVarLogTime_ms = AP_HAL::millis();
        float stateVar[24];
        getStateVariances(-1, stateVar);
        const struct log_ekfStateVar pktv1{
            LOG_PACKET_HEADER_INIT(LOG_XKV1_MSG),
            time_us : time_us,
            v00 : stateVar[0],
            v01 : stateVar[1],
            v02 : stateVar[2],
            v03 : stateVar[3],
            v04 : stateVar[4],
            v05 : stateVar[5],
            v06 : stateVar[6],
            v07 : stateVar[7],
            v08 : stateVar[8],
            v09 : stateVar[9],
            v10 : stateVar[10],
            v11 : stateVar[11]
        };
        AP::logger().WriteBlock(&pktv1, sizeof(pktv1));
        const struct log_ekfStateVar pktv2{
            LOG_PACKET_HEADER_INIT(LOG_XKV2_MSG),
            time_us : time_us,
            v00 : stateVar[12],
            v01 : stateVar[13],
            v02 : stateVar[14],
            v03 : stateVar[15],
            v04 : stateVar[16],
            v05 : stateVar[17],
            v06 : stateVar[18],
            v07 : stateVar[19],
            v08 : stateVar[20],
            v09 : stateVar[21],
            v10 : stateVar[22],
            v11 : stateVar[23]
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

    uint64_t time_us = AP_HAL::micros64();

    Log_Write_XKF5(time_us);

    for (uint8_t i=0; i<activeCores(); i++) {
        Log_Write_XKF1(i, time_us);
        Log_Write_XKF2(i, time_us);
        Log_Write_XKF3(i, time_us);
        Log_Write_XKF4(i, time_us);
        Log_Write_Quaternion(i, time_us);
    }

    // write range beacon fusion debug packet if the range value is non-zero
    Log_Write_Beacon(time_us);

    // write debug data for body frame odometry fusion
    Log_Write_BodyOdom(time_us);

    // log state variances every 0.49s
    Log_Write_State_Variances(time_us);

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<activeCores(); i++) {
            getTimingStatistics(i, timing);
            Log_EKF_Timing("XKT", i, time_us, timing);
        }
    }
}

