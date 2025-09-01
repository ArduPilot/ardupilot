// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Project headers
#include <sbgEComIds.h>

// Local headers
#include "sbgEComLog.h"
#include "sbgEComLogAirData.h"
#include "sbgEComLogEkf.h"
#include "sbgEComLogGnssPos.h"
#include "sbgEComLogGnssVel.h"
#include "sbgEComLogImu.h"
#include "sbgEComLogMag.h"
#include "sbgEComLogUtc.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogParse(SbgEComClass msgClass, SbgEComMsgId msgId, const void *pPayload, size_t payloadSize, SbgEComLogUnion *pLogData)
{
    SbgErrorCode        errorCode = SBG_NO_ERROR;
    SbgStreamBuffer     inputStream;

    assert(pPayload);
    assert(payloadSize > 0);
    assert(pLogData);

    //
    // Create an input stream buffer that points to the frame payload so we can easily parse it's content
    //
    sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

    //
    // Handle the different classes of messages differently
    //
    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        //
        // Parse the incoming log according to its type
        //
        switch (msgId)
        {
        case SBG_ECOM_LOG_IMU_SHORT:
            errorCode = sbgEComLogImuShortReadFromStream(&pLogData->imuShort, &inputStream);
            break;
        case SBG_ECOM_LOG_EKF_QUAT:
            errorCode = sbgEComLogEkfQuatReadFromStream(&pLogData->ekfQuatData, &inputStream);
            break;
        case SBG_ECOM_LOG_EKF_NAV:
            errorCode = sbgEComLogEkfNavReadFromStream(&pLogData->ekfNavData, &inputStream);
            break;
        case SBG_ECOM_LOG_UTC_TIME:
            errorCode = sbgEComLogUtcReadFromStream(&pLogData->utcData, &inputStream);
            break;
        case SBG_ECOM_LOG_GPS1_VEL:
        case SBG_ECOM_LOG_GPS2_VEL:
            errorCode = sbgEComLogGnssVelReadFromStream(&pLogData->gpsVelData, &inputStream);
            break;
        case SBG_ECOM_LOG_GPS1_POS:
        case SBG_ECOM_LOG_GPS2_POS:
            errorCode = sbgEComLogGnssPosReadFromStream(&pLogData->gpsPosData, &inputStream);
            break;
        case SBG_ECOM_LOG_MAG:
            errorCode = sbgEComLogMagReadFromStream(&pLogData->magData, &inputStream);
            break;

        default:
            errorCode = SBG_ERROR;
        }
    }
    else
    {
        //
        // Unhandled message class
        //
        errorCode = SBG_ERROR;
    }

    return errorCode;
}

void sbgEComLogCleanup(SbgEComLogUnion *pLogData, SbgEComClass msgClass, SbgEComMsgId msgId)
{
    assert(pLogData);

    SBG_UNUSED_PARAMETER(pLogData);
    SBG_UNUSED_PARAMETER(msgClass);
    SBG_UNUSED_PARAMETER(msgId);

    //
    // Nothing to do for now
    //
}
