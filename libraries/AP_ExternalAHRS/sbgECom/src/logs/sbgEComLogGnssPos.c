// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComLogGnssPos.h"

//----------------------------------------------------------------------//
//- Private definitions for status field                               -//
//----------------------------------------------------------------------//
#define SBG_ECOM_LOG_GNSS_POS_TYPE_SHIFT        (6u)                    /*!< Shift used to extract the GNSS position type part. */
#define SBG_ECOM_LOG_GNSS_POS_TYPE_MASK         (0x0000003Fu)           /*!< Mask used to keep only the GNSS position type part. */

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogGnssPosReadFromStream(SbgEComLogGnssPos *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    bool        shouldContinue;

    assert(pLogData);
    assert(pStreamBuffer);

    pLogData->timeStamp             = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->status                = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->timeOfWeek            = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->latitude              = sbgStreamBufferReadDoubleLE(pStreamBuffer);
    pLogData->longitude             = sbgStreamBufferReadDoubleLE(pStreamBuffer);
    pLogData->altitude              = sbgStreamBufferReadDoubleLE(pStreamBuffer);
    pLogData->undulation            = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->latitudeAccuracy      = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->longitudeAccuracy     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->altitudeAccuracy      = sbgStreamBufferReadFloatLE(pStreamBuffer);

    //
    // Read additional fields added in version 1.4
    //
    if (sbgStreamBufferGetSpace(pStreamBuffer) >= 5)
    {
        pLogData->numSvUsed         = sbgStreamBufferReadUint8LE(pStreamBuffer);
        pLogData->baseStationId     = sbgStreamBufferReadUint16LE(pStreamBuffer);
        pLogData->differentialAge   = sbgStreamBufferReadUint16LE(pStreamBuffer);
        shouldContinue              = true;
    }
    else
    {
        pLogData->numSvUsed         = UINT8_MAX;
        pLogData->baseStationId     = UINT16_MAX;
        pLogData->differentialAge   = UINT16_MAX;
        shouldContinue              = false;
    }

    //
    // Read additional status added in version 4.0
    //
    if ( shouldContinue && (sbgStreamBufferGetSpace(pStreamBuffer) >= 5) )
    {
        pLogData->numSvTracked      = sbgStreamBufferReadUint8LE(pStreamBuffer);
        pLogData->statusExt         = sbgStreamBufferReadUint32LE(pStreamBuffer);
    }
    else
    {
        pLogData->numSvTracked      = UINT8_MAX;
        pLogData->statusExt         = 0;
    }
    
    return sbgStreamBufferGetLastError(pStreamBuffer);
}

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

SbgEComGnssPosType sbgEComLogGnssPosGetType(const SbgEComLogGnssPos *pLogData)
{
    assert(pLogData);

    return (SbgEComGnssPosType)((pLogData->status >> SBG_ECOM_LOG_GNSS_POS_TYPE_SHIFT)&SBG_ECOM_LOG_GNSS_POS_TYPE_MASK);
}
