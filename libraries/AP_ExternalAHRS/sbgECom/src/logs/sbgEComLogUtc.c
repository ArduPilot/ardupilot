// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComLogUtc.h"

//----------------------------------------------------------------------//
//- Private methods                                                    -//
//----------------------------------------------------------------------//

/*!
 * Clock status and UTC time status definitions.
 */
#define SBG_ECOM_LOG_UTC_CLOCK_STATE_SHIFT          (1u)                    /*!< Shift used to extract the clock state part. */
#define SBG_ECOM_LOG_UTC_CLOCK_STATE_MASK           (0x000Fu)               /*!< Mask used to keep only the clock state part. */
#define SBG_ECOM_LOG_UTC_TIME_STATUS_SHIFT          (6u)                    /*!< Shift used to extract the UTC status part. */
#define SBG_ECOM_LOG_UTC_TIME_STATUS_MASK           (0x000Fu)               /*!< Mask used to keep only the UTC status part. */

#define SBG_ECOM_LOG_UTC_STATUS_HAS_CLOCK_INPUT     (0x0001u << 0)          /*!< Set to 1 if a stable input clock could be used to synchronized the internal clock. */
#define SBG_ECOM_LOG_UTC_STATUS_UTC_IS_ACCURATE     (0x0001u << 5)          /*!< The UTC time information is accurate and correctly timestamped with a PPS. */

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogUtcReadFromStream(SbgEComLogUtc *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    pLogData->timeStamp         = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->status            = sbgStreamBufferReadUint16LE(pStreamBuffer);
    pLogData->year              = sbgStreamBufferReadUint16LE(pStreamBuffer);
    pLogData->month             = sbgStreamBufferReadInt8LE(pStreamBuffer);
    pLogData->day               = sbgStreamBufferReadInt8LE(pStreamBuffer);
    pLogData->hour              = sbgStreamBufferReadInt8LE(pStreamBuffer);
    pLogData->minute            = sbgStreamBufferReadInt8LE(pStreamBuffer);
    pLogData->second            = sbgStreamBufferReadInt8LE(pStreamBuffer);
    pLogData->nanoSecond        = sbgStreamBufferReadInt32LE(pStreamBuffer);
    pLogData->gpsTimeOfWeek     = sbgStreamBufferReadUint32LE(pStreamBuffer);

    if (sbgStreamBufferGetSpace(pStreamBuffer) >= 3*sizeof(float))
    {
        pLogData->clkBiasStd        = sbgStreamBufferReadFloatLE(pStreamBuffer);
        pLogData->clkSfErrorStd     = sbgStreamBufferReadFloatLE(pStreamBuffer);
        pLogData->clkResidualError  = sbgStreamBufferReadFloatLE(pStreamBuffer);
    }
    else
    {
        pLogData->clkBiasStd        = NAN;
        pLogData->clkSfErrorStd     = NAN;
        pLogData->clkResidualError  = NAN;
    }

    return sbgStreamBufferGetLastError(pStreamBuffer);
}

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

SbgEComClockState sbgEComLogUtcGetClockState(const SbgEComLogUtc *pLogData)
{
    assert(pLogData);

    return (SbgEComClockState)((pLogData->status >> SBG_ECOM_LOG_UTC_CLOCK_STATE_SHIFT)&SBG_ECOM_LOG_UTC_CLOCK_STATE_MASK);
}
