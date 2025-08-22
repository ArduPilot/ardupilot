#include "sbgEComLogEkf.h"

//----------------------------------------------------------------------//
//- Private definitions                                                -//
//----------------------------------------------------------------------//

/*!
 * Solution status mode definitions.
 */
#define SBG_ECOM_LOG_EKF_SOLUTION_MODE_SHIFT        (0u)                /*!< Shift used to extract the clock status part. */
#define SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK         (0x0000000Fu)       /*!< Mask used to keep only the clock status part. */

//----------------------------------------------------------------------//
//- Public methods (SbgEComLogEkfQuat)                                 -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogEkfQuatReadFromStream(SbgEComLogEkfQuat *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    pLogData->timeStamp         = sbgStreamBufferReadUint32LE(pStreamBuffer);

    pLogData->quaternion[0]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->quaternion[1]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->quaternion[2]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->quaternion[3]     = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->eulerStdDev[0]    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->eulerStdDev[1]    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->eulerStdDev[2]    = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->status            = sbgStreamBufferReadUint32LE(pStreamBuffer);

    //
    // Added in sbgECom 5.0
    //
    if (sbgStreamBufferGetSpace(pStreamBuffer) >= 2*sizeof(float))
    {
        pLogData->magDeclination    = sbgStreamBufferReadFloatLE(pStreamBuffer);
        pLogData->magInclination    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    }
    else
    {
        pLogData->magDeclination        = NAN;
        pLogData->magInclination        = NAN;
    }

    return sbgStreamBufferGetLastError(pStreamBuffer);
}

//----------------------------------------------------------------------//
//- Public methods (SbgEComLogEkfNav)                                  -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogEkfNavReadFromStream(SbgEComLogEkfNav *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    pLogData->timeStamp             = sbgStreamBufferReadUint32LE(pStreamBuffer);

    pLogData->velocity[0]           = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocity[1]           = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocity[2]           = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->velocityStdDev[0]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocityStdDev[1]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocityStdDev[2]     = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->position[0]           = sbgStreamBufferReadDoubleLE(pStreamBuffer);
    pLogData->position[1]           = sbgStreamBufferReadDoubleLE(pStreamBuffer);
    pLogData->position[2]           = sbgStreamBufferReadDoubleLE(pStreamBuffer);

    pLogData->undulation            = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->positionStdDev[0]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->positionStdDev[1]     = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->positionStdDev[2]     = sbgStreamBufferReadFloatLE(pStreamBuffer);

    pLogData->status                = sbgStreamBufferReadUint32LE(pStreamBuffer);

    return sbgStreamBufferGetLastError(pStreamBuffer);
}

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

SbgEComSolutionMode sbgEComLogEkfGetSolutionMode(uint32_t status)
{
    return (SbgEComSolutionMode)((status >> SBG_ECOM_LOG_EKF_SOLUTION_MODE_SHIFT) & SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK);
}
