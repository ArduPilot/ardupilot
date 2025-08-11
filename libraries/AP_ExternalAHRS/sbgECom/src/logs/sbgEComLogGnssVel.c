// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComLogGnssVel.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogGnssVelReadFromStream(SbgEComLogGnssVel *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pLogData);
    assert(pStreamBuffer);

    pLogData->timeStamp         = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->status            = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->timeOfWeek        = sbgStreamBufferReadUint32LE(pStreamBuffer);
    
    pLogData->velocity[0]       = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocity[1]       = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocity[2]       = sbgStreamBufferReadFloatLE(pStreamBuffer);
    
    pLogData->velocityAcc[0]    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocityAcc[1]    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->velocityAcc[2]    = sbgStreamBufferReadFloatLE(pStreamBuffer);
    
    pLogData->course            = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->courseAcc         = sbgStreamBufferReadFloatLE(pStreamBuffer);

    return sbgStreamBufferGetLastError(pStreamBuffer);
}
