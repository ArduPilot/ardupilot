#include "sbgEComLogAirData.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogAirDataWriteToStream(const SbgEComLogAirData *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pLogData);
    assert(pStreamBuffer);

    sbgStreamBufferWriteUint32LE(pStreamBuffer, pLogData->timeStamp);
    sbgStreamBufferWriteUint16LE(pStreamBuffer, pLogData->status);

    sbgStreamBufferWriteFloatLE(pStreamBuffer,  pLogData->pressureAbs);
    sbgStreamBufferWriteFloatLE(pStreamBuffer,  pLogData->altitude);

    sbgStreamBufferWriteFloatLE(pStreamBuffer,  pLogData->pressureDiff);
    sbgStreamBufferWriteFloatLE(pStreamBuffer,  pLogData->trueAirspeed);

    sbgStreamBufferWriteFloatLE(pStreamBuffer,  pLogData->airTemperature);

    return sbgStreamBufferGetLastError(pStreamBuffer);
}
