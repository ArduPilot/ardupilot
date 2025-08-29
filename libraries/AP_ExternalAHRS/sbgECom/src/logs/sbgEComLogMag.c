// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComLogMag.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComLogMagReadFromStream(SbgEComLogMag *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    pLogData->timeStamp         = sbgStreamBufferReadUint32LE(pStreamBuffer);
    pLogData->status            = sbgStreamBufferReadUint16LE(pStreamBuffer);

    pLogData->magnetometers[0]  = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->magnetometers[1]  = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->magnetometers[2]  = sbgStreamBufferReadFloatLE(pStreamBuffer);
                
    pLogData->accelerometers[0] = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->accelerometers[1] = sbgStreamBufferReadFloatLE(pStreamBuffer);
    pLogData->accelerometers[2] = sbgStreamBufferReadFloatLE(pStreamBuffer);

    return sbgStreamBufferGetLastError(pStreamBuffer);
}

SbgErrorCode sbgEComLogMagWriteToStream(const SbgEComLogMag *pLogData, SbgStreamBuffer *pStreamBuffer)
{
    assert(pStreamBuffer);
    assert(pLogData);

    sbgStreamBufferWriteUint32LE(pStreamBuffer, pLogData->timeStamp);
    sbgStreamBufferWriteUint16LE(pStreamBuffer, pLogData->status);
    
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->magnetometers[0]);
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->magnetometers[1]);
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->magnetometers[2]);
        
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->accelerometers[0]);
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->accelerometers[1]);
    sbgStreamBufferWriteFloatLE(pStreamBuffer, pLogData->accelerometers[2]);
    
    return sbgStreamBufferGetLastError(pStreamBuffer);
}
