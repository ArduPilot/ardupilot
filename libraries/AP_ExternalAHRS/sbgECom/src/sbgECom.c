// sbgCommonLib headers
#include <sbgCommon.h>
#include <interfaces/sbgInterface.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgECom.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface)
{
    SbgErrorCode errorCode = SBG_NO_ERROR;
    
    assert(pHandle);
    assert(pInterface);
    
    //
    // Initialize the sbgECom handle
    //
    pHandle->pReceiveLogCallback    = NULL;
    pHandle->pUserArg               = NULL;

    //
    // Initialize the default number of trials and time out
    //
    pHandle->numTrials          = 3;
    pHandle->cmdDefaultTimeOut  = 0;

    //
    // Initialize the protocol 
    //
    errorCode = sbgEComProtocolInit(&pHandle->protocolHandle, pInterface);
    
    return errorCode;
}

void sbgEComSetReceiveLogCallback(SbgEComHandle *pHandle, SbgEComReceiveLogFunc pReceiveLogCallback, void *pUserArg)
{
    assert(pHandle);

    //
    // Define the callback and the user argument
    //
    pHandle->pReceiveLogCallback    = pReceiveLogCallback;
    pHandle->pUserArg               = pUserArg;
}
