/*!
 * \file            sbgECom.h
 * \ingroup         main
 * \author          SBG Systems
 * \date            05 February 2013
 *
 * \brief           Contains main sbgECom methods.
 *
 * \copyright       Copyright (C) 2007-2024, SBG Systems SAS. All rights reserved.
 * \beginlicense    The MIT license
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * \endlicense
 */

/*!
 * \defgroup    main Main Library
 * \brief       Main sbgECom library initialization and parsing methods.
 */

#ifndef SBG_ECOM_H
#define SBG_ECOM_H

#ifdef __cplusplus
extern "C" {
#endif

// sbgCommonLib headers
#include <sbgCommon.h>
#include <interfaces/sbgInterface.h>

// Local headers
#include "sbgEComIds.h"
#include "logs/sbgEComLog.h"
#include "protocol/sbgEComProtocol.h"

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgEComHandle SbgEComHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Callback definition called each time a new log is received.
 *
 * \param[in]   pHandle                                 Valid handle on the sbgECom instance that has called this callback.
 * \param[in]   msgClass                                Class of the message we have received
 * \param[in]   msg                                     Message ID of the log received.
 * \param[in]   pLogData                                Contains the received log data as an union.
 * \param[in]   pUserArg                                Optional user supplied argument.
 * \return                                              SBG_NO_ERROR if the received log has been used successfully.
 */
typedef SbgErrorCode (*SbgEComReceiveLogFunc)(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgEComLogUnion *pLogData, void *pUserArg);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 */
struct _SbgEComHandle
{
    SbgEComProtocol              protocolHandle;            /*!< Handle on the protocol system. */

    SbgEComReceiveLogFunc        pReceiveLogCallback;       /*!< Pointer on the method called each time a new binary log is received. */
    void                        *pUserArg;                  /*!< Optional user supplied argument for callbacks. */

    uint32_t                     numTrials;                 /*!< Number of trials when a command is sent (default is 3). */
    uint32_t                     cmdDefaultTimeOut;         /*!< Default time out in ms to get an answer from the device (default 500 ms). */
};

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Initialize the protocol system used to communicate with the product and return the created handle.
 * 
 * \param[out]  pHandle                         Pointer used to store the allocated and initialized sbgECom handle.
 * \param[in]   pInterface                      Interface to use for read/write operations.
 * \return                                      SBG_NO_ERROR if we have initialized the protocol system.
 */
SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface);

/*!
 * Define the callback that should be called each time a new binary log is received.
 * 
 * \param[in]   pHandle                         A valid sbgECom handle.
 * \param[in]   pReceiveLogCallback             Pointer on the callback to call when a new log is received.
 * \param[in]   pUserArg                        Optional user argument that will be passed to the callback method.
 */
void sbgEComSetReceiveLogCallback(SbgEComHandle *pHandle, SbgEComReceiveLogFunc pReceiveLogCallback, void *pUserArg);

#ifdef __cplusplus
}
#endif

#endif  // SBG_ECOM_H

