/*!
 * \file            sbgEComProtocol.h
 * \ingroup         protocol
 * \author          SBG Systems
 * \date            06 February 2013
 *
 * \brief           Implementation of the sbgECom binary communication protocol.
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
 * \defgroup    protocol Low Level Protocol
 * \brief       Defines the low level protocol method to receive and send frames.
 */

#ifndef SBG_ECOM_PROTOCOL_H
#define SBG_ECOM_PROTOCOL_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <interfaces/sbgInterface.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Constant definitions                                               -//
//----------------------------------------------------------------------//

#define SBG_ECOM_MAX_BUFFER_SIZE                (256)                  /*!< Maximum reception buffer size in bytes. */
#define SBG_ECOM_MAX_PAYLOAD_SIZE               (246)                  /*!< Maximum payload size in bytes. */
#define SBG_ECOM_SYNC_1                         (0xFF)                  /*!< First synchronization char of the frame. */
#define SBG_ECOM_SYNC_2                         (0x5A)                  /*!< Second synchronization char of the frame. */
#define SBG_ECOM_ETX                            (0x33)                  /*!< End of frame byte. */

#define SBG_ECOM_RX_TIME_OUT                    (450)                   /*!< Default time out for new frame reception. */

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Forward declaration.
 */
typedef struct _SbgEComProtocol SbgEComProtocol;

/*!
 * Function called each time a valid sbgECom frame is received.
 * 
 * This callback is used to intercept a valid and full sbgECom frame to easily
 * intercept and store raw stream.
 *
 * \param[in]   pProtocol                               sbgECom protocol handle instance.
 * \param[in]   msgClass                                Received frame message class.
 * \param[in]   msgId                                   Received frame message id.
 * \param[out]  pReceivedFrame                          Stream buffer initialized for read operations on the whole frame data.
 * \param[in]   pUserArg                                Optional user supplied argument.
 */
typedef void (*SbgEComProtocolFrameCb)(SbgEComProtocol *pProtocol, uint8_t msgClass, uint8_t msgId, SbgStreamBuffer *pReceivedFrame, void *pUserArg);

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Payload.
 *
 * This class is used to abstract whether some payload is received from a single
 * or multiple frames.
 */
typedef struct _SbgEComProtocolPayload
{
    bool                                 allocated;                                 /*!< True if the buffer is allocated with malloc(). */
    void                                *pBuffer;                                   /*!< Buffer. */
    size_t                               size;                                      /*!< Buffer size, in bytes. */
} SbgEComProtocolPayload;

/*!
 * Struct containing all protocol related data.
 *
 * The member variables related to large transfers are valid if and only if the large buffer is valid.
 */
struct _SbgEComProtocol
{
    SbgInterface                        *pLinkedInterface;                          /*!< Associated interface used by the protocol to read/write bytes. */
    uint8_t                              rxBuffer[SBG_ECOM_MAX_BUFFER_SIZE];        /*!< The reception buffer. */
    size_t                               rxBufferSize;                              /*!< The current reception buffer size in bytes. */
    size_t                               discardSize;                               /*!< Number of bytes to discard on the next receive attempt. */
    uint8_t                              nextLargeTxId;                             /*!< Transfer ID of the next large send. */

    //
    // Raw stream sbgECom frame reception callback
    //
    SbgEComProtocolFrameCb               pReceiveFrameCb;                           /*!< Optional callback used to intercept any received sbgECom frame. */
    void                                *pUserArg;                                  /*!< Optional user supplied argument for the callback. */

    //
    // Member variables related to large transfer reception.
    //
    uint8_t                             *pLargeBuffer;                              /*!< Buffer for large transfers, allocated with malloc() if valid. */
    size_t                               largeBufferSize;                           /*!< Size of the large transfer buffer, in bytes. */
    uint8_t                              msgClass;                                  /*!< Message class for the current large transfer. */
    uint8_t                              msgId;                                     /*!< Message ID for the current large transfer. */
    uint8_t                              transferId;                                /*!< ID of the current large transfer. */
    uint16_t                             pageIndex;                                 /*!< Expected page index of the next frame. */
    uint16_t                             nrPages;                                   /*!< Number of pages in the current transfer. */
};

//----------------------------------------------------------------------//
//- Public methods (SbgEComProtocolPayload)                            -//
//----------------------------------------------------------------------//

/*!
 * Payload constructor.
 *
 * \param[in]   pPayload                Payload.
 */
void sbgEComProtocolPayloadConstruct(SbgEComProtocolPayload *pPayload);

/*!
 * Payload destructor.
 *
 * \param[in]   pPayload                Payload.
 */
void sbgEComProtocolPayloadDestroy(SbgEComProtocolPayload *pPayload);

/*!
 * Get the buffer of a payload.
 *
 * \param[in]   pPayload                Payload.
 * \return                              Payload buffer.
 */
const void *sbgEComProtocolPayloadGetBuffer(const SbgEComProtocolPayload *pPayload);

/*!
 * Get the size of a payload buffer.
 *
 * \param[in]   pPayload                Payload.
 * \return                              Size of the payload buffer, in bytes.
 */
size_t sbgEComProtocolPayloadGetSize(const SbgEComProtocolPayload *pPayload);

//----------------------------------------------------------------------//
//- Public methods (SbgEComProtocol)                                   -//
//----------------------------------------------------------------------//

/*!
 * Initialize the protocol system used to communicate with the product and return the created handle.
 *
 * \param[in]   pProtocol                       Protocol instance to construct.
 * \param[in]   pInterface                      Interface to use for read/write operations.
 * \return                                      SBG_NO_ERROR if we have initialized the protocol system.
 */
SbgErrorCode sbgEComProtocolInit(SbgEComProtocol *pProtocol, SbgInterface *pInterface);

/*!
 * Send data.
 *
 * If the size is SBG_ECOM_MAX_PAYLOAD_SIZE or less, the data is sent in a single frame. Otherwise,
 * is it fragmented into multiple extended frames, each sent in order, which may block.
 *
 * \param[in]   pProtocol                       A valid protocol instance.
 * \param[in]   msgClass                        Message class.
 * \param[in]   msg                             Message ID.
 * \param[in]   pData                           Data buffer.
 * \param[in]   size                            Data buffer size, in bytes.
 * \return                                      SBG_NO_ERROR if the frame has been sent.
 */
SbgErrorCode sbgEComProtocolSend(SbgEComProtocol *pProtocol, uint8_t msgClass, uint8_t msg, const void *pData, size_t size);

/*!
 * Receive a frame.
 *
 * This function is equivalent to sbgEComProtocolReceive() with two exceptions :
 *  - the use of a payload object allows handling payloads not limited by the size of a user-provided buffer
 *  - the payload object allows direct access to the protocol work buffer to avoid an extra copy per call
 *
 * Any allocated resource associated with the given payload is released when calling this function.
 *
 * Because the payload buffer may directly refer to the protocol work buffer on return, it is only valid until
 * the next attempt to receive a frame, with any of the receive functions.
 *
 * \param[in]   pProtocol                       A valid protocol instance.
 * \param[out]  pMsgClass                       Message class, may be NULL.
 * \param[out]  pMsgId                          Message ID, may be NULL.
 * \param[out]  pPayload                        Payload.
 * \return                                      SBG_NO_ERROR if successful,
 *                                              SBG_NOT_READY if no complete frame has been received.
 */
SbgErrorCode sbgEComProtocolReceive2(SbgEComProtocol *pProtocol, uint8_t *pMsgClass, uint8_t *pMsgId, SbgEComProtocolPayload *pPayload);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_PROTOCOL_H
