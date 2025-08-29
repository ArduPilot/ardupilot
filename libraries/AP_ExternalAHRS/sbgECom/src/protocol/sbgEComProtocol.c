// sbgCommonLib headers
#include <sbgCommon.h>
#include <crc/sbgCrc.h>
#include <interfaces/sbgInterface.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComProtocol.h"

//----------------------------------------------------------------------//
//- Private functions                                                  -//
//----------------------------------------------------------------------//

/*!
 * Clear the content of a payload.
 *
 * Any allocated resource is released, and the payload returns to its constructed state.
 *
 * \param[in]   pPayload                    Payload.
 */
static void sbgEComProtocolPayloadClear(SbgEComProtocolPayload *pPayload)
{
    assert(pPayload);

    if (pPayload->allocated)
    {
        free(pPayload->pBuffer);

        pPayload->allocated = false;
    }

    pPayload->pBuffer   = NULL;
    pPayload->size      = 0;
}

/*!
 * Set the properties of a payload.
 *
 * \param[in]   pPayload                    Payload.
 * \param[in]   allocated                   True if the given buffer is allocated with malloc().
 * \param[in]   pBuffer                     Buffer.
 * \param[in]   size                        Buffer size, in bytes.
 */
static void sbgEComProtocolPayloadSet(SbgEComProtocolPayload *pPayload, bool allocated, void *pBuffer, size_t size)
{
    assert(pPayload);
    assert(pBuffer);

    pPayload->allocated = allocated;
    pPayload->pBuffer   = pBuffer;
    pPayload->size      = size;
}

/*!
 * Discard unused bytes from the work buffer of a protocol.
 *
 * \param[in]   pProtocol                   Protocol.
 */
static void sbgEComProtocolDiscardUnusedBytes(SbgEComProtocol *pProtocol)
{
    assert(pProtocol);

    if (pProtocol->discardSize != 0)
    {
        assert(pProtocol->discardSize <= pProtocol->rxBufferSize);

        memmove(pProtocol->rxBuffer, &pProtocol->rxBuffer[pProtocol->discardSize], pProtocol->rxBufferSize - pProtocol->discardSize);

        pProtocol->rxBufferSize -= pProtocol->discardSize;
        pProtocol->discardSize  = 0;
    }
}

/*!
 * Read data from the underlying interface into the work buffer of a protocol.
 *
 * \param[in]   pProtocol                   Protocol.
 */
static void sbgEComProtocolRead(SbgEComProtocol *pProtocol)
{
    SbgErrorCode                         errorCode;

    assert(pProtocol);

    if (pProtocol->rxBufferSize < sizeof(pProtocol->rxBuffer))
    {
        size_t                           nrBytesRead;

        errorCode = sbgInterfaceRead(pProtocol->pLinkedInterface, &pProtocol->rxBuffer[pProtocol->rxBufferSize], &nrBytesRead, sizeof(pProtocol->rxBuffer) - pProtocol->rxBufferSize);

        if (errorCode == SBG_NO_ERROR)
        {
            pProtocol->rxBufferSize += nrBytesRead;
        }
    }
}

/*!
 * Find SYNC bytes in the work buffer of a protocol.
 *
 * The output offset is set if either SBG_NO_ERROR or SBG_NOT_CONTINUOUS_FRAME is returned.
 *
 * \param[in]   pProtocol                   Protocol.
 * \param[in]   startOffset                 Start offset, in bytes.
 * \param[out]  pOffset                     Offset, in bytes.
 * \return                                  SBG_NO_ERROR if successful,
 *                                          SBG_NOT_CONTINUOUS_FRAME if only the first SYNC byte was found,
 *                                          SBG_NOT_READY otherwise.
 */
static SbgErrorCode sbgEComProtocolFindSyncBytes(SbgEComProtocol *pProtocol, size_t startOffset, size_t *pOffset)
{
    SbgErrorCode                         errorCode;

    assert(pProtocol);
    assert(pOffset);
    assert(pProtocol->rxBufferSize > 0);

    errorCode = SBG_NOT_READY;

    for (size_t i = startOffset; i < (pProtocol->rxBufferSize - 1); i++)
    {
        if ((pProtocol->rxBuffer[i] == SBG_ECOM_SYNC_1) && (pProtocol->rxBuffer[i + 1] == SBG_ECOM_SYNC_2))
        {
            *pOffset    = i;
            errorCode   = SBG_NO_ERROR;
            break;
        }
    }

    //
    // The SYNC bytes were not found, but check if the last byte in the work buffer is the first SYNC byte,
    // as it could result from receiving a partial frame.
    //
    if ((errorCode != SBG_NO_ERROR) && (pProtocol->rxBuffer[pProtocol->rxBufferSize - 1] == SBG_ECOM_SYNC_1))
    {
        *pOffset    = pProtocol->rxBufferSize - 1;
        errorCode   = SBG_NOT_CONTINUOUS_FRAME;
    }

    return errorCode;
}

/*!
 * Parse a frame in the work buffer of a protocol.
 *
 * A non-zero number of pages indicates the reception of an extended frame.
 *
 * \param[in]   pProtocol                   Protocol.
 * \param[in]   offset                      Frame offset in the protocol work buffer.
 * \param[out]  pEndOffset                  Frame end offset in the protocol work buffer.
 * \param[out]  pMsgClass                   Message class.
 * \param[out]  pMsgId                      Message ID.
 * \param[out]  pTransferId                 Transfer ID.
 * \param[out]  pPageIndex                  Page index.
 * \param[out]  pNrPages                    Number of pages.
 * \param[out]  pBuffer                     Payload buffer.
 * \param[out]  pSize                       Payload buffer size, in bytes.
 * \return                                  SBG_NO_ERROR if successful,
 *                                          SBG_NOT_READY if the frame is incomplete,
 *                                          SBG_INVALID_FRAME if the frame is invalid,
 *                                          SBG_INVALID_CRC if the frame CRC is invalid.
 */
static SbgErrorCode sbgEComProtocolParseFrame(SbgEComProtocol *pProtocol, size_t offset, size_t *pEndOffset, uint8_t *pMsgClass, uint8_t *pMsgId, uint8_t *pTransferId, uint16_t *pPageIndex, uint16_t *pNrPages, void **pBuffer, size_t *pSize)
{
    SbgErrorCode                         errorCode;
    SbgStreamBuffer                      streamBuffer;
    uint8_t                              msgId;
    uint8_t                              msgClass;
    size_t                               standardPayloadSize;

    assert(pProtocol);
    assert(offset < pProtocol->rxBufferSize);
    assert(pEndOffset);
    assert(pMsgClass);
    assert(pMsgId);
    assert(pTransferId);
    assert(pPageIndex);
    assert(pNrPages);
    assert(pBuffer);
    assert(pSize);

    sbgStreamBufferInitForRead(&streamBuffer, &pProtocol->rxBuffer[offset], pProtocol->rxBufferSize - offset);

    //
    // Skip SYNC bytes.
    //
    sbgStreamBufferSeek(&streamBuffer, 2, SB_SEEK_CUR_INC);

    msgId               = sbgStreamBufferReadUint8(&streamBuffer);
    msgClass            = sbgStreamBufferReadUint8(&streamBuffer);
    standardPayloadSize = sbgStreamBufferReadUint16LE(&streamBuffer);

    errorCode = sbgStreamBufferGetLastError(&streamBuffer);

    if (errorCode == SBG_NO_ERROR)
    {
        if (standardPayloadSize <= SBG_ECOM_MAX_PAYLOAD_SIZE)
        {
            if (sbgStreamBufferGetSize(&streamBuffer) >= (standardPayloadSize + 9))
            {
                size_t                   payloadSize;
                uint8_t                  transferId;
                uint16_t                 pageIndex;
                uint16_t                 nrPages;

                if ((msgClass & 0x80) == 0)
                {
                    payloadSize = standardPayloadSize;

                    transferId  = 0;
                    pageIndex   = 0;
                    nrPages     = 0;

                    errorCode = SBG_NO_ERROR;
                }
                else
                {
                    errorCode = SBG_INVALID_FRAME;
                }

                if (errorCode == SBG_NO_ERROR)
                {
                    void                *pPayloadAddr;
                    uint16_t             frameCrc;
                    uint8_t              lastByte;

                    pPayloadAddr = sbgStreamBufferGetCursor(&streamBuffer);

                    sbgStreamBufferSeek(&streamBuffer, payloadSize, SB_SEEK_CUR_INC);

                    frameCrc    = sbgStreamBufferReadUint16LE(&streamBuffer);
                    lastByte    = sbgStreamBufferReadUint8(&streamBuffer);

                    assert(sbgStreamBufferGetLastError(&streamBuffer) == SBG_NO_ERROR);

                    if (lastByte == SBG_ECOM_ETX)
                    {
                        uint16_t         computedCrc;

                        //
                        // The CRC spans from the header (excluding the SYNC bytes) up to the CRC bytes.
                        //
                        sbgStreamBufferSeek(&streamBuffer, 2, SB_SEEK_SET);
                        computedCrc = sbgCrc16Compute(sbgStreamBufferGetCursor(&streamBuffer), standardPayloadSize + 4);

                        if (frameCrc == computedCrc)
                        {
                            *pEndOffset     = offset + standardPayloadSize + 9;
                            *pMsgClass      = msgClass;
                            *pMsgId         = msgId;
                            *pTransferId    = transferId;
                            *pPageIndex     = pageIndex;
                            *pNrPages       = nrPages;
                            *pBuffer        = pPayloadAddr;
                            *pSize          = payloadSize;

                            errorCode = SBG_NO_ERROR;
                        }
                        else
                        {
                            errorCode = SBG_INVALID_CRC;
                            SBG_LOG_ERROR(errorCode, "invalid CRC, frame:%#" PRIx16 " computed:%#" PRIx16, frameCrc, computedCrc);
                        }
                    }
                    else
                    {
                        errorCode = SBG_INVALID_FRAME;
                        SBG_LOG_ERROR(errorCode, "invalid end-of-frame: byte:%#" PRIx8, lastByte);
                    }
                }
            }
            else
            {
                errorCode = SBG_NOT_READY;
            }
        }
        else
        {
            errorCode = SBG_INVALID_FRAME;
            SBG_LOG_ERROR(errorCode, "invalid payload size %zu", standardPayloadSize);
        }
    }
    else
    {
        errorCode = SBG_NOT_READY;
    }

    return errorCode;
}

/*!
 * Find a frame in the work buffer of a protocol.
 *
 * If an extended frame is received, the number of pages is set to a non-zero value.
 *
 * \param[in]   pProtocol                   Protocol.
 * \param[out]  pMsgClass                   Message class.
 * \param[out]  pMsgId                      Message ID.
 * \param[out]  pTransferId                 Transfer ID.
 * \param[out]  pPageIndex                  Page index.
 * \param[out]  pNrPages                    Number of pages.
 * \param[out]  pBuffer                     Payload buffer.
 * \param[out]  pSize                       Payload buffer size, in bytes.
 * \return                                  SBG_NO_ERROR if successful,
 *                                          SBG_NOT_READY if no frame was found.
 */
static SbgErrorCode sbgEComProtocolFindFrame(SbgEComProtocol *pProtocol, uint8_t *pMsgClass, uint8_t *pMsgId, uint8_t *pTransferId, uint16_t *pPageIndex, uint16_t *pNrPages, void **pBuffer, size_t *pSize)
{
    SbgErrorCode                         errorCode;
    size_t                               startOffset;

    assert(pProtocol);

    errorCode   = SBG_NOT_READY;
    startOffset = 0;

    while (startOffset < pProtocol->rxBufferSize)
    {
        size_t                           offset;

        errorCode = sbgEComProtocolFindSyncBytes(pProtocol, startOffset, &offset);

        if (errorCode == SBG_NO_ERROR)
        {
            size_t                       endOffset;

            errorCode = sbgEComProtocolParseFrame(pProtocol, offset, &endOffset, pMsgClass, pMsgId, pTransferId, pPageIndex, pNrPages, pBuffer, pSize);

            if (errorCode == SBG_NO_ERROR)
            {
                //
                // Valid frame found, discard all data up to and including that frame
                // on the next read.
                //
                pProtocol->discardSize = endOffset;
                
                //
                // If installed, call the method used to intercept received sbgECom frames
                //
                if (pProtocol->pReceiveFrameCb)
                {
                    SbgStreamBuffer     fullFrameStream;

                    sbgStreamBufferInitForRead(&fullFrameStream, &pProtocol->rxBuffer[offset], endOffset-offset);
                    pProtocol->pReceiveFrameCb(pProtocol, *pMsgClass, *pMsgId, &fullFrameStream, pProtocol->pUserArg);
                }

                break;
            }
            else if (errorCode == SBG_NOT_READY)
            {
                //
                // There may be a valid frame at the parse offset, but it's not complete.
                // Have all preceding bytes discarded on the next read.
                //
                pProtocol->discardSize = offset;
                break;
            }
            else
            {
                //
                // Not a valid frame, skip SYNC bytes and try again.
                //
                startOffset = offset + 2;
                errorCode = SBG_NOT_READY;
            }
        }
        else if (errorCode == SBG_NOT_CONTINUOUS_FRAME)
        {
            //
            // The first SYNC byte was found, but not the second. It may be a valid
            // frame, so keep the SYNC byte but have all preceding bytes discarded
            // on the next read.
            //
            pProtocol->discardSize = offset;
            errorCode = SBG_NOT_READY;
            break;
        }
        else
        {
            //
            // No SYNC byte found, discard all data.
            //
            pProtocol->rxBufferSize = 0;
            errorCode = SBG_NOT_READY;
            break;
        }
    }

    assert(pProtocol->discardSize <= pProtocol->rxBufferSize);

    return errorCode;
}

/*!
 * Send a standard frame.
 *
 * \param[in]   pProtocol                   Protocol.
 * \param[in]   msgClass                    Message class.
 * \param[in]   msgId                       Message ID.
 * \param[in]   pData                       Data buffer.
 * \param[in]   size                        Data buffer size, in bytes.
 * \return                                  SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgEComProtocolSendStandardFrame(SbgEComProtocol *pProtocol, uint8_t msgClass, uint8_t msgId, const void *pData, size_t size)
{
    uint8_t                              buffer[SBG_ECOM_MAX_BUFFER_SIZE];
    SbgStreamBuffer                      streamBuffer;
    const uint8_t                       *crcDataStart;
    const uint8_t                       *crcDataEnd;
    uint16_t                             crc;

    assert(pProtocol);
    assert((msgClass & 0x80) == 0);
    assert(size <= SBG_ECOM_MAX_PAYLOAD_SIZE);
    assert(pData || (size == 0));

    sbgStreamBufferInitForWrite(&streamBuffer, buffer, sizeof(buffer));

    sbgStreamBufferWriteUint8(&streamBuffer, SBG_ECOM_SYNC_1);
    sbgStreamBufferWriteUint8(&streamBuffer, SBG_ECOM_SYNC_2);

    crcDataStart = sbgStreamBufferGetCursor(&streamBuffer);

    sbgStreamBufferWriteUint8(&streamBuffer, msgId);
    sbgStreamBufferWriteUint8(&streamBuffer, msgClass);

    sbgStreamBufferWriteUint16LE(&streamBuffer, (uint16_t)size);

    sbgStreamBufferWriteBuffer(&streamBuffer, pData, size);

    crcDataEnd = sbgStreamBufferGetCursor(&streamBuffer);

    crc = sbgCrc16Compute(crcDataStart, crcDataEnd - crcDataStart);

    sbgStreamBufferWriteUint16LE(&streamBuffer, crc);

    sbgStreamBufferWriteUint8(&streamBuffer, SBG_ECOM_ETX);

    assert(sbgStreamBufferGetLastError(&streamBuffer) == SBG_NO_ERROR);

    return sbgInterfaceWrite(pProtocol->pLinkedInterface, sbgStreamBufferGetLinkedBuffer(&streamBuffer), sbgStreamBufferGetLength(&streamBuffer));
}

//----------------------------------------------------------------------//
//- Public methods (SbgEComProtocolPayload)                            -//
//----------------------------------------------------------------------//

void sbgEComProtocolPayloadConstruct(SbgEComProtocolPayload *pPayload)
{
    assert(pPayload);

    pPayload->allocated = false;
    pPayload->pBuffer   = NULL;
    pPayload->size      = 0;
}

void sbgEComProtocolPayloadDestroy(SbgEComProtocolPayload *pPayload)
{
    assert(pPayload);

    if (pPayload->allocated)
    {
        free(pPayload->pBuffer);
    }
}

const void *sbgEComProtocolPayloadGetBuffer(const SbgEComProtocolPayload *pPayload)
{
    assert(pPayload);

    return pPayload->pBuffer;
}

size_t sbgEComProtocolPayloadGetSize(const SbgEComProtocolPayload *pPayload)
{
    assert(pPayload);

    return pPayload->size;
}

//----------------------------------------------------------------------//
//- Public methods (SbgEComProtocol)                                   -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComProtocolInit(SbgEComProtocol *pProtocol, SbgInterface *pInterface)
{
    assert(pProtocol);
    assert(pInterface);

    memset(pProtocol, 0x00, sizeof(*pProtocol));

    pProtocol->pLinkedInterface = pInterface;

    return SBG_NO_ERROR;
}

SbgErrorCode sbgEComProtocolSend(SbgEComProtocol *pProtocol, uint8_t msgClass, uint8_t msgId, const void *pData, size_t size)
{
    SbgErrorCode                         errorCode;

    if (size <= SBG_ECOM_MAX_PAYLOAD_SIZE)
    {
        errorCode = sbgEComProtocolSendStandardFrame(pProtocol, msgClass, msgId, pData, size);
    }
    else
    {
        errorCode = SBG_INVALID_PARAMETER;
        SBG_LOG_ERROR(errorCode, "payload size too large: %zu", size);
    }

    return errorCode;
}

SbgErrorCode sbgEComProtocolReceive2(SbgEComProtocol *pProtocol, uint8_t *pMsgClass, uint8_t *pMsgId, SbgEComProtocolPayload *pPayload)
{
    SbgErrorCode                         errorCode;
    uint8_t                              msgClass;
    uint8_t                              msgId;
    uint8_t                              transferId;
    uint16_t                             pageIndex;
    uint16_t                             nrPages;
    void                                *pBuffer;
    size_t                               size;

    assert(pProtocol);
    assert(pProtocol->discardSize <= pProtocol->rxBufferSize);

    sbgEComProtocolPayloadClear(pPayload);

    sbgEComProtocolDiscardUnusedBytes(pProtocol);

    sbgEComProtocolRead(pProtocol);

    errorCode = sbgEComProtocolFindFrame(pProtocol, &msgClass, &msgId, &transferId, &pageIndex, &nrPages, &pBuffer, &size);

    if (errorCode == SBG_NO_ERROR)
    {
        if (nrPages == 0)
        {
            if (pMsgClass)
            {
                *pMsgClass = msgClass;
            }

            if (pMsgId)
            {
                *pMsgId = msgId;
            }

            sbgEComProtocolPayloadSet(pPayload, false, pBuffer, size);
        }
    }

    return errorCode;
}
