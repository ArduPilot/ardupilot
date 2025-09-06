/*!
 * \file            sbgStreamBufferCommon.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            02 January 2013
 *
 * \brief           Used to read/write data from/to a memory buffer stream.
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

#ifndef SBG_STREAM_BUFFER_COMMON_H
#define SBG_STREAM_BUFFER_COMMON_H

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- General definitions                                                -//
//----------------------------------------------------------------------//

/*!
 *  The default method should read and write using the platform endianness
 */
#if SBG_CONFIG_BIG_ENDIAN == 1
    /*!
     *  The platform is a big endian one so default methods should use big endian byte order.
     */
    #define sbgStreamBufferReadUint16       sbgStreamBufferReadUint16BE
    #define sbgStreamBufferReadInt16        sbgStreamBufferReadInt16BE

    #define sbgStreamBufferReadUint24       sbgStreamBufferReadUint24BE
    #define sbgStreamBufferReadInt24        sbgStreamBufferReadInt24BE

    #define sbgStreamBufferReadUint32       sbgStreamBufferReadUint32BE
    #define sbgStreamBufferReadInt32        sbgStreamBufferReadInt32BE

    #define sbgStreamBufferReadUint40       sbgStreamBufferReadUint40BE
    #define sbgStreamBufferReadInt40        sbgStreamBufferReadInt40BE

    #define sbgStreamBufferReadUint48       sbgStreamBufferReadUint48BE
    #define sbgStreamBufferReadInt48        sbgStreamBufferReadInt48BE

    #define sbgStreamBufferReadUint56       sbgStreamBufferReadUint56BE
    #define sbgStreamBufferReadInt56        sbgStreamBufferReadInt56BE

    #define sbgStreamBufferReadUint64       sbgStreamBufferReadUint64BE
    #define sbgStreamBufferReadInt64        sbgStreamBufferReadInt64BE

    #define sbgStreamBufferReadSizeT32      sbgStreamBufferReadSizeT32BE
    #define sbgStreamBufferReadSizeT64      sbgStreamBufferReadSizeT64BE

    #define sbgStreamBufferReadFloat        sbgStreamBufferReadFloatBE
    #define sbgStreamBufferReadDouble       sbgStreamBufferReadDoubleBE

    #define sbgStreamBufferWriteUint16      sbgStreamBufferWriteUint16BE
    #define sbgStreamBufferWriteInt16       sbgStreamBufferWriteInt16BE

    #define sbgStreamBufferWriteUint24      sbgStreamBufferWriteUint24BE
    #define sbgStreamBufferWriteInt24       sbgStreamBufferWriteInt24BE

    #define sbgStreamBufferWriteUint32      sbgStreamBufferWriteUint32BE
    #define sbgStreamBufferWriteInt32       sbgStreamBufferWriteInt32BE

    #define sbgStreamBufferWriteUint64      sbgStreamBufferWriteUint64BE
    #define sbgStreamBufferWriteInt64       sbgStreamBufferWriteInt64BE

    #define sbgStreamBufferWriteSizeT32     sbgStreamBufferWriteSizeT32BE
    #define sbgStreamBufferWriteSizeT64     sbgStreamBufferWriteSizeT64BE

    #define sbgStreamBufferWriteFloat       sbgStreamBufferWriteFloatBE
    #define sbgStreamBufferWriteDouble      sbgStreamBufferWriteDoubleBE

    #define sbgStreamBufferReadString       sbgStreamBufferReadStringBE
    #define sbgStreamBufferWriteString      sbgStreamBufferWriteStringBE
#else
    /*!
     *  The platform is a little endian one so default methods should use little endian byte order.
     */
    #define sbgStreamBufferReadUint16       sbgStreamBufferReadUint16LE
    #define sbgStreamBufferReadInt16        sbgStreamBufferReadInt16LE

    #define sbgStreamBufferReadUint24       sbgStreamBufferReadUint24LE
    #define sbgStreamBufferReadInt24        sbgStreamBufferReadInt24LE

    #define sbgStreamBufferReadUint32       sbgStreamBufferReadUint32LE
    #define sbgStreamBufferReadInt32        sbgStreamBufferReadInt32LE

    #define sbgStreamBufferReadUint40       sbgStreamBufferReadUint40LE
    #define sbgStreamBufferReadInt40        sbgStreamBufferReadInt40LE

    #define sbgStreamBufferReadUint48       sbgStreamBufferReadUint48LE
    #define sbgStreamBufferReadInt48        sbgStreamBufferReadInt48LE

    #define sbgStreamBufferReadUint56       sbgStreamBufferReadUint56LE
    #define sbgStreamBufferReadInt56        sbgStreamBufferReadInt56LE

    #define sbgStreamBufferReadUint64       sbgStreamBufferReadUint64LE
    #define sbgStreamBufferReadInt64        sbgStreamBufferReadInt64LE

    #define sbgStreamBufferReadSizeT32      sbgStreamBufferReadSizeT32LE
    #define sbgStreamBufferReadSizeT64      sbgStreamBufferReadSizeT64LE

    #define sbgStreamBufferReadFloat        sbgStreamBufferReadFloatLE
    #define sbgStreamBufferReadDouble       sbgStreamBufferReadDoubleLE

    #define sbgStreamBufferWriteUint16      sbgStreamBufferWriteUint16LE
    #define sbgStreamBufferWriteInt16       sbgStreamBufferWriteInt16LE

    #define sbgStreamBufferWriteUint24      sbgStreamBufferWriteUint24LE
    #define sbgStreamBufferWriteInt24       sbgStreamBufferWriteInt24LE

    #define sbgStreamBufferWriteUint32      sbgStreamBufferWriteUint32LE
    #define sbgStreamBufferWriteInt32       sbgStreamBufferWriteInt32LE

    #define sbgStreamBufferWriteUint64      sbgStreamBufferWriteUint64LE
    #define sbgStreamBufferWriteInt64       sbgStreamBufferWriteInt64LE

    #define sbgStreamBufferWriteSizeT32     sbgStreamBufferWriteSizeT32LE
    #define sbgStreamBufferWriteSizeT64     sbgStreamBufferWriteSizeT64LE

    #define sbgStreamBufferWriteFloat       sbgStreamBufferWriteFloatLE
    #define sbgStreamBufferWriteDouble      sbgStreamBufferWriteDoubleLE

    #define sbgStreamBufferReadString       sbgStreamBufferReadStringLE
    #define sbgStreamBufferWriteString      sbgStreamBufferWriteStringLE
#endif

/*!
 * Some methods are common between big and little endian.
 * This definitions just unify the API.
 */
#define sbgStreamBufferReadUint8LE      sbgStreamBufferReadUint8
#define sbgStreamBufferReadInt8LE       sbgStreamBufferReadInt8
#define sbgStreamBufferReadBooleanLE    sbgStreamBufferReadBoolean
#define sbgStreamBufferReadBufferLE     sbgStreamBufferReadBuffer

#define sbgStreamBufferWriteUint8LE     sbgStreamBufferWriteUint8
#define sbgStreamBufferWriteInt8LE      sbgStreamBufferWriteInt8
#define sbgStreamBufferWriteBooleanLE   sbgStreamBufferWriteBoolean
#define sbgStreamBufferWriteBufferLE    sbgStreamBufferWriteBuffer

#define sbgStreamBufferReadUint8BE      sbgStreamBufferReadUint8
#define sbgStreamBufferReadInt8BE       sbgStreamBufferReadInt8
#define sbgStreamBufferReadBooleanBE    sbgStreamBufferReadBoolean
#define sbgStreamBufferReadBufferBE     sbgStreamBufferReadBuffer

#define sbgStreamBufferWriteUint8BE     sbgStreamBufferWriteUint8
#define sbgStreamBufferWriteInt8BE      sbgStreamBufferWriteInt8
#define sbgStreamBufferWriteBooleanBE   sbgStreamBufferWriteBoolean
#define sbgStreamBufferWriteBufferBE    sbgStreamBufferWriteBuffer

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Stream buffer modes.
 */
typedef enum _SbgSBMode
{
    SB_MODE_READ,                       /*!< This stream buffer can perform read operations. */
    SB_MODE_WRITE                       /*!< This stream buffer can perform write operations. */
} SbgSBMode;

/*!
 * Enum used to define all seek modes
 */
typedef enum _SbgSBSeekOrigin
{
    SB_SEEK_SET,                            /*!< The offset is referenced to the begining of the stream. */
    SB_SEEK_CUR_INC,                        /*!< The offset is referenced to the current cursor position and increment the current cursor. */
    SB_SEEK_CUR_DEC,                        /*!< The offset is referenced to the current cursor position and decrement the current cursor. */
    SB_SEEK_END                             /*!< The offset is referenced to the end of the stream. */
} SbgSBSeekOrigin;

/*!
 * Defines a stream buffer.
 */
typedef struct _SbgStreamBuffer
{
    SbgSBMode            modes;             /*!< Defines the stream buffer modes (read/write). */
    size_t               bufferSize;        /*!< Size in bytes of the linked buffer. */
    uint8_t             *pBufferPtr;        /*!< Pointer to the buffer linked with this stream. */
    uint8_t             *pCurrentPtr;       /*!< Current pointer within the buffer. */
    SbgErrorCode         errorCode;         /*!< Current error code on stream buffer. */
} SbgStreamBuffer;

//----------------------------------------------------------------------//
//- Common operations methods                                          -//
//----------------------------------------------------------------------//

/*!
 * Initialize a stream buffer for both read and write operations and link it to a buffer.
 *
 * \param[in]   pHandle                                 Handle on an allocated stream buffer.
 * \param[in]   pLinkedBuffer                           Pointer on an allocated buffer to link with this stream.
 * \param[in]   bufferSize                              Size in bytes of the linked buffer.
 * \return                                              SBG_NO_ERROR if the stream buffer has been initialized successfully.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferInitForWrite(SbgStreamBuffer *pHandle, void *pLinkedBuffer, size_t bufferSize)
{
    assert(pHandle);
    assert(pLinkedBuffer);

    //
    // Initialize stream parameters
    //
    pHandle->modes = SB_MODE_WRITE;
    pHandle->bufferSize = bufferSize;
    pHandle->errorCode = SBG_NO_ERROR;

    //
    // Initialize the buffer
    //
    pHandle->pBufferPtr = (uint8_t*)pLinkedBuffer;
    pHandle->pCurrentPtr = (uint8_t*)pLinkedBuffer;

    //
    // For now, we don't handle any error, maybe we could add checks in debug mode only
    //
    return SBG_NO_ERROR;
}

/*!
 * Initialize a stream buffer for both read and write operations and link it to a buffer.
 *
 * \param[in]   pHandle                                 Handle on an allocated stream buffer.
 * \param[in]   pLinkedBuffer                           Pointer on an allocated buffer to link with this stream.
 * \param[in]   bufferSize                              Size in bytes of the linked buffer.
 * \return                                              SBG_NO_ERROR if the stream buffer has been initialized successfully.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferInitForRead(SbgStreamBuffer *pHandle, const void *pLinkedBuffer, size_t bufferSize)
{
    assert(pHandle);
    assert(pLinkedBuffer);

    //
    // Initialize stream parameters
    //
    pHandle->modes = SB_MODE_READ;
    pHandle->bufferSize = bufferSize;
    pHandle->errorCode = SBG_NO_ERROR;

    //
    // Initialize the buffer
    //
    pHandle->pBufferPtr = (uint8_t*)pLinkedBuffer;
    pHandle->pCurrentPtr = (uint8_t*)pLinkedBuffer;

    //
    // For now, we don't handle any error, maybe we could add checks in debug mode only
    //
    return SBG_NO_ERROR;
}

/*!
 * Return the error code that has occurred on the last stream buffer operation.
 *
 * \param[in]   pHandle                 Pointer to a valid Stream Buffer handle
 * \return                              Last stream buffer error code
 */
SBG_INLINE SbgErrorCode sbgStreamBufferGetLastError(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Return error code
    //
    return pHandle->errorCode;
}

/*!
 * Clear the last error code that has occurred on the last stream buffer operation.
 *
 * \param[in]   pHandle                 Pointer to a valid Stream Buffer handle
 */
SBG_INLINE void sbgStreamBufferClearLastError(SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Return error code
    //
    pHandle->errorCode = SBG_NO_ERROR;
}

/*!
 * Returns the size in bytes of this stream.
 *
 * The size is the linked buffer total size in bytes.
 * For example, for a SbgStreamBuffer linked with a buffer of 256 bytes,
 * this method will always returns 256 even if no data has been written or read.
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \return                              The allocated size of the linked buffer in bytes.
 */
SBG_INLINE size_t sbgStreamBufferGetSize(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Return the linked buffer size
    //
    return pHandle->bufferSize;
}

/*!
 * Returns the length in bytes of this stream.
 *
 * The length is computed using the current cursor position.
 * If no data has been read or written, this method will return 0.
 * If 4 uint32_t has been written, it should return 16.
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \return                              The current cursor position in bytes.
 */
SBG_INLINE size_t sbgStreamBufferGetLength(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Return the number of bytes between the begin of the stream and the current pointer
    //
    return ((size_t)pHandle->pCurrentPtr - (size_t)pHandle->pBufferPtr);
}

/*!
 * Returns the available space in this stream.
 *
 * The available space is just the delta between the linked buffer size
 * and the current buffer length (cursor position).
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \return                              The space available in this stream buffer in bytes.
 */
SBG_INLINE size_t sbgStreamBufferGetSpace(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Return the space left in bytes
    //
    return sbgStreamBufferGetSize(pHandle) - sbgStreamBufferGetLength(pHandle);
}

/*!
 * Move the current cursor position.
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \param[in]   offset                  Offset in bytes to apply (only positive).
 * \param[in]   origin                  Origin reference point to apply the offset from.
 * \return                              SBG_NO_ERROR if the stream current cursor position has been moved.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferSeek(SbgStreamBuffer *pHandle, size_t offset, SbgSBSeekOrigin origin)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // According to the origin reference point
        //
        switch (origin)
        {
        case SB_SEEK_SET:
            pHandle->pCurrentPtr = pHandle->pBufferPtr + offset;
            break;
        case SB_SEEK_CUR_INC:
            pHandle->pCurrentPtr += offset;
            break;
        case SB_SEEK_CUR_DEC:
            pHandle->pCurrentPtr -= offset;
            break;
        case SB_SEEK_END:
            pHandle->pCurrentPtr = pHandle->pBufferPtr + (pHandle->bufferSize - offset);
            break;
        default:
            pHandle->errorCode = SBG_INVALID_PARAMETER;
            SBG_LOG_ERROR(pHandle->errorCode, "Invalid origin parameter");
        }

        //
        // Make sure that no error has occurred
        //
        if (pHandle->errorCode == SBG_NO_ERROR)
        {
            //
            // Test if the current ptr is still within the buffer bounds
            //
            if (pHandle->pCurrentPtr < pHandle->pBufferPtr)
            {
                //
                // We are before the buffer so clamp to the begining of the buffer and raise an error
                //
                pHandle->pCurrentPtr = pHandle->pBufferPtr;
                pHandle->errorCode = SBG_BUFFER_OVERFLOW;
            }
            else if (pHandle->pCurrentPtr > pHandle->pBufferPtr + pHandle->bufferSize)
            {
                //
                // We are after the buffer so clamp to the end of the buffer and raise an error
                //
                pHandle->pCurrentPtr = pHandle->pBufferPtr + pHandle->bufferSize;
                pHandle->errorCode = SBG_BUFFER_OVERFLOW;
            }
        }
    }

    return pHandle->errorCode;
}

/*!
 * Returns the current offset in bytes from the beginning of the stream.
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \return                              Current offset in bytes from the beginning.
 */
SBG_INLINE size_t sbgStreamBufferTell(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    return (size_t)pHandle->pCurrentPtr - (size_t)pHandle->pBufferPtr;
}

/*!
 * Returns a pointer on the internal buffer.
 *
 * \param[in]   pHandle                 Valid handle on a stream buffer.
 * \return                              Pointer on the begining of the internal buffer.
 */
SBG_INLINE void *sbgStreamBufferGetLinkedBuffer(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    return pHandle->pBufferPtr;
}

/*!
 *  Returns a pointer on the internal buffer at the current cursor.
 *
 *  \param[in]  pHandle                 Valid handle on a stream buffer.
 *  \return                             Pointer on the current cursor of the internal buffer.
 */
SBG_INLINE void *sbgStreamBufferGetCursor(const SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    return pHandle->pCurrentPtr;
}

//----------------------------------------------------------------------//
//- Read operations methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Read an int8_t from a stream buffer.
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int8_t sbgStreamBufferReadInt8(SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int8_t))
        {
            //
            // Read the byte
            //
            return *((int8_t*)(pHandle->pCurrentPtr++));
        }
        else
        {
            //
            // We have a buffer overflow
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0;
}

/*!
 * Read an uint8_t from a stream buffer.
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint8_t sbgStreamBufferReadUint8(SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8_t))
        {
            //
            // Read the byte
            //
            return *((uint8_t*)(pHandle->pCurrentPtr++));
        }
        else
        {
            //
            // We have a buffer overflow
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0;
}

/*!
 * Read a boolean from a stream buffer.
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or false if we have an error.
 */
SBG_INLINE bool sbgStreamBufferReadBoolean(SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8_t))
        {
            //
            // Read the byte and check if the value is different than zero or not
            //
            if (*((uint8_t*)(pHandle->pCurrentPtr++)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            //
            // We have a buffer overflow
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return false
    //
    return false;
}

/*!
 * Read a buffer from a stream buffer.
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \param[out]  pBuffer             Allocated buffer used to hold read data.
 * \param[in]   numBytesToRead      Number of bytes to read from the stream buffer and to store in pBuffer.
 * \return                          SBG_NO_ERROR if the data has been read.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferReadBuffer(SbgStreamBuffer *pHandle, void *pBuffer, size_t numBytesToRead)
{
    assert(pHandle);
    assert((pBuffer) || (numBytesToRead == 0));

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if enough bytes in stream
        //
        if (sbgStreamBufferGetSpace(pHandle) >= numBytesToRead)
        {
            //
            // Copy from the stream buffer to the output buffer
            //
            memcpy(pBuffer, pHandle->pCurrentPtr, numBytesToRead);

            //
            // Update the current pointer
            //
            pHandle->pCurrentPtr += numBytesToRead;
        }
        else
        {
            //
            // Not enough data in stream
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    return pHandle->errorCode;
}

//----------------------------------------------------------------------//
//- Write operations methods                                           -//
//----------------------------------------------------------------------//

/*!
 * Write an int8_t into a stream buffer
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt8(SbgStreamBuffer *pHandle, int8_t value)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int8_t))
        {
            //
            // Write each byte
            //
            *(pHandle->pCurrentPtr++) = (int8_t)(value);
        }
        else
        {
            //
            // We are accessing a data that is outside the stream buffer
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    return pHandle->errorCode;
}

/*!
 * Write an uint8_t into a stream buffer
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint8(SbgStreamBuffer *pHandle, uint8_t value)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8_t))
        {
            //
            // Write each byte
            //
            *(pHandle->pCurrentPtr++) = (uint8_t)(value);
        }
        else
        {
            //
            // We are accessing a data that is outside the stream buffer
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    return pHandle->errorCode;
}

/*!
 * Write a boolean into a stream buffer
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteBoolean(SbgStreamBuffer *pHandle, bool value)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8_t))
        {
            //
            // Write the boolean as an uint8_t value (1 byte)
            //
            if (value)
            {
                *(pHandle->pCurrentPtr++) = 1;
            }
            else
            {
                *(pHandle->pCurrentPtr++) = 0;
            }
        }
        else
        {
            //
            // We are accessing a data that is outside the stream buffer
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    return pHandle->errorCode;
}

/*!
 * Write a buffer to a stream buffer.
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[out]  pBuffer             Buffer to write into the stream buffer.
 * \param[in]   numBytesToWrite     Number of bytes to write to the stream buffer.
 * \return                          SBG_NO_ERROR if the data has been written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteBuffer(SbgStreamBuffer *pHandle, const void *pBuffer, size_t numBytesToWrite)
{
    assert(pHandle);
    assert((pBuffer) || (numBytesToWrite == 0));

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= numBytesToWrite)
        {
            //
            // Copy from the stream buffer to the output buffer
            //
            memcpy(pHandle->pCurrentPtr, pBuffer, numBytesToWrite);

            //
            // Update the current pointer
            //
            pHandle->pCurrentPtr += numBytesToWrite;
        }
        else
        {
            //
            // We are accessing a data that is outside the stream buffer
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    return pHandle->errorCode;
}

#endif // SBG_STREAM_BUFFER_COMMON_H
