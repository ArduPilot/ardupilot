/*!
 * \file            sbgStreamBufferLE.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            17 February 2015
 *
 * \brief           Specific method of stream buffer for little endian readings/writings.
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

#ifndef SBG_STREAM_BUFFER_LE_H
#define SBG_STREAM_BUFFER_LE_H

#include "sbgStreamBufferCommon.h"

//----------------------------------------------------------------------//
//- Read operations methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Read an int16_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int16_t sbgStreamBufferReadInt16LE(SbgStreamBuffer *pHandle)
{
    int16_t bytesValues[2];

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                bytesValues[0] = *((int16_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int16_t);

                return bytesValues[0];
            #else
                //
                // Read the each bytes
                //
                bytesValues[0] = *(pHandle->pCurrentPtr++);
                bytesValues[1] = *(pHandle->pCurrentPtr++);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return bytesValues[1] | (bytesValues[0] << 8);
                #else
                    return bytesValues[0] | (bytesValues[1] << 8);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint16_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint16_t sbgStreamBufferReadUint16LE(SbgStreamBuffer *pHandle)
{
    uint16_t bytesValues[2];

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                bytesValues[0] = *((uint16_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint16_t);

                return bytesValues[0];
            #else
                //
                // Read the each bytes
                //
                bytesValues[0] = *(pHandle->pCurrentPtr++);
                bytesValues[1] = *(pHandle->pCurrentPtr++);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return bytesValues[1] | (bytesValues[0] << 8);
                #else
                    return bytesValues[0] | (bytesValues[1] << 8);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int24 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int32_t sbgStreamBufferReadInt24LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToInt32     value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[2] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[1] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 24 bits
            //
            return value.value >> (32-24);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint24 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint32_t sbgStreamBufferReadUint24LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToUint32    value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[2] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[1] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 24 bits
            //
            return value.value >> (32-24);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int32_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int32_t sbgStreamBufferReadInt32LE(SbgStreamBuffer *pHandle)
{
    int32_t bytesValues[4];

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                bytesValues[0] = *((int32_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int32_t);

                return bytesValues[0];
            #else
                //
                // Read the each bytes
                //
                bytesValues[0] = *(pHandle->pCurrentPtr++);
                bytesValues[1] = *(pHandle->pCurrentPtr++);
                bytesValues[2] = *(pHandle->pCurrentPtr++);
                bytesValues[3] = *(pHandle->pCurrentPtr++);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return bytesValues[3] | (bytesValues[2] << 8) | (bytesValues[1] << 16) | (bytesValues[0] << 24);
                #else
                    return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint32_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint32_t sbgStreamBufferReadUint32LE(SbgStreamBuffer *pHandle)
{
    uint32_t bytesValues[4];

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                bytesValues[0] = *((uint32_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint32_t);

                return bytesValues[0];
            #else
                //
                // Read the each bytes
                //
                bytesValues[0] = *(pHandle->pCurrentPtr++);
                bytesValues[1] = *(pHandle->pCurrentPtr++);
                bytesValues[2] = *(pHandle->pCurrentPtr++);
                bytesValues[3] = *(pHandle->pCurrentPtr++);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return bytesValues[3] | (bytesValues[2] << 8) | (bytesValues[1] << 16) | (bytesValues[0] << 24);
                #else
                    return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int40 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int64_t sbgStreamBufferReadInt40LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToInt64     value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 5*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[4] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[3] = *(pHandle->pCurrentPtr++);    // MSB
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 40 bits
            //
            return value.value >> (64-40);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint40 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int64_t sbgStreamBufferReadUint40LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToUint64    value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 5*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[4] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[3] = *(pHandle->pCurrentPtr++);    // MSB
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 40 bits
            //
            return value.value >> (64-40);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int48 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int64_t sbgStreamBufferReadInt48LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToInt64     value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 6*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[5] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[2] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 48 bits
            //
            return value.value >> (64-48);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint48 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint64_t sbgStreamBufferReadUint48LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToUint64    value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 6*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[5] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[2] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 48 bits
            //
            return value.value >> (64-48);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int56 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int64_t sbgStreamBufferReadInt56LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToInt64     value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 7*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[6] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[1] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 56 bits
            //
            return value.value >> (64-56);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an uint56 from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint64_t sbgStreamBufferReadUint56LE(SbgStreamBuffer *pHandle)
{
    SbgUint8ToUint64    value;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 7*sizeof(uint8_t))
        {
            //
            // Make sure the value is zero init
            //
            value.value = 0;

            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                //
                // Read the each bytes
                //
                value.buffer[6] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[1] = *(pHandle->pCurrentPtr++);
                value.buffer[0] = *(pHandle->pCurrentPtr++);    // MSB
            #else
                //
                // Read the each bytes
                //
                value.buffer[1] = *(pHandle->pCurrentPtr++);    // LSB
                value.buffer[2] = *(pHandle->pCurrentPtr++);
                value.buffer[3] = *(pHandle->pCurrentPtr++);
                value.buffer[4] = *(pHandle->pCurrentPtr++);
                value.buffer[5] = *(pHandle->pCurrentPtr++);
                value.buffer[6] = *(pHandle->pCurrentPtr++);
                value.buffer[7] = *(pHandle->pCurrentPtr++);    // MSB
            #endif

            //
            // Shift the value to handle the sign correctly for a 56 bits
            //
            return value.value >> (64-56);
        }
        else
        {
            //
            // We have a buffer overflow so return 0
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
 * Read an int64_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE int64_t sbgStreamBufferReadInt64LE(SbgStreamBuffer *pHandle)
{
    int64_t lowPart;
    int64_t highPart;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                lowPart = *((int64_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int64_t);

                return lowPart;
            #else
                //
                // Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
                //
                lowPart = sbgStreamBufferReadUint32LE(pHandle);
                highPart = sbgStreamBufferReadUint32LE(pHandle);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return (lowPart << 32) | highPart;
                #else
                    return lowPart | (highPart << 32);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0ll;
}

/*!
 * Read an uint64_t from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE uint64_t sbgStreamBufferReadUint64LE(SbgStreamBuffer *pHandle)
{
    uint64_t lowPart;
    uint64_t highPart;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                // Read the current value
                //
                lowPart = *((uint64_t*)pHandle->pCurrentPtr);

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint64_t);

                return lowPart;
            #else
                //
                // Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
                //
                lowPart = sbgStreamBufferReadUint32LE(pHandle);
                highPart = sbgStreamBufferReadUint32LE(pHandle);

                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    return (lowPart << 32) | highPart;
                #else
                    return lowPart | (highPart << 32);
                #endif
            #endif
        }
        else
        {
            //
            // We have a buffer overflow so return 0
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0ull;
}

/*!
 * Read a size_t from a stream buffer that has been stored in a uint32_t (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE size_t sbgStreamBufferReadSizeT32LE(SbgStreamBuffer *pHandle)
{
    assert(pHandle);

    //
    // Just call the read method for uint32_t
    // We assume that a size_t is at least 32 bits on all platforms
    //
    return (size_t)sbgStreamBufferReadUint32LE(pHandle);
}

/*!
 * Read a size_t from a stream buffer that has been stored in a uint64_t (Little endian version).

 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE size_t sbgStreamBufferReadSizeT64LE(SbgStreamBuffer *pHandle)
{
    uint64_t    size;

    assert(pHandle);

    //
    // Just call the read method for uint64_t
    //
    size = sbgStreamBufferReadUint64LE(pHandle);

    //
    // Make sure the read size can fit in the size_t in size_t is 32 bits
    //
    assert((sizeof(size_t) == 8) || ((sizeof(size_t) == 4) && (size <= UINT32_MAX)));

    //
    // Return the read value
    //
    return (size_t)size;
}

/*!
 * Read an float from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE float sbgStreamBufferReadFloatLE(SbgStreamBuffer *pHandle)
{
    SbgFloatNint    floatInt;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(float))
        {
            //
            // Read the float as an uint32_t
            //
            floatInt.valU = sbgStreamBufferReadUint32LE(pHandle);

            //
            // Return the float using an union to avoid compiler cast
            //
            return floatInt.valF;
        }
        else
        {
            //
            // We have a buffer overflow so return 0
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0.0f;
}

/*!
 * Read an double from a stream buffer (Little endian version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports read operations.
 * \return                          The read value or 0 if we have an error.
 */
SBG_INLINE double sbgStreamBufferReadDoubleLE(SbgStreamBuffer *pHandle)
{
    SbgDoubleNint       doubleInt;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(double))
        {
            //
            // Read the float as an uint64_t
            //
            doubleInt.valU = sbgStreamBufferReadUint64LE(pHandle);

            //
            // Return the double using an union to avoid compiler cast
            //
            return doubleInt.valF;
        }
        else
        {
            //
            // We have a buffer overflow so return 0
            //
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
        }
    }

    //
    // If we are here, it means we have an error so return 0
    //
    return 0.0;
}

//----------------------------------------------------------------------//
//- Write operations methods                                           -//
//----------------------------------------------------------------------//

/*!
 * Write an int16_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt16LE(SbgStreamBuffer *pHandle, int16_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((int16_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int16_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                #endif
            #endif
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
 * Write an uint16_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint16LE(SbgStreamBuffer *pHandle, uint16_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((uint16_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint16_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                #endif
            #endif
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
 * Write an int24 into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt24LE(SbgStreamBuffer *pHandle, int32_t value)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Make sure that the value is within 24 bit bonds
        //
        if ( (value >= SBG_MIN_INT_24) && (value <= SBG_MAX_INT_24) )
        {
            //
            // Test if we can access this item
            //
            if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(int8_t))
            {
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                #endif
            }
            else
            {
                //
                // We are accessing a data that is outside the stream buffer
                //
                pHandle->errorCode = SBG_BUFFER_OVERFLOW;
            }
        }
        else
        {
            //
            // The input value is not within a 24 bit integer bounds
            //
            pHandle->errorCode = SBG_INVALID_PARAMETER;
        }
    }

    return pHandle->errorCode;
}

/*!
 * Write an uint24 into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint24LE(SbgStreamBuffer *pHandle, uint32_t value)
{
    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Make sure that the value is within 24 bit bonds
        //
        if (value <= SBG_MAX_UINT_24)
        {
            //
            // Test if we can access this item
            //
            if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(uint8_t))
            {
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                #endif
            }
            else
            {
                //
                // We are accessing a data that is outside the stream buffer
                //
                pHandle->errorCode = SBG_BUFFER_OVERFLOW;
            }
        }
        else
        {
            //
            // The input value is not within a 24 bit integer bounds
            //
            pHandle->errorCode = SBG_INVALID_PARAMETER;
        }
    }

    return pHandle->errorCode;
}

/*!
 * Write an int32_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt32LE(SbgStreamBuffer *pHandle, int32_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((int32_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int32_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                #endif
            #endif
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
 * Write an uint32_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint32LE(SbgStreamBuffer *pHandle, uint32_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((uint32_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint32_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                #endif
            #endif
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
 * Write an uint48 into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint48LE(SbgStreamBuffer *pHandle, uint64_t value)
{
    assert(pHandle);
    assert(value < ((uint64_t)1 << 48));

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // Test if we can access this item
        //
        if (sbgStreamBufferGetSpace(pHandle) >= 6 * sizeof(uint8_t))
        {
            //
            // Store data according to platform endianness
            //
            #if (SBG_CONFIG_BIG_ENDIAN == 1)
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value);
            #else
                *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
            #endif
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
 * Write an int64_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt64LE(SbgStreamBuffer *pHandle, int64_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((int64_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(int64_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 56);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 48);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 48);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 56);
                #endif
            #endif
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
 * Write an uint64_t into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint64LE(SbgStreamBuffer *pHandle, uint64_t value)
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
        if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64_t))
        {
            //
            // Test if the platform supports un-aligned access and if the endianness is the same
            //
            #if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
                //
                //  Write the value
                //
                *((uint64_t*)(pHandle->pCurrentPtr)) = value;

                //
                //  Increment the current pointer
                //
                pHandle->pCurrentPtr += sizeof(uint64_t);
            #else
                //
                // Store data according to platform endianness
                //
                #if (SBG_CONFIG_BIG_ENDIAN == 1)
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 56);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 48);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                #else
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 8);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 16);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 24);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 32);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 40);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 48);
                    *(pHandle->pCurrentPtr++) = (uint8_t)(value >> 56);
                #endif
            #endif
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
 * Write an size_t into a stream buffer as a uint32_t (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteSizeT32LE(SbgStreamBuffer *pHandle, size_t value)
{
    assert(pHandle);

    //
    // Make sure the provided size_t value doesn't exceed a uint32_t storage
    //
    assert(value <= UINT32_MAX);

    //
    // Call the write method to store a uint32_t
    //
    return sbgStreamBufferWriteUint32LE(pHandle, (uint32_t)value);
}

/*!
 * Write an size_t into a stream buffer as a uint64_t (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteSizeT64LE(SbgStreamBuffer *pHandle, size_t value)
{
    assert(pHandle);

    //
    // Call the write method to store a uint64_t
    //
    return sbgStreamBufferWriteUint64LE(pHandle, (uint64_t)value);
}

/*!
 * Write an float into a stream buffer (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteFloatLE(SbgStreamBuffer *pHandle, float value)
{
    SbgFloatNint    floatInt;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // We use an union to avoid compiler cast
        //
        floatInt.valF = value;

        //
        // Write this float as an uint32_t
        //
        return sbgStreamBufferWriteUint32LE(pHandle, floatInt.valU);
    }

    return pHandle->errorCode;
}

/*!
 * Write an double into a stream buffer. (Little Endian Version).
 *
 * \param[in]   pHandle             Valid stream buffer handle that supports write operations.
 * \param[in]   value               The value to write.
 * \return                          SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteDoubleLE(SbgStreamBuffer *pHandle, double value)
{
    SbgDoubleNint   doubleInt;

    assert(pHandle);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // We use an union to avoid compiler cast
        //
        doubleInt.valF = value;

        //
        // Write this float as an uint64_t
        //
        return sbgStreamBufferWriteUint64LE(pHandle, doubleInt.valU);
    }

    return pHandle->errorCode;
}

/*!
* Read a C String from a stream buffer (Little Endian Version).
*
* \param[in]    pHandle             Valid stream buffer handle that supports read operations.
* \param[out]   pString             Buffer that can hold the read NULL terminated C string.
* \param[in]    maxSize             Maximum number of bytes that can be stored in pString (including the NULL char).
* \return                           SBG_NO_ERROR if the string has been read successfully from the stream buffer.
*                                   SBG_BUFFER_OVERFLOW if the provided string isn't big enough to hold the read string
*/
SBG_INLINE SbgErrorCode sbgStreamBufferReadStringLE(SbgStreamBuffer *pHandle, char *pString, size_t maxSize)
{
    size_t          stringLength;

    assert(pHandle);
    assert(pString);
    assert(maxSize > 0);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // The C string are stored in a stream buffer with a 32 bit size length and then the buffer itself
        //
        stringLength = sbgStreamBufferReadSizeT32LE(pHandle);

        if (stringLength <= maxSize)
        {
            //
            // Read the string buffer itself
            //
            sbgStreamBufferReadBuffer(pHandle, pString, stringLength);
        }
        else
        {
            pHandle->errorCode = SBG_BUFFER_OVERFLOW;
            SBG_LOG_ERROR(pHandle->errorCode, "Trying to store a string of %zu bytes into a buffer of %zu bytes.", stringLength, maxSize);
        }
    }

    return pHandle->errorCode;
}

/*!
* Write a NULL terminated C String into a stream buffer (Little Endian Version).
*
* \param[in]    pHandle             Valid stream buffer handle that supports write operations.
* \param[in]    pString             NULL terminated C String to write to the stream buffer.
* \return                           SBG_NO_ERROR if the string has been written successfully to the stream buffer.
*/
SBG_INLINE SbgErrorCode sbgStreamBufferWriteStringLE(SbgStreamBuffer *pHandle, const char *pString)
{
    size_t  stringLength;

    assert(pHandle);
    assert(pString);

    //
    // Test if we haven't already an error
    //
    if (pHandle->errorCode == SBG_NO_ERROR)
    {
        //
        // We write C string using a 32 bit size_t as the string length including the NULL char
        // We should thus make sure the provided string isn't too big to fit in a 32 bits size_t
        //
        stringLength = strlen(pString) + 1;

        if (stringLength <= UINT32_MAX)
        {
            //
            // Write the string length
            //
            if (sbgStreamBufferWriteSizeT32LE(pHandle, stringLength) == SBG_NO_ERROR)
            {
                //
                // Write the string buffer itself
                //
                sbgStreamBufferWriteBuffer(pHandle, pString, stringLength);
            }
        }
        else
        {
            pHandle->errorCode = SBG_INVALID_PARAMETER;
            SBG_LOG_ERROR(pHandle->errorCode, "The provided string is too big to fit in a 32 bit size_t");
        }
    }

    return pHandle->errorCode;
}

#endif // SBG_STREAM_BUFFER_LE_H
