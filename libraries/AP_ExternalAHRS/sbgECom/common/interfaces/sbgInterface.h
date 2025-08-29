/*!
 * \file            sbgInterface.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            10 December 2012
 *
 * \brief           This file implements the base interface for all Serial and Ethernet ports.
 *
 * An interface is used to provide a common API for both serial and Ethernet ports.
 * An interface can be opened/closed and some data can be written or read from it.
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

#ifndef SBG_INTERFACE_H
#define SBG_INTERFACE_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

/* sbgCommonLib headers */
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Constant definitions                                               -//
//----------------------------------------------------------------------//

#define SBG_IF_NAME_MAX_SIZE        (48)            /*!< Maximum size in bytes for the interface name string */

/*!
 * Type values reserved for standard interface types.
 */
#define SBG_IF_TYPE_UNKNOW          (0)             /*!< The interface type is not defined. */
#define SBG_IF_TYPE_SERIAL          (1)             /*!< The interface is a serial com port. */
#define SBG_IF_TYPE_ETH_UDP         (2)             /*!< The interface is an UDP one. */
#define SBG_IF_TYPE_ETH_TCP_IP      (3)             /*!< The interface is an TCP/IP one. */
#define SBG_IF_TYPE_FILE            (4)             /*!< The interface is a file. */
#define SBG_IF_TYPE_LAST_RESERVED   (999)           /*!< Last reserved value for standard types. */

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgInterface SbgInterface;

/*!
 * Handle that stores the internal interface handle (ie Serial or Ethernet)
 */
typedef void* SbgInterfaceHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Method to implement that close and destroy an interface.
 *
 * \param[in]   pInterface                              Interface instance.
 * \return                                              SBG_NO_ERROR if the interface has been closed successfully.
 */
typedef SbgErrorCode (*SbgInterfaceDestroyFunc)(SbgInterface *pInterface);

/*!
 * Method to implement to write a buffer to an interface.
 *
 * This method should return an error only if all bytes were not written successfully.
 * If you try to write zero byte, the method shouldn't return any error.
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   pBuffer                                 Pointer on an allocated buffer that contains the data to write
 * \param[in]   bytesToWrite                            Number of bytes we would like to write (can be zero).
 * \return                                              SBG_NO_ERROR if exactly bytesToWrite have been written successfully.
 */
typedef SbgErrorCode (*SbgInterfaceWriteFunc)(SbgInterface *pInterface, const void *pBuffer, size_t bytesToWrite);

/*!
 * Method to implement to read data from an interface.
 *
 * This method returns an error only if there is a 'low level' error on the interface.
 * If no byte is read at all or less bytes than bytesToRead, this method returns SBG_NO_ERROR.
 * You have to check pReadBytes field to know the number of bytes actually read.
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   pBuffer                                 Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]  pReadBytes                              Returns the number of bytes actually read (can be zero and up to bytesToRead).
 * \param[in]   bytesToRead                             Maximum number of bytes to try to read on the interface.
 * \return                                              SBG_NO_ERROR if zero or some bytes have been read successfully.
 */
typedef SbgErrorCode (*SbgInterfaceReadFunc)(SbgInterface *pInterface, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

/*!
 * Make an interface flush pending input and/or output data.
 *
 * If flags include SBG_IF_FLUSH_INPUT, all pending input data is discarded.
 * If flags include SBG_IF_FLUSH_OUTPUT, the function blocks until all output data has been written out.
 *
 * WARNING: The method has no action if not applicable for a type of interface
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   flags                                   Combination of the SBG_IF_FLUSH_INPUT and SBG_IF_FLUSH_OUTPUT flags.
 * \return                                              SBG_NO_ERROR if successful.
 */
typedef SbgErrorCode (*SbgInterfaceFlushFunc)(SbgInterface *pInterface, uint32_t flags);

/*!
 * Change an interface input and output speed in bps (bit per second)
 *
 * This method will try to change the speed immediately even if there are
 * pending bytes in the send buffer.
 *
 * If you would like to make sure that all bytes in the Tx buffer have been
 * sent before changing the speed, please flush the interface before.
 *
 * WARNING: The method has no action if not applicable for a type of interface
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   speed                                   The new interface speed to set in bps.
 * \return                                              SBG_NO_ERROR if successful.
 */
typedef SbgErrorCode (*SbgInterfaceSetSpeed)(SbgInterface *pInterface, uint32_t speed);

/*!
 * Returns the current interface baud rate in bps (bit per second)
 *
 * WARNING: The method will returns zero if not applicable for a type of interface
 *
 * \param[in]   pInterface                              Interface instance.
 * \return                                              The current interface baud rate in bps or zero if not applicable.
 */
typedef uint32_t (*SbgInterfaceGetSpeed)(const SbgInterface *pInterface);

/*!
 * Compute and return the delay needed by the interface to transmit / receive X number of bytes.
 *
 * WARNING: The method will returns zero if not applicable for a type of interface.
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   numBytes                                The number of bytes to transmit / receive to evaluate the needed delay.
 * \return                                              The expected delay in us needed to transmit / receive the specified number of bytes or 0 if not applicable.
 */
typedef uint32_t (*SbgInterfaceGetDelayFunc)(const SbgInterface *pInterface, size_t numBytes);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 *
 * The interface class is designed to allow custom user implementations. The type member stores
 * a type identifier allowing the identification of the underlying type, including custom
 * implementations. Standard interfaces provided by this library use types from 1 up to
 * and including SBG_IF_TYPE_LAST_RESERVED. Greater values are intended to identify custom
 * types that are normally specific to the project using this library. The value 0 identifies
 * an unknown interface type, usually indicating that the interface was not correctly initialized.
 */
struct _SbgInterface
{
    SbgInterfaceHandle           handle;                            /*!< Internal interface handle used to access the media. */
    uint32_t                     type;                              /*!< Opaque interface type. */
    char                         name[SBG_IF_NAME_MAX_SIZE];        /*!< The interface name as passed during the creation */

    SbgInterfaceDestroyFunc      pDestroyFunc;                      /*!< Optional method used to destroy an interface. */
    SbgInterfaceWriteFunc        pWriteFunc;                        /*!< Optional method used to write some data to this interface. */
    SbgInterfaceReadFunc         pReadFunc;                         /*!< Optional method used to read some data to this interface. */
    SbgInterfaceFlushFunc        pFlushFunc;                        /*!< Optional method used to make this interface flush all pending data. */
    SbgInterfaceSetSpeed         pSetSpeedFunc;                     /*!< Optional method used to set the interface speed in bps. */
    SbgInterfaceGetSpeed         pGetSpeedFunc;                     /*!< Optional method used to retrieve the interface speed in bps. */
    SbgInterfaceGetDelayFunc     pDelayFunc;                        /*!< Optional method used to compute an expected delay to transmit/receive X bytes */
};

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Initialize an interface instance to zero.
 *
 * \param[in]   pInterface                              The interface instance.
 */
SBG_COMMON_LIB_API void sbgInterfaceZeroInit(SbgInterface *pInterface);

/*!
 * Write some data to an interface.
 *
 * This method should return an error only if all bytes were not written successfully.
 * If you try to write zero byte, the method shouldn't return any error.
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   pBuffer                                 Pointer on an allocated buffer that contains the data to write
 * \param[in]   bytesToWrite                            Number of bytes we would like to write (can be zero).
 * \return                                              SBG_NO_ERROR if exactly bytesToWrite have been written successfully.
 *                                                      SBG_INVALID_PARAMETER if the interface doesn't support write operations.
 */
SBG_INLINE SbgErrorCode sbgInterfaceWrite(SbgInterface *pInterface, const void *pBuffer, size_t bytesToWrite)
{
    SbgErrorCode    errorCode;

    assert(pInterface);
    assert(pBuffer);

    if (pInterface->pWriteFunc)
    {
        errorCode = pInterface->pWriteFunc(pInterface, pBuffer, bytesToWrite);
    }
    else
    {
        errorCode = SBG_INVALID_PARAMETER;
    }

    return errorCode;
}

/*!
 * Try to read some data from an interface.
 *
 * This method returns an error only if there is a 'low level' error on the interface.
 * If no byte is read at all or less bytes than bytesToRead, this method returns SBG_NO_ERROR.
 * You have to check pReadBytes field to know the number of bytes actually read.
 *
 * \param[in]   pInterface                              Interface instance.
 * \param[in]   pBuffer                                 Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]  pReadBytes                              Returns the number of bytes actually read (can be zero and up to bytesToRead).
 * \param[in]   bytesToRead                             Maximum number of bytes to try to read on the interface.
 * \return                                              SBG_NO_ERROR if zero or some bytes have been read successfully.
 *                                                      SBG_INVALID_PARAMETER if the interface doesn't support read operations.
 */
SBG_INLINE SbgErrorCode sbgInterfaceRead(SbgInterface *pInterface, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
    SbgErrorCode    errorCode;

    assert(pInterface);
    assert(pBuffer);
    assert(pReadBytes);

    if (pInterface->pReadFunc)
    {
        errorCode = pInterface->pReadFunc(pInterface, pBuffer, pReadBytes, bytesToRead);
    }
    else
    {
        *pReadBytes = 0;
        errorCode   = SBG_INVALID_PARAMETER;
    }

    return errorCode;
}

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif // SBG_INTERFACE_H
