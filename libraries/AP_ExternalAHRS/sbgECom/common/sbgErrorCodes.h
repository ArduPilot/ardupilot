/*!
 * \file            sbgErrorCodes.h
 * \ingroup         common
 * \author          SBG Systems
 * \date            17 March 2015
 *
 * \brief           Header file that defines all error codes for SBG Systems libraries.
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

#ifndef SBG_ERROR_CODES_H
#define SBG_ERROR_CODES_H

#ifdef __cplusplus
extern "C" {
#endif

// Local headers
#include "sbgDefines.h"

//----------------------------------------------------------------------//
//- Errors code definitions                                            -//
//----------------------------------------------------------------------//

/*!
 * List of errors and status common to all SBG Systems projects.
 */
typedef enum _SbgErrorCode
{
    SBG_NO_ERROR = 0,                       /*!< The operation was successfully executed. */
    SBG_ERROR,                              /*!< We have a generic error. */
    SBG_NULL_POINTER,                       /*!< A pointer is null. */
    SBG_INVALID_CRC,                        /*!< The received frame has an invalid CRC. */
    SBG_INVALID_FRAME,                      /*!< The received frame is invalid <br> */
                                            /*!<    We have received an unexpected frame (not the cmd we are waiting for or with an invalid data size.<br> */
                                            /*!<    This could be caused by a desync between questions and answers.<br> */
                                            /*!<    You should flush the serial port to fix this. */
    SBG_TIME_OUT,                           /*!< We have started to receive a frame but not the end. */
    SBG_WRITE_ERROR,                        /*!< All bytes hasn't been written. */
    SBG_READ_ERROR,                         /*!< All bytes hasn't been read. */
    SBG_BUFFER_OVERFLOW,                    /*!< A buffer is too small to contain so much data. */
    SBG_INVALID_PARAMETER,                  /*!< An invalid parameter has been found. */
    SBG_NOT_READY,                          /*!< A device isn't ready (Rx isn't ready for example). */
    SBG_MALLOC_FAILED,                      /*!< Failed to allocate a buffer. */
    SBG_CALIB_MAG_NOT_ENOUGH_POINTS,        /*!< Not enough points were available to perform magnetometers calibration. */
    SBG_CALIB_MAG_INVALID_TAKE,             /*!< The calibration procedure could not be properly executed due to insufficient precision. */
    SBG_CALIB_MAG_SATURATION,               /*!< Saturation were detected when attempt to calibrate magnetos. */
    SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE,    /*!< 2D calibration procedure could not be performed. */

    SBG_DEVICE_NOT_FOUND,                   /*!< A device couldn't be founded or opened PC only error code */
    SBG_OPERATION_CANCELLED,                /*!< An operation was canceled.  PC only error code*/
    SBG_NOT_CONTINUOUS_FRAME,               /*!< We have received a frame that isn't a continuous one. PC only error code*/

    SBG_INCOMPATIBLE_HARDWARE,              /*!< Hence valid; the command cannot be executed because of hardware incompatibility */
    SBG_INVALID_VERSION                     /*!< Incompatible version */
} SbgErrorCode;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Returns a human readable string from an error code enum value.
 * 
 * \param[in]   errorCode                   The errorCode to convert to a string.
 * \return                                  Read only corresponding string.
 */
SBG_COMMON_LIB_API const char *sbgErrorCodeToString(SbgErrorCode errorCode);

#ifdef __cplusplus
}
#endif

#endif  // SBG_ERROR_CODES_H
