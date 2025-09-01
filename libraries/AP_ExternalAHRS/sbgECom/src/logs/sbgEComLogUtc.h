/*!
 * \file            sbgEComLogUtc.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            20 February 2013
 *
 * \brief           Parse logs used to report device UTC time.
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

#ifndef SBG_ECOM_LOG_UTC_H
#define SBG_ECOM_LOG_UTC_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * Internal clock alignment to PPS algorithm state.
 */
typedef enum _SbgEComClockState
{
    SBG_ECOM_CLOCK_STATE_ERROR              = 0,                        /*!< An error has occurred on the clock estimation. */
    SBG_ECOM_CLOCK_STATE_FREE_RUNNING       = 1,                        /*!< The clock is only based on the internal crystal using latest known clock bias and scale factor. */
    SBG_ECOM_CLOCK_STATE_STEERING           = 2,                        /*!< A PPS has been detected and the clock is converging to it. */
    SBG_ECOM_CLOCK_STATE_VALID              = 3                         /*!< The internal clock is converged to the PPS signal or is still considered to be accurate. */
} SbgEComClockState;

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_UTC_TIME message.
 */
typedef struct _SbgEComLogUtc
{
    uint32_t    timeStamp;                  /*!< Time in us since the sensor power up. */
    uint16_t    status;                     /*!< Clock state and UTC time status. */
    uint16_t    year;                       /*!< Year for example: 2013. */
    int8_t      month;                      /*!< Month in year [1 .. 12]. */
    int8_t      day;                        /*!< Day in month [1 .. 31]. */
    int8_t      hour;                       /*!< Hour in day [0 .. 23]. */
    int8_t      minute;                     /*!< Minute in hour [0 .. 59]. */
    int8_t      second;                     /*!< Second in minute [0 .. 60]. (60 is used only when a leap second is added) */
    int32_t     nanoSecond;                 /*!< Nanosecond of current second in ns. */
    uint32_t    gpsTimeOfWeek;              /*!< GPS time of week in ms. */
    float       clkBiasStd;                 /*!< Estimated internal clock bias standard deviation in seconds - set to NaN if not available. (added in 4.0) */
    float       clkSfErrorStd;              /*!< Estimated internal clock scale factor error standard deviation - set to NaN if not available. (added in 4.0) */
    float       clkResidualError;           /*!< Latest residual clock error from the GNSS PPS signal in seconds - set to NaN if not available. (added in 4.0) */
} SbgEComLogUtc;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_UTC_DATA message and fill the corresponding structure.
 * 
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogUtcReadFromStream(SbgEComLogUtc *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

/*!
 * Returns the clock alignment state.
 * 
 * \param[in]   pLogData                    Log instance.
 * \return                                  The clock status.
 */
SbgEComClockState sbgEComLogUtcGetClockState(const SbgEComLogUtc *pLogData);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_UTC_H
