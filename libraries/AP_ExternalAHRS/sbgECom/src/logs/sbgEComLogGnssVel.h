/*!
 * \file            sbgEComLogGnssVel.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            09 May 2023
 *
 * \brief           GNSS velocity logs.
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

#ifndef SBG_ECOM_LOG_GNSS_VEL_H
#define SBG_ECOM_LOG_GNSS_VEL_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * \brief Structure that stores data for the SBG_ECOM_LOG_GPS#_VEL message.
 * 
 * This structure holds information about GPS velocity, including timestamp,
 * status, time of week, velocity components, and their accuracies.
 *
 * \note Some GNSS protocols, such as NMEA, cannot provide a full 3D velocity.
 * An invalid down velocity component should be indicated by using the maximum
 * standard deviation value of 9999 m/s.
 */
typedef struct _SbgEComLogGnssVel
{
    uint32_t        timeStamp;              /*!< Time in microseconds since the sensor power up. */
    uint32_t        status;                 /*!< GPS velocity status, type and bitmask. */
    uint32_t        timeOfWeek;             /*!< GPS time of week in milliseconds. */
    float           velocity[3];            /*!< GPS North, East, Down velocity in m/s. */
    float           velocityAcc[3];         /*!< GPS North, East, Down velocity 1 sigma accuracy in m/s (0 to 9999). */
    float           course;                 /*!< Track ground course in degrees (0 to 360). */
    float           courseAcc;              /*!< Course accuracy in degrees (0 to 180). */
} SbgEComLogGnssVel;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_VEL message and fill the corresponding structure.
 * 
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogGnssVelReadFromStream(SbgEComLogGnssVel *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//


#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_GNSS_VEL_H
