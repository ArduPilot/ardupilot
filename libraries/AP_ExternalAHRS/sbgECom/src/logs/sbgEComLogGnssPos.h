/*!
 * \file            sbgEComLogGnssPos.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            09 May 2023
 *
 * \brief           GNSS position related logs.
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

#ifndef SBG_ECOM_LOG_GPS_POS_H
#define SBG_ECOM_LOG_GPS_POS_H

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
 * GNSS position types definitions.
 */
typedef enum _SbgEComGnssPosType
{
    SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION          = 0,                        /*!< No valid solution available. */
    SBG_ECOM_GNSS_POS_TYPE_UNKNOWN              = 1,                        /*!< An unknown solution type has been computed. */
    SBG_ECOM_GNSS_POS_TYPE_SINGLE               = 2,                        /*!< Single point solution position. */
    SBG_ECOM_GNSS_POS_TYPE_PSRDIFF              = 3,                        /*!< Standard Pseudorange Differential Solution (DGPS). */
    SBG_ECOM_GNSS_POS_TYPE_SBAS                 = 4,                        /*!< SBAS satellite used for differential corrections. */
    SBG_ECOM_GNSS_POS_TYPE_OMNISTAR             = 5,                        /*!< Omnistar VBS Position (L1 sub-meter). */
    SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT            = 6,                        /*!< Floating RTK ambiguity solution (20 cms RTK). */
    SBG_ECOM_GNSS_POS_TYPE_RTK_INT              = 7,                        /*!< Integer RTK ambiguity solution (2 cms RTK). */
    SBG_ECOM_GNSS_POS_TYPE_PPP_FLOAT            = 8,                        /*!< Precise Point Positioning with float ambiguities. */
    SBG_ECOM_GNSS_POS_TYPE_PPP_INT              = 9,                        /*!< Precise Point Positioning with fixed ambiguities. */
    SBG_ECOM_GNSS_POS_TYPE_FIXED                = 10                        /*!< Fixed location solution position. */
} SbgEComGnssPosType;

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_POS message.
 */
typedef struct _SbgEComLogGnssPos
{
    uint32_t        timeStamp;              /*!< Time in us since the sensor power up. */
    uint32_t        status;                 /*!< GPS position status, type and bitmask. */
    uint32_t        timeOfWeek;             /*!< GPS time of week in ms. */
    double          latitude;               /*!< Latitude in degrees, positive north. */
    double          longitude;              /*!< Longitude in degrees, positive east. */
    double          altitude;               /*!< Altitude above Mean Sea Level in meters. */
    float           undulation;             /*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
    float           latitudeAccuracy;       /*!< 1 sigma latitude accuracy in meters (0 to 9999). */
    float           longitudeAccuracy;      /*!< 1 sigma longitude accuracy in meters (0 to 9999). */
    float           altitudeAccuracy;       /*!< 1 sigma altitude accuracy in meters (0 to 9999). */
    uint8_t         numSvUsed;              /*!< Number of space vehicles used to compute the solution - set to 0xFF if not available. (added in 1.4) */
    uint16_t        baseStationId;          /*!< Base station id for differential corrections (0-4095) - set to 0xFFFF if differential are not used or not available. (added in 1.4). */
    uint16_t        differentialAge;        /*!< Differential correction age in 0.01 seconds - set to 0xFFFF if differential are not used or not available. (added in 1.4). */  
    uint8_t         numSvTracked;           /*!< Number of space vehicles tracked by the GNSS - set to 0xFF if not available. (added in 4.0) */
    uint32_t        statusExt;              /*!< Additional status for interference, spoofing and OSNMA (added in 4.0). */
} SbgEComLogGnssPos;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_POS message and fill the corresponding structure.
 * 
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogGnssPosReadFromStream(SbgEComLogGnssPos *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

/*!
 * Returns the GNSS position solution type.
 * 
 * Note: Method doesn't follow standard naming conventions because of legacy sbgEComLogGnssPosGetStatus method.
 * 
 * \param[in]   pLogData            Log instance.
 * \return                          The solution type.
 */
SbgEComGnssPosType sbgEComLogGnssPosGetType(const SbgEComLogGnssPos *pLogData);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_GNSS_POS_H
