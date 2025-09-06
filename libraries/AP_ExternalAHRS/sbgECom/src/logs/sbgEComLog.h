/*!
 * \file            sbgEComLog.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            06 February 2013
 *
 * \brief           Parse incoming sbgECom logs and store result in an union.
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
 * \defgroup    binaryLogs Binary Logs
 * \brief       All messages and logs that can be output by the device.
 */

#ifndef SBG_ECOM_LOG_H
#define SBG_ECOM_LOG_H

 // sbgCommonLib headers
#include <sbgCommon.h>

// Project headers
#include <sbgEComIds.h>

// Local headers
#include "sbgEComLogAirData.h"
#include "sbgEComLogEkf.h"
#include "sbgEComLogGnssPos.h"
#include "sbgEComLogGnssVel.h"
#include "sbgEComLogImu.h"
#include "sbgEComLogMag.h"
#include "sbgEComLogUtc.h"

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 *  Union used to store received logs data.
 */
typedef union _SbgEComLogUnion
{
    SbgEComLogImuShort              imuShort;           /*!< Stores data for the SBG_ECOM_LOG_IMU_SHORT message. */
    SbgEComLogEkfQuat               ekfQuatData;        /*!< Stores data for the SBG_ECOM_LOG_EKF_QUAT message. */
    SbgEComLogEkfNav                ekfNavData;         /*!< Stores data for the SBG_ECOM_LOG_EKF_NAV message. */
    SbgEComLogUtc                   utcData;            /*!< Stores data for the SBG_ECOM_LOG_UTC_TIME message. */
    SbgEComLogGnssPos               gpsPosData;         /*!< Stores data for the SBG_ECOM_LOG_GPS_POS message. */
    SbgEComLogGnssVel               gpsVelData;         /*!< Stores data for the SBG_ECOM_LOG_GPS#_VEL message. */
    SbgEComLogMag                   magData;            /*!< Stores data for the SBG_ECOM_LOG_MAG message. */
    SbgEComLogAirData               airData;            /*!< Stores data for the SBG_ECOM_LOG_AIR_DATA message. */
} SbgEComLogUnion;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse an incoming log and fill the output union.
 *
 * \param[in]   msgClass                    Received message class
 * \param[in]   msgId                       Received message ID
 * \param[in]   pPayload                    Read only pointer on the payload buffer.
 * \param[in]   payloadSize                 Payload size in bytes.
 * \param[out]  pLogData                    Pointer on the output union that stores parsed data.
 */
SbgErrorCode sbgEComLogParse(SbgEComClass msgClass, SbgEComMsgId msgId, const void *pPayload, size_t payloadSize, SbgEComLogUnion *pLogData);

/*!
 * Clean up resources allocated during parsing, if any.
 *
 * \param[in]   pLogData                    Log data.
 * \param[in]   msgClass                    Message class.
 * \param[in]   msgId                       Message ID.
 */
void sbgEComLogCleanup(SbgEComLogUnion *pLogData, SbgEComClass msgClass, SbgEComMsgId msgId);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_H
