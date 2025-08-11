/*!
 * \file            sbgEComLogMag.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            12 March 2013
 *
 * \brief           Parse magnetic field measurements logs.
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

#ifndef SBG_ECOM_LOG_MAG_H
#define SBG_ECOM_LOG_MAG_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Log magnetometers status definitions                               -//
//----------------------------------------------------------------------//

/*!
 * Log magnetometer data status mask definitions
 */
#define SBG_ECOM_MAG_MAG_X_BIT          (0x00000001u << 0)      /*!< Set to 1 if the magnetometer X passes Built In Test. */
#define SBG_ECOM_MAG_MAG_Y_BIT          (0x00000001u << 1)      /*!< Set to 1 if the magnetometer Y passes Built In Test. */
#define SBG_ECOM_MAG_MAG_Z_BIT          (0x00000001u << 2)      /*!< Set to 1 if the magnetometer Z passes Built In Test. */

#define SBG_ECOM_MAG_ACCEL_X_BIT        (0x00000001u << 3)      /*!< Set to 1 if the accelerometer X passes Built In Test. */
#define SBG_ECOM_MAG_ACCEL_Y_BIT        (0x00000001u << 4)      /*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define SBG_ECOM_MAG_ACCEL_Z_BIT        (0x00000001u << 5)      /*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define SBG_ECOM_MAG_MAGS_IN_RANGE      (0x00000001u << 6)      /*!< Set to 1 if all magnetometers are within operating range. */
#define SBG_ECOM_MAG_ACCELS_IN_RANGE    (0x00000001u << 7)      /*!< Set to 1 if all accelerometers are within operating range. */

#define SBG_ECOM_MAG_CALIBRATION_OK     (0x00000001u << 8)      /*!< Set to 1 if the magnetometers seems to be calibrated. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Represents data from the SBG_ECOM_LOG_MAG message.
 *
 * This structure encapsulates the magnetic field data, which is calibrated to correct for soft and hard iron effects. It provides
 * 3D magnetic field readings intended for use by the INS filter.
 */
typedef struct _SbgEComLogMag
{
    uint32_t    timeStamp;                  /*!< Time in us since the sensor power up. */
    uint16_t    status;                     /*!< Magnetometer status bitmask. */
    float       magnetometers[3];           /*!< X, Y, Z magnetometer data in arbitrary units (A.U.). */
    float       accelerometers[3];          /*!< X, Y, Z accelerometer data in (m/s^2). */
} SbgEComLogMag;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_MAG message and fill the corresponding structure.
 * 
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogMagReadFromStream(SbgEComLogMag *pLogData, SbgStreamBuffer *pStreamBuffer);

/*!
 * Write data for the SBG_ECOM_LOG_MAG message to the output stream buffer from the provided structure.
 *
 * \param[in]   pLogData                    Log structure instance to write.
 * \param[out]  pStreamBuffer               Output stream buffer to write the log to.
 * \return                                  SBG_NO_ERROR if the log has been written to the stream buffer.
 */
SbgErrorCode sbgEComLogMagWriteToStream(const SbgEComLogMag *pLogData, SbgStreamBuffer *pStreamBuffer);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_MAG_H
