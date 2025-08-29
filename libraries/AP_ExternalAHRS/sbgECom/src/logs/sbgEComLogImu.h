/*!
 * \file            sbgEComLogImu.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            25 February 2013
 *
 * \brief           Parse IMU (Inertial Measurement Unit) measurement logs.
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

#ifndef SBG_ECOM_LOG_IMU_H
#define SBG_ECOM_LOG_IMU_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Log Inertial Data definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * Log inertial data status mask definitions
 */
#define SBG_ECOM_IMU_COM_OK                 (0x00000001u << 0)      /*!< Set to 1 if the communication with the IMU is ok. */
#define SBG_ECOM_IMU_STATUS_BIT             (0x00000001u << 1)      /*!< Set to 1 if the IMU passes general Built in Tests (calibration, CPU, ...). */

#define SBG_ECOM_IMU_ACCEL_X_BIT            (0x00000001u << 2)      /*!< Set to 1 if the accelerometer X passes Built In Test. */
#define SBG_ECOM_IMU_ACCEL_Y_BIT            (0x00000001u << 3)      /*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define SBG_ECOM_IMU_ACCEL_Z_BIT            (0x00000001u << 4)      /*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define SBG_ECOM_IMU_GYRO_X_BIT             (0x00000001u << 5)      /*!< Set to 1 if the gyroscope X passes Built In Test. */
#define SBG_ECOM_IMU_GYRO_Y_BIT             (0x00000001u << 6)      /*!< Set to 1 if the gyroscope Y passes Built In Test. */
#define SBG_ECOM_IMU_GYRO_Z_BIT             (0x00000001u << 7)      /*!< Set to 1 if the gyroscope Z passes Built In Test. */

#define SBG_ECOM_IMU_ACCELS_IN_RANGE        (0x00000001u << 8)      /*!< Set to 1 if all accelerometers are within operating range. */
#define SBG_ECOM_IMU_GYROS_IN_RANGE         (0x00000001u << 9)      /*!< Set to 1 if all gyroscopes are within operating range. */
#define SBG_ECOM_IMU_GYROS_USE_HIGH_SCALE   (0x00000001u << 10)     /*!< Set if the gyroscope scale factor range is high. Applicable only for SBG_ECOM_LOG_IMU_SHORT logs. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_IMU_SHORT message.
 *
 * This message is sent asynchronously and must be used for post processing.
 *
 * The delta angle values are scaled based on the gyroscopes output. If any output exceeds
 * a predefined limit, the scale factor switches from standard to high range to prevent saturation.
 */
typedef struct _SbgEComLogImuShort
{
    uint32_t    timeStamp;                  /*!< Time in us since the sensor power up. */
    uint16_t    status;                     /*!< IMU status bitmask. */
    int32_t     deltaVelocity[3];           /*!< X, Y, Z delta velocity. Unit is 1048576 LSB for 1 m.s^-2. */
    int32_t     deltaAngle[3];              /*!< X, Y, Z delta angle. Unit is either 67108864 LSB for 1 rad.s^-1 (standard) or 12304174 LSB for 1 rad.s^-1 (high range), managed automatically. */
    int16_t     temperature;                /*!< IMU average temperature. Unit is 256 LSB for 1°C. */
} SbgEComLogImuShort;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_IMU_SHORT message and fill the corresponding structure.
 *
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogImuShortReadFromStream(SbgEComLogImuShort *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

/*!
 * Return from an IMU Short log, the X, Y or Z delta angle value in rad.s^-1
 *
 * \param[in]   pImuShort                   Input IMU short message instance.
 * \param[in]   idx                         The component to return from 0 to 2.
 * \return                                  The delta angle value converted in rad.s^-1.
 */
float sbgEComLogImuShortGetDeltaAngle(const SbgEComLogImuShort *pImuShort, size_t idx);

/*!
 * Return from an IMU Short log, the X, Y or Z delta velocity value in m.s^-2
 *
 * \param[in]   pImuShort                   Input IMU short message instance.
 * \param[in]   idx                         The component to return from 0 to 2.
 * \return                                  The delta velocity value converted in m.s^-2.
 */
float sbgEComLogImuShortGetDeltaVelocity(const SbgEComLogImuShort *pImuShort, size_t idx);

/*!
 * Return from an IMU Short log, the temperature in °C
 *
 * \param[in]   pImuShort                   Input IMU short message instance.
 * \return                                  The converted temperature in °C
 */
float sbgEComLogImuShortGetTemperature(const SbgEComLogImuShort *pImuShort);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_IMU_H
