/*!
 * \file            sbgEComLogEkf.h
 * \ingroup         binaryLogs
 * \author          SBG Systems
 * \date            25 February 2013
 *
 * \brief           Parse EKF measurements such as attitude, position and velocity logs.
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

#ifndef SBG_ECOM_LOG_EKF_H
#define SBG_ECOM_LOG_EKF_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Solution status definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * Solution bit masks definitions.
 */
#define SBG_ECOM_SOL_ATTITUDE_VALID         (0x00000001u << 4)      /*!< Set to 1 if attitude data is reliable (Roll/Pitch error < 0,5°). */
#define SBG_ECOM_SOL_HEADING_VALID          (0x00000001u << 5)      /*!< Set to 1 if heading data is reliable (Heading error < 1°). */
#define SBG_ECOM_SOL_VELOCITY_VALID         (0x00000001u << 6)      /*!< Set to 1 if velocity data is reliable (velocity error < 1.5 m/s). */
#define SBG_ECOM_SOL_POSITION_VALID         (0x00000001u << 7)      /*!< Set to 1 if position data is reliable (Position error < 10m). */
#define SBG_ECOM_SOL_VERT_REF_USED          (0x00000001u << 8)      /*!< Set to 1 if vertical reference is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_MAG_REF_USED           (0x00000001u << 9)      /*!< Set to 1 if magnetometer is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_VEL_USED          (0x00000001u << 10)     /*!< Set to 1 if GPS1 velocity is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_POS_USED          (0x00000001u << 11)     /*!< Set to 1 if GPS1 Position is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_HDT_USED          (0x00000001u << 13)     /*!< Set to 1 if GPS1 True Heading is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_VEL_USED          (0x00000001u << 14)     /*!< Set to 1 if GPS2 velocity is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_POS_USED          (0x00000001u << 15)     /*!< Set to 1 if GPS2 Position is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_HDT_USED          (0x00000001u << 17)     /*!< Set to 1 if GPS2 True Heading is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_ODO_USED               (0x00000001u << 18)     /*!< Set to 1 if Odometer is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_DVL_BT_USED            (0x00000001u << 19)     /*!< Set to 1 if DVL Bottom Tracking is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_DVL_WT_USED            (0x00000001u << 20)     /*!< Set to 1 if DVL Water Tracking is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_VEL1_USED              (0x00000001u << 21)     /*!< Set to 1 if generic velocity 1 is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_RESERVED_01            (0x00000001u << 22)     /*!< Reserved for future use. */
#define SBG_ECOM_SOL_RESERVED_02            (0x00000001u << 23)     /*!< Reserved for future use. */
#define SBG_ECOM_SOL_USBL_USED              (0x00000001u << 24)     /*!< Set to 1 if USBL / LBL is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_AIR_DATA_USED          (0x00000001u << 25)     /*!< Set to 1 if AirData (altimeter and/or true airspeed) is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_ZUPT_USED              (0x00000001u << 26)     /*!< Set to 1 if a Zero Velocity Update (ZUPT) is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_ALIGN_VALID            (0x00000001u << 27)     /*!< Set to 1 if sensor alignment and calibration parameters are valid */
#define SBG_ECOM_SOL_DEPTH_USED             (0x00000001u << 28)     /*!< Set to 1 if Depth sensor (for sub-sea navigation) is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_ZARU_USED              (0x00000001u << 29)     /*!< Set to 1 if a Zero Angular Rate Update (ZARU) is used in solution (data used and valid since 3s). */
/*!
 * Solution filter mode enum.
 */
typedef enum _SbgEComSolutionMode
{
    SBG_ECOM_SOL_MODE_UNINITIALIZED         = 0,                    /*!< The Kalman filter is not initialized and the returned data are all invalid. */
    SBG_ECOM_SOL_MODE_VERTICAL_GYRO         = 1,                    /*!< The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. */
    SBG_ECOM_SOL_MODE_AHRS                  = 2,                    /*!< A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. */
    SBG_ECOM_SOL_MODE_NAV_VELOCITY          = 3,                    /*!< The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. */
    SBG_ECOM_SOL_MODE_NAV_POSITION          = 4                     /*!< Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided. */
} SbgEComSolutionMode;

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * EFK computed orientation using quaternion.
 *
 * Note: In AHRS (Attitude and Heading Reference System) or INS (Inertial Navigation System) modes,
 * the yaw angle represents the geographic (true) heading of the device.
 *
 * To convert a geographic heading to a magnetic heading, use the following formula:
 *   Magnetic Heading = Geographic Heading - Magnetic Declination
 */
typedef struct _SbgEComLogEkfQuat
{
    uint32_t    timeStamp;              /*!< Time in us since the sensor power up. */
    float       quaternion[4];          /*!< Orientation quaternion stored in W, X, Y, Z form. */
    float       eulerStdDev[3];         /*!< Roll std. [0;+pi], pitch std. [0;+pi/2] and yaw std. [0;+pi] estimated 1-sigma accuracy in radians */
    uint32_t    status;                 /*!< EKF solution status bitmask and enum. */
    float       magDeclination;         /*!< Local magnetic declination in radians (positive east - set to NAN if not available). */
    float       magInclination;         /*!< Local magnetic inclination in radians (positive down - set to NAN if not available). */
} SbgEComLogEkfQuat;

/*!
 * EFK computed navigation data.
 */
typedef struct _SbgEComLogEkfNav
{
    uint32_t    timeStamp;              /*!< Time in us since the sensor power up. */
    float       velocity[3];            /*!< North, East, Down velocity in m.s^-1. */
    float       velocityStdDev[3];      /*!< North, East, Down velocity 1-sigma standard deviation in m.s^-1. */
    double      position[3];            /*!< Latitude, Longitude in degrees positive North and East.
                                         Altitude above Mean Sea Level in meters. */
    float       undulation;             /*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
    float       positionStdDev[3];      /*!< Latitude, longitude and altitude 1 sigma standard deviation in meters. */
    uint32_t    status;                 /*!< EKF solution status bitmask and enum. */
} SbgEComLogEkfNav;

//----------------------------------------------------------------------//
//- Public methods (SbgEComLogEkfQuat)                                 -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_EKF_QUAT message and fill the corresponding structure.
 *
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogEkfQuatReadFromStream(SbgEComLogEkfQuat *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public methods (SbgEComLogEkfNav)                                  -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_EKF_NAV message and fill the corresponding structure.
 *
 * \param[out]  pLogData                    Log structure instance to fill.
 * \param[in]   pStreamBuffer               Input stream buffer to read the log from.
 * \return                                  SBG_NO_ERROR if a valid log has been read from the stream buffer.
 */
SbgErrorCode sbgEComLogEkfNavReadFromStream(SbgEComLogEkfNav *pLogData, SbgStreamBuffer *pStreamBuffer);

//----------------------------------------------------------------------//
//- Public setters/getters                                             -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the solution mode from a solution status field.
 *
 * \param[in]   status              Status uint32_t value to extract the solution mode from it.
 * \return                          The extracted solution mode.
 */
SbgEComSolutionMode sbgEComLogEkfGetSolutionMode(uint32_t status);

#ifdef __cplusplus
}
#endif

#endif // SBG_ECOM_LOG_EKF_H
