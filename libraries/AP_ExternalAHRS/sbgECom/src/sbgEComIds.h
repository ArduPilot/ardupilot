/*!
 * \file            sbgEComIds.h
 * \ingroup         main
 * \author          SBG Systems
 * \date            25 February 2013
 *
 * \brief           Defines all sbgECom commands identifiers.
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

#ifndef SBG_ECOM_IDS_H
#define SBG_ECOM_IDS_H

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Definition of all class id for sbgECom                             -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the message classes available.
 *
 * Keep in mind that message classes are encoded on 7 bits.
 */
typedef enum _SbgEComClass
{
    SBG_ECOM_CLASS_LOG_ECOM_0           = 0x00,         /*!< Class that contains sbgECom protocol input/output log messages. */
    SBG_ECOM_CLASS_LOG_ECOM_1           = 0x01,         /*!< Class that contains special sbgECom output messages that handle high frequency output (deprecated in ELLIPSE firmware v3.x and above) . */

    SBG_ECOM_CLASS_LOG_NMEA_0           = 0x02,         /*!< Class that contains standard NMEA messages such as GGA/RMC/ZDA produced from the INS filer. */
    SBG_ECOM_CLASS_LOG_NMEA_1           = 0x03,         /*!< Class that contains proprietary NMEA (and NMEA like) output logs. */

    SBG_ECOM_CLASS_LOG_THIRD_PARTY_0    = 0x04,         /*!< Class that contains third party output logs using mostly binary protocols. */

    SBG_ECOM_CLASS_LOG_NMEA_GNSS        = 0x05,         /*!< Class that contains standard NMEA messages such as GGA/RMC/ZDA but produced from the internal GNSS data. */

    SBG_ECOM_CLASS_LOG_CMD_0            = 0x10,         /*!< Class that contains sbgECom protocol commands. */


    SBG_ECOM_CLASS_LOG_ALL              = 0xFF          /*!< Special value to represents all message classes. */
} SbgEComClass;

//----------------------------------------------------------------------//
//- Definition of all messages id for sbgECom                          -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the available ECom output logs from the sbgECom library.
 */
typedef enum _SbgEComLog
{
    SBG_ECOM_LOG_STATUS                     = 1,        /*!< Status general, clock, com aiding, solution, heave */

    SBG_ECOM_LOG_UTC_TIME                   = 2,        /*!< Provides UTC time reference */

    SBG_ECOM_LOG_IMU_DATA                   = 3,        /*!< DEPRECATED: Synchronous IMU measurements (time aligned to UTC - NEVER use for Post Processing). */

    SBG_ECOM_LOG_MAG                        = 4,        /*!< Magnetic data with associated accelerometer on each axis */
    SBG_ECOM_LOG_MAG_CALIB                  = 5,        /*!< Magnetometer calibration data (raw buffer) */

    SBG_ECOM_LOG_EKF_EULER                  = 6,        /*!< Includes roll, pitch, yaw and their accuracies on each axis */
    SBG_ECOM_LOG_EKF_QUAT                   = 7,        /*!< Includes the 4 quaternions values */
    SBG_ECOM_LOG_EKF_NAV                    = 8,        /*!< Position and velocities in NED coordinates with the accuracies on each axis */

    SBG_ECOM_LOG_SHIP_MOTION                = 9,        /*!< Heave, surge and sway and accelerations on each axis. */

    SBG_ECOM_LOG_GPS1_VEL                   = 13,       /*!< GPS velocities from primary or secondary GPS receiver */
    SBG_ECOM_LOG_GPS1_POS                   = 14,       /*!< GPS positions from primary or secondary GPS receiver */
    SBG_ECOM_LOG_GPS1_HDT                   = 15,       /*!< GPS true heading from dual antenna system */

    SBG_ECOM_LOG_GPS2_VEL                   = 16,       /*!< GPS 2 velocity log data. */
    SBG_ECOM_LOG_GPS2_POS                   = 17,       /*!< GPS 2 position log data. */
    SBG_ECOM_LOG_GPS2_HDT                   = 18,       /*!< GPS 2 true heading log data. */

    SBG_ECOM_LOG_ODO_VEL                    = 19,       /*!< Provides odometer velocity */

    SBG_ECOM_LOG_EVENT_A                    = 24,       /*!< Event markers sent when events are detected on sync in A pin */
    SBG_ECOM_LOG_EVENT_B                    = 25,       /*!< Event markers sent when events are detected on sync in B pin */
    SBG_ECOM_LOG_EVENT_C                    = 26,       /*!< Event markers sent when events are detected on sync in C pin */
    SBG_ECOM_LOG_EVENT_D                    = 27,       /*!< Event markers sent when events are detected on sync in D pin */
    SBG_ECOM_LOG_EVENT_E                    = 28,       /*!< Event markers sent when events are detected on sync in E pin */

    SBG_ECOM_LOG_DVL_BOTTOM_TRACK           = 29,       /*!< Doppler Velocity Log for bottom tracking data. */
    SBG_ECOM_LOG_DVL_WATER_TRACK            = 30,       /*!< Doppler Velocity log for water layer data. */

    SBG_ECOM_LOG_GPS1_RAW                   = 31,       /*!< GPS 1 raw data for post processing. */

    SBG_ECOM_LOG_SHIP_MOTION_HP             = 32,       /*!< Return delayed ship motion such as surge, sway, heave. */

    SBG_ECOM_LOG_AIR_DATA                   = 36,       /*!< Air Data aiding such as barometric altimeter and true air speed. */

    SBG_ECOM_LOG_USBL                       = 37,       /*!< Raw USBL position data for sub-sea navigation. */

    SBG_ECOM_LOG_GPS2_RAW                   = 38,       /*!< GPS 2 raw data for post processing. */


    SBG_ECOM_LOG_IMU_SHORT                  = 44,       /*!< Asynchronous IMU measurements output at the IMU rate and to use for Post Processing with Qinertia. */

    SBG_ECOM_LOG_EVENT_OUT_A                = 45,       /*!< Event marker used to timestamp each generated Sync Out A signal. */
    SBG_ECOM_LOG_EVENT_OUT_B                = 46,       /*!< Event marker used to timestamp each generated Sync Out B signal. */

    SBG_ECOM_LOG_DEPTH                      = 47,       /*!< Depth sensor measurement log used for sub-sea navigation. */
    SBG_ECOM_LOG_DIAG                       = 48,       /*!< Diagnostic log. */

    SBG_ECOM_LOG_RTCM_RAW                   = 49,       /*!< RTCM raw data. */

    SBG_ECOM_LOG_GPS1_SAT                   = 50,       /*!< GPS 1 Satellite data. */
    SBG_ECOM_LOG_GPS2_SAT                   = 51,       /*!< GNSS2 Satellite data. */

    SBG_ECOM_LOG_EKF_ROT_ACCEL_BODY         = 52,       /*!< INS body rotation rate and lateral acceleration (bias, earth rotation and gravity free compensated). */
    SBG_ECOM_LOG_EKF_ROT_ACCEL_NED          = 53,       /*!< INS North/East/Down rotation rate and lateral acceleration (bias, earth rotation and gravity free compensated). */
    SBG_ECOM_LOG_EKF_VEL_BODY               = 54,       /*!< INS X,Y,Z body velocity and standard deviation. */

    SBG_ECOM_LOG_SESSION_INFO               = 55,       /*!< Session information, including device information and current settings. */

    SBG_ECOM_LOG_EKF_DEBUG                  = 56,       /*!< EKF debug data. */

    SBG_ECOM_LOG_PTP_STATUS                 = 57,       /*!< PTP status. */

    SBG_ECOM_LOG_VELOCITY_1                 = 58,       /*!< Generic velocity 1 log. */

    SBG_ECOM_LOG_VIB_MON_FFT                = 59,       /*!< Vibration monitoring FFT data for post processing. */
    SBG_ECOM_LOG_VIB_MON_REPORT             = 60,       /*!< Vibration monitoring report information. */

    SBG_ECOM_LOG_ECOM_NUM_MESSAGES                      /*!< Helper definition to know the number of ECom messages */
} SbgEComLog;

/*!
 * Enum that defines all the available ECom output logs in the class SBG_ECOM_CLASS_LOG_ECOM_1
 */
typedef enum _SbgEComLog1MsgId
{
    SBG_ECOM_LOG_FAST_IMU_DATA              = 0,        /*!< Provides accelerometers, gyroscopes, time and status at 1KHz rate. */
    SBG_ECOM_LOG_ECOM_1_NUM_MESSAGES                    /*!< Helper definition to know the number of ECom messages */
} SbgEComLog1;

/*!
 * Defines standard NMEA output logs based on the INS filter data.
 */
typedef enum _SbgEComNmeaLog
{
    SBG_ECOM_LOG_NMEA_GGA                   = 0,        /*!< Latitude, Longitude, Altitude, Quality indicator. */
    SBG_ECOM_LOG_NMEA_RMC                   = 1,        /*!< Latitude, Longitude, velocity, course over ground. */
    SBG_ECOM_LOG_NMEA_ZDA                   = 2,        /*!< UTC Time. */
    SBG_ECOM_LOG_NMEA_HDT                   = 3,        /*!< Heading (True). */
    SBG_ECOM_LOG_NMEA_GST                   = 4,        /*!< GPS Pseudorange Noise Statistics. */
    SBG_ECOM_LOG_NMEA_VBW                   = 5,        /*!< Water referenced and ground referenced speed data. */
    SBG_ECOM_LOG_NMEA_DPT                   = 7,        /*!< Depth sensor output. */
    SBG_ECOM_LOG_NMEA_VTG                   = 8,        /*!< Track an Speed over the ground. */
    SBG_ECOM_LOG_NMEA_RTO                   = 9,        /*!< Rate and direction of turn. */
    SBG_ECOM_LOG_NMEA_GSV                   = 10,       /*!< GNSS Satellites in View with azimuth, elevation and SNR information. */
    SBG_ECOM_LOG_NMEA_GSA                   = 11,       /*!< GNSS DOP and active satellites. */
    SBG_ECOM_LOG_NMEA_NUM_MESSAGES                      /*!< Helper definition to know the number of NMEA messages */
} SbgEComNmeaLog;

/*!
 * Enum that defines all the available Proprietary NMEA output logs from the sbgECom library.
 */
typedef enum _SbgEComIdNmea1Log
{
    SBG_ECOM_LOG_NMEA_1_PRDID               = 0,        /*!< RDI proprietary sentence. Pitch, Roll, Heading */
    SBG_ECOM_LOG_NMEA_1_PSBGI               = 1,        /*!< SBG Systems proprietary sentence. Rotation rates, accelerations. */
    SBG_ECOM_LOG_NMEA_1_PASHR               = 2,        /*!< Proprietary sentence. Roll, Pitch, Heading, Heave. */
    SBG_ECOM_LOG_NMEA_1_PSBGB               = 3,        /*!< SBG Systems proprietary sentence. Attitude, heading, heave, angular rates, velocity. */

    SBG_ECOM_LOG_NMEA_1_PHINF               = 5,        /*!< Ixblue NMEA like log used to output Status information. */
    SBG_ECOM_LOG_NMEA_1_PHTRO               = 6,        /*!< Ixblue NMEA like log used to output Roll and Pitch. */
    SBG_ECOM_LOG_NMEA_1_PHLIN               = 7,        /*!< Ixblue NMEA like log used to output Surge, Sway and Heave. */
    SBG_ECOM_LOG_NMEA_1_PHOCT               = 8,        /*!< Ixblue NMEA like log used to output attitude and ship motion. */
    SBG_ECOM_LOG_NMEA_1_INDYN               = 9,        /*!< Ixblue NMEA like log used to output position, heading, attitude, attitude rate and speed. */

    SBG_ECOM_LOG_NMEA_1_GGK                 = 10,       /*!< Trimble NMEA like log with Time, Latitude, Longitude, Ellipsoidal height */
    SBG_ECOM_LOG_NMEA_1_PPS                 = 11,       /*!< Trimble (Applanix) NMEA like log with UTC and PPS information. */

    SBG_ECOM_LOG_NMEA_1_WASSP               = 12,       /*!< WASSP NMEA like log similar to PASHR one. */

    SBG_ECOM_LOG_NMEA_1_PSBGA               = 13,       /*!< SBG Systems proprietary sentence that reports EKF attitude and status. */

    SBG_ECOM_LOG_NMEA_1_NUM_MESSAGES                    /*!< Helper definition to know the number of NMEA messages */
} SbgEComIdNmea1Log;

/*!
 * Enum that defines all the available Proprietary output logs from the sbgECom library.
 */
typedef enum _SbgEComIdThirdParty
{
    SBG_ECOM_THIRD_PARTY_TSS1               = 0,        /*!< Roll, Pitch, Heave, heave accelerations */
    SBG_ECOM_THIRD_PARTY_KVH                = 1,        /*!< Roll, Pitch, Yaw */

    SBG_ECOM_THIRD_PARTY_PD0                = 2,        /*!< Teledyne PD0 DVL proprietary frame. */
    SBG_ECOM_THIRD_PARTY_SIMRAD_1000        = 3,        /*!< Kongsberg SimRad 1000 proprietary frame that outputs Roll, Pitch and Heading.  */
    SBG_ECOM_THIRD_PARTY_SIMRAD_3000        = 4,        /*!< Kongsberg SimRad 3000 proprietary frame that outputs Roll, Pitch and Heading. */

    SBG_ECOM_THIRD_PARTY_SEAPATH_B26        = 5,        /*!< Kongsberg Seapth Binary Log 26 used for MBES FM mode. */
    SBG_ECOM_THIRD_PARTY_DOLOG_HRP          = 6,        /*!< DOLOG Heading, Roll, Pitch proprietary and binary message. */
    SBG_ECOM_THIRD_PARTY_AHRS_500           = 7,        /*!< Crossbow AHRS-500 Data Packet output with attitude, rate, acceleration and status. */
    SBG_ECOM_THIRD_PARTY_ADA_01             = 8,        /*!< ADA specific Data Packet with IMU/INS/Status data */

    SBG_ECOM_THIRD_PARTY_AT_ITINS           = 9,        /*!< Cobham Aviator UAV 200 navigation (orientation & position) data */

    SBG_ECOM_THIRD_PARTY_KONGSBERG_MB       = 10,       /*!< Kongsberg multi-beam binary log. */

    SBG_ECOM_LOG_THIRD_PARTY_NUM_MESSAGES               /*!< Helper definition to know the number of third party messages */
} SbgEComIdThirdParty;

/*!
 * Defines standard NMEA output logs based on the GNSS data.
 */
typedef enum _SbgEComNmeaGnssLog
{
    SBG_ECOM_LOG_NMEA_GNSS_GGA              = 0,        /*!< Latitude, Longitude, Altitude, Quality indicator. */
    SBG_ECOM_LOG_NMEA_GNSS_RMC              = 1,        /*!< Latitude, Longitude, velocity, course over ground. */
    SBG_ECOM_LOG_NMEA_GNSS_ZDA              = 2,        /*!< UTC Time. */
    SBG_ECOM_LOG_NMEA_GNSS_HDT              = 3,        /*!< Heading (True). */
    SBG_ECOM_LOG_NMEA_GNSS_GST              = 4,        /*!< GPS Pseudorange Noise Statistics. */
    SBG_ECOM_LOG_NMEA_GNSS_VTG              = 8,        /*!< Track an Speed over the ground. */
    SBG_ECOM_LOG_NMEA_GNSS_GSA              = 11,       /*!< GNSS DOP and active satellites. */
} SbgEComNmeaGnssLog;


/*!
 * Enum that defines all the available commands for the sbgECom library.
 */
typedef enum _SbgEComCmd
{
    /* Acknowledge */
    SBG_ECOM_CMD_ACK                        = 0,        /*!< Acknowledge */

    /* Special settings commands */
    SBG_ECOM_CMD_SETTINGS_ACTION            = 1,        /*!< Performs various settings actions */
    SBG_ECOM_CMD_IMPORT_SETTINGS            = 2,        /*!< Imports a new settings structure to the sensor */
    SBG_ECOM_CMD_EXPORT_SETTINGS            = 3,        /*!< Export the whole configuration from the sensor */

    /* Device info */
    SBG_ECOM_CMD_INFO                       = 4,        /*!< Get basic device information */

    /* Sensor parameters */
    SBG_ECOM_CMD_INIT_PARAMETERS            = 5,        /*!< Initial configuration */
    SBG_ECOM_CMD_MOTION_PROFILE_ID          = 7,        /*!< Set/get motion profile information */
    SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM    = 8,        /*!< Sensor alignment and lever arm on vehicle configuration */
    SBG_ECOM_CMD_AIDING_ASSIGNMENT          = 9,        /*!< Aiding assignments such as RTCM / GPS / Odometer configuration */

    /* Magnetometer configuration */
    SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID      = 11,       /*!< Set/get magnetometer error model information */
    SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE   = 12,       /*!< Magnetometer aiding rejection mode */
    SBG_ECOM_CMD_SET_MAG_CALIB              = 13,       /*!< Set magnetic soft and hard Iron calibration data */

    /* Magnetometer on-board calibration */
    SBG_ECOM_CMD_START_MAG_CALIB            = 14,       /*!< Start / reset internal magnetic field logging for calibration. */
    SBG_ECOM_CMD_COMPUTE_MAG_CALIB          = 15,       /*!< Compute a magnetic calibration based on previously logged data. */

    /* GNSS configuration */
    SBG_ECOM_CMD_GNSS_1_MODEL_ID            = 17,       /*!< Set/get GNSS model information */
    SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT = 18,       /*!< DEPRECATED: GNSS installation configuration (lever arm, antenna alignments) */
    SBG_ECOM_CMD_GNSS_1_REJECT_MODES        = 19,       /*!< GNSS aiding rejection modes configuration. */

    /* Odometer configuration */
    SBG_ECOM_CMD_ODO_CONF                   = 20,       /*!< Odometer gain, direction configuration */
    SBG_ECOM_CMD_ODO_LEVER_ARM              = 21,       /*!< Odometer installation configuration (lever arm) */
    SBG_ECOM_CMD_ODO_REJECT_MODE            = 22,       /*!< Odometer aiding rejection mode configuration. */

    /* Interfaces configuration */
    SBG_ECOM_CMD_UART_CONF                  = 23,       /*!< UART interfaces configuration */
    SBG_ECOM_CMD_CAN_BUS_CONF               = 24,       /*!< CAN bus interface configuration */
    SBG_ECOM_CMD_CAN_OUTPUT_CONF            = 25,       /*!< CAN identifiers configuration */

    /* Events configuration */
    SBG_ECOM_CMD_SYNC_IN_CONF               = 26,       /*!< Synchronization inputs configuration */
    SBG_ECOM_CMD_SYNC_OUT_CONF              = 27,       /*!< Synchronization outputs configuration */

    /* Output configuration */
    SBG_ECOM_CMD_NMEA_TALKER_ID             = 29,       /*!< NMEA talker ID configuration */
    SBG_ECOM_CMD_OUTPUT_CONF                = 30,       /*!< Output configuration */

    /* Advanced configuration */
    SBG_ECOM_CMD_ADVANCED_CONF              = 32,       /*!< Advanced settings configuration */

    /* Features related commands */
    SBG_ECOM_CMD_FEATURES                   = 33,       /*!< Retrieve device features */

    /* Licenses related commands */
    SBG_ECOM_CMD_LICENSE_APPLY              = 34,       /*!< Upload and apply a new license */

    /* Message class output switch */
    SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE        = 35,       /*!< Enable/disable the output of an entire class */

    /* Ethernet configuration */
    SBG_ECOM_CMD_ETHERNET_CONF              = 36,       /*!< Set/get Ethernet configuration such as DHCP mode and IP address. */
    SBG_ECOM_CMD_ETHERNET_INFO              = 37,       /*!< Return the current IP used by the device. */

    /* Validity thresholds */
    SBG_ECOM_CMD_VALIDITY_THRESHOLDS        = 38,       /*!< Set/get Validity flag thresholds for position, velocity, attitude and heading */

    /* DVL configuration */
    SBG_ECOM_CMD_DVL_MODEL_ID               = 39,       /*!< Set/get DVL model id to use */
    SBG_ECOM_CMD_DVL_INSTALLATION           = 40,       /*!< DVL installation configuration (lever arm, alignments) */
    SBG_ECOM_CMD_DVL_REJECT_MODES           = 41,       /*!< DVL aiding rejection modes configuration. */

    /* AirData configuration */
    SBG_ECOM_CMD_AIRDATA_MODEL_ID           = 42,       /*!< Set/get AirData model id and protocol to use. */
    SBG_ECOM_CMD_AIRDATA_LEVER_ARM          = 43,       /*!< AirData installation configuration (lever arm, offsets) */
    SBG_ECOM_CMD_AIRDATA_REJECT_MODES       = 44,       /*!< AirData aiding rejection modes configuration. */

    /* Odometer configuration (using CAN) */
    SBG_ECOM_CMD_ODO_CAN_CONF               = 45,       /*!< Configuration for CAN based odometer (CAN ID & DBC) */

    SBG_ECOM_CMD_GNSS_1_INSTALLATION        = 46,       /*!< Define or retrieve the GNSS 1 main and secondary lever arms configuration. */

    /* REST API related commands */
    SBG_ECOM_CMD_API_POST                   = 47,       /*!< Command equivalent to the HTTP POST method for a REST API. */
    SBG_ECOM_CMD_API_GET                    = 48,       /*!< Command equivalent to the HTTP GET method for a REST API. */

    /* Misc. */
    SBG_ECOM_LOG_ECOM_NUM_CMDS                          /*!< Helper definition to know the number of commands */
} SbgEComCmd;

/*!
 *  This type defines any message identifier.
 *  Because message identifiers enum will be different with each class id, we use a generic uint8_t rather than an enum.
 */
typedef uint8_t SbgEComMsgId;

//----------------------------------------------------------------------//
//- Inline helpers for log IDs                                         -//
//----------------------------------------------------------------------//

/*!
 * Test if the message class is a binary log one.
 *
 * \param[in]   msgClass                Message class.
 * \return                              TRUE if the message class corresponds to a binary log.
 */
SBG_INLINE bool sbgEComMsgClassIsALog(SbgEComClass msgClass)
{
    //
    // Test if this class id is part of the enum
    //
    if ((msgClass == SBG_ECOM_CLASS_LOG_ECOM_0) || (msgClass == SBG_ECOM_CLASS_LOG_ECOM_1) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

#ifdef __cplusplus
}
#endif

#endif  // SBG_ECOM_IDS_H
