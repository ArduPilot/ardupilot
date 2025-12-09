/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "types.h"
#include "base_port.h"

#include "ISConstants.h"
#include "rtk_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

// *****************************************************************************
// ****** InertialSense binary message Data Identification Numbers (DIDs) ****** 
// ******                                                                 ******
// ****** NEVER REORDER THESE VALUES!                                     ******
// *****************************************************************************
/** Data identifiers - these are unsigned int and #define because enum are signed according to C standard */
typedef uint32_t eDataIDs;

#define DID_NULL                        (eDataIDs)0  /** NULL (INVALID) */
#define DID_DEV_INFO                    (eDataIDs)1  /** (dev_info_t) Device information */
#define DID_IMX_DEV_INFO                (DID_DEV_INFO)
#define DID_SYS_FAULT                   (eDataIDs)2  /** (system_fault_t) System fault information. This is broadcast automatically every 10s if a critical fault is detected. */
#define DID_PIMU                        (eDataIDs)3  /** (pimu_t) Preintegrated IMU (a.k.a. Coning and Sculling integral) in body/IMU frame.  Updated at IMU rate. Also know as delta theta delta velocity, or preintegrated IMU (PIMU). For clarification, the name "Preintegrated IMU" or "PIMU" throughout our User Manual. This data is integrated from the IMU data at the IMU update rate (startupImuDtMs, default 1ms).  The PIMU integration period (dt) and INS NAV update data period are the same.  DID_FLASH_CONFIG.startupNavDtMs sets the NAV output period at startup.  The minimum NAV update and output periods are found here:  https://docs.inertialsense.com/user-manual/application-config/imu_ins_gnss_configuration/#navigation-update-and-output-periods.  If a faster output data rate for IMU is desired, DID_IMU_RAW can be used instead. PIMU data acts as a form of compression, adding the benefit of higher integration rates for slower output data rates, preserving the IMU data without adding filter delay and addresses antialiasing. It is most effective for systems that have higher dynamics and lower communications data rates.  The minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). The PIMU value can be converted to IMU by dividing PIMU by dt (i.e. IMU = PIMU / dt)  */
#define DID_INS_1                       (eDataIDs)4  /** (ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA. */
#define DID_INS_2                       (eDataIDs)5  /** (ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define DID_GPS1_RCVR_POS               (eDataIDs)6  /** (gps_pos_t) GPS 1 position data from GNSS receiver. */
#define DID_SYS_CMD                     (eDataIDs)7  /** (system_command_t) System commands. Both the command and invCommand fields must be set at the same time for a command to take effect. */
#define DID_NMEA_BCAST_PERIOD           (eDataIDs)8  /** (nmea_msgs_t) Set broadcast periods for NMEA messages */
#define DID_RMC                         (eDataIDs)9  /** (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define DID_SYS_PARAMS                  (eDataIDs)10 /** (sys_params_t) System parameters / info */
#define DID_SYS_SENSORS                 (eDataIDs)11 /** (sys_sensors_t) System sensor information */
#define DID_FLASH_CONFIG                (eDataIDs)12 /** (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GPS1_POS                    (eDataIDs)13 /** (gps_pos_t) GPS 1 position data.  This comes from DID_GPS1_RCVR_POS or DID_GPS1_RTK_POS, depending on whichever is more accurate. */
#define DID_GPS2_POS                    (eDataIDs)14 /** (gps_pos_t) GPS 2 position data */
#define DID_GPS1_SAT                    (eDataIDs)15 /** (gps_sat_t) GPS 1 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GPS2_SAT                    (eDataIDs)16 /** (gps_sat_t) GPS 2 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GPS1_VERSION                (eDataIDs)17 /** (gps_version_t) GPS 1 version info */
#define DID_GPS2_VERSION                (eDataIDs)18 /** (gps_version_t) GPS 2 version info */
#define DID_MAG_CAL                     (eDataIDs)19 /** (mag_cal_t) Magnetometer calibration */
#define DID_UNUSED_20                   (eDataIDs)20 /** UNUSED */
#define DID_GPS1_RTK_POS_REL            (eDataIDs)21 /** (gps_rtk_rel_t) RTK precision position base to rover relative info. */
#define DID_GPS1_RTK_POS_MISC           (eDataIDs)22 /** (gps_rtk_misc_t) RTK precision position related data. */
#define DID_FEATURE_BITS                (eDataIDs)23 /** INTERNAL USE ONLY (feature_bits_t) */
#define DID_SENSORS_UCAL                (eDataIDs)24 /** INTERNAL USE ONLY (sensors_w_temp_t) Uncalibrated IMU output. */
#define DID_SENSORS_TCAL                (eDataIDs)25 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated IMU output. */
#define DID_SENSORS_TC_BIAS             (eDataIDs)26 /** INTERNAL USE ONLY (sensors_t) */
#define DID_UNUSED_27                   (eDataIDs)27 /** UNUSED */
#define DID_SENSORS_ADC                 (eDataIDs)28 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SCOMP                       (eDataIDs)29 /** INTERNAL USE ONLY (sensor_compensation_t) */
#define DID_GPS1_VEL                    (eDataIDs)30 /** (gps_vel_t) GPS 1 velocity data */
#define DID_GPS2_VEL                    (eDataIDs)31 /** (gps_vel_t) GPS 2 velocity data */
#define DID_HDW_PARAMS                  (eDataIDs)32 /** INTERNAL USE ONLY (hdw_params_t) */
#define DID_NVR_MANAGE_USERPAGE         (eDataIDs)33 /** INTERNAL USE ONLY (nvr_manage_t) */
#define DID_NVR_USERPAGE_SN             (eDataIDs)34 /** INTERNAL USE ONLY (nvm_group_sn_t) */
#define DID_NVR_USERPAGE_G0             (eDataIDs)35 /** INTERNAL USE ONLY (nvm_group_0_t) */
#define DID_NVR_USERPAGE_G1             (eDataIDs)36 /** INTERNAL USE ONLY (nvm_group_1_t) */
#define DID_DEBUG_STRING                (eDataIDs)37 /** INTERNAL USE ONLY (debug_string_t) */
#define DID_RTOS_INFO                   (eDataIDs)38 /** (rtos_info_t) RTOS information. */
#define DID_DEBUG_ARRAY                 (eDataIDs)39 /** INTERNAL USE ONLY (debug_array_t) */
#define DID_SENSORS_MCAL                (eDataIDs)40 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated and motion calibrated IMU output. */
#define DID_GPS1_TIMEPULSE              (eDataIDs)41 /** (gps_timepulse_t) GPS1 PPS time synchronization. */
#define DID_CAL_SC                      (eDataIDs)42 /** INTERNAL USE ONLY (sensor_cal_t) */
#define DID_CAL_TEMP_COMP               (eDataIDs)43 /** INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_MOTION                  (eDataIDs)44 /** INTERNAL USE ONLY (sensor_mcal_group_t) */
#define DID_GPS1_SIG                    (eDataIDs)45 /** (gps_sig_t) GPS 1 GNSS signal information. */
#define DID_SENSORS_ADC_SIGMA           (eDataIDs)46 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_REFERENCE_MAGNETOMETER      (eDataIDs)47 /** (magnetometer_t) Reference or truth magnetometer used for manufacturing calibration and testing */
#define DID_INL2_STATES                 (eDataIDs)48 /** (inl2_states_t) INS Extended Kalman Filter (EKF) states */
#define DID_INL2_COVARIANCE_LD          (eDataIDs)49 /** (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_STATUS                 (eDataIDs)50 /** (inl2_status_t) */
#define DID_INL2_MISC                   (eDataIDs)51 /** (inl2_misc_t) */
#define DID_MAGNETOMETER                (eDataIDs)52 /** (magnetometer_t) Magnetometer sensor output */
#define DID_BAROMETER                   (eDataIDs)53 /** (barometer_t) Barometric pressure sensor data */
#define DID_GPS1_RTK_POS                (eDataIDs)54 /** (gps_pos_t) GPS RTK position data */
#define DID_ROS_COVARIANCE_POSE_TWIST   (eDataIDs)55 /** (ros_covariance_pose_twist_t) INL2 EKF 6x6 covariance matrices packed in arrays containing their elements on main diagonal and below */
#define DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56 /** INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_IMU3_UNCAL                  (eDataIDs)57 /** INTERNAL USE ONLY (imu3_t) Uncalibrated triple IMU data.  We recommend use of DID_IMU or DID_PIMU as they are calibrated and oversampled and contain less noise.  Minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). */
#define DID_IMU                         (eDataIDs)58 /** (imu_t) Inertial measurement unit data down-sampled from IMU rate (DID_FLASH_CONFIG.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG.startupNavDtMs (1KHz max).  */
#define DID_INL2_MAG_OBS_INFO           (eDataIDs)59 /** (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_GPS_BASE_RAW                (eDataIDs)60 /** (gps_raw_t) GPS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GPS_RTK_OPT                 (eDataIDs)61 /** (gps_rtk_opt_t) RTK options - requires little endian CPU. */
#define DID_REFERENCE_PIMU              (eDataIDs)62 /** (pimu_t) Reference or truth IMU used for manufacturing calibration and testing */
#define DID_MANUFACTURING_INFO          (eDataIDs)63 /** INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_BIT                         (eDataIDs)64 /** (bit_t) System built-in self-test */
#define DID_INS_3                       (eDataIDs)65 /** (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define DID_INS_4                       (eDataIDs)66 /** (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define DID_INL2_NED_SIGMA              (eDataIDs)67 /** (inl2_ned_sigma_t) Standard deviation of INL2 EKF estimates in the NED frame. */
#define DID_STROBE_IN_TIME              (eDataIDs)68 /** (strobe_in_time_t) Timestamp for input strobe. */
#define DID_GPS1_RAW                    (eDataIDs)69 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GPS2_RAW                    (eDataIDs)70 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_WHEEL_ENCODER               (eDataIDs)71 /** (wheel_encoder_t) Wheel encoder data to be fused with GPS-INS measurements, set DID_GROUND_VEHICLE for configuration before sending this message */
#define DID_DIAGNOSTIC_MESSAGE          (eDataIDs)72 /** (diag_msg_t) Diagnostic message */
#define DID_SURVEY_IN                   (eDataIDs)73 /** (survey_in_t) Survey in, used to determine position for RTK base station. Base correction output cannot run during a survey and will be automatically disabled if a survey is started. */
#define DID_CAL_SC_INFO                 (eDataIDs)74 /** INTERNAL USE ONLY (sensor_cal_info_t) */
#define DID_PORT_MONITOR                (eDataIDs)75 /** (port_monitor_t) Data rate and status monitoring for each communications port. */
#define DID_RTK_STATE                   (eDataIDs)76 /** INTERNAL USE ONLY (rtk_state_t) */
#define DID_RTK_PHASE_RESIDUAL          (eDataIDs)77 /** INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_CODE_RESIDUAL           (eDataIDs)78 /** INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_DEBUG                   (eDataIDs)79 /** INTERNAL USE ONLY (rtk_debug_t) */
#define DID_EVB_STATUS                  (eDataIDs)80 /** (evb_status_t) EVB monitor and log control interface. */
#define DID_EVB_FLASH_CFG               (eDataIDs)81 /** (evb_flash_cfg_t) EVB configuration. */
#define DID_EVB_DEBUG_ARRAY             (eDataIDs)82 /** INTERNAL USE ONLY (debug_array_t) */
#define DID_EVB_RTOS_INFO               (eDataIDs)83 /** (evb_rtos_info_t) EVB-2 RTOS information. */
#define DID_GPS2_SIG                    (eDataIDs)84 /** (gps_sig_t) GPS 2 GNSS signal information. */
#define DID_IMU_MAG                     (eDataIDs)85 /** (imu_mag_t) DID_IMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_PIMU_MAG                    (eDataIDs)86 /** (pimu_mag_t) DID_PIMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_GROUND_VEHICLE              (eDataIDs)87 /** (ground_vehicle_t) Static configuration for wheel transform measurements. */
#define DID_POSITION_MEASUREMENT        (eDataIDs)88 /** (pos_measurement_t) External position estimate */
#define DID_RTK_DEBUG_2                 (eDataIDs)89 /** INTERNAL USE ONLY (rtk_debug_2_t) */
#define DID_CAN_CONFIG                  (eDataIDs)90 /** (can_config_t) Addresses for CAN messages*/
#define DID_GPS2_RTK_CMP_REL            (eDataIDs)91 /** (gps_rtk_rel_t) Dual GNSS RTK compassing / moving base to rover (GPS 1 to GPS 2) relative info. */
#define DID_GPS2_RTK_CMP_MISC           (eDataIDs)92 /** (gps_rtk_misc_t) RTK Dual GNSS RTK compassing related data. */
#define DID_EVB_DEV_INFO                (eDataIDs)93 /** (dev_info_t) EVB device information */
#define DID_INFIELD_CAL                 (eDataIDs)94 /** (infield_cal_t) Measure and correct IMU calibration error.  Estimate INS rotation to align INS with vehicle. */
#define DID_REFERENCE_IMU               (eDataIDs)95 /** (imu_t) Raw reference or truth IMU used for manufacturing calibration and testing. Input from testbed. */
#define DID_IMU3_RAW                    (eDataIDs)96 /** (imu3_t) Triple IMU data calibrated from DID_IMU3_UNCAL.  We recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define DID_IMU_RAW                     (eDataIDs)97 /** (imu_t) IMU data averaged from DID_IMU3_RAW.  Use this IMU data for output data rates faster than DID_FLASH_CONFIG.startupNavDtMs.  Otherwise we recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define DID_FIRMWARE_UPDATE             (eDataIDs)98 /** (firmware_payload_t) firmware update payload */
#define DID_RUNTIME_PROFILER            (eDataIDs)99 /** INTERNAL USE ONLY (runtime_profiler_t) System runtime profiler */

#define DID_EVENT                       (eDataIDs)119 /** INTERNAL USE ONLY (did_event_t)*/

#define DID_GPX_FIRST                   (eDataIDs)120 /** First of GPX DIDs */
#define DID_GPX_DEV_INFO                (eDataIDs)120 /** (dev_info_t) GPX device information */
#define DID_GPX_FLASH_CFG               (eDataIDs)121 /** (gpx_flash_cfg_t) GPX flash configuration */
#define DID_GPX_RTOS_INFO               (eDataIDs)122 /** (gpx_rtos_info_t) GPX RTOs info */
#define DID_GPX_STATUS                  (eDataIDs)123 /** (gpx_status_t) GPX status */
#define DID_GPX_DEBUG_ARRAY             (eDataIDs)124 /** (debug_array_t) GPX debug */
#define DID_GPX_BIT                     (eDataIDs)125 /** (gpx_bit_t) GPX BIT test */
#define DID_GPX_RMC                     (eDataIDs)126 /** (rmc_t) GPX rmc  */
#define DID_GPX_PORT_MONITOR            (eDataIDs)127 /** (port_monitor_t) Data rate and status monitoring for each communications port. */
#define DID_GPX_LAST                              127 /** Last of GPX DIDs */

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5) Update the DIDs in IS-src/python/src/ci_hdw/data_sets.py
// 6] Test!

/** Count of data ids (including null data id 0) - MUST BE MULTPLE OF 4 and larger than last DID number! */
#define DID_COUNT       (eDataIDs)132    // Used in SDK
#define DID_COUNT_UINS  (eDataIDs)100    // Used in IMX

/** Maximum number of data ids */
#define DID_MAX_COUNT   256

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/** Maximum number of satellite channels */
#define MAX_NUM_SATELLITES  50

/** Maximum number of satellite signals */
#define MAX_NUM_SAT_SIGNALS 100

/** Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN      24

/** Communications Protocol Version. See release notes. */

// Increment w/ breaking changes (in ISComm.cpp) that prevent backwards compatibility with older protocols. 
// #define PROTOCOL_VERSION_CHAR0   .   // Breaking changes (Packet)        (defined in ISComm.h) 
#define PROTOCOL_VERSION_CHAR1      1   // Breaking changes (Payload)

// Increment w/ non-breaking changes (in data_sets.h) that would still backward compatibility with older protocols
// #define PROTOCOL_VERSION_CHAR2   .   // Non-breaking changes (Packet):   (defined in ISComm.h)
#define PROTOCOL_VERSION_CHAR3      0   // Non-breaking changes (Payload):  

/** Rtk rover receiver index */
#define RECEIVER_INDEX_GPS1             1 // DO NOT CHANGE
#define RECEIVER_INDEX_EXTERNAL_BASE    2 // DO NOT CHANGE
#define RECEIVER_INDEX_GPS2             3 // DO NOT CHANGE

// Max number of devices across all hardware types: uINS-3, uINS-4, and IMX-5
#define NUM_IMU_DEVICES     3        // g_numImuDevices defines the actual number of hardware specific devices
#define NUM_MAG_DEVICES     2        // g_numMagDevices defines the actual number of hardware specific devices

/** INS status flags */
enum eInsStatusFlags
{
    /** Heading estimate is usable but outside spec (COARSE) */
    INS_STATUS_HDG_ALIGN_COARSE                 = (int)0x00000001,
    /** Velocity estimate is usable but outside spec (COARSE) */
    INS_STATUS_VEL_ALIGN_COARSE                 = (int)0x00000002,
    /** Position estimate is usable but outside spec (COARSE) */
    INS_STATUS_POS_ALIGN_COARSE                 = (int)0x00000004,
    /** Estimate is COARSE mask (usable but outside spec) */
    INS_STATUS_ALIGN_COARSE_MASK                = (int)0x00000007,

    /** Velocity aided by wheel sensor */
    INS_STATUS_WHEEL_AIDING_VEL                 = (int)0x00000008,

    /** Heading estimate is within spec (FINE).  `INS_STATUS_HDG_ALIGN_COARSE` and `INS_STATUS_HDG_ALIGN_FINE` flags indicate whether INS heading is aided by any heading sensor (including GPS or magnetometer).  More accurate heading sensors (i.e. GPS) are prioritized over less accurate sensors (i.e. magnetometers) and will fall back to the less accurate sensors when the more accurate sensors are not available.  A momentary blip in these alignment flags may occur during heading transition from higher to lower accuracy aiding sensors (i.e. GPS to magnetometer).  `INS_STATUS_HDG_ALIGN_FINE` and `INS_STATUS_HDG_ALIGN_COARSE` flags will not be set when no heading aiding is available.  */
    INS_STATUS_HDG_ALIGN_FINE                   = (int)0x00000010,
    /** Velocity estimate is within spec (FINE) */
    INS_STATUS_VEL_ALIGN_FINE                   = (int)0x00000020,
    /** Position estimate is within spec (FINE) */
    INS_STATUS_POS_ALIGN_FINE                   = (int)0x00000040,
    /** Estimate is FINE mask */
    INS_STATUS_ALIGN_FINE_MASK                  = (int)0x00000070,

    /** Heading aided by GPS */
    INS_STATUS_GPS_AIDING_HEADING               = (int)0x00000080,

    /** Position aided by GPS position */
    INS_STATUS_GPS_AIDING_POS                   = (int)0x00000100,
    /** GPS update event occurred in solution, potentially causing discontinuity in position path */
    INS_STATUS_GPS_UPDATE_IN_SOLUTION           = (int)0x00000200,
    /** Reference IMU used in EKF */
    INS_STATUS_EKF_USING_REFERENCE_IMU          = (int)0x00000400,
    /** Heading aided by magnetic heading */
    INS_STATUS_MAG_AIDING_HEADING               = (int)0x00000800,

    /** Nav mode (set) = estimating velocity and position. AHRS mode (cleared) = NOT estimating velocity and position */
    INS_STATUS_NAV_MODE                         = (int)0x00001000,

    /** In dead reckoning mode.  The GPS is not aiding the solution while the position is being estimated.  */
#define INS_STATUS_DEAD_RECKONING(insStatus)    (((insStatus)&(INS_STATUS_POS_ALIGN_FINE|INS_STATUS_POS_ALIGN_COARSE)) && (((insStatus)&INS_STATUS_GPS_AIDING_POS)==0)) 

    /** INS in stationary mode.  If initiated by zero velocity command, user should not move (keep system motionless) to assist on-board processing. */
    INS_STATUS_STATIONARY_MODE                  = (int)0x00002000,    
    /** Velocity aided by GPS velocity */
    INS_STATUS_GPS_AIDING_VEL                   = (int)0x00004000,
    /** Vehicle kinematic calibration is good */
    INS_STATUS_KINEMATIC_CAL_GOOD               = (int)0x00008000,

    /** INS/AHRS Solution Status */
    INS_STATUS_SOLUTION_MASK                    = (int)0x000F0000,
    INS_STATUS_SOLUTION_OFFSET                  = 16,
#define INS_STATUS_SOLUTION(insStatus)          (((insStatus)&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)

    INS_STATUS_SOLUTION_OFF                     = 0,    // System is off 
    INS_STATUS_SOLUTION_ALIGNING                = 1,    // System is in alignment mode
    INS_STATUS_SOLUTION_NAV                     = 3,    // System is in navigation mode and solution is good.
    INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE       = 4,    // System is in navigation mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_AHRS                    = 5,    // System is in AHRS mode and solution is good.
    INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE      = 6,    // System is in AHRS mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_VRS                     = 7,    // System is in VRS mode (no earth relative heading) and roll and pitch are good.
    INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE       = 8,    // System is in VRS mode (no earth relative heading) but roll and pitch uncertainty has exceeded the threshold.

    /** GPS compassing antenna offsets are not set in flashCfg. */
    INS_STATUS_RTK_COMPASSING_BASELINE_UNSET    = (int)0x00100000,
    /** GPS antenna baseline specified in flashCfg and measured by GPS do not match. */
    INS_STATUS_RTK_COMPASSING_BASELINE_BAD      = (int)0x00200000,
    INS_STATUS_RTK_COMPASSING_MASK              = (INS_STATUS_RTK_COMPASSING_BASELINE_UNSET|INS_STATUS_RTK_COMPASSING_BASELINE_BAD),

    /** Magnetometer is being recalibrated.  Device requires rotation to complete the calibration process. HDW_STATUS_MAG_RECAL_COMPLETE is set when complete. */
    INS_STATUS_MAG_RECALIBRATING                = (int)0x00400000,
    /** Magnetometer is experiencing interference or calibration is bad.  Attention may be required to remove interference (move the device) or recalibrate the magnetometer. */
    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL = (int)0x00800000,

    /** GPS navigation fix type (see eGpsNavFixStatus) */
    INS_STATUS_GPS_NAV_FIX_MASK                 = (int)0x03000000,
    INS_STATUS_GPS_NAV_FIX_OFFSET               = 24,
#define INS_STATUS_NAV_FIX_STATUS(insStatus)    (((insStatus)&INS_STATUS_GPS_NAV_FIX_MASK)>>INS_STATUS_GPS_NAV_FIX_OFFSET)

    /** RTK compassing heading is accurate and aiding INS heading.  (RTK fix and hold status) */
    INS_STATUS_RTK_COMPASSING_VALID             = (int)0x04000000,

    /* NOTE: If you add or modify these INS_STATUS_RTK_ values, please update eInsStatusRtkBase in IS-src/python/src/ci_hdw/data_sets.py */
    /** RTK error: Observations invalid or not received  (i.e. RTK differential corrections) */
    INS_STATUS_RTK_RAW_GPS_DATA_ERROR           = (int)0x08000000,
    /** RTK error: Either base observations or antenna position have not been received */
    INS_STATUS_RTK_ERR_BASE_DATA_MISSING        = (int)0x10000000,
    /** RTK error: base position moved when it should be stationary */
    INS_STATUS_RTK_ERR_BASE_POSITION_MOVING     = (int)0x20000000,
    /** RTK error: base position invalid or not surveyed */
    INS_STATUS_RTK_ERR_BASE_POSITION_INVALID    = (int)0x30000000,
    /** RTK error: NO base position received */
    INS_STATUS_RTK_ERR_BASE_MASK                = (int)0x30000000,
    /** GPS base mask */
    INS_STATUS_RTK_ERROR_MASK                   = (INS_STATUS_RTK_RAW_GPS_DATA_ERROR|INS_STATUS_RTK_ERR_BASE_MASK),
    
    /** RTOS task ran longer than allotted period */
    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN         = (int)0x40000000,
    /** General fault (see sys_params_t.genFaultCode) */
    INS_STATUS_GENERAL_FAULT                    = (int)0x80000000,

    /** Bitmask of all insStatus errors */
    INS_STATUS_ERROR_MASK                       =   INS_STATUS_GENERAL_FAULT | 
                                                    INS_STATUS_RTK_COMPASSING_MASK | 
                                                    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL |
                                                    INS_STATUS_RTK_ERROR_MASK |
                                                    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN,
};

/** GPS navigation fix type */
/* NOTE: If you modify this enum, please also modify the eGpsNavFixStatus enum
 *       in IS-src/python/src/ci_hdw/data_sets.py */
enum eGpsNavFixStatus
{
    GPS_NAV_FIX_NONE                            = (int)0x00000000,
    GPS_NAV_FIX_POSITIONING_3D                  = (int)0x00000001,
    GPS_NAV_FIX_POSITIONING_RTK_FLOAT           = (int)0x00000002,
    GPS_NAV_FIX_POSITIONING_RTK_FIX             = (int)0x00000003,        // Includes fix & hold
};

/** Hardware status flags */
enum eHdwStatusFlags
{
    /** Gyro motion detected */
    HDW_STATUS_MOTION_GYR                       = (int)0x00000001,
    /** Accelerometer motion detected */
    HDW_STATUS_MOTION_ACC                       = (int)0x00000002,
    /** Unit is moving and NOT stationary */
    HDW_STATUS_MOTION_MASK                      = (int)0x00000003,
    /** IMU gyro fault rejection. One of the redundant gyro sensors is divergent and being excluded. */
    HDW_STATUS_IMU_FAULT_REJECT_GYR             = (int)0x00000004,
    /** IMU accelerometer fault rejection. One of the redundant accelerometer sensors is divergent and being excluded. */
    HDW_STATUS_IMU_FAULT_REJECT_ACC             = (int)0x00000008,
    /** IMU fault rejection mask. One of the redundant IMU sensors is divergent and being excluded. */
    HDW_STATUS_IMU_FAULT_REJECT_MASK            = (int)0x0000000C,

    /** GPS satellite signals are being received (antenna and cable are good). Unset indicates weak signal or no output from GPS receiver. */
    HDW_STATUS_GPS_SATELLITE_RX_VALID           = (int)0x00000010,
    /** Event occurred on strobe input pin */
    HDW_STATUS_STROBE_IN_EVENT                  = (int)0x00000020,
    /** GPS time of week is valid and reported.  Otherwise the timeOfWeek is local system time. */
    HDW_STATUS_GPS_TIME_OF_WEEK_VALID           = (int)0x00000040,
    /** Reference IMU data being received */
    HDW_STATUS_REFERENCE_IMU_RX                 = (int)0x00000080,

    /** Sensor saturation on gyro */
    HDW_STATUS_SATURATION_GYR                   = (int)0x00000100,
    /** Sensor saturation on accelerometer */
    HDW_STATUS_SATURATION_ACC                   = (int)0x00000200,
    /** Sensor saturation on magnetometer */
    HDW_STATUS_SATURATION_MAG                   = (int)0x00000400,
    /** Sensor saturation on barometric pressure */
    HDW_STATUS_SATURATION_BARO                  = (int)0x00000800,

    /** Sensor saturation mask */
    HDW_STATUS_SATURATION_MASK                  = (int)0x00000F00,
    /** Sensor saturation offset */
    HDW_STATUS_SATURATION_OFFSET                = 8,

    /** System Reset is required for proper function */
    HDW_STATUS_SYSTEM_RESET_REQUIRED            = (int)0x00001000,
    /** GPS PPS timepulse signal has noise and occurred too frequently */
    HDW_STATUS_ERR_GPS_PPS_NOISE                = (int)0x00002000,
    /** Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).  */
    HDW_STATUS_MAG_RECAL_COMPLETE               = (int)0x00004000,
    /** System flash write staging or occurring now.  Processor will pause and not respond during a flash write, typically 150-250 ms. */
    HDW_STATUS_FLASH_WRITE_PENDING              = (int)0x00008000,

    /** Communications Tx buffer limited */
    HDW_STATUS_ERR_COM_TX_LIMITED               = (int)0x00010000,
    /** Communications Rx buffer overrun */
    HDW_STATUS_ERR_COM_RX_OVERRUN               = (int)0x00020000,

    /** GPS PPS timepulse signal has not been received or is in error */
    HDW_STATUS_ERR_NO_GPS_PPS                   = (int)0x00040000,
    /** Time synchronized by GPS PPS */
    HDW_STATUS_GPS_PPS_TIMESYNC                 = (int)0x00080000,

    /** Communications parse error count */
    HDW_STATUS_COM_PARSE_ERR_COUNT_MASK         = (int)0x00F00000,
    HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET       = 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus & HDW_STATUS_COM_PARSE_ERR_COUNT_MASK) >> HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

    /** (BIT) Built-in self-test running */
    HDW_STATUS_BIT_RUNNING                      = (int)0x01000000,
    /** (BIT) Built-in self-test passed */
    HDW_STATUS_BIT_PASSED                       = (int)0x02000000,
    /** (BIT) Built-in self-test failure */
    HDW_STATUS_BIT_FAILED                       = (int)0x03000000,
    /** (BIT) Built-in self-test mask */
    HDW_STATUS_BIT_MASK                         = (int)0x03000000,

    /** Temperature outside spec'd operating range */
    HDW_STATUS_ERR_TEMPERATURE                  = (int)0x04000000,
    
    /** IMX pins G5-G8 are configure for SPI use */
    HDW_STATUS_SPI_INTERFACE_ENABLED            = (int)0x08000000,

    /** Cause of system reset */
    HDW_STATUS_RESET_CAUSE_MASK                 = (int)0x70000000,
    /** Reset from backup mode (low-power state w/ CPU off) */
    HDW_STATUS_RESET_CAUSE_BACKUP_MODE          = (int)0x10000000,
    /** Reset from watchdog fault */
    HDW_STATUS_RESET_CAUSE_WATCHDOG_FAULT       = (int)0x20000000,
    /** Reset from software */
    HDW_STATUS_RESET_CAUSE_SOFT                 = (int)0x30000000,
    /** Reset from hardware (NRST pin low) */
    HDW_STATUS_RESET_CAUSE_HDW                  = (int)0x40000000,

    /** Critical System Fault, CPU error.  (see DID_SYS_FAULT.status, eSysFaultStatus) */
    HDW_STATUS_FAULT_SYS_CRITICAL               = (int)0x80000000,

    /** Bitmask of all hdwStatus errors */
    HDW_STATUS_ERROR_MASK                       =   HDW_STATUS_FAULT_SYS_CRITICAL | 
                                                    HDW_STATUS_IMU_FAULT_REJECT_MASK | 
                                                    HDW_STATUS_SATURATION_MASK | 
                                                    HDW_STATUS_ERR_GPS_PPS_NOISE |
                                                    HDW_STATUS_ERR_COM_TX_LIMITED |
                                                    HDW_STATUS_ERR_COM_RX_OVERRUN |
                                                    HDW_STATUS_ERR_NO_GPS_PPS |
                                                    HDW_STATUS_BIT_FAILED |
                                                    HDW_STATUS_ERR_TEMPERATURE,
};

/** System status flags */
enum eSysStatusFlags
{
    /** Allow IMX to drive Testbed-3 status LEDs */
    SYS_STATUS_TBED3_LEDS_ENABLED   = (int)0x00000001,

    SYS_STATUS_DMA_FAULT_DETECT                     = (int)0x00000002,

    SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2         = (int)0x00000004, // 0 = GPS1 is the primary NMEA GNSS source 1 = GPS2 is the primary NMEA GNSS source
    SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2_offest  = 2,
};

// Used to validate GPS position (and velocity)
#define GPS_THRESH_SATS_USED        5
#define GPS_THRESH_P_DOP            3.0f
#define GPS_THRESH_H_ACC            10.0f
#define GPS_THRESH_V_ACC            20.0f
#define GPS_THRESH_S_ACC            2.0f

/** GPS Status */
enum eGpsStatus
{
    // TODO: THIS FIELD WILL END OF LIFE IN PROTOCOL 3
    // PLEASE USE gps_pos_t.satsUsed for all new development
    GPS_STATUS_NUM_SATS_USED_MASK                   = (int)0x000000FF,

    /** Fix */
    GPS_STATUS_FIX_NONE                             = (int)0x00000000,
    GPS_STATUS_FIX_DEAD_RECKONING_ONLY              = (int)0x00000100,
    GPS_STATUS_FIX_2D                               = (int)0x00000200,
    GPS_STATUS_FIX_3D                               = (int)0x00000300,
    GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK               = (int)0x00000400,
    GPS_STATUS_FIX_TIME_ONLY                        = (int)0x00000500,
    GPS_STATUS_FIX_REF_LLA                          = (int)0x00000600,
    GPS_STATUS_FIX_UNUSED2                          = (int)0x00000700,
    GPS_STATUS_FIX_DGPS                             = (int)0x00000800,
    GPS_STATUS_FIX_SBAS                             = (int)0x00000900,
    GPS_STATUS_FIX_RTK_SINGLE                       = (int)0x00000A00,
    GPS_STATUS_FIX_RTK_FLOAT                        = (int)0x00000B00,
    GPS_STATUS_FIX_RTK_FIX                          = (int)0x00000C00,
    GPS_STATUS_FIX_MASK                             = (int)0x00001F00,
    GPS_STATUS_FIX_BIT_OFFSET                       = (int)8,

    /** Flags  */
    GPS_STATUS_FLAGS_FIX_OK                         = (int)0x00010000,      // within limits (e.g. DOP & accuracy)
    GPS_STATUS_FLAGS_DGPS_USED                      = (int)0x00020000,      // Differential GPS (DGPS) used.
    GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD               = (int)0x00040000,      // RTK feedback on the integer solutions to drive the float biases towards the resolved integers
    GPS_STATUS_FLAGS_UNUSED_1                       = (int)0x00080000,
    GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED      = (int)0x00100000,      // GPS1 RTK precision positioning mode enabled
    GPS_STATUS_FLAGS_STATIC_MODE                    = (int)0x00200000,      // Static mode
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED       = (int)0x00400000,      // GPS2 RTK moving base mode enabled
    GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR    = (int)0x00800000,      // GPS1 RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_DATA_MISSING     = (int)0x01000000,      // GPS1 RTK error: Either base observations or antenna position have not been received.
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MOVING  = (int)0x02000000,      // GPS1 RTK error: base position moved when it should be stationary
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_INVALID = (int)0x03000000,      // GPS1 RTK error: base position is invalid or not surveyed well
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK    = (int)0x03000000,      // GPS1 RTK error: base position error bitmask
    GPS_STATUS_FLAGS_ERROR_MASK                     = (GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR |
                                                       GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK),
    GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID        = (int)0x04000000,      // GPS1 RTK precision position and carrier phase range solution with fixed ambiguities (i.e. < 6cm horizontal accuracy).  The carrier phase range solution with floating ambiguities occurs if GPS_STATUS_FIX_RTK_FIX is set and GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID is not set (i.e. > 6cm horizontal accuracy).
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID         = (int)0x08000000,      // GPS2 RTK moving base heading.  Indicates RTK compassing heading valid and available in DID_GPS2_RTK_CMP_REL.
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD  = (int)0x00002000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET= (int)0x00004000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_MASK          = (GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET),
    GPS_STATUS_FLAGS_GPS_NMEA_DATA                  = (int)0x00008000,      // 1 = Data from NMEA message. GPS velocity is NED (not ECEF).
    GPS_STATUS_FLAGS_GPS_PPS_TIMESYNC               = (int)0x10000000,      // Time is synchronized by GPS PPS.

    GPS_STATUS_FLAGS_MASK                           = (int)0x1FFFE000,    
    GPS_STATUS_FLAGS_BIT_OFFSET                     = (int)16,

    GPS_STATUS_FLAGS_UNUSED_2                       = (int)0x20000000,
    GPS_STATUS_FLAGS_UNUSED_3                       = (int)0x40000000,
    GPS_STATUS_FLAGS_UNUSED_4                       = (int)0x80000000,
};

enum eGpsStatus2
{
    GPS_STATUS2_FLAGS_GNSS_POSSIBLE_JAM_DETECT      = (uint8_t) 0x01,
    GPS_STATUS2_FLAGS_GNSS_JAM_DETECTED             = (uint8_t) 0x02,
    GPS_STATUS2_FLAGS_GNSS_POSSIBLE_SPOOF_DETECT    = (uint8_t) 0x04,
    GPS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED           = (uint8_t) 0x08,
    
    GPS_STATUS2_FLAGS_JAM_SPOOF_POSSIBLE_MASK       = (uint8_t) 0x05,
    GPS_STATUS2_FLAGS_JAM_SPOOF_DETECTED_MASK       = (uint8_t) 0x0A,
    GPS_STATUS2_FLAGS_JAM_SPOOF_MASK                = (uint8_t) 0x0F,

    GPS_STATUS2_FLAGS_UNUSED                        = 0xF0,
};

PUSH_PACK_1

/** (DID_POSITION_MEASUREMENT) External position estimate*/
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in seconds */
    double      timeOfWeek;

    /** Position in ECEF (earth-centered earth-fixed) frame in meters */
    double      ecef[3];
    
    /** Heading with respect to NED frame (rad)*/
    float       psi;
    
    /** The Upper Diagonal of accuracy covariance matrix*/
    float       accuracyCovUD[6]; // Matrix accuracyCovUD Described below
    // 0 1 2
    // _ 3 4
    // _ _ 5
} pos_measurement_t;

/***
 * Product Hardware ID Mask  [6:4:6]
 * Product hardware ID is masked into 16 bits:
 *  [ 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 ]
 *    |- TYPE  -| |MAJOR| |- MINOR -|
 *
 *  Upper 6 bits are the hardware type (IMX, GPX, uINS, etc; 64 possible values)
 *  Middle 4 bits are the major hardware version (GPX-1, uINS-3, IMX-5, etc; 16 possible values)
 *  Lower 6 bits are the minor hardware version (IMX-6, uINS-3.2, GPX-1.0; 64 possible values)
 *
 *  If the TYPE and MAJOR are 0, then fall back to eIsHardwareType to determine the type from the legacy map:
 *      0 = Unknown
 *      1 = UINS32
 *      2 = EVB2
 *      3 = IMX5
 *      4 = GPX1
 */

#define HDW_TYPE__MASK                         0xFC00       // 6 bits, bits 10-15
#define HDW_TYPE__SHIFT                        10
#define DECODE_HDW_TYPE(x)                     (((x) & HDW_TYPE__MASK) >> HDW_TYPE__SHIFT)
// Use eIsHardwareType for hardware type
#define HDW_MAJOR__MASK                        0x03C0       // 4 bits, bits 6-9 (max value of 16)
#define HDW_MAJOR__SHIFT                       6
#define DECODE_HDW_MAJOR(x)                    (((x) & HDW_MAJOR__MASK) >> HDW_MAJOR__SHIFT)

#define HDW_MINOR__MASK                        0x003F       // 6 bits, bits 0-5 (max value of 63)
#define HDW_MINOR__SHIFT                       0
#define DECODE_HDW_MINOR(x)                    (((x) & HDW_MINOR__MASK) >> HDW_MINOR__SHIFT)

#define ENCODE_HDW_ID(type, major, minor)      ((((uint8_t)(type) << HDW_TYPE__SHIFT) & HDW_TYPE__MASK) | (((uint8_t)(major) << HDW_MAJOR__SHIFT) & HDW_MAJOR__MASK) | (((uint8_t)(minor) << HDW_MINOR__SHIFT) & HDW_MINOR__MASK))
#define ENCODE_UNIQUE_ID(hdwId, serialNo)      (((uint64_t)hdwId << 48) | (uint64_t)serialNo)
#define ENCODE_DEV_INFO_TO_HDW_ID(devinfo)     (((devinfo.hardwareType << HDW_TYPE__SHIFT) & HDW_TYPE__MASK) | ((devinfo.hardwareVer[0] << HDW_MAJOR__SHIFT) & HDW_MAJOR__MASK) | ((devinfo.hardwareVer[1] << HDW_MINOR__SHIFT) & HDW_MINOR__MASK))
#define ENCODE_DEV_INFO_TO_UNIQUE_ID(devinfo)  (((uint64_t)(ENCODE_DEV_INFO_TO_HDW_ID(devinfo)) << 48) | (uint64_t)devinfo.serialNumber)
#define DECODE_UNIQUE_ID_TO_HDW_ID(devId)      ((uint16_t)((devId >> 48) & 0xFFFF))
#define DECODE_UNIQUE_ID_TO_SERIALNO(devId)    ((uint32_t)devId)
#define DEV_INFO_MATCHES_HDW_ID(di, hdwId)     ( (ENCODE_DEV_INFO_TO_HDW_ID(di) & hdwId) == ENCODE_DEV_INFO_TO_HDW_ID(di) )

#define IS_HDW_TYPE_PERIPHERAL                 0x20        // non-peripherals are 0-31, peripherals are 32-63
enum eIsHardwareType
{
    IS_HARDWARE_TYPE_MIXED          = -1,   // Used for ci-hdw testing
    IS_HARDWARE_TYPE_UNKNOWN        = 0,
    IS_HARDWARE_TYPE_UINS           = 1,
    IS_HARDWARE_TYPE_EVB            = 2,
    IS_HARDWARE_TYPE_IMX            = 3,
    IS_HARDWARE_TYPE_GPX            = 4,
    IS_HDW_GNSS_UBLOX               = IS_HDW_TYPE_PERIPHERAL + 1,    // Ublox F9P
    IS_HDW_GNSS_SONY                = IS_HDW_TYPE_PERIPHERAL + 2,    // Sony CXD5610
    IS_HDW_GNSS_SEPTENTRIO          = IS_HDW_TYPE_PERIPHERAL + 3,    // Septentrio
    IS_HDW_GNSS_STM_TESSIO          = IS_HDW_TYPE_PERIPHERAL + 4,    // STM Tessio

    IS_HARDWARE_TYPE_COUNT          = 5     // Keep last
};

typedef uint16_t is_hardware_t;
static const is_hardware_t IS_HARDWARE_NONE     = ENCODE_HDW_ID(IS_HARDWARE_TYPE_UNKNOWN, 0, 0);
static const is_hardware_t IS_HARDWARE_ANY      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_MIXED, -1, -1);
static const is_hardware_t IS_HARDWARE_EVB_2_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_EVB, 2, 0);
static const is_hardware_t IS_HARDWARE_UINS_3_2 = ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 3, 2);
static const is_hardware_t IS_HARDWARE_IMX      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, -1, -1);
static const is_hardware_t IS_HARDWARE_IMX_5_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
static const is_hardware_t IS_HARDWARE_IMX_6_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 6, 0);
static const is_hardware_t IS_HARDWARE_GPX      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, -1, -1);
static const is_hardware_t IS_HARDWARE_GPX_1_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0);

static const is_hardware_t IS_HDW_UBLOX_F9P      = ENCODE_HDW_ID(IS_HDW_GNSS_UBLOX, 'F' - 'A', 9);
static const is_hardware_t IS_HDW_SONY_CXD5610   = ENCODE_HDW_ID(IS_HDW_GNSS_SONY, 5, 6);
static const is_hardware_t IS_HDW_TESSIO_6       = ENCODE_HDW_ID(IS_HDW_GNSS_STM_TESSIO, 6, 0);
static const is_hardware_t IS_HDW_SEPTENTRIO_G5  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'G' - 'A', 5);
static const is_hardware_t IS_HDW_SEPTENTRIO_P3  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'P' - 'A', 3);
static const is_hardware_t IS_HDW_SEPTENTRIO_M3  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'M' - 'A', 3);

extern const char* g_isHardwareTypeNames[IS_HARDWARE_TYPE_COUNT];

enum eHdwRunStates {
    HDW_STATE_UNKNOWN,
    HDW_STATE_BOOTLOADER,
    HDW_STATE_APP,
};

/** (DID_DEV_INFO) Device information */
typedef struct PACKED
{
    /** Reserved bits */
    uint16_t        reserved;

    /** Hardware Type: 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eIsHardwareType) */
    uint8_t         hardwareType;

    /** Device Run State : Bootloader, App, etc */
    uint8_t         hdwRunState;

    /** Serial number */
    uint32_t        serialNumber;

    /** Hardware version */
    uint8_t         hardwareVer[4];

    /** Firmware (software) version */
    uint8_t         firmwareVer[4];

    /** Build number */
    uint32_t        buildNumber;

    /** Communications protocol version */
    uint8_t         protocolVer[4];

    /** Repository revision number */
    uint32_t        repoRevision;

    /** Manufacturer name */
    char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];

    /** Build type (Release: 'a'=ALPHA, 'b'=BETA, 'c'=RELEASE CANDIDATE, 'r'=PRODUCTION RELEASE, 'd'=developer/debug) */
    uint8_t         buildType;
    
    /** Build date year - 2000 */
    uint8_t         buildYear;
    /** Build date month */
    uint8_t         buildMonth;
    /** Build date day */
    uint8_t         buildDay;

    /** Build time hour */
    uint8_t         buildHour;
    /** Build time minute */
    uint8_t         buildMinute;
    /** Build time second */
    uint8_t         buildSecond;
    /** Build time millisecond */
    uint8_t         buildMillisecond;

    /** Additional info */
    char            addInfo[DEVINFO_ADDINFO_STRLEN];

    /** Firmware MD5 hash */
    // uint32_t        firmwareMD5Hash[4];

} dev_info_t;

/** Add missing hardware descriptor to dev_info_t. */
void devInfoPopulateMissingHardware(dev_info_t *devInfo);

/**
 * @brief Convert NMEA talker string to ID (eNmeaMsgId).
 * 
 * @param a NMEA talker string
 * @param aSize Length of the talker string
 * @return int NMEA ID (eNmeaMsgId) on success or negative for failure. -1 for NMEA head not found, -2 for invalid length, -3 other error
 */
int getNmeaMsgId(const void* msg, int msgSize);

/**
 * @brief Convert NMEA ID (eNmeaMsgId) to talker string
 * 
 * @param msgId NMEA ID (eNmeaMsgId)
 * @param buf Talker id string output 
 * @param bufSize Max size of buffer talker string will be written to.  Must be 5 or larger.
 * @return int 0 on success, -1 on failure.
 */
int nmeaMsgIdToTalker(int msgId, void *buf, int bufSize);

/**
 * @brief Get RTCM ID from 
 * 
 * @param buff 
 * @param pos 
 * @param len 
 * @return unsigned int 
 */
unsigned int messageStatsGetbitu(const unsigned char *buff, int pos, int len);

#define RTCM3_MSG_ID(msg)       messageStatsGetbitu((const unsigned char*)msg, 24, 12)
#define RTCM3_MSG_LENGTH(msg)   messageStatsGetbitu((const unsigned char*)msg, 14, 10)

/** (DID_MANUFACTURING_INFO) Manufacturing info */
typedef struct PACKED
{
    /** Inertial Sense serial number */
    uint32_t    serialNumber;

    /** Hardware ID: This is a packed identifier, which includes the Hardware Type, hardwareVer Major, and hardwareVer Minor */
    uint16_t    hardwareId;

    /** Inertial Sense lot number */
    uint16_t    lotNumber;

    /** Inertial Sense manufacturing date (YYYYMMDDHHMMSS) */
    char        date[16];

    /** Key - write: unlock manufacturing info, read: number of times OTP has been set, 15 max */
    uint32_t    key;

    /** Platform / carrier board (ePlatformConfig::PLATFORM_CFG_TYPE_MASK).  Only valid if greater than zero. */
    int32_t     platformType;

    int32_t     reserved;

    /** Microcontroller unique identifier, 128 bits for SAM / 96 for STM32 */
    uint32_t    uid[4];
} manufacturing_info_t;

/** (DID_INS_1) INS output: euler rotation w/ respect to NED, NED position from reference LLA */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t    week;
    
    /** GPS time of week (since Sunday morning) in seconds */
    double      timeOfWeek;

    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t    insStatus;

    /** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
    uint32_t    hdwStatus;

    /** Euler angles: roll, pitch, yaw in radians with respect to NED */
    float       theta[3];

    /** Velocity U, V, W in meters per second.  Convert to NED velocity using "vectorBodyToReference(uvw, theta, vel_ned)". */
    float       uvw[3];

    /** WGS84 latitude, longitude, height above ellipsoid (degrees,degrees,meters) */
    double      lla[3];

    /** North, east and down (meters) offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
    float       ned[3];
} ins_1_t;


/** (DID_INS_2) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t    week;
    
    /** GPS time of week (since Sunday morning) in seconds */
    double      timeOfWeek;

    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t    insStatus;

    /** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
    uint32_t    hdwStatus;

    /** Quaternion body rotation with respect to NED: W, X, Y, Z */
    float       qn2b[4];

    /** Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
    float       uvw[3];

    /** WGS84 latitude, longitude, height above ellipsoid in meters (not MSL) */
    double      lla[3];
} ins_2_t;


/** (DID_INS_3) INS output: quaternion rotation w/ respect to NED, msl altitude */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t    week;
    
    /** GPS time of week (since Sunday morning) in seconds */
    double      timeOfWeek;

    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t    insStatus;

    /** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
    uint32_t    hdwStatus;

    /** Quaternion body rotation with respect to NED: W, X, Y, Z */
    float       qn2b[4];

    /** Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
    float       uvw[3];

    /** WGS84 latitude, longitude, height above ellipsoid in meters (not MSL) */
    double      lla[3];

    /** height above mean sea level (MSL) in meters */
    float       msl;
} ins_3_t;


/** (DID_INS_4) INS output: quaternion rotation w/ respect to ECEF, ECEF position */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t    week;
    
    /** GPS time of week (since Sunday morning) in seconds */
    double      timeOfWeek;

    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t    insStatus;

    /** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
    uint32_t    hdwStatus;

    /** Quaternion body rotation with respect to ECEF: W, X, Y, Z */
    float       qe2b[4];

    /** Velocity in ECEF (earth-centered earth-fixed) frame in meters per second */
    float       ve[3];

    /** Position in ECEF (earth-centered earth-fixed) frame in meters */
    double      ecef[3];
} ins_4_t;


/** Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
    /** Gyroscope P, Q, R in radians / second */
    float       pqr[3];

    /** Acceleration X, Y, Z in meters / second squared */
    float       acc[3];
} imus_t;


/** (DID_IMU, DID_REFERENCE_IMU) Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double      time;

    /** IMU Status (eImuStatus) */
    uint32_t    status;

    /** Inertial Measurement Unit (IMU) */
    imus_t      I;
} imu_t;


/** (DID_IMU3_UNCAL) Dual Inertial Measurement Units (IMUs) data */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;

    /** IMU Status (eImuStatus) */
    uint32_t                status;

    /** Inertial Measurement Units (IMUs) */
    imus_t                  I[3];

} imu3_t;


/** (DID_MAGNETOMETER) Magnetometer sensor data */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;
    
    /** Magnetometers */
    float                   mag[3];
} magnetometer_t;


/** (DID_BAROMETER) Barometric pressure sensor data */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;
    
    /** Barometric pressure in kilopascals */
    float                   bar;

    /** MSL altitude from barometric pressure sensor in meters */
    float                   mslBar;

    /** Temperature of barometric pressure sensor in Celsius */
    float                   barTemp;

    /** Relative humidity as a percent (%rH). Range is 0% - 100% */
    float                   humidity;
} barometer_t;


/** (DID_PIMU, DID_REFERENCE_PIMU) Preintegraed IMU (a.k.a. Coning and Sculling integral) in body/IMU frame. */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;

    /** Integral period in seconds for delta theta and delta velocity.  This is configured using DID_FLASH_CONFIG.startupNavDtMs. */
    float                    dt;

    /** IMU Status (eImuStatus) */
    uint32_t                status;

    /** IMU delta theta (gyroscope {p,q,r} integral) in radians in sensor frame */
    float                   theta[3];

    /** IMU delta velocity (accelerometer {x,y,z} integral) in m/s in sensor frame */
    float                   vel[3];
} pimu_t;


/** (DID_IMU_MAG) imu + mag */
typedef struct PACKED
{
    /** imu - raw or pre-integrated depending on data id */
    imu_t imu;
    
    /** mag */
    magnetometer_t mag;
} imu_mag_t;


/** (DID_PIMU_MAG) preintegrated imu + mag */
typedef struct PACKED
{
    /** Preintegrated IMU */
    pimu_t pimu;
    
    /** Magnetometer */
    magnetometer_t mag;
} pimu_mag_t;


/** IMU Status */
enum eImuStatus
{
    /** Sensor saturation on IMU1 gyro */
    IMU_STATUS_SATURATION_IMU1_GYR              = (int)0x00000001,
    /** Sensor saturation on IMU2 gyro */
    IMU_STATUS_SATURATION_IMU2_GYR              = (int)0x00000002,
    /** Sensor saturation on IMU3 gyro */
    IMU_STATUS_SATURATION_IMU3_GYR              = (int)0x00000004,
    /** Sensor saturation on IMU1 accelerometer */
    IMU_STATUS_SATURATION_IMU1_ACC              = (int)0x00000008,
    /** Sensor saturation on IMU2 accelerometer */
    IMU_STATUS_SATURATION_IMU2_ACC              = (int)0x00000010,
    /** Sensor saturation on IMU3 accelerometer */
    IMU_STATUS_SATURATION_IMU3_ACC              = (int)0x00000020,
    /** Sensor saturation mask */
    IMU_STATUS_SATURATION_MASK                  = (int)0x0000003F,

    /** Sensor shock detected */
    IMU_STATUS_SHOCK_PRESENT                    = (int)0x00000040,

    /** Magnetometer sample occurred */
    IMU_STATUS_MAG_UPDATE                        = (int)0x00000100,
    /** Data was received at least once from Reference IMU */
    IMU_STATUS_REFERENCE_IMU_PRESENT            = (int)0x00000200,
    /** Reserved */
    // IMU_STATUS_RESERVED2                        = (int)0x00000400,

//     /** Sensor saturation happened within past 10 seconds */
//     IMU_STATUS_SATURATION_HISTORY               = (int)0x00000100,
//     /** Sample rate fault happened within past 10 seconds */
//     IMU_STATUS_SAMPLE_RATE_FAULT_HISTORY        = (int)0x00000200,

    /** IMU1 gyros available */
    IMU_STATUS_GYR1_OK                          = (int)0x00010000,
    /** IMU2 gyros and accelerometers available */
    IMU_STATUS_GYR2_OK                          = (int)0x00020000,
    /** IMU3 gyros available */
    IMU_STATUS_GYR3_OK                          = (int)0x00040000,
    /** IMU1 accelerometers available */
    IMU_STATUS_ACC1_OK                          = (int)0x00080000,
    /** IMU2 accelerometers available */
    IMU_STATUS_ACC2_OK                          = (int)0x00100000,
    /** IMU3 accelerometers available */
    IMU_STATUS_ACC3_OK                          = (int)0x00200000,
    /** IMU1 available */
    IMU_STATUS_IMU1_OK                          = (int)(IMU_STATUS_GYR1_OK | IMU_STATUS_ACC1_OK),
    /** IMU2 available */
    IMU_STATUS_IMU2_OK                          = (int)(IMU_STATUS_GYR2_OK | IMU_STATUS_ACC2_OK),
    /** IMU3 available */
    IMU_STATUS_IMU3_OK                          = (int)(IMU_STATUS_GYR3_OK | IMU_STATUS_ACC3_OK),
    /** IMU gyros and accelerometers available */
    IMU_STATUS_IMU_OK_MASK                      = (int)0x003F0000,

    /** IMU fault rejection is excluding one of the gyros from the combined IMU output */
    IMU_STATUS_GYR_FAULT_REJECT                 = (int)0x01000000,
    /** IMU fault rejection is excluding one of the accelerometers from the combined IMU output */
    IMU_STATUS_ACC_FAULT_REJECT                 = (int)0x02000000,
};

/** (DID_GPS1_POS, DID_GPS1_RCVR_POS, DID_GPS2_POS) GPS position data */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t                week;

    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
    uint32_t                status;

    /** Position in ECEF {x,y,z} (m) */
    double                  ecef[3];
    
    /** Position - WGS84 latitude, longitude, height above ellipsoid (not MSL) (degrees, m) */
    double                  lla[3];

    /** Height above mean sea level (MSL) in meters */
    float                   hMSL;

    /** Horizontal accuracy in meters */
    float                   hAcc;

    /** Vertical accuracy in meters */
    float                   vAcc;

    /** Position dilution of precision (unitless) */
    float                   pDop;

    /** Average of all non-zero satellite carrier to noise ratios (signal strengths) in dBHz */
    float                   cnoMean;

    /** Time sync offset between local time since boot up to GPS time of week in seconds.  Add this to IMU and sensor time to get GPS time of week in seconds. */
    double                  towOffset;
    
    /** GPS leap second (GPS-UTC) offset. Receiver's best knowledge of the leap seconds offset from UTC to GPS time. Subtract from GPS time of week to get UTC time of week. (18 seconds as of December 31, 2016) */
    uint8_t                 leapS;

    /** Number of satellites used */
    uint8_t                 satsUsed;

    /** Standard deviation of cnoMean over past 5 seconds (dBHz x10) */
    uint8_t                 cnoMeanSigma;

    /** (see eGpsStatus2) GPS status2: [0x0X] Spoofing/Jamming status, [0xX0] Unused */
    uint8_t                 status2;

} gps_pos_t;


/** (DID_GPS1_VEL, DID_GPS2_VEL) GPS velocity data */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** GPS Velocity.  Velocity is in ECEF {vx,vy,vz} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is NOT set.  Velocity is in local tangent plane with no vertical velocity {vNorth, vEast, 0} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is set. */
    float                    vel[3];    

    /** Speed accuracy in meters / second */
    float                    sAcc;
    
    /** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
    uint32_t                status;
} gps_vel_t;


/** GPS Satellite information */
typedef struct PACKED
{
    /** GNSS identifier (see eSatSvGnssId) */
    uint8_t                 gnssId;

    /** Satellite identifier */
    uint8_t                 svId;

    /** (deg) Elevation (range: +/-90) */
    int8_t                  elev;

    /** (deg) Azimuth (range: +/-180) */
    int16_t                 azim;

    /** (dBHz) Carrier to noise ratio (signal strength) */
    uint8_t                 cno;

    /** (see eSatSvStatus) */
    uint16_t                status;
} gps_sat_sv_t;

/** Sat SV - GNSS System ID */
enum eSatSvGnssId
{
    SAT_SV_GNSS_ID_UNKNOWN  = 0,
    SAT_SV_GNSS_ID_GNSS     = 0,    // (multi-constellation)
    SAT_SV_GNSS_ID_GPS      = 1,    // GPS (USA)
    SAT_SV_GNSS_ID_SBS      = 2,    // SBAS (multiple regional systems, see flash config for selection)
    SAT_SV_GNSS_ID_GAL      = 3,    // Galileo (European Union)    
    SAT_SV_GNSS_ID_BEI      = 4,    // BeiDou (China)
    SAT_SV_GNSS_ID_QZS      = 5,    // QZSS (Japan)
    SAT_SV_GNSS_ID_GLO      = 6,    // GLONASS (Russia)    
    SAT_SV_GNSS_ID_IRN      = 7,    // IRNSS / NavIC (India)    
    SAT_SV_GNSS_ID_IME      = 8,    // IMES (Japan's Indoor Messaging System)
    SAT_SV_GNSS_ID_COUNT    = 9,    // Number of constellations
};

/** GPS Sat Status */
enum eSatSvStatus
{
    SAT_SV_STATUS_SIGNAL_QUALITY_MASK               = 0x0007,   // see eSatSigQuality
    SAT_SV_STATUS_USED_IN_SOLUTION                  = 0x0008,    // Used in the solution
    SAT_SV_STATUS_USED_IN_SOLUTION_OFFSET           = 3,
    SAT_SV_STATUS_HEALTH_UNKNOWN                    = 0x0000,    // 0 = unknown
    SAT_SV_STATUS_HEALTH_GOOD                       = 0x0010,    // 1 = healthy
    SAT_SV_STATUS_HEALTH_BAD                        = 0x0020,    // 2 = unhealthy
    SAT_SV_STATUS_HEALTH_MASK                       = 0x0030,
    SAT_SV_STATUS_HEALTH_OFFSET                     = 4,

    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_MASK           = 0x0300,    // 1=float, 2=fix
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_OFFSET         = 8,
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FLOAT          = 1,    
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FIX            = 2,    

    SAT_SV_STATUS_RTK_EPH_RTCM_PULSE                = 0x1000,   // Set when eph is Rx for this SV and cleared by next message send
    SAT_SV_STATUS_RTK_EPH_RTK_LIB_PULSE             = 0x2000,   // Set when eph is added to RTK LIB for this SV 
    SAT_SV_STATUS_RTK_EPH_PULSE_MASK                = SAT_SV_STATUS_RTK_EPH_RTK_LIB_PULSE | SAT_SV_STATUS_RTK_EPH_RTCM_PULSE,   // Set when eph is Rx for this SV and cleared by next message send

    SAT_SV_STATUS_RTK_EPH_SEND_PULSE                = 0x4000,   // Set when eph is Tx'd from device

    // SAT_SV_STATUS_HEALTH_MASK                       = 0x00000030,
    // NAV_SAT_FLAGS_HEALTH_OFFSET                     = 4,
    // SAT_SV_STATUS_DIFFCORR                          = 0x00000040,
    // SAT_SV_STATUS_SMOOTHED                          = 0x00000080,
    // SAT_SV_STATUS_ORBITSOURCE_MASK                  = 0x00000700,
    // SAT_SV_STATUS_ORBITSOURCE_OFFSET                = 8,
    // SAT_SV_STATUS_EPHAVAIL                          = 0x00000800,
    // SAT_SV_STATUS_ALMAVAIL                          = 0x00001000,
    // SAT_SV_STATUS_ANOAVAIL                          = 0x00002000,
    // SAT_SV_STATUS_AOPAVAIL                          = 0x00004000,    
};

/** (DID_GPS1_SAT, DID_GPS2_SAT) GPS satellite information */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;                
    /** Number of satellites in the sky */
    uint32_t                numSats;                    
    /** Satellite information list */
    gps_sat_sv_t            sat[MAX_NUM_SATELLITES];    
} gps_sat_t;

enum eSatSvSigId
{
    SAT_SV_SIG_ID_GPS_L1CA          = 0,
    SAT_SV_SIG_ID_GPS_L2CL          = 3,
    SAT_SV_SIG_ID_GPS_L2CM          = 4,
    SAT_SV_SIG_ID_GPS_L5I           = 6,
    SAT_SV_SIG_ID_GPS_L5Q           = 7,
    SAT_SV_SIG_ID_GPS_L5            = SAT_SV_SIG_ID_GPS_L5Q,

    SAT_SV_SIG_ID_SBAS_L1CA         = 0,
    SAT_SV_SIG_ID_SBAS_L2           = 1,
    SAT_SV_SIG_ID_SBAS_L5           = 2,

    SAT_SV_SIG_ID_Galileo_E1C2      = 0,
    SAT_SV_SIG_ID_Galileo_E1B2      = 1,
    SAT_SV_SIG_ID_Galileo_E1BC      = SAT_SV_SIG_ID_Galileo_E1B2,
    SAT_SV_SIG_ID_Galileo_E5aI      = 3,
    SAT_SV_SIG_ID_Galileo_E5aQ      = 4,
    SAT_SV_SIG_ID_Galileo_E5a       = SAT_SV_SIG_ID_Galileo_E5aQ,
    SAT_SV_SIG_ID_Galileo_E5bI      = 5,
    SAT_SV_SIG_ID_Galileo_E5bQ      = 6,
    SAT_SV_SIG_ID_Galileo_E5        = SAT_SV_SIG_ID_Galileo_E5bQ,

    SAT_SV_SIG_ID_BeiDou_B1D1       = 0,
    SAT_SV_SIG_ID_BeiDou_B1D2       = 1,
    SAT_SV_SIG_ID_BeiDou_B2D1       = 2,
    SAT_SV_SIG_ID_BeiDou_B2D2       = 3,
    SAT_SV_SIG_ID_BeiDou_B2         = SAT_SV_SIG_ID_BeiDou_B2D1,
    SAT_SV_SIG_ID_BeiDou_B1C        = 5,
    SAT_SV_SIG_ID_BeiDou_B2a        = 7,

    SAT_SV_SIG_ID_QZSS_L1CA         = 0,
    SAT_SV_SIG_ID_QZSS_L1S          = 1,
    SAT_SV_SIG_ID_QZSS_L2CM         = 4,
    SAT_SV_SIG_ID_QZSS_L2CL         = 5,
    SAT_SV_SIG_ID_QZSS_L2           = SAT_SV_SIG_ID_QZSS_L2CL,
    SAT_SV_SIG_ID_QZSS_L5I          = 8,
    SAT_SV_SIG_ID_QZSS_L5Q          = 9,
    SAT_SV_SIG_ID_QZSS_L5           = SAT_SV_SIG_ID_QZSS_L5Q,

    SAT_SV_SIG_ID_GLONASS_L1OF      = 0,
    SAT_SV_SIG_ID_GLONASS_L2OF      = 2,

    SAT_SV_SIG_ID_NAVIC_L5A         = 0, 
};

enum eSatSigQuality
{
    SAT_SIG_QUALITY_NO_SIGNAL                   = 0,     // no signal
    SAT_SIG_QUALITY_SEARCHING                   = 1,     // searching signal
    SAT_SIG_QUALITY_ACQUIRED                    = 2,     // signal acquired
    SAT_SIG_QUALITY_DETECTED                    = 3,     // signal detected but unusable
    SAT_SIG_QUALITY_CODE_LOCK_TIME_SYNC         = 4,     // code locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_1    = 5,     // code and carrier locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_2    = 6,     // "
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_3    = 7,     // "
};

enum eSatSigStatus
{
    SAT_SIG_STATUS_HEALTH_UNKNOWN               = 0x0000,    // 0 = unknown
    SAT_SIG_STATUS_HEALTH_GOOD                  = 0x0001,    // 1 = healthy
    SAT_SIG_STATUS_HEALTH_BAD                   = 0x0002,    // 2 = unhealthy
    SAT_SIG_STATUS_HEALTH_MASK                  = 0x0003,
    SAT_SIG_STATUS_USED_IN_SOLUTION             = 0x0004,  // Signal is used in the solution
    SAT_SIG_STATUS_USED_IN_SOLUTION_OFFSET      = 2,
};


/** GPS satellite signal information */
typedef struct PACKED
{
    /** GNSS identifier (see eSatSvGnssId) */
    uint8_t                    gnssId;

    /** Satellite identifier */
    uint8_t                    svId;

    /** Signal identifier, frequency description (eSatSvSigId) */
    uint8_t                    sigId;

    /** (dBHz) Carrier to noise ratio (signal strength) */
    uint8_t                    cno;

    /** Quality indicator (see eSatSigQuality) */
    uint8_t                    quality;

    /** Status flags (see eSatSigStatus) */
    uint16_t                status;

} gps_sig_sv_t;

/** (DID_GPS1_SIG, DID_GPS2_SIG) GPS satellite signal information */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;                
    /** Number of satellite signals in the following satelliate signal list */
    uint32_t                numSigs;                    
    /** Satellite signal list */
    gps_sig_sv_t            sig[MAX_NUM_SAT_SIGNALS];    
} gps_sig_t;

typedef uint8_t         gps_extension_ver_t[30];
#define GPS_VER_NUM_EXTENSIONS    6
/** (DID_GPS1_VERSION) GPS version strings */
typedef struct PACKED
{
    /** Software version */
    uint8_t                 swVersion[30];
    /** Hardware version */
    uint8_t                 hwVersion[10];        
    /** Extension 30 bytes array description  */
    gps_extension_ver_t     extension[GPS_VER_NUM_EXTENSIONS];        
} gps_version_t;

// (DID_INL2_STATES) INL2 - INS Extended Kalman Filter (EKF) states
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in seconds */
    double                  timeOfWeek;                    

    /** Quaternion body rotation with respect to ECEF */
    float                   qe2b[4];                    

    /** (m/s) Velocity in ECEF frame */
    float                   ve[3];                        

    /** (m)     Position in ECEF frame */
    double                  ecef[3];                

    /** (rad/s) Gyro bias */
    float                   biasPqr[3];               
    
    /** (m/s^2) Accelerometer bias */
    float                   biasAcc[3];                
    
    /** (m)     Barometer bias */
    float                   biasBaro;               
    
    /** (rad)   Magnetic declination */
    float                   magDec;                 
    
    /** (rad)   Magnetic inclination */
    float                   magInc;                 
} inl2_states_t;

// (DID_ROS_COVARIANCE_POSE_TWIST) INL2 - INS Extended Kalman Filter (EKF) state covariance
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in seconds */
    double                  timeOfWeek;

    /** Packed 6x6 lower-diagonal covariance matrix (21 values, row-major) for EKF pose errors:
     *  - Attitude (roll,pitch,yaw) error (body frame, rad²)
     *  - Position (x,y,z) error (ECEF frame, m²)
     *  Index layout: 
     *    0 __ __ __ __ __
     *    1  2 __ __ __ __
     *    3  4  5 __ __ __
     *    6  7  8  9 __ __
     *   10 11 12 13 14 __
     *   15 16 17 18 19 20  */
    float                    covPoseLD[21];

    /** Packed 6x6 lower-diagonal covariance matrix (21 values, row-major) for EKF twist errors:
     *  - Velocity (x,y,z) error (ECEF frame, (m/s)^2)
     *  - Angular rate (p,q,r) error (body frame, (rad/s)^2)
     *  Index layout: 
     *   0 __ __ __ __ __
     *   1  2 __ __ __ __
     *   3  4  5 __ __ __
     *   6  7  8  9 __ __
     *  10 11 12 13 14 __
     *  15 16 17 18 19 20  */
    float                    covTwistLD[21];
} ros_covariance_pose_twist_t;

// (DID_INL2_STATUS)
typedef struct PACKED
{
    int             ahrs;
    int             zero_accel;
    int             zero_angrate;
    int             accel_motion;
    int             rot_motion;
    int             zero_vel;
    int             ahrs_gps_cnt;       // Counter of sequential valid GPS data (for switching from AHRS to navigation)
    float           hdg_err;
    int             hdg_coarse;         // Flag whether initial attitude error converged
    int             hdg_aligned;        // Flag whether initial attitude error converged
    int             hdg_aligning;
    int             ekf_init_done;      // Hot EKF initialization completed
    int             mag_cal_good;
    int             mag_cal_done;
    int             stat_magfield;
} inl2_status_t;

/** Generic 1 axis sensor */
typedef struct PACKED
{
    /** Time in seconds */
    double                  time;

    /** Three axis sensor */
    float                   val;
} gen_1axis_sensor_t;

/** Generic 3 axis sensor */
typedef struct PACKED
{
    /** Time in seconds */
    double                  time;

    /** Three axis sensor */
    float                   val[3];
} gen_3axis_sensor_t;

/** Generic dual 3 axis sensor */
typedef struct PACKED
{
    /** Time in seconds */
    double                  time;

    /** First three axis sensor */
    float                   val1[3];

    /** Second three axis sensor */
    float                   val2[3];
} gen_dual_3axis_sensor_t;

/** Generic 3 axis sensor */
typedef struct PACKED
{
    /** Time in seconds */
    double                  time;

    /** Three axis sensor */
    double                  val[3];
} gen_3axis_sensord_t;

/** (DID_SYS_SENSORS) Output from system sensors */
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;

    /** Temperature in Celsius */
    float                   temp;

    /** Gyros in radians / second */
    float                   pqr[3];

    /** Accelerometers in meters / second squared */
    float                   acc[3];

    /** Magnetometers */
    float                   mag[3];

    /** Barometric pressure in kilopascals */
    float                   bar;

    /** Temperature of barometric pressure sensor in Celsius */
    float                   barTemp;

    /** MSL altitude from barometric pressure sensor in meters */
    float                   mslBar;
    
    /** Relative humidity as a percent (%rH). Range is 0% - 100% */
    float                   humidity;

    /** EVB system input voltage in volts. uINS pin 5 (G2/AN2).  Use 10K/1K resistor divider between Vin and GND.  */
    float                   vin;

    /** ADC analog input in volts. uINS pin 4, (G1/AN1). */
    float                   ana1;

    /** ADC analog input in volts. uINS pin 19 (G3/AN3). */
    float                   ana3;

    /** ADC analog input in volts. uINS pin 20 (G4/AN4). */
    float                   ana4;
} sys_sensors_t;

/** INS output */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Latitude, longitude and height above ellipsoid (rad, rad, m) */
    double                  lla[3];

    /** Velocities in body frames of X, Y and Z (m/s) */
    float                   uvw[3];

    /** Quaternion body rotation with respect to NED: W, X, Y, Z */
    float                    qn2b[4];
} ins_output_t;

/** (DID_SYS_PARAMS) System parameters */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** INS status flags (eInsStatusFlags) */
    uint32_t                insStatus;

    /** Hardware status flags (eHdwStatusFlags) */
    uint32_t                hdwStatus;

    /** IMU temperature */
    float                   imuTemp;

    /** Baro temperature */
    float                   baroTemp;

    /** MCU temperature (not available yet) */
    float                   mcuTemp;

    /** System status flags (eSysStatusFlags) */
    uint32_t                sysStatus;

    /** IMU sample period (ms). Zero disables sampling. */
    uint32_t                imuSamplePeriodMs;

    /** Preintegrated IMU (PIMU) integration period and navigation/AHRS filter output period (ms). */
    uint32_t                navOutputPeriodMs;
    
    /** Actual sample period relative to GPS PPS (sec) */
    double                  sensorTruePeriod;

    /** Flash config checksum used with host SDK synchronization */
    uint32_t                flashCfgChecksum;

    /** Navigation/AHRS filter update period (ms) */
    uint32_t                navUpdatePeriodMs;

    /** General fault code descriptor (eGenFaultCodes).  Set to zero to reset fault code. */
    uint32_t                genFaultCode;

    /** System up time in seconds (with double precision) */
    double                  upTime;

} sys_params_t;

/*! General Fault Code descriptor */
enum eGenFaultCodes
{
    /*! INS state limit overrun - UVW */
    GFC_INS_STATE_ORUN_UVW                  = 0x00000001,
    /*! INS state limit overrun - Latitude */
    GFC_INS_STATE_ORUN_LAT                  = 0x00000002,
    /*! INS state limit overrun - Altitude */
    GFC_INS_STATE_ORUN_ALT                  = 0x00000004,
    /*! Unhandled interrupt */
    GFC_UNHANDLED_INTERRUPT                 = 0x00000010,
    /*! GNSS receiver critical fault. See the corresponding GPS status fault flags (i.e. GPX_STATUS_FATAL_MASK) */
    GFC_GNSS_CRITICAL_FAULT                 = 0x00000020,
    /*! GNSS Tx limited */
    GFC_GNSS_TX_LIMITED                     = 0x00000040,
    /*! GNSS Rx overrun */
    GFC_GNSS_RX_OVERRUN                     = 0x00000080,
    /*! Fault: sensor initialization  */
    GFC_INIT_SENSORS                        = 0x00000100,
    /*! Fault: SPI bus initialization  */
    GFC_INIT_SPI                            = 0x00000200,
    /*! Fault: SPI configuration  */
    GFC_CONFIG_SPI                          = 0x00000400,
    /*! Fault: GNSS1 init  */
    GFC_GNSS1_INIT                          = 0x00000800,
    /*! Fault: GNSS2 init  */
    GFC_GNSS2_INIT                          = 0x00001000,
    /*! Flash failed to load valid values */
    GFC_FLASH_INVALID_VALUES                = 0x00002000,
    /*! Flash checksum failure */
    GFC_FLASH_CHECKSUM_FAILURE              = 0x00004000,
    /*! Flash write failure */
    GFC_FLASH_WRITE_FAILURE                 = 0x00008000,
    /*! System Fault: general */
    GFC_SYS_FAULT_GENERAL                   = 0x00010000,
    /*! System Fault: CRITICAL system fault (see DID_SYS_FAULT) */
    GFC_SYS_FAULT_CRITICAL                  = 0x00020000,
    /*! Sensor(s) saturated */
    GFC_SENSOR_SATURATION                   = 0x00040000,

    /*! Fault: IMU initialization */
    GFC_INIT_IMU                            = 0x00100000,
    /*! Fault: Barometer initialization */
    GFC_INIT_BAROMETER                      = 0x00200000,
    /*! Fault: Magnetometer initialization */
    GFC_INIT_MAGNETOMETER                   = 0x00400000,
    /*! Fault: I2C initialization */
    GFC_INIT_I2C                            = 0x00800000,
    /*! Fault: Chip erase line toggled but did not meet required hold time.  This is caused by noise/transient on chip erase pin. */
    GFC_CHIP_ERASE_INVALID                  = 0x01000000,
    /*! Fault: EKF GPS time fault */
    GFC_EKF_GNSS_TIME_FAULT                 = 0x02000000,
    /*! Fault: GPS receiver time fault */
    GFC_GNSS_RECEIVER_TIME                  = 0x04000000,
    /*! Fault: GNSS reciever ceneral fault. See the corresponding GPS status fault flags (i.e. GPX_STATUS_GENERAL_FAULT_MASK) */
    GFC_GNSS_GENERAL_FAULT                  = 0x08000000,

    /*! IMX GFC flags that relate to GPX status flags */
    GFC_GPX_STATUS_COMMON_MASK = GFC_GNSS1_INIT | GFC_GNSS2_INIT | GFC_GNSS_TX_LIMITED | GFC_GNSS_RX_OVERRUN | GFC_GNSS_CRITICAL_FAULT | GFC_GNSS_RECEIVER_TIME | GFC_GNSS_GENERAL_FAULT,
};


/** (DID_SYS_CMD) System Commands */
typedef struct PACKED
{
    /** System commands (see eSystemCommand) 1=save current persistent messages, 5=zero motion, 97=save flash, 99=software reset.  "invCommand" (following variable) must be set to bitwise inverse of this value for this command to be processed.  */
    uint32_t                command;

    /** Error checking field that must be set to bitwise inverse of command field for the command to take effect.  */
    uint32_t                invCommand;

} system_command_t;

enum eSystemCommand 
{
    SYS_CMD_NONE                                        = 0,            // (uint32 inv: 4294967295)
    SYS_CMD_SAVE_PERSISTENT_MESSAGES                    = 1,            // (uint32 inv: 4294967294)
    SYS_CMD_ENABLE_BOOTLOADER_AND_RESET                 = 2,            // (uint32 inv: 4294967293)
    SYS_CMD_ENABLE_SENSOR_STATS                         = 3,            // (uint32 inv: 4294967292)
    SYS_CMD_ENABLE_RTOS_STATS                           = 4,            // (uint32 inv: 4294967291)
    SYS_CMD_ZERO_MOTION                                 = 5,            // (uint32 inv: 4294967290)
    SYS_CMD_REF_POINT_STATIONARY                        = 6,            // (uint32 inv: 4294967289)
    SYS_CMD_REF_POINT_MOVING                            = 7,            // (uint32 inv: 4294967288)
    SYS_CMD_RESET_RTOS_STATS                            = 8,            // (uint32 inv: 4294967287)

    SYS_CMD_ENABLE_GPS_LOW_LEVEL_CONFIG                 = 10,           // (uint32 inv: 4294967285)
    SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE                  = 11,           // (uint32 inv: 4294967284)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS1       = 12,           // (uint32 inv: 4294967283)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS2       = 13,           // (uint32 inv: 4294967282)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0       = 14,           // (uint32 inv: 4294967281)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1       = 15,           // (uint32 inv: 4294967280)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2       = 16,           // (uint32 inv: 4294967279)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GPS1      = 17,           // (uint32 inv: 4294967278)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS1  = 18,           // (uint32 inv: 4294967277)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS2  = 19,           // (uint32 inv: 4294967276)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_USB   = 20,           // (uint32 inv: 4294967275)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER0  = 21,           // (uint32 inv: 4294967274)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER1  = 22,           // (uint32 inv: 4294967273)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER2  = 23,           // (uint32 inv: 4294967272)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_LOOPBACK      = 24,           // (uint32 inv: 4294967271)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_LOOPBACK     = 25,           // (uint32 inv: 4294967270)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER1_LOOPBACK     = 26,           // (uint32 inv: 4294967269)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER2_LOOPBACK     = 27,           // (uint32 inv: 4294967268)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK = 28,           // (uint32 inv: 4294967267)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE = 29,  // (uint32 inv: 4294967266)

    SYS_CMD_GPX_ENABLE_BOOTLOADER_MODE                  = 30,           // (uint32 inv: 4294967265)
    SYS_CMD_GPX_ENABLE_GNSS1_CHIPSET_BOOTLOADER         = 31,           // (uint32 inv: 4294967264)
    SYS_CMD_GPX_ENABLE_GNSS2_CHIPSET_BOOTLOADER         = 32,           // (uint32 inv: 4294967263)
    SYS_CMD_GPX_ENABLE_GNSS1_PASS_THROUGH               = 33,           // (uint32 inv: 4294967262)
    SYS_CMD_GPX_ENABLE_GNSS2_PASS_THROUGH               = 34,           // (uint32 inv: 4294967261)
    SYS_CMD_GPX_HARD_RESET_GNSS1                        = 36,           // (uint32 inv: 4294967259)
    SYS_CMD_GPX_HARD_RESET_GNSS2                        = 37,           // (uint32 inv: 4294967258)
    SYS_CMD_GPX_SOFT_RESET_GPX                          = 38,           // (uint32 inv: 4294967257)
    SYS_CMD_GPX_ENABLE_SERIAL_BRIDGE_CUR_PORT_LOOPBACK  = 39,           // (uint32 inv: 4294967256) // Enables serial bridge on IMX to GPX and loopback on GPX.
    SYS_CMD_GPX_ENABLE_SERIAL_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE  = 40,  // (uint32 inv: 4294967255) // Enables serial bridge on IMX to GPX and loopback on GPX (driver test mode).
    SYS_CMD_GPX_ENABLE_RTOS_STATS                       = 41,           // (uint32 inv: 4294967254)

    SYS_CMD_GNSS_RCVR_QUIET_MODE                        = 60,           // (uint32 inv: 4294967235) 
    SYS_CMD_GNSS_RCVR_SOFT_RESET                        = 61,           // (uint32 inv: 4294967287)
    SYS_CMD_GNSS_RCVR_HARD_RESET                        = 62,           // (uint32 inv: 4294967287)
    
    SYS_CMD_SAVE_FLASH                                  = 97,           // (uint32 inv: 4294967198)
    SYS_CMD_SAVE_GPS_ASSIST_TO_FLASH_RESET              = 98,           // (uint32 inv: 4294967197)
    SYS_CMD_SOFTWARE_RESET                              = 99,           // (uint32 inv: 4294967196)
    SYS_CMD_MANF_UNLOCK                                 = 1122334455,   // (uint32 inv: 3172632840)
    SYS_CMD_MANF_ERASE_CALIBRATION                      = 1357924679,   // (uint32 inv: 2937042616) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_FACTORY_RESET                          = 1357924680,   // (uint32 inv: 2937042615) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_CHIP_ERASE                             = 1357924681,   // (uint32 inv: 2937042614) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command.
    SYS_CMD_MANF_DOWNGRADE_CALIBRATION                  = 1357924682,   // (uint32 inv: 2937042613) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER                  = 1357924683,   // (uint32 inv: 2937042612) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command.

    SYS_CMD_FAULT_TEST_TRIG_MALLOC                      = 57005,
    SYS_CMD_FAULT_TEST_TRIG_HARD_FAULT                  = 57006,
    SYS_CMD_FAULT_TEST_TRIG_WATCHDOG                    = 57007,
};

enum eSerialPortBridge
{
    SERIAL_PORT_BRIDGE_DISABLED         = 0,

    SERIAL_PORT_BRIDGE_GPS1_TO_USB      = 1,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER0     = 2,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER1     = 3,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER2     = 4,

    SERIAL_PORT_BRIDGE_GPS2_TO_USB      = 5,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER0     = 6,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER1     = 7,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER2     = 8,

    SERIAL_PORT_BRIDGE_USB_TO_SER0      = 9,
    SERIAL_PORT_BRIDGE_USB_TO_SER1      = 10,
    SERIAL_PORT_BRIDGE_USB_TO_SER2      = 11,
    SERIAL_PORT_BRIDGE_SER0_TO_SER1     = 12,
    SERIAL_PORT_BRIDGE_SER0_TO_SER2     = 13,
    SERIAL_PORT_BRIDGE_SER1_TO_SER2     = 14,

    SERIAL_PORT_BRIDGE_USB_TO_USB       = 15,   // loopback
    SERIAL_PORT_BRIDGE_SER0_TO_SER0     = 16,   // loopback
    SERIAL_PORT_BRIDGE_SER1_TO_SER1     = 17,   // loopback
    SERIAL_PORT_BRIDGE_SER2_TO_SER2     = 18,   // loopback
};

#define NMEA_BUFFER_SIZE 256

typedef struct nmeaBroadcastMsgPair
{
    /** Message ID. (see eNmeaMsgId) */
    uint8_t msgID;

    /** Message period multiple. */
    uint8_t msgPeriod;
} nmeaBroadcastMsgPair_t;

#define MAX_nmeaBroadcastMsgPairs 20

/** (DID_NMEA_BCAST_PERIOD) Set NMEA message broadcast periods. This data structure is zeroed out on stop_all_broadcasts */
typedef struct PACKED
{
    /** Options: Port selection[0x0=current, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB, 0x100=preserve, 0x200=Persistent] (see RMC_OPTIONS_...) */
    uint32_t                options;

    /** NMEA message to be set.  Up to 20 message ID/period pairs.  Message ID of zero indicates the remaining pairs are not used. (see eNmeaMsgId) */
    nmeaBroadcastMsgPair_t  nmeaBroadcastMsgs[MAX_nmeaBroadcastMsgPairs];   

    /*  Example usage:
     *  If you are setting message GGA (6) at 1Hz and GGL (7) at 5Hz with the default DID_FLASH_CONFIG.startupGPSDtMs = 200 (5Hz)
     *  nmeaBroadcastMsgs[0].msgID = 6, nmeaBroadcastMsgs[0].msgPeriod = 5  
     *  nmeaBroadcastMsgs[1].msgID = 7, nmeaBroadcastMsgs[1].msgPeriod = 1 */           

} nmea_msgs_t;

typedef struct PACKED
{
    /** (rad/s) Gyros.  Units only apply for calibrated data. */
    float                   pqr[3];

    /** (m/s^2) Accelerometers.  Units only apply for calibrated data. */
    float                   acc[3];

    /** (°C) Temperature of IMU.  Units only apply for calibrated data. */
    float                   temp;
} sensors_imu_w_temp_t;

typedef struct PACKED
{                                       // Units only apply for calibrated data
    float                   mag[3];         // (uT)        Magnetometers
} sensors_mag_t;

typedef struct PACKED
{
    /** (rad/s) Gyros.  Units only apply for calibrated data. */
    float                   pqr[3];

    /** (m/s^2) Accelerometers.  Units only apply for calibrated data. */
    float                   acc[3];

    /** (uT) Magnetometers.  Units only apply for calibrated data. */
    float                   mag[3];
} sensors_mpu_t;

// (DID_SENSORS_TC_BIAS)
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
    double                  time;                                       // Units only apply for calibrated data

    sensors_mpu_t           mpu[NUM_IMU_DEVICES];
} sensors_t;

typedef struct PACKED
{
    float                   xyz[3];
} mag_xyz_t;

// (DID_SENSORS_UCAL, DID_SENSORS_TCAL, DID_SENSORS_MCAL)
typedef struct PACKED
{
    imu3_t                    imu3;

    /** (°C) Temperature of IMU.  Units only apply for calibrated data. */
    float                        temp[NUM_IMU_DEVICES];

    /** (uT) Magnetometers.  Units only apply for calibrated data. */
    mag_xyz_t                mag[NUM_MAG_DEVICES];
} sensors_w_temp_t;

typedef struct PACKED
{
    float                   lpfLsb[3];              // Low-pass filtered of g_sensors.lsb
    float                   lpfTemp;                // (°C) Low-pass filtered sensor temperature
    float                   k[3];                   // Slope (moved from flash to here)
    float                   temp;                   // (°C)    Temperature of sensor
    float                   tempRampRate;           // (°C/s) Temperature ramp rate
    uint32_t                tci;                    // Index of current temperature compensation point
    uint32_t                numTcPts;               // Total number of tc points
    float                   dtTemp;                 // (°C) Temperature from last calibration point
} sensor_comp_unit_t;

/** (DID_SCOMP) INTERNAL USE ONLY */
typedef struct PACKED
{                                                   // Sensor temperature compensation
    uint32_t                timeMs;                 // (ms) Time since boot up.
    sensor_comp_unit_t      pqr[NUM_IMU_DEVICES];
    sensor_comp_unit_t      acc[NUM_IMU_DEVICES];
    sensor_comp_unit_t      mag[NUM_MAG_DEVICES];
    imus_t                  referenceImu;            // External reference IMU
    float                   referenceMag[3];        // External reference magnetometer (heading reference)
    uint32_t                sampleCount;            // Number of samples collected
    uint32_t                calState;               // state machine (see eScompCalState)
    uint32_t                status;                 // Status used to control LED and indicate valid sensor samples (see eScompStatus)
    float                   alignAccel[3];          // Alignment acceleration
} sensor_compensation_t;

#define NUM_ANA_CHANNELS    4
typedef struct PACKED
{                                                   // LSB units for all except temperature, which is Celsius.
    double                  time;
    sensors_imu_w_temp_t    imu[NUM_IMU_DEVICES];
    sensors_mag_t           mag[NUM_MAG_DEVICES];   // Magnetometers
    float                   bar;                    // Barometric pressure
    float                   barTemp;                // Temperature of barometric pressure sensor
    float                   humidity;               // Relative humidity as a percent (%rH).  Range is 0% - 100%
    float                   ana[NUM_ANA_CHANNELS];  // ADC analog input
} sys_sensors_adc_t;

#if defined(IMX_5)
    #define NUM_COM_PORTS           4
#elif defined(IMX_6)
    #define NUM_COM_PORTS           4
#elif defined(GPX_1)
    #define NUM_COM_PORTS           6
    #define NUM_USR_PORTS           4
    #define NUM_GPS_PORTS           2
#else   // NPP and Unit Tests
    #define NUM_COM_PORTS           6
#endif

#define NUM_SERIAL_PORTS            6

#ifndef NUM_USR_PORTS
#define NUM_USR_PORTS           NUM_COM_PORTS
#endif

/** Realtime Message Controller (used in rmc_t). 
    The data sets available through RMC are broadcast at the availability of the data.  A goal of RMC is 
    to provide updates from each onboard sensor as fast as possible with minimal latency.  The RMC is 
    provided so that broadcast of sensor data is done as soon as it becomes available.   The exception to
    this rule is the INS output data, which has a configurable output data rate according to DID_RMC.insPeriodMs.
*/

#define RMC_OPTIONS_PORT_MASK           0x000000FF
#define RMC_OPTIONS_PORT_ALL            (RMC_OPTIONS_PORT_MASK)
#define RMC_OPTIONS_PORT_CURRENT        0x00000000
#define RMC_OPTIONS_PORT_SER0           0x00000001
#define RMC_OPTIONS_PORT_SER1           0x00000002    // also SPI
#define RMC_OPTIONS_PORT_SER2           0x00000004
#define RMC_OPTIONS_PORT_USB            0x00000008
#define RMC_OPTIONS_PRESERVE_CTRL       0x00000100    // Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define RMC_OPTIONS_PERSISTENT          0x00000200    // Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.  

                                                                // RMC message data rates:
#define RMC_BITS_INS1                   0x0000000000000001      // rmc.insPeriodMs (4ms default)
#define RMC_BITS_INS2                   0x0000000000000002      // "
#define RMC_BITS_INS3                   0x0000000000000004      // "
#define RMC_BITS_INS4                   0x0000000000000008      // "
#define RMC_BITS_IMU                    0x0000000000000010      // DID_FLASH_CONFIG.startupNavDtMs (4ms default)
#define RMC_BITS_PIMU                   0x0000000000000020      // "
#define RMC_BITS_BAROMETER              0x0000000000000040      // ~8ms
#define RMC_BITS_MAGNETOMETER           0x0000000000000080      // ~10ms
// #define RMC_BITS_UNUSED                 0x0000000000000100
// #define RMC_BITS_UNUSED                 0x0000000000000200 
#define RMC_BITS_GPS1_POS               0x0000000000000400      // DID_FLASH_CONFIG.startupGPSDtMs (200ms default)
#define RMC_BITS_GPS2_POS               0x0000000000000800      // "
#define RMC_BITS_GPS1_RAW               0x0000000000001000      // "
#define RMC_BITS_GPS2_RAW               0x0000000000002000      // "
#define RMC_BITS_GPS1_SAT               0x0000000000004000      // 1s
#define RMC_BITS_GPS2_SAT               0x0000000000008000      // "
#define RMC_BITS_GPS_BASE_RAW           0x0000000000010000      // 
#define RMC_BITS_STROBE_IN_TIME         0x0000000000020000      // On strobe input event
#define RMC_BITS_DIAGNOSTIC_MESSAGE     0x0000000000040000
#define RMC_BITS_IMU3_UNCAL             0x0000000000080000      // DID_FLASH_CONFIG.startupImuDtMs (1ms default)
#define RMC_BITS_GPS1_VEL               0x0000000000100000      // DID_FLASH_CONFIG.startupGPSDtMs (200ms default)
#define RMC_BITS_GPS2_VEL               0x0000000000200000      // "
#define RMC_BITS_GPS1_UBX_POS           0x0000000000400000      // "
#define RMC_BITS_GPS1_RTK_POS           0x0000000000800000      // "
#define RMC_BITS_GPS1_RTK_POS_REL       0x0000000001000000      // "
#define RMC_BITS_GPS1_RTK_POS_MISC      0x0000000004000000      // "
#define RMC_BITS_INL2_NED_SIGMA         0x0000000008000000
#define RMC_BITS_RTK_STATE              0x0000000010000000
#define RMC_BITS_RTK_CODE_RESIDUAL      0x0000000020000000
#define RMC_BITS_RTK_PHASE_RESIDUAL     0x0000000040000000
#define RMC_BITS_WHEEL_ENCODER          0x0000000080000000
#define RMC_BITS_GROUND_VEHICLE         0x0000000100000000
// #define RMC_BITS_UNUSED                 0x0000000200000000
#define RMC_BITS_IMU_MAG                0x0000000400000000
#define RMC_BITS_PIMU_MAG               0x0000000800000000
#define RMC_BITS_GPS1_RTK_HDG_REL       0x0000001000000000      // DID_FLASH_CONFIG.startupGPSDtMs (200ms default)
#define RMC_BITS_GPS1_RTK_HDG_MISC      0x0000002000000000      // "
#define RMC_BITS_REFERENCE_IMU          0x0000004000000000      // DID_FLASH_CONFIG.startupNavDtMs
#define RMC_BITS_REFERENCE_PIMU         0x0000008000000000      // "
#define RMC_BITS_IMU3_RAW               0x0000010000000000
#define RMC_BITS_IMU_RAW                0x0000020000000000
#define RMC_BITS_GPS1_SIG               0x0000040000000000      // 1s
#define RMC_BITS_GPS2_SIG               0x0000080000000000

// GPX messages could go into a local grmc if imx memory we expanded. (TM)
#define RMC_BITS_GPX_RTOS_INFO          0x0000100000000000
#define RMC_BITS_GPX_DEBUG              0x0000200000000000
#define RMC_BITS_GPX_STATUS             0x0000400000000000
#define RMC_BITS_GPX_DEV_INFO           0x0000800000000000
#define RMC_BITS_GPX_RMC                0x0001000000000000
// #define RMC_BITS_UNUSED                 0x0002000000000000
#define RMC_BITS_GPX_BIT                0x0004000000000000
#define RMC_BITS_GPX_PORT_MON           0x0008000000000000
#define RMC_BITS_GPX_RTK_DBG            0x0010000000000000

#define RMC_BITS_EVENT                  0x0800000000000000

#define RMC_BITS_MASK                   0x0FFFFFFFFFFFFFFF
#define RMC_BITS_INTERNAL_PPD           0x4000000000000000      // 
#define RMC_BITS_PRESET                 0x8000000000000000        // Indicate BITS is a preset.  This sets the rmc period multiple and enables broadcasting.

#define RMC_PRESET_PPD_NAV_PERIOD_MULT_MS   100         // uint8
#define RMC_PRESET_PPD_IMU3_PERIOD_MULT     255         // uint8

// Preset: Post Processing Data
#define RMC_PRESET_IMX_PPD_NO_IMU           (RMC_BITS_PRESET \
                                            | RMC_BITS_INS2 \
                                            | RMC_BITS_BAROMETER \
                                            | RMC_BITS_MAGNETOMETER \
                                            | RMC_BITS_GPS1_POS \
                                            | RMC_BITS_GPS2_POS \
                                            | RMC_BITS_GPS1_VEL \
                                            | RMC_BITS_GPS2_VEL \
                                            | RMC_BITS_GPS1_RAW \
                                            | RMC_BITS_GPS2_RAW \
                                            | RMC_BITS_GPS_BASE_RAW \
                                            | RMC_BITS_GPS1_RTK_POS_REL \
                                            | RMC_BITS_GPS1_RTK_HDG_REL \
                                            | RMC_BITS_INTERNAL_PPD \
                                            | RMC_BITS_DIAGNOSTIC_MESSAGE)
#define RMC_PRESET_IMX_PPD                  (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_PIMU \
                                            | RMC_BITS_REFERENCE_PIMU)
#define RMC_PRESET_IMX_PPD_IMU3_RAW         (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_IMU3_RAW \
                                            | RMC_BITS_PIMU)
#define RMC_PRESET_IMX_PPD_IMU3_UNCAL       (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_IMU3_UNCAL \
                                            | RMC_BITS_PIMU)
#define RMC_PRESET_INS                      (RMC_BITS_INS2 \
                                            | RMC_BITS_GPS1_POS \
                                            | RMC_BITS_PRESET)
#define RMC_PRESET_IMX_PPD_RTK_DBG          (RMC_PRESET_IMX_PPD \
                                            | RMC_BITS_RTK_STATE \
                                            | RMC_BITS_RTK_CODE_RESIDUAL \
                                            | RMC_BITS_RTK_PHASE_RESIDUAL \
                                            | RMC_BITS_GPX_DEBUG \
                                            | RMC_BITS_GPS1_SAT \
                                            | RMC_BITS_GPS2_SAT \
                                            | RMC_BITS_EVENT \
                                            | RMC_BITS_GPX_STATUS \
                                            | RMC_BITS_GPX_RTK_DBG)
#define RMC_PRESET_IMX_PPD_GROUND_VEHICLE   (RMC_PRESET_IMX_PPD \
                                            | RMC_BITS_WHEEL_ENCODER \
                                            | RMC_BITS_GROUND_VEHICLE)
#define RMC_PRESET_ALLAN_VARIANCE           (RMC_BITS_PRESET \
                                            | RMC_BITS_IMU)
#define RMC_PRESET_GPX_PPD                  (RMC_BITS_PRESET \
                                            | RMC_BITS_GPS1_POS \
                                            | RMC_BITS_GPS2_POS \
                                            | RMC_BITS_GPS1_VEL \
                                            | RMC_BITS_GPS2_VEL \
                                            | RMC_BITS_GPS1_RAW \
                                            | RMC_BITS_GPS2_RAW \
                                            | RMC_BITS_GPS_BASE_RAW \
                                            | RMC_BITS_GPS1_RTK_POS_REL \
                                            | RMC_BITS_GPS1_RTK_HDG_REL \
                                            | RMC_BITS_GPX_DEBUG \
                                            | RMC_BITS_GPX_PORT_MON \
                                            | RMC_BITS_EVENT \
                                            | RMC_BITS_GPX_STATUS)

/** (DID_RMC) Realtime message controller (RMC). */
typedef struct PACKED
{
    /** Data stream enable bits for the specified ports.  (see RMC_BITS_...) */
    uint64_t                bits;

    /** Options to select alternate ports to output data, etc.  (see RMC_OPTIONS_...) */
    uint32_t                options;
    
    /** IMU and Integrated IMU data transmit period is set using DID_SYS_PARAMS.navPeriodMs */
} rmc_t;

#define NMEA_GNGSV_FREQ_BAND1_BIT   (0x01)
#define NMEA_GNGSV_FREQ_BAND2_BIT   (0x01 << 1)
#define NMEA_GNGSV_FREQ_BAND3_BIT   (0x01 << 2)
#define NMEA_GNGSV_FREQ_5_BIT       (0x01 << 3)

#define NMEA_GNGSV_GPS_OFFSET       (SAT_SV_GNSS_ID_GPS << 4)
#define NMEA_GNGSV_GAL_OFFSET       (SAT_SV_GNSS_ID_GAL << 4)
#define NMEA_GNGSV_BEI_OFFSET       (SAT_SV_GNSS_ID_BEI << 4)
#define NMEA_GNGSV_QZS_OFFSET       (SAT_SV_GNSS_ID_QZS << 4)
#define NMEA_GNGSV_GLO_OFFSET       (SAT_SV_GNSS_ID_GLO << 4)

enum eNmeaMsgId
{
    NMEA_MSG_ID_INVALID     = 0,
    NMEA_MSG_ID_PIMU        = 1,
    NMEA_MSG_ID_PPIMU       = 2,
    NMEA_MSG_ID_PRIMU       = 3,
    NMEA_MSG_ID_PINS1       = 4,
    NMEA_MSG_ID_PINS2       = 5,
    NMEA_MSG_ID_PGPSP       = 6,
    NMEA_MSG_ID_GNGGA       = 7,
    NMEA_MSG_ID_GNGLL       = 8,
    NMEA_MSG_ID_GNGSA       = 9,
    NMEA_MSG_ID_GNRMC       = 10,
    NMEA_MSG_ID_GNZDA       = 11,
    NMEA_MSG_ID_PASHR       = 12,
    NMEA_MSG_ID_PSTRB       = 13,
    NMEA_MSG_ID_INFO        = 14,
    NMEA_MSG_ID_GNGSV       = 15,
    NMEA_MSG_ID_GNVTG       = 16,
    NMEA_MSG_ID_INTEL       = 17,
    NMEA_MSG_ID_POWGPS      = 18,
    NMEA_MSG_ID_POWTLV      = 19,
    NMEA_MSG_ID_COUNT,

    // IMX/GPX Input Commands
    NMEA_MSG_ID_ASCE,         // "ASCE" - NMEA messages broadcast enable
    NMEA_MSG_ID_BLEN,         // "BLEN" - Enable bootloader on IMX (app firmware update)    
    NMEA_MSG_ID_EBLE,         // "EBLE" - Enable bootloader on EVB
    NMEA_MSG_ID_NELB,         // "NELB" - Enable SAM-BA mode    
    NMEA_MSG_ID_PERS,         // "PERS" - Save perstent messages
    NMEA_MSG_ID_SRST,         // "SRTS" - Software reset
    NMEA_MSG_ID_STPB,         // "STPB" - Stop broadcasts on all ports
    NMEA_MSG_ID_STPC,         // "STPC" - Stop broadcasts on current port

    // Special case messages for each supported base message those with ID less than NMEA_MSG_ID_COUNT. 
    // Each base message get a 256 range of ID's for their special cases. Example for NMEA_MSG_ID_GNGSV:
    // NMEA_MSG_ID_GNGSV_START = NMEA_MSG_ID_GNGSV * NMEA_MSG_ID_SPECIAL_CASE_START giving a message ID 0x0f00 (3,840)
    // NOTE: Any ID greater than 256 is a special case, use the follow to extract the root case:
    //   if (msgId >= NMEA_MSG_ID_SPECIAL_CASE_START) msgId >>= 8;
    NMEA_MSG_ID_SPECIAL_CASE_START = 256,

    // Filtered GNGSV NMEA Message IDs:

    // GNGSV - All constellations
    NMEA_MSG_ID_GNGSV_START     = NMEA_MSG_ID_GNGSV * NMEA_MSG_ID_SPECIAL_CASE_START,                                                                                       // (3840) Used for reference only
    NMEA_MSG_ID_GNGSV_0         = NMEA_MSG_ID_GNGSV_START,                                                                                                                  // GNGSV_0 (3840) Clear all constellations and frequencies
    NMEA_MSG_ID_GNGSV_1         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND1_BIT),                                                                                    // GNGSV_1 (3841) Enable all constellations band1
    NMEA_MSG_ID_GNGSV_2         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND2_BIT),                                                                                    // GNGSV_2 (3842) Enable all constellations band2
    NMEA_MSG_ID_GNGSV_2_1       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                                                        // GNGSV_2_1 (3843) Enable all constellations band1, band2
    NMEA_MSG_ID_GNGSV_3         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT),                                                                                    // GNGSV_3 (3844) Enable all constellations band3
    NMEA_MSG_ID_GNGSV_3_1       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                                                        // GNGSV_3_1 (3845) Enable all constellations band1, band3
    NMEA_MSG_ID_GNGSV_3_2       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),                                                        // GNGSV_3_2 (3846) Enable all constellations band2, band3
    NMEA_MSG_ID_GNGSV_3_2_1     = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                            // GNGSV_3_2_1 (3847) Enable all constellations band1, band2, band3
    NMEA_MSG_ID_GNGSV_5         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT),                                                                                        // GNGSV_5 (3848) Enable all constellations band5
    NMEA_MSG_ID_GNGSV_5_1       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                                                            // GNGSV_5_1 (3849) Enable all constellations band1, band5
    NMEA_MSG_ID_GNGSV_5_2       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),                                                            // GNGSV_5_2 (3850) Enable all constellations band2, band5
    NMEA_MSG_ID_GNGSV_5_2_1     = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                                // GNGSV_5_2_1 (3851) Enable all constellations band1, band2, band5
    NMEA_MSG_ID_GNGSV_5_3       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT),                                                            // GNGSV_5_3 (3852) Enable all constellations band3, band5
    NMEA_MSG_ID_GNGSV_5_3_1     = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),                                // GNGSV_5_3_1 (3853) Enable all constellations band1, band3, band5
    NMEA_MSG_ID_GNGSV_5_3_2     = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),                                // GNGSV_5_3_2 (3854) Enable all constellations band2, band3, band5
    NMEA_MSG_ID_GNGSV_5_3_2_1   = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),    // GNGSV_5_3_2_1 (3855) Enable all constellations band1, band2, band3, band5                                                                                                              // GNGSV (3855) Enable all constellations and frequencies

    // GPGSV - GPS
    NMEA_MSG_ID_GPGSV_0         = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GPS_OFFSET),    // GPGSV_0 (3856) Disable all GPS frequencies
    NMEA_MSG_ID_GPGSV_1         = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GPS_OFFSET),        // GPGSV_1 (3857) Enable GPS L1
    NMEA_MSG_ID_GPGSV_2         = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_GPS_OFFSET),        // GPGSV_2 (3858) Enable GPS L2
    NMEA_MSG_ID_GPGSV_2_1       = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_GPS_OFFSET),      // GPGSV_2_1 (3859) Enable GPS L1, L2
    NMEA_MSG_ID_GPGSV_5         = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_GPS_OFFSET),        // GPGSV_5 (3864) Enable GPS L5
    NMEA_MSG_ID_GPGSV_5_1       = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_GPS_OFFSET),      // GPGSV_5_1 (3865) Enable GPS L1, L5
    NMEA_MSG_ID_GPGSV_5_2       = (NMEA_MSG_ID_GNGSV_5_2 + NMEA_GNGSV_GPS_OFFSET),      // GPGSV_5_2 (3866) Enable GPS L2, L5
    NMEA_MSG_ID_GPGSV_5_2_1     = (NMEA_MSG_ID_GNGSV_5_2_1 + NMEA_GNGSV_GPS_OFFSET),    // GPGSV_5_2_1 (3867) Enable GPS L1, L2, L5
    NMEA_MSG_ID_GPGSV           = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GPS_OFFSET),  // GPGSV (3871) Enable all GPS frequencies

    // GAGSV - Galileo
    NMEA_MSG_ID_GAGSV_0         = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GAL_OFFSET),    // GAGSV_0 (3888) Disable all Galileo frequencies
    NMEA_MSG_ID_GAGSV_1         = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GAL_OFFSET),        // GAGSV_1 (3889) Enable Galileo E1
    NMEA_MSG_ID_GAGSV_5         = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_GAL_OFFSET),        // GAGSV_5 (3896) Enable Galileo E5
    NMEA_MSG_ID_GAGSV_5_1       = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_GAL_OFFSET),      // GAGSV_5_1 (3897) Enable Galileo E1, E5
    NMEA_MSG_ID_GAGSV           = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GAL_OFFSET),  // GAGSV (3903) Enable all Galileo frequencies

    // GBGSV - Beido
    NMEA_MSG_ID_GBGSV_0         = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_BEI_OFFSET),    // GBGSV_0 (3904) Disable all Beidou frequencies
    NMEA_MSG_ID_GBGSV_1         = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_BEI_OFFSET),        // GBGSV_1 (3905) Enable Beidou B1
    NMEA_MSG_ID_GBGSV_2         = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_BEI_OFFSET),        // GBGSV_2 (3906) Enable Beidou B2
    NMEA_MSG_ID_GBGSV_2_1       = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_BEI_OFFSET),      // GBGSV_2_1 (3907) Enable Beidou B1, B2
    NMEA_MSG_ID_GBGSV_3         = (NMEA_MSG_ID_GNGSV_3 + NMEA_GNGSV_BEI_OFFSET),        // GBGSV_3 (3908) Enable Beidou B3 
    NMEA_MSG_ID_GBGSV_3_1       = (NMEA_MSG_ID_GNGSV_3_1 + NMEA_GNGSV_BEI_OFFSET),      // GBGSV_3_1 (3909) Enable Beidou B1, B3
    NMEA_MSG_ID_GBGSV_3_2       = (NMEA_MSG_ID_GNGSV_3_2 + NMEA_GNGSV_BEI_OFFSET),      // GBGSV_3_2 (3910) Enable Beidou B2, B3
    NMEA_MSG_ID_GBGSV_3_2_1     = (NMEA_MSG_ID_GNGSV_3_2_1 + NMEA_GNGSV_BEI_OFFSET),    // GBGSV_3_2_1 (3911) Enable Beidou B1, B2, B3
    NMEA_MSG_ID_GBGSV           = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_BEI_OFFSET),  // GBGSV (3919) Enable all Beidou frequencies

    // GQGSV - QZSS
    NMEA_MSG_ID_GQGSV_0         = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_QZS_OFFSET),    // GQGSV_0 (3920) Disable all QZSS frequencies
    NMEA_MSG_ID_GQGSV_1         = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_QZS_OFFSET),        // GQGSV_1 (3921) Enable QZSS L1
    NMEA_MSG_ID_GQGSV_2         = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_QZS_OFFSET),        // GQGSV_2 (3922) Enable QZSS L2
    NMEA_MSG_ID_GQGSV_2_1       = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_QZS_OFFSET),      // GQGSV_2_1 (3923) Enable QZSS L1, L2
    NMEA_MSG_ID_GQGSV_5         = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_QZS_OFFSET),        // GQGSV_5 (3928) Enable QZSS L5
    NMEA_MSG_ID_GQGSV_5_1       = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_QZS_OFFSET),      // GQGSV_5_1 (3929) Enable QZSS L1, L5
    NMEA_MSG_ID_GQGSV_5_2       = (NMEA_MSG_ID_GNGSV_5_2 + NMEA_GNGSV_QZS_OFFSET),      // GQGSV_5_2 (3930) Enable QZSS L2, L5
    NMEA_MSG_ID_GQGSV_5_2_1     = (NMEA_MSG_ID_GNGSV_5_2_1 + NMEA_GNGSV_QZS_OFFSET),    // GQGSV_5_2_1 (3931) Enable QZSS L1, L2, L5
    NMEA_MSG_ID_GQGSV           = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_QZS_OFFSET),  // GQGSV (3935) Enable all QZSS frequencies

    // GLGSV - Glonass
    NMEA_MSG_ID_GLGSV_0         = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GLO_OFFSET),    // GLGSV_0 (3936) Disable all Glonass frequencies
    NMEA_MSG_ID_GLGSV_1         = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GLO_OFFSET),        // GLGSV_1 (3937) Enable Glonass L1
    NMEA_MSG_ID_GLGSV_2         = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_GLO_OFFSET),        // GLGSV_2 (3938) Enable Glonass L2
    NMEA_MSG_ID_GLGSV_2_1       = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_GLO_OFFSET),      // GLGSV_2_1 (3939) Enable Glonass L1, L2
    NMEA_MSG_ID_GLGSV_3         = (NMEA_MSG_ID_GNGSV_3 + NMEA_GNGSV_GLO_OFFSET),        // GLGSV_3 (3940) Enable Glonass L3
    NMEA_MSG_ID_GLGSV_3_1       = (NMEA_MSG_ID_GNGSV_3_1 + NMEA_GNGSV_GLO_OFFSET),      // GLGSV_3_1 (3941) Enable Glonass L1, L3
    NMEA_MSG_ID_GLGSV_3_2       = (NMEA_MSG_ID_GNGSV_3_2 + NMEA_GNGSV_GLO_OFFSET),      // GLGSV_3_2 (3942) Enable Glonass L2, L3
    NMEA_MSG_ID_GLGSV_3_2_1     = (NMEA_MSG_ID_GNGSV_3_2_1 + NMEA_GNGSV_GLO_OFFSET),    // GLGSV_3_2_1 (3943) Enable Glonass L1, L2, L3
    NMEA_MSG_ID_GLGSV           = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GLO_OFFSET),  // GLGSV (3951) Enable all Glonass frequencies
    
    NMEA_MSG_ID_GNGSV_END       = NMEA_MSG_ID_GLGSV,                                    // (3951) Used for reference only
};

typedef struct {
    uint8_t constMask[SAT_SV_GNSS_ID_COUNT]; /* Constellation mask (see eGnGSVIndex)*/
} gsvMask_t;

#define NMEA_RMC_BITS_PIMU          (1<<NMEA_MSG_ID_PIMU)
#define NMEA_RMC_BITS_PPIMU         (1<<NMEA_MSG_ID_PPIMU)
#define NMEA_RMC_BITS_PRIMU         (1<<NMEA_MSG_ID_PRIMU)
#define NMEA_RMC_BITS_PINS1         (1<<NMEA_MSG_ID_PINS1)
#define NMEA_RMC_BITS_PINS2         (1<<NMEA_MSG_ID_PINS2)
#define NMEA_RMC_BITS_PGPSP         (1<<NMEA_MSG_ID_PGPSP)
#define NMEA_RMC_BITS_GNGGA         (1<<NMEA_MSG_ID_GNGGA)
#define NMEA_RMC_BITS_GNGLL         (1<<NMEA_MSG_ID_GNGLL)
#define NMEA_RMC_BITS_GNGSA         (1<<NMEA_MSG_ID_GNGSA)
#define NMEA_RMC_BITS_GNRMC         (1<<NMEA_MSG_ID_GNRMC)
#define NMEA_RMC_BITS_GNZDA         (1<<NMEA_MSG_ID_GNZDA)
#define NMEA_RMC_BITS_PASHR         (1<<NMEA_MSG_ID_PASHR)
#define NMEA_RMC_BITS_PSTRB         (1<<NMEA_MSG_ID_PSTRB)
#define NMEA_RMC_BITS_INFO          (1<<NMEA_MSG_ID_INFO)
#define NMEA_RMC_BITS_GNGSV         (1<<NMEA_MSG_ID_GNGSV)
#define NMEA_RMC_BITS_GNVTG         (1<<NMEA_MSG_ID_GNVTG)
#define NMEA_RMC_BITS_INTEL         (1<<NMEA_MSG_ID_INTEL)
#define NMEA_RMC_BITS_POWGPS        (1<<NMEA_MSG_ID_POWGPS)
#define NMEA_RMC_BITS_POWTLV        (1<<NMEA_MSG_ID_POWTLV)

typedef struct PACKED
{
     /** Data stream enable bits for the specified ports.  (see RMC_BITS_...) */
    uint32_t                nmeaBits;

    /** NMEA period multiple of above ISB period multiple indexed by NMEA_MSG_ID... */
    uint8_t                 nmeaPeriod[NMEA_MSG_ID_COUNT];

}rmcNmea_t;

/** Realtime message controller internal (RMCI). */
typedef struct PACKED
{
     /** Data stream enable bits and options for the specified ports.  (see RMC_BITS_...) */
    rmc_t                   rmc;
    
    /** Used for both the DID binary and NMEA messages.  */
    uint8_t                 periodMultiple[DID_COUNT];

    rmcNmea_t               rmcNmea;

} rmci_t;

// GPX Realtime Message Controller (GRMC) - message broadcast mechanism.
#define GRMC_OPTIONS_PORT_MASK           0x000000FF
#define GRMC_OPTIONS_PORT_ALL            (RMC_OPTIONS_PORT_MASK)
#define GRMC_OPTIONS_PORT_CURRENT        0x00000000
#define GRMC_OPTIONS_PORT_SER0           0x00000001
#define GRMC_OPTIONS_PORT_SER1           0x00000002    // also SPI
#define GRMC_OPTIONS_PORT_SER2           0x00000004
#define GRMC_OPTIONS_PORT_USB            0x00000008
#define GRMC_OPTIONS_PRESERVE_CTRL       0x00000100    // Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define GRMC_OPTIONS_PERSISTENT          0x00000200    // Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.  


enum GRMC_BIT_POS{
    GRMC_BIT_POS_DEV_INFO =             0,
    GRMC_BIT_POS_FLASH_CFG =            1,
    GRMC_BIT_POS_STATUS =               2,
    GRMC_BIT_POS_RTOS_INFO =            3,
    GRMC_BIT_POS_DEBUG_ARRAY =          4,
    GRMC_BIT_POS_GPS1_POS =             5,
    GRMC_BIT_POS_GPS1_VEL =             6,
    GRMC_BIT_POS_GPS1_SAT =             7,
    GRMC_BIT_POS_GPS1_SIG =             8,
    GRMC_BIT_POS_GPS1_RAW =             9,
    GRMC_BIT_POS_GPS1_VERSION =         10,
    GRMC_BIT_POS_GPS2_POS =             11,
    GRMC_BIT_POS_GPS2_VEL =             12,
    GRMC_BIT_POS_GPS2_SAT =             13,
    GRMC_BIT_POS_GPS2_SIG =             14,
    GRMC_BIT_POS_GPS2_RAW =             15,
    GRMC_BIT_POS_GPS2_VERSION =         16,
    GRMC_BIT_POS_GPS1_RTK_POS =         17,
    GRMC_BIT_POS_GPS1_RTK_POS_MISC =    18,
    GRMC_BIT_POS_GPS1_RTK_POS_REL =     19,
    GRMC_BIT_POS_GPS2_RTK_CMP_MISC =    20,
    GRMC_BIT_POS_GPS2_RTK_CMP_REL =     21,
    GRMC_BIT_POS_DID_RTK_DEBUG =        22,
    GRMC_BIT_POS_DID_PORT_MON =         23,
    GRMC_BIT_POS_DID_GPX_PORT_MON =     24,
    GRMC_BIT_POS_DID_GPS_BASE_RAW =     25,
    GRMC_BIT_POS_COUNT,
};

#define GRMC_BITS_DEV_INFO              (0x0000000000000001 << GRMC_BIT_POS_DEV_INFO)
#define GRMC_BITS_FLASH_CFG             (0x0000000000000001 << GRMC_BIT_POS_FLASH_CFG)
#define GRMC_BITS_STATUS                (0x0000000000000001 << GRMC_BIT_POS_STATUS)
#define GRMC_BITS_RTOS_INFO             (0x0000000000000001 << GRMC_BIT_POS_RTOS_INFO)
#define GRMC_BITS_DEBUG_ARRAY           (0x0000000000000001 << GRMC_BIT_POS_DEBUG_ARRAY)
#define GRMC_BITS_GPS1_POS              (0x0000000000000001 << GRMC_BIT_POS_GPS1_POS)
#define GRMC_BITS_GPS1_VEL              (0x0000000000000001 << GRMC_BIT_POS_GPS1_VEL)
#define GRMC_BITS_GPS1_SAT              (0x0000000000000001 << GRMC_BIT_POS_GPS1_SAT)
#define GRMC_BITS_GPS1_SIG              (0x0000000000000001 << GRMC_BIT_POS_GPS1_SIG)
#define GRMC_BITS_GPS1_RAW              (0x0000000000000001 << GRMC_BIT_POS_GPS1_RAW)
#define GRMC_BITS_GPS1_VERSION          (0x0000000000000001 << GRMC_BIT_POS_GPS1_VERSION)
#define GRMC_BITS_GPS2_POS              (0x0000000000000001 << GRMC_BIT_POS_GPS2_POS)
#define GRMC_BITS_GPS2_VEL              (0x0000000000000001 << GRMC_BIT_POS_GPS2_VEL)
#define GRMC_BITS_GPS2_SAT              (0x0000000000000001 << GRMC_BIT_POS_GPS2_SAT)
#define GRMC_BITS_GPS2_SIG              (0x0000000000000001 << GRMC_BIT_POS_GPS2_SIG)
#define GRMC_BITS_GPS2_RAW              (0x0000000000000001 << GRMC_BIT_POS_GPS2_RAW)
#define GRMC_BITS_GPS2_VERSION          (0x0000000000000001 << GRMC_BIT_POS_GPS2_VERSION)
#define GRMC_BITS_GPS1_RTK_POS          (0x0000000000000001 << GRMC_BIT_POS_GPS1_RTK_POS)
#define GRMC_BITS_GPS1_RTK_POS_MISC     (0x0000000000000001 << GRMC_BIT_POS_GPS1_RTK_POS_MISC)
#define GRMC_BITS_GPS1_RTK_POS_REL      (0x0000000000000001 << GRMC_BIT_POS_GPS1_RTK_POS_REL)
#define GRMC_BITS_GPS2_RTK_CMP_MISC     (0x0000000000000001 << GRMC_BIT_POS_GPS2_RTK_CMP_MISC)
#define GRMC_BITS_GPS2_RTK_CMP_REL      (0x0000000000000001 << GRMC_BIT_POS_GPS2_RTK_CMP_REL)
#define GRMC_BITS_DID_RTK_DEBUG         (0x0000000000000001 << GRMC_BIT_POS_DID_RTK_DEBUG)
#define GRMC_BITS_PORT_MON              (0x0000000000000001 << GRMC_BIT_POS_DID_PORT_MON)
#define GRMC_BITS_GPX_PORT_MON          (0x0000000000000001 << GRMC_BIT_POS_DID_GPX_PORT_MON)
#define GRMC_BITS_GPS_BASE_RAW          (0x0000000000000001 << GRMC_BIT_POS_DID_GPS_BASE_RAW)
#define GRMC_BITS_PRESET                (0x8000000000000000)    // Indicate BITS is a preset.  This sets the rmc period multiple and enables broadcasting.

#define GRMC_PRESET_DID_RTK_DEBUG_PERIOD_MS     1000
#define GRMC_PRESET_GPX_DEV_INFO_PERIOD_MS      1000
#define GRMC_PRESET_GPX_GPS1_VERSION_PERIOD_MS  1000
#define GRMC_PRESET_GPX_GPS2_VERSION_PERIOD_MS  1000
#define GRMC_PRESET_GPX_RTOS_INFO_PERIOD_MS     500
#define GRMC_PRESET_GPX_STATUS_PERIOD_MS        500
#define GRMC_PRESET_GPX_DEBUG_ARRAY_PERIOD_MS   500
#define GRMC_PRESET_GPX_PORT_MON_PERIOD_MS      500

#define GRMC_PRESET_GPX_BASE            (GRMC_BITS_PRESET \
                                        /*| GRMC_BITS_DEV_INFO*/ \
                                        /*| GRMC_BITS_RTOS_INFO*/ \
                                        | GRMC_BITS_STATUS \
                                        /*| GRMC_BITS_DEBUG_ARRAY*/)

#define GRMC_PRESET_GPX_GPS1            (GRMC_BITS_GPS1_POS \
                                        | GRMC_BITS_GPS1_VEL \
                                        | GRMC_BITS_GPS1_SAT \
                                        | GRMC_BITS_GPS1_SIG \
                                        | GRMC_BITS_GPS1_VERSION \
                                        /*| GRMC_BITS_GPS1_RTK_POS*/ \
                                        | GRMC_BITS_GPS1_RAW)

#define GRMC_PRESET_GPX_GPS2            (GRMC_BITS_GPS2_POS \
                                        | GRMC_BITS_GPS2_VEL \
                                        | GRMC_BITS_GPS2_SAT \
                                        | GRMC_BITS_GPS2_SIG \
                                        | GRMC_BITS_GPS2_VERSION \
                                        | GRMC_BITS_GPS2_RAW)

#define GRMC_PRESET_GPX_IMX             ( GRMC_PRESET_GPX_BASE\
                                        | GRMC_PRESET_GPX_GPS1 \
                                        | GRMC_PRESET_GPX_GPS2 \
                                        | GRMC_BITS_GPS2_RTK_CMP_REL \
                                        | GRMC_BITS_GPS2_RTK_CMP_MISC)

#define GRMC_PRESET_GPX_IMX_RTK_DBG     (GRMC_PRESET_GPX_IMX | GRMC_BITS_DID_RTK_DEBUG)


typedef struct PACKED 
{
    rmc_t rmc;

    /** Period of the message to be sent in ms*/
    uint16_t periodMultiple[GRMC_BIT_POS_COUNT];

    /** NMEA data stream enable bits for the specified ports.  (see NMEA_RMC_BITS_...) */
    rmcNmea_t rmcNmea;
} grmci_t;

enum eMagCalState
{
    MAG_CAL_STATE_DO_NOTHING    = (int)0, 

    /** COMMAND: Recalibrate magnetometers using multiple axis */
    MAG_CAL_STATE_MULTI_AXIS    = (int)1,

    /** COMMAND: Recalibrate magnetometers using only one axis */
    MAG_CAL_STATE_SINGLE_AXIS   = (int)2,

    /** COMMAND: Stop mag recalibration and do not save results */
    MAG_CAL_STATE_ABORT         = (int)101,

    /** STATUS: Mag recalibration is in progress */
    MAG_CAL_STATE_RECAL_RUNNING = (int)200,

    /** STATUS: Mag recalibration has completed */
    MAG_CAL_STATE_RECAL_COMPLETE = (int)201,

    /** STATUS: Mag recalibration mode not supported */
    MAG_CAL_STATE_RECAL_MODE_NOT_SUPPORTED = (int)202,
};

/** (DID_MAG_CAL) Magnetometer Calibration */
typedef struct PACKED
{
    /** Mag recalibration state.  COMMANDS: 1=multi-axis, 2=single-axis, 101=abort, STATUS: 200=running, 201=done (see eMagCalState) */
    uint32_t                state;
    
    /** Mag recalibration progress indicator: 0-100 % */
    float                    progress;

    /** Magnetic declination estimate */
    float                    declination;
} mag_cal_t;

// (DID_INL2_MAG_OBS_INFO)
typedef struct PACKED
{                                            // INL2 - Magnetometer observer info 
    /** Timestamp in milliseconds */
    uint32_t                timeOfWeekMs;    

    /** Number of calibration samples */
    uint32_t                Ncal_samples;

    /** Data ready to be processed */
    uint32_t                ready;

    /** Calibration data present.  Set to -1 to force mag recalibration. */    
    uint32_t                calibrated;

    /** Allow mag to auto-recalibrate */
    uint32_t                auto_recal;

    /** Bad sample data */        
    uint32_t                outlier;

    /** Heading from magnetometer */
    float                    magHdg;

    /** Heading from INS */            
    float                    insHdg;

    /** Difference between mag heading and (INS heading plus mag declination) */
    float                    magInsHdgDelta;

    /** Normalized innovation squared (likelihood metric) */
    float                    nis;

    /** Threshold for maximum NIS */
    float                    nis_threshold;

    /** Magnetometer calibration matrix. Must be initialized with a unit matrix, not zeros! */
    float                    Wcal[9];

    /** Active calibration set (0 or 1) */
    uint32_t                activeCalSet;

    /** Offset between magnetometer heading and estimate heading */
    float                    magHdgOffset;

    /** Scaled computed variance between calibrated magnetometer samples.  */
    float                   Tcal;

    /** Calibrated magnetometer output can be produced using: Bcal = Wcal * (Braw - bias_cal) */
    float                   bias_cal[3];
} inl2_mag_obs_info_t;

/** Built-in Test: Input Command */
enum eBitCommand
{
    BIT_CMD_NONE                                    = (int)0,       // No command
    BIT_CMD_OFF                                     = (int)1,       // Stop built-in test
    BIT_CMD_FULL_STATIONARY                         = (int)2,       // (FULL) Comprehensive test.  Requires system be completely stationary without vibrations. 
    BIT_CMD_BASIC_MOVING                            = (int)3,       // (BASIC) Ignores sensor output.  Can be run while moving.  This mode is automatically run after bootup.
    BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY           = (int)4,       // Same as BIT_CMD_FULL_STATIONARY but with higher requirements for accuracy.  In order to pass, this test may require the Infield Calibration (DID_INFIELD_CAL) to be run. 
    BIT_CMD_RESERVED_2                              = (int)5,   
    BIT_CMD_IMU_REJECT                              = (int)6,       // IMU fault rejection test 
    BIT_CMD_IMU_REJECT_CONTINUOUS                   = (int)7,       // Continuous IMU fault rejection test without ending
};

/** Built-in Test: State */
enum eBitState
{
    BIT_STATE_OFF                                   = (int)0,
    BIT_STATE_DONE                                  = (int)1,       // Test is finished
    BIT_STATE_RUNNING                               = (int)6,
    BIT_STATE_FINISHING                             = (int)7,       // Computing results
};

/** Built-in Test: Test Mode */
enum eBitTestMode
{
    BIT_TEST_MODE_FAILED                            = (int)98,      // Test mode ran and failed
    BIT_TEST_MODE_DONE                              = (int)99,      // Test mode ran and completed
    BIT_TEST_MODE_SIM_GPS_NOISE                     = (int)100,     // Simulate CNO noise
    BIT_TEST_MODE_COMMUNICATIONS_REPEAT             = (int)101,     // Send duplicate message 
    BIT_TEST_MODE_SERIAL_DRIVER_RX_OVERFLOW         = (int)102,     // Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    BIT_TEST_MODE_SERIAL_DRIVER_TX_OVERFLOW         = (int)103,     // Cause Tx buffer overflow on current serial port by sending too much data.
    BIT_TEST_MODE_IMU_FAULT_REJECTION               = (int)104,     // Simulate a fault on each IMU sensor and ensure it is detected and rejected.
};

/** Hardware built-in test (BIT) flags */
enum eHdwBitStatusFlags
{
    HDW_BIT_PASSED_MASK                     = (int)0x0000000F,
    HDW_BIT_PASSED_ALL                      = (int)0x00000001,
    HDW_BIT_PASSED_NO_GPS                   = (int)0x00000002,    // Passed w/o valid GPS signal
    HDW_BIT_MODE_MASK                       = (int)0x000000F0,    // BIT mode run
    HDW_BIT_MODE_OFFSET                     = (int)4,
#define HDW_BIT_MODE(hdwBitStatus)          (((hdwBitStatus)&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
    HDW_BIT_FAILED_MASK                     = (int)0xFFFFFF00,
    HDW_BIT_FAILED_AHRS_MASK                = (int)0xFFFF0F00,
    HDW_BIT_FAULT_NOISE_PQR                 = (int)0x00000100,
    HDW_BIT_FAULT_NOISE_ACC                 = (int)0x00000200,
    HDW_BIT_FAULT_MAGNETOMETER              = (int)0x00000400,
    HDW_BIT_FAULT_BAROMETER                 = (int)0x00000800,
    HDW_BIT_FAULT_GPS_NO_COM                = (int)0x00001000,    // No GPS serial communications
    HDW_BIT_FAULT_GPS_POOR_CNO              = (int)0x00002000,    // Poor GPS signal strength.  Check antenna
    HDW_BIT_FAULT_GPS_POOR_ACCURACY         = (int)0x00004000,    // Low number of satellites, or bad accuracy 
    HDW_BIT_FAULT_GPS_NOISE                 = (int)0x00008000,    // (Not implemented)
    HDW_BIT_FAULT_IMU_FAULT_REJECTION       = (int)0x00010000,    // IMU fault rejection failure
    HDW_BIT_FAULT_INCORRECT_HARDWARE_TYPE   = (int)0x01000000,    // Hardware type does not match firmware
};

/** Calibration built-in test flags */
enum eCalBitStatusFlags
{
    CAL_BIT_PASSED_MASK                     = (int)0x0000000F,
    CAL_BIT_PASSED_ALL                      = (int)0x00000001,
    CAL_BIT_MODE_MASK                       = (int)0x000000F0,    // BIT mode run
    CAL_BIT_MODE_OFFSET                     = (int)4,
#define CAL_BIT_MODE(calBitStatus)          (((calBitStatus)&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
    CAL_BIT_FAILED_MASK                     = (int)0x00FFFF00,
    CAL_BIT_FAULT_TCAL_EMPTY                = (int)0x00000100,    // Temperature calibration not present
    CAL_BIT_FAULT_TCAL_TSPAN                = (int)0x00000200,    // Temperature calibration temperature range is inadequate
    CAL_BIT_FAULT_TCAL_INCONSISTENT         = (int)0x00000400,    // Temperature calibration number of points or slopes are not consistent
    CAL_BIT_FAULT_TCAL_CORRUPT              = (int)0x00000800,    // Temperature calibration memory corruption
    CAL_BIT_FAULT_TCAL_PQR_BIAS             = (int)0x00001000,    // Temperature calibration gyro bias
    CAL_BIT_FAULT_TCAL_PQR_SLOPE            = (int)0x00002000,    // Temperature calibration gyro slope
    CAL_BIT_FAULT_TCAL_PQR_LIN              = (int)0x00004000,    // Temperature calibration gyro linearity
    CAL_BIT_FAULT_TCAL_ACC_BIAS             = (int)0x00008000,    // Temperature calibration accelerometer bias
    CAL_BIT_FAULT_TCAL_ACC_SLOPE            = (int)0x00010000,    // Temperature calibration accelerometer slope
    CAL_BIT_FAULT_TCAL_ACC_LIN              = (int)0x00020000,    // Temperature calibration accelerometer linearity
    CAL_BIT_FAULT_CAL_SERIAL_NUM            = (int)0x00040000,    // Calibration info: wrong device serial number
    CAL_BIT_FAULT_MCAL_MAG_INVALID          = (int)0x00080000,    // Motion calibration MAG Cross-axis alignment is poorly formed
    CAL_BIT_FAULT_MCAL_EMPTY                = (int)0x00100000,    // Motion calibration Cross-axis alignment is not calibrated
    CAL_BIT_FAULT_MCAL_IMU_INVALID          = (int)0x00200000,    // Motion calibration IMU Cross-axis alignment is poorly formed
    CAL_BIT_FAULT_MOTION_PQR                = (int)0x00400000,    // Motion on gyros
    CAL_BIT_FAULT_MOTION_ACC                = (int)0x00800000,    // Motion on accelerometers
    CAL_BIT_NOTICE_IMU1_PQR_BIAS            = (int)0x01000000,    // IMU 1 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU2_PQR_BIAS            = (int)0x02000000,    // IMU 2 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU1_ACC_BIAS            = (int)0x10000000,    // IMU 1 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
    CAL_BIT_NOTICE_IMU2_ACC_BIAS            = (int)0x20000000,    // IMU 2 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
};


/** (DID_BIT) Built-in self-test (BIT) parameters */
typedef struct PACKED
{
    /** BIT input command (see eBitCommand).  Ignored when zero.  */
    uint8_t                 command;

    /** BIT last input command (see eBitCommand) */
    uint8_t                 lastCommand;

    /** BIT current state (see eBitState) */
    uint8_t                 state;

    /** Unused */
    uint8_t                 reserved;

    /** Hardware BIT status (see eHdwBitStatusFlags) */
    uint32_t                hdwBitStatus;

    /** Calibration BIT status (see eCalBitStatusFlags) */
    uint32_t                calBitStatus;

    /** Temperature calibration bias */
    float                   tcPqrBias;
    float                   tcAccBias;

    /** Temperature calibration slope */
    float                   tcPqrSlope;
    float                   tcAccSlope;

    /** Temperature calibration linearity */
    float                   tcPqrLinearity;
    float                   tcAccLinearity;

    /** Gyro error (rad/s) */
    float                   pqr;

    /** Accelerometer error (m/s^2) */
    float                   acc;

    /** Angular rate standard deviation */
    float                   pqrSigma;

    /** Acceleration standard deviation */
    float                   accSigma;

    /** Self-test mode (see eBitTestMode) */
    uint8_t                 testMode;

    /** Self-test mode bi-directional variable used with testMode */
    uint8_t                 testVar;

    /** The hardware type detected (see "Product Hardware ID").  This is used to ensure correct firmware is used. */
    uint16_t                detectedHardwareId;

} bit_t;

// GPX Built-in Test (GPX-BIT)
enum eGPXBit_resultsPos{
    GPXBit_resultsPos_PPS1 = 0,
    GPXBit_resultsPos_PPS2,
    GPXBit_resultsPos_UART,
    GPXBit_resultsPos_IO,
    GPXBit_resultsPos_GPS,

    GPXBit_resultsPos_FINISHED,

    GPXBit_resultsPos_CANCELED,
    GPXBit_resultsPos_ERROR,
};

enum eGPXBit_results{
    GPXBit_resultsBit_PPS1              = (0x01 << GPXBit_resultsPos_PPS1),
    GPXBit_resultsBit_PPS2              = (0x01 << GPXBit_resultsPos_PPS2),
    GPXBit_resultsBit_UART              = (0x01 << GPXBit_resultsPos_UART),
    GPXBit_resultsBit_IO                = (0x01 << GPXBit_resultsPos_IO),
    GPXBit_resultsBit_GPS               = (0x01 << GPXBit_resultsPos_GPS),
    GPXBit_resultsBit_FINISHED          = (0x01 << GPXBit_resultsPos_FINISHED),
    GPXBit_resultsBit_CANCELED          = (0x01 << GPXBit_resultsPos_CANCELED),
    GPXBit_resultsBit_ERROR             = (0x01 << GPXBit_resultsPos_ERROR),
};

#define GPXBit_RESULT_GPS_QT_EXIT_Mask      GPXBit_resultsBit_PPS1 | GPXBit_resultsBit_PPS2

enum eGPXBit_CMD{
    GPXBit_CMD_NONE                                     = 0,
    GPXBit_CMD_START_MANUF_TEST                         = 1,
    GPXBit_CMD_ALERT_UART_TEST_STR                      = 2,
    GPXBit_CMD_ALERT_PPS1_RX                            = 3,
    GPXBit_CMD_ALERT_PPS2_RX                            = 4,
    GPXBit_CMD_REPORT                                   = 5,
    GPXBit_CMD_STOP                                     = 6,

    GPXBit_CMD_START_SIM_GPS_NOISE                      = 7,
    GPXBit_CMD_START_COMMUNICATIONS_REPEAT              = 8,        // Send duplicate message
    GPXBit_CMD_START_SERIAL_DRIVER_TX_OVERFLOW          = 9,        // Cause Tx buffer overflow on current serial port by sending too much data.
    GPXBit_CMD_START_SERIAL_DRIVER_RX_OVERFLOW          = 10,       // Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    GPXBit_CMD_FORCE_SYS_FAULT_WATCHDOG_COMM_TASK       = 11,       // Cause watchdog reset by stalling COMM task
    GPXBit_CMD_FORCE_SYS_FAULT_WATCHDOG_RTK_TASK        = 12,       // Cause watchdog reset by stalling RTK task
    GPXBit_CMD_FORCE_SYS_FAULT_HARD_FAULT               = 13,       // Cause hard fault
    GPXBit_CMD_FORCE_SYS_FAULT_MALLOC                   = 14,       // Cause malloc failure
};

enum eGPXBit_test_mode{
    GPXBit_test_mode_NONE                               = (int)0,
    GPXBit_test_mode_FAILURE                            = (int)8,
    GPXBit_test_mode_DONE                               = (int)9,
    GPXBit_test_mode_MANUFACTURING                      = (int)10,      // Standard manufacturing

    GPXBit_test_mode_SIM_GPS_NOISE                      = (int)100,     // Simulate CNO noise
    GPXBit_test_mode_COMMUNICATIONS_REPEAT              = (int)101,     // Send duplicate message
    GPXBit_test_mode_SERIAL_DRIVER_TX_OVERFLOW          = (int)102,     // Cause Tx buffer overflow on current serial port by sending too much data.
    GPXBit_test_mode_SERIAL_DRIVER_RX_OVERFLOW          = (int)103,     // Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    GPXBit_test_mode_SYS_FAULT_WATCHDOG_COMM_TASK       = (int)104,     // Cause watchdog reset by stalling COMM task
    GPXBit_test_mode_SYS_FAULT_WATCHDOG_RTK_TASK        = (int)105,     // Cause watchdog reset by stalling RTK task
};

#define GPXBit_resultMasks_PASSED  (GPXBit_resultsBit_PPS1 | GPXBit_resultsBit_PPS2 | GPXBit_resultsBit_UART | GPXBit_resultsBit_IO | GPXBit_resultsBit_GPS | GPXBit_resultsBit_FINISHED)

/** (DID_GPX_BIT) Built-in self-test parameters */
typedef struct PACKED
{
    /** GPX built-in test status (see eGPXBit_results) */
    uint32_t                results;
    
    /** Command (see eGPXBit_CMD) */
    uint8_t                 command;

    /** Port used with the test */
    uint8_t                 port;

    /** Self-test mode (see eGPXBit_test_mode) */
    uint8_t                 testMode;

    /** Built-in self-test state (see eGPXBit_state) */
    uint8_t                 state;

    /** The hardware ID detected (see "Product Hardware ID").  This is used to ensure correct firmware is used. */
    uint16_t                detectedHardwareId;

    /** Unused */
    uint8_t                 reserved[2];

} gpx_bit_t;

enum eInfieldCalState
{
    /** User Commands: */
    INFIELD_CAL_STATE_CMD_OFF                           = 0,

    /** Initialization Commands.  Select one of the following to clear prior samples and set the mode.  Zero accels requires vertical alignment.  No motion is required for all unless disabled.  */
    INFIELD_CAL_STATE_CMD_INIT_ZERO_IMU                         = 1,            // Zero accel and gyro biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_GYRO                        = 2,            // Zero only gyro  biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ACCEL                       = 3,            // Zero only accel biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE                    = 4,            // Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_IMU                = 5,            // Zero gyro and accel biases.  Zero (level) INS attitude by adjusting INS rotation. 
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_GYRO               = 6,            // Zero only gyro  biases.  Zero (level) INS attitude by adjusting INS rotation. 
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_ACCEL              = 7,            // Zero only accel biases.  Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_MOTION_DETECT     = 0x00010000,   // Bitwise AND this with the above init commands to disable motion detection during sampling (allow for more tolerant sampling).
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_REQUIRE_VERTIAL   = 0x00020000,   // Bitwise AND this with the above init commands to disable vertical alignment requirement for accelerometer bias calibration (allow for more tolerant sampling).

    /** Sample and End Commands: */
    INFIELD_CAL_STATE_CMD_START_SAMPLE                          = 8,    // Initiate 5 second sensor sampling and averaging.  Run for each orientation and 180 degree yaw rotation.
    INFIELD_CAL_STATE_CMD_SAVE_AND_FINISH                       = 9,    // Run this command to compute and save results.  Must be run following INFIELD_CAL_STATE_CMD_START_SAMPLE.
    
    /** Status: (read only) */
    INFIELD_CAL_STATE_READY_FOR_SAMPLING                        = 50,   // System has been initialized and is waiting for user to intiate sampling.  User must send a command to exit this state.
    INFIELD_CAL_STATE_SAMPLING                                  = 51,   // System is averaging the IMU data.  Minimize all motion and vibration.
    INFIELD_CAL_STATE_RUN_BIT_AND_FINISH                        = 52,   // Follow up calibration zero with BIT and copy out IMU biases.
    INFIELD_CAL_STATE_SAVED_AND_FINISHED                        = 53,   // Calculations are complete and DID_INFIELD_CAL.imu holds the update IMU biases.  Updates are saved to flash. 

    /** Error Status: (read only) */
    INFIELD_CAL_STATE_ERROR_NOT_INITIALIZED                     = 100,  // Init command (INFIELD_CAL_STATE_CMD_INIT_...) not set. 
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_MOTION_DETECTED        = 101,  // Error: Motion detected. Sampling aborted. 
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_NOT_VERTICAL           = 102,  // Error: System not vertical. Sampling aborted. 
    INFIELD_CAL_STATE_ERROR_NO_SAMPLES_COLLECTED                = 103,  // Error: No samples have been collected
    INFIELD_CAL_STATE_ERROR_POOR_CAL_FIT                        = 104,  // Error: Calibration zero is not 

    /** Internal Use Only */
    INFIELD_CAL_STATE_CMD_MASK                                  = 0x0000FFFF,
    INFIELD_CAL_STATE_CMD_START_SAMPLE_BIT                      = 11,    // Initiate 5 second sensor sample and averaging.  Does not save sample into cal data.
};

enum eInfieldCalStatus
{
    INFIELD_CAL_STATUS_AXIS_DN_GRAVITY                          = 0x00000001,   // Axis points in direction of gravity more than any other axis.
    INFIELD_CAL_STATUS_AXIS_DN_SAMPLED                          = 0x00000002,   // Sampled
    INFIELD_CAL_STATUS_AXIS_DN_SAMPLED_180                      = 0x00000004,   // Sampled based on average of two orientations with 180 degree delta yaw. 
    INFIELD_CAL_STATUS_AXIS_UP_GRAVITY                          = 0x00000008,   // Axis points in direction of gravity more than any other axis.
    INFIELD_CAL_STATUS_AXIS_UP_SAMPLED                          = 0x00000010,   // Sampled
    INFIELD_CAL_STATUS_AXIS_UP_SAMPLED_180                      = 0x00000020,   // Sampled based on average of two orientations with 180 degree delta yaw.

    INFIELD_CAL_STATUS_SAMPLE_X_OFFSET                          = 0,
    INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET                          = 6,
    INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET                          = 12,
    
    INFIELD_CAL_STATUS_AXIS_MASK                                = 0x0000003F,
    INFIELD_CAL_STATUS_AXES_GRAVITY_MASK                        = (\
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_X_OFFSET) | \
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET) | \
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET)),

    INFIELD_CAL_STATUS_ENABLED_ZERO_ACCEL                       = 0x00100000,    // Zero accel bias.  Require vertical alignment for sampling. 
    INFIELD_CAL_STATUS_ENABLED_ZERO_GYRO                        = 0x00200000,    // Zero gyro bias.
    INFIELD_CAL_STATUS_ENABLED_ZERO_ATTITUDE                    = 0x00400000,    // Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATUS_ENABLED_MOTION_DETECT                    = 0x00800000,    // Require no motion during sampling. 
    INFIELD_CAL_STATUS_ENABLED_NORMAL_MASK                      = 0x00F00000,
    INFIELD_CAL_STATUS_ENABLED_BIT                              = 0x01000000,    // Used for BIT 
    INFIELD_CAL_STATUS_DISABLED_REQUIRE_VERTICAL                = 0x02000000,    // Do not require vertical alignment for accelerometer calibration. 

    INFIELD_CAL_STATUS_AXIS_NOT_VERTICAL                        = 0x10000000,    // Axis is not aligned vertically and cannot be used for zero accel sampling.  
    INFIELD_CAL_STATUS_MOTION_DETECTED                          = 0x20000000,    // System is not stationary and cannot be used for infield calibration.
};

/** Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
    /** Vertical axis acceleration (m/s^2) */
    float                   acc[3];
} imus_acc_t;

typedef struct PACKED
{
    imus_acc_t              dev[NUM_IMU_DEVICES];

    float                    yaw;        // (rad) Heading of IMU sample.  Used to determine how to average additional samples.  0 = invalid, 999 = averaged
} infield_cal_direction_t;

typedef struct PACKED
{
    infield_cal_direction_t down;       // Pointed toward earth
    infield_cal_direction_t up;         // Pointed toward sky
} infield_cal_vaxis_t;

// (DID_INFIELD_CAL)
typedef struct PACKED
{
    /** Used to set and monitor the state of the infield calibration system. (see eInfieldCalState) */
    uint32_t                state;

    /** Infield calibration status. (see eInfieldCalStatus) */
    uint32_t                status;

    /** Number of samples used in IMU average. sampleTimeMs = 0 means "imu" member contains the IMU bias from flash.  */
    uint32_t                sampleTimeMs;

    /** Dual purpose variable.  1.) This is the averaged IMU sample when sampleTimeMs != 0.  2.) This is a mirror of the motion calibration IMU bias from flash when sampleTimeMs = 0. */ 
    imus_t                  imu[NUM_IMU_DEVICES];

    /** Collected data used to solve for the bias error and INS rotation.  Vertical axis: 0 = X, 1 = Y, 2 = Z  */
    infield_cal_vaxis_t     calData[3];

} infield_cal_t;


/** System Configuration (used with DID_FLASH_CONFIG.sysCfgBits) */
enum eSysConfigBits
{
    UNUSED1                                             = (int)0x00000001,
    /*! Enable mag continuous calibration.  Allow slow background magnetometer calibration in the EKF. */
    SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL              = (int)0x00000002,
    /*! Enable automatic mag recalibration */
    SYS_CFG_BITS_AUTO_MAG_RECAL                         = (int)0x00000004,
    /*! Disable mag declination estimation */
    SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION            = (int)0x00000008,

    /*! Disable LEDs */
    SYS_CFG_BITS_DISABLE_LEDS                           = (int)0x00000010,

    /** Magnetometer recalibration.  (see eMagCalState) 1 = multi-axis, 2 = single-axis */
    SYS_CFG_BITS_MAG_RECAL_MODE_MASK                    = (int)0x00000700,
    SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET                  = 8,
#define SYS_CFG_BITS_MAG_RECAL_MODE(sysCfgBits) ((sysCfgBits&SYS_CFG_BITS_MAG_RECAL_MODE_MASK)>>SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET)

    // When set WMM will be used to set declanation
    SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION             = (int)0x00000800,

    /** Disable magnetometer fusion */
    SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION            = (int)0x00001000,
    /** Disable barometer fusion */
    SYS_CFG_BITS_DISABLE_BAROMETER_FUSION               = (int)0x00002000,
    /** Disable GPS 1 fusion */
    SYS_CFG_BITS_DISABLE_GPS1_FUSION                    = (int)0x00004000,
    /** Disable GPS 2 fusion */
    SYS_CFG_BITS_DISABLE_GPS2_FUSION                    = (int)0x00008000,

    /** Disable automatic Zero Velocity Updates (ZUPT).  Disabling automatic ZUPT is useful for degraded GPS environments or applications with very slow velocities. */
    SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES     = (int)0x00010000,
    /** Disable automatic Zero Angular Rate Updates (ZARU).  Disabling automatic ZARU is useful for applications with small/slow angular rates. */
    SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES = (int)0x00020000,
    /** Disable INS EKF updates */
    SYS_CFG_BITS_DISABLE_INS_EKF                        = (int)0x00040000,
    /** Prevent built-in test (BIT) from running automatically on startup */
    SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP            = (int)0x00080000,

    /** Disable wheel encoder fusion */
    SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION           = (int)0x00100000,

    SYS_CFG_BITS_UNUSED3                                = (int)0x00200000,

    /** Brownout reset threshold voltage level */
    SYS_CFG_BITS_BOR_LEVEL_0                            = 0x0,              // 1.65 - 1.75 V  (default)
    SYS_CFG_BITS_BOR_LEVEL_1                            = 0x1,              // 2.0  - 2.1  V
    SYS_CFG_BITS_BOR_LEVEL_2                            = 0x2,              // 2.25 - 2.35 V
    SYS_CFG_BITS_BOR_LEVEL_3                            = 0x3,              // 2.5  - 2.6  V
    SYS_CFG_BITS_BOR_THRESHOLD_MASK                     = (int)0x00C00000,
    SYS_CFG_BITS_BOR_THRESHOLD_OFFSET                   = 22,

    /** Use reference IMU in EKF instead of onboard IMU */
    SYS_CFG_USE_REFERENCE_IMU_IN_EKF                    = (int)0x01000000,
    /** Reference point stationary on strobe input */
    SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT    = (int)0x02000000,
};

/** GPX GNSS satellite system signal constellation (used with nvm_flash_cfg_t.gnssSatSigConst) */
enum eGpxGnssSatSigConst
{
    /*! GPS  */
    GPX_GNSS_SAT_SIG_CONST_GPS_L1                       = (uint16_t)0x0001,
    GPX_GNSS_SAT_SIG_CONST_GPS_L5                       = (uint16_t)0x0002,
    GPX_GNSS_SAT_SIG_CONST_GPS                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GPS_L1 | GPX_GNSS_SAT_SIG_CONST_GPS_L5),
    /*! QZSS  */
    GPX_GNSS_SAT_SIG_CONST_QZS_L1                       = (uint16_t)0x0004,
    GPX_GNSS_SAT_SIG_CONST_QZS_L5                       = (uint16_t)0x0008,
    GPX_GNSS_SAT_SIG_CONST_QZS                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_QZS_L1 | GPX_GNSS_SAT_SIG_CONST_QZS_L5),
    /*! Galileo  */
    GPX_GNSS_SAT_SIG_CONST_GAL_E1                       = (uint16_t)0x0010,
    GPX_GNSS_SAT_SIG_CONST_GAL_E5                       = (uint16_t)0x0020,
    GPX_GNSS_SAT_SIG_CONST_GAL                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GAL_E1 | GPX_GNSS_SAT_SIG_CONST_GAL_E5),
    /*! BeiDou  */
    GPX_GNSS_SAT_SIG_CONST_BDS_B1                       = (uint16_t)0x0040,
    GPX_GNSS_SAT_SIG_CONST_BDS_B2                       = (uint16_t)0x0080,
    GPX_GNSS_SAT_SIG_CONST_BDS                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_BDS_B1 | GPX_GNSS_SAT_SIG_CONST_BDS_B2),
    /*! GLONASS  */
    GPX_GNSS_SAT_SIG_CONST_GLO_L1                       = (uint16_t)0x0300,
    GPX_GNSS_SAT_SIG_CONST_GLO                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GLO_L1),
    /*! SBAS  */
    GPX_GNSS_SAT_SIG_CONST_SBS_L1                       = (uint16_t)0x1000,
    GPX_GNSS_SAT_SIG_CONST_SBS                          = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_SBS_L1),
    /*! IRNSS / NavIC  */
    GPX_GNSS_SAT_SIG_CONST_IRN                          = (uint16_t)0x2000,
    /*! IMES  */
    GPX_GNSS_SAT_SIG_CONST_IME                          = (uint16_t)0x4000,
};

/** GNSS satellite system signal constellation (used with nvm_flash_cfg_t.gnssSatSigConst) */
enum eGnssSatSigConst
{
    /*! GPS  */
    GNSS_SAT_SIG_CONST_GPS                              = (uint16_t)0x0003,
    /*! QZSS  */
    GNSS_SAT_SIG_CONST_QZS                              = (uint16_t)0x000C,
    /*! Galileo  */
    GNSS_SAT_SIG_CONST_GAL                              = (uint16_t)0x0030,
    /*! BeiDou  */
    GNSS_SAT_SIG_CONST_BDS                              = (uint16_t)0x00C0,
    /*! GLONASS  */
    GNSS_SAT_SIG_CONST_GLO                              = (uint16_t)0x0300,
    /*! SBAS  */
    GNSS_SAT_SIG_CONST_SBS                              = (uint16_t)0x1000,
    /*! IRNSS / NavIC  */
    GNSS_SAT_SIG_CONST_IRN                              = (uint16_t)0x2000,
    /*! IMES  */
    GNSS_SAT_SIG_CONST_IME                              = (uint16_t)0x4000,

    /*! GNSS ALL */
    GNSS_SAT_SIG_CONST_ALL = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_QZS | \
        GNSS_SAT_SIG_CONST_GAL | \
        GNSS_SAT_SIG_CONST_BDS | \
        GNSS_SAT_SIG_CONST_GLO | \
        GNSS_SAT_SIG_CONST_SBS | \
        GNSS_SAT_SIG_CONST_IRN | \
        GNSS_SAT_SIG_CONST_IME,

    /*! GNSS default */
    GNSS_SAT_SIG_CONST_DEFAULT = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_SBS | \
        GNSS_SAT_SIG_CONST_QZS | \
        GNSS_SAT_SIG_CONST_GAL | \
        GNSS_SAT_SIG_CONST_GLO | \
        GNSS_SAT_SIG_CONST_BDS,

    GNSS_SAT_SIG_CONST_DEFAULT_INTEL = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_GAL,
};

/** RTK Configuration (used with nvm_flash_cfg_t.RTKCfgBits) */
enum eRTKConfigBits
{
    /** Enable onboard RTK GNSS precision positioning (GPS1) DEPRECATED */
    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED  = (int)0x00000001,

    /** Enable external RTK GNSS positioning (GPS1) */
    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING             = (int)0x00000002,
    
    /** Enable dual GNSS RTK compassing (GPS2 to GPS1) */
    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING              = (int)0x00000004,    

    /** Enable dual GNSS RTK compassing (GPS2 to GPS1) DEPRECATED */
    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED   = (int)0x00000008,

    /** Mask of RTK GNSS positioning types */
    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_MASK        = (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING | RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED),

    /** Mask of dual GNSS RTK compassing types */
    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_MASK         = (RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING | RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED),

    /** Mask of RTK position, heading, and base modes */
    RTK_CFG_BITS_ROVER_MODE_MASK                        = (int)0x0000000F,
    
    /** Enable RTK base and output ublox data from GPS 1 on serial port 0 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0            = (int)0x00000010,

    /** Enable RTK base and output ublox data from GPS 1 on serial port 1 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1            = (int)0x00000020,

    /** Enable RTK base and output ublox data from GPS 1 on serial port 2 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2            = (int)0x00000040,

    /** Enable RTK base and output ublox data from GPS 1 on USB port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB             = (int)0x00000080,

    /** Enable RTK base and output RTCM3 data from GPS 1 on serial port 0 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0            = (int)0x00000100,
    
    /** Enable RTK base and output RTCM3 data from GPS 1 on serial port 1 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1            = (int)0x00000200,

    /** Enable RTK base and output RTCM3 data from GPS 1 on serial port 2 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2            = (int)0x00000400,

    /** Enable RTK base and output RTCM3 data from GPS 1 on USB port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB             = (int)0x00000800,

    /** Enable RTK base and output ublox data from GPS 2 on serial port 0 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0            = (int)0x00001000,

    /** Enable RTK base and output ublox data from GPS 2 on serial port 1 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1            = (int)0x00002000,

    /** Enable RTK base and output ublox data from GPS 2 on serial port 2 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2            = (int)0x00004000,

    /** Enable RTK base and output ublox data from GPS 2 on USB port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB             = (int)0x00008000,

    /** Enable RTK base and output RTCM3 data from GPS 2 on serial port 0 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0            = (int)0x00010000,
    
    /** Enable RTK base and output RTCM3 data from GPS 2 on serial port 1 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1            = (int)0x00020000,

    /** Enable RTK base and output RTCM3 data from GPS 2 on serial port 2 */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2            = (int)0x00040000,

    /** Enable RTK base and output RTCM3 data from GPS 2 on USB port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB             = (int)0x00080000,

    /** Enable base mode moving position. (For future use. Not implemented. This bit should always be 0 for now.) TODO: Implement moving base. */
    RTK_CFG_BITS_BASE_POS_MOVING                        = (int)0x00100000,
    
    /** Reserved for future use */
    RTK_CFG_BITS_RESERVED1                              = (int)0x00200000,    
    
    /** When using RTK, specifies whether the base station is identical hardware to this rover. If so, there are optimizations enabled to get fix faster. */
    RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER         = (int)0x00400000,

    /** Forward all messages between the selected GPS and serial port.  Disable for RTK base use (to forward only GPS raw messages and use the surveyed location refLLA instead of current GPS position).  */
    RTK_CFG_BITS_GPS_PORT_PASS_THROUGH                  = (int)0x00800000,

    /** Enable RTK base and output RTCM3 data from GPS 1 on the current serial port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_CUR_PORT        = (int)0x01000000,

    /** Enable RTK base and output RTCM3 data from GPS 2 on the current serial port */
    RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_CUR_PORT        = (int)0x02000000,
    
    /** If this bit is set in conjuction with set current port this will clear current port */
    RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT       = (int)0x04000000,

    /** Mask of RTK base and output RTCM3 data on the current serial ports */
    RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK        = (RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_CUR_PORT | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_CUR_PORT),

    RTK_CFG_BITS_BASE_GNSS1_UBLOX_MASK = (
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB  ),

    RTK_CFG_BITS_BASE_GNSS2_UBLOX_MASK = (
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB  ),

    RTK_CFG_BITS_BASE_GNSS1_RTCM3_MASK = (
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB  ),

    RTK_CFG_BITS_BASE_GNSS2_RTCM3_MASK = (
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB  ),

    RTK_CFG_BITS_BASE_UBLOX_MASK = ( RTK_CFG_BITS_BASE_GNSS1_UBLOX_MASK | RTK_CFG_BITS_BASE_GNSS2_UBLOX_MASK),
    RTK_CFG_BITS_BASE_RTCM3_MASK = ( RTK_CFG_BITS_BASE_GNSS1_RTCM3_MASK | RTK_CFG_BITS_BASE_GNSS2_RTCM3_MASK),

    /** All base station bits */
    RTK_CFG_BITS_BASE_MODE = ( RTK_CFG_BITS_BASE_UBLOX_MASK | RTK_CFG_BITS_BASE_RTCM3_MASK),

    /** Base station bits enabled on Ser0 */
    RTK_CFG_BITS_RTK_BASE_SER0 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0),

    /** Base station bits enabled on Ser1 */
    RTK_CFG_BITS_RTK_BASE_SER1 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1),

    /** Base station bits enabled on Ser2 */
    RTK_CFG_BITS_RTK_BASE_SER2 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2),

    /** Base station bits for GPS1 Ublox */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS1_UBLOX = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB),

    /** Base station bits for GPS2 Ublox */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS2_UBLOX = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB),

    /** Base station bits for GPS1 RTCM */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS1_RTCM = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 | 
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 | 
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB),

    /** Base station bits for GPS2 RTCM */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS2_RTCM = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB),

    /** Rover on-board RTK engine used */
    RTK_CFG_BITS_ROVER_MODE_ONBOARD_MASK = (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED | RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED),

    /** Mask of Rover, Compassing, and Base modes */
    RTK_CFG_BITS_ALL_MODES_MASK = (RTK_CFG_BITS_ROVER_MODE_MASK | RTK_CFG_BITS_BASE_MODE),    
};

#define DEFAULT_DYNAMIC_MODEL                   DYNAMIC_MODEL_AIRBORNE_4G
#define DEFAULT_GNSS_MIN_ELEVATION_ANGLE        (10.0f * C_DEG2RAD_F)  // (rad)
#define DEFAULT_GNSS_RTK_CN0_MINIMUM            30  // (dBHz)
#define DEFAULT_GNSS_RTK_CN0_DYN_MIN_OFFSET     20  // (dBHz)

/** Sensor Configuration (used with nvm_flash_cfg_t.sensorConfig) */
enum eSensorConfig
{
    /** Gyro full-scale sensing range selection: +- 250, 500, 1000, 2000, 4000 deg/s */    
    SENSOR_CFG_GYR_FS_250               = (int)0x00000000,
    SENSOR_CFG_GYR_FS_500               = (int)0x00000001,
    SENSOR_CFG_GYR_FS_1000              = (int)0x00000002,
    SENSOR_CFG_GYR_FS_2000              = (int)0x00000003,
    SENSOR_CFG_GYR_FS_4000              = (int)0x00000004,
    SENSOR_CFG_GYR_FS_MASK              = (int)0x00000007,
    SENSOR_CFG_GYR_FS_OFFSET            = (int)0,
    
    /** Accelerometer full-scale sensing range selection: +- 2, 4, 8, 16 m/s^2 */
    SENSOR_CFG_ACC_FS_2G                = (int)0x00000000,
    SENSOR_CFG_ACC_FS_4G                = (int)0x00000001,
    SENSOR_CFG_ACC_FS_8G                = (int)0x00000002,
    SENSOR_CFG_ACC_FS_16G               = (int)0x00000003,
    SENSOR_CFG_ACC_FS_MASK              = (int)0x00000030,
    SENSOR_CFG_ACC_FS_OFFSET            = (int)4,
    
    /** Gyro digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The following 
    bit values can be used to override the bandwidth (frequency) to: 250, 184, 92, 41, 20, 10, 5 Hz */
    SENSOR_CFG_GYR_DLPF_250HZ           = (int)0x00000000,
    SENSOR_CFG_GYR_DLPF_184HZ           = (int)0x00000001,
    SENSOR_CFG_GYR_DLPF_92HZ            = (int)0x00000002,
    SENSOR_CFG_GYR_DLPF_41HZ            = (int)0x00000003,
    SENSOR_CFG_GYR_DLPF_20HZ            = (int)0x00000004,
    SENSOR_CFG_GYR_DLPF_10HZ            = (int)0x00000005,
    SENSOR_CFG_GYR_DLPF_5HZ             = (int)0x00000006,
    SENSOR_CFG_GYR_DLPF_MASK            = (int)0x00000F00,
    SENSOR_CFG_GYR_DLPF_OFFSET          = (int)8,

    /** Accelerometer digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The 
    following bit values can be used to override the bandwidth (frequency) to: 218, 218, 99, 45, 21, 10, 5 Hz */
    SENSOR_CFG_ACC_DLPF_218HZ           = (int)0x00000000,
    SENSOR_CFG_ACC_DLPF_218HZb          = (int)0x00000001,
    SENSOR_CFG_ACC_DLPF_99HZ            = (int)0x00000002,
    SENSOR_CFG_ACC_DLPF_45HZ            = (int)0x00000003,
    SENSOR_CFG_ACC_DLPF_21HZ            = (int)0x00000004,
    SENSOR_CFG_ACC_DLPF_10HZ            = (int)0x00000005,
    SENSOR_CFG_ACC_DLPF_5HZ             = (int)0x00000006,
    SENSOR_CFG_ACC_DLPF_MASK            = (int)0x0000F000,
    SENSOR_CFG_ACC_DLPF_OFFSET          = (int)12,

    /** Euler rotation of IMU and magnetometer from Hardware Frame to Sensor Frame.  Rotation applied in the order of yaw, pitch, roll from the sensor frame (labeled on uINS). */
    SENSOR_CFG_SENSOR_ROTATION_MASK             = (int)0x001F0000,
    SENSOR_CFG_SENSOR_ROTATION_OFFSET           = (int)16,
    SENSOR_CFG_SENSOR_ROTATION_0_0_0            = (int)0,    // roll, pitch, yaw rotation (deg).
    SENSOR_CFG_SENSOR_ROTATION_0_0_90           = (int)1,
    SENSOR_CFG_SENSOR_ROTATION_0_0_180          = (int)2,
    SENSOR_CFG_SENSOR_ROTATION_0_0_N90          = (int)3,
    SENSOR_CFG_SENSOR_ROTATION_90_0_0           = (int)4,
    SENSOR_CFG_SENSOR_ROTATION_90_0_90          = (int)5,
    SENSOR_CFG_SENSOR_ROTATION_90_0_180         = (int)6,
    SENSOR_CFG_SENSOR_ROTATION_90_0_N90         = (int)7,
    SENSOR_CFG_SENSOR_ROTATION_180_0_0          = (int)8,
    SENSOR_CFG_SENSOR_ROTATION_180_0_90         = (int)9,
    SENSOR_CFG_SENSOR_ROTATION_180_0_180        = (int)10,
    SENSOR_CFG_SENSOR_ROTATION_180_0_N90        = (int)11,
    SENSOR_CFG_SENSOR_ROTATION_N90_0_0          = (int)12,
    SENSOR_CFG_SENSOR_ROTATION_N90_0_90         = (int)13,
    SENSOR_CFG_SENSOR_ROTATION_N90_0_180        = (int)14,
    SENSOR_CFG_SENSOR_ROTATION_N90_0_N90        = (int)15,
    SENSOR_CFG_SENSOR_ROTATION_0_90_0           = (int)16,
    SENSOR_CFG_SENSOR_ROTATION_0_90_90          = (int)17,
    SENSOR_CFG_SENSOR_ROTATION_0_90_180         = (int)18,
    SENSOR_CFG_SENSOR_ROTATION_0_90_N90         = (int)19,
    SENSOR_CFG_SENSOR_ROTATION_0_N90_0          = (int)20,
    SENSOR_CFG_SENSOR_ROTATION_0_N90_90         = (int)21,
    SENSOR_CFG_SENSOR_ROTATION_0_N90_180        = (int)22,
    SENSOR_CFG_SENSOR_ROTATION_0_N90_N90        = (int)23,

    /** Magnetometer output data rate (ODR).  Set to enable 100Hz output data rate.  System reset required to enable. */
    // SENSOR_CFG_MAG_ODR_100_HZ                   = (int)0x00200000,       // This is commented out to save instruction space memory.  Uncomment after the system has been optimized.

    /** Disable magnetometer sensor (sensorConfig[22]) */    
    SENSOR_CFG_DISABLE_MAGNETOMETER             = (int)0x00400000,
    /** Disable barometometer sensor (sensorConfig[23]) */    
    SENSOR_CFG_DISABLE_BAROMETER                = (int)0x00800000,

    /** Triple IMU fault detection level. Higher levels add new features to previous levels */
    SENSOR_CFG_IMU_FAULT_DETECT_MASK            = (int)0xFF000000,
    SENSOR_CFG_IMU_FAULT_DETECT_GYR             = (int)0x01000000,      // Enable triple IMU gyro fault detection.           Must be enabled for other gyr detection modes (offline, large bias, and noise).
    SENSOR_CFG_IMU_FAULT_DETECT_ACC             = (int)0x02000000,      // Enable triple IMU accelerometer fault detection.  Must be enabled for other acc detection modes (offline, large bias, and noise).

    // Set to ZERO to exclude from build
    SENSOR_CFG_IMU_FAULT_DETECT_OFFLINE         = 0,    // (int)0x04000000,      // One or more IMUs is offline or stuck
    SENSOR_CFG_IMU_FAULT_DETECT_LARGE_BIAS      = 0,    // (int)0x08000000,
    SENSOR_CFG_IMU_FAULT_DETECT_SENSOR_NOISE    = 0,    // (int)0x10000000,
};

/** IO configuration (used with nvm_flash_cfg_t.ioConfig) */
enum eIoConfig
{
    /** Strobe (input and output) trigger on rising edge (0 = falling edge) (ioConfig[0]) */
    IO_CONFIG_STROBE_TRIGGER_HIGH               = (int)0x00000001,

    // G1,G2 - STROBE, CAN, Ser2, I2C (future) (ioConfig[3-1])
    /** G1,G2 - STROBE input on G2 */
    IO_CONFIG_G1G2_STROBE_INPUT_G2              = (int)0x00000002,
    /** G1,G2 - CAN Bus */
    IO_CONFIG_G1G2_CAN_BUS                      = (int)0x00000004,
    /** G1,G2 - General Communications on Ser2. Excludes GPS communications. */
    IO_CONFIG_G1G2_COM2                         = (int)0x00000006,
    /** G1,G2 - I2C */
    IO_CONFIG_G1G2_I2C                          = (int)0x00000008,
    /** G1,G2 - MASK.  Note: This G1,G2 setting is overridden when GPS1 or GPS2 is configured to use Ser2. */
    IO_CONFIG_G1G2_MASK                         = (int)0x0000000E,
    /** G1,G2 - Default */
    IO_CONFIG_G1G2_DEFAULT                      = IO_CONFIG_G1G2_COM2,

    // G9 - STROBE, QDEC0 (future) (ioConfig[5-4])
    /** G9 - Strobe input */
    IO_CONFIG_G9_STROBE_INPUT                   = (int)0x00000010,
    /** G9 - Enable Nav update strobe output pulse on G9 (uINS pin 10) indicating preintegrated IMU and navigation updates */
    IO_CONFIG_G9_STROBE_OUTPUT_NAV              = (int)0x00000020,
    /** G9 - SPI DRDY */
    IO_CONFIG_G9_SPI_DRDY                       = (int)0x00000030,
    /** G9 - Bit mask */
    IO_CONFIG_G9_MASK                           = (int)0x00000030,
    /** G9 - Default */
    IO_CONFIG_G9_DEFAULT                        = (int)0,    

    // G6,G7 - Ser1, QDEC0 (future) (ioConfig[7-6])
    /** G6,G7 - General Communications on Ser1. Excludes GPS communications.  Overriden when SPI is enabled (G9 held low on bootup/config). */
    IO_CONFIG_G6G7_COM1                         = (int)0x00000040,
    /** G6,G7 - Quadrature wheel encoder input (G6 QDEC0-A).  Overriden when SPI is enabled (G9 held low on bootup/config). */
//  IO_CONFIG_G6G7_QDEC0_INPUT_G6               = (int)0x00000080,
    /** G6,G7 - Bit mask */
    IO_CONFIG_G6G7_MASK                         = (int)0x000000C0,
    /** G6,G7 - Default */
    IO_CONFIG_G6G7_DEFAULT                      = IO_CONFIG_G6G7_COM1,    

    // G5,G8 - STROBE, QDEC1 (future), SPI (enabled when G9 is held low on bootup/config) (ioConfig[10-8])
    /** G5,G8 - Strobe input on G5 */
    IO_CONFIG_G5G8_STROBE_INPUT_G5              = (int)0x00000100,
    /** G5,G8 - Strobe input on G8 */
    IO_CONFIG_G5G8_STROBE_INPUT_G8              = (int)0x00000200,
    /** G5,G8 - Strobe input on both G5 and G8 */
    IO_CONFIG_G5G8_STROBE_INPUT_G5_G8           = (int)0x00000300,
    /** G5,G8 - Strobe input on both G5 and G8 */
    IO_CONFIG_G5G8_G6G7_SPI_ENABLE              = (int)0x00000400,
    /** G5,G8 - Quadrature wheel encoder input (G5 QDEC1-B, G8 QDEC1-A) */
    IO_CONFIG_G5G8_QDEC_INPUT                   = (int)0x00000500,
    /** G5,G8 - Bit mask */
    IO_CONFIG_G5G8_MASK                         = (int)0x00000700,
    /** G5,G8 - Default */
    IO_CONFIG_G5G8_DEFAULT                      = (int)0,    

    /** G15 (GPS PPS) - STROBE (ioConfig[11]) */
    IO_CONFIG_G15_STROBE_INPUT                  = (int)0x00000800,

    /** GPS TIMEPULSE source (ioConfig[15-13]) */
    IO_CFG_GNSS1_PPS_SOURCE_OFFSET              = (int)13,
    IO_CFG_GNSS1_PPS_SOURCE_MASK                = (int)0x00000007,
    IO_CFG_GNSS1_PPS_SOURCE_BITMASK             = (int)(IO_CFG_GNSS1_PPS_SOURCE_MASK<<IO_CFG_GNSS1_PPS_SOURCE_OFFSET),    
    IO_CFG_GNSS1_PPS_SOURCE_DISABLED            = (int)0,
    IO_CFG_GNSS1_PPS_SOURCE_G15                 = (int)1,
    IO_CFG_GNSS1_PPS_SOURCE_G2                  = (int)3,
    IO_CFG_GNSS1_PPS_SOURCE_G5                  = (int)4,
    IO_CFG_GNSS1_PPS_SOURCE_G8                  = (int)5,
    IO_CFG_GNSS1_PPS_SOURCE_G9                  = (int)6,

 #define SET_STATUS_OFFSET_MASK(result,val,offset,mask)    { (result) &= ~((mask)<<(offset)); (result) |= ((val)<<(offset)); }    
 #define IO_CFG_GNSS1_PPS_SOURCE(ioConfig) (((ioConfig)>>IO_CFG_GNSS1_PPS_SOURCE_OFFSET)&IO_CFG_GNSS1_PPS_SOURCE_MASK)
     
     /** GPS 1 source OFFSET (ioConfig[18-16]) */
     IO_CONFIG_GPS1_SOURCE_OFFSET                = (int)16,
     /** GPS 2 source OFFSET (ioConfig[21-19]) */
     IO_CONFIG_GPS2_SOURCE_OFFSET                = (int)19,
     /** GPS 1 type OFFSET   (ioConfig[24-22]) */
     IO_CONFIG_GPS1_TYPE_OFFSET                  = (int)22,
     /** GPS 2 type OFFSET   (ioConfig[27-25]) */
     IO_CONFIG_GPS2_TYPE_OFFSET                  = (int)25,

    /** GPS 1 skip initialization (ioConfig[12]) */
    IO_CONFIG_GPS1_NO_INIT                      = (int)0x00001000,
    /** GPS 2 skip initialization (ioConfig[28]) */
    IO_CONFIG_GPS2_NO_INIT                      = (int)0x10000000,

    /** GPS source MASK */
    IO_CONFIG_GPS_SOURCE_MASK                   = (int)0x00000007,
    /** GPS source - Disable */
    IO_CONFIG_GPS_SOURCE_DISABLE                = (int)0,
    /** GPS source - Serial 0 */
    IO_CONFIG_GPS_SOURCE_SER0                   = (int)3,
    /** GPS source - Serial 1 */
    IO_CONFIG_GPS_SOURCE_SER1                   = (int)4,
    /** GPS source - Serial 2 */
    IO_CONFIG_GPS_SOURCE_SER2                   = (int)5,
    /** GPS source - last type */
    IO_CONFIG_GPS_SOURCE_LAST                   = IO_CONFIG_GPS_SOURCE_SER2,    // set to last source

    /** GPS type MASK */
    IO_CONFIG_GPS_TYPE_MASK                     = (int)0x00000007,
    /** GPS type - Unused.  USE this when adding a new GNSS Receiver */
    IO_CONFIG_GPS_TYPE_UNUSED                    = (int)0,
    /** GPS type - ublox ZED-F9P w/ RTK */
    IO_CONFIG_GPS_TYPE_UBX_F9P                  = (int)1,
    /** GPS type - NMEA */
    IO_CONFIG_GPS_TYPE_NMEA                     = (int)2,
    /** GPS type - InertialSense GPX */
    IO_CONFIG_GPS_TYPE_GPX                        = (int)3,
    /** GPS type - last type */
    IO_CONFIG_GPS_TYPE_LAST                        = IO_CONFIG_GPS_TYPE_GPX,        // Set to last type

#define IO_CONFIG_GPS1_SOURCE(ioConfig)     (((ioConfig)>>IO_CONFIG_GPS1_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS2_SOURCE(ioConfig)     (((ioConfig)>>IO_CONFIG_GPS2_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS1_TYPE(ioConfig)       (((ioConfig)>>IO_CONFIG_GPS1_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)
#define IO_CONFIG_GPS2_TYPE(ioConfig)       (((ioConfig)>>IO_CONFIG_GPS2_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)

#define SET_IO_CFG_GPS1_SOURCE(result,val)  SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS1_SOURCE_OFFSET, IO_CONFIG_GPS_SOURCE_MASK)
#define SET_IO_CFG_GPS2_SOURCE(result,val)  SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS2_SOURCE_OFFSET, IO_CONFIG_GPS_SOURCE_MASK)
#define SET_IO_CFG_GPS1_TYPE(result,val)    SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS1_TYPE_OFFSET, IO_CONFIG_GPS_TYPE_MASK)
#define SET_IO_CFG_GPS2_TYPE(result,val)    SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS2_TYPE_OFFSET, IO_CONFIG_GPS_TYPE_MASK)

    /** IMU 1 disable (ioConfig[29]) */    
    IO_CONFIG_IMU_1_DISABLE                         = (int)0x20000000,
    /** IMU 2 disable (ioConfig[30]) */
    IO_CONFIG_IMU_2_DISABLE                         = (int)0x40000000,
    /** IMU 3 disable (ioConfig[31]) */
    IO_CONFIG_IMU_3_DISABLE                         = (int)0x80000000,
};

#define IO_CONFIG_DEFAULT     (IO_CONFIG_G1G2_DEFAULT | IO_CONFIG_G5G8_DEFAULT | IO_CONFIG_G6G7_DEFAULT | IO_CONFIG_G9_DEFAULT)    

enum eIoConfig2
{
    // NOTE IO_CFG_G11_STROBE_INPUT and IO_CFG_G12_STROBE_INPUT
    // cannot be set at the same time. If this is attemped 
    // IO_CFG_G12_STROBE_INPUT will be set to IO_CFG_G12_SWO
    /** G11 (SWDIO) - (eIoConfig2[0]) */
    IO_CFG_G11_OFFSET                       = (int)0,
    IO_CFG_G11_MASK                         = (int)0x01,
    IO_CFG_G11_BITMASK                      = (int)(IO_CFG_G11_MASK<<IO_CFG_G11_OFFSET),
    IO_CFG_G11_SWDIO                        = (int)0,
    IO_CFG_G11_STROBE_INPUT                 = (int)1,
    IO_CFG_G11_SWDIO_val                    = (int)0x00,
    IO_CFG_G11_STROBE_INPUT_val             = (int)0x01,
    IO_CFG_G11_DEFAULT                      = IO_CFG_G11_SWDIO,


    /** G12 (SWO) - (eIoConfig2[2-1]) */
    IO_CFG_G12_OFFSET                       = (int)1,
    IO_CFG_G12_MASK                         = (int)0x03,
    IO_CFG_G12_BITMASK                      = (int)(IO_CFG_G12_MASK<<IO_CFG_G12_OFFSET),
    IO_CFG_G12_SWO                          = (int)0,
    IO_CFG_G12_XSCL                         = (int)1,
    IO_CFG_G12_STROBE_INPUT                 = (int)2,
    IO_CFG_G12_SWO_val                      = (int)0x00,
    IO_CFG_G12_XSCL_val                     = (int)0x02,
    IO_CFG_G12_STROBE_INPUT_val             = (int)0x04,
    IO_CFG_G12_DEFAULT                      = IO_CFG_G12_SWO,

    /** G13 (DRDY) - (eIoConfig2[4-3]) */
    IO_CFG_G13_OFFSET                       = (int)3,
    IO_CFG_G13_MASK                         = (int)0x03,
    IO_CFG_G13_BITMASK                      = (int)(IO_CFG_G13_MASK<<IO_CFG_G13_OFFSET),
    IO_CFG_G13_DRDY                         = (int)0,
    IO_CFG_G13_XSDA                         = (int)1,
    IO_CFG_G13_STROBE_INPUT                 = (int)2,
    IO_CFG_G13_DRDY_val                     = (int)0x00,
    IO_CFG_G13_XSDA_val                     = (int)0x08,
    IO_CFG_G13_STROBE_INPUT_val             = (int)0x10,
    IO_CFG_G13_DEFAULT                      = (int)IO_CFG_G13_DRDY,

    /** UNUSED (eIoConfig2[5]) */

    /** GNSS2 TIMEPULSE source (eIoConfig2[7-6]) */
    IO_CFG_GNSS2_PPS_SOURCE_OFFSET          = (int)6,
    IO_CFG_GNSS2_PPS_SOURCE_MASK            = (int)0x03,
    IO_CFG_GNSS2_PPS_SOURCE_BITMASK         = (int)(IO_CFG_GNSS2_PPS_SOURCE_MASK<<IO_CFG_GNSS2_PPS_SOURCE_OFFSET),    
    IO_CFG_GNSS2_PPS_SOURCE_DISABLED        = (int)0,
    IO_CFG_GNSS2_PPS_SOURCE_G11             = (int)1,
    IO_CFG_GNSS2_PPS_SOURCE_G12             = (int)2,
    IO_CFG_GNSS2_PPS_SOURCE_G13             = (int)3,    
    IO_CFG_GNSS2_PPS_SOURCE_DISABLED_val    = (int)0x00,
    IO_CFG_GNSS2_PPS_SOURCE_G11_val         = (int)0x04,
    IO_CFG_GNSS2_PPS_SOURCE_G12_val         = (int)0x80,
    IO_CFG_GNSS2_PPS_SOURCE_G13_val         = (int)0xC0,
};

#define IO_CFG_GNSS2_PPS_SOURCE(ioConfig) (((ioConfig)>>IO_CFG_GNSS2_PPS_SOURCE_OFFSET)&IO_CFG_GNSS2_PPS_SOURCE_MASK)

enum ePlatformConfig
{
    // IMX Carrier Board
    PLATFORM_CFG_TYPE_MASK                      = (int)0x0000003F,
    PLATFORM_CFG_TYPE_FROM_MANF_OTP             = (int)0x00000080,  // Type is overwritten from manufacturing OTP memory.  Write protection, prevents direct change of platformType in flashConfig.
    PLATFORM_CFG_TYPE_NONE                      = (int)0,           // IMX-5 default
    PLATFORM_CFG_TYPE_RUG3_G0                   = (int)8,           // PCB RUG-3.x.  GPS1 timepulse on G15/GNSS_PPS TIMESYNC (pin 20)
    PLATFORM_CFG_TYPE_RUG3_G1                   = (int)9,           // "
    PLATFORM_CFG_TYPE_RUG3_G2                   = (int)10,          // "
    PLATFORM_CFG_TYPE_EVB2_G2                   = (int)11,          
    PLATFORM_CFG_TYPE_TBED3                     = (int)12,          // Testbed-3
    PLATFORM_CFG_TYPE_IG1_0_G2                  = (int)13,          // PCB IG-1.0.  GPS1 timepulse on G8
    PLATFORM_CFG_TYPE_IG1_G1                    = (int)14,          // PCB IG-1.1 and later.  GPS1 timepulse on G15/GNSS_PPS TIMESYNC (pin 20)
    PLATFORM_CFG_TYPE_IG1_G2                    = (int)15,  
    PLATFORM_CFG_TYPE_IG2                       = (int)16,          // IG-2 and IS-IMX-GPX-DEV-1 (w/ IMX-5 and GPX-1)
    PLATFORM_CFG_TYPE_LAMBDA_G1                 = (int)17,          // Enable UBX output on Lambda for testbed
    PLATFORM_CFG_TYPE_LAMBDA_G2                 = (int)18,          // "
    PLATFORM_CFG_TYPE_TBED2_G1_W_LAMBDA         = (int)19,          // Enable UBX input from Lambda
    PLATFORM_CFG_TYPE_TBED2_G2_W_LAMBDA         = (int)20,          // "
    PLATFORM_CFG_TYPE_COUNT                     = (int)21,

    // Presets
    PLATFORM_CFG_PRESET_MASK                    = (int)0x0000FF00,
    PLATFORM_CFG_PRESET_OFFSET                  = (int)8,

    // RUG-3 - Presets
    PLATFORM_CFG_RUG3_PRESET__0__PRESETS_DISABLED                               = 0,    // Don't use presets.  IOEXP_BITS can be set directly.
    PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1          = 1,    // RUG-3-G0 default
    PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GPS1          = 2,
    PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GPS1          = 3,
    PLATFORM_CFG_RUG3_PRESET__4__S0_RS232_7_9___S1_RS232_8_10__S2_GPS1          = 4,
    PLATFORM_CFG_RUG3_PRESET__5__S1_RS485_7_8_9_10_____________S2_GPS1__S0_GPS2 = 5,
    PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GPS1__S0_GPS2 = 6,
    PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2 = 7,    // RUG-3-G2 default
    PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GPS1__S0_GPS2 = 8,
    PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GPS1__S0_GPS2 = 9,
    PLATFORM_CFG_RUG3_PRESET__COUNT                                             = 10,

    PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT    = PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1,
    PLATFORM_CFG_RUG3_PRESET__G2_DEFAULT    = PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2,

    // RUG-3 - I/O Expander disabled if platform type is != PLATFORM_CFG_TYPE_RUG3_x.
    PLATFORM_CFG_RUG3_IOEXP_BIT_MASK            = (int)0x00FF0000,
    PLATFORM_CFG_RUG3_IOEXP_BIT_OFFSET          = (int)16,

    RUG3_IOEXP_BIT_OFFSET_n232_485              = (int)0,
    RUG3_IOEXP_BIT_OFFSET_n232_TTL              = (int)1,
    RUG3_IOEXP_BIT_OFFSET_nRS_CAN               = (int)2,
    RUG3_IOEXP_BIT_OFFSET_nGPS2_RS              = (int)3,
    RUG3_IOEXP_BIT_OFFSET_nSPIEN                = (int)4,
    RUG3_IOEXP_BIT_OFFSET_nSPI_SER              = (int)5,
    RUG3_IOEXP_BIT_OFFSET_nGPSRST               = (int)6,

    PLATFORM_CFG_UPDATE_IO_CONFIG               = (int)0x01000000,    // Generate ioConfig based on platform config
};

/** (DID_WHEEL_ENCODER) Message to communicate wheel encoder measurements to GPS-INS */
typedef struct PACKED
{
    /** (Do not use, internal development only) Time of measurement in current GPS week */
    double timeOfWeek;

    /** Status */
    uint32_t status;

    /** (Do not use, internal development only) Left wheel angle (rad) */
    float theta_l;

    /** (Do not use, internal development only) Right wheel angle (rad) */
    float theta_r;
    
    /** Left wheel angular rate (rad/s). Positive when wheel is turning toward the forward direction of the vehicle. Use WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT in DID_FLASH_CONFIG::wheelConfig to reverse this. */
    float omega_l;

    /** Right wheel angular rate (rad/s). Positive when wheel is turning toward the forward direction of the vehicle. Use WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT in DID_FLASH_CONFIG::wheelConfig to reverse this. */
    float omega_r;

    /** (Do not use, internal development only) Left wheel revolution count */
    uint32_t wrap_count_l;

    /** (Do not use, internal development only) Right wheel revolution count */
    uint32_t wrap_count_r;

    /** Wheel encoder velocity noise variance (rad^2/s^2) */
    float var_wheel_omega;

    /** Wheel encoder angle noise variance (rad^2) */
    float var_wheel_theta;

} wheel_encoder_t;

enum eWheelCfgBits
{
    WHEEL_CFG_BITS_ENABLE_ENCODER           = (int)0x00000002,
    WHEEL_CFG_BITS_ENABLE_CONTROL           = (int)0x00000004,
    WHEEL_CFG_BITS_ENABLE_MASK              = (int)0x0000000F,
    WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT   = (int)0x00000100,  // Used to reverse direction of DID_WHEEL_ENCODER::omega_l 
    WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT  = (int)0x00000200,  // Used to reverse direction of DID_WHEEL_ENCODER::omega_r
    WHEEL_CFG_BITS_ENCODER_SOURCE           = (int)0x00000400,  // 0 = uINS, 1 = EVB
};

typedef enum
{
    GV_MODE_STANDBY                         = 0,
    GV_MODE_LEARNING                        = 1,
    GV_CMD_LEARNING_START                   = 2,    // Use provided transform and sigma
    GV_CMD_LEARNING_RESUME                  = 3,    // Reset sigma values
    GV_CMD_LEARNING_CLEAR_AND_START         = 4,    // Zero transform and reset sigma values
    GV_CMD_LEARNING_STOP_AND_SAVE           = 5,
    GV_CMD_LEARNING_CANCEL                  = 6,
 } eGroundVehicleMode;

typedef struct PACKED
{
    /** Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle) in radians */
    float                   e_b2w[3];

    /** Euler angle standard deviation of measurements describing the rotation from imu (body) to the wheel frame (center of the non-steering axle) in radians */
    float                   e_b2w_sigma[3];

    /** Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame in meters */
    float                   t_b2w[3];

    /** Translation standard deviation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame in meters */
    float                   t_b2w_sigma[3];

} wheel_transform_t;

typedef struct PACKED
{
    /** Config bits (see eWheelCfgBits) */
    uint32_t                bits;

    /** Euler angles and offset describing the rotation and tranlation from imu (body) to the wheel frame (center of the non-steering axle) */
    wheel_transform_t       transform;

    /** Distance between the left and right wheels */
    float                   track_width;

    /** Estimate of wheel radius */
    float                   radius;

} wheel_config_t;

typedef enum
{
    /** Kinematic learing is solving for the translation from IMU to wheel (wheel_config). */ 
    GV_STATUS_LEARNING_ENABLED      = 0x00000001,
    
    /** Navigation is running without GPS input. */ 
    GV_STATUS_DEAD_RECKONING        = 0x01000000,

    /** Vehicle kinematic parameters agree with GPS. */ 
    GV_STATUS_KINEMATIC_CAL_GOOD    = 0x02000000,

    /** Vehicle kinematic learning has converged and is complete. */ 
    GV_STATUS_LEARNING_CONVERGED    = 0x04000000,

    /** Vehicle kinematic learning data (wheel_config_t) is missing. */ 
    GV_STATUS_LEARNING_NEEDED       = 0x08000000,

} eGroundVehicleStatus;

/** (DID_GROUND_VEHICLE) Configuration of ground vehicle kinematic constraints. */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Ground vehicle status flags (eGroundVehicleStatus) */
    uint32_t                status;

    /** Current mode of the ground vehicle.  Use this field to apply commands. (see eGroundVehicleMode) */
    uint32_t                mode;

    /** Wheel transform, track width, and wheel radius. */
    wheel_config_t          wheelConfig;

} ground_vehicle_t;

typedef enum
{
    DYNAMIC_MODEL_PORTABLE          = 0,
    DYNAMIC_MODEL_FIXED_POSITION    = 1,
    DYNAMIC_MODEL_STATIONARY        = 2,
    DYNAMIC_MODEL_PEDESTRIAN        = 3,
    DYNAMIC_MODEL_GROUND_VEHICLE    = 4,
    DYNAMIC_MODEL_MARINE            = 5,
    DYNAMIC_MODEL_AIRBORNE_1G       = 6,
    DYNAMIC_MODEL_AIRBORNE_2G       = 7,
    DYNAMIC_MODEL_AIRBORNE_4G       = 8,
    DYNAMIC_MODEL_WRIST             = 9,
    DYNAMIC_MODEL_INDOOR            = 10,
    DYNAMIC_MODEL_COUNT    // Must be last
} eDynamicModel;

typedef enum
{
    IMU_SHOCK_OPTIONS_ENABLE            = 0x01,
    IMU_SHOCK_OPTIONS_FAST_RECOVERY     = 0x02
} eImuShockOptions;

/** (DID_FLASH_CONFIG) Configuration data
 * IMPORTANT: These fields should not be deleted, they can be deprecated and marked as reserved,
 * or new fields added to the end.  
 * NOTE: The key value must be incremented to ensure the defaults are restored anytime the fields 
 * change or the default values change.  Default changes should be noted in the changelog.
*/
typedef struct PACKED
{
    /** Size of group or union, which is nvm_group_x_t + padding */
    uint32_t                size;

    /** Checksum, excluding size and checksum.  0xFFFFFFFF is invalid. */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /** IMU sample (system input) period in milliseconds set on startup. Cannot be larger than startupNavDtMs. Zero disables sensor/IMU sampling. */
    uint32_t                startupImuDtMs;

    /** Navigation filter (system output) output period in milliseconds set on startup.  Used to initialize sysParams.navOutputPeriodMs. */
    uint32_t                startupNavDtMs;

    /** Serial port 0 baud rate in bits per second */
    uint32_t                ser0BaudRate;

    /** Serial port 1 baud rate in bits per second */
    uint32_t                ser1BaudRate;

    /** Rotation in radians about the X,Y,Z axes from Sensor Frame to Intermediate Output Frame.  Order applied: Z,Y,X. */
    float                   insRotation[3];

    /** X,Y,Z offset in meters from Intermediate Output Frame to INS Output Frame. */
    float                   insOffset[3];

    /** X,Y,Z offset in meters in Sensor Frame to GPS 1 antenna. */
    float                   gps1AntOffset[3];
 
    /** INS dynamic platform model (see eDynamicModel).  Options are: 0=PORTABLE, 2=STATIONARY, 3=PEDESTRIAN, 4=GROUND VEHICLE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST.  Used to balance noise and performance characteristics of the system.  The dynamics selected here must be at least as fast as your system or you experience accuracy error.  This is tied to the GPS position estimation model and intend in the future to be incorporated into the INS position model. */
    uint8_t                 dynamicModel;

    /** Debug */
    uint8_t                 debug;

    /** Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS */
    uint16_t                gnssSatSigConst;

    /** System configuration bits (see eSysConfigBits). */
    uint32_t                sysCfgBits;

    /** Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations (deg, deg, m) */
    double                  refLla[3];

    /** Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup (deg, deg, m).  Updated when the distance between current LLA and lastLla exceeds lastLlaUpdateDistance. */
    double                  lastLla[3];

    /** Last LLA GPS time since week start (Sunday morning) in milliseconds */
    uint32_t                lastLlaTimeOfWeekMs;

    /** Last LLA GPS number of weeks since January 6th, 1980 */
    uint32_t                lastLlaWeek;

    /** Distance between current and last LLA that triggers an update of lastLla  */
    float                   lastLlaUpdateDistance;

    /** Hardware interface configuration bits (see eIoConfig). */
    uint32_t                ioConfig;

    /** Hardware platform specifying the IMX carrier board type (i.e. RUG, EVB, IG) and configuration bits (see ePlatformConfig).  The platform type is used to simplify the GPS and I/O configuration process.  Bit PLATFORM_CFG_UPDATE_IO_CONFIG is excluded from the flashConfig checksum and from determining whether to upload. */
    uint32_t                platformConfig;

    /** X,Y,Z offset in meters in Sensor Frame origin to GPS 2 antenna. */
    float                   gps2AntOffset[3];

    /** Euler (roll, pitch, yaw) rotation in radians from INS Sensor Frame to Intermediate ZeroVelocity Frame.  Order applied: heading, pitch, roll. */
    float                   zeroVelRotation[3];

    /** X,Y,Z offset in meters from Intermediate ZeroVelocity Frame to Zero Velocity Frame. */
    float                   zeroVelOffset[3];

    /** (sec) User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.  */
    float                   gpsTimeUserDelay;

    /** Earth magnetic field (magnetic north) declination (heading offset from true north) in radians */
    float                   magDeclination;

    /** Time between GPS time synchronization pulses in milliseconds.  Requires reboot to take effect. */
    uint32_t                gpsTimeSyncPeriodMs;
    
    /** GPS measurement (system input) update period in milliseconds set on startup. 200ms minimum (5Hz max). */
    uint32_t                startupGPSDtMs;
    
    /** RTK configuration bits (see eRTKConfigBits). */
    uint32_t                RTKCfgBits;

    /** Sensor config to specify the full-scale sensing ranges and output rotation for the IMU and magnetometer (see eSensorConfig) */
    uint32_t                sensorConfig;

    /** Minimum elevation of a satellite above the horizon to be used in the solution (radians). Low elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere. */
    float                   gpsMinimumElevation;

    /** Serial port 2 baud rate in bits per second */
    uint32_t                ser2BaudRate;

    /** Wheel encoder: euler angles describing the rotation from imu to left wheel */
    wheel_config_t          wheelConfig;

    /** Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection. */
    float                   magInterferenceThreshold;

    /** Magnetometer calibration quality sensitivity threshold. Typical range is 10-20 (10 default) and 1000 to disable mag calibration quality check, forcing it to be always good. */
    float                   magCalibrationQualityThreshold;

    /** (dBHz) GNSS CN0 absolute minimum threshold for signals.  Used to filter signals in RTK solution. */
    uint8_t                 gnssCn0Minimum;

    /** (dBHz) GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum. */
    uint8_t                 gnssCn0DynMinOffset;

    /** IMU gyro fault rejection threshold low */
    uint8_t                 imuRejectThreshGyroLow;

    /** IMU gyro fault rejection threshold high */
    uint8_t                 imuRejectThreshGyroHigh;

    /** (ms) IMU shock detection latency.  Time used for EKF rewind to prevent shock from influencing EKF estimates.  */
    uint8_t                 imuShockDetectLatencyMs;

    /** (ms) IMU shock rejection latch time.  Time required following detected shock end to disable shock rejection.  */
    uint8_t                 imuShockRejectLatchMs;

    /* IMU shock rejection options (see eImuShockOptions) */
    uint8_t                 imuShockOptions;

    /* (m/s^2) IMU shock detection. Min acceleration difference between the 3 IMUs to detect the start of a shock. */
    uint8_t                 imuShockDeltaAccHighThreshold;

    /* (m/s^2) IMU shock detection. Max acceleration difference between the 3 IMUs within the latch time to detect the end of a shock. */
    uint8_t                 imuShockDeltaAccLowThreshold;

    /* (deg/s) IMU shock detection. Min angular rate difference between the 3 IMUs to detect the start of a shock. */
    uint8_t                 imuShockDeltaGyroHighThreshold;

    /* (deg/s) IMU shock detection. Max angular rate difference between the 3 IMUs within the latch time to detect the end of a shock. */
    uint8_t                 imuShockDeltaGyroLowThreshold;

    /** Hardware interface configuration bits for GNSS2 PPS (see eIoConfig2). */
    uint8_t                 ioConfig2;

} nvm_flash_cfg_t;

/** (DID_INL2_NED_SIGMA) Standard deviation of INL2 EKF estimates in the NED frame. */
typedef struct PACKED
{                                            
    /** Timestamp in milliseconds */
    unsigned int            timeOfWeekMs;    
    /** NED position error sigma */
    float                   StdPosNed[3];        
    /** NED velocity error sigma */
    float                   StdVelNed[3];        
    /** NED attitude error sigma */
    float                   StdAttNed[3];        
    /** Acceleration bias error sigma */
    float                   StdAccBias[3];        
    /** Angular rate bias error sigma */
    float                   StdGyrBias[3];        
    /** Barometric altitude bias error sigma */
    float                   StdBarBias;        
    /** Mag declination error sigma */
    float                   StdMagDeclination;    
} inl2_ned_sigma_t;

/** (DID_STROBE_IN_TIME) Timestamp for input strobe. */
typedef struct PACKED
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t                week;

    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Strobe input pin (i.e. G1, G2, G5, G9, G11, G12, G13, G15) */
    uint16_t                pin;

    /** Strobe serial index number */
    uint16_t                count;
} strobe_in_time_t;

#define DEBUG_I_ARRAY_SIZE      9
#define DEBUG_F_ARRAY_SIZE      9
#define DEBUG_LF_ARRAY_SIZE     3

/* (DID_DEBUG_ARRAY) */
typedef struct PACKED
{
    int32_t                 i[DEBUG_I_ARRAY_SIZE];
    float                   f[DEBUG_F_ARRAY_SIZE];
    double                  lf[DEBUG_LF_ARRAY_SIZE];
} debug_array_t;

#define DEBUG_STRING_SIZE        80

/* (DID_DEBUG_STRING) */
typedef struct PACKED
{
    uint8_t     s[DEBUG_STRING_SIZE];
} debug_string_t;

POP_PACK

PUSH_PACK_8

typedef struct PACKED
{
    gtime_t time;
    double rp_ecef[3]; // Rover position
    double rv_ecef[3]; // Rover velocity
    double ra_ecef[3]; // Rover acceleration
    double bp_ecef[3]; // Base position
    double bv_ecef[3]; // Base velocity
    double qr[6]; // rover position and velocity covariance main diagonal
    double b[24]; // satellite bias
    double qb[24]; // main diagonal of sat bias covariances
    uint8_t sat_id[24]; // satellite id of b[]
} rtk_state_t;

typedef struct PACKED
{
    gtime_t time;
    int32_t nv; // number of measurements
    uint8_t sat_id_i[24]; // sat id of measurements (reference sat)
    uint8_t sat_id_j[24]; // sat id of measurements
    uint8_t type[24]; // type (0 = dd-range, 1 = dd-phase, 2 = baseline)
    double v[24]; // residual
} rtk_residual_t;

typedef struct PACKED
{
    gtime_t time;

    uint8_t rtkd_unused8_1;
    uint8_t code_outlier;                    //!< Code residual in float solution too large
    uint8_t phase_outlier;                   //!< Phase residual in float solution too large
    uint8_t rtkd_unused8_2;

    uint8_t rtkd_unused8_3;
    uint8_t rtkd_unused8_4;
    uint8_t bad_baseline_holdamb;            //!< Bad baseline during hold ambiguity (may not be needed, consider removing)
    uint8_t rtkd_unused8_5;

    uint8_t outc_ovfl;                       //!< Observation/reject outage counter
    uint8_t rtkd_unused8_6;
    uint8_t rtkd_unused8_7;
    uint8_t large_v2b;                       //!< Vector to base distance too large

    uint8_t base_position_update;            //!< Received position of base correction counter
    uint8_t rover_position_error;
    uint8_t reset_bias;                      //!< Satellite bias reset counter
    uint8_t rtkd_unused8_8;

    float   pos_variance;                    //!< position variance

    uint8_t diff_age_error;                  //!< Difference age too large
    uint8_t rtkd_unused8_9;
    uint8_t rover_packet_age_ms;             //!< Age of last received rover packet  (TODO) convert to int16_t
    uint8_t base_packet_age_ms;              //!< Age of last received base packet  (TODO) convert to int16_t

    uint32_t rtkd_unused32_1;

    uint32_t cycle_slips;                    //!< Accumulation of total cycle slips

    float rtk_to_rcvr_pos_error;             //!< RTK position Error with respect to GNSS receiver

    uint8_t rtkd_unused8_10;
    uint8_t rtkd_unused8_11;
    uint8_t error_count;                     //!< Pre-filtered observations error count
    uint8_t error_code;                      //!< Pre-filtered observations error code

    uint32_t rtkd_unused32_2;

    uint8_t rtkd_unused8_12;
    uint8_t rtkd_unused8_13;
    uint8_t warning_count;                   //!< Pre-filtered observations warning count
    uint8_t warning_code;                    //!< Pre-filtered observations warning code

    double double_debug[4];

    uint8_t debug[2];
    uint8_t obs_base_unfiltered;             //!< Number of base observations from the receiver (before filtering)
    uint8_t obs_rover_unfiltered;            //!< Number of rovr observations from the receiver (before filtering)

    uint8_t rtkd_unused8_14;
    uint8_t rtkd_unused8_15;
    uint8_t rtkd_unused8_16;
    uint8_t obs_unhealthy;                   //!< number of sats marked as "unhealthy" by GNSS receiver (nonzero terms in svh)

    uint8_t obs_rover_relpos;                //!< nu - number of observations input to relpos() before selsat(), rover
    uint8_t obs_base_relpos;                 //!< nr - number of observations input to relpos() before selsat(), base
    uint8_t obs_pairs_used_float;            //!< number of sat pairs used to compute the float solution
    uint8_t obs_pairs_used_fixed;            //!< number of sat pairs used to compute the fixed solution

    uint8_t obs_eph_relpos;                  //!< number of sats with ephemeris available (min is 0, max is nu)
    uint8_t obs_low_snr_rover;               //!< number of sats with low snr at rover and exclude from solution
    uint8_t obs_low_snr_base;                //!< number of sats with low snr at base and exclude from solution
    uint8_t rtkd_unused8_17;

    uint8_t obs_zero_L1_rover;               //!< number of sats with zero L1 pseudorange or phase at rover
    uint8_t obs_zero_L1_base;                //!< number of sats with zero L1 pseudorange or phase at base
    uint8_t obs_low_elev;                    //!< number of sats with low elevation
    uint8_t rtkd_unused8_18;

    uint8_t rtkd_unused8_19;
    uint8_t rtkd_unused8_20;
    uint8_t reserved[2];
} rtk_debug_t;

POP_PACK

PUSH_PACK_1

/** (DID_GPS_RTK_OPT) RTK processing options */
typedef struct
{
    /** positioning mode (PMODE_???) */
    int32_t mode;           

    /** solution type (0:forward,1:backward,2:combined) */
    int32_t soltype;

    /** number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    int32_t nf;

    /** navigation systems */
    int32_t navsys;

    /** elevation mask angle (rad) */
    float elmin;

    /** Min snr to consider satellite for rtk */
    int32_t snrmin;
    int32_t snrrange; // snr range from the highest snr satellite to consider (overrides snrmin if non-zero)

    /** AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int32_t modear;

    /** GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    int32_t glomodear;

    /** GPS AR mode (0:off,1:on) */
    int32_t gpsmodear;

    /** SBAS AR mode (0:off,1:on) */
    int32_t sbsmodear;

    /** BeiDou AR mode (0:off,1:on) */
    int32_t bdsmodear;

    /** AR filtering to reject bad sats (0:off,1:on) */
    int32_t arfilter;

    /** obs outage count to reset bias */
    int32_t maxout;

    /** reject count to reset bias */
    int32_t maxrej;

    /** min lock count to fix ambiguity */
    int32_t minlock;

    /** min sats to fix integer ambiguities */
    int32_t minfixsats;

    /** min sats to hold integer ambiguities */
    int32_t minholdsats;

    /** min sats to drop sats in AR */
    int32_t mindropsats;

    /** use stdev estimates from receiver to adjust measurement variances */
    int32_t rcvstds;

    /** min fix count to hold ambiguity */
    int32_t minfix;

    /** max iteration to resolve ambiguity */
    int32_t armaxiter;

    /** dynamics model (0:none,1:velociy,2:accel) */
    int32_t dynamics;

    /** interpolate reference obs (for post mission) */
    int32_t intpref;

    /** rover position for fixed mode */
    int32_t rovpos;

    /** base position for relative mode */
    int32_t refpos;

    /** code/phase error ratio */
    float eratio[NFREQ];

    /** measurement error factor */
    float err[7];

    /** initial-state std [0]bias,[1]iono [2]trop */
    float std[3];

    /** process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
    float prn[6];

    /** satellite clock stability (sec/sec) */
    double sclkstab;

    /** AR validation threshold */
    float thresar[8];

    /** elevation mask of AR for rising satellite (rad) */
    float elmaskar;

    /** elevation mask to hold ambiguity (rad) */
    float elmaskhold;

    /** slip threshold of geometry-free phase (m) */
    float thresslip;

    /* slip threshold of doppler (m) */
    float thresdop;

    /** variance for fix-and-hold pseudo measurements (cycle^2) */
    float varholdamb;

    /** gain used for GLO and SBAS sats to adjust ambiguity */
    float gainholdamb;

    /** max difference of time (sec) */
    float maxtdiff;

    /** reset sat biases after this long trying to get fix if not acquired */
    int fix_reset_base_msgs;

    /** reject threshold of innovation for phase [0] and code [1] (m) */
    float maxinno[2];
    /** reject thresholds of NIS for phase [0] and code [1] */
    float maxnis_lo[2];
    float maxnis_hi[2];

    /** reject threshold of gdop */
    double maxgdop;

    /** baseline length constraint {const,sigma before fix, sigma after fix} (m) */
    float baseline[3];
    float max_baseline_error;
    float reset_baseline_error;

    /** maximum error wrt ubx position (triggers reset if more than this far) (m) */
    float max_ubx_error;

    /** rover position for fixed mode {x,y,z} (ecef) (m) */
    double ru[3];

    /** base position for relative mode {x,y,z} (ecef) (m) */
    double rb[3];

    /** max averaging epochs */
    int32_t maxaveep;

    /** output single by dgps/float/fix/ppp outage */
    int32_t outsingle;

    /** velocity constraint in compassing mode {var before fix, var after fix} (m^2/s^2) **/
    float velcon[2];
} prcopt_t;
typedef prcopt_t gps_rtk_opt_t;

/** Raw satellite observation data */
typedef struct PACKED
{
    /** Receiver local time approximately aligned to the GPS time system (GPST) */
    gtime_t time;

    /** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
    uint8_t sat;

    /** receiver number */
    uint8_t rcv;

    /** Cno, carrier-to-noise density ratio (signal strength) (0.25 dB-Hz) */
    uint8_t SNR[NFREQ+NEXOBS];

    /** Loss of Lock Indicator. Set to non-zero values only when carrier-phase is valid (L > 0).  bit1 = loss-of-lock, bit2 = half-cycle-invalid */
    uint8_t LLI[NFREQ+NEXOBS];

    /** Code indicator: CODE_L1C (1) = L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS), CODE_L1X (12) = E1B+C,L1C(D+P) (GAL,QZS), CODE_L1I (47) = B1I (BeiDou) */
    uint8_t code[NFREQ+NEXOBS];

    /** Estimated carrier phase measurement standard deviation (0.004 cycles), zero means invalid */
    uint8_t qualL[NFREQ+NEXOBS];

    /** Estimated pseudorange measurement standard deviation (0.01 m), zero means invalid */
    uint8_t qualP[NFREQ+NEXOBS];

    /** reserved, for alignment */
    uint8_t reserved;

    /** Observation data carrier-phase (cycle). The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX specification. */
    double L[NFREQ+NEXOBS];

    /** Observation data pseudorange (m). GLONASS inter frequency channel delays are compensated with an internal calibration table */
    double P[NFREQ+NEXOBS]; 

    /** Observation data Doppler measurement (positive sign for approaching satellites) (Hz) */
    float D[NFREQ+NEXOBS];
} obsd_t;

#define GPS_RAW_MESSAGE_BUF_SIZE    1000
#define MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE (GPS_RAW_MESSAGE_BUF_SIZE / sizeof(obsd_t))

/** observation data */
typedef struct
{
    /** number of observation slots used */
    uint32_t n;

    /** number of observation slots allocated */
    uint32_t nmax;

    /** observation data buffer */
    obsd_t* data;
} obs_t;

/** non-Glonass ephemeris data */
typedef struct
{
    /** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
    int32_t sat;

    /** IODE Issue of Data, Ephemeris (ephemeris version) */
    int32_t iode;
    
    /** IODC Issue of Data, Clock (clock version) */
    int32_t iodc;

    /** SV accuracy (URA index) IRN-IS-200H p.97 */
    int32_t sva;            

    /** SV health GPS/QZS (0:ok) */
    int32_t svh;            

    /** GPS/QZS: gps week, GAL: galileo week */
    int32_t week;

    /** GPS/QZS: code on L2. (00 = Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid).  GAL/CMP: data sources */
    int32_t code;

    /** GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel). CMP: nav type */
    int32_t flag;

    /** Time Of Ephemeris, ephemeris reference epoch in seconds within the week (s) */
    gtime_t toe;
    
    /** clock data reference time (s) (20.3.4.5) */
    gtime_t toc;
    
    /** T_trans (s) */
    gtime_t ttr;

    /** Orbit semi-major axis (m) */
    double A;

    /** Orbit eccentricity (non-dimensional)  */
    double e;

    /** Orbit inclination angle at reference time (rad) */
    double i0;

    /** Longitude of ascending node of orbit plane at weekly epoch (rad) */
    double OMG0;

    /** Argument of perigee (rad) */
    double omg;

    /** Mean anomaly at reference time (rad) */
    double M0;

    /** Mean Motion Difference From Computed Value (rad) */
    double deln;

    /** Rate of Right Ascension (rad/s) */
    double OMGd;

    /** Rate of Inclination Angle (rad/s) */
    double idot;

    /** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius (m) */
    double crc;

    /** Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m) */
    double crs;

    /** Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad)  */
    double cuc;

    /** Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad) */
    double cus;

    /** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad) */
    double cic;

    /** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad) */
    double cis;

    /** Time Of Ephemeris, ephemeris reference epoch in seconds within the week (s), same as <toe> above but represented as double type. Note that toe is computed as eph->toe = gst2time(week, eph->toes). This is the expiration time and is generally ~2 hours ahead of current time. */
    double toes;

    /** Fit interval (h) (0: 4 hours, 1: greater than 4 hours) */
    double fit;

    /** SV clock offset, af0 (s) */
    double f0;
    
    /** SV clock drift, af1 (s/s, non-dimensional) */
    double f1;
    
    /** SV clock drift rate, af2 (1/s) */
    double f2;

    /** Group delay parameters GPS/QZS: tgd[0] = TGD (IRN-IS-200H p.103). Galilleo: tgd[0] = BGD E5a/E1, tgd[1] = BGD E5b/E1. Beidou: tgd[0] = BGD1, tgd[1] = BGD2 */
    double tgd[4];

    /** Adot for CNAV, not used */
    double Adot;
    
    /** First derivative of mean motion n (second derivative of mean anomaly M), ndot for CNAV (rad/s/s). Not used. */
    double ndot;
} eph_t;

/** Glonass ephemeris data */
typedef struct
{        
    /** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
    int32_t sat;

    /** IODE (0-6 bit of tb field) */
    int32_t iode;

    /** satellite frequency number */
    int32_t frq;

    /** satellite health */
    int32_t svh;
    
    /** satellite accuracy */
    int32_t sva;
    
    /** satellite age of operation */
    int32_t age;

    /** Ephemeris reference epoch in seconds within the week in GPS time gpst (s) */
    gtime_t toe;

    /** message frame time in gpst (s) */
    gtime_t tof;

    /** satellite position (ecef) (m) */
    double pos[3];

    /** satellite velocity (ecef) (m/s) */
    double vel[3];

    /** satellite acceleration (ecef) (m/s^2) */
    double acc[3];

    /** SV clock bias (s) */
    double taun;

    /** relative frequency bias */
    double gamn;

    /** delay between L1 and L2 (s) */
    double dtaun;
} geph_t;

/** SBAS message type */
typedef struct
{
    /** receiption time - week */
    int32_t week;
    
    /** reception time - tow */
    int32_t tow;

    /** SBAS satellite PRN number */
    int32_t prn;

    /** SBAS message (226bit) padded by 0 */
    uint8_t msg[29];

    /** reserved for alighment */
    uint8_t reserved[3];
} sbsmsg_t;

/** station parameter type */
typedef struct
{
    /** antenna delta type (0:enu,1:xyz) */
    int32_t deltype;
    
    /** station position (ecef) (m) */
    double pos[3];

    /** antenna position delta (e/n/u or x/y/z) (m) */
    double del[3];

    /** antenna height (m) */
    double hgt;
    
    /** station id */
    int32_t stationId;
} sta_t;

/** almanac type */
typedef struct
{
    /** satellite number */
    int32_t sat;

    /** sv health (0:ok) */
    int32_t svh;

    /** as and sv config */
    int32_t svconf;

    /* GPS/QZS: gps week, GAL: galileo week */
    int32_t week;

    /* Toa */
    gtime_t toa;        
                        
    /** SV orbit parameters - A */
    double A;

    /** SV orbit parameters - e */
    double e;

    /** SV orbit parameters - i0 */
    double i0;

    /** SV orbit parameters - OMG0 */
    double OMG0;
    
    /** SV orbit parameters - omg */
    double omg;
    
    /** SV orbit parameters - M0 */
    double M0;
    
    /** SV orbit parameters - OMGd */
    double OMGd;

    /** Toa (s) in week - toas */
    double toas;

    /** SV clock parameters - af0 */
    double f0;
    
    /** SV clock parameters - af1 */
    double f1;
} alm_t;

/** ionosphere model and utc parameters */
typedef struct
{
    double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
    double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */

    double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
    double utc_glo[4];  /* GLONASS UTC GPS time parameters */
    double utc_gal[4];  /* Galileo UTC GPS time parameters */
    double utc_qzs[4];  /* QZS UTC GPS time parameters */
    double utc_cmp[4];  /* BeiDou UTC parameters */
    double utc_irn[4];  /* IRNSS UTC parameters */
    double utc_sbs[4];  /* SBAS UTC parameters */

    int32_t leaps;      /* leap seconds (s) */
    
    alm_t alm;          /* almanac */
} ion_model_utc_alm_t;

/** RTK solution status */
typedef enum
{
    /** No status */
    rtk_solution_status_none = 0,

    /** RTK fix */
    rtk_solution_status_fix = 1,

    /** RTK float */
    rtk_solution_status_float = 2,

    /** RTK SBAS */
    rtk_solution_status_sbas = 3,

    /** RTK DGPS */
    rtk_solution_status_dgps = 4,

    /** RTK SINGLE */
    rtk_solution_status_single = 5
} eRtkSolStatus;

/** (DID_GPS1_RTK_POS_REL, DID_GPS2_RTK_CMP_REL) - RTK and Dual GNSS heading base to rover relative info. */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Age of differential (seconds) */
    float                   differentialAge;

    /** Ambiguity resolution ratio factor for validation */
    float                   arRatio;

    /** Vector from base to rover (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1 */
    float                   baseToRoverVector[3];

    /** Distance from base to rover (m) */
    float                   baseToRoverDistance;
    
    /** Angle from north to baseToRoverVector in local tangent plane. (rad) */
    float                   baseToRoverHeading;

    /** Accuracy of baseToRoverHeading. (rad) */
    float                   baseToRoverHeadingAcc;

    /** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
    uint32_t                status;
    
} gps_rtk_rel_t;

/** (DID_GPS1_RTK_POS_MISC, DID_GPS2_RTK_CMP_MISC) - requires little endian CPU */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options. []: standard deviations {ECEF - x,y,z} or {north, east, down} (meters) */
    float                   accuracyPos[3];

    /** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options. []: Absolute value of means square root of estimated covariance NE, EU, UN */
    float                   accuracyCov[3];

    /** Ambiguity resolution threshold for validation */
    float                   arThreshold;

    /** Geometric dilution of precision (meters) */
    float                   gDop;
    
    /** Horizontal dilution of precision (meters) */
    float                   hDop;
    
    /** Vertical dilution of precision (meters) */
    float                   vDop;

    /** Base Position - latitude, longitude, height (degrees, meters) */
    double                  baseLla[3];

    /** Cycle slip counter */
    uint32_t                cycleSlipCount;
    


    /** Rover gps observation element counter */
    uint32_t                roverGpsObservationCount;

    /** Base station gps observation element counter */
    uint32_t                baseGpsObservationCount;

    /** Rover glonass observation element counter */
    uint32_t                roverGlonassObservationCount;

    /** Base station glonass observation element counter */
    uint32_t                baseGlonassObservationCount;


    /** Rover galileo observation element counter */
    uint32_t                roverGalileoObservationCount;

    /** Base station galileo observation element counter */
    uint32_t                baseGalileoObservationCount;

    /** Rover beidou observation element counter */
    uint32_t                roverBeidouObservationCount;

    /** Base station beidou observation element counter */
    uint32_t                baseBeidouObservationCount;


    /** Rover qzs observation element counter */
    uint32_t                roverQzsObservationCount;

    /** Base station qzs observation element counter */
    uint32_t                baseQzsObservationCount;

    /** Rover gps ephemeris element counter */
    uint32_t                roverGpsEphemerisCount;

    /** Base station gps ephemeris element counter */
    uint32_t                baseGpsEphemerisCount;


    /** Rover glonass ephemeris element counter */
    uint32_t                roverGlonassEphemerisCount;

    /** Base station glonass ephemeris element counter */
    uint32_t                baseGlonassEphemerisCount;
    
    /** Rover galileo ephemeris element counter */
    uint32_t                roverGalileoEphemerisCount;

    /** Base station galileo ephemeris element counter */
    uint32_t                baseGalileoEphemerisCount;


    /** Rover beidou ephemeris element counter */
    uint32_t                roverBeidouEphemerisCount;

    /** Base station beidou ephemeris element counter */
    uint32_t                baseBeidouEphemerisCount;

    /** Rover qzs ephemeris element counter */
    uint32_t                roverQzsEphemerisCount;

    /** Base station qzs ephemeris element counter */
    uint32_t                baseQzsEphemerisCount;


    /** Rover sbas element counter */
    uint32_t                roverSbasCount;

    /** Base station sbas element counter */
    uint32_t                baseSbasCount;

    /** Base station antenna position element counter */
    uint32_t                baseAntennaCount;

    /** Ionosphere model, utc and almanac count */
    uint32_t                ionUtcAlmCount;
    
    
    /** Number of checksum failures from received corrections */
    uint32_t                correctionChecksumFailures;

    /** Time to first RTK fix. */
    uint32_t                timeToFirstFixMs;
    
} gps_rtk_misc_t;

/** RAW data types for DID_GPS_BASE_RAW and DID_GPS2_RAW */
typedef enum
{
    /** obsd_t */
    raw_data_type_observation = 1,

    /** eph_t */
    raw_data_type_ephemeris = 2,

    /** geph_t */
    raw_data_type_glonass_ephemeris = 3,

    /** sbsmsg_t */
    raw_data_type_sbas = 4,

    /** sta_t */
    raw_data_type_base_station_antenna_position = 5,

    /** ion_model_utc_alm_t */
    raw_data_type_ionosphere_model_utc_alm = 6,
    
    /** gps_rtk_misc_t */
    raw_data_type_rtk_solution = 123
} eRawDataType;



typedef union PACKED
{   
    /** Satellite observation data */
    obsd_t              obs[MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE];
    
    /** Satellite non-GLONASS ephemeris data (GPS, Galileo, Beidou, QZSS) */
    eph_t               eph;
    
    /** Satellite GLONASS ephemeris data */
    geph_t              gloEph;
    
    /** Satellite-Based Augmentation Systems (SBAS) data */
    sbsmsg_t            sbas;
        
    /** Base station information (base position, antenna position, antenna height, etc.) */
    sta_t               sta;

    /** Ionosphere model and UTC parameters */
    ion_model_utc_alm_t ion;

    /** Byte buffer */
    uint8_t             buf[GPS_RAW_MESSAGE_BUF_SIZE];

} uGpsRawData;

/** Message wrapper for DID_GPS1_RAW, DID_GPS2_RAW, and DID_GPS_BASE_RAW.  The contents of data can vary for this message and are determined by `dataType` field. */
typedef struct PACKED
{
    /** Receiver index (1=RECEIVER_INDEX_GPS1, 2=RECEIVER_INDEX_EXTERNAL_BASE, or 3=RECEIVER_INDEX_GPS2) */
    uint8_t receiverIndex;

    /** Type of data (eRawDataType: 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel) */
    uint8_t dataType;

    /** Number of observations in data (obsd_t) when dataType==1 (raw_data_type_observation). */
    uint8_t obsCount;

    /** Reserved */
    uint8_t reserved;

    /** Interpret based on dataType (see eRawDataType) */    
    uGpsRawData data;
} gps_raw_t;

// (DID_GPS1_TIMEPULSE)
typedef struct
{
    /*! (s)    Week seconds offset from MCU to GPS time. */
    double      towOffset;

    /*! (s)    Week seconds for next timepulse (from start of GPS week) */
    double      towGps;

    /*! (s)    Local MCU week seconds */
    double      timeMcu;

    /*! (ms) Local timestamp of TIM-TP message used to validate timepulse. */
    uint32_t    msgTimeMs;

    /*! (ms) Local timestamp of time sync pulse external interrupt used to validate timepulse. */
    uint32_t    plsTimeMs;

    /*! Counter for successful timesync events. */
    uint8_t     syncCount;

    /*! Counter for failed timesync events. */
    uint8_t     badPulseAgeCount;

    /*! Counter for GPS PPS interrupt re-initalization. */
    uint8_t     ppsInterruptReinitCount;

    /*! Counter of GPS PPS via GPIO, not interrupt. */
    uint8_t     plsCount;

    /*! (ms) Local timestamp of last valid PPS sync. */
    uint32_t    lastSyncTimeMs;

    /*! (ms) Time since last valid PPS sync. */
    uint32_t    sinceLastSyncTimeMs;

} gps_timepulse_t;

/**
* Diagnostic message
*/
typedef struct 
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t timeOfWeekMs;
    
    /** Message length, including null terminator */
    uint32_t messageLength;
    
    /** Message data, max size of message is 256 */
    char message[256];
} diag_msg_t;

typedef enum
{
    // default state
    SURVEY_IN_STATE_OFF                     = 0,

    // commands
    SURVEY_IN_STATE_CANCEL                  = 1,
    SURVEY_IN_STATE_START_3D                = 2,
    SURVEY_IN_STATE_START_FLOAT             = 3,
    SURVEY_IN_STATE_START_FIX               = 4,

    // status
    SURVEY_IN_STATE_RUNNING_3D              = 8,
    SURVEY_IN_STATE_RUNNING_FLOAT           = 9,
    SURVEY_IN_STATE_RUNNING_FIX             = 10,
    SURVEY_IN_STATE_SAVE_POS                = 19,
    SURVEY_IN_STATE_DONE                    = 20
} eSurveyInStatus;

/**
* Survey in status
*/
typedef struct
{
    /** State of current survey, eSurveyInStatus */
    uint32_t state;

    /** Maximum time (milliseconds) survey will run if minAccuracy is not first achieved. (ignored if 0). */
    uint32_t maxDurationSec;

    /** Required horizontal accuracy (m) for survey to complete before maxDuration. (ignored if 0) */
    float minAccuracy;

    /** Elapsed time (seconds) of the survey. */
    uint32_t elapsedTimeSec;

    /** Approximate horizontal accuracy of the survey (m). */
    float hAccuracy;

    /** The current surveyed latitude, longitude, altitude (deg, deg, m) */
    double lla[3];
} survey_in_t;


//////////////////////////////////////////////////////////////////////////
//  GPX
//////////////////////////////////////////////////////////////////////////

/** GPX System Configuration (used with DID_GPX_FLASH_CFG.sysCfgBits) */
enum eGpxSysConfigBits
{
    /** Disable (tri-state) VCC_RF (GPX pin 16) output supplied via VAUX (GPX pin 40). */
    GPX_SYS_CFG_BITS_DISABLE_VCC_RF                         = 0x00000001,

    /** Brownout reset threshold voltage level */
    GPX_SYS_CFG_BITS_BOR_LEVEL_0                            = 0x0,              // 1.65 - 1.75 V  (default)
    GPX_SYS_CFG_BITS_BOR_LEVEL_1                            = 0x1,              // 2.0  - 2.1  V
    GPX_SYS_CFG_BITS_BOR_LEVEL_2                            = 0x2,              // 2.25 - 2.35 V
    GPX_SYS_CFG_BITS_BOR_LEVEL_3                            = 0x3,              // 2.5  - 2.6  V
    GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK                     = (int)0x00C00000,
    GPX_SYS_CFG_BITS_BOR_THRESHOLD_OFFSET                   = 22,
};

/**
 * (DID_GPX_FLASH_CFG) GPX flash config.
 * IMPORTANT: These fields should not be deleted, they can be deprecated and marked as reserved,
 * or new fields added to the end.  
 * NOTE: The key value must be incremented to ensure the defaults are restored anytime the fields 
 * change or the default values change.  Default changes should be noted in the changelog.
*/
typedef struct
{  
    /** Size of this struct */
    uint32_t                size;

    /** Checksum, excluding size and checksum.  0xFFFFFFFF is invalid. */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /** Serial port 0 baud rate in bits per second */
    uint32_t                ser0BaudRate;

    /** Serial port 1 baud rate in bits per second */
    uint32_t                ser1BaudRate;

    /** Serial port 2 baud rate in bits per second */
    uint32_t                ser2BaudRate;

    /** GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max). */
    uint32_t                startupGPSDtMs;

    /** X,Y,Z offset in meters in Sensor Frame to GPS 1 antenna. */
    float                   gps1AntOffset[3];

    /** X,Y,Z offset in meters in Sensor Frame to GPS 2 antenna. */
    float                   gps2AntOffset[3];
 
    /** Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS */
    uint16_t                gnssSatSigConst;

    /** Dynamic platform model (see eDynamicModel).  Options are: 0=PORTABLE, 1=FIXED POSITION, 2=STATIONARY, 3=PEDESTRIAN, 4=GROUND VEHICLE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST.  Used to balance noise and performance characteristics of the system.  The dynamics selected here must be at least as fast as your system or you experience accuracy error.  This is tied to the GPS position estimation model and intend in the future to be incorporated into the INS position model. */
    uint8_t                 dynamicModel;

    /** Debug */
    uint8_t                 debug;

    /** Time between GPS time synchronization pulses in milliseconds.  Requires reboot to take effect. */
    uint32_t                gpsTimeSyncPeriodMs;

    /** (sec) User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.  */
    float                   gpsTimeUserDelay;

    /** Minimum elevation of a satellite above the horizon to be used in the solution (radians). Low elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere. */
    float                   gpsMinimumElevation;

    /** RTK configuration bits (see eRTKConfigBits). */
    uint32_t                RTKCfgBits;

    /** (dBHz) GNSS CN0 absolute minimum threshold for signals.  Used to filter signals in RTK solution. */
    uint8_t                 gnssCn0Minimum;

    /** (dBHz) GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum. */
    uint8_t                 gnssCn0DynMinOffset;

    /** Reserved */
    uint8_t                 reserved1[2];

    /** System configuration bits (see eGpxSysConfigBits). */
    uint32_t                sysCfgBits;

    /** Reserved */
    uint32_t                reserved2;
    
    /** Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations (deg, deg, m) */
    double                  refLla[3];

} gpx_flash_cfg_t;

/** GPX status flags */
enum eGpxStatus
{
    /** Communications parse error count */
    GPX_STATUS_COM_PARSE_ERR_COUNT_MASK                 = (int)0x0000000F,
    GPX_STATUS_COM_PARSE_ERR_COUNT_OFFSET               = 0,
#define GPX_STATUS_COM_PARSE_ERROR_COUNT(gpxStatus) ((gpxStatus&GPX_STATUS_COM_PARSE_ERR_COUNT_MASK)>>GPX_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

    /** Rx communications not dectected in last 30 seconds */
    GPX_STATUS_COM0_RX_TRAFFIC_NOT_DECTECTED            = (int)0x00000010,
    GPX_STATUS_COM1_RX_TRAFFIC_NOT_DECTECTED            = (int)0x00000020,
    GPX_STATUS_COM2_RX_TRAFFIC_NOT_DECTECTED            = (int)0x00000040,
    GPX_STATUS_USB_RX_TRAFFIC_NOT_DECTECTED             = (int)0x00000080,

    /** General Fault mask */
    GPX_STATUS_GENERAL_FAULT_MASK                       = (int)0xFFFF0000,

    /** RTK buffer filled causing data loss */
    GPX_STATUS_FAULT_RTK_QUEUE_LIMITED                  = (int)0x00010000,

    /** GNSS receiver time fault **/
    GPX_STATUS_FAULT_GNSS_RCVR_TIME                     = (int)0x00100000,
    /** DMA Fault detected **/
    GPX_STATUS_FAULT_DMA                                = (int)0x00800000,

    /** Fatal faults - critical failure resulting in CPU reset */
    GPX_STATUS_FATAL_MASK                               = (int)0x1F000000,
    GPX_STATUS_FATAL_OFFSET                             = 24,
    GPX_STATUS_FATAL_RESET_LOW_POW                      = (int)1,       // reset from low power
    GPX_STATUS_FATAL_RESET_BROWN                        = (int)2,       // reset from brown out
    GPX_STATUS_FATAL_RESET_WATCHDOG                     = (int)3,       // reset from watchdog
    GPX_STATUS_FATAL_CPU_EXCEPTION                      = (int)4,                     
    GPX_STATUS_FATAL_UNHANDLED_INTERRUPT                = (int)5,
    GPX_STATUS_FATAL_STACK_OVERFLOW                     = (int)6,
    GPX_STATUS_FATAL_KERNEL_OOPS                        = (int)7,
    GPX_STATUS_FATAL_KERNEL_PANIC                       = (int)8,
    GPX_STATUS_FATAL_UNALIGNED_ACCESS                   = (int)9,
    GPX_STATUS_FATAL_MEMORY_ERROR                       = (int)10,
    GPX_STATUS_FATAL_BUS_ERROR                          = (int)11,      // bad pointer or malloc
    GPX_STATUS_FATAL_USAGE_ERROR                        = (int)12,
    GPX_STATUS_FATAL_DIV_ZERO                           = (int)13,
    GPX_STATUS_FATAL_SER0_REINIT                        = (int)14,
    GPX_STATUS_FATAL_UNKNOWN                            = (int)0x1F,    // TODO: Temporarily set to (5 bits). Reset to 0xFF when gpx_flash_cfg.debug is no longer used with fault reporting. (WHJ) 

    /** Internal use */
    GPX_STATUS_FAULT_RP                                 = (int)0x20000000,
    /** Reserved for future fault status */
    GPX_STATUS_FAULT_UNUSED                             = (int)0xC0000000,
};

/** Hardware status flags */
enum eGPXHdwStatusFlags
{
    /** GNSS1 satellite signals are being received (antenna and cable are good) */
    GPX_HDW_STATUS_GNSS1_SATELLITE_RX                   = (int)0x00000001,
    /** GNSS2 satellite signals are being received (antenna and cable are good) */
    GPX_HDW_STATUS_GNSS2_SATELLITE_RX                   = (int)0x00000002,
    /** GPS time of week is valid and reported.  Otherwise the timeOfWeek is local system time. */
    GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID             = (int)0x00000004,
    /** GPS time of week is valid and reported.  Otherwise the timeOfWeek is local system time. */
    GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID             = (int)0x00000008,
    
    /** GNSS 1 reset required count */
    GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK               = (int)0x00000070,
    GPX_HDW_STATUS_GNSS1_RESET_COUNT_OFFSET             = 4,
#define GPX_HDW_STATUS_GNSS1_RESET_COUNT(hdwStatus)     ((hdwStatus&GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK)>>GPX_HDW_STATUS_GNSS1_RESET_COUNT_OFFSET)
 
    /** Failed to communicate or setup GNSS receiver 1 */
    GPX_HDW_STATUS_FAULT_GNSS1_INIT                     = (int)0x00000080,
    GPX_HDW_STATUS_GNSS1_FAULT_FLAG_OFFSET              = 7,

    /** GNSS 2 reset required count */
    GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK               = (int)0x00000700,
    GPX_HDW_STATUS_GNSS2_RESET_COUNT_OFFSET             = 8,
#define GPX_HDW_STATUS_GNSS2_RESET_COUNT(hdwStatus)     ((hdwStatus&GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK)>>GPX_HDW_STATUS_GNSS2_RESET_COUNT_OFFSET)

    /** Failed to communicate or setup GNSS receiver 2 */
    GPX_HDW_STATUS_FAULT_GNSS2_INIT                     = (int)0x00000800,
    GPX_HDW_STATUS_GNSS2_FAULT_FLAG_OFFSET              = 11,

    /** GNSS is faulting firmware update REQUIRED */
    GPX_HDW_STATUS_GNSS_FW_UPDATE_REQUIRED              = (int)0x00001000,
    /** Enables LED in Manufacturing TBed */
    GPX_HDW_STATUS_LED_ENABLED                          = (int)0x00002000,
    /** System Reset is Required for proper function */
    GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED                = (int)0x00004000,
    /** System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typically 150-250 ms. */
    GPX_HDW_STATUS_FLASH_WRITE_PENDING                  = (int)0x00008000,

    /** Communications Tx buffer limited */
    GPX_HDW_STATUS_ERR_COM_TX_LIMITED                   = (int)0x00010000,
    /** Communications Rx buffer overrun */
    GPX_HDW_STATUS_ERR_COM_RX_OVERRUN                   = (int)0x00020000,    
    /** GPS1 PPS timepulse signal has not been received or is in error */
    GPX_HDW_STATUS_ERR_NO_GPS1_PPS                      = (int)0x00040000,
    /** GPS2 PPS timepulse signal has not been received or is in error */
    GPX_HDW_STATUS_ERR_NO_GPS2_PPS                      = (int)0x00080000,
    /** GPS PPS error mask */
    GPX_HDW_STATUS_ERR_PPS_MASK                         = (int)0x000C0000,

    /** GPS1 signal strength low (<20)*/
    GPX_HDW_STATUS_ERR_LOW_CNO_GPS1                     = (int)0x00100000,
    /** GPS2 signal strength low (<20)*/
    GPX_HDW_STATUS_ERR_LOW_CNO_GPS2                     = (int)0x00200000,
    /** GPS1 signal irregular. High Cno standard deviation over 5 second period detected. 10x CNO mean sigma (i.e. >1.0 dBHz) */
    GPX_HDW_STATUS_ERR_CNO_GPS1_IR                      = (int)0x00400000,
    /** GPS2 signal irregular. High Cno standard deviation over 5 second period detected. 10x CNO mean sigma (i.e. >1.0 dBHz) */
    GPX_HDW_STATUS_ERR_CNO_GPS2_IR                      = (int)0x00800000,
    /** GPS signal error mask*/
    GPX_HDW_STATUS_ERR_CNO_MASK                         = (int)0x00F00000,

    /** (BIT) Built-in self-test running */
    GPX_HDW_STATUS_BIT_RUNNING                          = (int)0x01000000,
    /** (BIT) Built-in self-test passed */
    GPX_HDW_STATUS_BIT_PASSED                           = (int)0x02000000,
    /** (BIT) Built-in self-test failure */
    GPX_HDW_STATUS_BIT_FAULT                            = (int)0x03000000,
    /** (BIT) Built-in self-test mask */
    GPX_HDW_STATUS_BIT_MASK                             = (int)0x03000000,
    GPX_HDW_STATUS_BIT_OFFSET                           = 24,
    /** Temperature outside spec'd operating range */
    GPX_HDW_STATUS_ERR_TEMPERATURE                      = (int)0x04000000,
    /** Time synchronized by GPS PPS */
    GPX_HDW_STATUS_GPS_PPS_TIMESYNC                     = (int)0x08000000,

    /** Cause of system reset */
    GPX_HDW_STATUS_RESET_CAUSE_MASK                     = (int)0x70000000,    
    /** Reset from Backup mode (low-power state w/ CPU off) */
    GPX_HDW_STATUS_RESET_CAUSE_BACKUP_MODE              = (int)0x10000000,
    /** Reset from Software */
    GPX_HDW_STATUS_RESET_CAUSE_SOFT                     = (int)0x20000000,
    /** Reset from Hardware (NRST pin low) */
    GPX_HDW_STATUS_RESET_CAUSE_HDW                      = (int)0x40000000,
    
    /** Critical System Fault, CPU error.  (see DID_GPX_STATUS.status, eGpxStatus::GPX_STATUS_FATAL_MASK) */
    GPX_HDW_STATUS_FAULT_SYS_CRITICAL                   = (int)0x80000000,
};

typedef enum {
    cxdRst_PowerOn          = 0,
    cxdRst_Watchdog         = 1,
    cxdRst_ErrOpCode        = 2,
    cxdRst_ErrOpCode_FwUp   = 3,
    cxdRst_ErrOpCode_init   = 4,
    cxdRst_UserRequested    = 5,
    cxdRst_FWUpdate         = 6,
    cxdRst_SysCmd           = 7,
    cxdRst_InitTimeout      = 8,
    cxdRst_Status5          = 9,
    cxdRst_StatusNot0       = 10,
    cxdRst_flashUpdate      = 11,
    cxdRst_RTKEphMissing    = 12,
    cxdRst_Max
} eGNSSDriverRstCause;

typedef enum {
    kReset = 0,
    kInit,
    kRun,
    kPassthrough,
    kFwInit,    // initializing into FwUpdate mode (prep for code injections)
    kFwUpdate,  // ready and able to accept code injections
    kError,
    kShutdown,
    kReinit,
    kHardReset,
} eGPXGnssRunState;

#define GNSS_RECEIVER_COUNT 2

typedef struct
{
    uint8_t lastRstCause;   /** Last reset cause (see eGNSSDriverRstCause) **/
    uint8_t fwUpdateState;  /** GNSS FW update status (see FirmwareUpdateState) **/
    uint8_t initState;      /** GNSS init status (see InitSteps) **/
    uint8_t runState;       /** GNSS run status (see eGPXGnssRunState) **/
} gpx_gnss_status_t;

#define GPX_INVALID_MCU_TEMP    -274.0f // 1 degree less than  absolute 0 

/**
* (DID_GPX_STATUS) GPX status.
*/
typedef struct
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Status (eGpxStatus) */
    uint32_t                status;

    /** GRMC BITS (see GRMC_BITS_...) **/
    uint64_t                grmcBitsSer0;
    uint64_t                grmcBitsSer1;
    uint64_t                grmcBitsSer2;
    uint64_t                grmcBitsUSB;
    /** (see NMEA_MSG_ID...) */
    uint64_t                grmcNMEABitsSer0;
    uint64_t                grmcNMEABitsSer1;
    uint64_t                grmcNMEABitsSer2;
    uint64_t                grmcNMEABitsUSB;

    /** Hardware status flags (eGPXHdwStatusFlags) */
    uint32_t                hdwStatus;

    /** MCU temperature (GPX_INVALID_MCU_TEMP if not availible) */
    float                   mcuTemp;

    /** Nav output period (ms). */
    uint32_t                navOutputPeriodMs;

    /** Flash config checksum used with host SDK synchronization */
    uint32_t                flashCfgChecksum;

    /** RTK Mode bits (see eRTKConfigBits) **/
    uint32_t                rtkMode;

    gpx_gnss_status_t       gnssStatus[GNSS_RECEIVER_COUNT];

    /** port */
    uint8_t                 gpxSourcePort;

    double                  upTime;     //!< Time in seconds, since system was started
} gpx_status_t;


//////////////////////////////////////////////////////////////////////////
//  EVB
//////////////////////////////////////////////////////////////////////////

typedef enum
{
    /** SD card logger: card ready */
    EVB_STATUS_SD_CARD_READY                = 0x00000001,

    /** SD card Logger: running */
    EVB_STATUS_SD_LOG_ENABLED               = 0x00000002,

    /** SD card error: card file system */
    EVB_STATUS_SD_ERR_CARD_FAULT            = 0x00000010,

    /** SD card error: card full */
    EVB_STATUS_SD_ERR_CARD_FULL             = 0x00000020,

    /** SD card error: mask */
    EVB_STATUS_SD_ERR_CARD_MASK             = 0x000000F0,

    /** WiFi: enabled */
    EVB_STATUS_WIFI_ENABLED                 = 0x00010000,

    /** WiFi: connected to access point (hot spot) or another device */
    EVB_STATUS_WIFI_CONNECTED               = 0x00020000,

    /** XBee: enabled */
    EVB_STATUS_XBEE_ENABLED                 = 0x00100000,

    /** XBee: connected */
    EVB_STATUS_XBEE_CONNECTED               = 0x00200000,

    /** XBee: configured */
    EVB_STATUS_XBEE_CONFIGURED              = 0x00400000,

    /** XBee: failed to configure */
    EVB_STATUS_XBEE_CONFIG_FAILURE          = 0x00800000,

    /** System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typicaly 150-250 ms. */
    EVB_STATUS_FLASH_WRITE_IN_PROGRESS      = 0x01000000,

    /** Manufacturing unlocked */
    EVB_STATUS_MANF_UNLOCKED                = 0x02000000,

} eEvbStatus;

/** EVB-2 communications ports. */
enum eEvb2CommPorts
{
    EVB2_PORT_UINS0     = 0,
    EVB2_PORT_UINS1     = 1,
    EVB2_PORT_XBEE      = 2,
    EVB2_PORT_XRADIO    = 3,        // H4-8 (orange) Tx, H4-7 (brown) Rx 
    EVB2_PORT_BLE       = 4,        
    EVB2_PORT_SP330     = 5,        // H3-2 (brown) Tx, H3-5 (green)  Rx
    EVB2_PORT_GPIO_H8   = 6,        // H8-5 (brown) Tx, H8-6 (orange) Rx
    EVB2_PORT_USB       = 7,
    EVB2_PORT_WIFI      = 8,        
    EVB2_PORT_CAN       = 9,        // H2-3 CANL (brown), H2-4 CANH (orange)
    EVB2_PORT_COUNT
};

/** EVB-2 Communications Bridge Options */
enum eEvb2ComBridgeOptions
{
    EVB2_CB_OPTIONS_TRISTATE_UINS_IO    = 0x00000001,
    EVB2_CB_OPTIONS_SP330_RS422         = 0x00000002,
    EVB2_CB_OPTIONS_XBEE_ENABLE         = 0x00000010,
    EVB2_CB_OPTIONS_WIFI_ENABLE         = 0x00000020,
    EVB2_CB_OPTIONS_BLE_ENABLE          = 0x00000040,
    EVB2_CB_OPTIONS_SPI_ENABLE          = 0x00000080,
    EVB2_CB_OPTIONS_CAN_ENABLE          = 0x00000100,
    EVB2_CB_OPTIONS_I2C_ENABLE          = 0x00000200,       // Tied to uINS G1,G2
};

enum eEvb2PortOptions
{
    EVB2_PORT_OPTIONS_RADIO_RTK_FILTER  = 0x00000001, // Allow RTCM3, NMEA, and RTCM3.  Reject IS binary.
    EVB2_PORT_OPTIONS_DEFAULT           = EVB2_PORT_OPTIONS_RADIO_RTK_FILTER,
};

/**
* (DID_EVB_STATUS) EVB-2 status and logger control interface
*/
typedef struct
{
    /** GPS number of weeks since January 6th, 1980 */
    uint32_t                week;

    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Firmware (software) version */
    uint8_t                 firmwareVer[4];

    /** Status (eEvbStatus) */
    uint32_t                evbStatus;

    /** Data logger control state. (see eEvb2LoggerMode) */
    uint32_t                loggerMode;

    /** logger */
    uint32_t                loggerElapsedTimeMs;

    /** WiFi IP address */
    uint32_t                wifiIpAddr;

    /** System command (see eSystemCommand).  99 = software reset */
    uint32_t                sysCommand;

    /** Time sync offset between local time since boot up to GPS time of week in seconds.  Add this to IMU and sensor time to get GPS time of week in seconds. */
    double                  towOffset;

} evb_status_t;

#define WIFI_SSID_PSK_SIZE      40

typedef struct
{
    /** WiFi SSID */
    char                    ssid[WIFI_SSID_PSK_SIZE];

    /** WiFi PSK */
    char                    psk[WIFI_SSID_PSK_SIZE];

} evb_wifi_t;

typedef struct
{  
    /** Server IP address */
    union {
        uint32_t        u32;
        uint8_t         u8[4];
    } ipAddr;

    /** Server port */
    uint32_t            port;

} evb_server_t;

typedef enum
{
    EVB_CFG_BITS_WIFI_SELECT_MASK               = 0x00000003,
    EVB_CFG_BITS_WIFI_SELECT_OFFSET             = 0,
    EVB_CFG_BITS_SERVER_SELECT_MASK             = 0x0000000C,
    EVB_CFG_BITS_SERVER_SELECT_OFFSET           = 2,
    EVB_CFG_BITS_NO_STREAM_PPD_ON_LOG_BUTTON    = 0x00000010,   // Don't enable PPD stream when log button is pressed
    EVB_CFG_BITS_ENABLE_ADC4                    = 0x00000200,
    EVB_CFG_BITS_ENABLE_ADC10                   = 0x00000400,
} eEvbFlashCfgBits;

#define NUM_WIFI_PRESETS     3
#define EVB_CFG_BITS_SET_IDX_WIFI(bits,idx)     {(bits)&=EVB_CFG_BITS_WIFI_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_WIFI_SELECT_OFFSET)&EVB_CFG_BITS_WIFI_SELECT_MASK);}
#define EVB_CFG_BITS_SET_IDX_SERVER(bits,idx)   {(bits)&=EVB_CFG_BITS_SERVER_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_SERVER_SELECT_OFFSET)&EVB_CFG_BITS_SERVER_SELECT_MASK);}
#define EVB_CFG_BITS_IDX_WIFI(bits)             (((bits)&EVB_CFG_BITS_WIFI_SELECT_MASK)>>EVB_CFG_BITS_WIFI_SELECT_OFFSET)
#define EVB_CFG_BITS_IDX_SERVER(bits)           (((bits)&EVB_CFG_BITS_SERVER_SELECT_MASK)>>EVB_CFG_BITS_SERVER_SELECT_OFFSET)

/**
* (DID_EVB_FLASH_CFG) EVB-2 flash config for monitor, config, and logger control interface
*/
typedef struct
{  
    /** Size of this struct */
    uint32_t                size;

    /** Checksum, excluding size and checksum */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /** Communications bridge preset. (see eEvb2ComBridgePreset) */
    uint8_t                 cbPreset;

    // 32-bit alignment
    uint8_t                 reserved1[3];

    /** Communications bridge forwarding */
    uint32_t                cbf[EVB2_PORT_COUNT];

    /** Communications bridge options (see eEvb2ComBridgeOptions) */
    uint32_t                cbOptions;

    /** Config bits (see eEvbFlashCfgBits) */
    uint32_t                bits;

    /** Radio preamble ID (PID) - 0x0 to 0x9. Only radios with matching PIDs can communicate together. Different PIDs minimize interference between multiple sets of networks. Checked before the network ID. */
    uint32_t                radioPID;

    /** Radio network ID (NID) - 0x0 to 0x7FFF. Only radios with matching NID can communicate together. Checked after the preamble ID. */
    uint32_t                radioNID;

    /** Radio power level - Transmitter output power level. (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)  */
    uint32_t                radioPowerLevel;

    /** WiFi SSID and PSK */
    evb_wifi_t              wifi[NUM_WIFI_PRESETS];

    /** Server IP and port */
    evb_server_t            server[NUM_WIFI_PRESETS];

    /** Encoder tick to wheel rotation conversion factor (in radians).  Encoder tick count per revolution on 1 channel x gear ratio x 2pi. */
    float                   encoderTickToWheelRad;

    /** CAN baudrate */
    uint32_t                CANbaud_kbps;

    /** CAN receive address */
    uint32_t                can_receive_address;

    /** EVB port for uINS communications and SD card logging. 0=uINS-Ser0 (default), 1=uINS-Ser1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts) */
    uint8_t                 uinsComPort;

    /** EVB port for uINS aux com and RTK corrections. 0=uINS-Ser0, 1=uINS-Ser1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts) */
    uint8_t                 uinsAuxPort;

    // Ensure 32-bit alignment
    uint8_t                 reserved2[2];

    /** Enable radio RTK filtering, etc. (see eEvb2PortOptions) */
    uint32_t                portOptions;

    /** Baud rate for EVB serial port H3 (SP330 RS233 and RS485/422). */
    uint32_t                h3sp330BaudRate;

    /** Baud rate for EVB serial port H4 (TLL to external radio). */
    uint32_t                h4xRadioBaudRate;

    /** Baud rate for EVB serial port H8 (TLL). */
    uint32_t                h8gpioBaudRate;

    /** Wheel encoder configuration (see eWheelCfgBits) */
    uint32_t                wheelCfgBits;

    /** Wheel update period.  Sets the wheel encoder and control update period. (ms) */
    uint32_t                velocityControlPeriodMs;

} evb_flash_cfg_t;


/** EVB-2 communications bridge configuration. */
enum eEvb2ComBridgePreset
{
    /** No change.  Sending this value causes no effect. */
    EVB2_CB_PRESET_NA = 0,

    /** No connections.  Off: XBee, WiFi */
    EVB2_CB_PRESET_ALL_OFF = 1,

    /** [uINS Hub] LED-GRN (uINS-COM): USB, RS232, H8.  (uINS-AUX): XRadio.  Off: XBee, WiFi */
    EVB2_CB_PRESET_RS232 = 2,

    /** [uINS Hub] LED-BLU (uINS-COM): USB, RS232, H8.  (uINS-AUX): XBee, XRadio.  Off: WiFi */
    EVB2_CB_PRESET_RS232_XBEE = 3,

    /** [uINS Hub] LED-PUR (uINS-COM): USB, RS422, H8.  (uINS-AUX): WiFi, XRadio.  Off: XBee */
    EVB2_CB_PRESET_RS422_WIFI = 4,

    /** [uINS Hub] LED-CYA (uINS-SER1 SPI): USB, RS423, H8.  Off: WiFi, XBee.  A reset is required following selection of this CBPreset to enable SPI on the uINS, in order to assert uINS pin 10 (G9/nSPI_EN) during bootup. */
    EVB2_CB_PRESET_SPI_RS232 = 5,

    /** [USB Hub]  LED-YEL (USB): RS232, H8, XBee, XRadio. */
    EVB2_CB_PRESET_USB_HUB_RS232 = 6,

    /** [USB Hub]  LED-WHT (USB): RS485/RS422, H8, XRadio. */
    EVB2_CB_PRESET_USB_HUB_RS422 = 7,
    
    /** Number of bridge configuration presets */
    EVB2_CB_PRESET_COUNT = 8,
    
};

#define EVB2_CB_PRESET_DEFAULT      EVB2_CB_PRESET_RS232

/** Data logger control.  Values labeled CMD  */
enum eEvb2LoggerMode
{
    /** Do not change.  Sending this value causes no effect. */
    EVB2_LOG_NA                         = 0,

    /** Start new log */
    EVB2_LOG_CMD_START                  = 2,

    /** Stop logging */
    EVB2_LOG_CMD_STOP                   = 4,

    /** Purge all data logs from drive */
    EVB2_LOG_CMD_PURGE                  = 1002,
        
};

enum ePortMonPortType
{
    PORT_MON_PORT_TYPE_UART             = (uint8_t)(1 << 4),
    PORT_MON_PORT_TYPE_USB              = (uint8_t)(2 << 4),
    PORT_MON_PORT_TYPE_SPI              = (uint8_t)(3 << 4),
    PORT_MON_PORT_TYPE_I2C              = (uint8_t)(4 << 4),
    PORT_MON_PORT_TYPE_CAN              = (uint8_t)(5 << 4),
    PORT_MON_PORT_TYPE_MAX              = (uint8_t)(6 << 4)
};

/** 
* (DID_PORT_MONITOR) Data rate and status monitoring for each communications port. 
*/
typedef port_stats_t port_monitor_set_t;

typedef struct
{
    /** Port monitor set */
    port_stats_t port[NUM_SERIAL_PORTS];

    /** Number of ports in the port[] array */
    uint8_t activePorts;        // FIXME: This should be moved to BEFORE the port definition, so on the receiving end, we know how many ports to expect.
} port_monitor_t;

/** Stores data for the event mask */
typedef struct
{
    /** Prioity mask (see eEventPriority) */
    int8_t priorityLevel;
      
    /** ID mask field (see eEventProtocol ie 0x01 << eEventProtocol) */
    uint32_t msgTypeIdMask;
} did_event_mask_t;

/** Sent in the data field of DID_EVENT for eEventProtocol:
 *  EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER,
 *  EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER,
 *  EVENT_MSG_TYPE_ID_ENA_FILTER 
*/
typedef struct
{
    /**target port mask 0x80 for current port other port (0x01 << TARGET_PORT) where target port is */
    uint8_t portMask;

    did_event_mask_t eventMask;

} did_event_filter_t;

#define EVENT_MEM_REQ_SIZE  16

typedef struct
{
    uint32_t reqAddr;
    uint8_t data[EVENT_MEM_REQ_SIZE];
} did_event_memResp_t;

typedef struct
{
    uint32_t reqAddr;
} did_event_memReq_t;

enum eEventMsgTypeID
{
    EVENT_MSG_TYPE_ID_RAW               = 1,
    EVENT_MSG_TYPE_ID_ASCII             = 2,
    EVENT_MSG_TYPE_ID_RTMC3_RCVR1       = 11,
    EVENT_MSG_TYPE_ID_RTMC3_RCVR2       = 12,
    EVENT_MSG_TYPE_ID_RTMC3_EXT         = 13,
    EVENT_MSG_TYPE_ID_SONY_BIN_RCVR1    = 14,
    EVENT_MSG_TYPE_ID_SONY_BIN_RCVR2    = 15,
    // EVENT_MSG_TYPE_ID_DBG_READ          = 16,

    EVENT_MSG_TYPE_ID_IMX_MEM_READ      = 20,
    EVENT_MSG_TYPE_ID_GPX_MEM_READ      = 21,
    EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_INST = 22,
    EVENT_MSG_TYPE_ID_IMX_SER0_REG      = 23,
    EVENT_MSG_TYPE_ID_IMX_SER0_CFG      = 24,
    EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_CHAN = 25,
    EVENT_MSG_TYPE_ID_IMX_GPIO_TX_0_REG = 26,

    EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_INST = 27,
    EVENT_MSG_TYPE_ID_GPX_SER0_REG      = 28,
    EVENT_MSG_TYPE_ID_GPX_SER0_CFG      = 29,
    EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_CHAN = 30,
    EVENT_MSG_TYPE_ID_GPX_GPIO_RX_0_REG = 31,

    EVENT_MSG_TYPE_ID_FILTER_RESPONSE   = (uint16_t)-4,
    EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER  = (uint16_t)-3,
    EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER  = (uint16_t)-2,
    EVENT_MSG_TYPE_ID_ENA_FILTER        = (uint16_t)-1,
};

typedef struct{
    uint32_t                inst_CCR;         /*!< DMA channel x configuration register        */
    uint32_t                inst_CNDTR;       /*!< DMA channel x number of data register       */
    uint32_t                inst_CPAR;        /*!< DMA channel x peripheral address register   */
    uint32_t                inst_CMAR;        /*!< DMA channel x memory address register       */

    uint8_t                 *ptr_start;
    uint8_t                 *ptr_end;
    uint16_t                active_tx_len;
    uint8_t                 done;                       // Currently only used in TX

    uint8_t                 cfg_dir;                    // DMA_RX or DMA_TX
    uint8_t                 cfg_circular;               // DMA_CIRC_ON or DMA_CIRC_OFF
    uint8_t                 cfg_priority;               // DMA_PRIO_LOW, DMA_PRIO_MEDIUM, DMA_PRIO_HIGH, DMA_PRIO_VERY_HIGH
    uint8_t                 cfg_interrupt;
    uint8_t                 cfg_interrupt_priority;     // 0 to 15, 15 is low
    uint8_t                 cfg_dma_channel_select;     // 0 to 7. See RM0394 11.6.7
    uint8_t                 cfg_parent_type;            // DMA_PARENT_USART, ...
    void                    *cfg_parent;                // Pointer to parent init base
    uint32_t                *cfg_periph_reg;            // Pointer to peripheral register
    uint8_t                 *cfg_buf;
    uint16_t                cfg_buf_len;                // This doesn't correspond to the length register, it is just however big the buffer is
    uint8_t                 cfg_linear_buf;             // If true, the buffer is user-specified and we treat it like a non-circular buffer.
    void                    *cfg_tcie_handler;          // If n
    
    int                     lastDmaUsed;                // Number of bytes in the buffer minus bytes last read.  This is used to identify buffer overflow.
    uint8_t                 overflow;                   // Buffer overflow

} eventImxDmaTxInst_t;

enum eEventPriority
{
    EVENT_PRIORITY_FAULT            = 0,
    EVENT_PRIORITY_ERR              = 1,
    EVENT_PRIORITY_WARNING          = 2,
    EVENT_PRIORITY_INFO             = 3,
    EVENT_PRIORITY_INFO_VERBOSE     = 4,
    EVENT_PRIORITY_DBG              = 5,
    EVENT_PRIORITY_DBG_VERBOSE      = 6, 
    EVENT_PRIORITY_TRIVIAL          = 7,
    
    // None should be used on all messages that should only be broadcast based on ID
    EVENT_PRIORITY_NONE             = -1,
};

typedef struct
{
    /** Time (uptime in seconds) */
    double          time;

    /** Serial number */
    uint32_t        senderSN;
  
    /** Hardware: 0=Host, 1=uINS, 2=EVB, 3=IMX, 4=GPX (see "Product Hardware ID") */
    uint16_t        senderHdwId;
    
    /** see eEventPriority */
    int8_t          priority;
    uint8_t         res8;

    /** see eEventMsgTypeID */
    uint16_t        msgTypeID;
    uint16_t        length;
    
    uint8_t data[1];
} did_event_t;

#define DID_EVENT_HEADER_SIZE           (sizeof(did_event_t) - sizeof(uint8_t))

enum eSysFaultStatus
{
    SYS_FAULT_STATUS_HARDWARE_RESET                 = 0x00000000,
    SYS_FAULT_STATUS_USER_RESET                     = 0x00000001,
    SYS_FAULT_STATUS_ENABLE_BOOTLOADER              = 0x00000002,
    // General:
    SYS_FAULT_STATUS_SOFT_RESET                     = 0x00000010,
    SYS_FAULT_STATUS_FLASH_MIGRATION_EVENT          = 0x00000020,
    SYS_FAULT_STATUS_FLASH_MIGRATION_COMPLETED      = 0x00000040,
    SYS_FAULT_STATUS_RTK_MISC_ERROR                 = 0x00000080,
    SYS_FAULT_STATUS_MCUBOOT_SWAP_FAILURE           = 0x00000100,
    SYS_FAULT_STATUS_MASK_GENERAL_ERROR             = 0xFFFFFFF0,
    // Critical: (usually associated with system reset)
    SYS_FAULT_STATUS_HARD_FAULT                     = 0x00010000,
    SYS_FAULT_STATUS_USAGE_FAULT                    = 0x00020000,
    SYS_FAULT_STATUS_MEM_MANGE                      = 0x00040000,
    SYS_FAULT_STATUS_BUS_FAULT                      = 0x00080000,
    SYS_FAULT_STATUS_MALLOC_FAILED                  = 0x00100000,
    SYS_FAULT_STATUS_STACK_OVERFLOW                 = 0x00200000,
    SYS_FAULT_STATUS_INVALID_CODE_OPERATION         = 0x00400000,
    SYS_FAULT_STATUS_FLASH_MIGRATION_MARKER_UPDATED = 0x00800000,
    SYS_FAULT_STATUS_WATCHDOG_RESET                 = 0x01000000,
    SYS_FAULT_STATUS_RTK_BUFFER_LIMIT               = 0x02000000,
    SYS_FAULT_STATUS_SENSOR_CALIBRATION             = 0x04000000,
    SYS_FAULT_STATUS_HARDWARE_DETECTION             = 0x08000000,
    SYS_FAULT_STATUS_MASK_CRITICAL_ERROR            = 0xFFFF0000,
};

/** (DID_SYS_FAULT) System Fault Information */ 
typedef struct 
{
    /** System fault status (see eSysFaultStatus) */
    uint32_t status;

    /** Fault Type at HardFault */
    uint32_t g1Task;

    /** Multipurpose register - Line number of fault */
    uint32_t g2FileNum;
    
    /** Multipurpose register - File number at fault */
    uint32_t g3LineNum;
        
    /** Multipurpose register - at time of fault.  */
    uint32_t g4;

    /** Multipurpose register - link register value at time of fault.  */
    uint32_t g5Lr;
    
    /** Program Counter value at time of fault */
    uint32_t pc;
    
    /** Program Status Register value at time of fault */
    uint32_t psr;
        
} system_fault_t;

/** RTOS tasks */
typedef enum
{
    /** Task 0: Sample    */
    IMX_TASK_SAMPLE = 0,

    /** Task 1: Nav */
    IMX_TASK_NAV,

    /** Task 2: Communications */
    IMX_TASK_COMMUNICATIONS,

    /** Task 3: Maintenance */
    IMX_TASK_MAINTENANCE,

    /** Task 4: Idle */
    IMX_TASK_IDLE,

    /** Task 5: Timer */
    IMX_TASK_TIMER,

    /** Number of RTOS tasks */
    IMX_RTOS_NUM_TASKS                 // Keep last
} eImxRtosTask;

/** RTOS tasks */
typedef enum
{
    /** Task 0: Communication */
    GPX_TASK_COMM = 0,

    /** Task 1: RTK */
    GPX_TASK_RTK,

    /** Task 2: Idle */
    GPX_TASK_IDLE,

    /** Task 3: Timer */
    GPX_TASK_TIMER,

    /** Number of RTOS tasks */
    GPX_RTOS_NUM_TASKS,                 // Keep last
} eGpxRtosTask;

/** EVB RTOS tasks */
typedef enum
{
    /** Task 0: Communications */
    EVB_TASK_COMMUNICATIONS,

    /** Task 1: Logger */
    EVB_TASK_LOGGER,

    /** Task 2: WiFi */
    EVB_TASK_WIFI,

    /** Task 3: Maintenance */
    EVB_TASK_MAINTENANCE,

    /** Task 4: Idle */
    EVB_TASK_IDLE,

    /** Task 5: Timer */
    EVB_TASK_TIMER,

    /** Task 6: SPI to uINS */
    EVB_TASK_SPI_UINS_COM,

    /** Number of RTOS tasks */
    EVB_RTOS_NUM_TASKS                  // Keep last
} eEvbRtosTask;

/** RTOS tasks */
typedef enum
{
#if defined(GPX_1)
    TASK_IDLE           = GPX_TASK_IDLE,
    TASK_TIMER          = GPX_TASK_TIMER,
    RTOS_NUM_TASKS      = GPX_RTOS_NUM_TASKS
#else   // IMX_5    
    TASK_IDLE           = IMX_TASK_IDLE,
    TASK_TIMER          = IMX_TASK_TIMER,
    RTOS_NUM_TASKS      = IMX_RTOS_NUM_TASKS
#endif
} eRtosTask;

/** Max task name length - do not change */
#define MAX_TASK_NAME_LEN 12

/** RTOS task info */
typedef struct PACKED
{
    /** Task name */
    char                    name[MAX_TASK_NAME_LEN];

    /** Task priority (0 - 8) */
    uint32_t                priority;

    /** Stack high water mark bytes */
    uint32_t                stackUnused;

    /** Task period ms */
    uint32_t                periodMs;

    /** Last runtime microseconds */
    uint32_t                runtimeUs;

    /** Average runtime */
    float                   avgRuntimeUs;

    /** Average of runtimes less than avgRuntimeUs */
    float                   lowerRuntimeUs;

    /** Average of runtimes greater than avgRuntimeUs */
    float                   upperRuntimeUs;

    /** Max runtime microseconds */
    uint32_t                maxRuntimeUs;

    /** Local time when task loop started (following delay) */
    uint32_t                startTimeUs;

    /** Counter of times task took too long to run */
    uint16_t                gapCount;

    /** Counter of times task took too long to run twice in a row */
    uint8_t                 doubleGapCount;

    /** Reserved */
    uint8_t                 reserved;

    /** Processor usage percent */
    float                   cpuUsage;

    /** Handle */
    uint32_t                handle;
    
} rtos_task_t;

/** Internal RTOS task profiling info (processor ticks instead of usec) */
typedef struct PACKED
{
    /** Time in microseconds */
    uint32_t                timeTicks;

    /** Runtime in microseconds */
    uint32_t                runtimeTicks;

    /** LPF average runtime */
    float                   avgRuntimeTicks;

    /** Average of runtimes less than avgRuntimeTicks */
    float                   lowerRuntimeTicks;

    /** Average of runtimes greater than avgRuntimeTicks */
    float                   upperRuntimeTicks;

    /** Maximum runtime microseconds */
    uint32_t                maxRuntimeTicks;

    /** Local time when task loop started (following delay) */
    uint32_t                startTimeTicks;

    /** Counter of times task took too long to run */
    uint16_t                gapCount;

    /** Counter of times task took too long to run back-to-back */
    uint8_t                 doubleGapCount;

    /** Indicates whether gap occurd on last update */
    uint8_t                 gapOnLast;

    /** Task period ms */
    uint32_t                periodTicks;

} rtos_profile_t;

/** (DID_RTOS_INFO) */
typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

    /** Total memory allocated using RTOS pvPortMalloc() */
    uint32_t                mallocSize;

    /** Total memory freed using RTOS vPortFree() */
    uint32_t                freeSize;

    /** Tasks */
    rtos_task_t             task[IMX_RTOS_NUM_TASKS];
} rtos_info_t;

/** (DID_GPX_RTOS_INFO) */
typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

    /** Total memory allocated using RTOS pvPortMalloc() */
    uint32_t                mallocSize;

    /** Total memory freed using RTOS vPortFree() */
    uint32_t                freeSize;

    /** Tasks */
    rtos_task_t             task[GPX_RTOS_NUM_TASKS];

} gpx_rtos_info_t;

/** (DID_EVB_RTOS_INFO) */
typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

    /** Total memory allocated using RTOS pvPortMalloc() */
    uint32_t                mallocSize;

    /** Total memory freed using RTOS vPortFree() */
    uint32_t                freeSize;

    /** Tasks */
    rtos_task_t             task[EVB_RTOS_NUM_TASKS];

} evb_rtos_info_t;


typedef struct 
{
    uint32_t runTimeUs;
    uint32_t maxRuntimeUs;
    uint32_t StartTimeUs;
    uint32_t startPeriodUs;
} runtime_profile_t;

/** (DID_RUNTIME_PROFILER) */
#define RUNTIME_PROFILE_COUNT    4
typedef struct 
{
    runtime_profile_t p[RUNTIME_PROFILE_COUNT];
} runtime_profiler_t;


enum
{
    CID_INS_TIME,
    CID_INS_STATUS,
    CID_INS_EULER,
    CID_INS_QUATN2B,
    CID_INS_QUATE2B,
    CID_INS_UVW,
    CID_INS_VE,
    CID_INS_LAT,
    CID_INS_LON,
    CID_INS_ALT,
    CID_INS_NORTH_EAST,
    CID_INS_DOWN,
    CID_INS_ECEF_X,
    CID_INS_ECEF_Y,
    CID_INS_ECEF_Z,
    CID_INS_MSL,
    CID_PREINT_PX,
    CID_PREINT_QY,
    CID_PREINT_RZ,
    CID_DUAL_PX,
    CID_DUAL_QY,
    CID_DUAL_RZ,
    CID_GPS1_POS,
    CID_GPS2_POS,
    CID_GPS1_RTK_POS_REL,
    CID_GPS2_RTK_CMP_REL,
    CID_ROLL_ROLLRATE,
    NUM_CIDS
};

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
    CAN_BAUDRATE_20_KBPS   =   20,
    CAN_BAUDRATE_33_KBPS   =   33,
    CAN_BAUDRATE_50_KBPS   =   50,
    CAN_BAUDRATE_83_KBPS   =   83,
    CAN_BAUDRATE_100_KBPS  =  100,
    CAN_BAUDRATE_125_KBPS  =  125,
    CAN_BAUDRATE_200_KBPS  =  200,
    CAN_BAUDRATE_250_KBPS  =  250,
    CAN_BAUDRATE_500_KBPS  =  500,
    CAN_BAUDRATE_1000_KBPS = 1000,

    CAN_BAUDRATE_COUNT = 10
} can_baudrate_t;

/** (DID_CAN_BCAST_PERIOD) Broadcast period of CAN messages */
typedef struct PACKED
{
    /** Broadcast period multiple - CAN time message. 0 to disable. */
    uint16_t                can_period_mult[NUM_CIDS];
    
    /** Transmit address. */
    uint32_t                can_transmit_address[NUM_CIDS];
    
    /** Baud rate (kbps)  (See can_baudrate_t for valid baud rates)  */
    uint16_t                can_baudrate_kbps;

    /** Receive address. */
    uint32_t                can_receive_address;

} can_config_t;

#if defined(INCLUDE_LUNA_DATA_SETS)
#include "luna_data_sets.h"
#endif

/** Union of datasets */
typedef union PACKED
{
    dev_info_t                  devInfo;
    ins_1_t                     ins1;
    ins_2_t                     ins2;
    ins_3_t                     ins3;
    ins_4_t                     ins4;
    imu_t                       imu;
    imu3_t                      imu3;
    magnetometer_t              mag;
    mag_cal_t                   magCal;
    barometer_t                 baro;
    wheel_encoder_t             wheelEncoder;
    ground_vehicle_t            groundVehicle;
    pos_measurement_t           posMeasurement;
    pimu_t                      pImu;
    gps_pos_t                   gpsPos;
    gps_vel_t                   gpsVel;
    gps_sat_t                   gpsSat;
    gps_sig_t                   gpsSig;
    gps_version_t               gpsVer;
    gps_rtk_rel_t               gpsRtkRel;
    gps_rtk_misc_t              gpsRtkMisc;
    inl2_states_t               inl2States;
    inl2_ned_sigma_t            inl2NedSigma;
    nvm_flash_cfg_t             flashCfg;
    survey_in_t                 surveyIn;
    sys_params_t                sysParams;
    sys_sensors_t               sysSensors;
    rtos_info_t                 rtosInfo;
    gpx_rtos_info_t             gRtosInfo;
    gps_raw_t                   gpsRaw;
    sys_sensors_adc_t           sensorsAdc;
    rmc_t                       rmc;
    evb_status_t                evbStatus;
    infield_cal_t               infieldCal;
    gpx_status_t                gpxStatus;
    debug_array_t               imxDebugArray;
    debug_array_t               gpxDebugArray;
    port_monitor_t              portMonitor;
    did_event_t                 event;
    manufacturing_info_t        manfInfo;
    bit_t                       bit;

#if defined(INCLUDE_LUNA_DATA_SETS)
    evb_luna_velocity_control_t wheelController;
#endif
} uDatasets;

/** Union of INS output datasets */
typedef union PACKED
{
    ins_1_t                     ins1;
    ins_2_t                     ins2;
    ins_3_t                     ins3;
    ins_4_t                     ins4;
} uInsOutDatasets;

POP_PACK

/**
Creates a 32 bit checksum from data

@param data the data to create a checksum for
@param count the number of bytes in data

@return the 32 bit checksum for data
*/
uint32_t checksum32(const void* data, int count);
uint32_t serialNumChecksum32(const void* data, int size);
uint32_t flashChecksum32(const void* data, int size);

/**
Flip the endianess of 32 bit values in data

@param data the data to flip 32 bit values in
@param dataLength the number of bytes in data
*/
void flipEndianess32(uint8_t* data, int dataLength);

/**
Flip the bytes of a float in place (4 bytes) - ptr is assumed to be at least 4 bytes

@param ptr the float to flip
*/
void flipFloat(uint8_t* ptr);

/**
Flip the bytes of a float (4 bytes) - ptr is assumed to be at least 4 bytes

@param val the float to flip
@return the flipped float
*/
float flipFloatCopy(float val);

/**
Flip the bytes of a double in place (8 bytes) - ptr is assumed to be at least 8 bytes
Only flips each 4 byte pair, does not flip the individual bytes within the pair

@param ptr the double to flip
*/
void flipDouble(void* ptr);

/**
Flip the bytes of a double in place (8 bytes)
Unlike flipDouble, this also flips the individual bytes in each 4 byte pair

@param val the double to flip
@return the flipped double
*/
double flipDoubleCopy(double val);

/**
Flip double (64 bit) floating point values in data

@param data the data to flip doubles in
@param dataLength the number of bytes in data
@param offset offset into data to start flipping at
@param offsets a list of offsets of all doubles in data, starting at position 0
@param offsetsLength the number of items in offsets
*/
void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

/**
Flip string values in data - this compensates for the fact that flipEndianess32 is called on all the data

@param data the data to flip string values in
@param dataLength the number of bytes in data
@param offset the offset into data to start flipping strings at
@param offsets a list of offsets and byte lengths into data where strings start at
@param offsetsLength the number of items in offsets, should be 2 times the string count
*/
void flipStrings(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

// BE_SWAP: if big endian then swap, else no-op
// LE_SWAP: if little endian then swap, else no-op
#if CPU_IS_BIG_ENDIAN
#define BE_SWAP64F(_i) flipDoubleCopy(_i)
#define BE_SWAP32F(_i) flipFloatCopy(_i)
#define BE_SWAP32(_i) (SWAP32(_i))
#define BE_SWAP16(_i) (SWAP16(_i))
#define LE_SWAP64F(_i) (_i)
#define LE_SWAP32F(_i) (_i)
#define LE_SWAP32(_i) (_i)
#define LE_SWAP16(_i) (_i)
#else // little endian
#define BE_SWAP64F(_i) (_i)
#define BE_SWAP32F(_i) (_i)
#define BE_SWAP32(_i) (_i)
#define BE_SWAP16(_i) (_i)
#define LE_SWAP64F(_i) flipDoubleCopy(_i)
#define LE_SWAP32F(_i) flipFloatCopy(_i)
#define LE_SWAP32(_i) (SWAP32(_i))
#define LE_SWAP16(_i) (SWAP16(_i))
#endif

/**
Get the offsets of double / int64 (64 bit) values given a data id

@param dataId the data id to get double offsets for
@param offsetsLength receives the number of double offsets

@return a list of offets of doubles or 0 if none, offset will have high bit set if it is an int64 instead of a double
*/
uint16_t* getDoubleOffsets(eDataIDs dataId, uint16_t* offsetsLength);

/**
Gets the offsets and lengths of strings given a data id

@param dataId the data id to get string offsets and lengths for
@param offsetsLength receives the number of items in the return value

@return a list of offsets and lengths of strings for the data id or 0 if none
*/
uint16_t* getStringOffsetsLengths(eDataIDs dataId, uint16_t* offsetsLength);

/** DID to RMC bit look-up table */
extern const uint64_t g_didToRmcBit[DID_COUNT];
uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits, uint64_t devInfoRmcBits);

/** DID to NMEA RMC bit look-up table */
extern const uint64_t g_didToNmeaRmcBit[DID_COUNT];

/** DID to GPX RMC bit look-up table */
extern const uint64_t g_gpxDidToGrmcBit[DID_COUNT];
extern const uint16_t g_gpxGRMCPresetLookup[GRMC_BIT_POS_COUNT];

#ifndef GPX_1

/*
Convert gnssID to ubx gnss indicator (ref [2] 25)

@param gnssID gnssID of satellite
@return ubx gnss indicator
*/
int ubxSys(int gnssID);

#endif

/*
Convert satellite constelation and prn/slot number to satellite number

@param sys satellite system (SYS_GPS,SYS_GLO,...)
@param prn satellite prn/slot number
@return satellite number (0:error)
*/
int satNo(int sys, int prn);

/*
convert satellite gnssID + svID to satellite number

@param gnssID satellite system 
@param svID satellite prn/slot number
@return satellite number (0:error)
*/
int satNumCalc(int gnssID, int svID);

void profiler_start(runtime_profile_t *p, uint32_t timeUs);
void profiler_stop(runtime_profile_t *p, uint32_t timeUs);
void profiler_maintenance_1s(runtime_profiler_t *p);


#ifdef __cplusplus
}
#endif

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
