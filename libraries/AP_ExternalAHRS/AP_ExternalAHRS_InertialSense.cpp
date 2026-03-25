/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  support for Inertial Sense INS
 */

#ifdef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#undef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#endif
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_InertialSense.h"

#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_GPS/AP_GPS_FixType.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

/*
 * Inertial Sense SDK types and function implementations, condensed from
 * ISConstants.h, data_sets.h, data_sets.c, and ISComm.c.
 * Only the subset used by this driver is included.
 */
using eDataIDs = uint32_t;
#define DID_NULL                  (eDataIDs)0   /** NULL (INVALID) */
#define DID_DEV_INFO              (eDataIDs)1   /** (dev_info_t) Device information */
#define DID_FLASH_CONFIG          (eDataIDs)12  /** (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GPS1_POS              (eDataIDs)13  /** (gps_pos_t) GPS 1 position data */
#define DID_GPS2_POS              (eDataIDs)14  /** (gps_pos_t) GPS 2 position data */
#define DID_RMC                   (eDataIDs)9   /** (rmc_t) Realtime Message Controller */
#define DID_GPS1_RTK_POS_MISC     (eDataIDs)22  /** (gps_rtk_misc_t) RTK precision position */
#define DID_GPS1_VEL              (eDataIDs)30  /** (gps_vel_t) GPS 1 velocity data */
#define DID_GPS2_VEL              (eDataIDs)31  /** (gps_vel_t) GPS 2 velocity data */
#define DID_MAGNETOMETER          (eDataIDs)52  /** (magnetometer_t) Magnetometer sensor */
#define DID_BAROMETER             (eDataIDs)53  /** (barometer_t) Barometric pressure sensor */
#define DID_BIT                   (eDataIDs)64  /** (bit_t) System built-in self-test */
#define DID_INS_3                 (eDataIDs)65  /** (ins_3_t) INS output: quaternion NED to body */
#define DID_INL2_NED_SIGMA        (eDataIDs)67  /** (inl2_ned_sigma_t) INL2 EKF std deviations */

// GPS type field within ioConfig (from data_sets.h eIoConfig)
static constexpr uint8_t  IO_CONFIG_GPS2_TYPE_OFFSET = 25;
static constexpr uint32_t IO_CONFIG_GPS_TYPE_MASK    = 0x7U;
#define IO_CONFIG_GPS2_TYPE(ioConfig) (((ioConfig) >> IO_CONFIG_GPS2_TYPE_OFFSET) & IO_CONFIG_GPS_TYPE_MASK)

#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN      24

enum InsStatusFlags
{
    INS_STATUS_HDG_ALIGN_COARSE                 = (int)0x00000001,
    INS_STATUS_VEL_ALIGN_COARSE                 = (int)0x00000002,
    INS_STATUS_POS_ALIGN_COARSE                 = (int)0x00000004,
    INS_STATUS_ALIGN_COARSE_MASK                = (int)0x00000007,
    INS_STATUS_WHEEL_AIDING_VEL                 = (int)0x00000008,
    INS_STATUS_HDG_ALIGN_FINE                   = (int)0x00000010,
    INS_STATUS_VEL_ALIGN_FINE                   = (int)0x00000020,
    INS_STATUS_POS_ALIGN_FINE                   = (int)0x00000040,
    INS_STATUS_ALIGN_FINE_MASK                  = (int)0x00000070,
    INS_STATUS_GPS_AIDING_HEADING               = (int)0x00000080,
    INS_STATUS_GPS_AIDING_POS                   = (int)0x00000100,
    INS_STATUS_GPS_UPDATE_IN_SOLUTION           = (int)0x00000200,
    INS_STATUS_EKF_USING_REFERENCE_IMU          = (int)0x00000400,
    INS_STATUS_MAG_AIDING_HEADING               = (int)0x00000800,
    INS_STATUS_NAV_MODE                         = (int)0x00001000,
#define INS_STATUS_DEAD_RECKONING(s) (((s)&(INS_STATUS_POS_ALIGN_FINE|INS_STATUS_POS_ALIGN_COARSE))&&(((s)&INS_STATUS_GPS_AIDING_POS)==0))
    INS_STATUS_STATIONARY_MODE                  = (int)0x00002000,
    INS_STATUS_GPS_AIDING_VEL                   = (int)0x00004000,
    INS_STATUS_KINEMATIC_CAL_GOOD               = (int)0x00008000,
    INS_STATUS_SOLUTION_MASK                    = (int)0x000F0000,
    INS_STATUS_SOLUTION_OFFSET                  = 16,
#define INS_STATUS_SOLUTION(s)      (((s)&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)
    INS_STATUS_SOLUTION_OFF                     = 0,
    INS_STATUS_SOLUTION_ALIGNING                = 1,
    INS_STATUS_SOLUTION_NAV                     = 3,
    INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE       = 4,
    INS_STATUS_SOLUTION_AHRS                    = 5,
    INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE      = 6,
    INS_STATUS_SOLUTION_VRS                     = 7,
    INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE       = 8,
    INS_STATUS_RTK_COMPASSING_BASELINE_UNSET    = (int)0x00100000,
    INS_STATUS_RTK_COMPASSING_BASELINE_BAD      = (int)0x00200000,
    INS_STATUS_RTK_COMPASSING_MASK              = (INS_STATUS_RTK_COMPASSING_BASELINE_UNSET|INS_STATUS_RTK_COMPASSING_BASELINE_BAD),
    INS_STATUS_MAG_RECALIBRATING                = (int)0x00400000,
    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL = (int)0x00800000,
    INS_STATUS_GPS_NAV_FIX_MASK                 = (int)0x03000000,
    INS_STATUS_GPS_NAV_FIX_OFFSET               = 24,
#define INS_STATUS_NAV_FIX_STATUS(s) (((s)&INS_STATUS_GPS_NAV_FIX_MASK)>>INS_STATUS_GPS_NAV_FIX_OFFSET)
    INS_STATUS_RTK_COMPASSING_VALID             = (int)0x04000000,
    INS_STATUS_RTK_RAW_GPS_DATA_ERROR           = (int)0x08000000,
    INS_STATUS_RTK_ERR_BASE_DATA_MISSING        = (int)0x10000000,
    INS_STATUS_RTK_ERR_BASE_POSITION_MOVING     = (int)0x20000000,
    INS_STATUS_RTK_ERR_BASE_POSITION_INVALID    = (int)0x30000000,
    INS_STATUS_RTK_ERR_BASE_MASK                = (int)0x30000000,
    INS_STATUS_RTK_ERROR_MASK                   = (INS_STATUS_RTK_RAW_GPS_DATA_ERROR|INS_STATUS_RTK_ERR_BASE_MASK),
    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN         = (int)0x40000000,
    INS_STATUS_GENERAL_FAULT                    = (int)0x80000000,
    INS_STATUS_ERROR_MASK                       = INS_STATUS_GENERAL_FAULT |
                                                  INS_STATUS_RTK_COMPASSING_MASK |
                                                  INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL |
                                                  INS_STATUS_RTK_ERROR_MASK |
                                                  INS_STATUS_RTOS_TASK_PERIOD_OVERRUN,
};

enum GpsNavFixStatus
{
    GPS_NAV_FIX_NONE                    = (int)0x00000000,
    GPS_NAV_FIX_POSITIONING_3D          = (int)0x00000001,
    GPS_NAV_FIX_POSITIONING_RTK_FLOAT   = (int)0x00000002,
    GPS_NAV_FIX_POSITIONING_RTK_FIX     = (int)0x00000003,
};

enum HdwStatusFlags
{
    HDW_STATUS_MOTION_GYR               = (int)0x00000001,
    HDW_STATUS_MOTION_ACC               = (int)0x00000002,
    HDW_STATUS_MOTION_MASK              = (int)0x00000003,
    HDW_STATUS_IMU_FAULT_REJECT_GYR     = (int)0x00000004,
    HDW_STATUS_IMU_FAULT_REJECT_ACC     = (int)0x00000008,
    HDW_STATUS_IMU_FAULT_REJECT_MASK    = (int)0x0000000C,
    HDW_STATUS_GPS_SATELLITE_RX_VALID   = (int)0x00000010,
    HDW_STATUS_STROBE_IN_EVENT          = (int)0x00000020,
    HDW_STATUS_GPS_TIME_OF_WEEK_VALID   = (int)0x00000040,
    HDW_STATUS_REFERENCE_IMU_RX         = (int)0x00000080,
    HDW_STATUS_SATURATION_GYR           = (int)0x00000100,
    HDW_STATUS_SATURATION_ACC           = (int)0x00000200,
    HDW_STATUS_SATURATION_MAG           = (int)0x00000400,
    HDW_STATUS_SATURATION_BARO          = (int)0x00000800,
    HDW_STATUS_SATURATION_MASK          = (int)0x00000F00,
    HDW_STATUS_SATURATION_OFFSET        = 8,
    HDW_STATUS_SYSTEM_RESET_REQUIRED    = (int)0x00001000,
    HDW_STATUS_ERR_GPS_PPS_NOISE        = (int)0x00002000,
    HDW_STATUS_MAG_RECAL_COMPLETE       = (int)0x00004000,
    HDW_STATUS_FLASH_WRITE_PENDING      = (int)0x00008000,
    HDW_STATUS_ERR_COM_TX_LIMITED       = (int)0x00010000,
    HDW_STATUS_ERR_COM_RX_OVERRUN       = (int)0x00020000,
    HDW_STATUS_ERR_NO_GPS_PPS           = (int)0x00040000,
    HDW_STATUS_GPS_PPS_TIMESYNC         = (int)0x00080000,
    HDW_STATUS_COM_PARSE_ERR_COUNT_MASK = (int)0x00F00000,
    HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET = 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(s) ((s & HDW_STATUS_COM_PARSE_ERR_COUNT_MASK) >> HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)
    HDW_STATUS_BIT_RUNNING              = (int)0x01000000,
    HDW_STATUS_BIT_PASSED               = (int)0x02000000,
    HDW_STATUS_BIT_FAILED               = (int)0x03000000,
    HDW_STATUS_BIT_MASK                 = (int)0x03000000,
    HDW_STATUS_ERR_TEMPERATURE          = (int)0x04000000,
    HDW_STATUS_SPI_INTERFACE_ENABLED    = (int)0x08000000,
    HDW_STATUS_RESET_CAUSE_MASK         = (int)0x70000000,
    HDW_STATUS_RESET_CAUSE_BACKUP_MODE  = (int)0x10000000,
    HDW_STATUS_RESET_CAUSE_WATCHDOG_FAULT = (int)0x20000000,
    HDW_STATUS_RESET_CAUSE_SOFT         = (int)0x30000000,
    HDW_STATUS_RESET_CAUSE_HDW          = (int)0x40000000,
    HDW_STATUS_FAULT_SYS_CRITICAL       = (int)0x80000000,
    HDW_STATUS_ERROR_MASK               = HDW_STATUS_FAULT_SYS_CRITICAL |
                                          HDW_STATUS_IMU_FAULT_REJECT_MASK |
                                          HDW_STATUS_SATURATION_MASK |
                                          HDW_STATUS_ERR_GPS_PPS_NOISE |
                                          HDW_STATUS_ERR_COM_TX_LIMITED |
                                          HDW_STATUS_ERR_COM_RX_OVERRUN |
                                          HDW_STATUS_ERR_NO_GPS_PPS |
                                          HDW_STATUS_BIT_FAILED |
                                          HDW_STATUS_ERR_TEMPERATURE,
};

enum GpsStatus
{
    GPS_STATUS_NUM_SATS_USED_MASK                   = (int)0x000000FF,
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
    GPS_STATUS_FLAGS_FIX_OK                         = (int)0x00010000,
    GPS_STATUS_FLAGS_DGPS_USED                      = (int)0x00020000,
    GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD               = (int)0x00040000,
    GPS_STATUS_FLAGS_UNUSED_1                       = (int)0x00080000,
    GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED      = (int)0x00100000,
    GPS_STATUS_FLAGS_STATIC_MODE                    = (int)0x00200000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED       = (int)0x00400000,
    GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR    = (int)0x00800000,
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_DATA_MISSING     = (int)0x01000000,
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MOVING  = (int)0x02000000,
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_INVALID = (int)0x03000000,
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK    = (int)0x03000000,
    GPS_STATUS_FLAGS_ERROR_MASK                     = (GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR |
                                                       GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK),
    GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID        = (int)0x04000000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID         = (int)0x08000000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD  = (int)0x00002000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET= (int)0x00004000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_MASK          = (GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET),
    GPS_STATUS_FLAGS_GPS_NMEA_DATA                  = (int)0x00008000,
    GPS_STATUS_FLAGS_GPS_PPS_TIMESYNC               = (int)0x10000000,
    GPS_STATUS_FLAGS_MASK                           = (int)0x1FFFE000,
    GPS_STATUS_FLAGS_BIT_OFFSET                     = (int)16,
    GPS_STATUS_FLAGS_UNUSED_2                       = (int)0x20000000,
    GPS_STATUS_FLAGS_UNUSED_3                       = (int)0x40000000,
    GPS_STATUS_FLAGS_UNUSED_4                       = (int)0x80000000,
};

#define IS_HDW_TYPE_PERIPHERAL 0x20
enum IsHardwareType
{
    IS_HARDWARE_TYPE_MIXED      = -1,
    IS_HARDWARE_TYPE_UNKNOWN    = 0,
    IS_HARDWARE_TYPE_UINS       = 1,
    IS_HARDWARE_TYPE_EVB        = 2,
    IS_HARDWARE_TYPE_IMX        = 3,
    IS_HARDWARE_TYPE_GPX        = 4,
    IS_HDW_GNSS_UBLOX           = IS_HDW_TYPE_PERIPHERAL + 1,
    IS_HDW_GNSS_SONY            = IS_HDW_TYPE_PERIPHERAL + 2,
    IS_HDW_GNSS_SEPTENTRIO      = IS_HDW_TYPE_PERIPHERAL + 3,
    IS_HDW_GNSS_STM_TESSIO      = IS_HDW_TYPE_PERIPHERAL + 4,
    IS_HARDWARE_TYPE_COUNT      = 5,
};

enum BitState
{
    BIT_STATE_OFF       = (int)0,
    BIT_STATE_DONE      = (int)1,
    BIT_STATE_RUNNING   = (int)6,
    BIT_STATE_FINISHING = (int)7,
};

enum HdwBitStatusFlags
{
    HDW_BIT_PASSED_MASK                   = (int)0x0000000F,
    HDW_BIT_PASSED_ALL                    = (int)0x00000001,
    HDW_BIT_PASSED_NO_GPS                 = (int)0x00000002,
    HDW_BIT_MODE_MASK                     = (int)0x000000F0,
    HDW_BIT_MODE_OFFSET                   = (int)4,
#define HDW_BIT_MODE(s) (((s)&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
    HDW_BIT_FAILED_MASK                   = (int)0xFFFFFF00,
    HDW_BIT_FAILED_AHRS_MASK              = (int)0xFFFF0F00,
    HDW_BIT_FAULT_NOISE_PQR               = (int)0x00000100,
    HDW_BIT_FAULT_NOISE_ACC               = (int)0x00000200,
    HDW_BIT_FAULT_MAGNETOMETER            = (int)0x00000400,
    HDW_BIT_FAULT_BAROMETER               = (int)0x00000800,
    HDW_BIT_FAULT_GPS_NO_COM              = (int)0x00001000,
    HDW_BIT_FAULT_GPS_POOR_CNO            = (int)0x00002000,
    HDW_BIT_FAULT_GPS_POOR_ACCURACY       = (int)0x00004000,
    HDW_BIT_FAULT_GPS_NOISE               = (int)0x00008000,
    HDW_BIT_FAULT_IMU_FAULT_REJECTION     = (int)0x00010000,
    HDW_BIT_FAULT_INCORRECT_HARDWARE_TYPE = (int)0x01000000,
};

enum CalBitStatusFlags
{
    CAL_BIT_PASSED_MASK             = (int)0x0000000F,
    CAL_BIT_PASSED_ALL              = (int)0x00000001,
    CAL_BIT_MODE_MASK               = (int)0x000000F0,
    CAL_BIT_MODE_OFFSET             = (int)4,
#define CAL_BIT_MODE(s) (((s)&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
    CAL_BIT_FAILED_MASK             = (int)0x00FFFF00,
    CAL_BIT_FAULT_TCAL_EMPTY        = (int)0x00000100,
    CAL_BIT_FAULT_TCAL_TSPAN        = (int)0x00000200,
    CAL_BIT_FAULT_TCAL_INCONSISTENT = (int)0x00000400,
    CAL_BIT_FAULT_TCAL_CORRUPT      = (int)0x00000800,
    CAL_BIT_FAULT_TCAL_PQR_BIAS     = (int)0x00001000,
    CAL_BIT_FAULT_TCAL_PQR_SLOPE    = (int)0x00002000,
    CAL_BIT_FAULT_TCAL_PQR_LIN      = (int)0x00004000,
    CAL_BIT_FAULT_TCAL_ACC_BIAS     = (int)0x00008000,
    CAL_BIT_FAULT_TCAL_ACC_SLOPE    = (int)0x00010000,
    CAL_BIT_FAULT_TCAL_ACC_LIN      = (int)0x00020000,
    CAL_BIT_FAULT_CAL_SERIAL_NUM    = (int)0x00040000,
    CAL_BIT_FAULT_MCAL_MAG_INVALID  = (int)0x00080000,
    CAL_BIT_FAULT_MCAL_EMPTY        = (int)0x00100000,
    CAL_BIT_FAULT_MCAL_IMU_INVALID  = (int)0x00200000,
    CAL_BIT_FAULT_MOTION_PQR        = (int)0x00400000,
    CAL_BIT_FAULT_MOTION_ACC        = (int)0x00800000,
    CAL_BIT_NOTICE_IMU1_PQR_BIAS    = (int)0x01000000,
    CAL_BIT_NOTICE_IMU2_PQR_BIAS    = (int)0x02000000,
    CAL_BIT_NOTICE_IMU1_ACC_BIAS    = (int)0x10000000,
    CAL_BIT_NOTICE_IMU2_ACC_BIAS    = (int)0x20000000,
};

#define RMC_OPTIONS_PORT_MASK           0x000000FF
#define RMC_OPTIONS_PORT_ALL            (RMC_OPTIONS_PORT_MASK)
#define RMC_OPTIONS_PORT_CURRENT        0x00000000
#define RMC_OPTIONS_PORT_SER0           0x00000001
#define RMC_OPTIONS_PORT_SER1           0x00000002
#define RMC_OPTIONS_PORT_SER2           0x00000004
#define RMC_OPTIONS_PORT_USB            0x00000008
#define RMC_OPTIONS_PRESERVE_CTRL       0x00000100
#define RMC_OPTIONS_PERSISTENT          0x00000200
#define RMC_BITS_INS1                   0x0000000000000001ULL
#define RMC_BITS_INS2                   0x0000000000000002ULL
#define RMC_BITS_INS3                   0x0000000000000004ULL
#define RMC_BITS_INS4                   0x0000000000000008ULL
#define RMC_BITS_IMU                    0x0000000000000010ULL
#define RMC_BITS_PIMU                   0x0000000000000020ULL
#define RMC_BITS_BAROMETER              0x0000000000000040ULL
#define RMC_BITS_MAGNETOMETER           0x0000000000000080ULL
#define RMC_BITS_GPS1_POS               0x0000000000000400ULL
#define RMC_BITS_GPS2_POS               0x0000000000000800ULL
#define RMC_BITS_GPS1_RAW               0x0000000000001000ULL
#define RMC_BITS_GPS2_RAW               0x0000000000002000ULL
#define RMC_BITS_GPS1_SAT               0x0000000000004000ULL
#define RMC_BITS_GPS2_SAT               0x0000000000008000ULL
#define RMC_BITS_GPS_BASE_RAW           0x0000000000010000ULL
#define RMC_BITS_STROBE_IN_TIME         0x0000000000020000ULL
#define RMC_BITS_DIAGNOSTIC_MESSAGE     0x0000000000040000ULL
#define RMC_BITS_GPS1_VEL               0x0000000000100000ULL
#define RMC_BITS_GPS2_VEL               0x0000000000200000ULL
#define RMC_BITS_GPS1_RTK_POS           0x0000000000800000ULL
#define RMC_BITS_GPS1_RTK_POS_REL       0x0000000001000000ULL
#define RMC_BITS_GPS1_RTK_POS_MISC      0x0000000004000000ULL
#define RMC_BITS_INL2_NED_SIGMA         0x0000000008000000ULL
#define RMC_BITS_INTERNAL_PPD           0x4000000000000000ULL
#define RMC_BITS_PRESET                 0x8000000000000000ULL

#define RMC_PRESET_IMX_PPD_NO_IMU  (RMC_BITS_PRESET       \
                                   | RMC_BITS_INS2         \
                                   | RMC_BITS_BAROMETER    \
                                   | RMC_BITS_MAGNETOMETER \
                                   | RMC_BITS_GPS1_POS     \
                                   | RMC_BITS_GPS2_POS     \
                                   | RMC_BITS_GPS1_VEL     \
                                   | RMC_BITS_GPS2_VEL     \
                                   | RMC_BITS_GPS1_RAW     \
                                   | RMC_BITS_GPS2_RAW     \
                                   | RMC_BITS_GPS_BASE_RAW \
                                   | RMC_BITS_GPS1_RTK_POS_REL \
                                   | RMC_BITS_INTERNAL_PPD \
                                   | RMC_BITS_DIAGNOSTIC_MESSAGE)
#define RMC_PRESET_IMX_PPD         (RMC_PRESET_IMX_PPD_NO_IMU \
                                   | RMC_BITS_PIMU)

struct PACKED AP_ExternalAHRS_InertialSense::dev_info_t
{
    uint16_t        reserved;
    uint8_t         hardwareType;
    uint8_t         hdwRunState;
    uint32_t        serialNumber;
    uint8_t         hardwareVer[4];
    uint8_t         firmwareVer[4];
    uint32_t        buildNumber;
    uint8_t         protocolVer[4];
    uint32_t        repoRevision;
    char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];
    uint8_t         buildType;
    uint8_t         buildYear;
    uint8_t         buildMonth;
    uint8_t         buildDay;
    uint8_t         buildHour;
    uint8_t         buildMinute;
    uint8_t         buildSecond;
    uint8_t         buildMillisecond;
    char            addInfo[DEVINFO_ADDINFO_STRLEN];
};

struct PACKED AP_ExternalAHRS_InertialSense::ins_3_t
{
    uint32_t    week;
    double      timeOfWeek;
    uint32_t    insStatus;
    uint32_t    hdwStatus;
    float       qn2b[4];
    float       uvw[3];
    double      lla[3];
    float       msl;
};

struct PACKED AP_ExternalAHRS_InertialSense::gps_pos_t
{
    uint32_t    week;
    uint32_t    timeOfWeekMs;
    uint32_t    status;
    double      ecef[3];
    double      lla[3];
    float       hMSL;
    float       hAcc;
    float       vAcc;
    float       pDop;
    float       cnoMean;
    double      towOffset;
    uint8_t     leapS;
    uint8_t     satsUsed;
    uint8_t     cnoMeanSigma;
    uint8_t     status2;
};

struct PACKED AP_ExternalAHRS_InertialSense::gps_vel_t
{
    uint32_t    timeOfWeekMs;
    float       vel[3];
    float       sAcc;
    uint32_t    status;
};

struct PACKED AP_ExternalAHRS_InertialSense::gps_rtk_misc_t
{
    uint32_t    timeOfWeekMs;
    float       accuracyPos[3];
    float       accuracyCov[3];
    float       arThreshold;
    float       gDop;
    float       hDop;
    float       vDop;
    double      baseLla[3];
    uint32_t    cycleSlipCount;
    uint32_t    roverGpsObservationCount;
    uint32_t    baseGpsObservationCount;
    uint32_t    roverGlonassObservationCount;
    uint32_t    baseGlonassObservationCount;
    uint32_t    roverGalileoObservationCount;
    uint32_t    baseGalileoObservationCount;
    uint32_t    roverBeidouObservationCount;
    uint32_t    baseBeidouObservationCount;
    uint32_t    roverQzsObservationCount;
    uint32_t    baseQzsObservationCount;
    uint32_t    roverSbasCount;
    uint32_t    baseSbasCount;
    uint32_t    baseAntennaCount;
    uint32_t    ionUtcAlmCount;
    uint32_t    correctionChecksumFailures;
    uint32_t    timeToFirstFixMs;
};

struct PACKED AP_ExternalAHRS_InertialSense::magnetometer_t
{
    double      time;
    float       mag[3];
};

struct PACKED AP_ExternalAHRS_InertialSense::barometer_t
{
    double      time;
    float       bar;
    float       mslBar;
    float       barTemp;
    float       humidity;
};

struct PACKED AP_ExternalAHRS_InertialSense::inl2_ned_sigma_t
{
    unsigned int    timeOfWeekMs;
    float           StdPosNed[3];
    float           StdVelNed[3];
    float           StdAttNed[3];
    float           StdAccBias[3];
    float           StdGyrBias[3];
    float           StdBarBias;
    float           StdMagDeclination;
};

struct PACKED AP_ExternalAHRS_InertialSense::bit_t
{
    uint8_t     command;
    uint8_t     lastCommand;
    uint8_t     state;
    uint8_t     reserved;
    uint32_t    hdwBitStatus;
    uint32_t    calBitStatus;
    float       tcPqrBias;
    float       tcAccBias;
    float       tcPqrSlope;
    float       tcAccSlope;
    float       tcPqrLinearity;
    float       tcAccLinearity;
    float       pqr;
    float       acc;
    float       pqrSigma;
    float       accSigma;
    uint8_t     testMode;
    uint8_t     testVar;
    uint16_t    detectedHardwareId;
};

struct PACKED rmc_t
{
    uint64_t    bits;
    uint32_t    options;
};

enum PktSpecialChars
{
    PSC_NMEA_START_BYTE     = 0x24,
    PSC_NMEA_PRE_END_BYTE   = 0x0D,
    PSC_NMEA_END_BYTE       = 0x0A,
    PSC_ISB_PREAMBLE_BYTE1  = 0xEF,
    PSC_ISB_PREAMBLE_BYTE2  = 0x49,
    PSC_ISB_PREAMBLE        = (PSC_ISB_PREAMBLE_BYTE2 << 8) | PSC_ISB_PREAMBLE_BYTE1,
    UBLOX_START_BYTE1       = 0xB5,
    UBLOX_START_BYTE2       = 0x62,
    RTCM3_START_BYTE        = 0xD3,
    SPARTN_START_BYTE       = 0x73,
    SONY_START_BYTE         = 0x7F,
};

void AP_ExternalAHRS_InertialSense::dev_info_populate_missing_hardware(dev_info_t *devInfo)
{
    if (devInfo->hardwareType != IS_HARDWARE_TYPE_UNKNOWN) {
        return;
    }
    int year = ((int)(devInfo->buildYear)) + 2000;
    if (year <= 2024) {
        switch (devInfo->hardwareVer[0]) {
        case 2: devInfo->hardwareType = IS_HARDWARE_TYPE_EVB;  break;
        case 3: devInfo->hardwareType = IS_HARDWARE_TYPE_UINS; break;
        case 5: devInfo->hardwareType = IS_HARDWARE_TYPE_IMX;  break;
        }
    }
}

union checksum16_u
{
    uint16_t ck;
    struct { uint8_t a; uint8_t b; };
};

/*
 * ISB-protocol Fletcher-16 checksum.  NOT interchangeable with
 * ArduPilot's crc_fletcher16(): the ISB variant uses natural uint8
 * overflow, whereas crc_fletcher16() uses modulo-255 arithmetic.
 */
uint16_t AP_ExternalAHRS_InertialSense::isb_fletcher16(uint16_t cksum_init, const void* data, uint32_t size)
{
    checksum16_u cksum;
    cksum.ck = cksum_init;
    for (uint32_t i = 0; i < size; i++) {
        cksum.a += ((const uint8_t*)data)[i];
        cksum.b += cksum.a;
    }
    return cksum.ck;
}

#define PKT_PARSER_TIMEOUT_MS   100

int AP_ExternalAHRS_InertialSense::is_comm_reset_buffer(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.head = c->rxBuf.tail = c->rxBuf.scan = c->rxBuf.scanPrior = c->rxBuf.start;
    c->processPkt = nullptr;
    return (int)c->rxBuf.size;
}

void AP_ExternalAHRS_InertialSense::is_comm_reset_parser(is_comm_instance_t* c)
{
    c->parser.state = 0;
    c->rxBuf.scanPrior = c->rxBuf.scan;
    c->rxBuf.scan = c->rxBuf.head;
    c->processPkt = nullptr;
}

void AP_ExternalAHRS_InertialSense::is_comm_to_isb_p_data(const is_comm_instance_t *comm, p_data_t *data)
{
    data->hdr.id     = comm->rxPkt.dataHdr.id;
    data->hdr.offset = comm->rxPkt.offset;
    data->hdr.size   = (uint16_t)comm->rxPkt.data.size;
    data->ptr        = comm->rxPkt.data.ptr;
}

AP_ExternalAHRS_InertialSense::ProtocolType AP_ExternalAHRS_InertialSense::report_parse_error(is_comm_instance_t* c, ParseErrorType errorType)
{
    if (!c->rxErrorState) {
        c->rxErrorState = 1;
        c->rxErrorCount++;
        c->rxErrorType = errorType;
        c->rxErrorTypeCount[errorType]++;
        return _PTYPE_PARSE_ERROR;
    }
    return _PTYPE_NONE;
}

AP_ExternalAHRS_InertialSense::ProtocolType AP_ExternalAHRS_InertialSense::parse_error_reset_state(is_comm_instance_t* c, ParseErrorType errorType)
{
    is_comm_reset_parser(c);
    return report_parse_error(c, errorType);
}

void AP_ExternalAHRS_InertialSense::valid_packet_reset(is_comm_instance_t* c, int pktSize)
{
    c->rxBuf.head += pktSize;
    c->rxPktCount++;
    c->processPkt = nullptr;
    c->rxErrorState = 0;
}

void AP_ExternalAHRS_InertialSense::set_parser_start(is_comm_instance_t* c, pFnProcessPkt processPkt)
{
    c->parser.state = 1;
    c->rxBuf.head   = c->rxBuf.scan;
    c->processPkt   = processPkt;
}

AP_ExternalAHRS_InertialSense::ProtocolType AP_ExternalAHRS_InertialSense::process_isb_pkt(void* v)
{
    is_comm_instance_t* c = (is_comm_instance_t*)v;
    is_comm_parser_t* p = &(c->parser);
    int numBytes = 0;

    switch (p->state) {
    case 0:
        if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE1) {
            p->state++;
        }
        return _PTYPE_NONE;

    case 1:
        if (*(c->rxBuf.scan) == PSC_ISB_PREAMBLE_BYTE2) {
            p->state++;
            return _PTYPE_NONE;
        }
        return parse_error_reset_state(c, EPARSE_INVALID_PREAMBLE);

    case 2: {
        numBytes = (int)(c->rxBuf.scan - c->rxBuf.head);
        if (numBytes < (int)(sizeof(packet_hdr_t) - 1)) {
            return _PTYPE_NONE;
        }
        p->state++;
        packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
        p->size = (uint16_t)(sizeof(packet_hdr_t) + isbPkt->hdr.payloadSize + 2);
        if (p->size > PKT_BUF_SIZE) {
            return parse_error_reset_state(c, EPARSE_INVALID_SIZE);
        }
        return _PTYPE_NONE;
    }

    default:
        numBytes = (int)(c->rxBuf.scan - c->rxBuf.head) + 1;
        if (numBytes < (int)(p->size)) {
            return _PTYPE_NONE;
        }
        break;
    }

    p->state = 0;

    packet_buf_t *isbPkt = (packet_buf_t*)(c->rxBuf.head);
    if (isbPkt->hdr.payloadSize > PKT_BUF_SIZE) {
        return parse_error_reset_state(c, EPARSE_INVALID_SIZE);
    }
    if ((isbPkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) &&
        (isbPkt->payload.offset + isbPkt->hdr.payloadSize > PKT_BUF_SIZE)) {
        return parse_error_reset_state(c, EPARSE_INVALID_HEADER);
    }

    uint16_t payloadSize = isbPkt->hdr.payloadSize;
    uint8_t *payload     = c->rxBuf.head + sizeof(packet_hdr_t);
    int bytes_cksum      = p->size - 2;
    uint16_t calcCksum   = isb_fletcher16(0, c->rxBuf.head, (uint32_t)bytes_cksum);
    uint16_t rxCksum;
    memcpy(&rxCksum, payload + payloadSize, sizeof(uint16_t));
    if (rxCksum != calcCksum) {
        return parse_error_reset_state(c, EPARSE_INVALID_CHKSUM);
    }

    valid_packet_reset(c, numBytes);

    packet_t *pkt          = &(c->rxPkt);
    pkt->hdr.preamble      = isbPkt->hdr.preamble;
    pkt->hdr.flags         = isbPkt->hdr.flags;
    pkt->id = pkt->hdr.id  = isbPkt->hdr.id;
    pkt->hdr.payloadSize   = payloadSize;

    if (pkt->hdr.flags & ISB_FLAGS_PAYLOAD_W_OFFSET) {
        pkt->data.size    = (payloadSize < 2 ? 0u : (uint32_t)(payloadSize - 2));
        pkt->data.ptr     = (pkt->data.size ? payload + 2 : nullptr);
        memcpy(&pkt->offset, payload, sizeof(uint16_t));
        pkt->dataHdr.size = (uint16_t)pkt->data.size;
    } else {
        pkt->data.size    = payloadSize;
        pkt->data.ptr     = (payloadSize ? payload : nullptr);
        pkt->offset       = 0;
    }

    pkt->checksum = rxCksum;
    pkt->size     = p->size;
    c->ackNeeded  = 0;

    uint8_t ptype = pkt->hdr.flags & PKT_TYPE_MASK;
    switch (ptype) {
    case PKT_TYPE_SET_DATA:
    case PKT_TYPE_DATA:
        if (pkt->data.size <= MAX_DATASET_SIZE) {
            if (ptype == PKT_TYPE_SET_DATA) {
                c->ackNeeded = PKT_TYPE_ACK;
            }
            return _PTYPE_INERTIAL_SENSE_DATA;
        } else {
            c->ackNeeded = PKT_TYPE_NACK;
        }
        break;

    case PKT_TYPE_GET_DATA: {
        p_data_get_t *get = (p_data_get_t*)&(isbPkt->payload.data);
        if (get->size <= MAX_DATASET_SIZE) {
            return _PTYPE_INERTIAL_SENSE_CMD;
        }
        break;
    }

    case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
    case PKT_TYPE_STOP_DID_BROADCAST:
    case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT:
        return _PTYPE_INERTIAL_SENSE_CMD;

    case PKT_TYPE_ACK:
    case PKT_TYPE_NACK:
        return _PTYPE_INERTIAL_SENSE_ACK;
    }

    return parse_error_reset_state(c, EPARSE_INVALID_DATATYPE);
}

void AP_ExternalAHRS_InertialSense::is_comm_init(is_comm_instance_t* c, uint8_t *buffer, int bufferSize,
                          pfnIsCommHandler pktHandler)
{
    memset(c, 0, sizeof(is_comm_instance_t));
    memset(buffer, 0, (size_t)bufferSize);

    c->rxBuf.size  = (uint32_t)bufferSize;
    c->rxBuf.start = buffer;
    c->rxBuf.end   = buffer + bufferSize;

    is_comm_reset_buffer(c);

    c->cb.protocolMask = DEFAULT_PROTO_MASK;
    c->rxPkt.data.ptr  = c->rxBuf.start;
    c->rxErrorState    = 1;
    c->cb.all          = pktHandler;
}

void AP_ExternalAHRS_InertialSense::is_comm_enable_protocol(is_comm_instance_t* comm, ProtocolType ptype)
{
    if (comm) {
        comm->cb.protocolMask |= (0x01u << ptype);
    }
}

AP_ExternalAHRS_InertialSense::pfnIsCommIsbDataHandler AP_ExternalAHRS_InertialSense::is_comm_register_isb_handler(is_comm_instance_t* comm,
                                                              pfnIsCommIsbDataHandler cbHandler)
{
    if (!comm) {
        return nullptr;
    }
    pfnIsCommIsbDataHandler priorCb = comm->cb.isbData;
    comm->cb.isbData = cbHandler;
    comm->cb.protocolMask |= ENABLE_PROTOCOL_ISB;
    return priorCb;
}

int AP_ExternalAHRS_InertialSense::is_comm_free(is_comm_instance_t* c)
{
    is_comm_buffer_t *buf = &(c->rxBuf);
    int bytesFree = (int)(buf->end - buf->tail);

    if (bytesFree < (int)(buf->size)) {
        if (c->processPkt != nullptr) {
            if (buf->head != buf->start) {
                int shift = (int)(buf->head - buf->start);
                memmove(buf->start, buf->head, (size_t)(buf->tail - buf->head));
                buf->head = buf->start;
                buf->tail -= shift;
                buf->scan -= shift;
                bytesFree = (int)(buf->end - buf->tail);
            } else if (bytesFree == 0) {
                report_parse_error(c, EPARSE_RXBUFFER_FLUSHED);
                return is_comm_reset_buffer(c);
            }
        } else {
            if (buf->scan >= buf->tail) {
                return is_comm_reset_buffer(c);
            }
        }
    }
    return bytesFree;
}

void AP_ExternalAHRS_InertialSense::is_comm_encode_hdr(packet_t *pkt, uint8_t flags, uint16_t did,
                                 uint16_t data_size, uint16_t offset, const void* data)
{
    pkt->hdr.preamble    = PSC_ISB_PREAMBLE;
    pkt->hdr.flags       = flags;
    pkt->hdr.id          = (uint8_t)did;
    pkt->hdr.payloadSize = data_size;
    pkt->offset          = offset;
    if (offset) {
        pkt->hdr.flags       |= ISB_FLAGS_PAYLOAD_W_OFFSET;
        pkt->hdr.payloadSize  = (uint16_t)(data_size + 2);
    }
    pkt->data.ptr  = (uint8_t*)data;
    pkt->data.size = data_size;
    pkt->size      = (uint16_t)(pkt->hdr.payloadSize + sizeof(packet_hdr_t) + 2);
    pkt->hdrCksum  = isb_fletcher16(0, &pkt->hdr, sizeof(pkt->hdr));
}

void AP_ExternalAHRS_InertialSense::memcpy_inc_update_checksum(uint8_t **dstBuf, const uint8_t* srcBuf,
                                     int len, uint16_t *checksum)
{
    *checksum = isb_fletcher16(*checksum, srcBuf, (uint32_t)len);
    memcpy(*dstBuf, srcBuf, (size_t)len);
    *dstBuf += len;
}

int AP_ExternalAHRS_InertialSense::is_comm_write_isb_precomp_to_buffer(uint8_t *buf, uint32_t buf_size,
                                                 is_comm_instance_t* comm, packet_t *pkt)
{
    if (pkt->size > buf_size) {
        return -1;
    }
    uint16_t checksum = pkt->hdrCksum;
    memcpy(buf, (uint8_t*)&(pkt->hdr), sizeof(packet_hdr_t));
    buf += sizeof(packet_hdr_t);
    if (pkt->offset) {
        memcpy_inc_update_checksum(&buf, (uint8_t*)&(pkt->offset), 2, &checksum);
    }
    if (pkt->data.size) {
        memcpy_inc_update_checksum(&buf, (uint8_t*)pkt->data.ptr,
                                (int)pkt->data.size, &checksum);
    }
    pkt->checksum = checksum;
    memcpy(buf, &checksum, sizeof(checksum));
    buf += 2;
    comm->txPktCount++;
    return (int)pkt->size;
}

int AP_ExternalAHRS_InertialSense::is_comm_write_to_buf(uint8_t* buf, uint32_t buf_size,
                                  is_comm_instance_t* comm, uint8_t flags,
                                  uint16_t did, uint16_t data_size, uint16_t offset,
                                  const void* data)
{
    packet_t txPkt;
    is_comm_encode_hdr(&txPkt, flags, did, data_size, offset, data);
    return is_comm_write_isb_precomp_to_buffer(buf, buf_size, comm, &txPkt);
}

int AP_ExternalAHRS_InertialSense::is_comm_get_data_to_buf(uint8_t *buf, uint32_t buf_size,
                                    is_comm_instance_t* comm, uint32_t did,
                                    uint32_t size, uint32_t offset,
                                    uint32_t periodMultiple)
{
    p_data_get_t get;
    get.id     = (uint16_t)did;
    get.offset = (uint16_t)offset;
    get.size   = (uint16_t)size;
    get.period = (uint16_t)periodMultiple;
    return is_comm_write_to_buf(buf, buf_size, comm, PKT_TYPE_GET_DATA, 0,
                                 sizeof(p_data_get_t), 0, &get);
}

AP_ExternalAHRS_InertialSense::ProtocolType AP_ExternalAHRS_InertialSense::is_comm_parse_timeout(is_comm_instance_t* c, uint32_t timeMs)
{
    is_comm_buffer_t *buf = &(c->rxBuf);

#if PKT_PARSER_TIMEOUT_MS
    if (c->processPkt) {
        if (timeMs > c->parser.timeMs + PKT_PARSER_TIMEOUT_MS) {
            c->rxBuf.head++;
            is_comm_reset_parser(c);
        }
    }
#endif

    while (buf->scan < buf->tail) {
        if (c->processPkt == nullptr) {
            if (*(buf->scan) == PSC_ISB_PREAMBLE_BYTE1) {
                if (c->cb.protocolMask & ENABLE_PROTOCOL_ISB) {
                    set_parser_start(c, process_isb_pkt);
                }
            } else {
                if (report_parse_error(c, EPARSE_STREAM_UNPARSABLE)) {
                    return _PTYPE_PARSE_ERROR;
                }
            }
        } else {
            ProtocolType ptype = c->processPkt(c);
            if (ptype != _PTYPE_NONE) {
                buf->scan++;
                return ptype;
            }
        }
        buf->scan++;
    }

#if PKT_PARSER_TIMEOUT_MS
    if (c->processPkt) {
        c->parser.timeMs = timeMs;
    }
#endif

    return _PTYPE_NONE;
}

AP_ExternalAHRS_InertialSense::ProtocolType AP_ExternalAHRS_InertialSense::is_comm_parse(is_comm_instance_t* comm)
{
    return is_comm_parse_timeout(comm, 0);
}

void AP_ExternalAHRS_InertialSense::parse_messages(is_comm_instance_t* comm, port_handle_t port)
{
    if (!comm) {
        return;
    }
    ProtocolType ptype;
    while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE) {
        int notConsumed = -1;
        switch (ptype) {
        case _PTYPE_INERTIAL_SENSE_DATA:
            if (comm->cb.isbData) {
                p_data_t data;
                is_comm_to_isb_p_data(comm, &data);
                notConsumed = comm->cb.isbData(comm->cb.context, &data, port);
            }
            break;
        case _PTYPE_INERTIAL_SENSE_ACK:
        case _PTYPE_INERTIAL_SENSE_CMD:
            break;
        default:
            if (comm->cb.generic[ptype]) {
                notConsumed = comm->cb.generic[ptype](
                    comm->cb.context,
                    comm->rxPkt.data.ptr + comm->rxPkt.offset,
                    (int)comm->rxPkt.data.size,
                    port);
            }
            break;
        }
        if (comm->cb.all && notConsumed) {
            comm->cb.all(comm->cb.context, ptype, &(comm->rxPkt), port);
        }
    }
}

void AP_ExternalAHRS_InertialSense::is_comm_buffer_parse_messages(uint8_t *buf, uint32_t buf_size,
                                           is_comm_instance_t* comm)
{
    int n = (int)MIN((int)buf_size, is_comm_free(comm));
    memcpy(comm->rxBuf.tail, buf, (size_t)n);
    comm->rxBuf.tail += n;
    parse_messages(comm, nullptr);
}

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_InertialSense *AP_ExternalAHRS_InertialSense::instance = nullptr;

AP_ExternalAHRS_InertialSense::AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS ExternalAHRS: no UART");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IS ExternalAHRS: baud %" PRIu32 " port %d", baudrate, port_num);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialSense::update_thread, void), "IS", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Inertial Sense failed to allocate ExternalAHRS update thread");
    }

    // don't offer IMU by default, the processing can take the main loop below minimum rate
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
}

int8_t AP_ExternalAHRS_InertialSense::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
}

const char* AP_ExternalAHRS_InertialSense::get_name() const
{
    return _name;
}

bool AP_ExternalAHRS_InertialSense::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return _healthy && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::initialised(void) const
{
    uint32_t now = AP_HAL::millis();
    return initialized && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense unhealthy");
        return false;
    }

    if (_fix_type < AP_GPS_FixType::FIX_3D) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense no GPS lock");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_InertialSense::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (initialised()) {
        status.flags.initalized = true;
    }

    if (healthy()) {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

bool AP_ExternalAHRS_InertialSense::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = vel_cov * vel_gate_scale;
    posVar = pos_cov * pos_gate_scale;
    hgtVar = hgt_cov * hgt_gate_scale;
    tasVar = 0;
    return true;
}

int AP_ExternalAHRS_InertialSense::stop_message_broadcasting()
{
    int size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, nullptr);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to stop all port broadcasts");
        return -3;
    }

    size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT, 0, 0, 0, nullptr);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to stop cur port broadcast");
        return -3;
    }

    return 0;
}

int AP_ExternalAHRS_InertialSense::enable_message_broadcasting()
{
    int size;

    // Ask for INS_3 at the EAHRS-configured rate (device runs at 250 Hz)
    const uint16_t ins3_period = MAX(1, 250 / get_rate());
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INS_3, 0, 0, ins3_period);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_INS_3");
        return -4;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_POS, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_GPS1_POS");
        return -5;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_VEL, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_GPS1_VEL");
        return -5;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS2_POS, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_GPS2_POS");
        return -5;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS2_VEL, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_GPS2_VEL");
        return -5;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_RTK_POS_MISC, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_GPS1_RTK_POS_MISC");
        return -5;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_MAGNETOMETER, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_MAGNETOMETER");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_BAROMETER, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_BAROMETER");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INL2_NED_SIGMA, 0, 0, 1);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_INL2_NED_SIGMA");
        return -6;
    }

    // request flash config once to determine number of GPS sensors
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_FLASH_CONFIG, 0, 0, 0);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_FLASH_CONFIG");
        return -7;
    }

    // request a device info message
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_DEV_INFO, 0, 0, 0);
    if (uart->write(buffer, (uint16_t)size) != (size_t)size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: failed to request DID_DEV_INFO");
        return -6;
    }

    return 0;
}

void AP_ExternalAHRS_InertialSense::initialize()
{
    if (uart == nullptr) {
        return;
    }

    uart->begin(baudrate);

    is_comm_init(&comm, comm_buf, sizeof(comm_buf), nullptr);

    instance = this;
    is_comm_register_isb_handler(&comm, &AP_ExternalAHRS_InertialSense::isb_data_handler);

    int error = 0;

    if ((error = stop_message_broadcasting())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: stop_message_broadcasting failed %d", error);
        return;
    }
    hal.scheduler->delay(500);

    if ((error = enable_message_broadcasting())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IS: enable_message_broadcasting failed %d", error);
        return;
    }

    initialized = true;
}

void AP_ExternalAHRS_InertialSense::handle_ins3_message(ins_3_t* ins)
{
    last_filter_pkt = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    Quaternion q(ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3]);
    state.quat = q;
    state.have_quaternion = true;

    Vector3f uvw(ins->uvw[0], ins->uvw[1], ins->uvw[2]);
    state.velocity = q * uvw;
    state.have_velocity = true;

    state.location = Location{
        (int32_t)(ins->lla[0] * 1e7),
        (int32_t)(ins->lla[1] * 1e7),
        (int32_t)(ins->msl * 100),
        Location::AltFrame::ABSOLUTE};
    state.have_location = true;
    state.last_location_update_us = AP_HAL::micros();

    switch ((GpsNavFixStatus)INS_STATUS_NAV_FIX_STATUS(ins->insStatus)) {
    case GpsNavFixStatus::GPS_NAV_FIX_NONE:
        _fix_type = AP_GPS_FixType::NONE;
        break;

    case GpsNavFixStatus::GPS_NAV_FIX_POSITIONING_3D:
        _fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case GpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FLOAT:
        _fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case GpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FIX:
        _fix_type = AP_GPS_FixType::RTK_FIXED;
        break;

    default:
        _fix_type = AP_GPS_FixType::NO_GPS;
    }

    if (_fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }

    const uint32_t hdwStatus = ins->hdwStatus;
    const bool hardware_healthy = (hdwStatus & HDW_STATUS_ERROR_MASK) == HDW_STATUS_BIT_PASSED;

    WITH_SEMAPHORE(sem);
    const bool was_healthy = _healthy;
    _healthy = hardware_healthy;

    // Only report diagnostics on the transition to unhealthy to avoid flooding
    // the GCS at the INS_3 message rate while semaphores are held.
    if (!hardware_healthy && was_healthy) {
        const uint32_t bit_state = hdwStatus & HDW_STATUS_BIT_MASK;

        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: hdwStatus 0x%08" PRIx32, hdwStatus);

        // BIT state
        if (bit_state == HDW_STATUS_BIT_RUNNING)
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: BIT running");
        else if (bit_state == HDW_STATUS_BIT_FAILED)
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: BIT failed");
        else if (bit_state != HDW_STATUS_BIT_PASSED)
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: BIT not started (0x%" PRIx32 ")", bit_state);

        // Individual error bits
        if (hdwStatus & HDW_STATUS_FAULT_SYS_CRITICAL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IS: critical system fault");
        }
        if (hdwStatus & HDW_STATUS_IMU_FAULT_REJECT_GYR) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IS: redundant gyro rejected");
        }
        if (hdwStatus & HDW_STATUS_IMU_FAULT_REJECT_ACC) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IS: redundant accel rejected");
        }
        if (hdwStatus & HDW_STATUS_SATURATION_GYR) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: gyro saturation");
        }
        if (hdwStatus & HDW_STATUS_SATURATION_ACC) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: accel saturation");
        }
        if (hdwStatus & HDW_STATUS_SATURATION_MAG) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: mag saturation");
        }
        if (hdwStatus & HDW_STATUS_SATURATION_BARO) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: baro saturation");
        }
        if (hdwStatus & HDW_STATUS_ERR_GPS_PPS_NOISE) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: GPS PPS noise");
        }
        if (hdwStatus & HDW_STATUS_ERR_COM_TX_LIMITED) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: COM TX limited");
        }
        if (hdwStatus & HDW_STATUS_ERR_COM_RX_OVERRUN) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: COM RX overrun");
        }
        if (hdwStatus & HDW_STATUS_ERR_NO_GPS_PPS) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: no GPS PPS");
        }
        if (hdwStatus & HDW_STATUS_ERR_TEMPERATURE) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IS: temperature fault");
        }
    }
}

void AP_ExternalAHRS_InertialSense::handle_gps_pos_message(gps_pos_t* pos)
{
    last_gps_pkt = AP_HAL::millis();

    AP_GPS_FixType fix_type = AP_GPS_FixType::NONE;

    int fix = pos->status & GPS_STATUS_FIX_MASK;
    switch (fix) {
    case GPS_STATUS_FIX_2D:
        fix_type = AP_GPS_FixType::FIX_2D;
        break;

    case GPS_STATUS_FIX_3D:
        fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case GPS_STATUS_FIX_DGPS:
        fix_type = AP_GPS_FixType::DGPS;
        break;

    case GPS_STATUS_FIX_RTK_FLOAT:
        fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case GPS_STATUS_FIX_RTK_FIX:
        fix_type = AP_GPS_FixType::RTK_FIXED;
        break;
    }

    int32_t latitude = (int32_t)(pos->lla[0] * 1e7);
    int32_t longitude = (int32_t)(pos->lla[1] * 1e7);
    int32_t msl_altitude = (int32_t)(pos->hMSL * 100);

    gps_data_msg.gps_week = (uint16_t)pos->week;
    gps_data_msg.ms_tow = pos->timeOfWeekMs;
    gps_data_msg.fix_type = fix_type;
    gps_data_msg.satellites_in_view = pos->satsUsed;
    gps_data_msg.horizontal_pos_accuracy = pos->hAcc;
    gps_data_msg.vertical_pos_accuracy = pos->vAcc;
    gps_data_msg.longitude = longitude;
    gps_data_msg.latitude = latitude;
    gps_data_msg.msl_altitude = msl_altitude;

    gps_data_msg.hdop = pos->hAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handle_gps_vel_message(gps_vel_t* vel)
{
    gps_data_msg.horizontal_vel_accuracy = vel->sAcc;
    gps_data_msg.ned_vel_north = vel->vel[0];
    gps_data_msg.ned_vel_east = vel->vel[1];
    gps_data_msg.ned_vel_down = vel->vel[2];

    if (gps_data_msg.fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{
            gps_data_msg.latitude,
            gps_data_msg.longitude,
            gps_data_msg.msl_altitude,
            Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    uint8_t gps_instance;
    if (AP::gps().get_first_external_instance(gps_instance)) {
        AP::gps().handle_external(gps_data_msg, gps_instance);
    }

    gps_data_msg.vdop = vel->sAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handle_gps_rtk_pos_misc_message(gps_rtk_misc_t* misc)
{
    gps_data_msg.hdop = misc->hDop * 100;
    gps_data_msg.vdop = misc->vDop * 100;
}

void AP_ExternalAHRS_InertialSense::handle_gps2_pos_message(gps_pos_t* pos)
{
    AP_GPS_FixType fix_type = AP_GPS_FixType::NONE;
    const int fix = pos->status & GPS_STATUS_FIX_MASK;
    switch (fix) {
    case GPS_STATUS_FIX_2D:
        fix_type = AP_GPS_FixType::FIX_2D;
        break;
    case GPS_STATUS_FIX_3D:
        fix_type = AP_GPS_FixType::FIX_3D;
        break;
    case GPS_STATUS_FIX_DGPS:
        fix_type = AP_GPS_FixType::DGPS;
        break;
    case GPS_STATUS_FIX_RTK_FLOAT:
        fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;
    case GPS_STATUS_FIX_RTK_FIX:
        fix_type = AP_GPS_FixType::RTK_FIXED;
        break;
    }

    gps2_data_msg.gps_week             = (uint16_t)pos->week;
    gps2_data_msg.ms_tow               = pos->timeOfWeekMs;
    gps2_data_msg.fix_type             = fix_type;
    gps2_data_msg.satellites_in_view   = pos->satsUsed;
    gps2_data_msg.horizontal_pos_accuracy = pos->hAcc;
    gps2_data_msg.vertical_pos_accuracy   = pos->vAcc;
    gps2_data_msg.longitude            = (int32_t)(pos->lla[1] * 1e7);
    gps2_data_msg.latitude             = (int32_t)(pos->lla[0] * 1e7);
    gps2_data_msg.msl_altitude         = (int32_t)(pos->hMSL * 100);
    gps2_data_msg.hdop                 = pos->hAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handle_gps2_vel_message(gps_vel_t* vel)
{
    gps2_data_msg.horizontal_vel_accuracy = vel->sAcc;
    gps2_data_msg.ned_vel_north           = vel->vel[0];
    gps2_data_msg.ned_vel_east            = vel->vel[1];
    gps2_data_msg.ned_vel_down            = vel->vel[2];
    gps2_data_msg.vdop                    = vel->sAcc * 100;

    // Find the second GPS_TYPE_EXTERNAL_AHRS instance and deliver GPS2 data to it
    uint8_t n_external = 0;
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (AP::gps().get_type(i) == AP_GPS::GPS_TYPE_EXTERNAL_AHRS) {
            if (++n_external == 2) {
                AP::gps().handle_external(gps2_data_msg, i);
                break;
            }
        }
    }
}

// Condensed nvm_flash_cfg_t from data_sets.h — only the fields needed to reach ioConfig.
struct PACKED nvm_flash_cfg_t {
    uint32_t    size;
    uint32_t    checksum;
    uint32_t    key;
    uint32_t    startupImuDtMs;
    uint32_t    startupNavDtMs;
    uint32_t    ser0BaudRate;
    uint32_t    ser1BaudRate;
    float       insRotation[3];
    float       insOffset[3];
    float       gps1AntOffset[3];
    uint8_t     dynamicModel;
    uint8_t     debug;
    uint16_t    gnssSatSigConst;
    uint32_t    sysCfgBits;
    double      refLla[3];
    double      lastLla[3];
    uint32_t    lastLlaTimeOfWeekMs;
    uint32_t    lastLlaWeek;
    float       lastLlaUpdateDistance;
    uint32_t    ioConfig;
};

void AP_ExternalAHRS_InertialSense::handle_flash_config_message(const uint8_t *raw, uint16_t size)
{
    if (size < sizeof(nvm_flash_cfg_t)) {
        return;
    }
    nvm_flash_cfg_t cfg;
    memcpy(&cfg, raw, sizeof(cfg));
    _num_gps_sensors = (IO_CONFIG_GPS2_TYPE(cfg.ioConfig) != 0) ? 2 : 1;
}

void AP_ExternalAHRS_InertialSense::handle_magnetometer_message(magnetometer_t* _mag)
{
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f{_mag->mag[0], _mag->mag[1], _mag->mag[2]};

    AP::compass().handle_external(mag);
#endif
}

void AP_ExternalAHRS_InertialSense::handle_barometer_message(barometer_t* bar)
{
#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = bar->bar * 1000.0f;
    baro.temperature = bar->barTemp;

    AP::baro().handle_external(baro);
#endif
}

void AP_ExternalAHRS_InertialSense::handle_inl2_ned_sigma_message(inl2_ned_sigma_t *sigmas)
{
    float pos_std = Vector3f(sigmas->StdPosNed[0], sigmas->StdPosNed[1], sigmas->StdPosNed[2]).length();
    float vel_std = Vector3f(sigmas->StdVelNed[0], sigmas->StdVelNed[1], sigmas->StdVelNed[2]).length();

    pos_cov = pos_std * pos_std;
    vel_cov = vel_std * vel_std;
    hgt_cov = pos_std * pos_std;
}

void AP_ExternalAHRS_InertialSense::handle_dev_info_message(dev_info_t *dev_info)
{
    dev_info_populate_missing_hardware(dev_info);

    const char *hardware_type;
    switch (dev_info->hardwareType) {
    case 1:
        hardware_type = "uINS";
        break;

    case 2:
        hardware_type = "EVB";
        break;

    case 3:
        hardware_type = "IMX";
        break;

    case 4:
        hardware_type = "GPX";
        break;

    default:
        hardware_type = "unknown";
    }

    hal.util->snprintf(_name, sizeof(_name), "%s %s", dev_info->manufacturer, hardware_type);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IS: %s SN:%" PRIu32, _name, dev_info->serialNumber);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IS: HW %u.%u.%u FW %u.%u.%u",
                  dev_info->hardwareVer[0], dev_info->hardwareVer[1], dev_info->hardwareVer[2],
                  dev_info->firmwareVer[0], dev_info->firmwareVer[1], dev_info->firmwareVer[2]);
}

void AP_ExternalAHRS_InertialSense::handle_bit_message(bit_t* bit)
{
    if (bit->state != BIT_STATE_DONE) {
        return;
    }

    if (bit->hdwBitStatus & HDW_BIT_FAILED_MASK) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IS: hardware BIT failed (0x%08" PRIx32 ")", bit->hdwBitStatus);
        return;
    }

    if (bit->calBitStatus & CAL_BIT_FAILED_MASK) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "IS: calibration BIT failed (0x%08" PRIx32 ")", bit->calBitStatus);
        return;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
}

void AP_ExternalAHRS_InertialSense::update()
{
    if (!check_uart()) {
        hal.scheduler->delay_microseconds(100);
    }
}

int AP_ExternalAHRS_InertialSense::parse_isb_data(void* ctx, p_data_t* data, port_handle_t port)
{
    switch (data->hdr.id)
    {
    case DID_INS_3:
        handle_ins3_message((ins_3_t*)data->ptr);
        break;

    case DID_GPS1_POS:
        handle_gps_pos_message((gps_pos_t*)data->ptr);
        break;

    case DID_GPS1_VEL:
        handle_gps_vel_message((gps_vel_t*)data->ptr);
        break;

    case DID_GPS2_POS:
        handle_gps2_pos_message((gps_pos_t*)data->ptr);
        break;

    case DID_GPS2_VEL:
        handle_gps2_vel_message((gps_vel_t*)data->ptr);
        break;

    case DID_FLASH_CONFIG:
        handle_flash_config_message(data->ptr, data->hdr.size);
        break;

    case DID_MAGNETOMETER:
        handle_magnetometer_message((magnetometer_t *)data->ptr);
        break;

    case DID_INL2_NED_SIGMA:
        handle_inl2_ned_sigma_message((inl2_ned_sigma_t *)data->ptr);
        break;

    case DID_DEV_INFO:
        handle_dev_info_message((dev_info_t*)data->ptr);
        break;

    case DID_BIT:
        handle_bit_message((bit_t*)data->ptr);
        break;

    default:
        break;
    }

    return 0;
}

bool AP_ExternalAHRS_InertialSense::check_uart()
{
    if (!initialized) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (!uart->available())
        return false;

    auto len = uart->read(buffer, MIN(uart->available(), 1024u));
    is_comm_buffer_parse_messages(buffer, len, &comm);

    return true;
}

void AP_ExternalAHRS_InertialSense::update_thread()
{
    if (!initialized) {
        initialize();
    }

    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

uint8_t AP_ExternalAHRS_InertialSense::num_gps_sensors(void) const
{
    return _num_gps_sensors;
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
