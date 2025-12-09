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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>

/*
 * Inertial Sense SDK types, condensed from ISConstants.h, base_port.h,
 * data_sets.h, and ISComm.h. The original files are not included to minimise
 * the number of new files added to ArduPilot.
 */

// ===== Struct packing macros (from ISConstants.h) =====
#ifdef _MSC_VER
#  define IS_PUSH_PACK __pragma(pack(push, 1))
#  define IS_POP_PACK  __pragma(pack(pop))
#else
#  define IS_PUSH_PACK _Pragma("pack(push, 1)")
#  define IS_POP_PACK  _Pragma("pack(pop)")
#endif
#define IS_PACKED

// ===== base_port.h: opaque port handle =====
typedef void* port_handle_t;

// ===== data_sets.h: data identifier constants =====
typedef uint32_t eDataIDs;
#define DID_NULL                  (eDataIDs)0   /** NULL (INVALID) */
#define DID_DEV_INFO              (eDataIDs)1   /** (dev_info_t) Device information */
#define DID_PIMU                  (eDataIDs)3   /** (pimu_t) Preintegrated IMU */
#define DID_RMC                   (eDataIDs)9   /** (rmc_t) Realtime Message Controller */
#define DID_GPS1_POS              (eDataIDs)13  /** (gps_pos_t) GPS 1 position data */
#define DID_GPS1_RTK_POS_MISC     (eDataIDs)22  /** (gps_rtk_misc_t) RTK precision position */
#define DID_GPS1_VEL              (eDataIDs)30  /** (gps_vel_t) GPS 1 velocity data */
#define DID_MAGNETOMETER          (eDataIDs)52  /** (magnetometer_t) Magnetometer sensor */
#define DID_BAROMETER             (eDataIDs)53  /** (barometer_t) Barometric pressure sensor */
#define DID_BIT                   (eDataIDs)64  /** (bit_t) System built-in self-test */
#define DID_INS_3                 (eDataIDs)65  /** (ins_3_t) INS output: quaternion NED to body */
#define DID_INL2_NED_SIGMA        (eDataIDs)67  /** (inl2_ned_sigma_t) INL2 EKF std deviations */

#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN      24

// ===== data_sets.h: status flag enums =====

enum eInsStatusFlags
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

enum eGpsNavFixStatus
{
    GPS_NAV_FIX_NONE                    = (int)0x00000000,
    GPS_NAV_FIX_POSITIONING_3D          = (int)0x00000001,
    GPS_NAV_FIX_POSITIONING_RTK_FLOAT   = (int)0x00000002,
    GPS_NAV_FIX_POSITIONING_RTK_FIX     = (int)0x00000003,
};

enum eHdwStatusFlags
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

enum eGpsStatus
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
enum eIsHardwareType
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

enum eBitState
{
    BIT_STATE_OFF       = (int)0,
    BIT_STATE_DONE      = (int)1,
    BIT_STATE_RUNNING   = (int)6,
    BIT_STATE_FINISHING = (int)7,
};

enum eHdwBitStatusFlags
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

enum eCalBitStatusFlags
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

// ===== data_sets.h: RMC option/bit macros =====
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

// ===== data_sets.h: packed data structures =====

IS_PUSH_PACK

typedef struct IS_PACKED
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
} dev_info_t;

typedef struct IS_PACKED
{
    uint32_t    week;
    double      timeOfWeek;
    uint32_t    insStatus;
    uint32_t    hdwStatus;
    float       qn2b[4];
    float       uvw[3];
    double      lla[3];
    float       msl;
} ins_3_t;

typedef struct IS_PACKED
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
} gps_pos_t;

typedef struct IS_PACKED
{
    uint32_t    timeOfWeekMs;
    float       vel[3];
    float       sAcc;
    uint32_t    status;
} gps_vel_t;

typedef struct IS_PACKED
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
} gps_rtk_misc_t;

typedef struct IS_PACKED
{
    double      time;
    float       mag[3];
} magnetometer_t;

typedef struct IS_PACKED
{
    double      time;
    float       bar;
    float       mslBar;
    float       barTemp;
    float       humidity;
} barometer_t;

typedef struct IS_PACKED
{
    double      time;
    float       dt;
    uint32_t    status;
    float       theta[3];
    float       vel[3];
} pimu_t;

typedef struct IS_PACKED
{
    unsigned int    timeOfWeekMs;
    float           StdPosNed[3];
    float           StdVelNed[3];
    float           StdAttNed[3];
    float           StdAccBias[3];
    float           StdGyrBias[3];
    float           StdBarBias;
    float           StdMagDeclination;
} inl2_ned_sigma_t;

typedef struct IS_PACKED
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
} bit_t;

typedef struct IS_PACKED
{
    uint64_t    bits;
    uint32_t    options;
} rmc_t;

IS_POP_PACK

// ===== ISComm.h: protocol type enum =====

typedef enum
{
    _PTYPE_NONE                 = 0,
    _PTYPE_PARSE_ERROR          = 1,
    _PTYPE_INERTIAL_SENSE_ACK   = 2,
    _PTYPE_INERTIAL_SENSE_CMD   = 3,
    _PTYPE_INERTIAL_SENSE_DATA  = 4,
    _PTYPE_NMEA                 = 5,
    _PTYPE_UBLOX                = 6,
    _PTYPE_RTCM3                = 7,
    _PTYPE_SPARTN               = 8,
    _PTYPE_SONY                 = 9,
    _PTYPE_FIRST_DATA           = _PTYPE_INERTIAL_SENSE_DATA,
    _PTYPE_LAST_DATA            = _PTYPE_SONY,
    _PTYPE_SIZE                 = _PTYPE_LAST_DATA + 1,
} protocol_type_t;

typedef enum
{
    PKT_TYPE_INVALID                        = 0,
    PKT_TYPE_ACK                            = 1,
    PKT_TYPE_NACK                           = 2,
    PKT_TYPE_GET_DATA                       = 3,
    PKT_TYPE_DATA                           = 4,
    PKT_TYPE_SET_DATA                       = 5,
    PKT_TYPE_STOP_BROADCASTS_ALL_PORTS      = 6,
    PKT_TYPE_STOP_DID_BROADCAST             = 7,
    PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT   = 8,
    PKT_TYPE_COUNT                          = 9,
    PKT_TYPE_MAX_COUNT                      = 16,
    PKT_TYPE_MASK                           = 0x0F,
    ISB_FLAGS_MASK                          = 0xF0,
    ISB_FLAGS_EXTENDED_PAYLOAD              = 0x10,
    ISB_FLAGS_PAYLOAD_W_OFFSET              = 0x20,
} eISBPacketFlags;

typedef enum
{
    ENABLE_PROTOCOL_ISB     = (0x00000001 << _PTYPE_INERTIAL_SENSE_DATA),
    ENABLE_PROTOCOL_NMEA    = (0x00000001 << _PTYPE_NMEA),
    ENABLE_PROTOCOL_UBLOX   = (0x00000001 << _PTYPE_UBLOX),
    ENABLE_PROTOCOL_RTCM3   = (0x00000001 << _PTYPE_RTCM3),
    ENABLE_PROTOCOL_SPARTN  = (0x00000001 << _PTYPE_SPARTN),
    ENABLE_PROTOCOL_SONY    = (0x00000001 << _PTYPE_SONY),
} eProtocolMask;

#define MAX_DATASET_SIZE    1024
#ifndef PKT_BUF_SIZE
#  define PKT_BUF_SIZE      2048
#endif
// Only ISB is needed; keep macro for compatibility
#define DEFAULT_PROTO_MASK  ENABLE_PROTOCOL_ISB

// ===== ISComm.h: packed packet structures =====

IS_PUSH_PACK

typedef struct
{
    uint16_t    preamble;
    uint8_t     flags;
    uint8_t     id;
    uint16_t    payloadSize;
} packet_hdr_t;

typedef struct
{
    uint8_t     id;
    uint16_t    size;
    uint16_t    offset;
} p_data_hdr_t;

typedef struct
{
    uint8_t     *ptr;
    uint32_t    size;
} bufPtr_t;

typedef struct
{
    union
    {
        struct
        {
            packet_hdr_t    hdr;
            uint16_t        offset;
        };
        struct
        {
            uint16_t        preamble;
            uint8_t         flags;
            p_data_hdr_t    dataHdr;
        };
    };
    bufPtr_t    data;
    uint16_t    hdrCksum;
    uint16_t    checksum;
    uint16_t    size;
    uint16_t    id;
} packet_t;

typedef struct
{
    packet_hdr_t    hdr;
    union
    {
        uint8_t     data;
        uint16_t    offset;
    }               payload;
} packet_buf_t;

typedef struct
{
    p_data_hdr_t    hdr;
    uint8_t         *ptr;
} p_data_t;

typedef struct
{
    p_data_hdr_t    hdr;
    uint8_t         buf[MAX_DATASET_SIZE];
} p_data_buf_t;

typedef struct
{
    uint16_t    id;
    uint16_t    size;
    uint16_t    offset;
    uint16_t    period;
} p_data_get_t;

IS_POP_PACK

// ===== ISComm.h: parser and callback types =====

typedef enum {
    EPARSE_INVALID_PREAMBLE,
    EPARSE_INVALID_SIZE,
    EPARSE_INVALID_CHKSUM,
    EPARSE_INVALID_DATATYPE,
    EPARSE_MISSING_EOS_MARKER,
    EPARSE_INCOMPLETE_PACKET,
    EPARSE_INVALID_HEADER,
    EPARSE_INVALID_PAYLOAD,
    EPARSE_RXBUFFER_FLUSHED,
    EPARSE_STREAM_UNPARSABLE,
    NUM_EPARSE_ERRORS
} eParseErrorType;

typedef struct
{
    int16_t     state;
    uint16_t    size;
    uint32_t    timeMs;
} is_comm_parser_t;

typedef protocol_type_t (*pFnProcessPkt)(void*);

typedef int(*pfnIsCommPortWrite)(port_handle_t port, const uint8_t* buf, int len);
typedef int(*pfnIsCommPortRead)(port_handle_t port, uint8_t* buf, int bufLen);
typedef int(*pfnIsCommIsbDataHandler)(void* ctx, p_data_t* data, port_handle_t port);
typedef int(*pfnIsCommGenMsgHandler)(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);
typedef int(*pfnIsCommHandler)(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);

typedef struct
{
    uint8_t*    start;
    uint8_t*    end;
    uint32_t    size;
    uint8_t*    head;
    uint8_t*    tail;
    uint8_t*    scan;
    uint8_t*    scanPrior;
} is_comm_buffer_t;

typedef struct
{
    uint32_t                    protocolMask;
    void*                       context;
    pfnIsCommHandler            all;
    pfnIsCommIsbDataHandler     isbData;
    pfnIsCommGenMsgHandler      generic[_PTYPE_SIZE];
} is_comm_callbacks_t;

IS_PUSH_PACK

typedef struct
{
    is_comm_buffer_t    rxBuf;
    uint32_t            txPktCount;
    uint32_t            rxPktCount;
    uint32_t            rxErrorCount;
    eParseErrorType     rxErrorType;
    uint32_t            rxErrorTypeCount[NUM_EPARSE_ERRORS];
    pFnProcessPkt       processPkt;
    is_comm_parser_t    parser;
    uint32_t            ackNeeded;
    packet_t            rxPkt;
    uint8_t             rxErrorState;
    is_comm_callbacks_t cb;
} is_comm_instance_t;

IS_POP_PACK

#undef IS_PUSH_PACK
#undef IS_POP_PACK
#undef IS_PACKED

class AP_ExternalAHRS_InertialSense: public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override;

private:
    int initialize();
    void update_thread();
    bool check_uart();

    int stop_message_broadcasting();
    int enable_message_broadcasting();

    void handleIns3Message(ins_3_t* ins);
    void handleGpsPosMessage(gps_pos_t* pos);
    void handleGpsVelMessage(gps_vel_t* vel);
    void handleGpsRtkPosMiscMessage(gps_rtk_misc_t* misc);
    void handlePimuMessage(pimu_t* pimu);
    void handleMagnetometerMessage(magnetometer_t* mag);
    void handleBarometerMessage(barometer_t* bar);
    void handleInl2NedSigmaMessage(inl2_ned_sigma_t *sigmas);
    void handleDevInfoMessage(dev_info_t *dev_info);
    void handleBitMessage(bit_t* bit);
    int parseIsbData(void* ctx, p_data_t* data, port_handle_t port);

    // callback helper
    static AP_ExternalAHRS_InertialSense *instance;
    static int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
        return instance->parseIsbData(ctx, data, port);
    }
    int ppd_fd;

    bool initialized = false;
    bool _healthy = false;
    AP_GPS_FixType _fix_type = AP_GPS_FixType::NONE;

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;
    uint8_t buffer[1024];

    int imu_sample_duration = 20;

    uint32_t last_imu_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    uint8_t comm_buf[2048];
    is_comm_instance_t comm;

    float vel_cov;
    float pos_cov;
    float hgt_cov;

    AP_ExternalAHRS::gps_data_message_t gps_data_msg;

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
};

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
