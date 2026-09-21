#pragma once
#include "AP_GPS_UBLOX_CFG_Keys.h"
#include "AP_GPS_config.h"
#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include "AP_GPS.h"

// key-value pair for u-blox CFG-VALSET configuration lists
struct PACKED ubx_config_list {
    AP::UBXConfigKey key;
    uint32_t value;
};

// output rates for message scheduling
#define RATE_POSLLH 1
#define RATE_STATUS 1
#define RATE_SOL 1
#define RATE_TIMEGPS 5
#define RATE_PVT 1
#define RATE_VELNED 1
#define RATE_DOP 1
#define RATE_HW 5
#define RATE_HW2 5
#define RATE_TIM_TM2 1

// epochs per RTCM output for moving baseline
#define RTK_MB_RTCM_RATE 1

/**********************************************************************
 * Common UART config arrays (CFGv2 driver)                           *
 **********************************************************************/
static constexpr ubx_config_list config_common_uart[] = {
    { AP::UBXConfigKey::CFG_RATE_NAV,                    1 },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_PVT_UART1,   RATE_PVT },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1, RATE_TIMEGPS },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_DOP_UART1,   RATE_DOP },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_MON_RF_UART1,    RATE_HW },
};

static constexpr ubx_config_list config_common_uart1[] = {
    { AP::UBXConfigKey::CFG_RATE_NAV,                    1 },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_PVT_UART1,   RATE_PVT },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1, RATE_TIMEGPS },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_DOP_UART1,   RATE_DOP },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_MON_RF_UART1,    RATE_HW },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_PVT_UART2,   0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_TIMEGPS_UART2, 0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_DOP_UART2,   0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_MON_RF_UART2,    0U },
};

static constexpr ubx_config_list config_common_uart2[] = {
    { AP::UBXConfigKey::CFG_UART2_ENABLED,               1 },
    { AP::UBXConfigKey::CFG_RATE_NAV,                    1 },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_PVT_UART2,   RATE_PVT },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_TIMEGPS_UART2, RATE_TIMEGPS },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_DOP_UART2,   RATE_DOP },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_MON_RF_UART2,    RATE_HW },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_PVT_UART1,   0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1, 0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_DOP_UART1,   0U },
    { AP::UBXConfigKey::CFG_MSGOUT_UBX_MON_RF_UART1,    0U },
};

/*
  config for F9 GPS in moving baseline base role
  See ZED-F9P integration manual section 3.1.5.6.1
 */
#if GPS_MOVING_BASELINE

// MB base on UART1
static constexpr ubx_config_list config_MB_Base_uart1[] = {
 { AP::UBXConfigKey::CFG_UART1OUTPROT_RTCM3X, 1},
 { AP::UBXConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
};

// MB base on UART2
static constexpr ubx_config_list config_MB_Base_uart2[] = {
 { AP::UBXConfigKey::CFG_UART2_ENABLED, 1},
 { AP::UBXConfigKey::CFG_UART2_BAUDRATE, 460800},
 { AP::UBXConfigKey::CFG_UART2OUTPROT_RTCM3X, 1},
 { AP::UBXConfigKey::CFG_UART1OUTPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_UART1INPROT_RTCM3X, 1},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, RTK_MB_RTCM_RATE},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
};

// MB rover on UART1
// RTCM msg types set to 0 to prevent getting RTCM data from a GPS previously configured as base
static constexpr ubx_config_list config_MB_Rover_uart1[] = {
 { AP::UBXConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_UART1INPROT_RTCM3X, 1},
 { AP::UBXConfigKey::CFG_UART2INPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
};

// MB rover on UART2
static constexpr ubx_config_list config_MB_Rover_uart2[] = {
 { AP::UBXConfigKey::CFG_UART2_ENABLED, 1},
 { AP::UBXConfigKey::CFG_UART2_BAUDRATE, 460800},
 { AP::UBXConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_UART2INPROT_RTCM3X, 1},
 { AP::UBXConfigKey::CFG_UART1INPROT_RTCM3X, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1},
 { AP::UBXConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { AP::UBXConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
};

#endif // GPS_MOVING_BASELINE

/**********************************************************************
 * M10 specific configuration                                        *
 * Use B1C not B1 for Beidou on M10 to allow solid 5Hz,              *
 * disable Glonass, enable QZSS                                      *
 **********************************************************************/
static constexpr ubx_config_list config_M10[] = {
    { AP::UBXConfigKey::CFG_SIGNAL_BDS_ENA,       1 },
    { AP::UBXConfigKey::CFG_SIGNAL_BDS_B1_ENA,    0 },
    { AP::UBXConfigKey::CFG_SIGNAL_BDS_B1C_ENA,   1 },
    { AP::UBXConfigKey::CFG_SIGNAL_GLO_ENA,       0 },
    { AP::UBXConfigKey::CFG_SIGNAL_QZSS_ENA,      1 },
    { AP::UBXConfigKey::CFG_SIGNAL_QZSS_L1CA_ENA, 1 },
    { AP::UBXConfigKey::CFG_SIGNAL_QZSS_L1S_ENA,  1 },
    { AP::UBXConfigKey::CFG_NAVSPG_DYNMODEL,       8 },  // Air < 4g
};

/**********************************************************************
 * L5 override configuration                                          *
 **********************************************************************/
static constexpr ubx_config_list config_L5_ovrd_ena[] = {
    { AP::UBXConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 1 },
    { AP::UBXConfigKey::CFG_SIGNAL_GPS_L5_ENA,     1 },
};

static constexpr ubx_config_list config_L5_ovrd_dis[] = {
    { AP::UBXConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 0 },
};
