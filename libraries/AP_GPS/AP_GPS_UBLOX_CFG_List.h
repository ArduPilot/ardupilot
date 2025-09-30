#include "AP_GPS_UBLOX_CFG_Keys.h"


/***********************************************************************
 * Configurations for F9 GPS                                           *
 ***********************************************************************/
/*
  config for F9 GPS in moving baseline base role
  See ZED-F9P integration manual section 3.1.5.6.1
 */
#define UBX_CFG_MB_BASE_UART1 \
 { ConfigKey::CFG_UART1OUTPROT_RTCM3X, 1}, \
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0},

#define UBX_CFG_MB_BASE_UART2 \
 { ConfigKey::CFG_UART2_ENABLED, 1}, \
 { ConfigKey::CFG_UART2_BAUDRATE, 460800}, \
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 1}, \
 { ConfigKey::CFG_UART1OUTPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 1}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, RTK_MB_RTCM_RATE}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0},

/*
  config for F9 GPS in moving baseline rover role
  See ZED-F9P integration manual section 3.1.5.6.1.
  Note that we list the RTCM msg types as 0 to prevent getting RTCM
  data from a GPS previously configured as a base
 */
#define UBX_CFG_MB_ROVER_UART1 \
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 1}, \
 { ConfigKey::CFG_UART2INPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0},

#define UBX_CFG_MB_ROVER_UART2 \
 { ConfigKey::CFG_UART2_ENABLED, 1}, \
 { ConfigKey::CFG_UART2_BAUDRATE, 460800}, \
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_UART2INPROT_RTCM3X, 1}, \
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 0}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1}, \
 { ConfigKey::CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART2, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 0}, \
 { ConfigKey::CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 0},


/************************************************************************
 * M10 specific configuration                                           *
 ************************************************************************/
 /*
  config changes for M10 in legacy driver
  we need to use B1C not B1 signal for Beidou on M10 to allow solid 5Hz,
  and also disable Glonass and enable QZSS
 */
#define UBX_CFG_M10 \
 { ConfigKey::CFG_SIGNAL_BDS_ENA, 1}, \
 { ConfigKey::CFG_SIGNAL_BDS_B1_ENA, 0}, \
 { ConfigKey::CFG_SIGNAL_BDS_B1C_ENA, 1}, \
 { ConfigKey::CFG_SIGNAL_GLO_ENA, 0}, \
 { ConfigKey::CFG_SIGNAL_QZSS_ENA, 1}, \
 { ConfigKey::CFG_SIGNAL_QZSS_L1CA_ENA, 1}, \
 { ConfigKey::CFG_SIGNAL_QZSS_L1S_ENA, 1}, \
 { ConfigKey::CFG_NAVSPG_DYNMODEL, 8}  // Air < 4g

/**********************************************************************
 * Constant Common Configurations                                              *
 **********************************************************************/
#define UBX_CFG_L5_OVRD_ENA \
    {ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 1}, \
    {ConfigKey::CFG_SIGNAL_GPS_L5_ENA, 1},

#define UBX_CFG_L5_OVRD_DIS \
    {ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 0}

#define UBX_CFG_COMMON_UART(X) \
    X(CFG_RATE,   NAV,                    uint16_t, 1) \
    X(CFG_MSGOUT, UBX_NAV_PVT_UART1,      uint8_t,  RATE_PVT) \
    X(CFG_MSGOUT, UBX_NAV_TIMEGPS_UART1,  uint8_t,  RATE_TIMEGPS) \
    X(CFG_MSGOUT, UBX_NAV_DOP_UART1,      uint8_t,  RATE_DOP) \
    X(CFG_MSGOUT, UBX_MON_RF_UART1,       uint8_t,  RATE_HW)

#define UBX_CFG_COMMON_UART1(X) \
    X(CFG_RATE,   NAV,                    uint16_t, 1) \
    X(CFG_MSGOUT, UBX_NAV_PVT_UART1,      uint8_t,  RATE_PVT) \
    X(CFG_MSGOUT, UBX_NAV_TIMEGPS_UART1,  uint8_t,  RATE_TIMEGPS) \
    X(CFG_MSGOUT, UBX_NAV_DOP_UART1,      uint8_t,  RATE_DOP) \
    X(CFG_MSGOUT, UBX_MON_RF_UART1,       uint8_t,  RATE_HW) \
    X(CFG_MSGOUT, UBX_NAV_PVT_UART2,      uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_NAV_TIMEGPS_UART2,  uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_NAV_DOP_UART2,      uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_MON_RF_UART2,       uint8_t,  0U)

#define UBX_CFG_COMMON_UART2(X) \
    X(CFG_RATE,   NAV,                    uint16_t, 1) \
    X(CFG_MSGOUT, UBX_NAV_PVT_UART2,      uint8_t,  RATE_PVT) \
    X(CFG_MSGOUT, UBX_NAV_TIMEGPS_UART2,  uint8_t,  RATE_TIMEGPS) \
    X(CFG_MSGOUT, UBX_NAV_DOP_UART2,      uint8_t,  RATE_DOP) \
    X(CFG_MSGOUT, UBX_MON_RF_UART2,       uint8_t,  RATE_HW) \
    X(CFG_MSGOUT, UBX_NAV_PVT_UART1,      uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_NAV_TIMEGPS_UART1,  uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_NAV_DOP_UART1,      uint8_t,  0U) \
    X(CFG_MSGOUT, UBX_MON_RF_UART1,       uint8_t,  0U)
