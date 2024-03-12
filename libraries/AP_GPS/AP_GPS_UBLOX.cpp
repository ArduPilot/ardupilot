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

//
//  u-blox GPS driver for ArduPilot
//	Origin code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//  Substantially rewritten for new GPS driver structure by Andrew Tridgell
//
#include "AP_GPS_UBLOX.h"

#if AP_GPS_UBLOX_ENABLED

#include "AP_GPS.h"
#include <AP_HAL/Util.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "RTCM3_Parser.h"
#include <stdio.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
    #define UBLOX_SPEED_CHANGE  1
#else
    #define UBLOX_SPEED_CHANGE 0
#endif


#define UBLOX_DEBUGGING 0
#define UBLOX_FAKE_3DLOCK 0
#ifndef CONFIGURE_PPS_PIN
#define CONFIGURE_PPS_PIN 0
#endif

// this is number of epochs per output. A higher value will reduce
// the uart bandwidth needed and allow for higher latency
#define RTK_MB_RTCM_RATE 1

// use this to enable debugging of moving baseline configs
#define UBLOX_MB_DEBUGGING 0

// debug VALGET/VALSET configuration
#define UBLOX_CFG_DEBUGGING 0

extern const AP_HAL::HAL& hal;

#if UBLOX_DEBUGGING
#if defined(HAL_BUILD_AP_PERIPH)
 extern "C" {
   void can_printf(const char *fmt, ...);
 }
 # define Debug(fmt, args ...)  do {can_printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args);} while(0)
#else
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#endif
#else
 # define Debug(fmt, args ...)
#endif

#if UBLOX_MB_DEBUGGING
#if defined(HAL_BUILD_AP_PERIPH)
 extern "C" {
   void can_printf(const char *fmt, ...);
 }
 # define MB_Debug(fmt, args ...)  do {can_printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args);} while(0)
#else
 # define MB_Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#endif
#else
 # define MB_Debug(fmt, args ...)
#endif

#if UBLOX_CFG_DEBUGGING
#if defined(HAL_BUILD_AP_PERIPH)
 extern "C" {
   void can_printf(const char *fmt, ...);
 }
 # define CFG_Debug(fmt, args ...)  do {can_printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args);} while(0)
#else
 # define CFG_Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#endif
#else
 # define CFG_Debug(fmt, args ...)
#endif

AP_GPS_UBLOX::AP_GPS_UBLOX(AP_GPS &_gps,
                           AP_GPS::Params &_params,
                           AP_GPS::GPS_State &_state,
                           AP_HAL::UARTDriver *_port,
                           AP_GPS::GPS_Role _role) :
    AP_GPS_Backend(_gps, _params, _state, _port),
    _next_message(STEP_PVT),
    _ublox_port(255),
    _unconfigured_messages(CONFIG_ALL),
    _hardware_generation(UBLOX_UNKNOWN_HARDWARE_GENERATION),
    next_fix(AP_GPS::NO_FIX),
    noReceivedHdop(true),
    role(_role)
{
    // stop any config strings that are pending
    gps.send_blob_start(state.instance, nullptr, 0);

    // start the process of updating the GPS rates
    _request_next_config();

#if CONFIGURE_PPS_PIN
    _unconfigured_messages |= CONFIG_TP5;
#endif

#if GPS_MOVING_BASELINE
    if (role == AP_GPS::GPS_ROLE_MB_BASE && !mb_use_uart2()) {
        rtcm3_parser = new RTCM3_Parser;
        if (rtcm3_parser == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "u-blox %d: failed RTCMv3 parser allocation", state.instance + 1);
        }
        _unconfigured_messages |= CONFIG_RTK_MOVBASE;
    }
    if (role == AP_GPS::GPS_ROLE_MB_ROVER) {
        _unconfigured_messages |= CONFIG_RTK_MOVBASE;
        state.gps_yaw_configured = true;
    }
#endif
}

AP_GPS_UBLOX::~AP_GPS_UBLOX()
{
#if GPS_MOVING_BASELINE
    delete rtcm3_parser;
#endif
}

#if GPS_MOVING_BASELINE
/*
  config for F9 GPS in moving baseline base role
  See ZED-F9P integration manual section 3.1.5.6.1
 */
const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_MB_Base_uart1[] {
 { ConfigKey::CFG_UART1OUTPROT_RTCM3X, 1},
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART1, 0},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART1, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
};

const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_MB_Base_uart2[] {
 { ConfigKey::CFG_UART2_ENABLED, 1},
 { ConfigKey::CFG_UART2_BAUDRATE, 460800},
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 1},
 { ConfigKey::CFG_UART1OUTPROT_RTCM3X, 0},
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 1},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART2, RTK_MB_RTCM_RATE},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
};


/*
  config for F9 GPS in moving baseline rover role
  See ZED-F9P integration manual section 3.1.5.6.1.
  Note that we list the RTCM msg types as 0 to prevent getting RTCM
  data from a GPS previously configured as a base
 */
const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_MB_Rover_uart1[] {
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 1},
 { ConfigKey::CFG_UART2INPROT_RTCM3X, 0},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART1, 1},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
};

const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_MB_Rover_uart2[] {
 { ConfigKey::CFG_UART2_ENABLED, 1},
 { ConfigKey::CFG_UART2_BAUDRATE, 460800},
 { ConfigKey::CFG_UART2OUTPROT_RTCM3X, 0},
 { ConfigKey::CFG_UART2INPROT_RTCM3X, 1},
 { ConfigKey::CFG_UART1INPROT_RTCM3X, 0},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART1, 1},
 { ConfigKey::MSGOUT_UBX_NAV_RELPOSNED_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART2, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_0_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE4072_1_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1077_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1087_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1097_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1127_UART1, 0},
 { ConfigKey::MSGOUT_RTCM_3X_TYPE1230_UART1, 0},
};
#endif // GPS_MOVING_BASELINE

/*
  config changes for M10
  we need to use B1C not B1 signal for Beidou on M10 to allow solid 5Hz,
  and also disable Glonass and enable QZSS
 */
const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_M10[] {
 { ConfigKey::CFG_SIGNAL_BDS_ENA, 1},
 { ConfigKey::CFG_SIGNAL_BDS_B1_ENA, 0},
 { ConfigKey::CFG_SIGNAL_BDS_B1C_ENA, 1},
 { ConfigKey::CFG_SIGNAL_GLO_ENA, 0},
 { ConfigKey::CFG_SIGNAL_QZSS_ENA, 1},
 { ConfigKey::CFG_SIGNAL_QZSS_L1CA_ENA, 1},
 { ConfigKey::CFG_SIGNAL_QZSS_L1S_ENA, 1},
 { ConfigKey::CFG_NAVSPG_DYNMODEL, 8}, // Air < 4g
};


/*
  config changes for L5 modules
*/
const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_L5_ovrd_ena[] {
    {ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 1},
    {ConfigKey::CFG_SIGNAL_GPS_L5_ENA, 1},
};

const AP_GPS_UBLOX::config_list AP_GPS_UBLOX::config_L5_ovrd_dis[] {
    {ConfigKey::CFG_SIGNAL_L5_HEALTH_OVRD, 0},
};

void
AP_GPS_UBLOX::_request_next_config(void)
{
    // don't request config if we shouldn't configure the GPS
    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        return;
    }

    // Ensure there is enough space for the largest possible outgoing message
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+sizeof(struct ubx_cfg_nav_rate)+2)) {
        // not enough space - do it next time
        return;
    }

    if (_unconfigured_messages == CONFIG_RATE_SOL && havePvtMsg) {
        /*
          we don't need SOL if we have PVT and TIMEGPS. This is needed
          as F9P doesn't support the SOL message
         */
        _unconfigured_messages &= ~CONFIG_RATE_SOL;
    }

    Debug("Unconfigured messages: 0x%x Current message: %u\n", (unsigned)_unconfigured_messages, (unsigned)_next_message);

    // check AP_GPS_UBLOX.h for the enum that controls the order.
    // This switch statement isn't maintained against the enum in order to reduce code churn
    switch (_next_message++) {
    case STEP_PVT:
        if(!_request_message_rate(CLASS_NAV, MSG_PVT)) {
            _next_message--;
        }
        break;
    case STEP_TIMEGPS:
        if(!_request_message_rate(CLASS_NAV, MSG_TIMEGPS)) {
            _next_message--;
        }
        break;
    case STEP_PORT:
        _request_port();
        break;
    case STEP_POLL_SVINFO:
        // not required once we know what generation we are on
        if(_hardware_generation == UBLOX_UNKNOWN_HARDWARE_GENERATION) {
            if (!_send_message(CLASS_NAV, MSG_NAV_SVINFO, 0, 0)) {
                _next_message--;
            }
        }
        break;
    case STEP_POLL_SBAS:
        if (gps._sbas_mode != AP_GPS::SBAS_Mode::DoNotChange) {
            _send_message(CLASS_CFG, MSG_CFG_SBAS, nullptr, 0);
        } else {
            _unconfigured_messages &= ~CONFIG_SBAS;
        }
        break;
    case STEP_POLL_NAV:
        if (!_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_POLL_GNSS:
        if (!_send_message(CLASS_CFG, MSG_CFG_GNSS, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_POLL_TP5:
#if CONFIGURE_PPS_PIN
        if (!_send_message(CLASS_CFG, MSG_CFG_TP5, nullptr, 0)) {
            _next_message--;
        }
#endif
        break;
    case STEP_NAV_RATE:
        if (!_send_message(CLASS_CFG, MSG_CFG_RATE, nullptr, 0)) {
            _next_message--;
        }
        break;
    case STEP_POSLLH:
        if(!_request_message_rate(CLASS_NAV, MSG_POSLLH)) {
            _next_message--;
        }
        break;
    case STEP_STATUS:
        if(!_request_message_rate(CLASS_NAV, MSG_STATUS)) {
            _next_message--;
        }
        break;
    case STEP_SOL:
        if(!_request_message_rate(CLASS_NAV, MSG_SOL)) {
            _next_message--;
        }
        break;
    case STEP_VELNED:
        if(!_request_message_rate(CLASS_NAV, MSG_VELNED)) {
            _next_message--;
        }
        break;
    case STEP_DOP:
       if(! _request_message_rate(CLASS_NAV, MSG_DOP)) {
            _next_message--;
        }
        break;
    case STEP_MON_HW:
        if(!_request_message_rate(CLASS_MON, MSG_MON_HW)) {
            _next_message--;
        }
        break;
    case STEP_MON_HW2:
        if(!_request_message_rate(CLASS_MON, MSG_MON_HW2)) {
            _next_message--;
        }
        break;
    case STEP_RAW:
#if UBLOX_RXM_RAW_LOGGING
        if(gps._raw_data == 0) {
            _unconfigured_messages &= ~CONFIG_RATE_RAW;
        } else if(!_request_message_rate(CLASS_RXM, MSG_RXM_RAW)) {
            _next_message--;
        }
#else
        _unconfigured_messages & = ~CONFIG_RATE_RAW;
#endif
        break;
    case STEP_RAWX:
#if UBLOX_RXM_RAW_LOGGING
        if(gps._raw_data == 0) {
            _unconfigured_messages &= ~CONFIG_RATE_RAW;
        } else if(!_request_message_rate(CLASS_RXM, MSG_RXM_RAWX)) {
            _next_message--;
        }
#else
        _unconfigured_messages & = ~CONFIG_RATE_RAW;
#endif
        break;
    case STEP_VERSION:
        if(!_have_version && !hal.util->get_soft_armed()) {
            _request_version();
        } else {
            _unconfigured_messages &= ~CONFIG_VERSION;
        }
        break;
    case STEP_TMODE:
        if (supports_F9_config()) {
            if (!_configure_valget(ConfigKey::TMODE_MODE)) {
                _next_message--;
            }
        }
        break;
    case STEP_RTK_MOVBASE:
#if GPS_MOVING_BASELINE
        if (supports_F9_config()) {
            static_assert(sizeof(active_config.done_mask)*8 >= ARRAY_SIZE(config_MB_Base_uart1), "done_mask too small, base1");
            static_assert(sizeof(active_config.done_mask)*8 >= ARRAY_SIZE(config_MB_Base_uart2), "done_mask too small, base2");
            static_assert(sizeof(active_config.done_mask)*8 >= ARRAY_SIZE(config_MB_Rover_uart1), "done_mask too small, rover1");
            static_assert(sizeof(active_config.done_mask)*8 >= ARRAY_SIZE(config_MB_Rover_uart2), "done_mask too small, rover2");
            if (role == AP_GPS::GPS_ROLE_MB_BASE) {
                const config_list *list = mb_use_uart2()?config_MB_Base_uart2:config_MB_Base_uart1;
                uint8_t list_length = mb_use_uart2()?ARRAY_SIZE(config_MB_Base_uart2):ARRAY_SIZE(config_MB_Base_uart1);
                if (!_configure_config_set(list, list_length, CONFIG_RTK_MOVBASE)) {
                    _next_message--;
                }
            }
            if (role == AP_GPS::GPS_ROLE_MB_ROVER) {
                const config_list *list = mb_use_uart2()?config_MB_Rover_uart2:config_MB_Rover_uart1;
                uint8_t list_length = mb_use_uart2()?ARRAY_SIZE(config_MB_Rover_uart2):ARRAY_SIZE(config_MB_Rover_uart1);
                if (!_configure_config_set(list, list_length, CONFIG_RTK_MOVBASE)) {
                    _next_message--;
                }
            }
        }
#endif
        break;
    case STEP_TIM_TM2:
#if UBLOX_TIM_TM2_LOGGING
        if(!_request_message_rate(CLASS_TIM, MSG_TIM_TM2)) {
            _next_message--;
        }
#else
        _unconfigured_messages &= ~CONFIG_TIM_TM2;
#endif
        break;


    case STEP_M10: {
        if (_hardware_generation == UBLOX_M10) {
            // special handling of M10 config
            const config_list *list = config_M10;
            const uint8_t list_length = ARRAY_SIZE(config_M10);
            Debug("Sending M10 settings");
            if (!_configure_config_set(list, list_length, CONFIG_M10, UBX_VALSET_LAYER_RAM | UBX_VALSET_LAYER_BBR)) {
                _next_message--;
            }
        }
        break;
    }

    case STEP_L5: {
        if (supports_l5 && option_set(AP_GPS::DriverOptions::GPSL5HealthOverride)) {
            const config_list *list = config_L5_ovrd_ena;
            const uint8_t list_length = ARRAY_SIZE(config_L5_ovrd_ena);
            if (!_configure_config_set(list, list_length, CONFIG_L5, UBX_VALSET_LAYER_RAM | UBX_VALSET_LAYER_BBR)) {
                _next_message--;
            }
        } else if (supports_l5 && !option_set(AP_GPS::DriverOptions::GPSL5HealthOverride)) {
            const config_list *list = config_L5_ovrd_dis;
            const uint8_t list_length = ARRAY_SIZE(config_L5_ovrd_dis);
            if (!_configure_config_set(list, list_length, CONFIG_L5, UBX_VALSET_LAYER_RAM | UBX_VALSET_LAYER_BBR)) {
                _next_message--;
            }
        }
        break;
    }

    default:
        // this case should never be reached, do a full reset if it is hit
        _next_message = STEP_PVT;
        break;
    }
}

void
AP_GPS_UBLOX::_verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    uint8_t desired_rate;
    uint32_t config_msg_id;
    switch(msg_class) {
    case CLASS_NAV:
        switch(msg_id) {
        case MSG_POSLLH:
            desired_rate = havePvtMsg ? 0 : RATE_POSLLH;
            config_msg_id = CONFIG_RATE_POSLLH;
            break;
        case MSG_STATUS:
            desired_rate = havePvtMsg ? 0 : RATE_STATUS;
            config_msg_id = CONFIG_RATE_STATUS;
            break;
        case MSG_SOL:
            desired_rate = havePvtMsg ? 0 : RATE_SOL;
            config_msg_id = CONFIG_RATE_SOL;
            break;
        case MSG_PVT:
            desired_rate = RATE_PVT;
            config_msg_id = CONFIG_RATE_PVT;
            break;
        case MSG_TIMEGPS:
            desired_rate = RATE_TIMEGPS;
            config_msg_id = CONFIG_RATE_TIMEGPS;
            break;
        case MSG_VELNED:
            desired_rate = havePvtMsg ? 0 : RATE_VELNED;
            config_msg_id = CONFIG_RATE_VELNED;
            break;
        case MSG_DOP:
            desired_rate = RATE_DOP;
            config_msg_id = CONFIG_RATE_DOP;
            break;
        default:
            return;
        }
        break;
    case CLASS_MON:
        switch(msg_id) {
        case MSG_MON_HW:
            desired_rate = RATE_HW;
            config_msg_id = CONFIG_RATE_MON_HW;
            break;
        case MSG_MON_HW2:
            desired_rate = RATE_HW2;
            config_msg_id = CONFIG_RATE_MON_HW2;
            break;
        default:
            return;
        }
        break;
#if UBLOX_RXM_RAW_LOGGING
    case CLASS_RXM:
        switch(msg_id) {
        case MSG_RXM_RAW:
            desired_rate = gps._raw_data;
            config_msg_id = CONFIG_RATE_RAW;
            break;
        case MSG_RXM_RAWX:
            desired_rate = gps._raw_data;
            config_msg_id = CONFIG_RATE_RAW;
            break;
        default:
            return;
        }
        break;
#endif // UBLOX_RXM_RAW_LOGGING
#if UBLOX_TIM_TM2_LOGGING
    case CLASS_TIM:
        if (msg_id == MSG_TIM_TM2) {
            desired_rate = RATE_TIM_TM2;
            config_msg_id = CONFIG_TIM_TM2;
            break;
        }
        return;
#endif // UBLOX_TIM_TM2_LOGGING
    default:
        return;
    }

    if (rate == desired_rate) {
        // coming in at correct rate; mark as configured
        _unconfigured_messages &= ~config_msg_id;
        return;
    }

    // coming in at wrong rate; try to configure it
    _configure_message_rate(msg_class, msg_id, desired_rate);
    _unconfigured_messages |= config_msg_id;
    _cfg_needs_save = true;
}

// Requests the ublox driver to identify what port we are using to communicate
void
AP_GPS_UBLOX::_request_port(void)
{
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+2)) {
        // not enough space - do it next time
        return;
    }
    _send_message(CLASS_CFG, MSG_CFG_PRT, nullptr, 0);
}

// Ensure there is enough space for the largest possible outgoing message
// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
AP_GPS_UBLOX::read(void)
{
    bool parsed = false;
    uint32_t millis_now = AP_HAL::millis();

    // walk through the gps configuration at 1 message per second
    if (millis_now - _last_config_time >= _delay_time) {
        _request_next_config();
        _last_config_time = millis_now;
        if (_unconfigured_messages) { // send the updates faster until fully configured
            if (!havePvtMsg && (_unconfigured_messages & CONFIG_REQUIRED_INITIAL)) {
                _delay_time = 300;
            } else {
                _delay_time = 750;
            }
        } else {
            _delay_time = 2000;
        }
    }

    if(!_unconfigured_messages && gps._save_config && !_cfg_saved &&
       _num_cfg_save_tries < 5 && (millis_now - _last_cfg_sent_time) > 5000 &&
       !hal.util->get_soft_armed()) {
        //save the configuration sent until now
        if (gps._save_config == 1 ||
            (gps._save_config == 2 && _cfg_needs_save)) {
            _save_cfg();
        }
    }

    const uint16_t numc = MIN(port->available(), 8192U);
    for (uint16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        uint8_t data;
        if (!port->read(data)) {
            break;
        }
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&data, 1);
#endif

#if GPS_MOVING_BASELINE
        if (rtcm3_parser) {
            if (rtcm3_parser->read(data)) {
                // we've found a RTCMv3 packet. We stop parsing at
                // this point and reset u-blox parse state. We need to
                // stop parsing to give the higher level driver a
                // chance to send the RTCMv3 packet to another (rover)
                // GPS
                _step = 0;
                break;
            }
        }
#endif

	reset:
        switch(_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            Debug("reset %u", __LINE__);
            FALLTHROUGH;
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        // We sniff the class and message ID to decide whether we
        // are going to gather the message bytes or just discard
        // them.
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;                       // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                     // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte

            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > sizeof(_buffer)) {
                Debug("large payload %u", (unsigned)_payload_length);
                // assume any payload bigger then what we know about is noise
                _payload_length = 0;
                _step = 0;
				goto reset;
            }
            _payload_counter = 0;                       // prepare to receive payload
            if (_payload_length == 0) {
                // bypass payload and go straight to checksum
                _step++;
            }
            break;

        // Receive message data
        //
        case 6:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 7:
            _step++;
            if (_ck_a != data) {
                Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;
				goto reset;
            }
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
                Debug("bad ckb %x should be %x", data, _ck_b);
                break;                                                  // bad checksum
            }

#if GPS_MOVING_BASELINE
            if (rtcm3_parser) {
                // this is a uBlox packet, discard any partial RTCMv3 state
                rtcm3_parser->reset();
            }
#endif
            if (_parse_gps()) {
                parsed = true;
            }
            break;
        }
    }
    return parsed;
}

// Private Methods /////////////////////////////////////////////////////////////
void AP_GPS_UBLOX::log_mon_hw(void)
{
#if HAL_LOGGING_ENABLED
    if (!should_log()) {
        return;
    }
    struct log_Ubx1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_UBX1_MSG),
        time_us    : AP_HAL::micros64(),
        instance   : state.instance,
        noisePerMS : _buffer.mon_hw_60.noisePerMS,
        jamInd     : _buffer.mon_hw_60.jamInd,
        aPower     : _buffer.mon_hw_60.aPower,
        agcCnt     : _buffer.mon_hw_60.agcCnt,
        config     : _unconfigured_messages,
    };
    if (_payload_length == 68) {
        pkt.noisePerMS = _buffer.mon_hw_68.noisePerMS;
        pkt.jamInd     = _buffer.mon_hw_68.jamInd;
        pkt.aPower     = _buffer.mon_hw_68.aPower;
        pkt.agcCnt     = _buffer.mon_hw_68.agcCnt;
    }
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}

void AP_GPS_UBLOX::log_mon_hw2(void)
{
#if HAL_LOGGING_ENABLED
    if (!should_log()) {
        return;
    }

    struct log_Ubx2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_UBX2_MSG),
        time_us   : AP_HAL::micros64(),
        instance  : state.instance,
        ofsI      : _buffer.mon_hw2.ofsI,
        magI      : _buffer.mon_hw2.magI,
        ofsQ      : _buffer.mon_hw2.ofsQ,
        magQ      : _buffer.mon_hw2.magQ,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}

#if UBLOX_TIM_TM2_LOGGING
void AP_GPS_UBLOX::log_tim_tm2(void)
{
#if HAL_LOGGING_ENABLED
    if (!should_log()) {
        return;
    }

// @LoggerMessage: UBXT
// @Description: uBlox specific UBX-TIM-TM2 logging, see uBlox interface description
// @Field: TimeUS: Time since system startup
// @Field: I: GPS instance number
// @Field: ch: Channel (i.e. EXTINT) upon which the pulse was measured
// @Field: flags: Bitmask
// @Field: count: Rising edge counter
// @Field: wnR: Week number of last rising edge
// @Field: MsR: Tow of rising edge
// @Field: SubMsR: Millisecond fraction of tow of rising edge in nanoseconds
// @Field: wnF: Week number of last falling edge
// @Field: MsF: Tow of falling edge
// @Field: SubMsF: Millisecond fraction of tow of falling edge in nanoseconds
// @Field: accEst: Accuracy estimate

    AP::logger().WriteStreaming("UBXT",
        "TimeUS,I,ch,flags,count,wnR,MsR,SubMsR,wnF,MsF,SubMsF,accEst",
        "s#----ss-sss",
        "F-----CI-CII",
        "QBBBHHIIHIII",
        AP_HAL::micros64(),
        state.instance,
        _buffer.tim_tm2.ch,
        _buffer.tim_tm2.flags,
        _buffer.tim_tm2.count,
        _buffer.tim_tm2.wnR,
        _buffer.tim_tm2.towMsR,
        _buffer.tim_tm2.towSubMsR,
        _buffer.tim_tm2.wnF,
        _buffer.tim_tm2.towMsF,
        _buffer.tim_tm2.towSubMsF,
        _buffer.tim_tm2.accEst);
#endif
}
#endif // UBLOX_TIM_TM2_LOGGING

#if UBLOX_RXM_RAW_LOGGING
void AP_GPS_UBLOX::log_rxm_raw(const struct ubx_rxm_raw &raw)
{
#if HAL_LOGGING_ENABLED
    if (!should_log()) {
        return;
    }

    uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<raw.numSV; i++) {
        struct log_GPS_RAW pkt = {
            LOG_PACKET_HEADER_INIT(LOG_GPS_RAW_MSG),
            time_us    : now,
            iTOW       : raw.iTOW,
            week       : raw.week,
            numSV      : raw.numSV,
            sv         : raw.svinfo[i].sv,
            cpMes      : raw.svinfo[i].cpMes,
            prMes      : raw.svinfo[i].prMes,
            doMes      : raw.svinfo[i].doMes,
            mesQI      : raw.svinfo[i].mesQI,
            cno        : raw.svinfo[i].cno,
            lli        : raw.svinfo[i].lli
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif
}

void AP_GPS_UBLOX::log_rxm_rawx(const struct ubx_rxm_rawx &raw)
{
#if HAL_LOGGING_ENABLED
    if (!should_log()) {
        return;
    }

    uint64_t now = AP_HAL::micros64();

    struct log_GPS_RAWH header = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_RAWH_MSG),
        time_us    : now,
        rcvTow     : raw.rcvTow,
        week       : raw.week,
        leapS      : raw.leapS,
        numMeas    : raw.numMeas,
        recStat    : raw.recStat
    };
    AP::logger().WriteBlock(&header, sizeof(header));

    for (uint8_t i=0; i<raw.numMeas; i++) {
        struct log_GPS_RAWS pkt = {
            LOG_PACKET_HEADER_INIT(LOG_GPS_RAWS_MSG),
            time_us    : now,
            prMes      : raw.svinfo[i].prMes,
            cpMes      : raw.svinfo[i].cpMes,
            doMes      : raw.svinfo[i].doMes,
            gnssId     : raw.svinfo[i].gnssId,
            svId       : raw.svinfo[i].svId,
            freqId     : raw.svinfo[i].freqId,
            locktime   : raw.svinfo[i].locktime,
            cno        : raw.svinfo[i].cno,
            prStdev    : raw.svinfo[i].prStdev,
            cpStdev    : raw.svinfo[i].cpStdev,
            doStdev    : raw.svinfo[i].doStdev,
            trkStat    : raw.svinfo[i].trkStat
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif
}
#endif // UBLOX_RXM_RAW_LOGGING

void AP_GPS_UBLOX::unexpected_message(void)
{
    Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
    if (++_disable_counter == 0) {
        // disable future sends of this message id, but
        // only do this every 256 messages, as some
        // message types can't be disabled and we don't
        // want to get into an ack war
        Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
        _configure_message_rate(_class, _msg_id, 0);
    }
}

// return size of a config key, or 0 if unknown
uint8_t AP_GPS_UBLOX::config_key_size(ConfigKey key) const
{
    // step over the value
    const uint8_t key_size = (uint32_t(key) >> 28) & 0x07; // mask off the storage size
    switch (key_size) {
    case 0x1: // bit
    case 0x2: // byte
        return 1;
    case 0x3: // 2 bytes
        return 2;
    case 0x4: // 4 bytes
        return 4;
    case 0x5: // 8 bytes
        return 8;
    default:
        break;
    }
    // invalid
    return 0;
}

/*
  find index of a config key in the active_config list, or -1
 */
int8_t AP_GPS_UBLOX::find_active_config_index(ConfigKey key) const
{
    if (active_config.list == nullptr) {
        return -1;
    }
    for (uint8_t i=0; i<active_config.count; i++) {
        if (key == active_config.list[i].key) {
            return (int8_t)i;
        }
    }

    return -1;
}

bool
AP_GPS_UBLOX::_parse_gps(void)
{
    if (_class == CLASS_ACK) {
        Debug("ACK %u", (unsigned)_msg_id);

        if(_msg_id == MSG_ACK_ACK) {
            switch(_buffer.ack.clsID) {
            case CLASS_CFG:
                switch(_buffer.ack.msgID) {
                case MSG_CFG_CFG:
                    _cfg_saved = true;
                    _cfg_needs_save = false;
                    break;
                case MSG_CFG_GNSS:
                    _unconfigured_messages &= ~CONFIG_GNSS;
                    break;
                case MSG_CFG_MSG:
                    // There is no way to know what MSG config was ack'ed, assume it was the last
                    // one requested. To verify it rerequest the last config we sent. If we miss
                    // the actual ack we will catch it next time through the poll loop, but that
                    // will be a good chunk of time later.
                    break;
                case MSG_CFG_NAV_SETTINGS:
                    _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
                    break;
                case MSG_CFG_RATE:
                    // The GPS will ACK a update rate that is invalid. in order to detect this
                    // only accept the rate as configured by reading the settings back and
                    // validating that they all match the target values
                    break;
                case MSG_CFG_SBAS:
                    _unconfigured_messages &= ~CONFIG_SBAS;
                    break;
                case MSG_CFG_TP5:
                    _unconfigured_messages &= ~CONFIG_TP5;
                    break;
                }
                break;
            case CLASS_MON:
                switch(_buffer.ack.msgID) {
                case MSG_MON_HW:
                    _unconfigured_messages &= ~CONFIG_RATE_MON_HW;
                    break;
                case MSG_MON_HW2:
                    _unconfigured_messages &= ~CONFIG_RATE_MON_HW2;
                    break;
                }
            }
        }
        if(_msg_id == MSG_ACK_NACK) {
            switch(_buffer.nack.clsID) {
            case CLASS_CFG:
                switch(_buffer.nack.msgID) {
                case MSG_CFG_VALGET:
                    if (active_config.list != nullptr) {
                        /*
                          likely this device does not support fetching multiple keys at once, go one at a time
                        */
                        if (active_config.fetch_index == -1) {
                            CFG_Debug("NACK starting %u", unsigned(active_config.count));
                            active_config.fetch_index = 0;
                        } else {
                            // the device does not support the config key we asked for,
                            // consider the bit as done
                            active_config.done_mask |= (1U<<active_config.fetch_index);
                            CFG_Debug("NACK %d 0x%x done=0x%x",
                                     int(active_config.fetch_index),
                                     unsigned(active_config.list[active_config.fetch_index].key),
                                     unsigned(active_config.done_mask));
                            if (active_config.done_mask == (1U<<active_config.count)-1) {
                                // all done!
                                _unconfigured_messages &= ~active_config.unconfig_bit;
                            }
                            active_config.fetch_index++;
                        }
                        if (active_config.fetch_index < active_config.count) {
                            _configure_valget(active_config.list[active_config.fetch_index].key);
                        }
                    }
                    break;
                }
            }
        }
        return false;
    }

    if (_class == CLASS_CFG) {
        switch(_msg_id) {
        case  MSG_CFG_NAV_SETTINGS:
	    Debug("Got settings %u min_elev %d drLimit %u\n", 
                  (unsigned)_buffer.nav_settings.dynModel,
                  (int)_buffer.nav_settings.minElev,
                  (unsigned)_buffer.nav_settings.drLimit);
            _buffer.nav_settings.mask = 0;
            if (gps._navfilter != AP_GPS::GPS_ENGINE_NONE &&
                _buffer.nav_settings.dynModel != gps._navfilter) {
                // we've received the current nav settings, change the engine
                // settings and send them back
                Debug("Changing engine setting from %u to %u\n",
                      (unsigned)_buffer.nav_settings.dynModel, (unsigned)gps._navfilter);
                _buffer.nav_settings.dynModel = gps._navfilter;
                _buffer.nav_settings.mask |= 1;
            }
            if (gps._min_elevation != -100 &&
                _buffer.nav_settings.minElev != gps._min_elevation) {
                Debug("Changing min elevation to %d\n", (int)gps._min_elevation);
                _buffer.nav_settings.minElev = gps._min_elevation;
                _buffer.nav_settings.mask |= 2;
            }
            if (_buffer.nav_settings.mask != 0) {
                _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                              &_buffer.nav_settings,
                              sizeof(_buffer.nav_settings));
                _unconfigured_messages |= CONFIG_NAV_SETTINGS;
                _cfg_needs_save = true;
            } else {
                _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
            }
            return false;

#if UBLOX_GNSS_SETTINGS
        case MSG_CFG_GNSS:
            if (params.gnss_mode != 0) {
                struct ubx_cfg_gnss start_gnss = _buffer.gnss;
                uint8_t gnssCount = 0;
                Debug("Got GNSS Settings %u %u %u %u:\n",
                    (unsigned)_buffer.gnss.msgVer,
                    (unsigned)_buffer.gnss.numTrkChHw,
                    (unsigned)_buffer.gnss.numTrkChUse,
                    (unsigned)_buffer.gnss.numConfigBlocks);
#if UBLOX_DEBUGGING
                for(int i = 0; i < _buffer.gnss.numConfigBlocks; i++) {
                    Debug("  %u %u %u 0x%08x\n",
                    (unsigned)_buffer.gnss.configBlock[i].gnssId,
                    (unsigned)_buffer.gnss.configBlock[i].resTrkCh,
                    (unsigned)_buffer.gnss.configBlock[i].maxTrkCh,
                    (unsigned)_buffer.gnss.configBlock[i].flags);
                }
#endif

                for(int i = 0; i < UBLOX_MAX_GNSS_CONFIG_BLOCKS; i++) {
                    if((params.gnss_mode & (1 << i)) && i != GNSS_SBAS) {
                        gnssCount++;
                    }
                }
                for(int i = 0; i < _buffer.gnss.numConfigBlocks; i++) {
                    // Reserve an equal portion of channels for all enabled systems that supports it
                    if(params.gnss_mode & (1 << _buffer.gnss.configBlock[i].gnssId)) {
                        if(GNSS_SBAS !=_buffer.gnss.configBlock[i].gnssId && (_hardware_generation > UBLOX_M8 || GNSS_GALILEO !=_buffer.gnss.configBlock[i].gnssId)) {
                            _buffer.gnss.configBlock[i].resTrkCh = (_buffer.gnss.numTrkChHw - 3) / (gnssCount * 2);
                            _buffer.gnss.configBlock[i].maxTrkCh = _buffer.gnss.numTrkChHw;
                        } else {
                            if(GNSS_SBAS ==_buffer.gnss.configBlock[i].gnssId) {
                                _buffer.gnss.configBlock[i].resTrkCh = 1;
                                _buffer.gnss.configBlock[i].maxTrkCh = 3;
                            }
                            if(GNSS_GALILEO ==_buffer.gnss.configBlock[i].gnssId) {
                                _buffer.gnss.configBlock[i].resTrkCh = (_buffer.gnss.numTrkChHw - 3) / (gnssCount * 2);
                                _buffer.gnss.configBlock[i].maxTrkCh = 8; //Per the M8 receiver description UBX-13003221 - R16, 4.1.1.3 it is not recommended to set the number of galileo channels higher then eight
                            }
                        }
                        _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags | 0x00000001;
                    } else {
                        _buffer.gnss.configBlock[i].resTrkCh = 0;
                        _buffer.gnss.configBlock[i].maxTrkCh = 0;
                        _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags & 0xFFFFFFFE;
                    }
                }
                if (memcmp(&start_gnss, &_buffer.gnss, sizeof(start_gnss))) {
                    _send_message(CLASS_CFG, MSG_CFG_GNSS, &_buffer.gnss, 4 + (8 * _buffer.gnss.numConfigBlocks));
                    _unconfigured_messages |= CONFIG_GNSS;
                    _cfg_needs_save = true;
                } else {
                    _unconfigured_messages &= ~CONFIG_GNSS;
                }
            } else {
                _unconfigured_messages &= ~CONFIG_GNSS;
            }
            return false;
#endif

        case MSG_CFG_SBAS:
            if (gps._sbas_mode != AP_GPS::SBAS_Mode::DoNotChange) {
	        Debug("Got SBAS settings %u %u %u 0x%x 0x%x\n", 
                      (unsigned)_buffer.sbas.mode,
                      (unsigned)_buffer.sbas.usage,
                      (unsigned)_buffer.sbas.maxSBAS,
                      (unsigned)_buffer.sbas.scanmode2,
                      (unsigned)_buffer.sbas.scanmode1);
                if (_buffer.sbas.mode != gps._sbas_mode) {
                    _buffer.sbas.mode = gps._sbas_mode;
                    _send_message(CLASS_CFG, MSG_CFG_SBAS,
                                  &_buffer.sbas,
                                  sizeof(_buffer.sbas));
                    _unconfigured_messages |= CONFIG_SBAS;
                    _cfg_needs_save = true;
                } else {
                    _unconfigured_messages &= ~CONFIG_SBAS;
                }
            } else {
                    _unconfigured_messages &= ~CONFIG_SBAS;
            }
            return false;
        case MSG_CFG_MSG:
            if(_payload_length == sizeof(ubx_cfg_msg_rate_6)) {
                // can't verify the setting without knowing the port
                // request the port again
                if(_ublox_port >= UBLOX_MAX_PORTS) {
                    _request_port();
                    return false;
                }
                _verify_rate(_buffer.msg_rate_6.msg_class, _buffer.msg_rate_6.msg_id,
                             _buffer.msg_rate_6.rates[_ublox_port]);
            } else {
                _verify_rate(_buffer.msg_rate.msg_class, _buffer.msg_rate.msg_id,
                             _buffer.msg_rate.rate);
            }
            return false;
        case MSG_CFG_PRT:
           _ublox_port = _buffer.prt.portID;
           return false;
        case MSG_CFG_RATE:
            if(_buffer.nav_rate.measure_rate_ms != params.rate_ms ||
               _buffer.nav_rate.nav_rate != 1 ||
               _buffer.nav_rate.timeref != 0) {
               _configure_rate();
                _unconfigured_messages |= CONFIG_RATE_NAV;
                _cfg_needs_save = true;
            } else {
                _unconfigured_messages &= ~CONFIG_RATE_NAV;
            }
            return false;
            
#if CONFIGURE_PPS_PIN
        case MSG_CFG_TP5: {
            // configure the PPS pin for 1Hz, zero delay
            Debug("Got TP5 ver=%u 0x%04x %u\n", 
                  (unsigned)_buffer.nav_tp5.version,
                  (unsigned)_buffer.nav_tp5.flags,
                  (unsigned)_buffer.nav_tp5.freqPeriod);
#ifdef HAL_GPIO_PPS
            hal.gpio->attach_interrupt(HAL_GPIO_PPS, FUNCTOR_BIND_MEMBER(&AP_GPS_UBLOX::pps_interrupt, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_FALLING);
#endif
            const uint16_t desired_flags = 0x003f;
            const uint16_t desired_period_hz = _pps_freq;

            if (_buffer.nav_tp5.flags != desired_flags ||
                _buffer.nav_tp5.freqPeriod != desired_period_hz) {
                _buffer.nav_tp5.tpIdx = 0;
                _buffer.nav_tp5.reserved1[0] = 0;
                _buffer.nav_tp5.reserved1[1] = 0;
                _buffer.nav_tp5.antCableDelay = 0;
                _buffer.nav_tp5.rfGroupDelay = 0;
                _buffer.nav_tp5.freqPeriod = desired_period_hz;
                _buffer.nav_tp5.freqPeriodLock = desired_period_hz;
                _buffer.nav_tp5.pulseLenRatio = 1;
                _buffer.nav_tp5.pulseLenRatioLock = 2;
                _buffer.nav_tp5.userConfigDelay = 0;
                _buffer.nav_tp5.flags = desired_flags;
                _send_message(CLASS_CFG, MSG_CFG_TP5,
                              &_buffer.nav_tp5,
                              sizeof(_buffer.nav_tp5));
                _unconfigured_messages |= CONFIG_TP5;
                _cfg_needs_save = true;
            } else {
                _unconfigured_messages &= ~CONFIG_TP5;
            }
            return false;
        }
#endif // CONFIGURE_PPS_PIN
        case MSG_CFG_VALGET: {
            uint8_t cfg_len = _payload_length - sizeof(ubx_cfg_valget);
            const uint8_t *cfg_data = (const uint8_t *)(&_buffer) + sizeof(ubx_cfg_valget);
            while (cfg_len >= 5) {
                ConfigKey id;
                memcpy(&id, cfg_data, sizeof(uint32_t));
                cfg_len -= 4;
                cfg_data += 4;
                switch (id) {
                    case ConfigKey::TMODE_MODE: {
                        uint8_t mode = cfg_data[0];
                        if (mode != 0) {
                            // ask for mode 0, to disable TIME mode
                            mode = 0;
                            _configure_valset(ConfigKey::TMODE_MODE, &mode);
                            _cfg_needs_save = true;
                            _unconfigured_messages |= CONFIG_TMODE_MODE;
                        } else {
                            _unconfigured_messages &= ~CONFIG_TMODE_MODE;
                        }
                        break;
                    }
                    default:
                        break;
                }
                // see if it is in active config list
                int8_t cfg_idx = find_active_config_index(id);
                if (cfg_idx >= 0) {
                    const uint8_t key_size = config_key_size(id);
                    if (cfg_len < key_size ||
                        memcmp(&active_config.list[cfg_idx].value, cfg_data, key_size) != 0) {
                        _configure_valset(id, &active_config.list[cfg_idx].value, active_config.layers);
                        _unconfigured_messages |= active_config.unconfig_bit;
                        active_config.done_mask &= ~(1U << cfg_idx);
                        _cfg_needs_save = true;
                    } else {
                        active_config.done_mask |= (1U << cfg_idx);
                        CFG_Debug("done %u mask=0x%x all_mask=0x%x",
                                  unsigned(cfg_idx),
                                  unsigned(active_config.done_mask),
                                  (1U<<active_config.count)-1);
                        if (active_config.done_mask == (1U<<active_config.count)-1) {
                            // all done!
                            _unconfigured_messages &= ~active_config.unconfig_bit;
                        }
                    }
                    if (active_config.fetch_index >= 0 &&
                        active_config.fetch_index < active_config.count &&
                        id == active_config.list[active_config.fetch_index].key) {
                        active_config.fetch_index++;
                        if (active_config.fetch_index < active_config.count) {
                            _configure_valget(active_config.list[active_config.fetch_index].key);
                            CFG_Debug("valget %d 0x%x", int(active_config.fetch_index),
                                  unsigned(active_config.list[active_config.fetch_index].key));
                        }
                    }
                }

                // step over the value
                uint8_t step_size = config_key_size(id);
                if (step_size == 0) {
                    return false;
                }
                cfg_len -= step_size;
                cfg_data += step_size;
            }
        }
        }
    }

    if (_class == CLASS_MON) {
        switch(_msg_id) {
        case MSG_MON_HW:
            if (_payload_length == 60 || _payload_length == 68) {
                log_mon_hw();
            }
            break;
        case MSG_MON_HW2:
            if (_payload_length == 28) {
                log_mon_hw2();  
            }
            break;
        case MSG_MON_VER: {
            bool check_L1L5 = false;
            _have_version = true;
            strncpy(_version.hwVersion, _buffer.mon_ver.hwVersion, sizeof(_version.hwVersion));
            strncpy(_version.swVersion, _buffer.mon_ver.swVersion, sizeof(_version.swVersion));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                                             "u-blox %d HW: %s SW: %s",
                                             state.instance + 1,
                                             _version.hwVersion,
                                             _version.swVersion);
            // check for F9 and M9. The F9 does not respond to SVINFO,
            // so we need to use MON_VER for hardware generation
            if (strncmp(_version.hwVersion, "00190000", 8) == 0) {
                if (strncmp(_version.swVersion, "EXT CORE 1", 10) == 0) {
                    // a F9
                    if (_hardware_generation != UBLOX_F9) {
                        // need to ensure time mode is correctly setup on F9
                        _unconfigured_messages |= CONFIG_TMODE_MODE;
                    }
                    _hardware_generation = UBLOX_F9;
                }
                if (strncmp(_version.swVersion, "EXT CORE 4", 10) == 0) {
                    // a M9
                    _hardware_generation = UBLOX_M9;
                }
                check_L1L5 = true;
            }
            // check for M10
            if (strncmp(_version.hwVersion, "000A0000", 8) == 0) {
                _hardware_generation = UBLOX_M10;
                _unconfigured_messages |= CONFIG_M10;
                // M10 does not support CONFIG_GNSS
                _unconfigured_messages &= ~CONFIG_GNSS;
                check_L1L5 = true;
            }
            if (check_L1L5) {
                // check if L1L5 in extension
                if (memmem(_buffer.mon_ver.extension, sizeof(_buffer.mon_ver.extension), "L1L5", 4) != nullptr) {
                    supports_l5 = true;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "u-blox supports L5 Band");
                    _unconfigured_messages |= CONFIG_L5;
                }
            }
            break;
        }
        default:
            unexpected_message();
        }
        return false;
    }

#if UBLOX_RXM_RAW_LOGGING
    if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAW && gps._raw_data != 0) {
        log_rxm_raw(_buffer.rxm_raw);
        return false;
    } else if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAWX && gps._raw_data != 0) {
        log_rxm_rawx(_buffer.rxm_rawx);
        return false;
    }
#endif // UBLOX_RXM_RAW_LOGGING

#if UBLOX_TIM_TM2_LOGGING
    if ((_class == CLASS_TIM) && (_msg_id == MSG_TIM_TM2) && (_payload_length == 28)) {
        log_tim_tm2();
        return false;
    }
#endif // UBLOX_TIM_TM2_LOGGING

    if (_class != CLASS_NAV) {
        unexpected_message();
        return false;
    }

    switch (_msg_id) {
    case MSG_POSLLH:
        Debug("MSG_POSLLH next_fix=%u", next_fix);
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_POSLLH;
            break;
        }
        _check_new_itow(_buffer.posllh.itow);
        _last_pos_time        = _buffer.posllh.itow;
        state.location.lng    = _buffer.posllh.longitude;
        state.location.lat    = _buffer.posllh.latitude;
        state.have_undulation = true;
        state.undulation = (_buffer.posllh.altitude_msl - _buffer.posllh.altitude_ellipsoid) * 0.001;
        set_alt_amsl_cm(state, _buffer.posllh.altitude_msl / 10);

        state.status          = next_fix;
        _new_position = true;
        state.horizontal_accuracy = _buffer.posllh.horizontal_accuracy*1.0e-3f;
        state.vertical_accuracy = _buffer.posllh.vertical_accuracy*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
#if UBLOX_FAKE_3DLOCK
        state.location.lng = 1491652300L;
        state.location.lat = -353632610L;
        state.location.alt = 58400;
        state.vertical_accuracy = 0;
        state.horizontal_accuracy = 0;
#endif
        break;
    case MSG_STATUS:
        Debug("MSG_STATUS fix_status=%u fix_type=%u",
              _buffer.status.fix_status,
              _buffer.status.fix_type);
        _check_new_itow(_buffer.status.itow);
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_STATUS;
            break;
        }
        if (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) {
            if( (_buffer.status.fix_type == AP_GPS_UBLOX::FIX_3D) &&
                (_buffer.status.fix_status & AP_GPS_UBLOX::NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_DGPS;
            }else if( _buffer.status.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.status.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
#if UBLOX_FAKE_3DLOCK
        state.status = AP_GPS::GPS_OK_FIX_3D;
        next_fix = state.status;
#endif
        break;
    case MSG_DOP:
        Debug("MSG_DOP");
        noReceivedHdop = false;
        _check_new_itow(_buffer.dop.itow);
        state.hdop        = _buffer.dop.hDOP;
        state.vdop        = _buffer.dop.vDOP;
#if UBLOX_FAKE_3DLOCK
        state.hdop = 130;
        state.hdop = 170;
#endif
        break;
    case MSG_SOL:
        Debug("MSG_SOL fix_status=%u fix_type=%u",
              _buffer.solution.fix_status,
              _buffer.solution.fix_type);
        _check_new_itow(_buffer.solution.itow);
        if (havePvtMsg) {
            state.time_week = _buffer.solution.week;
            break;
        }
        if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) {
            if( (_buffer.solution.fix_type == AP_GPS_UBLOX::FIX_3D) &&
                (_buffer.solution.fix_status & AP_GPS_UBLOX::NAV_STATUS_DGPS_USED)) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_DGPS;
            }else if( _buffer.solution.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.solution.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
        if(noReceivedHdop) {
            state.hdop = _buffer.solution.position_DOP;
        }
        state.num_sats    = _buffer.solution.satellites;
        if (next_fix >= AP_GPS::GPS_OK_FIX_2D) {
            state.last_gps_time_ms = AP_HAL::millis();
            state.time_week_ms    = _buffer.solution.itow;
            state.time_week       = _buffer.solution.week;
        }
#if UBLOX_FAKE_3DLOCK
        next_fix = state.status;
        state.num_sats = 10;
        state.time_week = 1721;
        state.time_week_ms = AP_HAL::millis() + 3*60*60*1000 + 37000;
        state.last_gps_time_ms = AP_HAL::millis();
        state.hdop = 130;
#endif
        break;

#if GPS_MOVING_BASELINE
    case MSG_RELPOSNED:
        {
            if (role != AP_GPS::GPS_ROLE_MB_ROVER) {
                // ignore RELPOSNED if not configured as a rover
                break;
            }
            // note that we require the yaw to come from a fixed solution, not a float solution
            // yaw from a float solution would only be acceptable with a very large separation between
            // GPS modules
            const uint32_t valid_mask = static_cast<uint32_t>(RELPOSNED::relPosHeadingValid) |
                                        static_cast<uint32_t>(RELPOSNED::relPosValid) |
                                        static_cast<uint32_t>(RELPOSNED::gnssFixOK) |
                                        static_cast<uint32_t>(RELPOSNED::isMoving) |
                                        static_cast<uint32_t>(RELPOSNED::carrSolnFixed);
            const uint32_t invalid_mask = static_cast<uint32_t>(RELPOSNED::refPosMiss) |
                                          static_cast<uint32_t>(RELPOSNED::refObsMiss) |
                                          static_cast<uint32_t>(RELPOSNED::carrSolnFloat);

            _check_new_itow(_buffer.relposned.iTOW);
            if (_buffer.relposned.iTOW != _last_relposned_itow+200) {
                // useful for looking at packet loss on links
                MB_Debug("RELPOSNED ITOW %u %u\n", unsigned(_buffer.relposned.iTOW), unsigned(_last_relposned_itow));
            }
            _last_relposned_itow = _buffer.relposned.iTOW;
            MB_Debug("RELPOSNED flags: %lx valid: %lx invalid: %lx\n", _buffer.relposned.flags, valid_mask, invalid_mask);
            if (((_buffer.relposned.flags & valid_mask) == valid_mask) &&
                ((_buffer.relposned.flags & invalid_mask) == 0)) {
                if (calculate_moving_base_yaw(_buffer.relposned.relPosHeading * 1e-5,
                                          _buffer.relposned.relPosLength * 0.01,
                                          _buffer.relposned.relPosD*0.01)) {
                    state.have_gps_yaw_accuracy = true;
                    state.gps_yaw_accuracy = _buffer.relposned.accHeading * 1e-5;
                    _last_relposned_ms = AP_HAL::millis();
                }
                state.relPosHeading = _buffer.relposned.relPosHeading * 1e-5;
                state.relPosLength  = _buffer.relposned.relPosLength * 0.01;
                state.relPosD       = _buffer.relposned.relPosD * 0.01;
                state.accHeading    = _buffer.relposned.accHeading * 1e-5;
                state.relposheading_ts = AP_HAL::millis();
            } else {
                state.have_gps_yaw_accuracy = false;
            }
        }
        break;
#endif // GPS_MOVING_BASELINE

    case MSG_PVT:
        Debug("MSG_PVT");

        havePvtMsg = true;
        // position
        _check_new_itow(_buffer.pvt.itow);
        _last_pvt_itow = _buffer.pvt.itow;
        _last_pos_time        = _buffer.pvt.itow;
        state.location.lng    = _buffer.pvt.lon;
        state.location.lat    = _buffer.pvt.lat;
        state.have_undulation = true;
        state.undulation = (_buffer.pvt.h_msl - _buffer.pvt.h_ellipsoid) * 0.001;
        set_alt_amsl_cm(state, _buffer.pvt.h_msl / 10);
        switch (_buffer.pvt.fix_type)
        {
            case 0:
                state.status = AP_GPS::NO_FIX;
                break;
            case 1:
                state.status = AP_GPS::NO_FIX;
                break;
            case 2:
                state.status = AP_GPS::GPS_OK_FIX_2D;
                break;
            case 3:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                if (_buffer.pvt.flags & 0b00000010)  // diffsoln
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                if (_buffer.pvt.flags & 0b01000000)  // carrsoln - float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                if (_buffer.pvt.flags & 0b10000000)  // carrsoln - fixed
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            case 4:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                "Unexpected state %d", _buffer.pvt.flags);
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 5:
                state.status = AP_GPS::NO_FIX;
                break;
            default:
                state.status = AP_GPS::NO_FIX;
                break;
        }
        next_fix = state.status;
        _new_position = true;
        state.horizontal_accuracy = _buffer.pvt.h_acc*1.0e-3f;
        state.vertical_accuracy = _buffer.pvt.v_acc*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        // SVs
        state.num_sats    = _buffer.pvt.num_sv;
        // velocity     
        _last_vel_time         = _buffer.pvt.itow;
        state.ground_speed     = _buffer.pvt.gspeed*0.001f;          // m/s
        state.ground_course    = wrap_360(_buffer.pvt.head_mot * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.pvt.velN * 0.001f;
        state.velocity.y = _buffer.pvt.velE * 0.001f;
        state.velocity.z = _buffer.pvt.velD * 0.001f;
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.pvt.s_acc*0.001f;
        _new_speed = true;
        // dop
        if(noReceivedHdop) {
            state.hdop        = _buffer.pvt.p_dop;
            state.vdop        = _buffer.pvt.p_dop;
        }
                    
        state.last_gps_time_ms = AP_HAL::millis();
        
        // time
        state.time_week_ms    = _buffer.pvt.itow;
#if UBLOX_FAKE_3DLOCK
        state.location.lng = 1491652300L;
        state.location.lat = -353632610L;
        state.location.alt = 58400;
        state.vertical_accuracy = 0;
        state.horizontal_accuracy = 0;
        state.status = AP_GPS::GPS_OK_FIX_3D;
        state.num_sats = 10;
        state.time_week = 1721;
        state.time_week_ms = AP_HAL::millis() + 3*60*60*1000 + 37000;
        state.last_gps_time_ms = AP_HAL::millis();
        state.hdop = 130;
        state.speed_accuracy = 0;
        next_fix = state.status;
#endif
        break;
    case MSG_TIMEGPS:
        Debug("MSG_TIMEGPS");
        _check_new_itow(_buffer.timegps.itow);
        if (_buffer.timegps.valid & UBX_TIMEGPS_VALID_WEEK_MASK) {
            state.time_week = _buffer.timegps.week;
        }
        break;
    case MSG_VELNED:
        Debug("MSG_VELNED");
        if (havePvtMsg) {
            _unconfigured_messages |= CONFIG_RATE_VELNED;
            break;
        }
        _check_new_itow(_buffer.velned.itow);
        _last_vel_time         = _buffer.velned.itow;
        state.ground_speed     = _buffer.velned.speed_2d*0.01f;          // m/s
        state.ground_course    = wrap_360(_buffer.velned.heading_2d * 1.0e-5f);       // Heading 2D deg * 100000
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.velned.ned_north * 0.01f;
        state.velocity.y = _buffer.velned.ned_east * 0.01f;
        state.velocity.z = _buffer.velned.ned_down * 0.01f;
        velocity_to_speed_course(state);
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.velned.speed_accuracy*0.01f;
#if UBLOX_FAKE_3DLOCK
        state.speed_accuracy = 0;
#endif
        _new_speed = true;
        break;
    case MSG_NAV_SVINFO:
        {
        Debug("MSG_NAV_SVINFO\n");
        static const uint8_t HardwareGenerationMask = 0x07;
        _check_new_itow(_buffer.svinfo_header.itow);
        _hardware_generation = _buffer.svinfo_header.globalFlags & HardwareGenerationMask;
        switch (_hardware_generation) {
            case UBLOX_5:
            case UBLOX_6:
                // only 7 and newer support CONFIG_GNSS
                _unconfigured_messages &= ~CONFIG_GNSS;
                break;
            case UBLOX_7:
            case UBLOX_M8:
#if UBLOX_SPEED_CHANGE
                port->begin(4000000U);
                Debug("Changed speed to 4Mhz for SPI-driven UBlox\n");
#endif
                break;
            default:
                hal.console->printf("Wrong Ublox Hardware Version%u\n", _hardware_generation);
                break;
        };
        _unconfigured_messages &= ~CONFIG_VERSION;
        /* We don't need that anymore */
        _configure_message_rate(CLASS_NAV, MSG_NAV_SVINFO, 0);
        break;
        }
    default:
        Debug("Unexpected NAV message 0x%02x", (unsigned)_msg_id);
        if (++_disable_counter == 0) {
            Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
            _configure_message_rate(CLASS_NAV, _msg_id, 0);
        }
        return false;
    }

    if (state.have_gps_yaw) {
        // when we are a rover we want to ensure we have both the new
        // PVT and the new RELPOSNED message so that we give a
        // consistent view
        if (AP_HAL::millis() - _last_relposned_ms > 400) {
            // we have stopped receiving valid RELPOSNED messages, disable yaw reporting
            state.have_gps_yaw = false;
        } else if (_last_relposned_itow != _last_pvt_itow) {
            // wait until ITOW matches
            return false;
        }
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}

/*
 *  handle pps interrupt
 */
#ifdef HAL_GPIO_PPS
void
AP_GPS_UBLOX::pps_interrupt(uint8_t pin, bool high, uint32_t timestamp_us)
{
    _last_pps_time_us = timestamp_us;
}

void
AP_GPS_UBLOX::set_pps_desired_freq(uint8_t freq)
{
    _pps_freq = freq;
    _unconfigured_messages |= CONFIG_TP5;
}
#endif


// UBlox auto configuration

/*
 *  update checksum for a set of bytes
 */
void
AP_GPS_UBLOX::_update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
    while (len--) {
        ck_a += *data;
        ck_b += ck_a;
        data++;
    }
}


/*
 *  send a ublox message
 */
bool
AP_GPS_UBLOX::_send_message(uint8_t msg_class, uint8_t msg_id, const void *msg, uint16_t size)
{
    if (port->txspace() < (sizeof(struct ubx_header) + 2 + size)) {
        return false;
    }
    struct ubx_header header;
    uint8_t ck_a=0, ck_b=0;
    header.preamble1 = PREAMBLE1;
    header.preamble2 = PREAMBLE2;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    _update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
    _update_checksum((uint8_t *)msg, size, ck_a, ck_b);

    port->write((const uint8_t *)&header, sizeof(header));
    port->write((const uint8_t *)msg, size);
    port->write((const uint8_t *)&ck_a, 1);
    port->write((const uint8_t *)&ck_b, 1);
    return true;
}

/*
 *  requests the given message rate for a specific message class
 *  and msg_id
 *  returns true if it sent the request, false if waiting on knowing the port
 */
bool
AP_GPS_UBLOX::_request_message_rate(uint8_t msg_class, uint8_t msg_id)
{
    // Without knowing what communication port is being used it isn't possible to verify
    // always ensure we have a port before sending the request
    if(_ublox_port >= UBLOX_MAX_PORTS) {
        _request_port();
        return false;
    } else {
        struct ubx_cfg_msg msg;
        msg.msg_class = msg_class;
        msg.msg_id    = msg_id;
        return _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
    }
}

/*
 *  configure a UBlox GPS for the given message rate for a specific
 *  message class and msg_id
 */
bool
AP_GPS_UBLOX::_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+sizeof(struct ubx_cfg_msg_rate)+2)) {
        return false;
    }

    struct ubx_cfg_msg_rate msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    msg.rate      = rate;
    return _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
}

/*
 *  configure F9/M10 based key/value pair - VALSET
 */
bool
AP_GPS_UBLOX::_configure_valset(ConfigKey key, const void *value, uint8_t layers)
{
    if (!supports_F9_config()) {
        return false;
    }
    const uint8_t len = config_key_size(key);
    struct ubx_cfg_valset msg {};
    uint8_t buf[sizeof(msg)+len];
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+sizeof(buf)+2)) {
        return false;
    }
    msg.version = 1;
    msg.layers = layers;
    msg.transaction = 0;
    msg.key = uint32_t(key);
    memcpy(buf, &msg, sizeof(msg));
    memcpy(&buf[sizeof(msg)], value, len);
    auto ret = _send_message(CLASS_CFG, MSG_CFG_VALSET, buf, sizeof(buf));
    return ret;
}

/*
 *  configure F9 based key/value pair - VALGET
 */
bool
AP_GPS_UBLOX::_configure_valget(ConfigKey key)
{
    if (!supports_F9_config()) {
        return false;
    }
    struct {
        struct ubx_cfg_valget msg;
        ConfigKey key;
    } msg {};
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+sizeof(msg)+2)) {
        return false;
    }
    msg.msg.version = 0;
    msg.msg.layers = 0; // ram
    msg.key = key;
    return _send_message(CLASS_CFG, MSG_CFG_VALGET, &msg, sizeof(msg));
}

/*
 *  configure F9 based key/value pair for a complete configuration set
 *
 *  this method requests each configuration variable from the GPS.
 *  When we handle the reply in _parse_gps we may then choose to set a
 *  MSG_CFG_VALSET back to the GPS if we don't like its response.
 */
bool
AP_GPS_UBLOX::_configure_config_set(const config_list *list, uint8_t count, uint32_t unconfig_bit, uint8_t layers)
{
    active_config.list = list;
    active_config.count = count;
    active_config.done_mask = 0;
    active_config.unconfig_bit = unconfig_bit;
    active_config.layers = layers;
    // we start by fetching multiple values at once (with fetch_index
    // -1) then if we get a NACK for VALGET we switch to fetching one
    // value at a time. This copes with the M10S which can only fetch
    // one value at a time
    active_config.fetch_index = -1;

    uint8_t buf[sizeof(ubx_cfg_valget)+count*sizeof(ConfigKey)];
    struct ubx_cfg_valget msg {};
    if (port->txspace() < (uint16_t)(sizeof(struct ubx_header)+sizeof(buf)+2)) {
        return false;
    }
    msg.version = 0;
    msg.layers = 0; // ram
    memcpy(buf, &msg, sizeof(msg));
    for (uint8_t i=0; i<count; i++) {
        memcpy(&buf[sizeof(msg)+i*sizeof(ConfigKey)], &list[i].key, sizeof(ConfigKey));
    }
    return _send_message(CLASS_CFG, MSG_CFG_VALGET, buf, sizeof(buf));
}

/*
 * save gps configurations to non-volatile memory sent until the call of
 * this message
 */
void
AP_GPS_UBLOX::_save_cfg()
{
    static const ubx_cfg_cfg save_cfg {
      clearMask: 0,
      saveMask: SAVE_CFG_ALL,
      loadMask: 0
    };
    _send_message(CLASS_CFG, MSG_CFG_CFG, &save_cfg, sizeof(save_cfg));
    _last_cfg_sent_time = AP_HAL::millis();
    _num_cfg_save_tries++;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                     "GPS %d: u-blox saving config",
                                     state.instance + 1);
}

/*
  detect a Ublox GPS. Adds one byte, and returns true if the stream
  matches a UBlox
 */
bool
AP_GPS_UBLOX::_detect(struct UBLOX_detect_state &state, uint8_t data)
{
reset:
	switch (state.step) {
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
                break;
            }
            state.step = 0;
            FALLTHROUGH;
        case 0:
            if (PREAMBLE1 == data)
                state.step++;
            break;
        case 2:
            state.step++;
            state.ck_b = state.ck_a = data;
            break;
        case 3:
            state.step++;
            state.ck_b += (state.ck_a += data);
            break;
        case 4:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_length = data;
            break;
        case 5:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_counter = 0;
            break;
        case 6:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == state.payload_length)
                state.step++;
            break;
        case 7:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
				goto reset;
            }
            break;
        case 8:
            state.step = 0;
			if (state.ck_b == data) {
				// a valid UBlox packet
				return true;
			} else {
				goto reset;
			}
    }
    return false;
}

void
AP_GPS_UBLOX::_request_version(void)
{
    _send_message(CLASS_MON, MSG_MON_VER, nullptr, 0);
}

void
AP_GPS_UBLOX::_configure_rate(void)
{
    struct ubx_cfg_nav_rate msg;
    // require a minimum measurement rate of 5Hz
    msg.measure_rate_ms = gps.get_rate_ms(state.instance);
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    _send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

static const char *reasons[] = {"navigation rate",
                                "posllh rate",
                                "status rate",
                                "solution rate",
                                "velned rate",
                                "dop rate",
                                "hw monitor rate",
                                "hw2 monitor rate",
                                "raw rate",
                                "version",
                                "navigation settings",
                                "GNSS settings",
                                "SBAS settings",
                                "PVT rate",
                                "time pulse settings",
                                "TIMEGPS rate",
                                "Time mode settings",
                                "RTK MB",
                                "TIM TM2",
                                "M10",
                                "L5 Enable Disable"};

static_assert((1 << ARRAY_SIZE(reasons)) == CONFIG_LAST, "UBLOX: Missing configuration description");

void
AP_GPS_UBLOX::broadcast_configuration_failure_reason(void) const {
    for (uint8_t i = 0; i < ARRAY_SIZE(reasons); i++) {
        if (_unconfigured_messages & (1 << i)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: u-blox %s configuration 0x%02x",
                (unsigned int)(state.instance + 1), reasons[i], (unsigned int)_unconfigured_messages);
            break;
        }
    }
}

/*
  return velocity lag in seconds
 */
bool AP_GPS_UBLOX::get_lag(float &lag_sec) const
{
    switch (_hardware_generation) {
    case UBLOX_UNKNOWN_HARDWARE_GENERATION:
        lag_sec = 0.22f;
        // always bail out in this case, it's used to indicate we have yet to receive a valid
        // hardware generation, however the user may have inhibited us detecting the generation
        // so if we aren't allowed to do configuration, we will accept this as the default delay
        return gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE;
    case UBLOX_5:
    case UBLOX_6:
    default:
        lag_sec = 0.22f;
        break;
    case UBLOX_7:
    case UBLOX_M8:
        // based on flight logs the 7 and 8 series seem to produce about 120ms lag
        lag_sec = 0.12f;
        break;
    case UBLOX_F9:
    case UBLOX_M9:
    case UBLOX_M10:
        // F9 lag not verified yet from flight log, but likely to be at least
        // as good as M8
        lag_sec = 0.12f;
#if GPS_MOVING_BASELINE
        if (role == AP_GPS::GPS_ROLE_MB_BASE ||
            role == AP_GPS::GPS_ROLE_MB_ROVER) {
            // the moving baseline rover will lag about 40ms from the
            // base. We need to provide the more conservative value to
            // ensure that the EKF allocates a larger enough buffer
            lag_sec += 0.04;
        }
#endif
        break;
    };
    return true;
}

#if HAL_LOGGING_ENABLED
void AP_GPS_UBLOX::Write_AP_Logger_Log_Startup_messages() const
{
    AP_GPS_Backend::Write_AP_Logger_Log_Startup_messages();

    if (_have_version) {
        AP::logger().Write_MessageF("u-blox %d HW: %s SW: %s",
                                           state.instance+1,
                                           _version.hwVersion,
                                           _version.swVersion);
    }
}
#endif

// uBlox specific check_new_itow(), handling message length
void AP_GPS_UBLOX::_check_new_itow(uint32_t itow)
{
    check_new_itow(itow, _payload_length + sizeof(ubx_header) + 2);
}

// support for retrieving RTCMv3 data from a moving baseline base
bool AP_GPS_UBLOX::get_RTCMV3(const uint8_t *&bytes, uint16_t &len)
{
#if GPS_MOVING_BASELINE
    if (rtcm3_parser) {
        len = rtcm3_parser->get_len(bytes);
        return len > 0;
    }
#endif
    return false;
}

// clear previous RTCM3 packet
void AP_GPS_UBLOX::clear_RTCMV3(void)
{
#if GPS_MOVING_BASELINE
    if (rtcm3_parser) {
        rtcm3_parser->clear_packet();
    }
#endif
}

// ublox specific healthy checks
bool AP_GPS_UBLOX::is_healthy(void) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        // allow for fake ublox moving baseline
        return true;
    }
#endif
#if GPS_MOVING_BASELINE
    if ((role == AP_GPS::GPS_ROLE_MB_BASE ||
        role == AP_GPS::GPS_ROLE_MB_ROVER) &&
        !supports_F9_config()) {
        // need F9 or above for moving baseline
        return false;
    }
    if (role == AP_GPS::GPS_ROLE_MB_BASE && rtcm3_parser == nullptr && !mb_use_uart2()) {
        // we haven't initialised RTCMv3 parser
        return false;
    }
#endif
    return true;
}

// return true if GPS is capable of F9 config
bool AP_GPS_UBLOX::supports_F9_config(void) const
{
    return _hardware_generation == UBLOX_F9 || _hardware_generation == UBLOX_M10;
}

#endif
