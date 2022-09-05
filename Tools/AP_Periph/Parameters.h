#pragma once

#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"

// Global parameter class.
//
class Parameters {
public:
    static const uint16_t k_format_version = 2;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_gps,
        k_param_compass,
        k_param_can_node,
        k_param_can_baudrate0,
        k_param_baro,
        k_param_buzz_volume,
        k_param_led_brightness,
        k_param_airspeed,
        k_param_rangefinder,
        k_param_flash_bootloader,
        k_param_rangefinder_baud,
        k_param_adsb_baudrate,
        k_param_hardpoint_id,
        k_param_hardpoint_rate,
        k_param_baro_enable,
        k_param_esc_number,
        k_param_battery,
        k_param_debug,
        k_param_serial_number,
        k_param_adsb_port,
        k_param_servo_channels,
        k_param_rangefinder_port,
        k_param_gps_port,
        k_param_msp_port,
        k_param_notify,
        k_param_esc_pwm_type,
        k_param_logger,
        k_param_log_bitmask,
        k_param_can_baudrate1,
        k_param_can_baudrate2,
        k_param_can_protocol0,
        k_param_can_protocol1,
        k_param_can_protocol2,
        k_param_sysid_this_mav,
        k_param_serial_manager,
        k_param_gps_mb_only_can_port,
        k_param_scripting,
        k_param_esc_telem_port,
        k_param_can_fdmode,
        k_param_can_fdbaudrate0,
        k_param_can_fdbaudrate1,
        k_param_node_stats,
        k_param_rangefinder_max_rate,
        k_param_efi,
        k_param_efi_port,
        k_param_efi_baudrate,
        k_param_esc_telem_rate,
    };

    AP_Int16 format_version;
    AP_Int16 can_node;
    
    AP_Int32 can_baudrate[HAL_NUM_CAN_IFACES];
#if HAL_NUM_CAN_IFACES >= 2
    AP_Enum<AP_CANManager::Driver_Type> can_protocol[HAL_NUM_CAN_IFACES];
#endif

#ifdef HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY
    AP_Int8 buzz_volume;
#endif
#ifdef AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
    AP_Int8 led_brightness;
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Int8 baro_enable;
#endif
#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    AP_Int8 flash_bootloader;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    AP_Int32 rangefinder_baud;
    AP_Int8 rangefinder_port;
    AP_Int16 rangefinder_max_rate;
#endif

#ifdef HAL_PERIPH_ENABLE_ADSB
    AP_Int32 adsb_baudrate;
    AP_Int8 adsb_port;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    AP_Int16 hardpoint_id;
    AP_Int8 hardpoint_rate;
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    AP_Int8 esc_number;
#endif

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_Int8 gps_port;
#if GPS_MOVING_BASELINE
    AP_Int8 gps_mb_only_can_port;
#endif
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    AP_Int8 msp_port;
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    AP_Int8 esc_pwm_type;
#if HAL_WITH_ESC_TELEM && !HAL_GCS_ENABLED
    AP_Int8 esc_telem_port;
#endif
#if HAL_WITH_ESC_TELEM
    AP_Int32 esc_telem_rate;
#endif
#endif

    AP_Int8 debug;

    AP_Int32 serial_number;

#if HAL_LOGGING_ENABLED
    AP_Int32        log_bitmask;
#endif

#if HAL_GCS_ENABLED
    AP_Int16 sysid_this_mav;
#endif

#ifdef HAL_PERIPH_ENABLE_EFI
    AP_Int32 efi_baudrate;
    AP_Int8 efi_port;
#endif
    
#if HAL_CANFD_SUPPORTED
    AP_Int8 can_fdmode;
    AP_Int8 can_fdbaudrate[HAL_NUM_CAN_IFACES];
#else
    static constexpr uint8_t can_fdmode = 0;
#endif
    Parameters() {}
};

extern const AP_Param::Info var_info[];
