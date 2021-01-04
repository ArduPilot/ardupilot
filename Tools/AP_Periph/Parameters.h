#pragma once

#include <AP_Common/AP_Common.h>

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
        k_param_can_baudrate,
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
    };

    AP_Int16 format_version;
    AP_Int16 can_node;
    AP_Int32 can_baudrate;
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
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    AP_Int8 msp_port;
#endif
    
    AP_Int8 debug;

    AP_Int32 serial_number;

    Parameters() {}
};

extern const AP_Param::Info var_info[];
