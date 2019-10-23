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
    };

    AP_Int16 format_version;
    AP_Int16 can_node;
    AP_Int32 can_baudrate;
#ifdef HAL_PERIPH_ENABLE_BUZZER
    AP_Int8 buzz_volume;
#endif
#ifdef AP_PERIPH_HAVE_LED
    AP_Int8 led_brightness;
#endif

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    AP_Int8 flash_bootloader;
#endif

    Parameters() {}
};

extern const AP_Param::Info var_info[];
