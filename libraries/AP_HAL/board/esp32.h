#pragma once

//generated header
#include <hwdef.h>

// coming from their respective esp-idf/sdkconfig files ...
// also note that '#ifdef CONFIG_IDF_TARGET_ESP32' is only true on Classic esp32 targets, not s3.
// also note that '#ifdef CONFIG_IDF_TARGET_ESP32S3' is only true on esp32s3 targets, not classic.

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DIY
#include "esp32diy.h" // Charles
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_BUZZ
#include "esp32buzz.h" //Buzz
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_ICARUS
#include "esp32icarus.h" //Alex
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_EMPTY
#include "esp32empty.h" //wiktor-m
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_TOMTE76
#include "esp32tomte76.h" //tomte76 on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_NICK
#include "esp32nick.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT
#include "esp32s3devkit.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
#include "esp32s3empty.h"
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3BUZZ
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DYNAMIC
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3BUZZ_PERIPH
// nothing here. both these use hwdef.h from hwdef.dat, not this - todo maybe stop requring SUBTYPES for hwdef.dat boards?
#else
#error "Invalid CONFIG_HAL_BOARD_SUBTYPE for esp32"
#endif

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_WITH_DRONECAN 0
#define HAL_WITH_UAVCAN 0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// boards derived from hwdef.dat dont necessarily have this so make a reasonable fallback
#ifndef HAL_ESP32_BOARD_NAME 
#define HAL_ESP32_BOARD_NAME "esp32generic"
#endif

// some of these are optionally defined in the *generated* hwdef.h from hwdef.dat so we wrap them here
#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 0
#endif
#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
#endif
#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif

#define HAL_WITH_IO_MCU 0

#define O_CLOEXEC 0
#define HAL_STORAGE_SIZE (16384)

#ifdef __cplusplus
// allow for static semaphores
#include <type_traits>
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_BinarySemaphore ESP32::BinarySemaphore
#endif




#define HAL_MEM_CLASS HAL_MEM_CLASS_192

// disable uncommon stuff that we'd otherwise get 
#define HAL_EXTERNAL_AHRS_ENABLED 0
#define HAL_GENERATOR_ENABLED 0

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

// whenver u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#define XCHAL_ERRATUM_453 0
//#define CONFIG_FREERTOS_CORETIMER_0 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#define CONFIG_SYSVIEW_ENABLE 0
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#define USE_LIBC_REALLOC 0
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_LWIP_PPP_SUPPORT 0
#define CONFIG_LWIP_STATS 0
//#define CONFIG_ESP32_WIFI_CSI_ENABLED 0
//#define CONFIG_ESP32_WIFI_NVS_ENABLED 0
#define CONFIG_NEWLIB_NANO_FORMAT 0
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#define CONFIG_LWIP_STATS 0
#define LWIP_COMPAT_SOCKET_INET 0
#define LWIP_COMPAT_SOCKET_ADDR 0
//#define CONFIG_ESP32_WIFI_TX_BA_WIN 0
//#define CONFIG_ESP32_WIFI_RX_BA_WIN 0

// absolutely essential, as it defualts to 1324 in AP_Logger/AP_Logger.cpp, and that NOT enough.
// ....with stack checking enabled in FreRTOS and GDB connected, GDB reports:
// 0x4037ba21 in panic_abort (details=0x3fccdbb1 "***ERROR*** A stack overflow in task log_io has been detected.")
#define HAL_LOGGING_STACK_SIZE 1024*3

// turn off all the compasses by default.. 
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// we don't need 32, 16 is enough
#define NUM_SERVO_CHANNELS 16

// disble temp cal of gyros by default
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

//turn off a bunch of advanced plane scheduler table things. see ArduPlane.cpp
#define AP_ADVANCEDFAILSAFE_ENABLED 0
#define AP_ICENGINE_ENABLED 0
#define AP_OPTICALFLOW_ENABLED 0
#define AP_RPM_ENABLED 0
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#define HAL_MOUNT_ENABLED 0
#define AP_CAMERA_ENABLED 0
#define HAL_SOARING_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
#define HAL_ADSB_ENABLED 0
#define HAL_BUTTON_ENABLED 0 
#define AP_GRIPPER_ENABLED 0
#define AP_LANDINGGEAR_ENABLED 0

// disable avoid-fence-follow in copter, these all kinda need each other, so its all or none.
#define AP_AVOIDANCE_ENABLED 0
#define AP_FENCE_ENABLED 0
#define MODE_FOLLOW_ENABLED 0
#define AP_OAPATHPLANNER_ENABLED 0


// other big things..
#define HAL_QUADPLANE_ENABLED 0
#define HAL_GYROFFT_ENABLED 0

// remove once ESP32 isn't so chronically slow
#define AP_SCHEDULER_OVERTIME_MARGIN_US 50000UL

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED 1
#endif

#ifndef HAL_WITH_DSP
#define HAL_WITH_DSP 0
#endif

#ifndef BOARD_SAFETY_ENABLE_DEFAULT
# define BOARD_SAFETY_ENABLE_DEFAULT 0
#endif

// buzz todo on the devkitm boards there's a led, maybe we can drive it with RCOutputRGBLed(HAL_RCOUT_RGBLED_RED, HAL_RCOUT_RGBLED_GREEN, HAL_RCOUT_RGBLED_BLUE));  ?
// right-now LEDs on the esp32 are untested and unsupported.
#ifndef BUILD_DEFAULT_LED_TYPE
#define BUILD_DEFAULT_LED_TYPE (Notify_LED_None)
#endif


// three hardware serial + two virtual for tcp and udp
#ifndef HAL_UART_NUM_SERIAL_PORTS 
#define HAL_UART_NUM_SERIAL_PORTS 5
#endif

// unwanted features
#ifndef HAL_LANDING_DEEPSTALL_ENABLED
#define HAL_LANDING_DEEPSTALL_ENABLED 0
#endif
#ifndef HAL_PICCOLO_CAN_ENABLED
#define HAL_PICCOLO_CAN_ENABLED 0
#endif
#ifndef HAL_PERIPH_ENABLE_EFI
#define HAL_PERIPH_ENABLE_EFI 0
#endif
#ifndef AP_CHECK_FIRMWARE_ENABLED
#define AP_CHECK_FIRMWARE_ENABLED 0
#endif
#ifndef HAL_NMEA_OUTPUT_ENABLED
#define HAL_NMEA_OUTPUT_ENABLED 0
#endif


// periphs need compass and calibration and other stuff disabled - esp32_hwdef.py does this for s3 builds, but this is for classic esp32 and safer.
// these were all worked out by repeated linker errors on classic esp32 periph builds and working out how to disable the related subsystem.
#ifdef HAL_BUILD_AP_PERIPH
    // assume can-enabled on periph unless added elsewhere.  eg esp32_hwdef.py currently adds it for s3 periph builds.
    #ifndef HAL_NUM_CAN_IFACES
    #define HAL_NUM_CAN_IFACES 1
    #endif
    #ifndef COMPASS_CAL_ENABLED
    #define COMPASS_CAL_ENABLED 0
    #endif
    #ifndef AP_COMPASS_ENABLED
    #define AP_COMPASS_ENABLED 0
    #endif
    #ifndef HAL_MISSION_ENABLED
    #define HAL_MISSION_ENABLED 0
    #endif
    #ifndef AP_MISSION_ENABLED
    #define AP_MISSION_ENABLED 0
    #endif
    #ifndef HAL_RALLY_ENABLED
    #define HAL_RALLY_ENABLED 0
    #endif
    #ifndef AP_FETTEC_ONEWIRE_ENABLED
    #define AP_FETTEC_ONEWIRE_ENABLED 0
    #endif
    #ifndef HAL_SCHEDULER_ENABLED
    #define HAL_SCHEDULER_ENABLED 0
    #endif
    #ifndef HAL_LOGGING_ENABLED
    #define HAL_LOGGING_ENABLED 0
    #endif
    #ifndef HAL_GCS_ENABLED
    #define HAL_GCS_ENABLED 0
    #endif
    // weird link error if this is disabled, scripting enabled for now, and a bunch of other lua_generated_bindings.cpp stuff disabled.
    #ifndef AP_SCRIPTING_ENABLED
        #define AP_SCRIPTING_ENABLED 1
    #endif
    #ifndef HAL_ENABLE_DRONECAN_DRIVERS
    #define HAL_ENABLE_DRONECAN_DRIVERS 0
    #endif
    #ifndef AP_NETWORKING_ENABLED
    #define AP_NETWORKING_ENABLED 0
    #endif
    #ifndef HAL_VISUALODOM_ENABLED
    #define HAL_VISUALODOM_ENABLED 0
    #endif
    #ifndef AP_FRSKY_SPORT_TELEM_ENABLED
    #define AP_FRSKY_SPORT_TELEM_ENABLED 0
    #endif
    #ifndef AP_RCPROTOCOL_ENABLED
    #define AP_RCPROTOCOL_ENABLED 0
    #endif
    #ifndef AP_RC_CHANNEL_ENABLED
    #define AP_RC_CHANNEL_ENABLED 0
    #endif
    #ifndef AP_RANGEFINDER_ENABLED
    #define AP_RANGEFINDER_ENABLED 0
    #endif
    #ifndef AP_AHRS_DCM_ENABLED
    #define AP_AHRS_DCM_ENABLED 0
    #endif
    // turn off ekfs as well
    #ifndef AP_AHRS_ENABLED
    #define AP_AHRS_ENABLED 0
    #endif
    // esp32 doesnt use AP_FILESYSTEM_SYS_ENABLED yet, as it needs integration.
    #ifndef AP_FILESYSTEM_SYS_ENABLED
    #define AP_FILESYSTEM_SYS_ENABLED 0
    #endif
    #ifndef AP_ARMING_ENABLED
    #define AP_ARMING_ENABLED 0
    #endif
    #ifndef AP_CUSTOMROTATIONS_ENABLED
    #define AP_CUSTOMROTATIONS_ENABLED 0
    #endif
    #define PERIPH_FW TRUE

# else 
    // non-periph build....
    // non-periph builds dont have CAN by default
    #ifndef HAL_NUM_CAN_IFACES
    #define HAL_NUM_CAN_IFACES 0
#endif



#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID 0
#endif


#endif