// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for AP_Periph builds

#ifndef AP_SCHEDULER_ENABLED
#define AP_SCHEDULER_ENABLED 0
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

/*
  AP_Periph doesn't include the SERIAL parameter tree, instead each
  supported serial device type has it's own parameter within AP_Periph
  for which port is used.
 */
#define DEFAULT_SERIAL0_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL8_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL9_PROTOCOL SerialProtocol_None

#ifndef HAL_LOGGING_MAVLINK_ENABLED
#define HAL_LOGGING_MAVLINK_ENABLED 0
#endif

#ifndef AP_MISSION_ENABLED
#define AP_MISSION_ENABLED 0
#endif

#ifndef HAL_RALLY_ENABLED
#define HAL_RALLY_ENABLED 0
#endif

#ifndef HAL_NMEA_OUTPUT_ENABLED
#define HAL_NMEA_OUTPUT_ENABLED 0
#endif

#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID 0
#endif

#define PERIPH_FW TRUE
#define HAL_BUILD_AP_PERIPH

#ifndef HAL_WATCHDOG_ENABLED_DEFAULT
#define HAL_WATCHDOG_ENABLED_DEFAULT true
#endif

#ifndef AP_FETTEC_ONEWIRE_ENABLED
#define AP_FETTEC_ONEWIRE_ENABLED 0
#endif

#ifndef HAL_TORQEEDO_ENABLED
#define HAL_TORQEEDO_ENABLED 0
#endif

#ifndef AP_KDECAN_ENABLED
#define AP_KDECAN_ENABLED 0
#endif

#ifndef HAL_GENERATOR_ENABLED
#define HAL_GENERATOR_ENABLED 0
#endif

#ifndef HAL_BARO_WIND_COMP_ENABLED
#define HAL_BARO_WIND_COMP_ENABLED 0
#endif

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED (HAL_GCS_ENABLED || HAL_LOGGING_ENABLED)
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif

#ifndef AP_AIRSPEED_AUTOCAL_ENABLE
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED 0
#endif

#ifndef AP_VOLZ_ENABLED
#define AP_VOLZ_ENABLED 0
#endif

#ifndef AP_ROBOTISSERVO_ENABLED
#define AP_ROBOTISSERVO_ENABLED 0
#endif

#ifndef AP_SBUSOUTPUT_ENABLED
#define AP_SBUSOUTPUT_ENABLED 0
#endif

// by default an AP_Periph defines as many servo output channels as
// there are PWM outputs:
#ifndef NUM_SERVO_CHANNELS
#ifdef HAL_PWM_COUNT
#define NUM_SERVO_CHANNELS HAL_PWM_COUNT
#else
#define NUM_SERVO_CHANNELS 0
#endif
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED 0
#endif

#ifndef AP_BATTERY_ESC_ENABLED
#define AP_BATTERY_ESC_ENABLED 0
#endif

// disable compass calibrations on periphs; cal is done on the autopilot
#ifndef COMPASS_CAL_ENABLED
#define COMPASS_CAL_ENABLED 0
#endif
#ifndef COMPASS_MOT_ENABLED
#define COMPASS_MOT_ENABLED 0
#endif
#ifndef COMPASS_LEARN_ENABLED
#define COMPASS_LEARN_ENABLED 0
#endif

#ifndef HAL_EXTERNAL_AHRS_ENABLED
#define HAL_EXTERNAL_AHRS_ENABLED 0
#endif

// disable RC_Channels library:
#ifndef AP_RC_CHANNEL_ENABLED
#define AP_RC_CHANNEL_ENABLED 0
#endif

#define HAL_CRSF_TELEM_ENABLED 0

#ifndef AP_SERVORELAYEVENTS_ENABLED
#define AP_SERVORELAYEVENTS_ENABLED 0
#endif

#ifndef AP_RELAY_ENABLED
#define AP_RELAY_ENABLED 0
#endif

/*
 * GPS Backends - we selectively turn backends on.
 *   Note also that f103-GPS explicitly disables some of these backends.
 */
#define AP_GPS_BACKEND_DEFAULT_ENABLED 0

#ifndef AP_GPS_ERB_ENABLED
#define AP_GPS_ERB_ENABLED 0
#endif

#ifndef AP_GPS_GSOF_ENABLED
#define AP_GPS_GSOF_ENABLED defined(HAL_PERIPH_ENABLE_GPS)
#endif

#ifndef AP_GPS_NMEA_ENABLED
#define AP_GPS_NMEA_ENABLED 0
#endif

#ifndef AP_GPS_SBF_ENABLED
#define AP_GPS_SBF_ENABLED defined(HAL_PERIPH_ENABLE_GPS)
#endif

#ifndef AP_GPS_SBP_ENABLED
#define AP_GPS_SBP_ENABLED 0
#endif

#ifndef AP_GPS_SBP2_ENABLED
#define AP_GPS_SBP2_ENABLED 0
#endif

#ifndef AP_GPS_SIRF_ENABLED
#define AP_GPS_SIRF_ENABLED 0
#endif

#ifndef AP_GPS_MAV_ENABLED
#define AP_GPS_MAV_ENABLED 0
#endif

#ifndef AP_GPS_NOVA_ENABLED
#define AP_GPS_NOVA_ENABLED defined(HAL_PERIPH_ENABLE_GPS)
#endif

#ifndef HAL_SIM_GPS_ENABLED
#define HAL_SIM_GPS_ENABLED (AP_SIM_ENABLED && defined(HAL_PERIPH_ENABLE_GPS))
#endif

/*
 * Airspeed Backends - we selectively turn backends *off*
 */
#ifndef AP_AIRSPEED_ANALOG_ENABLED
#define AP_AIRSPEED_ANALOG_ENABLED 0
#endif

// disable various rangefinder backends
#define AP_RANGEFINDER_ANALOG_ENABLED 0
#define AP_RANGEFINDER_HC_SR04_ENABLED 0
#define AP_RANGEFINDER_PWM_ENABLED 0

// AP_Periph expects ROTATION_NONE
#ifndef AP_RANGEFINDER_DEFAULT_ORIENTATION
#define AP_RANGEFINDER_DEFAULT_ORIENTATION ROTATION_NONE
#endif

// no CAN manager in AP_Periph:
#define HAL_CANMANAGER_ENABLED 0

// SLCAN is off by default:
#ifndef AP_CAN_SLCAN_ENABLED
#define AP_CAN_SLCAN_ENABLED 0
#endif

// Periphs don't use the FFT library:
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

// MSP parsing is off by default in AP_Periph:
#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 0
#endif

// periph does not make use of compass scaling or diagonals
#ifndef AP_COMPASS_DIAGONALS_ENABLED
#define AP_COMPASS_DIAGONALS_ENABLED 0
#endif

// disable various battery monitor backends:
#ifndef AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
#define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 0
#endif

#ifndef AP_BATT_MONITOR_MAX_INSTANCES
#define AP_BATT_MONITOR_MAX_INSTANCES 1
#endif

// Capacity tracking off
#ifndef AP_BATT_MONITOR_BATTERY_CAPACITY
#define AP_BATT_MONITOR_BATTERY_CAPACITY 0
#endif

#ifndef RANGEFINDER_MAX_INSTANCES
#define RANGEFINDER_MAX_INSTANCES 1
#endif

// by default AP_Periphs don't use INS:
#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

// no fence by default in AP_Periph:
#ifndef AP_FENCE_ENABLED
#define AP_FENCE_ENABLED 0
#endif

// periph does not save temperature cals etc:
#ifndef HAL_ENABLE_SAVE_PERSISTENT_PARAMS
#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS 0
#endif

#ifndef AP_WINCH_ENABLED
#define AP_WINCH_ENABLED 0
#endif

#ifndef AP_VIDEOTX_ENABLED
#define AP_VIDEOTX_ENABLED 0
#endif

#ifndef AP_FRSKY_TELEM_ENABLED
#define AP_FRSKY_TELEM_ENABLED 0
#endif

#ifndef HAL_SPEKTRUM_TELEM_ENABLED
#define HAL_SPEKTRUM_TELEM_ENABLED 0
#endif

#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED 0
#endif

#ifndef NOTIFY_LED_OVERRIDE_DEFAULT
#define NOTIFY_LED_OVERRIDE_DEFAULT 1       // rgb_source_t::mavlink
#endif

#ifndef HAL_PROXIMITY_ENABLED
#define HAL_PROXIMITY_ENABLED 0
#endif

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
#define AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED 0
#endif

#define AP_BATTERY_ENABLED defined(HAL_PERIPH_ENABLE_BATTERY)
#define AP_AHRS_ENABLED defined(HAL_PERIPH_ENABLE_AHRS)
#define AP_COMPASS_ENABLED defined(HAL_PERIPH_ENABLE_MAG)
#define AP_BARO_ENABLED defined(HAL_PERIPH_ENABLE_BARO)
#define AP_GPS_ENABLED defined(HAL_PERIPH_ENABLE_GPS)
#define AP_RPM_ENABLED defined(HAL_PERIPH_ENABLE_RPM)
#define AP_RCPROTOCOL_ENABLED defined(HAL_PERIPH_ENABLE_RCIN)

#ifndef AP_BOOTLOADER_ALWAYS_ERASE
#define AP_BOOTLOADER_ALWAYS_ERASE 1
#endif

#ifndef GPS_MOVING_BASELINE
#define GPS_MOVING_BASELINE 0
#endif

#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED AP_GPS_ENABLED && (GPS_MOVING_BASELINE || BOARD_FLASH_SIZE>=256)
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif

// for boards other than AP_Periph we are always expecting delays when
// not initialised.  We can't afford that on AP_Periph as you may end
// up with a bricked node if you write a bad firmware to it.
#ifndef AP_HAL_CHIBIOS_IN_EXPECTED_DELAY_WHEN_NOT_INITIALISED
#define AP_HAL_CHIBIOS_IN_EXPECTED_DELAY_WHEN_NOT_INITIALISED 0
#endif
