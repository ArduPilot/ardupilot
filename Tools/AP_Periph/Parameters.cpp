
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

extern const AP_HAL::HAL &hal;

#ifndef HAL_PERIPH_LED_BRIGHT_DEFAULT
#define HAL_PERIPH_LED_BRIGHT_DEFAULT 100
#endif

#ifndef HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT
#define HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT 115200
#endif

#ifndef AP_PERIPH_RANGEFINDER_PORT_DEFAULT
#define AP_PERIPH_RANGEFINDER_PORT_DEFAULT 3
#endif

#ifndef HAL_PERIPH_GPS_PORT_DEFAULT
#define HAL_PERIPH_GPS_PORT_DEFAULT 3
#endif

#ifndef HAL_PERIPH_ADSB_BAUD_DEFAULT
#define HAL_PERIPH_ADSB_BAUD_DEFAULT 57600
#endif
#ifndef HAL_PERIPH_ADSB_PORT_DEFAULT
#define HAL_PERIPH_ADSB_PORT_DEFAULT 1
#endif

#ifndef AP_PERIPH_MSP_PORT_DEFAULT
#define AP_PERIPH_MSP_PORT_DEFAULT 1
#endif

#ifndef AP_PERIPH_ESC_TELEM_PORT_DEFAULT
#define AP_PERIPH_ESC_TELEM_PORT_DEFAULT -1
#endif

#ifndef AP_PERIPH_ESC_TELEM_RATE_DEFAULT
#define AP_PERIPH_ESC_TELEM_RATE_DEFAULT 50
#endif

#ifndef AP_PERIPH_BARO_ENABLE_DEFAULT
#define AP_PERIPH_BARO_ENABLE_DEFAULT 1
#endif

#ifndef HAL_PERIPH_BATT_HIDE_MASK_DEFAULT
#define HAL_PERIPH_BATT_HIDE_MASK_DEFAULT 0
#endif

#ifndef AP_PERIPH_EFI_PORT_DEFAULT
#define AP_PERIPH_EFI_PORT_DEFAULT 3
#endif

#ifndef HAL_PERIPH_EFI_BAUDRATE_DEFAULT
#define HAL_PERIPH_EFI_BAUDRATE_DEFAULT 115200
#endif

#ifndef HAL_DEFAULT_MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 3
#else
#define MAV_SYSTEM_ID HAL_DEFAULT_MAV_SYSTEM_ID
#endif

#ifndef APD_ESC_SERIAL_0
  #define APD_ESC_SERIAL_0 -1
#endif

#ifndef APD_ESC_SERIAL_1
  #define APD_ESC_SERIAL_1 -1
#endif

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

/*
 *  AP_Periph parameter definitions
 *
 */

const AP_Param::Info AP_Periph_FW::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: CAN_NODE
    // @DisplayName: DroneCAN node ID used by this node on all networks
    // @Description: Value of 0 requests any ID from a DNA server, any other value sets that ID ignoring DNA
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(can_node,         "CAN_NODE", HAL_CAN_DEFAULT_NODE_ID),

    // @Param: CAN_BAUDRATE
    // @DisplayName: Bitrate of CAN interface
    // @Description: Bit rate can be set up to from 10000 to 1000000
    // @Range: 10000 1000000
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_baudrate,     0, "CAN_BAUDRATE", 1000000),

#if AP_CAN_SLCAN_ENABLED
    // @Param: CAN_SLCAN_CPORT
    // @DisplayName: SLCAN Route
    // @Description: CAN Interface ID to be routed to SLCAN, 0 means no routing
    // @Values: 0:Disabled,1:First interface,2:Second interface
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(can_slcan_cport, "CAN_SLCAN_CPORT", 1),
#endif

#ifdef HAL_GPIO_PIN_GPIO_CAN1_TERM
    // @Param: CAN_TERMINATE
    // @DisplayName: Enable CAN software temination in this node
    // @Description: Enable CAN software temination in this node
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_terminate,   0, "CAN_TERMINATE", 0),
#endif

#if HAL_NUM_CAN_IFACES >= 2
    // @Param: CAN_PROTOCOL
    // @DisplayName: Enable use of specific protocol to be used on this port
    // @Description: Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN
    // @Values: 0:Disabled,1:UAVCAN,4:PiccoloCAN,6:EFI_NWPMU,7:USD1,8:KDECAN
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_protocol,     0, "CAN_PROTOCOL", float(AP_CAN::Protocol::DroneCAN)),
    
    // @Param: CAN2_BAUDRATE
    // @CopyFieldsFrom: CAN_BAUDRATE
    // @DisplayName: Bitrate of CAN2 interface
    GARRAY(can_baudrate,     1, "CAN2_BAUDRATE", 1000000),

    // @Param: CAN2_PROTOCOL
    // @CopyFieldsFrom: CAN_PROTOCOL
    GARRAY(can_protocol,     1, "CAN2_PROTOCOL", float(AP_CAN::Protocol::DroneCAN)),

#ifdef HAL_GPIO_PIN_GPIO_CAN2_TERM
    // @Param: CAN2_TERMINATE
    // @CopyFieldsFrom: CAN_TERMINATE
    GARRAY(can_terminate,    1, "CAN2_TERMINATE", 0),
#endif
#endif

#if HAL_NUM_CAN_IFACES >= 3
    // @Param: CAN3_BAUDRATE
    // @DisplayName: Bitrate of CAN3 interface
    // @CopyFieldsFrom: CAN_BAUDRATE
    GARRAY(can_baudrate,    2, "CAN3_BAUDRATE", 1000000),

    // @Param: CAN3_PROTOCOL
    // @CopyFieldsFrom: CAN_PROTOCOL
    GARRAY(can_protocol,    2, "CAN3_PROTOCOL", float(AP_CAN::Protocol::DroneCAN)),

#ifdef HAL_GPIO_PIN_GPIO_CAN3_TERM
    // @Param: CAN3_TERMINATE
    // @CopyFieldsFrom: CAN_TERMINATE
    GARRAY(can_terminate,    2, "CAN3_TERMINATE", 0),
#endif
#endif

#if HAL_CANFD_SUPPORTED
    // @Param: CAN_FDMODE
    // @DisplayName: Enable CANFD mode
    // @Description: Enabling this option sets the CAN bus to be in CANFD mode with BRS.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(can_fdmode,     "CAN_FDMODE", 0),

    // @Param: CAN_FDBAUDRATE
    // @DisplayName: Set up bitrate for data section on CAN1
    // @Description: This sets the bitrate for the data section of CAN1.
    // @Values: 1:1M, 2:2M, 4:4M, 5:5M, 8:8M
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_fdbaudrate,    0, "CAN_FDBAUDRATE", HAL_CANFD_SUPPORTED),

#if HAL_NUM_CAN_IFACES >= 2
    // @Param: CAN2_FDBAUDRATE
    // @CopyFieldsFrom: CAN_FDBAUDRATE
    // @DisplayName: Set up bitrate for data section on CAN2
    // @Description: This sets the bitrate for the data section of CAN2.
    GARRAY(can_fdbaudrate,    1, "CAN2_FDBAUDRATE", HAL_CANFD_SUPPORTED),
#endif
#endif

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    // @Param: FLASH_BOOTLOADER
    // @DisplayName: Trigger bootloader update
    // @Description: DANGER! When enabled, the App will perform a bootloader update by copying the embedded bootloader over the existing bootloader. This may take a few seconds to perform and should only be done if you know what you're doing.
    // @Range: 0 1
    // @User: Advanced
    GSCALAR(flash_bootloader,     "FLASH_BOOTLOADER", 0),
#endif

    // @Param: DEBUG
    // @DisplayName: Debug
    // @Description: Debug
    // @Bitmask: 0:Show free stack space, 1:Auto Reboot after 15sec, 2:Enable sending stats
    // @User: Advanced
    GSCALAR(debug, "DEBUG", 0),


    // @Param: BRD_SERIAL_NUM
    // @DisplayName: Serial number of device
    // @Description: Non-zero positive values will be shown on the CAN App Name string
    // @Range: 0 2147483648
    // @User: Advanced
    GSCALAR(serial_number, "BRD_SERIAL_NUM", 0),

#ifdef HAL_PERIPH_ENABLE_BUZZER_WITHOUT_NOTIFY
    // @Param: BUZZER_VOLUME
    // @DisplayName: Buzzer volume
    // @Description: Control the volume of the buzzer
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    GSCALAR(buzz_volume,     "BUZZER_VOLUME", 100),
#endif

#if HAL_PERIPH_ENABLE_GPS
    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Param: GPS_PORT
    // @DisplayName: GPS Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to GPS.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(gps_port, "GPS_PORT", HAL_PERIPH_GPS_PORT_DEFAULT),

#if GPS_MOVING_BASELINE
    // @Param: MB_CAN_PORT
    // @DisplayName: Moving Baseline CAN Port option
    // @Description: Autoselect dedicated CAN port on which moving baseline data will be transmitted.
    // @Values: 0:Sends moving baseline data on all ports,1:auto select remaining port for transmitting Moving baseline Data
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(gps_mb_only_can_port, "GPS_MB_ONLY_PORT", 0),
#endif
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY
    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery_lib, "BATT", AP_BattMonitor),

    // @Param: BATT_HIDE_MASK
    // @DisplayName: Battery hide mask
    // @Description: Instance mask of local battery index(es) to prevent transmitting their status over CAN. This is useful for hiding a "battery" instance that is used locally in the peripheral but don't want them to be treated as a battery source(s) to the autopilot. For example, an AP_Periph battery monitor with multiple batteries that monitors each locally for diagnostic or other purposes, but only reports as a single SUM battery monitor to the autopilot.
    // @Bitmask: 0:BATT, 1:BATT2, 2:BATT3, 3:BATT4, 4:BATT5, 5:BATT6, 6:BATT7, 7:BATT8, 8:BATT9, 9:BATTA, 10:BATTB, 11:BATTC, 12:BATTD, 13:BATTE, 14:BATTF, 15:BATTG
    // @User: Advanced
    GSCALAR(battery_hide_mask, "BATT_HIDE_MASK", HAL_PERIPH_BATT_HIDE_MASK_DEFAULT),
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,         "COMPASS_",     Compass),
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    // Baro driver
    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(baro, "BARO", AP_Baro),

    // @Param: BARO_ENABLE
    // @DisplayName: Barometer Enable
    // @Description: Barometer Enable
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(baro_enable, "BARO_ENABLE", AP_PERIPH_BARO_ENABLE_DEFAULT),
#endif

#ifdef AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY
    // @Param: LED_BRIGHTNESS
    // @DisplayName: LED Brightness
    // @Description: Select the RGB LED brightness level.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    GSCALAR(led_brightness, "LED_BRIGHTNESS", HAL_PERIPH_LED_BRIGHT_DEFAULT),
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    // Airspeed driver
    // @Group: ARSPD
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed, "ARSPD", AP_Airspeed),
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    // @Param: RNGFND_BAUDRATE
    // @DisplayName: Rangefinder serial baudrate
    // @Description: Rangefinder serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GARRAY(rangefinder_baud, 0, "RNGFND_BAUDRATE", HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT),

    // @Param: RNGFND_PORT
    // @DisplayName: Rangefinder Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to Rangefinder.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(rangefinder_port, 0, "RNGFND_PORT", AP_PERIPH_RANGEFINDER_PORT_DEFAULT),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Param: RNGFND2_BAUDRATE
    // @DisplayName: Rangefinder serial baudrate
    // @Description: Rangefinder serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GARRAY(rangefinder_baud, 1, "RNGFND2_BAUDRATE", HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT),

    // @Param: RNGFND2_PORT
    // @DisplayName: Rangefinder Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to Rangefinder.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(rangefinder_port, 1, "RNGFND2_PORT", -1),
#endif

    // @Param: RNGFND_MAX_RATE
    // @DisplayName: Rangefinder max rate
    // @Description: This is the maximum rate we send rangefinder data in Hz. Zero means no limit
    // @Units: Hz
    // @Range: 0 200
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rangefinder_max_rate, "RNGFND_MAX_RATE", 50),
    
    // Rangefinder driver
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder, "RNGFND", RangeFinder),
#endif

#ifdef HAL_PERIPH_ENABLE_ADSB
    // @Param: ADSB_BAUDRATE
    // @DisplayName: ADSB serial baudrate
    // @Description: ADSB serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(adsb_baudrate, "ADSB_BAUDRATE", HAL_PERIPH_ADSB_BAUD_DEFAULT),

    // @Param: ADSB_PORT
    // @DisplayName: ADSB Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to ADSB.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(adsb_port, "ADSB_PORT", HAL_PERIPH_ADSB_PORT_DEFAULT),
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    // @Param: HARDPOINT_ID
    // @DisplayName: Hardpoint ID
    // @Description: Hardpoint ID
    // @User: Advanced
    GSCALAR(hardpoint_id, "HARDPOINT_ID", HAL_PWM_HARDPOINT_ID_DEFAULT),

    // @Param: HARDPOINT_RATE
    // @DisplayName: Hardpoint PWM rate
    // @Description: Hardpoint PWM rate
    // @Range: 10 100
    // @Units: Hz
    // @Increment: 1
    // @User: Advanced
    GSCALAR(hardpoint_rate, "HARDPOINT_RATE", 100),
#endif

#if defined(HAL_PERIPH_ENABLE_HWESC) || defined(HAL_PERIPH_ENABLE_ESC_APD)
    // @Param: ESC_NUMBER
    // @DisplayName: ESC number
    // @Description: This is the ESC number to report as in UAVCAN ESC telemetry feedback packets.
    // @Increment: 1
    // @User: Advanced
    GARRAY(esc_number, 0, "ESC_NUMBER", 0),
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    // Servo driver
    // @Group: OUT
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    GOBJECT(servo_channels, "OUT",     SRV_Channels),

    // @Param: ESC_RATE
    // @DisplayName: ESC Update Rate
    // @Description: Rate in Hz that ESC PWM outputs (function is MotorN) will update at
    // @Units: Hz
    // @Range: 50 400
    // @Increment: 1
    // @User: Advanced
    GSCALAR(esc_rate, "ESC_RATE", 400), // effective Copter and QuadPlane default after clamping

    // @Param: ESC_PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output
    // @Values: 1:Normal,2:OneShot,3:OneShot125,4:Brushed,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(esc_pwm_type, "ESC_PWM_TYPE",     0),

    // @Param: ESC_CMD_TIMO
    // @DisplayName: ESC Command Timeout
    // @Description: This is the duration (ms) with which to hold the last driven ESC command before timing out and zeroing the ESC outputs. To disable zeroing of outputs in event of CAN loss, use 0. Use values greater than the expected duration between two CAN frames to ensure Periph is not starved of ESC Raw Commands.
    // @Range: 0 10000
    // @Units: ms
    // @User: Advanced
    GSCALAR(esc_command_timeout_ms, "ESC_CMD_TIMO",     200),

#if HAL_WITH_ESC_TELEM && !HAL_GCS_ENABLED
    // @Param: ESC_TELEM_PORT
    // @DisplayName: ESC Telemetry Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to ESC Telemetry
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(esc_telem_port, "ESC_TELEM_PORT", AP_PERIPH_ESC_TELEM_PORT_DEFAULT),
#endif

#if HAL_WITH_ESC_TELEM
    // @Param: ESC_TELEM_RATE
    // @DisplayName: ESC Telemetry update rate
    // @Description: This is the rate at which ESC Telemetry will be sent across the CAN bus
    // @Range: 0 1000
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(esc_telem_rate, "ESC_TELEM_RATE", AP_PERIPH_ESC_TELEM_RATE_DEFAULT),
#endif
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    // @Group: TEMP
    // @Path: ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp
    GOBJECT(temperature_sensor,         "TEMP",     AP_TemperatureSensor),
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    // @Param: MSP_PORT
    // @DisplayName: MSP Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to MSP
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(msp_port, "MSP_PORT", AP_PERIPH_MSP_PORT_DEFAULT),
#endif
    
#ifdef HAL_PERIPH_ENABLE_NOTIFY
    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),
#endif

#if HAL_LOGGING_ENABLED
    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Bitmask: 2:GPS
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          4),
#endif

#if HAL_GCS_ENABLED
    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),
#endif

#if HAL_GCS_ENABLED || defined(HAL_PERIPH_SHOW_SERIAL_MANAGER_PARAMS)
    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),
#endif

#if AP_SCRIPTING_ENABLED
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    GOBJECT(scripting, "SCR_", AP_Scripting),
#endif

#if AP_STATS_ENABLED
    // @Group: Node
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    GOBJECT(node_stats, "STAT", AP_Stats),
#endif

#ifdef HAL_PERIPH_ENABLE_EFI
    // @Param: EFI_BAUDRATE
    // @DisplayName: EFI serial baudrate
    // @Description: EFI  serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(efi_baudrate, "EFI_BAUDRATE", HAL_PERIPH_EFI_BAUDRATE_DEFAULT),

    // @Param: EFI_PORT
    // @DisplayName: EFI Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to EFI.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(efi_port, "EFI_PORT", AP_PERIPH_EFI_PORT_DEFAULT),

    // EFI driver
    // @Group: EFI
    // @Path: ../libraries/AP_EFI/AP_EFI.cpp
    GOBJECT(efi, "EFI", AP_EFI),
#endif

#ifdef HAL_PERIPH_ENABLE_PROXIMITY
    // @Param: PRX_BAUDRATE
    // @DisplayName: Proximity Sensor serial baudrate
    // @Description: Proximity Sensor serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(proximity_baud, "PRX_BAUDRATE", HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT),

    // @Param: PRX_PORT
    // @DisplayName: Proximity Sensor Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to Proximity Sensor.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(proximity_port, "PRX_PORT", AP_PERIPH_RANGEFINDER_PORT_DEFAULT),

    // @Param: PRX_MAX_RATE
    // @DisplayName: Proximity Sensor max rate
    // @Description: This is the maximum rate we send Proximity Sensor data in Hz. Zero means no limit
    // @Units: Hz
    // @Range: 0 200
    // @Increment: 1
    // @User: Advanced
    GSCALAR(proximity_max_rate, "PRX_MAX_RATE", 50),

    // Proximity driver
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    GOBJECT(proximity, "PRX", AP_Proximity),
#endif  // HAL_PERIPH_ENABLE_PROXIMITY

#if HAL_NMEA_OUTPUT_ENABLED
    // @Group: NMEA_
    // @Path: ../libraries/AP_NMEA_Output/AP_NMEA_Output.cpp
    GOBJECT(nmea, "NMEA_",   AP_NMEA_Output),
#endif

#if AP_KDECAN_ENABLED
    // @Group: KDE_
    // @Path: ../libraries/AP_KDECAN/AP_KDECAN.cpp
    GOBJECT(kdecan, "KDE_",   AP_KDECAN),
#endif

#if defined(HAL_PERIPH_ENABLE_ESC_APD)
    GARRAY(pole_count, 0, "ESC_NUM_POLES", 22),
#endif

#if defined(HAL_PERIPH_ENABLE_ESC_APD)
    // @Param: ESC_APD_SERIAL_1
    // @DisplayName: ESC APD Serial 1
    // @Description: Which serial port to use for APD ESC data
    // @Range: 0 6
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(esc_serial_port, 0, "ESC_APD_SERIAL_1", APD_ESC_SERIAL_0),

  #if APD_ESC_INSTANCES > 1
    GARRAY(esc_number, 1, "ESC_NUMBER2", 1),

    GARRAY(pole_count, 1, "ESC_NUM_POLES2", 22),

    // @Param: ESC_APD_SERIAL_2
    // @DisplayName: ESC APD Serial 2
    // @Description: Which serial port to use for APD ESC data
    // @Range: 0 6
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(esc_serial_port, 1, "ESC_APD_SERIAL_2", APD_ESC_SERIAL_1),
  #endif
#endif

#ifdef HAL_PERIPH_ENABLE_NETWORKING
    // @Group: NET_
    // @Path: networking.cpp
    GOBJECT(networking_periph, "NET_", Networking_Periph),
#endif

#ifdef HAL_PERIPH_ENABLE_RPM
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

#ifdef HAL_PERIPH_ENABLE_RCIN
    // @Group: RC
    // @Path: rc_in.cpp
    GOBJECT(g_rcin, "RC",  Parameters_RCIN),
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY_BALANCE
    // @Group: BAL
    // @Path: batt_balance.cpp
    GOBJECT(battery_balance, "BAL",  BattBalance),
#endif

#ifdef HAL_PERIPH_ENABLE_SERIAL_OPTIONS
    // @Group: UART
    // @Path: serial_options.cpp
    GOBJECT(serial_options, "UART",  SerialOptions),
#endif
    
    // NOTE: sim parameters should go last
#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),

#if AP_AHRS_ENABLED
    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),
#endif
#endif // AP_SIM_ENABLED

#if HAL_PERIPH_CAN_MIRROR
    // @Param: CAN_MIRROR_PORTS
    // @DisplayName: CAN ports to mirror traffic between
    // @Description: Any set ports will participate in blindly mirroring traffic from one port to the other. It is the users responsibility to ensure that no loops exist that cause traffic to be infinitly repeated, and both ports must be running the same baud rates.
    // @Bitmask: 0:CAN1, 1:CAN2, 2:CAN3
    // @User: Advanced
    GSCALAR(can_mirror_ports, "CAN_MIRROR_PORTS", 0),
#endif // HAL_PERIPH_CAN_MIRROR

#ifdef HAL_PERIPH_ENABLE_RTC
    // @Group: RTC
    // @Path: ../libraries/AP_RTC/AP_RTC.cpp
    GOBJECT(rtc,                   "RTC",    AP_RTC),
#endif

#ifdef HAL_PERIPH_ENABLE_RELAY
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                 "RELAY", AP_Relay),
#endif

#ifdef HAL_PERIPH_ENABLE_DEVICE_TEMPERATURE
    // @Param: TEMP_MSG_RATE
    // @DisplayName: Temperature sensor message rate
    // @Description: This is the rate Temperature sensor data is sent in Hz. Zero means no send. Each sensor with source DroneCAN is sent in turn.
    // @Units: Hz
    // @Range: 0 200
    // @Increment: 1
    // @User: Standard
    GSCALAR(temperature_msg_rate, "TEMP_MSG_RATE", 0),
#endif

    // @Param: OPTIONS
    // @DisplayName: AP Periph Options
    // @Description: Bitmask of AP Periph Options
    // @Bitmask: 0: Enable continuous sensor probe
    // @User: Standard
    GSCALAR(options, "OPTIONS", AP_PERIPH_PROBE_CONTINUOUS),

#ifdef HAL_PERIPH_ENABLE_RPM_STREAM
    // @Param: RPM_MSG_RATE
    // @DisplayName: RPM sensor message rate
    // @Description: This is the rate RPM sensor data is sent in Hz. Zero means no send. Each sensor with a set ID is sent in turn.
    // @Units: Hz
    // @Range: 0 200
    // @Increment: 1
    // @User: Standard
    GSCALAR(rpm_msg_rate, "RPM_MSG_RATE", 0),
#endif

#if AP_EXTENDED_ESC_TELEM_ENABLED && HAL_WITH_ESC_TELEM
    // @Param: ESC_EXT_TLM_RATE
    // @DisplayName: ESC Extended telemetry message rate
    // @Description: This is the rate at which extended ESC Telemetry will be sent across the CAN bus for each ESC
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @User: Advanced
    GSCALAR(esc_extended_telem_rate, "ESC_EXT_TLM_RATE", AP_PERIPH_ESC_TELEM_RATE_DEFAULT / 10),
#endif

    AP_VAREND
};


void AP_Periph_FW::load_parameters(void)
{
    AP_Param::setup_sketch_defaults();

    AP_Param::check_var_info();

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {
        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
    }
    g.format_version.set_default(Parameters::k_format_version);

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
