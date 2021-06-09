#include "AP_Periph.h"

extern const AP_HAL::HAL &hal;

#ifndef HAL_PERIPH_LED_BRIGHT_DEFAULT
#define HAL_PERIPH_LED_BRIGHT_DEFAULT 100
#endif

#ifndef HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT
#define HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT 115200
#endif

#ifndef HAL_PERIPH_RANGEFINDER_PORT_DEFAULT
#define HAL_PERIPH_RANGEFINDER_PORT_DEFAULT 3
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

/*
 *  AP_Periph parameter definitions
 *
 */

#define GSCALAR(v, name, def) { periph.g.v.vtype, name, Parameters::k_param_ ## v, &periph.g.v, {def_value : def} }
#define GARRAY(v, index, name, def) { periph.g.v[index].vtype, name, Parameters::k_param_ ## v ## index, &periph.g.v[index], {def_value : def} }
#define ASCALAR(v, name, def) { periph.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&periph.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &periph.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&periph.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&periph.v, {group_info : class::var_info} }

const AP_Param::Info AP_Periph_FW::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: CAN_NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly or 0 for dynamic node allocation
    // @Range: 0 250
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

#if HAL_NUM_CAN_IFACES >= 2
    // @Param: CAN_PROTOCOL
    // @DisplayName: Enable use of specific protocol to be used on this port
    // @Description: Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN
    // @Values: 0:Disabled,1:UAVCAN,3:ToshibaCAN,4:PiccoloCAN,5:CANTester,6:EFI_NWPMU,7:USD1,8:KDECAN,9:PacketDigital
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_protocol,     0, "CAN_PROTOCOL", AP_CANManager::Driver_Type_UAVCAN),
    
    // @Param: CAN2_BAUDRATE
    // @DisplayName: Bitrate of CAN2 interface
    // @Description: Bit rate can be set up to from 10000 to 1000000
    // @Range: 10000 1000000
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_baudrate,     1, "CAN2_BAUDRATE", 1000000),

    // @Param: CAN2_PROTOCOL
    // @DisplayName: Enable use of specific protocol to be used on this port
    // @Description: Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN
    // @Values: 0:Disabled,1:UAVCAN,3:ToshibaCAN,4:PiccoloCAN,5:CANTester,6:EFI_NWPMU,7:USD1,8:KDECAN,9:PacketDigital
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_protocol,     1, "CAN2_PROTOCOL", AP_CANManager::Driver_Type_UAVCAN),
#endif

#if HAL_NUM_CAN_IFACES >= 3
    // @Param: CAN3_BAUDRATE
    // @DisplayName: Bitrate of CAN3 interface
    // @Description: Bit rate can be set up to from 10000 to 1000000
    // @Range: 10000 1000000
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_baudrate,    2, "CAN3_BAUDRATE", 1000000),

    // @Param: CAN3_PROTOCOL
    // @DisplayName: Enable use of specific protocol to be used on this port
    // @Description: Enabling this option starts selected protocol that will use this virtual driver. At least one CAN port must be UAVCAN or else CAN1 gets set to UAVCAN
    // @Values: 0:Disabled,1:UAVCAN,3:ToshibaCAN,4:PiccoloCAN,5:CANTester,6:EFI_NWPMU,7:USD1,8:KDECAN,9:PacketDigital
    // @User: Advanced
    // @RebootRequired: True
    GARRAY(can_protocol,    2, "CAN3_PROTOCOL", AP_CANManager::Driver_Type_UAVCAN),
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
    // @Values: 0:Disabled, 1:Show free stack space
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

#ifdef HAL_PERIPH_ENABLE_GPS
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
#endif

#ifdef HAL_PERIPH_ENABLE_BATTERY
    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery, "BATT", AP_BattMonitor),
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
    GSCALAR(baro_enable, "BARO_ENABLE", 1),
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
    // @Group: ARSP
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed, "ARSP", AP_Airspeed),
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    // @Param: RNGFND_BAUDRATE
    // @DisplayName: Rangefinder serial baudrate
    // @Description: Rangefinder serial baudrate.
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200,230:230400,256:256000,460:460800,500:500000,921:921600,1500:1500000
    // @Increment: 1
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(rangefinder_baud, "RNGFND_BAUDRATE", HAL_PERIPH_RANGEFINDER_BAUDRATE_DEFAULT),

    // @Param: RNGFND_PORT
    // @DisplayName: Rangefinder Serial Port
    // @Description: This is the serial port number where SERIALx_PROTOCOL will be set to Rangefinder.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(rangefinder_port, "RNGFND_PORT", HAL_PERIPH_RANGEFINDER_PORT_DEFAULT),

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

#ifdef HAL_PERIPH_ENABLE_HWESC
    // @Param: ESC_NUMBER
    // @DisplayName: ESC number
    // @Description: This is the ESC number to report as in UAVCAN ESC telemetry feedback packets.
    // @Increment: 1
    // @User: Advanced
    GSCALAR(esc_number, "ESC_NUMBER", 0),
#endif

#ifdef HAL_PERIPH_ENABLE_RC_OUT
    // Servo driver
    // @Group: OUT
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    GOBJECT(servo_channels, "OUT",     SRV_Channels),

    // @Param: ESC_PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed,4:DShot150,5:DShot300,6:DShot600,7:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(esc_pwm_type, "ESC_PWM_TYPE",     0),
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

    AP_VAREND
};


void AP_Periph_FW::load_parameters(void)
{
    AP_Param::setup_sketch_defaults();

    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {
        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
    }

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
