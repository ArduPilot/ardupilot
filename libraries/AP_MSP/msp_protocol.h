/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

/* Protocol numbers used both by the wire format, config system, and
   field setters.
*/

#define MSP_PROTOCOL_VERSION                0

#define API_VERSION_MAJOR                   1
#define API_VERSION_MINOR                   42 // for compatibility with DJI OSD

#define API_VERSION_LENGTH                  2

#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
#define BETAFLIGHT_IDENTIFIER "BTFL"
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"
#define ARDUPILOT_IDENTIFIER "ARDU"

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

#define MSP_API_VERSION                 1    //out message
#define MSP_FC_VARIANT                  2    //out message
#define MSP_FC_VERSION                  3    //out message
#define MSP_BOARD_INFO                  4    //out message
#define MSP_BUILD_INFO                  5    //out message

#define MSP_NAME                        10   //out message          Returns user set board name - betaflight
#define MSP_SET_NAME                    11   //in message           Sets board name - betaflight

//
// MSP commands for Cleanflight original features
//
#define MSP_BATTERY_CONFIG              32
#define MSP_SET_BATTERY_CONFIG          33

#define MSP_MODE_RANGES                 34    //out message         Returns all mode ranges
#define MSP_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define MSP_FEATURE_CONFIG              36
#define MSP_SET_FEATURE_CONFIG          37

#define MSP_BOARD_ALIGNMENT_CONFIG      38
#define MSP_SET_BOARD_ALIGNMENT_CONFIG  39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER_CONFIG                42
#define MSP_SET_MIXER_CONFIG            43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

#define MSP_ADJUSTMENT_RANGES           52
#define MSP_SET_ADJUSTMENT_RANGE        53

// private - only to be used by the configurator, the commands are likely to change
#define MSP_CF_SERIAL_CONFIG            54
#define MSP_SET_CF_SERIAL_CONFIG        55

#define MSP_VOLTAGE_METER_CONFIG        56
#define MSP_SET_VOLTAGE_METER_CONFIG    57

#define MSP_SONAR_ALTITUDE              58 //out message get sonar altitude [cm]

#define MSP_PID_CONTROLLER              59
#define MSP_SET_PID_CONTROLLER          60

#define MSP_ARMING_CONFIG               61
#define MSP_SET_ARMING_CONFIG           62

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define MSP_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from MSP_RX_MAP

// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
// DEPRECATED - #define MSP_BF_CONFIG                   66 //out message baseflight-specific settings that aren't covered elsewhere
// DEPRECATED - #define MSP_SET_BF_CONFIG               67 //in message baseflight-specific settings save

#define MSP_REBOOT                      68 //in message reboot settings

// Use MSP_BUILD_INFO instead
// DEPRECATED - #define MSP_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

#define MSP_DATAFLASH_SUMMARY           70 //out message - get description of dataflash chip
#define MSP_DATAFLASH_READ              71 //out message - get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72 //in message - erase dataflash chip

// No-longer needed
// DEPRECATED - #define MSP_LOOP_TIME                   73 //out message         Returns FC cycle time i.e looptime parameter // DEPRECATED
// DEPRECATED - #define MSP_SET_LOOP_TIME               74 //in message          Sets FC cycle time i.e looptime parameter    // DEPRECATED

#define MSP_FAILSAFE_CONFIG             75 //out message         Returns FC Fail-Safe settings
#define MSP_SET_FAILSAFE_CONFIG         76 //in message          Sets FC Fail-Safe settings

#define MSP_RXFAIL_CONFIG               77 //out message         Returns RXFAIL settings
#define MSP_SET_RXFAIL_CONFIG           78 //in message          Sets RXFAIL settings

#define MSP_SDCARD_SUMMARY              79 //out message         Get the state of the SD card

#define MSP_BLACKBOX_CONFIG             80 //out message         Get blackbox settings
#define MSP_SET_BLACKBOX_CONFIG         81 //in message          Set blackbox settings

#define MSP_TRANSPONDER_CONFIG          82 //out message         Get transponder settings
#define MSP_SET_TRANSPONDER_CONFIG      83 //in message          Set transponder settings

#define MSP_OSD_CONFIG                  84 //out message         Get osd settings - betaflight
#define MSP_SET_OSD_CONFIG              85 //in message          Set osd settings - betaflight

#define MSP_OSD_CHAR_READ               86 //out message         Get osd settings - betaflight
#define MSP_OSD_CHAR_WRITE              87 //in message          Set osd settings - betaflight

#define MSP_VTX_CONFIG                  88 //out message         Get vtx settings - betaflight
#define MSP_SET_VTX_CONFIG              89 //in message          Set vtx settings - betaflight

// Betaflight Additional Commands
#define MSP_ADVANCED_CONFIG             90
#define MSP_SET_ADVANCED_CONFIG         91

#define MSP_FILTER_CONFIG               92
#define MSP_SET_FILTER_CONFIG           93

#define MSP_PID_ADVANCED                94
#define MSP_SET_PID_ADVANCED            95

#define MSP_SENSOR_CONFIG               96
#define MSP_SET_SENSOR_CONFIG           97

#define MSP_CAMERA_CONTROL              98

#define MSP_SET_ARMING_DISABLED         99

//
// OSD specific
//
#define MSP_OSD_VIDEO_CONFIG            180
#define MSP_SET_OSD_VIDEO_CONFIG        181

// External OSD displayport mode messages
#define MSP_DISPLAYPORT                 182

#define MSP_COPY_PROFILE                183

#define MSP_BEEPER_CONFIG               184
#define MSP_SET_BEEPER_CONFIG           185

#define MSP_SET_TX_INFO                 186 // in message           Used to send runtime information from TX lua scripts to the firmware
#define MSP_TX_INFO                     187 // out message          Used by TX lua scripts to read information from the firmware

//
// Multwii original MSP commands
//

// See MSP_API_VERSION and MSP_MIXER_CONFIG
//DEPRECATED - #define MSP_IDENT                100    //out message         mixerMode + multiwii version + protocol version + capability variable


#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         servos
#define MSP_MOTOR                104    //out message         motors
#define MSP_RC                   105    //out message         rc channels and more
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer
#define MSP_ANALOG               110    //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         P I D coeff (9 are used currently)
// Legacy Multiicommand that was never used.
//DEPRECATED - #define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_MISC                 114    //out message         powermeter trig
// Legacy Multiicommand that was never used and always wrong
//DEPRECATED - #define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119    //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONFIGURATIONS 120    //out message         All servo configurations.
#define MSP_NAV_STATUS           121    //out message         Returns navigation status
#define MSP_NAV_CONFIG           122    //out message         Returns navigation parameters
#define MSP_MOTOR_3D_CONFIG      124    //out message         Settings needed for reversible ESCs
#define MSP_RC_DEADBAND          125    //out message         deadbands for yaw alt pitch roll
#define MSP_SENSOR_ALIGNMENT     126    //out message         orientation of acc,gyro,mag
#define MSP_LED_STRIP_MODECOLOR  127    //out message         Get LED strip mode_color settings
#define MSP_VOLTAGE_METERS       128    //out message         Voltage (per meter)
#define MSP_CURRENT_METERS       129    //out message         Amperage (per meter)
#define MSP_BATTERY_STATE        130    //out message         Connected/Disconnected, Voltage, Current Used
#define MSP_MOTOR_CONFIG         131    //out message         Motor configuration (min/max throttle, etc)
#define MSP_GPS_CONFIG           132    //out message         GPS configuration
#define MSP_COMPASS_CONFIG       133    //out message         Compass configuration
#define MSP_ESC_SENSOR_DATA      134    //out message         Extra ESC data from 32-Bit ESCs (Temperature, RPM)
#define MSP_GPS_RESCUE           135    //out message         GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
#define MSP_GPS_RESCUE_PIDS      136    //out message         GPS Rescues's throttleP and velocity PIDS + yaw P
#define MSP_VTXTABLE_BAND        137    //out message         vtxTable band/channel data
#define MSP_VTXTABLE_POWERLEVEL  138    //out message         vtxTable powerLevel data
#define MSP_MOTOR_TELEMETRY      139    //out message         Per-motor telemetry data (RPM, packet stats, ESC temp, etc.)

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          P I D coeff (9 are used currently)
// Legacy multiiwii command that was never used.
//DEPRECATED - #define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_SET_WP               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    //in message          Select Setting Number (0-2)
#define MSP_SET_HEADING          211    //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONFIGURATION 212    //in message          Servo settings
#define MSP_SET_MOTOR            214    //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215    //in message          Sets nav config parameters - write to the eeprom
#define MSP_SET_MOTOR_3D_CONFIG  217    //in message          Settings needed for reversible ESCs
#define MSP_SET_RC_DEADBAND      218    //in message          deadbands for yaw alt pitch roll
#define MSP_SET_RESET_CURR_PID   219    //in message          resetting the current pid profile to defaults
#define MSP_SET_SENSOR_ALIGNMENT 220    //in message          set the orientation of the acc,gyro,mag
#define MSP_SET_LED_STRIP_MODECOLOR 221 //in  message         Set LED strip mode_color settings
#define MSP_SET_MOTOR_CONFIG     222    //out message         Motor configuration (min/max throttle, etc)
#define MSP_SET_GPS_CONFIG       223    //out message         GPS configuration
#define MSP_SET_COMPASS_CONFIG   224    //out message         Compass configuration
#define MSP_SET_GPS_RESCUE       225    //in message          GPS Rescues's angle, initialAltitude, descentDistance, rescueGroundSpeed, sanityChecks and minSats
#define MSP_SET_GPS_RESCUE_PIDS  226    //in message          GPS Rescues's throttleP and velocity PIDS + yaw P
#define MSP_SET_VTXTABLE_BAND    227    //in message          set vtxTable band/channel data (one band at a time)
#define MSP_SET_VTXTABLE_POWERLEVEL 228 //in message          set vtxTable powerLevel data (one powerLevel at a time)

// #define MSP_BIND                 240    //in message          no param
// #define MSP_ALARMS               242

#define MSP_EEPROM_WRITE         250    //in message          no param
#define MSP_RESERVE_1            251    //reserved for system usage
#define MSP_RESERVE_2            252    //reserved for system usage
#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4
#define MSP_V2_FRAME             255    //MSPv2 payload indicator

// Additional commands that are not compatible with MultiWii
#define MSP_STATUS_EX            150    //out message         cycletime, errors_count, CPU load, sensor present etc
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)
#define MSP_GPSSTATISTICS        166    //out message         get GPS debugging data
#define MSP_MULTIPLE_MSP         230    //out message         request multiple MSPs in one request - limit is the TX buffer; returns each MSP in the order they were requested starting with length of MSP; MSPs with input arguments are not supported
#define MSP_MODE_RANGES_EXTRA    238    //out message         Reads the extra mode range data
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_SERVO_MIX_RULES      241    //out message         Returns servo mixer configuration
#define MSP_SET_SERVO_MIX_RULE   242    //in message          Sets servo mixer configuration
#define MSP_SET_PASSTHROUGH      245    //in message          Sets up passthrough to different peripherals (4way interface, uart, etc...)
#define MSP_SET_RTC              246    //in message          Sets the RTC clock
#define MSP_RTC                  247    //out message         Gets the RTC clock
#define MSP_SET_BOARD_INFO       248    //in message          Sets the board information for this board
#define MSP_SET_SIGNATURE        249    //in message          Sets the signature of the board and serial number

// V2 commands
#define MSP2_SENSOR_RANGEFINDER     0x1F01
#define MSP2_SENSOR_OPTIC_FLOW      0x1F02
#define MSP2_SENSOR_GPS             0x1F03
#define MSP2_SENSOR_COMPASS         0x1F04
#define MSP2_SENSOR_BAROMETER       0x1F05
#define MSP2_SENSOR_AIRSPEED        0x1F06
