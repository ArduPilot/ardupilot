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
/*
  FlySky IBUS2 protocol definitions.

  IBUS2 is a 1.5 Mbit/s half-duplex UART protocol. Each cycle has:
    Frame 1 (master→devices): channel values, variable length ≤37 bytes, PacketType=0
    Frame 2 (master→device):  command, fixed 21 bytes, PacketType=1
    Frame 3 (device→master):  response, fixed 21 bytes, PacketType=2

  CRC8 uses polynomial 0x125 (i.e. 0x25 with implicit leading 1 bit).
  Timing: Frame1→Frame2 gap 125±25 µs; Frame2→Frame3 gap 160±25 µs.

  Reference: IBU2_UART_Protocol_V2_4_en_.pdf,
             Telemetry_Adapter_Protocol___20240923_en_.pdf
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_IBUS2_ENABLED
#define AP_IBUS2_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 2048)
#endif

#if AP_IBUS2_ENABLED

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

// -----------------------------------------------------------------------
// Frame sizes
// -----------------------------------------------------------------------
#define IBUS2_FRAME1_MIN   4    // minimum: 1 hdr + 1 len + 1 addr + 1 crc
#define IBUS2_FRAME1_MAX   37   // maximum frame 1 length (per spec)
#define IBUS2_FRAME2_SIZE  21   // fixed: 1 hdr + 19 data + 1 crc
#define IBUS2_FRAME3_SIZE  21   // fixed: 1 hdr + 19 data + 1 crc

// -----------------------------------------------------------------------
// PacketType values (2-bit field at the top of the first byte)
// -----------------------------------------------------------------------
#define IBUS2_PKT_CHANNELS  0   // Frame 1: channel values
#define IBUS2_PKT_COMMAND   1   // Frame 2: host command
#define IBUS2_PKT_RESPONSE  2   // Frame 3: device response

// -----------------------------------------------------------------------
// Command / response codes (CommandCode, 6-bit field)
// -----------------------------------------------------------------------
enum class IBUS2Cmd : uint8_t {
    RESET       = 0,  // host resets device
    GET_TYPE    = 1,  // host queries device type / capabilities
    GET_VALUE   = 2,  // host requests sensor data
    GET_PARAM   = 3,  // host reads a device parameter
    SET_PARAM   = 4,  // host writes a device parameter
};

// -----------------------------------------------------------------------
// Sensor types (returned parameter name / sensor type table from spec)
// -----------------------------------------------------------------------
enum class IBUS2SensorType : uint8_t {
    ERROR_TYPE      = 0,
    VOLTAGE         = 1,
    CURRENT         = 2,
    RPM             = 3,
    AIR_PRESSURE    = 4,
    HYDRAULICS      = 5,
    SPEED           = 6,
    POWER           = 7,
    TIME            = 8,
    CONS_CAPACITY   = 9,
    REMA_CAPACITY   = 10,
    RUN_TIME        = 11,
    REMA_TIME       = 12,
    MAX_VOLTAGE     = 13,
    MIN_VOLTAGE     = 14,
    MAX_CURRENT     = 15,
    AVE_CURRENT     = 16,
    YAW_ANGLE       = 17,
    PITCH_ANGLE     = 18,
    ROLL_ANGLE      = 19,
    START_ANGLE     = 20,
    ALTITUDE        = 21,
    HEIGHT          = 22,
    DISTANCE        = 23,
    SIGNAL          = 24,
    CONDITION       = 25,
    FLOWMETER       = 26,
    TEMPERATURE     = 27,
    ANGLE           = 28,
    ANGULAR_SPEED   = 29,
    ACCELERATION    = 30,
    ANG_ACCEL       = 31,
    THROTTLE        = 32,
    AIRSPEED        = 33,
    MOTOR_TEMP      = 34,
    MCU_TEMP        = 35,
    MOS_TEMP        = 36,
    SBEC_VOLTAGE    = 37,
    ELEC_ANGLE      = 38,
    TIMING          = 39,
    NUM_CELLS       = 40,
};

// -----------------------------------------------------------------------
// Device return type (Appendix 5 of spec)
// -----------------------------------------------------------------------
enum class IBUS2DeviceType : uint8_t {
    DIGITAL_SERVO = 0xF8,
};

// -----------------------------------------------------------------------
// Frame 1: channel values (variable length ≤ 37 bytes total)
//
// Byte 0: PacketType:2 | PacketSubtype:4 | SynchronizationLost:1 | FailsafeTriggered:1
// Byte 1: Length (actual total frame length, max 37)
// Byte 2: AddressLevel1:3 | AddressLevel2:3 | Reserved:2
// Bytes 3..Length-2: Channels (compressed)
// Last byte: CRC8
// -----------------------------------------------------------------------
struct PACKED IBUS2_Frame1_Header {
    uint8_t pkt_type      : 2;  // must be IBUS2_PKT_CHANNELS (0)
    uint8_t subtype       : 4;  // 0=compressed channel data, 1=decompression key, 2=runaway
    uint8_t sync_lost     : 1;  // 1 = receiver lost sync with transmitter
    uint8_t failsafe      : 1;  // 1 = failsafe triggered
    uint8_t length;             // total frame length (including this header and CRC)
    uint8_t addr_level1   : 3;  // destination AddressLevel1
    uint8_t addr_level2   : 3;  // destination AddressLevel2
    uint8_t reserved      : 2;
};
static_assert(sizeof(IBUS2_Frame1_Header) == 3, "IBUS2_Frame1_Header size");

// -----------------------------------------------------------------------
// Frame 2: command (fixed 21 bytes)
//
// Byte 0: PacketType:2 (=1) | CommandCode:6
// Bytes 1-19: Data (command-specific)
// Byte 20: CRC8
// -----------------------------------------------------------------------
struct PACKED IBUS2_Frame2 {
    uint8_t pkt_type    : 2;  // must be IBUS2_PKT_COMMAND (1)
    uint8_t cmd_code    : 6;  // IBUS2Cmd
    uint8_t data[19];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Frame2) == 21, "IBUS2_Frame2 size");

// -----------------------------------------------------------------------
// Frame 3: response (fixed 21 bytes, same layout as Frame 2)
//
// Byte 0: PacketType:2 (=2) | CommandCode:6
// Bytes 1-19: Data (response-specific)
// Byte 20: CRC8
// -----------------------------------------------------------------------
struct PACKED IBUS2_Frame3 {
    uint8_t pkt_type    : 2;  // must be IBUS2_PKT_RESPONSE (2)
    uint8_t cmd_code    : 6;  // IBUS2Cmd echoed from Frame 2
    uint8_t data[19];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Frame3) == 21, "IBUS2_Frame3 size");

// -----------------------------------------------------------------------
// Frame 2 GET_TYPE command (CommandCode=1)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Cmd_GetType {
    uint8_t pkt_type    : 2;
    uint8_t cmd_code    : 6;  // = (uint8_t)IBUS2Cmd::GET_TYPE
    uint8_t reserved[19];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Cmd_GetType) == 21, "IBUS2_Cmd_GetType size");

// -----------------------------------------------------------------------
// Frame 3 GET_TYPE response (CommandCode=1)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Resp_GetType {
    uint8_t pkt_type          : 2;
    uint8_t cmd_code          : 6;  // = (uint8_t)IBUS2Cmd::GET_TYPE
    uint8_t type;                   // device type (IBUS2DeviceType)
    uint8_t value_length;           // max sensor data per packet (1-16)
    uint8_t channels_types    : 1;  // 1 = device needs channel type data
    uint8_t failsafe          : 1;  // 1 = device needs failsafe data
    uint8_t rx_internal_sens  : 1;  // 1 = device needs internal receiver sensor data
    uint8_t reserved_bits     : 5;
    uint8_t reserved[16];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Resp_GetType) == 21, "IBUS2_Resp_GetType size");

// -----------------------------------------------------------------------
// Frame 2 GET_VALUE command (CommandCode=2, no parameters)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Cmd_GetValue {
    uint8_t pkt_type    : 2;
    uint8_t cmd_code    : 6;  // = (uint8_t)IBUS2Cmd::GET_VALUE
    uint8_t reserved[19];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Cmd_GetValue) == 21, "IBUS2_Cmd_GetValue size");

// -----------------------------------------------------------------------
// Frame 3 GET_VALUE response (CommandCode=2)
// Value[14] contains packed sensor data (see spec §3 and Telemetry Adapter PDF §3.3)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Resp_GetValue {
    uint8_t pkt_type    : 2;
    uint8_t cmd_code    : 6;  // = (uint8_t)IBUS2Cmd::GET_VALUE
    uint8_t value[14];        // sensor data, format defined by device
    uint8_t vid;              // Vendor ID
    uint8_t pid;              // Product ID
    uint8_t reserved[3];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Resp_GetValue) == 21, "IBUS2_Resp_GetValue size");

// -----------------------------------------------------------------------
// Frame 2 GET_PARAM command (CommandCode=3)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Cmd_GetParam {
    uint8_t pkt_type        : 2;
    uint8_t cmd_code        : 6;  // = (uint8_t)IBUS2Cmd::GET_PARAM
    uint16_t param_type;
    uint8_t reserved[17];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Cmd_GetParam) == 21, "IBUS2_Cmd_GetParam size");

// -----------------------------------------------------------------------
// Frame 3 GET_PARAM response (CommandCode=3)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Resp_GetParam {
    uint8_t pkt_type        : 2;
    uint8_t cmd_code        : 6;  // = (uint8_t)IBUS2Cmd::GET_PARAM
    uint16_t param_type;
    uint8_t param_length;
    uint8_t param_value[16];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Resp_GetParam) == 21, "IBUS2_Resp_GetParam size");

// -----------------------------------------------------------------------
// Frame 2 SET_PARAM command (CommandCode=4)
// -----------------------------------------------------------------------
struct PACKED IBUS2_Cmd_SetParam {
    uint8_t pkt_type        : 2;
    uint8_t cmd_code        : 6;  // = (uint8_t)IBUS2Cmd::SET_PARAM
    uint16_t param_type;
    uint8_t param_length;
    uint8_t param_value[16];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Cmd_SetParam) == 21, "IBUS2_Cmd_SetParam size");

// -----------------------------------------------------------------------
// Frame 3 SET_PARAM response (CommandCode=4)
// param_length == 0 means parameter not supported; non-zero means success.
// -----------------------------------------------------------------------
struct PACKED IBUS2_Resp_SetParam {
    uint8_t pkt_type        : 2;
    uint8_t cmd_code        : 6;  // = (uint8_t)IBUS2Cmd::SET_PARAM
    uint16_t param_type;
    uint8_t param_length;   // 0 = not supported, non-zero = success
    uint8_t reserved[16];
    uint8_t crc8;
};
static_assert(sizeof(IBUS2_Resp_SetParam) == 21, "IBUS2_Resp_SetParam size");

// -----------------------------------------------------------------------
// CRC helpers
// -----------------------------------------------------------------------

// Compute CRC8 over buf[0..len-1] using polynomial 0x25 (IBUS2 spec: 0x125).
uint8_t ibus2_crc8(const uint8_t *buf, uint16_t len);

// Return true if buf[0..len-1] has valid CRC8 (last byte is the CRC).
bool ibus2_crc8_ok(const uint8_t *buf, uint16_t len);

// Write CRC byte into buf[len-1] covering buf[0..len-2].
void ibus2_crc8_write(uint8_t *buf, uint16_t len);

#endif  // AP_IBUS2_ENABLED
