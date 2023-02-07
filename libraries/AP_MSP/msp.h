#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED !HAL_MINIMIZE_FEATURES
#endif

// define for enabling MSP sensor drivers
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED HAL_MSP_ENABLED && !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH)
#endif

// define for enabling MSP DisplayPort
#ifndef HAL_WITH_MSP_DISPLAYPORT
#define HAL_WITH_MSP_DISPLAYPORT  HAL_MSP_ENABLED && !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH)
#endif

#include <AP_HAL/UARTDriver.h>

#include "msp_osd.h"
#include "msp_protocol.h"
#include "msp_sbuf.h"
#include "msp_version.h"
#include "msp_sensors.h"

#if HAL_MSP_ENABLED

// betaflight/src/main/common/utils.h
#define MSP_ARRAYEND(x) (&(x)[ARRAY_SIZE(x)])
#define MSP_UNUSED(x) (void)(x)
// betaflight/src/main/msp/msp_serial.c
#define JUMBO_FRAME_SIZE_LIMIT 255
// betaflight/src/main/msp/msp.h
#define MSP_V2_FRAME_ID 255
#define MSP_VERSION_MAGIC_INITIALIZER { 'M', 'M', 'X' }
#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE 512
#define MSP_MAX_HEADER_SIZE     9
// inav/src/main/msp/msp_protocol_v2_sensor.h
#define MSP2_IS_SENSOR_MESSAGE(x)   ((x) >= 0x1F00U && (x) <= 0x1FFFU)
// betaflight/src/main/io/displayport_msp.h
// MSP displayport V2 attribute byte bit functions
#define DISPLAYPORT_MSP_ATTR_VERSION 1U<<7 // Format indicator; must be zero for V2 (and V1)
#define DISPLAYPORT_MSP_ATTR_BLINK   1U<<6 // Device local blink
#define DISPLAYPORT_MSP_ATTR_MASK    (~(DISPLAYPORT_MSP_ATTR_VERSION|DISPLAYPORT_MSP_ATTR_BLINK))
// betaflight/src/main/io/displayport_msp.c
#define OSD_MSP_DISPLAYPORT_MAX_STRING_LENGTH 30

class AP_MSP_Telem_Backend;

namespace MSP
{
typedef enum {
    MSP_V1          = 0,
    MSP_V2_OVER_V1  = 1,
    MSP_V2_NATIVE   = 2,
    MSP_VERSION_COUNT
} msp_version_e;

// return positive for ACK, negative on error, zero for no reply
typedef enum {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0
} MSPCommandResult;

typedef struct msp_packet_s {
    sbuf_t buf;
    int16_t cmd;
    uint8_t flags;
    int16_t result;
} msp_packet_t;

typedef enum {
    MSP_FLAG_DONT_REPLY           = (1 << 0),
} msp_flags_e;

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} msp_state_e;

typedef enum : uint8_t {
    MSP_DISPLAYPORT_HEARTBEAT = 0,
    MSP_DISPLAYPORT_RELEASE = 1,
    MSP_DISPLAYPORT_CLEAR_SCREEN = 2,
    MSP_DISPLAYPORT_WRITE_STRING = 3,
    MSP_DISPLAYPORT_DRAW_SCREEN = 4,
    MSP_DISPLAYPORT_SET_OPTIONS = 5,
} msp_displayport_subcmd_e;

typedef struct PACKED {
    uint8_t size;
    uint8_t cmd;
} msp_header_v1_t;

typedef struct PACKED {
    uint16_t size;
} msp_header_jumbo_t;

typedef struct PACKED {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} msp_header_v2_t;

typedef struct msp_port_s {
    AP_HAL::UARTDriver *uart;
    msp_state_e c_state;
    uint8_t in_buf[MSP_PORT_INBUF_SIZE];
    uint_fast16_t offset;
    uint_fast16_t data_size;
    msp_version_e msp_version;
    uint8_t cmd_flags;
    uint16_t cmd_msp;
    uint8_t checksum1;
    uint8_t checksum2;
} msp_port_t;

// betaflight/src/main/sensors/battery.h
typedef enum : uint8_t {
    MSP_BATTERY_OK = 0,
    MSP_BATTERY_WARNING,
    MSP_BATTERY_CRITICAL,
    MSP_BATTERY_NOT_PRESENT,
    MSP_BATTERY_INIT
} battery_state_e;

uint8_t msp_serial_checksum_buf(uint8_t checksum, const uint8_t *data, uint32_t len);
uint32_t msp_serial_send_frame(msp_port_t *msp, const uint8_t * hdr, uint32_t hdr_len, const uint8_t * data, uint32_t data_len, const uint8_t * crc, uint32_t crc_len);
uint32_t msp_serial_encode(msp_port_t *msp, msp_packet_t *packet, msp_version_e msp_version, bool is_request=false);
bool msp_parse_received_data(msp_port_t *msp, uint8_t c);
}

#endif //HAL_MSP_ENABLED
