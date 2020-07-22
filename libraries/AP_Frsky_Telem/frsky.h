#pragma once

#include <AP_HAL/UARTDriver.h>

#include <stdint.h>

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 1
#endif

// for SPort X protocol
#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20
// for SPort D protocol
#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D
// for SPort packet parser
#define TELEMETRY_RX_BUFFER_SIZE    19U  // 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
#define SPORT_PACKET_SIZE           9U
#define STUFF_MASK                  0x20
#define SPORT_DATA_FRAME            0x10
#define SPORT_UPLINK_FRAME          0x30
#define SPORT_UPLINK_FRAME_RW       0x31
#define SPORT_DOWNLINK_FRAME        0x32

namespace FRSky
{
typedef union {
    struct PACKED {
        uint8_t sensor;
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
    };
    uint8_t raw[8];
} sport_packet_t;

enum class ParseState : uint8_t {
    IDLE,
    START,
    IN_FRAME,
    XOR,
};

typedef struct PACKED {
    uint8_t telemetry_rx_buffer_count;
    uint8_t telemetry_rx_buffer[TELEMETRY_RX_BUFFER_SIZE];
    ParseState state;
} sport_parse_state_t;

// serial tx helpers
void send_byte(AP_HAL::UARTDriver *port, uint8_t value);
void send_uint16(AP_HAL::UARTDriver *port, uint16_t id, uint16_t data);
void send_sport_frame(AP_HAL::UARTDriver *port, uint8_t frame, uint16_t appid, uint32_t data);
// packet parser helpers
bool parse_sport_telemetry_data(uint8_t data, sport_parse_state_t &parse_state);
bool should_process_sport_packet(const uint8_t *packet);
bool should_process_sport_packet(const uint8_t *packet, const uint8_t *previous_packet);
// sensor id helper
uint8_t get_sport_sensor_id(uint8_t physical_id);
}