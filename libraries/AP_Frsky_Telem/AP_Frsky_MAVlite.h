#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_Frsky_SPort.h"

#include <stdint.h>

#define MAVLITE_MAX_PAYLOAD_LEN                 31 // 7 float params + cmd_id + options
#define MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)    static_cast<uint8_t>(1 + ceilf((LEN-2)/5.0f)) // number of sport packets required to transport a message with LEN payload
#define SPORT_PACKET_QUEUE_LENGTH               static_cast<uint8_t>(30U*MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN))

class AP_Frsky_MAVlite
{
public:
    typedef struct {
        uint8_t msgid = 0;                          // ID of message in payload
        uint8_t len = 0;                            // Length of payload
        uint8_t payload[MAVLITE_MAX_PAYLOAD_LEN];
        int16_t checksum = 0;                       // sent at end of packet
    } mavlite_message_t;

    // public parsing methods
    bool parse(mavlite_message_t &rxmsg, const AP_Frsky_SPort::sport_packet_t packet) ;
    bool encode(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue, mavlite_message_t &msg);

    // public mavlite message helpers
    static bool mavlite_msg_get_float(float &value, const mavlite_message_t &msg, const uint8_t offset);
    static bool mavlite_msg_set_float(mavlite_message_t &msg, const float value, const uint8_t offset);

    static bool mavlite_msg_get_string(char* value, const mavlite_message_t &msg, const uint8_t offset);
    static bool mavlite_msg_set_string(mavlite_message_t &msg, const char* value, const uint8_t offset);

    static bool mavlite_msg_get_uint16(uint16_t &value, const mavlite_message_t &msg, const uint8_t offset);
    static bool mavlite_msg_set_uint16(mavlite_message_t &msg, const uint16_t value, const uint8_t offset);

    static bool mavlite_msg_get_uint8(uint8_t &value, const mavlite_message_t &msg, const uint8_t offset);
    static bool mavlite_msg_set_uint8(mavlite_message_t &msg, const uint8_t value, const uint8_t offset);

    static void bit8_pack(uint8_t &value, const uint8_t bit_value, const uint8_t bit_count, const uint8_t bit_offset);
    static uint8_t bit8_unpack(const uint8_t value, const  uint8_t bit_count, const uint8_t bit_offset);

private:
    enum class ParseState : uint8_t {
        IDLE=0,
        ERROR,
        GOT_START,
        GOT_LEN,
        GOT_SEQ,
        GOT_MSGID,
        GOT_PAYLOAD,
        MESSAGE_RECEIVED,
    }; // state machine for mavlite messages

    typedef struct {
        ParseState parse_state = ParseState::IDLE;
        uint8_t current_rx_seq = 0;
        uint8_t payload_next_byte = 0;
    } mavlite_status_t;

    mavlite_status_t _txstatus;
    mavlite_status_t _rxstatus;

    mavlite_message_t _rxmsg;

    static bool mavlite_msg_get_bytes(uint8_t *bytes, const mavlite_message_t &msg, const uint8_t offset, const uint8_t count);
    static bool mavlite_msg_set_bytes(mavlite_message_t &msg, const uint8_t *bytes,  const uint8_t offset, const uint8_t count);

    void encoder_reset(mavlite_message_t &txmsg);
    void parser_reset();
    bool parse(const uint8_t byte, const uint8_t offset);
    bool encode(uint8_t &byte, uint8_t offset, mavlite_message_t &txmsg);
    void update_checksum(mavlite_message_t &msg, const uint8_t c);
};
