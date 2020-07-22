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
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "frsky.h"

#include <stdint.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

#define MAVLITE_MAX_PAYLOAD_LEN                 31 // 7 float params + cmd_id + options
#define MAVLITE_MSG_SPORT_PACKETS_COUNT(LEN)    ((uint8_t)(1 + ceilf((LEN-2)/5.0f))) // number of sport packets required to transport a message with LEN payload
#define SPORT_PACKET_QUEUE_LENGTH               ((uint8_t)30U*MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN))

namespace MAVLITE
{
typedef struct {
    uint8_t msgid = 0;      // ID of message in payload
    uint8_t len = 0;        // Length of payload
    uint8_t payload[MAVLITE_MAX_PAYLOAD_LEN];
    int16_t checksum = 0;   // sent at end of packet
} mavlite_message_t;

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

void mavlite_init(mavlite_message_t &msg, mavlite_status_t &status);
bool mavlite_rx_parse(uint8_t byte, uint8_t offset, mavlite_message_t &rxmsg, mavlite_status_t &status);
bool mavlite_tx_encode(uint8_t &byte, uint8_t offset, mavlite_message_t &txmsg, mavlite_status_t &status);
void mavlite_update_checksum(mavlite_message_t &msg, uint8_t c);

bool mavlite_msg_get_bytes(uint8_t *bytes, const mavlite_message_t &msg, uint8_t offset, uint8_t count);
bool mavlite_msg_set_bytes(mavlite_message_t &msg, const uint8_t *bytes,  uint8_t offset, uint8_t count);

bool mavlite_msg_get_float(float &value, const mavlite_message_t &msg, uint8_t offset);
bool mavlite_msg_set_float(mavlite_message_t &msg, float value, uint8_t offset);

bool mavlite_msg_get_string(char* value, const mavlite_message_t &msg, uint8_t offset);
bool mavlite_msg_set_string(mavlite_message_t &msg, char* value, uint8_t offset);

bool mavlite_msg_get_uint16(uint16_t &value, const mavlite_message_t &msg, uint8_t offset);
bool mavlite_msg_set_uint16(mavlite_message_t &msg,uint16_t value, uint8_t offset);

bool mavlite_msg_get_uint8(uint8_t &value, const mavlite_message_t &msg, uint8_t offset);
bool mavlite_msg_set_uint8(mavlite_message_t &msg,uint8_t value, uint8_t offset);

void bit8_pack(uint8_t &value, uint8_t bit_value, uint8_t bit_count,uint8_t bit_offset);
uint8_t bit8_unpack(uint8_t value, uint8_t bit_count,uint8_t bit_offset);
}

#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL