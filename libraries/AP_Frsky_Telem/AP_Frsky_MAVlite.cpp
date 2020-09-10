#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_Frsky_MAVlite.h"

extern const AP_HAL::HAL& hal;

void AP_Frsky_MAVlite::update_checksum(mavlite_message_t &msg, const uint8_t c)
{
    msg.checksum += c; //0-1FF
    msg.checksum += msg.checksum >> 8;
    msg.checksum &= 0xFF;
}

void AP_Frsky_MAVlite::parser_reset(void)
{
    _rxmsg.checksum = 0;
    _rxmsg.len = 0;
    _rxmsg.msgid = 0;

    _rxstatus.current_rx_seq = 0;
    _rxstatus.payload_next_byte = 0;
    _rxstatus.parse_state = ParseState::IDLE;
}

void AP_Frsky_MAVlite::encoder_reset(mavlite_message_t &txmsg)
{
    txmsg.checksum = 0;
    
    _txstatus.current_rx_seq = 0;
    _txstatus.payload_next_byte = 0;
    _txstatus.parse_state = ParseState::IDLE;
}

/*
 Parses sport packets and if successfull fills the rxmsg mavlite struct
 */
bool AP_Frsky_MAVlite::parse(mavlite_message_t &rxmsg, const AP_Frsky_SPort::sport_packet_t packet)
{
    for (uint8_t i=0; i<6; i++) {
        parse(packet.raw[i+2], i);
    }
    if (_rxstatus.parse_state == ParseState::MESSAGE_RECEIVED) {
        rxmsg = _rxmsg;
        return true;
    }
    return false;
}

/*
 Warning:
 make sure that all packets pushed by this method are sequential and not interleaved by packets inserted by another thread!
 */
bool AP_Frsky_MAVlite::encode(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue, mavlite_message_t &msg)
{
    // let's check if there's enough room to send it
    if (queue.space() < MAVLITE_MSG_SPORT_PACKETS_COUNT(msg.len)) {
        return false;
    }
    encoder_reset(msg);
    // prevent looping forever
    uint8_t packet_count = 0;
    while (_txstatus.parse_state != ParseState::MESSAGE_RECEIVED && packet_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {

        AP_Frsky_SPort::sport_packet_t packet {0};

        for (uint8_t i=0; i<6; i++) {
            // read msg and fill packet one byte at the time, ignore sensor and frame byte
            encode(packet.raw[i+2], i, msg);
        }

        if (_txstatus.parse_state == ParseState::ERROR) {
            break;
        }

        queue.push(packet);
    }
    _txstatus.parse_state = ParseState::IDLE;
    return true;
}

bool AP_Frsky_MAVlite::parse(uint8_t byte, uint8_t offset)
{
    switch (_rxstatus.parse_state) {
    case ParseState::IDLE:
    case ParseState::ERROR:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        } else {
            _rxstatus.parse_state = ParseState::ERROR;
        }
        break;
    case ParseState::GOT_START:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        } else {
            _rxmsg.len = byte;
            _rxstatus.parse_state = ParseState::GOT_LEN;
            update_checksum(_rxmsg, byte);
        }
        break;
    case ParseState::GOT_LEN:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        } else {
            _rxmsg.msgid = byte;
            _rxstatus.parse_state = ParseState::GOT_MSGID;
            update_checksum(_rxmsg, byte);
        }
        break;
    case ParseState::GOT_MSGID:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        } else {
            _rxmsg.payload[_rxstatus.payload_next_byte++] = byte;
            _rxstatus.parse_state = ParseState::GOT_PAYLOAD;
            update_checksum(_rxmsg, byte);
        }
        break;
    case ParseState::GOT_SEQ:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        } else {
            if ( _rxstatus.payload_next_byte < _rxmsg.len ) {
                _rxmsg.payload[_rxstatus.payload_next_byte++] = byte;
                _rxstatus.parse_state = ParseState::GOT_PAYLOAD;
                update_checksum(_rxmsg, byte);
            } else {
                if ( _rxmsg.checksum == byte ) {
                    _rxstatus.parse_state = ParseState::MESSAGE_RECEIVED;
                    return true;
                } else {
                    _rxstatus.parse_state = ParseState::ERROR;
                }
            }
        }
        break;
    case ParseState::GOT_PAYLOAD:
        if ( offset == 0) {
            if ( byte == 0x00 ) {
                parser_reset();
                _rxstatus.parse_state = ParseState::GOT_START;
            } else {
                if ((byte & 0x3F) != _rxstatus.current_rx_seq + 1) {
                    _rxstatus.parse_state = ParseState::ERROR;
                } else {
                    _rxstatus.current_rx_seq = (byte & 0x3F);
                    _rxstatus.parse_state = ParseState::GOT_SEQ;
                }
                update_checksum(_rxmsg, byte);
            }
        } else {
            if ( _rxstatus.payload_next_byte < _rxmsg.len ) {
                _rxmsg.payload[_rxstatus.payload_next_byte++] = byte;
                update_checksum(_rxmsg, byte);
            } else {
                if ( _rxmsg.checksum == byte ) {
                    _rxstatus.parse_state = ParseState::MESSAGE_RECEIVED;
                    return true;
                } else {
                    _rxstatus.parse_state = ParseState::ERROR;
                }
            }
        }
        break;
    case ParseState::MESSAGE_RECEIVED:
        if ( offset == 0 && byte == 0x00 ) {
            parser_reset();
            _rxstatus.parse_state = ParseState::GOT_START;
        }
        break;
    }
    return false;
}

bool AP_Frsky_MAVlite::encode(uint8_t &byte, const uint8_t offset, mavlite_message_t &txmsg)
{
    switch (_txstatus.parse_state) {
    case ParseState::IDLE:
    case ParseState::ERROR:
        if ( offset == 0 ) {
            byte = 0x00;
            encoder_reset(txmsg);
            _txstatus.parse_state = ParseState::GOT_START;
        } else {
            _txstatus.parse_state = ParseState::ERROR;
        }
        break;
    case ParseState::GOT_START:
        byte = txmsg.len;
        _txstatus.parse_state = ParseState::GOT_LEN;
        update_checksum(txmsg, byte);
        break;
    case ParseState::GOT_LEN:
        byte = txmsg.msgid;
        _txstatus.parse_state = ParseState::GOT_MSGID;
        update_checksum(txmsg, byte);
        break;
    case ParseState::GOT_MSGID:
        byte = txmsg.payload[_txstatus.payload_next_byte++];
        _txstatus.parse_state = ParseState::GOT_PAYLOAD;
        update_checksum(txmsg, byte);
        break;
    case ParseState::GOT_SEQ:
        if ( _txstatus.payload_next_byte < txmsg.len ) {
            byte = txmsg.payload[_txstatus.payload_next_byte++];
            _txstatus.parse_state = ParseState::GOT_PAYLOAD;
            update_checksum(txmsg, byte);
        } else {
            byte = txmsg.checksum;
            _txstatus.parse_state = ParseState::MESSAGE_RECEIVED;
            return true;
        }
        break;
    case ParseState::GOT_PAYLOAD:
        if ( offset == 0) {
            byte = ++_txstatus.current_rx_seq;
            update_checksum(txmsg, byte);
            _txstatus.parse_state = ParseState::GOT_SEQ;
        } else {
            if ( _txstatus.payload_next_byte < txmsg.len ) {
                byte = txmsg.payload[_txstatus.payload_next_byte++];
                update_checksum(txmsg, byte);
            } else {
                byte = (uint8_t)txmsg.checksum;
                _txstatus.parse_state = ParseState::MESSAGE_RECEIVED;
                return true;
            }
        }
        break;
    case ParseState::MESSAGE_RECEIVED:
        break;
    }
    return false;
}

bool AP_Frsky_MAVlite::mavlite_msg_get_bytes(uint8_t *bytes, const mavlite_message_t &msg, const uint8_t offset, const uint8_t count)
{
    if (offset + count > MAVLITE_MAX_PAYLOAD_LEN) {
        return false;
    }
    memcpy(bytes, &msg.payload[offset], count);
    return true;
}

bool AP_Frsky_MAVlite::mavlite_msg_set_bytes(mavlite_message_t &msg, const uint8_t *bytes, const uint8_t offset, const uint8_t count)
{
    if (offset + count > MAVLITE_MAX_PAYLOAD_LEN) {
        return false;
    }
    memcpy(&msg.payload[offset], bytes, count);
    msg.len += count;
    return true;
}


bool AP_Frsky_MAVlite::mavlite_msg_get_float(float &value, const mavlite_message_t &msg, const uint8_t offset)
{
    return mavlite_msg_get_bytes((uint8_t*)&value, msg, offset, 4);
}

bool AP_Frsky_MAVlite::mavlite_msg_get_uint16(uint16_t &value, const mavlite_message_t &msg, const uint8_t offset)
{
    return mavlite_msg_get_bytes((uint8_t*)&value, msg, offset, 2);
}

bool AP_Frsky_MAVlite::mavlite_msg_get_uint8(uint8_t &value, const mavlite_message_t &msg, const uint8_t offset)
{
    return mavlite_msg_get_bytes((uint8_t*)&value, msg, offset, 1);
}

uint8_t AP_Frsky_MAVlite::bit8_unpack(const uint8_t value, const uint8_t bit_count, const uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (uint8_t i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    return (value & mask) >> bit_offset;
}

void AP_Frsky_MAVlite::bit8_pack(uint8_t &value, const uint8_t bit_value, const uint8_t bit_count, const uint8_t bit_offset)
{
    uint8_t mask = 0;
    for (uint8_t i=bit_offset; i<=bit_count; i++) {
        mask |= 1 << i;
    }
    value |= (bit_value<<bit_offset) & mask;
}

bool AP_Frsky_MAVlite::mavlite_msg_get_string(char* value, const mavlite_message_t &msg, const uint8_t offset)
{
    if (mavlite_msg_get_bytes((uint8_t*)value, msg, offset, MIN((uint8_t)16, msg.len - offset))) {
        value[MIN((uint8_t)16, msg.len - offset)] = 0x00; // terminator
        return true;
    }
    return false;
}

bool AP_Frsky_MAVlite::mavlite_msg_set_float(mavlite_message_t &msg, const float value, const uint8_t offset)
{
    return mavlite_msg_set_bytes(msg, (uint8_t*)&value, offset, 4);
}

bool AP_Frsky_MAVlite::mavlite_msg_set_uint16(mavlite_message_t &msg, const uint16_t value, const uint8_t offset)
{
    return mavlite_msg_set_bytes(msg, (uint8_t*)&value, offset, 2);
}

bool AP_Frsky_MAVlite::mavlite_msg_set_uint8(mavlite_message_t &msg, const uint8_t value, const uint8_t offset)
{
    return mavlite_msg_set_bytes(msg, (uint8_t*)&value, offset, 1);
}

bool AP_Frsky_MAVlite::mavlite_msg_set_string(mavlite_message_t &msg, const char* value, const uint8_t offset)
{
    return mavlite_msg_set_bytes(msg, (uint8_t*)value, offset, MIN((uint8_t)16, strlen(value)));
}

