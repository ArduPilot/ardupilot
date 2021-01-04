#include "AP_Frsky_MAVlite_SPortToMAVlite.h"

#include "AP_Frsky_MAVlite.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
void AP_Frsky_MAVlite_SPortToMAVlite::reset(void)
{
    checksum = 0;

    expected_seq = 0;
    payload_next_byte = 0;
    parse_state = State::WANT_LEN;
}

void AP_Frsky_MAVlite_SPortToMAVlite::update_checksum(const uint8_t c)
{
    checksum += c; //0-1FF
    checksum += checksum >> 8;
    checksum &= 0xFF;
}

/*
 Parses sport packets and if successfull fills the rxmsg mavlite struct
 */
bool AP_Frsky_MAVlite_SPortToMAVlite::process(AP_Frsky_MAVlite_Message &rxmsg, const AP_Frsky_SPort::sport_packet_t &packet)
{
    // the two skipped bytes in packet.raw here are sensor and frame.
    // appid and data are used to transport the mavlite message.

    // deal with packet sequence number:
    const uint8_t received_seq = (packet.raw[2] & 0x3F);
    // if the first byte of any sport passthrough packet is zero then we reset:
    if (received_seq == 0) {
        reset();
    }
    if (received_seq != expected_seq) {
        parse_state = State::ERROR;
        return false;
    }
    update_checksum(received_seq);
    expected_seq = received_seq + 1;

    // deal with the remainder (post-sequence) of the packet:
    for (uint8_t i=3; i<ARRAY_SIZE(packet.raw); i++) {
        parse(packet.raw[i]);
    }
    if (parse_state == State::MESSAGE_RECEIVED) {
        rxmsg = _rxmsg;
        return true;
    }
    return false;
}

void AP_Frsky_MAVlite_SPortToMAVlite::parse(uint8_t byte)
{
    switch (parse_state) {

    case State::IDLE:
        // it is an error to receive anything but offset==0 byte=0xx0
        // in this state
        parse_state = State::ERROR;
        return;

    case State::ERROR:
        // waiting for offset==0 && byte==0x00 to bump us into WANT_LEN
        return;

    case State::WANT_LEN:
        _rxmsg.len = byte;
        update_checksum(byte);
        parse_state = State::WANT_MSGID;
        return;

    case State::WANT_MSGID:
        _rxmsg.msgid = byte;
        update_checksum(byte);
        if (_rxmsg.len == 0) {
            parse_state = State::WANT_CHECKSUM;
        } else {
            parse_state = State::WANT_PAYLOAD;
        }
        return;

    case State::WANT_PAYLOAD:
        // add byte to payload
        _rxmsg.payload[payload_next_byte++] = byte;
        update_checksum(byte);

        if (payload_next_byte >= _rxmsg.len) {
            parse_state = State::WANT_CHECKSUM;
        }
        return;

    case State::WANT_CHECKSUM:
        if (checksum != byte) {
            parse_state = State::ERROR;
            return;
        }
        parse_state = State::MESSAGE_RECEIVED;
        return;

    case State::MESSAGE_RECEIVED:
        return;
    }
}
#endif