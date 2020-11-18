#include "AP_Frsky_MAVlite_MAVliteToSPort.h"

#include "AP_Frsky_MAVlite.h"

#include "AP_Frsky_SPort.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
void AP_Frsky_MAVlite_MAVliteToSPort::reset()
{
    checksum = 0;

    packet_offs = 2;  // someone else sets frame and sensorid
    state = State::WANT_LEN;
    next_seq = 0;

    payload_count = 0;
}

void AP_Frsky_MAVlite_MAVliteToSPort::update_checksum(const uint8_t c)
{
    checksum += c; //0-1FF
    checksum += checksum >> 8;
    checksum &= 0xFF;
}

/*
 Warning:
 make sure that all packets pushed by this method are sequential and not interleaved by packets inserted by another thread!
 */
bool AP_Frsky_MAVlite_MAVliteToSPort::process(ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue, const AP_Frsky_MAVlite_Message &msg)
{
    // let's check if there's enough room to send it
    if (queue.space() < MAVLITE_MSG_SPORT_PACKETS_COUNT(msg.len)) {
        return false;
    }
    reset();

    process_byte(msg.len, queue);
    process_byte(msg.msgid, queue);

    for (uint8_t i=0; i<msg.len; i++) {
        process_byte(msg.payload[i], queue);
    }

    // byte isn't important in this call; checksum is used in
    // process_byte in case we need to start a new packet just to hold
    // it.
    process_byte(0, queue);

    return true;
}

void AP_Frsky_MAVlite_MAVliteToSPort::process_byte(const uint8_t b, ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> &queue)
{
    if (packet_offs == 2) {
        // start of a packet (since we skip setting sensorid and
        // frame).  Emit a sequence number.
        packet.raw[packet_offs++] = next_seq;
        update_checksum(next_seq);
        next_seq += 1;
    }
    switch (state) {
    case State::WANT_LEN:
        packet.raw[packet_offs++] = b;
        update_checksum(b);
        payload_len = b;
        state = State::WANT_MSGID;
        break;
    case State::WANT_MSGID:
        packet.raw[packet_offs++] = b;
        update_checksum(b);
        if (b == 0) {
            state = State::WANT_CHECKSUM;
        } else {
            state = State::WANT_PAYLOAD;
        }
        break;
    case State::WANT_PAYLOAD:
        packet.raw[packet_offs++] = b;
        update_checksum(b);
        payload_count++;
        if (payload_count >= payload_len) {
            state = State::WANT_CHECKSUM;
        }
        break;
    case State::WANT_CHECKSUM:
        packet.raw[packet_offs++] = checksum;
        queue.push(packet);
        state = State::DONE;
        break;
    case State::DONE:
        return;
    }
    if (packet_offs >= ARRAY_SIZE(packet.raw)) {
        // it's cooked
        queue.push(packet);
        packet_offs = 2;  // skip setting sensorid and frame
    }
}
#endif