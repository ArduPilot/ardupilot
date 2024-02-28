#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_NOOPLOOP_ENABLED

#define NOOPLOOP_MSG_BUF_MAX      256

class AP_Beacon_Nooploop : public AP_Beacon_Backend
{

public:
    // constructor
    using AP_Beacon_Backend::AP_Beacon_Backend;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update the state of the sensor
    void update() override;

private:
    enum class MsgType : uint8_t {
        INVALID = 0,
        NODE_FRAME2,
        SETTING_FRAME0
    };

    // process one byte received on serial port
    // message is stored in _msgbuf
    MsgType parse_byte(uint8_t b);

    // send setting_frame0 to tag. tag will ack setting_frame0 with anchor position filled
    void request_setting();

    // parse node_frame2 to get tag position and distance
    void parse_node_frame2();

    // parse setting_frame0 to get anchor position
    void parse_setting_frame0();

    enum class ParseState : uint8_t {
        HEADER = 0,         // waiting for header
        H55_FUNCTION_MARK,  // waiting for function mark
        H54_FUNCTION_MARK,  // waiting for function mark
        LEN_L,              // waiting for low byte of length
        LEN_H,              // waiting for high byte of length
        NF2_PAYLOAD,        // receiving payload bytes
        SF0_PAYLOAD,        // receiving payload bytes
    } _state = ParseState::HEADER;

    // members
    uint8_t _msgbuf[NOOPLOOP_MSG_BUF_MAX];      // buffer to hold most recent message from tag
    uint16_t _msg_len;                          // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint16_t _frame_len;                        // message supplied frame length
    uint8_t _crc_expected;                      // calculated crc which is compared against actual received crc
    uint32_t _last_update_ms;                   // last time we receive data from tag
    bool _anchor_pos_avail;                     // flag indicates if we got anchor position or not
    uint32_t _last_request_setting_ms;          // last time we sent request_setting0 packet to tag
};

#endif  // AP_BEACON_NOOPLOOP_ENABLED
