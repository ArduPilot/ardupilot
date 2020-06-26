#pragma once

#include "AP_VisualOdom_Backend.h"

#if HAL_VISUALODOM_ENABLED

class AP_VisualOdom_NoopLoop : public AP_VisualOdom_Backend
{

public:

    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    // constructor
    AP_VisualOdom_NoopLoop(AP_VisualOdom &frontend);

    // update sensor driver
    void update() override;

    // arming check
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

protected:

    // process one byte received on serial port
    // returns true if a message has been successfully parsed
    // message is stored in _msgbuf
    bool parse_byte(uint8_t b);

    // parse msgbuf and update the EKF
    void parse_msgbuf();

    // enum of valid function marks
    enum class FunctionMark : uint8_t {
        ANCHOR_FRAME0 = 0x00,
        TAG_FRAME0 = 0x01,
        NODE_FRAME0 = 0x02,
        NODE_FRAME1 = 0x03,
        NODE_FRAME2 = 0x04,
        NODE_FRAME3 = 0x05
    };

    enum class ParseState : uint8_t {
        HEADER = 0,         // waiting for header
        FUNCTION_MARK,      // waiting for function mark
        LEN_L,              // waiting for low byte of length
        LEN_H,              // waiting for high byte of length
        PAYLOAD,            // receiving payload bytes
    } _state;

    // constants
    static const uint16_t msgbuf_len_max = 128; // maximum message length

    // members
    AP_HAL::UARTDriver *_uart = nullptr;        // pointer to uart configured for use with nooploop
    uint8_t _msgbuf[msgbuf_len_max];            // buffer to hold most recent message from tag
    uint8_t _msg_len;                           // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint16_t _frame_len;                        // message supplied frame length
    uint8_t _crc_expected;                      // calculated crc which is compared against actual received crc
};

#endif
