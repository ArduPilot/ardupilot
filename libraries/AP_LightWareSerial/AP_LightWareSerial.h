#pragma once

#include "AP_LightWareSerial_config.h"

#if AP_LIGHTWARESERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#define LIGHTWARE_PAYLOAD_LEN_MAX 256 // maximum payload size we can accept (in some configurations sensor may send as large as 1023)

class AP_LightWareSerial
{
public:
    AP_LightWareSerial(AP_HAL::UARTDriver *&uart_ref) :
        _uart(uart_ref)
    {
    }

protected:

    // initialise sensor
    void initialise();

    // send message to sensor
    void send_message(uint8_t msgid, bool write, const uint8_t *payload, uint16_t payload_len);

    // process one byte received on serial port
    // returns true if a complete message has been received
    // state is stored in _msg structure
    bool parse_byte(uint8_t b);

    enum class ParseState {
        HEADER = 0,
        FLAGS_L,
        FLAGS_H,
        MSG_ID,
        PAYLOAD,
        CRC_L,
        CRC_H
    } _parse_state; // state of incoming message processing
    uint16_t _payload_recv;     // number of message's payload bytes received so far
    uint16_t _crc_expected;     // latest message's expected crc

    // structure holding latest message contents
    struct {
        uint8_t flags_low;      // flags low byte
        uint8_t flags_high;     // flags high byte
        uint16_t payload_len;   // latest message payload length (1+ bytes in payload)
        uint8_t payload[LIGHTWARE_PAYLOAD_LEN_MAX];   // payload
        uint8_t msgid;          // latest message's message id
        uint8_t crc_low;        // crc low byte
        uint8_t crc_high;       // crc high byte
    } _msg;

private:
    AP_HAL::UARTDriver *& _uart;
};
#endif // AP_LIGHTWARESERIAL_ENABLED
