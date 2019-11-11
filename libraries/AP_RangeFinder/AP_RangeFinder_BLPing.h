#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_BLPing : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    void init_sensor();

    // send message to sensor
    void send_message(uint16_t msgid, const uint8_t *payload, uint16_t payload_len);

    // read a distance from the sensor
    bool get_reading(uint16_t &reading_cm) override;

    uint16_t read_timeout_ms() const override { return 500; }

    // process one byte received on serial port
    // returns true if a distance message has been successfully parsed
    // state is stored in msg structure
    bool parse_byte(uint8_t b);

    enum class ParseState {
        HEADER1 = 0,
        HEADER2,
        LEN_L,
        LEN_H,
        MSG_ID_L,
        MSG_ID_H,
        SRC_ID,
        DST_ID,
        PAYLOAD,
        CRC_L,
        CRC_H
    };

    uint32_t last_init_ms;      // system time that sensor was last initialised
    uint16_t distance_cm;       // latest distance

    // structure holding latest message contents
    struct {
        ParseState state;       // state of incoming message processing
        uint8_t payload[20];    // payload
        uint16_t payload_len;   // latest message payload length
        uint16_t msgid;         // latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint16_t crc;           // latest message's crc
        uint16_t crc_expected;  // latest message's expected crc
    } msg;
};
