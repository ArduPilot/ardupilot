#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @brief Simple class to deal with Ping Protocol
 *
 *  Protocol documentation can be found here: https:* github.com/bluerobotics/ping-protocol
 *
 *  Byte     Type        Name            Description
 *  --------------------------------------------------------------------------------------------------------------
 *  0        uint8_t     start1          'B'
 *  1        uint8_t     start2          'R'
 *  2-3      uint16_t    payload_length  number of bytes in payload (low byte, high byte)
 *  4-5      uint16_t    message id      message id (low byte, high byte)
 *  6        uint8_t     src_device_id   id of device sending the message
 *  7        uint8_t     dst_device_id   id of device of the intended recipient
 *  8-n      uint8_t[]   payload         message payload
 *  (n+1)-(n+2)  uint16_t    checksum    the sum of all the non-checksum bytes in the message (low byte, high byte)
 */
class PingProtocol {
    static constexpr uint8_t _frame_header1 = 0x42; // // header first byte ('B')
    static constexpr uint8_t _frame_header2 = 0x52; // // header first byte ('R')
    static constexpr uint16_t _src_id = 0; // vehicle's source id
    static constexpr uint16_t _dst_id = 1; // sensor's id

public:
    enum class MessageId {
        INVALID = 0,
        SET_PING_INTERVAL = 1004,
        DISTANCE_SIMPLE = 1211,
        CONTINUOUS_START = 1400,
    };

    /**
     * @brief Process a single byte received on serial port
     *  return a valid MessageId if there is a message in buffer.
     *
     * @param byte
     * @return MessageId
     */
    MessageId parse_byte(uint8_t b);

    /**
     * @brief Send a message with a defined payload
     *
     * @param uart
     * @param msgid
     * @param payload
     * @param payload_len
     */
    void send_message(AP_HAL::UARTDriver *uart, PingProtocol::MessageId msg_id, const uint8_t *payload, uint16_t payload_len) const;

    /**
     * @brief Get distance from message
     *
     * @return uint32_t
     */
    uint32_t get_distance_mm() const;

    /**
     * @brief Get confidence from message
     *
     * @return uint8_t
     */
    uint8_t get_confidence() const;

    /**
     * @brief Get the message id available in bufffer
     *
     * @return MessageId
     */
    MessageId get_message_id() const { return static_cast<MessageId>(msg.id); };

protected:
    /**
     * @brief State for the parser logic
     *
     */
    enum class ParserState {
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

    /**
     * @brief Structure holding the last message available with its state
     *
     */
    struct {
        ParserState state;      // state of incoming message processing
        bool done;              // inform if the message is complete or not
        uint8_t payload[20];    // payload
        uint16_t payload_len;   // latest message payload length
        uint16_t id;            // latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint16_t crc;           // latest message's crc
        uint16_t crc_expected;  // latest message's expected crc
    } msg;
};

/**
 * @brief Class for Blue Robotics Ping1D sensor
 *
 */
class AP_RangeFinder_BLPing : public AP_RangeFinder_Backend_Serial
{
    static constexpr uint16_t _sensor_rate_ms = 50; // initialise sensor at no more than 20hz

public:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Update class state
     *
     */
    void update(void) override;

protected:
    /**
     * @brief Return the sensor type
     *
     * @return MAV_DISTANCE_SENSOR
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    /**
     * @brief Sensor protocol class
     *
     */
    PingProtocol protocol;

private:
    /**
     * @brief Do the necessary sensor initiation
     *
     */
    void init_sensor();

    /**
     * @brief Read serial interface and calculate new distance
     *
     * @param reading_cm
     * @return true
     * @return false
     */
    bool get_reading(uint16_t &reading_cm) override;

    /**
     * @brief Timeout between messages
     *
     * @return uint16_t
     */
    uint16_t read_timeout_ms() const override { return 1000; }

    /**
     * @brief system time that sensor was last initialised
     *
     */
    uint32_t last_init_ms;
};
