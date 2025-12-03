#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF40C_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_SF40C_TIMEOUT_MS            200   // requests timeout after 0.2 seconds
#define PROXIMITY_SF40C_PAYLOAD_LEN_MAX       256   // maximum payload size we can accept (in some configurations sensor may send as large as 1023)
#define PROXIMITY_SF40C_COMBINE_READINGS        7   // combine this many readings together to improve efficiency

class AP_Proximity_LightWareSF40C : public AP_Proximity_Backend_Serial
{

public:
    // constructor
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    uint16_t rxspace() const override {
        return 1280;
    };

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max_m() const override { return 100.0f; }
    float distance_min_m() const override { return 0.20f; }

private:

    // initialise sensor
    void initialise();

    // restart sensor and re-init our state
    void restart_sensor();

    // message ids
    enum class MessageID : uint8_t {
        PRODUCT_NAME = 0,
        HARDWARE_VERSION = 1,
        FIRMWARE_VERSION = 2,
        SERIAL_NUMBER = 3,
        TEXT_MESSAGE = 7,
        USER_DATA = 9,
        TOKEN = 10,
        SAVE_PARAMETERS = 12,
        RESET = 14,
        STAGE_FIRMWARE = 16,
        COMMIT_FIRMWARE = 17,
        INCOMING_VOLTAGE = 20,
        STREAM = 30,
        DISTANCE_OUTPUT = 48,
        LASER_FIRING = 50,
        TEMPERATURE = 55,
        BAUD_RATE = 90,
        DISTANCE = 105,
        MOTOR_STATE = 106,
        MOTOR_VOLTAGE = 107,
        OUTPUT_RATE = 108,
        FORWARD_OFFSET = 109,
        REVOLUTIONS = 110,
        ALARM_STATE = 111,
        ALARM1 = 112,
        ALARM2 = 113,
        ALARM3 = 114,
        ALARM4 = 115,
        ALARM5 = 116,
        ALARM6 = 117,
        ALARM7 = 118
    };

    // motor states
    enum class MotorState : uint8_t {
        UNKNOWN = 0,
        PREPARING_FOR_STARTUP = 1,
        WAITING_FOR_FIVE_REVS = 2,
        RUNNING_NORMALLY = 3,
        FAILED_TO_COMMUNICATE = 4
    };

    // send message to sensor
    void send_message(MessageID msgid, bool write, const uint8_t *payload, uint16_t payload_len);

    // request motor state
    void request_motor_state();

    // request start of streaming of distances
    void request_stream_start();

    // request token of sensor (required for reset)
    void request_token();
    bool got_token() const { return (_sensor_state.token[0] != 0 || _sensor_state.token[1] != 0); }
    void clear_token() { memset(_sensor_state.token, 0, ARRAY_SIZE(_sensor_state.token)); }

    // request reset of sensor
    void request_reset();

    // check and process replies from sensor
    void process_replies();

    // process one byte received on serial port
    // state is stored in msg structure.  when a full package is received process_message is called
    void parse_byte(uint8_t b);

    // process the latest message held in the msg structure
    void process_message();

    // internal variables
    uint32_t _last_request_ms;              // system time of last request
    uint32_t _last_reply_ms;                // system time of last valid reply
    uint32_t _last_restart_ms;              // system time we restarted the sensor
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    AP_Proximity_Boundary_3D::Face _face;   // face of _face_distance
    float _face_distance;                   // shortest distance (in meters) on face
    float _face_yaw_deg;                    // yaw angle (in degrees) of shortest distance on face
    bool _face_distance_valid;              // true if face has at least one valid distance

    // state of sensor
    struct {
        MotorState motor_state; // motor state (1=starting-up,2=waiting for first 5 revs, 3=normal, 4=comm failure)
        uint8_t output_rate;    // output rate number (0 = 20010, 1 = 10005, 2 = 6670, 3 = 2001)
        bool streaming;         // true if distance messages are being streamed
        uint8_t token[2];       // token (supplied by sensor) required for reset
    } _sensor_state;

    enum class ParseState {
        HEADER = 0,
        FLAGS_L,
        FLAGS_H,
        MSG_ID,
        PAYLOAD,
        CRC_L,
        CRC_H
    };

    // structure holding latest message contents
    struct {
        ParseState state;       // state of incoming message processing
        uint8_t flags_low;      // flags low byte
        uint8_t flags_high;     // flags high byte
        uint16_t payload_len;   // latest message payload length (1+ bytes in payload)
        uint8_t payload[PROXIMITY_SF40C_PAYLOAD_LEN_MAX];   // payload
        MessageID msgid;        // latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint8_t crc_low;        // crc low byte
        uint8_t crc_high;       // crc high byte
        uint16_t crc_expected;  // latest message's expected crc
    } _msg;

    // convert buffer to uint32, uint16
    uint32_t buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const;
    uint16_t buff_to_uint16(uint8_t b0, uint8_t b1) const;
};

#endif // AP_PROXIMITY_LIGHTWARE_SF40C_ENABLED
