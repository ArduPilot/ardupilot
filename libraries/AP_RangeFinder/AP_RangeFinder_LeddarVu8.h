#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LEDDARVU8_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define LEDDARVU8_PAYLOAD_LENGTH (8*2)

class AP_RangeFinder_LeddarVu8 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_LeddarVu8(_state, _params);
    }

protected:

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    // return sensor type as laser
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    // get a reading, distance returned in reading_cm
    bool get_reading(float &reading_m) override;

    // maximum time between readings before we change state to NoData:
    uint16_t read_timeout_ms() const override { return 500; }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // function codes
    enum class FunctionCode : uint8_t {
        READ_HOLDING_REGISTER = 0x03,
        READ_INPUT_REGISTER = 0x04,
        WRITE_HOLDING_REGISTER = 0x06,
        WRITE_MULTIPLE_REGISTER = 0x10,
        READ_WRITE_MULTIPLE_REGISTER = 0x17
    };

    // register numbers for reading input registers
    enum class RegisterNumber : uint8_t {
        REGISTER_STATUS = 1,    // 0 = detections not ready, 1 = ready
        NUMBER_OF_SEGMENTS = 2,
        NUMBER_OF_DETECTIONS = 11,
        PERCENTAGE_OF_LIGHT_SOURCE_POWER = 12,
        TIMESTAMP_LOW = 14,
        TIMESTAMP_HIGH = 15,
        FIRST_DISTANCE0 = 16,   // distance of first detection for 1st segment, zero if no detection
        FIRST_AMPLITUDE0 = 24,  // amplitude of first detection * 64 for 1st segment
        FIRST_FLAG0 = 32,       // flags of first detection for 1st segment.  Bit0:Valid, Bit1:Result of object demerging, Bit2:Reserved, Bit3:Saturated
        // registers exist for distance, amplitude and flags for subsequent detections but are not used in this driver
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_ADDRESS,
        WAITING_FOR_FUNCTION_CODE,
        WAITING_FOR_PAYLOAD_LEN,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // get sensor address from RNGFNDx_ADDR parameter
    uint8_t get_sensor_address() const;

    // send request to device to provide distances
    void request_distances();

    // process one byte received on serial port
    // returns true if successfully parsed a message
    // if distances are valid, valid_readings is set to true and distance is stored in reading_cm
    bool parse_byte(uint8_t b, bool &valid_reading, uint16_t &reading_cm);

    // structure holding latest message contents
    // the order of fields matches the incoming message so it can be used to calculate the crc
    struct PACKED {
        uint8_t address;                            // device address (required for calculating crc)
        uint8_t function_code;                      // function code (always 0x04 but required for calculating crc)
        uint8_t payload_len;                        // message payload length
        uint8_t payload[LEDDARVU8_PAYLOAD_LENGTH];  // payload
        uint16_t crc;                               // latest message's crc
        uint16_t payload_recv;                      // number of message's payload bytes received so far
        ParseState state;                           // state of incoming message processing
    } parsed_msg;
    uint32_t last_distance_ms;                      // system time of last successful distance sensor read
    uint32_t last_distance_request_ms;              // system time of last request to sensor to send distances
};

#endif  // AP_RANGEFINDER_LEDDARVU8_ENABLED
