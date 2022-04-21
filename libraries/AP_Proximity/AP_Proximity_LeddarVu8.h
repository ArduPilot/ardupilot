#pragma once

#include "AP_Proximity.h"
#ifndef AP_PROXIMITY_LEDDARVU8_ENABLED
#define AP_PROXIMITY_LEDDARVU8_ENABLED HAL_PROXIMITY_ENABLED
#endif

#if (HAL_PROXIMITY_ENABLED && AP_PROXIMITY_LEDDARVU8_ENABLED)
#include "AP_Proximity_Backend_Serial.h"

#define LEDDARVU8_PAYLOAD_LENGTH (8*2)
#define LEDDARVU8_ADDR_DEFAULT              0x01    // modbus default device id
#define LEDDARVU8_DIST_MAX_CM               18500   // maximum possible distance reported by lidar
#define LEDDARVU8_DIST_MIN_CM               5       // maximum possible distance reported by lidar
#define LEDDARVU8_OUT_OF_RANGE_ADD_CM       100     // add this many cm to out-of-range values
#define LEDDARVU8_TIMEOUT_MS                200     // timeout in milliseconds if no distance messages received
#define LEDDARVU8_START_ANGLE 		        0.0f  // hardcoded for 48 deg FOV: Starting 2-D horizontal angle of distances received in payload
#define LEDDARVU8_ANGLE_STEP		        45.0f    // hardcoded for 48 deg FOV: Angle step size of each distance received. Starts from LEDDARVU8_START_ANGLE
class AP_Proximity_LeddarVu8 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update the state of the sensor
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return LEDDARVU8_DIST_MAX_CM * 0.01; }
    float distance_min() const override { return LEDDARVU8_DIST_MIN_CM * 0.01; }

    // get distances for the 8 channels of the leddarvu8
    bool get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const;
    
private:

    // send message to the sensor to start streaming 2-D data
    void send_sensor_start();

    // read bytes from the sensor
    void read_sensor_data();

    // parse one byte from the sensor. Return false on error.
    // bool parse_byte(uint8_t data, bool &valid_reading);

    // parse payload, to pick out distances, and feed them to the correct faces
    void parse_payload();

    // reset certain variables and flags 
    void reset();

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
    
    bool _initialized;
    uint32_t last_distance_ms;                      // system time of last successful distance sensor read
    uint32_t last_distance_request_ms;              // system time of last request to sensor to send distances

    AP_Proximity_Temp_Boundary _temp_boundary; // temporary boundary to store incoming payload

};

#endif // HAL_PROXIMITY_ENABLED && AP_PROXIMITY_LEDDARVU8_ENABLED
