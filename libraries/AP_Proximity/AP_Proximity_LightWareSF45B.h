#pragma once

#include "AP_Proximity_LightWareSerial.h"

#if HAL_PROXIMITY_ENABLED
#include <Filter/Filter.h>

class AP_Proximity_LightWareSF45B : public AP_Proximity_LightWareSerial
{

public:
    // constructor
    AP_Proximity_LightWareSF45B(AP_Proximity &_frontend,
                                AP_Proximity::Proximity_State &_state,
                                AP_Proximity_Params& _params) :
            AP_Proximity_LightWareSerial(_frontend, _state, _params) {}

    uint16_t rxspace() const override {
        return 1280;
    };

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 50.0f; }
    float distance_min() const override { return 0.20f; }

private:

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
        DISTANCE_OUTPUT = 27,
        STREAM = 30,
        DISTANCE_DATA_CM = 44,
        DISTANCE_DATA_MM = 45,
        LASER_FIRING = 50,
        TEMPERATURE = 57,
        UPDATE_RATE = 66,
        NOISE = 74,
        ZERO_OFFSET = 75,
        LOST_SIGNAL_COUNTER = 76,
        BAUD_RATE = 79,
        I2C_ADDRESS = 80,
        STEPPER_STATUS = 93,
        SCAN_ON_STARTUP = 94,
        SCAN_ENABLE = 96,
        SCAN_POSITION = 97,
        SCAN_LOW_ANGLE = 98,
        SCAN_HIGH_ANGLE = 99
    };

    // initialise sensor
    void initialise();

    // request start of streaming of distances
    void request_stream_start();

    // check and process replies from sensor
    void process_replies();

    // process the latest message held in the msg structure
    void process_message();

    // convert an angle (in degrees) to a mini sector number
    uint8_t convert_angle_to_minisector(float angle_deg) const;

    // internal variables
    uint32_t _last_init_ms;                 // system time of last re-initialisation
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    bool _init_complete;                    // true once sensor initialisation is complete
    ModeFilterInt16_Size5 _distance_filt{2};// mode filter to reduce glitches

    // 3D boundary face and distance for latest readings
    AP_Proximity_Boundary_3D::Face _face;   // face of most recently received distance
    float _face_distance;                   // shortest distance (in meters) on face
    float _face_yaw_deg;                    // yaw angle (in degrees) of shortest distance on face
    bool _face_distance_valid;              // true if face has at least one valid distance

    // mini sector (5 degrees) angles and distances (used to populate obstacle database for path planning)
    uint8_t _minisector = UINT8_MAX;        // mini sector number (from 0 to 71) of most recently received distance
    float _minisector_distance;             // shortest distance (in meters) in mini sector
    float _minisector_angle;                // angle (in degrees) of shortest distance in mini sector
    bool _minisector_distance_valid;        // true if mini sector has at least one valid distance

    // state of sensor
    struct {
        uint8_t update_rate;        // sensor reported update rate enum from UPDATE_RATE message
        uint32_t streaming_fields;  // sensor reported bitmask of fields sent in DISTANCE_DATA_CM message
        uint32_t stream_data_type;  // sensor reported stream value.  5 if DISTANCE_DATA_CM messages are being streamed
    } _sensor_state;

};

#endif // HAL_PROXIMITY_ENABLED
