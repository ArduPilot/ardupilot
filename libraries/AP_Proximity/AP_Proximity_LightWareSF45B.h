#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF45B_SERIAL_ENABLED || AP_PROXIMITY_LIGHTWARE_SF45B_I2C_ENABLED
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <Filter/Filter.h>

static const uint32_t PROXIMITY_SF45B_TIMEOUT_MS = 200;
static const uint32_t PROXIMITY_SF45B_REINIT_INTERVAL_MS = 5000;    // re-initialise sensor after this many milliseconds
static const float PROXIMITY_SF45B_COMBINE_READINGS_DEG = 5.0f;     // combine readings from within this many degrees to improve efficiency
static const uint32_t PROXIMITY_SF45B_DESIRED_FIELDS = ((uint32_t)1 << 0 | (uint32_t)1 << 8);   // first return (unfiltered), yaw angle
static const uint16_t PROXIMITY_SF45B_DESIRED_FIELD_COUNT = 2;      // DISTANCE_DATA_CM message should contain two fields

class AP_Proximity_LightWareSF45B
{
public:
    // constructor
    AP_Proximity_LightWareSF45B() {}

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
        SCAN_SPEED = 85,
        STEPPER_STATUS = 93,
        SCAN_ON_STARTUP = 94,
        SCAN_ENABLE = 96,
        SCAN_POSITION = 97,
        SCAN_LOW_ANGLE = 98,
        SCAN_HIGH_ANGLE = 99
    };

    // convert an angle (in degrees) to a mini sector number
    uint8_t convert_angle_to_minisector(float angle_deg) const { return wrap_360(angle_deg + (PROXIMITY_SF45B_COMBINE_READINGS_DEG * 0.5f)) / PROXIMITY_SF45B_COMBINE_READINGS_DEG; };

    // internal variables
    uint32_t _last_init_ms;                 // system time of last re-initialisation
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    bool _init_complete;                    // true once sensor initialisation is complete
    ModeFilterInt16_Size3 _distance_filt{1};// mode filter to reduce glitches

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
};

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_SERIAL_ENABLED
