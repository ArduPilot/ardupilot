
#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD19_ENABLED
#include "AP_Proximity_Backend_Serial.h"

#define MESSAGE_LENGTH_LD19         47

// Minimum and maximum distance that the sensor can read in meters
#define MAX_READ_DISTANCE_LD19          12.0f
#define MIN_READ_DISTANCE_LD19           0.02f

class AP_Proximity_LD19 : public AP_Proximity_Backend_Serial
{
public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // Update the state of the sensor
    void update(void) override;

    // Get the max and min distances for the sensor being used
    float distance_max() const override { return MAX_READ_DISTANCE_LD19; }
    float distance_min() const override { return MIN_READ_DISTANCE_LD19; }

private:

    // Get and parse the sensor data
    void parse_response_data();
    void get_readings();

    // Store and keep track of the bytes being read from the sensor
    uint8_t _response[MESSAGE_LENGTH_LD19];
    bool _response_data;
    uint16_t _byte_count;

    // Store for error-tracking purposes
    uint32_t  _last_distance_received_ms;

    // Boundary to store the measurements
    AP_Proximity_Temp_Boundary _temp_boundary;
};
#endif // AP_PROXIMITY_LD19_ENABLED