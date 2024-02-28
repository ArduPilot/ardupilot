#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_TERARANGERTOWER_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_TRTOWER_TIMEOUT_MS            300                               // requests timeout after 0.3 seconds

class AP_Proximity_TeraRangerTower : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(int16_t angle_deg, uint16_t distance_mm);

    // reply related variables
    uint8_t buffer[20]; // buffer where to store data from serial
    uint8_t buffer_count;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
};

#endif // AP_PROXIMITY_TERARANGERTOWER_ENABLED
