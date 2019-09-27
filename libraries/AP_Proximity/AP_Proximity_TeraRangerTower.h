#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_TRTOWER_TIMEOUT_MS            300                               // requests timeout after 0.3 seconds

class AP_Proximity_TeraRangerTower : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_TeraRangerTower(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(int16_t angle_deg, uint16_t distance_cm);

    // reply related variables
    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t buffer[20]; // buffer where to store data from serial
    uint8_t buffer_count;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
};
