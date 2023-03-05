#pragma once

#include "AP_Radar.h"
#include <AP_HAL/utility/OwnPtr.h>

#if HAL_MSP_RADAR_ENABLED

class AP_Radar_MSP : public Radar_backend
{
public:
    /// constructor
    using Radar_backend::Radar_backend;

    // initialise the sensor
    void init() override {}

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // get update from msp
    void handle_msp(const MSP::msp_radar_pos_message_t &pkt) override;

    // detect if the sensor is available
    static AP_Radar_MSP *detect(AP_Radar &_frontend);

private:
    radar_peer_t peers[RADAR_MAX_PEERS]; // temporary storage of our last peer states
    uint8_t update_count;
};

#endif // HAL_MSP_RADAR_ENABLED
