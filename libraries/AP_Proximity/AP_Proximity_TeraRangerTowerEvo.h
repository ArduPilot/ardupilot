#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_TRTOWER_TIMEOUT_MS            300                               // requests timeout after 0.3 seconds

class AP_Proximity_TeraRangerTowerEvo : public AP_Proximity_Backend_Serial {

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    // check and process replies from sensor
    void initialise_modes();
    bool read_sensor_data();
    void update_sector_data(int16_t angle_deg, uint16_t distance_cm);
    void set_mode(const uint8_t *c, int length);

    enum InitState {
        InitState_Printout = 0,
        InitState_Sequence,
        InitState_Rate,
        InitState_StreamStart,
        InitState_Finished
    };
    
    // reply related variables
    uint8_t buffer[21]; // buffer where to store data from serial
    uint8_t buffer_count;

    // request related variables
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    uint32_t _last_request_sent_ms;         // system time of last command set
    const uint16_t _mode_request_delay = 1000;
    enum InitState _current_init_state = InitState_Printout;

    // tower evo operating modes

    const uint8_t BINARY_MODE[4] = {(uint8_t)0x00, (uint8_t)0x11, (uint8_t)0x02, (uint8_t)0x4C};
    const uint8_t TOWER_MODE[4] = {(uint8_t)0x00, (uint8_t)0x31, (uint8_t)0x03, (uint8_t)0xE5};
    const uint8_t SEQUENCE_MODE[4] = {(uint8_t)0x00, (uint8_t)0x31, (uint8_t)0x02, (uint8_t)0xE2};
    const uint8_t ACTIVATE_STREAM[5] = {(uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x02, (uint8_t)0x01, (uint8_t)0xDF};
    const uint8_t REFRESH_50_HZ[5] = { (uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x03, (uint8_t)0x02, (uint8_t)0xC3};
    const uint8_t REFRESH_100_HZ[5] = { (uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x03, (uint8_t)0x03, (uint8_t)0xC4};
    const uint8_t REFRESH_250_HZ[5] = { (uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x03, (uint8_t)0x04, (uint8_t)0xD1};
    const uint8_t REFRESH_500_HZ[5] = { (uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x03, (uint8_t)0x05, (uint8_t)0xD6};
    const uint8_t REFRESH_600_HZ[5] = { (uint8_t)0x00, (uint8_t)0x52, (uint8_t)0x03, (uint8_t)0x06, (uint8_t)0xDF};
};
