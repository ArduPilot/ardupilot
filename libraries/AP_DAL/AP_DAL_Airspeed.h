#pragma once

#include <AP_Logger/LogStructure.h>

#include <AP_Airspeed/AP_Airspeed.h>

class AP_DAL_Airspeed {
public:

    // Airspeed-like methods:

    // return health status of sensor
    bool healthy(uint8_t i) const {
        return _RASI[i].healthy;
    }
    // return health status of primary sensor
    bool healthy() const {
        return healthy(get_primary());
    }

    // return true if airspeed is enabled, and airspeed use is set
    bool use(uint8_t i) const {
        return _RASI[i].use;
    }
    bool use() const {
        return use(get_primary());
    }

    // return the current airspeed in m/s
    float get_airspeed(uint8_t i) const {
        return _RASI[i].airspeed;
    }
    float get_airspeed() const {
        return get_airspeed(get_primary());
    }

    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return _RASI[i].last_update_ms; }
    uint32_t last_update_ms() const { return last_update_ms(get_primary()); }

    // get number of sensors
    uint8_t get_num_sensors(void) const { return _RASH.num_sensors; }

    // get current primary sensor
    uint8_t get_primary(void) const { return _RASH.primary; }

    // AP_DAL methods:
    AP_DAL_Airspeed();

    void start_frame();

    void handle_message(const log_RASH &msg) {
        _RASH = msg;
    }
    void handle_message(const log_RASI &msg) {
        _RASI[msg.instance] = msg;
    }

private:

    struct log_RASH _RASH;
    struct log_RASI _RASI[AIRSPEED_MAX_SENSORS];
};
