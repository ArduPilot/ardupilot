#pragma once

#include "AP_Generator.h"

#if HAL_GENERATOR_ENABLED

class AP_Generator_Backend
{
public:

    // Constructor
    AP_Generator_Backend(AP_Generator& frontend);

    // Declare a virtual destructor in case Generator drivers want to override
    virtual ~AP_Generator_Backend() {};

    virtual void init(void) = 0;
    virtual void update(void) = 0;

    // Set default to not fail arming checks
    virtual bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const { return true; }

    // Set default to not fail failsafes
    virtual AP_BattMonitor::Failsafe update_failsafes(void) const {
        return AP_BattMonitor::Failsafe::None;
    }

    virtual bool healthy(void) const = 0;

    // Generator controls must return true if present in generator type
    virtual bool stop(void) { return false; }
    virtual bool idle(void) { return false; }
    virtual bool run(void) { return false; }

    // Use generator mavlink message
    virtual void send_generator_status(const GCS_MAVLINK &channel) {}

protected:

    // Update frontend
    void update_frontend(void);

    // Measurements readings to write to front end
    float _voltage;
    float _current;
    float _fuel_remain_pct; // Decimal from 0 to 1
    float _consumed_mah;
    uint16_t _rpm;

    AP_Generator& _frontend;

};
#endif
