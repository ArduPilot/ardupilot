#pragma once

// AP_Stats is used to collect and put to permanent storage data about
// the vehicle's autopilot

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Stats
{
public:

    // these variables are periodically written into the actual
    // parameters.  If you add a variable here, make sure to update
    // init() to set initial values from the parameters!
    uint32_t flttime; // seconds in flight (or driving)

    void init();

    // copy state into underlying parameters:
    void flush();

    // periodic update function (e.g. put our values to permanent storage):
    // call at least 1Hz
    void update();

    void set_flying(bool b);

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct {
        AP_Int16 bootcount;
        AP_Int32 flttime;
    } params;

    uint64_t last_flush_ms; // in terms of system uptime
    const uint16_t flush_interval_ms = 30000;

    uint64_t _flying_ms;

    void update_flighttime();

};
