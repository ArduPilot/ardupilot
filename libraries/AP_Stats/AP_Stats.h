#pragma once

// AP_Stats is used to collect and put to permanent storage data about
// the vehicle's autopilot

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Stats
{
public:

    void init();

    // copy state into underlying parameters:
    void flush();

    // periodic update function (e.g. put our values to permanent storage):
    // call at least 1Hz
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct {
        AP_Int16 bootcount;
    } params;

    uint64_t last_flush_ms; // in terms of system uptime
    const uint16_t flush_interval_ms = 30000;
};
