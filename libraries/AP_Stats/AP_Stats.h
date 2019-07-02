#pragma once

// AP_Stats is used to collect and put to permanent storage data about
// the vehicle's autopilot

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Stats
{
public:
    // constructor
    AP_Stats();
    
    // these variables are periodically written into the actual
    // parameters.  If you add a variable here, make sure to update
    // init() to set initial values from the parameters!
    uint32_t flttime; // seconds in flight (or driving)
    uint32_t runtime; // total wallclock time spent running ArduPilot (seconds)
    uint32_t reset;   // last time AP_Stats parameters were reset (in seconds since AP_Stats Jan 1st 2016)
    uint32_t flttime_boot; // seconds in flight (or driving), at boot
    
    void init();

    // copy state into underlying parameters:
    void flush();

    // periodic update function (e.g. put our values to permanent storage):
    // call at least 1Hz
    void update();

    void set_flying(bool b);

    // accessor for is_flying
    bool get_is_flying(void) {
        return _flying_ms != 0;
    }

    // accessor for flighttime. Returns 0 if not flying, otherwise
    // total time flying since boot in seconds
    uint32_t get_flight_time_s(void);
    
    // get singleton
    static AP_Stats *get_singleton(void) {
        return _singleton;
    }
    
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_Stats *_singleton;
    
    struct {
        AP_Int16 bootcount;
        AP_Int32 flttime;
        AP_Int32 runtime;
        AP_Int32 reset;
    } params;

    void copy_variables_from_parameters();

    uint64_t last_flush_ms; // in terms of system uptime
    const uint16_t flush_interval_ms = 30000;

    uint64_t _flying_ms;
    uint64_t _last_runtime_ms;

    void update_flighttime();
    void update_runtime();

};

namespace AP {
    AP_Stats *stats();
};
