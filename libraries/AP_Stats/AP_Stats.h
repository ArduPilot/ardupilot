#pragma once

// AP_Stats is used to collect and put to permanent storage data about
// the vehicle's autopilot

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_Stats
{
public:

    void init();

    static const struct AP_Param::GroupInfo var_info[];

private:

    struct {
        AP_Int16 bootcount;
    } params;

};
