// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_LEADFILTER_H__
#define __AP_LEADFILTER_H__

#include <stdint.h>
#include <AP_LeadFilter.h>

/// @class	AP_LeadFilter
/// @brief	Object managing GPS lag
class AP_LeadFilter {
public:
    /// Constructor
    ///
    ///
    AP_LeadFilter() :
        _last_velocity(0) {
    }

    // setup min and max radio values in CLI
    int32_t         get_position(int32_t pos, int16_t vel, float lag_in_seconds = 1.0);
    void            clear() { _last_velocity = 0; }

private:
    int16_t         _last_velocity;

};

#endif // __AP_LEADFILTER_H__
