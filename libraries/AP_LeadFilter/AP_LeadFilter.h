// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef AP_LeadFilter_h
#define AP_LeadFilter_h

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
    int32_t         get_position(int32_t pos, int16_t vel);

private:
    int16_t         _last_velocity;

};

#endif
