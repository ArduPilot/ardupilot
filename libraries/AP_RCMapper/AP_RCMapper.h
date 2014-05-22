/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_RCMAPPER_H
#define AP_RCMAPPER_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>

class RCMapper
{
public:
    /// Constructor
    ///
    RCMapper();

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch_roll; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch_pitch; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch_throttle; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch_yaw; }
    
    /// flightmode
    uint8_t flightmode() const { return _ch_flightmode; }
    
    /// tune
    uint8_t tune() const { return _ch_tune; }
    
    /// aux1
    uint8_t aux1() const { return _ch_aux1; }
    
    /// aux2
    uint8_t aux2() const { return _ch_aux2; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // channel mappings
    AP_Int8 _ch_roll;
    AP_Int8 _ch_pitch;
    AP_Int8 _ch_yaw;
    AP_Int8 _ch_throttle;
    AP_Int8 _ch_flightmode;
    AP_Int8 _ch_tune;
    AP_Int8 _ch_aux1;
    AP_Int8 _ch_aux2;
};
#endif
