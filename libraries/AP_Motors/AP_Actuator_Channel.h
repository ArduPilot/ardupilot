// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ACTUATOR_CHANNEL_H__
#define __AP_ACTUATOR_CHANNEL_H__

#include <AP_Common.h>

class AP_Actuator_Channel {
public:
    /* Output active/highZ control */
    virtual void enable_ch() = 0;
    virtual void disable_ch() = 0;

    /* Set the value the actuator is driven with */
    virtual void write(int16_t value) = 0;

    /* Set output frequency (1/period) if using PWM or similar */
    virtual void set_freq(uint16_t freq_hz) = 0;

    /* Does the channel support reverse thrust (ESCs only) */
    virtual bool capability_reverse() const { return false; }
};

#endif
