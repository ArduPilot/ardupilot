// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ACTUATOR_PWM_H__
#define __AP_ACTUATOR_PWM_H__

#include "AP_Actuator_Channel.h"

class AP_Actuator_PWM : public AP_Actuator_Channel {
public:
    AP_Actuator_PWM(uint8_t _ch) : ch(_ch) {}

    /* Output active/highZ control */
    void enable_ch();
    void disable_ch();

    /* Set the value the actuator is driven with */
    void write(int16_t value);

    /* Set output frequency (1/period) */
    void set_freq(uint16_t freq_hz);

protected:
    uint8_t ch;
};

#endif
