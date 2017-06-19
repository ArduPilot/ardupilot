#pragma once

#include "AP_HAL_Linux.h"

#include "ToneAlarm.h"

namespace Linux {

class ToneAlarm_Raspilot : public ToneAlarm {
public:
    ToneAlarm_Raspilot();
    bool init() override;
    void stop() override;
    bool play() override;

private:
    void _set_pwm0_period(uint32_t time_us);
    void _set_pwm0_duty(uint8_t percent);

    volatile uint32_t *_pwm;
    volatile uint32_t *_clk;
};

}
