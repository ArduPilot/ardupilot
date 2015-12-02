#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux.h"
#include "Util.h"

class Linux::PWM_Sysfs {
public:
    PWM_Sysfs(uint8_t chip, uint8_t channel);
    ~PWM_Sysfs();

    enum Polarity {
        NORMAL = 0,
        INVERSE = 1,
    };

    void enable(bool value);
    bool is_enabled();
    void set_period(uint32_t nsec_period);
    uint32_t get_period();
    void set_freq(uint32_t freq);
    uint32_t get_freq();

    /*
     * This is the main method, to be called on hot path. It doesn't log any
     * failure so not to risk flooding the log. If logging is necessary, check
     * the return value.
     */
    bool set_duty_cycle(uint32_t nsec_duty_cycle);

    /*
     * This is supposed to be called in the hot path, so it returns the cached
     * value rather than getting it from hardware.
     */
    uint32_t get_duty_cycle();

    void set_polarity(PWM_Sysfs::Polarity polarity);
    PWM_Sysfs::Polarity get_polarity();

private:
    uint32_t _nsec_duty_cycle_value;
    int _duty_cycle_fd;
    const uint16_t _channel;
    const uint8_t _chip;
};
