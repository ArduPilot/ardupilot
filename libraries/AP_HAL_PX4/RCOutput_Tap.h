#pragma once

#include "AP_HAL_PX4.h"
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>

namespace PX4 {

class RCOutput_Tap : public AP_HAL::RCOutput {
public:
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;

    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }

    void cork() override;
    void push() override;

private:
    static const uint8_t MAX_MOTORS = 4;

    void _send_outputs(void);

    uint8_t _enabled_channels = 0;
    bool _corking = false;

    uint16_t _period[MAX_MOTORS];
    uint16_t _esc_pwm_min = 0;
    uint16_t _esc_pwm_max = 0;
    perf_counter_t _perf_rcout;
};

}
