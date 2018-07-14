#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL.h"

class HALSITL::RCOutput : public AP_HAL::RCOutput {
public:
    explicit RCOutput(SITL_State *sitlState): _sitlState(sitlState), _freq_hz(50) {}
    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t* period_us, uint8_t len) override;
    uint16_t read_last_sent(uint8_t ch);
    void read_last_sent(uint16_t* period_us, uint8_t len);
    void cork(void);
    void push(void);
    void set_output_mode(uint16_t mask, enum output_mode mode) override;
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }

private:
    SITL_State *_sitlState;
    uint16_t _freq_hz;
    uint16_t _enable_mask;
    bool _corked;
    uint16_t _pending[SITL_NUM_CHANNELS];
    enum output_mode _output_mode = MODE_PWM_NORMAL;
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;
    // we keep the last_sent value separately, as we need to keep the unscaled
    // value for systems with brushed motors which scale outputs
    uint16_t _last_sent[SITL_NUM_CHANNELS];
};

#endif
