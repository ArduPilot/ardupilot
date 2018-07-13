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
    void set_default_rate(uint16_t freq_hz) override;
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
    struct pwm_group {
        uint8_t chan[4];  // chan number, zero based, 255 for disabled

        // below this line is not initialised by board config
        enum output_mode current_mode;
        uint16_t ch_mask;
        bool pwm_started;
        uint32_t rc_frequency;
    };
    static pwm_group pwm_group_list[];
    static uint8_t pwm_group_number;

private:


    SITL_State *_sitlState;
    uint16_t _freq_hz;
    uint16_t _enable_mask;
    uint16_t period[SITL_NUM_CHANNELS];
    bool _corked;

    // mask of channels that are running in high speed
    uint16_t fast_channel_mask;
    uint16_t _pending[SITL_NUM_CHANNELS];
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;
    // total number of channels on FMU
    uint8_t num_fmu_channels;
    // number of active fmu channels
    uint8_t active_fmu_channels;
    // we keep the last_sent value separately, as we need to keep the unscaled
    // value for systems with brushed motors which scale outputs
    uint16_t _last_sent[SITL_NUM_CHANNELS];
    void push_local();
    void set_group_mode(pwm_group &group);
};

#endif
