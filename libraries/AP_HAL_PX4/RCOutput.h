#pragma once

#include "AP_HAL_PX4.h"
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

#define PX4_NUM_OUTPUT_CHANNELS 16

class PX4::PX4RCOutput : public AP_HAL::RCOutput 
{
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    uint16_t read_last_sent(uint8_t ch) override;
    void     read_last_sent(uint16_t* period_us, uint8_t len) override;
    void     set_safety_pwm(uint32_t chmask, uint16_t period_us) override;
    void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;
    bool     force_safety_on(void) override;
    void     force_safety_off(void) override;
    void     force_safety_no_wait(void) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }
    bool     get_esc_scaling(uint16_t &min_pwm, uint16_t &max_pwm) override {
        min_pwm = _esc_pwm_min;
        max_pwm = _esc_pwm_max;
        return true;
    }
    float    scale_esc_to_unity(uint16_t pwm) override {
        return 2.0 * ((float) pwm - _esc_pwm_min) / (_esc_pwm_max - _esc_pwm_min) - 1.0;
    }
    void cork();
    void push();

    void set_output_mode(uint16_t mask, enum output_mode mode) override;

    void timer_tick(void) override;
    bool enable_px4io_sbus_out(uint16_t rate_hz) override;

    // set default output update rate
    void set_default_rate(uint16_t rate_hz) override;

private:
    int _pwm_fd;
    int _alt_fd;
    uint16_t _freq_hz;
    uint16_t _period[PX4_NUM_OUTPUT_CHANNELS];
    // we keep the last_sent value separately, as we need to keep the unscaled
    // value for systems with brushed motors which scale outputs
    uint16_t _last_sent[PX4_NUM_OUTPUT_CHANNELS];
    volatile uint8_t _max_channel;
    volatile bool _need_update;
    bool _sbus_enabled:1;
    perf_counter_t  _perf_rcout;
    uint32_t _last_output;
    uint32_t _last_config_us;
    unsigned _servo_count;
    unsigned _alt_servo_count;
    uint32_t _rate_mask_main;
    uint32_t _rate_mask_alt;
    uint16_t _enabled_channels;
    uint32_t _period_max;
    uint32_t _last_safety_options_check_ms;
    uint16_t _last_safety_options = 0xFFFF;
    struct {
        int pwm_sub;
        actuator_outputs_s outputs;
    } _outputs[ORB_MULTI_MAX_INSTANCES] {};
    actuator_armed_s _armed;

    orb_advert_t _actuator_armed_pub;
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;

    void _init_alt_channels(void);
    void _arm_actuators(bool arm);
    void set_freq_fd(int fd, uint32_t chmask, uint16_t freq_hz, uint32_t &rate_mask);
    bool _corking;
    enum output_mode _output_mode = MODE_PWM_NORMAL;
    void _send_outputs(void);
    enum AP_HAL::Util::safety_state _safety_state_request = AP_HAL::Util::SAFETY_NONE;
    uint32_t _safety_state_request_last_ms = 0;
    void force_safety_pending_requests(void);
    uint16_t _default_rate_hz = 50;
};
