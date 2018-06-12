#pragma once

#include "AP_HAL_F4Light_Namespace.h"
#include <timer.h>


#include <AP_HAL/RCOutput.h>

#include "AP_HAL_F4Light.h"
#include "GPIO.h"
#include <AP_HAL/HAL.h>


#define PWM_IGNORE_THIS_CHANNEL 1

#define F4Light_MAX_OUTPUT_CHANNELS 12 // motors and servos


enum BOARD_PWM_MODES {
    BOARD_PWM_NORMAL=0,
    BOARD_PWM_ONESHOT,
    BOARD_PWM_ONESHOT125,
    BOARD_PWM_BRUSHED,
    BOARD_PWM_ONESHOT42,
    BOARD_PWM_PWM125,
};

#define MOTORID1 0
#define MOTORID2 1
#define MOTORID3 2
#define MOTORID4 3
#define MOTORID5 4
#define MOTORID6 5
#define MOTORID7 6
#define MOTORID8 7
#define MOTORID9 8
#define MOTORID10 9
#define MOTORID11 10
#define MOTORID12 11


class F4Light::RCOutput : public AP_HAL::RCOutput {
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    
    void cork() override{ _corked = true; }
    void push() override;
    
    static void    lateInit(); // 2nd stage with loaded parameters
    
    void set_output_mode(uint16_t mask, enum output_mode mode) override { _set_output_mode(mode); };
    
    static void _set_output_mode(enum output_mode mode);
    
    static void do_4way_if(AP_HAL::UARTDriver* uart);
/* can be overrided
    void     set_safety_pwm(uint32_t chmask, uint16_t period_us) override;
    void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;
    bool     force_safety_on(void) override;
    void     force_safety_off(void) override;
    void     force_safety_no_wait(void) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }

    void _timer_tick(void);
    bool enable_sbus_out(uint16_t rate_hz) override;
*/

    static uint32_t inline _timer_period(uint16_t speed_hz, const timer_dev *dev) {
        return (uint32_t)((float)(dev->state->freq + speed_hz/2) / speed_hz);
    }


private:
    static void InitPWM(void);
    static void set_pwm(uint8_t ch, uint16_t pwm);
    static uint16_t _period[F4Light_MAX_OUTPUT_CHANNELS];
    static uint16_t _freq[F4Light_MAX_OUTPUT_CHANNELS];
    static void _set_pin_output_mode(uint8_t ch);
    
    static bool is_servo_enabled(uint8_t ch);

    static void init_channel(uint8_t ch);
    static void init_channels();

    static uint16_t _enabled_channels;
    static bool _sbus_enabled;
    static bool _corked;
    static void _init_alt_channels() {}// we don't has channels more than 8
    static uint8_t _used_channels;
    static enum BOARD_PWM_MODES _mode;
    static bool _once_mode;
    static uint8_t _servo_mask;

    static uint8_t  _initialized[F4Light_MAX_OUTPUT_CHANNELS];
    
    //static uint32_t _timer_frequency[F4Light_MAX_OUTPUT_CHANNELS];
    
    static void _timer3_isr_event(TIM_TypeDef*);

    static uint32_t _timer2_preload;
    static uint16_t _timer3_preload;

    static uint8_t _pwm_type;
//    static float _freq_scale;

    static const timer_dev* out_timers[16]; // array of timers, used to rc_out
    static uint8_t num_out_timers;

    static void fill_timers(); 
};

