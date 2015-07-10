#ifndef __AP_HAL_LINUX_RCOUTPUT_BEBOP_H__
#define __AP_HAL_LINUX_RCOUTPUT_BEBOP_H__

#include <AP_HAL_Linux.h>

enum bebop_bldc_motor {
    BEBOP_BLDC_RIGHT_FRONT = 0,
    BEBOP_BLDC_LEFT_FRONT,
    BEBOP_BLDC_LEFT_BACK,
    BEBOP_BLDC_RIGHT_BACK,
    BEBOP_BLDC_MOTORS_NUM,
};

enum bebop_bldc_sound {
    BEBOP_BLDC_SOUND_NONE = 0,
    BEBOP_BLDC_SOUND_SHORT_BEEP,
    BEBOP_BLDC_SOUND_BOOT_BEEP,
    BEBOP_BLDC_SOUND_BEBOP,
};

class Linux::LinuxRCOutput_Bebop : public AP_HAL::RCOutput {
    public:
    LinuxRCOutput_Bebop();
    void     init(void* dummy);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm);

private:
    AP_HAL::Semaphore *_i2c_sem;
    uint16_t _period_us[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _rpm[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _frequency;
    uint16_t _min_pwm;
    uint16_t _max_pwm;
    uint8_t  _state;

    uint8_t _checksum(uint8_t *data, unsigned int len);
    void _start_prop();
    void _set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM]);
    void _get_obs_data(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM],
                    uint16_t *batt_mv,
                    uint8_t *status,
                    uint8_t *error,
                    uint8_t *motors_err,
                    uint8_t *temp);
    void _toggle_gpio(uint8_t mask);
    void _stop_prop();
    void _clear_error();
    void _play_sound(uint8_t sound);
    uint16_t _period_us_to_rpm(uint16_t period_us);

    /* thread related members */
    pthread_t _thread;
    pthread_mutex_t _mutex;
    pthread_cond_t _cond;
    void _run_rcout();
    static void *_control_thread(void *arg);
};

#endif // __AP_HAL_LINUX_RCOUTPUT_BEBOP_H__
