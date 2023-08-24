#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/I2CDevice.h>

struct bldc_info;

namespace Linux {

enum bebop_bldc_motor {
    BEBOP_BLDC_MOTOR_1 = 0,
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_DISCO
    BEBOP_BLDC_MOTOR_2,
    BEBOP_BLDC_MOTOR_3,
    BEBOP_BLDC_MOTOR_4,
#endif
    BEBOP_BLDC_MOTORS_NUM,
};

enum bebop_bldc_sound {
    BEBOP_BLDC_SOUND_NONE = 0,
    BEBOP_BLDC_SOUND_SHORT_BEEP,
    BEBOP_BLDC_SOUND_BOOT_BEEP,
    BEBOP_BLDC_SOUND_BEBOP,
};

/* description of the bldc status */
#define BEBOP_BLDC_STATUS_INIT          0
#define BEBOP_BLDC_STATUS_IDLE          1
#define BEBOP_BLDC_STATUS_RAMPING       2
#define BEBOP_BLDC_STATUS_SPINNING_1    3
#define BEBOP_BLDC_STATUS_SPINNING_2    4
#define BEBOP_BLDC_STATUS_STOPPING      5
#define BEBOP_BLDC_STATUS_CRITICAL      6

/* description of the bldc errno */
#define BEBOP_BLDC_ERRNO_EEPROM         1
#define BEBOP_BLDC_ERRNO_MOTOR_STALLED  2
#define BEBOP_BLDC_ERRNO_PROP_SECU      3
#define BEBOP_BLDC_ERRNO_COM_LOST       4
#define BEBOP_BLDC_ERRNO_BATT_LEVEL     9
#define BEBOP_BLDC_ERRNO_LIPO           10
#define BEBOP_BLDC_ERRNO_MOTOR_HW       11

class BebopBLDC_ObsData {
public:
    uint16_t rpm[BEBOP_BLDC_MOTORS_NUM];
    uint8_t rpm_saturated[BEBOP_BLDC_MOTORS_NUM];
    uint16_t batt_mv;
    uint8_t status;
    uint8_t error;
    uint8_t motors_err;
    uint8_t temperature;
};

class RCOutput_Bebop : public AP_HAL::RCOutput {
public:
    RCOutput_Bebop(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    static RCOutput_Bebop *from(AP_HAL::RCOutput *rcout) {
        return static_cast<RCOutput_Bebop*>(rcout);
    }

    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    int      read_obs_data(BebopBLDC_ObsData &data);
    void     play_note(uint8_t pwm, uint16_t period_us, uint16_t duration_ms);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _request_period_us[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _period_us[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _rpm[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _frequency;
    uint16_t _min_pwm;
    uint16_t _max_pwm;
    uint8_t _n_motors=4;
    uint8_t  _state;
    bool     _corking = false;
    uint16_t _max_rpm;

    uint8_t _checksum(uint8_t *data, unsigned int len);
    void _start_prop();
    void _toggle_gpio(uint8_t mask);
    void _set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM]);
    bool _get_info(struct bldc_info *info);
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

}
