#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

//some code are copied from paparazziuav

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MAMBO
#include "RCOutput_Mambo.h"
#include "Util.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>

#define PWM_NB_BITS   (9)

/* PWM can take value between 0 and 511 */
#ifndef PWM_TOTAL_RANGE
#define PWM_TOTAL_RANGE (1<<PWM_NB_BITS)
#endif

#define PWM_REG_RATIO_PRECISION_MASK (PWM_NB_BITS<<16)
#define PWM_REG_SATURATION (PWM_REG_RATIO_PRECISION_MASK|PWM_TOTAL_RANGE)

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

enum {
  SiP6_PWM0_START = (1<<0),
  SiP6_PWM1_START = (1<<1),
  SiP6_PWM2_START = (1<<2),
  SiP6_PWM3_START = (1<<3),
};

RCOutput_Mambo::RCOutput_Mambo()
{
}

void RCOutput_Mambo::init()
{
    _min_pwm = 1100;
    _max_pwm = 1900;
    _frequency = 50;
    _cork = false;
    _changed = false;

    actuators_fd = open("/dev/pwm", O_RDWR);
    pwm_delos_quadruplet m = {{ 1, 1, 1, 1 }};
    int ret = ioctl(actuators_fd, PWM_DELOS_SET_SPEEDS, &m);
    printf("Return Speeds: %d\n", ret);

    unsigned int control_reg = (SiP6_PWM0_START|SiP6_PWM1_START|SiP6_PWM2_START|SiP6_PWM3_START);
    ret = ioctl(actuators_fd, PWM_DELOS_SET_CTRL, &control_reg);
    printf("Return control: %d\n", ret);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RCOutput_Mambo::_update, void));
}

void RCOutput_Mambo::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t RCOutput_Mambo::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_Mambo::enable_ch(uint8_t ch)
{
}

void RCOutput_Mambo::disable_ch(uint8_t ch)
{
}

void RCOutput_Mambo::write(uint8_t ch, uint16_t period_us)
{
    uint16_t rpm_ref;
    unsigned int val;
    if (ch < 4) {
        _period_us[ch] = period_us;
        rpm_ref = _period_us_to_rpm(period_us);

        val = rpm_ref & 0xffff;
        if( rpm_ref > PWM_TOTAL_RANGE ) { val = PWM_REG_SATURATION; }
        /* The upper 16-bit word of the ratio register contains the number of bits used to code the ratio command  */
        val |= PWM_REG_RATIO_PRECISION_MASK;
        _pwm.val[ch] = val;
    }
    if (!_cork) push();
}

void RCOutput_Mambo::cork()
{
    _cork = true;
}

void RCOutput_Mambo::push()
{
    if (!_cork) {
        return;
    }
    _cork = false;
  
    _changed = true;
}

void RCOutput_Mambo::_update() {
    if (_changed) {
        _changed = false;
        ioctl(actuators_fd, PWM_DELOS_SET_RATIOS, &_pwm);
    }
}

uint16_t RCOutput_Mambo::read(uint8_t ch)
{
    if (ch < 4) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_Mambo::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

void RCOutput_Mambo::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
}

uint16_t RCOutput_Mambo::_period_us_to_rpm(uint16_t period_us)
{
    period_us = constrain_int16(period_us, _min_pwm, _max_pwm);
    float period_us_fl = period_us;
    float rpm_fl = (period_us_fl - _min_pwm)/(_max_pwm - _min_pwm) * 511;

    return (uint16_t)rpm_fl;
}

#endif
