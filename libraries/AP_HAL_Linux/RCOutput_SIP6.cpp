/*
  Driver for PWM outputs on Parrot SIP6 boards (Mambo, Rolling Spider), based
  on a similar driver from Paparazzi UAS.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ROLLING_SPIDER
#include "RCOutput_SIP6.h"
#include "Util.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* build ioctl unique identifiers for R/W operations */
#define IOCTL_PWM_MAGIC 'p'
#define IOCTL_PWM_DELOS_SET_RATIOS _IOR(IOCTL_PWM_MAGIC, 9,  pwm_delos_quadruplet*)
#define IOCTL_PWM_DELOS_SET_SPEEDS _IOR(IOCTL_PWM_MAGIC, 10, pwm_delos_quadruplet*)
#define IOCTL_PWM_DELOS_SET_CTRL   _IOR(IOCTL_PWM_MAGIC, 11, unsigned int)
#define IOCTL_PWM_DELOS_REQUEST    _IO(IOCTL_PWM_MAGIC, 12)

#define PWM_NB_BITS   (9)

/* PWM can take value between 0 and 511 */
#ifndef PWM_TOTAL_RANGE
#define PWM_TOTAL_RANGE (1<<PWM_NB_BITS)
#endif

#define PWM_REG_RATIO_PRECISION_MASK (PWM_NB_BITS<<16)
#define PWM_REG_SATURATION (PWM_REG_RATIO_PRECISION_MASK|PWM_TOTAL_RANGE)

#define SIP6_MOTOR_EN_PIN 39

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

enum {
    SIP6_PWM0_START = (1<<0),
    SIP6_PWM1_START = (1<<1),
    SIP6_PWM2_START = (1<<2),
    SIP6_PWM3_START = (1<<3),
};

RCOutput_SIP6::RCOutput_SIP6()
{
}

void RCOutput_SIP6::init()
{
    _min_pwm = 1100;
    _max_pwm = 1900;
    _frequency = 50;
    _cork = false;

    actuators_fd = open("/dev/pwm", O_RDWR);
    if (actuators_fd == -1) {
        AP_HAL::panic("Failed to open /dev/pwm: %s", strerror(errno));
    }

    pwm_delos_quadruplet m = {{ 1, 1, 1, 1 }};
    int ret = ioctl(actuators_fd, IOCTL_PWM_DELOS_SET_SPEEDS, &m);
    if (ret != 0) {
        AP_HAL::panic("ioctl PWM_DELOS_SET_SPEEDS failed: %s", strerror(errno));
    }

    unsigned int control_reg = (SIP6_PWM0_START|SIP6_PWM1_START|SIP6_PWM2_START|SIP6_PWM3_START);
    ret = ioctl(actuators_fd, IOCTL_PWM_DELOS_SET_CTRL, &control_reg);
    if (ret != 0) {
        AP_HAL::panic("ioctl PWM_DELOS_SET_CTRL failed: %s", strerror(errno));
    }

    // Enable motor controllers
    hal.gpio->pinMode(SIP6_MOTOR_EN_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(SIP6_MOTOR_EN_PIN, 0);
}

void RCOutput_SIP6::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t RCOutput_SIP6::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_SIP6::enable_ch(uint8_t ch)
{
}

void RCOutput_SIP6::disable_ch(uint8_t ch)
{
}

void RCOutput_SIP6::write(uint8_t ch, uint16_t period_us)
{
    uint16_t rpm_ref;
    unsigned int val;
    if (ch < 4) {
        _period_us[ch] = period_us;
        rpm_ref = _period_us_to_rpm(period_us);

        val = rpm_ref & 0xffff;
        if( rpm_ref > PWM_TOTAL_RANGE ) {
            val = PWM_REG_SATURATION;
        }
        /*
          The upper 16-bit word of the ratio register contains the number of
          bits used to code the ratio command
         */
        val |= PWM_REG_RATIO_PRECISION_MASK;
        _pwm.val[ch] = val;
    }
    if (!_cork) push();
}

void RCOutput_SIP6::cork()
{
    _cork = true;
}

void RCOutput_SIP6::push()
{
    if (!_cork) {
        return;
    }
    _cork = false;

    ioctl(actuators_fd, IOCTL_PWM_DELOS_SET_RATIOS, &_pwm);
}

uint16_t RCOutput_SIP6::read(uint8_t ch)
{
    if (ch < 4) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_SIP6::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

void RCOutput_SIP6::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
}

uint16_t RCOutput_SIP6::_period_us_to_rpm(uint16_t period_us)
{
    period_us = constrain_int16(period_us, _min_pwm, _max_pwm);
    float period_us_fl = period_us;
    float rpm_fl = (period_us_fl - _min_pwm) / (_max_pwm - _min_pwm) * 511;

    return (uint16_t) rpm_fl;
}

#endif
