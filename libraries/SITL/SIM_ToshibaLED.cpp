#include "SIM_ToshibaLED.h"

#include <stdio.h>

#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

int SITL::ToshibaLED::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        // this is really just because it is unexpected from the
        // ArduPilot code, rather than being incorrect from a
        // simulated device perspective.
        AP_HAL::panic("Reading from Toshiba LED?!");
    }
    const struct I2C::i2c_msg &msg = data->msgs[0];
    const uint8_t reg = msg.buf[0];
    const uint8_t val = msg.buf[1];
    switch(reg) {
    case TOSHIBA_LED_PWM0:
        // ::fprintf(stderr, "ToshibaLED: pwm0=%u %u %u\n", msg.buf[1], msg.buf[2], msg.buf[3]);
        _pwm0 = val;
        break;
    case TOSHIBA_LED_PWM1:
        // ::fprintf(stderr, "ToshibaLED: pwm1=%u\n", val);
        _pwm1 = val;
        break;
    case TOSHIBA_LED_PWM2:
        // ::fprintf(stderr, "ToshibaLED: pwm2=%u\n", val);
        _pwm2 = val;
        break;
    case TOSHIBA_LED_ENABLE:
        if (val != 0x03) {
            AP_HAL::panic("Unexpected enable value (%u)", val);
        }
        // ::fprintf(stderr, "ToshibaLED: enabling\n");
        _enabled = true;
        break;
    default:
        AP_HAL::panic("Unexpected register (%u)", reg);
    }
    // kill(0, SIGTRAP);
    return -1;
}
