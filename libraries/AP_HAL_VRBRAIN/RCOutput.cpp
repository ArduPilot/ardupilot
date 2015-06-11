/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "RCOutput.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_output.h>

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;

void VRBRAINRCOutput::init(void* unused)
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
    _pwm_fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
        hal.scheduler->panic("Unable to open " PWM_OUTPUT_DEVICE_PATH);
    }
    if (ioctl(_pwm_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming\n");
    }
    if (ioctl(_pwm_fd, PWM_SERVO_SET_ARM_OK, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming OK\n");
    }
    _rate_mask = 0;

    _servo_count = 0;


    if (ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");        
        return;
    }
}


void VRBRAINRCOutput::_init_alt_channels(void)
{

}

void VRBRAINRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // we can't set this per channel yet
    if (freq_hz > 50) {
        // we're being asked to set the alt rate
        if (ioctl(_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, (unsigned long)freq_hz) != 0) {
            hal.console->printf("RCOutput: Unable to set alt rate to %uHz\n", (unsigned)freq_hz);
            return;
        }
        _freq_hz = freq_hz;
    }

    /* work out the new rate mask. The PX4IO board has 3 groups of servos. 

       Group 0: channels 0 1
       Group 1: channels 4 5 6 7
       Group 2: channels 2 3

       Channels within a group must be set to the same rate.

       For the moment we never set the channels above 8 to more than
       50Hz
     */
    if (freq_hz > 50) {
        // we are setting high rates on the given channels
        _rate_mask |= chmask & 0xFF;
        if (_rate_mask & 0x07) {
            _rate_mask |= 0x07;
        }
        if (_rate_mask & 0x38) {
            _rate_mask |= 0x38;
        }
        if (_rate_mask & 0xC0) {
            _rate_mask |= 0xC0;
        }
    } else {
        // we are setting low rates on the given channels
        if (chmask & 0x07) {
            _rate_mask &= ~0x07;
        }
        if (chmask & 0x38) {
            _rate_mask &= ~0x38;
        }
        if (chmask & 0xC0) {
            _rate_mask &= ~0xC0;
        }
    }

    if (ioctl(_pwm_fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, _rate_mask) != 0) {
        hal.console->printf("RCOutput: Unable to set alt rate mask to 0x%x\n", (unsigned)_rate_mask);
    }
}

uint16_t VRBRAINRCOutput::get_freq(uint8_t ch)
{
    if (_rate_mask & (1U<<ch)) {
        return _freq_hz;
    }
    return 50;
}

void VRBRAINRCOutput::enable_ch(uint8_t ch)
{
    _enabled_channels |= (1U<<ch);
}

void VRBRAINRCOutput::disable_ch(uint8_t ch)
{
    _enabled_channels &= ~(1U<<ch);
}

void VRBRAINRCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup disarmed PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

void VRBRAINRCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FAILSAFE_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup failsafe PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

bool VRBRAINRCOutput::force_safety_on(void)
{
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);
    return (ret == OK);
}

void VRBRAINRCOutput::force_safety_off(void)
{
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);
    if (ret != OK) {
        hal.console->printf("Failed to force safety off\n");
    }
}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _servo_count) {
        return;
    }
    if (!(_enabled_channels & (1U<<ch))) {
        // not enabled
        return;
    }
    if (ch >= _max_channel) {
        _max_channel = ch + 1;
    }
    if (period_us != _period[ch]) {
        _period[ch] = period_us;
        _need_update = true;
        up_pwm_servo_set(ch, period_us);
    }
}

void VRBRAINRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        write(i, period_us[i]);
    }
}

uint16_t VRBRAINRCOutput::read(uint8_t ch)
{
    if (ch >= VRBRAIN_NUM_OUTPUT_CHANNELS) {
        return 0;
    }
    return _period[ch];
}

void VRBRAINRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

void VRBRAINRCOutput::_timer_tick(void)
{
    uint32_t now = hal.scheduler->micros();

    // always send at least at 20Hz, otherwise the IO board may think
    // we are dead
    if (now - _last_output > 50000) {
        _need_update = true;
    }

    if (_need_update && _pwm_fd != -1) {
        _need_update = false;
        perf_begin(_perf_rcout);
        ::write(_pwm_fd, _period, _max_channel*sizeof(_period[0]));
        perf_end(_perf_rcout);
        _last_output = now;
    }
}

#endif // CONFIG_HAL_BOARD
