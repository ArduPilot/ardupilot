/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "RCOutput.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_output.h>

extern const AP_HAL::HAL& hal;

using namespace PX4;

void PX4RCOutput::init(void* unused) 
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
    _pwm_fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
        hal.scheduler->panic("Unable to open " PWM_OUTPUT_DEVICE_PATH);
    }
    if (ioctl(_pwm_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming\n");
    }
    _rate_mask = 0;
    _alt_fd = -1;    
    _servo_count = 0;
    _alt_servo_count = 0;

    if (ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");        
        return;
    }

    /* if we have 8 servos, then we are attached to PX4IO. In that
     * case, we want to open the PX4FMU driver for the 4 additional
     * channels
     */
    if (_servo_count <= 4) {
        return;
    }
    _alt_fd = open("/dev/px4fmu", O_RDWR);
    if (_alt_fd == -1) {
        hal.console->printf("RCOutput: failed to open /dev/px4fmu");
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt IO arming\n");
    }
    if (ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");        
        return;
    }
}

void PX4RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
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
        if (_rate_mask & 0x3) {
            _rate_mask |= 0x3;
        }
        if (_rate_mask & 0xc) {
            _rate_mask |= 0xc;
        }
        if (_rate_mask & 0xF0) {
            _rate_mask |= 0xF0;
        }
    } else {
        // we are setting low rates on the given channels
        if (chmask & 0x3) {
            _rate_mask &= ~0x3;
        }
        if (chmask & 0xc) {
            _rate_mask &= ~0xc;
        }
        if (chmask & 0xf0) {
            _rate_mask &= ~0xf0;
        }
    }

    if (ioctl(_pwm_fd, PWM_SERVO_SELECT_UPDATE_RATE, _rate_mask) != 0) {
        hal.console->printf("RCOutput: Unable to set alt rate mask to 0x%x\n", (unsigned)_rate_mask);
    }
}

uint16_t PX4RCOutput::get_freq(uint8_t ch) 
{
    if (_rate_mask & (1<<ch)) {
        return _freq_hz;
    }
    return 50;
}

void PX4RCOutput::enable_ch(uint8_t ch)
{
    // channels are always enabled ...
}

void PX4RCOutput::enable_mask(uint32_t chmask)
{
    // channels are always enabled ...
}

void PX4RCOutput::disable_ch(uint8_t ch)
{
    // channels are always enabled ...
}

void PX4RCOutput::disable_mask(uint32_t chmask)
{
    // channels are always enabled ...
}

void PX4RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _servo_count + _alt_servo_count) {
        return;
    }
    if (ch >= _max_channel) {
        _max_channel = ch + 1;
    }
    if (period_us != _period[ch]) {
        _period[ch] = period_us;
        _need_update = true;
    }
}

void PX4RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        write(i, period_us[i]);
    }
}

uint16_t PX4RCOutput::read(uint8_t ch) 
{
    if (ch >= PX4_NUM_OUTPUT_CHANNELS) {
        return 0;
    }
    return _period[ch];
}

void PX4RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

void PX4RCOutput::_timer_tick(void)
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
        if (_max_channel <= _servo_count) {
            ::write(_pwm_fd, _period, _max_channel*sizeof(_period[0]));
        } else {
            // we're using both sets of outputs
            ::write(_pwm_fd, _period, _servo_count*sizeof(_period[0]));
            if (_alt_fd != -1 && _alt_servo_count > 0) {
                uint8_t n = _max_channel - _servo_count;
                if (n > _alt_servo_count) {
                    n = _alt_servo_count;
                }
                ::write(_alt_fd, &_period[_servo_count], n*sizeof(_period[0]));
            }
        }
        perf_end(_perf_rcout);
        _last_output = now;
    }
}

#endif // CONFIG_HAL_BOARD
