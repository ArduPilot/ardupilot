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
}

void PX4RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
    if (freq_hz == _freq_hz) {
        // avoid the ioctl() if possible
        return;
    }
    // we can't set this per channel yet
    if (ioctl(_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, (unsigned long)freq_hz) == 0) {
        _freq_hz = freq_hz;
    }
}

uint16_t PX4RCOutput::get_freq(uint8_t ch) 
{
	return _freq_hz;
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
    if (ch >= PX4_NUM_OUTPUT_CHANNELS) {
        return;
    }
    if (ch > _max_channel) {
        _max_channel = ch;
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
        ::write(_pwm_fd, _period, _max_channel*sizeof(_period[0]));
        perf_end(_perf_rcout);
        _last_output = now;
    }
}

#endif // CONFIG_HAL_BOARD
