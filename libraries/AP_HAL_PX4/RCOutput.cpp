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
    _pwm_fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
        hal.scheduler->panic("Unable to open " PWM_OUTPUT_DEVICE_PATH);
    }
    ioctl(_pwm_fd, PWM_SERVO_ARM, 0);
}

void PX4RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
	// no support for this yet
}

uint16_t PX4RCOutput::get_freq(uint8_t ch) 
{
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
    ioctl(_pwm_fd, PWM_SERVO_SET(ch), (unsigned long)period_us);
}

void PX4RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        write(i, period_us[i]);
    }
}

uint16_t PX4RCOutput::read(uint8_t ch) 
{
    servo_position_t pos;
	if (ioctl(_pwm_fd, PWM_SERVO_GET(ch), (unsigned long)&pos) == 0) {
       return pos;
    }
    return 0;
}

void PX4RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

#endif // CONFIG_HAL_BOARD
