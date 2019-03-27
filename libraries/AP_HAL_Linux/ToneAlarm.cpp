#include "ToneAlarm.h"

#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

ToneAlarm::ToneAlarm()
{
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    period_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/period",O_WRONLY|O_CLOEXEC);
    duty_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/duty",O_WRONLY|O_CLOEXEC);
    run_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/run",O_WRONLY|O_CLOEXEC);
#endif
}

bool ToneAlarm::init()
{
    if ((period_fd == -1) || (duty_fd == -1) || (run_fd == -1)) {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        hal.console->printf("ToneAlarm: Error!! please check if PWM overlays are loaded correctly");
#endif
        return false;
    }
    return true;
}

void ToneAlarm::set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (is_zero(frequency) || is_zero(volume)) {
        dprintf(run_fd,"0");
    } else {
        dprintf(run_fd,"0");
        dprintf(period_fd,"%u",(unsigned int)roundf(1000000000/frequency));
        dprintf(duty_fd,"%u",(unsigned int)roundf(volume*500000000/frequency));
        dprintf(run_fd,"1");
    }
}
