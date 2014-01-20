/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   AP_BoardConfig - board specific configuration
 */


#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_BoardConfig.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define BOARD_PWM_COUNT_DEFAULT 2
#else
#define BOARD_PWM_COUNT_DEFAULT 4
#endif
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_BoardConfig::var_info[] PROGMEM = {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Param: PWM_COUNT
    // @DisplayName: PWM Count
    // @Description: Number of auxillary PWMs to enable. On PX4v1 only 0 or 2 is valid. On Pixhawk 0, 2, 4 or 6 is valid.
    // @Values: 0:No PWMs,2:Two PWMs,4:Four PWMs,6:Six PWMs
    AP_GROUPINFO("PWM_COUNT",    0, AP_BoardConfig, _pwm_count, BOARD_PWM_COUNT_DEFAULT),
#endif

    AP_GROUPEND
};


void AP_BoardConfig::init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    /* configurre the FMU driver for the right number of PWMs */

    // ensure only valid values are set, rounding up
    if (_pwm_count > 6) _pwm_count.set(6);
    if (_pwm_count < 0) _pwm_count.set(0);
    if (_pwm_count == 1) _pwm_count.set(2);
    if (_pwm_count == 3) _pwm_count.set(4);
    if (_pwm_count == 5) _pwm_count.set(6);

    int fd = open("/dev/px4fmu", 0);
    if (fd == -1) {
        hal.scheduler->panic("Unable to open /dev/px4fmu");
    }
    if (ioctl(fd, PWM_SERVO_SET_COUNT, _pwm_count.get()) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt PWM to %u channels\n", _pwm_count.get());  
    }   
    close(fd);
#endif    
}
