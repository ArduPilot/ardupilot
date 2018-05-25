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
 *   AP_BoardConfig - px4 driver loading and setup
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <nuttx/arch.h>
#include <spawn.h>

extern const AP_HAL::HAL& hal;

/* 
   declare driver main entry points
 */
extern "C" {
    int fmu_main(int, char **);
    int px4io_main(int, char **);
    int adc_main(int, char **);
    int tone_alarm_main(int, char **);
};

/*
  setup PWM pins
 */
void AP_BoardConfig::px4_setup_pwm()
{
    /* configure the FMU driver for the right number of PWMs */
    static const struct {
        uint8_t mode_parm;
        uint8_t mode_value;
        uint8_t num_gpios;
    } mode_table[] = {
        /* table mapping BRD_PWM_COUNT to ioctl arguments */
        { 0, PWM_SERVO_MODE_NONE, 6 },
        { 2, PWM_SERVO_MODE_2PWM, 4 },
        { 4, PWM_SERVO_MODE_4PWM, 2 },
        { 6, PWM_SERVO_MODE_6PWM, 0 },
        { 7, PWM_SERVO_MODE_3PWM1CAP, 2 },
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        { 8, PWM_SERVO_MODE_12PWM, 0 },
#endif
    };
    uint8_t mode_parm = (uint8_t)pwm_count.get();
    uint8_t i;
    for (i=0; i<ARRAY_SIZE(mode_table); i++) {
        if (mode_table[i].mode_parm == mode_parm) {
            break;
        }
    }
    if (i == ARRAY_SIZE(mode_table)) {
        hal.console->printf("RCOutput: invalid BRD_PWM_COUNT %u\n", mode_parm);
    } else {
        int fd = open("/dev/px4fmu", 0);
        if (fd == -1) {
            AP_HAL::panic("Unable to open /dev/px4fmu");
        }
        if (ioctl(fd, PWM_SERVO_SET_MODE, mode_table[i].mode_value) != 0) {
            hal.console->printf("RCOutput: unable to setup AUX PWM with BRD_PWM_COUNT %u\n", mode_parm);
        }
        close(fd);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        if (mode_table[i].num_gpios < 2) {
            // reduce change of config mistake where relay and PWM interfere
            AP_Param::set_default_by_name("RELAY_PIN", -1);
            AP_Param::set_default_by_name("RELAY_PIN2", -1);
        }
#endif
    }
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety_mask()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // setup channels to ignore the armed state
    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd != -1) {
        if (ioctl(px4io_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)state.ignore_safety_channels) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
    }
    int px4fmu_fd = open("/dev/px4fmu", 0);
    if (px4fmu_fd != -1) {
        uint16_t mask = state.ignore_safety_channels;
        if (px4io_fd != -1) {
            mask >>= 8;
        }
        if (ioctl(px4fmu_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)mask) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
        close(px4fmu_fd);
    }
    if (px4io_fd != -1) {
        close(px4io_fd);
    }
#endif
}

extern "C" int waitpid(pid_t, int *, int);

/*
  start one px4 driver
 */
bool AP_BoardConfig::px4_start_driver(main_fn_t main_function, const char *name, const char *arguments)
{
    char *s = strdup(arguments);
    char *args[10];
    uint8_t nargs = 0;
    char *saveptr = nullptr;

    // parse into separate arguments
    for (char *tok=strtok_r(s, " ", &saveptr); tok; tok=strtok_r(nullptr, " ", &saveptr)) {
        args[nargs++] = tok;
        if (nargs == ARRAY_SIZE(args)-1) {
            break;
        }
    }
    args[nargs++] = nullptr;

    printf("Starting driver %s %s\n", name, arguments);
    pid_t pid;

    if (task_spawn(&pid, name, main_function, nullptr, nullptr,
                   args, nullptr) != 0) {
        free(s);
        printf("Failed to spawn %s\n", name);
        return false;
    }

    // wait for task to exit and gather status
    int status = -1;
    if (waitpid(pid, &status, 0) != pid) {
        printf("waitpid failed for %s\n", name);
        free(s);
        return false;
    }
    free(s);
    return (status >> 8) == 0;
}

/*
  play a tune
 */
void AP_BoardConfig::px4_tone_alarm(const char *tone_string)
{
    px4_start_driver(tone_alarm_main, "tone_alarm", tone_string);
}

/*
  setup px4io, possibly updating firmware
 */
void AP_BoardConfig::px4_setup_px4io(void)
{
    if (px4_start_driver(px4io_main, "px4io", "start norc")) {
        printf("px4io started OK\n");
    } else {
        // might be in bootloader mode if user held down safety switch
        // at power on
        printf("Loading /etc/px4io/px4io.bin\n");
        px4_tone_alarm("MBABGP");
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
        // we need to close uartC to prevent conflict between bootloader and
        // uartC reada
        hal.uartC->end();
#endif
        if (px4_start_driver(px4io_main, "px4io", "update /etc/px4io/px4io.bin")) {
            printf("upgraded PX4IO firmware OK\n");
            px4_tone_alarm("MSPAA");
        } else {
            printf("Failed to upgrade PX4IO firmware\n");
            px4_tone_alarm("MNGGG");
        }
        hal.scheduler->delay(1000);
        if (px4_start_driver(px4io_main, "px4io", "start norc")) {
            printf("px4io started OK\n");
        } else {
            sensor_config_error("px4io start failed");
        }
    }

    /*
      see if we need to update px4io firmware
     */
    if (px4_start_driver(px4io_main, "px4io", "checkcrc /etc/px4io/px4io.bin")) {
        printf("PX4IO CRC OK\n");
    } else {
        printf("PX4IO CRC failure\n");
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
        // we need to close uartC to prevent conflict between bootloader and
        // uartC reada
        hal.uartC->end();
#endif
        px4_tone_alarm("MBABGP");
        if (px4_start_driver(px4io_main, "px4io", "safety_on")) {
            printf("PX4IO disarm OK\n");
        } else {
            printf("PX4IO disarm failed\n");
        }
        hal.scheduler->delay(1000);

        if (px4_start_driver(px4io_main, "px4io", "forceupdate 14662 /etc/px4io/px4io.bin")) {
            hal.scheduler->delay(1000);
            if (px4_start_driver(px4io_main, "px4io", "start norc")) {
                printf("px4io restart OK\n");
                px4_tone_alarm("MSPAA");
            } else {
                px4_tone_alarm("MNGGG");
                sensor_config_error("PX4IO restart failed");
            }
        } else {
            printf("PX4IO update failed\n");
            px4_tone_alarm("MNGGG");
        }
    }
}

/*
  setup required peripherals like adc, rcinput and rcoutput
 */
void AP_BoardConfig::px4_setup_peripherals(void)
{
    // always start adc
    if (px4_start_driver(adc_main, "adc", "start")) {
        hal.analogin->init();
        printf("ADC started OK\n");
    } else {
        sensor_config_error("no ADC found");
    }

#if HAL_PX4_HAVE_PX4IO
    if (state.io_enable.get() != 0) {
        px4_setup_px4io();
    }
#endif

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    const char *fmu_mode = "mode_serial";
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    const char *fmu_mode = "mode_rcin";
#else
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    const char *fmu_mode = "mode_pwm";
#else
    const char *fmu_mode = "mode_pwm4";
#endif
#endif
    if (px4_start_driver(fmu_main, "fmu", fmu_mode)) {
        printf("fmu %s started OK\n", fmu_mode);
    } else {
        sensor_config_error("fmu start failed");
    }

    hal.gpio->init();
    hal.rcin->init();
    hal.rcout->init();
}

#endif // HAL_BOARD_PX4
