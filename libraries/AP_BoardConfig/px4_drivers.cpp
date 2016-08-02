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
 *   AP_BoardConfig - px4 driver loading and setup
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
#define BOARD_PWM_COUNT_DEFAULT 2
#define BOARD_SER1_RTSCTS_DEFAULT 0 // no flow control on UART5 on FMUv1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define BOARD_PWM_COUNT_DEFAULT 6
#define BOARD_SER1_RTSCTS_DEFAULT 2
#else // V2
#define BOARD_PWM_COUNT_DEFAULT 4
#define BOARD_SER1_RTSCTS_DEFAULT 2
#endif

extern const AP_HAL::HAL& hal;

#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
extern "C" int uavcan_main(int argc, const char *argv[]);

#define _UAVCAN_IOCBASE             (0x4000)                        // IOCTL base for module UAVCAN
#define _UAVCAN_IOC(_n)             (_IOC(_UAVCAN_IOCBASE, _n))

#define UAVCAN_IOCG_NODEID_INPROGRESS  _UAVCAN_IOC(1)               // query if node identification is in progress
#endif


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
    };
    uint8_t mode_parm = (uint8_t)px4.pwm_count.get();
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
        if (mode_table[i].num_gpios < 2) {
            // reduce change of config mistake where relay and PWM interfere
            AP_Param::set_default_by_name("RELAY_PIN", -1);
            AP_Param::set_default_by_name("RELAY_PIN2", -1);
        }
    }
}


/*
  setup flow control on UARTs
 */
void AP_BoardConfig::px4_setup_uart()
{
    hal.uartC->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser1_rtscts.get());
    if (hal.uartD != NULL) {
        hal.uartD->set_flow_control((AP_HAL::UARTDriver::flow_control)px4.ser2_rtscts.get());
    }
}

/*
  setup safety switch
 */
void AP_BoardConfig::px4_setup_safety()
{
    // setup channels to ignore the armed state
    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd != -1) {
        if (ioctl(px4io_fd, PWM_SERVO_IGNORE_SAFETY, (uint16_t)(0x0000FFFF & px4.ignore_safety_channels)) != 0) {
            hal.console->printf("IGNORE_SAFETY failed\n");
        }
        close(px4io_fd);
    }

    if (px4.safety_enable.get() == 0) {
        hal.rcout->force_safety_off();
    }
}


/*
  setup SBUS
 */
void AP_BoardConfig::px4_setup_sbus(void)
{
    if (px4.sbus_out_rate.get() >= 1) {
        static const struct {
            uint8_t value;
            uint16_t rate;
        } rates[] = {
            { 1, 50 },
            { 2, 75 },
            { 3, 100 },
            { 4, 150 },
            { 5, 200 },
            { 6, 250 },
            { 7, 300 }
        };
        uint16_t rate = 300;
        for (uint8_t i=0; i<ARRAY_SIZE(rates); i++) {
            if (rates[i].value == px4.sbus_out_rate) {
                rate = rates[i].rate;
            }
        }
        if (!hal.rcout->enable_sbus_out(rate)) {
            hal.console->printf("Failed to enable SBUS out\n");
        }
    }
}


/*
  setup CANBUS drivers
 */
void AP_BoardConfig::px4_setup_canbus(void)
{
#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    if (px4.can_enable >= 1) {
        const char *args[] = { "uavcan", "start", NULL };
        int ret = uavcan_main(3, args);
        if (ret != 0) {
            hal.console->printf("UAVCAN: failed to start\n");
        } else {
            hal.console->printf("UAVCAN: started\n");            
            // give some time for CAN bus initialisation
            hal.scheduler->delay(2000);
        }
    }
    if (px4.can_enable >= 2) {
        const char *args[] = { "uavcan", "start", "fw", NULL };
        int ret = uavcan_main(4, args);
        if (ret != 0) {
            hal.console->printf("UAVCAN: failed to start servers\n");
        } else {
            uint32_t start_wait_ms = AP_HAL::millis();
            int fd = open("/dev/uavcan/esc", 0); // design flaw of uavcan driver, this should be /dev/uavcan/node one day
            if (fd == -1) {
                AP_HAL::panic("Configuration invalid - unable to open /dev/uavcan/esc");
            }

            // delay startup, UAVCAN still discovering nodes
            while (ioctl(fd, UAVCAN_IOCG_NODEID_INPROGRESS,0) == OK &&
                   AP_HAL::millis() - start_wait_ms < 7000) {
                hal.scheduler->delay(500);
            }
            hal.console->printf("UAVCAN: node discovery complete\n");
            close(fd);
        }
   }
#endif // CONFIG_ARCH_BOARD_PX4FMU_V1
}

void AP_BoardConfig::px4_setup()
{
    px4_setup_pwm();
    px4_setup_safety();
    px4_setup_uart();
    px4_setup_sbus();
    px4_setup_canbus();
}

#endif // HAL_BOARD_PX4
