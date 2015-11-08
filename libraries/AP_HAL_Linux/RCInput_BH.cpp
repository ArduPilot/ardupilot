#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "GPIO_RPI.h"
#include "RCInput.h"
#include "RCInput_BH.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// This is a prototype code for RC input decoding on Raspberry Pi.
// It uses pigpio daemon to sample GPIOs over DMA with 1 microsecond resolution.
// This code should be rewritten to configure DMA GPIO sampling without pigpio.
#if HAL_BOARD_SUBTYPE_LINUX_BH_1
static uint16_t gpios[MAX_GPIOS] = { RPI_GPIO_5,
                                     RPI_GPIO_6,
                                     RPI_GPIO_13,
                                     RPI_GPIO_19,
                                     RPI_GPIO_26,
                                     RPI_GPIO_12,
                                     RPI_GPIO_20,
                                     RPI_GPIO_21};
#else
static uint16_t gpios[MAX_GPIOS] = { RPI_GPIO_5,
                                     RPI_GPIO_6,
                                     RPI_GPIO_12,
                                     RPI_GPIO_13,
                                     RPI_GPIO_19,
                                     RPI_GPIO_20,
                                     RPI_GPIO_21,
                                     RPI_GPIO_26};
#endif // HAL_BOARD_SUBTYPE_LINUX_BH_1

void RCInput_BH::init(void*)
{
    curtick = 0;
    for(int i=0; i<MAX_GPIOS; i++) {
      width_s0[i] = 0;
      width_s1[i] = 0;
      prevtick[i] = 0;
    }

    // Kills pigpio daemon in case it was run with wrong parameters
    unlink("/var/run/pigpio.pid");
    system("killall pigpiod -q");
    hal.scheduler->delay(1000);

    // Starts pigpio daemon with 1 microsecond sampling resolution

    system("pigpiod -s 1 -b 256 -f");
    hal.scheduler->delay(5000); // Wait enough time for pigpiod to be ready

    // Configures pigpiod to send GPIO change notifications to /dev/pigpio0
    // BH use GPIO5/6/13/12/19/20/21/26 b100001110000011000001100000
    // b100001110000011000001100000 = 0x4383060
    //system("pigs NO NB 0 0x4383060");

    uint32_t pins = 0;
    for(int i = 0; i < MAX_GPIOS; i++) {
      pins = pins | (1 << gpios[i]);
    }
    char pigs_cmd[128] = {0};
    sprintf(pigs_cmd, "pigs NO NB 0 0x%x", pins);

    system(pigs_cmd);
    hal.scheduler->delay(1000);

    // Configures /dev/pigpio0 for non-blocking read

    pigpio = open("/dev/pigpio0", O_RDONLY);

    if (pigpio == -1)
        hal.scheduler->panic("No pigpio interface for RCInput");

    set_num_channels(MAX_GPIOS);
}

void RCInput_BH::_timer_tick()
{
    static bool is_first_read = true;
    static uint32_t old_level = 0;

    while (true) {
        int bytesAvailable;
        ioctl(pigpio, FIONREAD, &bytesAvailable);

        if (bytesAvailable >= 12) {
            ::read(pigpio, reinterpret_cast<uint8_t*>(&gpioReport), 12);
            if (is_first_read) {
                old_level = gpioReport.level;
                curtick = gpioReport.tick;
                for(int i=0; i<MAX_GPIOS; i++) {
                    prevtick[i] = curtick;
                }
                is_first_read = false;
                continue;
            }
        } else {
            break;
        }

        uint32_t old_bit = 0;
        uint32_t new_bit = 0;
        curtick = gpioReport.tick;
        for(int i = 0; i < MAX_GPIOS; i++) {
            old_bit = (old_level >> gpios[i]) & 0x01;
            new_bit = (gpioReport.level >> gpios[i]) & 0x01;
            if (new_bit != old_bit) {
                if (new_bit == 0) {
                    width_s1[i] = curtick - prevtick[i];
                    _process_rc_pulse(width_s0[i], width_s1[i], i);
                } else {
                    width_s0[i] = curtick - prevtick[i];
                }
                prevtick[i] = curtick;
            }
        }
        old_level = gpioReport.level;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
