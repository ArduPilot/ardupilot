#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "ToneAlarm_Raspilot.h"

#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_Math/AP_Math.h>

#include "GPIO.h"
#include "Util_RPI.h"

#define RASPILOT_TONE_PIN RPI_GPIO_18
#define RASPILOT_TONE_PIN_ALT 5

#define RPI1_PWM_BASE  0x2020C000
#define RPI1_CLK_BASE  0x20101000

#define RPI2_PWM_BASE  0x3F20C000
#define RPI2_CLK_BASE  0x3F101000

#define	RPI_PWM_CTL  0
#define	RPI_PWM_RNG1 4
#define	RPI_PWM_DAT1 5

#define	RPI_PWMCLK_CNTL 40
#define	RPI_PWMCLK_DIV  41

using namespace Linux;

extern const AP_HAL::HAL &hal;

ToneAlarm_Raspilot::ToneAlarm_Raspilot()
{
    // initialy no tune to play
    tune_num = -1;
    tune_pos = 0;
}

bool ToneAlarm_Raspilot::init()
{
    uint32_t pwm_address, clk_address;
    int mem_fd;

    // play startup tune
    tune_num = 0;

    int rpi_version = UtilRPI::from(hal.util)->get_rpi_version();

    if (rpi_version == 1) {
        pwm_address = RPI1_PWM_BASE;
        clk_address = RPI1_CLK_BASE;
    } else {
        pwm_address = RPI2_PWM_BASE;
        clk_address = RPI2_CLK_BASE;
    }

    // open /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC) ) < 0) {
        AP_HAL::panic("Can't open /dev/mem");
    }

    // mmap GPIO
    void *pwm_map = mmap(
        nullptr,              // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        pwm_address           // Offset to GPIO peripheral
    );

    void *clk_map = mmap(
        nullptr,              // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        clk_address           // Offset to GPIO peripheral
    );

    // No need to keep mem_fd open after mmap
    close(mem_fd);

    if (pwm_map == MAP_FAILED || clk_map == MAP_FAILED) {
        AP_HAL::panic("ToneAlarm: Error!! Can't open /dev/mem");
    }

    _pwm  = (volatile uint32_t *)pwm_map;
    _clk  = (volatile uint32_t *)clk_map;

    hal.gpio->pinMode(RASPILOT_TONE_PIN, HAL_GPIO_ALT, 5);

    return true;
}

void ToneAlarm_Raspilot::stop()
{
    _set_pwm0_duty(0);
}

bool ToneAlarm_Raspilot::play()
{
    uint32_t cur_time = AP_HAL::millis();

    if (tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }

    if (cur_note != 0){
        _set_pwm0_period(1000000/cur_note);
        _set_pwm0_duty(50);
        cur_note =0;
        prev_time = cur_time;
    }

    if ((cur_time - prev_time) > duration){
        stop();
        if (tune[tune_num][tune_pos] == '\0'){
            if (!tune_repeat[tune_num]){
                tune_num = -1;
            }

            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }

    return false;
}

void ToneAlarm_Raspilot::_set_pwm0_period(uint32_t time_us)
{
    // stop clock and waiting for busy flag doesn't work, so kill clock
    *(_clk + RPI_PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    // set frequency
    // DIVI is the integer part of the divisor
    // the fractional part (DIVF) drops clock cycles to get the output frequency, bad for servo motors
    // 320 bits for one cycle of 20 milliseconds = 62.5 us per bit = 16 kHz
    int idiv = (int) (19200000.0f / (320000000.0f / time_us));
    if (idiv < 1 || idiv > 0x1000) {
      return;
    }
    *(_clk + RPI_PWMCLK_DIV)  = 0x5A000000 | (idiv<<12);

    // source=osc and enable clock
    *(_clk + RPI_PWMCLK_CNTL) = 0x5A000011;

    // disable PWM
    *(_pwm + RPI_PWM_CTL) = 0;

    // needs some time until the PWM module gets disabled, without the delay the PWM module crashs
    usleep(10);

    // filled with 0 for 20 milliseconds = 320 bits
    *(_pwm + RPI_PWM_RNG1) = 320;

    // init with 0%
    _set_pwm0_duty(0);

    // start PWM1 in serializer mode
    *(_pwm + RPI_PWM_CTL) = 3;
}

void ToneAlarm_Raspilot::_set_pwm0_duty(uint8_t percent)
{
    int bit_count = constrain_int32(320 * percent / 100, 320, 0);
    unsigned int bits = 0;

    // FIXME: bits overflows for any bit_count > 32
    while (bit_count) {
      bits <<= 1;
      bits |= 1;
      bit_count--;
    }

    *(_pwm + RPI_PWM_DAT1) = bits;
}

#endif
