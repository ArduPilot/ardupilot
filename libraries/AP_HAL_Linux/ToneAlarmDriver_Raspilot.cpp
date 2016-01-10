#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "ToneAlarmDriver_Raspilot.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <sys/mman.h>

#include "GPIO.h"
#include "Util_RPI.h"

#define RASPILOT_TONE_PIN     RPI_GPIO_18
#define RASPILOT_TONE_PIN_ALT 5

#define RPI1_GPIO_BASE 0x20200000
#define RPI1_PWM_BASE  0x2020C000
#define RPI1_CLK_BASE  0x20101000

#define RPI2_GPIO_BASE 0x3F200000
#define RPI2_PWM_BASE  0x3F20C000
#define RPI2_CLK_BASE  0x3F101000

#define	RPI_PWM_CTL  0
#define	RPI_PWM_RNG1 4
#define	RPI_PWM_DAT1 5

#define	RPI_PWMCLK_CNTL 40
#define	RPI_PWMCLK_DIV  41

using namespace Linux;

extern const AP_HAL::HAL& hal;

ToneAlarm_Raspilot::ToneAlarm_Raspilot()
{
    tune_num = -1;                    //initialy no tune to play
    tune_pos = 0;
}

bool ToneAlarm_Raspilot::init()
{
    tune_num = 0;                    //play startup tune

    int rpi_version = UtilRPI::from(hal.util)->get_rpi_version();
    uint32_t gpio_address = rpi_version == 1? RPI1_GPIO_BASE: RPI2_GPIO_BASE;
    uint32_t pwm_address  = rpi_version == 1? RPI1_PWM_BASE: RPI2_PWM_BASE;
    uint32_t clk_address  = rpi_version == 1? RPI1_CLK_BASE: RPI2_CLK_BASE;
    // open /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        AP_HAL::panic("Can't open /dev/mem");
    }

    // mmap GPIO
    gpio_map = mmap(
        NULL,                 // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        gpio_address          // Offset to GPIO peripheral
    );

    pwm_map = mmap(
        NULL,                 // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        pwm_address           // Offset to GPIO peripheral
    );

    clk_map = mmap(
        NULL,                 // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        clk_address           // Offset to GPIO peripheral
    );

    close(mem_fd); // No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED || pwm_map == MAP_FAILED || clk_map == MAP_FAILED) {
        AP_HAL::panic("ToneAlarm: Error!! Can't open /dev/mem");
    }

    gpio = (volatile uint32_t *)gpio_map; // Always use volatile pointer!
    pwm  = (volatile uint32_t *)pwm_map;
    clk  = (volatile uint32_t *)clk_map;

    GPIO_MODE_IN(RASPILOT_TONE_PIN);
    GPIO_MODE_ALT(RASPILOT_TONE_PIN, RASPILOT_TONE_PIN_ALT);

    return true;
}

void ToneAlarm_Raspilot::stop()
{
    setPWM0Duty(0);
}

bool ToneAlarm_Raspilot::play()
{
    uint32_t cur_time = AP_HAL::millis();
    if(tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }
    if(cur_note != 0){
        setPWM0Period(1000000/cur_note);
        setPWM0Duty(50);
        cur_note =0;
        prev_time = cur_time;
    }
    if((cur_time - prev_time) > duration){
        stop();
        if(tune[tune_num][tune_pos] == '\0'){
            if(!tune_repeat[tune_num]){
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

void ToneAlarm_Raspilot::setPWM0Period(uint32_t time_us)
{
    // stop clock and waiting for busy flag doesn't work, so kill clock
    *(clk + RPI_PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    // set frequency
    // DIVI is the integer part of the divisor
    // the fractional part (DIVF) drops clock cycles to get the output frequency, bad for servo motors
    // 320 bits for one cycle of 20 milliseconds = 62.5 us per bit = 16 kHz
    int idiv = (int) (19200000.0f / (320000000.0f / time_us));
    if (idiv < 1 || idiv > 0x1000) {
      return;
    }
    *(clk + RPI_PWMCLK_DIV)  = 0x5A000000 | (idiv<<12);

    // source=osc and enable clock
    *(clk + RPI_PWMCLK_CNTL) = 0x5A000011;

    // disable PWM
    *(pwm + RPI_PWM_CTL) = 0;

    // needs some time until the PWM module gets disabled, without the delay the PWM module crashs
    usleep(10);

    // filled with 0 for 20 milliseconds = 320 bits
    *(pwm + RPI_PWM_RNG1) = 320;

    // init with 0%
    setPWM0Duty(0);

    // start PWM1 in serializer mode
    *(pwm + RPI_PWM_CTL) = 3;
}

void ToneAlarm_Raspilot::setPWM0Duty(uint8_t percent)
{
    int bitCount;
    unsigned int bits = 0;

    bitCount = 320 * percent / 100;
    if (bitCount > 320) bitCount = 320;
    if (bitCount < 0) bitCount = 0;
    bits = 0;
    while (bitCount) {
      bits <<= 1;
      bits |= 1;
      bitCount--;
    }
    *(pwm + RPI_PWM_DAT1) = bits;
}

#endif
