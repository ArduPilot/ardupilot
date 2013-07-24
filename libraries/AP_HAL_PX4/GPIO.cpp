/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "GPIO.h"
#include "defines.h"
#include "config.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* PX4 headers */
#include <drivers/drv_led.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_gpio.h>

#define LOW     1
#define HIGH    0

extern const AP_HAL::HAL& hal;

using namespace PX4;

PX4GPIO::PX4GPIO()
{}

void PX4GPIO::init()
{
#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
    _led_fd = open(LED_DEVICE_PATH, O_RDWR);
    if (_led_fd == -1) {
        hal.scheduler->panic("Unable to open " LED_DEVICE_PATH);
    }
    if (ioctl(_led_fd, LED_OFF, LED_BLUE) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO LED BLUE\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_RED) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    }
#endif
    _tone_alarm_fd = open("/dev/tone_alarm", O_WRONLY);
    if (_tone_alarm_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/tone_alarm");
    }

    _gpio_fd = open(PX4FMU_DEVICE_PATH, O_RDWR);
    if (_gpio_fd == -1) {
        hal.scheduler->panic("Unable to open GPIO");
    }
#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
    if (ioctl(_gpio_fd, GPIO_CLEAR, GPIO_EXT_1) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_1\n");
    }
#endif

}

void PX4GPIO::pinMode(uint8_t pin, uint8_t output)
{}

int8_t PX4GPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t PX4GPIO::read(uint8_t pin) {
    return 0;
}

void PX4GPIO::write(uint8_t pin, uint8_t value)
{

    //char *user_tune = "MBL4O6C";

    switch (pin) {

#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
        case A_LED_PIN:    // Arming LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_RED);
            } else {
                ioctl(_led_fd, LED_ON, LED_RED);
            }
            break;

        case B_LED_PIN:    // not used yet 
            break;

        case C_LED_PIN:    // GPS LED 
            if (value == LOW) { 
                ioctl(_led_fd, LED_OFF, LED_BLUE);
            } else { 
                ioctl(_led_fd, LED_ON, LED_BLUE);
            }
            break;
#endif

        case PX4_GPIO_PIEZO_PIN:    // Piezo beeper 
            if (value == LOW) { // this is inverted 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 3);    // Alarm on !! 
                //::write(_tone_alarm_fd, &user_tune, sizeof(user_tune));
            } else { 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 0);    // Alarm off !! 
            }
            break;

        case PX4_GPIO_EXT_RELAY_PIN: // Ext Relay 
#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
            if (value == LOW) {
                ioctl(_gpio_fd, GPIO_CLEAR, GPIO_EXT_1);
            } else {
                ioctl(_gpio_fd, GPIO_SET, GPIO_EXT_1);
            }
#endif
            break;
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* PX4GPIO::channel(uint16_t n) {
    return new PX4DigitalSource(0);
}

/* Interrupt interface: */
bool PX4GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}


PX4DigitalSource::PX4DigitalSource(uint8_t v) :
    _v(v)
{}

void PX4DigitalSource::mode(uint8_t output)
{}

uint8_t PX4DigitalSource::read() {
    return _v;
}

void PX4DigitalSource::write(uint8_t value) {
    _v = value;
}

#endif // CONFIG_HAL_BOARD
