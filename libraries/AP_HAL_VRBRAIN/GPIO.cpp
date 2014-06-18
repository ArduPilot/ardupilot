/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "GPIO.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* VRBRAIN headers */
#include <drivers/drv_led.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_gpio.h>

#include <arch/board/board.h>
#include <board_config.h>

#define LOW     0
#define HIGH    1

extern const AP_HAL::HAL& hal;

using namespace VRBRAIN;

VRBRAINGPIO::VRBRAINGPIO()
{}

void VRBRAINGPIO::init()
{
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V4) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V5) || defined(CONFIG_ARCH_BOARD_VRHERO_V1)
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





















}

void VRBRAINGPIO::pinMode(uint8_t pin, uint8_t output)
{
    switch (pin) {



    }
}

int8_t VRBRAINGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t VRBRAINGPIO::read(uint8_t pin) {

    switch (pin) {











































    }
    return LOW;
}

void VRBRAINGPIO::write(uint8_t pin, uint8_t value)
{
    switch (pin) {

#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V4) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V5) || defined(CONFIG_ARCH_BOARD_VRHERO_V1)
        case HAL_GPIO_A_LED_PIN:    // Arming LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_RED);
            } else {
                ioctl(_led_fd, LED_ON, LED_RED);
            }
            break;

        case HAL_GPIO_B_LED_PIN:    // not used yet 
            break;

        case HAL_GPIO_C_LED_PIN:    // GPS LED 
            if (value == LOW) { 
                ioctl(_led_fd, LED_OFF, LED_BLUE);
            } else { 
                ioctl(_led_fd, LED_ON, LED_BLUE);
            }
            break;
#endif

        case VRBRAIN_GPIO_PIEZO_PIN:    // Piezo beeper 
            if (value == LOW) { // this is inverted 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 3);    // Alarm on !! 
                //::write(_tone_alarm_fd, &user_tune, sizeof(user_tune));
            } else { 
                ioctl(_tone_alarm_fd, TONE_SET_ALARM, 0);    // Alarm off !! 
            }
            break;








































    }
}

void VRBRAINGPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* VRBRAINGPIO::channel(uint16_t n) {
    return new VRBRAINDigitalSource(0);
}

/* Interrupt interface: */
bool VRBRAINGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

/*
  return true when USB connected
 */
bool VRBRAINGPIO::usb_connected(void)
{
    return stm32_gpioread(GPIO_OTGFS_VBUS);
}


VRBRAINDigitalSource::VRBRAINDigitalSource(uint8_t v) :
    _v(v)
{}

void VRBRAINDigitalSource::mode(uint8_t output)
{}

uint8_t VRBRAINDigitalSource::read() {
    return _v;
}

void VRBRAINDigitalSource::write(uint8_t value) {
    _v = value;
}

void VRBRAINDigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
