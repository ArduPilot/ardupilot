/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "GPIO.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* VRBRAIN headers */
#include <drivers/drv_led.h>
#include <drivers/drv_buzzer.h>
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
    _led_fd = open(LED_DEVICE_PATH, O_RDWR);
    if (_led_fd == -1) {
        AP_HAL::panic("Unable to open " LED_DEVICE_PATH);
    }
    if (ioctl(_led_fd, LED_OFF, LED_BLUE) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO LED BLUE\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_RED) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    }
    if (ioctl(_led_fd, LED_OFF, LED_GREEN) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED GREEN\n");
    }
#if defined(LED_EXT1)
    if (ioctl(_led_fd, LED_OFF, LED_EXT1) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED EXT 1\n");
    }
#endif
#if defined(LED_EXT2)
    if (ioctl(_led_fd, LED_OFF, LED_EXT2) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED EXT 2\n");
    }
#endif
#if defined(LED_EXT3)
    if (ioctl(_led_fd, LED_OFF, LED_EXT3) != 0) {
         hal.console->printf("GPIO: Unable to setup GPIO LED EXT 3\n");
    }
#endif

#if defined(BUZZER_EXT)
    _buzzer_fd = open(BUZZER_DEVICE_PATH, O_RDWR);
    if (_buzzer_fd == -1) {
        AP_HAL::panic("Unable to open " BUZZER_DEVICE_PATH);
    }
    if (ioctl(_buzzer_fd, BUZZER_OFF, BUZZER_EXT) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO BUZZER\n");
    }
#endif

#if defined(GPIO_GPIO0_OUTPUT)
    stm32_configgpio(GPIO_GPIO0_OUTPUT);
#endif

#if defined(GPIO_GPIO1_OUTPUT)
    stm32_configgpio(GPIO_GPIO1_OUTPUT);
#endif
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


uint8_t VRBRAINGPIO::read(uint8_t pin)
{

    switch (pin) {
        case EXTERNAL_RELAY1_PIN:
#if defined(GPIO_GPIO0_OUTPUT)
        	return (stm32_gpioread(GPIO_GPIO0_OUTPUT))?HIGH:LOW;
#endif
            break;
        case EXTERNAL_RELAY2_PIN:
#if defined(GPIO_GPIO1_OUTPUT)
        	return (stm32_gpioread(GPIO_GPIO1_OUTPUT))?HIGH:LOW;
#endif
            break;
    }
    return LOW;
}

void VRBRAINGPIO::write(uint8_t pin, uint8_t value)
{
    switch (pin) {

        case HAL_GPIO_A_LED_PIN:    // Arming LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_GREEN);
            } else {
                ioctl(_led_fd, LED_ON, LED_GREEN);
            }
            break;

        case HAL_GPIO_B_LED_PIN:    // not used yet
            if (value == LOW) { 
                ioctl(_led_fd, LED_OFF, LED_BLUE);
            } else { 
                ioctl(_led_fd, LED_ON, LED_BLUE);
            }
            break;

        case HAL_GPIO_C_LED_PIN:    // GPS LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_RED);
            } else {
                ioctl(_led_fd, LED_ON, LED_RED);
            }
            break;

        case EXTERNAL_LED_GPS:
#if defined(LED_EXT1)
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_EXT1);
            } else {
                ioctl(_led_fd, LED_ON, LED_EXT1);
            }
#endif
            break;

        case EXTERNAL_LED_ARMED:
#if defined(LED_EXT2)
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_EXT2);
            } else {
                ioctl(_led_fd, LED_ON, LED_EXT2);
            }
#endif
            break;

        case EXTERNAL_LED_MOTOR1:
            break;

        case EXTERNAL_LED_MOTOR2:
            break;

        case BUZZER_PIN:
#if defined(BUZZER_EXT)
            if (value == LOW) {
                ioctl(_buzzer_fd, BUZZER_OFF, BUZZER_EXT);
            } else {
                ioctl(_buzzer_fd, BUZZER_ON, BUZZER_EXT);
            }
#endif
            break;

        case EXTERNAL_RELAY1_PIN:
#if defined(GPIO_GPIO0_OUTPUT)
        	stm32_gpiowrite(GPIO_GPIO0_OUTPUT, (value==HIGH));
#endif
        	break;
        case EXTERNAL_RELAY2_PIN:
#if defined(GPIO_GPIO1_OUTPUT)
        	stm32_gpiowrite(GPIO_GPIO1_OUTPUT, (value==HIGH));
#endif
        	break;

    }
}

void VRBRAINGPIO::toggle(uint8_t pin)
{
    switch (pin) {

        case HAL_GPIO_A_LED_PIN:    // Arming LED
			ioctl(_led_fd, LED_TOGGLE, LED_GREEN);
            break;

        case HAL_GPIO_B_LED_PIN:    // not used yet
			ioctl(_led_fd, LED_TOGGLE, LED_BLUE);
            break;

        case HAL_GPIO_C_LED_PIN:    // GPS LED
			ioctl(_led_fd, LED_TOGGLE, LED_RED);
            break;

        case EXTERNAL_LED_GPS:
#if defined(LED_EXT1)
			ioctl(_led_fd, LED_TOGGLE, LED_EXT1);
#endif
            break;

        case EXTERNAL_LED_ARMED:
#if defined(LED_EXT2)
			ioctl(_led_fd, LED_TOGGLE, LED_EXT2);
#endif
            break;

        case EXTERNAL_LED_MOTOR1:

        	break;

        case EXTERNAL_LED_MOTOR2:

            break;

        case BUZZER_PIN:
#if defined(BUZZER_EXT)
			ioctl(_buzzer_fd, BUZZER_TOGGLE, BUZZER_EXT);
#endif
            break;

        default:
            write(pin, !read(pin));
        	break;
    }
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
