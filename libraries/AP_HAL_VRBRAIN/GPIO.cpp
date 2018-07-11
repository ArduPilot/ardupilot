#include <AP_HAL/AP_HAL.h>

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

    _led_fd = open(LED0_DEVICE_PATH, O_RDWR);
    if (_led_fd == -1) {
        AP_HAL::panic("Unable to open " LED0_DEVICE_PATH);
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



    _tone_alarm_fd = open(TONEALARM0_DEVICE_PATH, O_WRONLY);
    if (_tone_alarm_fd == -1) {
        AP_HAL::panic("Unable to open " TONEALARM0_DEVICE_PATH);
    }

    _gpio_fmu_fd = open(PX4FMU_DEVICE_PATH, 0);
    if (_gpio_fmu_fd == -1) {
        AP_HAL::panic("Unable to open GPIO");
    }
#ifdef GPIO_SERVO_1
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_SERVO_1) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_1\n");
    }
#endif
#ifdef GPIO_SERVO_2
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_SERVO_2) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_2\n");
    }
#endif
#ifdef GPIO_SERVO_3
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_SERVO_3) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_3\n");
    }
#endif
#ifdef GPIO_SERVO_4
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_SERVO_4) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_4\n");
    }
#endif
}

void VRBRAINGPIO::pinMode(uint8_t pin, uint8_t output)
{
    switch (pin) {
    }
}

uint8_t VRBRAINGPIO::read(uint8_t pin) {
    switch (pin) {

#ifdef GPIO_SERVO_3
        case EXTERNAL_RELAY1_PIN: {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_SERVO_3)?HIGH:LOW;
        }
#endif

#ifdef GPIO_SERVO_4
        case EXTERNAL_RELAY2_PIN: {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_SERVO_4)?HIGH:LOW;
        }
#endif

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

#ifdef GPIO_SERVO_1
        case EXTERNAL_LED_GPS:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_SERVO_1);
            break;
#endif

#ifdef GPIO_SERVO_2
        case EXTERNAL_LED_ARMED:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_SERVO_2);
            break;
#endif

#ifdef GPIO_SERVO_3
        case EXTERNAL_RELAY1_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_SERVO_3);
            break;
#endif

#ifdef GPIO_SERVO_4
        case EXTERNAL_RELAY2_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_SERVO_4);
            break;
#endif

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
    /*
      we use a combination of voltage on the USB connector and the
      open of the /dev/ttyACM0 character device. This copes with
      systems where the VBUS may go high even with no USB connected
      (such as AUAV-X2)
     */
    return stm32_gpioread(GPIO_OTGFS_VBUS) && _usb_connected;
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
