#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "GPIO.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* PX4 headers */
#include <drivers/drv_led.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>
#include <arch/board/board.h>
#include <board_config.h>

#define LOW     0
#define HIGH    1

extern const AP_HAL::HAL& hal;

using namespace PX4;

PX4GPIO::PX4GPIO()
{}

void PX4GPIO::init()
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
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
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V4
    if (ioctl(_led_fd, LED_OFF, LED_GREEN) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO LED GREEN\n");
    }
#endif
#endif
#if !defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    _gpio_fmu_fd = open(PX4FMU_DEVICE_PATH, 0);
    if (_gpio_fmu_fd == -1) {
        AP_HAL::panic("Unable to open GPIO");
    }
#endif
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    if (ioctl(_gpio_fmu_fd, GPIO_CLEAR, GPIO_EXT_1) != 0) {
        hal.console->printf("GPIO: Unable to setup GPIO_1\n");
    }
#endif

#ifdef PX4IO_DEVICE_PATH
    // also try to setup for the relay pins on the IO board
    _gpio_io_fd = open(PX4IO_DEVICE_PATH, O_RDWR);
    if (_gpio_io_fd == -1) {
        hal.console->printf("GPIO: Unable to open px4io\n");
    }
#endif
}

void PX4GPIO::pinMode(uint8_t pin, uint8_t output)
{
    switch (pin) {
    case PX4_GPIO_FMU_SERVO_PIN(0) ... PX4_GPIO_FMU_SERVO_PIN(5):
        uint32_t pinmask = 1U<<(pin-PX4_GPIO_FMU_SERVO_PIN(0));
        if (output) {
            uint8_t old_value = read(pin);
            if (old_value) {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_HIGH, pinmask);
            } else {
                ioctl(_gpio_fmu_fd, GPIO_SET_OUTPUT_LOW, pinmask);
            }
        } else {
            ioctl(_gpio_fmu_fd, GPIO_SET_INPUT, pinmask);
        }
        break;
    }
}

uint8_t PX4GPIO::read(uint8_t pin) {
    switch (pin) {

#ifdef GPIO_EXT_1
        case PX4_GPIO_EXT_FMU_RELAY1_PIN: {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_EXT_1)?HIGH:LOW;
        }
#endif

#ifdef GPIO_EXT_2
        case PX4_GPIO_EXT_FMU_RELAY2_PIN: {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & GPIO_EXT_2)?HIGH:LOW;
        }
#endif

#ifdef PX4IO_P_SETUP_RELAYS_POWER1
        case PX4_GPIO_EXT_IO_RELAY1_PIN: {
            uint32_t relays = 0;
            if (_gpio_io_fd == -1) {
                return LOW;
            }
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & PX4IO_P_SETUP_RELAYS_POWER1)?HIGH:LOW;
        }
#endif

#ifdef PX4IO_P_SETUP_RELAYS_POWER2
        case PX4_GPIO_EXT_IO_RELAY2_PIN: {
            uint32_t relays = 0;
            if (_gpio_io_fd == -1) {
                return LOW;
            }
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & PX4IO_P_SETUP_RELAYS_POWER2)?HIGH:LOW;
        }
#endif

#ifdef PX4IO_P_SETUP_RELAYS_ACC1
        case PX4_GPIO_EXT_IO_ACC1_PIN: {
            uint32_t relays = 0;
            if (_gpio_io_fd == -1) {
                return LOW;
            }
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & PX4IO_P_SETUP_RELAYS_ACC1)?HIGH:LOW;
        }
#endif

#ifdef PX4IO_P_SETUP_RELAYS_ACC2
        case PX4_GPIO_EXT_IO_ACC2_PIN: {
            uint32_t relays = 0;
            if (_gpio_io_fd == -1) {
                return LOW;
            }
            ioctl(_gpio_io_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & PX4IO_P_SETUP_RELAYS_ACC2)?HIGH:LOW;
        }
#endif

    case PX4_GPIO_FMU_SERVO_PIN(0) ... PX4_GPIO_FMU_SERVO_PIN(5): {
            uint32_t relays = 0;
            ioctl(_gpio_fmu_fd, GPIO_GET, (unsigned long)&relays);
            return (relays & (1U<<(pin-PX4_GPIO_FMU_SERVO_PIN(0))))?HIGH:LOW;
        }
    }
    return LOW;
}

void PX4GPIO::write(uint8_t pin, uint8_t value)
{
    switch (pin) {

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
        case HAL_GPIO_A_LED_PIN:    // Arming LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_RED);
            } else {
                ioctl(_led_fd, LED_ON, LED_RED);
            }
            break;

        case HAL_GPIO_B_LED_PIN:    // Green LED
            if (value == LOW) {
                ioctl(_led_fd, LED_OFF, LED_GREEN);
            } else {
                ioctl(_led_fd, LED_ON, LED_GREEN);
            }
            break;

        case HAL_GPIO_C_LED_PIN:    // GPS LED 
            if (value == LOW) { 
                ioctl(_led_fd, LED_OFF, LED_BLUE);
            } else { 
                ioctl(_led_fd, LED_ON, LED_BLUE);
            }
            break;
#endif

#ifdef GPIO_EXT_1
        case PX4_GPIO_EXT_FMU_RELAY1_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_EXT_1);
            break;
#endif

#ifdef GPIO_EXT_2
        case PX4_GPIO_EXT_FMU_RELAY2_PIN:
            ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, GPIO_EXT_2);
            break;
#endif

#ifdef PX4IO_P_SETUP_RELAYS_POWER1
        case PX4_GPIO_EXT_IO_RELAY1_PIN:
            if (_gpio_io_fd != -1) {
                ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, PX4IO_P_SETUP_RELAYS_POWER1);
            }
            break;
#endif

#ifdef PX4IO_P_SETUP_RELAYS_POWER2
        case PX4_GPIO_EXT_IO_RELAY2_PIN:
            if (_gpio_io_fd != -1) {
                ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, PX4IO_P_SETUP_RELAYS_POWER2);
            }
            break;
#endif

#ifdef PX4IO_P_SETUP_RELAYS_ACC1
        case PX4_GPIO_EXT_IO_ACC1_PIN:
            if (_gpio_io_fd != -1) {
                ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, PX4IO_P_SETUP_RELAYS_ACC1);
            }
            break;
#endif

#ifdef PX4IO_P_SETUP_RELAYS_ACC2
        case PX4_GPIO_EXT_IO_ACC2_PIN:
            if (_gpio_io_fd != -1) {
                ioctl(_gpio_io_fd, value==LOW?GPIO_CLEAR:GPIO_SET, PX4IO_P_SETUP_RELAYS_ACC2);
            }
            break;
#endif

    case PX4_GPIO_FMU_SERVO_PIN(0) ... PX4_GPIO_FMU_SERVO_PIN(5):
        ioctl(_gpio_fmu_fd, value==LOW?GPIO_CLEAR:GPIO_SET, 1U<<(pin-PX4_GPIO_FMU_SERVO_PIN(0)));
        break;
    }
}

void PX4GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
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

/*
  return true when USB connected
 */
bool PX4GPIO::usb_connected(void)
{
    /*
      we use a combination of voltage on the USB connector and the
      open of the /dev/ttyACM0 character device. This copes with
      systems where the VBUS may go high even with no USB connected
      (such as AUAV-X2)
     */
    return stm32_gpioread(GPIO_OTGFS_VBUS) && _usb_connected;
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

void PX4DigitalSource::toggle() {
    _v = !_v;
}

#endif // CONFIG_HAL_BOARD
