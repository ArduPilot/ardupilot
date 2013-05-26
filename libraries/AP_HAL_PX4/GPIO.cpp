#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "GPIO.h"

#include "config.h"

#include <sys/types.h>
#include <sys/stat.h>
//#include <sys/ioctrl.h>
#include <fcntl.h>
#include <unistd.h>
//#include <sdtint.h>

#include <drivers/drv_led.h>


extern const AP_HAL::HAL& hal;

using namespace PX4;

PX4GPIO::PX4GPIO()
{}

void PX4GPIO::init()
{
	_led_fd = open(LED_DEVICE_PATH, O_RDWR);
	if (_led_fd == -1) {
		hal.scheduler->panic("Unable to open " LED_DEVICE_PATH);
	}
	
	if (ioctl(_led_fd, LED_OFF, LED_BLUE) != 0) {
        	hal.console->printf("GPIO: Unable to setup GPIO LED RED\n");
    	}

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

	switch (pin) {
		case A_LED_PIN:
			// LED A arming
			if (value == 0) {
				ioctl(_led_fd, LED_OFF, LED_RED);
			} else {
				ioctl(_led_fd, LED_ON, LED_RED);
			}
			break;

		case B_LED_PIN:
			// LED B
			break;

		case C_LED_PIN:
			// LED C gps
                        if (value == 0) { 
                                ioctl(_led_fd, LED_OFF, LED_BLUE);
                        } else { 
                                ioctl(_led_fd, LED_ON, LED_BLUE);
                        }
			break;
	}

}

/* Alternative interface: */
AP_HAL::DigitalSource* PX4GPIO::channel(uint16_t n) {
    return new PX4DigitalSource(0);
}

/* Interrupt interface: */
bool PX4GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
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

#endif

