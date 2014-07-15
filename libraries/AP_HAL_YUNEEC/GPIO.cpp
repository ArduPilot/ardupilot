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
  Flymaple port by Mike McCauley
 */
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "GPIO.h"
#include "FlymapleWirish.h"

using namespace YUNEEC;

// Glue into libmaple
void libmaple_pinMode(uint8_t pin, uint8_t output)
{
    WiringPinMode mode = output ? OUTPUT : INPUT;
    pinMode(pin, mode);
}

uint8_t libmaple_digitalRead(uint8_t pin) 
{
    return digitalRead(pin);
}
void libmaple_digitalWrite(uint8_t pin, uint8_t value)
{
    digitalWrite(pin, value);
}

YUNEECGPIO::YUNEECGPIO()
{}

void YUNEECGPIO::init()
{}

void YUNEECGPIO::pinMode(uint8_t pin, uint8_t output)
{
    return libmaple_pinMode(pin, output);
}

int8_t YUNEECGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return pin;
}


uint8_t YUNEECGPIO::read(uint8_t pin)
{
    return libmaple_digitalRead(pin);
}

void YUNEECGPIO::write(uint8_t pin, uint8_t value)
{
    libmaple_digitalWrite(pin, value);
}

void YUNEECGPIO::toggle(uint8_t pin)
{
    libmaple_digitalWrite(pin, !libmaple_digitalRead(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* YUNEECGPIO::channel(uint16_t n) {
    return new YUNEECDigitalSource(n);
}

/* Interrupt interface: */
bool YUNEECGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    // YUNEEC can only handle RISING, FALLING and CHANGE
    ExtIntTriggerMode YUNEEC_interrupt_mode;
    if (mode == HAL_GPIO_INTERRUPT_FALLING)
	YUNEEC_interrupt_mode = FALLING;
    else if (mode == HAL_GPIO_INTERRUPT_RISING)
	YUNEEC_interrupt_mode = RISING;
    else 
	return false;
	
    // REVISIT: Assumes pin and interrupt number are the same. Are they? 
    attachInterrupt(interrupt_num, p, YUNEEC_interrupt_mode);
    return true;
}

bool    YUNEECGPIO::usb_connected(void)
{
    return SerialUSB.isConnected();
}

YUNEECDigitalSource::YUNEECDigitalSource(uint8_t v) :
    _v(v)
{
    SerialUSB.println(_v);
}

void YUNEECDigitalSource::mode(uint8_t output)
{
    libmaple_pinMode(_v, output);
}

uint8_t YUNEECDigitalSource::read() {
    return libmaple_digitalRead(_v);
}

void YUNEECDigitalSource::write(uint8_t value) {
    libmaple_digitalWrite(_v, value);
}

void YUNEECDigitalSource::toggle() {
    libmaple_digitalWrite(_v, !libmaple_digitalRead(_v));
}
#endif
