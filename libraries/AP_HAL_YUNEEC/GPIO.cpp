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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <utility/pipmap_typedef.h>

using namespace YUNEEC;

void setPinMode(const uint32_t port, const uint16_t bit, const uint8_t output)
{
	enum rcc_periph_clken clken;
	switch(port){
	case GPIOA:
		clken = RCC_GPIOA;
		break;
	case GPIOB:
		clken = RCC_GPIOB;
		break;
	case GPIOC:
		clken = RCC_GPIOC;
		break;
	case GPIOD:
		clken = RCC_GPIOD;
		break;
	case GPIOE:
		clken = RCC_GPIOE;
		break;
	case GPIOF:
		clken = RCC_GPIOF;
		break;
	default:
		return;
		break;
	}

	rcc_periph_clock_enable(clken);

	if(output == HAL_GPIO_INPUT){
		gpio_mode_setup(port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, bit);
		return;
	}
	else
		gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, bit);
		gpio_set_output_options(port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, bit);
		return;
	}
}

YUNEECGPIO::YUNEECGPIO()
{}

void YUNEECGPIO::init()
{}

void YUNEECGPIO::pinMode(uint8_t pin, uint8_t output)
{
	if(!have_this_pin(pin)) return;

	uint32_t port = get_port(pin);
	uint16_t bit = get_bit(pin);


	setPinMode(port, bit, output);
}

int8_t YUNEECGPIO::analogPinToDigitalPin(uint8_t pin)
{}

uint8_t YUNEECGPIO::read(uint8_t pin)
{
	if(!have_this_pin(pin)) return 0;

	uint32_t port = get_port(pin);
	uint16_t bit = get_bit(pin);

    return (gpio_port_read(port) & bit) ? 1 : 0;
}

void YUNEECGPIO::write(uint8_t pin, uint8_t value)
{
	if(!have_this_pin(pin)) return;

	uint32_t port = get_port(pin);
	uint16_t bit = get_bit(pin);

	gpio_port_write(port, (1U << bit));
}

void YUNEECGPIO::toggle(uint8_t pin)
{
	if(!have_this_pin(pin)) return;

	uint32_t port = get_port(pin);
	uint16_t bit = get_bit(pin);

    gpio_toggle(port, bit);
}

/* Alternative interface: */
AP_HAL::DigitalSource* YUNEECGPIO::channel(uint16_t n) {
	if(!have_this_pin(pin)) return NULL;

	uint32_t port = get_port(pin);
	uint16_t bit = get_bit(pin);

	return new YUNEECDigitalSource(port, bit);
}

/* Interrupt interface: */
bool YUNEECGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
}

bool    YUNEECGPIO::usb_connected(void)
{
}

void YUNEECDigitalSource::mode(uint8_t output)
{
	setPinMode(_port, _bit, output);
}

uint8_t YUNEECDigitalSource::read() {
    return (gpio_port_read(_port) & _bit) ? 1 : 0;
}

void YUNEECDigitalSource::write(uint8_t value) {
	gpio_port_write(_port, (1U << _bit));
}

void YUNEECDigitalSource::toggle() {
    gpio_toggle(_port, _bit);
}
#endif
