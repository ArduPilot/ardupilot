#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "GPIO.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <utility/pinmap_typedef.h>

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
	else{
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
{
	return pin;
}

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

	if(value == 0)
		gpio_clear(port, bit);
	else
		gpio_set(port, bit);

	return;
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
	if(!have_this_pin(n)) return NULL;

	uint32_t port = get_port(n);
	uint16_t bit = get_bit(n);

	return new YUNEECDigitalSource(port, bit);
}

/* Interrupt interface: */
bool YUNEECGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
	return false;
}

bool    YUNEECGPIO::usb_connected(void)
{
	return false;
}

void YUNEECDigitalSource::mode(uint8_t output)
{
	setPinMode(_port, _bit, output);
}

uint8_t YUNEECDigitalSource::read() {
    return (gpio_port_read(_port) & _bit) ? 1 : 0;
}

void YUNEECDigitalSource::write(uint8_t value) {
	if(value == 0)
		gpio_clear(_port, _bit);
	else
		gpio_set(_port, _bit);
}

void YUNEECDigitalSource::toggle() {
    gpio_toggle(_port, _bit);
}

#endif

