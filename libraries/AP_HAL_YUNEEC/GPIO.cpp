/*
  YUNEEC port by Maelok
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "GPIO.h"
#include <utility/pinmap_typedef.h>

#include <stm32f37x.h>
#include <stm32f37x_rcc.h>
#include <stm32f37x_gpio.h>

using namespace YUNEEC;

void setPinMode(GPIO_TypeDef* port, uint16_t bit, uint8_t output) {
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t clken;

	if(port == GPIOA)
		clken = RCC_AHBPeriph_GPIOA;
	else if(port == GPIOB)
		clken = RCC_AHBPeriph_GPIOB;
	else if(port == GPIOC)
		clken = RCC_AHBPeriph_GPIOC;
	else if(port == GPIOD)
		clken = RCC_AHBPeriph_GPIOD;
	else if(port == GPIOE)
		clken = RCC_AHBPeriph_GPIOE;
	else if(port == GPIOF)
		clken = RCC_AHBPeriph_GPIOF;
	else
		return;

    RCC->AHBENR |= clken;

	if (output == HAL_GPIO_INPUT)
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_Pin = bit;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(port, &GPIO_InitStructure);
}
YUNEECGPIO::YUNEECGPIO()
{}

void YUNEECGPIO::init()
{}

void YUNEECGPIO::pinMode(uint8_t pin, uint8_t output)
{
	if(!have_this_pin(pin)) return;

	GPIO_TypeDef* port = get_port(pin);
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

	GPIO_TypeDef* port = get_port(pin);
	uint16_t bit = get_bit(pin);

	return (port->IDR & bit);
}

void YUNEECGPIO::write(uint8_t pin, uint8_t value)
{
	if(!have_this_pin(pin)) return;

	GPIO_TypeDef* port = get_port(pin);
	uint16_t bit = get_bit(pin);

	if(value == 0)
		port->BRR = bit;
	else
		port->BSRR = bit;
}

void YUNEECGPIO::toggle(uint8_t pin)
{
	if(!have_this_pin(pin)) return;

	GPIO_TypeDef* port = get_port(pin);
	uint16_t bit = get_bit(pin);

	port->ODR ^= bit;
}

/* Alternative interface: */
AP_HAL::DigitalSource* YUNEECGPIO::channel(uint16_t n) {
	if(!have_this_pin(n)) return NULL;

	GPIO_TypeDef* port = get_port(n);
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
	return (_port->IDR & _bit);
}

void YUNEECDigitalSource::write(uint8_t value) {
	if(value == 0)
		_port->BRR = _bit;
	else
		_port->BSRR = _bit;
}

void YUNEECDigitalSource::toggle() {
	_port->ODR ^= _bit;
}

#endif

