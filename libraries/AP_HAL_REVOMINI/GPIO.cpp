
#include "GPIO.h"
#include <gpio_hal.h>
#include <ext_interrupts.h>
#include <exti.h>
#include <boards.h>
#include "io.h"

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode);

using namespace REVOMINI;

REVOMINIGPIO::REVOMINIGPIO()
{
}

void REVOMINIGPIO::init()
{
}

void REVOMINIGPIO::pinMode(uint8_t pin, uint8_t output)
{
    gpio_pin_mode outputMode;
    bool pwm = false;

    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return;
    }

    switch(output) {
    case OUTPUT:
        outputMode = GPIO_OUTPUT_PP;
        break;
    case OUTPUT_OPEN_DRAIN:
        outputMode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_INPUT_FLOATING;
        break;
    case INPUT_ANALOG:
        outputMode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_INPUT_PD;
        break;
    case PWM:
        outputMode = GPIO_AF_OUTPUT_PP;
        pwm = true;
        break;
    case PWM_OPEN_DRAIN:
        outputMode = GPIO_AF_OUTPUT_OD;
        pwm = true;
        break;
    default:
        assert_param(0);
        return;
    }

    gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, outputMode);

    if (PIN_MAP[pin].timer_device != NULL) {

		if (pwm)
		{
			if (PIN_MAP[pin].timer_device->regs == TIM1)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM1);
			else if (PIN_MAP[pin].timer_device->regs == TIM2)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM2);
			else if (PIN_MAP[pin].timer_device->regs == TIM3)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM3);
			else if (PIN_MAP[pin].timer_device->regs == TIM4)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM4);
			else if (PIN_MAP[pin].timer_device->regs == TIM5)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM5);
			else if (PIN_MAP[pin].timer_device->regs == TIM8)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM8);
			else if (PIN_MAP[pin].timer_device->regs == TIM13)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM13);
			else if (PIN_MAP[pin].timer_device->regs == TIM14)
				GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, GPIO_AF_TIM14);

			timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, TIMER_PWM);
		}
        /* Enable/disable timer channels if we're switching into or out of PWM. */
        //timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, pwm ? TIMER_PWM : TIMER_DISABLED);
    }

}

int8_t  REVOMINIGPIO::analogPinToDigitalPin(uint8_t pin){
    return pin;
}

uint8_t REVOMINIGPIO::read(uint8_t pin)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return 0;
    }

    return gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit) ?
        HIGH : LOW;
}

void REVOMINIGPIO::write(uint8_t pin, uint8_t value)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return;
    }

    gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, value);

}

void REVOMINIGPIO::toggle(uint8_t pin)
{
    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return;
    }
    gpio_toggle_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit);

}



/* Alternative interface: */
AP_HAL::DigitalSource* REVOMINIGPIO::channel(uint16_t pin) {

    if ((pin < 0) || (pin >= BOARD_NR_GPIO_PINS)) {
        return NULL;
    }

    uint8_t bit;
    gpio_dev *device;
    bit = PIN_MAP[pin].gpio_bit;
    device = PIN_MAP[pin].gpio_device;

    return new REVOMINIDigitalSource(device, bit);
}

/* Interrupt interface: */
bool REVOMINIGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                  uint8_t mode)
{
    if ((interrupt_num < 0) || (interrupt_num >= BOARD_NR_GPIO_PINS) || !p) {
        return false;
    }

    exti_trigger_mode outMode = exti_out_mode((ExtIntTriggerMode)mode);

    exti_attach_interrupt((afio_exti_num)(PIN_MAP[interrupt_num].gpio_bit),
                          gpio_exti_port(PIN_MAP[interrupt_num].gpio_device),
                          p,
                          outMode);

    return true;
}

bool REVOMINIGPIO::usb_connected(void)
{
    return gpio_read_bit(_GPIOC,5);
}

void REVOMINIDigitalSource::mode(uint8_t output)
{
    gpio_pin_mode outputMode;

       switch(output) {
       case OUTPUT:
           outputMode = GPIO_OUTPUT_PP;
           break;
       case OUTPUT_OPEN_DRAIN:
           outputMode = GPIO_OUTPUT_OD;
           break;
       case INPUT:
       case INPUT_FLOATING:
           outputMode = GPIO_INPUT_FLOATING;
           break;
       case INPUT_ANALOG:
           outputMode = GPIO_INPUT_ANALOG;
           break;
       case INPUT_PULLUP:
           outputMode = GPIO_INPUT_PU;
           break;
       case INPUT_PULLDOWN:
           outputMode = GPIO_INPUT_PD;
           break;
       default:
           assert_param(0);
           return;
       }
    gpio_set_mode(_device, _bit, outputMode);
}

uint8_t REVOMINIDigitalSource::read()
{
    return gpio_read_bit(_device, _bit) ?
        HIGH : LOW;
}

void REVOMINIDigitalSource::write(uint8_t value)
{
    gpio_write_bit(_device, _bit, value);
}

void REVOMINIDigitalSource::toggle()
{
    gpio_write_bit(_device, _bit, !gpio_read_bit(_device, _bit));
}

static inline exti_trigger_mode exti_out_mode(ExtIntTriggerMode mode) {
    switch (mode) {
    case RISING:
        return EXTI_RISING;
    case FALLING:
        return EXTI_FALLING;
    case CHANGE:
        return EXTI_RISING_FALLING;
    }
    // Can't happen
    assert_param(0);
    return (exti_trigger_mode)0;
}
