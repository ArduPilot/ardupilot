#include "pinmap_typedef.h"

#include <stm32f37x.h>

enum HAVE_PINS {NONE = 0, HAVE = 1};

#define PINS_NUM	96

const static uint8_t PIN_MAP[PINS_NUM] = {
		HAVE/*PA0*/, HAVE/*PA1*/, HAVE/*PA2*/, HAVE/*PA3*/, HAVE/*PA4*/, HAVE/*PA5*/, HAVE/*PA6*/, HAVE/*PA7*/, HAVE/*PA8*/, HAVE/*PA9*/, HAVE/*PA10*/, HAVE/*PA11*/, HAVE/*PA12*/, HAVE/*PA13*/, HAVE/*PA14*/, HAVE/*PA15*/,
		HAVE/*PB0*/, HAVE/*PB1*/, HAVE/*PB2*/, HAVE/*PB3*/, HAVE/*PB4*/, HAVE/*PB5*/, HAVE/*PB6*/, HAVE/*PB7*/, HAVE/*PB8*/, HAVE/*PB9*/, NONE/*PB10*/, NONE/*PB11*/, NONE/*PB12*/, NONE/*PB13*/, HAVE/*PB14*/, HAVE/*PB15*/,
		HAVE/*PC0*/, HAVE/*PC1*/, HAVE/*PC2*/, HAVE/*PC3*/, HAVE/*PC4*/, HAVE/*PC5*/, HAVE/*PC6*/, HAVE/*PC7*/, HAVE/*PC8*/, HAVE/*PC9*/, HAVE/*PC10*/, HAVE/*PC11*/, HAVE/*PC12*/, HAVE/*PC13*/, HAVE/*PC14*/, HAVE/*PC15*/,
		NONE/*PD0*/, NONE/*PD1*/, HAVE/*PD2*/, NONE/*PD3*/, NONE/*PD4*/, NONE/*PD5*/, NONE/*PD6*/, NONE/*PD7*/, HAVE/*PD8*/, NONE/*PD9*/, NONE/*PD10*/, NONE/*PD11*/, NONE/*PD12*/, NONE/*PD13*/, NONE/*PD14*/, NONE/*PD15*/,
		NONE/*PE0*/, NONE/*PE1*/, NONE/*PE2*/, NONE/*PE3*/, NONE/*PE4*/, NONE/*PE5*/, NONE/*PE6*/, NONE/*PE7*/, HAVE/*PE8*/, HAVE/*PE9*/, NONE/*PE10*/, NONE/*PE11*/, NONE/*PE12*/, NONE/*PE13*/, NONE/*PE14*/, NONE/*PE15*/,
		HAVE/*PF0*/, HAVE/*PF1*/, NONE/*PF2*/, NONE/*PF3*/, NONE/*PF4*/, NONE/*PF5*/, HAVE/*PF6*/, HAVE/*PF7*/, NONE/*PF8*/, NONE/*PF9*/, NONE/*PF10*/, NONE/*PF11*/, NONE/*PF12*/, NONE/*PF13*/, NONE/*PF14*/, NONE/*PF15*/
};

uint8_t have_this_pin(uint8_t pin) {
	if(pin >= PINS_NUM)
		return NONE;
	else
		return PIN_MAP[pin];
}

GPIO_TypeDef * get_port(uint8_t pin) {
	if(!have_this_pin(pin)) return 0;

	switch(pin){
		case PA0 ... PA15: return GPIOA;
			break;
		case PB0 ... PB15: return GPIOB;
			break;
		case PC0 ... PC15: return GPIOC;
			break;
		case PD0 ... PD15: return GPIOD;
			break;
		case PE0 ... PE15: return GPIOE;
			break;
		case PF0 ... PF15: return GPIOF;
			break;
		default: return 0;
			break;
	}
}

uint16_t get_bit(uint8_t pin) {
	if(!have_this_pin(pin))
		return 0;
	else
		return (uint16_t)(1 << (pin % 16));
}

uint8_t get_adc_channel(uint8_t pin) {
	if(!have_this_pin(pin))
		return 0xff;

	switch(pin){
	case PA0: return ADC_Channel_0;
	case PA1: return ADC_Channel_1;
	case PA2: return ADC_Channel_2;
	case PA3: return ADC_Channel_3;
	case PA4: return ADC_Channel_4;
	case PA5: return ADC_Channel_5;
	case PA6: return ADC_Channel_6;
	case PA7: return ADC_Channel_7;
	case PB0: return ADC_Channel_8;
	case PB1: return ADC_Channel_9;
	case PC0: return ADC_Channel_10;
	case PC1: return ADC_Channel_11;
	case PC2: return ADC_Channel_12;
	case PC3: return ADC_Channel_13;
	case PC4: return ADC_Channel_14;
	case PC5: return ADC_Channel_15;
	default: return 0xff;
	}
}
