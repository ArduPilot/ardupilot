#ifndef _PINMAP_TYPEDEF_H_
#define _PINMAP_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stm32f37x.h>

enum GPIOS  {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
			 PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
			 PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
			 PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
			 PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
			 PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15};

uint8_t have_this_pin(uint8_t pin);

GPIO_TypeDef * get_port(uint8_t pin);

uint16_t get_bit(uint8_t pin);

uint8_t get_adc_channel(uint8_t pin);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
