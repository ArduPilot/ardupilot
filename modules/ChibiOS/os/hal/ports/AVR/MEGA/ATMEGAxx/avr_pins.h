/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef AVR_PINS_H
#define AVR_PINS_H

#include <avr/io.h>

#if AVR_SPI_USE_SPI1

#if defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega162__)
  #define PIN_SPI1            PINB
  #define PORT_SPI1           PORTB
  #define DDR_SPI1            DDRB
  #define SPI1_SS             4
  #define SPI1_SCK            7
  #define SPI1_MOSI           5
  #define SPI1_MISO           6
#elif defined(__AVR_ATmega328P__)
  #define PIN_SPI1            PINB
  #define PORT_SPI1           PORTB
  #define DDR_SPI1            DDRB
  #define SPI1_SS             2
  #define SPI1_SCK            5
  #define SPI1_MOSI           3
  #define SPI1_MISO           4
#elif defined(__AVR_ATmega2560__) || \
      defined(__AVR_ATmega1280__) || \
      defined(__AVR_ATmega128__)
  #define PIN_SPI1            PINB
  #define PORT_SPI1           PORTB
  #define DDR_SPI1            DDRB
  #define SPI1_SS             0
  #define SPI1_SCK            1
  #define SPI1_MOSI           2
  #define SPI1_MISO           3
#elif defined(__AVR_AT90CAN128__) || \
      defined(__AVR_AT90CAN64__)  || \
      defined(__AVR_AT90CAN32__)
  #define PIN_SPI1            PINB
  #define PORT_SPI1           PORTB
  #define DDR_SPI1            DDRB
  #define SPI1_SS             0
  #define SPI1_SCK            1
  #define SPI1_MOSI           2
  #define SPI1_MISO           3
#else
  #warning "Device not supported by SPI driver"
#endif

#endif /* AVR_SPI_USE_SPI1 */

#if AVR_ADC_USE_ADC1

#if defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__)
  #define PINADC              PINA
  #define PORTADC             PORTA
  #define DDRADC              DDRA
#elif defined(__AVR_ATmega328P__)
  #define PINADC              PINC
  #define PORTADC             PORTC
  #define DDRADC              DDRC
#elif defined(__AVR_ATmega2560__) || \
      defined(__AVR_ATmega1280__) || \
      defined(__AVR_ATmega128__)
  #define PINADC              PINF
  #define PORTADC             PORTF
  #define DDRADC              DDRF
#elif defined(__AVR_AT90CAN128__) || \
      defined(__AVR_AT90CAN64__)  || \
      defined(__AVR_AT90CAN32__)
  #define PINADC              PINF
  #define PORTADC             PORTF
  #define DDRADC              DDRF
#else
  #warning "Device not supported by ADC driver"
#endif

#endif /* AVR_ADC_USE_ADC1 */

#endif /* AVR_PINS_H */
