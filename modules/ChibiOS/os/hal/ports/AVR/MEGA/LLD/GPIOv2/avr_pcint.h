/*
    ChibiOS - Copyright (C) 2025 Nathan Lewis

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

/**
 * @file  GPIOv2/avr_pcint.h
 * @brief AVR/MEGA Pin Change Interrupts units common header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef AVR_PCINT_H
#define AVR_PCINT_H

#include "avr_pins.h"

/**
 * @brief   PCINT0 support enable switch.
 * @details If set to @p TRUE support for events on PCINT0..7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_PCINT0)
  #define AVR_GPIO_USE_PCINT0             FALSE
#endif

/**
 * @brief   PCINT1 support enable switch.
 * @details If set to @p TRUE support for events on PCINT8..15 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_PCINT1)
  #define AVR_GPIO_USE_PCINT1             FALSE
#endif

/**
 * @brief   PCINT2 support enable switch.
 * @details If set to @p TRUE support for events on PCINT16..23 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_PCINT2)
  #define AVR_GPIO_USE_PCINT2             FALSE
#endif

/**
 * @brief   PCINT3 support enable switch.
 * @details If set to @p TRUE support for events on PCINT24..31 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_PCINT3)
  #define AVR_GPIO_USE_PCINT3             FALSE
#endif

/**
 * @brief   PCINT4 support enable switch.
 * @details If set to @p TRUE support for events on PCINT32..39 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_PCINT4)
  #define AVR_GPIO_USE_PCINT4             FALSE
#endif


/**
 * For each PCINTx bank, the following macros should be defined:
 *   - IOPORTn_PCMSK           - PCINT mask register for the port
 *   - IOPORTn_PCINT           - ID of PCINT bank that corresponds to this port
 *                               should be equal to "x"
 *   - IOPORTn_PCINT_OFFSET    - macro to convert a pad in the bank to its bit
 *                               in the mask register
 *   - IOPORTn_PCINT_PAD_VALID - macro that returns TRUE if pad has a
 *                               corresponding PCINT signal
 *
 *   NOTE: more than one of the above can be defined per PCINTx bank if it
 *         spans more than one IO port (e.g. Bank 1 on ATmega2560, ATmega32U2)
 *
 *   - PCINTx_ENABLE - macro to enable the PCINTx bank
 *   - PCINTx_GET    - macro to collect the state of the PCINT signals
 *   - PCINTx_COUNT  - number of PCINT signals in the bank
 */

#if AVR_GPIO_USE_PCINT0

  #if defined(__AVR_ATmega164__) || defined(__AVR_ATmega164P__) || \
      defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164PA__) || \
      defined(__AVR_ATmega324__) || defined(__AVR_ATmega324P__) || \
      defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324PA__) || \
      defined(__AVR_ATmega324PB__) || \
      defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644PA__) || \
      defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || \
      defined(__AVR_ATmega406__)

    #define IOPORT1_PCMSK                 PCMSK0
    #define IOPORT1_PCINT                 0
    #define IOPORT1_PCINT_OFFSET(pad)     (pad)
    #define IOPORT1_PCINT_PAD_VALID(pad)  (true)

    #define PCINT0_ENABLE()               PCICR |= (1U << PCIE0)
    #define PCINT0_GET()                  (PINA)
    #define PCINT0_COUNT()                (8U)

  #elif defined(__AVR_ATmega162__)

    #define IOPORT1_PCMSK                 PCMSK0
    #define IOPORT1_PCINT                 0
    #define IOPORT1_PCINT_OFFSET(pad)     (pad)
    #define IOPORT1_PCINT_PAD_VALID(pad)  (true)

    #define PCINT0_ENABLE()               GICR |= (1U << PCIE0)
    #define PCINT0_GET()                  (PINA)
    #define PCINT0_COUNT()                (8U)

  #elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48P__) || \
      defined(__AVR_ATmega48A__) || defined(__AVR_ATmega48PA__) || \
      defined(__AVR_ATmega48PB__) || \
      defined(__AVR_ATmega88__) || defined(__AVR_ATmega88P__) || \
      defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || \
      defined(__AVR_ATmega88PB__) || \
      defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega168A__) || defined(__AVR_ATmega168PA__) || \
      defined(__AVR_ATmega168PB) || \
      defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
      defined(__AVR_ATmega328PB__) || \
      defined(__AVR_ATmega640__) || \
      defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || \
      defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || \
      defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) || \
      defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
      defined(__AVR_ATmega32U2__) || \
      defined(__AVR_ATmega16C1__) || defined(__AVR_ATmega16M1__) || \
      defined(__AVR_ATmega32C1__) || defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64C1__) || defined(__AVR_ATmega64M1__) || \
      defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
      defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

    #define IOPORT2_PCMSK                 PCMSK0
    #define IOPORT2_PCINT                 0
    #define IOPORT2_PCINT_OFFSET(pad)     (pad)
    #define IOPORT2_PCINT_PAD_VALID(pad)  (true)

    #define PCINT0_ENABLE()               PCICR |= (1U << PCIE0)
    #define PCINT0_GET()                  (PINB)
    #define PCINT0_COUNT()                (8U)

  #elif defined(__AVR_ATmega169__) || defined(__AVR_ATmega169P__) || \
      defined(__AVR_ATmega169A__) || defined(__AVR_ATmega169PA__) || \
      defined(__AVR_ATmega325__) || defined(__AVR_ATmega325P__) || \
      defined(__AVR_ATmega325A__) || defined(__AVR_ATmega325PA__) || \
      defined(__AVR_ATmega329__) || defined(__AVR_ATmega329P__) || \
      defined(__AVR_ATmega329A__) || defined(__AVR_ATmega329PA__) || \
      defined(__AVR_ATmega645__) || defined(__AVR_ATmega645P__) || \
      defined(__AVR_ATmega645A__) || defined(__AVR_ATmega645PA__) || \
      defined(__AVR_ATmega649__) || defined(__AVR_ATmega649P__) || \
      defined(__AVR_ATmega649A__) || defined(__AVR_ATmega649PA__) || \
      defined(__AVR_ATmega3250__) || defined(__AVR_ATmega3250P__) || \
      defined(__AVR_ATmega3250A__) || defined(__AVR_ATmega3250PA__) || \
      defined(__AVR_ATmega3290__) || defined(__AVR_ATmega3290P__) || \
      defined(__AVR_ATmega3290A__) || defined(__AVR_ATmega3290PA__) || \
      defined(__AVR_ATmega6450__) || defined(__AVR_ATmega6450P__) || \
      defined(__AVR_ATmega6450A__) || defined(__AVR_ATmega6450PA__) || \
      defined(__AVR_ATmega6490__) || defined(__AVR_ATmega6490P__) || \
      defined(__AVR_ATmega6490A__) || defined(__AVR_ATmega6490PA__)

    #define IOPORT5_PCMSK                 PCMSK0
    #define IOPORT5_PCINT                 0
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  (true)

    #define PCINT0_ENABLE()               EIMSK |= (1U << PCIE0)
    #define PCINT0_GET()                  (PINE)
    #define PCINT0_COUNT()                (8U)

  #else
    #error "PCINT0 is not supported on this ATmega device"
  #endif
#endif /* AVR_GPIO_USE_PCINT0 */

#if AVR_GPIO_USE_PCINT1
  #if defined(__AVR_ATmega164__) || defined(__AVR_ATmega164P__) || \
      defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164PA__) || \
      defined(__AVR_ATmega324__) || defined(__AVR_ATmega324P__) || \
      defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324PA__) || \
      defined(__AVR_ATmega324PB__) || \
      defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644PA__) || \
      defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || \
      defined(__AVR_ATmega406__)

    #define IOPORT2_PCMSK                 PCMSK1
    #define IOPORT2_PCINT                 1
    #define IOPORT2_PCINT_OFFSET(pad)     (pad)
    #define IOPORT2_PCINT_PAD_VALID(pad)  (true)

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINB)
    #define PCINT1_COUNT()                (8U)

  #elif defined(__AVR_ATmega162__)

    #define IOPORT2_PCMSK                 PCMSK1
    #define IOPORT2_PCINT                 1
    #define IOPORT2_PCINT_OFFSET(pad)     (pad)
    #define IOPORT2_PCINT_PAD_VALID(pad)  (true)

    #define PCINT1_ENABLE()               GICR |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINB)
    #define PCINT1_COUNT()                (8U)

  #elif defined(__AVR_ATmega169__) || defined(__AVR_ATmega169P__) || \
      defined(__AVR_ATmega169A__) || defined(__AVR_ATmega169PA__) || \
      defined(__AVR_ATmega325__) || defined(__AVR_ATmega325P__) || \
      defined(__AVR_ATmega325A__) || defined(__AVR_ATmega325PA__) || \
      defined(__AVR_ATmega329__) || defined(__AVR_ATmega329P__) || \
      defined(__AVR_ATmega329A__) || defined(__AVR_ATmega329PA__) || \
      defined(__AVR_ATmega645__) || defined(__AVR_ATmega645P__) || \
      defined(__AVR_ATmega645A__) || defined(__AVR_ATmega645PA__) || \
      defined(__AVR_ATmega649__) || defined(__AVR_ATmega649P__) || \
      defined(__AVR_ATmega649A__) || defined(__AVR_ATmega649PA__) || \
      defined(__AVR_ATmega3250__) || defined(__AVR_ATmega3250P__) || \
      defined(__AVR_ATmega3250A__) || defined(__AVR_ATmega3250PA__) || \
      defined(__AVR_ATmega3290__) || defined(__AVR_ATmega3290P__) || \
      defined(__AVR_ATmega3290A__) || defined(__AVR_ATmega3290PA__) || \
      defined(__AVR_ATmega6450__) || defined(__AVR_ATmega6450P__) || \
      defined(__AVR_ATmega6450A__) || defined(__AVR_ATmega6450PA__) || \
      defined(__AVR_ATmega6490__) || defined(__AVR_ATmega6490P__) || \
      defined(__AVR_ATmega6490A__) || defined(__AVR_ATmega6490PA__)

    #define IOPORT2_PCMSK                 PCMSK1
    #define IOPORT2_PCINT                 1
    #define IOPORT2_PCINT_OFFSET(pad)     (pad)
    #define IOPORT2_PCINT_PAD_VALID(pad)  (true)

    #define PCINT1_ENABLE()               EIMSK |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINB)
    #define PCINT1_COUNT()                (8U)

  #elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48P__) || \
      defined(__AVR_ATmega48A__) || defined(__AVR_ATmega48PA__) || \
      defined(__AVR_ATmega48PB__) || \
      defined(__AVR_ATmega88__) || defined(__AVR_ATmega88P__) || \
      defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || \
      defined(__AVR_ATmega88PB__) || \
      defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega168A__) || defined(__AVR_ATmega168PA__) || \
      defined(__AVR_ATmega168PB) || \
      defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
      defined(__AVR_ATmega328PB__)

    #define IOPORT3_PCMSK                 PCMSK1
    #define IOPORT3_PCINT                 1
    #define IOPORT3_PCINT_OFFSET(pad)     (pad)
    #define IOPORT3_PCINT_PAD_VALID(pad)  (true)

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINC)
    #define PCINT1_COUNT()                (7U)

  #elif defined(__AVR_ATmega16C1__) || defined(__AVR_ATmega16M1__) || \
      defined(__AVR_ATmega32C1__) || defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64C1__) || defined(__AVR_ATmega64M1__)

    #define IOPORT3_PCMSK                 PCMSK1
    #define IOPORT3_PCINT                 1
    #define IOPORT3_PCINT_OFFSET(pad)     (pad)
    #define IOPORT3_PCINT_PAD_VALID(pad)  (true)

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINC)
    #define PCINT1_COUNT()                (8U)

  #elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
      defined(__AVR_ATmega32U2__)

    #define IOPORT3_PCMSK                 PCMSK1
    #define IOPORT3_PCINT                 1
    #define IOPORT3_PCINT_OFFSET(pad) \
        ((pad == 2) ? 3 : ((pad == 4) ? 2 : ((pad == 5) ? 1 : 0)))
    #define IOPORT3_PCINT_PAD_VALID(pad) \
        ((bool)(((1U << pad) & 0b01110100) != 0))

    #define IOPORT4_PCMSK                 PCMSK1
    #define IOPORT4_PCINT                 1
    #define IOPORT4_PCINT_OFFSET(pad)     (pad - 4)
    #define IOPORT4_PCINT_PAD_VALID(pad)  ((bool)(pad == 4))

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET() \
        ((PINC & 0x40) >> 6) \
        ((PINC & 0x20) >> 4) \
        ((PINC & 0x10) >> 2) \
        ((PINC & 0x04) << 1) \
        | ((PIND & 0x20) >> 1)
    #define PCINT1_COUNT()                (5U)

  #elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)

    #define IOPORT5_PCMSK                 PCMSK1
    #define IOPORT5_PCINT                 1
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  ((bool)(pad == 0))

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET()                  (PINE)
    #define PCINT1_COUNT()                (1U)

  #elif defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
      defined(__AVR_ATmega2560__)

    #define IOPORT5_PCMSK                 PCMSK1
    #define IOPORT5_PCINT                 1
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  ((bool)(pad == 0))

    #define IOPORT10_PCMSK                PCMSK1
    #define IOPORT10_PCINT                1
    #define IOPORT10_PCINT_OFFSET(pad)    (pad + 1)
    #define IOPORT10_PCINT_PAD_VALID(pad) ((bool)(pad < 7))

    #define PCINT1_ENABLE()               PCICR |= (1U << PCIE1)
    #define PCINT1_GET()                  ((PINJ << 1) | (PINE & 0x01))
    #define PCINT1_COUNT()                (8U)

  #else
    #error "PCINT1 is not supported on this ATmega device"
  #endif
#endif /* AVR_GPIO_USE_PCINT1 */

#if AVR_GPIO_USE_PCINT2

  #if defined(__AVR_ATmega164__) || defined(__AVR_ATmega164P__) || \
      defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164PA__) || \
      defined(__AVR_ATmega324__) || defined(__AVR_ATmega324P__) || \
      defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324PA__) || \
      defined(__AVR_ATmega324PB__) || \
      defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644PA__) || \
      defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)

    #define IOPORT3_PCMSK                 PCMSK2
    #define IOPORT3_PCINT                 2
    #define IOPORT3_PCINT_OFFSET(pad)     (pad)
    #define IOPORT3_PCINT_PAD_VALID(pad)  (true)

    #define PCINT2_ENABLE()               PCICR |= (1U << PCIE2)
    #define PCINT2_GET()                  (PINC)
    #define PCINT2_COUNT()                (8U)

  #elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48P__) || \
      defined(__AVR_ATmega48A__) || defined(__AVR_ATmega48PA__) || \
      defined(__AVR_ATmega48PB__) || \
      defined(__AVR_ATmega88__) || defined(__AVR_ATmega88P__) || \
      defined(__AVR_ATmega88A__) || defined(__AVR_ATmega88PA__) || \
      defined(__AVR_ATmega88PB__) || \
      defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega168A__) || defined(__AVR_ATmega168PA__) || \
      defined(__AVR_ATmega168PB) || \
      defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
      defined(__AVR_ATmega328PB__) || \
      defined(__AVR_ATmega16C1__) || defined(__AVR_ATmega16M1__) || \
      defined(__AVR_ATmega32C1__) || defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64C1__) || defined(__AVR_ATmega64M1__)

    #define IOPORT4_PCMSK                 PCMSK2
    #define IOPORT4_PCINT                 2
    #define IOPORT4_PCINT_OFFSET(pad)     (pad)
    #define IOPORT4_PCINT_PAD_VALID(pad)  (true)

    #define PCINT2_ENABLE()               PCICR |= (1U << PCIE2)
    #define PCINT2_GET()                  (PIND)
    #define PCINT2_COUNT()                (8U)

  #elif defined(__AVR_ATmega3290__) || defined(__AVR_ATmega3290P__) || \
  defined(__AVR_ATmega3290A__) || defined(__AVR_ATmega3290PA__) || \
      defined(__AVR_ATmega6490__) || defined(__AVR_ATmega6490P__) || \
      defined(__AVR_ATmega6490A__) || defined(__AVR_ATmega6490PA__) || \
      defined(__AVR_ATmega3250__) || defined(__AVR_ATmega3250P__) || \
      defined(__AVR_ATmega3250A__) || defined(__AVR_ATmega3250PA__) || \
      defined(__AVR_ATmega6450__) || defined(__AVR_ATmega6450P__) || \
      defined(__AVR_ATmega6450A__) || defined(__AVR_ATmega6450PA__)

    #define IOPORT8_PCMSK                 PCMSK2
    #define IOPORT8_PCINT                 2
    #define IOPORT8_PCINT_OFFSET(pad)     (pad)
    #define IOPORT8_PCINT_PAD_VALID(pad)  (true)

    #define PCINT2_ENABLE()               EIMSK |= (1U << PCIE2)
    #define PCINT2_GET()                  (PINH)
    #define PCINT2_COUNT()                (8U)

  #elif defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
      defined(__AVR_ATmega2560__)

    #define IOPORT11_PCMSK                PCMSK2
    #define IOPORT11_PCINT                 2
    #define IOPORT11_PCINT_OFFSET(pad)    (pad)
    #define IOPORT11_PCINT_PAD_VALID(pad) (true)

    #define PCINT2_ENABLE()               PCICR |= (1U << PCIE2)
    #define PCINT2_GET()                  (PINK)
    #define PCINT2_COUNT()                (8U)

  #else
      #error "PCINT2 is not supported on this ATmega device"
  #endif
#endif /* AVR_GPIO_USE_PCINT2 */

#if AVR_GPIO_USE_PCINT3

  #if defined(__AVR_ATmega164__) || defined(__AVR_ATmega164P__) || \
      defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164PA__) || \
      defined(__AVR_ATmega324__) || defined(__AVR_ATmega324P__) || \
      defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324PA__) || \
      defined(__AVR_ATmega324PB__) || \
      defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644PA__) || \
      defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)

    #define IOPORT4_PCMSK                 PCMSK3
    #define IOPORT4_PCINT                 3
    #define IOPORT4_PCINT_OFFSET(pad)     (pad)
    #define IOPORT4_PCINT_PAD_VALID(pad)  (true)

    #define PCINT3_ENABLE()               PCICR |= (1U << PCIE3)
    #define PCINT3_GET()                  (PIND)
    #define PCINT3_COUNT()                (8U)

  #elif defined(__AVR_ATmega328PB__)

    #define IOPORT5_PCMSK                 PCMSK3
    #define IOPORT5_PCINT                 3
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  ((bool)(pad < 4))

    #define PCINT3_ENABLE()               PCICR |= (1U << PCIE3)
    #define PCINT3_GET()                  (PINE)
    #define PCINT3_COUNT()                (4U)

  #elif defined(__AVR_ATmega16C1__) || defined(__AVR_ATmega16M1__) || \
      defined(__AVR_ATmega32C1__) || defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64C1__) || defined(__AVR_ATmega64M1__)

    #define IOPORT5_PCMSK                 PCMSK3
    #define IOPORT5_PCINT                 3
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  ((bool)(pad < 3))

    #define PCINT3_ENABLE()               PCICR |= (1U << PCIE3)
    #define PCINT3_GET()                  (PINE)
    #define PCINT3_COUNT()                (3U)

  #elif defined(__AVR_ATmega3290__) || defined(__AVR_ATmega3290P__) || \
      defined(__AVR_ATmega3290A__) || defined(__AVR_ATmega3290PA__) || \
      defined(__AVR_ATmega6490__) || defined(__AVR_ATmega6490P__) || \
      defined(__AVR_ATmega6490A__) || defined(__AVR_ATmega6490PA__) || \
      defined(__AVR_ATmega3250__) || defined(__AVR_ATmega3250P__) || \
      defined(__AVR_ATmega3250A__) || defined(__AVR_ATmega3250PA__) || \
      defined(__AVR_ATmega6450__) || defined(__AVR_ATmega6450P__) || \
      defined(__AVR_ATmega6450A__) || defined(__AVR_ATmega6450PA__)

    #define IOPORT10_PCMSK                PCMSK3
    #define IOPORT10_PCINT                3
    #define IOPORT10_PCINT_OFFSET(pad)    (pad)
    #define IOPORT10_PCINT_PAD_VALID(pad) (pad < 7)

    #define PCINT3_ENABLE()               EIMSK |= (1U << PCIE3)
    #define PCINT3_GET()                  (PINJ)
    #define PCINT3_COUNT()                (7U)

  #else
    #error "PCINT3 is not supported on this ATmega device"
  #endif
#endif /* AVR_GPIO_USE_PCINT3 */

#if AVR_GPIO_USE_PCINT4

  #if defined(__AVR_ATmega324PB__)

    #define IOPORT5_PCMSK                 PCMSK4
    #define IOPORT5_PCINT                 4
    #define IOPORT5_PCINT_OFFSET(pad)     (pad)
    #define IOPORT5_PCINT_PAD_VALID(pad)  (true)

    #define PCINT4_ENABLE()               PCICR |= (1U << PCIE4)
    #define PCINT4_GET()                  (PINE)
    #define PCINT4_COUNT()                (8U)

  #else
    #error "PCINT4 is not supported on this ATmega device"
  #endif
#endif /* AVR_GPIO_USE_PCINT4 */

#endif /* AVR_PCINT_H */

/** @} */
