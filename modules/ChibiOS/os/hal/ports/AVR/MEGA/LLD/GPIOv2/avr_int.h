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
 * @file  GPIOv2/avr_int.h
 * @brief AVR/MEGA External Interrupts units common header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef AVR_INT_H
#define AVR_INT_H

#include "avr_pins.h"

/**
 * @brief   INT0 support enable switch.
 * @details If set to @p TRUE support for events on INT0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT0)
  #define AVR_GPIO_USE_INT0           FALSE
#endif

/**
 * @brief   INT1 support enable switch.
 * @details If set to @p TRUE support for events on INT1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT1)
  #define AVR_GPIO_USE_INT1           FALSE
#endif

/**
 * @brief   INT2 support enable switch.
 * @details If set to @p TRUE support for events on INT2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT2)
  #define AVR_GPIO_USE_INT2           FALSE
#endif

/**
 * @brief   INT3 support enable switch.
 * @details If set to @p TRUE support for events on INT3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT3)
  #define AVR_GPIO_USE_INT3           FALSE
#endif

/**
 * @brief   INT4 support enable switch.
 * @details If set to @p TRUE support for events on INT4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT4)
  #define AVR_GPIO_USE_INT4           FALSE
#endif

/**
 * @brief   INT5 support enable switch.
 * @details If set to @p TRUE support for events on INT5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT5)
  #define AVR_GPIO_USE_INT5           FALSE
#endif

/**
 * @brief   INT6 support enable switch.
 * @details If set to @p TRUE support for events on INT6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT6)
  #define AVR_GPIO_USE_INT6           FALSE
#endif

/**
 * @brief   INT7 support enable switch.
 * @details If set to @p TRUE support for events on INT7 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_GPIO_USE_INT7)
  #define AVR_GPIO_USE_INT7           FALSE
#endif


/**
 * For each INTx pin, the following macros must be defined:
 *   - INTx_IOPORT      - ChibiOS IOPORT that corresponds to the interrupt pin
 *   - INTx_PAD         - pad on port that has the INTx signal
 *   - INTx_EICR        - register to configure the interrupt
 *                        (EICRA, MCUCSR, etc.)
 *   - INTx_EICR_BASE   - offset within register of configuration bits
 *   - INTx_ENABLE()    - operation to enable interrupt line
 *   - INTx_DISABLE()   - operation to disable interrupt line
 *   - INTx_ISENABLED() - return TRUE if interrupt line is enabled, false
 *                        otherwise
 *   - INTx_LEGACY      - NOTE: only define if the pin uses the legacy compact
 *                        configuration format (e.g. INT2 on old devices)
 */

#if AVR_GPIO_USE_INT0

/* INT0 on PD0 - EICRA register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__) || \
    defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT0_IOPORT                 IOPORT4
  #define INT0_PAD                    0
  #define INT0_EICR                   EICRA
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (EIMSK |= (1 << INT0))
  #define INT0_DISABLE()              (EIMSK &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((EIMSK & (1 << INT0)) != 0))

/* INT0 on PD1 - EICRA register */
#elif defined(__AVR_ATmega169__) || defined(__AVR_ATmega169A__) || \
    defined(__AVR_ATmega169P__) || defined(__AVR_ATmega169PA__) || \
    defined(__AVR_ATmega329__) || defined(__AVR_ATmega329A__) || \
    defined(__AVR_ATmega329P__) || defined(__AVR_ATmega329PA__) || \
    defined(__AVR_ATmega649__) || defined(__AVR_ATmega649A__) || \
    defined(__AVR_ATmega649P__) || \
    defined(__AVR_ATmega3290__) || defined(__AVR_ATmega3290A__) || \
    defined(__AVR_ATmega3290P__) || defined(__AVR_ATmega3290PA__) || \
    defined(__AVR_ATmega6490__) || defined(__AVR_ATmega6490A__) || \
    defined(__AVR_ATmega6490P__) || \
    defined(__AVR_ATmega165__) || defined(__AVR_ATmega165A__) || \
    defined(__AVR_ATmega165P__) || defined(__AVR_ATmega165PA__) || \
    defined(__AVR_ATmega325__) || defined(__AVR_ATmega325A__) || \
    defined(__AVR_ATmega325P__) || defined(__AVR_ATmega325PA__) || \
    defined(__AVR_ATmega645__) || defined(__AVR_ATmega645A__) || \
    defined(__AVR_ATmega645P__) || \
    defined(__AVR_ATmega3250__) || defined(__AVR_ATmega3250A__) || \
    defined(__AVR_ATmega3250P__) || defined(__AVR_ATmega3250PA__) || \
    defined(__AVR_ATmega6450__) || defined(__AVR_ATmega6450A__) || \
    defined(__AVR_ATmega6450P__)

  #define INT0_IOPORT                 IOPORT4
  #define INT0_PAD                    1
  #define INT0_EICR                   EICRA
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (EIMSK |= (1 << INT0))
  #define INT0_DISABLE()              (EIMSK &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((EIMSK & (1 << INT0)) != 0))

/* INT0 on PD2 - EICRA register */
#elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48A__) || \
    defined(__AVR_ATmega48P__) || defined(__AVR_ATmega48PA__) || \
    defined(__AVR_ATmega48PB__) || \
    defined(__AVR_ATmega88__) || defined(__AVR_ATmega88A__) || \
    defined(__AVR_ATmega88P__) || defined(__AVR_ATmega88PA__) || \
    defined(__AVR_ATmega88PB__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega168A__) || \
    defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168PA__) || \
    defined(__AVR_ATmega168PB__) || \
    defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || \
    defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164P__) || \
    defined(__AVR_ATmega164PA__) || \
    defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__) || \
    defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || \
    defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)

  #define INT0_IOPORT                 IOPORT4
  #define INT0_PAD                    2
  #define INT0_EICR                   EICRA
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (EIMSK |= (1 << INT0))
  #define INT0_DISABLE()              (EIMSK &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((EIMSK & (1 << INT0)) != 0))

/* INT0 on PD2 - MCUCR register (has ISC00 and ISC01) */
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) || \
    defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) || \
    defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) || \
    defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) || \
    defined(__AVR_ATmega162__)

  #define INT0_IOPORT                 IOPORT4
  #define INT0_PAD                    2
  #define INT0_EICR                   MCUCR
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (GICR |= (1 << INT0))
  #define INT0_DISABLE()              (GICR &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((GICR & (1 << INT0)) != 0))

/* INT0 on PA4 - EICRA register */
#elif defined(__AVR_ATmega406__)

  #define INT0_IOPORT                 IOPORT1
  #define INT0_PAD                    4
  #define INT0_EICR                   EICRA
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (EIMSK |= (1 << INT0))
  #define INT0_DISABLE()              (EIMSK &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((EIMSK & (1 << INT0)) != 0))

/* INT0 on PD6 - EICRA register */
#elif defined(__AVR_ATmega64M1__) || defined(__AVR_ATmega64C1__) || \
    defined(__AVR_ATmega32M1__) || defined(__AVR_ATmega32C1__) || \
    defined(__AVR_ATmega16M1__) || defined(__AVR_ATmega16C1__)

  #define INT0_IOPORT                 IOPORT4
  #define INT0_PAD                    6
  #define INT0_EICR                   EICRA
  #define INT0_EICR_BASE              0
  #define INT0_ENABLE()               (EIMSK |= (1 << INT0))
  #define INT0_DISABLE()              (EIMSK &= ~(1 << INT0))
  #define INT0_ISENABLED()            ((bool)((EIMSK & (1 << INT0)) != 0))

#else
  #error "INT0 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT0 */

#if AVR_GPIO_USE_INT1

/* INT1 on PD1 - EICRA register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__) || \
    defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT1_IOPORT                 IOPORT4
  #define INT1_PAD                    1
  #define INT1_EICR                   EICRA
  #define INT1_EICR_BASE              2
  #define INT1_ENABLE()               (EIMSK |= (1 << INT1))
  #define INT1_DISABLE()              (EIMSK &= ~(1 << INT1))
  #define INT1_ISENABLED()            ((bool)((EIMSK & (1 << INT1)) != 0))

/* INT1 on PD3 - EICRA register */
#elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48A__) || \
    defined(__AVR_ATmega48P__) || defined(__AVR_ATmega48PA__) || \
    defined(__AVR_ATmega48PB__) || \
    defined(__AVR_ATmega88__) || defined(__AVR_ATmega88A__) || \
    defined(__AVR_ATmega88P__) || defined(__AVR_ATmega88PA__) || \
    defined(__AVR_ATmega88PB__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega168A__) || \
    defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168PA__) || \
    defined(__AVR_ATmega168PB__) || \
    defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || \
    defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164P__) || \
    defined(__AVR_ATmega164PA__) || \
    defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__) || \
    defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || \
    defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)

  #define INT1_IOPORT                 IOPORT4
  #define INT1_PAD                    3
  #define INT1_EICR                   EICRA
  #define INT1_EICR_BASE              2
  #define INT1_ENABLE()               (EIMSK |= (1 << INT1))
  #define INT1_DISABLE()              (EIMSK &= ~(1 << INT1))
  #define INT1_ISENABLED()            ((bool)((EIMSK & (1 << INT1)) != 0))

/* INT1 on PD3 - MCUCR register (has ISC10 and ISC11) */
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) || \
    defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) || \
    defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) || \
    defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) || \
    defined(__AVR_ATmega162__)

  #define INT1_IOPORT                 IOPORT4
  #define INT1_PAD                    3
  #define INT1_EICR                   MCUCR
  #define INT1_EICR_BASE              2
  #define INT1_ENABLE()               (GICR |= (1 << INT1))
  #define INT1_DISABLE()              (GICR &= ~(1 << INT1))
  #define INT1_ISENABLED()            ((bool)((GICR & (1 << INT1)) != 0))

/* INT1 on PA5 - EICRA register */
#elif defined(__AVR_ATmega406__)

  #define INT1_IOPORT                 IOPORT1
  #define INT1_PAD                    5
  #define INT1_EICR                   EICRA
  #define INT1_EICR_BASE              2
  #define INT1_ENABLE()               (EIMSK |= (1 << INT1))
  #define INT1_DISABLE()              (EIMSK &= ~(1 << INT1))
  #define INT1_ISENABLED()            ((bool)((EIMSK & (1 << INT1)) != 0))

/* INT1 on PB3 - EICRA register */
#elif defined(__AVR_ATmega64M1__) || defined(__AVR_ATmega64C1__) || \
    defined(__AVR_ATmega32M1__) || defined(__AVR_ATmega32C1__) || \
    defined(__AVR_ATmega16M1__) || defined(__AVR_ATmega16C1__)

  #define INT1_IOPORT                 IOPORT2
  #define INT1_PAD                    3
  #define INT1_EICR                   EICRA
  #define INT1_EICR_BASE              2
  #define INT1_ENABLE()               (EIMSK |= (1 << INT1))
  #define INT1_DISABLE()              (EIMSK &= ~(1 << INT1))
  #define INT1_ISENABLED()            ((bool)((EIMSK & (1 << INT1)) != 0))

#else
  #error "INT1 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT1 */

#if AVR_GPIO_USE_INT2

/* INT2 on PD2 - EICRA register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__) || \
    defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT2_IOPORT                 IOPORT4
  #define INT2_PAD                    2
  #define INT2_EICR                   EICRA
  #define INT2_EICR_BASE              4
  #define INT2_ENABLE()               (EIMSK |= (1 << INT2))
  #define INT2_DISABLE()              (EIMSK &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((EIMSK & (1 << INT2)) != 0))

/* INT2 on PB2 - EICRA register */
#elif defined(__AVR_ATmega164A__) || defined(__AVR_ATmega164P__) || \
    defined(__AVR_ATmega164PA__) || \
    defined(__AVR_ATmega324A__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega324PB__) || \
    defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || \
    defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)

  #define INT2_IOPORT                 IOPORT2
  #define INT2_PAD                    2
  #define INT2_EICR                   EICRA
  #define INT2_EICR_BASE              4
  #define INT2_ENABLE()               (EIMSK |= (1 << INT2))
  #define INT2_DISABLE()              (EIMSK &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((EIMSK & (1 << INT2)) != 0))

/* INT2 on PB2 - MCUCSR register (legacy - only ISC2 bit) */
#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) || \
    defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) || \
    defined(__AVR_ATmega8535__)

  #define INT2_IOPORT                 IOPORT2
  #define INT2_PAD                    2
  #define INT2_EICR                   MCUCSR
  #define INT2_EICR_BASE              6
  #define INT2_LEGACY                 1
  #define INT2_ENABLE()               (GICR |= (1 << INT2))
  #define INT2_DISABLE()              (GICR &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((GICR & (1 << INT2)) != 0))

/* INT2 on PE0 - EMCUCR register (legacy - only ISC2 bit) */
#elif defined(__AVR_ATmega8515__) || defined(__AVR_ATmega162__)

  #define INT2_IOPORT                 IOPORT5
  #define INT2_PAD                    0
  #define INT2_EICR                   EMCUCR
  #define INT2_EICR_BASE              0
  #define INT2_LEGACY                 1
  #define INT2_ENABLE()               (GICR |= (1 << INT2))
  #define INT2_DISABLE()              (GICR &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((GICR & (1 << INT2)) != 0))

/* INT2 on PA6 - EICRA register */
#elif defined(__AVR_ATmega406__)

  #define INT2_IOPORT                 IOPORT1
  #define INT2_PAD                    6
  #define INT2_EICR                   EICRA
  #define INT2_EICR_BASE              4
  #define INT2_ENABLE()               (EIMSK |= (1 << INT2))
  #define INT2_DISABLE()              (EIMSK &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((EIMSK & (1 << INT2)) != 0))

/* INT2 on PB5 - EICRA register */
#elif defined(__AVR_ATmega64M1__) || defined(__AVR_ATmega64C1__) || \
    defined(__AVR_ATmega32M1__) || defined(__AVR_ATmega32C1__) || \
    defined(__AVR_ATmega16M1__) || defined(__AVR_ATmega16C1__)

  #define INT2_IOPORT                 IOPORT2
  #define INT2_PAD                    5
  #define INT2_EICR                   EICRA
  #define INT2_EICR_BASE              4
  #define INT2_ENABLE()               (EIMSK |= (1 << INT2))
  #define INT2_DISABLE()              (EIMSK &= ~(1 << INT2))
  #define INT2_ISENABLED()            ((bool)((EIMSK & (1 << INT2)) != 0))

#else
  #error "INT2 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT2 */

#if AVR_GPIO_USE_INT3

/* INT3 on PD3 - EICRA register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__) || \
    defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT3_IOPORT                 IOPORT4
  #define INT3_PAD                    3
  #define INT3_EICR                   EICRA
  #define INT3_EICR_BASE              6
  #define INT3_ENABLE()               (EIMSK |= (1 << INT3))
  #define INT3_DISABLE()              (EIMSK &= ~(1 << INT3))
  #define INT3_ISENABLED()            ((bool)((EIMSK & (1 << INT3)) != 0))

/* INT3 on PA7 - EICRA register */
#elif defined(__AVR_ATmega406__)

  #define INT3_IOPORT                 IOPORT1
  #define INT3_PAD                    7
  #define INT3_EICR                   EICRA
  #define INT3_EICR_BASE              6
  #define INT3_ENABLE()               (EIMSK |= (1 << INT3))
  #define INT3_DISABLE()              (EIMSK &= ~(1 << INT3))
  #define INT3_ISENABLED()            ((bool)((EIMSK & (1 << INT3)) != 0))

/* INT3 on PC0 - EICRA register */
#elif defined(__AVR_ATmega64M1__) || defined(__AVR_ATmega64C1__) || \
    defined(__AVR_ATmega32M1__) || defined(__AVR_ATmega32C1__) || \
    defined(__AVR_ATmega16M1__) || defined(__AVR_ATmega16C1__)

  #define INT3_IOPORT                 IOPORT3
  #define INT3_PAD                    0
  #define INT3_EICR                   EICRA
  #define INT3_EICR_BASE              6
  #define INT3_ENABLE()               (EIMSK |= (1 << INT3))
  #define INT3_DISABLE()              (EIMSK &= ~(1 << INT3))
  #define INT3_ISENABLED()            ((bool)((EIMSK & (1 << INT3)) != 0))

#else
  #error "INT3 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT3 */

#if AVR_GPIO_USE_INT4

/* INT4 on PE4 - EICRB register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT4_IOPORT                 IOPORT5
  #define INT4_PAD                    4
  #define INT4_EICR                   EICRB
  #define INT4_EICR_BASE              0
  #define INT4_ENABLE()               (EIMSK |= (1 << INT4))
  #define INT4_DISABLE()              (EIMSK &= ~(1 << INT4))
  #define INT4_ISENABLED()            ((bool)((EIMSK & (1 << INT4)) != 0))

/* INT4 on PC7 - EICRB register */
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__)

  #define INT4_IOPORT                 IOPORT3
  #define INT4_PAD                    7
  #define INT4_EICR                   EICRB
  #define INT4_EICR_BASE              0
  #define INT4_ENABLE()               (EIMSK |= (1 << INT4))
  #define INT4_DISABLE()              (EIMSK &= ~(1 << INT4))
  #define INT4_ISENABLED()            ((bool)((EIMSK & (1 << INT4)) != 0))

#else
  #error "INT4 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT4 */

#if AVR_GPIO_USE_INT5

/* INT5 on PE5 - EICRB register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT5_IOPORT                 IOPORT5
  #define INT5_PAD                    5
  #define INT5_EICR                   EICRB
  #define INT5_EICR_BASE              2
  #define INT5_ENABLE()               (EIMSK |= (1 << INT5))
  #define INT5_DISABLE()              (EIMSK &= ~(1 << INT5))
  #define INT5_ISENABLED()            ((bool)((EIMSK & (1 << INT5)) != 0))

/* INT5 on PC4 - EICRB register */
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__)

  #define INT5_IOPORT                 IOPORT3
  #define INT5_PAD                    4
  #define INT5_EICR                   EICRB
  #define INT5_EICR_BASE              2
  #define INT5_ENABLE()               (EIMSK |= (1 << INT5))
  #define INT5_DISABLE()              (EIMSK &= ~(1 << INT5))
  #define INT5_ISENABLED()            ((bool)((EIMSK & (1 << INT5)) != 0))

#else
  #error "INT5 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT5 */

#if AVR_GPIO_USE_INT6

/* INT6 on PE6 - EICRB register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_ATmega16U4__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT6_IOPORT                 IOPORT5
  #define INT6_PAD                    6
  #define INT6_EICR                   EICRB
  #define INT6_EICR_BASE              4
  #define INT6_ENABLE()               (EIMSK |= (1 << INT6))
  #define INT6_DISABLE()              (EIMSK &= ~(1 << INT6))
  #define INT6_ISENABLED()            ((bool)((EIMSK & (1 << INT6)) != 0))

/* INT6 on PC6 - EICRB register */
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__)

  #define INT6_IOPORT                 IOPORT3
  #define INT6_PAD                    6
  #define INT6_EICR                   EICRB
  #define INT6_EICR_BASE              4
  #define INT6_ENABLE()               (EIMSK |= (1 << INT6))
  #define INT6_DISABLE()              (EIMSK &= ~(1 << INT6))
  #define INT6_ISENABLED()            ((bool)((EIMSK & (1 << INT6)) != 0))

#else
  #error "INT6 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT6 */

#if AVR_GPIO_USE_INT7

/* INT7 on PE7 - EICRB register */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) || \
    defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) || \
    defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || \
    defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega2561__) || \
    defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__) || \
    defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB1287__)

  #define INT7_IOPORT                 IOPORT5
  #define INT7_PAD                    7
  #define INT7_EICR                   EICRB
  #define INT7_EICR_BASE              6
  #define INT7_ENABLE()               (EIMSK |= (1 << INT7))
  #define INT7_DISABLE()              (EIMSK &= ~(1 << INT7))
  #define INT7_ISENABLED()            ((bool)((EIMSK & (1 << INT7)) != 0))

/* INT7 on PC2 - EICRB register */
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || \
    defined(__AVR_ATmega32U2__)

  #define INT7_IOPORT                 IOPORT3
  #define INT7_PAD                    2
  #define INT7_EICR                   EICRB
  #define INT7_EICR_BASE              6
  #define INT7_ENABLE()               (EIMSK |= (1 << INT7))
  #define INT7_DISABLE()              (EIMSK &= ~(1 << INT7))
  #define INT7_ISENABLED()            ((bool)((EIMSK & (1 << INT7)) != 0))

#else
  #error "INT7 is not supported on this ATmega device"
#endif

#endif /* AVR_GPIO_USE_INT7 */

#endif /* AVR_INT_H */

/** @} */
