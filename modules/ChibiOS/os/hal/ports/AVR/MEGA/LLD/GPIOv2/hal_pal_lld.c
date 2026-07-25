/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.
              Copyright (C) 2006-2026 Nathan Lewis
              Copyright (C) 2006-2026 Theodore Ateba

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
 * @file  GPIOv2/hal_pal_lld.c
 * @brief AVR/MEGA PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) || \
    defined(__DOXYGEN__)
/**
 * @brief   Event records for the GPIO pin change channels.
 */
palevent_t _pal_events[_pal_event_count()];
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

typedef struct {
  uint8_t rising;
  uint8_t falling;
} avr_gpio_event_config_t;

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) || \
    defined(__DOXYGEN__)

#define INT0_BASE (0U)
#define INT1_BASE (INT0_BASE + INT0_EVENT_COUNT)
#define INT2_BASE (INT1_BASE + INT1_EVENT_COUNT)
#define INT3_BASE (INT2_BASE + INT2_EVENT_COUNT)
#define INT4_BASE (INT3_BASE + INT3_EVENT_COUNT)
#define INT5_BASE (INT4_BASE + INT4_EVENT_COUNT)
#define INT6_BASE (INT5_BASE + INT5_EVENT_COUNT)
#define INT7_BASE (INT6_BASE + INT6_EVENT_COUNT)

#if PCINT0_EVENT_COUNT || defined(__DOXYGEN__)
avr_gpio_event_config_t _event_config_pcint0;
volatile uint8_t _pin_state_pcint0;
#endif
#define PCINT0_BASE (INT7_BASE + INT7_EVENT_COUNT)

#if PCINT1_EVENT_COUNT || defined(__DOXYGEN__)
avr_gpio_event_config_t _event_config_pcint1;
volatile uint8_t _pin_state_pcint1;
#endif
#define PCINT1_BASE (PCINT0_BASE + PCINT0_EVENT_COUNT)

#if PCINT2_EVENT_COUNT || defined(__DOXYGEN__)
avr_gpio_event_config_t _event_config_pcint2;
volatile uint8_t _pin_state_pcint2;
#endif
#define PCINT2_BASE (PCINT1_BASE + PCINT1_EVENT_COUNT)

#if PCINT3_EVENT_COUNT || defined(__DOXYGEN__)
avr_gpio_event_config_t _event_config_pcint3;
volatile uint8_t _pin_state_pcint3;
#endif
#define PCINT3_BASE (PCINT2_BASE + PCINT2_EVENT_COUNT)

#if PCINT4_EVENT_COUNT || defined(__DOXYGEN__)
avr_gpio_event_config_t _event_config_pcint4;
volatile uint8_t _pin_state_pcint4`;
#endif
#define PCINT4_BASE (PCINT3_BASE + PCINT3_EVENT_COUNT)

#endif /* PAL_USE_WAIT || PAL_USE_CALLBACKS */

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

#if PAL_USE_CALLBACKS || PAL_USE_WAIT || defined(__DOXYGEN__)

#if INT0_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT0_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT0_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT1_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT1_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT1_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT2_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT2_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT2_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT3_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT3_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT3_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT4_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT4_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT4_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT5_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT5_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT5_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT6_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT6_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT6_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if INT7_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(INT7_vect) {
  OSAL_IRQ_PROLOGUE();
  _pal_isr_code(INT7_BASE);
  OSAL_IRQ_EPILOGUE();
}
#endif

#define _pcint_isr_code(n) \
    do { \
      const uint8_t pins = PCINT##n##_GET(); \
      const uint8_t changed = PCMSK##n & (pins ^ _pin_state_pcint##n); \
      const uint8_t rising = _event_config_pcint##n.rising & changed & pins; \
      const uint8_t falling = \
          _event_config_pcint##n.falling & changed & ~pins; \
      uint8_t events = rising | falling; \
      for (uint8_t i = 0; events != 0; i++, events >>= 1) { \
        if (events & 1) { \
          _pal_isr_code(PCINT##n##_BASE + i); \
        } \
      } \
      _pin_state_pcint##n = pins; \
    } while (0)

#if PCINT0_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PCINT0_vect) {
  OSAL_IRQ_PROLOGUE();
  _pcint_isr_code(0);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if PCINT1_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PCINT1_vect) {
  OSAL_IRQ_PROLOGUE();
  _pcint_isr_code(1);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if PCINT2_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PCINT2_vect) {
  OSAL_IRQ_PROLOGUE();
  _pcint_isr_code(2);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if PCINT3_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PCINT3_vect) {
  OSAL_IRQ_PROLOGUE();
  _pcint_isr_code(3);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if PCINT4_EVENT_COUNT || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(PCINT4_vect) {
  OSAL_IRQ_PROLOGUE();
  _pcint_isr_code(4);
  OSAL_IRQ_EPILOGUE();
}
#endif

#endif /* PAL_USE_WAIT || PAL_USE_CALLBACKS */

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   AVR GPIO ports configuration.
 * @details GPIO registers initialization.
 *
 * @param[in] config    the AVR ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

#if (PAL_USE_CALLBACKS == TRUE) || (PAL_USE_WAIT == TRUE) || \
    defined(__DOXYGEN__)
  uint8_t i;

  for (i = 0; i < _pal_event_count(); i++) {
    _pal_init_event(i);
  }
#endif

#if defined(PORTA) || defined(__DOXYGEN__)
  PORTA = config->porta.out;
  DDRA = config->porta.dir;
#endif

#if defined(PORTB) || defined(__DOXYGEN__)
  PORTB = config->portb.out;
  DDRB = config->portb.dir;
#endif

#if defined(PORTC) || defined(__DOXYGEN__)
  PORTC = config->portc.out;
  DDRC = config->portc.dir;
#endif

#if defined(PORTD) || defined(__DOXYGEN__)
  PORTD = config->portd.out;
  DDRD = config->portd.dir;
#endif

#if defined(PORTE) || defined(__DOXYGEN__)
  PORTE = config->porte.out;
  DDRE = config->porte.dir;
#endif

#if defined(PORTF) || defined(__DOXYGEN__)
  PORTF = config->portf.out;
  DDRF = config->portf.dir;
#endif

#if defined(PORTG) || defined(__DOXYGEN__)
  PORTG = config->portg.out;
  DDRG = config->portg.dir;
#endif

#if defined(PORTH) || defined(__DOXYGEN__)
  PORTH = config->porth.out;
  DDRH = config->porth.dir;
#endif

#if defined(PORTJ) || defined(__DOXYGEN__)
  PORTJ = config->portj.out;
  DDRJ = config->portj.dir;
#endif

#if defined(PORTK) || defined(__DOXYGEN__)
  PORTK = config->portk.out;
  DDRK = config->portk.dir;
#endif

#if defined(PORTL) || defined(__DOXYGEN__)
  PORTL = config->portl.out;
  DDRL = config->portl.dir;
#endif

#if (PAL_USE_CALLBACKS == TRUE) || (PAL_USE_WAIT == TRUE) || \
    defined(__DOXYGEN__)
  #if PCINT0_EVENT_COUNT || defined(__DOXYGEN__)
    PCINT0_ENABLE();
  #endif

  #if PCINT1_EVENT_COUNT || defined(__DOXYGEN__)
    PCINT1_ENABLE();
  #endif

  #if PCINT2_EVENT_COUNT || defined(__DOXYGEN__)
    PCINT2_ENABLE();
  #endif

  #if PCINT3_EVENT_COUNT || defined(__DOXYGEN__)
    PCINT3_ENABLE();
  #endif

  #if PCINT4_EVENT_COUNT || defined(__DOXYGEN__)
    PCINT4_ENABLE();
  #endif
#endif /* PAL_USE_CALLBACKS || PAL_USE_WAIT */
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port the port identifier
 * @param[in] mask the group mask
 * @param[in] mode the mode
 *
 * @note This function is not meant to be invoked directly by the application
 *       code.
 * @note @p PAL_MODE_UNCONNECTED is implemented as output as recommended by
 *       the AVR Family User's Guide. Unconnected pads are set to input
 *       with pull-up by default.
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

  switch (mode) {
  case PAL_MODE_RESET:
  case PAL_MODE_INPUT:
  case PAL_MODE_INPUT_ANALOG:
    port->dir &= ~mask;
    port->out &= ~mask;
    break;
  case PAL_MODE_UNCONNECTED:
  case PAL_MODE_INPUT_PULLUP:
    port->dir &= ~mask;
    port->out |= mask;
    break;
  case PAL_MODE_OUTPUT_PUSHPULL:
    port->dir |= mask;
    break;
  }
}

#if (PAL_USE_CALLBACKS == TRUE) || (PAL_USE_WAIT == TRUE) || \
    defined(__DOXYGEN__)

/**
 * @brief   Pad event enable.
 * @details This function programs an event callback in the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad event mode
 * @param[in] callback  event callback function
 * @param[in] arg       callback argument
 *
 * @notapi
 */
void _pal_lld_enablepadevent(ioportid_t     port,
                             iopadid_t      pad,
                             ioeventmode_t  mode) {
  #define __enablepadevent_int_code(n) \
      if (port == INT##n##_IOPORT && pad == INT##n##_PAD) { \
        if (mode) { \
          if (mode == PAL_EVENT_MODE_FALLING_EDGE) { \
            INT##n##_EICR = (INT##n##_EICR & ~(3U << INT##n##_EICR_BASE)) \
                | (2U << INT##n##_EICR_BASE); \
          } else if (mode == PAL_EVENT_MODE_RISING_EDGE) { \
            INT##n##_EICR = (INT##n##_EICR & ~(3U << INT##n##_EICR_BASE)) \
                | (3U << INT##n##_EICR_BASE); \
          } else { \
            INT##n##_EICR = (INT##n##_EICR & ~(3U << INT##n##_EICR_BASE)) \
                | (1U << INT##n##_EICR_BASE); \
          } \
          INT##n##_ENABLE(); \
        } else { \
          INT##n##_DISABLE(); \
        } \
      }

  #define __enablepadevent_legacy_int_code(n) \
      if (port == INT##n##_IOPORT && pad == INT##n##_PAD) { \
        if (mode) { \
          if (mode == PAL_EVENT_MODE_FALLING_EDGE) { \
            INT##n##_EICR = (INT##n##_EICR & (1U << INT##n##_EICR_BASE)); \
          } else { \
            INT##n##_EICR = (INT##n##_EICR & (1U << INT##n##_EICR_BASE)) \
                | (1U << INT##n##_EICR_BASE); \
          } \
          INT##n##_ENABLE(); \
        } else { \
          INT##n##_DISABLE(); \
        } \
        return; \
      }

  #define _event_config(p) _event_config_pcint##p
  #define _pin_state(p) _pin_state_pcint##p
  #define _pin_get(p) PCINT##p##_GET()

  #define __enablepadevent_pcint_code(n, p) \
      if (port == IOPORT##n) { \
        const uint8_t padmask = (1U << IOPORT##n##_PCINT_OFFSET(pad)); \
        if (mode) { \
          const uint8_t rising = \
              (mode & PAL_EVENT_MODE_RISING_EDGE) ? padmask : 0; \
          const uint8_t falling = \
              (mode & PAL_EVENT_MODE_FALLING_EDGE) ? padmask : 0; \
          _event_config(p).rising = \
              (_event_config(p).rising & ~padmask) | rising; \
          _event_config(p).falling = \
              (_event_config(p).falling & ~padmask) | falling; \
          _pin_state(p) = \
              (_pin_state(p) & ~padmask) | (_pin_get(p) & padmask); \
          IOPORT##n##_PCMSK |= padmask; \
        } else { \
          IOPORT##n##_PCMSK &= ~padmask; \
        } \
        return; \
      }

  #define _enablepadevent_pcint_code(n) \
      __enablepadevent_pcint_code(n, IOPORT##n##_PCINT)

#if INT0_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(0);
#endif

#if INT1_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(1);
#endif

#if INT2_EVENT_COUNT || defined(__DOXYGEN__)
#if defined(INT2_LEGACY)
  __enablepadevent_legacy_int_code(2);
#else
  __enablepadevent_int_code(2);
#endif
#endif

#if INT3_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(3);
#endif

#if INT4_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(4);
#endif

#if INT5_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(5);
#endif

#if INT6_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(6);
#endif

#if INT7_EVENT_COUNT || defined(__DOXYGEN__)
  __enablepadevent_int_code(7);
#endif

#if defined(IOPORT1_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(1);
#endif

#if defined(IOPORT2_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(2);
#endif

#if defined(IOPORT3_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(3);
#endif

#if defined(IOPORT4_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(4);
#endif

#if defined(IOPORT5_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(5);
#endif

#if defined(IOPORT8_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(8);
#endif

#if defined(IOPORT10_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(10);
#endif

#if defined(IOPORT11_PCMSK) || defined(__DOXYGEN__)
  _enablepadevent_pcint_code(11);
#endif
}

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
void _pal_lld_disablepadevent(ioportid_t port, iopadid_t pad) {
  #define _disablepadevent_int_code(n) \
      if (port == INT##n##_IOPORT && pad == INT##n##_PAD) { \
        INT##n##_DISABLE(); \
        return; \
      }

  #define _disablepadevent_pcint_code(n) \
      if (port == IOPORT##n) { \
        IOPORT##n##_PCMSK &= ~(1U << IOPORT##n##_PCINT_OFFSET(pad)); \
        return; \
      }

#if INT0_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(0);
#endif

#if INT1_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(1);
#endif

#if INT2_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(2);
#endif

#if INT3_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(3);
#endif

#if INT4_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(4);
#endif

#if INT5_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(5);
#endif

#if INT6_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(6);
#endif

#if INT7_EVENT_COUNT || defined(__DOXYGEN__)
  _disablepadevent_int_code(7);
#endif

#if defined(IOPORT1_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(1);
#endif

#if defined(IOPORT2_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(2);
#endif

#if defined(IOPORT3_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(3);
#endif

#if defined(IOPORT4_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(4);
#endif

#if defined(IOPORT5_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(5);
#endif

#if defined(IOPORT8_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(8);
#endif

#if defined(IOPORT10_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(10);
#endif

#if defined(IOPORT11_PCMSK) || defined(__DOXYGEN__)
  _disablepadevent_pcint_code(11);
#endif
}

/**
 * @brief   Returns a PAL event structure associated to a pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
palevent_t* _pal_lld_get_pad_event(ioportid_t port, iopadid_t pad) {
  #define _get_pad_event_int_code(n) \
      if (port == INT##n##_IOPORT && pad == INT##n##_PAD) { \
        return &_pal_events[INT##n##_BASE]; \
      }

  #define ___get_pad_event_pcint_code(n, p) \
      if (port == IOPORT##n) { \
        if (IOPORT##n##_PCINT_PAD_VALID(pad)) { \
          const uint8_t offset = IOPORT##n##_PCINT_OFFSET(pad); \
          return &_pal_events[PCINT##p##_BASE + offset]; \
        } \
      }

  #define __get_pad_event_pcint_code(n, p) \
      ___get_pad_event_pcint_code(n, p)

  #define _get_pad_event_pcint_code(n) \
      __get_pad_event_pcint_code(n, IOPORT##n##_PCINT)

#if INT0_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(0);
#endif

#if INT1_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(1);
#endif

#if INT2_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(2);
#endif

#if INT3_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(3);
#endif

#if INT4_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(4);
#endif

#if INT5_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(5);
#endif

#if INT6_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(6);
#endif

#if INT7_EVENT_COUNT || defined(__DOXYGEN__)
  _get_pad_event_int_code(7);
#endif

#if defined(IOPORT1_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(1);
#endif

#if defined(IOPORT2_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(2);
#endif

#if defined(IOPORT3_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(3);
#endif

#if defined(IOPORT4_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(4);
#endif

#if defined(IOPORT5_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(5);
#endif

#if defined(IOPORT8_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(8);
#endif

#if defined(IOPORT10_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(10);
#endif

#if defined(IOPORT11_PCMSK) || defined(__DOXYGEN__)
  _get_pad_event_pcint_code(11);
#endif

  return NULL;
}

/**
 * @brief   Pad event enable check.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @return              Pad event status.
 * @retval false        if the pad event is disabled.
 * @retval true         if the pad event is enabled.
 *
 * @notapi
 */
bool pal_lld_ispadeventenabled(ioportid_t port, iopadid_t pad) {
  #define _ispadeventenabled_int_code(n) \
      if (port == INT##n##_IOPORT && pad == INT##n##_PAD) { \
        return INT##n##_ISENABLED(); \
      }

  #define _ispadeventenabled_pcint_code(n) \
      if (port == IOPORT##n) { \
        return (IOPORT##n##_PCMSK & \
            (1U << IOPORT##n##_PCINT_OFFSET(pad))) != 0; \
      }

#if INT0_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(0);
#endif

#if INT1_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(1);
#endif

#if INT2_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(2);
#endif

#if INT3_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(3);
#endif

#if INT4_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(4);
#endif

#if INT5_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(5);
#endif

#if INT6_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(6);
#endif

#if INT7_EVENT_COUNT || defined(__DOXYGEN__)
  _ispadeventenabled_int_code(7);
#endif

#if defined(IOPORT1_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(1);
#endif

#if defined(IOPORT2_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(2);
#endif

#if defined(IOPORT3_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(3);
#endif

#if defined(IOPORT4_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(4);
#endif

#if defined(IOPORT5_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(5);
#endif

#if defined(IOPORT8_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(8);
#endif

#if defined(IOPORT10_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(10);
#endif

#if defined(IOPORT11_PCMSK) || defined(__DOXYGEN__)
  _ispadeventenabled_pcint_code(11);
#endif

  return false;
}
#endif /* PAL_USE_CALLBACKS || PAL_USE_WAIT */

/**
 * @brief   Make a line identifier with a given port and pad identifiers.
 *
 * @param[in] port  the port identifier
 * @param[in] pad   the pad identifier
 *
 * @return    line  the builded line
 *
 * @notapi
 */
ioline_t _pal_lld_setlineid(ioportid_t port, uint8_t pad) {

  ioline_t line;

  line.port = port;
  line.pad  = pad;

  return line;
}

/**
 * @brief   Get a port identifier from a line identifier.
 *
 * @param[in] line  the line identifier
 *
 * @return    port  the port of the corresponding line
 *
 * @notapi
 */
ioportid_t _pal_lld_getportfromline(ioline_t line) {

  return line.port;
}

/**
 * @brief   Get a pad identifier from a line identifier.
 *
 * @param[in] line  the line identifier
 *
 * @return    pad   the pad of the corresponding line
 *
 * @notapi
 */
uint8_t _pal_lld_getpadfromline(ioline_t line) {

  return line.pad;
}

#endif /* HAL_USE_PAL */

/** @} */
