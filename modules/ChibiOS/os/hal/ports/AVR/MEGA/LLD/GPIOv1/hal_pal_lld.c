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

/**
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   AVR/MEGA PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
/**
 * @brief   Event records for the 16 GPIO EXTI channels.
 */
palevent_t _pal_events[16];
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.                                               */
/*==========================================================================*/

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
  (void)port;
  (void)pad;
  (void)mode;

  /* TODO: Implement the interruption here. */
}

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

