/*
    ChibiOS - Copyright (C) 2017 Theodore Ateba

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
 * @file    GPIOv1/hal_pal_lld.h
 * @brief   AVR XMEGA GPIO low level driver header file.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*==========================================================================*/
/* Unsupported modes and specific modes.                                    */
/*==========================================================================*/

#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_OUTPUT_OPENDRAIN

/*==========================================================================*/
/* I/O Ports Types and constants.                                           */
/*==========================================================================*/

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 8

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFF)

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 * @note    In this driver the pad number and the port identifier are
 *          encoded in a structure of type ioline_t.
 */
#define PAL_LINE(port, pad) _pal_lld_setlineid(port, pad)

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line) _pal_lld_getportfromline(line)

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line) _pal_lld_getpadfromline(line)

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

/**
 * @brief   AVR setup registers.
 */
typedef struct {
  uint8_t  out;
  uint8_t  dir;
} avr_gpio_setup_t;

/**
 * @brief   AVR registers block.
 * @note    On some devices registers do not follow this layout on some
 *          ports, the ports with abnormal layout cannot be used through
 *          PAL driver. Example: PORT F on Mega128.
 */
typedef struct {
  volatile uint8_t  in;
  volatile uint8_t  dir;
  volatile uint8_t  out;
} avr_gpio_registers_t;

/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 */
typedef struct {
#if defined(PORTA) || defined(__DOXYGEN__)
  avr_gpio_setup_t porta;
#endif
#if defined(PORTB) || defined(__DOXYGEN__)
  avr_gpio_setup_t portb;
#endif
#if defined(PORTC) || defined(__DOXYGEN__)
  avr_gpio_setup_t portc;
#endif
#if defined(PORTD) || defined(__DOXYGEN__)
  avr_gpio_setup_t portd;
#endif
#if defined(PORTE) || defined(__DOXYGEN__)
  avr_gpio_setup_t porte;
#endif
#if defined(PORTF) || defined(__DOXYGEN__)
  avr_gpio_setup_t portf;
#endif
#if defined(PORTG) || defined(__DOXYGEN__)
  avr_gpio_setup_t portg;
#endif
#if defined(PORTH) || defined(__DOXYGEN__)
  avr_gpio_setup_t porth;
#endif
#if defined(PORTJ) || defined(__DOXYGEN__)
  avr_gpio_setup_t portj;
#endif
#if defined(PORTK) || defined(__DOXYGEN__)
  avr_gpio_setup_t portk;
#endif
#if defined(PORTL) || defined(__DOXYGEN__)
  avr_gpio_setup_t portl;
#endif
} PALConfig;

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint8_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint8_t iomode_t;

/**
 * @brief   Port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef volatile PORT_t * ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint8_t iopadid_t;

/**
 * @brief   Type of an I/O line.
 */
typedef struct {
  ioportid_t  port; /* Line port identifier.  */
  iopadid_t   pad;  /* Line pad identifier.   */
} ioline_t;

/**
 * @brief   Type of an event mode.
 */
typedef uint8_t ioeventmode_t;

/*==========================================================================*/
/* I/O Ports Identifiers.                                                   */
/*==========================================================================*/

#if defined(PORTA) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port A identifier.
 */
#define IOPORT1     (&PORTA)
#endif

#if defined(PORTB) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port B identifier.
 */
#define IOPORT2     (&PORTB)
#endif

#if defined(PORTC) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port C identifier.
 */
#define IOPORT3     (&PORTC)
#endif

#if defined(PORTD) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port D identifier.
 */
#define IOPORT4     (&PORTD)
#endif

#if defined(PORTE) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port E identifier.
 */
#define IOPORT5     (&PORTE)
#endif

#if defined(PORTF) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port F identifier.
 */
#define IOPORT6     (&PORTF)
#endif

#if defined(PORTG) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port G identifier.
 */
#define IOPORT7     (&PORTG)
#endif

#if defined(PORTH) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port H identifier.
 */
#define IOPORT8     (&PORTH)
#endif

#if defined(PORTJ) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port J identifier.
 */
#define IOPORT9     (&PORTJ)
#endif

#if defined(PORTK) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port K identifier.
 */
#define IOPORT10    (&PORTK)
#endif

#if defined(PORTL) || defined(__DOXYGEN__)
/**
 * @brief   GPIO port L identifier.
 */
#define IOPORT11    (&PORTL)
#endif

/*==========================================================================*/
/* Implementation, some of the following macros could be implemented as     */
/* Functions, if so please put them in hal_pal_lld.c.                       */
/*==========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config the architecture-dependent ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) _pal_lld_init(config)

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) ((port)->IN)

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port) ((port)->OUT)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) ((port)->OUT = bits)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode(port, mask << offset, mode)

/**
 * @brief   Sets a pad logical state to @p PAL_HIGH.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_setpad(port, pad)     ((port)->OUTSET |= (1 << pad))

/**
 * @brief   Clears a pad logical state to @p PAL_LOW.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_clearpad(port, pad)   ((port)->OUTCLR |= (1 << pad))

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
#define pal_lld_enablepadevent(port, pad, mode, callback, arg)              \
  _pal_lld_enablepadevent(port, pad, mode, callback, arg)

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_disablepadevent(port, pad)                                  \
  _pal_lld_disablepadevent(port, pad)

/**
 * @brief   Returns a PAL event structure associated to a pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_get_pad_event(port, pad)                                    \
  &_pal_events[pad]; (void)(port)

/**
 * @brief   Returns a PAL event structure associated to a line.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
#define pal_lld_get_line_event(line)                                        \
  &_pal_events[PAL_PAD(line)]

#if !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
extern palevent_t _pal_events[16];
#endif
#endif

extern ROMCONST PALConfig pal_default_config;

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
  ioline_t _pal_lld_setlineid(ioportid_t port, uint8_t pad);
  ioportid_t _pal_lld_getportfromline(ioline_t line);
  uint8_t _pal_lld_getpadfromline(ioline_t line);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H */

/** @} */
