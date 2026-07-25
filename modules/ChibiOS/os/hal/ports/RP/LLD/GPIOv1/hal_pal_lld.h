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
 * @file    GPIOv1/hal_pal_lld.h
 * @brief   RP PAL low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

/* Specifies palInit() without parameter, required until all platforms will
   be updated to the new style.*/
#define PAL_NEW_INIT

#undef PAL_MODE_RESET
#undef PAL_MODE_UNCONNECTED
#undef PAL_MODE_INPUT
#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_PUSHPULL
#undef PAL_MODE_OUTPUT_OPENDRAIN

/**
 * @name    RP-specific I/O mode flags
 * @{
 */
#define PAL_RP_IOCTRL_INOVER_NOTINV         (0U << 16)
#define PAL_RP_IOCTRL_INOVER_INV            (1U << 16)
#define PAL_RP_IOCTRL_INOVER_DRVLOW         (2U << 16)
#define PAL_RP_IOCTRL_INOVER_DRVHIGH        (3U << 16)

#define PAL_RP_IOCTRL_OEOVER_DRVPERI        (0U << 12)
#define PAL_RP_IOCTRL_OEOVER_DRVINVPERI     (1U << 12)
#define PAL_RP_IOCTRL_OEOVER_DISABLE        (2U << 12)
#define PAL_RP_IOCTRL_OEOVER_ENABLE         (3U << 12)

#define PAL_RP_IOCTRL_OUTOVER_DRVPERI       (0U << 8)
#define PAL_RP_IOCTRL_OUTOVER_DRVINVPERI    (1U << 8)
#define PAL_RP_IOCTRL_OUTOVER_DRVLOW        (2U << 8)
#define PAL_RP_IOCTRL_OUTOVER_DRVHIGH       (3U << 8)

#define PAL_RP_IOCTRL_FUNCSEL(n)            ((n) << 0)
#define PAL_RP_IOCTRL_FUNCSEL_SPI           PAL_RP_IOCTRL_FUNCSEL(1U)
#define PAL_RP_IOCTRL_FUNCSEL_UART          PAL_RP_IOCTRL_FUNCSEL(2U)
#define PAL_RP_IOCTRL_FUNCSEL_I2C           PAL_RP_IOCTRL_FUNCSEL(3U)
#define PAL_RP_IOCTRL_FUNCSEL_PWM           PAL_RP_IOCTRL_FUNCSEL(4U)
#define PAL_RP_IOCTRL_FUNCSEL_SIO           PAL_RP_IOCTRL_FUNCSEL(5U)
#define PAL_RP_IOCTRL_FUNCSEL_PIO0          PAL_RP_IOCTRL_FUNCSEL(6U)
#define PAL_RP_IOCTRL_FUNCSEL_PIO1          PAL_RP_IOCTRL_FUNCSEL(7U)
#define PAL_RP_IOCTRL_FUNCSEL_USB           PAL_RP_IOCTRL_FUNCSEL(9U)
#define PAL_RP_IOCTRL_FUNCSEL_NULL          PAL_RP_IOCTRL_FUNCSEL(31U)

#define PAL_RP_GPIO_OE                      (1U << (23 + 0))

#define PAL_RP_PAD_OD                       (1U << (24 + 7))

#define PAL_RP_PAD_IE                       (1U << (24 + 6))

#define PAL_RP_PAD_DRIVE2                   (0U << (24 + 4))
#define PAL_RP_PAD_DRIVE4                   (1U << (24 + 4))
#define PAL_RP_PAD_DRIVE8                   (2U << (24 + 4))
#define PAL_RP_PAD_DRIVE12                  (3U << (24 + 4))

#define PAL_RP_PAD_PUE                      (1U << (24 + 3))

#define PAL_RP_PAD_PDE                      (1U << (24 + 2))

#define PAL_RP_PAD_SCHMITT                  (1U << (24 + 1))

#define PAL_RP_PAD_SLEWFAST                 (1U << (24 + 0))
/** @} */

/**
 * @name    RP-specific I/O event flags
 * @{
 */
#define RP_PAL_EVENT_MODE_LEVEL_LOW         4U
#define RP_PAL_EVENT_MODE_LEVEL_HIGH        8U
/** @} */

/**
 * @name    Alternate functions
 * @{
 * @param[in] n         alternate function selector
 */
#define PAL_MODE_ALTERNATE(n)               (PAL_RP_IOCTRL_FUNCSEL(n)   |   \
                                             PAL_RP_PAD_IE              |   \
                                             PAL_RP_PAD_SCHMITT)
#define PAL_MODE_ALTERNATE_SPI              (PAL_MODE_ALTERNATE(1U))
#define PAL_MODE_ALTERNATE_UART             (PAL_MODE_ALTERNATE(2U))
#define PAL_MODE_ALTERNATE_I2C              (PAL_MODE_ALTERNATE(3U))
#define PAL_MODE_ALTERNATE_PWM              (PAL_MODE_ALTERNATE(4U))
#define PAL_MODE_ALTERNATE_SIO              (PAL_MODE_ALTERNATE(5U))
#define PAL_MODE_ALTERNATE_PIO0             (PAL_MODE_ALTERNATE(6U))
#define PAL_MODE_ALTERNATE_PIO1             (PAL_MODE_ALTERNATE(7U))
#define PAL_MODE_ALTERNATE_CLK              (PAL_MODE_ALTERNATE(8U))
#define PAL_MODE_ALTERNATE_USB              (PAL_MODE_ALTERNATE(9U))
/** @} */

/**
 * @name    Standard I/O mode flags
 * @{
 */
/**
 * @brief   Implemented as post-reset state.
 */
#define PAL_MODE_RESET                  (PAL_RP_IOCTRL_FUNCSEL_NULL     |   \
                                         PAL_RP_PAD_DRIVE4              |   \
                                         PAL_RP_PAD_IE                  |   \
                                         PAL_RP_PAD_SCHMITT)

/**
 * @brief   Implemented as input with pull-up.
 */
#define PAL_MODE_UNCONNECTED            (PAL_RP_IOCTRL_FUNCSEL_SIO      |   \
                                         PAL_RP_PAD_IE                  |   \
                                         PAL_RP_PAD_SCHMITT             |   \
                                         PAL_RP_PAD_PUE)

/**
 * @brief   Regular input high-Z pad.
 */
#define PAL_MODE_INPUT                  (PAL_RP_IOCTRL_FUNCSEL_SIO      |   \
                                         PAL_RP_PAD_IE                  |   \
                                         PAL_RP_PAD_SCHMITT)

/**
 * @brief   Input pad with weak pull up resistor.
 */
#define PAL_MODE_INPUT_PULLUP           (PAL_RP_IOCTRL_FUNCSEL_SIO      |   \
                                         PAL_RP_PAD_IE                  |   \
                                         PAL_RP_PAD_SCHMITT             |   \
                                         PAL_RP_PAD_PUE)

/**
 * @brief   Input pad with weak pull down resistor.
 */
#define PAL_MODE_INPUT_PULLDOWN         (PAL_RP_IOCTRL_FUNCSEL_SIO      |   \
                                         PAL_RP_PAD_IE                  |   \
                                         PAL_RP_PAD_SCHMITT             |   \
                                         PAL_RP_PAD_PDE)

/**
 * @brief   Analog input mode.
 */
#define PAL_MODE_INPUT_ANALOG           (PAL_RP_IOCTRL_FUNCSEL_NULL)

/**
 * @brief   Push-pull output pad.
 */
#define PAL_MODE_OUTPUT_PUSHPULL        (PAL_RP_IOCTRL_FUNCSEL_SIO      |   \
                                         PAL_RP_GPIO_OE                 |   \
                                         PAL_RP_PAD_IE)
/** @} */

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @name    Port related definitions
 * @{
 */
/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 32

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFFFFFF)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 * @note    In this driver the pad number is encoded in the lower 4 bits of
 *          the GPIO address which are guaranteed to be zero.
 */
#define PAL_LINE(port, pad)             ((pad), (port))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                  0U

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                   (line)

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

/**
 * @brief   Type of digital I/O port sized unsigned integer.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Type of digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Type of an event mode.
 */
typedef uint32_t ioeventmode_t;

/**
 * @brief   Type of a port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef uint32_t ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint32_t iopadid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/* The low level driver wraps the definitions already present in the STM32   */
/* firmware library.                                                         */
/*===========================================================================*/

/**
 * @brief   User port identifier.
 */
#define IOPORT1                         0U

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   GPIO ports subsystem initialization.
 *
 * @notapi
 */
#define pal_lld_init()                  __pal_lld_init()

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port)          (SIO->GPIO_IN)

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
#define pal_lld_readlatch(port)         (SIO->GPIO_OUT)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits)                                       \
  do {                                                                      \
    (void)port;                                                             \
    SIO->GPIO_OUT = (bits);                                                 \
  } while (false)

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits)                                         \
  do {                                                                      \
    (void)port;                                                             \
    SIO->GPIO_OUT_SET = (bits);                                             \
  } while (false)

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits)                                       \
  do {                                                                      \
    (void)port;                                                             \
    SIO->GPIO_OUT_CLR = (bits);                                             \
  } while (false)

/**
 * @brief   Toggles a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be XORed on the specified port
 *
 * @notapi
 */
#define pal_lld_toggleport(port, bits)                                      \
  do {                                                                      \
    (void)port;                                                             \
    SIO->GPIO_OUT_XOR = (bits);                                             \
  } while (false)

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The operation is not guaranteed to be atomic on all the
 *          architectures, for atomicity and/or portability reasons you may
 *          need to enclose port I/O operations between @p osalSysLock() and
 *          @p osalSysUnlock().
 * @note    Programming an unknown or unsupported mode is silently ignored.
 * @note    The function can be called from any context.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */
#define pal_lld_setpadmode(port, pad, mode)                                 \
  __pal_lld_pad_set_mode(port, pad, mode)

__STATIC_INLINE void __pal_lld_pad_set_mode(ioportid_t port,
                                            iopadid_t pad,
                                            iomode_t mode) {
  uint32_t ctrlbits, padbits, oebits;

  (void)port;

  ctrlbits = (mode & 0x007FFFFFU) >> 0U;
  oebits   = (mode & 0x00800000U) >> 23U;
  padbits  = (mode & 0xFF000000U) >> 24U;

  /* Setting up GPIO direction first.*/
  if (oebits != 0U) {
    SIO->GPIO_OE_SET = 1U << pad;
  }
  else {
    SIO->GPIO_OE_CLR = 1U << pad;
  }

  /* Then IO and PAD settings.*/
  IO_BANK0->GPIO[pad].CTRL = ctrlbits;
  PADS_BANK0->GPIO[pad] = padbits;
}


#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)

#if !defined(RP_PAL_EVENT_CORE_AFFINITY)
  /** @brief Core that handles all the PAL event interrupts. */
  #define RP_PAL_EVENT_CORE_AFFINITY        0
#endif

#if !defined(RP_IO_IRQ_BANK0_PRIORITY)
  /** @brief IRQ priority of the external pad events. */
  #define RP_IO_IRQ_BANK0_PRIORITY          2
#endif

#if !defined(__DOXYGEN__)
extern palevent_t _pal_events[30];
#endif

/**
 * @brief   Returns a PAL event structure associated to a line.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
#define pal_lld_get_line_event(line)                                        \
  &_pal_events[line]

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
 * @brief   Line event enable.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] line      line number
 * @param[in] mode      line event mode
 *
 * @notapi
 */
#define pal_lld_enablelineevent(line, mode)                             \
  _pal_lld_enablelineevent(line, mode)

/**
 * @brief   Line event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
#define pal_lld_disablelineevent(line)                                  \
  _pal_lld_disablelineevent(line)

/**
 * @brief Force a trigger event on the line.
 * 
 * @param[in]         line identifier
 * 
 * @notapi
 */
#define pal_lld_forcelineevent(line)                                    \
  _pal_lld_forcelineevent(line)

/**
 * @brief Clear any previously forced trigger on the line.
 * 
 * @param[in]         line identifier
 * 
 * @notapi
 */
#define pal_lld_unforcelineevent(line)                                    \
  _pal_lld_unforcelineevent(line)

#endif /* (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE) */

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_enablelineevent(ioline_t line, ioeventmode_t mode);
  void _pal_lld_disablelineevent(ioline_t line);
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
extern "C" {
#endif
  void __pal_lld_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H */

/** @} */
