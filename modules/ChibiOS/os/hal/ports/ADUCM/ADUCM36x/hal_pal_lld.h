/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    ADUCM36x/hal_pal_lld.h
 * @brief   ADUCM36x PAL low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#include "aducm_gp.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* IRQ priority checks.*/
#if PAL_USE_CALLBACKS || PAL_USE_WAIT
#error "External interrupt still unsupported on this platform."
#endif

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
 * @name    ADUCM-specific I/O mode flags
 * @{
 */
#define PAL_ADUCM_CON_MASK              (3U << 0U)
#define PAL_ADUCM_CON_OFF               0U
#define PAL_ADUCM_CON(n)                (((n) & 3U) << 0U)

#define PAL_ADUCM_OEN_MASK              (1U << 2U)
#define PAL_ADUCM_OEN_OFF               2U
#define PAL_ADUCM_OEN_INPUT             (0U << 2U)
#define PAL_ADUCM_OEN_OUTPUT            (1U << 2U)

#define PAL_ADUCM_PUL_MASK              (1U << 3U)
#define PAL_ADUCM_PUL_OFF               3U
#define PAL_ADUCM_PUL_FLOATING          (0U << 3U)
#define PAL_ADUCM_PUL_PULLUP            (1U << 3U)

#define PAL_ADUCM_OCE_MASK              (1U << 4U)
#define PAL_ADUCM_OCE_OFF               4U
#define PAL_ADUCM_OCE_NORMAL            (0U << 4U)
#define PAL_ADUCM_OCE_HIGHZ             (1U << 4U)

/**
 * @brief   Connection multiplexer.
 *
 * @param[in] n         connection multiplexer selector
 */
#define PAL_MODE_MULTIPLEXER(n)         PAL_ADUCM_CON(n)
/** @} */

/**
 * @name    Standard I/O mode flags
 * @{
 */
/**
 * @brief   Implemented as input with pull-up.
 */
#define PAL_MODE_RESET                  (PAL_ADUCM_OEN_INPUT |              \
                                         PAL_ADUCM_PUL_PULLUP)

/**
 * @brief   Implemented as High-Z.
 */
#define PAL_MODE_UNCONNECTED            PAL_ADUCM_OCE_HIGHZ

/**
 * @brief   Regular input floating.
 */
#define PAL_MODE_INPUT                  (PAL_ADUCM_OEN_INPUT |              \
                                         PAL_ADUCM_PUL_FLOATING)

/**
 * @brief   Input pad with weak pull up resistor.
 */
#define PAL_MODE_INPUT_PULLUP           (PAL_ADUCM_OEN_INPUT |              \
                                         PAL_ADUCM_PUL_PULLUP)

/**
 * @brief   Not available: High-Z.
 * @note    Input floating here could be misleading.
 */
#define PAL_MODE_INPUT_PULLDOWN         PAL_ADUCM_OCE_HIGHZ

/**
 * @brief   Not available: High-Z.
 * @note    Input floating here could be misleading.
 */
#define PAL_MODE_INPUT_ANALOG           PAL_ADUCM_OCE_HIGHZ

/**
 * @brief   Push-pull output pad.
 */
#define PAL_MODE_OUTPUT_PUSHPULL        PAL_ADUCM_OEN_OUTPUT

/**
 * @brief   Not available: High-Z.
 * @note    Standard output could create short-circuit.
 */
#define PAL_MODE_OUTPUT_OPENDRAIN       PAL_ADUCM_OCE_HIGHZ
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
#define PAL_IOPORTS_WIDTH 8

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFF)
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
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)((uint32_t)(port)) | ((uint32_t)(pad)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  ((aducm_gp_t *)(((uint32_t)(line)) & 0xFFFFFFF0U))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((uint32_t)((uint32_t)(line) & 0x0000000FU))

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
typedef aducm_gp_t * ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint32_t iopadid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/* The low level driver wraps the definitions already present in the ADUCM   */
/* firmware library.                                                         */
/*===========================================================================*/

/**
 * @brief   GP port 0 identifier.
 */
#define IOPORT0         GP0

/**
 * @brief   GP port 1 identifier.
 */
#define IOPORT1         GP1

/**
 * @brief   GP port 2 identifier.
 */
#define IOPORT2         GP2

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   GPIO ports subsystem initialization.
 *
 * @notapi
 */
#define pal_lld_init() _pal_lld_init()

/**
 * @brief   Reads an I/O port.
 * @details This function is implemented by reading the GPIN register, the
 *          implementation has no side effects.
 * @note    This function is not meant to be invoked directly by the application
 *          code.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) ((ioportmask_t)((port)->IN))

/**
 * @brief   Reads the output latch.
 * @details This function is implemented by reading the GPOUT register, the
 *          implementation has no side effects.
 * @note    This function is not meant to be invoked directly by the application
 *          code.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port) ((ioportmask_t)((port)->OUT))

/**
 * @brief   Writes on a I/O port.
 * @details This function is implemented by writing the GPOUT register, the
 *          implementation has no side effects.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) ((port)->OUT = (uint32_t)(bits))

/**
 * @brief   Sets a bits mask on a I/O port.
 * @details This function is implemented by writing the GPSET register, the
 *          implementation has no side effects.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits) ((port)->SET = (uint8_t)(bits))

/**
 * @brief   Clears a bits mask on a I/O port.
 * @details This function is implemented by writing the GPCLR register, the
 *          implementation has no side effects.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits) ((port)->CLR = (uint8_t)(bits))

/**
 * @brief   Writes a group of bits.
 * @details This function is implemented by writing the GPOUT register, the
 *          implementation has no side effects.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    the group bit offset within the port
 * @param[in] bits      bits to be written. Values exceeding the group
 *                      width are masked.
 *
 * @notapi
 */
#define pal_lld_writegroup(port, mask, offset, bits) {                      \
  uint8_t w = ((uint8_t)(bits) & (uint8_t)(mask)) << (offset);              \
  (port)->OUT = w;                                                          \
}

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
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
 * @brief   Writes a logical state on an output pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] bit       logical value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 *
 * @notapi
 */
#define pal_lld_writepad(port, pad, bit) pal_lld_writegroup(port, 1, pad, bit)

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(void);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H */

/** @} */
