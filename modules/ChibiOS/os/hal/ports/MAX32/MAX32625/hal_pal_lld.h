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
 * @file    MAX32625/hal_pal_lld.h
 * @brief   MAX32625 PAL low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#include "max32_gpio.h"

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
 * @name    MAX32-specific I/O mode flags
 * @{
 */
#define PAL_MAX32_OMODE_OFF                     0U
#define PAL_MAX32_OMODE_MASK                    (0xFU << PAL_MAX32_OMODE_OFF)
#define PAL_MAX32_OMODE_HIGHZ_PULLUP            (0x0U << PAL_MAX32_OMODE_OFF)
#define PAL_MAX32_OMODE_OPENDRAIN               (0x1U << PAL_MAX32_OMODE_OFF)  
#define PAL_MAX32_OMODE_OPENDRAIN_PULLUP        (0x2U << PAL_MAX32_OMODE_OFF) 
#define PAL_MAX32_OMODE_NORMAL_HIGHZ            (0x4U << PAL_MAX32_OMODE_OFF)
#define PAL_MAX32_OMODE_NORMAL_DRIVE            (0x5U << PAL_MAX32_OMODE_OFF)   
#define PAL_MAX32_OMODE_SLOW_HIGHZ              (0x6U << PAL_MAX32_OMODE_OFF)  
#define PAL_MAX32_OMODE_SLOW_DRIVE              (0x7U << PAL_MAX32_OMODE_OFF)  
#define PAL_MAX32_OMODE_FAST_HIGHZ              (0x8U << PAL_MAX32_OMODE_OFF)  
#define PAL_MAX32_OMODE_FAST_DRIVE              (0x9U << PAL_MAX32_OMODE_OFF)  
#define PAL_MAX32_OMODE_HIGHZ_PULLDOWN          (0xAU << PAL_MAX32_OMODE_OFF) 
#define PAL_MAX32_OMODE_OPENSOURCE              (0xBU << PAL_MAX32_OMODE_OFF) 
#define PAL_MAX32_OMODE_OPENSOURCE_PULLDOWN     (0xCU << PAL_MAX32_OMODE_OFF) 
#define PAL_MAX32_OMODE_DISABLED                (0xFU << PAL_MAX32_OMODE_OFF)

#define PAL_MAX32_FUNC_OFF                      4U
#define PAL_MAX32_FUNC_MASK                     (0xFU << PAL_MAX32_FUNC_OFF)
#define PAL_MAX32_FUNC_GPIO                     (0x0U << PAL_MAX32_FUNC_OFF)
#define PAL_MAX32_FUNC_PULSE                    (0x1U << PAL_MAX32_FUNC_OFF)
#define PAL_MAX32_FUNC_TIMER                    (0x2U << PAL_MAX32_FUNC_OFF)

#define PAL_MAX32_IMODE_OFF                     8U
#define PAL_MAX32_IMODE_MASK                    (0x3U << PAL_MAX32_IMODE_OFF)
#define PAL_MAX32_IMODE_NORMAL_INPUT            (0x0U << PAL_MAX32_IMODE_OFF)  
#define PAL_MAX32_IMODE_INVERTED_INPUT          (0x1U << PAL_MAX32_IMODE_OFF)  
#define PAL_MAX32_IMODE_ALWAYS_LOW              (0x2U << PAL_MAX32_IMODE_OFF)  
#define PAL_MAX32_IMODE_ALWAYS_HIGH             (0x3U << PAL_MAX32_IMODE_OFF)


/**
 * @name    Standard I/O mode flags
 * @{
 */
/**
 * @brief   Implemented as input with pull-up.
 */
#define PAL_MODE_RESET                  (PAL_MAX32_OMODE_DISABLED       |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Implemented as High-Z.
 */
#define PAL_MODE_UNCONNECTED            (PAL_MAX32_OMODE_DISABLED       |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Regular input floating.
 */
#define PAL_MODE_INPUT                  (PAL_MAX32_OMODE_NORMAL_HIGHZ   |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Input pad with weak pull up resistor.
 */
#define PAL_MODE_INPUT_PULLUP           (PAL_MAX32_OMODE_HIGHZ_PULLUP   |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Not available: High-Z.
 * @note    Input floating here could be misleading.
 */
#define PAL_MODE_INPUT_PULLDOWN         (PAL_MAX32_OMODE_HIGHZ_PULLDOWN |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Not available: High-Z.
 * @note    Input floating here could be misleading.
 */
#define PAL_MODE_INPUT_ANALOG           (PAL_MAX32_OMODE_NORMAL_HIGHZ   |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Push-pull output pad.
 */
#define PAL_MODE_OUTPUT_PUSHPULL        (PAL_MAX32_OMODE_NORMAL_DRIVE   |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)

/**
 * @brief   Not available: High-Z.
 * @note    Standard output could create short-circuit.
 */
#define PAL_MODE_OUTPUT_OPENDRAIN       (PAL_MAX32_OMODE_OPENDRAIN      |\
                                         PAL_MAX32_FUNC_GPIO            |\
                                         PAL_MAX32_IMODE_NORMAL_INPUT)
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
  ((ioline_t)((uint32_t)(port)) | ((uint32_t)(pad << 16U)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  ((max32_gpio_t *)(((uint32_t)(line)) & 0xFFF0FFFFU))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((uint32_t)(((uint32_t)(line) & 0x000F0000)) >> 16U)
  
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
typedef max32_gpio_t * ioportid_t;

/**
 * @brief   Type of an pad identifier.
 */
typedef uint32_t iopadid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/* The low level driver wraps the definitions already present in the MAX32   */
/* firmware library.                                                         */
/*===========================================================================*/

/**
 * @brief   GPIO port 0 identifier.
 */
#define IOPORT0         GPIO0

/**
 * @brief   GPIO port 1 identifier.
 */
#define IOPORT1         GPIO1

/**
 * @brief   GPIO port 2 identifier.
 */
#define IOPORT2         GPIO2

/**
 * @brief   GPIO port 3 identifier.
 */
#define IOPORT3         GPIO3

/**
 * @brief   GPIO port 4 identifier.
 */
#define IOPORT4         GPIO4

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
#define pal_lld_readport(port) ((ioportmask_t)((port)->IN_VAL))

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
#define pal_lld_readlatch(port) ((ioportmask_t)((port)->OUT_VAL))

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
#define pal_lld_writeport(port, bits) ((port)->OUT_VAL = (uint32_t)(bits))

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
  (port)->OUT_VAL = w;                                                      \
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
  void pal_lld_setport(ioportid_t port, uint32_t bits);
  void pal_lld_clearport(ioportid_t port, uint32_t bits);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H */

/** @} */
