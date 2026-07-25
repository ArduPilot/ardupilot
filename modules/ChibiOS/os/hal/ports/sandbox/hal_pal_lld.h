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
 * @file    sandbox/hal_pal_lld.h
 * @brief   Sanbox PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

#if PAL_USE_CALLBACKS
#error "PAL_USE_CALLBACKS not supported"
#endif

#if PAL_USE_WAIT
#error "PAL_USE_WAIT not supported"
#endif

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

/* Specifies palInit() without parameter, required until all platforms will
   be updated to the new style.*/
#define PAL_NEW_INIT

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
#define PAL_IOPORTS_WIDTH           32U

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT              ((ioportmask_t)0xFFFFFFFFU)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)((uint32_t)(port) << 8) | ((uint32_t)(pad)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  (ioportid_t)((uint32_t)(line) >> 8)

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((iopadid_t)((uint32_t)(line) & 0x000000FFU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0xFFFFFFFFU
/** @} */

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Port Identifier.
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
/*===========================================================================*/

/**
 * @name    Generic port identifiers.
 * @{
 */
#define IOPORT1         VPIO0
#define IOPORT2         VPIO1
#define IOPORT3         VPIO2
#define IOPORT4         VPIO3
#define IOPORT5         VPIO4
#define IOPORT6         VPIO5
#define IOPORT7         VPIO6
#define IOPORT8         VPIO7
/** @} */

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @notapi
 */
#define pal_lld_init()

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) __pal_lld_readport(port)

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
#define pal_lld_readlatch(port) __pal_lld_readlatch(port)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) __pal_lld_writeport(port, bits)

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
#define pal_lld_setport(port, bits) __pal_lld_setport(port, bits)

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
#define pal_lld_clearport(port, bits) __pal_lld_clearport(port, bits)

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
#define pal_lld_toggleport(port, bits) __pal_lld_toggleport(port, bits)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

__attribute__((always_inline))
static inline uint32_t __pal_lld_readport(ioportid_t port) {

  __syscall2r(96, SB_VGPIO_READ, port);
  return (ioportmask_t)r0;
}

__attribute__((always_inline))
static inline uint32_t __pal_lld_readlatch(ioportid_t port) {

  __syscall2r(96, SB_VGPIO_READLATCH, port);
  return (ioportmask_t)r0;
}

__attribute__((always_inline))
static inline void __pal_lld_writeport(ioportid_t port, uint32_t bits) {

  __syscall3r(96, SB_VGPIO_WRITE, port, bits);
}

__attribute__((always_inline))
static inline void __pal_lld_setport(ioportid_t port, uint32_t bits) {

  __syscall3r(96, SB_VGPIO_SET, port, bits);
}

__attribute__((always_inline))
static inline void __pal_lld_clearport(ioportid_t port, uint32_t bits) {

  __syscall3r(96, SB_VGPIO_CLEAR, port, bits);
}

__attribute__((always_inline))
static inline void __pal_lld_toggleport(ioportid_t port, uint32_t bits) {

  __syscall3r(96, SB_VGPIO_TOGGLE, port, bits);
}

#endif /* HAL_USE_PAL == TRUE */

#endif /* HAL_PAL_LLD_H */

/** @} */
