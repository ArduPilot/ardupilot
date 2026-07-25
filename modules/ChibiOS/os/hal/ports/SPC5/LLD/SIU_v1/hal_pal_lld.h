/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/SIU_v1/hal_pal_lld.h
 * @brief   SPC5xx SIU low level driver header.
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

#undef PAL_MODE_RESET
#undef PAL_MODE_UNCONNECTED
#undef PAL_MODE_INPUT
#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_PUSHPULL
#undef PAL_MODE_OUTPUT_OPENDRAIN

/**
 * @name    SIU-specific PAL modes
 * @{
 */
#define PAL_SPC5_PA_MASK            (15U << 10)
#define PAL_SPC5_PA_GPIO            (0U << 10)
#define PAL_SPC5_PA_PRIMARY         PAL_SPC5_PA(0)
#define PAL_SPC5_PA(n)              ((1U << (n)) << 10)
#define PAL_SPC5_OBE                (1U << 9)
#define PAL_SPC5_IBE                (1U << 8)
#define PAL_SPC5_DSC_MASK           (3U << 6)
#define PAL_SPC5_DSC_10PF           (0U << 6)
#define PAL_SPC5_DSC_20PF           (1U << 6)
#define PAL_SPC5_DSC_30PF           (2U << 6)
#define PAL_SPC5_DSC_50PF           (3U << 6)
#define PAL_SPC5_ODE                (1U << 5)
#define PAL_SPC5_HYS                (1U << 4)
#define PAL_SPC5_SRC_MASK           (3U << 2)
#define PAL_SPC5_SRC_MIN            (0U << 2)
#define PAL_SPC5_SRC_MID            (1U << 2)
#define PAL_SPC5_SRC_MAX            (3U << 2)
#define PAL_SPC5_WPE                (1U << 1)
#define PAL_SPC5_WPS                (1U << 0)
/** @} */

/**
 * @name    Pads mode constants
 * @{
 */
/**
 * @brief   After reset state.
 */
#define PAL_MODE_RESET                  0

/**
 * @brief   Safe state for <b>unconnected</b> pads.
 */
#define PAL_MODE_UNCONNECTED            (PAL_SPC5_WPE | PAL_SPC5_WPS)

/**
 * @brief   Regular input high-Z pad.
 */
#define PAL_MODE_INPUT                  (PAL_SPC5_IBE)

/**
 * @brief   Input pad with weak pull up resistor.
 */
#define PAL_MODE_INPUT_PULLUP           (PAL_SPC5_IBE | PAL_SPC5_WPE |      \
                                         PAL_SPC5_WPS)

/**
 * @brief   Input pad with weak pull down resistor.
 */
#define PAL_MODE_INPUT_PULLDOWN         (PAL_SPC5_IBE | PAL_SPC5_WPE)

/**
 * @brief   Push-pull output pad.
 */
#define PAL_MODE_OUTPUT_PUSHPULL        (PAL_SPC5_IBE | PAL_SPC5_OBE)

/**
 * @brief   Open-drain output pad.
 */
#define PAL_MODE_OUTPUT_OPENDRAIN       (PAL_SPC5_IBE | PAL_SPC5_OBE |      \
                                         PAL_SPC5_ODE)

/**
 * @brief   Primary function input pad.
 * @note    Both the IBE and OBE bits are specified in this mask.
 */
#define PAL_MODE_INPUT_ALTERNATE_PRIMARY                                    \
  PAL_MODE_INPUT_ALTERNATE(0)

/**
 * @brief   Alternate function input pad.
 * @note    Both the IBE and OBE bits are specified in this mask.
 */
#define PAL_MODE_INPUT_ALTERNATE(n)     (PAL_SPC5_IBE | PAL_SPC5_PA(n))

/**
 * @brief   Primary function output pad.
 * @note    Both the IBE and OBE bits are specified in this mask.
 */
#define PAL_MODE_OUTPUT_ALTERNATE_PRIMARY                                   \
  PAL_MODE_OUTPUT_ALTERNATE(0)

/**
 * @brief   Alternate function output pad.
 * @note    Both the IBE and OBE bits are specified in this mask.
 */
#define PAL_MODE_OUTPUT_ALTERNATE(n)    (PAL_SPC5_IBE | PAL_SPC5_OBE |      \
                                         PAL_SPC5_PA(n))
/** @} */

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 16

/**
 * @brief   Whole port mask.
 * @brief   This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFF)

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint16_t ioportmask_t;

/**
 * @brief   Port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef uint32_t ioportid_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint16_t iomode_t;

/**
 * @brief   SIU/SIUL register initializer type.
 */
typedef struct {
  int32_t                   pcr_index;
  uint8_t                   gpdo_value;
  iomode_t                  pcr_value;
} spc_siu_init_t;

/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  const spc_siu_init_t      *inits;
} PALConfig;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @name    Port identifiers
 * @{
 */
#define PORT0           0
#define PORT1           1
#define PORT2           2
#define PORT3           3
#define PORT4           4
#define PORT5           5
#define PORT6           6
#define PORT7           7
#define PORT8           8
#define PORT9           9
#define PORT10          10
#define PORT11          11
#define PORT12          12
#define PORT13          13
#define PORT14          14
#define PORT15          15
#define PORT16          16
#define PORT17          17
#define PORT18          18
#define PORT19          19
#define PORT20          20
#define PORT21          21
#define PORT22          22
#define PORT23          23
#define PORT24          24
#define PORT25          25
#define PORT26          26
#define PORT27          27
#define PORT28          28
#define PORT29          29
#define PORT30          30
#define PORT31          31
/** @} */

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Port bit helper macro.
 * @note    Overrides the one in @p pal.h.
 *
 * @param[in] n         bit position within the port
 *
 * @return              The bit mask.
 */
#define PAL_PORT_BIT(n) ((ioportmask_t)(0x8000U >> (n)))

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) _pal_lld_init(config)

#if SPC5_SIU_SUPPORTS_PORTS || defined(__DOXYGEN__)
/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) (((volatile uint16_t *)SIU.PGPDI)[port])

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
#define pal_lld_readlatch(port) (((volatile uint16_t *)SIU.PGPDO)[port])

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits)                                       \
  (((volatile uint16_t *)SIU.PGPDO)[port] = (bits))

/**
 * @brief   Reads a group of bits.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @return              The group logical states.
 *
 * @notapi
 */
#define pal_lld_readgroup(port, mask, offset)                               \
  _pal_lld_readgroup(port, mask, offset)

/**
 * @brief   Writes a group of bits.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] bits      bits to be written. Values exceeding the group width
 *                      are masked.
 *
 * @notapi
 */
#define pal_lld_writegroup(port, mask, offset, bits)                        \
  _pal_lld_writegroup(port, mask, offset, bits)

#endif /* SPC5_SIU_SUPPORTS_PORTS */

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
 * @brief   Reads a logical state from an I/O pad.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @return              The logical state.
 * @retval PAL_LOW      low logical state.
 * @retval PAL_HIGH     high logical state.
 *
 * @notapi
 */
#define pal_lld_readpad(port, pad)                                          \
  (SIU.GPDI[((port) * 16) + (pad)].R)

/**
 * @brief   Writes a logical state on an output pad.
 * @note    This function is not meant to be invoked directly by the
 *          application  code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] bit       logical value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 *
 * @notapi
 */
#define pal_lld_writepad(port, pad, bit)                                    \
  (SIU.GPDO[((port) * 16) + (pad)].R = (bit))

/**
 * @brief   Sets a pad logical state to @p PAL_HIGH.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_setpad(port, pad)                                           \
  (SIU.GPDO[((port) * 16) + (pad)].R = 1)

/**
 * @brief   Clears a pad logical state to @p PAL_LOW.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_clearpad(port, pad)                                         \
  (SIU.GPDO[((port) * 16) + (pad)].R = 0)

/**
 * @brief   Toggles a pad logical state.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_togglepad(port, pad)                                        \
  (SIU.GPDO[((port) * 16) + (pad)].R = ~SIU.GPDO[((port) * 16) + (pad)].R)

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */
#define pal_lld_setpadmode(port, pad, mode)

extern const PALConfig pal_default_config;

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  ioportmask_t _pal_lld_readgroup(ioportid_t port,
                                  ioportmask_t mask,
                                  uint_fast8_t offset);
  void _pal_lld_writegroup(ioportid_t port,
                           ioportmask_t mask,
                           uint_fast8_t offset,
                           ioportmask_t bits);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL */

#endif /* HAL_PAL_LLD_H */

/** @} */
