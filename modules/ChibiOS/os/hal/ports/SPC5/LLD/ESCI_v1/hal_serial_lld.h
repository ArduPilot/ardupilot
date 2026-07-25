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
 * @file    SPC5xx/ESCI_v1/hal_serial_lld.c
 * @brief   SPC5xx low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Serial port modes
 * @{
 */
#define SD_MODE_PARITY_MASK     0x03        /**< @brief Parity field mask.  */
#define SD_MODE_PARITY_NONE     0x00        /**< @brief No parity.          */
#define SD_MODE_PARITY_EVEN     0x01        /**< @brief Even parity.        */
#define SD_MODE_PARITY_ODD      0x02        /**< @brief Odd parity.         */

#define SD_MODE_NORMAL          0x00        /**< @brief Normal operations.  */
#define SD_MODE_LOOPBACK        0x80        /**< @brief Internal loopback.  */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   eSCI-A driver enable switch.
 * @details If set to @p TRUE the support for eSCI-A is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SPC5_USE_ESCIA) || defined(__DOXYGEN__)
#define SPC5_USE_ESCIA                      FALSE
#endif

/**
 * @brief   eSCI-B driver enable switch.
 * @details If set to @p TRUE the support for eSCI-B is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SPC5_USE_ESCIB) || defined(__DOXYGEN__)
#define SPC5_USE_ESCIB                      FALSE
#endif

/**
 * @brief   eSCI-C driver enable switch.
 * @details If set to @p TRUE the support for eSCI-C is included.
 * @note    The default is @p TRUE.
 */
#if !defined(SPC5_USE_ESCIC) || defined(__DOXYGEN__)
#define SPC5_USE_ESCIC                      FALSE
#endif

/**
 * @brief   eSCI-A interrupt priority level setting.
 */
#if !defined(SPC5_ESCIA_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_ESCIA_PRIORITY                 8
#endif

/**
 * @brief   eSCI-B interrupt priority level setting.
 */
#if !defined(SPC5_ESCIB_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_ESCIB_PRIORITY                 8
#endif

/**
 * @brief   eSCI-C interrupt priority level setting.
 */
#if !defined(SPC5_ESCIC_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_ESCIC_PRIORITY                 8
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SPC5_USE_ESCIA && !SPC5_HAS_ESCIA
#error "eSCI-A not present in the selected device"
#endif

#if SPC5_USE_ESCIB && !SPC5_HAS_ESCIB
#error "eSCI-B not present in the selected device"
#endif

#if SPC5_USE_ESCIC && !SPC5_HAS_ESCIC
#error "eSCI-C not present in the selected device"
#endif

#if !SPC5_USE_ESCIA && !SPC5_USE_ESCIB && !SPC5_USE_ESCIC
#error "SERIAL driver activated but no eSCI peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Generic Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 * @note    This structure content is architecture dependent, each driver
 *          implementation defines its own version and the custom static
 *          initializers.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /**
   * @brief Mode flags.
   */
  uint8_t                   sc_mode;
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
    volatile sdstate_t      state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the volatile eSCI registers block.*/                        \
  volatile struct ESCI_tag  *escip;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SPC5_USE_ESCIA && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#if SPC5_USE_ESCIB && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#if SPC5_USE_ESCIC && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
