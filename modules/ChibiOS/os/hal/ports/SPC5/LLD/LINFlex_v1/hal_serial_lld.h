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
 * @file    SPC5xx/LINFlex_v1/hal_serial_lld.h
 * @brief   SPC5xx low level serial driver header.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

#include "spc5_linflex.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Serial driver allowable modes
 * @{
 */
#define SD_MODE_8BITS_PARITY_NONE           (SPC5_UARTCR_WL)
#define SD_MODE_8BITS_PARITY_EVEN           (SPC5_UARTCR_WL |               \
                                             SPC5_UARTCR_PCE)
#define SD_MODE_8BITS_PARITY_ODD            (SPC5_UARTCR_WL |               \
                                             SPC5_UARTCR_PCE |              \
                                             SPC5_UARTCR_OP)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LINFlex-0 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-0 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX0) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX0            FALSE
#endif

/**
 * @brief   LINFlex-1 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-1 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX1) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX1            FALSE
#endif

/**
 * @brief   LINFlex-2 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-2 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX2) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX2            FALSE
#endif

/**
 * @brief   LINFlex-3 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-3 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX3) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX3            FALSE
#endif

/**
 * @brief   LINFlex-4 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-4 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX4) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX4            FALSE
#endif

/**
 * @brief   LINFlex-5 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-5 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX5) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX5            FALSE
#endif

/**
 * @brief   LINFlex-6 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-6 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX6) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX6            FALSE
#endif

/**
 * @brief   LINFlex-7 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-7 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX7) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX7            FALSE
#endif

/**
 * @brief   LINFlex-8 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-8 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX8) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX8            FALSE
#endif

/**
 * @brief   LINFlex-9 driver enable switch.
 * @details If set to @p TRUE the support for LINFlex-9 is included.
 */
#if !defined(SPC5_SERIAL_USE_LINFLEX9) || defined(__DOXYGEN__)
#define SPC5_SERIAL_USE_LINFLEX9            FALSE
#endif

/**
 * @brief   LINFlex-0 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX0_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX0_PRIORITY       8
#endif

/**
 * @brief   LINFlex-1 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX1_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX1_PRIORITY       8
#endif

/**
 * @brief   LINFlex-2 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX2_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX2_PRIORITY       8
#endif

/**
 * @brief   LINFlex-3 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX3_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX3_PRIORITY       8
#endif

/**
 * @brief   LINFlex-4 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX4_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX4_PRIORITY       8
#endif

/**
 * @brief   LINFlex-5 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX5_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX5_PRIORITY       8
#endif

/**
 * @brief   LINFlex-6 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX6_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX6_PRIORITY       8
#endif

/**
 * @brief   LINFlex-7 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX7_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX7_PRIORITY       8
#endif

/**
 * @brief   LINFlex-8 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX8_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX8_PRIORITY       8
#endif

/**
 * @brief   LINFlex-9 interrupt priority level setting.
 */
#if !defined(SPC5_SERIAL_LINFLEX9_PRIORITY) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX9_PRIORITY       8
#endif

/**
 * @brief   LINFlex-0 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX0_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX0_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-0 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX0_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX0_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-1 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX1_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX1_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-1 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX1_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX1_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-2 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX2_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX2_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-2 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX2_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX2_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-3 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX3_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX3_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-3 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX3_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX3_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-4 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX4_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX4_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-4 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX4_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX4_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-5 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX5_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX5_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-5 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX5_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX5_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-6 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX6_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX6_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-6 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX6_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX6_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-7 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX7_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX7_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-7 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX7_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX7_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-8 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX8_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX8_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-8 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX8_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX8_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif

/**
 * @brief   LINFlex-9 peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX9_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX9_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   LINFlex-9 peripheral configuration when stopped.
 * @note    The default configuration is 0 (never run) in run mode and
 *          0 (never run) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_SERIAL_LINFLEX9_STOP_PCTL) || defined(__DOXYGEN__)
#define SPC5_SERIAL_LINFLEX9_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SPC5_SERIAL_USE_LINFLEX0 && !SPC5_HAS_LINFLEX0
#error "LINFlex-0 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX1 && !SPC5_HAS_LINFLEX1
#error "LINFlex-1 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX2 && !SPC5_HAS_LINFLEX2
#error "LINFlex-2 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX3 && !SPC5_HAS_LINFLEX3
#error "LINFlex-3 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX4 && !SPC5_HAS_LINFLEX4
#error "LINFlex-4 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX5 && !SPC5_HAS_LINFLEX5
#error "LINFlex-5 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX6 && !SPC5_HAS_LINFLEX6
#error "LINFlex-6 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX7 && !SPC5_HAS_LINFLEX7
#error "LINFlex-7 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX8 && !SPC5_HAS_LINFLEX8
#error "LINFlex-8 not present in the selected device"
#endif

#if SPC5_SERIAL_USE_LINFLEX9 && !SPC5_HAS_LINFLEX9
#error "LINFlex-9 not present in the selected device"
#endif

#if !SPC5_SERIAL_USE_LINFLEX0 && !SPC5_SERIAL_USE_LINFLEX1 &&               \
    !SPC5_SERIAL_USE_LINFLEX2 && !SPC5_SERIAL_USE_LINFLEX3 &&               \
    !SPC5_SERIAL_USE_LINFLEX4 && !SPC5_SERIAL_USE_LINFLEX5 &&               \
    !SPC5_SERIAL_USE_LINFLEX6 && !SPC5_SERIAL_USE_LINFLEX7 &&               \
    !SPC5_SERIAL_USE_LINFLEX8 && !SPC5_SERIAL_USE_LINFLEX9
#error "SERIAL driver activated but no LINFlex peripheral assigned"
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
  uint32_t                  speed;
  /**
   * @brief Mode flags.
   */
  uint8_t                   mode;
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
  /* Pointer to the volatile LINFlex registers block.*/                     \
  volatile struct spc5_linflex *linflexp;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SPC5_SERIAL_USE_LINFLEX0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if SPC5_SERIAL_USE_LINFLEX1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif
#if SPC5_SERIAL_USE_LINFLEX2 && !defined(__DOXYGEN__)
extern SerialDriver SD3;
#endif
#if SPC5_SERIAL_USE_LINFLEX3 && !defined(__DOXYGEN__)
extern SerialDriver SD4;
#endif
#if SPC5_SERIAL_USE_LINFLEX4 && !defined(__DOXYGEN__)
extern SerialDriver SD5;
#endif
#if SPC5_SERIAL_USE_LINFLEX5 && !defined(__DOXYGEN__)
extern SerialDriver SD6;
#endif
#if SPC5_SERIAL_USE_LINFLEX6 && !defined(__DOXYGEN__)
extern SerialDriver SD7;
#endif
#if SPC5_SERIAL_USE_LINFLEX7 && !defined(__DOXYGEN__)
extern SerialDriver SD8;
#endif
#if SPC5_SERIAL_USE_LINFLEX8 && !defined(__DOXYGEN__)
extern SerialDriver SD9;
#endif
#if SPC5_SERIAL_USE_LINFLEX9 && !defined(__DOXYGEN__)
extern SerialDriver SD10;
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
