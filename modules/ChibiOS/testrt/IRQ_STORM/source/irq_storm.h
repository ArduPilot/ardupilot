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
 * @file    irq_storm.h
 * @brief   IRQ Storm stress test header.
 *
 * @addtogroup IRQ_STORM
 * @{
 */

#ifndef IRQ_STORM_H
#define IRQ_STORM_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Timings randomization.
 */
#if !defined(IRQ_STORM_CFG_RANDOMIZE) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_RANDOMIZE             FALSE
#endif

/**
 * @brief   Number of test iterations.
 */
#if !defined(IRQ_STORM_CFG_ITERATIONS) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_ITERATIONS            100
#endif

/**
 * @brief   Number of storm threads.
 */
#if !defined(IRQ_STORM_CFG_NUM_THREADS) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_NUM_THREADS           8
#endif

/**
 * @brief   Priority of storm threads.
 */
#if !defined(IRQ_STORM_CFG_THREADS_PRIORITY) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_THREADS_PRIORITY      (tprio_t)(NORMALPRIO-20)
#endif

/**
 * @brief   Mailboxes size.
 */
#if !defined(IRQ_STORM_CFG_MAILBOX_SIZE) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_MAILBOX_SIZE          4
#endif

/**
 * @brief   Stack size for worker threads.
 */
#if !defined(IRQ_STORM_CFG_STACK_SIZE) || defined(__DOXYGEN__)
#define IRQ_STORM_CFG_STACK_SIZE            128
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct {
  /**
   * @brief   Stream for output.
   */
  BaseSequentialStream  *out;
  /**
   * @brief   LED line.
   */
  ioline_t              line;
  /**
   * @brief   GPT driver 1.
   */
  GPTDriver             *gpt1p;
  /**
   * @brief   GPT driver 2.
   */
  GPTDriver             *gpt2p;
  /**
   * @brief   GPT1 configuration 1.
   */
  const GPTConfig       *gptcfg1p;
  /**
   * @brief   GPT1 configuration 2.
   */
  const GPTConfig       *gptcfg2p;
  /**
   * @brief   System clock.
   */
  uint32_t              sysclk;
} irq_storm_config_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void irq_storm_gpt1_cb(GPTDriver *gptp);
  void irq_storm_gpt2_cb(GPTDriver *gptp);
  void irq_storm_execute(const irq_storm_config_t *cfg);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* IRQ_STORM_H */

/** @} */
