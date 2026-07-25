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
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"
#include "irq_storm.h"

#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*
 * GPT4 configuration.
 */
static const GPTConfig gpt4cfg = {
  1000000,              /* 1MHz timer clock.*/
  irq_storm_gpt1_cb,    /* Timer callback.*/
  0,
  0
};

/*
 * GPT3 configuration.
 */
static const GPTConfig gpt3cfg = {
  1000000,              /* 1MHz timer clock.*/
  irq_storm_gpt2_cb,    /* Timer callback.*/
  0,
  0
};

/*
 * IRQ Storm configuration.
 */
const irq_storm_config_t portab_irq_storm_config = {
  (BaseSequentialStream  *)&PORTAB_SD1,
  PORTAB_LINE_LED1,
  &GPTD4,
  &GPTD3,
  &gpt4cfg,
  &gpt3cfg,
  STM32_SYSCLK
};

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void portab_setup(void) {

}

/** @} */
