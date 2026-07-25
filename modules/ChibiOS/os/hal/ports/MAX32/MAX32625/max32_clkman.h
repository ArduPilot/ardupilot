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
 * @file    MAX32625/aducm_cc.h
 * @brief   CC helper driver header.
 * @note    This file requires definitions from the ADI header file
 *          @p MAX32625.h.
 *
 * @addtogroup MAX32625_CC
 * @{
 */

#ifndef MAX32_CLKMAN_H
#define MAX32_CLKMAN_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    GPIO peripherals specific CC operations
 * @{
 */
/**
 * @brief   Enables the GPIO input peripheral clock setting the selected 
 *          clock divider.
 *
 * @api
 */
#define clkmanEnableGPIO() {                                                     \
  MXC_CLKMAN->sys_clk_ctrl_6_gpio = MAX32_GPIO_CLKMAN;                       \
}

/**
 * @brief   Disables the GPIO input peripheral clock.
 *
 * @api
 */
#define clkmanDisableGPIO() {                                                    \
  MXC_CLKMAN->sys_clk_ctrl_6_gpio = MAX32_GPIO_DISABLED;                     \
}

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* MAX32_CLKMAN_H */

/** @} */
