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
 * @file    USARTv3/stm32_usart.h
 * @brief   STM32 USART helpers header.
 *
 * @addtogroup STM32_
 * @{
 */

#ifndef STM32_USART_H
#define STM32_USART_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    CR2 register additional macros
 * @{
 */
#define USART_CR1_DATA7                     (USART_CR1_M1)
#define USART_CR1_DATA8                     (0U)
#define USART_CR1_DATA9                     (USART_CR1_M0)
#define USART_CR1_OVER16                    (0)
/** @} */

/**
 * @name    CR2 register additional macros
 * @{
 */
#define USART_CR2_STOP1_BITS                (0U << 12)
#define USART_CR2_STOP0P5_BITS              (1U << 12)
#define USART_CR2_STOP2_BITS                (2U << 12)
#define USART_CR2_STOP1P5_BITS              (3U << 12)
/** @} */

/**
 * @name    CR3 register additional macros
 * @{
 */
#define USART_CR3_TXFTCFG_NONFULL           (USART_CR3_TXFTCFG_Msk) /* Pseudo value.*/
#define USART_CR3_TXFTCFG_1E                (0U)
#define USART_CR3_TXFTCFG_1Q                (USART_CR3_TXFTCFG_0)
#define USART_CR3_TXFTCFG_1H                (USART_CR3_TXFTCFG_1)
#define USART_CR3_TXFTCFG_3Q                (USART_CR3_TXFTCFG_1 | USART_CR3_TXFTCFG_0)
#define USART_CR3_TXFTCFG_7E                (USART_CR3_TXFTCFG_2)
#define USART_CR3_TXFTCFG_EMPTY             (USART_CR3_TXFTCFG_2 | USART_CR3_TXFTCFG_0)

#define USART_CR3_RXFTCFG_NONEMPTY          (USART_CR3_RXFTCFG_Msk) /* Pseudo value.*/
#define USART_CR3_RXFTCFG_1E                (0U)
#define USART_CR3_RXFTCFG_1Q                (USART_CR3_RXFTCFG_0)
#define USART_CR3_RXFTCFG_1H                (USART_CR3_RXFTCFG_1)
#define USART_CR3_RXFTCFG_3Q                (USART_CR3_RXFTCFG_1 | USART_CR3_RXFTCFG_0)
#define USART_CR3_RXFTCFG_7E                (USART_CR3_RXFTCFG_2)
#define USART_CR3_RXFTCFG_FULL              (USART_CR3_RXFTCFG_2 | USART_CR3_RXFTCFG_0)
/** @} */

/**
 * @name   PRESC register additional macros
 * @{
 */
#define USART_PRESC_N(n)                    ((n) << USART_PRESC_PRESCALER_Pos)
#define USART_PRESC1                        USART_PRESC_N(0U)
#define USART_PRESC2                        USART_PRESC_N(1U)
#define USART_PRESC4                        USART_PRESC_N(2U)
#define USART_PRESC6                        USART_PRESC_N(3U)
#define USART_PRESC8                        USART_PRESC_N(4U)
#define USART_PRESC10                       USART_PRESC_N(5U)
#define USART_PRESC12                       USART_PRESC_N(6U)
#define USART_PRESC16                       USART_PRESC_N(7U)
#define USART_PRESC32                       USART_PRESC_N(8U)
#define USART_PRESC64                       USART_PRESC_N(9U)
#define USART_PRESC128                      USART_PRESC_N(10U)
#define USART_PRESC256                      USART_PRESC_N(11U)
/** @} */

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

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32_USART_H */

/** @} */
