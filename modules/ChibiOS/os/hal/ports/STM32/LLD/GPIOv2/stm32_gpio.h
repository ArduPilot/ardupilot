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
 * @file    GPIOv2/stm32_gpio.h
 * @brief   STM32 GPIO units common header.
 * @note    This file requires definitions from the ST STM32 header file.
 *
 * @addtogroup STM32_GPIOv2
 * @{
 */

#ifndef STM32_GPIO_H
#define STM32_GPIO_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Discarded definitions from the ST headers, the PAL driver uses its own
   definitions in order to have an unified handling for all devices.
   Unfortunately the ST headers have no uniform definitions for the same
   objects across the various sub-families.*/
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOF
#undef GPIOG
#undef GPIOH
#undef GPIOI
#undef GPIOJ
#undef GPIOK

/**
 * @name    GPIO ports definitions
 * @{
 */
#define GPIOA                           ((stm32_gpio_t *)GPIOA_BASE)
#define GPIOB                           ((stm32_gpio_t *)GPIOB_BASE)
#define GPIOC                           ((stm32_gpio_t *)GPIOC_BASE)
#define GPIOD                           ((stm32_gpio_t *)GPIOD_BASE)
#define GPIOE                           ((stm32_gpio_t *)GPIOE_BASE)
#define GPIOF                           ((stm32_gpio_t *)GPIOF_BASE)
#define GPIOG                           ((stm32_gpio_t *)GPIOG_BASE)
#define GPIOH                           ((stm32_gpio_t *)GPIOH_BASE)
#define GPIOI                           ((stm32_gpio_t *)GPIOI_BASE)
#define GPIOJ                           ((stm32_gpio_t *)GPIOJ_BASE)
#define GPIOK                           ((stm32_gpio_t *)GPIOK_BASE)
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

/**
 * @brief   STM32 GPIO registers block.
 */
typedef struct {

  volatile uint32_t     MODER;
  volatile uint32_t     OTYPER;
  volatile uint32_t     OSPEEDR;
  volatile uint32_t     PUPDR;
  volatile uint32_t     IDR;
  volatile uint32_t     ODR;
  volatile union {
    uint32_t            W;
    struct {
      uint16_t          set;
      uint16_t          clear;
    } H;
  } BSRR;
  volatile uint32_t     LOCKR;
  volatile uint32_t     AFRL;
  volatile uint32_t     AFRH;
} stm32_gpio_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* STM32_GPIO_H */

/** @} */
