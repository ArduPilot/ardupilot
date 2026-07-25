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
 * @file    GPIOv2/max32_gpio.h
 * @brief   MAX32 GPIO units common header.
 * @note    This file requires definitions from the ST MAX32 header file.
 *
 * @addtogroup MAX32_GPIOv2
 * @{
 */

#ifndef MAX32_GPIO_H
#define MAX32_GPIO_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define GPIO0_BASE                      (MXC_BASE_GPIO + \
                                         MXC_R_GPIO_OFFS_RST_MODE_P0)
#define GPIO1_BASE                      (MXC_BASE_GPIO + \
                                         MXC_R_GPIO_OFFS_RST_MODE_P1)
#define GPIO2_BASE                      (MXC_BASE_GPIO + \
                                         MXC_R_GPIO_OFFS_RST_MODE_P2)
#define GPIO3_BASE                      (MXC_BASE_GPIO + \
                                         MXC_R_GPIO_OFFS_RST_MODE_P3)
#define GPIO4_BASE                      (MXC_BASE_GPIO + \
                                         MXC_R_GPIO_OFFS_RST_MODE_P4)
/**
 * @name    GPIO ports definitions
 * @{
 */
#define GPIO0                           ((max32_gpio_t *)GPIO0_BASE)
#define GPIO1                           ((max32_gpio_t *)GPIO1_BASE)
#define GPIO2                           ((max32_gpio_t *)GPIO2_BASE)
#define GPIO3                           ((max32_gpio_t *)GPIO3_BASE)
#define GPIO4                           ((max32_gpio_t *)GPIO4_BASE)
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
 * @brief   MAX32 GPIO registers block.
 */
typedef struct {

  volatile uint32_t     RST;
  volatile uint32_t     RESERVED0[15];
  volatile uint32_t     FREE;
  volatile uint32_t     RESERVED1[15];
  volatile uint32_t     OUT_MODE;
  volatile uint32_t     RESERVED2[15];
  volatile uint32_t     OUT_VAL;
  volatile uint32_t     RESERVED3[15];
  volatile uint32_t     FUNC_SEL;
  volatile uint32_t     RESERVED4[15];
  volatile uint32_t     IN_MODE;
  volatile uint32_t     RESERVED5[15];
  volatile uint32_t     IN_VAL;
  volatile uint32_t     RESERVED6[15];
  volatile uint32_t     INTMODE;
  volatile uint32_t     RESERVED7[15]; 
  volatile uint32_t     INTFL;
  volatile uint32_t     RESERVED8[15];
  volatile uint32_t     INTEN;
  volatile uint32_t     RESERVED9[15];
} max32_gpio_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* MAX32_GPIO_H */

/** @} */
