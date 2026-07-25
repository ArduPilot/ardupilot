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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#include "hal.h"
#include "max32_gpio.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of MAX32625 GPIO port setup.
 */
typedef struct {
  uint32_t              outmode;
  uint32_t              outval;
  uint32_t              funcsel;
  uint32_t              inmode;
} gpio_setup_t;

/**
 * @brief   Type of MAX32625 GPIO initialization data.
 */
typedef struct {
  gpio_setup_t          GPIO0Data;
  gpio_setup_t          GPIO1Data;
  gpio_setup_t          GPIO2Data;
  gpio_setup_t          GPIO3Data;
  gpio_setup_t          GPIO4Data;
} gpio_config_t;

/**
 * @brief   MAX32625 GPIO static initialization data.
 */
static const gpio_config_t gpio_default_config = {
  {VAL_GPIO0_OUTMODE, VAL_GPIO0_OUTVAL, VAL_GPIO0_FUNCSEL, VAL_GPIO0_INMODE},
  {VAL_GPIO1_OUTMODE, VAL_GPIO1_OUTVAL, VAL_GPIO1_FUNCSEL, VAL_GPIO1_INMODE},
  {VAL_GPIO2_OUTMODE, VAL_GPIO2_OUTVAL, VAL_GPIO2_FUNCSEL, VAL_GPIO2_INMODE},
  {VAL_GPIO3_OUTMODE, VAL_GPIO3_OUTVAL, VAL_GPIO3_FUNCSEL, VAL_GPIO3_INMODE},
  {VAL_GPIO4_OUTMODE, VAL_GPIO4_OUTVAL, VAL_GPIO4_FUNCSEL, VAL_GPIO4_INMODE}
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void gpio_init(max32_gpio_t *gpiop, const gpio_setup_t *config) {

  gpiop->OUT_MODE  = config->outmode;
  gpiop->OUT_VAL  = config->outval;
  gpiop->FUNC_SEL  = config->funcsel;
  gpiop->IN_MODE  = config->inmode;
}

static void max32_gpio_init(void) {

  /* Initializing all the defined GP ports.*/
  gpio_init(GPIO0, &gpio_default_config.GPIO0Data);
  gpio_init(GPIO1, &gpio_default_config.GPIO1Data);
  gpio_init(GPIO2, &gpio_default_config.GPIO2Data);
  gpio_init(GPIO3, &gpio_default_config.GPIO3Data);
  gpio_init(GPIO4, &gpio_default_config.GPIO4Data);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Early initialization code.
 * @details GPIO ports and system clocks are initialized before everything
 *          else.
 */
void __early_init(void) {

  max32_gpio_init();
  max32_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @note    You can add your board-specific code here.
 */
void boardInit(void) {

}
