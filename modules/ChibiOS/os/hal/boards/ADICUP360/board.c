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
#include "aducm_gp.h"

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
 * @brief   Type of ADUCM36x GPIO port setup.
 */
typedef struct {
  uint32_t              con;
  uint32_t              oen;
  uint32_t              pul;
  uint32_t              oce;
} gp_setup_t;

/**
 * @brief   Type of ADUCM36x GPIO initialization data.
 */
typedef struct {
  gp_setup_t            GP0Data;
  gp_setup_t            GP1Data;
  gp_setup_t            GP2Data;
} gp_config_t;

/**
 * @brief   ADUCM36x GPIO static initialization data.
 */
static const gp_config_t gp_default_config = {
  {VAL_GP0CON, VAL_GP0OEN, VAL_GP0PUL, VAL_GP0OCE},
  {VAL_GP1CON, VAL_GP1OEN, VAL_GP1PUL, VAL_GP1OCE},
  {VAL_GP2CON, VAL_GP2OEN, VAL_GP2PUL, VAL_GP2OCE}
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void gp_init(aducm_gp_t *gpp, const gp_setup_t *config) {

  gpp->CON  = config->con;
  gpp->OEN  = config->oen;
  gpp->PUL  = config->pul;
  gpp->OCE  = config->oce;
}

static void aducm_gpio_init(void) {

  /* Initializing all the defined GP ports.*/
  gp_init(GP0, &gp_default_config.GP0Data);
  gp_init(GP1, &gp_default_config.GP1Data);
  gp_init(GP2, &gp_default_config.GP2Data);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Early initialization code.
 * @details GP ports and system clocks are initialized before everything
 *          else.
 */
void __early_init(void) {

  aducm_gpio_init();
  aducm_clock_init();
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {

  (void)sdcp;
  /* CHTODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   SDC card write protection detection.
 */
bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  /* CHTODO: Fill the implementation.*/
  return false;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  /* CHTODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* CHTODO: Fill the implementation.*/
  return false;
}
#endif

/**
 * @brief   Board-specific initialization code.
 * @note    You can add your board-specific code here.
 */
void boardInit(void) {

}
