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

#include "portab.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

void spi_circular_cb(SPIDriver *spip);
void spi_error_cb(SPIDriver *spip);

/*
 * Circular SPI configuration (31,25MHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig c_spicfg = {
  .circular         = true,
  .data_cb          = spi_circular_cb,
  .error_cb         = spi_error_cb,
  .ssport           = GPIOD,
  .sspad            = 14U,
  .cfg1             = SPI_CFG1_MBR_DIV4 | SPI_CFG1_DSIZE_8BITS,
  .cfg2             = 0U,
  .dtr1rx           = 0U,
  .dtr1tx           = 0U,
  .dtr2rx           = 0U,
  .dtr2tx           = 0U
};

/*
 * Maximum speed SPI configuration (31,25MHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig hs_spicfg = {
  .circular         = false,
  .data_cb          = NULL,
  .error_cb         = spi_error_cb,
  .ssport           = GPIOD,
  .sspad            = 14U,
  .cfg1             = SPI_CFG1_MBR_DIV4 | SPI_CFG1_DSIZE_8BITS,
  .cfg2             = 0U,
  .dtr1rx           = 0U,
  .dtr1tx           = 0U,
  .dtr2rx           = 0U,
  .dtr2tx           = 0U
};

/*
 * Low speed SPI configuration (1,953125MHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig ls_spicfg = {
  .circular         = false,
  .data_cb          = NULL,
  .error_cb         = spi_error_cb,
  .ssport           = GPIOD,
  .sspad            = 14U,
  .cfg1             = SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_8BITS,
  .cfg2             = 0U,
  .dtr1rx           = 0U,
  .dtr1tx           = 0U,
  .dtr2rx           = 0U,
  .dtr2tx           = 0U
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

  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) |
                          PAL_STM32_OSPEED_HIGHEST);    /* SPI1 SCK.        */
  palSetPadMode(GPIOG, 9, PAL_MODE_ALTERNATE(5) |
                          PAL_STM32_OSPEED_HIGHEST);    /* SPI1 MISO.       */
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(5) |
                          PAL_STM32_OSPEED_HIGHEST);    /* SPI1 MOSI.       */
  palSetPadMode(GPIOD, 14, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);   /* SPI1 NSS.        */
  palSetPad(GPIOD, 14);
}

/** @} */
