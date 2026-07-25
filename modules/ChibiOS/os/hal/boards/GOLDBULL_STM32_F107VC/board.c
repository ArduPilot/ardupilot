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

#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
  {VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH},
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

#if HAL_USE_MMC_SPI
/*
 * Card detection through the card internal pull-up on D3.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  return !palReadPad(GPIOC, GPIOC_SPI3_SD_CD);
}

/*
 * Card write protection detection is not possible, the card is always
 * reported as not protected.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  return FALSE;
}
#endif

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

  /*
   * Several I/O pins are re-mapped:
   *   USART3 to the PD8/PD9 pins.
   *   I2C1 to the PB8/PB9 pins.
   *   SPI3 to the PC10/PC11/PC12 pins.
   */
  //AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_FULLREMAP |
  //              AFIO_MAPR_I2C1_REMAP |
  //              AFIO_MAPR_SPI3_REMAP |
  //              AFIO_MAPR_ETH_REMAP;
  /*
     * Several I/O pins are re-mapped:
     *   .
     */
    AFIO->MAPR |= AFIO_MAPR_ETH_REMAP |
                  AFIO_MAPR_I2C1_REMAP |
                  AFIO_MAPR_SPI3_REMAP |
                  AFIO_MAPR_CAN_REMAP_REMAP3 |
                  AFIO_MAPR_USART1_REMAP |
                  AFIO_MAPR_USART2_REMAP;







}
