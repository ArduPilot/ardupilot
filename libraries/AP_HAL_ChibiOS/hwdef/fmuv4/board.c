/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config = {

  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR, VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH},
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR, VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH},
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR, VAL_GPIOC_ODR, VAL_GPIOC_AFRL, VAL_GPIOC_AFRH},
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR, VAL_GPIOD_ODR, VAL_GPIOD_AFRL, VAL_GPIOD_AFRH},
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR, VAL_GPIOE_ODR, VAL_GPIOE_AFRL, VAL_GPIOE_AFRH},
  {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR, VAL_GPIOF_ODR, VAL_GPIOF_AFRL, VAL_GPIOF_AFRH},
  {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR, VAL_GPIOG_ODR, VAL_GPIOG_AFRL, VAL_GPIOG_AFRH},
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR, VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH},
  {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR, VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH}
};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {
  stm32_clock_init();
}

void __late_init(void) {
  halInit();
  chSysInit();
#ifdef HAL_USB_PRODUCT_ID
  setup_usb_strings();
#endif
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {
  static bool last_status = false;

  if (blkIsTransferring(sdcp))
    return last_status;
  return last_status = (bool)palReadPad(GPIOC, 11);
}

/**
 * @brief   SDC card write protection detection.
 */
bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  return false;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return false;
}
#endif

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {

  //Setup ADC pins for Voltage and Current Sensing
  palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); //Pin PA2
  palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); //Pin PA3
  palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); //Pin PA4

  palSetPadMode(GPIOE, 12, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID2);
  palClearPad(GPIOE, 12);

  /* External interrupts */
  // GPIO_GYRO_DRDY
  palSetPadMode(GPIOB, 0, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);
  // GPIO_MAG_DRDY  
  palSetPadMode(GPIOB, 1, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);

  // GPIO_ACCEL_DRDY  
  palSetPadMode(GPIOB, 4, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);

  // GPIOI_MPU_DRDY  
  palSetPadMode(GPIOD, 15, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);



  /* SPI chip selects */
  // // SPIDEV_CS_MS5611  
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
  palSetPadMode(GPIOD, 7, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOD, 7);
  // GPIO_SPI_CS_FRAM
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
  palSetPadMode(GPIOD, 10, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOD, 10);

  // SPIDEV_CS_MPU
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
  palSetPadMode(GPIOC, 2, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOC, 2);
  // SPIDEV_CS_EXT_MPU  
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
  palSetPadMode(GPIOE, 4, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOE, 4);
  // SPIDEV_CS_EXT_MS5611
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
  palSetPadMode(GPIOC, 14, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOC, 14);
  // SPIDEV_CS_EXT_LSM9DS0_AM
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
  palSetPadMode(GPIOC, 15, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOC, 15);
  // SPIDEV_CS_EXT_LSM9DS0_G
  //(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
  palSetPadMode(GPIOC, 13, PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
  palSetPad(GPIOC, 13);

  //PWM O/P Setup

  // GPIO_TIM1_CH1OUT
  //  (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
  palSetPadMode(GPIOE, 9, PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_STM32_ALTERNATE(1));
  palClearPad(GPIOE, 9);
  // GPIO_TIM1_CH2OUT
  //  (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
  palSetPadMode(GPIOE, 11, PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_STM32_ALTERNATE(1));
  palClearPad(GPIOE, 11);
  // GPIO_TIM1_CH3OUT
  //  (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
  palSetPadMode(GPIOE, 13, PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_STM32_ALTERNATE(1));
  palClearPad(GPIOE, 13);
  // GPIO_TIM1_CH4OUT
  //  (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
  palSetPadMode(GPIOE, 14, PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_OSPEED_MID2 | PAL_STM32_ALTERNATE(1));
  palClearPad(GPIOE, 14);
  //call extra board specific initialisation method
  HAL_BOARD_INIT_HOOK_CALL
}
