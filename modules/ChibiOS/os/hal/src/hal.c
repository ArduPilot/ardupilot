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
 * @file    hal.c
 * @brief   HAL subsystem code.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   HAL initialization.
 * @details This function invokes the low level initialization code then
 *          initializes all the drivers enabled in the HAL. Finally the
 *          board-specific initialization is performed by invoking
 *          @p boardInit() (usually defined in @p board.c).
 *
 * @init
 */
void halInit(void) {

  /* Initializes the OS Abstraction Layer.*/
  osalInit();

  /* Platform low level initializations.*/
  hal_lld_init();

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)
#if defined(PAL_NEW_INIT)
  palInit();
#else
  palInit(&pal_default_config);
#endif
#endif
#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)
  adcInit();
#endif
#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)
  canInit();
#endif
#if (HAL_USE_CRY == TRUE) || defined(__DOXYGEN__)
  cryInit();
#endif
#if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)
  dacInit();
#endif
#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)
  eflInit();
#endif
#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)
  gptInit();
#endif
#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)
  i2cInit();
#endif
#if (HAL_USE_I2S == TRUE) || defined(__DOXYGEN__)
  i2sInit();
#endif
#if (HAL_USE_ICU == TRUE) || defined(__DOXYGEN__)
  icuInit();
#endif
#if (HAL_USE_EICU == TRUE) || defined(__DOXYGEN__)
  eicuInit();
#endif
#ifndef _ARDUPILOT_
#if (HAL_USE_MAC == TRUE) || defined(__DOXYGEN__)
  macInit();
#endif
#endif // _ARDUPILOT_
#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)
  pwmInit();
#endif
#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)
  sdInit();
#endif
#if (HAL_USE_SDC == TRUE) || defined(__DOXYGEN__)
  sdcInit();
#endif
#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)
  sioInit();
#endif
#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)
  spiInit();
#endif
#if (HAL_USE_TRNG == TRUE) || defined(__DOXYGEN__)
  trngInit();
#endif
#if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)
  uartInit();
#endif
#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)
  usbInit();
#endif
#if (HAL_USE_MMC_SPI == TRUE) || defined(__DOXYGEN__)
  mmcInit();
#endif
#if (HAL_USE_SERIAL_USB == TRUE) || defined(__DOXYGEN__)
  sduInit();
#endif
#if (HAL_USE_RTC == TRUE) || defined(__DOXYGEN__)
  rtcInit();
#endif
#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)
  wdgInit();
#endif
#if (HAL_USE_WSPI == TRUE) || defined(__DOXYGEN__)
  wspiInit();
#endif

  /* Community driver overlay initialization.*/
#if defined(HAL_USE_COMMUNITY) || defined(__DOXYGEN__)
#if (HAL_USE_COMMUNITY == TRUE) || defined(__DOXYGEN__)
  halCommunityInit();
#endif
#endif

  /* Board specific initialization.*/
  boardInit();

/*
 *  The ST driver is a special case, it is only initialized if the OSAL is
 *  configured to require it.
 */
#if OSAL_ST_MODE != OSAL_ST_MODE_NONE
  stInit();
#endif
}

/** @} */
