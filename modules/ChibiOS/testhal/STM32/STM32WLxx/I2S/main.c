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

#include "ch.h"
#include "hal.h"

#define STM32_I2SCFGR_CHLEN_16BIT           (0U << SPI_I2SCFGR_CHLEN_Pos)
#define STM32_I2SCFGR_CHLEN_32BIT           (1U << SPI_I2SCFGR_CHLEN_Pos)

#define STM32_I2SCFGR_DATLEN_16BIT          (0U << SPI_I2SCFGR_DATLEN_Pos)
#define STM32_I2SCFGR_DATLEN_24BIT          (1U << SPI_I2SCFGR_DATLEN_Pos)
#define STM32_I2SCFGR_DATLEN_32BIT          (2U << SPI_I2SCFGR_DATLEN_Pos)

#define STM32_I2SCFGR_CKPOL_LOW             (0U << SPI_I2SCFGR_CKPOL_Pos)
#define STM32_I2SCFGR_CKPOL_HIGH            (1U << SPI_I2SCFGR_CKPOL_Pos)

#define STM32_I2SCFGR_I2SSTD_I2S            (0U << SPI_I2SCFGR_I2SSTD_Pos)
#define STM32_I2SCFGR_I2SSTD_MSB            (1U << SPI_I2SCFGR_I2SSTD_Pos)
#define STM32_I2SCFGR_I2SSTD_LSB            (2U << SPI_I2SCFGR_I2SSTD_Pos)
#define STM32_I2SCFGR_I2SSTD_PCM            (3U << SPI_I2SCFGR_I2SSTD_Pos)

#define STM32_I2SCFGR_I2SCFG_SLAVE_TX       (0U << SPI_I2SCFGR_I2SCFG_Pos)
#define STM32_I2SCFGR_I2SCFG_SLAVE_RX       (1U << SPI_I2SCFGR_I2SCFG_Pos)
#define STM32_I2SCFGR_I2SCFG_MASTER_TX      (2U << SPI_I2SCFGR_I2SCFG_Pos)
#define STM32_I2SCFGR_I2SCFG_MASTER_RX      (3U << SPI_I2SCFGR_I2SCFG_Pos)

#define STM32_I2SCFGR_I2SMOD_SPI            (0U << SPI_I2SCFGR_I2SMOD_Pos)
#define STM32_I2SCFGR_I2SMOD_I2S            (1U << SPI_I2SCFGR_I2SMOD_Pos)

#define STM32_I2SSPR_ODD_0                  (0U << SPI_I2SPR_ODD_Pos)
#define STM32_I2SSPR_ODD_1                  (1U << SPI_I2SPR_ODD_Pos)

#define I2S_BUF_SIZE            256

static uint16_t i2s_rx_buf[I2S_BUF_SIZE];

static void i2scallback(I2SDriver *i2sp);

static const I2SConfig i2scfg = {
  .tx_buffer = NULL,
  .rx_buffer = i2s_rx_buf,
  .size = I2S_BUF_SIZE,
  .end_cb = i2scallback,
  .i2scfgr = STM32_I2SCFGR_I2SSTD_MSB |
             STM32_I2SCFGR_I2SCFG_MASTER_RX |
             STM32_I2SCFGR_I2SMOD_I2S,
  .i2spr = 0x117                                // 1024KHz clock
};

static void i2scallback(I2SDriver *i2sp) {

  if (i2sIsBufferComplete(i2sp)) {
    /* 2nd buffer half processing.*/
  }
  else {
    /* 1st buffer half processing.*/
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Starting and configuring the I2S driver.
   */
  i2sStart(&I2SD2, &i2scfg);
  /* Configure PB13 as I2S_CK.*/
  palSetLineMode(LINE_ARD_A5, PAL_MODE_ALTERNATE(5));
  /* Configure PA10 as I2S_SD.*/
  palSetLineMode(LINE_ARD_A2, PAL_MODE_ALTERNATE(5));

  /*
   * Starting continuous I2S transfer.
   */
  i2sStartExchange(&I2SD2);

  /*
   * Normal main() thread activity, if the button 1 is pressed then the I2S
   * transfer is stopped.
   */
  while (true) {
    if (palReadLine(LINE_BUTTON_1) == PAL_LOW) {
      i2sStopExchange(&I2SD2);
      palClearLine(LINE_LED_RED);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}
