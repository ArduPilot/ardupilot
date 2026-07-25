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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * SPC563Mxx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 1...15       Lowest...Highest.
 * DMA priorities:
 * 0...15       Highest...Lowest.
 */

#define SPC563Mxx_MCUCONF

/*
 * HAL driver system settings.
 */
#define SPC5_NO_INIT                        FALSE
#define SPC5_CLK_BYPASS                     FALSE
#define SPC5_ALLOW_OVERCLOCK                FALSE
#define SPC5_CLK_PREDIV_VALUE               2
#define SPC5_CLK_MFD_VALUE                  80
#define SPC5_CLK_RFD                        SPC5_RFD_DIV4
#define SPC5_FLASH_BIUCR                    (BIUCR_BANK1_TOO |              \
                                             BIUCR_MASTER4_PREFETCH |       \
                                             BIUCR_MASTER0_PREFETCH |       \
                                             BIUCR_DPFEN |                  \
                                             BIUCR_IPFEN |                  \
                                             BIUCR_PFLIM_ON_MISS |          \
                                             BIUCR_BFEN)
#define SPC5_EMIOS_GPRE_VALUE               20

/*
 * EDMA driver settings.
 */
#define SPC5_EDMA_CR_SETTING                (EDMA_CR_GRP1PRI(1) |           \
                                             EDMA_CR_GRP0PRI(0) |           \
                                             EDMA_CR_EMLM       |           \
                                             EDMA_CR_ERGA)
#define SPC5_EDMA_GROUP0_PRIORITIES         0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#define SPC5_EDMA_GROUP1_PRIORITIES         0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#define SPC5_EDMA_ERROR_HANDLER()           osalSysHalt("DMA failure")

/*
 * ADC driver settings.
 */
#define SPC5_ADC_USE_ADC0_Q0                FALSE
#define SPC5_ADC_USE_ADC0_Q1                FALSE
#define SPC5_ADC_USE_ADC0_Q2                FALSE
#define SPC5_ADC_USE_ADC1_Q3                FALSE
#define SPC5_ADC_USE_ADC1_Q4                FALSE
#define SPC5_ADC_USE_ADC1_Q5                FALSE
#define SPC5_ADC_FIFO0_DMA_IRQ_PRIO         12
#define SPC5_ADC_FIFO1_DMA_IRQ_PRIO         12
#define SPC5_ADC_FIFO2_DMA_IRQ_PRIO         12
#define SPC5_ADC_FIFO3_DMA_IRQ_PRIO         12
#define SPC5_ADC_FIFO4_DMA_IRQ_PRIO         12
#define SPC5_ADC_FIFO5_DMA_IRQ_PRIO         12
#define SPC5_ADC_CR_CLK_PS                  ADC_CR_CLK_PS(5)
#define SPC5_ADC_PUDCR                      {ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE,ADC_PUDCR_NONE}

/*
 * SERIAL driver system settings.
 */
#define SPC5_USE_ESCIA                      TRUE
#define SPC5_USE_ESCIB                      FALSE
#define SPC5_ESCIA_PRIORITY                 8
#define SPC5_ESCIB_PRIORITY                 8

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI1                  FALSE
#define SPC5_SPI_USE_DSPI2                  FALSE
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_RX_ONLY
#define SPC5_SPI_DSPI1_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4 | SPC5_MCR_PCSIS5)
#define SPC5_SPI_DSPI2_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4 | SPC5_MCR_PCSIS5)
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI1_IRQ_PRIO             10
#define SPC5_SPI_DSPI2_IRQ_PRIO             10
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       osalSysHalt("DSPI DMA failure")

/*
 * ICU driver system settings.
 */
#define SPC5_ICU_USE_EMIOS_CH1              FALSE
#define SPC5_ICU_USE_EMIOS_CH2              FALSE
#define SPC5_ICU_USE_EMIOS_CH3              FALSE
#define SPC5_ICU_USE_EMIOS_CH4              FALSE
#define SPC5_ICU_USE_EMIOS_CH5              FALSE
#define SPC5_ICU_USE_EMIOS_CH6              FALSE
#define SPC5_ICU_USE_EMIOS_CH11             FALSE
#define SPC5_ICU_USE_EMIOS_CH13             FALSE
#define SPC5_EMIOS_FLAG_F1_PRIORITY         8
#define SPC5_EMIOS_FLAG_F2_PRIORITY         8
#define SPC5_EMIOS_FLAG_F3_PRIORITY         8
#define SPC5_EMIOS_FLAG_F4_PRIORITY         8
#define SPC5_EMIOS_FLAG_F5_PRIORITY         8
#define SPC5_EMIOS_FLAG_F6_PRIORITY         8
#define SPC5_EMIOS_FLAG_F11_PRIORITY        8
#define SPC5_EMIOS_FLAG_F13_PRIORITY        8

/*
 * PWM driver system settings.
 */
#define SPC5_PWM_USE_EMIOS_CH0              FALSE
#define SPC5_PWM_USE_EMIOS_CH8              FALSE
#define SPC5_PWM_USE_EMIOS_CH9              FALSE
#define SPC5_PWM_USE_EMIOS_CH10             FALSE
#define SPC5_PWM_USE_EMIOS_CH12             FALSE
#define SPC5_PWM_USE_EMIOS_CH14             FALSE
#define SPC5_PWM_USE_EMIOS_CH15             FALSE
#define SPC5_PWM_USE_EMIOS_CH23             FALSE
#define SPC5_EMIOS_FLAG_F0_PRIORITY         8
#define SPC5_EMIOS_FLAG_F8_PRIORITY         8
#define SPC5_EMIOS_FLAG_F9_PRIORITY         8
#define SPC5_EMIOS_FLAG_F10_PRIORITY        8
#define SPC5_EMIOS_FLAG_F12_PRIORITY        8
#define SPC5_EMIOS_FLAG_F14_PRIORITY        8
#define SPC5_EMIOS_FLAG_F15_PRIORITY        8
#define SPC5_EMIOS_FLAG_F23_PRIORITY        8

/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                FALSE

#define SPC5_CAN_USE_FLEXCAN0               FALSE
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN0_IRQ_PRIORITY      11

#define SPC5_CAN_USE_FLEXCAN1               FALSE
#define SPC5_CAN_FLEXCAN1_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN1_IRQ_PRIORITY      11

#endif /* MCUCONF_H */
