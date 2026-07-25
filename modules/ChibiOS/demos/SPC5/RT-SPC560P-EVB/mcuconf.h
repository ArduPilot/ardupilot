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
 * SPC560Pxx drivers configuration.
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

#define SPC560Pxx_MCUCONF

/*
 * HAL driver system settings.
 */
#define SPC5_NO_INIT                        FALSE
#define SPC5_ALLOW_OVERCLOCK                FALSE
#define SPC5_DISABLE_WATCHDOG               TRUE
#define SPC5_FMPLL0_IDF_VALUE               5
#define SPC5_FMPLL0_NDIV_VALUE              32
#define SPC5_FMPLL0_ODF                     SPC5_FMPLL_ODF_DIV4
#define SPC5_FMPLL1_IDF_VALUE               5
#define SPC5_FMPLL1_NDIV_VALUE              60
#define SPC5_FMPLL1_ODF                     SPC5_FMPLL_ODF_DIV4
#define SPC5_AUX0CLK_SRC                    SPC5_CGM_SS_FMPLL0
#define SPC5_MCONTROL_DIVIDER_VALUE         2
#define SPC5_FMPLL1_CLK_DIVIDER_VALUE       2
#define SPC5_AUX2CLK_SRC                    SPC5_CGM_SS_FMPLL0
#define SPC5_SP_CLK_DIVIDER_VALUE           2
#define SPC5_AUX3CLK_SRC                    SPC5_CGM_SS_FMPLL0
#define SPC5_FR_CLK_DIVIDER_VALUE           2
#define SPC5_CLOCK_FAILURE_HOOK()           osalSysHalt("clock failure")

/*
 * EDMA driver settings.
 */
#define SPC5_EDMA_CR_SETTING                (EDMA_CR_GRP1PRI(1) |           \
                                             EDMA_CR_GRP0PRI(0) |           \
                                             EDMA_CR_EMLM       |           \
                                             EDMA_CR_ERGA)
#define SPC5_EDMA_GROUP0_PRIORITIES         0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#define SPC5_EDMA_ERROR_HANDLER()           osalSysHalt("DMA failure")

/*
 * PWM driver system settings.
 */
#define SPC5_PWM_USE_SMOD0                  FALSE
#define SPC5_PWM_USE_SMOD1                  FALSE
#define SPC5_PWM_USE_SMOD2                  FALSE
#define SPC5_PWM_USE_SMOD3                  FALSE
#define SPC5_PWM_SMOD0_PRIORITY             7
#define SPC5_PWM_SMOD1_PRIORITY             7
#define SPC5_PWM_SMOD2_PRIORITY             7
#define SPC5_PWM_SMOD3_PRIORITY             7

#define SPC5_PWM_USE_SMOD4                  FALSE
#define SPC5_PWM_USE_SMOD5                  FALSE
#define SPC5_PWM_USE_SMOD6                  FALSE
#define SPC5_PWM_USE_SMOD7                  FALSE
#define SPC5_PWM_SMOD4_PRIORITY             7
#define SPC5_PWM_SMOD5_PRIORITY             7
#define SPC5_PWM_SMOD6_PRIORITY             7
#define SPC5_PWM_SMOD7_PRIORITY             7

/*
 * ICU driver system settings.
 */
#define SPC5_ICU_USE_SMOD0                  FALSE
#define SPC5_ICU_USE_SMOD1                  FALSE
#define SPC5_ICU_USE_SMOD2                  FALSE
#define SPC5_ICU_USE_SMOD3                  FALSE
#define SPC5_ICU_USE_SMOD4                  FALSE
#define SPC5_ICU_USE_SMOD5                  FALSE
#define SPC5_ICU_ETIMER0_PRIORITY           7

#define SPC5_ICU_USE_SMOD6                  FALSE
#define SPC5_ICU_USE_SMOD7                  FALSE
#define SPC5_ICU_USE_SMOD8                  FALSE
#define SPC5_ICU_USE_SMOD9                  FALSE
#define SPC5_ICU_USE_SMOD10                 FALSE
#define SPC5_ICU_USE_SMOD11                 FALSE
#define SPC5_ICU_ETIMER1_PRIORITY           7

/*
 * SERIAL driver system settings.
 */
#define SPC5_SERIAL_USE_LINFLEX0            TRUE
#define SPC5_SERIAL_USE_LINFLEX1            FALSE
#define SPC5_SERIAL_LINFLEX0_PRIORITY       8
#define SPC5_SERIAL_LINFLEX1_PRIORITY       8

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI0                  FALSE
#define SPC5_SPI_USE_DSPI1                  FALSE
#define SPC5_SPI_USE_DSPI2                  FALSE
#define SPC5_SPI_USE_DSPI3                  FALSE
#define SPC5_SPI_USE_DSPI4                  FALSE
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_RX_ONLY
#define SPC5_SPI_DSPI0_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4 | SPC5_MCR_PCSIS5 | SPC5_MCR_PCSIS6 | SPC5_MCR_PCSIS7)
#define SPC5_SPI_DSPI1_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4 | SPC5_MCR_PCSIS5 | SPC5_MCR_PCSIS6 | SPC5_MCR_PCSIS7)
#define SPC5_SPI_DSPI2_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3)
#define SPC5_SPI_DSPI3_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3)
#define SPC5_SPI_DSPI4_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3)
#define SPC5_SPI_DSPI0_TX1_DMA_CH_ID        4
#define SPC5_SPI_DSPI0_TX2_DMA_CH_ID        5
#define SPC5_SPI_DSPI0_RX_DMA_CH_ID         6
#define SPC5_SPI_DSPI1_TX1_DMA_CH_ID        7
#define SPC5_SPI_DSPI1_TX2_DMA_CH_ID        8
#define SPC5_SPI_DSPI1_RX_DMA_CH_ID         9
#define SPC5_SPI_DSPI2_TX1_DMA_CH_ID        10
#define SPC5_SPI_DSPI2_TX2_DMA_CH_ID        11
#define SPC5_SPI_DSPI2_RX_DMA_CH_ID         12
#define SPC5_SPI_DSPI3_TX1_DMA_CH_ID        13
#define SPC5_SPI_DSPI3_TX2_DMA_CH_ID        14
#define SPC5_SPI_DSPI3_RX_DMA_CH_ID         15
#define SPC5_SPI_DSPI4_TX1_DMA_CH_ID        1
#define SPC5_SPI_DSPI4_TX2_DMA_CH_ID        2
#define SPC5_SPI_DSPI4_RX_DMA_CH_ID         3
#define SPC5_SPI_DSPI0_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI3_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI4_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI0_IRQ_PRIO             10
#define SPC5_SPI_DSPI1_IRQ_PRIO             10
#define SPC5_SPI_DSPI2_IRQ_PRIO             10
#define SPC5_SPI_DSPI3_IRQ_PRIO             10
#define SPC5_SPI_DSPI4_IRQ_PRIO             10
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       osalSysHalt("DSPI DMA failure")

/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                FALSE

#define SPC5_CAN_USE_FLEXCAN0               FALSE
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN0_PRIORITY          12
#define SPC5_CAN_FLEXCAN0_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN0_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

/*
* ADC driver system settings.
*/
#define SPC5_ADC_USE_ADC0                   FALSE
#define SPC5_ADC_ADC0_CLK_FREQUENCY         HALF_PERIPHERAL_SET_CLOCK_FREQUENCY
#define SPC5_ADC_ADC0_AUTO_CLOCK_OFF        FALSE
#define SPC5_ADC_ADC0_WD_PRIORITY           12
#define SPC5_ADC_ADC0_DMA_CH_ID             1
#define SPC5_ADC_ADC0_DMA_IRQ_PRIO          12
#define SPC5_ADC_ADC0_START_PCTL            (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ADC_ADC0_STOP_PCTL             (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_ADC_USE_ADC1                   FALSE
#define SPC5_ADC_ADC1_CLK_FREQUENCY         HALF_PERIPHERAL_SET_CLOCK_FREQUENCY
#define SPC5_ADC_ADC1_AUTO_CLOCK_OFF        FALSE
#define SPC5_ADC_ADC1_WD_PRIORITY           12
#define SPC5_ADC_ADC1_DMA_CH_ID             2
#define SPC5_ADC_ADC1_DMA_IRQ_PRIO          12
#define SPC5_ADC_ADC1_START_PCTL            (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ADC_ADC1_STOP_PCTL             (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#endif /* MCUCONF_H */
