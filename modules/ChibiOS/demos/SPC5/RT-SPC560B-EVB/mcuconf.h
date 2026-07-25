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
 * SPC560Bxx drivers configuration.
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

#define SPC560Bxx_MCUCONF

/*
 * HAL driver system settings.
 */
#define SPC5_NO_INIT                        FALSE
#define SPC5_ALLOW_OVERCLOCK                FALSE
#define SPC5_DISABLE_WATCHDOG               TRUE
#define SPC5_FMPLL0_IDF_VALUE               1
#define SPC5_FMPLL0_NDIV_VALUE              32
#define SPC5_FMPLL0_ODF                     SPC5_FMPLL_ODF_DIV4
#define SPC5_XOSCDIV_VALUE                  1
#define SPC5_IRCDIV_VALUE                   1
#define SPC5_PERIPHERAL1_CLK_DIV_VALUE      2
#define SPC5_PERIPHERAL2_CLK_DIV_VALUE      2
#define SPC5_PERIPHERAL3_CLK_DIV_VALUE      2
#define SPC5_CLOCK_FAILURE_HOOK()           osalSysHalt("clock failure")

#define SPC5_EMIOS0_GPRE_VALUE              20
#define SPC5_EMIOS1_GPRE_VALUE              20

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
 * SERIAL driver system settings.
 */
#define SPC5_SERIAL_USE_LINFLEX0            TRUE
#define SPC5_SERIAL_USE_LINFLEX1            FALSE
#define SPC5_SERIAL_USE_LINFLEX2            FALSE
#define SPC5_SERIAL_USE_LINFLEX3            FALSE
#define SPC5_SERIAL_USE_LINFLEX4            FALSE
#define SPC5_SERIAL_USE_LINFLEX5            FALSE
#define SPC5_SERIAL_USE_LINFLEX6            FALSE
#define SPC5_SERIAL_USE_LINFLEX7            FALSE
#define SPC5_SERIAL_USE_LINFLEX8            FALSE
#define SPC5_SERIAL_USE_LINFLEX9            FALSE
#define SPC5_SERIAL_LINFLEX0_PRIORITY       8
#define SPC5_SERIAL_LINFLEX1_PRIORITY       8
#define SPC5_SERIAL_LINFLEX2_PRIORITY       8
#define SPC5_SERIAL_LINFLEX3_PRIORITY       8
#define SPC5_SERIAL_LINFLEX4_PRIORITY       8
#define SPC5_SERIAL_LINFLEX5_PRIORITY       8
#define SPC5_SERIAL_LINFLEX6_PRIORITY       8
#define SPC5_SERIAL_LINFLEX7_PRIORITY       8
#define SPC5_SERIAL_LINFLEX8_PRIORITY       8
#define SPC5_SERIAL_LINFLEX9_PRIORITY       8

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI0                  FALSE
#define SPC5_SPI_USE_DSPI1                  FALSE
#define SPC5_SPI_USE_DSPI2                  FALSE
#define SPC5_SPI_USE_DSPI3                  FALSE
#define SPC5_SPI_USE_DSPI4                  FALSE
#define SPC5_SPI_USE_DSPI5                  FALSE
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_RX_ONLY
#define SPC5_SPI_DSPI0_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4 | SPC5_MCR_PCSIS5)
#define SPC5_SPI_DSPI1_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3 | SPC5_MCR_PCSIS4)
#define SPC5_SPI_DSPI2_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1 | SPC5_MCR_PCSIS2 | SPC5_MCR_PCSIS3)
#define SPC5_SPI_DSPI3_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1)
#define SPC5_SPI_DSPI4_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1)
#define SPC5_SPI_DSPI5_MCR                  (0 | SPC5_MCR_PCSIS0 | SPC5_MCR_PCSIS1)
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
#define SPC5_SPI_DSPI5_TX1_DMA_CH_ID        4
#define SPC5_SPI_DSPI5_TX2_DMA_CH_ID        5
#define SPC5_SPI_DSPI5_RX_DMA_CH_ID         6
#define SPC5_SPI_DSPI0_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI3_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI4_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI5_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI0_IRQ_PRIO             10
#define SPC5_SPI_DSPI1_IRQ_PRIO             10
#define SPC5_SPI_DSPI2_IRQ_PRIO             10
#define SPC5_SPI_DSPI3_IRQ_PRIO             10
#define SPC5_SPI_DSPI4_IRQ_PRIO             10
#define SPC5_SPI_DSPI5_IRQ_PRIO             10
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       osalSysHalt("DSPI DMA failure")

/*
 * ICU-PWM driver system settings.
 */
#define SPC5_ICU_USE_EMIOS0_CH0             FALSE
#define SPC5_ICU_USE_EMIOS0_CH1             FALSE
#define SPC5_ICU_USE_EMIOS0_CH2             FALSE
#define SPC5_ICU_USE_EMIOS0_CH3             FALSE
#define SPC5_ICU_USE_EMIOS0_CH4             FALSE
#define SPC5_ICU_USE_EMIOS0_CH5             FALSE
#define SPC5_ICU_USE_EMIOS0_CH6             FALSE
#define SPC5_ICU_USE_EMIOS0_CH7             FALSE
#define SPC5_ICU_USE_EMIOS0_CH24            FALSE

#define SPC5_PWM_USE_EMIOS0_GROUP0          FALSE
#define SPC5_PWM_USE_EMIOS0_GROUP1          FALSE

#define SPC5_EMIOS0_GFR_F0F1_PRIORITY       8
#define SPC5_EMIOS0_GFR_F2F3_PRIORITY       8
#define SPC5_EMIOS0_GFR_F4F5_PRIORITY       8
#define SPC5_EMIOS0_GFR_F6F7_PRIORITY       8
#define SPC5_EMIOS0_GFR_F8F9_PRIORITY       8
#define SPC5_EMIOS0_GFR_F10F11_PRIORITY     8
#define SPC5_EMIOS0_GFR_F12F13_PRIORITY     8
#define SPC5_EMIOS0_GFR_F14F15_PRIORITY     8
#define SPC5_EMIOS0_GFR_F16F17_PRIORITY     8
#define SPC5_EMIOS0_GFR_F18F19_PRIORITY     8
#define SPC5_EMIOS0_GFR_F20F21_PRIORITY     8
#define SPC5_EMIOS0_GFR_F22F23_PRIORITY     8
#define SPC5_EMIOS0_GFR_F24F25_PRIORITY     8

#define SPC5_EMIOS0_START_PCTL              (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_EMIOS0_STOP_PCTL               (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_ICU_USE_EMIOS1_CH24            FALSE

#define SPC5_PWM_USE_EMIOS1_GROUP0          FALSE
#define SPC5_PWM_USE_EMIOS1_GROUP1          FALSE
#define SPC5_PWM_USE_EMIOS1_GROUP2          FALSE

#define SPC5_EMIOS1_GFR_F0F1_PRIORITY       8
#define SPC5_EMIOS1_GFR_F2F3_PRIORITY       8
#define SPC5_EMIOS1_GFR_F4F5_PRIORITY       8
#define SPC5_EMIOS1_GFR_F6F7_PRIORITY       8
#define SPC5_EMIOS1_GFR_F8F9_PRIORITY       8
#define SPC5_EMIOS1_GFR_F10F11_PRIORITY     8
#define SPC5_EMIOS1_GFR_F12F13_PRIORITY     8
#define SPC5_EMIOS1_GFR_F14F15_PRIORITY     8
#define SPC5_EMIOS1_GFR_F16F17_PRIORITY     8
#define SPC5_EMIOS1_GFR_F18F19_PRIORITY     8
#define SPC5_EMIOS1_GFR_F20F21_PRIORITY     8
#define SPC5_EMIOS1_GFR_F22F23_PRIORITY     8
#define SPC5_EMIOS1_GFR_F24F25_PRIORITY     8

#define SPC5_EMIOS1_START_PCTL              (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_EMIOS1_STOP_PCTL               (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                FALSE

#define SPC5_CAN_USE_FLEXCAN0               FALSE
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN0_PRIORITY          11
#define SPC5_CAN_FLEXCAN0_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN0_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN1               FALSE
#define SPC5_CAN_FLEXCAN1_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN1_PRIORITY          11
#define SPC5_CAN_FLEXCAN1_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN1_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN2               FALSE
#define SPC5_CAN_FLEXCAN2_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN2_PRIORITY          11
#define SPC5_CAN_FLEXCAN2_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN2_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN3               FALSE
#define SPC5_CAN_FLEXCAN3_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN3_PRIORITY          11
#define SPC5_CAN_FLEXCAN3_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN3_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN4               FALSE
#define SPC5_CAN_FLEXCAN4_USE_EXT_CLK       FALSE
#define SPC5_CAN_FLEXCAN4_PRIORITY          11
#define SPC5_CAN_FLEXCAN4_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN4_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN5               FALSE
#define SPC5_CAN_FLEXCAN5_PRIORITY          11
#define SPC5_CAN_FLEXCAN5_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN5_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
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
