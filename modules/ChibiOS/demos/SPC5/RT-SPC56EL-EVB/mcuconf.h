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
 * SPC56ELxx drivers configuration.
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

#define SPC56ELxx_MCUCONF

/*
 * HAL driver system settings.
 */
#define SPC5_NO_INIT                        FALSE
#define SPC5_ALLOW_OVERCLOCK                FALSE
#define SPC5_DISABLE_WATCHDOG               TRUE
#define SPC5_FMPLL0_CLK_SRC                 SPC5_FMPLL_SRC_XOSC
#define SPC5_FMPLL0_IDF_VALUE               5
#define SPC5_FMPLL0_NDIV_VALUE              60
#define SPC5_FMPLL0_ODF                     SPC5_FMPLL_ODF_DIV4
#define SPC5_FMPLL1_CLK_SRC                 SPC5_FMPLL_SRC_XOSC
#define SPC5_FMPLL1_IDF_VALUE               5
#define SPC5_FMPLL1_NDIV_VALUE              60
#define SPC5_FMPLL1_ODF                     SPC5_FMPLL_ODF_DIV4
#define SPC5_SYSCLK_DIVIDER_VALUE           2
#define SPC5_AUX0CLK_SRC                    SPC5_CGM_SS_FMPLL1
#define SPC5_MCONTROL_DIVIDER_VALUE         15
#define SPC5_SWG_DIVIDER_VALUE              2
#define SPC5_AUX1CLK_SRC                    SPC5_CGM_SS_FMPLL1
#define SPC5_FLEXRAY_DIVIDER_VALUE          2
#define SPC5_AUX2CLK_SRC                    SPC5_CGM_SS_FMPLL1
#define SPC5_FLEXCAN_DIVIDER_VALUE          2
#define SPC5_ME_ME_BITS                     (SPC5_ME_ME_RUN1 |              \
                                             SPC5_ME_ME_RUN2 |              \
                                             SPC5_ME_ME_RUN3 |              \
                                             SPC5_ME_ME_HALT0 |             \
                                             SPC5_ME_ME_STOP0)
#define SPC5_ME_SAFE_MC_BITS                (SPC5_ME_MC_PDO)
#define SPC5_ME_DRUN_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_RUN0_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_RUN1_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_RUN2_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_RUN3_MC_BITS                (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_HALT0_MC_BITS               (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_STOP0_MC_BITS               (SPC5_ME_MC_SYSCLK_FMPLL0 |     \
                                             SPC5_ME_MC_IRCON |             \
                                             SPC5_ME_MC_XOSC0ON |           \
                                             SPC5_ME_MC_PLL0ON |            \
                                             SPC5_ME_MC_PLL1ON |            \
                                             SPC5_ME_MC_FLAON_NORMAL |      \
                                             SPC5_ME_MC_MVRON)
#define SPC5_ME_RUN_PC3_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#define SPC5_ME_RUN_PC4_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#define SPC5_ME_RUN_PC5_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#define SPC5_ME_RUN_PC6_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#define SPC5_ME_RUN_PC7_BITS                (SPC5_ME_RUN_PC_RUN0 |          \
                                             SPC5_ME_RUN_PC_RUN1 |          \
                                             SPC5_ME_RUN_PC_RUN2 |          \
                                             SPC5_ME_RUN_PC_RUN3)
#define SPC5_ME_LP_PC4_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#define SPC5_ME_LP_PC5_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#define SPC5_ME_LP_PC6_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#define SPC5_ME_LP_PC7_BITS                 (SPC5_ME_LP_PC_HALT0 |          \
                                             SPC5_ME_LP_PC_STOP0)
#define SPC5_CLOCK_FAILURE_HOOK()           chSysHalt("Clock failure")

/*
 * EDMA driver settings.
 */
#define SPC5_EDMA_CR_SETTING                0
#define SPC5_EDMA_GROUP0_PRIORITIES                                         \
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#define SPC5_EDMA_ERROR_HANDLER()           chSysHalt("DMA failure")

/*
 * SERIAL driver system settings.
 */
#define SPC5_SERIAL_USE_LINFLEX0            TRUE
#define SPC5_SERIAL_USE_LINFLEX1            TRUE
#define SPC5_SERIAL_LINFLEX0_PRIORITY       8
#define SPC5_SERIAL_LINFLEX1_PRIORITY       8
#define SPC5_SERIAL_LINFLEX0_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_SERIAL_LINFLEX0_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#define SPC5_SERIAL_LINFLEX1_START_PCTL     (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_SERIAL_LINFLEX1_STOP_PCTL      (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

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
#define SPC5_PWM_FLEXPWM0_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_PWM_FLEXPWM0_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_PWM_USE_SMOD4                  FALSE
#define SPC5_PWM_USE_SMOD5                  FALSE
#define SPC5_PWM_USE_SMOD6                  FALSE
#define SPC5_PWM_USE_SMOD7                  FALSE
#define SPC5_PWM_SMOD4_PRIORITY             7
#define SPC5_PWM_SMOD5_PRIORITY             7
#define SPC5_PWM_SMOD6_PRIORITY             7
#define SPC5_PWM_SMOD7_PRIORITY             7
#define SPC5_PWM_FLEXPWM1_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_PWM_FLEXPWM1_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

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
#define SPC5_ICU_ETIMER0_START_PCTL         (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ICU_ETIMER0_STOP_PCTL          (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_ICU_USE_SMOD6                  FALSE
#define SPC5_ICU_USE_SMOD7                  FALSE
#define SPC5_ICU_USE_SMOD8                  FALSE
#define SPC5_ICU_USE_SMOD9                  FALSE
#define SPC5_ICU_USE_SMOD10                 FALSE
#define SPC5_ICU_USE_SMOD11                 FALSE
#define SPC5_ICU_ETIMER1_PRIORITY           7
#define SPC5_ICU_ETIMER1_START_PCTL         (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ICU_ETIMER1_STOP_PCTL          (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_ICU_USE_SMOD12                 FALSE
#define SPC5_ICU_USE_SMOD13                 FALSE
#define SPC5_ICU_USE_SMOD14                 FALSE
#define SPC5_ICU_USE_SMOD15                 FALSE
#define SPC5_ICU_USE_SMOD16                 FALSE
#define SPC5_ICU_USE_SMOD17                 FALSE
#define SPC5_ICU_ETIMER2_PRIORITY           7
#define SPC5_ICU_ETIMER2_START_PCTL         (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ICU_ETIMER2_STOP_PCTL          (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI0                  FALSE
#define SPC5_SPI_USE_DSPI1                  FALSE
#define SPC5_SPI_USE_DSPI2                  FALSE
#define SPC5_SPI_DSPI0_MCR                  (SPC5_MCR_PCSIS0 |              \
                                             SPC5_MCR_PCSIS1 |              \
                                             SPC5_MCR_PCSIS2 |              \
                                             SPC5_MCR_PCSIS3 |              \
                                             SPC5_MCR_PCSIS4 |              \
                                             SPC5_MCR_PCSIS5 |              \
                                             SPC5_MCR_PCSIS6 |              \
                                             SPC5_MCR_PCSIS7)
#define SPC5_SPI_DSPI1_MCR                  (SPC5_MCR_PCSIS0 |              \
                                             SPC5_MCR_PCSIS1 |              \
                                             SPC5_MCR_PCSIS2 |              \
                                             SPC5_MCR_PCSIS3 |              \
                                             SPC5_MCR_PCSIS4 |              \
                                             SPC5_MCR_PCSIS5 |              \
                                             SPC5_MCR_PCSIS6 |              \
                                             SPC5_MCR_PCSIS7)
#define SPC5_SPI_DSPI2_MCR                  (SPC5_MCR_PCSIS0 |              \
                                             SPC5_MCR_PCSIS1 |              \
                                             SPC5_MCR_PCSIS2 |              \
                                             SPC5_MCR_PCSIS3 |              \
                                             SPC5_MCR_PCSIS4 |              \
                                             SPC5_MCR_PCSIS5 |              \
                                             SPC5_MCR_PCSIS6 |              \
                                             SPC5_MCR_PCSIS7)
#define SPC5_SPI_DSPI0_TX1_DMA_CH_ID        4
#define SPC5_SPI_DSPI0_TX2_DMA_CH_ID        5
#define SPC5_SPI_DSPI0_RX_DMA_CH_ID         6
#define SPC5_SPI_DSPI1_TX1_DMA_CH_ID        7
#define SPC5_SPI_DSPI1_TX2_DMA_CH_ID        8
#define SPC5_SPI_DSPI1_RX_DMA_CH_ID         9
#define SPC5_SPI_DSPI2_TX1_DMA_CH_ID        10
#define SPC5_SPI_DSPI2_TX2_DMA_CH_ID        11
#define SPC5_SPI_DSPI2_RX_DMA_CH_ID         12
#define SPC5_SPI_DSPI0_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         10
#define SPC5_SPI_DSPI0_IRQ_PRIO             10
#define SPC5_SPI_DSPI1_IRQ_PRIO             10
#define SPC5_SPI_DSPI2_IRQ_PRIO             10
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       chSysHalt("DMA failure")
#define SPC5_SPI_DSPI0_START_PCTL           (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_SPI_DSPI0_STOP_PCTL            (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#define SPC5_SPI_DSPI1_START_PCTL           (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_SPI_DSPI1_STOP_PCTL            (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))
#define SPC5_SPI_DSPI2_START_PCTL           (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_SPI_DSPI2_STOP_PCTL            (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#endif /* MCUCONF_H */
