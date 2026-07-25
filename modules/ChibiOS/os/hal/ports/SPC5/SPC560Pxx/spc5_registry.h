/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC560Pxx/spc5_registry.h
 * @brief   SPC560Pxx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef SPC5_REGISTRY_H
#define SPC5_REGISTRY_H

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(_SPC560P34L1_) || defined(_SPC560P34L3_)
#define _SPC560P34_
#define _SPC560PXX_SMALL_
#elif defined(_SPC560P40L1_) || defined(_SPC560P40L3_)
#define _SPC560P40_
#define _SPC560PXX_SMALL_
#elif defined(_SPC560P44L3_) || defined(_SPC560P44L5_)
#define _SPC560P44_
#define _SPC560PXX_MEDIUM_
#elif defined(_SPC560P50L3_) || defined(_SPC560P50L5_)
#define _SPC560P50_
#define _SPC560PXX_MEDIUM_
#elif defined(_SPC560P54L3_) || defined(_SPC560P54L5_) ||                   \
      defined(_SPC56AP54L3_) || defined(_SPC56AP54L5_)
#define _SPC560P54_
#define _SPC560PXX_LARGE_
#elif defined(_SPC560P60L3_) || defined(_SPC560P60L5_) ||                   \
      defined(_SPC56AP60L3_) || defined(_SPC56AP60L5_)
#define _SPC560P60_
#define _SPC560PXX_LARGE_
#else
#error "SPC56xPxx platform not defined"
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    SPC560Pxx capabilities
 * @{
 */
/* Clock attributes.*/
#if defined(_SPC560PXX_SMALL_)
#define SPC5_HAS_FMPLL1                     FALSE
#define SPC5_HAS_CLOCKOUT                   TRUE
#define SPC5_HAS_AC0                        FALSE
#define SPC5_HAS_AC1                        FALSE
#define SPC5_HAS_AC2                        FALSE
#define SPC5_HAS_AC3                        FALSE
#define SPC5_HAS_CMU0                       TRUE
#define SPC5_HAS_CMU1                       FALSE

#elif defined(_SPC560PXX_MEDIUM_)
#define SPC5_HAS_FMPLL1                     TRUE
#define SPC5_HAS_CLOCKOUT                   TRUE
#define SPC5_HAS_AC0                        TRUE
#define SPC5_HAS_AC1                        TRUE
#define SPC5_HAS_AC2                        TRUE
#define SPC5_HAS_AC3                        TRUE
#define SPC5_HAS_CMU0                       TRUE
#define SPC5_HAS_CMU1                       TRUE

#else /* defined(_SPC560PXX_LARGE_) */
#define SPC5_HAS_FMPLL1                     FALSE
#define SPC5_HAS_CLOCKOUT                   TRUE
#define SPC5_HAS_AC0                        FALSE
#define SPC5_HAS_AC1                        FALSE
#define SPC5_HAS_AC2                        FALSE
#define SPC5_HAS_AC3                        TRUE
#define SPC5_HAS_CMU0                       TRUE
#define SPC5_HAS_CMU1                       TRUE
#endif

/* DSPI attribures.*/
#define SPC5_HAS_DSPI0                      TRUE
#define SPC5_HAS_DSPI1                      TRUE
#define SPC5_HAS_DSPI2                      TRUE
#define SPC5_DSPI_FIFO_DEPTH                5
#define SPC5_DSPI0_PCTL                     4
#define SPC5_DSPI1_PCTL                     5
#define SPC5_DSPI2_PCTL                     6
#define SPC5_DSPI0_TX1_DMA_DEV_ID           1
#define SPC5_DSPI0_TX2_DMA_DEV_ID           0
#define SPC5_DSPI0_RX_DMA_DEV_ID            2
#define SPC5_DSPI1_TX1_DMA_DEV_ID           3
#define SPC5_DSPI1_TX2_DMA_DEV_ID           0
#define SPC5_DSPI1_RX_DMA_DEV_ID            4
#define SPC5_DSPI2_TX1_DMA_DEV_ID           5
#define SPC5_DSPI2_TX2_DMA_DEV_ID           0
#define SPC5_DSPI2_RX_DMA_DEV_ID            6
#define SPC5_DSPI0_TFFF_HANDLER             vector76
#define SPC5_DSPI0_TFFF_NUMBER              76
#define SPC5_DSPI0_RFDF_HANDLER             vector78
#define SPC5_DSPI0_RFDF_NUMBER              78
#define SPC5_DSPI1_TFFF_HANDLER             vector96
#define SPC5_DSPI1_TFFF_NUMBER              96
#define SPC5_DSPI1_RFDF_HANDLER             vector98
#define SPC5_DSPI1_RFDF_NUMBER              98
#define SPC5_DSPI2_TFFF_HANDLER             vector116
#define SPC5_DSPI2_TFFF_NUMBER              116
#define SPC5_DSPI2_RFDF_HANDLER             vector118
#define SPC5_DSPI2_RFDF_NUMBER              118
#define SPC5_DSPI0_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI0_PCTL, SPC5_SPI_DSPI0_START_PCTL)
#define SPC5_DSPI0_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI0_PCTL, SPC5_SPI_DSPI0_STOP_PCTL)
#define SPC5_DSPI1_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI1_PCTL, SPC5_SPI_DSPI1_START_PCTL)
#define SPC5_DSPI1_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI1_PCTL, SPC5_SPI_DSPI1_STOP_PCTL)
#define SPC5_DSPI2_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI2_PCTL, SPC5_SPI_DSPI2_START_PCTL)
#define SPC5_DSPI2_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI2_PCTL, SPC5_SPI_DSPI2_STOP_PCTL)

#if defined(_SPC560PXX_MEDIUM_) || defined(_SPC560PXX_LARGE_)
#define SPC5_HAS_DSPI3                      TRUE
#define SPC5_DSPI3_PCTL                     7
#define SPC5_DSPI3_TX1_DMA_DEV_ID           7
#define SPC5_DSPI3_TX2_DMA_DEV_ID           0
#define SPC5_DSPI3_RX_DMA_DEV_ID            8
#define SPC5_DSPI3_TFFF_HANDLER             vector219
#define SPC5_DSPI3_TFFF_NUMBER              219
#define SPC5_DSPI3_RFDF_HANDLER             vector221
#define SPC5_DSPI3_RFDF_NUMBER              221
#define SPC5_DSPI3_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI3_PCTL, SPC5_SPI_DSPI3_START_PCTL)
#define SPC5_DSPI3_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI3_PCTL, SPC5_SPI_DSPI3_STOP_PCTL)
#else
#define SPC5_HAS_DSPI3                      FALSE
#endif

#if defined(_SPC560PXX_LARGE_)
#define SPC5_HAS_DSPI4                      TRUE
#define SPC5_DSPI4_PCTL                     8
#define SPC5_DSPI4_TX1_DMA_DEV_ID           15
#define SPC5_DSPI4_TX2_DMA_DEV_ID           0
#define SPC5_DSPI4_RX_DMA_DEV_ID            21
#define SPC5_DSPI4_TFFF_HANDLER             vector258
#define SPC5_DSPI4_TFFF_NUMBER              258
#define SPC5_DSPI4_RFDF_HANDLER             vector260
#define SPC5_DSPI4_RFDF_NUMBER              260
#define SPC5_DSPI4_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI4_PCTL, SPC5_SPI_DSPI4_START_PCTL)
#define SPC5_DSPI4_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI0_PCTL, SPC5_SPI_DSPI4_STOP_PCTL)
#else
#define SPC5_HAS_DSPI4                      FALSE
#endif

#define SPC5_HAS_DSPI5                      FALSE
#define SPC5_HAS_DSPI6                      FALSE
#define SPC5_HAS_DSPI7                      FALSE

/* eDMA attributes.*/
#define SPC5_HAS_EDMA                       TRUE
#define SPC5_EDMA_NCHANNELS                 16
#define SPC5_EDMA_HAS_MUX                   TRUE

/* LINFlex attributes.*/
#define SPC5_HAS_LINFLEX0                   TRUE
#define SPC5_LINFLEX0_PCTL                  48
#define SPC5_LINFLEX0_RXI_HANDLER           vector79
#define SPC5_LINFLEX0_TXI_HANDLER           vector80
#define SPC5_LINFLEX0_ERR_HANDLER           vector81
#define SPC5_LINFLEX0_RXI_NUMBER            79
#define SPC5_LINFLEX0_TXI_NUMBER            80
#define SPC5_LINFLEX0_ERR_NUMBER            81
#define SPC5_LINFLEX0_CLK                   (halSPCGetSystemClock() / 1)

#define SPC5_HAS_LINFLEX1                   TRUE
#define SPC5_LINFLEX1_PCTL                  49
#define SPC5_LINFLEX1_RXI_HANDLER           vector99
#define SPC5_LINFLEX1_TXI_HANDLER           vector100
#define SPC5_LINFLEX1_ERR_HANDLER           vector101
#define SPC5_LINFLEX1_RXI_NUMBER            99
#define SPC5_LINFLEX1_TXI_NUMBER            100
#define SPC5_LINFLEX1_ERR_NUMBER            101
#define SPC5_LINFLEX1_CLK                   (halSPCGetSystemClock() / 1)

#define SPC5_HAS_LINFLEX2                   FALSE
#define SPC5_HAS_LINFLEX3                   FALSE
#define SPC5_HAS_LINFLEX4                   FALSE
#define SPC5_HAS_LINFLEX5                   FALSE
#define SPC5_HAS_LINFLEX6                   FALSE
#define SPC5_HAS_LINFLEX7                   FALSE
#define SPC5_HAS_LINFLEX8                   FALSE
#define SPC5_HAS_LINFLEX9                   FALSE

/* SIUL attributes.*/
#define SPC5_HAS_SIUL                       TRUE
#define SPC5_SIUL_NUM_PORTS                 8
#if defined(_SPC560PXX_SMALL_)
#define SPC5_SIUL_NUM_PCRS                  72
#else
#define SPC5_SIUL_NUM_PCRS                  108
#endif
#define SPC5_SIUL_NUM_PADSELS               36

/* FlexPWM attributes.*/
#if defined(_SPC560PXX_SMALL_) || defined(_SPC560PXX_MEDIUM_)
#define SPC5_HAS_FLEXPWM0                   TRUE
#define SPC5_FLEXPWM0_PCTL                  41
#define SPC5_FLEXPWM0_RF0_HANDLER           vector179
#define SPC5_FLEXPWM0_COF0_HANDLER          vector180
#define SPC5_FLEXPWM0_CAF0_HANDLER          vector181
#define SPC5_FLEXPWM0_RF1_HANDLER           vector182
#define SPC5_FLEXPWM0_COF1_HANDLER          vector183
#define SPC5_FLEXPWM0_CAF1_HANDLER          vector184
#define SPC5_FLEXPWM0_RF2_HANDLER           vector185
#define SPC5_FLEXPWM0_COF2_HANDLER          vector186
#define SPC5_FLEXPWM0_CAF2_HANDLER          vector187
#define SPC5_FLEXPWM0_RF3_HANDLER           vector188
#define SPC5_FLEXPWM0_COF3_HANDLER          vector189
#define SPC5_FLEXPWM0_CAF3_HANDLER          vector190
#define SPC5_FLEXPWM0_FFLAG_HANDLER         vector191
#define SPC5_FLEXPWM0_REF_HANDLER           vector192
#define SPC5_FLEXPWM0_RF0_NUMBER            179
#define SPC5_FLEXPWM0_COF0_NUMBER           180
#define SPC5_FLEXPWM0_CAF0_NUMBER           181
#define SPC5_FLEXPWM0_RF1_NUMBER            182
#define SPC5_FLEXPWM0_COF1_NUMBER           183
#define SPC5_FLEXPWM0_CAF1_NUMBER           184
#define SPC5_FLEXPWM0_RF2_NUMBER            185
#define SPC5_FLEXPWM0_COF2_NUMBER           186
#define SPC5_FLEXPWM0_CAF2_NUMBER           187
#define SPC5_FLEXPWM0_RF3_NUMBER            188
#define SPC5_FLEXPWM0_COF3_NUMBER           189
#define SPC5_FLEXPWM0_CAF3_NUMBER           190
#define SPC5_FLEXPWM0_FFLAG_NUMBER          191
#define SPC5_FLEXPWM0_REF_NUMBER            192
#define SPC5_FLEXPWM0_CLK                   SPC5_MCONTROL_CLK
#else /* defined(_SPC560PXX_LARGE_) */
#define SPC5_HAS_FLEXPWM0                   FALSE
#endif /* defined(_SPC560PXX_LARGE_) */

#define SPC5_HAS_FLEXPWM1                   FALSE

/* eTimer attributes.*/
#define SPC5_HAS_ETIMER0                    TRUE
#define SPC5_ETIMER0_PCTL                   38
#define SPC5_ETIMER0_TC0IR_HANDLER          vector157
#define SPC5_ETIMER0_TC1IR_HANDLER          vector158
#define SPC5_ETIMER0_TC2IR_HANDLER          vector159
#define SPC5_ETIMER0_TC3IR_HANDLER          vector160
#define SPC5_ETIMER0_TC4IR_HANDLER          vector161
#define SPC5_ETIMER0_TC5IR_HANDLER          vector162
#define SPC5_ETIMER0_WTIF_HANDLER           vector165
#define SPC5_ETIMER0_RCF_HANDLER            vector167
#define SPC5_ETIMER0_TC0IR_NUMBER           157
#define SPC5_ETIMER0_TC1IR_NUMBER           158
#define SPC5_ETIMER0_TC2IR_NUMBER           159
#define SPC5_ETIMER0_TC3IR_NUMBER           160
#define SPC5_ETIMER0_TC4IR_NUMBER           161
#define SPC5_ETIMER0_TC5IR_NUMBER           162
#define SPC5_ETIMER0_WTIF_NUMBER            165
#define SPC5_ETIMER0_RCF_NUMBER             167
#define SPC5_ETIMER0_CLK                    SPC5_MCONTROL_CLK

#if defined(_SPC560PXX_MEDIUM_) || defined(_SPC560PXX_LARGE_)
#define SPC5_HAS_ETIMER1                    TRUE
#define SPC5_ETIMER1_PCTL                   39
#define SPC5_ETIMER1_TC0IR_HANDLER          vector168
#define SPC5_ETIMER1_TC1IR_HANDLER          vector169
#define SPC5_ETIMER1_TC2IR_HANDLER          vector170
#define SPC5_ETIMER1_TC3IR_HANDLER          vector171
#define SPC5_ETIMER1_TC4IR_HANDLER          vector172
#define SPC5_ETIMER1_TC5IR_HANDLER          vector173
#define SPC5_ETIMER1_RCF_HANDLER            vector178
#define SPC5_ETIMER1_TC0IR_NUMBER           168
#define SPC5_ETIMER1_TC1IR_NUMBER           169
#define SPC5_ETIMER1_TC2IR_NUMBER           170
#define SPC5_ETIMER1_TC3IR_NUMBER           171
#define SPC5_ETIMER1_TC4IR_NUMBER           172
#define SPC5_ETIMER1_TC5IR_NUMBER           173
#define SPC5_ETIMER1_RCF_NUMBER             178
#define SPC5_ETIMER1_CLK                    SPC5_MCONTROL_CLK

#else /* !(defined(_SPC560PXX_MEDIUM_) || defined(_SPC560PXX_LARGE_)) */
#define SPC5_HAS_ETIMER1                    FALSE
#endif /* !(defined(_SPC560PXX_MEDIUM_) || defined(_SPC560PXX_LARGE_)) */

#define SPC5_HAS_ETIMER2                    FALSE
#define SPC5_HAS_ETIMER3                    FALSE

/* FlexCAN attributes.*/
#define SPC5_HAS_FLEXCAN0                                   TRUE
#define SPC5_FLEXCAN0_PCTL                                  16
#define SPC5_FLEXCAN0_MB                                    32
#define SPC5_FLEXCAN0_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_HANDLER           vector65
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_HANDLER              vector66
#define SPC5_FLEXCAN0_FLEXCAN_ESR_WAK_HANDLER               vector67
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_03_HANDLER             vector68
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_07_HANDLER             vector69
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_11_HANDLER             vector70
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_15_HANDLER             vector71
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_HANDLER             vector72
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_NUMBER            65
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_NUMBER               66
#define SPC5_FLEXCAN0_FLEXCAN_ESR_WAK_NUMBER                67
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_03_NUMBER              68
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_07_NUMBER              69
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_11_NUMBER              70
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_15_NUMBER              71
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_NUMBER              72
#define SPC5_FLEXCAN0_ENABLE_CLOCK()                                        \
  halSPCSetPeripheralClockMode(SPC5_FLEXCAN0_PCTL, SPC5_CAN_FLEXCAN0_START_PCTL)
#define SPC5_FLEXCAN0_DISABLE_CLOCK()                                       \
  halSPCSetPeripheralClockMode(SPC5_FLEXCAN0_PCTL, SPC5_CAN_FLEXCAN0_STOP_PCTL)

/* ADC attributes.*/
#define SPC5_ADC_HAS_TRC                    TRUE

#define SPC5_HAS_ADC0                       TRUE
#define SPC5_ADC_ADC0_HAS_CTR0              TRUE
#define SPC5_ADC_ADC0_HAS_CTR1              FALSE
#define SPC5_ADC_ADC0_HAS_CTR2              FALSE
#define SPC5_ADC_ADC0_HAS_NCMR0             TRUE
#define SPC5_ADC_ADC0_HAS_NCMR1             FALSE
#define SPC5_ADC_ADC0_HAS_NCMR2             FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR0           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR1           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR2           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR3           TRUE
#define SPC5_ADC_ADC0_HAS_CIMR0             FALSE
#define SPC5_ADC_ADC0_HAS_CIMR1             FALSE
#define SPC5_ADC_ADC0_HAS_CIMR2             FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR0           FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR1           FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR2           FALSE
#define SPC5_ADC0_PCTL                      32
#define SPC5_ADC0_DMA_DEV_ID                20
#define SPC5_ADC0_EOC_HANDLER               vector62
#define SPC5_ADC0_EOC_NUMBER                62
#define SPC5_ADC0_WD_HANDLER                vector64
#define SPC5_ADC0_WD_NUMBER                 64

#define SPC5_HAS_ADC1                       TRUE
#define SPC5_ADC_ADC1_HAS_CTR0              TRUE
#define SPC5_ADC_ADC1_HAS_CTR1              FALSE
#define SPC5_ADC_ADC1_HAS_CTR2              FALSE
#define SPC5_ADC_ADC1_HAS_NCMR0             TRUE
#define SPC5_ADC_ADC1_HAS_NCMR1             FALSE
#define SPC5_ADC_ADC1_HAS_NCMR2             FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR0           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR1           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR2           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR3           TRUE
#define SPC5_ADC_ADC0_HAS_CIMR0             FALSE
#define SPC5_ADC_ADC0_HAS_CIMR1             FALSE
#define SPC5_ADC_ADC0_HAS_CIMR2             FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR0           FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR1           FALSE
#define SPC5_ADC_ADC0_HAS_CEOCFR2           FALSE
#define SPC5_ADC1_PCTL                      33
#define SPC5_ADC1_DMA_DEV_ID                21
#define SPC5_ADC1_EOC_HANDLER               vector82
#define SPC5_ADC1_EOC_NUMBER                82
#define SPC5_ADC1_WD_HANDLER                vector84
#define SPC5_ADC1_WD_NUMBER                 84
/** @} */

#endif /* SPC5_REGISTRY_H */

/** @} */
