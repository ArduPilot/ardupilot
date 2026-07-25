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
 * @file    SPC560Bxx/spc560b_registry.h
 * @brief   SPC560Bxx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef SPC5_REGISTRY_H
#define SPC5_REGISTRY_H

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(_SPC560B54L3_)
#define SPC5_NUM_DSPI                       3
#define SPC5_NUM_LINFLEX                    4
#define SPC5_NUM_GPIO                       77

#elif defined(_SPC560B54L5_)
#define SPC5_NUM_DSPI                       5
#define SPC5_NUM_LINFLEX                    6
#define SPC5_NUM_GPIO                       121

#elif defined(_SPC560B60L3_)
#define SPC5_NUM_DSPI                       3
#define SPC5_NUM_LINFLEX                    6
#define SPC5_NUM_GPIO                       77

#elif defined(_SPC560B60L5_)
#define SPC5_NUM_DSPI                       5
#define SPC5_NUM_LINFLEX                    6
#define SPC5_NUM_GPIO                       121

#elif defined(_SPC560B60L7_)
#define SPC5_NUM_DSPI                       6
#define SPC5_NUM_LINFLEX                    8
#define SPC5_NUM_GPIO                       149

#elif defined(_SPC560B64L5_)
#define SPC5_NUM_DSPI                       5
#define SPC5_NUM_LINFLEX                    8
#define SPC5_NUM_GPIO                       149

#elif defined(_SPC560B64L7_)
#define SPC5_NUM_DSPI                       6
#define SPC5_NUM_LINFLEX                    10
#define SPC5_NUM_GPIO                       149

#else
#error "SPC560Bxx platform not defined"
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    SPC560Bxx capabilities
 * @{
 */
/* DSPI attribures.*/
#define SPC5_DSPI_FIFO_DEPTH                4

#if SPC5_NUM_DSPI > 0
#define SPC5_HAS_DSPI0                      TRUE
#define SPC5_DSPI0_PCTL                     4
#define SPC5_DSPI0_TX1_DMA_DEV_ID           1
#define SPC5_DSPI0_TX2_DMA_DEV_ID           0
#define SPC5_DSPI0_RX_DMA_DEV_ID            2
#define SPC5_DSPI0_TFFF_HANDLER             vector76
#define SPC5_DSPI0_TFFF_NUMBER              76
#define SPC5_DSPI0_RFDF_HANDLER             vector78
#define SPC5_DSPI0_RFDF_NUMBER              78
#define SPC5_DSPI0_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI0_PCTL, SPC5_SPI_DSPI0_START_PCTL)
#define SPC5_DSPI0_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI0_PCTL, SPC5_SPI_DSPI0_STOP_PCTL)
#else
#define SPC5_HAS_DSPI0                      FALSE
#endif

#if SPC5_NUM_DSPI > 1
#define SPC5_HAS_DSPI1                      TRUE
#define SPC5_DSPI1_PCTL                     5
#define SPC5_DSPI1_TX1_DMA_DEV_ID           3
#define SPC5_DSPI1_TX2_DMA_DEV_ID           0
#define SPC5_DSPI1_RX_DMA_DEV_ID            4
#define SPC5_DSPI1_TFFF_HANDLER             vector96
#define SPC5_DSPI1_TFFF_NUMBER              96
#define SPC5_DSPI1_RFDF_HANDLER             vector98
#define SPC5_DSPI1_RFDF_NUMBER              98
#define SPC5_DSPI1_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI1_PCTL, SPC5_SPI_DSPI1_START_PCTL)
#define SPC5_DSPI1_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI1_PCTL, SPC5_SPI_DSPI1_STOP_PCTL)
#else
#define SPC5_HAS_DSPI1                      FALSE
#endif

#if SPC5_NUM_DSPI > 2
#define SPC5_HAS_DSPI2                      TRUE
#define SPC5_DSPI2_PCTL                     6
#define SPC5_DSPI2_TX1_DMA_DEV_ID           5
#define SPC5_DSPI2_TX2_DMA_DEV_ID           0
#define SPC5_DSPI2_RX_DMA_DEV_ID            6
#define SPC5_DSPI2_TFFF_HANDLER             vector116
#define SPC5_DSPI2_TFFF_NUMBER              116
#define SPC5_DSPI2_RFDF_HANDLER             vector118
#define SPC5_DSPI2_RFDF_NUMBER              118
#define SPC5_DSPI2_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI2_PCTL, SPC5_SPI_DSPI2_START_PCTL)
#define SPC5_DSPI2_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI2_PCTL, SPC5_SPI_DSPI2_STOP_PCTL)
#else
#define SPC5_HAS_DSPI2                      FALSE
#endif

#if SPC5_NUM_DSPI > 3
#define SPC5_HAS_DSPI3                      TRUE
#define SPC5_DSPI3_PCTL                     7
#define SPC5_DSPI3_TX1_DMA_DEV_ID           7
#define SPC5_DSPI3_TX2_DMA_DEV_ID           0
#define SPC5_DSPI3_RX_DMA_DEV_ID            8
#define SPC5_DSPI3_TFFF_HANDLER             vector184
#define SPC5_DSPI3_TFFF_NUMBER              184
#define SPC5_DSPI3_RFDF_HANDLER             vector186
#define SPC5_DSPI3_RFDF_NUMBER              186
#define SPC5_DSPI3_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI3_PCTL, SPC5_SPI_DSPI3_START_PCTL)
#define SPC5_DSPI3_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI3_PCTL, SPC5_SPI_DSPI3_STOP_PCTL)
#else
#define SPC5_HAS_DSPI3                      FALSE
#endif

#if SPC5_NUM_DSPI > 4
#define SPC5_HAS_DSPI4                      TRUE
#define SPC5_DSPI4_PCTL                     8
#define SPC5_DSPI4_TX1_DMA_DEV_ID           9
#define SPC5_DSPI4_TX2_DMA_DEV_ID           0
#define SPC5_DSPI4_RX_DMA_DEV_ID            10
#define SPC5_DSPI4_TFFF_HANDLER             vector213
#define SPC5_DSPI4_TFFF_NUMBER              213
#define SPC5_DSPI4_RFDF_HANDLER             vector215
#define SPC5_DSPI4_RFDF_NUMBER              215
#define SPC5_DSPI4_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI4_PCTL, SPC5_SPI_DSPI4_START_PCTL)
#define SPC5_DSPI4_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI4_PCTL, SPC5_SPI_DSPI4_STOP_PCTL)
#else
#define SPC5_HAS_DSPI4                      FALSE
#endif

#if SPC5_NUM_DSPI > 5
#define SPC5_HAS_DSPI5                      TRUE
#define SPC5_DSPI5_PCTL                     9
#define SPC5_DSPI5_TX1_DMA_DEV_ID           11
#define SPC5_DSPI5_TX2_DMA_DEV_ID           0
#define SPC5_DSPI5_RX_DMA_DEV_ID            12
#define SPC5_DSPI5_TFFF_HANDLER             vector221
#define SPC5_DSPI5_TFFF_NUMBER              221
#define SPC5_DSPI5_RFDF_HANDLER             vector223
#define SPC5_DSPI5_RFDF_NUMBER              223
#define SPC5_DSPI5_ENABLE_CLOCK()                                           \
  halSPCSetPeripheralClockMode(SPC5_DSPI5_PCTL, SPC5_SPI_DSPI5_START_PCTL)
#define SPC5_DSPI5_DISABLE_CLOCK()                                          \
  halSPCSetPeripheralClockMode(SPC5_DSPI5_PCTL, SPC5_SPI_DSPI5_STOP_PCTL)
#else
#define SPC5_HAS_DSPI5                      FALSE
#endif

#define SPC5_HAS_DSPI6                      FALSE
#define SPC5_HAS_DSPI7                      FALSE

/* eDMA attributes.*/
#define SPC5_HAS_EDMA                       TRUE
#define SPC5_EDMA_NCHANNELS                 16
#define SPC5_EDMA_HAS_MUX                   TRUE
#define SPC5_EDMA_MUX_PCTL                  23

/* LINFlex attributes.*/
#if SPC5_NUM_LINFLEX > 0
#define SPC5_HAS_LINFLEX0                   TRUE
#define SPC5_LINFLEX0_PCTL                  48
#define SPC5_LINFLEX0_RXI_HANDLER           vector79
#define SPC5_LINFLEX0_TXI_HANDLER           vector80
#define SPC5_LINFLEX0_ERR_HANDLER           vector81
#define SPC5_LINFLEX0_RXI_NUMBER            79
#define SPC5_LINFLEX0_TXI_NUMBER            80
#define SPC5_LINFLEX0_ERR_NUMBER            81
#define SPC5_LINFLEX0_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX0                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 1
#define SPC5_HAS_LINFLEX1                   TRUE
#define SPC5_LINFLEX1_PCTL                  49
#define SPC5_LINFLEX1_RXI_HANDLER           vector99
#define SPC5_LINFLEX1_TXI_HANDLER           vector100
#define SPC5_LINFLEX1_ERR_HANDLER           vector101
#define SPC5_LINFLEX1_RXI_NUMBER            99
#define SPC5_LINFLEX1_TXI_NUMBER            100
#define SPC5_LINFLEX1_ERR_NUMBER            101
#define SPC5_LINFLEX1_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX1                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 2
#define SPC5_HAS_LINFLEX2                   TRUE
#define SPC5_LINFLEX2_PCTL                  50
#define SPC5_LINFLEX2_RXI_HANDLER           vector119
#define SPC5_LINFLEX2_TXI_HANDLER           vector120
#define SPC5_LINFLEX2_ERR_HANDLER           vector121
#define SPC5_LINFLEX2_RXI_NUMBER            119
#define SPC5_LINFLEX2_TXI_NUMBER            120
#define SPC5_LINFLEX2_ERR_NUMBER            121
#define SPC5_LINFLEX2_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX2                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 3
#define SPC5_HAS_LINFLEX3                   TRUE
#define SPC5_LINFLEX3_PCTL                  51
#define SPC5_LINFLEX3_RXI_HANDLER           vector122
#define SPC5_LINFLEX3_TXI_HANDLER           vector123
#define SPC5_LINFLEX3_ERR_HANDLER           vector124
#define SPC5_LINFLEX3_RXI_NUMBER            122
#define SPC5_LINFLEX3_TXI_NUMBER            123
#define SPC5_LINFLEX3_ERR_NUMBER            124
#define SPC5_LINFLEX3_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX3                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 4
#define SPC5_HAS_LINFLEX4                   TRUE
#define SPC5_LINFLEX4_PCTL                  52
#define SPC5_LINFLEX4_RXI_HANDLER           vector187
#define SPC5_LINFLEX4_TXI_HANDLER           vector188
#define SPC5_LINFLEX4_ERR_HANDLER           vector189
#define SPC5_LINFLEX4_RXI_NUMBER            187
#define SPC5_LINFLEX4_TXI_NUMBER            188
#define SPC5_LINFLEX4_ERR_NUMBER            189
#define SPC5_LINFLEX4_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX4                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 5
#define SPC5_HAS_LINFLEX5                   TRUE
#define SPC5_LINFLEX5_PCTL                  53
#define SPC5_LINFLEX5_RXI_HANDLER           vector199
#define SPC5_LINFLEX5_TXI_HANDLER           vector200
#define SPC5_LINFLEX5_ERR_HANDLER           vector201
#define SPC5_LINFLEX5_RXI_NUMBER            199
#define SPC5_LINFLEX5_TXI_NUMBER            200
#define SPC5_LINFLEX5_ERR_NUMBER            201
#define SPC5_LINFLEX5_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX5                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 6
#define SPC5_HAS_LINFLEX6                   TRUE
#define SPC5_LINFLEX6_PCTL                  54
#define SPC5_LINFLEX6_RXI_HANDLER           vector216
#define SPC5_LINFLEX6_TXI_HANDLER           vector217
#define SPC5_LINFLEX6_ERR_HANDLER           vector218
#define SPC5_LINFLEX6_RXI_NUMBER            216
#define SPC5_LINFLEX6_TXI_NUMBER            217
#define SPC5_LINFLEX6_ERR_NUMBER            218
#define SPC5_LINFLEX6_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX6                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 7
#define SPC5_HAS_LINFLEX7                   TRUE
#define SPC5_LINFLEX7_PCTL                  55
#define SPC5_LINFLEX7_RXI_HANDLER           vector224
#define SPC5_LINFLEX7_TXI_HANDLER           vector225
#define SPC5_LINFLEX7_ERR_HANDLER           vector226
#define SPC5_LINFLEX7_RXI_NUMBER            224
#define SPC5_LINFLEX7_TXI_NUMBER            225
#define SPC5_LINFLEX7_ERR_NUMBER            226
#define SPC5_LINFLEX7_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX7                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 8
#define SPC5_HAS_LINFLEX8                   TRUE
#define SPC5_LINFLEX8_PCTL                  12
#define SPC5_LINFLEX8_RXI_HANDLER           vector227
#define SPC5_LINFLEX8_TXI_HANDLER           vector228
#define SPC5_LINFLEX8_ERR_HANDLER           vector229
#define SPC5_LINFLEX8_RXI_NUMBER            227
#define SPC5_LINFLEX8_TXI_NUMBER            228
#define SPC5_LINFLEX8_ERR_NUMBER            229
#define SPC5_LINFLEX8_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX8                   FALSE
#endif

#if SPC5_NUM_LINFLEX > 9
#define SPC5_HAS_LINFLEX9                   TRUE
#define SPC5_LINFLEX9_PCTL                  13
#define SPC5_LINFLEX9_RXI_HANDLER           vector230
#define SPC5_LINFLEX9_TXI_HANDLER           vector231
#define SPC5_LINFLEX9_ERR_HANDLER           vector232
#define SPC5_LINFLEX9_RXI_NUMBER            230
#define SPC5_LINFLEX9_TXI_NUMBER            231
#define SPC5_LINFLEX9_ERR_NUMBER            232
#define SPC5_LINFLEX9_CLK                   (halSPCGetSystemClock() /       \
                                             SPC5_PERIPHERAL1_CLK_DIV_VALUE)
#else
#define SPC5_HAS_LINFLEX9                   FALSE
#endif

/* SIUL attributes.*/
#define SPC5_HAS_SIUL                       TRUE
#define SPC5_SIUL_PCTL                      68
#define SPC5_SIUL_NUM_PORTS                 10
#define SPC5_SIUL_NUM_PCRS                  149
#define SPC5_SIUL_NUM_PADSELS               64
#define SPC5_SIUL_SYSTEM_PINS               32,33,121,122

/* eMIOS attributes.*/
#define SPC5_HAS_EMIOS0                     TRUE
#define SPC5_EMIOS0_PCTL                    72
#define SPC5_EMIOS0_GFR_F0F1_HANDLER        vector141
#define SPC5_EMIOS0_GFR_F2F3_HANDLER        vector142
#define SPC5_EMIOS0_GFR_F4F5_HANDLER        vector143
#define SPC5_EMIOS0_GFR_F6F7_HANDLER        vector144
#define SPC5_EMIOS0_GFR_F8F9_HANDLER        vector145
#define SPC5_EMIOS0_GFR_F10F11_HANDLER      vector146
#define SPC5_EMIOS0_GFR_F12F13_HANDLER      vector147
#define SPC5_EMIOS0_GFR_F14F15_HANDLER      vector148
#define SPC5_EMIOS0_GFR_F16F17_HANDLER      vector149
#define SPC5_EMIOS0_GFR_F18F19_HANDLER      vector150
#define SPC5_EMIOS0_GFR_F20F21_HANDLER      vector151
#define SPC5_EMIOS0_GFR_F22F23_HANDLER      vector152
#define SPC5_EMIOS0_GFR_F24F25_HANDLER      vector153
#define SPC5_EMIOS0_GFR_F26F27_HANDLER      vector154
#define SPC5_EMIOS0_GFR_F0F1_NUMBER         141
#define SPC5_EMIOS0_GFR_F2F3_NUMBER         142
#define SPC5_EMIOS0_GFR_F4F5_NUMBER         143
#define SPC5_EMIOS0_GFR_F6F7_NUMBER         144
#define SPC5_EMIOS0_GFR_F8F9_NUMBER         145
#define SPC5_EMIOS0_GFR_F10F11_NUMBER       146
#define SPC5_EMIOS0_GFR_F12F13_NUMBER       147
#define SPC5_EMIOS0_GFR_F14F15_NUMBER       148
#define SPC5_EMIOS0_GFR_F16F17_NUMBER       149
#define SPC5_EMIOS0_GFR_F18F19_NUMBER       150
#define SPC5_EMIOS0_GFR_F20F21_NUMBER       151
#define SPC5_EMIOS0_GFR_F22F23_NUMBER       152
#define SPC5_EMIOS0_GFR_F24F25_NUMBER       153
#define SPC5_EMIOS0_GFR_F26F27_NUMBER       154

#define SPC5_EMIOS0_CLK                     (halSPCGetSystemClock() /         \
                                             SPC5_PERIPHERAL3_CLK_DIV_VALUE / \
                                             SPC5_EMIOS0_GPRE_VALUE)

#define SPC5_HAS_EMIOS1                     TRUE
#define SPC5_EMIOS1_PCTL                    73
#define SPC5_EMIOS1_GFR_F0F1_HANDLER        vector157
#define SPC5_EMIOS1_GFR_F2F3_HANDLER        vector158
#define SPC5_EMIOS1_GFR_F4F5_HANDLER        vector159
#define SPC5_EMIOS1_GFR_F6F7_HANDLER        vector160
#define SPC5_EMIOS1_GFR_F8F9_HANDLER        vector161
#define SPC5_EMIOS1_GFR_F10F11_HANDLER      vector162
#define SPC5_EMIOS1_GFR_F12F13_HANDLER      vector163
#define SPC5_EMIOS1_GFR_F14F15_HANDLER      vector164
#define SPC5_EMIOS1_GFR_F16F17_HANDLER      vector165
#define SPC5_EMIOS1_GFR_F18F19_HANDLER      vector166
#define SPC5_EMIOS1_GFR_F20F21_HANDLER      vector167
#define SPC5_EMIOS1_GFR_F22F23_HANDLER      vector168
#define SPC5_EMIOS1_GFR_F24F25_HANDLER      vector169
#define SPC5_EMIOS1_GFR_F26F27_HANDLER      vector170
#define SPC5_EMIOS1_GFR_F0F1_NUMBER         157
#define SPC5_EMIOS1_GFR_F2F3_NUMBER         158
#define SPC5_EMIOS1_GFR_F4F5_NUMBER         159
#define SPC5_EMIOS1_GFR_F6F7_NUMBER         160
#define SPC5_EMIOS1_GFR_F8F9_NUMBER         161
#define SPC5_EMIOS1_GFR_F10F11_NUMBER       162
#define SPC5_EMIOS1_GFR_F12F13_NUMBER       163
#define SPC5_EMIOS1_GFR_F14F15_NUMBER       164
#define SPC5_EMIOS1_GFR_F16F17_NUMBER       165
#define SPC5_EMIOS1_GFR_F18F19_NUMBER       166
#define SPC5_EMIOS1_GFR_F20F21_NUMBER       167
#define SPC5_EMIOS1_GFR_F22F23_NUMBER       168
#define SPC5_EMIOS1_GFR_F24F25_NUMBER       169
#define SPC5_EMIOS1_GFR_F26F27_NUMBER       170

#define SPC5_EMIOS1_CLK                     (halSPCGetSystemClock() /         \
                                             SPC5_PERIPHERAL3_CLK_DIV_VALUE / \
                                             SPC5_EMIOS1_GPRE_VALUE)

/* FlexCAN attributes.*/
#define SPC5_HAS_FLEXCAN0                                   TRUE
#define SPC5_FLEXCAN0_PCTL                                  16
#define SPC5_FLEXCAN0_MB                                    64
#define SPC5_FLEXCAN0_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_HANDLER           vector65
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_HANDLER              vector66
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_03_HANDLER             vector68
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_07_HANDLER             vector69
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_11_HANDLER             vector70
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_15_HANDLER             vector71
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_HANDLER             vector72
#define SPC5_FLEXCAN0_FLEXCAN_BUF_32_63_HANDLER             vector73
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_NUMBER            65
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_NUMBER               66
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_03_NUMBER              68
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_07_NUMBER              69
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_11_NUMBER              70
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_15_NUMBER              71
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_NUMBER              72
#define SPC5_FLEXCAN0_FLEXCAN_BUF_32_63_NUMBER              73
#define SPC5_FLEXCAN0_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN0_PCTL, SPC5_CAN_FLEXCAN0_START_PCTL);
#define SPC5_FLEXCAN0_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN0_PCTL, SPC5_CAN_FLEXCAN0_STOP_PCTL);

#define SPC5_HAS_FLEXCAN1                                   TRUE
#define SPC5_FLEXCAN1_PCTL                                  17
#define SPC5_FLEXCAN1_MB                                    64
#define SPC5_FLEXCAN1_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN1_FLEXCAN_ESR_ERR_INT_HANDLER           vector85
#define SPC5_FLEXCAN1_FLEXCAN_ESR_BOFF_HANDLER              vector86
#define SPC5_FLEXCAN1_FLEXCAN_BUF_00_03_HANDLER             vector88
#define SPC5_FLEXCAN1_FLEXCAN_BUF_04_07_HANDLER             vector89
#define SPC5_FLEXCAN1_FLEXCAN_BUF_08_11_HANDLER             vector90
#define SPC5_FLEXCAN1_FLEXCAN_BUF_12_15_HANDLER             vector91
#define SPC5_FLEXCAN1_FLEXCAN_BUF_16_31_HANDLER             vector92
#define SPC5_FLEXCAN1_FLEXCAN_BUF_32_63_HANDLER             vector93
#define SPC5_FLEXCAN1_FLEXCAN_ESR_ERR_INT_NUMBER            85
#define SPC5_FLEXCAN1_FLEXCAN_ESR_BOFF_NUMBER               86
#define SPC5_FLEXCAN1_FLEXCAN_BUF_00_03_NUMBER              88
#define SPC5_FLEXCAN1_FLEXCAN_BUF_04_07_NUMBER              89
#define SPC5_FLEXCAN1_FLEXCAN_BUF_08_11_NUMBER              90
#define SPC5_FLEXCAN1_FLEXCAN_BUF_12_15_NUMBER              91
#define SPC5_FLEXCAN1_FLEXCAN_BUF_16_31_NUMBER              92
#define SPC5_FLEXCAN1_FLEXCAN_BUF_32_63_NUMBER              93
#define SPC5_FLEXCAN1_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN1_PCTL, SPC5_CAN_FLEXCAN1_START_PCTL);
#define SPC5_FLEXCAN1_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN1_PCTL, SPC5_CAN_FLEXCAN1_STOP_PCTL);

#define SPC5_HAS_FLEXCAN2                                   TRUE
#define SPC5_FLEXCAN2_PCTL                                  18
#define SPC5_FLEXCAN2_MB                                    64
#define SPC5_FLEXCAN2_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN2_FLEXCAN_ESR_ERR_INT_HANDLER           vector105
#define SPC5_FLEXCAN2_FLEXCAN_ESR_BOFF_HANDLER              vector106
#define SPC5_FLEXCAN2_FLEXCAN_BUF_00_03_HANDLER             vector108
#define SPC5_FLEXCAN2_FLEXCAN_BUF_04_07_HANDLER             vector109
#define SPC5_FLEXCAN2_FLEXCAN_BUF_08_11_HANDLER             vector110
#define SPC5_FLEXCAN2_FLEXCAN_BUF_12_15_HANDLER             vector111
#define SPC5_FLEXCAN2_FLEXCAN_BUF_16_31_HANDLER             vector112
#define SPC5_FLEXCAN2_FLEXCAN_BUF_32_63_HANDLER             vector113
#define SPC5_FLEXCAN2_FLEXCAN_ESR_ERR_INT_NUMBER            105
#define SPC5_FLEXCAN2_FLEXCAN_ESR_BOFF_NUMBER               106
#define SPC5_FLEXCAN2_FLEXCAN_BUF_00_03_NUMBER              108
#define SPC5_FLEXCAN2_FLEXCAN_BUF_04_07_NUMBER              109
#define SPC5_FLEXCAN2_FLEXCAN_BUF_08_11_NUMBER              110
#define SPC5_FLEXCAN2_FLEXCAN_BUF_12_15_NUMBER              111
#define SPC5_FLEXCAN2_FLEXCAN_BUF_16_31_NUMBER              112
#define SPC5_FLEXCAN2_FLEXCAN_BUF_32_63_NUMBER              113
#define SPC5_FLEXCAN2_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN2_PCTL, SPC5_CAN_FLEXCAN2_START_PCTL);
#define SPC5_FLEXCAN2_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN2_PCTL, SPC5_CAN_FLEXCAN2_START_PCTL);

#define SPC5_HAS_FLEXCAN3                                   TRUE
#define SPC5_FLEXCAN3_PCTL                                  19
#define SPC5_FLEXCAN3_MB                                    64
#define SPC5_FLEXCAN3_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN3_FLEXCAN_ESR_ERR_INT_HANDLER           vector173
#define SPC5_FLEXCAN3_FLEXCAN_ESR_BOFF_HANDLER              vector174
#define SPC5_FLEXCAN3_FLEXCAN_BUF_00_03_HANDLER             vector176
#define SPC5_FLEXCAN3_FLEXCAN_BUF_04_07_HANDLER             vector177
#define SPC5_FLEXCAN3_FLEXCAN_BUF_08_11_HANDLER             vector178
#define SPC5_FLEXCAN3_FLEXCAN_BUF_12_15_HANDLER             vector179
#define SPC5_FLEXCAN3_FLEXCAN_BUF_16_31_HANDLER             vector180
#define SPC5_FLEXCAN3_FLEXCAN_BUF_32_63_HANDLER             vector181
#define SPC5_FLEXCAN3_FLEXCAN_ESR_ERR_INT_NUMBER            173
#define SPC5_FLEXCAN3_FLEXCAN_ESR_BOFF_NUMBER               174
#define SPC5_FLEXCAN3_FLEXCAN_BUF_00_03_NUMBER              176
#define SPC5_FLEXCAN3_FLEXCAN_BUF_04_07_NUMBER              177
#define SPC5_FLEXCAN3_FLEXCAN_BUF_08_11_NUMBER              178
#define SPC5_FLEXCAN3_FLEXCAN_BUF_12_15_NUMBER              179
#define SPC5_FLEXCAN3_FLEXCAN_BUF_16_31_NUMBER              180
#define SPC5_FLEXCAN3_FLEXCAN_BUF_32_63_NUMBER              181
#define SPC5_FLEXCAN3_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN3_PCTL, SPC5_CAN_FLEXCAN3_START_PCTL);
#define SPC5_FLEXCAN3_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN3_PCTL, SPC5_CAN_FLEXCAN3_STOP_PCTL);

#define SPC5_HAS_FLEXCAN4                                   TRUE
#define SPC5_FLEXCAN4_PCTL                                  20
#define SPC5_FLEXCAN4_MB                                    64
#define SPC5_FLEXCAN4_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN4_FLEXCAN_ESR_ERR_INT_HANDLER           vector190
#define SPC5_FLEXCAN4_FLEXCAN_ESR_BOFF_HANDLER              vector191
#define SPC5_FLEXCAN4_FLEXCAN_BUF_00_03_HANDLER             vector193
#define SPC5_FLEXCAN4_FLEXCAN_BUF_04_07_HANDLER             vector194
#define SPC5_FLEXCAN4_FLEXCAN_BUF_08_11_HANDLER             vector195
#define SPC5_FLEXCAN4_FLEXCAN_BUF_12_15_HANDLER             vector196
#define SPC5_FLEXCAN4_FLEXCAN_BUF_16_31_HANDLER             vector197
#define SPC5_FLEXCAN4_FLEXCAN_BUF_32_63_HANDLER             vector198
#define SPC5_FLEXCAN4_FLEXCAN_ESR_ERR_INT_NUMBER            190
#define SPC5_FLEXCAN4_FLEXCAN_ESR_BOFF_NUMBER               191
#define SPC5_FLEXCAN4_FLEXCAN_BUF_00_03_NUMBER              193
#define SPC5_FLEXCAN4_FLEXCAN_BUF_04_07_NUMBER              194
#define SPC5_FLEXCAN4_FLEXCAN_BUF_08_11_NUMBER              195
#define SPC5_FLEXCAN4_FLEXCAN_BUF_12_15_NUMBER              196
#define SPC5_FLEXCAN4_FLEXCAN_BUF_16_31_NUMBER              197
#define SPC5_FLEXCAN4_FLEXCAN_BUF_32_63_NUMBER              198
#define SPC5_FLEXCAN4_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN4_PCTL, SPC5_CAN_FLEXCAN4_START_PCTL);
#define SPC5_FLEXCAN4_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN4_PCTL, SPC5_CAN_FLEXCAN4_STOP_PCTL);

#define SPC5_HAS_FLEXCAN5                                   TRUE
#define SPC5_FLEXCAN5_PCTL                                  21
#define SPC5_FLEXCAN5_MB                                    64
#define SPC5_FLEXCAN5_SHARED_IRQ                            TRUE
#define SPC5_FLEXCAN5_FLEXCAN_ESR_ERR_INT_HANDLER           vector202
#define SPC5_FLEXCAN5_FLEXCAN_ESR_BOFF_HANDLER              vector203
#define SPC5_FLEXCAN5_FLEXCAN_BUF_00_03_HANDLER             vector205
#define SPC5_FLEXCAN5_FLEXCAN_BUF_04_07_HANDLER             vector206
#define SPC5_FLEXCAN5_FLEXCAN_BUF_08_11_HANDLER             vector207
#define SPC5_FLEXCAN5_FLEXCAN_BUF_12_15_HANDLER             vector208
#define SPC5_FLEXCAN5_FLEXCAN_BUF_16_31_HANDLER             vector209
#define SPC5_FLEXCAN5_FLEXCAN_BUF_32_63_HANDLER             vector210
#define SPC5_FLEXCAN5_FLEXCAN_ESR_ERR_INT_NUMBER            202
#define SPC5_FLEXCAN5_FLEXCAN_ESR_BOFF_NUMBER               203
#define SPC5_FLEXCAN5_FLEXCAN_BUF_00_03_NUMBER              205
#define SPC5_FLEXCAN5_FLEXCAN_BUF_04_07_NUMBER              206
#define SPC5_FLEXCAN5_FLEXCAN_BUF_08_11_NUMBER              207
#define SPC5_FLEXCAN5_FLEXCAN_BUF_12_15_NUMBER              208
#define SPC5_FLEXCAN5_FLEXCAN_BUF_16_31_NUMBER              209
#define SPC5_FLEXCAN5_FLEXCAN_BUF_32_63_NUMBER              210
#define SPC5_FLEXCAN5_ENABLE_CLOCK()                        halSPCSetPeripheralClockMode(SPC5_FLEXCAN5_PCTL, SPC5_CAN_FLEXCAN5_START_PCTL);
#define SPC5_FLEXCAN5_DISABLE_CLOCK()                       halSPCSetPeripheralClockMode(SPC5_FLEXCAN5_PCTL, SPC5_CAN_FLEXCAN5_STOP_PCTL);

/* ADC attributes.*/
#define SPC5_ADC_HAS_TRC                    FALSE

#define SPC5_HAS_ADC0                       TRUE
#define SPC5_ADC_ADC0_HAS_CTR0              TRUE
#define SPC5_ADC_ADC0_HAS_CTR1              TRUE
#define SPC5_ADC_ADC0_HAS_CTR2              TRUE
#define SPC5_ADC_ADC0_HAS_NCMR0             TRUE
#define SPC5_ADC_ADC0_HAS_NCMR1             TRUE
#define SPC5_ADC_ADC0_HAS_NCMR2             TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR0           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR1           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR2           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR3           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR4           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR5           TRUE
#define SPC5_ADC_ADC0_HAS_THRHLR6           FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR7           FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR8           FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR9           FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR10          FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR11          FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR12          FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR13          FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR14          FALSE
#define SPC5_ADC_ADC0_HAS_THRHLR15          FALSE
#define SPC5_ADC_ADC0_HAS_CWENR0            TRUE
#define SPC5_ADC_ADC0_HAS_CWENR1            TRUE
#define SPC5_ADC_ADC0_HAS_CWENR2            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL0            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL1            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL2            FALSE
#define SPC5_ADC_ADC0_HAS_CWSEL3            FALSE
#define SPC5_ADC_ADC0_HAS_CWSEL4            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL5            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL6            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL7            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL8            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL9            TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL10           TRUE
#define SPC5_ADC_ADC0_HAS_CWSEL11           TRUE
#define SPC5_ADC_ADC0_HAS_CIMR0             TRUE
#define SPC5_ADC_ADC0_HAS_CIMR1             TRUE
#define SPC5_ADC_ADC0_HAS_CIMR2             TRUE
#define SPC5_ADC_ADC0_HAS_CEOCFR0           TRUE
#define SPC5_ADC_ADC0_HAS_CEOCFR1           TRUE
#define SPC5_ADC_ADC0_HAS_CEOCFR2           TRUE
#define SPC5_ADC0_PCTL                      32
#define SPC5_ADC0_DMA_DEV_ID                29
#define SPC5_ADC0_EOC_HANDLER               vector62
#define SPC5_ADC0_EOC_NUMBER                62
#define SPC5_ADC0_WD_HANDLER                vector64
#define SPC5_ADC0_WD_NUMBER                 64

#define SPC5_HAS_ADC1                       TRUE
#define SPC5_ADC_ADC1_HAS_CTR0              TRUE
#define SPC5_ADC_ADC1_HAS_CTR1              TRUE
#define SPC5_ADC_ADC1_HAS_CTR2              FALSE
#define SPC5_ADC_ADC1_HAS_NCMR0             TRUE
#define SPC5_ADC_ADC1_HAS_NCMR1             TRUE
#define SPC5_ADC_ADC1_HAS_NCMR2             FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR0           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR1           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR2           TRUE
#define SPC5_ADC_ADC1_HAS_THRHLR3           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR4           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR5           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR6           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR7           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR8           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR9           FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR10          FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR11          FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR12          FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR13          FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR14          FALSE
#define SPC5_ADC_ADC1_HAS_THRHLR15          FALSE
#define SPC5_ADC_ADC1_HAS_CWENR0            TRUE
#define SPC5_ADC_ADC1_HAS_CWENR1            TRUE
#define SPC5_ADC_ADC1_HAS_CWENR2            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL0            TRUE
#define SPC5_ADC_ADC1_HAS_CWSEL1            TRUE
#define SPC5_ADC_ADC1_HAS_CWSEL2            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL3            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL4            TRUE
#define SPC5_ADC_ADC1_HAS_CWSEL5            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL6            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL7            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL8            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL9            FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL10           FALSE
#define SPC5_ADC_ADC1_HAS_CWSEL11           FALSE
#define SPC5_ADC_ADC1_HAS_CIMR0             TRUE
#define SPC5_ADC_ADC1_HAS_CIMR1             TRUE
#define SPC5_ADC_ADC1_HAS_CIMR2             TRUE
#define SPC5_ADC_ADC1_HAS_CEOCFR0           TRUE
#define SPC5_ADC_ADC1_HAS_CEOCFR1           TRUE
#define SPC5_ADC_ADC1_HAS_CEOCFR2           TRUE
#define SPC5_ADC1_PCTL                      33
#define SPC5_ADC1_DMA_DEV_ID                30
#define SPC5_ADC1_EOC_HANDLER               vector82
#define SPC5_ADC1_EOC_NUMBER                82
#define SPC5_ADC1_WD_HANDLER                vector84
#define SPC5_ADC1_WD_NUMBER                 84
/** @} */

#endif /* SPC5_REGISTRY_H */

/** @} */
