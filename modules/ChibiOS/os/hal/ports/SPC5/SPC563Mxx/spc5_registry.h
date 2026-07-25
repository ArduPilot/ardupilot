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
 * @file    SPC563Mxx/spc5_registry.h
 * @brief   SPC563Mxx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef SPC5_REGISTRY_H
#define SPC5_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    SPC563Mxx capabilities
 * @{
 */
/* DSPI attribures.*/
#define SPC5_HAS_DSPI0                      FALSE
#define SPC5_HAS_DSPI1                      TRUE
#define SPC5_HAS_DSPI2                      TRUE
#define SPC5_HAS_DSPI3                      FALSE
#define SPC5_HAS_DSPI4                      FALSE
#define SPC5_HAS_DSPI5                      FALSE
#define SPC5_HAS_DSPI6                      FALSE
#define SPC5_HAS_DSPI7                      FALSE
#define SPC5_DSPI_FIFO_DEPTH                4
#define SPC5_DSPI1_TFFF_HANDLER             vector133
#define SPC5_DSPI1_TFFF_NUMBER              133
#define SPC5_DSPI1_RFDF_HANDLER             vector135
#define SPC5_DSPI1_RFDF_NUMBER              135
#define SPC5_DSPI2_TFFF_HANDLER             vector138
#define SPC5_DSPI2_TFFF_NUMBER              138
#define SPC5_DSPI2_RFDF_HANDLER             vector140
#define SPC5_DSPI2_RFDF_NUMBER              140
#define SPC5_DSPI1_ENABLE_CLOCK()
#define SPC5_DSPI1_DISABLE_CLOCK()
#define SPC5_DSPI2_ENABLE_CLOCK()
#define SPC5_DSPI2_DISABLE_CLOCK()

/* eDMA attributes.*/
#define SPC5_HAS_EDMA                       TRUE
#define SPC5_EDMA_NCHANNELS                 32
#define SPC5_EDMA_HAS_MUX                   FALSE
#define SPC5_SPI_DSPI1_TX1_DMA_CH_ID        12
#define SPC5_SPI_DSPI1_TX2_DMA_CH_ID        25
#define SPC5_SPI_DSPI1_RX_DMA_CH_ID         13
#define SPC5_SPI_DSPI2_TX1_DMA_CH_ID        14
#define SPC5_SPI_DSPI2_TX2_DMA_CH_ID        26
#define SPC5_SPI_DSPI2_RX_DMA_CH_ID         15

/* eQADC attributes.*/
#define SPC5_HAS_EQADC                      TRUE

/* eSCI attributes.*/
#define SPC5_HAS_ESCIA                      TRUE
#define SPC5_ESCIA_HANDLER                  vector146
#define SPC5_ESCIA_NUMBER                   146

#define SPC5_HAS_ESCIB                      TRUE
#define SPC5_ESCIB_HANDLER                  vector149
#define SPC5_ESCIB_NUMBER                   149

#define SPC5_HAS_ESCIC                      FALSE

/* SIU attributes.*/
#define SPC5_HAS_SIU                        TRUE
#define SPC5_SIU_SUPPORTS_PORTS             FALSE

/* EMIOS attributes.*/
#define SPC5_HAS_EMIOS                      TRUE

#define SPC5_EMIOS_NUM_CHANNELS             16

#define SPC5_EMIOS_FLAG_F0_HANDLER          vector51
#define SPC5_EMIOS_FLAG_F1_HANDLER          vector52
#define SPC5_EMIOS_FLAG_F2_HANDLER          vector53
#define SPC5_EMIOS_FLAG_F3_HANDLER          vector54
#define SPC5_EMIOS_FLAG_F4_HANDLER          vector55
#define SPC5_EMIOS_FLAG_F5_HANDLER          vector56
#define SPC5_EMIOS_FLAG_F6_HANDLER          vector57
#define SPC5_EMIOS_FLAG_F8_HANDLER          vector59
#define SPC5_EMIOS_FLAG_F9_HANDLER          vector60
#define SPC5_EMIOS_FLAG_F10_HANDLER         vector61
#define SPC5_EMIOS_FLAG_F11_HANDLER         vector62
#define SPC5_EMIOS_FLAG_F12_HANDLER         vector63
#define SPC5_EMIOS_FLAG_F13_HANDLER         vector64
#define SPC5_EMIOS_FLAG_F14_HANDLER         vector65
#define SPC5_EMIOS_FLAG_F15_HANDLER         vector66
#define SPC5_EMIOS_FLAG_F23_HANDLER         vector209
#define SPC5_EMIOS_FLAG_F0_NUMBER           51
#define SPC5_EMIOS_FLAG_F1_NUMBER           52
#define SPC5_EMIOS_FLAG_F2_NUMBER           53
#define SPC5_EMIOS_FLAG_F3_NUMBER           54
#define SPC5_EMIOS_FLAG_F4_NUMBER           55
#define SPC5_EMIOS_FLAG_F5_NUMBER           56
#define SPC5_EMIOS_FLAG_F6_NUMBER           57
#define SPC5_EMIOS_FLAG_F8_NUMBER           59
#define SPC5_EMIOS_FLAG_F9_NUMBER           60
#define SPC5_EMIOS_FLAG_F10_NUMBER          61
#define SPC5_EMIOS_FLAG_F11_NUMBER          62
#define SPC5_EMIOS_FLAG_F12_NUMBER          63
#define SPC5_EMIOS_FLAG_F13_NUMBER          64
#define SPC5_EMIOS_FLAG_F14_NUMBER          65
#define SPC5_EMIOS_FLAG_F15_NUMBER          66
#define SPC5_EMIOS_FLAG_F23_NUMBER          209

#define SPC5_EMIOS_CLK                      (SPC5_SYSCLK /                  \
                                             SPC5_EMIOS_GPRE_VALUE)
#define SPC5_EMIOS_ENABLE_CLOCK()
#define SPC5_EMIOS_DISABLE_CLOCK()

/* FlexCAN attributes.*/
#define SPC5_HAS_FLEXCAN0                                   TRUE
#define SPC5_FLEXCAN0_MB                                    64
#define SPC5_FLEXCAN0_SHARED_IRQ                            FALSE
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_HANDLER              vector152
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_HANDLER           vector153
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_HANDLER                vector155
#define SPC5_FLEXCAN0_FLEXCAN_BUF_01_HANDLER                vector156
#define SPC5_FLEXCAN0_FLEXCAN_BUF_02_HANDLER                vector157
#define SPC5_FLEXCAN0_FLEXCAN_BUF_03_HANDLER                vector158
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_HANDLER                vector159
#define SPC5_FLEXCAN0_FLEXCAN_BUF_05_HANDLER                vector160
#define SPC5_FLEXCAN0_FLEXCAN_BUF_06_HANDLER                vector161
#define SPC5_FLEXCAN0_FLEXCAN_BUF_07_HANDLER                vector162
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_HANDLER                vector163
#define SPC5_FLEXCAN0_FLEXCAN_BUF_09_HANDLER                vector164
#define SPC5_FLEXCAN0_FLEXCAN_BUF_10_HANDLER                vector165
#define SPC5_FLEXCAN0_FLEXCAN_BUF_11_HANDLER                vector166
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_HANDLER                vector167
#define SPC5_FLEXCAN0_FLEXCAN_BUF_13_HANDLER                vector168
#define SPC5_FLEXCAN0_FLEXCAN_BUF_14_HANDLER                vector169
#define SPC5_FLEXCAN0_FLEXCAN_BUF_15_HANDLER                vector170
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_HANDLER             vector171
#define SPC5_FLEXCAN0_FLEXCAN_BUF_32_63_HANDLER             vector172
#define SPC5_FLEXCAN0_FLEXCAN_ESR_BOFF_NUMBER               152
#define SPC5_FLEXCAN0_FLEXCAN_ESR_ERR_INT_NUMBER            153
#define SPC5_FLEXCAN0_FLEXCAN_BUF_00_NUMBER                 155
#define SPC5_FLEXCAN0_FLEXCAN_BUF_01_NUMBER                 156
#define SPC5_FLEXCAN0_FLEXCAN_BUF_02_NUMBER                 157
#define SPC5_FLEXCAN0_FLEXCAN_BUF_03_NUMBER                 158
#define SPC5_FLEXCAN0_FLEXCAN_BUF_04_NUMBER                 159
#define SPC5_FLEXCAN0_FLEXCAN_BUF_05_NUMBER                 160
#define SPC5_FLEXCAN0_FLEXCAN_BUF_06_NUMBER                 161
#define SPC5_FLEXCAN0_FLEXCAN_BUF_07_NUMBER                 162
#define SPC5_FLEXCAN0_FLEXCAN_BUF_08_NUMBER                 163
#define SPC5_FLEXCAN0_FLEXCAN_BUF_09_NUMBER                 164
#define SPC5_FLEXCAN0_FLEXCAN_BUF_10_NUMBER                 165
#define SPC5_FLEXCAN0_FLEXCAN_BUF_11_NUMBER                 166
#define SPC5_FLEXCAN0_FLEXCAN_BUF_12_NUMBER                 167
#define SPC5_FLEXCAN0_FLEXCAN_BUF_13_NUMBER                 168
#define SPC5_FLEXCAN0_FLEXCAN_BUF_14_NUMBER                 169
#define SPC5_FLEXCAN0_FLEXCAN_BUF_15_NUMBER                 170
#define SPC5_FLEXCAN0_FLEXCAN_BUF_16_31_NUMBER              171
#define SPC5_FLEXCAN0_FLEXCAN_BUF_32_63_NUMBER              172
#define SPC5_FLEXCAN0_ENABLE_CLOCK()
#define SPC5_FLEXCAN0_DISABLE_CLOCK()

#define SPC5_HAS_FLEXCAN1                                   TRUE
#define SPC5_FLEXCAN1_MB                                    32
#define SPC5_FLEXCAN1_SHARED_IRQ                            FALSE
#define SPC5_FLEXCAN1_FLEXCAN_ESR_BOFF_HANDLER              vector173
#define SPC5_FLEXCAN1_FLEXCAN_ESR_ERR_INT_HANDLER           vector174
#define SPC5_FLEXCAN1_FLEXCAN_BUF_00_HANDLER                vector176
#define SPC5_FLEXCAN1_FLEXCAN_BUF_01_HANDLER                vector177
#define SPC5_FLEXCAN1_FLEXCAN_BUF_02_HANDLER                vector178
#define SPC5_FLEXCAN1_FLEXCAN_BUF_03_HANDLER                vector179
#define SPC5_FLEXCAN1_FLEXCAN_BUF_04_HANDLER                vector180
#define SPC5_FLEXCAN1_FLEXCAN_BUF_05_HANDLER                vector181
#define SPC5_FLEXCAN1_FLEXCAN_BUF_06_HANDLER                vector182
#define SPC5_FLEXCAN1_FLEXCAN_BUF_07_HANDLER                vector183
#define SPC5_FLEXCAN1_FLEXCAN_BUF_08_HANDLER                vector184
#define SPC5_FLEXCAN1_FLEXCAN_BUF_09_HANDLER                vector185
#define SPC5_FLEXCAN1_FLEXCAN_BUF_10_HANDLER                vector186
#define SPC5_FLEXCAN1_FLEXCAN_BUF_11_HANDLER                vector187
#define SPC5_FLEXCAN1_FLEXCAN_BUF_12_HANDLER                vector188
#define SPC5_FLEXCAN1_FLEXCAN_BUF_13_HANDLER                vector189
#define SPC5_FLEXCAN1_FLEXCAN_BUF_14_HANDLER                vector190
#define SPC5_FLEXCAN1_FLEXCAN_BUF_15_HANDLER                vector191
#define SPC5_FLEXCAN1_FLEXCAN_BUF_16_31_HANDLER             vector192
#define SPC5_FLEXCAN1_FLEXCAN_ESR_BOFF_NUMBER               173
#define SPC5_FLEXCAN1_FLEXCAN_ESR_ERR_INT_NUMBER            174
#define SPC5_FLEXCAN1_FLEXCAN_BUF_00_NUMBER                 176
#define SPC5_FLEXCAN1_FLEXCAN_BUF_01_NUMBER                 177
#define SPC5_FLEXCAN1_FLEXCAN_BUF_02_NUMBER                 178
#define SPC5_FLEXCAN1_FLEXCAN_BUF_03_NUMBER                 179
#define SPC5_FLEXCAN1_FLEXCAN_BUF_04_NUMBER                 180
#define SPC5_FLEXCAN1_FLEXCAN_BUF_05_NUMBER                 181
#define SPC5_FLEXCAN1_FLEXCAN_BUF_06_NUMBER                 182
#define SPC5_FLEXCAN1_FLEXCAN_BUF_07_NUMBER                 183
#define SPC5_FLEXCAN1_FLEXCAN_BUF_08_NUMBER                 184
#define SPC5_FLEXCAN1_FLEXCAN_BUF_09_NUMBER                 185
#define SPC5_FLEXCAN1_FLEXCAN_BUF_10_NUMBER                 186
#define SPC5_FLEXCAN1_FLEXCAN_BUF_11_NUMBER                 187
#define SPC5_FLEXCAN1_FLEXCAN_BUF_12_NUMBER                 188
#define SPC5_FLEXCAN1_FLEXCAN_BUF_13_NUMBER                 189
#define SPC5_FLEXCAN1_FLEXCAN_BUF_14_NUMBER                 190
#define SPC5_FLEXCAN1_FLEXCAN_BUF_15_NUMBER                 191
#define SPC5_FLEXCAN1_FLEXCAN_BUF_16_31_NUMBER              192
#define SPC5_FLEXCAN1_ENABLE_CLOCK()
#define SPC5_FLEXCAN1_DISABLE_CLOCK()
/** @} */

#endif /* SPC5_REGISTRY_H */

/** @} */
