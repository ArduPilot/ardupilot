[#ftl]
[@pp.dropOutputFile /]
[@pp.changeOutputFile name="mcuconf.h" /]
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

#ifndef _MCUCONF_H_
#define _MCUCONF_H_

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
#define SPC5_NO_INIT                        ${conf.instance.initialization_settings.do_not_init.value[0]?upper_case}
#define SPC5_CLK_BYPASS                     ${conf.instance.initialization_settings.clock_bypass.value[0]?upper_case}
#define SPC5_ALLOW_OVERCLOCK                ${conf.instance.initialization_settings.allow_overclocking.value[0]?upper_case}
#define SPC5_CLK_PREDIV_VALUE               ${conf.instance.initialization_settings.fmpll0_settings.prediv_value.value[0]}
#define SPC5_CLK_MFD_VALUE                  ${conf.instance.initialization_settings.fmpll0_settings.mfd_value.value[0]}
#define SPC5_CLK_RFD                        ${conf.instance.initialization_settings.fmpll0_settings.rfd_value.value[0]}
#define SPC5_FLASH_BIUCR                    (BIUCR_BANK1_TOO |              \
                                             BIUCR_MASTER4_PREFETCH |       \
                                             BIUCR_MASTER0_PREFETCH |       \
                                             BIUCR_DPFEN |                  \
                                             BIUCR_IPFEN |                  \
                                             BIUCR_PFLIM_ON_MISS |          \
                                             BIUCR_BFEN)
#define SPC5_EMIOS_GPRE_VALUE               ${conf.instance.initialization_settings.clocks.emios_global_prescaler.value[0]}

/*
 * EDMA driver settings.
 */
#define SPC5_EDMA_CR_SETTING                (EDMA_CR_GRP1PRI(1) |           \
                                             EDMA_CR_GRP0PRI(0) |           \
                                             EDMA_CR_EMLM       |           \
                                             EDMA_CR_ERGA)
#define SPC5_EDMA_GROUP0_PRIORITIES         [#rt/]
[#list conf.instance.edma_settings.group_0_channels_priorities.* as channel]
  [#if channel_has_next]
${channel.value[0]}, [#rt/]
  [#else]
${channel.value[0]}
  [/#if]
[/#list]
#define SPC5_EDMA_GROUP1_PRIORITIES         [#rt/]
[#list conf.instance.edma_settings.group_1_channels_priorities.* as channel]
  [#if channel_has_next]
${channel.value[0]}, [#rt/]
  [#else]
${channel.value[0]}
  [/#if]
[/#list]
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#define SPC5_EDMA_ERROR_HANDLER()           osalSysHalt("DMA failure")

/*
 * ADC driver settings.
 */
#define SPC5_ADC_USE_ADC0_Q0                ${conf.instance.eqadc_settings.fifo0.value[0]?upper_case}
#define SPC5_ADC_USE_ADC0_Q1                ${conf.instance.eqadc_settings.fifo1.value[0]?upper_case}
#define SPC5_ADC_USE_ADC0_Q2                ${conf.instance.eqadc_settings.fifo2.value[0]?upper_case}
#define SPC5_ADC_USE_ADC1_Q3                ${conf.instance.eqadc_settings.fifo3.value[0]?upper_case}
#define SPC5_ADC_USE_ADC1_Q4                ${conf.instance.eqadc_settings.fifo4.value[0]?upper_case}
#define SPC5_ADC_USE_ADC1_Q5                ${conf.instance.eqadc_settings.fifo5.value[0]?upper_case}
#define SPC5_ADC_FIFO0_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo0.value[0]}
#define SPC5_ADC_FIFO1_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo1.value[0]}
#define SPC5_ADC_FIFO2_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo2.value[0]}
#define SPC5_ADC_FIFO3_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo3.value[0]}
#define SPC5_ADC_FIFO4_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo4.value[0]}
#define SPC5_ADC_FIFO5_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.eqadc_fifo5.value[0]}
#define SPC5_ADC_CR_CLK_PS                  ADC_CR_CLK_PS(${conf.instance.eqadc_settings.divider.value[0]})
[#assign AN0 = conf.instance.eqadc_settings.an0.value[0]?upper_case?replace(" ", "_") /]
[#assign AN1 = conf.instance.eqadc_settings.an1.value[0]?upper_case?replace(" ", "_") /]
[#assign AN2 = conf.instance.eqadc_settings.an2.value[0]?upper_case?replace(" ", "_") /]
[#assign AN3 = conf.instance.eqadc_settings.an3.value[0]?upper_case?replace(" ", "_") /]
[#assign AN4 = conf.instance.eqadc_settings.an4.value[0]?upper_case?replace(" ", "_") /]
[#assign AN5 = conf.instance.eqadc_settings.an5.value[0]?upper_case?replace(" ", "_") /]
[#assign AN6 = conf.instance.eqadc_settings.an6.value[0]?upper_case?replace(" ", "_") /]
[#assign AN7 = conf.instance.eqadc_settings.an7.value[0]?upper_case?replace(" ", "_") /]
#define SPC5_ADC_PUDCR                      {ADC_PUDCR_${AN0},ADC_PUDCR_${AN1},ADC_PUDCR_${AN2},ADC_PUDCR_${AN3},ADC_PUDCR_${AN4},ADC_PUDCR_${AN5},ADC_PUDCR_${AN6},ADC_PUDCR_${AN7}}

/*
 * SERIAL driver system settings.
 */
#define SPC5_USE_ESCIA                      ${(conf.instance.esci_settings.esci0.value[0] == "Serial")?string?upper_case}
#define SPC5_USE_ESCIB                      ${(conf.instance.esci_settings.esci1.value[0] == "Serial")?string?upper_case}
#define SPC5_ESCIA_PRIORITY                 ${conf.instance.irq_priority_settings.esci0.value[0]}
#define SPC5_ESCIB_PRIORITY                 ${conf.instance.irq_priority_settings.esci1.value[0]}

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI1                  ${conf.instance.dspi_settings.dspi_b.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI2                  ${conf.instance.dspi_settings.dspi_c.value[0]?upper_case}
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_${conf.instance.dspi_settings.dma_mode.value[0]?upper_case?replace(" ", "_")}
[#assign bs0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs0[0].@index[0]?trim?number] /]
[#assign bs1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs1[0].@index[0]?trim?number] /]
[#assign bs2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs2[0].@index[0]?trim?number] /]
[#assign bs3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs3[0].@index[0]?trim?number] /]
[#assign bs4 = [""," | SPC5_MCR_PCSIS4"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs4[0].@index[0]?trim?number] /]
[#assign bs5 = [""," | SPC5_MCR_PCSIS5"][conf.instance.dspi_settings.inactive_states.dspi_b___pcs5[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI1_MCR                  (0${bs0 + bs1 + bs2 + bs3 + bs4 + bs5})
[#assign cs0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs0[0].@index[0]?trim?number] /]
[#assign cs1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs1[0].@index[0]?trim?number] /]
[#assign cs2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs2[0].@index[0]?trim?number] /]
[#assign cs3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs3[0].@index[0]?trim?number] /]
[#assign cs4 = [""," | SPC5_MCR_PCSIS4"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs4[0].@index[0]?trim?number] /]
[#assign cs5 = [""," | SPC5_MCR_PCSIS5"][conf.instance.dspi_settings.inactive_states.dspi_c___pcs5[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI2_MCR                  (0${cs0 + cs1 + cs2 + cs3 + cs4 + cs5})
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_b.value[0]}
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_c.value[0]}
#define SPC5_SPI_DSPI1_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_b.value[0]}
#define SPC5_SPI_DSPI2_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_c.value[0]}
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       ${conf.instance.dspi_settings.dma_error_hook.value[0]}

/*
 * ICU driver system settings.
 */
#define SPC5_ICU_USE_EMIOS_CH0              ${(conf.instance.emios200_settings.emios_uc0.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH1              ${(conf.instance.emios200_settings.emios_uc1.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH2              ${(conf.instance.emios200_settings.emios_uc2.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH3              ${(conf.instance.emios200_settings.emios_uc3.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH4              ${(conf.instance.emios200_settings.emios_uc4.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH5              ${(conf.instance.emios200_settings.emios_uc5.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH6              ${(conf.instance.emios200_settings.emios_uc6.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH8              ${(conf.instance.emios200_settings.emios_uc8.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH9              ${(conf.instance.emios200_settings.emios_uc9.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH10             ${(conf.instance.emios200_settings.emios_uc10.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH11             ${(conf.instance.emios200_settings.emios_uc11.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH12             ${(conf.instance.emios200_settings.emios_uc12.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH13             ${(conf.instance.emios200_settings.emios_uc13.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH14             ${(conf.instance.emios200_settings.emios_uc14.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH15             ${(conf.instance.emios200_settings.emios_uc15.value[0] == "ICU")?string?upper_case}
#define SPC5_ICU_USE_EMIOS_CH23             ${(conf.instance.emios200_settings.emios_uc23.value[0] == "ICU")?string?upper_case}

/*
 * PWM driver system settings.
 */
#define SPC5_PWM_USE_EMIOS_CH0              ${(conf.instance.emios200_settings.emios_uc0.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH8              ${(conf.instance.emios200_settings.emios_uc8.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH9              ${(conf.instance.emios200_settings.emios_uc9.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH10             ${(conf.instance.emios200_settings.emios_uc10.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH12             ${(conf.instance.emios200_settings.emios_uc12.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH14             ${(conf.instance.emios200_settings.emios_uc14.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH15             ${(conf.instance.emios200_settings.emios_uc15.value[0] == "PWM")?string?upper_case}
#define SPC5_PWM_USE_EMIOS_CH23             ${(conf.instance.emios200_settings.emios_uc23.value[0] == "PWM")?string?upper_case}

/*
 * eMIOS channel priorities.
 */
#define SPC5_EMIOS_FLAG_F0_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc0.value[0]}
#define SPC5_EMIOS_FLAG_F1_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc1.value[0]}
#define SPC5_EMIOS_FLAG_F2_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc2.value[0]}
#define SPC5_EMIOS_FLAG_F3_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc3.value[0]}
#define SPC5_EMIOS_FLAG_F4_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc4.value[0]}
#define SPC5_EMIOS_FLAG_F5_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc5.value[0]}
#define SPC5_EMIOS_FLAG_F6_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc6.value[0]}
#define SPC5_EMIOS_FLAG_F8_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc8.value[0]}
#define SPC5_EMIOS_FLAG_F9_PRIORITY         ${conf.instance.irq_priority_settings.emios_uc9.value[0]}
#define SPC5_EMIOS_FLAG_F10_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc10.value[0]}
#define SPC5_EMIOS_FLAG_F11_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc11.value[0]}
#define SPC5_EMIOS_FLAG_F12_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc12.value[0]}
#define SPC5_EMIOS_FLAG_F13_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc13.value[0]}
#define SPC5_EMIOS_FLAG_F14_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc14.value[0]}
#define SPC5_EMIOS_FLAG_F15_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc15.value[0]}
#define SPC5_EMIOS_FLAG_F23_PRIORITY        ${conf.instance.irq_priority_settings.emios_uc23.value[0]}

/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                ${conf.instance.flexcan_settings.flexcan_enable_filters.value[0]?upper_case}

#define SPC5_CAN_USE_FLEXCAN0               ${conf.instance.flexcan_settings.flexcan0.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan0_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_IRQ_PRIORITY      ${conf.instance.irq_priority_settings.flexcan0.value[0]}
#define SPC5_CAN_NUM_RX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_rx_mailboxes.value[0]}
#define SPC5_CAN_NUM_TX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_tx_mailboxes.value[0]}

#define SPC5_CAN_USE_FLEXCAN1               ${conf.instance.flexcan_settings.flexcan1.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan1_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_IRQ_PRIORITY      ${conf.instance.irq_priority_settings.flexcan1.value[0]}

#endif /* _MCUCONF_H_ */
