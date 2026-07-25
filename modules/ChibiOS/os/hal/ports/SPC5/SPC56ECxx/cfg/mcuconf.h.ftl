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
 * SPC56ECxx drivers configuration.
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

#define SPC56ECxx_MCUCONF

/*
 * HAL driver system settings.
 */
#define SPC5_NO_INIT                        ${conf.instance.initialization_settings.do_not_init.value[0]?upper_case}
#define SPC5_ALLOW_OVERCLOCK                ${conf.instance.initialization_settings.allow_overclocking.value[0]?upper_case}
#define SPC5_DISABLE_WATCHDOG               ${conf.instance.initialization_settings.disable_watchdog.value[0]?upper_case}
#define SPC5_FMPLL0_IDF_VALUE               ${conf.instance.initialization_settings.fmpll0_settings.idf_value.value[0]}
#define SPC5_FMPLL0_NDIV_VALUE              ${conf.instance.initialization_settings.fmpll0_settings.ndiv_value.value[0]}
#define SPC5_FMPLL0_ODF                     ${conf.instance.initialization_settings.fmpll0_settings.odf_value.value[0]}
#define SPC5_XOSCDIV_VALUE                  ${conf.instance.initialization_settings.clocks.fxosc_divider.value[0]}
#define SPC5_IRCDIV_VALUE                   ${conf.instance.initialization_settings.clocks.firc_divider.value[0]}
#define SPC5_PERIPHERAL1_CLK_DIV_VALUE      ${conf.instance.initialization_settings.clocks.peripheral_set_1_clock_divider.value[0]}
#define SPC5_PERIPHERAL2_CLK_DIV_VALUE      ${conf.instance.initialization_settings.clocks.peripheral_set_2_clock_divider.value[0]}
#define SPC5_PERIPHERAL3_CLK_DIV_VALUE      ${conf.instance.initialization_settings.clocks.peripheral_set_3_clock_divider.value[0]}
#define SPC5_Z0_CLK_DIV_VALUE               ${conf.instance.initialization_settings.clocks.z0_core_clock_divider.value[0]}
#define SPC5_FEC_CLK_DIV_VALUE              ${conf.instance.initialization_settings.clocks.fec_clock_divider.value[0]}
#define SPC5_FLASH_CLK_DIV_VALUE            ${conf.instance.initialization_settings.clocks.flash_controller_clock_divider.value[0]}
#define SPC5_CLOCK_FAILURE_HOOK()           ${conf.instance.initialization_settings.clocks.clock_failure_hook.value[0]}

#define SPC5_EMIOS0_GPRE_VALUE              ${conf.instance.initialization_settings.clocks.emios0_global_prescaler.value[0]?number}
#define SPC5_EMIOS1_GPRE_VALUE              ${conf.instance.initialization_settings.clocks.emios1_global_prescaler.value[0]?number}

/*
*  for the unexpected Reset on "Load from RAM" Issue,
*  This switch has to be activated.
*/
#define SPC56ECXX_FMPLL_CLOCK_ERRATA_WORKAROUND FALSE

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
 * SERIAL driver system settings.
 */
#define SPC5_SERIAL_USE_LINFLEX0            ${(conf.instance.linflex_settings.linflex0.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX1            ${(conf.instance.linflex_settings.linflex1.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX2            ${(conf.instance.linflex_settings.linflex2.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX3            ${(conf.instance.linflex_settings.linflex3.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX4            ${(conf.instance.linflex_settings.linflex4.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX5            ${(conf.instance.linflex_settings.linflex5.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX6            ${(conf.instance.linflex_settings.linflex6.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX7            ${(conf.instance.linflex_settings.linflex7.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX8            ${(conf.instance.linflex_settings.linflex8.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX9            ${(conf.instance.linflex_settings.linflex9.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_LINFLEX0_PRIORITY       ${conf.instance.irq_priority_settings.linflex0.value[0]}
#define SPC5_SERIAL_LINFLEX1_PRIORITY       ${conf.instance.irq_priority_settings.linflex1.value[0]}
#define SPC5_SERIAL_LINFLEX2_PRIORITY       ${conf.instance.irq_priority_settings.linflex2.value[0]}
#define SPC5_SERIAL_LINFLEX3_PRIORITY       ${conf.instance.irq_priority_settings.linflex3.value[0]}
#define SPC5_SERIAL_LINFLEX4_PRIORITY       ${conf.instance.irq_priority_settings.linflex4.value[0]}
#define SPC5_SERIAL_LINFLEX5_PRIORITY       ${conf.instance.irq_priority_settings.linflex5.value[0]}
#define SPC5_SERIAL_LINFLEX6_PRIORITY       ${conf.instance.irq_priority_settings.linflex6.value[0]}
#define SPC5_SERIAL_LINFLEX7_PRIORITY       ${conf.instance.irq_priority_settings.linflex7.value[0]}
#define SPC5_SERIAL_LINFLEX8_PRIORITY       ${conf.instance.irq_priority_settings.linflex8.value[0]}
#define SPC5_SERIAL_LINFLEX9_PRIORITY       ${conf.instance.irq_priority_settings.linflex9.value[0]}

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI0                  ${conf.instance.dspi_settings.dspi_0.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI1                  ${conf.instance.dspi_settings.dspi_1.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI2                  ${conf.instance.dspi_settings.dspi_2.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI3                  ${conf.instance.dspi_settings.dspi_3.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI4                  ${conf.instance.dspi_settings.dspi_4.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI5                  ${conf.instance.dspi_settings.dspi_5.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI6                  ${conf.instance.dspi_settings.dspi_6.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI7                  ${conf.instance.dspi_settings.dspi_7.value[0]?upper_case}
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_${conf.instance.dspi_settings.dma_mode.value[0]?upper_case?replace(" ", "_")}
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs3[0].@index[0]?trim?number] /]
[#assign s4 = [""," | SPC5_MCR_PCSIS4"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs4[0].@index[0]?trim?number] /]
[#assign s5 = [""," | SPC5_MCR_PCSIS5"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs5[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI0_MCR                  (0${s0 + s1 + s2 + s3 + s4 + s5})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs3[0].@index[0]?trim?number] /]
[#assign s4 = [""," | SPC5_MCR_PCSIS4"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs4[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI1_MCR                  (0${s0 + s1 + s2 + s3 + s4})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs3[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI2_MCR                  (0${s0 + s1 + s2 + s3})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_3___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_3___pcs1[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI3_MCR                  (0${s0 + s1})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_4___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_4___pcs1[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI4_MCR                  (0${s0 + s1})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_5___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_5___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_5___pcs2[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI5_MCR                  (0${s0 + s1 + s2})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_6___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_6___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_6___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_6___pcs3[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI6_MCR                  (0${s0 + s1 + s2 + s3})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_7___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_7___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_7___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_7___pcs3[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI7_MCR                  (0${s0 + s1 + s2 + s3})
#define SPC5_SPI_DSPI0_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi0_tx1.value[0]}
#define SPC5_SPI_DSPI0_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi0_tx2.value[0]}
#define SPC5_SPI_DSPI0_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi0_rx.value[0]}
#define SPC5_SPI_DSPI1_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi1_tx1.value[0]}
#define SPC5_SPI_DSPI1_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi1_tx2.value[0]}
#define SPC5_SPI_DSPI1_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi1_rx.value[0]}
#define SPC5_SPI_DSPI2_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi2_tx1.value[0]}
#define SPC5_SPI_DSPI2_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi2_tx2.value[0]}
#define SPC5_SPI_DSPI2_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi2_rx.value[0]}
#define SPC5_SPI_DSPI3_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi3_tx1.value[0]}
#define SPC5_SPI_DSPI3_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi3_tx2.value[0]}
#define SPC5_SPI_DSPI3_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi3_rx.value[0]}
#define SPC5_SPI_DSPI4_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi4_tx1.value[0]}
#define SPC5_SPI_DSPI4_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi4_tx2.value[0]}
#define SPC5_SPI_DSPI4_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi4_rx.value[0]}
#define SPC5_SPI_DSPI5_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi5_tx1.value[0]}
#define SPC5_SPI_DSPI5_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi5_tx2.value[0]}
#define SPC5_SPI_DSPI5_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi5_rx.value[0]}
#define SPC5_SPI_DSPI6_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi6_tx1.value[0]}
#define SPC5_SPI_DSPI6_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi6_tx2.value[0]}
#define SPC5_SPI_DSPI6_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi6_rx.value[0]}
#define SPC5_SPI_DSPI7_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi7_tx1.value[0]}
#define SPC5_SPI_DSPI7_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi7_tx2.value[0]}
#define SPC5_SPI_DSPI7_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi7_rx.value[0]}
#define SPC5_SPI_DSPI0_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_0.value[0]}
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_1.value[0]}
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_2.value[0]}
#define SPC5_SPI_DSPI3_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_3.value[0]}
#define SPC5_SPI_DSPI4_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_4.value[0]}
#define SPC5_SPI_DSPI5_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_5.value[0]}
#define SPC5_SPI_DSPI6_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_6.value[0]}
#define SPC5_SPI_DSPI7_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_7.value[0]}
#define SPC5_SPI_DSPI0_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_0.value[0]}
#define SPC5_SPI_DSPI1_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_1.value[0]}
#define SPC5_SPI_DSPI2_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_2.value[0]}
#define SPC5_SPI_DSPI3_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_3.value[0]}
#define SPC5_SPI_DSPI4_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_4.value[0]}
#define SPC5_SPI_DSPI5_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_5.value[0]}
#define SPC5_SPI_DSPI6_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_6.value[0]}
#define SPC5_SPI_DSPI7_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_7.value[0]}
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       ${conf.instance.dspi_settings.dma_error_hook.value[0]}

/*
 * ICU-PWM driver system settings.
 */
#define SPC5_ICU_USE_EMIOS0_CH0             ${conf.instance.emios_settings.emios0_ch0.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH1             ${conf.instance.emios_settings.emios0_ch1.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH2             ${conf.instance.emios_settings.emios0_ch2.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH3             ${conf.instance.emios_settings.emios0_ch3.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH4             ${conf.instance.emios_settings.emios0_ch4.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH5             ${conf.instance.emios_settings.emios0_ch5.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH6             ${conf.instance.emios_settings.emios0_ch6.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH7             ${conf.instance.emios_settings.emios0_ch7.value[0]?upper_case}
#define SPC5_ICU_USE_EMIOS0_CH24            ${conf.instance.emios_settings.emios0_ch24.value[0]?upper_case}

#define SPC5_PWM_USE_EMIOS0_GROUP0          ${conf.instance.emios_settings.emios0_group0.value[0]?upper_case}
#define SPC5_PWM_USE_EMIOS0_GROUP1          ${conf.instance.emios_settings.emios0_group1.value[0]?upper_case}

#define SPC5_EMIOS0_GFR_F0F1_PRIORITY       ${conf.instance.irq_priority_settings.emios0_uc0.value[0]}
#define SPC5_EMIOS0_GFR_F2F3_PRIORITY       ${conf.instance.irq_priority_settings.emios0_uc1.value[0]}
#define SPC5_EMIOS0_GFR_F4F5_PRIORITY       ${conf.instance.irq_priority_settings.emios0_uc2.value[0]}
#define SPC5_EMIOS0_GFR_F6F7_PRIORITY       ${conf.instance.irq_priority_settings.emios0_uc3.value[0]}
#define SPC5_EMIOS0_GFR_F8F9_PRIORITY       ${conf.instance.irq_priority_settings.emios0_uc4.value[0]}
#define SPC5_EMIOS0_GFR_F10F11_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc5.value[0]}
#define SPC5_EMIOS0_GFR_F12F13_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc6.value[0]}
#define SPC5_EMIOS0_GFR_F14F15_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc7.value[0]}
#define SPC5_EMIOS0_GFR_F16F17_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc8.value[0]}
#define SPC5_EMIOS0_GFR_F18F19_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc9.value[0]}
#define SPC5_EMIOS0_GFR_F20F21_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc10.value[0]}
#define SPC5_EMIOS0_GFR_F22F23_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc11.value[0]}
#define SPC5_EMIOS0_GFR_F24F25_PRIORITY     ${conf.instance.irq_priority_settings.emios0_uc12.value[0]}

#define SPC5_EMIOS0_START_PCTL              (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_EMIOS0_STOP_PCTL               (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_ICU_USE_EMIOS1_CH24            ${conf.instance.emios_settings.emios1_ch24.value[0]?upper_case}

#define SPC5_PWM_USE_EMIOS1_GROUP0          ${conf.instance.emios_settings.emios1_group0.value[0]?upper_case}
#define SPC5_PWM_USE_EMIOS1_GROUP1          ${conf.instance.emios_settings.emios1_group1.value[0]?upper_case}
#define SPC5_PWM_USE_EMIOS1_GROUP2          ${conf.instance.emios_settings.emios1_group2.value[0]?upper_case}

#define SPC5_EMIOS1_GFR_F0F1_PRIORITY       ${conf.instance.irq_priority_settings.emios1_uc0.value[0]}
#define SPC5_EMIOS1_GFR_F2F3_PRIORITY       ${conf.instance.irq_priority_settings.emios1_uc1.value[0]}
#define SPC5_EMIOS1_GFR_F4F5_PRIORITY       ${conf.instance.irq_priority_settings.emios1_uc2.value[0]}
#define SPC5_EMIOS1_GFR_F6F7_PRIORITY       ${conf.instance.irq_priority_settings.emios1_uc3.value[0]}
#define SPC5_EMIOS1_GFR_F8F9_PRIORITY       ${conf.instance.irq_priority_settings.emios1_uc4.value[0]}
#define SPC5_EMIOS1_GFR_F10F11_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc5.value[0]}
#define SPC5_EMIOS1_GFR_F12F13_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc6.value[0]}
#define SPC5_EMIOS1_GFR_F14F15_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc7.value[0]}
#define SPC5_EMIOS1_GFR_F16F17_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc8.value[0]}
#define SPC5_EMIOS1_GFR_F18F19_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc9.value[0]}
#define SPC5_EMIOS1_GFR_F20F21_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc10.value[0]}
#define SPC5_EMIOS1_GFR_F22F23_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc11.value[0]}
#define SPC5_EMIOS1_GFR_F24F25_PRIORITY     ${conf.instance.irq_priority_settings.emios1_uc12.value[0]}

#define SPC5_EMIOS1_START_PCTL              (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_EMIOS1_STOP_PCTL               (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                ${conf.instance.flexcan_settings.flexcan_enable_filters.value[0]?upper_case}

#define SPC5_CAN_USE_FLEXCAN0               ${conf.instance.flexcan_settings.flexcan0.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan0_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_PRIORITY          ${conf.instance.irq_priority_settings.flexcan0.value[0]}
#define SPC5_CAN_NUM_RX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_rx_mailboxes.value[0]}
#define SPC5_CAN_NUM_TX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_tx_mailboxes.value[0]}
#define SPC5_CAN_FLEXCAN0_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN0_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN1               ${conf.instance.flexcan_settings.flexcan1.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan1_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_PRIORITY          ${conf.instance.irq_priority_settings.flexcan1.value[0]}
#define SPC5_CAN_FLEXCAN1_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN1_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN2               ${conf.instance.flexcan_settings.flexcan2.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN2_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan2_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN2_PRIORITY          ${conf.instance.irq_priority_settings.flexcan2.value[0]}
#define SPC5_CAN_FLEXCAN2_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN2_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN3               ${conf.instance.flexcan_settings.flexcan3.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN3_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan3_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN3_PRIORITY          ${conf.instance.irq_priority_settings.flexcan3.value[0]}
#define SPC5_CAN_FLEXCAN3_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN3_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN4               ${conf.instance.flexcan_settings.flexcan4.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN4_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan4_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN4_PRIORITY          ${conf.instance.irq_priority_settings.flexcan4.value[0]}
#define SPC5_CAN_FLEXCAN4_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN4_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN5               ${conf.instance.flexcan_settings.flexcan5.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN5_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan5_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN5_PRIORITY          ${conf.instance.irq_priority_settings.flexcan5.value[0]}
#define SPC5_CAN_FLEXCAN5_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN5_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

/*
* ADC driver system settings.
*/
[#if conf.instance.adc_settings.adc0_clock_divider.value[0] == "Peripheral clock frequency"]
  [#assign clk_f0 = "PERIPHERAL_SET_CLOCK_FREQUENCY"]
[#else]
  [#assign clk_f0 = "HALF_PERIPHERAL_SET_CLOCK_FREQUENCY"]
[/#if]

[#if conf.instance.adc_settings.dma_mode.value[0] == "true" ]
  [#assign dma_mode = "SPC5_ADC_DMA_ON"]
[#else]
  [#assign dma_mode = "SPC5_ADC_DMA_OFF"]
[/#if]

#define SPC5_ADC_DMA_MODE                   ${dma_mode}

#define SPC5_ADC_USE_ADC0                   ${conf.instance.adc_settings.adc0.value[0]?upper_case}
#define SPC5_ADC_ADC0_CLK_FREQUENCY         ${clk_f0}
#define SPC5_ADC_ADC0_AUTO_CLOCK_OFF        ${conf.instance.adc_settings.adc0_auto_clock_off_mode.value[0]?upper_case}
#define SPC5_ADC_ADC0_WD_PRIORITY           ${conf.instance.irq_priority_settings.adc0.value[0]}
#define SPC5_ADC_ADC0_EOC_PRIORITY          SPC5_ADC_ADC0_WD_PRIORITY
#define SPC5_ADC_ADC0_DMA_CH_ID             ${conf.instance.edma_mux_settings.adc_channels.adc0.value[0]}
#define SPC5_ADC_ADC0_DMA_IRQ_PRIO          ${conf.instance.irq_priority_settings.adc0.value[0]}
#define SPC5_ADC_ADC0_START_PCTL            (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ADC_ADC0_STOP_PCTL             (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

[#if conf.instance.adc_settings.adc1_clock_divider.value[0] == "Peripheral clock frequency"]
  [#assign clk_f1 = "PERIPHERAL_SET_CLOCK_FREQUENCY"]
[#else]
  [#assign clk_f1 = "HALF_PERIPHERAL_SET_CLOCK_FREQUENCY"]
[/#if]
#define SPC5_ADC_USE_ADC1                   ${conf.instance.adc_settings.adc1.value[0]?upper_case}
#define SPC5_ADC_ADC1_CLK_FREQUENCY         ${clk_f1}
#define SPC5_ADC_ADC1_AUTO_CLOCK_OFF        ${conf.instance.adc_settings.adc1_auto_clock_off_mode.value[0]?upper_case}
#define SPC5_ADC_ADC1_WD_PRIORITY           ${conf.instance.irq_priority_settings.adc1.value[0]}
#define SPC5_ADC_ADC1_EOC_PRIORITY          SPC5_ADC_ADC1_WD_PRIORITY
#define SPC5_ADC_ADC1_DMA_CH_ID             ${conf.instance.edma_mux_settings.adc_channels.adc1.value[0]}
#define SPC5_ADC_ADC1_DMA_IRQ_PRIO          ${conf.instance.irq_priority_settings.adc1.value[0]}
#define SPC5_ADC_ADC1_START_PCTL            (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_ADC_ADC1_STOP_PCTL             (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#endif /* _MCUCONF_H_ */
