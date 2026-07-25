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
#define SPC5_NO_INIT                        ${conf.instance.initialization_settings.do_not_init.value[0]?upper_case}
#define SPC5_ALLOW_OVERCLOCK                ${conf.instance.initialization_settings.allow_overclocking.value[0]?upper_case}
#define SPC5_DISABLE_WATCHDOG               ${conf.instance.initialization_settings.disable_watchdog.value[0]?upper_case}
#define SPC5_FMPLL0_CLK_SRC                 SPC5_FMPLL_SRC_${conf.instance.initialization_settings.fmpll0_settings.clock_source.value[0]}
#define SPC5_FMPLL0_IDF_VALUE               ${conf.instance.initialization_settings.fmpll0_settings.idf_value.value[0]}
#define SPC5_FMPLL0_NDIV_VALUE              ${conf.instance.initialization_settings.fmpll0_settings.ndiv_value.value[0]}
#define SPC5_FMPLL0_ODF                     ${conf.instance.initialization_settings.fmpll0_settings.odf_value.value[0]}
#define SPC5_FMPLL1_CLK_SRC                 SPC5_FMPLL_SRC_${conf.instance.initialization_settings.fmpll1_settings.clock_source.value[0]}
#define SPC5_FMPLL1_IDF_VALUE               ${conf.instance.initialization_settings.fmpll1_settings.idf_value.value[0]}
#define SPC5_FMPLL1_NDIV_VALUE              ${conf.instance.initialization_settings.fmpll1_settings.ndiv_value.value[0]}
#define SPC5_FMPLL1_ODF                     ${conf.instance.initialization_settings.fmpll1_settings.odf_value.value[0]}
#define SPC5_SYSCLK_DIVIDER_VALUE           ${conf.instance.initialization_settings.clocks.system_clock_divider.value[0]}
#define SPC5_AUX0CLK_SRC                    SPC5_CGM_SS_${conf.instance.initialization_settings.clocks.aux0_clock_source.value[0]}
#define SPC5_MCONTROL_DIVIDER_VALUE         ${conf.instance.initialization_settings.clocks.motor_control_clock_divider.value[0]}
#define SPC5_SWG_DIVIDER_VALUE              ${conf.instance.initialization_settings.clocks.swg_clock_divider.value[0]}
#define SPC5_AUX1CLK_SRC                    SPC5_CGM_SS_${conf.instance.initialization_settings.clocks.aux1_clock_source.value[0]}
#define SPC5_FLEXRAY_DIVIDER_VALUE          ${conf.instance.initialization_settings.clocks.flexray_clock_divider.value[0]}
#define SPC5_AUX2CLK_SRC                    SPC5_CGM_SS_${conf.instance.initialization_settings.clocks.aux2_clock_source.value[0]}
#define SPC5_FLEXCAN_DIVIDER_VALUE          ${conf.instance.initialization_settings.clocks.flexcan_clock_divider.value[0]}
#define SPC5_CLOCK_FAILURE_HOOK()           ${conf.instance.initialization_settings.clocks.clock_failure_hook.value[0]}

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
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#define SPC5_EDMA_ERROR_HANDLER()           osalSysHalt("DMA failure")

/*
 * PWM driver system settings.
 */
[#assign pwm0_all_sm = conf.instance.flexpwm_settings.synchronized_flexpwm0.value[0]?upper_case /]
[#assign pwm1_all_sm = conf.instance.flexpwm_settings.synchronized_flexpwm1.value[0]?upper_case /]
#define SPC5_PWM0_USE_SYNC_SMOD             ${pwm0_all_sm}
#define SPC5_PWM1_USE_SYNC_SMOD             ${pwm1_all_sm}
#define SPC5_PWM_USE_SMOD0                  ${conf.instance.flexpwm_settings.flexpwm0_sm0.value[0]?upper_case}
[#if pwm0_all_sm == "FALSE"]
#define SPC5_PWM_USE_SMOD1                  ${conf.instance.flexpwm_settings.flexpwm0_sm1.value[0]?upper_case}
#define SPC5_PWM_USE_SMOD2                  ${conf.instance.flexpwm_settings.flexpwm0_sm2.value[0]?upper_case}
#define SPC5_PWM_USE_SMOD3                  ${conf.instance.flexpwm_settings.flexpwm0_sm3.value[0]?upper_case}
[#else]
#define SPC5_PWM_USE_SMOD1                  TRUE
#define SPC5_PWM_USE_SMOD2                  TRUE
#define SPC5_PWM_USE_SMOD3                  TRUE
[/#if]
#define SPC5_PWM_SMOD0_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm0_sm0.value[0]}
#define SPC5_PWM_SMOD1_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm0_sm1.value[0]}
#define SPC5_PWM_SMOD2_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm0_sm2.value[0]}
#define SPC5_PWM_SMOD3_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm0_sm3.value[0]}

#define SPC5_PWM_USE_SMOD4                  ${conf.instance.flexpwm_settings.flexpwm1_sm0.value[0]?upper_case}
[#if pwm1_all_sm == "FALSE"]
#define SPC5_PWM_USE_SMOD5                  ${conf.instance.flexpwm_settings.flexpwm1_sm1.value[0]?upper_case}
#define SPC5_PWM_USE_SMOD6                  ${conf.instance.flexpwm_settings.flexpwm1_sm2.value[0]?upper_case}
#define SPC5_PWM_USE_SMOD7                  ${conf.instance.flexpwm_settings.flexpwm1_sm3.value[0]?upper_case}
[#else]
#define SPC5_PWM_USE_SMOD5                  TRUE
#define SPC5_PWM_USE_SMOD6                  TRUE
#define SPC5_PWM_USE_SMOD7                  TRUE
[/#if]
#define SPC5_PWM_SMOD4_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm1_sm0.value[0]}
#define SPC5_PWM_SMOD5_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm1_sm1.value[0]}
#define SPC5_PWM_SMOD6_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm1_sm2.value[0]}
#define SPC5_PWM_SMOD7_PRIORITY             ${conf.instance.irq_priority_settings.flexpwm1_sm3.value[0]}

/*
 * ICU driver system settings.
 */
#define SPC5_ICU_USE_SMOD0                  ${conf.instance.etimer_settings.etimer0_ch0.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD1                  ${conf.instance.etimer_settings.etimer0_ch1.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD2                  ${conf.instance.etimer_settings.etimer0_ch2.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD3                  ${conf.instance.etimer_settings.etimer0_ch3.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD4                  ${conf.instance.etimer_settings.etimer0_ch4.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD5                  ${conf.instance.etimer_settings.etimer0_ch5.value[0]?upper_case}
#define SPC5_ICU_ETIMER0_PRIORITY           ${conf.instance.irq_priority_settings.etimer0.value[0]}

#define SPC5_ICU_USE_SMOD6                  ${conf.instance.etimer_settings.etimer1_ch0.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD7                  ${conf.instance.etimer_settings.etimer1_ch1.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD8                  ${conf.instance.etimer_settings.etimer1_ch2.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD9                  ${conf.instance.etimer_settings.etimer1_ch3.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD10                 ${conf.instance.etimer_settings.etimer1_ch4.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD11                 ${conf.instance.etimer_settings.etimer1_ch5.value[0]?upper_case}
#define SPC5_ICU_ETIMER1_PRIORITY           ${conf.instance.irq_priority_settings.etimer1.value[0]}

#define SPC5_ICU_USE_SMOD12                 ${conf.instance.etimer_settings.etimer2_ch0.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD13                 ${conf.instance.etimer_settings.etimer2_ch1.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD14                 ${conf.instance.etimer_settings.etimer2_ch2.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD15                 ${conf.instance.etimer_settings.etimer2_ch3.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD16                 ${conf.instance.etimer_settings.etimer2_ch4.value[0]?upper_case}
#define SPC5_ICU_USE_SMOD17                 ${conf.instance.etimer_settings.etimer2_ch5.value[0]?upper_case}
#define SPC5_ICU_ETIMER2_PRIORITY           ${conf.instance.irq_priority_settings.etimer2.value[0]}

/*
 * SERIAL driver system settings.
 */
#define SPC5_SERIAL_USE_LINFLEX0            ${(conf.instance.linflex_settings.linflex0.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_USE_LINFLEX1            ${(conf.instance.linflex_settings.linflex1.value[0] == "Serial")?string?upper_case}
#define SPC5_SERIAL_LINFLEX0_PRIORITY       ${conf.instance.irq_priority_settings.linflex0.value[0]}
#define SPC5_SERIAL_LINFLEX1_PRIORITY       ${conf.instance.irq_priority_settings.linflex1.value[0]}

/*
 * SPI driver system settings.
 */
#define SPC5_SPI_USE_DSPI0                  ${conf.instance.dspi_settings.dspi_0.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI1                  ${conf.instance.dspi_settings.dspi_1.value[0]?upper_case}
#define SPC5_SPI_USE_DSPI2                  ${conf.instance.dspi_settings.dspi_2.value[0]?upper_case}
#define SPC5_SPI_DMA_MODE                   SPC5_SPI_DMA_${conf.instance.dspi_settings.dma_mode.value[0]?upper_case?replace(" ", "_")}
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs3[0].@index[0]?trim?number] /]
[#assign s4 = [""," | SPC5_MCR_PCSIS4"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs4[0].@index[0]?trim?number] /]
[#assign s5 = [""," | SPC5_MCR_PCSIS5"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs5[0].@index[0]?trim?number] /]
[#assign s6 = [""," | SPC5_MCR_PCSIS6"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs6[0].@index[0]?trim?number] /]
[#assign s7 = [""," | SPC5_MCR_PCSIS7"][conf.instance.dspi_settings.inactive_states.dspi_0___pcs7[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI0_MCR                  (0${s0 + s1 + s2 + s3 + s4 + s5 + s6 + s7})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_1___pcs3[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI1_MCR                  (0${s0 + s1 + s2 + s3})
[#assign s0 = [""," | SPC5_MCR_PCSIS0"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs0[0].@index[0]?trim?number] /]
[#assign s1 = [""," | SPC5_MCR_PCSIS1"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs1[0].@index[0]?trim?number] /]
[#assign s2 = [""," | SPC5_MCR_PCSIS2"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs2[0].@index[0]?trim?number] /]
[#assign s3 = [""," | SPC5_MCR_PCSIS3"][conf.instance.dspi_settings.inactive_states.dspi_2___pcs3[0].@index[0]?trim?number] /]
#define SPC5_SPI_DSPI2_MCR                  (0${s0 + s1 + s2 + s3})
#define SPC5_SPI_DSPI0_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi0_tx1.value[0]}
#define SPC5_SPI_DSPI0_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi0_tx2.value[0]}
#define SPC5_SPI_DSPI0_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi0_rx.value[0]}
#define SPC5_SPI_DSPI1_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi1_tx1.value[0]}
#define SPC5_SPI_DSPI1_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi1_tx2.value[0]}
#define SPC5_SPI_DSPI1_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi1_rx.value[0]}
#define SPC5_SPI_DSPI2_TX1_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi2_tx1.value[0]}
#define SPC5_SPI_DSPI2_TX2_DMA_CH_ID        ${conf.instance.edma_mux_settings.dspi_channels.dspi2_tx2.value[0]}
#define SPC5_SPI_DSPI2_RX_DMA_CH_ID         ${conf.instance.edma_mux_settings.dspi_channels.dspi2_rx.value[0]}
#define SPC5_SPI_DSPI0_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_0.value[0]}
#define SPC5_SPI_DSPI1_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_1.value[0]}
#define SPC5_SPI_DSPI2_DMA_IRQ_PRIO         ${conf.instance.irq_priority_settings.dspi_2.value[0]}
#define SPC5_SPI_DSPI0_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_0.value[0]}
#define SPC5_SPI_DSPI1_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_1.value[0]}
#define SPC5_SPI_DSPI2_IRQ_PRIO             ${conf.instance.irq_priority_settings.dspi_2.value[0]}
#define SPC5_SPI_DMA_ERROR_HOOK(spip)       ${conf.instance.dspi_settings.dma_error_hook.value[0]}

/*
 * CAN driver system settings.
 */
#define SPC5_CAN_USE_FILTERS                ${conf.instance.flexcan_settings.flexcan_enable_filters.value[0]?upper_case}

#define SPC5_CAN_USE_FLEXCAN0               ${conf.instance.flexcan_settings.flexcan0.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan0_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN0_IRQ_PRIORITY      ${conf.instance.irq_priority_settings.flexcan0.value[0]}
#define SPC5_CAN_NUM_RX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_rx_mailboxes.value[0]}
#define SPC5_CAN_NUM_TX_MAILBOXES			${conf.instance.flexcan_settings.can_configurations.mailboxes_configuration.number_of_tx_mailboxes.value[0]}
#define SPC5_CAN_FLEXCAN0_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN0_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN1               ${conf.instance.flexcan_settings.flexcan1.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan1_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN1_IRQ_PRIORITY      ${conf.instance.irq_priority_settings.flexcan1.value[0]}
#define SPC5_CAN_FLEXCAN1_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN1_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
                                             SPC5_ME_PCTL_LP(0))

#define SPC5_CAN_USE_FLEXCAN2               ${conf.instance.flexcan_settings.flexcan2.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN2_USE_EXT_CLK       ${conf.instance.flexcan_settings.flexcan2_use_external_clock.value[0]?upper_case}
#define SPC5_CAN_FLEXCAN2_IRQ_PRIORITY      ${conf.instance.irq_priority_settings.flexcan2.value[0]}
#define SPC5_CAN_FLEXCAN2_START_PCTL        (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#define SPC5_CAN_FLEXCAN2_STOP_PCTL         (SPC5_ME_PCTL_RUN(0) |          \
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
