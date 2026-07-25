[#ftl]
[@pp.dropOutputFile /]
[@pp.changeOutputFile name="hal_spi_lld_cfg.c" /]
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
 * @file    hal_spi_lld_cfg.c
 * @brief   SPI Driver configuration code.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"
#include "hal_spi_lld_cfg.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

[#list conf.instance.dspi_settings.spi_configurations.configs.spi_configuration_settings as settings]
  [#assign name = settings.symbolic_name.value[0]?trim /]
  [#-- Transfer group.--]
  [#assign cpol = settings.transfer.clock_polarity[0].@index[0]?trim?number /]
  [#assign cpha = settings.transfer.clock_phase[0].@index[0]?trim?number /]
  [#assign frame_size = settings.transfer.frame_size[0].@index[0]?trim?number + 4 /]
  [#assign frame_ordering = settings.transfer.frame_ordering[0].@index[0]?trim?number /]
  [#-- Timings group.--]
  [#assign brp = settings.timings.baud_rate_prescaler.value[0]?trim /]
  [#assign brd = settings.timings.baud_rate_divider.value[0]?trim /]
  [#assign dbr = (settings.timings.double_baud_rate.value[0]?trim?lower_case == "true") /]
  [#assign cssckp = settings.timings.cssck_prescaler.value[0]?trim /]
  [#assign cssckd = settings.timings.cssck_divider.value[0]?trim /]
  [#assign ascp = settings.timings.asc_prescaler.value[0]?trim /]
  [#assign ascd = settings.timings.asc_divider.value[0]?trim /]
  [#assign dtp = settings.timings.dt_prescaler.value[0]?trim /]
  [#assign dtd = settings.timings.dt_divider.value[0]?trim /]
  [#-- Chip Select group.--]
  [#assign mode = settings.chip_select.mode[0].@index[0]?trim?number /]
  [#assign gpio_port = settings.chip_select.gpio_port.value[0]?trim /]
  [#assign gpio_bit = settings.chip_select.gpio_bit.value[0]?trim?number /]
  [#assign pcs_line = settings.chip_select.pcs_line[0].@index[0]?trim?number /]
  [#-- Notifications group.--]
  [#assign cb = settings.notifications.transfer_complete_callback.value[0]?string?trim /]
  [#if cb == ""]
    [#assign cb = "NULL" /]
  [/#if]
/**
 * @brief   Structure defining the SPI configuration "${name}".
 */
const SPIConfig spi_config_${name} = {
  ${cb},
  ${gpio_port},
  ${gpio_bit},
  0 | SPC5_CTAR_FMSZ(${frame_size})[#rt]
  [#if dbr]
 | SPC5_CTAR_DBR[#rt]
  [/#if]
  [#if cpol != 0]
 | SPC5_CTAR_CPOL[#rt]
  [/#if]
  [#if cpha != 0]
 | SPC5_CTAR_CPHA[#rt]
  [/#if]
  [#if frame_ordering != 0]
 | SPC5_CTAR_LSBFE[#rt]
  [/#if]
 |
      SPC5_CTAR_PCSSCK_${cssckp} | SPC5_CTAR_PASC_${ascp} |
      SPC5_CTAR_PDT_${dtp} | SPC5_CTAR_PBR_${brp} |
      SPC5_CTAR_CSSCK_${cssckd} | SPC5_CTAR_ASC_${ascd} |
      SPC5_CTAR_DT_${dtd} | SPC5_CTAR_BR_${brd},
  0[#rt]
  [#if mode != 1]
 | SPC5_PUSHR_CONT[#rt]
  [/#if]
 | SPC5_PUSHR_CTAS(0) | SPC5_PUSHR_PCS(${pcs_line})
};

[/#list]
 /*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

 /*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

#endif /* HAL_USE_SPI */

/** @} */
