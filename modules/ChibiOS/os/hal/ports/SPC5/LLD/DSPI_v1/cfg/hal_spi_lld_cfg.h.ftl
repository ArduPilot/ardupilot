[#ftl]
[@pp.dropOutputFile /]
[@pp.changeOutputFile name="hal_spi_lld_cfg.h" /]
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
 * @file    hal_spi_lld_cfg.h
 * @brief   SPI Driver configuration macros and structures.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI_LLD_CFG_H_
#define _SPI_LLD_CFG_H_

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* List of the SPIConfig structures defined in spi_lld_cfg.c.*/
[#list conf.instance.dspi_settings.spi_configurations.configs.spi_configuration_settings as settings]
extern const SPIConfig spi_config_${settings.symbolic_name.value[0]?trim};
[/#list]

#ifdef __cplusplus
extern "C" {
#endif
  /* List of the callback functions referenced from the SPIConfig
     structures in spi_lld_cfg.c.*/
[#assign transfer_complete_callbacks = []]
[#list conf.instance.dspi_settings.spi_configurations.configs.spi_configuration_settings as settings]
  [#assign callback = settings.notifications.transfer_complete_callback.value[0]?string?trim /]
  [#if callback != ""]
    [#if !transfer_complete_callbacks?seq_contains(callback)]
      [#assign transfer_complete_callbacks = transfer_complete_callbacks + [callback]]
    [/#if]
  [/#if]
[/#list]
[#list transfer_complete_callbacks?sort as cb]
  void ${cb}(SPIDriver *spip);
[/#list]
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* _SPI_LLD_CFG_H_ */

/** @} */
