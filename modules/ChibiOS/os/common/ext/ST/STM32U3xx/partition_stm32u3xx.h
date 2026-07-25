/**
  ******************************************************************************
  * @file    partition_stm32u3xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32U3xx Device Header File for Initial Setup for
  *          Secure / Non-Secure Zones based on CMSIS CORE V5.4.0
  *
  *          The file is included in system_stm32u3xx_s.c in secure application.
  *          It includes the configuration section that allows to select the
  *          STM32U3xx device partitioning file for system core secure attributes
  *          and interrupt secure and non-secure assignment.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32u3xx
  * @{
  */

#ifndef PARTITION_STM32U3XX_H
#define PARTITION_STM32U3XX_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Secure_configuration_section
  * @{
  */

#if defined(STM32U375xx)
  #include "partition_stm32u375xx.h"
#elif defined(STM32U385xx)
  #include "partition_stm32u385xx.h"
#else
  #error "Please select first the target STM32U3xx device used in your application (in stm32u3xx.h file)"
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PARTITION_STM32U3XX_H */
/**
  * @}
  */

/**
  * @}
  */




