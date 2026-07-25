/**
  ******************************************************************************
  * @file    system_stm32wlxx.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex Device System Source File for STM32WLxx devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Apache License, Version 2.0,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/Apache-2.0
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32wlxx_system
  * @{
  */

/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_STM32WLXX_H
#define __SYSTEM_STM32WLXX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/** @addtogroup STM32WLxx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup STM32WLxx_System_Exported_types
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) from within HAL_Init()
      2) by calling CMSIS function SystemCoreClockUpdate()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  */

extern uint32_t SystemCoreClock;            /*!< System Clock Frequency */

extern const uint32_t AHBPrescTable[16];    /*!< AHB prescalers table values */
extern const uint32_t APBPrescTable[8];     /*!< APB prescalers table values */
extern const uint32_t MSIRangeTable[16];    /*!< MSI ranges table values     */

/**
  * @}
  */

/** @addtogroup STM32WLxx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32WLxx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32WLxx_System_Exported_Functions
  * @{
  */
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32WLXX_H */

/**
  * @}
  */
  
/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
