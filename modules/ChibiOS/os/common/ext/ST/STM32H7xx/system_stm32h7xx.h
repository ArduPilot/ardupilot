/**
  ******************************************************************************
  * @file    system_stm32h7xx.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-Mx Device System Source File for STM32H7xx devices.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
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

/** @addtogroup stm32h7xx_system
  * @{
  */

/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef SYSTEM_STM32H7XX_H
#define SYSTEM_STM32H7XX_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup STM32H7xx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup STM32H7xx_System_Exported_types
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
extern uint32_t SystemCoreClock;             /*!< System Domain1 Clock Frequency  */
extern uint32_t SystemD2Clock;               /*!< System Domain2 Clock Frequency  */
extern const  uint8_t D1CorePrescTable[16] ; /*!< D1CorePrescTable prescalers table values */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32H7xx_System_Exported_Functions
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

#endif /* SYSTEM_STM32H7XX_H */

/**
  * @}
  */

/**
  * @}
  */
