/**
  ******************************************************************************
  * @file    stm32wlxx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32WLxx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32WLxx device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
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

/** @addtogroup stm32wlxx
  * @{
  */

#ifndef __STM32WLxx_H
#define __STM32WLxx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32WL)
#define STM32WL
#endif /* STM32WL */

/* Uncomment the line below according to the target STM32WL device used in your
   application
  */

#if !defined (STM32WL55xx) && !defined (STM32WL54xx) && !defined (STM32WLE5xx) && !defined (STM32WLE4xx)
  /* #define STM32WL55xx */   /*!< STM32WL55xx Devices */
  /* #define STM32WL54xx */   /*!< STM32WL54xx Devices */
  /* #define STM32WLE5xx */   /*!< STM32WLE5xx Devices */
  /* #define STM32WLE4xx */   /*!< STM32WLE4xx Devices */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
  /*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

/**
  * @brief CMSIS Device version number
  */
#define __STM32WLxx_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32WLxx_CMSIS_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32WLxx_CMSIS_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __STM32WLxx_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32WLxx_CMSIS_DEVICE_VERSION        ((__STM32WLxx_CMSIS_VERSION_MAIN << 24)\
                                                |(__STM32WLxx_CMSIS_VERSION_SUB1 << 16)\
                                                |(__STM32WLxx_CMSIS_VERSION_SUB2 << 8 )\
                                                |(__STM32WLxx_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32WL55xx)
  #include "stm32wl55xx.h"
#elif defined(STM32WLE5xx)
  #include "stm32wle5xx.h"
#elif defined(STM32WL54xx)
  #include "stm32wl54xx.h"
#elif defined(STM32WLE4xx)
  #include "stm32wle4xx.h"
#else
 #error "Please select first the target STM32WLxx device used in your application, for instance xxx (in stm32wlxx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  ERROR = 0,
  SUCCESS = !ERROR
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#if defined(CORE_CM0PLUS)
#else
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
#endif
/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32wlxx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32WLxx_H */
/**
  * @}
  */

/**
  * @}
  */




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
