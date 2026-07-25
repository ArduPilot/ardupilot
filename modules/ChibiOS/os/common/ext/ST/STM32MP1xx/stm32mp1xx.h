/**
  ******************************************************************************
  * @file    stm32mp1xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32MP1xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32MP1xx device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32mp1xx
  * @{
  */

#ifndef __STM32MP1xx_H
#define __STM32MP1xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/* Uncomment the line below according to the target STM32MP1 device used in your
   application
  */

#if !defined (STM32MP1)
#define STM32MP1
#endif /* STM32MP1 */

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
#define __STM32MP1xx_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32MP1xx_CMSIS_VERSION_SUB1   (0x04U) /*!< [23:16] sub1 version */
#define __STM32MP1xx_CMSIS_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __STM32MP1xx_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32MP1xx_CMSIS_VERSION        ((__CMSIS_DEVICE_VERSION_MAIN     << 24)\
                                      |(__CMSIS_DEVICE_HAL_VERSION_SUB1 << 16)\
                                      |(__CMSIS_DEVICE_HAL_VERSION_SUB2 << 8 )\
                                      |(__CMSIS_DEVICE_HAL_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */
#if defined(CORE_CM4)
#if defined(STM32MP15xx)  /* keep for backward compatibility STM32MP15xx = STM32MP157Cxx */
  #include "stm32mp157cxx_cm4.h"
#elif defined(STM32MP157Axx)
  #include "stm32mp157axx_cm4.h"
#elif defined(STM32MP157Cxx)
  #include "stm32mp157cxx_cm4.h"
#elif defined(STM32MP157Dxx)
  #include "stm32mp157dxx_cm4.h"
#elif defined(STM32MP157Fxx)
  #include "stm32mp157fxx_cm4.h"
#elif defined(STM32MP153Axx)
  #include "stm32mp153axx_cm4.h"
#elif defined(STM32MP153Cxx)
  #include "stm32mp153cxx_cm4.h"
#elif defined(STM32MP153Dxx)
  #include "stm32mp153dxx_cm4.h"
#elif defined(STM32MP153Fxx)
  #include "stm32mp153fxx_cm4.h"
#elif defined(STM32MP151Axx)
  #include "stm32mp151axx_cm4.h"
#elif defined(STM32MP151Cxx)
  #include "stm32mp151cxx_cm4.h"
#elif defined(STM32MP151Dxx)
  #include "stm32mp151dxx_cm4.h"
#elif defined(STM32MP151Fxx)
  #include "stm32mp151fxx_cm4.h"
#else
 #error "Please select first the target STM32MP1xx device used in your application (in stm32mp1xx.h file)"
#endif
#endif

#if defined(CORE_CA7)
#if defined(STM32MP15xx)  /* keep for backward compatibility STM32MP15xx = STM32MP157Cxx */
  #include "stm32mp157cxx_ca7.h"
#elif defined(STM32MP157Axx)
  #include "stm32mp157axx_ca7.h"
#elif defined(STM32MP157Cxx)
  #include "stm32mp157cxx_ca7.h"
#elif defined(STM32MP157Dxx)
  #include "stm32mp157dxx_ca7.h"
#elif defined(STM32MP157Fxx)
  #include "stm32mp157fxx_ca7.h"
#elif defined(STM32MP153Axx)
  #include "stm32mp153axx_ca7.h"
#elif defined(STM32MP153Cxx)
  #include "stm32mp153cxx_ca7.h"
#elif defined(STM32MP153Dxx)
  #include "stm32mp153dxx_ca7.h"
#elif defined(STM32MP153Fxx)
  #include "stm32mp153fxx_ca7.h"
#elif defined(STM32MP151Axx)
  #include "stm32mp151axx_ca7.h"
#elif defined(STM32MP151Cxx)
  #include "stm32mp151cxx_ca7.h"
#elif defined(STM32MP151Dxx)
  #include "stm32mp151dxx_ca7.h"
#elif defined(STM32MP151Fxx)
  #include "stm32mp151fxx_ca7.h"
#else
 #error "Please select first the target STM32MP1xx device used in your application (in stm32mp1xx.h file)"
#endif
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

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32mp1xx_hal.h"
#endif /* USE_HAL_DRIVER */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32MP1xx_H */
/**
  * @}
  */

/**
  * @}
  */




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
