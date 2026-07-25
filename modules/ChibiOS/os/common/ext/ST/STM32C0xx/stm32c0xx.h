/**
  ******************************************************************************
  * @file    stm32c0xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32C0xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32C0xx device used in the target application
  *              - To use or not the peripherals drivers in application code(i.e.
  *                code will be based on direct access to peripherals registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/** @addtogroup stm32c0xx
  * @{
  */

#ifndef STM32C0xx_H
#define STM32C0xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32C0)
#define STM32C0
#endif /* STM32C0 */

/* Uncomment the line below according to the target STM32C0 device used in your
   application
  */

#if !defined (STM32C011xx) && !defined (STM32C031xx) \
    && !defined (STM32C051xx) && !defined (STM32C071xx) \
    && !defined (STM32C091xx) && !defined (STM32C092xx)
  /* #define STM32C011xx */   /*!< STM32C011xx Devices */
  /* #define STM32C031xx */   /*!< STM32C031xx Devices */
  /* #define STM32C051xx */   /*!< STM32C051xx Devices */
  /* #define STM32C071xx */   /*!< STM32C071xx Devices */
  /* #define STM32C091xx */   /*!< STM32C091xx Devices */
  /* #define STM32C092xx */   /*!< STM32C092xx Devices */
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
  * @brief CMSIS Device version number V1.0.0
  */
#define __STM32C0_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32C0_CMSIS_VERSION_SUB1   (0x03U) /*!< [23:16] sub1 version */
#define __STM32C0_CMSIS_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __STM32C0_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32C0_CMSIS_VERSION        ((__STM32C0_CMSIS_VERSION_MAIN << 24)\
                                       |(__STM32C0_CMSIS_VERSION_SUB1 << 16)\
                                       |(__STM32C0_CMSIS_VERSION_SUB2 << 8 )\
                                       |(__STM32C0_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32C011xx)
  #include "stm32c011xx.h"
#elif defined(STM32C031xx)
  #include "stm32c031xx.h"
#elif defined(STM32C051xx)
  #include "stm32c051xx.h"  
#elif defined(STM32C071xx)
  #include "stm32c071xx.h"
#elif defined(STM32C091xx)
  #include "stm32c091xx.h"
#elif defined(STM32C092xx)
  #include "stm32c092xx.h"
#else
 #error "Please select first the target STM32C0xx device used in your application (in stm32c0xx.h file)"
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
  SUCCESS = 0,
  ERROR = !SUCCESS
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

/* Use of interrupt control for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    SET_BIT((REG), (BIT));                                   \
    __set_PRIMASK(primask);                                  \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    CLEAR_BIT((REG), (BIT));                                 \
    __set_PRIMASK(primask);                                  \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)            \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    MODIFY_REG((REG), (CLEARMSK), (SETMASK));                \
    __set_PRIMASK(primask);                                  \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define ATOMIC_SETH_BIT(REG, BIT) ATOMIC_SET_BIT(REG, BIT)                                   \

/* Atomic 16-bit register access macro to clear one or several bits */
#define ATOMIC_CLEARH_BIT(REG, BIT) ATOMIC_CLEAR_BIT(REG, BIT)                               \

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK) ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK) \

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32c0xx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32C0xx_H */
/**
  * @}
  */

/**
  * @}
  */

