/**
  ******************************************************************************
  * @file    stm32u5xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32U5xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32U5xx device used in the target application
  *              - To use or not the peripheral's drivers in application code(i.e.
  *                code will be based on direct access to peripheral's registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/** @addtogroup stm32u5xx
  * @{
  */

#ifndef STM32U5xx_H
#define STM32U5xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined (STM32U5)
#define STM32U5
#endif /* STM32U5 */

/* Uncomment the line below according to the target STM32U5 device used in your
   application
  */

#if !defined (STM32U575xx)  && !defined (STM32U585xx) \
    && !defined (STM32U595xx)  && !defined (STM32U599xx) \
    && !defined (STM32U5A5xx)  && !defined (STM32U5A9xx) \
    && !defined (STM32U5F7xx)  && !defined (STM32U5G7xx) \
    && !defined (STM32U5F9xx)  && !defined (STM32U5G9xx) \
    && !defined (STM32U535xx)  && !defined (STM32U545xx) \
  /* #define STM32U575xx */   /*!< STM32U575CIU6 STM32U575CIT6 STM32U575RIT6 STM32U575VIT6 STM32U575ZIT6 STM32U575QII6 STM32U575AII6 STM32U575CIU6Q STM32U575CIT6Q STM32U575OIY6Q STM32U575VIT6Q STM32U575QII6Q STM32U575ZIT6Q STM32U575RIT6Q STM32U575CGU6 STM32U575CGT6 STM32U575RGT6 STM32U575VGT6 STM32U575ZGT6 STM32U575QGI6 STM32U575AGI6 STM32U575CGU6Q STM32U575CGT6Q STM32U575OGY6Q STM32U575VGT6Q STM32U575QGI6Q STM32U575ZGT6Q STM32U575RGT6Q STM32U575AGI6Q Devices  */
  /* #define STM32U585xx */   /*!< STM32U585CIU6 STM32U585CIT6 STM32U585RIT6 STM32U585VIT6 STM32U585AII6 STM32U585QII6 STM32U585ZIT6 STM32U585OIY6Q STM32U585VIT6Q STM32U585QEI6Q STM32U585RIT6Q STM32U585AII6Q STM32U585CIU6Q STM32U585CIT6Q STM32U585ZET6Q Devices  */
  /* #define STM32U595xx */   /*!< STM32U595AJH6 STM32U595ZJT6 STM32U595QJI6 STM32U595VJT6 STM32U595RJT6  STM32U595AJH6Q  STM32U595ZJY6QTR  STM32U595ZJT6Q  STM32U595QJI6Q  STM32U595VJT6Q  STM32U595RJT6Q STM32U595AIH6 STM32U595ZIT6 STM32U595QII6 STM32U595VIT6 STM32U595RIT6  STM32U595AIH6Q STM32U595ZIY6QTR STM32U595ZIT6Q STM32U595QII6Q STM32U595VIT6Q STM32U595RIT6Q Devices */
  /* #define STM32U599xx */   /*!< STM32U599VJT6 STM32U599NJH6Q  STM32U599BJY6QTR  STM32U599ZJY6QTR STM32U599ZJT6Q   STM32U599VJT6Q STM32U599NIH6Q  STM32U599ZIY6QTR STM32U599ZIT6Q STM32U599VIT6Q Devices */
  /* #define STM32U5A5xx */   /*!< STM32U5A5AJH6 STM32U5A5ZJT6 STM32U5A5QJI6 STM32U5A5VJT6 STM32U5A5RJT6 STM32U5A5AJH6Q STM32U5A5ZJY6QTR STM32U5A5ZJT6Q STM32U5A5QJI6Q STM32U5A5VJT6Q STM32U5A5RJT6Q STM32U5A5QII3Q Devices */
  /* #define STM32U5A9xx */   /*!< STM32U5A9NJH6Q STM32U5A9BJY6QTR STM32U5A9ZJY6QTR STM32U5A9ZJT6Q STM32U5A9VJT6Q Devices */
  /* #define STM32U5F7xx */   /*!< STM32U5F7VJT6Q STM32U5F7VJT6 STM32U5F7VIT6Q STM32U5F7VIT6 Devices */
  /* #define STM32U5G7xx */   /*!< STM32U5G7VJT6Q STM32U5G7VJT6 Devices */
  /* #define STM32U5F9xx */   /*!< STM32U5F9NJH6Q STM32U5F9BJY6QTR STM32U5F9ZJJ6QTR STM32U5F9ZJT6Q STM32U5F9VJT6Q STM32U5F9ZIJ6QTR STM32U5F9ZIT6Q STM32U5F9VIT6Q Devices */
  /* #define STM32U5G9xx */   /*!< STM32U5G9NJH6Q STM32U5G9BJY6QTR STM32U5G9ZJJ6QTR STM32U5G9ZJT6Q STM32U5G9VJT6Q Devices */
  /* #define STM32U535xx */   /*!< STM32U535CET6 STM32U535CEU6 STM32U535RET6 STM32U535REI6 STM32U535VET6 STM32U535VEI6 STM32U535CET6Q STM32U535CEU6Q STM32U535RET6Q STM32U535REI6Q STM32U535VET6Q STM32U535VEI6Q STM32U535NEY6Q STM32U535JEY6Q Devices */
  /* #define STM32U545xx */   /*!< STM32U545CET6 STM32U545CEU6 STM32U545RET6 STM32U545REI6 STM32U545VET6 STM32U545VEI6 STM32U545CET6Q STM32U545CEU6Q STM32U545RET6Q STM32U545REI6Q STM32U545VET6Q STM32U545VEI6Q STM32U545NEY6Q STM32U545JEY6Q Devices */
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
  * @brief CMSIS Device version number 1.3.1
  */
#define __STM32U5_CMSIS_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32U5_CMSIS_VERSION_SUB1   (0x03U) /*!< [23:16] sub1 version */
#define __STM32U5_CMSIS_VERSION_SUB2   (0x01U) /*!< [15:8]  sub2 version */
#define __STM32U5_CMSIS_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32U5_CMSIS_VERSION        ((__STM32U5_CMSIS_VERSION_MAIN << 24U)\
                                       |(__STM32U5_CMSIS_VERSION_SUB1 << 16U)\
                                       |(__STM32U5_CMSIS_VERSION_SUB2 << 8U )\
                                       |(__STM32U5_CMSIS_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(STM32U575xx)
  #include "stm32u575xx.h"
#elif defined(STM32U585xx)
  #include "stm32u585xx.h"
#elif defined(STM32U595xx)
  #include "stm32u595xx.h"
#elif defined(STM32U599xx)
  #include "stm32u599xx.h"
#elif defined(STM32U5A5xx)
  #include "stm32u5a5xx.h"
#elif defined(STM32U5A9xx)
  #include "stm32u5a9xx.h"
#elif defined(STM32U5F9xx)
  #include "stm32u5f9xx.h"
#elif defined(STM32U5G9xx)
  #include "stm32u5g9xx.h"
#elif defined(STM32U5F7xx)
  #include "stm32u5f7xx.h"
#elif defined(STM32U5G7xx)
  #include "stm32u5g7xx.h"
#elif defined(STM32U535xx)
  #include "stm32u535xx.h"
#elif defined(STM32U545xx)
  #include "stm32u545xx.h"
#else
 #error "Please select first the target STM32U5xx device used in your application (in stm32u5xx.h file)"
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

/* Use of CMSIS compiler intrinsics for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)                          \
  do {                                                                     \
    uint32_t val;                                                          \
    do {                                                                   \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);               \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define ATOMIC_SETH_BIT(REG, BIT)                            \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);       \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear one or several bits */
#define ATOMIC_CLEARH_BIT(REG, BIT)                          \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK)                         \
  do {                                                                     \
    uint16_t val;                                                          \
    do {                                                                   \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);               \
  } while(0)

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))


/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "stm32u5xx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32U5xx_H */
/**
  * @}
  */

/**
  * @}
  */




