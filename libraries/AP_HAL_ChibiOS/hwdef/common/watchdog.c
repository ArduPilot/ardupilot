/*
  independent watchdog support
 */

#include "hal.h"
#include "watchdog.h"

#ifndef IWDG_BASE
#if defined(STM32H7)
#define IWDG_BASE             0x58004800
#elif defined(STM32F7) || defined(STM32F4) || defined(STM32F1)
#define IWDG_BASE             0x40003000
#else
#error "Unknown IWDG_BASE"
#endif
#endif

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_Regs;

#define IWDGD (*(IWDG_Regs *)(IWDG_BASE))

/*
  setup the watchdog
 */
void stm32_watchdog_init(void)
{
    // setup for 1s reset
    IWDGD.KR = 0x5555;
    IWDGD.PR = 8;
    IWDGD.RLR = 0xFFF;
    IWDGD.KR = 0xCCCC;
}

/*
  pat the dog, to prevent a reset. If not called for 1s
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void)
{
    IWDGD.KR = 0xAAAA;
}
