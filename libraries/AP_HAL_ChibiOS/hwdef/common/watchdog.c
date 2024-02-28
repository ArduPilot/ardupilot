/*
  independent watchdog support
 */

#include "hal.h"
#include "watchdog.h"
#include "stm32_util.h"

#ifndef IWDG_BASE
#if defined(STM32H7)
#define IWDG_BASE             0x58004800
#elif defined(STM32F7) || defined(STM32F4)
#define IWDG_BASE             0x40003000
#elif defined(STM32F1) || defined(STM32F3)
#define IWDG_BASE             0x40003000
#else
#error "Unsupported IWDG MCU config"
#endif
#endif

#ifndef RCC_BASE
#error "Unsupported IWDG RCC MCU config"
#endif

/*
  defines for working out if the reset was from the watchdog
 */
#if defined(STM32H7)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0xD0))
#define WDG_RESET_CLEAR (1U<<16)
#define WDG_RESET_IS_IWDG (1U<<26)
#define WDG_RESET_IS_SFT (1U<<24)
#elif defined(STM32F7) || defined(STM32F4)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x74))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
#define WDG_RESET_IS_SFT (1U<<28)
#elif defined(STM32F1) || defined(STM32F3)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x24))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
#define WDG_RESET_IS_SFT (1U<<28)
#elif defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x94))
#define WDG_RESET_CLEAR (1U<<23)
#define WDG_RESET_IS_IWDG (1U<<29)
#define WDG_RESET_IS_SFT (1U<<28)
#else
#error "Unsupported IWDG MCU config"
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

static uint32_t reset_reason;
static bool watchdog_enabled;

/*
  setup the watchdog
 */
void stm32_watchdog_init(void)
{
    // setup for 2s reset
    IWDGD.KR = 0x5555;
    IWDGD.PR = 2; // div16
    IWDGD.RLR = 0xFFF;
    IWDGD.KR = 0xCCCC;
    watchdog_enabled = true;
}

/*
  pat the dog, to prevent a reset. If not called for 1s
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void)
{
    if (watchdog_enabled) {
        IWDGD.KR = 0xAAAA;
    }
}

/*
  save reason code for reset
 */
void stm32_watchdog_save_reason(void)
{
    if (reset_reason == 0) {
        reset_reason = WDG_RESET_STATUS;
    }
}

/*
  clear reason code for reset
 */
void stm32_watchdog_clear_reason(void)
{
    WDG_RESET_STATUS = WDG_RESET_CLEAR;
}

/*
  return true if reboot was from a watchdog reset
 */
bool stm32_was_watchdog_reset(void)
{
    stm32_watchdog_save_reason();
    return (reset_reason & WDG_RESET_IS_IWDG) != 0;
}

/*
  return true if reboot was from a software reset
 */
bool stm32_was_software_reset(void)
{
    stm32_watchdog_save_reason();
    return (reset_reason & WDG_RESET_IS_SFT) != 0;
}

/*
  save persistent watchdog data
 */
void stm32_watchdog_save(const uint32_t *data, uint32_t nwords)
{
    set_rtc_backup(1, data, nwords);
}

/*
  load persistent watchdog data
 */
void stm32_watchdog_load(uint32_t *data, uint32_t nwords)
{
    get_rtc_backup(1, data, nwords);
}
