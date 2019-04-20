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
#elif defined(STM32F1)
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
#elif defined(STM32F7) || defined(STM32F4)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x74))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
#elif defined(STM32F1)
#define WDG_RESET_STATUS (*(__IO uint32_t *)(RCC_BASE + 0x24))
#define WDG_RESET_CLEAR (1U<<24)
#define WDG_RESET_IS_IWDG (1U<<29)
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

static bool was_watchdog_reset;
static bool watchdog_enabled;
static bool boot_safety_state;

/*
  setup the watchdog
 */
void stm32_watchdog_init(void)
{
    // setup for 2s reset
    IWDGD.KR = 0x5555;
    IWDGD.PR = 16;
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
    if (WDG_RESET_STATUS & WDG_RESET_IS_IWDG) {
        was_watchdog_reset = true;
        boot_safety_state = stm32_get_backup_safety_state();
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
    return was_watchdog_reset;
}

/*
  set the safety state in backup register

  This is stored so that the safety state can be restored after a
  watchdog reset
 */
void stm32_set_backup_safety_state(bool safety_on)
{
    uint32_t v = get_rtc_backup1();
    uint32_t v2 = safety_on?(v|1):(v&~1);
    if (v != v2) {
        set_rtc_backup1(v2);
    }
}

/*
  get the safety state in backup register
  return true if safety is marked as safety on
*/
bool stm32_get_backup_safety_state(void)
{
    uint32_t v = get_rtc_backup1();
    return (v&1) != 0;
}

/*
  get the safety state in backup register from initial boot
*/
bool stm32_get_boot_backup_safety_state(void)
{
    return boot_safety_state;
}
