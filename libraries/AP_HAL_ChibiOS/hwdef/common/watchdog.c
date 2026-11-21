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

#if STM32_WDG_TIMEOUT_MS > 4096 || STM32_WDG_TIMEOUT_MS < 20
#error "Watchdog timeout out of range"
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
  lockup detection using SysTick at a priority above all peripheral
  interrupts and the kernel lock, so we get a crash dump even if
  threads and the ChibiOS timer are starved by an interrupt storm
 */
#if AP_WATCHDOG_LOCKUP_DETECT_ENABLED
#define LOCKUP_DETECT_HZ 100
#define LOCKUP_DETECT_PRIORITY 1
// trigger the crash dump this long before the nominal watchdog timeout
#define LOCKUP_DETECT_MS (STM32_WDG_TIMEOUT_MS - 250)
#define LOCKUP_DETECT_TICKS ((LOCKUP_DETECT_MS * LOCKUP_DETECT_HZ + 999) / 1000)
#if defined(STM32_CORE_CK)
#define LOCKUP_SYSTICK_CK STM32_CORE_CK
#else
#define LOCKUP_SYSTICK_CK STM32_HCLK
#endif
static volatile uint32_t lockup_heartbeat;
static volatile bool lockup_paused;
#endif

/*
  setup the watchdog
 */
void stm32_watchdog_init(void)
{
    // setup the watchdog timeout
    // t = 4 * 2^PR * (RLR+1) / 32KHz
    IWDGD.KR = 0x5555;
    IWDGD.PR = 3; // changing this would change the definition of STM32_WDG_TIMEOUT_MS
    IWDGD.RLR = STM32_WDG_TIMEOUT_MS - 1;
    IWDGD.KR = 0xCCCC;
    watchdog_enabled = true;
}

/*
  return true if the watchdog has been started
 */
bool stm32_watchdog_enabled(void)
{
    return watchdog_enabled;
}

/*
  pat the dog, to prevent a reset. If not called for STM32_WDG_TIMEOUT_MS
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void)
{
    if (watchdog_enabled) {
        IWDGD.KR = 0xAAAA;
#if AP_WATCHDOG_LOCKUP_DETECT_ENABLED
        lockup_heartbeat++;
#endif
    }
}

#if AP_WATCHDOG_LOCKUP_DETECT_ENABLED
/*
  SysTick handler. This runs above the kernel priority so must not
  use any ChibiOS APIs
 */
void SysTick_Handler(void);
void SysTick_Handler(void)
{
    static uint32_t last_heartbeat;
    static uint16_t stall_ticks;
    const uint32_t heartbeat = lockup_heartbeat;
    if (heartbeat != last_heartbeat || lockup_paused) {
        last_heartbeat = heartbeat;
        stall_ticks = 0;
        return;
    }
    if (++stall_ticks >= LOCKUP_DETECT_TICKS) {
        // escalates to HardFault and a crash dump
        SysTick->CTRL = 0;
        __builtin_trap();
    }
}

/*
  pause lockup detection, for operations that legitimately block the
  CPU for a long time such as erasing the bootloader flash sector
 */
void stm32_lockup_detect_pause(bool pause)
{
    lockup_paused = pause;
}

/*
  start lockup detection, called once the main loop is running
 */
void stm32_lockup_detect_start(void)
{
    if (!watchdog_enabled) {
        return;
    }
    SysTick->CTRL = 0;
    SysTick->LOAD = (LOCKUP_SYSTICK_CK / LOCKUP_DETECT_HZ) - 1;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    nvicSetSystemHandlerPriority(HANDLER_SYSTICK, LOCKUP_DETECT_PRIORITY);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}
#endif // AP_WATCHDOG_LOCKUP_DETECT_ENABLED

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
