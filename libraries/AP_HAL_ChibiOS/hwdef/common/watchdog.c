/*
  independent watchdog support
 */

  #include "hal.h"
  #include "watchdog.h"

  #if defined(STM32_HW)

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
    define for controlling how long the watchdog is set for.
  */
  #ifndef STM32_WDG_TIMEOUT_MS
  #define STM32_WDG_TIMEOUT_MS 2048
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
    pat the dog, to prevent a reset. If not called for STM32_WDG_TIMEOUT_MS
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

  #endif // STM32_HW

#if defined(RP2350)

#include "hal.h"

/*
  default watchdog timeout in milliseconds for RP2350
*/
#ifndef RP2350_WDG_TIMEOUT_MS
#define RP2350_WDG_TIMEOUT_MS 2000U
#endif

static const WDGConfig rp2350_wdg_cfg = {
    .rlr = RP2350_WDG_TIMEOUT_MS,
};

static bool rp2350_watchdog_enabled;

/*
  SCRATCH[6] dual-purpose register for WD detection on RP2350.
  On RP2350, the WD PSM reset (ChibiOS WDSEL=ALL_BITS) resets the WATCHDOG
  peripheral itself, so WATCHDOG->REASON is always 0 after any WD-triggered
  reset.  SCRATCH registers survive PSM-level resets (confirmed by hardware
  test: SCRATCH preserved across WD-triggered PSM reset, 2026-04-10).
  We therefore use SCRATCH[6] as the sole WD detection mechanism:
    RP2350_WDG_ARMED_CANARY ('WDOG'): written on every rp2350_watchdog_pat()
      call (ArduPilot app-level only; AP_Bootloader calls wdgReset() directly
      and never writes this canary).  When WD fires and PSM-resets the board,
      SCRATCH[6] still holds the canary, allowing detection at next boot.
    RP2350_WDG_REASON_CLEARED: written by rp2350_watchdog_clear_reason() to
      prevent re-detection after the reason has been consumed.
  Detection is cached in RAM at rp2350_watchdog_init() time (before any pat
  can overwrite SCRATCH[6]) and returned by rp2350_was_watchdog_reset().
  Explicit Scheduler::reboot() writes RP2350_RESET_DIAG_SCHEDULER_REBOOT to
  SCRATCH[7], enabling false-positive suppression for software reboots.
*/
#define RP2350_WDG_ARMED_CANARY   0x57444F47U  /* 'WDOG' - app is petting WD */
#define RP2350_WDG_REASON_CLEARED 0xDEADC0DEU  /* reason consumed, do not re-report */

/* cached result of WD-reset detection, set once at rp2350_watchdog_init() time */
static bool rp2350_wd_reset_detected;

/*
  initialise and start the RP2350 watchdog.
  Must be called before rp2350_watchdog_pat(); caches the WD-reset detection
  result in rp2350_wd_reset_detected so that later rp2350_was_watchdog_reset()
  calls return a consistent answer even after rp2350_watchdog_pat() has
  re-armed the canary in SCRATCH[6].
*/
void rp2350_watchdog_init(void)
{
/*
 * Cache whether the last reset was a WD reset BEFORE starting the WD and BEFORE any rp2350_watchdog_pat() overwrites SCRATCH[6].
 * Conditions for a genuine WD reset: 1.
 * 2.
 * not an explicit software reboot via Scheduler::reboot(), which writes 'SCHD' to SCRATCH[7] before calling NVIC_SystemReset()
 */
    rp2350_wd_reset_detected =
        (WATCHDOG->SCRATCH[6] == RP2350_WDG_ARMED_CANARY) &&
        (WATCHDOG->SCRATCH[RP2350_RESET_DIAG_SCRATCH_IDX] != RP2350_RESET_DIAG_SCHEDULER_REBOOT);

    wdgStart(&WDGD1, &rp2350_wdg_cfg);
    rp2350_watchdog_enabled = true;
}

/*
  reload the watchdog counter to prevent a reset.
  Also writes the armed canary to SCRATCH[6] on every call so that any
  subsequent WD-triggered PSM reset can be detected at next boot.
  (rp2350_watchdog_init() caches the detection result before any pat can
  overwrite SCRATCH[6], so this does not cause false positives.)
*/
void rp2350_watchdog_pat(void)
{
    if (rp2350_watchdog_enabled) {
        WATCHDOG->SCRATCH[6] = RP2350_WDG_ARMED_CANARY;
        wdgReset(&WDGD1);
    }
}

/*
  return true if the last reboot was caused by the watchdog timer AND the
  reason has not already been consumed by rp2350_watchdog_clear_reason().
  Returns the cached rp2350_wd_reset_detected flag set at init() time.
*/
bool rp2350_was_watchdog_reset(void)
{
    return rp2350_wd_reset_detected;
}

/*
  mark the watchdog reset reason as consumed so it is not re-reported.
  Writes the consumed sentinel to SCRATCH[6] and clears the RAM cache.
  The next rp2350_watchdog_pat() will re-arm the canary for future WD resets.
*/
void rp2350_watchdog_clear_reason(void)
{
    WATCHDOG->SCRATCH[6] = RP2350_WDG_REASON_CLEARED;
    rp2350_wd_reset_detected = false;
}

/* no persistent save needed: REASON is a hardware register, always valid */
void rp2350_watchdog_save_reason(void)
{
}

/*
 * RP2350 persistent data save/load across WD resets using a no-init SRAM buffer.
 * SRAM on RP2350 is in the always-on power domain: NOT reset by the PSM watchdog reset.
 * The buffer is in ".ram0" (after __ram0_noinit__) so ChibiOS CRT0 doesn't zero it.
 * AP_Bootloader only zeros its own BSS, leaving the app noinit region intact.
 * 32 words gives comfortable headroom for future growth.
 */
#define RP2350_WD_PERSIST_MAGIC     0x5750444fU  /* 'WPDO' little-endian */
#define RP2350_WD_PERSIST_MAX_WORDS 32U

typedef struct {
    uint32_t magic;                              /* RP2350_WD_PERSIST_MAGIC when valid */
    uint32_t nwords;                             /* number of valid words in data[] */
    uint32_t data[RP2350_WD_PERSIST_MAX_WORDS];  /* copy of HAL::Util::PersistentData */
} rp2350_wd_persist_t;

/*
 * Placed in ".ram0" (no-init section): CRT0 doesn't touch anything past __ram0_noinit__,
 * so the struct retains its value across WD-triggered PSM resets.
 */
static rp2350_wd_persist_t wd_persist_buf __attribute__((section(".ram0")));

void rp2350_watchdog_save(const uint32_t *data, uint32_t nwords)
{
    /* Cap at our buffer limit to prevent overflow. */
    if (nwords > RP2350_WD_PERSIST_MAX_WORDS) {
        nwords = RP2350_WD_PERSIST_MAX_WORDS;
    }

    wd_persist_buf.nwords = nwords;
    for (uint32_t i = 0U; i < nwords; i++) {
        wd_persist_buf.data[i] = data[i];
    }
    /* Write magic last so a partial write leaves an invalid header. */
    wd_persist_buf.magic = RP2350_WD_PERSIST_MAGIC;
}

void rp2350_watchdog_load(uint32_t *data, uint32_t nwords)
{
    /* Reject invalid header (power-on-reset, no prior save, or partial write). */
    if (wd_persist_buf.magic != RP2350_WD_PERSIST_MAGIC) {
        return;
    }

    uint32_t saved = wd_persist_buf.nwords;
    if (saved > RP2350_WD_PERSIST_MAX_WORDS) {
        saved = RP2350_WD_PERSIST_MAX_WORDS;  /* guard against corrupt nwords */
    }
    const uint32_t copy = (saved < nwords) ? saved : nwords;

    for (uint32_t i = 0U; i < copy; i++) {
        data[i] = wd_persist_buf.data[i];
    }
    /* Zero any words the caller wants that we did not save. */
    for (uint32_t i = copy; i < nwords; i++) {
        data[i] = 0U;
    }

    /* Invalidate the buffer so a cold POR (even without re-arm by save())
     * does not replay stale data on the next boot. */
    wd_persist_buf.magic = 0U;
}

bool rp2350_was_software_reset(void)
{
    /* RP2350 WATCHDOG REASON.FORCE bit indicates a forced (software) reset */
    return (WATCHDOG->REASON & WATCHDOG_REASON_FORCE) != 0U;
}

#endif // RP2350
