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

#define WDG_SAFETY_BIT  0x01
#define WDG_ARMED_BIT   0x02

#define BKP_IDX_FLAGS   0x01
#define BKP_IDX_HOME    0x02
#define BKP_IDX_RPY     0x05

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
static uint32_t boot_backup_state[8];

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
    if (WDG_RESET_STATUS & WDG_RESET_IS_IWDG) {
        was_watchdog_reset = true;
        uint8_t i;
        for (i=0; i<sizeof(boot_backup_state)/sizeof(boot_backup_state[0]); i++) {
            boot_backup_state[i] = get_rtc_backup(i);
        }
    }
}

/*
  clear reason code for reset
 */
void stm32_watchdog_clear_reason(void)
{
    WDG_RESET_STATUS = WDG_RESET_CLEAR;
    uint8_t i;
    for (i=1; i<sizeof(boot_backup_state)/sizeof(boot_backup_state[0]); i++) {
        set_rtc_backup(i, 0);
    }
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
    uint32_t v = get_rtc_backup(BKP_IDX_FLAGS);
    uint32_t v2 = safety_on?(v | WDG_SAFETY_BIT):(v & ~WDG_SAFETY_BIT);
    if (v != v2) {
        set_rtc_backup(BKP_IDX_FLAGS, v2);
    }
}

/*
  get the safety state in backup register from initial boot
*/
bool stm32_get_boot_backup_safety_state(void)
{
    return (boot_backup_state[BKP_IDX_FLAGS] & WDG_SAFETY_BIT) != 0;
}

/*
  set the armed state in backup register

  This is stored so that the armed state can be restored after a
  watchdog reset
 */
void stm32_set_backup_armed(bool armed)
{
    uint32_t v = get_rtc_backup(BKP_IDX_FLAGS);
    uint32_t v2 = armed?(v | WDG_ARMED_BIT): (v & ~WDG_ARMED_BIT);
    if (v != v2) {
        set_rtc_backup(BKP_IDX_FLAGS, v2);
    }
}

/*
  get the armed state in backup register from initial boot
*/
bool stm32_get_boot_backup_armed(void)
{
    return (boot_backup_state[BKP_IDX_FLAGS] & WDG_ARMED_BIT) != 0;
}

/*
  set home state in backup
 */
void stm32_set_backup_home(int32_t lat, int32_t lon, int32_t alt_cm)
{
    set_rtc_backup(BKP_IDX_HOME, (uint32_t)lat);
    set_rtc_backup(BKP_IDX_HOME+1, (uint32_t)lon);
    set_rtc_backup(BKP_IDX_HOME+2, (uint32_t)alt_cm);
}

/*
  get home state from backup
 */
void stm32_get_backup_home(int32_t *lat, int32_t *lon, int32_t *alt_cm)
{
    *lat = (int32_t)boot_backup_state[BKP_IDX_HOME];
    *lon = (int32_t)boot_backup_state[BKP_IDX_HOME+1];
    *alt_cm = (int32_t)boot_backup_state[BKP_IDX_HOME+2];
}

/*
  set attitude in backup
 */
void stm32_set_attitude(int32_t roll_cd, int32_t pitch_cd, int32_t yaw_cd)
{
    set_rtc_backup(BKP_IDX_RPY, (uint32_t)roll_cd);
    set_rtc_backup(BKP_IDX_RPY+1, (uint32_t)pitch_cd);
    set_rtc_backup(BKP_IDX_RPY+2, (uint32_t)yaw_cd);
}

/*
  get attitude from backup
 */
void stm32_get_attitude(int32_t *roll_cd, int32_t *pitch_cd, int32_t *yaw_cd)
{
    *roll_cd = (int32_t)boot_backup_state[BKP_IDX_RPY];
    *pitch_cd = (int32_t)boot_backup_state[BKP_IDX_RPY+1];
    *yaw_cd = (int32_t)boot_backup_state[BKP_IDX_RPY+2];
}
