#pragma once

/* Hand-maintained hwdef macros for building AP_Bootloader against Zephyr, in
 * place of the generated ChibiOS hwdef.h. */

// i.MX RT1176 has no separate internal program flash - it always boots XIP from
// external NOR, so the bootloader and app share one flash device.
#define BOARD_FLASH_SIZE            (4 * 1024)
#define FLASH_BOOTLOADER_LOAD_KB    128
#define FLASH_RESERVE_END_KB        0
#define APP_START_OFFSET_KB         0

// bl_protocol.cpp's default APP_START_ADDRESS formula
// (FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB+APP_START_OFFSET_KB)*1024)
// needs FLASH_LOAD_ADDRESS - the FlexSPI XIP base.
#define FLASH_LOAD_ADDRESS 0x30000000

// Binary compatibility with the PX4 v6xrt bootloader in both directions: either
// bootloader can flash either app, so a board is never stranded.
#define APP_VECTOR_OFFSET 0x2000U

// zephyr_hwdef.py parses APJ_BOARD_ID from hwdef-bl.dat but (unlike
// chibios_hwdef.py) doesn't currently emit it into the generated hwdef.h -
// duplicated here so it exists as a real macro. Keep in sync with
// hwdef-bl.dat's own APJ_BOARD_ID line.
#ifndef APJ_BOARD_ID
#define APJ_BOARD_ID 35
#endif

/* Fast-reboot (reboot to bootloader) support, via the SNVS signature register. */
#define AP_FASTBOOT_ENABLED 1

// Same values as AP_HAL_ChibiOS/hwdef/common/stm32_util.h's rtc_boot_magic, so
// an app that writes the ChibiOS magic is understood by this bootloader.
enum rtc_boot_magic {
    RTC_BOOT_OFF   = 0,
    RTC_BOOT_HOLD  = 0xb0070001,
    RTC_BOOT_FAST  = 0xb0070002,
    RTC_BOOT_CANBL = 0xb0080000, // ORd with 8-bit local node ID
    RTC_BOOT_FWOK  = 0xb0093a26  // FW ran 30s (set by app, unused here yet)
};

enum rtc_boot_magic check_fast_reboot(void);
void set_fast_reboot(enum rtc_boot_magic v);

// The Zephyr bootloader does not yet detect a hardware watchdog reset.
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef HAL_USE_CAN
#define HAL_USE_CAN FALSE
#endif
// HAL_NUM_CAN_IFACES deliberately left undefined (evaluates to 0).

static inline bool stm32_was_watchdog_reset(void) { return false; }
static inline void stm32_watchdog_clear_reason(void) {}

// MCUBoot-compatible overwrite-only A/B (mcuboot_ab.cpp): a valid imgtool
// image staged in slot 1 (app-region offset 2 MB = 0x30220000) is copied over
// slot 0 before the normal boot flow. The SHA256 TLV is checked, so the copy
// is integrity-verified but not authenticated; the ED25519 TLV is not
// enforced. See libraries/AP_HAL_Zephyr/BOOTLOADER_SECURITY.md.
#define AP_BOOTLOADER_MCUBOOT_AB 1
