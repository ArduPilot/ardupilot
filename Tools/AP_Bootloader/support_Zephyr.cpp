/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* Zephyr implementation of support.h's contract - the same role support.cpp plays
 * for ChibiOS, so the protocol code above is unchanged between the two. */
#ifdef __ZEPHYR__

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "support.h"
#include "hwdef_zephyr.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <fsl_romapi.h>

// board_info is defined in AP_Bootloader.cpp (declared extern in support.h) -
// not redefined here.

static const struct device *bl_console_dev(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb_cdc_acm0))
    /* Bootloader protocol runs over USB CDC ACM, so uploader.py reaches the board
     * without a UART bridge. */
    static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(usb_cdc_acm0));
#else
    static const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
    return dev;
}

/*
  byte input with a timeout, matching cin()'s contract - returns -1 on
  timeout, else the byte read.
*/
int16_t cin(unsigned timeout_ms)
{
    const struct device *dev = bl_console_dev();
    unsigned char c;
    int64_t deadline = k_uptime_get() + timeout_ms;
    do {
        if (uart_poll_in(dev, &c) == 0) {
            return c;
        }
    } while (k_uptime_get() < deadline);
    return -1;
}

int cin_word(uint32_t *wp, unsigned timeout_ms)
{
    uint8_t *p = (uint8_t *)wp;
    for (uint8_t i = 0; i < 4; i++) {
        int16_t b = cin(timeout_ms);
        if (b < 0) {
            return -1;
        }
        p[i] = (uint8_t)b;
    }
    return 0;
}

void cout(const uint8_t *data, uint32_t len)
{
    const struct device *dev = bl_console_dev();
    for (uint32_t i = 0; i < len; i++) {
        uart_poll_out(dev, data[i]);
    }
}

void port_setbaud(uint32_t baudrate)
{
    // TODO(zephyr-bootloader, UNTESTED): PROTO_SET_BAUD support - not
    // implemented, matches this board using a fixed-baud USB CDC console.
    (void)baudrate;
}

void init_uarts(void)
{
    // TODO(zephyr-bootloader, UNTESTED): Zephyr's own board/devicetree init
    // already brings up the console UART before main() runs - nothing extra
    // needed here for the single-console case this board uses.
}

// The FCB this board's ROM reads at cold boot to configure FlexSPI - already
// present in flash, so this is a working copy and not a fresh definition.
extern "C" const uint8_t mr_vmu_rt1176_flexspi_nor_config[];
#define ROM_API_CONFIG \
    (const_cast<flexspi_nor_config_t *>( \
        reinterpret_cast<const flexspi_nor_config_t *>(mr_vmu_rt1176_flexspi_nor_config)))

/* The protocol's "sector" is the erase unit. Use the part's 64KB block rather
 * than a 4KB sector so an erase matches what the ROM API actually does. */
#define FLASH_SECTOR_SIZE 65536U
#define FLASH_PAGE_SIZE   256U

/* Erase failure diagnostics, readable over SWD - CHIP_ERASE can only report a
   single FAILURE byte, which says nothing about which unit failed or why. */
volatile uint32_t g_erase_fail_unit = 0xFFFFFFFFU;
volatile int32_t  g_erase_fail_status = 0;
volatile uint32_t g_erase_units_done = 0U;
volatile uint32_t g_prog_fail_offset = 0xFFFFFFFFU;
volatile int32_t  g_prog_fail_status = 0;
volatile uint32_t g_prog_pages_done = 0U;

/* FlexSPI instance for the ROM API is 1, not 0 - FlexSPI1 is instance 1 in the
 * ROM's numbering, and passing 0 silently targets the wrong controller. */
#define ROM_API_INSTANCE 1U

/* Working copy of the FCB in RAM: the ROM API needs a writable config, and the
 * one in flash is the image the ROM itself reads at cold boot. */
static flexspi_nor_config_t romapi_config;
static bool romapi_ready;

static void romapi_ensure_init(void)
{
    if (!romapi_ready) {
        memcpy(&romapi_config, mr_vmu_rt1176_flexspi_nor_config, sizeof(romapi_config));
        ROM_API_Init();
        /*
          The NOR driver itself must be initialised, not just the ROM API
          bookkeeping - without this the erase/program entry points reject
          every call with kStatus_InvalidArgument.
         */
        (void)ROM_FLEXSPI_NorFlash_Init(ROM_API_INSTANCE, &romapi_config);
        romapi_ready = true;
    }
}

// page at which the main firmware starts
static uint32_t flash_base_page;
// number of pages for the main firmware
static uint16_t num_pages;
// flash address of the main firmware
static const uint8_t *flash_base = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024U);

/* Initialise flash_base_page and num_pages from the ROM API's geometry. */
void flash_init(void)
{
    romapi_ensure_init();
    num_pages = (BOARD_FLASH_SIZE * 1024U) / FLASH_SECTOR_SIZE;
    flash_base_page = ((FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB) * 1024U) / FLASH_SECTOR_SIZE;
    num_pages -= (FLASH_RESERVE_END_KB * 1024U) / FLASH_SECTOR_SIZE;
}

/*
  read a word at offset relative to flash base
 */
uint32_t flash_func_read_word(uint32_t offset)
{
    return *(const volatile uint32_t *)(flash_base + offset);
}

/* This NOR is programmed a page at a time (FLASH_PAGE_SIZE) by the ROM API. */
__ramfunc bool flash_func_write_words(uint32_t offset, uint32_t *v, uint8_t n)
{
    romapi_ensure_init();
    while (n > 0) {
        const uint32_t page_offset = offset & ~(FLASH_PAGE_SIZE - 1);
        const uint32_t word_in_page = (offset - page_offset) / sizeof(uint32_t);
        const uint32_t words_per_page = FLASH_PAGE_SIZE / sizeof(uint32_t);
        const uint32_t words_this_page = MIN((uint32_t)n, words_per_page - word_in_page);

        uint8_t page[FLASH_PAGE_SIZE];
        memset(page, 0xFF, sizeof(page));
        memcpy(&page[word_in_page * sizeof(uint32_t)], v, words_this_page * sizeof(uint32_t));

        // ROM API addresses are flash-relative; flash_base is memory-mapped
        const uint32_t rom_offset = (flash_base_page * FLASH_SECTOR_SIZE) + page_offset;
        // same XIP constraint as the erase path - see flash_func_erase_sector
        const unsigned int key = irq_lock();
        status_t status = ROM_FLEXSPI_NorFlash_ProgramPage(ROM_API_INSTANCE, &romapi_config,
                                                           rom_offset, (const uint32_t *)page);
        irq_unlock(key);
        if (status != kStatus_Success) {
            g_prog_fail_offset = offset;
            g_prog_fail_status = (int32_t)status;
            return false;
        }
        g_prog_pages_done++;

        offset += words_this_page * sizeof(uint32_t);
        v += words_this_page;
        n -= words_this_page;
    }
    return true;
}

__ramfunc bool flash_func_write_word(uint32_t offset, uint32_t v)
{
    return flash_func_write_words(offset, &v, 1);
}

uint32_t flash_func_sector_size(uint32_t sector)
{
    if (sector >= num_pages-flash_base_page) {
        return 0;
    }
    return FLASH_SECTOR_SIZE;
}

bool flash_func_is_erased(uint32_t sector)
{
    const volatile uint32_t *p = (const volatile uint32_t *)(flash_base + sector * FLASH_SECTOR_SIZE);
    for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / sizeof(uint32_t); i++) {
        if (p[i] != 0xFFFFFFFF) {
            return false;
        }
    }
    return true;
}

__ramfunc bool flash_func_erase_sector(uint32_t sector, bool force_erase)
{
    if (force_erase || !flash_func_is_erased(sector)) {
        romapi_ensure_init();
        /* Range erase rather than EraseSector/EraseBlock: this part's FCB sets a block
         * size the sector calls do not match. */
        /* Interrupts must be off for the whole operation, as support.cpp does - the ROM
         * API is not re-entrant and a preempting IRQ corrupts the operation. */
        const unsigned int key = irq_lock();
        status_t status = ROM_FLEXSPI_NorFlash_Erase(ROM_API_INSTANCE, &romapi_config,
                                                     (flash_base_page + sector) * FLASH_SECTOR_SIZE,
                                                     FLASH_SECTOR_SIZE);
        irq_unlock(key);
        if (status != kStatus_Success) {
            g_erase_fail_unit = sector;
            g_erase_fail_status = (int32_t)status;
            return false;
        }
        g_erase_units_done++;
    }
    return true;
}

uint32_t flash_func_read_otp(uint32_t idx)
{
    (void)idx;
    return 0;
}

/* Serial number words, indexed by BYTE offset like support.cpp's version. */
uint32_t flash_func_read_sn(uint32_t idx)
{
    static uint8_t uid[16];
    static bool uid_valid;
    if (!uid_valid) {
        ssize_t n = hwinfo_get_device_id(uid, sizeof(uid));
        if (n < 0) {
            return 0;
        }
        if ((size_t)n < sizeof(uid)) {
            memset(&uid[n], 0, sizeof(uid) - n);
        }
        uid_valid = true;
    }
    if (idx + sizeof(uint32_t) > sizeof(uid)) {
        return 0;
    }
    uint32_t v;
    memcpy(&v, &uid[idx], sizeof(v));
    return v;
}

void flash_set_keep_unlocked(bool set)
{
    (void)set;
}

void lock_bl_port(void)
{
}

/* Buffer writes up to one flash page before programming, mirroring support.cpp. */
static struct {
    uint32_t buffer[FLASH_PAGE_SIZE / sizeof(uint32_t)];
    uint32_t page_addr;   // page-ALIGNED base address of buffer[0]
    bool dirty;
} fbuf;

#define FLASH_PAGE_WORDS (FLASH_PAGE_SIZE / sizeof(uint32_t))

/*
  flush the write buffer
 */
bool flash_write_flush(void)
{
    if (!fbuf.dirty) {
        return true;
    }
    fbuf.dirty = false;
    return flash_func_write_words(fbuf.page_addr, fbuf.buffer, FLASH_PAGE_WORDS);
}

/* Write to flash, buffering a whole page at a time. */
bool flash_write_buffer(uint32_t address, const uint32_t *v, uint8_t nwords)
{
    while (nwords > 0) {
        const uint32_t page_addr = address & ~(FLASH_PAGE_SIZE - 1);
        if (fbuf.dirty && page_addr != fbuf.page_addr) {
            if (!flash_write_flush()) {
                return false;
            }
        }
        if (!fbuf.dirty) {
            fbuf.page_addr = page_addr;
            memset(fbuf.buffer, 0xff, sizeof(fbuf.buffer));
            fbuf.dirty = true;
        }
        const uint32_t word_in_page = (address - page_addr) / sizeof(uint32_t);
        const uint32_t n = MIN((uint32_t)nwords, FLASH_PAGE_WORDS - word_in_page);
        memcpy(&fbuf.buffer[word_in_page], v, n * sizeof(uint32_t));
        address += n * sizeof(uint32_t);
        v += n;
        nwords -= n;
        if (word_in_page + n == FLASH_PAGE_WORDS) {
            if (!flash_write_flush()) {
                return false;
            }
        }
    }
    return true;
}

/*
  support.cpp returns the STM32 DBGMCU IDCODE here. RT1176 has no equivalent
  single ID register, so return the first word of the SoC UID - unique per
  board and enough for uploader.py to report something meaningful.
 */
uint32_t get_mcu_id(void)
{
    return flash_func_read_sn(0);
}

uint32_t get_mcu_desc(uint32_t len, uint8_t *buf)
{
    const char *desc = "MIMXRT1176";
    uint32_t n = strlen(desc);
    if (n > len) {
        n = len;
    }
    memcpy(buf, desc, n);
    return n;
}

/* Fast-reboot signature in an SNVS LP General Purpose Register: it survives a
 * warm reset, which is what lets reboot-to-bootloader work without BOOT0. */
#define SNVS_LPCR           (*(volatile uint32_t *)(0x40C90000u + 0x38u))
#define SNVS_LPGPR3         (*(volatile uint32_t *)(0x40C90000u + 0x10Cu))
#define SNVS_LPCR_GPR_Z_DIS 0x1000000u

uint32_t board_get_rtc_signature(void)
{
    return SNVS_LPGPR3;
}

void board_set_rtc_signature(uint32_t sig)
{
    /* Retention must stay enabled or the value is cleared by a later security
       event - set it before writing (same as the app-side writer). */
    SNVS_LPCR |= SNVS_LPCR_GPR_Z_DIS;
    SNVS_LPGPR3 = sig;
}

// Fast-reboot accessors used by AP_Bootloader.cpp, on top of the SNVS
// signature above. Mirrors AP_HAL_ChibiOS/hwdef/common/stm32_util.c.
enum rtc_boot_magic check_fast_reboot(void)
{
    return (enum rtc_boot_magic)board_get_rtc_signature();
}

void set_fast_reboot(enum rtc_boot_magic v)
{
    if (check_fast_reboot() != v) {
        board_set_rtc_signature((uint32_t)v);
    }
}

/* Bootloader status LED. support.cpp drives palSetLine(); this uses the Zephyr
 * GPIO API against the DTS-declared LED. */
static const struct gpio_dt_spec bl_led =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led_amber), gpios, {0});

static bool bl_led_ready(void)
{
    static bool checked, ready;
    if (!checked) {
        checked = true;
        ready = gpio_is_ready_dt(&bl_led) &&
                gpio_pin_configure_dt(&bl_led, GPIO_OUTPUT_INACTIVE) == 0;
    }
    return ready;
}

void led_on(unsigned led)
{
    (void)led;
    if (bl_led_ready()) {
        (void)gpio_pin_set_dt(&bl_led, 1);
    }
}

void led_off(unsigned led)
{
    (void)led;
    if (bl_led_ready()) {
        (void)gpio_pin_set_dt(&bl_led, 0);
    }
}

void led_toggle(unsigned led)
{
    (void)led;
    if (bl_led_ready()) {
        (void)gpio_pin_toggle_dt(&bl_led);
    }
}

void led_pulses(uint8_t npulses)
{
    (void)npulses;
}

void thread_sleep_ms(uint32_t ms)
{
    k_msleep(ms);
}

void thread_sleep_us(uint32_t us)
{
    k_usleep(us);
}

void custom_startup(void)
{
}

extern "C" {
void uprintf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        cout((const uint8_t *)buf, MIN((uint32_t)n, sizeof(buf) - 1));
    }
}
}

#endif // __ZEPHYR__
