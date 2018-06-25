/*
  bootloader support functions
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include "support.h"
#include "mcu_f4.h"
#include "mcu_f7.h"

int16_t cin(unsigned timeout_ms)
{
    uint8_t b = 0;
    if (chnReadTimeout(&SDU1, &b, 1, MS2ST(timeout_ms)) != 1) {
        chThdSleepMicroseconds(100);
        return -1;
    }
    return b;
}

int cin_word(uint32_t *wp, unsigned timeout_ms)
{
    if (chnReadTimeout(&SDU1, (uint8_t *)wp, 4, MS2ST(timeout_ms)) != 4) {
        chThdSleepMicroseconds(100);
        return -1;
    }
    return 0;
}


void cout(uint8_t *data, uint32_t len)
{
    chnWriteTimeout(&SDU1, data, len, MS2ST(100));
}

void cfini(void)
{
    sduStop(&SDU1);
}

static uint32_t flash_base_page;
static uint8_t num_pages;
static const uint8_t *flash_base = (const uint8_t *)(0x08000000 + FLASH_BOOTLOADER_LOAD_KB*1024U);

/*
  initialise flash_base_page and num_pages
 */
void flash_init(void)
{
    uint32_t reserved = 0;
    num_pages = stm32_flash_getnumpages();
    while (reserved < FLASH_BOOTLOADER_LOAD_KB * 1024U &&
           flash_base_page < num_pages) {
        reserved += stm32_flash_getpagesize(flash_base_page);
        flash_base_page++;
    }
}

void flash_set_keep_unlocked(bool set)
{
    stm32_flash_keep_unlocked(set);
}

/*
  read a word at offset relative to FLASH_BOOTLOADER_LOAD_KB
 */
uint32_t flash_func_read_word(uint32_t offset)
{
    return *(const uint32_t *)(flash_base + offset);
}

void flash_func_write_word(uint32_t offset, uint32_t v)
{
    stm32_flash_write(uint32_t(flash_base+offset), &v, sizeof(v));
}

uint32_t flash_func_sector_size(uint32_t sector)
{
    if (sector >= flash_base_page+num_pages) {
        return 0;
    }
    return stm32_flash_getpagesize(flash_base_page+sector);
}

void flash_func_erase_sector(uint32_t sector)
{
    if (!stm32_flash_ispageerased(flash_base_page+sector)) {
        stm32_flash_erasepage(flash_base_page+sector);
    }
}

// read one-time programmable memory
uint32_t flash_func_read_otp(uint32_t idx)
{
    if (idx & 3) {
        return 0;
    }

    if (idx > OTP_SIZE) {
        return 0;
    }

    return *(uint32_t *)(idx + OTP_BASE);
}

// read chip serial number
uint32_t flash_func_read_sn(uint32_t idx)
{
    return *(uint32_t *)(UDID_START + idx);
}

uint32_t get_mcu_id(void)
{
    return *(uint32_t *)DBGMCU_BASE;
}

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

uint32_t get_mcu_desc(uint32_t max, uint8_t *revstr)
{
    uint32_t idcode = (*(uint32_t *)DBGMCU_BASE);
    int32_t mcuid = idcode & DEVID_MASK;
    uint16_t revid = ((idcode & REVID_MASK) >> 16);

    mcu_des_t des = mcu_descriptions[STM32_UNKNOWN];

    for (int i = 0; i < ARRAY_SIZE_SIMPLE(mcu_descriptions); i++) {
        if (mcuid == mcu_descriptions[i].mcuid) {
            des = mcu_descriptions[i];
            break;
        }
    }

    for (int i = 0; i < ARRAY_SIZE_SIMPLE(silicon_revs); i++) {
        if (silicon_revs[i].revid == revid) {
            des.rev = silicon_revs[i].rev;
        }
    }

    uint8_t *endp = &revstr[max - 1];
    uint8_t *strp = revstr;

    while (strp < endp && *des.desc) {
        *strp++ = *des.desc++;
    }

    if (strp < endp) {
        *strp++ = ',';
    }

    if (strp < endp) {
        *strp++ = des.rev;
    }

    return  strp - revstr;
}

/*
  see if we should limit flash to 1M on devices with older revisions
 */
bool check_limit_flash_1M(void)
{
    uint32_t idcode = (*(uint32_t *)DBGMCU_BASE);
    uint16_t revid = ((idcode & REVID_MASK) >> 16);

    for (int i = 0; i < ARRAY_SIZE_SIMPLE(silicon_revs); i++) {
        if (silicon_revs[i].revid == revid) {
            return silicon_revs[i].limit_flash_size_1M;
        }
    }
    return false;
}

void led_on(unsigned led)
{
#ifdef HAL_GPIO_PIN_LED_BOOTLOADER
    if (led == LED_BOOTLOADER) {
        palWriteLine(HAL_GPIO_PIN_LED_BOOTLOADER, HAL_LED_ON);
    }
#endif
#ifdef HAL_GPIO_PIN_LED_ACTIVITY
    if (led == LED_ACTIVITY) {
        palWriteLine(HAL_GPIO_PIN_LED_ACTIVITY, HAL_LED_ON);
    }
#endif
}

void led_off(unsigned led)
{
#ifdef HAL_GPIO_PIN_LED_BOOTLOADER
    if (led == LED_BOOTLOADER) {
        palWriteLine(HAL_GPIO_PIN_LED_BOOTLOADER, !HAL_LED_ON);
    }
#endif
#ifdef HAL_GPIO_PIN_LED_ACTIVITY
    if (led == LED_ACTIVITY) {
        palWriteLine(HAL_GPIO_PIN_LED_ACTIVITY, !HAL_LED_ON);
    }
#endif
}

void led_toggle(unsigned led)
{
#ifdef HAL_GPIO_PIN_LED_BOOTLOADER
    if (led == LED_BOOTLOADER) {
        palToggleLine(HAL_GPIO_PIN_LED_BOOTLOADER);
    }
#endif
#ifdef HAL_GPIO_PIN_LED_ACTIVITY
    if (led == LED_ACTIVITY) {
        palToggleLine(HAL_GPIO_PIN_LED_ACTIVITY);
    }
#endif
}

extern "C" {
    int vsnprintf(char *str, size_t size, const char *fmt, va_list ap);
}

// printf to USB for debugging
void uprintf(const char *fmt, ...)
{
    char msg[200];
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    chnWriteTimeout(&SDU1, (const uint8_t *)msg, n, MS2ST(100));
}

// generate a pulse sequence forever, for debugging
void led_pulses(uint8_t npulses)
{
    led_off(LED_BOOTLOADER);
    while (true) {
        for (uint8_t i=0; i<npulses; i++) {
            led_on(LED_BOOTLOADER);
            chThdSleepMilliseconds(200);
            led_off(LED_BOOTLOADER);
            chThdSleepMilliseconds(200);
        }
        chThdSleepMilliseconds(2000);
    }
}

//simple variant of std c function to reduce used flash space
void *memcpy(void *dest, const void *src, size_t n)
{
    uint8_t *tdest = (uint8_t *)dest;
    uint8_t *tsrc = (uint8_t *)src;
    for (int i=0; i<n; i++) {
        tdest[i] = tsrc[i];
    }
    return dest;
}

//simple variant of std c function to reduce used flash space
int strcmp(const char *s1, const char *s2)
{
    while ((*s1 != 0) && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return (*s1 - *s2);
}

void lock_bl_port(void)
{
}
