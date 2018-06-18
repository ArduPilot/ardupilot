/*
  bootloader support functions
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>
#include "support.h"

int16_t cin(unsigned timeout_ms)
{
    uint8_t b = 0;
    if (chnReadTimeout(&SDU1, &b, 1, MS2ST(timeout_ms)) != 1) {
        chThdSleepMilliseconds(1);
        return -1;
    }
    return b;
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
    stm32_flash_erasepage(flash_base_page+sector);    
}

uint32_t flash_func_read_otp(uint32_t idx)
{
    return 0;
}

uint32_t flash_func_read_sn(uint32_t idx)
{
    return 0;
}

uint32_t get_mcu_id(void)
{
    return 0;
}

uint32_t get_mcu_desc(uint32_t len, uint8_t *buf)
{
    buf[0] = 'A';
    return 1;
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
    for(int i=0; i<n; i++) {
        tdest[i] = tsrc[i];
    }
    return dest;
}

//simple variant of std c function to reduce used flash space
int strcmp(const char *s1, const char *s2)
{
    while( (*s1 != 0) && (*s1 == *s2) ) {
        s1++;
        s2++;
    }
    return (*s1 - *s2);
}
