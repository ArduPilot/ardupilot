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
#include <AP_Math/AP_Math.h>
#include "support.h"
#include "mcu_f1.h"
#include "mcu_f3.h"
#include "mcu_f4.h"
#include "mcu_f7.h"
#include "mcu_h7.h"
#include "mcu_g4.h"

// optional uprintf() code for debug
// #define BOOTLOADER_DEBUG SD1

#if defined(BOOTLOADER_DEV_LIST)
static BaseChannel *uarts[] = { BOOTLOADER_DEV_LIST };
#if HAL_USE_SERIAL == TRUE
static SerialConfig sercfg;
#endif
static int8_t locked_uart = -1;
static uint8_t last_uart;

#ifndef BOOTLOADER_BAUDRATE
#define BOOTLOADER_BAUDRATE 115200
#endif

// #pragma GCC optimize("O0")

static bool cin_data(uint8_t *data, uint8_t len, unsigned timeout_ms)
{
    for (uint8_t i=0; i<ARRAY_SIZE(uarts); i++) {
        if (locked_uart == -1 || locked_uart == i) {
            if (chnReadTimeout(uarts[i], data, len, chTimeMS2I(timeout_ms)) == len) {
                last_uart = i;
                return true;
            }
        }
    }
    chThdSleepMicroseconds(500);
    return false;
}

int16_t cin(unsigned timeout_ms)
{
    uint8_t b = 0;
    if (cin_data(&b, 1, timeout_ms)) {
        return b;
    }
    return -1;
}

int cin_word(uint32_t *wp, unsigned timeout_ms)
{
    if (cin_data((uint8_t *)wp, 4, timeout_ms)) {
        return 0;
    }
    return -1;
}


void cout(uint8_t *data, uint32_t len)
{
    chnWriteTimeout(uarts[last_uart], data, len, chTimeMS2I(100));
}
#endif // BOOTLOADER_DEV_LIST

static uint32_t flash_base_page;
static uint16_t num_pages;
static const uint8_t *flash_base = (const uint8_t *)(0x08000000 + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024U);

/*
  initialise flash_base_page and num_pages
 */
void flash_init(void)
{
    uint32_t reserved = 0;
    num_pages = stm32_flash_getnumpages();
    /*
      advance flash_base_page to account for (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)
     */
    while (reserved < (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB) * 1024U &&
           flash_base_page < num_pages) {
        reserved += stm32_flash_getpagesize(flash_base_page);
        flash_base_page++;
    }
    /*
      reduce num_pages to account for FLASH_RESERVE_END_KB
     */
    reserved = 0;
    while (reserved < FLASH_RESERVE_END_KB * 1024U) {
        reserved += stm32_flash_getpagesize(num_pages-1);
        num_pages--;
    }
}

void flash_set_keep_unlocked(bool set)
{
    stm32_flash_keep_unlocked(set);
}

/*
  read a word at offset relative to flash base
 */
uint32_t flash_func_read_word(uint32_t offset)
{
    return *(const uint32_t *)(flash_base + offset);
}

bool flash_func_write_word(uint32_t offset, uint32_t v)
{
    return stm32_flash_write(uint32_t(flash_base+offset), &v, sizeof(v));
}

bool flash_func_write_words(uint32_t offset, uint32_t *v, uint8_t n)
{
    return stm32_flash_write(uint32_t(flash_base+offset), v, n*sizeof(*v));
}

uint32_t flash_func_sector_size(uint32_t sector)
{
    if (sector >= num_pages-flash_base_page) {
        return 0;
    }
    return stm32_flash_getpagesize(flash_base_page+sector);
}

bool flash_func_erase_sector(uint32_t sector)
{
    if (!stm32_flash_ispageerased(flash_base_page+sector)) {
        return stm32_flash_erasepage(flash_base_page+sector);
    }
    return true;
}

// read one-time programmable memory
uint32_t flash_func_read_otp(uint32_t idx)
{
#ifndef OTP_SIZE
    return 0;
#else
    if (idx & 3) {
        return 0;
    }

    if (idx > OTP_SIZE) {
        return 0;
    }

    return *(uint32_t *)(idx + OTP_BASE);
#endif
}

// read chip serial number
uint32_t flash_func_read_sn(uint32_t idx)
{
    return *(uint32_t *)(UDID_START + idx);
}

/*
  we use a write buffer for flashing, both for efficiency and to
  ensure that we only ever do 32 byte aligned writes on STM32H7. If
  you attempt to do writes on a H7 of less than 32 bytes or not
  aligned then the flash can end up in a CRC error state, which can
  generate a hardware fault (a double ECC error) on flash read, even
  after a power cycle
 */
static struct {
    uint32_t buffer[8];
    uint32_t address;
    uint8_t n;
} fbuf;

/*
  flush the write buffer
 */
bool flash_write_flush(void)
{
    if (fbuf.n == 0) {
        return true;
    }
    fbuf.n = 0;
    return flash_func_write_words(fbuf.address, fbuf.buffer, ARRAY_SIZE(fbuf.buffer));
}

/*
  write to flash with buffering to 32 bytes alignment
 */
bool flash_write_buffer(uint32_t address, const uint32_t *v, uint8_t nwords)
{
    if (fbuf.n > 0 && address != fbuf.address + fbuf.n*4) {
        if (!flash_write_flush()) {
            return false;
        }
    }
    while (nwords > 0) {
        if (fbuf.n == 0) {
            fbuf.address = address;
            memset(fbuf.buffer, 0xff, sizeof(fbuf.buffer));
        }
        uint8_t n = MIN(ARRAY_SIZE(fbuf.buffer)-fbuf.n, nwords);
        memcpy(&fbuf.buffer[fbuf.n], v, n*4);
        address += n*4;
        v += n;
        nwords -= n;
        fbuf.n += n;
        if (fbuf.n == ARRAY_SIZE(fbuf.buffer)) {
            if (!flash_write_flush()) {
                return false;
            }
        }
    }
    return true;
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

    for (int i = 0; i < ARRAY_SIZE(mcu_descriptions); i++) {
        if (mcuid == mcu_descriptions[i].mcuid) {
            des = mcu_descriptions[i];
            break;
        }
    }

    for (int i = 0; i < ARRAY_SIZE(silicon_revs); i++) {
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
#ifdef STM32F427xx
    uint32_t idcode = (*(uint32_t *)DBGMCU_BASE);
    uint16_t revid = ((idcode & REVID_MASK) >> 16);

    for (int i = 0; i < ARRAY_SIZE(silicon_revs); i++) {
        if (silicon_revs[i].revid == revid) {
            return silicon_revs[i].limit_flash_size_1M;
        }
    }
#endif
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
#ifdef BOOTLOADER_DEBUG
    va_list ap;
    static bool initialised;
    static SerialConfig debug_sercfg;
    char umsg[200];
    if (!initialised) {
        initialised = true;
        debug_sercfg.speed = 57600;
        sdStart(&BOOTLOADER_DEBUG, &debug_sercfg);
    }
    va_start(ap, fmt);
    uint32_t n = vsnprintf(umsg, sizeof(umsg), fmt, ap);
    va_end(ap);
    if (n > sizeof(umsg)) {
        n = sizeof(umsg);
    }
    chnWriteTimeout(&BOOTLOADER_DEBUG, (const uint8_t *)umsg, n, chTimeMS2I(100));
#endif
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
int strncmp(const char *s1, const char *s2, size_t n)
{
    while ((*s1 != 0) && (*s1 == *s2) && n--) {
        s1++;
        s2++;
    }
    if (n == 0) {
        return 0;
    }
    return (*s1 - *s2);
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

//simple variant of std c function to reduce used flash space
size_t strlen(const char *s1)
{
    size_t ret = 0;
    while (*s1++) ret++;
    return ret;
}

//simple variant of std c function to reduce used flash space
void *memset(void *s, int c, size_t n)
{
    uint8_t *b = (uint8_t *)s;
    while (n--) {
        *b++ = c;
    }
    return s;
}

#if defined(BOOTLOADER_DEV_LIST)
void lock_bl_port(void)
{
    locked_uart = last_uart;
}

/*
  initialise serial ports
 */
void init_uarts(void)
{
#if HAL_USE_SERIAL_USB == TRUE
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg1);
    
    usbDisconnectBus(serusbcfg1.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg1.usbp, &usbcfg);
    usbConnectBus(serusbcfg1.usbp);
#endif

#if HAL_USE_SERIAL == TRUE
    sercfg.speed = BOOTLOADER_BAUDRATE;
    
    for (uint8_t i=0; i<ARRAY_SIZE(uarts); i++) {
#if HAL_USE_SERIAL_USB == TRUE
        if (uarts[i] == (BaseChannel *)&SDU1) {
            continue;
        }
#endif
        sdStart((SerialDriver *)uarts[i], &sercfg);
    }
#endif
}


/*
  set baudrate on the current port
 */
void port_setbaud(uint32_t baudrate)
{
#if HAL_USE_SERIAL_USB == TRUE
    if (uarts[last_uart] == (BaseChannel *)&SDU1) {
        // can't set baudrate on USB
        return;
    }
#endif
#if HAL_USE_SERIAL == TRUE
    memset(&sercfg, 0, sizeof(sercfg));
    sercfg.speed = baudrate;
    sdStart((SerialDriver *)uarts[last_uart], &sercfg);
#endif
}
#endif // BOOTLOADER_DEV_LIST
