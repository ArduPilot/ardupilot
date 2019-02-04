/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32_util.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32_dma.h>
#include <hrt.h>

static int64_t utc_time_offset;

/*
  setup the timer capture digital filter for a channel
 */
void stm32_timer_set_input_filter(stm32_tim_t *tim, uint8_t channel, uint8_t filter_mode)
{
    switch (channel) {
    case 0:
        tim->CCMR1 |= STM32_TIM_CCMR1_IC1F(filter_mode);
        break;
    case 1:
        tim->CCMR1 |= STM32_TIM_CCMR1_IC2F(filter_mode);
        break;
    case 2:
        tim->CCMR2 |= STM32_TIM_CCMR2_IC3F(filter_mode);
        break;
    case 3:
        tim->CCMR2 |= STM32_TIM_CCMR2_IC4F(filter_mode);
        break;
    }
}

/*
  set the input source of a timer channel
 */    
void stm32_timer_set_channel_input(stm32_tim_t *tim, uint8_t channel, uint8_t input_source)
{
    switch (channel) {
        case 0:
            tim->CCER &= ~STM32_TIM_CCER_CC1E;
            tim->CCMR1 &= ~STM32_TIM_CCMR1_CC1S_MASK;
            tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(input_source);
            tim->CCER |= STM32_TIM_CCER_CC1E;
            break;
        case 1:
            tim->CCER &= ~STM32_TIM_CCER_CC2E;
            tim->CCMR1 &= ~STM32_TIM_CCMR1_CC2S_MASK;
            tim->CCMR1 |= STM32_TIM_CCMR1_CC2S(input_source);
            tim->CCER |= STM32_TIM_CCER_CC2E;
            break;
        case 2:
            tim->CCER &= ~STM32_TIM_CCER_CC3E;
            tim->CCMR2 &= ~STM32_TIM_CCMR2_CC3S_MASK;
            tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(input_source);
            tim->CCER |= STM32_TIM_CCER_CC3E;
            break;
        case 3:
            tim->CCER &= ~STM32_TIM_CCER_CC4E;
            tim->CCMR2 &= ~STM32_TIM_CCMR2_CC4S_MASK;
            tim->CCMR2 |= STM32_TIM_CCMR2_CC4S(input_source);
            tim->CCER |= STM32_TIM_CCER_CC4E;
            break;
    }
}

#if CH_DBG_ENABLE_STACK_CHECK == TRUE
void show_stack_usage(void)
{
  thread_t *tp;

  tp = chRegFirstThread();
  do {
      uint32_t stklimit = (uint32_t)tp->wabase;
      uint8_t *p = (uint8_t *)tp->wabase;
      while (*p == CH_DBG_STACK_FILL_VALUE) {
          p++;
      }
      uint32_t stack_left = ((uint32_t)p) - stklimit;
      printf("%s %u\n", tp->name, (unsigned)stack_left);
      tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

/*
  flush all memory. Used in chSysHalt()
 */
void memory_flush_all(void)
{
    cacheBufferFlush(HAL_RAM_BASE_ADDRESS, HAL_RAM_SIZE_KB * 1024U);
}

/*
  set the utc time
 */
void stm32_set_utc_usec(uint64_t time_utc_usec)
{
    uint64_t now = hrt_micros64();
    if (now <= time_utc_usec) {
        utc_time_offset = time_utc_usec - now;
    }
}

/*
  get system clock in UTC microseconds
*/
uint64_t stm32_get_utc_usec()
{
    return hrt_micros64() + utc_time_offset;
}

struct utc_tm {
    uint8_t tm_year; // since 1900
    uint8_t tm_mon;  // zero based
    uint8_t tm_mday; // zero based
    uint8_t tm_hour;
    uint8_t tm_min;
    uint8_t tm_sec;
};


/*
  return true if a year is a leap year
 */
static bool is_leap(uint32_t y)
{
    y += 1900;
    return (y % 4) == 0 && ((y % 100) != 0 || (y % 400) == 0);
}

static const uint8_t ndays[2][12] ={
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};

/*
  parse a seconds since 1970 into a utc_tm structure
  code based on _der_gmtime from samba
 */
static void parse_utc_seconds(uint64_t utc_sec, struct utc_tm *tm)
{
    uint32_t secday = utc_sec % (3600U * 24U);
    uint32_t days = utc_sec / (3600U * 24U);

    memset(tm, 0, sizeof(*tm));

    tm->tm_sec = secday % 60U;
    tm->tm_min = (secday % 3600U) / 60U;
    tm->tm_hour = secday / 3600U;
    tm->tm_year = 70;

    if (days > (2000 * 365)) {
        // don't look for dates too far into the future
        return;
    }

    while (true) {
        unsigned dayinyear = (is_leap(tm->tm_year) ? 366 : 365);
        if (days < dayinyear) {
            break;
        }
        tm->tm_year += 1;
        days -= dayinyear;
    }
    tm->tm_mon = 0;

    while (true) {
        unsigned daysinmonth = ndays[is_leap(tm->tm_year)?1:0][tm->tm_mon];
        if (days < daysinmonth) {
            break;
        }
        days -= daysinmonth;
        tm->tm_mon++;
    }
    tm->tm_mday = days + 1;
}


/*
  get time for fat filesystem. This is based on
  rtcConvertDateTimeToFAT from the ChibiOS RTC driver. We don't use
  the hw RTC clock as it is very inaccurate
 */
uint32_t get_fattime()
{
    if (utc_time_offset == 0) {
        // return a fixed time
        return ((uint32_t)0 | (1 << 16)) | (1 << 21);
    }
    uint64_t utc_usec = stm32_get_utc_usec();
    uint64_t utc_sec = utc_usec / 1000000UL;
    struct utc_tm tm;

    parse_utc_seconds(utc_sec, &tm);
    
    uint32_t fattime;

    fattime  = tm.tm_sec  >> 1U;
    fattime |= tm.tm_min  << 5U;
    fattime |= tm.tm_hour << 11U;
    fattime |= tm.tm_mday << 16U;
    fattime |= (tm.tm_mon+1)  << 21U;
    fattime |= (uint32_t)((tm.tm_year-80) << 25U);
    
    return fattime;
}

#if !defined(NO_FASTBOOT)
// get RTC backup register 0
static uint32_t get_rtc_backup0(void)
{
	return RTC->BKP0R;
}

// set RTC backup register 0
static void set_rtc_backup0(uint32_t v)
{
    if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
        RCC->BDCR |= STM32_RTCSEL;
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }
#ifdef PWR_CR_DBP
    PWR->CR |= PWR_CR_DBP;
#else
    PWR->CR1 |= PWR_CR1_DBP;
#endif
    RTC->BKP0R = v;
}

// see if RTC registers is setup for a fast reboot
enum rtc_boot_magic check_fast_reboot(void)
{
    return (enum rtc_boot_magic)get_rtc_backup0();
}

// set RTC register for a fast reboot
void set_fast_reboot(enum rtc_boot_magic v)
{
    set_rtc_backup0(v);
}

#endif //NO_FASTBOOT

/*
  enable peripheral power if needed This is done late to prevent
  problems with CTS causing SiK radios to stay in the bootloader. A
  SiK radio will stay in the bootloader if CTS is held to GND on boot
*/
void peripheral_power_enable(void)
{
#if defined(HAL_GPIO_PIN_nVDD_5V_PERIPH_EN) || defined(HAL_GPIO_PIN_nVDD_5V_HIPOWER_EN) || defined(HAL_GPIO_PIN_VDD_3V3_SENSORS_EN) || defined(HAL_GPIO_PIN_nVDD_3V3_SD_CARD_EN) || defined(HAL_GPIO_PIN_VDD_3V3_SD_CARD_EN)
    // we don't know what state the bootloader had the CTS pin in, so
    // wait here with it pulled up from the PAL table for enough time
    // for the radio to be definately powered down
    uint8_t i;
    for (i=0; i<100; i++) {
        // use a loop as this may be a 16 bit timer
        chThdSleep(chTimeMS2I(1));
    }
#ifdef HAL_GPIO_PIN_nVDD_5V_PERIPH_EN
    palWriteLine(HAL_GPIO_PIN_nVDD_5V_PERIPH_EN, 0);
#endif
#ifdef HAL_GPIO_PIN_nVDD_5V_HIPOWER_EN
    palWriteLine(HAL_GPIO_PIN_nVDD_5V_HIPOWER_EN, 0);
#endif
#ifdef HAL_GPIO_PIN_VDD_3V3_SENSORS_EN
    // the TBS-Colibri-F7 needs PE3 low at power on
    palWriteLine(HAL_GPIO_PIN_VDD_3V3_SENSORS_EN, 1);
#endif
#ifdef HAL_GPIO_PIN_nVDD_3V3_SD_CARD_EN
    // the TBS-Colibri-F7 needs PG7 low for SD card
    palWriteLine(HAL_GPIO_PIN_nVDD_3V3_SD_CARD_EN, 0);
#endif
#ifdef HAL_GPIO_PIN_VDD_3V3_SD_CARD_EN
    // others need it active high
    palWriteLine(HAL_GPIO_PIN_VDD_3V3_SD_CARD_EN, 1);
#endif
#endif
}

#if defined(STM32F7) || defined(STM32F4)
/*
  read mode of a pin. This allows a pin config to be read, changed and
  then written back
 */
iomode_t palReadLineMode(ioline_t line)
{
    ioportid_t port = PAL_PORT(line);
    uint8_t pad = PAL_PAD(line);
    iomode_t ret = 0;
    ret |= (port->MODER >> (pad*2)) & 0x3;
    ret |= ((port->OTYPER >> pad)&1) << 2;
    ret |= ((port->OSPEEDR >> (pad*2))&3) << 3;
    ret |= ((port->PUPDR >> (pad*2))&3) << 5;
    if (pad < 8) {
        ret |= ((port->AFRL >> (pad*4))&0xF) << 7;
    } else {
        ret |= ((port->AFRH >> ((pad-8)*4))&0xF) << 7;
    }
    return ret;
}
#endif
