/******************************************************************************
 * The MIT License

(c) 2017 night_ghost@ykoctpa.ru
 
based on:

 *
 * Copyright (c) 2010 Bryan Newbold.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file boards.h

(c) 2017 night_ghost@ykoctpa.ru
 
 
 based on:
 
 * @author Bryan Newbold <bnewbold@leaflabs.com>,
 *         Marti Bolivar <mbolivar@leaflabs.com>
 
 * @brief Board-specific pin information.
 *
 * To add a new board type, add a new pair of files to
 * /wirish/boards/, update the section below with a new "BOARD" type,
 * and update /wirish/rules.mk to include your boards/your_board.cpp
 * file in the top-level Makefile build.
 */

#ifndef _BOARDS_H_
#define _BOARDS_H_

#include <hal.h>
#include <stm32f4xx_conf.h>
#include <hal_types.h>
#include <exti.h>
#include <gpio_hal.h>
#include <timer.h>
#include <adc.h>
#include <usart.h>
#include <pwm_in.h>

/* Set of all possible pin names; not all boards have all these (note
 * that we use the Dx convention since all of the Maple's pins are
 * "digital" pins (e.g. can be used with digitalRead() and
 * digitalWrite()), but not all of them are connected to ADCs. */
enum {
    D0=0, D1,   D2,   D3,   D4,   D5,   D6,   D7,   D8,   D9, 
    D10,  D11,  D12,  D13,  D14,  D15,  D16,  D17,  D18,  D19,
    D20,  D21,  D22,  D23,  D24,  D25,  D26,  D27,  D28,  D29, 
    D30,  D31,  D32,  D33,  D34,  D35,  D36,  D37,  D38,  D39, 
    D40,  D41,  D42,  D43,  D44,  D45,  D46,  D47,  D48,  D49, 
    D50,  D51,  D52,  D53,  D54,  D55,  D56,  D57,  D58,  D59, 
    D60,  D61,  D62,  D63,  D64,  D65,  D66,  D67,  D68,  D69,  
    D70,  D71,  D72,  D73,  D74,  D75,  D76,  D77,  D78,  D79,  
    D80,  D81,  D82,  D83,  D84,  D85,  D86,  D87,  D88,  D89, 
    D90,  D91,  D92,  D93,  D94,  D95,  D96,  D97,  D98,  D99,  
    D100, D101, D102, D103, D104, D105, D106, D107, D108, D109, 
    D110, D111, };


/**
 * @brief Stores STM32-specific information related to a given Maple pin.
 * @see PIN_MAP
 */

typedef struct stm32_pin_info {
    const gpio_dev  * const gpio_device;  /**< Maple pin's GPIO device */
    const timer_dev * const timer_device; /**< Pin's timer device, if any. */
    const adc_dev   * const adc_device;   /**< ADC device, if any. */
    uint8_t gpio_bit;                     /**< Pin's GPIO port bit. */
    timer_Channel timer_channel;          /**< Timer channel, or 0 if none. */
    uint8_t adc_channel;                  /**< Pin ADC channel, or nADC if none. */
} stm32_pin_info;

/**
 * @brief Maps each Maple pin to a corresponding stm32_pin_info.
 * @see stm32_pin_info
 */
extern const stm32_pin_info PIN_MAP[];


/**
 * @brief Generic board initialization function.
 *
 * This function is called before main().  It ensures that the clocks
 * and peripherals are configured properly for use with wirish, then
 * calls boardInit().
 *
 * @see boardInit()
 */
void init(void);


/**
 * @brief Board-specific initialization function.
 *
 * This function is called from init() after all generic board
 * initialization has been performed.  Each board is required to
 * define its own.
 *
 * @see init()
 */
extern void boardInit(void);


#ifdef __cplusplus
 extern "C" {
#endif
extern void pre_init(void);

void board_set_rtc_register(uint32_t sig, uint16_t reg);
uint32_t board_get_rtc_register(uint16_t reg);

static inline void goDFU();
static inline void goDFU(){            // Reboot to BootROM - to DFU mode
    asm volatile("\
    ldr     r0, =0x1FFF0000\n\
    ldr     sp,[r0, #0]    \n\
    ldr     r0,[r0, #4]    \n\
    bx      r0             \n\
    ");
}

extern unsigned __isr_vector_start; // defined by link script

static inline bool is_bare_metal();
static inline bool is_bare_metal() {
    return (uint32_t)&__isr_vector_start == 0x08000000;
}

void NMI_Handler();
void emerg_delay(uint32_t n);
extern void SetSysClock(uint8_t oc);

extern voidFuncPtr boardEmergencyHandler;

extern void clock_gettime(uint32_t mode, void *ptr);
extern void systemInit(uint8_t oc);

#ifdef __cplusplus
 }
#endif


#ifndef BOARD_NR_GPIO_PINS
 #error "Board type has not been selected correctly."
#endif

// pin names, see PIN_MAP array
enum {
    PB10 =0,
    PB2  =1,
    PB12 =2,
    PB13 =3,
    PB14 =4,
    PB15 =5,
    PC0  =6,
    PC1  =7,
    PC2  =8,
    PC3  =9,
    PC4  =10,
    PC5  =11,
    PC6  =12,
    PC7  =13,
    PC8  =14,
    PC9  =15,
    PC10 =16,
    PC11 =17,
    PC12 =18,
    PC13 =19,
    PC14 =20,
    PC15 =21,
    PA8  =22,
    PA9  =23,
    PA10 =24,
    PB9  =25,
    PD2  =26,
    PD3  =27,
    PD6  =28,
    PG11 =29,
    PG12 =30,
    PG13 =31,
    PG14 =32,
    PG8  =33,
    PG7  =34,
    PG6  =35,
    PB5  =36,
    PB6  =37,
    PB7  =38,
    PF6  =39,
    PF7  =40,
    PF8  =41,
    PF9  =42,
    PF10 =43,
    PF11 =44,
    PB1  =45,
    PB0  =46,
    PA0  =47,
    PA1  =48,
    PA2  =49,
    PA3  =50,
    PA4  =51,
    PA5  =52,
    PA6  =53,
    PA7  =54,
    PF0  =55,
    PD11 =56,
    PD14 =57,
    PF1  =58,
    PD12 =59,
    PD15 =60,
    PF2  =61,
    PD13 =62,
    PD0  =63,
    PF3  =64,
    PE3  =65,
    PD1  =66,
    PF4  =67,
    PE4  =68,
    PE7  =69,
    PF5  =70,
    PE5  =71,
    PE8  =72,
    PF12 =73,
    PE6  =74,
    PE9  =75,
    PF13 =76,
    PE10 =77,
    PF14 =78,
    PG9  =79,
    PE11 =80,
    PF15 =81,
    PG10 =82,
    PE12 =83,
    PG0  =84,
    PD5  =85,
    PE13 =86,
    PG1  =87,
    PD4  =88,
    PE14 =89,
    PG2  =90,
    PE1  =91,
    PE15 =92,
    PG3  =93,
    PE0  =94,
    PD8  =95,
    PG4  =96,
    PD9  =97,
    PG5  =98,
    PD10 =99,
    PB11 =100,
    PB8  =101,
    PE2  =102,
    PA15 =103,
    PB3  =104,
    PB4  =105,
    PA13 =106,
    PA14 =107,
    PA11 =108,
};

/* Set derived definitions */

#define CLOCK_SPEED_MHZ                 CYCLES_PER_MICROSECOND
#define CLOCK_SPEED_HZ                  (CLOCK_SPEED_MHZ * 1000000UL)

// PX4 writes as
// *(uint32_t *)0x40002850 = 0xb007b007;
#define BOOT_RTC_SIGNATURE      0xb007b007
#define DFU_RTC_SIGNATURE       0xDEADBEEF
#define FORCE_APP_RTC_SIGNATURE 0x4000AbbA


#define DSM_BIND_SIGNATURE      0xD82B14D0 // "DSMBIND" last nibble for DSM code
#define DSM_BIND_SIGN_MASK      0xF // mask for last nibble - DSM code

#define CONSOLE_PORT_SIGNATURE  0xC07501e0 // "console" last nibble for port number
#define CONSOLE_PORT_MASK       0xF // mask for last nibble - port number


#define MASS_STORAGE_SIGNATURE  0x5106a8ed // "storaged"


#define OVERCLOCK_SIGNATURE     0xFACED1A0 // "FaceDia" last nibble for mode
#define OVERCLOCK_SIG_MASK      0xF        // mode mask
#define OV_GUARD_SIGNATURE      0xBABEFACE //
#define OV_GUARD_FAIL_SIGNATURE 0xBABEFA17 

// Backup SRAM registers usage
#define RTC_SIGNATURE_REG       0 
#define RTC_DSM_BIND_REG        1   
#define RTC_MASS_STORAGE_REG    2
#define RTC_CONSOLE_REG         3
#define RTC_OVERCLOCK_REG       4
#define RTC_OV_GUARD_REG        5




#define digitalPinToPort(P)        ( PIN_MAP[P].gpio_device )
#define digitalPinToBitMask(P)     ( BIT(PIN_MAP[P].gpio_bit) )
#define portOutputRegister(port)   ( &(port->regs->ODR) )
#define portInputRegister(port)    ( &(port->regs->IDR) )

#define portSetRegister(pin)            ( &(PIN_MAP[pin].gpio_device->regs->BSRR) )
#define portClearRegister(pin)          ( &(PIN_MAP[pin].gpio_device->regs->BRR) )

#define portConfigRegister(pin)         ( &(PIN_MAP[pin].gpio_device->regs->CRL) )


#endif
