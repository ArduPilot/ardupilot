#ifndef _HAL_TYPES_H_
#define _HAL_TYPES_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32.h"
#include "util.h"
#include "../../Config.h"

#ifdef __cplusplus
  extern "C" {
#endif


typedef void (*voidFuncPtr)(void);
typedef void (*revo_isr_handler)(uint32_t arg);

typedef uint64_t Handler;


typedef union Revo_Hal_Handler { // кровь кишки ассемблер :) преобразование функторов в унифицированный вид
    voidFuncPtr vp;
    revo_isr_handler isr;
//  AP_HAL::MemberProc mp;          это С а не С++ поэтому мы не можем объявить поддержку функторов явно, и вынуждены передавать
    Handler h;      // treat as handle             <-- как 64-битное число
    uint32_t w[2]; // words, to check. если функтор то старшее - адрес флеша, младшее - адрес в RAM. Если ссылка на функцию то младшее - адрес флеша, старшее 0
} Revo_hal_handler;

extern void revo_call_handler(uint64_t hh, uint32_t arg);

extern uint32_t SystemCoreClock;
extern void systemInit(uint8_t oc);
extern uint32_t hal_micros();

#ifdef __cplusplus
  }
#endif


#define __attr_flash __attribute__((section (".USER_FLASH")))
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

#define NAKED __attribute__((__naked__))

#define __io volatile
#define __deprecated __attribute__((__deprecated__))
#define __weak __attribute__((weak))
#ifndef __always_inline
#define __always_inline inline __attribute__((always_inline))
#endif
#ifndef __unused
#define __unused __attribute__((unused))
#endif

#ifndef NULL
#define NULL 0
#endif





/**
 * Invalid stm32_pin_info adc_channel value.
 * @see stm32_pin_info
 */
#define nADC 0xFF


/**
 * Variable attribute, instructs the linker to place the marked
 * variable in Flash instead of RAM. */
#define __FLASH__ __attr_flash

#ifndef INLINE
 #define INLINE __attribute__ ((always_inline)) inline
#endif
#define WEAK __attribute__((weak))

#ifndef IN_CCM
 #define IN_CCM  __attribute__((section(".ccm")))
#endif


//#define OK	1
//#define ERROR	0

#define ADDRESS_IN_RAM(a) ((uint32_t)(a) >= SRAM1_BASE && (uint32_t)(a) < (uint32_t)(STM32_SRAM_END) )
#define ADDRESS_IN_CCM(a) ((uint32_t)(a) >= CCMDATARAM_BASE && (uint32_t)(a) < (uint32_t)(STM32_CCM_END) )
#define ADDRESS_IN_FLASH(a) ((a)>FLASH_BASE && (a)<CCMDATARAM_BASE)



#endif
