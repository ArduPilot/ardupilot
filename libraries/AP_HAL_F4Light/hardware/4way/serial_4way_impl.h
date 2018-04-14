/*
 * based on Cleanflight 4-way driver
*/
#pragma once

#include <hal.h>
#include <timer.h>
#include <boards.h>

#define MAX_SUPPORTED_MOTORS 6

typedef struct {
    const gpio_dev * gpio_device;      //< pin's GPIO device 
    uint8_t gpio_bit;                        //< Pin's GPIO port bit.
} escHardware_t;



extern bool isEscHi(uint8_t selEsc);
extern bool isEscLo(uint8_t selEsc);
extern void setEscHi(uint8_t selEsc);
extern void setEscLo(uint8_t selEsc);
extern void setEscInput(uint8_t selEsc);
extern void setEscOutput(uint8_t selEsc);


#ifndef HIGH
#define HIGH 1
#define LOW 0
#endif

extern uint8_t selected_esc;

#define ESC_IS_HI  isEscHi(selected_esc)
#define ESC_IS_LO  isEscLo(selected_esc)
#define ESC_SET_HI setEscHi(selected_esc)
#define ESC_SET_LO setEscLo(selected_esc)
#define ESC_INPUT  setEscInput(selected_esc)
#define ESC_OUTPUT setEscOutput(selected_esc)

typedef struct ioMem_s {
    uint8_t D_NUM_BYTES;
    uint8_t D_FLASH_ADDR_H;
    uint8_t D_FLASH_ADDR_L;
    uint8_t *D_PTR_I;
} ioMem_t;


static inline  void delay_us(uint16_t t) {  hal_delay_microseconds(t); }
static inline  uint32_t micros() {  return hal_micros(); }
static inline  uint32_t millis() {  return systick_uptime(); }
