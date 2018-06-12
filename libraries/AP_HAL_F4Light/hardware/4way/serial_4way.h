/*
 * based on Cleanflight 4way driver
 *
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <AP_HAL/AP_HAL.h>
#include <hal.h>
#include <systick.h>

#include "serial_4way_impl.h"

#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_SERIAL_4WAY_SK_BOOTLOADER

#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3
#define imARM_BLB 4



typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
} uint8_32_u;


extern uint16_t _crc_xmodem_update (uint16_t crc, uint8_t data);
extern bool isMcuConnected(void);

extern uint8_t esc4wayInit(const uint8_t *output_channels, uint8_t nm);

// rem: App: Wait at least appx. 500 ms for BLHeli to jump into
// bootloader mode before try to connect any ESC
// Start to activate here
extern void esc4wayProcess(AP_HAL::UARTDriver *uartPort);

