/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#include "hal.h"
#include "ch.h"
#include <AP_HAL_ChibiOS/hwdef/common/flash.h>
#include "protocol.h"

#if !defined(STM32F100xB) && !defined(STM32F103xB)
#error This bootloader is only for the medium-density F100/F103 IOMCUs
#endif

namespace IOMCU_BL
{

static_assert(FLASH_BOOTLOADER_LOAD_KB == 4 && BOARD_FLASH_SIZE == 64,
              "The legacy IOMCU protocol uses a 4 KiB loader and 60 KiB application");
static_assert(HAL_MEMORY_TOTAL_KB == 8,
              "The shared bootloader must fit the F100's 8 KiB RAM");
static_assert(CH_CFG_ST_FREQUENCY == 1000,
              "UART timeouts use one system tick per millisecond");

static Protocol protocol;

int receive(uint32_t timeout_ms)
{
    const systime_t start = chVTGetSystemTimeX();
    do {
        const uint32_t status = USART2->SR;
        if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            const uint8_t byte = USART2->DR;
            if (!(status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))) {
                return byte;
            }
        }
    } while (chVTTimeElapsedSinceX(start) < sysinterval_t(timeout_ms));
    return -1;
}

void send(uint8_t byte)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = byte;
}

uint32_t read_word(uint32_t offset)
{
    return *reinterpret_cast<volatile const uint32_t *>(APP_BASE + offset);
}

bool erase_application()
{
    for (uint32_t page = 4; page < 64; page++) {
        if (!stm32_flash_erasepage(page)) {
            return false;
        }
    }
    return true;
}

bool write_word(uint32_t offset, uint32_t word)
{
    if ((offset & 3) || offset >= APP_SIZE) {
        return false;
    }
    return stm32_flash_write(APP_BASE + offset, &word, sizeof(word));
}

static void jump_to_app()
{
    const uint32_t sp = read_word(0);
    const uint32_t pc = read_word(4);
    if (!valid_app_vectors(sp, pc)) {
        return;
    }
    while (!(USART2->SR & USART_SR_TC)) {}
    chSysDisable();
    SysTick->CTRL = 0;
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xffffffff;
        NVIC->ICPR[i] = 0xffffffff;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    USART2->CR1 = 0;
    // Leave the same reset clock and GPIO state as the original loader.
    RCC->CFGR = 0;
    while (RCC->CFGR & RCC_CFGR_SWS) {}
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_HSEBYP);
    RCC->APB1RSTR = RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST;
    RCC->APB2RSTR = 0;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;
    SCB->VTOR = APP_BASE;
    __DSB();
    __ISB();
    // No C code may run after changing the stack selection and pointer.
    asm volatile("msr control, %2\n"
                 "isb\n"
                 "msr msp, %0\n"
                 "msr basepri, %2\n"
                 "cpsie i\n"
                 "bx %1\n" : : "r"(sp), "r"(pc), "r"(0) : "memory");
    __builtin_unreachable();
}

}

extern "C" int main(void)
{
    using namespace IOMCU_BL;
    // PB5 is sampled once. Releasing safety does not cancel recovery mode;
    // a complete BOOT command may still launch the application.
    const bool safety_pressed = palReadLine(HAL_GPIO_PIN_SAFETY_INPUT) != 0;
    rccEnableUSART2(false);
    USART2->BRR = (STM32_PCLK1 + 57600) / 115200;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    const systime_t start = chVTGetSystemTimeX();
    bool boot_attempted = false;
    while (true) {
        const int c = receive(0);
        if (c >= 0 && protocol.command(c)) {
            jump_to_app();
        }
        if (!boot_attempted && !safety_pressed && !protocol.active() &&
            chVTTimeElapsedSinceX(start) >= TIME_MS2I(200)) {
            boot_attempted = true;
            jump_to_app();
        }
        const bool led_on = (chVTGetSystemTimeX() / TIME_MS2I(50)) & 1;
        palWriteLine(HAL_GPIO_PIN_LED_BOOTLOADER, led_on);
    }
}
