/*
 * UART_PPM.cpp --- fake UART to get serial data from PPM inputs
 *
 */

#pragma once

#include "AP_HAL_F4Light.h"
#include "Scheduler.h"
#include <AP_HAL/UARTDriver.h>


#define USART_PPM_BUF_SIZE 256


class F4Light::UART_PPM : public AP_HAL::UARTDriver  {
public:
    UART_PPM(uint8_t n);

    /* F4Light implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void inline begin(uint32_t b, uint16_t rxS, uint16_t txS) {   begin(b); }
    void inline end() {  }
    void flush() {}
    bool inline is_initialized(){ return _initialized; }

    inline void set_blocking_writes(bool blocking) {  _blocking = blocking; }

    inline bool tx_pending() {   return 0; }

    uint32_t available() override;
    int16_t read() override;

    uint32_t inline  txspace() override {    return 0; } // can't TX
    size_t write(uint8_t c) { return 1; }
    size_t write(const uint8_t *buffer, size_t size) { return size; }

    static void putch(uint8_t c, uint8_t n);

private:

    bool _initialized;
    bool _blocking;
    uint8_t _id;
    
    static ring_buffer ppm_rxrb[2] IN_CCM;
    static uint8_t     rx_buf[2][USART_PPM_BUF_SIZE] IN_CCM;

};


