
#pragma once

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>

#include <usart.h>

#include <gpio_hal.h>
#include <hal.h>
#include "Scheduler.h"

#define DEFAULT_TX_TIMEOUT 10000 // in uS - 10ms

enum UART_STOP_BITS {
    UART_Stop_Bits_1  = ((uint16_t)0x0000),
    UART_Stop_Bits_0_5= ((uint16_t)0x1000),
    UART_Stop_Bits_2  = ((uint16_t)0x2000),
    UART_Stop_Bits_1_5= ((uint16_t)0x3000),
};

enum UART_PARITY {
    UART_Parity_No   = ((uint16_t)0x0000),
    UART_Parity_Even = ((uint16_t)0x0400),
    UART_Parity_Odd  = ((uint16_t)0x0600),
};

class F4Light::UARTDriver : public AP_HAL::UARTDriver  {
public:
    UARTDriver(const struct usart_dev *usart);

    inline void begin(uint32_t b){
        begin(b, (UART_Parity_No <<16) | UART_Stop_Bits_1);
    }

    void begin(uint32_t b, uint32_t mode); // must be
    inline void begin(uint32_t b, uint16_t rxS, uint16_t txS) {   begin(b); }
    inline void end() { _initialized=false; if(_usart_device) usart_disable(_usart_device);  }
    void flush();
    inline bool is_initialized(){ return _initialized; }
  
    inline void set_blocking_writes(bool blocking) {  _blocking = blocking; }

    inline bool tx_pending() {   return  usart_txfifo_nbytes(_usart_device) > 0; }

    inline void setCallback(Handler cb) { usart_set_callback(_usart_device, cb); }

    uint32_t available() override;
    uint32_t inline  txspace() override {    return usart_txfifo_freebytes(_usart_device); }
    int16_t read() override;

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    inline void disable(){ _usart_device = NULL; } // pins used for another needs

    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    void update_timestamp(); 

private:

    const struct usart_dev *_usart_device;
    bool _initialized;
    bool _blocking;
    uint32_t _baudrate;
    uint32_t _receive_timestamp[2];
    uint8_t _time_idx;
};


