/*
 * UARTDriver.cpp --- AP_HAL_F4Light UART driver.

 (c) 2017 night_ghost@ykoctpa.ru
 
based on:
 
 * UART driver
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
#include "UARTDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <usb.h>
#include <usart.h>
#include <gpio_hal.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

using namespace F4Light;

UARTDriver::UARTDriver(const struct usart_dev *usart):
    _usart_device(usart),
    _initialized(false),
    _blocking(true)
{
}

//uint8_t mode = (UART_Parity_No <<16) | UART_Stop_Bits_1
void UARTDriver::begin(uint32_t baud, uint32_t bmode) {

    if(!_usart_device) return;

#ifdef BOARD_SBUS_UART
    if(_initialized &&  hal_param_helper->_uart_sbus && _usart_device==UARTS[hal_param_helper->_uart_sbus]) return; //already used as SBUS
#endif

    _baudrate = baud;

    uint32_t mode=0;

    if(_usart_device->tx_pin < BOARD_NR_GPIO_PINS){
        const stm32_pin_info *txi = &PIN_MAP[_usart_device->tx_pin];
        gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, _usart_device->gpio_af);
        gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
        mode |= USART_Mode_Tx;
    } 
	
    if(_usart_device->rx_pin < BOARD_NR_GPIO_PINS){
	const stm32_pin_info *rxi = &PIN_MAP[_usart_device->rx_pin];
        gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, _usart_device->gpio_af);
        gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_OD_PU); 
        mode |= USART_Mode_Rx;
    }

    if(!mode) return;

    usart_disable(_usart_device);
        
    usart_init(_usart_device);
    usart_setup(_usart_device, (uint32_t)baud, 
                UART_Word_8b, bmode & 0xffff /*USART_StopBits_1*/ , (bmode>>16) & 0xffff /* USART_Parity_No*/, mode, UART_HardwareFlowControl_None);
    usart_enable(_usart_device);

    usart_set_callback(_usart_device, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&UARTDriver::update_timestamp, void)) );
    
    _initialized = true;
}


void UARTDriver::flush() {
    if (!_initialized) {
        return;
    }
    usart_reset_rx(_usart_device);
    usart_reset_tx(_usart_device);
}


uint32_t UARTDriver::available() {
    if (!_initialized) {
        return 0;
    }

    uint16_t v=usart_data_available(_usart_device); 
    return v;
}

int16_t UARTDriver::read() {
    if (available() == 0) {
        return -1;
    }
    return usart_getc(_usart_device);
}

size_t UARTDriver::write(uint8_t c) {

    if (!_initialized) { 
        return 0;
    }
    uint16_t n;
    uint16_t tr=2; // попытки
    while(tr) {
        n = usart_putc(_usart_device, c);
        if(n==0) { // no place for character
            hal_yield(0);
            if(!_blocking) tr--; // in unlocking mode we reduce the retry count
        } else break; // sent!
    } 
    return n;
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    uint16_t tr=2; // tries
    uint16_t n;
    uint16_t sent=0;
    while(tr && size) {

        n = usart_tx(_usart_device, buffer, size);
        if(n<size) { // no place for character
            hal_yield(0);
            if(!_blocking) tr--; // in unlocking mode we reduce the retry count
        } else break; // sent
        buffer+=n;
        sent+=n;
        size-=n;
    }
    return sent;
}

void UARTDriver::update_timestamp(){  // called from ISR
    _time_idx ^= 1; 
    _receive_timestamp[_time_idx] = AP_HAL::micros(); 
}

// this is mostly a 
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes) {

    // timestamp is 32 bits so read is atomic, in worst case we get 2nd timestamp
    uint32_t time_from_last_byte = AP_HAL::micros() -  _receive_timestamp[_time_idx];
    uint32_t transport_time_us = 0;
    if (_baudrate > 0) {
        // assume 10 bits per byte
        transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes+available());
    }
    return AP_HAL::micros64() - (time_from_last_byte + transport_time_us);
}


#endif // CONFIG_HAL_BOARD
