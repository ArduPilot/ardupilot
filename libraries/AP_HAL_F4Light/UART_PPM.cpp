/*
    (c) 2017 night_ghost@ykoctpa.ru
 

 * UART_PPM.cpp --- fake UART to get serial data from PPM inputs
 *
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT

#include "UART_PPM.h"


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>



using namespace F4Light;

ring_buffer UART_PPM::ppm_rxrb[2] IN_CCM;
uint8_t     UART_PPM::rx_buf[2][USART_PPM_BUF_SIZE] IN_CCM;

UART_PPM::UART_PPM(uint8_t n):
    _initialized(false),
    _blocking(true),
    _id(n)
{
    rb_init(&ppm_rxrb[n], USART_PPM_BUF_SIZE, rx_buf[n]);
}

void UART_PPM::begin(uint32_t baud) {
    if(_initialized) return;

    // signal that uart connected
    _initialized = true;
}

uint32_t UART_PPM::available() {
    return rb_full_count(&ppm_rxrb[_id]);  
}

int16_t UART_PPM::read() {
    if (available() <= 0) return -1;
    return rb_remove(&ppm_rxrb[_id]);
}

void UART_PPM::putch(uint8_t c, uint8_t n){
    /* If the buffer is full  ignore new bytes. */
    rb_safe_insert(&ppm_rxrb[n], c);
}

#endif // CONFIG_HAL_BOARD
