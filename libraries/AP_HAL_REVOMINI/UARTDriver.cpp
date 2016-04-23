/*
 * UARTDriver.cpp --- AP_HAL_REVOMINI UART driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
#include "UARTDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <usb.h>
#include <usart.h>
#include <gpio_hal.h>

//static usb_attr_t usb_attr;

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

//definisco qui i parametri per le varie seriali preconfigurate

REVOMINIUARTDriver::REVOMINIUARTDriver(struct usart_dev *usart, uint8_t use_usb):
    _usart_device(usart),
    _usb(use_usb),
    _usb_present(0),
    _initialized(false)
{
}

void REVOMINIUARTDriver::begin(uint32_t baud) {

    if(_usb == 1)
	_usb_present = gpio_read_bit(_GPIOC,5);
    else
	_usb_present = 0;

    if(_usb_present == 1)
    {
	/*
	usb_attr_t usb_attr;

	usb_open();
	usb_default_attr(&usb_attr);
	usb_attr.preempt_prio = 0;
	usb_attr.sub_prio = 0;
	usb_attr.use_present_pin = 1;
	usb_attr.present_port = _GPIOC;
	usb_attr.present_pin = 5;
	usb_ioctl(I_USB_SETATTR, &usb_attr);

	delay_us(1000);
	*/
    }
    else
    {
	const stm32_pin_info *txi = &PIN_MAP[_usart_device->tx_pin];
	const stm32_pin_info *rxi = &PIN_MAP[_usart_device->rx_pin];

	gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, _usart_device->gpio_af);
	gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
	gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, _usart_device->gpio_af);
	gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_PP);

	usart_init(_usart_device);
	usart_setup(_usart_device, (uint32)baud, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, DEFAULT_TX_TIMEOUT);
	usart_enable(_usart_device);
    }
    _initialized = true;
}

void REVOMINIUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
    begin(baud);
}

void REVOMINIUARTDriver::end() {
    if(_usb_present == 1)
	usb_close();
    else
	usart_disable(_usart_device);
}

void REVOMINIUARTDriver::flush() {
    if(_usb_present ==1){
	usb_reset_rx();
	//usb_reset_tx();
    }else {
	usart_reset_rx(_usart_device);
	usart_reset_tx(_usart_device);
    }

}

void REVOMINIUARTDriver::set_blocking_writes(bool blocking) {
    if(_usb_present == 0){
	usart_reset_tx(_usart_device);
	_usart_device->usetxrb = !blocking;
    }
}

bool REVOMINIUARTDriver::tx_pending() {
    if(_usb_present == 0){
	if (usart_txfifo_nbytes(_usart_device) > 0)
	    {
	    return true;
	    }
    }
    return false;
}


/* REVOMINI implementations of Stream virtual methods */
int16_t REVOMINIUARTDriver::available() {
    if(_usb_present == 1)
	return usb_data_available();
    else
    return usart_data_available(_usart_device);
}

int16_t REVOMINIUARTDriver::txspace() {
    if(_usb_present == 1)
	return 255;
    else
	return usart_txfifo_freebytes(_usart_device);
}

int16_t REVOMINIUARTDriver::read() {
    if(_usb_present == 1){
	if (usb_data_available() <= 0)
	    return (-1);
	return usb_getc();
    } else {
	if (available() <= 0)
	    return (-1);
	return usart_getc(_usart_device);
    }
}

/* REVOMINI implementations of Print virtual methods */
size_t REVOMINIUARTDriver::write(uint8_t c) {

    if (hal.scheduler->in_timerprocess()) {
        // not allowed from timers
        return 0;
    }

    if(_usb_present == 1){
	usb_putc(c);
	return 1;
    }
    else{
	usart_putc(_usart_device, c);
	return 1;
    }
}

size_t REVOMINIUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif // CONFIG_HAL_BOARD

