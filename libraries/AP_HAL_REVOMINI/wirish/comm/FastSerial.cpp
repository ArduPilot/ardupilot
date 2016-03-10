/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/**
 * @file FastSerial.cpp
 *
 * @brief Wiring-like serial api
 */

#include "FastSerial.h"
#include "usb.h"
#include <usart.h>
#include <gpio_hal.h>

//#define TX1 BOARD_USART1_TX_PIN
//#define RX1 BOARD_USART1_RX_PIN
//#define TX2 BOARD_USART2_TX_PIN
//#define RX2 BOARD_USART2_RX_PIN
//#define TX3 BOARD_USART3_TX_PIN
//#define RX3 BOARD_USART3_RX_PIN
//#define TX4 BOARD_UART4_TX_PIN
//#define RX4 BOARD_UART4_RX_PIN
//#define TX5 BOARD_UART5_TX_PIN
//#define RX5 BOARD_UART5_RX_PIN
static usb_attr_t usb_attr;
static uint8_t usb_connected;

FastSerial::FastSerial()
{
}

//uint8_t FastSerial::_serialInitialized = 0;

// Constructor /////////////////////////////////////////////////////////////////

FastSerial::FastSerial(usart_dev *usart_device,
                       uint8 tx_pin,
                       uint8 rx_pin) {
    if(usb == 0){
    this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->begin(57600);
    }
}


void FastSerial::init(usart_dev *usart_device,
                       uint8 tx_pin,
                       uint8 rx_pin)
{
	this->usart_device = usart_device;
	this->tx_pin = tx_pin;
	this->rx_pin = rx_pin;
}

void FastSerial::configure(uint8 port)
{
	usb=0; // Set 0 the use of usb inside FastSerial
	
	if (port == 0)
	{
		this->init(FSUSART0, FSTXPIN0, FSRXPIN0);
	}
	else if (port == 1)
	{
		this->init(FSUSART1, FSTXPIN1, FSRXPIN1);
	}
	else if (port == 2)
	{
		this->init(FSUSART2, FSTXPIN2, FSRXPIN2);
	}
	else if (port == 3)
	{
		this->init(FSUSART3, FSTXPIN3, FSRXPIN3);
	}
	else if (port == 99)
	{
		usb=1;
	}
}


#define disable_timer_if_necessary(dev, ch) ((void)0)

void FastSerial::begin(long baud) {
	if (usb == 0)
	begin(baud, DEFAULT_TX_TIMEOUT);
	else
    {
	//begin(baud, DEFAULT_TX_TIMEOUT);

    usb_open();

    usb_default_attr(&usb_attr);
	usb_attr.preempt_prio = 1;
	usb_attr.sub_prio = 3;
	usb_attr.use_present_pin = 1;
	usb_attr.present_port = _GPIOD;
	usb_attr.present_pin = 4;

    usb_ioctl(I_USB_SETATTR, &usb_attr);
    usb_ioctl(I_USB_CONNECTED, &usb_connected);
    usb_present = usb_connected;

    }
}

void FastSerial::begin(long baud, uint32_t tx_timeout) {
    assert_param((uint32)baud <= this->usart_device->max_baud);

    if ((uint32)baud > this->usart_device->max_baud) {
        return;
    }

    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];
    
	
    uint8 mode;
    if (this->usart_device->USARTx == USART1) mode = GPIO_AF_USART1;
    else if (this->usart_device->USARTx == USART2) mode = GPIO_AF_USART2;
    else if (this->usart_device->USARTx == USART3) mode = GPIO_AF_USART3;
    else if (this->usart_device->USARTx == UART4) mode = GPIO_AF_UART4;
    else if (this->usart_device->USARTx == UART5) mode = GPIO_AF_UART5;
    else if (this->usart_device->USARTx == USART6) mode = GPIO_AF_USART6;
    else 
    {
		assert_param(0);
		return;
    }
   
    gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, mode);
    gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);    
    gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, mode);
    gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_PP);

    usart_init(this->usart_device);
    usart_setup(this->usart_device, (uint32)baud, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, tx_timeout);
    usart_enable(this->usart_device);
}

void FastSerial::end(void) {
 if (usb == 0 )
    usart_disable(this->usart_device);
}

int FastSerial::available(void) {
    if (usb == 0)
	return usart_data_available(this->usart_device);
	else
	return usb_data_available();
}

int FastSerial::txspace(void)
{
    if (usb == 0)
	return txfifo_freebytes();
	else 
	return 256;

}

void FastSerial::use_tx_fifo(bool enable)
{
if (usb == 0)
	{
	usart_reset_tx(this->usart_device);
	usart_use_tx_fifo(this->usart_device, (enable ? 1 : 0));
	}
}
void FastSerial::set_blocking_writes(bool enable)
{
if (usb == 0)
	{

	usart_reset_tx(this->usart_device);
	usart_use_tx_fifo(this->usart_device, (enable ? 0 : 1));
	}
}

uint8_t FastSerial::get_blocking_writes(void)
{
if (usb == 0)
	{
	uint8_t blocking;
	blocking = (this->usart_device->usetxrb ? 0 : 1);
	return blocking;
	} else {
	    return 0;
	}
}

void FastSerial::use_timeout(uint8_t enable)
{
if (usb == 0)
	{

	usart_use_timeout(this->usart_device, enable);
	}
}

void FastSerial::set_timeout(uint32_t timeout)
{
if (usb == 0)
	{
	usart_set_timeout(usart_device, timeout);
	}
}

int FastSerial::read(void) {
    if (usb == 0)
    {
	if (available() <= 0)
	    return (-1);
	return usart_getc(this->usart_device);
    }
    else
    {
	if (usb_data_available() <= 0)
	    return (-1);
	return usb_getc();
    }
}

int FastSerial::peek(void)
{
if (usb == 0)
	{

	if (available() <= 0)
		return (-1);
	// pull character from tail
	return usart_getc(this->usart_device);
	}
	else
	{
	if (usb_data_available() <= 0)
		return (-1);

	// pull character from tail
	return usb_getc();
}

}

// return time elasped from last uart interrupt
// that functionality is good for sincronize the use of serial resource and buffer
long FastSerial::getPortLic(void){
long ret=0;
if (usb == 0)
	{
	if (this->usart_device->USARTx == USART1) ret =  uart1_lic_millis;
	if (this->usart_device->USARTx == USART2) ret =  uart2_lic_millis;
	if (this->usart_device->USARTx == USART3) ret =  uart3_lic_millis;
	if (this->usart_device->USARTx == UART4)  ret =  uart4_lic_millis;
	if (this->usart_device->USARTx == UART5)  ret =  uart5_lic_millis;
	}
return(ret);
}

void FastSerial::flush(void) {
if (usb == 0)
	{
    usart_reset_rx(this->usart_device);
    usart_reset_tx(this->usart_device);
	}
else
{
    //rb_reset(rxfifo); // reset the rxfifo on usb.
	usb_reset_rx();
    usb_reset_tx();
}
}
uint32_t FastSerial::txfifo_nbytes(void)
{
if (usb == 0)
	return usart_txfifo_nbytes(this->usart_device);
	else
	return(256);
}
uint32_t FastSerial::txfifo_freebytes(void)
{
if (usb == 0)
	return usart_txfifo_freebytes(this->usart_device);
	else
	return(256);
}

void FastSerial::write(uint8_t ch) {
    if (usb == 0)
	usart_putc(this->usart_device, ch);
	else
	usb_putc(ch);
}

void FastSerial::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
	begin(baud);
}

