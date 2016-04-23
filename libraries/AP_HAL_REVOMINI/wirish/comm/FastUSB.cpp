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

#include "FastUSB.h"
#include <usb.h>
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

FastUSB::FastUSB()
{
}

FastUSB::FastUSB(usart_dev *usart_device,
                       uint8 tx_pin,
                       uint8 rx_pin) {
/*
	this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
*/
}

void FastUSB::init(usart_dev *usart_device,
                       uint8 tx_pin,
                       uint8 rx_pin)
{
	/*this->usart_device = usart_device;
	this->tx_pin = tx_pin;
	this->rx_pin = rx_pin;
*/
}

void FastUSB::configure(uint8 port)
{

}

#define disable_timer_if_necessary(dev, ch) ((void)0)

void FastUSB::begin(long baud) {
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

}

void FastUSB::begin(long baud, uint32_t tx_timeout) {

    usb_open();
    
    usb_default_attr(&usb_attr);
	usb_attr.preempt_prio = 1;
	usb_attr.sub_prio = 3;
	usb_attr.use_present_pin = 1;
	usb_attr.present_port = _GPIOD;
	usb_attr.present_pin = 4;

    usb_ioctl(I_USB_SETATTR, &usb_attr);
    usb_ioctl(I_USB_CONNECTED, &usb_connected);

}

void FastUSB::end(void) {
    //usart_disable(this->usart_device);
}

int FastUSB::available(void) {
    return usb_data_available();
}

int FastUSB::txspace(void)
{
    return 0;

}

void FastUSB::use_tx_fifo(bool enable)
{

}
void FastUSB::set_blocking_writes(bool enable)
{
	//usart_reset_tx(this->usart_device);
	//usart_use_tx_fifo(this->usart_device, (enable ? 0 : 1));
}

void FastUSB::use_timeout(uint8_t enable)
{
	//usart_use_timeout(this->usart_device, enable);
}

void FastUSB::set_timeout(uint32_t timeout)
{
	//usart_set_timeout(usart_device, timeout);
}

int FastUSB::read(void) {
	if (available() <= 0)
		return (-1);
    return usb_getc();
}

int FastUSB::peek(void)
{
	if (available() <= 0)
		return (-1);

	// pull character from tail
	return usb_getc();
}

// return time elasped from last uart interrupt
// that functionality is good for sincronize the use of serial resource and buffer
long FastUSB::getPortLic(void){
long ret=0;
/*
if (this->usart_device->USARTx == USART1) ret =  uart1_lic_millis;
if (this->usart_device->USARTx == USART2) ret =  uart2_lic_millis;
if (this->usart_device->USARTx == USART3) ret =  uart3_lic_millis;
if (this->usart_device->USARTx == UART4)  ret =  uart4_lic_millis;
if (this->usart_device->USARTx == UART5)  ret =  uart5_lic_millis;
*/
return(ret);
}

void FastUSB::flush(void) {
    //rb_reset(rxfifo); // reset the rxfifo on usb.
	usb_reset_rx();
    usb_reset_tx();
}
uint32_t FastUSB::txfifo_nbytes(void)
{
	return (0);//usart_txfifo_nbytes(this->usart_device);
}
uint32_t FastUSB::txfifo_freebytes(void)
{
	return (1);//usart_txfifo_freebytes(this->usart_device);
}

void FastUSB::write(uint8_t ch) {
	usb_putc(ch);
}

void FastUSB::begin(long baud, unsigned int rxSpace, unsigned int txSpace)
{
	begin(baud);
}
