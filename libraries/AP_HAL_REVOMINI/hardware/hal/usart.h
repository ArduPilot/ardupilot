#ifndef _USART_H
#define _USART_H

#include <stm32f4xx.h>
#include "ring_buffer.h"
#include <hal_types.h>
#include <revomini_MP32V1F4.h>

/*
 * Devices
 */

#ifndef USART_RX_BUF_SIZE
#define USART_RX_BUF_SIZE               512
#endif

#ifndef USART_TX_BUF_SIZE
#define USART_TX_BUF_SIZE               512
#endif

/** USART device type */
typedef struct usart_dev {
    USART_TypeDef* USARTx;             /**< Register map */
    uint32_t clk;
    ring_buffer *rxrb;                 /**< RX ring buffer */
    ring_buffer *txrb;                 /**< TX ring buffer */
    uint8_t txbusy:6;
    uint8_t usetxrb:1;
    uint8_t use_timeout:1;
    uint32_t max_baud;                 /**< Maximum baud */
    uint8_t rx_buf[USART_RX_BUF_SIZE]; /**< @brief Deprecated.
                                      * Actual RX buffer used by rb.
                                      * This field will be removed in
                                      * a future release. */
    uint8_t tx_buf[USART_TX_BUF_SIZE];
    IRQn_Type irq;
    uint32_t tx_timeout;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint8_t gpio_af;
} usart_dev;

#ifdef __cplusplus
  extern "C" {
#endif

extern usart_dev *_USART1;
extern usart_dev *_USART2;
extern usart_dev *_USART3;
extern usart_dev *_UART4;
extern usart_dev *_UART5;
extern usart_dev *_USART6;

/**
 * @brief Initialize a serial port.
 * @param dev         Serial port to be initialized
 */
void usart_init(usart_dev *dev);

 
/*
	USART_Word_Length 
	-----------------
	USART_WordLength_8b
	USART_WordLength_9b  

	USART_Stop_Bits
	---------------
	USART_StopBits_1
	USART_StopBits_0_5
	USART_StopBits_2
	USART_StopBits_1_5   

	USART_Parity
	------------
	USART_Parity_No
	USART_Parity_Even
	USART_Parity_Odd 

	USART_Mode
	----------
	USART_Mode_Rx
	USART_Mode_Tx  

	USART_Hardware_Flow_Control
	---------------------------
	USART_HardwareFlowControl_None
	USART_HardwareFlowControl_RTS
	USART_HardwareFlowControl_CTS
	USART_HardwareFlowControl_RTS_CTS  
*/

/**
 * @brief Configure a serial port's baud rate.
 *
 * @param dev         			Serial port to be configured
 * @param baudRate    			Baud rate for transmit/receive.
 * @param wordLength  			Specifies the number of data bits transmitted or received in a frame. This parameter can be a value of USART_Word_Length 
 * @param stopBits				Specifies the number of stop bits transmitted. This parameter can be a value of USART_Stop_Bits 
 * @param parity				Specifies the parity mode. This parameter can be a value of USART_Parity 
 * @param mode					Specifies wether the Receive or Transmit mode is enabled or disabled. This parameter can be a value of USART_Mode 
 * @param hardwareFlowControl	Specifies wether the hardware flow control mode is enabled or disabled. This parameter can be a value of USART_Hardware_Flow_Control
 */
void usart_setup(usart_dev *dev, 
				 uint32_t  baudRate, 
				 uint16_t  wordLength, 
				 uint16_t  stopBits, 
				 uint16_t  parity, 
				 uint16_t  mode, 
				 uint16_t  hardwareFlowControl,
				 uint32_t  tx_timeout);

/**
 * @brief Enable a serial port.
 *
 * USART is enabled in single buffer transmission mode, multibuffer
 * receiver mode, 8n1.
 *
 * Serial port must have a baud rate configured to work properly.
 *
 * @param dev Serial port to enable.
 * @see usart_set_baud_rate()
 */
void usart_enable(usart_dev *dev);

/**
 * @brief Turn off a serial port.
 * @param dev Serial port to be disabled
 */
void usart_disable(usart_dev *dev);

/**
 *  @brief Call a function on each USART.
 *  @param fn Function to call.
 */
void usart_foreach(void (*fn)(usart_dev*));

static inline void usart_use_tx_fifo(usart_dev *dev, uint8_t enable) {
	if (enable > 0)
	{
		dev->usetxrb=1;
	} else {
		dev->usetxrb=0;
	}
}

static inline void usart_use_timeout(usart_dev *dev, uint8_t enable) {
	if (enable > 0)
	{
		dev->use_timeout=1;
	} else {
		dev->use_timeout=0;
	}
}

static inline void usart_set_timeout(usart_dev *dev, uint32_t timeout) {
	dev->tx_timeout = timeout;
}

static inline uint32_t usart_txfifo_nbytes(usart_dev *dev) {
	return rb_full_count(dev->txrb);
}
static inline uint32_t usart_txfifo_freebytes(usart_dev *dev) {
	return (USART_TX_BUF_SIZE-rb_full_count(dev->txrb));
}
/**
 * @brief Disable all serial ports.
 */
static inline void usart_disable_all(void) {
    usart_foreach(usart_disable);
}

/**
 * @brief Nonblocking USART transmit
 * @param dev Serial port to transmit over
 * @param buf Buffer to transmit
 * @param len Maximum number of bytes to transmit
 * @return Number of bytes transmitted
 */
uint32_t usart_tx(usart_dev *dev, const uint8_t *buf, uint32_t len);

/**
 * @brief Transmit an unsigned integer to the specified serial port in
 *        decimal format.
 *
 * This function blocks until the integer's digits have been
 * completely transmitted.
 *
 * @param dev Serial port to send on
 * @param val Number to print
 */
void usart_putudec(usart_dev *dev, uint32_t val);

/**
 * @brief Transmit one character on a serial port.
 *
 * This function blocks until the character has been successfully
 * transmitted.
 *
 * @param dev Serial port to send on.
 * @param byte Byte to transmit.
 */
static inline void usart_putc(usart_dev* dev, uint8_t bt) {
	//uint32_t ret=0;
	//uint32_t cnt=0;
	//if (!usart_tx(dev, &byte, 1))
	//;
	usart_tx(dev, &bt, 1);
}

/**
 * @brief Transmit a character string on a serial port.
 *
 * This function blocks until str is completely transmitted.
 *
 * @param dev Serial port to send on
 * @param str String to send
 */
static inline void usart_putstr(usart_dev *dev, const char* str) {
    uint32_t i = 0;
    while (str[i] != '\0') {
        usart_putc(dev, str[i++]);
    }
}

/**
 * @brief Read one character from a serial port.
 *
 * It's not safe to call this function if the serial port has no data
 * available.
 *
 * @param dev Serial port to read from
 * @return byte read
 * @see usart_data_available()
 */
static inline uint8_t usart_getc(usart_dev *dev) {
    return rb_remove(dev->rxrb);
}

/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint32_t usart_data_available(usart_dev *dev) {
    return rb_full_count(dev->rxrb);
}


/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(usart_dev *dev) {
    rb_reset(dev->rxrb);
}

static inline void usart_reset_tx(usart_dev *dev) {
    rb_reset(dev->txrb);
    dev->txbusy = 0;
}



#ifdef __cplusplus
  }
#endif

#endif
