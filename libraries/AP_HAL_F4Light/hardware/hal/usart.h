#ifndef _USART_H
#define _USART_H

#include <hal_types.h>
#include "ring_buffer.h"

/*
 * Devices
 */

#ifndef USART_RX_BUF_SIZE
#define USART_RX_BUF_SIZE               512
#endif

#ifndef USART_TX_BUF_SIZE
#define USART_TX_BUF_SIZE               1024
#endif

typedef void (* usart_cb)();

typedef struct usart_state {
    Handler callback;

    uint8_t txbusy;

    uint8_t rx_buf[USART_RX_BUF_SIZE];
    uint8_t tx_buf[USART_TX_BUF_SIZE];

    uint8_t is_used; // flag that USART is used

} usart_state;

/** USART device type */
typedef struct usart_dev {
    USART_TypeDef* USARTx;             /**< Register map */
    uint32_t clk;
    IRQn_Type irq;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint8_t gpio_af;
    usart_state *state;
    ring_buffer *rxrb;                 /**< RX ring buffer */
    ring_buffer *txrb;                 /**< TX ring buffer */
} usart_dev;


extern const usart_dev * const UARTS[];

#ifdef __cplusplus
  extern "C" {
#endif

extern const usart_dev * const _USART1;
extern const usart_dev * const _USART2; 
extern const usart_dev * const _USART3;
extern const usart_dev * const _UART4;
extern const usart_dev * const _UART5;
extern const usart_dev * const _USART6;

#define USART_F_RXNE 0x20
#define USART_F_TXE  0x80
#define USART_F_ORE  0x8

#define USART_MASK_IDLEIE 0x10
#define USART_MASK_RXNEIE 0x20
#define USART_MASK_TCEIE  0x40
#define USART_MASK_TXEIE  0x80
#define USART_MASK_PEIE   0x100

#define USART_MASK2_LBDIE  0x40

#define USART_MASK3_CTSIE  0x400
#define USART_MASK3_EIE  0x1

#define UART_Mode_Rx      (0x0004)
#define UART_Mode_Tx      (0x0008)

#define UART_HardwareFlowControl_None       (0x0000)
#define UART_HardwareFlowControl_RTS        (0x0100)
#define UART_HardwareFlowControl_CTS        (0x0200)
#define UART_HardwareFlowControl_RTS_CTS    (0x0300)

#define UART_Word_8b                  ((uint16_t)0x0000)
#define UART_Word_9b                  ((uint16_t)0x1000)

/**
 * @brief Initialize a serial port.
 * @param dev         Serial port to be initialized
 */
void usart_init(const usart_dev *dev);

 
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
void usart_setup(const usart_dev *dev, 
				 uint32_t  baudRate, 
				 uint16_t  wordLength, 
				 uint16_t  stopBits, 
				 uint16_t  parity, 
				 uint16_t  mode, 
				 uint16_t  hardwareFlowControl);

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
static inline void usart_enable(const usart_dev *dev)
{
    dev->state->is_used=true;

    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(dev->USARTx));

    /* Enable USART */
//    USART_Cmd(dev->USARTx, ENABLE);
    
    dev->USARTx->CR1 |= USART_CR1_UE; /* Enable the selected USART by setting the UE bit in the CR1 register */

}


static inline uint8_t usart_is_used(const usart_dev *dev)
{
    return dev->state->is_used;
}


/**
 * @brief Turn off a serial port.
 * @param dev Serial port to be disabled
 */
static inline void usart_disable(const usart_dev *dev){
    /* Disable the selected USART by clearing the UE bit in the CR1 register */
    dev->USARTx->CR1 &= (uint16_t)~((uint16_t)USART_CR1_UE);
    
    /* Clean up buffer */
    dev->state->is_used=false;
}

/**
 *  @brief Call a function on each USART.
 *  @param fn Function to call.
 */
void usart_foreach(void (*fn)(const usart_dev*));


static inline uint32_t usart_txfifo_nbytes(const usart_dev *dev) {
	return rb_full_count(dev->txrb);
}
static inline uint32_t usart_txfifo_freebytes(const usart_dev *dev) {
        uint32_t used = rb_full_count(dev->txrb);
        if(used >= USART_TX_BUF_SIZE/2) return 0; // leave half for dirty writes without check - thanks s_s
	return (USART_TX_BUF_SIZE/2-used);
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
uint32_t usart_tx(const usart_dev *dev, const uint8_t *buf, uint32_t len);

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
void usart_putudec(const usart_dev *dev, uint32_t val);

/**
 * @brief Transmit one character on a serial port.
 *
 * This function blocks until the character has been successfully
 * transmitted.
 *
 * @param dev Serial port to send on.
 * @param byte Byte to transmit.
 */
static inline uint32_t usart_putc(const usart_dev* dev, uint8_t bt) {
	//uint32_t ret=0;
	//uint32_t cnt=0;
	//if (!usart_tx(dev, &byte, 1))
	//;
	return usart_tx(dev, &bt, 1);
}

/**
 * @brief Transmit a character string on a serial port.
 *
 * This function blocks until str is completely transmitted.
 *
 * @param dev Serial port to send on
 * @param str String to send
    used to inform about exception 

 */


static inline void usart_putstr(const usart_dev *dev, const char* str) {
    uint32_t i = 0;
    uint8_t c;
    while ( (c=str[i++]) )  usart_putc(dev, c);
    
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
static inline uint8_t usart_getc(const usart_dev *dev) {
    return rb_remove(dev->rxrb);
}

/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint32_t usart_data_available(const usart_dev *dev) {
    return rb_full_count(dev->rxrb);
}


/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(const usart_dev *dev) {
    rb_reset(dev->rxrb);
}

static inline void usart_reset_tx(const usart_dev *dev) {
    rb_reset(dev->txrb);
    dev->state->txbusy = 0;
}

static inline void usart_set_callback(const usart_dev *dev, Handler cb)
{
    dev->state->callback = cb;
}


void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void USART4_IRQHandler(void);
void USART5_IRQHandler(void);
void USART6_IRQHandler(void);

void UART4_IRQHandler(void);
void UART5_IRQHandler(void);


#ifdef __cplusplus
  }
#endif

#endif
