/*
  YUNEEC port by Maelok
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "UARTDriver.h"
#include <AP_Math.h>
#include <stdlib.h>

#include <stm32f37x.h>
#include <stm32f37x_usart.h>
#include <stm32f37x_gpio.h>
#include <stm32f37x_rcc.h>

using namespace YUNEEC;

#define MAX_USART_PORTS 3
YUNEECUARTDriver::Buffer __YUNEECUARTDriver__rxBuffer[MAX_USART_PORTS];
YUNEECUARTDriver::Buffer __YUNEECUARTDriver__txBuffer[MAX_USART_PORTS];

YUNEECUARTDriver::YUNEECUARTDriver(
		const uint8_t portNumber,
		USART_TypeDef *usart, GPIO_TypeDef* port, IRQn_Type usartIRQn,
		const uint32_t usartClk, const uint32_t portClk,
		const uint16_t rx_bit, const uint16_t tx_bit,
		const uint8_t rx_pinSource, const uint8_t tx_pinSource) :
		_usart_info{usart, port, usartIRQn, usartClk, portClk, rx_bit, tx_bit, rx_pinSource, tx_pinSource, 57600},
		_initialized(false),
		_rxBuffer(&__YUNEECUARTDriver__rxBuffer[portNumber]),
		_txBuffer(&__YUNEECUARTDriver__txBuffer[portNumber])
{}

void YUNEECUARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) {
	bool need_allocate = true;

	// If we are currently open...
	if (_open) {
		// If the caller wants to preserve the buffer sizing, work out what
		// it currently is...
		if (0 == rxSpace)
			rxSpace = _rxBuffer->mask + 1;
		if (0 == txSpace)
			txSpace = _txBuffer->mask + 1;

		if (rxSpace == (_rxBuffer->mask + 1U) && txSpace == (_txBuffer->mask + 1U)) {
			// avoid re-allocating the buffers if possible
			need_allocate = false;
			// disable USART and interrupt
			NVIC_DisableIRQ(_usart_info.usartIRQn);
			USART_Cmd(_usart_info.usart, DISABLE);

		} else {
			// close the port in its current configuration, clears _open
			end();
		}
	}

	if (need_allocate) {
		// allocate buffers
		if (!_allocBuffer(_rxBuffer, rxSpace ? rxSpace : _default_rx_buffer_size)
        || !_allocBuffer(_txBuffer, txSpace ? txSpace : _default_tx_buffer_size)) {
			end();
			return; // couldn't allocate buffers - fatal
		}
	}

	// reset buffer pointers
	_txBuffer->head = _txBuffer->tail = 0;
	_rxBuffer->head = _rxBuffer->tail = 0;

	// mark the port as open
	_open = true;

	// configure USART port
	if(baud != 0)
		_usart_info.baudrate = baud;

	_configPort(_usart_info);

	// Enable Rx Interrupt
	USART_ITConfig(_usart_info.usart, USART_IT_RXNE, ENABLE);

	// Enable USART
	USART_Cmd(_usart_info.usart, ENABLE);

	_initialized = true;

}

void YUNEECUARTDriver::end() {
	// disable interrupt and reset USART port
	NVIC_DisableIRQ(_usart_info.usartIRQn);
	USART_DeInit(_usart_info.usart);

	_freeBuffer(_rxBuffer);
	_freeBuffer(_txBuffer);

	_open = false;
}

void YUNEECUARTDriver::flush() {
	_rxBuffer->head = _rxBuffer->tail;
	_txBuffer->tail = _txBuffer->head;
}

bool YUNEECUARTDriver::is_initialized() {
	return _initialized;
}
void YUNEECUARTDriver::set_blocking_writes(bool blocking) {
    _nonblocking_writes = !blocking;
}
bool YUNEECUARTDriver::tx_pending() {
	return false;
}

/* YUNEEC implementations of Stream virtual methods */
int16_t YUNEECUARTDriver::available() {
	if (!_open)
		return (-1);
	return ((_rxBuffer->head - _rxBuffer->tail) & _rxBuffer->mask);
}

int16_t YUNEECUARTDriver::txspace() {
	if (!_open)
		return (-1);
	return ((_txBuffer->mask+1) - ((_txBuffer->head - _txBuffer->tail) & _txBuffer->mask));
}

int16_t YUNEECUARTDriver::read() {
	uint8_t c;

	// if the head and tail are equal, the buffer is empty
	if (!_open || (_rxBuffer->head == _rxBuffer->tail))
		return (-1);

	// pull character from tail
	c = _rxBuffer->bytes[_rxBuffer->tail];
	_rxBuffer->tail = (_rxBuffer->tail + 1) & _rxBuffer->mask;

	return (c);
}

/* YUNEEC implementations of Print virtual methods */
size_t YUNEECUARTDriver::write(uint8_t c) {
	uint8_t i;

	if (!_open) // drop bytes if not open
		return 0;

	// wait for room in the tx buffer
	i = (_txBuffer->head + 1) & _txBuffer->mask;

	// if the port is set into non-blocking mode, then drop the byte
	// if there isn't enough room for it in the transmit buffer
	if (_nonblocking_writes && i == _txBuffer->tail) {
		return 0;
	}

	while (i == _txBuffer->tail)
		;

	// add byte to the buffer
	_txBuffer->bytes[_txBuffer->head] = c;
	_txBuffer->head = i;

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	  USART_ITConfig(_usart_info.usart, USART_IT_TXE, ENABLE);

	// return number of bytes written (always 1)
	return 1;
}

size_t YUNEECUARTDriver::write(const uint8_t *buffer, size_t size)
{
	if (!_open) // drop bytes if not open
		return 0;

    if (!_nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    int16_t space = txspace();
    if (space <= 0) {
        return 0;
    }
    if (size > (size_t)space) {
        // throw away remainder if too much data
        size = space;
    }
    if (_txBuffer->tail > _txBuffer->head) {
        // perform as single memcpy
        memcpy(&_txBuffer->bytes[_txBuffer->head], buffer, size);
        _txBuffer->head = (_txBuffer->head + size) & _txBuffer->mask;
        // enable the data-ready interrupt, as it may be off if the buffer is empty
        USART_ITConfig(_usart_info.usart, USART_IT_TXE, ENABLE);
        return size;
    }

    // perform as two memcpy calls
    uint16_t n = (_txBuffer->mask+1) - _txBuffer->head;
    if (n > size) n = size;
    memcpy(&_txBuffer->bytes[_txBuffer->head], buffer, n);
    _txBuffer->head = (_txBuffer->head + n) & _txBuffer->mask;
    buffer += n;
    n = size - n;
    if (n > 0) {
        memcpy(&_txBuffer->bytes[0], buffer, n);
        _txBuffer->head = (_txBuffer->head + n) & _txBuffer->mask;
    }

    // enable the data-ready interrupt, as it may be off if the buffer is empty
    USART_ITConfig(_usart_info.usart, USART_IT_TXE, ENABLE);
    return size;
}

//-----------------------------------------------------------------
// USART config
//-----------------------------------------------------------------
void YUNEECUARTDriver::_configPort(const struct USART_Info &usart_info)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Reset register of USART  */
	USART_DeInit(usart_info.usart);

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(usart_info.portClk, ENABLE);

	/* Enable USART clock */
	if(usart_info.usart == USART1)
		RCC_APB2PeriphClockCmd(usart_info.usartClk, ENABLE);
	else
		RCC_APB1PeriphClockCmd(usart_info.usartClk, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(usart_info.port, usart_info.tx_pinSource, GPIO_AF_7);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(usart_info.port, usart_info.rx_pinSource, GPIO_AF_7);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = usart_info.tx_bit;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(usart_info.port, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = usart_info.rx_bit;
	GPIO_Init(usart_info.port, &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = usart_info.baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(usart_info.usart, &USART_InitStructure);

	/* Enable the USART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = usart_info.usartIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//-----------------------------------------------------------------
// Buffer management
//-----------------------------------------------------------------

bool YUNEECUARTDriver::_allocBuffer(Buffer *buffer, uint16_t size)
{
	uint8_t  mask;
	uint8_t	 shift;

	// init buffer state
	buffer->head = buffer->tail = 0;

	// Compute the power of 2 greater or equal to the requested buffer size
	// and then a mask to simplify wrapping operations.  Using __builtin_clz
	// would seem to make sense, but it uses a 256(!) byte table.
	// Note that we ignore requests for more than BUFFER_MAX space.
	for (shift = 1; (1U << shift) < min(_max_buffer_size, size); shift++)
		;
	mask = (1U << shift) - 1;

	// If the descriptor already has a buffer allocated we need to take
	// care of it.
	if (buffer->bytes) {

		// If the allocated buffer is already the correct size then
		// we have nothing to do
		if (buffer->mask == mask)
			return true;

		// Dispose of the old buffer.
		free(buffer->bytes);
	}
	buffer->mask = mask;

	// allocate memory for the buffer - if this fails, we fail.
	buffer->bytes = (uint8_t *) malloc(buffer->mask + (size_t)1);

	return (buffer->bytes != NULL);
}

void YUNEECUARTDriver::_freeBuffer(Buffer *buffer)
{
	buffer->head = buffer->tail = 0;
	buffer->mask = 0;
	if (NULL != buffer->bytes) {
		free(buffer->bytes);
		buffer->bytes = NULL;
	}
}

#endif
