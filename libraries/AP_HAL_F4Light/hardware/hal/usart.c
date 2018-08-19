/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: LeafLabs


*/

#pragma GCC optimize ("O2")

#include <usart.h>
#include <hal.h>
#include <systick.h>
/*
 * Devices
 */

static ring_buffer usart1_txrb IN_CCM;
static ring_buffer usart1_rxrb IN_CCM;
static usart_state u1state IN_CCM;

static const usart_dev usart1 = {
    .USARTx = USART1,
    .clk = RCC_APB2Periph_USART1,
    .txrb = &usart1_txrb,
    .rxrb = &usart1_rxrb,
    .state = &u1state,
    .irq = USART1_IRQn,
    .rx_pin = BOARD_USART1_RX_PIN,
    .tx_pin = BOARD_USART1_TX_PIN,
    .gpio_af = GPIO_AF_USART1
};
/** USART1 device */
const usart_dev * const _USART1 = &usart1;

#if defined(BOARD_USART2_RX_PIN) && defined(BOARD_USART2_RX_PIN)
static ring_buffer usart2_txrb IN_CCM;
static ring_buffer usart2_rxrb IN_CCM;
static usart_state u2state IN_CCM;

static const usart_dev usart2 = {
    .USARTx = USART2,
    .clk = RCC_APB1Periph_USART2,
    .txrb = &usart2_txrb,
    .rxrb = &usart2_rxrb,
    .state = &u2state,
    .irq = USART2_IRQn,
    .rx_pin = BOARD_USART2_RX_PIN,
    .tx_pin = BOARD_USART2_TX_PIN,
    .gpio_af = GPIO_AF_USART2
};
/** USART2 device */
const usart_dev * const _USART2 = &usart2;
#else 
#define _USART2 NULL
#endif

static ring_buffer usart3_txrb IN_CCM;
static ring_buffer usart3_rxrb IN_CCM;
static usart_state u3state IN_CCM;

static const usart_dev usart3 = {
    .USARTx = USART3,
    .clk = RCC_APB1Periph_USART3,
    .txrb = &usart3_txrb,
    .rxrb = &usart3_rxrb,
    .state = &u3state,
    .irq = USART3_IRQn,
    .rx_pin = BOARD_USART3_RX_PIN,
    .tx_pin = BOARD_USART3_TX_PIN,
    .gpio_af = GPIO_AF_USART3
};
/** USART3 device */
const usart_dev * const _USART3 = &usart3;

#if defined(BOARD_USART4_RX_PIN) && defined(BOARD_USART4_TX_PIN)
static ring_buffer uart4_txrb IN_CCM;
static ring_buffer uart4_rxrb IN_CCM;
static usart_state u4state IN_CCM;

static const usart_dev uart4 = {
    .USARTx = UART4,
    .clk = RCC_APB1Periph_UART4,
    .txrb = &uart4_txrb,
    .rxrb = &uart4_rxrb,
    .state = &u4state,
    .irq = UART4_IRQn,
    .rx_pin = BOARD_USART4_RX_PIN,
    .tx_pin = BOARD_USART4_TX_PIN,
    .gpio_af = GPIO_AF_UART4
};
/** UART4 device */
const usart_dev * const _UART4 = &uart4;
#endif

#if defined(BOARD_USART5_RX_PIN) && defined(BOARD_USART5_TX_PIN)
static ring_buffer uart5_txrb IN_CCM;
static ring_buffer uart5_rxrb IN_CCM;
static usart_state u5state IN_CCM;

static const usart_dev uart5 = {
    .USARTx = UART5,
    .clk = RCC_APB1Periph_UART5,
    .txrb = &uart5_txrb,  
    .rxrb = &uart5_rxrb,
    .state = &u5state,
    .irq = UART5_IRQn,
    .rx_pin = BOARD_USART5_RX_PIN,
    .tx_pin = BOARD_USART5_TX_PIN,
    .gpio_af = GPIO_AF_UART5
};
/* UART5 device */
const usart_dev * const _UART5 = &uart5;
#endif

#if defined(BOARD_USART6_RX_PIN) && defined(BOARD_USART6_TX_PIN)
static ring_buffer usart6_txrb IN_CCM;
static ring_buffer usart6_rxrb IN_CCM;
static usart_state u6state IN_CCM;

static const usart_dev usart6 =  {
    .USARTx = USART6,
    .clk = RCC_APB2Periph_USART6,
    .txrb = &usart6_txrb,
    .rxrb = &usart6_rxrb,
    .state = &u6state,
    .irq = USART6_IRQn,
    .rx_pin = BOARD_USART6_RX_PIN,
    .tx_pin = BOARD_USART6_TX_PIN,
    .gpio_af = GPIO_AF_USART6
};
/** UART6 device */
const usart_dev * const _USART6 = &usart6;
#endif

const usart_dev * const UARTS[] = {
    NULL,
    &usart1,
#if defined(BOARD_USART2_RX_PIN) && defined(BOARD_USART2_RX_PIN)
    &usart2,
#else
    NULL, 
#endif
    &usart3,
#if defined(BOARD_USART4_RX_PIN) && defined(BOARD_USART4_TX_PIN)
    &uart4,
#else
    NULL,
#endif
#if defined(BOARD_USART5_RX_PIN) && defined(BOARD_USART5_TX_PIN)
    &uart5,
#else
    NULL,
#endif
#if defined(BOARD_USART6_RX_PIN) && defined(BOARD_USART6_TX_PIN)
    &usart6,
#else
    NULL,
#endif
    
};

void usart_foreach(void (*fn)(const usart_dev*))
{
    fn(_USART1);
#if defined(BOARD_USART2_RX_PIN) && defined(BOARD_USART2_RX_PIN)
    fn(_USART2);
#endif
    fn(_USART3);
#if defined(BOARD_USART4_RX_PIN) && defined(BOARD_USART4_TX_PIN)
    fn(_UART4);
#endif
#if defined(BOARD_USART5_RX_PIN) && defined(BOARD_USART5_TX_PIN)
    fn(_UART5);
#endif
#if defined(BOARD_USART6_RX_PIN) && defined(BOARD_USART6_TX_PIN)
    fn(_USART6);
#endif
}

extern uint32_t us_ticks;

/**
 * @brief Initialize a serial port.
 * @param dev         Serial port to be initialized
 */
void usart_init(const usart_dev *dev)  {
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(dev->USARTx));

    // Turn on peripheral clocks
    if (dev->USARTx == USART1 || dev->USARTx == USART6 )
	RCC_APB2PeriphClockCmd(dev->clk, ENABLE);       // we must wait some time before access to
    else
	RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

}

void usart_setup(const usart_dev *dev, uint32_t baudRate, uint16_t wordLength,
	uint16_t stopBits, uint16_t parity, uint16_t mode, uint16_t hardwareFlowControl)
{
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(dev->USARTx));
    assert_param(IS_USART_BAUDRATE(baud));
    assert_param(IS_USART_STOPBITS(stopbits));
    assert_param(IS_USART_PARITY(parity));
    assert_param(IS_USART_WORD_LENGTH(wordLength));
    assert_param(IS_USART_MODE(mode));
    assert_param(IS_USART_HARDWARE_FLOW_CONTROL(hardwareFlowControl));

    memset(dev->state, 0, sizeof(*dev->state));

    dev->state->txbusy = 0;
    dev->state->callback = 0;

    /* Disable USARTx */
    usart_disable(dev);

    rb_init(dev->txrb, USART_TX_BUF_SIZE, dev->state->tx_buf);
    rb_init(dev->rxrb, USART_RX_BUF_SIZE, dev->state->rx_buf);

    /* Enables the USART's 8x oversampling mode. */
    USART_OverSampling8Cmd(dev->USARTx, ENABLE);

    USART_ClockInitTypeDef USART_InitClock;
    USART_ClockStructInit(&USART_InitClock);
    USART_ClockInit(dev->USARTx, &USART_InitClock);

    USART_InitTypeDef USART_config;
    USART_StructInit(&USART_config);
    USART_config.USART_BaudRate = baudRate;
    USART_config.USART_WordLength = wordLength;
    USART_config.USART_StopBits = stopBits;
    USART_config.USART_Parity = parity;
    USART_config.USART_Mode = mode;
    USART_config.USART_HardwareFlowControl = hardwareFlowControl;

    USART_Init(dev->USARTx, &USART_config);

    dev->USARTx->CR1 &= ~(USART_MASK_IDLEIE | USART_MASK_RXNEIE | USART_MASK_TCEIE | USART_MASK_TXEIE | USART_MASK_PEIE);
    dev->USARTx->CR2 &= ~(USART_MASK2_LBDIE);
    dev->USARTx->CR3 &= ~(USART_MASK3_CTSIE | USART_MASK3_EIE);
    
    if(mode & USART_Mode_Rx) { /* Enable Rx request */
        USART_ClearFlag(dev->USARTx, USART_FLAG_RXNE);
        dev->USARTx->CR1 |= USART_MASK_RXNEIE;
    }

    if(mode & USART_Mode_Tx) {
        USART_ClearFlag(dev->USARTx, USART_FLAG_TC);
    }    

    enable_nvic_irq(dev->irq, UART_INT_PRIORITY);
}



uint32_t usart_tx(const usart_dev *dev, const uint8_t *buf, uint32_t len)
{
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));
    assert_param(IS_USART_DATA(Data));

    uint32_t tosend = len;
    uint32_t sent = 0;

    while (tosend)    {
        if (rb_is_full(dev->txrb))
	    break;
	rb_insert(dev->txrb, *buf++);
	sent++;
	tosend--;
    }
    if (dev->state->txbusy == 0 && sent > 0)	    {
	dev->state->txbusy = 1;
        dev->USARTx->CR1 |= USART_MASK_TXEIE;
    }

    return sent;
}

void usart_putudec(const usart_dev *dev, uint32_t val) {
    char digits[12];
    int i = 0;

    do	{
	digits[i++] = val % 10 + '0';
	val /= 10;
    }  while (val > 0);

    while (--i >= 0){
	usart_putc(dev, digits[i]);
    }
}

/*
 * Interrupt handlers.
 */


static inline void usart_rx_irq(const usart_dev *dev)    {
#ifdef ISR_PERF
        uint32_t t=stopwatch_getticks();
#endif

	/* Check on Receive Data register Not Empty interrupt */
        uint16_t sr = dev->USARTx->SR;
	if( (sr & USART_F_RXNE) && (dev->USARTx->CR1 & USART_MASK_RXNEIE) ){
#ifdef USART_SAFE_INSERT
	    /* If the buffer is full and the user defines USART_SAFE_INSERT, ignore new bytes. */
	    rb_safe_insert(dev->rxrb, (uint8_t) dev->USARTx->DR);
#else
	    /* By default, push bytes around in the ring buffer. */
	    rb_push_insert(dev->rxrb, (uint8_t)dev->USARTx->DR);
#endif

            if(dev->state->callback) {
                revo_call_handler(dev->state->callback, (uint32_t)dev); 
            }
	}

        if( sr & USART_F_ORE ){
	    (void)dev->USARTx->DR; // cleared after reading sr, dr
	}

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif

}
    
static inline void usart_tx_irq(const usart_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=stopwatch_getticks();
#endif
    /* Check USART Transmit Data Register Empty Interrupt */
    uint16_t sr = dev->USARTx->SR;
    if( (sr & USART_F_TXE) && (dev->USARTx->CR1 & USART_MASK_TXEIE) ){

	if (dev->txrb && !rb_is_empty(dev->txrb))  {
	    dev->USARTx->DR = rb_remove(dev->txrb);
	    dev->state->txbusy = 1;
	} else   {
	    /* Disable the USART Transmit Data Register Empty Interrupt */
	    dev->USARTx->CR1 &= ~USART_MASK_TXEIE;
	    dev->state->txbusy = 0;
	    // nops needed to deactivate the irq before irq handler is left
            asm volatile("nop");
            asm volatile("nop");
        }
    }
#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif

}

void USART1_IRQHandler(void)
{
    usart_rx_irq(_USART1);
    usart_tx_irq(_USART1);
}

#if defined(BOARD_USART2_RX_PIN) && defined(BOARD_USART2_RX_PIN)
void USART2_IRQHandler(void)
{
    usart_rx_irq(_USART2);
    usart_tx_irq(_USART2);
}
#endif

void USART3_IRQHandler(void)
{
    usart_rx_irq(_USART3);
    usart_tx_irq(_USART3);
}

#if defined( BOARD_USART4_RX_PIN) && defined( BOARD_USART4_TX_PIN)
void UART4_IRQHandler(void)
{
    usart_rx_irq(_UART4);
    usart_tx_irq(_UART4);
}
#endif

#if defined( BOARD_USART5_RX_PIN) && defined( BOARD_USART5_TX_PIN)
void UART5_IRQHandler(void)
{
    usart_rx_irq(_UART5);
    usart_tx_irq(_UART5);
}
#endif

#if defined( BOARD_USART6_RX_PIN) && defined( BOARD_USART6_TX_PIN)
void USART6_IRQHandler(void)
{
    usart_rx_irq(_USART6);
    usart_tx_irq(_USART6);
}
#endif
