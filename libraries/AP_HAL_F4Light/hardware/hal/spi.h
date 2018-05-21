#ifndef _SPI_H_
#define _SPI_H_


#include "gpio_hal.h"
#include <stm32f4xx.h>
#include "dma.h"
#include "nvic.h"


/*
 * Devices
 */

typedef struct SPI_DMA {
    uint32_t   channel;
    dma_stream stream_rx;
    dma_stream stream_tx;
} Spi_DMA;

typedef struct SPI_state {
    Handler        handler;
    uint8_t *      dst;
    uint16_t       len;
    volatile bool  busy;
} spi_state;

/** SPI device type */
typedef struct spi_dev {
    SPI_TypeDef* regs;          
    uint8_t      afio;
    IRQn_Type    irq;
    uint16_t     clock;
    Spi_DMA      dma;
    spi_state *  state;
} spi_dev;

extern const spi_dev * const _SPI1;
extern const spi_dev * const _SPI2;
extern const spi_dev * const _SPI3;


/**
 * @brief SPI mode configuration.
 *
 * Determines a combination of clock polarity (CPOL), which determines
 * idle state of the clock line, and clock phase (CPHA), which
 * determines which clock edge triggers data capture.
 */
typedef enum spi_mode {
    SPI_MODE_0=0,  /**< Clock line idles low (0), data capture on first
                    clock transition. */
    SPI_MODE_1,  /**< Clock line idles low (0), data capture on second
                    clock transition */
    SPI_MODE_2,  /**< Clock line idles high (1), data capture on first
                    clock transition. */
    SPI_MODE_3   /**< Clock line idles high (1), data capture on
                    second clock transition. */
} spi_mode;

/**
 * @brief SPI baud rate configuration, as a divisor of f_PCLK, the
 *        PCLK clock frequency.
 */
 
typedef enum spi_baud_rate {
    SPI_BAUD_PCLK_DIV_2   = (0),    // f_PCLK/2
    SPI_BAUD_PCLK_DIV_4   = (1<<3), // f_PCLK/4
    SPI_BAUD_PCLK_DIV_8   = (2<<3), // f_PCLK/8
    SPI_BAUD_PCLK_DIV_16  = (3<<3), // f_PCLK/16
    SPI_BAUD_PCLK_DIV_32  = (4<<3), // f_PCLK/32
    SPI_BAUD_PCLK_DIV_64  = (5<<3), // f_PCLK/64
    SPI_BAUD_PCLK_DIV_128 = (6<<3), // f_PCLK/128
    SPI_BAUD_PCLK_DIV_256 = (7<<3), // f_PCLK/256
} spi_baud_rate;



// SR1 bits
#define SPI_BIT_RXNE               (1<<0)
#define SPI_BIT_TXE                (1<<1)
#define SPI_BIT_CHSIDE             (1<<2)
#define SPI_BIT_UDR                (1<<3)
#define SPI_BIT_CRCERR             (1<<4)
#define SPI_BIT_MODF               (1<<5)
#define SPI_BIT_OVR                (1<<6)
#define SPI_BIT_BSY                (1<<7)
#define SPI_BIT_TIFRFE             (1<<8)


// CR2 bits
#define SPI_DMAreq_Tx              (1<<1)
#define SPI_DMAreq_Rx              (1<<0)

// Available SPI interrupts in CR2
typedef enum spi_interrupt {
    SPI_TXE_INTERRUPT  = 1<<7,   // TX buffer empty interrupt
    SPI_RXNE_INTERRUPT = 1<<6,   // RX buffer not empty interrupt
    SPI_ERR_INTERRUPT  = 1<<5,   // Error interrupt (CRC, overrun, and mode fault errors for SPI; underrun, overrun errors for I2S)
                                     
                                                                            
    SPI_RXNE_TXE_INTERRUPTS = SPI_RXNE_INTERRUPT | SPI_TXE_INTERRUPT,
    SPI_INTERRUPTS_ALL = SPI_TXE_INTERRUPT  | SPI_RXNE_INTERRUPT | SPI_ERR_INTERRUPT
} spi_interrupt;


// CR1 bits
#define SPI_BIT_CPHA                  (1<<0)
#define SPI_BIT_CPOL                  (1<<1)
#define SPI_BIT_MSTR                  (1<<2)
// 3..5 - BR
#define SPI_BIT_SPE                   (1<<6)
#define SPI_BIT_LSBFIRST              (1<<7)
#define SPI_BIT_SSI                   (1<<8)
#define SPI_BIT_SSM                   (1<<9)
#define SPI_BIT_RXONLY                (1<<10)
#define SPI_BIT_DFF                   (1<<11)
#define SPI_BIT_CRCNEXT               (1<<12)
#define SPI_BIT_CRCEN                 (1<<13)
#define SPI_BIT_BIDIOE                (1<<14)
#define SPI_BIT_BIDIMODE              (1<<15)


// modes
#define SPI_CPHA_1Edge                (0)
#define SPI_CPHA_2Edge                (SPI_BIT_CPHA)

#define SPI_CPOL_Low                  (0)
#define SPI_CPOL_High                 (SPI_BIT_CPOL)


#define SPI_Mode_Master               (SPI_BIT_MSTR | SPI_BIT_SSI)
#define SPI_Mode_Slave                (0)
#define SPI_size_16b                  (SPI_BIT_DFF)
#define SPI_size_8b                   (0)

#define SPI_FirstBit_MSB              (0)
#define SPI_FirstBit_LSB              (SPI_BIT_LSBFIRST)

#define SPI_NSS_Soft                  (SPI_BIT_SSM)
#define SPI_NSS_Hard                  (0)

#define SPI_CR1_CLEAR_MASK            (SPI_BIT_SPE) // clears all excluding Enable
#define I2SCFGR_CLEAR_MASK            (0xF040) // don't touch reserved bits

#define SPI_Direction_2Lines_FullDuplex (0)
#define SPI_Direction_2Lines_RxOnly     (SPI_BIT_RXONLY)
#define SPI_Direction_1Line_Rx          (SPI_BIT_BIDIMODE)
#define SPI_Direction_1Line_Tx          (SPI_BIT_BIDIMODE | SPI_BIT_BIDIOE)



#ifdef __cplusplus
  extern "C" {
#endif

/**
 * @brief Enable a SPI peripheral
 * @param dev Device to enable
 */
 

static inline void spi_peripheral_enable(const spi_dev *dev) {
    dev->regs->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Disable a SPI peripheral
 * @param dev Device to disable
 */

static inline void spi_peripheral_disable(const spi_dev *dev) {
    dev->regs->CR1 &= (uint16_t)~SPI_CR1_SPE;
}

void spi_reconfigure(const spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode);

/**
 * Initialize and reset a SPI device.
 * dev Device to initialize and reset.
 */
void spi_init(const spi_dev *dev);

/**
 * @brief Configure GPIO bit modes for use as a SPI port's pins.
 * @param as_master If true, configure bits for use as a bus master.
 *                  Otherwise, configure bits for use as slave.
 */

static inline void spi_master_enable(const spi_dev *dev,
                       spi_baud_rate baudPrescaler,
                       spi_mode mode,
                       uint16_t bitorder)
{
    spi_reconfigure(dev, 1, baudPrescaler, bitorder, mode);
}

void spi_gpio_slave_cfg(const spi_dev *dev,
                  const gpio_dev *comm_dev, uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit);
                  
/**
 * @brief Configure and enable a SPI device as bus master.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 * @param dev Device to configure as bus master
 */
 
void spi_gpio_master_cfg(const spi_dev *dev,
                  const gpio_dev *comm_dev,
                  uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit);

                       
/**
 * @brief Configure and enable a SPI device as a bus slave.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 * @param dev Device to configure as a bus slave
 * @param mode SPI mode
 * @param flags Logical OR of spi_cfg_flag values.
 * @see spi_cfg_flag
 */
void spi_slave_enable(const spi_dev *dev,
                      spi_mode mode,
                      uint16_t bitorder);


void spi_set_speed(const spi_dev *dev, uint16_t baudPrescaler);                   


void spi_foreach(void (*fn)(const spi_dev*));

uint32_t spi_tx(const spi_dev *dev, const void *buf, uint16_t len);

int spimaster_transfer(const spi_dev *dev,
                       const uint8_t *txbuf,
                       uint16_t txcount,
                       uint8_t *rxbuf,
                       uint16_t rxcount);
                       
                       

static inline uint8_t spi_is_enabled(const spi_dev *dev) {
    return dev->regs->CR1 & SPI_CR1_SPE;
}

static inline void spi_peripheral_disable_all(void) {
    spi_foreach(spi_peripheral_disable);
}

static inline void spi_enable_irq(const spi_dev *dev, spi_interrupt interrupt_flags) {
    dev->regs->CR2 |= interrupt_flags;
}

static inline void spi_disable_irq(const spi_dev *dev, spi_interrupt interrupt_flags) {
    dev->regs->CR2 &= ~interrupt_flags;
}

static inline bool spi_is_irq_enabled(const spi_dev *dev, uint32_t interrupt_flags) {
    return dev->regs->CR2 & interrupt_flags;
}

// returns SPI ord size
static inline uint16_t spi_dff(const spi_dev *dev) {
    return (dev->regs->CR1 & SPI_size_16b) == SPI_size_8b ? SPI_size_8b : SPI_size_16b;
}

static inline uint8_t spi_is_rx_nonempty(const spi_dev *dev) {
    return (dev->regs->SR & SPI_BIT_RXNE);
}

static inline uint8_t spi_rx_reg(const spi_dev *dev) {
    return (uint8_t)dev->regs->DR;
}

static inline uint8_t spi_is_tx_empty(const spi_dev *dev) {
    return (dev->regs->SR & SPI_BIT_TXE);
}

static inline void spi_tx_reg(const spi_dev *dev, uint8_t val) {
    dev->regs->DR = val;
}

static inline uint8_t spi_is_busy(const spi_dev *dev) {
    return (dev->regs->SR & SPI_BIT_BSY);
}

static inline void spi_wait_busy(const spi_dev *dev) {// Wait until the transfer is complete - to not disable CS too early 
    uint32_t dly=3000;
    while (dev->regs->SR & SPI_BIT_BSY){ // but datasheet prohibits this usage
        dly--;
        if(dly==0) break;
    }
}

static inline void spi_enable_dma_req(const spi_dev *dev, uint16_t SPI_DMAReq) {   // Enable the selected SPI DMA requests 
    dev->regs->CR2 |= SPI_DMAReq;
}

static inline void spi_disable_dma_req(const spi_dev *dev, uint16_t SPI_DMAReq) {    // Disable the selected SPI DMA requests
    dev->regs->CR2 &= (uint16_t)~SPI_DMAReq;
}

static inline void spi_attach_interrupt(const spi_dev *dev, Handler handler){
    dev->state->handler = handler;

    enable_nvic_irq(dev->irq, SPI_INT_PRIORITY); 
}

static inline void spi_detach_interrupt(const spi_dev *dev){
    dev->state->handler = 0;
}
                                       
#ifdef __cplusplus
  }
#endif

#endif
