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
    SPI_TypeDef* SPIx;          
    uint8_t      afio;
    IRQn_Type    irq;
    uint16_t     clock;
    Spi_DMA      dma;
    spi_state *  state;
} spi_dev;

extern const spi_dev * const _SPI1;
extern const spi_dev * const _SPI2;
extern const spi_dev * const _SPI3;

typedef enum spi_firstbit {
	LSBFIRST=0,
	MSBFIRST
} spi_firstbit;

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
    SPI_BAUD_PCLK_DIV_2   = ((uint16_t)0x0000), /**< f_PCLK/2 */
    SPI_BAUD_PCLK_DIV_4   = ((uint16_t)0x0008), /**< f_PCLK/4 */
    SPI_BAUD_PCLK_DIV_8   = ((uint16_t)0x0010), /**< f_PCLK/8 */
    SPI_BAUD_PCLK_DIV_16  = ((uint16_t)0x0018), /**< f_PCLK/16 */
    SPI_BAUD_PCLK_DIV_32  = ((uint16_t)0x0020), /**< f_PCLK/32 */
    SPI_BAUD_PCLK_DIV_64  = ((uint16_t)0x0028), /**< f_PCLK/64 */
    SPI_BAUD_PCLK_DIV_128 = ((uint16_t)0x0030), /**< f_PCLK/128 */
    SPI_BAUD_PCLK_DIV_256 = ((uint16_t)0x0038), /**< f_PCLK/256 */
} spi_baud_rate;


#define SPI_BIT_RXNE               ((uint16_t)0x0001)
#define SPI_BIT_TXE                ((uint16_t)0x0002)
#define SPI_BIT_CRCERR             ((uint16_t)0x0010)
#define SPI_BIT_MODF               ((uint16_t)0x0020)
#define SPI_BIT_OVR                ((uint16_t)0x0040)
#define SPI_BIT_BSY                ((uint16_t)0x0080)
#define SPI_BIT_TIFRFE             ((uint16_t)0x0100)

#define SPI_size_16b                ((uint16_t)0x0800)
#define SPI_size_8b                 ((uint16_t)0x0000)

#define SPI_DMAreq_Tx               ((uint16_t)0x0002)
#define SPI_DMAreq_Rx               ((uint16_t)0x0001)

/** Available SPI interrupts  - see SPI_I2S_GetITStatus */
typedef enum spi_interrupt {
    SPI_TXE_INTERRUPT  = 1<<7,   /**< TX buffer empty interrupt */
    SPI_RXNE_INTERRUPT = 1<<6,   /**< RX buffer not empty interrupt */
    SPI_ERR_INTERRUPT  = 1<<5,   /**< * Error interrupt (CRC, overrun,
                                      * and mode fault errors for SPI;
                                      * underrun, overrun errors for I2S)
                                      */
                                                                            
    SPI_RXNE_TXE_INTERRUPTS = SPI_RXNE_INTERRUPT | SPI_TXE_INTERRUPT,
    SPI_INTERRUPTS_ALL = SPI_TXE_INTERRUPT  | SPI_RXNE_INTERRUPT | SPI_ERR_INTERRUPT
} spi_interrupt;

/**
 * @brief Mask for all spi_interrupt values
 * @see spi_interrupt
 */
#define SPI_INTERRUPTS_ALL              (SPI_TXE_INTERRUPT  |           \
                                         SPI_RXNE_INTERRUPT |           \
                                         SPI_ERR_INTERRUPT)
                                         

#ifdef __cplusplus
  extern "C" {
#endif

/**
 * @brief Enable a SPI peripheral
 * @param dev Device to enable
 */
 

static inline void spi_peripheral_enable(const spi_dev *dev) {
    //    SPI_Cmd(dev->SPIx, ENABLE);
    dev->SPIx->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Disable a SPI peripheral
 * @param dev Device to disable
 */

static inline void spi_peripheral_disable(const spi_dev *dev) {
    //SPI_Cmd(dev->SPIx, DISABLE);
    dev->SPIx->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
}

void spi_reconfigure(const spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode);

/**
 * @brief Initialize and reset a SPI device.
 * @param dev Device to initialize and reset.
 */
void spi_init(const spi_dev *dev);

/**
 * @brief Configure GPIO bit modes for use as a SPI port's pins.
 * @param as_master If true, configure bits for use as a bus master.
 *                  Otherwise, configure bits for use as slave.
 * @param nss_dev NSS pin's GPIO device
 * @param comm_dev SCK, MISO, MOSI pins' GPIO device
 * @param nss_bit NSS pin's GPIO bit on nss_dev
 * @param sck_bit SCK pin's GPIO bit on comm_dev
 * @param miso_bit MISO pin's GPIO bit on comm_dev
 * @param mosi_bit MOSI pin's GPIO bit on comm_dev
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
 * @param baud Bus baud rate
 * @param mode SPI mode
 * @param flags Logical OR of spi_cfg_flag values.
 * @see spi_cfg_flag
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
    return dev->SPIx->CR1 & SPI_CR1_SPE;
}

static inline void spi_peripheral_disable_all(void) {
    spi_foreach(spi_peripheral_disable);
}


//[ new ones - for enum spi_interrupt
static inline void spi_enable_irq(const spi_dev *dev, spi_interrupt interrupt_flags) {
    dev->SPIx->CR2 |= interrupt_flags;
}

static inline void spi_disable_irq(const spi_dev *dev, spi_interrupt interrupt_flags) {
    dev->SPIx->CR2 &= ~interrupt_flags;
}


static inline bool spi_is_irq_enabled(const spi_dev *dev, uint32_t interrupt_flags) {
    return dev->SPIx->CR2 & interrupt_flags;
}
//]

static inline uint16_t spi_dff(const spi_dev *dev) {
    return ((dev->SPIx->CR1 & SPI_size_16b) == SPI_size_8b ? SPI_size_8b : SPI_size_16b);
}

static inline uint8_t spi_is_rx_nonempty(const spi_dev *dev) {
    return (dev->SPIx->SR & SPI_BIT_RXNE);
}

static inline uint8_t spi_rx_reg(const spi_dev *dev) {
    return (uint8_t)dev->SPIx->DR;
}

static inline uint8_t spi_is_tx_empty(const spi_dev *dev) {
    return (dev->SPIx->SR & SPI_BIT_TXE);
}

static inline void spi_tx_reg(const spi_dev *dev, uint8_t val) {
    dev->SPIx->DR = val;
}

static inline uint8_t spi_is_busy(const spi_dev *dev) {
    return (dev->SPIx->SR & SPI_BIT_BSY);
}

static inline void spi_wait_busy(const spi_dev *dev) {
// Wait until the transfer is complete - to not disable CS too early 
    uint32_t dly=3000;
    while (dev->SPIx->SR & SPI_BIT_BSY){ // but datasheet prohibits this usage
        dly--;
        if(dly==0) break;
    }
}

static inline void spi_enable_dma_req(const spi_dev *dev, uint16_t SPI_DMAReq) {
    /* Enable the selected SPI DMA requests */
    dev->SPIx->CR2 |= SPI_DMAReq;
}

static inline void spi_disable_dma_req(const spi_dev *dev, uint16_t SPI_DMAReq) {
    /* Disable the selected SPI DMA requests */
    dev->SPIx->CR2 &= (uint16_t)~SPI_DMAReq;
}


static inline void spi_attach_interrupt(const spi_dev *dev, Handler handler){
    dev->state->handler = handler;
    
    IRQn_Type irq=dev->irq;
    
//    NVIC_ClearPendingIRQ(irq);
//    NVIC_EnableIRQ(irq);
//    NVIC_SetPriority(irq, SPI_INT_PRIORITY); 
    enable_nvic_irq(irq, SPI_INT_PRIORITY); 
}

static inline void spi_detach_interrupt(const spi_dev *dev){
    dev->state->handler = 0;
}
                                       
#ifdef __cplusplus
  }
#endif

#endif
