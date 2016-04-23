#ifndef _SPI_H_
#define _SPI_H_

#include <gpio_hal.h>
#include <stm32f4xx.h>
#include "ring_buffer.h"


/*
 * Devices
 */

/** SPI device type */
typedef struct spi_dev {
    SPI_TypeDef* SPIx;          
    uint8_t afio;
    IRQn_Type irq;
} spi_dev;

extern spi_dev *_SPI1;
extern spi_dev *_SPI2;
extern spi_dev *_SPI3;

typedef enum spi_firstbit {
	LSBFIRST,
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
    SPI_MODE_0,  /**< Clock line idles low (0), data capture on first
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
    SPI_BAUD_PCLK_DIV_2   = SPI_BaudRatePrescaler_2,   /**< f_PCLK/2 */
    SPI_BAUD_PCLK_DIV_4   = SPI_BaudRatePrescaler_4,   /**< f_PCLK/4 */
    SPI_BAUD_PCLK_DIV_8   = SPI_BaudRatePrescaler_8,   /**< f_PCLK/8 */
    SPI_BAUD_PCLK_DIV_16  = SPI_BaudRatePrescaler_16,  /**< f_PCLK/16 */
    SPI_BAUD_PCLK_DIV_32  = SPI_BaudRatePrescaler_32,  /**< f_PCLK/32 */
    SPI_BAUD_PCLK_DIV_64  = SPI_BaudRatePrescaler_64,  /**< f_PCLK/64 */
    SPI_BAUD_PCLK_DIV_128 = SPI_BaudRatePrescaler_128, /**< f_PCLK/128 */
    SPI_BAUD_PCLK_DIV_256 = SPI_BaudRatePrescaler_256, /**< f_PCLK/256 */
} spi_baud_rate;

/** Available SPI interrupts */
typedef enum spi_interrupt {
    SPI_TXE_INTERRUPT  = SPI_I2S_IT_TXE,  /**< TX buffer empty interrupt */
    SPI_RXNE_INTERRUPT = SPI_I2S_IT_RXNE, /**< RX buffer not empty interrupt */
    SPI_ERR_INTERRUPT  = SPI_I2S_IT_ERR   /**<
                                          * Error interrupt (CRC, overrun,
                                          * and mode fault errors for SPI;
                                          * underrun, overrun errors for I2S)
                                          */
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
 * @brief Initialize and reset a SPI device.
 * @param dev Device to initialize and reset.
 */
void spi_init(spi_dev *dev);

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
void spi_gpio_cfg(spi_dev *dev,
				  uint8_t as_master,
                  gpio_dev *nss_dev,
                  uint8_t nss_bit,
                  gpio_dev *comm_dev,
                  uint8_t sck_bit,
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
void spi_master_enable(spi_dev *dev,
                       spi_baud_rate baudPrescaler,
                       spi_mode mode,
                       uint16_t bitorder);
                       
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
void spi_slave_enable(spi_dev *dev,
                      spi_mode mode,
                      uint16_t bitorder);

                   

/*
uint32_t spi_tx(spi_dev *dev, const void *buf, uint32_t len);
void spi_tx_byte(spi_dev *dev, uint8_t data);
uint8_t spi_rx_byte(spi_dev *dev);
void spi_write(spi_dev *dev, uint8_t reg, uint8_t data);
void spi_tx_buf(spi_dev *dev, uint8_t *txbuf, uint32_t txcount);
*/

void spi_foreach(void (*fn)(spi_dev*));
void spi_peripheral_enable(spi_dev *dev);
void spi_peripheral_disable(spi_dev *dev);
uint32_t spi_tx(spi_dev *dev, const void *buf, uint32_t len);
int spimaster_transfer(spi_dev *dev,
                       uint8_t *txbuf,
                       uint32_t txcount,
                       uint8_t *rxbuf,
                       uint32_t rxcount);
                       
                       

static inline uint8_t spi_is_enabled(spi_dev *dev) {
    return dev->SPIx->CR1 & SPI_CR1_SPE;
}

static inline void spi_peripheral_disable_all(void) {
    spi_foreach(spi_peripheral_disable);
}

static inline void spi_irq_enable(spi_dev *dev, uint32_t interrupt_flags) {
	SPI_I2S_ITConfig(dev->SPIx, interrupt_flags, ENABLE);
}

static inline void spi_irq_disable(spi_dev *dev, uint32_t interrupt_flags) {
	SPI_I2S_ITConfig(dev->SPIx, interrupt_flags, DISABLE);
}

static inline uint16_t spi_dff(spi_dev *dev) {
    return ((dev->SPIx->CR1 & SPI_DataSize_16b) == SPI_DataSize_8b ? SPI_DataSize_8b : SPI_DataSize_16b);
}

static inline uint8_t spi_is_rx_nonempty(spi_dev *dev) {
	return (dev->SPIx->SR & SPI_I2S_FLAG_RXNE);
}

static inline uint16_t spi_rx_reg(spi_dev *dev) {
    return (uint16_t)dev->SPIx->DR;
}

static inline uint8_t spi_is_tx_empty(spi_dev *dev) {
	return (dev->SPIx->SR & SPI_I2S_FLAG_TXE);
}

static inline void spi_tx_reg(spi_dev *dev, uint16_t val) {
    dev->SPIx->DR = val;
}

static inline uint8_t spi_is_busy(spi_dev *dev) {
	return (dev->SPIx->SR & SPI_I2S_FLAG_BSY);
}

                                       
#ifdef __cplusplus
  }
#endif

#endif
