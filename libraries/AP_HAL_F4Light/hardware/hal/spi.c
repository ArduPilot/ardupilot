/*

(c) 2017 night_ghost@ykoctpa.ru
 
based on: datasheet and logical analyser

*/


#pragma GCC optimize ("O2")

#include <spi.h>
#include <hal.h>

/*
 * SPI devices
 */

static spi_state spi1_state IN_CCM;
static spi_state spi2_state IN_CCM;
static spi_state spi3_state IN_CCM;

static const spi_dev spi1 = {
    .regs     = SPI1,
    .afio     = GPIO_AF_SPI1,
    .irq      = SPI1_IRQn,
    .clock    = RCC_APB2_bit_SPI1,
    .dma      = { DMA_CR_CH3, DMA2_STREAM2, DMA2_STREAM3 }, // SPI1, see Errata:  DMA2 transfers can spoils data
    .state    = &spi1_state,
};
/** SPI device 1 */
const spi_dev * const _SPI1 = &spi1;

static const spi_dev spi2 = {
    .regs     = SPI2,
    .afio     = GPIO_AF_SPI2,
    .irq      = SPI2_IRQn,
    .clock    = RCC_APB1_bit_SPI2,
    .dma      = { DMA_CR_CH0, DMA1_STREAM3, DMA1_STREAM4 }, // SPI2
    .state    = &spi2_state,
};
/** SPI device 2 */
const spi_dev * const _SPI2 = &spi2;

static const spi_dev spi3 = {
    .regs     = SPI3,
    .afio     = GPIO_AF_SPI3,
    .irq      = SPI3_IRQn,
    .clock    = RCC_APB1_bit_SPI3,
    .dma      = { DMA_CR_CH0, DMA1_STREAM2, DMA1_STREAM5 }, // SPI3
    .state    = &spi3_state,
};
/** SPI device 3 */
const spi_dev * const _SPI3 = &spi3;


void spi_init(const spi_dev *dev) {
    
  if (dev->regs == SPI1) {
    RCC_doAPB2_reset(RCC_APB2_bit_SPI1);
  }  else if (dev->regs == SPI2)  {
    RCC_doAPB1_reset(RCC_APB1_bit_SPI2);
  } else if (dev->regs == SPI3) {
    RCC_doAPB1_reset(RCC_APB1_bit_SPI3);
  } else if (dev->regs == SPI4) {
    RCC_doAPB2_reset(RCC_APB2_bit_SPI4);
  } else if (dev->regs == SPI5) {
    RCC_doAPB2_reset(RCC_APB2_bit_SPI5);
  } else if (dev->regs == SPI6) {
    RCC_doAPB2_reset(RCC_APB2_bit_SPI6);
  }
}

/**
 * @brief Call a function on each SPI port
 * @param fn Function to call.
 */
void spi_foreach(void (*fn)(const spi_dev*)) {
    fn(_SPI1);
    fn(_SPI2);
    fn(_SPI3);
}



void spi_gpio_master_cfg(const spi_dev *dev,
                  const gpio_dev *comm_dev,
                  uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit) {

	
	/* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_AF_OUTPUT_PP);
        gpio_set_af_mode(comm_dev, sck_bit, dev->afio);
        gpio_set_speed(comm_dev, sck_bit, GPIO_speed_100MHz);
        
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_OD_PU);
        gpio_set_af_mode(comm_dev, miso_bit, dev->afio);
        gpio_set_speed(comm_dev, miso_bit, GPIO_speed_100MHz);
        
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_AF_OUTPUT_PP);
	gpio_set_af_mode(comm_dev, mosi_bit, dev->afio);        
	gpio_set_speed(comm_dev, mosi_bit, GPIO_speed_100MHz);
}

void spi_gpio_slave_cfg(const spi_dev *dev,
                  const gpio_dev *comm_dev,
                  uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit) {
    gpio_set_mode(comm_dev, sck_bit, GPIO_INPUT_FLOATING); /* Configure SCK pin */
    gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_PP); /* Configure MISO pin */
    gpio_set_mode(comm_dev, mosi_bit, GPIO_INPUT_FLOATING); /* Configure MOSI pin */
}

/*
 * SPI auxiliary routines
 */
void spi_reconfigure(const spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode) {
    memset(dev->state, 0, sizeof(spi_state));    
    
    spi_disable_irq(dev, SPI_INTERRUPTS_ALL);  
    
    spi_init(dev);
    spi_peripheral_disable(dev);

    /* SPI configuration */
    uint16_t cpol;
    uint16_t cpha;
    
    switch(mode) {
    case SPI_MODE_0:
	cpol = SPI_CPOL_Low;
	cpha = SPI_CPHA_1Edge;
	break;
    case SPI_MODE_1:
	cpol = SPI_CPOL_Low;
	cpha = SPI_CPHA_2Edge;
	break;
    case SPI_MODE_2:
	cpol = SPI_CPOL_High;
	cpha = SPI_CPHA_1Edge;
	break;
    case SPI_MODE_3:
	cpol = SPI_CPOL_High;
	cpha = SPI_CPHA_2Edge;
	break;
    default:
	break;
    }


    uint16_t spi_mode;
    if (ismaster) spi_mode = SPI_Mode_Master;
    else          spi_mode = SPI_Mode_Slave;

    uint16_t cr1 = dev->regs->CR1 &= SPI_CR1_CLEAR_MASK;  // Clear all except SPE

    // direction, NSS management, first transmitted bit, BaudRate prescaler, master/salve mode, CPOL and CPHA
    dev->regs->CR1 = cr1 | SPI_Direction_2Lines_FullDuplex | SPI_size_8b | SPI_NSS_Soft | spi_mode | cpol | cpha  | baudPrescaler | bitorder;

    dev->regs->I2SCFGR &= (uint16_t)~(SPI_I2SCFGR_I2SMOD); // activate the SPI mode (clear I2SMOD in I2SCFGR register) 
    dev->regs->CRCPR = 7; // SPI_CRCPolynomial;

    spi_peripheral_enable(dev);
        
    uint32_t dly=1000;
    while ( (dev->regs->SR & SPI_BIT_TXE) == 0) { // wait for TXE bit
        dly--;
        if(dly==0) break;
    }
        
    (void) dev->regs->DR; //  read out garbage data if any
}


void spi_set_speed(const spi_dev *dev, uint16_t baudPrescaler) {

#define BR_CLEAR_MASK 0xFFC7

    spi_peripheral_disable(dev);
    dev->regs->CR1 = (dev->regs->CR1 & BR_CLEAR_MASK) | baudPrescaler;
    spi_peripheral_enable(dev);
}



// Transmit command and/or receive result in bidirectional master mode, in polling
int spimaster_transfer(const spi_dev *dev,
                       const uint8_t *txbuf,
                       uint16_t txcount,
                       uint8_t *rxbuf,
                       uint16_t rxcount)
{
    uint16_t txc_in=txcount;

    // Transfer command data out

    while (txcount--){
        while (!(dev->regs->SR & SPI_BIT_TXE)){ // just for case
            if(!spi_is_busy(dev) ) break;
        }	    
        dev->regs->DR = *txbuf++;
        uint16_t dly=1000; // ~20uS so byte already transferred
        while (!(dev->regs->SR & SPI_BIT_RXNE)) {
            if(--dly==0) break;
        }
        (void) dev->regs->DR; // read out unneeded data
    }

    if(txc_in && rxcount) delay_ns100(5); // small delay between TX and RX, to give the chip time to think over domestic affairs

    // Transfer response data in
    while (rxcount--){
        while (!(dev->regs->SR & SPI_BIT_TXE));
        dev->regs->DR = 0xFF;
        uint16_t dly=1000;
        while (!(dev->regs->SR & SPI_BIT_RXNE)) {
            if(--dly==0) break;
        }
        *rxbuf++ = dev->regs->DR;
    }

    spi_wait_busy(dev); // Wait until the transfer is complete - to not disable CS too early 
    return 0;
}


static void isr_handler(const spi_dev *dev){
    NVIC_ClearPendingIRQ(dev->irq);
    if(dev->state->handler) revo_call_handler(dev->state->handler, dev->regs->SR);
    else { // disable interrupts
        spi_disable_irq(dev, SPI_INTERRUPTS_ALL);
    }
}

void SPI1_IRQHandler();
void SPI2_IRQHandler();
void SPI3_IRQHandler();

void SPI1_IRQHandler() {
    isr_handler(&spi1);
}

void SPI2_IRQHandler() {
    isr_handler(&spi2);
}

void SPI3_IRQHandler() {
    isr_handler(&spi3);
}
