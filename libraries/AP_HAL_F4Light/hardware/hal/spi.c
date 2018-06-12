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
    .SPIx     = SPI1,
    .afio     = GPIO_AF_SPI1,
    .irq      = SPI1_IRQn,
    .clock    = RCC_APB2Periph_SPI1,
    .dma      = { DMA_CR_CH3, DMA2_STREAM2, DMA2_STREAM3 }, // SPI1, see Errata:  DMA2 transfers can spoils data
    .state    = &spi1_state,
};
/** SPI device 1 */
const spi_dev * const _SPI1 = &spi1;

static const spi_dev spi2 = {
    .SPIx     = SPI2,
    .afio     = GPIO_AF_SPI2,
    .irq      = SPI2_IRQn,
    .clock    = RCC_APB1Periph_SPI2,
    .dma      = { DMA_CR_CH0, DMA1_STREAM3, DMA1_STREAM4 }, // SPI2
    .state    = &spi2_state,
};
/** SPI device 2 */
const spi_dev * const _SPI2 = &spi2;

static const spi_dev spi3 = {
    .SPIx     = SPI3,
    .afio     = GPIO_AF_SPI3,
    .irq      = SPI3_IRQn,
    .clock    = RCC_APB1Periph_SPI3,
    .dma      = { DMA_CR_CH0, DMA1_STREAM2, DMA1_STREAM5 }, // SPI3
    .state    = &spi3_state,
};
/** SPI device 3 */
const spi_dev * const _SPI3 = &spi3;


void spi_init(const spi_dev *dev) {
    
  if (dev->SPIx == SPI1) {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
  }  else if (dev->SPIx == SPI2)  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
  } else if (dev->SPIx == SPI3) {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
  } else if (dev->SPIx == SPI4) {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI4, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI4, DISABLE);
  } else if (dev->SPIx == SPI5) {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5, DISABLE);
  } else if (dev->SPIx == SPI6) {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI6, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI6, DISABLE);
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

        /* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_INPUT_FLOATING);
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_PP);
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_INPUT_FLOATING);
}

/*
 * SPI auxiliary routines
 */
void spi_reconfigure(const spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode) {
    SPI_InitTypeDef  SPI_InitStructure;	

    memset(dev->state, 0, sizeof(spi_state));    
    
    spi_disable_irq(dev, SPI_INTERRUPTS_ALL);  
    
    spi_init(dev);

    spi_peripheral_disable(dev);

    /* Enable the SPI clock */
    if (dev->SPIx == SPI1)
        RCC_APB2PeriphClockCmd(dev->clock, ENABLE);
    else if (dev->SPIx == SPI2)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    else
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
                	
    /* SPI configuration */
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	
    switch(mode) {
    case SPI_MODE_0:
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	break;
    case SPI_MODE_1:
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	break;
    case SPI_MODE_2:
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	break;
    case SPI_MODE_3:
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	break;
    default:
	break;
    }


    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = baudPrescaler;
    if (bitorder == LSBFIRST)
    	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    else
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    if (ismaster)
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    else
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	
    SPI_Init(dev->SPIx, &SPI_InitStructure);

    SPI_Cmd(dev->SPIx, ENABLE);
        
    uint32_t dly=1000;
    while (SPI_I2S_GetFlagStatus(dev->SPIx, SPI_BIT_TXE) == RESET) {
        dly--;
        if(dly==0) break;
    }
        
    (void) dev->SPIx->DR; 
}


void spi_set_speed(const spi_dev *dev, uint16_t baudPrescaler) {

#define BR_CLEAR_MASK 0xFFC7

    SPI_Cmd(dev->SPIx, DISABLE);

    dev->SPIx->CR1 = (dev->SPIx->CR1 & BR_CLEAR_MASK) | baudPrescaler;

    SPI_Cmd(dev->SPIx, ENABLE);
}



// Transmit command and/or receive result in bidirectional master mode
int spimaster_transfer(const spi_dev *dev,
                       const uint8_t *txbuf,
                       uint16_t txcount,
                       uint8_t *rxbuf,
                       uint16_t rxcount)
{
    uint16_t txc_in=txcount;

    // Transfer command data out

    while (txcount--){
        while (!(dev->SPIx->SR & SPI_BIT_TXE)){ // just for case
            if(!spi_is_busy(dev) ) break;
        }	    
        dev->SPIx->DR = *txbuf++;
        uint16_t dly=1000; // ~20uS so byte already transferred
        while (!(dev->SPIx->SR & SPI_BIT_RXNE)) {
            if(--dly==0) break;
        }
        (void) dev->SPIx->DR; // read out unneeded data
    }

    if(txc_in && rxcount) delay_ns100(5); // small delay between TX and RX, to give the chip time to think over domestic affairs

    // Transfer response data in
    while (rxcount--){
        while (!(dev->SPIx->SR & SPI_BIT_TXE));
        dev->SPIx->DR = 0xFF;
        uint16_t dly=1000;
        while (!(dev->SPIx->SR & SPI_BIT_RXNE)) {
            if(--dly==0) break;
        }
        *rxbuf++ = dev->SPIx->DR;
    }

    // Wait until the transfer is complete - to not disable CS too early 
    uint32_t dly=3000;
    while (dev->SPIx->SR & SPI_BIT_BSY){ // but datasheet prohibits this usage
        dly--;
        if(dly==0) break;
    }

    return 0;
}


static void isr_handler(const spi_dev *dev){
    NVIC_ClearPendingIRQ(dev->irq);
    if(dev->state->handler) revo_call_handler(dev->state->handler, dev->SPIx->SR);
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
