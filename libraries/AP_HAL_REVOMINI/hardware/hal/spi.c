#include <spi.h>
#include <hal.h>

/*
 * SPI devices
 */

static spi_dev spi1 = {
    .SPIx     = SPI1,
    .afio     = GPIO_AF_SPI1,
    .irq	  = SPI1_IRQn,
};
/** SPI device 1 */
spi_dev *_SPI1 = &spi1;

static spi_dev spi2 = {
    .SPIx     = SPI2,
    .afio     = GPIO_AF_SPI2,
    .irq	  = SPI2_IRQn,
};
/** SPI device 2 */
spi_dev *_SPI2 = &spi2;

static spi_dev spi3 = {
    .SPIx     = SPI3,
    .afio     = GPIO_AF_SPI3,
    .irq	  = SPI3_IRQn,
};
/** SPI device 2 */
spi_dev *_SPI3 = &spi3;


void spi_init(spi_dev *dev) {
	SPI_I2S_DeInit(dev->SPIx);
}

/**
 * @brief Call a function on each SPI port
 * @param fn Function to call.
 */
void spi_foreach(void (*fn)(spi_dev*)) {
    fn(_SPI1);
    fn(_SPI2);
    fn(_SPI3);
}

/**
 * @brief Enable a SPI peripheral
 * @param dev Device to enable
 */
void spi_peripheral_enable(spi_dev *dev) {
	SPI_Cmd(dev->SPIx, ENABLE);
}

/**
 * @brief Disable a SPI peripheral
 * @param dev Device to disable
 */
void spi_peripheral_disable(spi_dev *dev) {
	SPI_Cmd(dev->SPIx, DISABLE);
}

void spi_gpio_cfg(spi_dev *dev,
				  uint8_t as_master,
                  gpio_dev *nss_dev,
                  uint8_t nss_bit,
                  gpio_dev *comm_dev,
                  uint8_t sck_bit,
                  uint8_t miso_bit,
                  uint8_t mosi_bit) {
    if (as_master) {
		/* Configure NSS pin */	
        gpio_set_mode(nss_dev, nss_bit, GPIO_OUTPUT_PP);
		gpio_write_bit(nss_dev, nss_bit, 1);
		/* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_AF_OUTPUT_PP);
        gpio_set_af_mode(comm_dev, sck_bit, dev->afio);
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_OD);
        gpio_set_af_mode(comm_dev, miso_bit, dev->afio);
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_AF_OUTPUT_PP);
		gpio_set_af_mode(comm_dev, mosi_bit, dev->afio);        
    } else {
		/* Configure NSS pin */	
        gpio_set_mode(nss_dev, nss_bit, GPIO_INPUT_FLOATING);
        /* Configure SCK pin */
        gpio_set_mode(comm_dev, sck_bit, GPIO_INPUT_FLOATING);
        /* Configure MISO pin */
        gpio_set_mode(comm_dev, miso_bit, GPIO_AF_OUTPUT_PP);
        /* Configure MOSI pin */
        gpio_set_mode(comm_dev, mosi_bit, GPIO_INPUT_FLOATING);
    }
}

/*
 * SPI auxiliary routines
 */
static void spi_reconfigure(spi_dev *dev, uint8_t ismaster, uint16_t baudPrescaler, uint16_t bitorder, uint8_t mode) {
	SPI_InitTypeDef  SPI_InitStructure;
	
    spi_irq_disable(dev, SPI_INTERRUPTS_ALL);
    SPI_I2S_DeInit(dev->SPIx);

	/* Enable the SPI clock */
	if (dev->SPIx == SPI1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  
	else if (dev->SPIx == SPI2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	else
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* SPI configuration */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	
	switch(mode)
	{
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

	if (!ismaster)
	{
		/* Enable the Rx buffer not empty interrupt */
		spi_irq_enable(dev, SPI_I2S_IT_RXNE);
	}
	
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure the Priority Group to 1 bit */                
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
	/* Configure the SPI interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = dev->irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	    	
    SPI_Cmd(dev->SPIx, ENABLE);
}

/**
 * @brief Configure and enable a SPI device as bus master.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 */
void spi_master_enable(spi_dev *dev,
                       spi_baud_rate baudPrescaler,
                       spi_mode mode,
                       uint16_t bitorder) 
{
    spi_reconfigure(dev, 1, baudPrescaler, bitorder, mode);
}

/**
 * @brief Configure and enable a SPI device as a bus slave.
 *
 * The device's peripheral will be disabled before being reconfigured.
 *
 */
void spi_slave_enable(spi_dev *dev,
                      spi_mode mode,
                      uint16_t bitorder)
{
    spi_reconfigure(dev, 0, 0, bitorder, mode);
}

/* test code */
#if 0
__IO uint8_t SPI_BLOCKED = 0;

#define TXBUFFERSIZE   16
#define RXBUFFERSIZE   TXBUFFERSIZE

__IO uint8_t TxBuffer [TXBUFFERSIZE];
__IO uint8_t Tx_Idx = 0;
__IO uint8_t *RxBufferPtr;
__IO uint8_t Rx_Idx = 0;

__IO uint8_t Rx_Len = 0;
__IO uint8_t Tx_Len = 0;

 
/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* SPI in Slave Receiver mode--------------------------------------- */
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{
		if (Rx_Idx < Rx_Len)
		{
			RxBufferPtr[Rx_Idx++] = SPI_I2S_ReceiveData(SPI2);
		}
		else
		{
			/* End of transmission */
//			if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == RESET) {
				//SPI_BLOCKED = 0;
	//		}
		}
	}

	/* SPI in Master Tramitter mode--------------------------------------- */
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET)
	{
		if (Tx_Idx < Tx_Len)
		{
			/* Send Transaction data */
			SPI_I2S_SendData(SPI2, TxBuffer[Tx_Idx++]);
		}
		else
		{
			/* End of transmission */
			if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == RESET) {
				/* Disable the Tx buffer empty interrupt */
				SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
				SPI_BLOCKED = 0;
			}
		}
    }
}

uint8_t spi_write_test(spi_dev *dev, uint8_t *buffer, uint8_t len)
{
	assert_param(len <= TXBUFFERSIZE && len > 0);
	
	// disable interrupts
	SPI_I2S_ITConfig(dev->SPIx, SPI_I2S_IT_TXE, DISABLE);
	SPI_DataSizeConfig(dev->SPIx, SPI_DataSize_8b);
	
    /*
     * check if I2C interace is in use
     */
	if(SPI_BLOCKED == 0)
    {
		// set buffer length and start index
		Tx_Len = len;
		Tx_Idx = 0;
		Rx_Len = 0;
		Rx_Idx = 0;
		
		__IO uint8_t *tmp_buffer_ptr = TxBuffer;
		uint8_t i;
		for(i=0; i<len; ++i) {
			*tmp_buffer_ptr++ = *buffer++; // copies faster than using indexed arrays
		}
                    
		// notify that transfer has started
		SPI_BLOCKED = 1; 

		/* Enable the Tx buffer empty interrupt */
		SPI_I2S_ITConfig(dev->SPIx, SPI_I2S_IT_TXE, ENABLE);
	} 
	else
	{
		//errno_r = EBUSY;
		return ERROR;
	}
	
    /*
     * wait till finished
     */
    while(SPI_BLOCKED == 1);
    
    return OK;
}


uint8_t spi_read_test(spi_dev *dev, uint8_t *rx_buffer, uint8_t rxlen)
{
	assert_param(rx_buffer != NULL && rxlen > 0);

	// disable interrupts
	SPI_I2S_ITConfig(dev->SPIx, SPI_I2S_IT_RXNE, DISABLE);
		
    /*
     * check if I2C interface is in use
     */
    if(SPI_BLOCKED == 0)
    {
   		RxBufferPtr = rx_buffer;
        Rx_Len = rxlen;
        Rx_Idx = 0;        
		Tx_Idx = 0;
		Tx_Len = 0;

		// enable interrupt
		SPI_I2S_ITConfig(dev->SPIx, SPI_I2S_IT_RXNE, ENABLE);

		// notify that transfer has started
		SPI_BLOCKED = 1; 
    }
	else 
	{
		//errno_r = EBUSY;
		return ERROR;
	}

    /*
     * wait till finished
    */
    while (Rx_Idx < Rx_Len);
    //while(SPI_BLOCKED == 1);
  
    return OK;
}

uint8_t spi_is_busy_test()
{
        return SPI_BLOCKED;
}

/* test code */
#endif

// Transmit command and/or receive result in bidirectional master mode
int spimaster_transfer(spi_dev *dev,
                       uint8_t *txbuf,
                       uint32_t txcount,
                       uint8_t *rxbuf,
                       uint32_t rxcount)
{
	//errno_r = 0;

	// Validate parameters
	if ((txbuf == NULL) && (txcount != 0))
	{
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((txcount == 0) && (txbuf != NULL))
	{
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((rxbuf == NULL) && (rxcount != 0))
	{
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}

	if ((rxcount == 0) && (rxbuf != NULL))
	{
		//errno_r = EINVAL;
		return __LINE__ - 3;
	}


	// Transfer command data out
	while (txcount--)
	{
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
		dev->SPIx->DR = *txbuf++;
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
		(void) dev->SPIx->DR;
	}	

	// Transfer response data in
	while (rxcount--)
	{
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
		dev->SPIx->DR = 0;
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
		*rxbuf++ = dev->SPIx->DR;
	}

	// Wait until the transfer is complete
	while (dev->SPIx->SR & SPI_I2S_FLAG_BSY);

	return 0;
}


/*
void spi_tx_byte(spi_dev *dev, uint8_t data) 
{
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
    dev->SPIx->DR = data;
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
    (void) dev->SPIx->DR;	
    while ((dev->SPIx->SR & SPI_I2S_FLAG_BSY));    
}

void spi_rx_byte(spi_dev *dev, uint8_t *data)
{
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
    dev->SPIx->DR = 0;
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
    *data = dev->SPIx->DR;	    
}

void spi_tx(spi_dev *dev, uint8_t *txbuf, uint32_t txcount) 
{
	while (txcount--)
	{
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
		dev->SPIx->DR = *txbuf++;
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
		(void) dev->SPIx->DR;
	}	
    while ((dev->SPIx->SR & SPI_I2S_FLAG_BSY));	
}
*/
uint32_t spi_tx(spi_dev *dev, const void *buf, uint32_t len) {
    uint32_t txed = 0;
    uint8_t byte_frame = spi_dff(dev) == SPI_DataSize_8b;
    while (spi_is_tx_empty(dev) && (txed < len)) {
        if (byte_frame) {
            dev->SPIx->DR = ((const uint8_t*)buf)[txed++];
        } else {
            dev->SPIx->DR = ((const uint16_t*)buf)[txed++];
        }
    }
    return txed;
}
/*
void spi_rx(spi_dev *dev, uint8_t *rxbuf, uint32_t rxcount) )
{
	while (rxcount--)
	{
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
		dev->SPIx->DR = 0;
		while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
		*rxbuf++ = dev->SPIx->DR;
	}    
	while (dev->SPIx->SR & SPI_I2S_FLAG_BSY);
}

void spi_write(spi_dev *dev, uint8_t reg, uint8_t data)
{
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
    dev->SPIx->DR = reg;
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
    (void) dev->SPIx->DR;
    
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_TXE));
    dev->SPIx->DR = data;
    while (!(dev->SPIx->SR & SPI_I2S_FLAG_RXNE));
    (void) dev->SPIx->DR;
    
    while ((dev->SPIx->SR & SPI_I2S_FLAG_BSY));
}
*/
