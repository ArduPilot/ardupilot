#include <i2c.h>
#include "gpiopins.h"
#include "systick.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"



#define TIMEOUT 500
__IO uint32_t sTimeout = I2C_LONG_TIMEOUT;

static i2c_dev i2c_dev1 = {
    .I2Cx         = I2C1,
    .gpio_port    = &gpiob,
    .sda_pin      = 9,
    .scl_pin      = 8,
    .clk       	  = RCC_APB1Periph_I2C1,
    .gpio_af	  = GPIO_AF_I2C1,
    .ev_nvic_line = I2C1_EV_IRQn,
    .er_nvic_line = I2C1_ER_IRQn,
    .timeout      = I2C_LONG_TIMEOUT
};
/** I2C1 device */
i2c_dev* const _I2C1 = &i2c_dev1;

static i2c_dev i2c_dev2 = {
    .I2Cx         = I2C2,
    .gpio_port    = &gpiob,
    .sda_pin      = 11,
    .scl_pin      = 10,
    .clk       	  = RCC_APB1Periph_I2C2,
    .gpio_af	  = GPIO_AF_I2C2,
    .ev_nvic_line = I2C2_EV_IRQn,
    .er_nvic_line = I2C2_ER_IRQn,
    .timeout      = I2C_LONG_TIMEOUT
};
/** I2C2 device */
i2c_dev* const _I2C2 = &i2c_dev2;


typedef enum {TX = 0, RX = 1, TXREG = 2} I2C_Dir;

__IO I2C_Dir I2C_DIR;
__IO uint8_t I2C_BLOCKED = 0;
__IO uint8_t I2CADDRESS = 0;

#define I2C_BUF_SIZE 16

__IO uint16_t sEEAddress = 0;
__IO uint32_t  sEETimeout = I2C_LONG_TIMEOUT;
__IO uint16_t* sEEDataReadPointer;
__IO uint8_t*  sEEDataWritePointer;
__IO uint8_t   sEEDataNum;

/**
 * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval None
 */
static void i2c_lowLevel_deinit(i2c_dev *dev)
    {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* sEE_I2C Peripheral Disable */
    I2C_Cmd(dev->I2Cx, DISABLE);

    /* sEE_I2C DeInit */
    I2C_DeInit(dev->I2Cx);

    /*!< sEE_I2C Periph clock disable */
    RCC_APB1PeriphClockCmd(dev->clk, DISABLE);

    /*!< GPIO configuration */
    /*!< Configure I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /*!< Configure I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);
    }

/**
 * @brief  Initializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval None
 */
static void i2c_lowLevel_init(i2c_dev *dev)
    {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //DMA_InitTypeDef sEEDMA_InitStructure;

    /* Enable the i2c */
    RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

    /* Reset the Peripheral */
    RCC_APB1PeriphResetCmd(dev->clk, ENABLE);
    RCC_APB1PeriphResetCmd(dev->clk, DISABLE);

    /* Enable the GPIOs for the SCL/SDA Pins */
    RCC_AHB1PeriphClockCmd(dev->gpio_port->clk, ENABLE);

    /* GPIO configuration */
    /* Configure SCL */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->scl_pin);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /* Configure SDA */
    GPIO_InitStructure.GPIO_Pin = BIT(dev->sda_pin);
    GPIO_Init(dev->gpio_port->GPIOx, &GPIO_InitStructure);

    /* Connect GPIO pins to peripheral */
    GPIO_PinAFConfig(dev->gpio_port->GPIOx, dev->scl_pin, dev->gpio_af);
    GPIO_PinAFConfig(dev->gpio_port->GPIOx, dev->sda_pin, dev->gpio_af);

    if(dev->I2Cx == I2C1) {
	/* Configure and enable I2C DMA TX Channel interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C1_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure and enable I2C DMA RX Channel interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C1_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	/*!< I2C DMA TX and RX channels configuration */
	/* Enable the DMA clock */
	RCC_AHB1PeriphClockCmd(sEE_I2C1_DMA_CLK, ENABLE);

	/* Clear any pending flag on Rx Stream  */
	DMA_ClearFlag(sEE_I2C1_DMA_STREAM_TX,
		sEE1_TX_DMA_FLAG_FEIF | sEE1_TX_DMA_FLAG_DMEIF | sEE1_TX_DMA_FLAG_TEIF
			| sEE1_TX_DMA_FLAG_HTIF | sEE1_TX_DMA_FLAG_TCIF );
	/* Disable the EE I2C Tx DMA stream */
	DMA_Cmd(sEE_I2C1_DMA_STREAM_TX, DISABLE);
	/* Configure the DMA stream for the EE I2C peripheral TX direction */
	DMA_DeInit(sEE_I2C1_DMA_STREAM_TX );
	dev->DMA_InitStructure.DMA_Channel = sEE_I2C1_DMA_CHANNEL;
	dev->DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DR;
	dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) 0; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_BufferSize = 0xFFFF; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dev->DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dev->DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dev->DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dev->DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	dev->DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	dev->DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dev->DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dev->DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dev->DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(sEE_I2C1_DMA_STREAM_TX, &dev->DMA_InitStructure);

	/* Clear any pending flag on Rx Stream */
	DMA_ClearFlag(sEE_I2C1_DMA_STREAM_RX,
		sEE1_RX_DMA_FLAG_FEIF | sEE1_RX_DMA_FLAG_DMEIF | sEE1_RX_DMA_FLAG_TEIF
			| sEE1_RX_DMA_FLAG_HTIF | sEE1_RX_DMA_FLAG_TCIF );
	/* Disable the EE I2C DMA Rx stream */
	DMA_Cmd(sEE_I2C1_DMA_STREAM_RX, DISABLE);
	/* Configure the DMA stream for the EE I2C peripheral RX direction */
	DMA_DeInit(sEE_I2C1_DMA_STREAM_RX );
	DMA_Init(sEE_I2C1_DMA_STREAM_RX, &dev->DMA_InitStructure);

	/* Enable the DMA Channels Interrupts */
	DMA_ITConfig(sEE_I2C1_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
	DMA_ITConfig(sEE_I2C1_DMA_STREAM_RX, DMA_IT_TC, ENABLE);

    } else if (dev->I2Cx == I2C2) {
	/* Configure and enable I2C DMA TX Channel interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C2_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure and enable I2C DMA RX Channel interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C2_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	/*!< I2C DMA TX and RX channels configuration */
	/* Enable the DMA clock */
	RCC_AHB1PeriphClockCmd(sEE_I2C2_DMA_CLK, ENABLE);

	/* Clear any pending flag on Rx Stream  */
	DMA_ClearFlag(sEE_I2C2_DMA_STREAM_TX,
		sEE2_TX_DMA_FLAG_FEIF | sEE2_TX_DMA_FLAG_DMEIF | sEE2_TX_DMA_FLAG_TEIF
			| sEE2_TX_DMA_FLAG_HTIF | sEE2_TX_DMA_FLAG_TCIF );
	/* Disable the EE I2C Tx DMA stream */
	DMA_Cmd(sEE_I2C2_DMA_STREAM_TX, DISABLE);
	/* Configure the DMA stream for the EE I2C peripheral TX direction */
	DMA_DeInit(sEE_I2C2_DMA_STREAM_TX );
	dev->DMA_InitStructure.DMA_Channel = sEE_I2C2_DMA_CHANNEL;
	dev->DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C2->DR;
	dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) 0; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_BufferSize = 0xFFFF; /* This parameter will be configured durig communication */
	dev->DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dev->DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dev->DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dev->DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dev->DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	dev->DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	dev->DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dev->DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dev->DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dev->DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(sEE_I2C2_DMA_STREAM_TX, &dev->DMA_InitStructure);

	/* Clear any pending flag on Rx Stream */
	DMA_ClearFlag(sEE_I2C2_DMA_STREAM_RX,
		sEE2_RX_DMA_FLAG_FEIF | sEE2_RX_DMA_FLAG_DMEIF | sEE2_RX_DMA_FLAG_TEIF
			| sEE2_RX_DMA_FLAG_HTIF | sEE2_RX_DMA_FLAG_TCIF );
	/* Disable the EE I2C DMA Rx stream */
	DMA_Cmd(sEE_I2C2_DMA_STREAM_RX, DISABLE);
	/* Configure the DMA stream for the EE I2C peripheral RX direction */
	DMA_DeInit(sEE_I2C2_DMA_STREAM_RX );
	DMA_Init(sEE_I2C2_DMA_STREAM_RX, &dev->DMA_InitStructure);

	/* Enable the DMA Channels Interrupts */
	DMA_ITConfig(sEE_I2C2_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
	DMA_ITConfig(sEE_I2C2_DMA_STREAM_RX, DMA_IT_TC, ENABLE);
    }

}

void i2c_init(i2c_dev *dev, uint16_t address, uint32_t speed)
    {
    I2C_InitTypeDef I2C_InitStructure;

    i2c_lowLevel_init(dev);

    /* I2C configuration */
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = address;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = speed;

    I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(dev->I2Cx, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(dev->I2Cx, &I2C_InitStructure);

    sEEAddress = sEE_HW_ADDRESS;

    I2C_BLOCKED = 0;
    }

/**
 * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval None
 */
void i2c_deinit(i2c_dev *dev)
    {
    i2c_lowLevel_deinit(dev);
    }

/*
 void i2c_set_speed(i2c_dev *dev, uint32_t speed)
 {
 I2C_InitTypeDef  I2C_InitStructure;
 I2C_InitStructure.I2C_ClockSpeed = speed;
 I2C_Init(dev->I2Cx, &I2C_InitStructure);
 }
 */

/**
 * @brief  Initializes DMA channel used by the I2C EEPROM driver.
 * @param  None
 * @retval None
 */
void sEE_LowLevel_DMAConfig(i2c_dev *dev,uint32_t pBuffer, uint32_t BufferSize,
	uint32_t Direction)
    {
    //DMA_InitTypeDef sEEDMA_InitStructure;
    /* Initialize the DMA with the new parameters */
    if (Direction == sEE_DIRECTION_TX)
	{
	if (dev->I2Cx == I2C1){
		/* Configure the DMA Tx Stream with the buffer address and the buffer size */
	    dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pBuffer;
	    dev->DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	    dev->DMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;
		DMA_Init(sEE_I2C1_DMA_STREAM_TX, &dev->DMA_InitStructure);
	} else if (dev->I2Cx == I2C2){
		/* Configure the DMA Tx Stream with the buffer address and the buffer size */
	    dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pBuffer;
	    dev->DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	    dev->DMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;
		DMA_Init(sEE_I2C2_DMA_STREAM_TX, &dev->DMA_InitStructure);
	}

	}
    else
	{
	if(dev->I2Cx == I2C1){
	    /* Configure the DMA Rx Stream with the buffer address and the buffer size */
	    dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pBuffer;
	    dev->DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    dev->DMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;
	    DMA_Init(sEE_I2C1_DMA_STREAM_RX, &dev->DMA_InitStructure);
	} else if (dev->I2Cx == I2C2) {
	    /* Configure the DMA Rx Stream with the buffer address and the buffer size */
	    dev->DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) pBuffer;
	    dev->DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    dev->DMA_InitStructure.DMA_BufferSize = (uint32_t) BufferSize;
	    DMA_Init(sEE_I2C2_DMA_STREAM_RX, &dev->DMA_InitStructure);
	}
	}
}

uint8_t i2c_is_busy()
    {
    return I2C_BLOCKED;
    }

/**
 * @brief  Wait for EEPROM Standby state.
 *
 * @note  This function allows to wait and check that EEPROM has finished the
 *        last operation. It is mostly used after Write operation: after receiving
 *        the buffer to be written, the EEPROM may need additional time to actually
 *        perform the write operation. During this time, it doesn't answer to
 *        I2C packets addressed to it. Once the write operation is complete
 *        the EEPROM responds to its address.
 *
 * @param  None
 * @retval sEE_OK (0) if operation is correctly performed, else return value
 *         different from sEE_OK (0) or the timeout user callback.
 */
uint32_t sEE_WaitEepromStandby(i2c_dev *dev, uint8_t addr)
    {

    __IO uint16_t tmpSR1 = 0;
    __IO uint32_t sEETrials = 0;


    /*!< While the bus is busy */
    dev->timeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_BUSY ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    /* Keep looping till the slave acknowledge his address or maximum number
     of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
     in STM324x7I_eval_i2c_ee.h file) */
    while (1)
	{
	//Disable interrupts
	//I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

	/*!< Send START condition */
	I2C_GenerateSTART(dev->I2Cx, ENABLE);

	/*!< Test on EV5 and clear it */
	dev->timeout = I2C_TIMEOUT;
	while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ))
	    {

	    }
	if ((dev->timeout--) == 0)
	    {
	    //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	    return I2C_ERROR;
	    }

	/*!< Send EEPROM address for write */
	I2C_Send7bitAddress(dev->I2Cx, addr, I2C_Direction_Transmitter );

	/* Wait for ADDR flag to be set (Slave acknowledged his address) */
	dev->timeout = I2C_LONG_TIMEOUT;
	do
	    {
	    /* Get the current value of the SR1 register */
	    tmpSR1 = dev->I2Cx->SR1;

	    /* Update the timeout value and exit if it reach 0 */
	    if ((dev->timeout--) == 0)
		{
		//I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
		return I2C_ERROR;
		}
	    }
	/* Keep looping till the Address is acknowledged or the AF flag is
	 set (address not acknowledged at time) */
	while ((tmpSR1 & (I2C_SR1_ADDR | I2C_SR1_AF ))== 0);

	/* Check if the ADDR flag has been set */
if(	tmpSR1 & I2C_SR1_ADDR)
	    {
	    /* Clear ADDR Flag by reading SR1 then SR2 registers (SR1 have already
	     been read) */
	    (void)dev->I2Cx->SR2;

	    /*!< STOP condition */
	    I2C_GenerateSTOP(dev->I2Cx, ENABLE);

	    //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	    /* Exit the function */
	    return I2C_OK;
	    }
	else
	    {
	    /*!< Clear AF flag */
	    I2C_ClearFlag(dev->I2Cx, I2C_FLAG_AF);
	    }

	/* Check if the maximum allowed number of trials has bee reached */
	if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
	    {
	    //I2C_ITConfig(dev->I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
	    /* If the maximum number of trials has been reached, exit the function */
	    return I2C_ERROR;
	    }
	}
    }


/* Send a buffer to the i2c port */
uint32_t i2c_write(i2c_dev *dev, uint8_t addr, uint8_t *tx_buff, uint8_t *len)
{

    /* Set the pointer to the Number of data to be written. This pointer will be used
     by the DMA Transfer Completer interrupt Handler in order to reset the
     variable to 0. User should check on this variable in order to know if the
     DMA transfer has been complete or not. */
    dev->dmaDataWritePointer = len;

    uint16_t sent = 0;
    uint8_t *buffer = tx_buff;

    sent = 0;

    // While the bus is busy
    /*!< While the bus is busy */
    dev->timeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_BUSY ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send START condition
    I2C_GenerateSTART(dev->I2Cx, ENABLE);

    // Test on EV5 and clear it (cleared by reading SR1 then writing to DR)
    dev->timeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send address for write
    I2C_Send7bitAddress(dev->I2Cx, addr, I2C_Direction_Transmitter );

    dev->timeout = sEE_FLAG_TIMEOUT;
    // Test on EV6 and clear it
    while (!I2C_CheckEvent(dev->I2Cx,
	    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    I2C_SendData(dev->I2Cx, *buffer++);

    // Test on EV8 and clear it
    dev->timeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}
    if ((uint16_t)(*len) < 2)
	{
	/* Send the current byte */
	I2C_SendData(dev->I2Cx, *buffer);
	/* Point to the next byte to be written */
	sent++;
	/* Test on EV8 and clear it */
	dev->timeout = sEE_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	    {
	    if ((dev->timeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< STOP condition */
	I2C_GenerateSTOP(dev->I2Cx, DISABLE);
	I2C_ClearFlag(dev->I2Cx, I2C_FLAG_STOPF );
	/* Send STOP condition */
	I2C_GenerateSTOP(dev->I2Cx, ENABLE);
	}
    else
	{
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    sEE_LowLevel_DMAConfig(dev,(uint32_t) buffer, (uint8_t)(*len),
		sEE_DIRECTION_TX);

    /* Enable the DMA Tx Stream */
    if (dev->I2Cx == I2C1){
	DMA_Cmd(sEE_I2C1_DMA_STREAM_TX, ENABLE);
    } else if(dev->I2Cx == I2C2){
	DMA_Cmd(sEE_I2C2_DMA_STREAM_TX, ENABLE);
    }
    /* Enable the sEE_I2C peripheral DMA requests */
    I2C_DMACmd(dev->I2Cx, ENABLE);
	}

    return I2C_OK;
}
uint32_t i2c_read(i2c_dev *dev, uint8_t addr, uint8_t *tx_buff, uint8_t txlen, uint8_t *rx_buff, uint8_t *rxlen)
{

    /* Set the pointer to the Number of data to be read. This pointer will be used
     by the DMA Transfer Completer interrupt Handler in order to reset the
     variable to 0. User should check on this variable in order to know if the
     DMA transfer has been complete or not. */
    dev->dmaDataReadPointer = (uint16_t*) rxlen;

    uint8_t *buffer8 = rx_buff;

    // While the bus is busy
    dev->timeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_BUSY ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send START condition
    I2C_GenerateSTART(dev->I2Cx, ENABLE);

    // Test on EV5 and clear it (cleared by reading SR1 then writing to DR)
    dev->timeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send address for write
    I2C_Send7bitAddress(dev->I2Cx, addr, I2C_Direction_Transmitter );
    dev->timeout = sEE_FLAG_TIMEOUT;
    // Test on EV6 and clear it
    while (!I2C_CheckEvent(dev->I2Cx,
	    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    I2C_SendData(dev->I2Cx, *tx_buff++);

    // Test on EV8 and clear it
    dev->timeout = sEE_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_BTF ) == RESET)
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send STRAT condition a second time
    I2C_GenerateSTART(dev->I2Cx, ENABLE);

    // Test on EV5 and clear it (cleared by reading SR1 then writing to DR)
    dev->timeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(dev->I2Cx, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((dev->timeout--) == 0)
	    return I2C_ERROR;
	}

    // Send address for read
    I2C_Send7bitAddress(dev->I2Cx, addr, I2C_Direction_Receiver );

    if ((uint16_t)(*rxlen) < 2)
	{
	dev->timeout = sEE_FLAG_TIMEOUT;
	while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_ADDR ) == RESET)
	    {
	    if ((dev->timeout--) == 0)
		return I2C_ERROR;
	    }
	// Disable Acknowledgement
	I2C_AcknowledgeConfig(dev->I2Cx, DISABLE);

	/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
	(void) dev->I2Cx->SR2;
	/*!< STOP condition */
	I2C_GenerateSTOP(dev->I2Cx, DISABLE);
	I2C_ClearFlag(dev->I2Cx, I2C_FLAG_STOPF );
	/* Send STOP condition */
	I2C_GenerateSTOP(dev->I2Cx, ENABLE);

	/* Wait for the byte to be received */
	dev->timeout = sEE_FLAG_TIMEOUT;
	while (I2C_GetFlagStatus(dev->I2Cx, I2C_FLAG_RXNE ) == RESET)
	    {
	    if ((dev->timeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< Read the byte received from the EEPROM */
	*buffer8 = I2C_ReceiveData(dev->I2Cx);

	(uint16_t)(*rxlen)--;
	/* Wait to make sure that STOP control bit has been cleared */
	dev->timeout = sEE_FLAG_TIMEOUT;
	while (dev->I2Cx->CR1 & I2C_CR1_STOP )
	    {
	    if ((dev->timeout--) == 0)
		return I2C_ERROR;
	    }

	// Re-Enable Acknowledgement to be ready for another reception
	I2C_AcknowledgeConfig(dev->I2Cx, ENABLE);
	}
    else/* More than one Byte Master Reception procedure (DMA) -----------------*/
	{
	/*!< Test on EV6 and clear it */
	dev->timeout = sEE_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(dev->I2Cx,
		I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ))
	    {
	    if ((dev->timeout--) == 0)
		return I2C_ERROR;
	    }

	/* Configure the DMA Rx Channel with the buffer address and the buffer size */
	sEE_LowLevel_DMAConfig(dev, (uint32_t) buffer8, (uint16_t)(*rxlen),
		sEE_DIRECTION_RX);

	/* Inform the DMA that the next End Of Transfer Signal will be the last one */
	I2C_DMALastTransferCmd(dev->I2Cx, ENABLE);

	/* Enable the DMA Rx Stream */
	if (dev->I2Cx == I2C1){
		DMA_Cmd(sEE_I2C1_DMA_STREAM_RX, ENABLE);
	} else if(dev->I2Cx == I2C2){
		DMA_Cmd(sEE_I2C2_DMA_STREAM_RX, ENABLE);
	}
	/* Enable the sEE_I2C peripheral DMA requests */
	I2C_DMACmd(dev->I2Cx, ENABLE);
	}
    return I2C_OK;
    }
/**
 * @brief  Reads a block of data from the EEPROM.
 * @param  pBuffer : pointer to the buffer that receives the data read from
 *         the EEPROM.
 * @param  ReadAddr : EEPROM's internal address to start reading from.
 * @param  NumByteToRead : pointer to the variable holding number of bytes to
 *         be read from the EEPROM.
 *
 *        @note The variable pointed by NumByteToRead is reset to 0 when all the
 *              data are read from the EEPROM. Application should monitor this
 *              variable in order know when the transfer is complete.
 *
 * @note When number of data to be read is higher than 1, this function just
 *       configures the communication and enable the DMA channel to transfer data.
 *       Meanwhile, the user application may perform other tasks.
 *       When number of data to be read is 1, then the DMA is not used. The byte
 *       is read in polling mode.
 *
 * @retval sEE_OK (0) if operation is correctly performed, else return value
 *         different from sEE_OK (0) or the timeout user callback.
 */
uint32_t sEE_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr,
	uint16_t* NumByteToRead)
    {
    /* Set the pointer to the Number of data to be read. This pointer will be used
     by the DMA Transfer Completer interrupt Handler in order to reset the
     variable to 0. User should check on this variable in order to know if the
     DMA transfer has been complete or not. */
    sEEDataReadPointer = NumByteToRead;

    /*!< While the bus is busy */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_BUSY ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send START condition */
    I2C_GenerateSTART(sEE_I2C2, ENABLE);

    /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(sEE_I2C2, sEEAddress, I2C_Direction_Transmitter );

    /*!< Test on EV6 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send the EEPROM's internal address to read from: MSB of the address first */
    I2C_SendData(sEE_I2C2, (uint8_t)((ReadAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send the EEPROM's internal address to read from: LSB of the address */
    I2C_SendData(sEE_I2C2, (uint8_t)(ReadAddr & 0x00FF));

    /*!< Test on EV8 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_BTF ) == RESET)
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send STRAT condition a second time */
    I2C_GenerateSTART(sEE_I2C2, ENABLE);

    /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send EEPROM address for read */
    I2C_Send7bitAddress(sEE_I2C2, sEEAddress, I2C_Direction_Receiver );

    /* If number of data to be read is 1, then DMA couldn't be used */
    /* One Byte Master Reception procedure (POLLING) ---------------------------*/
    if ((uint16_t)(*NumByteToRead) < 2)
	{
	/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
	sEETimeout = sEE_FLAG_TIMEOUT;
	while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_ADDR ) == RESET)
	    {
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< Disable Acknowledgement */
	I2C_AcknowledgeConfig(sEE_I2C2, DISABLE);

	/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
	(void) sEE_I2C2 ->SR2;

	/*!< STOP condition */
	//I2C_GenerateSTOP(sEE_I2C, DISABLE);
	//I2C_ClearFlag(sEE_I2C, I2C_FLAG_STOPF );
	/* Send STOP condition */
	I2C_GenerateSTOP(sEE_I2C2, ENABLE);

	/* Wait for the byte to be received */
	sEETimeout = sEE_FLAG_TIMEOUT;
	while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_RXNE ) == RESET)
	    {
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< Read the byte received from the EEPROM */
	*pBuffer = I2C_ReceiveData(sEE_I2C2 );

	/*!< Decrement the read bytes counter */
	(uint16_t)(*NumByteToRead)--;

	/* Wait to make sure that STOP control bit has been cleared */
	sEETimeout = sEE_FLAG_TIMEOUT;
	while (sEE_I2C2 ->CR1 & I2C_CR1_STOP )
	    {
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< Re-Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(sEE_I2C2, ENABLE);
	}
    else/* More than one Byte Master Reception procedure (DMA) -----------------*/
	{
	/*!< Test on EV6 and clear it */
	sEETimeout = sEE_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(sEE_I2C2,
		I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ))
	    {
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }

	/* Configure the DMA Rx Channel with the buffer address and the buffer size */
	sEE_LowLevel_DMAConfig(_I2C2, (uint32_t) pBuffer, (uint16_t)(*NumByteToRead),
		sEE_DIRECTION_RX);

	/* Inform the DMA that the next End Of Transfer Signal will be the last one */
	I2C_DMALastTransferCmd(sEE_I2C2, ENABLE);

	/* Enable the DMA Rx Stream */
	DMA_Cmd(sEE_I2C2_DMA_STREAM_RX, ENABLE);

	/* Enable the sEE_I2C peripheral DMA requests */
	I2C_DMACmd(sEE_I2C2, ENABLE);
	}

    /* If all operations OK, return sEE_OK (0) */
    return I2C_OK;
    }

/**
 * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
 *
 * @note   The number of bytes (combined to write start address) must not
 *         cross the EEPROM page boundary. This function can only write into
 *         the boundaries of an EEPROM page.
 *         This function doesn't check on boundaries condition (in this driver
 *         the function sEE_WriteBuffer() which calls sEE_WritePage() is
 *         responsible of checking on Page boundaries).
 *
 * @param  pBuffer : pointer to the buffer containing the data to be written to
 *         the EEPROM.
 * @param  WriteAddr : EEPROM's internal address to write to.
 * @param  NumByteToWrite : pointer to the variable holding number of bytes to
 *         be written into the EEPROM.
 *
 *        @note The variable pointed by NumByteToWrite is reset to 0 when all the
 *              data are written to the EEPROM. Application should monitor this
 *              variable in order know when the transfer is complete.
 *
 * @note This function just configure the communication and enable the DMA
 *       channel to transfer data. Meanwhile, the user application may perform
 *       other tasks in parallel.
 *
 * @retval sEE_OK (0) if operation is correctly performed, else return value
 *         different from sEE_OK (0) or the timeout user callback.
 */
uint32_t sEE_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint8_t* NumByteToWrite)
    {
    /* Set the pointer to the Number of data to be written. This pointer will be used
     by the DMA Transfer Completer interrupt Handler in order to reset the
     variable to 0. User should check on this variable in order to know if the
     DMA transfer has been complete or not. */
    sEEDataWritePointer = NumByteToWrite;

    /*!< While the bus is busy */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_BUSY ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send START condition */
    I2C_GenerateSTART(sEE_I2C2, ENABLE);

    /*!< Test on EV5 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_MODE_SELECT ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(sEE_I2C2, sEEAddress, I2C_Direction_Transmitter );

	sEETimeout = sEE_FLAG_TIMEOUT;
    /*!< Test on EV6 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(sEE_I2C2, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /*!< Send the EEPROM's internal address to write to : LSB of the address */
    I2C_SendData(sEE_I2C2, (uint8_t)(WriteAddr & 0x00FF));

    /*!< Test on EV8 and clear it */
    sEETimeout = sEE_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
	sEE_LowLevel_DMAConfig(_I2C2,(uint32_t) pBuffer, (uint8_t)(*NumByteToWrite),
		sEE_DIRECTION_TX);

    /* Enable the DMA Tx Stream */
    DMA_Cmd(sEE_I2C2_DMA_STREAM_TX, ENABLE);

    /* Enable the sEE_I2C peripheral DMA requests */
    I2C_DMACmd(sEE_I2C2, ENABLE);

    /* If all operations OK, return sEE_OK (0) */
    return I2C_OK;
}

/**
 * @brief  Writes buffer of data to the I2C EEPROM.
 * @param  pBuffer : pointer to the buffer  containing the data to be written
 *         to the EEPROM.
 * @param  WriteAddr : EEPROM's internal address to write to.
 * @param  NumByteToWrite : number of bytes to write to the EEPROM.
 * @retval None
 */
uint32_t sEE_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr,
	uint16_t NumByteToWrite)
    {
    uint8_t NumOfPage = 0, NumOfSingle = 0, count = 0;
    uint16_t Addr = 0;

    Addr = WriteAddr % sEE_PAGESIZE;
    count = sEE_PAGESIZE - Addr;
    NumOfPage = NumByteToWrite / sEE_PAGESIZE;
    NumOfSingle = NumByteToWrite % sEE_PAGESIZE;

    /*!< If WriteAddr is sEE_PAGESIZE aligned  */
    if (Addr == 0)
	{
	/*!< If NumByteToWrite < sEE_PAGESIZE */
	if (NumOfPage == 0)
	    {
	    /* Store the number of data to be written */
	    sEEDataNum = NumOfSingle;
	    /* Start writing data */
	    sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
	    /* Wait transfer through DMA to be complete */
	    sEETimeout = sEE_LONG_TIMEOUT;
	    while (sEEDataNum > 0)
		{
		if ((sEETimeout--) == 0)
		    {
		    return I2C_ERROR;
		    };
		}
	    sEE_WaitEepromStandbyState();
	    }
	/*!< If NumByteToWrite > sEE_PAGESIZE */
	else
	    {
	    while (NumOfPage--)
		{
		/* Store the number of data to be written */
		sEEDataNum = sEE_PAGESIZE;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		WriteAddr += sEE_PAGESIZE;
		pBuffer += sEE_PAGESIZE;
		}

	    if (NumOfSingle != 0)
		{
		/* Store the number of data to be written */
		sEEDataNum = NumOfSingle;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		}
	    }
	}
    /*!< If WriteAddr is not sEE_PAGESIZE aligned  */
    else
	{
	/*!< If NumByteToWrite < sEE_PAGESIZE */
	if (NumOfPage == 0)
	    {
	    /*!< If the number of data to be written is more than the remaining space
	     in the current page: */
	    if (NumByteToWrite > count)
		{
		/* Store the number of data to be written */
		sEEDataNum = count;
		/*!< Write the data contained in same page */
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();

		/* Store the number of data to be written */
		sEEDataNum = (NumByteToWrite - count);
		/*!< Write the remaining data in the following page */
		sEE_WritePage((uint8_t*) (pBuffer + count), (WriteAddr + count),
			(uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		}
	    else
		{
		/* Store the number of data to be written */
		sEEDataNum = NumOfSingle;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		}
	    }
	/*!< If NumByteToWrite > sEE_PAGESIZE */
	else
	    {
	    NumByteToWrite -= count;
	    NumOfPage = NumByteToWrite / sEE_PAGESIZE;
	    NumOfSingle = NumByteToWrite % sEE_PAGESIZE;

	    if (count != 0)
		{
		/* Store the number of data to be written */
		sEEDataNum = count;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		WriteAddr += count;
		pBuffer += count;
		}

	    while (NumOfPage--)
		{
		/* Store the number of data to be written */
		sEEDataNum = sEE_PAGESIZE;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			};
		    }
		sEE_WaitEepromStandbyState();
		WriteAddr += sEE_PAGESIZE;
		pBuffer += sEE_PAGESIZE;
		}
	    if (NumOfSingle != 0)
		{
		/* Store the number of data to be written */
		sEEDataNum = NumOfSingle;
		sEE_WritePage(pBuffer, WriteAddr, (uint8_t*) (&sEEDataNum));
		/* Wait transfer through DMA to be complete */
		sEETimeout = sEE_LONG_TIMEOUT;
		while (sEEDataNum > 0)
		    {
		    if ((sEETimeout--) == 0)
			{
			return I2C_ERROR;
			}
		    }
		sEE_WaitEepromStandbyState();
		}
	    }
	}
    return I2C_OK;
    }

/**
 * @brief  Wait for EEPROM Standby state.
 *
 * @note  This function allows to wait and check that EEPROM has finished the
 *        last operation. It is mostly used after Write operation: after receiving
 *        the buffer to be written, the EEPROM may need additional time to actually
 *        perform the write operation. During this time, it doesn't answer to
 *        I2C packets addressed to it. Once the write operation is complete
 *        the EEPROM responds to its address.
 *
 * @param  None
 * @retval sEE_OK (0) if operation is correctly performed, else return value
 *         different from sEE_OK (0) or the timeout user callback.
 */
uint32_t sEE_WaitEepromStandbyState(void)
    {
    __IO uint16_t tmpSR1 = 0;
    __IO uint32_t sEETrials = 0;

    /*!< While the bus is busy */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_BUSY ))
	{
	if ((sEETimeout--) == 0)
	    return I2C_ERROR;
	}

    /* Keep looping till the slave acknowledge his address or maximum number
     of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
     in stm324xg_eval_i2c_ee.h file) */
    while (1)
	{
	/*!< Send START condition */
	I2C_GenerateSTART(sEE_I2C2, ENABLE);

	/*!< Test on EV5 and clear it */
	sEETimeout = sEE_FLAG_TIMEOUT;
	while (!I2C_CheckEvent(sEE_I2C2, I2C_EVENT_MASTER_MODE_SELECT ))
	    {
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }

	/*!< Send EEPROM address for write */
	I2C_Send7bitAddress(sEE_I2C2, sEEAddress, I2C_Direction_Transmitter );

	/* Wait for ADDR flag to be set (Slave acknowledged his address) */
	sEETimeout = sEE_LONG_TIMEOUT;
	do
	    {
	    /* Get the current value of the SR1 register */
	    tmpSR1 = sEE_I2C2 ->SR1;

	    /* Update the timeout value and exit if it reach 0 */
	    if ((sEETimeout--) == 0)
		return I2C_ERROR;
	    }
	/* Keep looping till the Address is acknowledged or the AF flag is
	 set (address not acknowledged at time) */
	while ((tmpSR1 & (I2C_SR1_ADDR | I2C_SR1_AF ))== 0);

	/* Check if the ADDR flag has been set */
if(	tmpSR1 & I2C_SR1_ADDR)
	    {
	    /* Clear ADDR Flag by reading SR1 then SR2 registers (SR1 have already
	     been read) */
	    (void)sEE_I2C2->SR2;

	    /*!< STOP condition */
	    I2C_GenerateSTOP(sEE_I2C2, ENABLE);

	    /* Exit the function */
	    return I2C_OK;
	    }
	else
	    {
	    /*!< Clear AF flag */
	    I2C_ClearFlag(sEE_I2C2, I2C_FLAG_AF);
	    }

	/* Check if the maximum allowed number of trials has bee reached */
	if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
	    {
	    /* If the maximum number of trials has been reached, exit the function */
	    return I2C_ERROR;
	    }
	}
    }
/**
 * @brief  This function handles the DMA Tx Channel interrupt Handler.
 * @param  None
 * @retval None
 */
void sEE_I2C1_DMA_TX_IRQHandler(void)
    {
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(sEE_I2C1_DMA_STREAM_TX, sEE1_TX_DMA_FLAG_TCIF )
	    != RESET)
	{
	/* Disable the DMA Tx Stream and Clear TC flag */
	DMA_Cmd(sEE_I2C1_DMA_STREAM_TX, DISABLE);
	DMA_ClearFlag(sEE_I2C1_DMA_STREAM_TX, sEE1_TX_DMA_FLAG_TCIF );

	/*!< Wait till all data have been physically transferred on the bus */
	_I2C1->timeout = sEE_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(sEE_I2C1, I2C_FLAG_BTF ))
	    {
	    if ((_I2C1->timeout--) == 0)
		return;
	    }

	/*!< Send STOP condition */
	I2C_GenerateSTOP(sEE_I2C1, ENABLE);

	/* Reset the variable holding the number of data to be written */
	*_I2C1->dmaDataWritePointer = 0;
	}
    }

/**
 * @brief  This function handles the DMA Rx Channel interrupt Handler.
 * @param  None
 * @retval None
 */
void sEE_I2C1_DMA_RX_IRQHandler(void)
    {
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(sEE_I2C1_DMA_STREAM_RX, sEE1_RX_DMA_FLAG_TCIF )
	    != RESET)
	{
	/*!< Send STOP Condition */
	I2C_GenerateSTOP(sEE_I2C1, ENABLE);

	/* Disable the DMA Rx Stream and Clear TC Flag */
	DMA_Cmd(sEE_I2C1_DMA_STREAM_RX, DISABLE);
	DMA_ClearFlag(sEE_I2C1_DMA_STREAM_RX, sEE1_RX_DMA_FLAG_TCIF );

	/* Reset the variable holding the number of data to be read */
	*_I2C1->dmaDataReadPointer = 0;
	}
    }
/**
 * @brief  This function handles the DMA Tx Channel interrupt Handler.
 * @param  None
 * @retval None
 */
void sEE_I2C2_DMA_TX_IRQHandler(void)
    {
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(sEE_I2C2_DMA_STREAM_TX, sEE2_TX_DMA_FLAG_TCIF )
	    != RESET)
	{
	/* Disable the DMA Tx Stream and Clear TC flag */
	DMA_Cmd(sEE_I2C2_DMA_STREAM_TX, DISABLE);
	DMA_ClearFlag(sEE_I2C2_DMA_STREAM_TX, sEE2_TX_DMA_FLAG_TCIF );

	/*!< Wait till all data have been physically transferred on the bus */
	_I2C2->timeout = sEE_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(sEE_I2C2, I2C_FLAG_BTF ))
	    {
	    if ((_I2C2->timeout--) == 0)
		return;
	    }

	/*!< Send STOP condition */
	I2C_GenerateSTOP(sEE_I2C2, ENABLE);

	/* Reset the variable holding the number of data to be written */
	*_I2C2->dmaDataWritePointer = 0;
	}
    }

/**
 * @brief  This function handles the DMA Rx Channel interrupt Handler.
 * @param  None
 * @retval None
 */
void sEE_I2C2_DMA_RX_IRQHandler(void)
    {
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(sEE_I2C2_DMA_STREAM_RX, sEE2_RX_DMA_FLAG_TCIF )
	    != RESET)
	{
	/*!< Send STOP Condition */
	I2C_GenerateSTOP(sEE_I2C2, ENABLE);

	/* Disable the DMA Rx Stream and Clear TC Flag */
	DMA_Cmd(sEE_I2C2_DMA_STREAM_RX, DISABLE);
	DMA_ClearFlag(sEE_I2C2_DMA_STREAM_RX, sEE2_RX_DMA_FLAG_TCIF );

	/* Reset the variable holding the number of data to be read */
	*_I2C2->dmaDataReadPointer = 0;
	}
    }

/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */
uint32_t sEE_TIMEOUT_UserCallback(void)
    {
    /* Block communication and all processes */
    while (1)
	{
	}
    }


/**
 * @brief Reset an I2C bus.
 *
 * Reset is accomplished by clocking out pulses until any hung slaves
 * release SDA and SCL, then generating a START condition, then a STOP
 * condition.
 *
 * @param dev I2C device
 */
void i2c_bus_reset(const i2c_dev *dev) {
    /* Release both lines */
    i2c_master_release_bus(dev);

    /*
     * Make sure the bus is free by clocking it until any slaves release the
     * bus.
     */
    while (!gpio_read_bit(dev->gpio_port, dev->sda_pin)) {
        /* Wait for any clock stretching to finish */
        while (!gpio_read_bit(dev->gpio_port, dev->scl_pin))
            ;
        delay_us(10);

        /* Pull low */
        gpio_write_bit(dev->gpio_port, dev->scl_pin, 0);
        delay_us(10);

        /* Release high again */
        gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
        delay_us(10);
    }

    /* Generate start then stop condition */
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 0);
    delay_us(10);
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 0);
    delay_us(10);
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
    delay_us(10);
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 1);
}

void i2c_master_release_bus(const i2c_dev *dev) {
    gpio_write_bit(dev->gpio_port, dev->scl_pin, 1);
    gpio_write_bit(dev->gpio_port, dev->sda_pin, 1);
    gpio_set_mode(dev->gpio_port, dev->scl_pin, GPIO_OUTPUT_OD);
    gpio_set_mode(dev->gpio_port, dev->sda_pin, GPIO_OUTPUT_OD);
}
