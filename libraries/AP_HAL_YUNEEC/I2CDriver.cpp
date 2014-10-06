#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "I2CDriver.h"
#include <stm32f37x.h>
#include <stm32f37x_rcc.h>
#include <stm32f37x_i2c.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define TIMING_CLEAR_MASK      	 			((uint32_t)0xF0FFFFFF)  /*<! I2C TIMING clear register Mask */
#define I2C_CR2_CLEAR_MASK					(uint32_t)(0x03FF67FF)
#define I2C_CR2_PARTIAL_CLEAR_MASK			(uint32_t)(0x03FF6400)

#define __I2C_GET_TXIS(device)              (device->ISR & ((uint32_t)0x00000002))
#define __I2C_GET_BUSY(device)              (device->ISR & ((uint32_t)0x00008000))
#define __I2C_GET_TCR(device)               (device->ISR & ((uint32_t)0x00000080))
#define __I2C_GET_TC(device)                (device->ISR & ((uint32_t)0x00000040))
#define __I2C_GET_RXNE(device)     			(device->ISR & ((uint32_t)0x00000004))
#define __I2C_GET_NACKF(device)             (device->ISR & ((uint32_t)0x00000010))

#define I2C_CR2_Write_NoReg					(uint32_t)(0x02002000)
#define I2C_CR2_WriteReg_Seq1				(uint32_t)(0x01002000)
#define I2C_CR2_WriteReg_Seq2				(uint32_t)(0x02000000)

#define I2C_CR2_Read_NoReg					(uint32_t)(0x02002400)
#define I2C_CR2_ReadReg_Seq1				(uint32_t)(0x00002000)
#define I2C_CR2_ReadReg_Seq2				(uint32_t)(0x02002400)


#define __I2C_TIMEOUT(cmd, start, timeout, isTimeout)		do {\
																if (timeout > 0) {\
																	isTimeout = ((hal.scheduler->millis() - start) >= timeout)?true:false;\
																	if (isTimeout == true)\
																		goto error;\
																}\
															} while ( (cmd) == 0 )

// I2C clock: HSI
#define I2C_FAST_MODE_TIMING		0x00310309
#define I2C_STANDARD_MODE_TIMING 	0x10420F13

YUNEECI2CDriver::YUNEECI2CDriver(I2C_TypeDef* i2c, GPIO_TypeDef* port,
								const uint32_t i2cClk,	const uint32_t portClk,
								const uint16_t scl_bit, const uint16_t sda_bit,
								const uint8_t scl_pinSource, const uint8_t sda_pinSource,
								AP_HAL::Semaphore* semaphore) :
	_i2c_info{i2c, port, i2cClk, portClk, scl_bit, sda_bit, scl_pinSource, sda_pinSource},
	_semaphore(semaphore), _lockup_count(0), _ignore_errors(false), _timeout(0)
{}

void YUNEECI2CDriver::begin() {
	_i2c_config(_i2c_info);
}

void YUNEECI2CDriver::end() {
	/* Disable I2C */
	_i2c_info.i2c->CR1 &= ~I2C_CR1_PE;
	/* Reset I2C */
    RCC->APB1RSTR &= ~_i2c_info.i2cClk;
    RCC->APB1RSTR |= _i2c_info.i2cClk;
    /* Disable I2C clock */
    RCC->APB1ENR &= ~_i2c_info.i2cClk;
    /* Disable I2Cx SCL and SDA Pin Clock */
    RCC->AHBENR &= ~_i2c_info.portClk;
}

void YUNEECI2CDriver::setTimeout(uint16_t ms) {
	_timeout  = ms;
}

void YUNEECI2CDriver::setHighSpeed(bool active) {
	/* Disable I2Cx Peripheral */
	_i2c_info.i2c->CR1 &= (uint32_t)~((uint32_t)I2C_CR1_PE);
    if (active) {
    	_i2c_info.i2c->TIMINGR = I2C_FAST_MODE_TIMING & TIMING_CLEAR_MASK;
    } else {
    	_i2c_info.i2c->TIMINGR = I2C_STANDARD_MODE_TIMING & TIMING_CLEAR_MASK;
    }
    /* Enable I2Cx Peripheral */
    _i2c_info.i2c->CR1 |= I2C_CR1_PE;
}

uint8_t YUNEECI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data) {
	uint32_t tmpreg = 0;
	uint32_t start;
	bool isTimeout = false;
	/*
	* Configure slave address, nbytes, no reload and generate start
	*/
	/* Get the CR2 register value */
	tmpreg = _i2c_info.i2c->CR2;
	/* clear tmpreg specific bits */
	tmpreg &= ~I2C_CR2_CLEAR_MASK;
	/* update tmpreg */
	tmpreg |= ((uint32_t)addr << 1) & I2C_CR2_SADD;
	tmpreg |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_Write_NoReg;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Transmit data */
	while (len > 0) {
	    /* Wait until TXIS flag is set */
		start = hal.scheduler->millis();
		__I2C_TIMEOUT(__I2C_GET_TXIS(_i2c_info.i2c), start, _timeout, isTimeout);

		_i2c_info.i2c->TXDR = *data;
		len--;
		data++;
	}

    /* Wait until Stop flag is set */
	start = hal.scheduler->millis();
    __I2C_TIMEOUT(!(__I2C_GET_BUSY(_i2c_info.i2c)), start, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
	return 	writeRegisters(addr, reg, 1, &val);
}

uint8_t YUNEECI2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	uint32_t tmpreg = 0;
	uint32_t start;
	bool isTimeout = false;
	/*
	* Configure slave address, nbytes, no reload and generate start
	* Send register address to write
	*/
	/* Get the CR2 register value */
	tmpreg = _i2c_info.i2c->CR2;
	/* clear tmpreg specific bits */
	tmpreg &= ~I2C_CR2_CLEAR_MASK;
	/* update tmpreg */
	tmpreg |= ((uint32_t)addr << 1) & I2C_CR2_SADD;
	tmpreg |= ((uint32_t)1 << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_WriteReg_Seq1;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Wait until TXIS flag is set */
	start = hal.scheduler->millis();
	__I2C_TIMEOUT(__I2C_GET_TXIS(_i2c_info.i2c), start, _timeout, isTimeout);

	/* Send register address */
	_i2c_info.i2c->TXDR = reg;

	/* Wait until TCR flag is set */
	start = hal.scheduler->millis();
	__I2C_TIMEOUT(__I2C_GET_TCR(_i2c_info.i2c), start, _timeout, isTimeout);

	/* Configure to send data */
	tmpreg &= ~I2C_CR2_PARTIAL_CLEAR_MASK;
	tmpreg |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_WriteReg_Seq2;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Transmit data */
	while (len > 0) {
		/* Wait until TXIS flag is set */
		start = hal.scheduler->millis();
		__I2C_TIMEOUT(__I2C_GET_TXIS(_i2c_info.i2c), start, _timeout, isTimeout);

		_i2c_info.i2c->TXDR = *data;
		len--;
		data++;
	}

    /* Wait until Stop flag is set */
	start = hal.scheduler->millis();
    __I2C_TIMEOUT(!(__I2C_GET_BUSY(_i2c_info.i2c)), start, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data) {
	memset(data, 0, len);

	uint32_t tmpreg = 0;
	uint32_t start;
	bool isTimeout = false;
	/*
	* Configure slave address, nbytes, no reload and generate start
	*/
	/* Get the CR2 register value */
	tmpreg = _i2c_info.i2c->CR2;
	/* clear tmpreg specific bits */
	tmpreg &= ~I2C_CR2_CLEAR_MASK;
	/* update tmpreg */
	tmpreg |= ((uint32_t)addr << 1) & I2C_CR2_SADD;
	tmpreg |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_Read_NoReg;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Receive data */
	while (len > 0) {
		start = hal.scheduler->millis();
		__I2C_TIMEOUT(__I2C_GET_RXNE(_i2c_info.i2c), start, _timeout, isTimeout);
		*data = (uint8_t)_i2c_info.i2c->RXDR;
		len--;
		data++;
	}

    /* Wait until Stop flag is set */
	start = hal.scheduler->millis();
    __I2C_TIMEOUT(!(__I2C_GET_BUSY(_i2c_info.i2c)), start, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}
uint8_t YUNEECI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
    return readRegisters(addr, reg, 1, data);
}

uint8_t YUNEECI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	memset(data, 0, len);

	uint32_t tmpreg = 0;
	uint32_t start;
	bool isTimeout = false;
	/*
	* Configure slave address, nbytes, no reload and generate start
	*/
	/* Get the CR2 register value */
	tmpreg = _i2c_info.i2c->CR2;
	/* clear tmpreg specific bits */
	tmpreg &= ~I2C_CR2_CLEAR_MASK;
	/* update tmpreg */
	tmpreg |= ((uint32_t)addr << 1) & I2C_CR2_SADD;
	tmpreg |= ((uint32_t)1 << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_ReadReg_Seq1;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Wait until TXIS flag is set */
	start = hal.scheduler->millis();
	__I2C_TIMEOUT(__I2C_GET_TXIS(_i2c_info.i2c), start, _timeout, isTimeout);

	/* Send register address */
	_i2c_info.i2c->TXDR = reg;

	/* Wait until TC flag is set */
	start = hal.scheduler->millis();
	__I2C_TIMEOUT(__I2C_GET_TC(_i2c_info.i2c), start, _timeout, isTimeout);

	/* Configure to read data */
	tmpreg &= ~I2C_CR2_PARTIAL_CLEAR_MASK;
	tmpreg |= ((uint32_t)len << 16) & I2C_CR2_NBYTES;
	tmpreg |= I2C_CR2_ReadReg_Seq2;
	/* update CR2 register */
	__disable_irq();
	_i2c_info.i2c->CR2 = tmpreg;
	__enable_irq();

	/* Receive data */
	while (len > 0) {
		start = hal.scheduler->millis();
		__I2C_TIMEOUT(__I2C_GET_RXNE(_i2c_info.i2c), start, _timeout, isTimeout);
		*data = (uint8_t)_i2c_info.i2c->RXDR;
		len--;
		data++;
	}

    /* Wait until Busy flag is set */
	start = hal.scheduler->millis();
    __I2C_TIMEOUT(!(__I2C_GET_BUSY(_i2c_info.i2c)), start, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::lockup_count() {
	return _lockup_count;;
}


void YUNEECI2CDriver::_i2c_bus_reset(struct I2C_Info &i2c_info) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Disable I2C device */
	i2c_info.i2c->CR1 &= ~I2C_CR1_PE;

	/*
	 * Release both lines
	 */
    /* Enable I2Cx SCL and SDA Pin Clock */
    RCC->AHBENR |= i2c_info.portClk;

    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.port, i2c_info.scl_pinSource, GPIO_AF_0);
    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.port, i2c_info.sda_pinSource, GPIO_AF_0);
    i2c_info.port->BSRR = i2c_info.scl_bit;
    i2c_info.port->BSRR = i2c_info.sda_bit;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = i2c_info.scl_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = i2c_info.sda_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.port, &GPIO_InitStructure);

    /*
     * Make sure the bus is free by clocking it until any slaves release the
     * bus.
     */
	while (!((i2c_info.port->IDR) & i2c_info.sda_bit)) {
        /* Wait for any clock stretching to finish */
		while (!((i2c_info.port->IDR) & i2c_info.scl_bit))
			;
		hal.scheduler->delay_microseconds(10);

		/* pull low */
		i2c_info.port->BRR = i2c_info.scl_bit;
		hal.scheduler->delay_microseconds(10);

		/* release high again */
		i2c_info.port->BSRR = i2c_info.scl_bit;
		hal.scheduler->delay_microseconds(10);
	}

    /* generate start then stop condition */
	i2c_info.port->BRR = i2c_info.sda_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.port->BRR = i2c_info.scl_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.port->BSRR = i2c_info.scl_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.port->BSRR = i2c_info.sda_bit;

	/* Restart I2C device */
	_i2c_config(i2c_info);
}

void YUNEECI2CDriver::_i2c_config(struct I2C_Info &i2c_info) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

	/* Disable I2C device */
	i2c_info.i2c->CR1 &= ~I2C_CR1_PE;

	/* Reset I2C */
    RCC->APB1RSTR |= i2c_info.i2cClk;
    RCC->APB1RSTR &= ~i2c_info.i2cClk;

    /* Disable I2C Clock */
    RCC->APB1ENR &= ~i2c_info.i2cClk;
    /* Disable GPIO Clock */
    RCC->AHBENR |= i2c_info.portClk;

    /* Enable I2Cx SCL and SDA Pin Clock */
    RCC->AHBENR |= i2c_info.portClk;
    /* Connect PXx to I2C_SCL */
    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.port, i2c_info.scl_pinSource, GPIO_AF_4);
    /* Connect PXx to I2C_SDA */
    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.port, i2c_info.sda_pinSource, GPIO_AF_4);
    /* Set GPIO frequency to 50MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /* Select Alternate function mode */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    /* Select output Open Drain type */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    /* Disable internal Pull-up */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    /* Initialize I2Cx SCL Pin */
    GPIO_InitStructure.GPIO_Pin = i2c_info.scl_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.port, &GPIO_InitStructure);
    /* Initialize I2Cx SDA Pin */
    GPIO_InitStructure.GPIO_Pin = i2c_info.sda_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.port, &GPIO_InitStructure);

    /* Enable I2C clock */
    RCC->APB1ENR |= i2c_info.i2cClk;
    /* Initialize I2C_InitStructure to their default values */
    I2C_InitStructure.I2C_Timing              = I2C_FAST_MODE_TIMING;          /* Initialize the I2C_Timing member */
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
    I2C_InitStructure.I2C_AnalogFilter        = I2C_AnalogFilter_Enable;       /* Initialize the I2C_AnalogFilter member */
    I2C_InitStructure.I2C_DigitalFilter       = 0x00;                          /* Initialize the I2C_DigitalFilter member */
    I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */
    I2C_Init((I2C_TypeDef*)i2c_info.i2c, &I2C_InitStructure);
	/* Disable WakeUp from STOP mode */
    i2c_info.i2c->CR1 &= ~I2C_CR1_WUPEN;
    /* Disable all interrupt */
    i2c_info.i2c->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_ADDRIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_NACKIE);
}

#endif
