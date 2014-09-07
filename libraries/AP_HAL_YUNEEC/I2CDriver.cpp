#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "I2CDriver.h"
#include <stm32f37x.h>
#include <stm32f37x_rcc.h>
#include <stm32f37x_i2c_cpal_conf.h>
#include <stm32f37x_i2c_cpal_hal.h>
#include <stm32f37x_i2c_cpal.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;
extern I2C_TypeDef* CPAL_I2C_DEVICE[];

// I2C clock: HSI
#define I2C_FAST_MODE_TIMING		0x00310309
#define I2C_STANDARD_MODE_TIMING 	0x10420F13

void YUNEECI2CDriver::begin() {
	/* Set SYSCLK as I2C clock source */
	if (_I2C_DevStructure->CPAL_Dev == CPAL_I2C1)
		RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	else
		RCC_I2CCLKConfig(RCC_I2C2CLK_HSI);

	/* Configure the device structure */
	CPAL_I2C_StructInit(_I2C_DevStructure);      /* Set all fields to default values */

    /* Configure the device mode to master */
    _I2C_DevStructure->wCPAL_Options = CPAL_OPT_I2C_AUTOMATIC_END | CPAL_OPT_I2C_NOSTOP | CPAL_OPT_I2C_NOSTOP_MODE;
	_I2C_DevStructure->pCPAL_I2C_Struct->I2C_Timing = I2C_FAST_MODE_TIMING;
	_I2C_DevStructure->pCPAL_TransferRx = &_sRxStructure;
	_I2C_DevStructure->pCPAL_TransferTx = &_sTxStructure;

	/* Initialize CPAL device with the selected parameters */
	CPAL_I2C_Init(_I2C_DevStructure);
}

void YUNEECI2CDriver::end() {
	CPAL_I2C_DeInit(_I2C_DevStructure);
}

void YUNEECI2CDriver::setTimeout(uint16_t ms) {
	_timeout  = ((uint32_t)ms);
}

void YUNEECI2CDriver::setHighSpeed(bool active) {
    if (active) {
    	_I2C_DevStructure->pCPAL_I2C_Struct->I2C_Timing = I2C_FAST_MODE_TIMING;
    } else {
    	_I2C_DevStructure->pCPAL_I2C_Struct->I2C_Timing = I2C_STANDARD_MODE_TIMING;
    }

    I2C_Init(CPAL_I2C_DEVICE[_I2C_DevStructure->CPAL_Dev], _I2C_DevStructure->pCPAL_I2C_Struct);
}

uint8_t YUNEECI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data) {
	// if not initiated, try to begin i2c cpal
	if (_I2C_DevStructure->CPAL_State == CPAL_STATE_DISABLED)
		begin();

	/* Initialize local Transmission structures */
	_sTxStructure.pbBuffer 	= data;
	_sTxStructure.wNumData 	= (uint32_t)len;   /* Maximum Number of data to be received */
	_sTxStructure.wAddr1 	= (uint32_t)addr << 1;    /* The write device address */
	_sTxStructure.wAddr2 	= (uint32_t)0;       /* Not needed */

	// Disable i2c register mode
	_I2C_DevStructure->wCPAL_Options |= CPAL_OPT_NO_MEM_ADDR;

	/* Force the CPAL state to ready */
	_I2C_DevStructure->CPAL_State = CPAL_STATE_READY;
	_I2C_DevStructure->wCPAL_DevError = CPAL_I2C_ERR_NONE;

	/* Start writing data in master mode */
	if (CPAL_I2C_Write(_I2C_DevStructure) == CPAL_PASS) {
		uint32_t time_start = hal.scheduler->millis();
		// wait for writing complete
		while( (_I2C_DevStructure->CPAL_State != CPAL_STATE_READY) && \
			   (_I2C_DevStructure->CPAL_State != CPAL_STATE_ERROR) && \
			   ((hal.scheduler->millis() - time_start) < _timeout))
			;
		// if no error occurs, return success
		if (_I2C_DevStructure->CPAL_State == CPAL_STATE_READY)
			return 0;
	}

    // transmission failed
    if (!_ignore_errors)
        _lockup_count++;

	return 1;
}

uint8_t YUNEECI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
	return 	writeRegisters(addr, reg, 1, &val);
}

uint8_t YUNEECI2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	// if not initiated, try to begin i2c cpal
	if (_I2C_DevStructure->CPAL_State == CPAL_STATE_DISABLED)
		begin();

	/* Initialize local Transmission structures */
	_sTxStructure.pbBuffer = data;
	_sTxStructure.wNumData = (uint32_t)len;   /* Maximum Number of data to be received */
	_sTxStructure.wAddr1 = (uint32_t)addr << 1;    /* The write device address */
	_sTxStructure.wAddr2 = (uint32_t)reg;       /* The write register address */

	// Enable i2c register mode
	_I2C_DevStructure->wCPAL_Options &= ~CPAL_OPT_NO_MEM_ADDR;

	/* Force the CPAL state to ready */
	_I2C_DevStructure->CPAL_State = CPAL_STATE_READY;
	_I2C_DevStructure->wCPAL_DevError = CPAL_I2C_ERR_NONE;

	/* Start writing data in master mode */
	if (CPAL_I2C_Write(_I2C_DevStructure) == CPAL_PASS) {
		// wait for writing complete
		uint32_t time_start = hal.scheduler->millis();
		while( (_I2C_DevStructure->CPAL_State != CPAL_STATE_READY) && \
			   (_I2C_DevStructure->CPAL_State != CPAL_STATE_ERROR) && \
			   ((hal.scheduler->millis() - time_start) < _timeout))
			;
		// if no error occurs, return success
		if (_I2C_DevStructure->CPAL_State == CPAL_STATE_READY)
			return 0;
	}

    if (!_ignore_errors)
        _lockup_count++;

	return 1;
}

uint8_t YUNEECI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data) {
	// if not initiated, try to begin i2c cpal
	if (_I2C_DevStructure->CPAL_State == CPAL_STATE_DISABLED)
		begin();

	memset(data, 0, len);

	/* Initialize local Reception structures */
	_sRxStructure.pbBuffer = data;
	_sRxStructure.wNumData = (uint32_t)len;   /* Maximum Number of data to be received */
	_sRxStructure.wAddr1 = (uint32_t)addr << 1;    /* The write device address */
	_sRxStructure.wAddr2 = (uint32_t)0;       /* Not needed */

	// Disable i2c register mode
	_I2C_DevStructure->wCPAL_Options |= CPAL_OPT_NO_MEM_ADDR;

	/* Force the CPAL state to ready (in case a read operation has been initiated) */
	_I2C_DevStructure->CPAL_State = CPAL_STATE_READY;
	_I2C_DevStructure->wCPAL_DevError = CPAL_I2C_ERR_NONE;

	/* Start reading data in master mode */
	if (CPAL_I2C_Read(_I2C_DevStructure) == CPAL_PASS) {
		uint32_t time_start = hal.scheduler->millis();
		// wait for reading complete
		while( (_I2C_DevStructure->CPAL_State != CPAL_STATE_READY) && \
			   (_I2C_DevStructure->CPAL_State != CPAL_STATE_ERROR) && \
			   ((hal.scheduler->millis() - time_start) < _timeout))
			;
		// if no error occurs, return success
		if (_I2C_DevStructure->CPAL_State == CPAL_STATE_READY)
			return 0;
	}

	if (!_ignore_errors)
		_lockup_count++;

	return 1;
}
uint8_t YUNEECI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
    return readRegisters(addr, reg, 1, data);
}

uint8_t YUNEECI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	// if not initiated, try to begin i2c cpal
	if (_I2C_DevStructure->CPAL_State == CPAL_STATE_DISABLED)
		begin();

	memset(data, 0, len);

	/* Initialize local Reception structures */
	_sRxStructure.pbBuffer = data;
	_sRxStructure.wNumData = (uint32_t)len;   /* Maximum Number of data to be received */
	_sRxStructure.wAddr1 = (uint32_t)addr << 1;    /* The write device address */
	_sRxStructure.wAddr2 = (uint32_t)reg;       /* The write register address */

	// Enable i2c register mode
	_I2C_DevStructure->wCPAL_Options &= ~CPAL_OPT_NO_MEM_ADDR;

	/* Force the CPAL state to ready (in case a read operation has been initiated) */
	_I2C_DevStructure->CPAL_State = CPAL_STATE_READY;
	_I2C_DevStructure->wCPAL_DevError = CPAL_I2C_ERR_NONE;

	/* Start reading data in master mode */
	if (CPAL_I2C_Read(_I2C_DevStructure) == CPAL_PASS) {
		uint32_t time_start = hal.scheduler->millis();
		// wait for reading complete
		while( (_I2C_DevStructure->CPAL_State != CPAL_STATE_READY) && \
			   (_I2C_DevStructure->CPAL_State != CPAL_STATE_ERROR) && \
			   ((hal.scheduler->millis() - time_start) < _timeout))
			;
		// if no error occurs, return success
		if (_I2C_DevStructure->CPAL_State == CPAL_STATE_READY)
			return 0;
	}

	if (!_ignore_errors)
		_lockup_count++;

    return 1;
}

uint8_t YUNEECI2CDriver::lockup_count() {
	return _lockup_count;;
}

#endif
