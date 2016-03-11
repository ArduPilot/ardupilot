#include "HardwareI2C.h"

#define I2CDELAY 50
//#define DELAYI2C

HardwareI2C::HardwareI2C(uint32_t i2c_num)
{
    switch (i2c_num) {
    case 1:
        this->i2c_d = _I2C1;
        //this->begin();
        break;
    case 2:
        this->i2c_d = _I2C2;
        //this->begin();
        break;
    default:
        assert_param(0);
        break;
    }
}

////////////// Public Methods ////////////////////////////////////////

void HardwareI2C::begin()
{
	/* set as master */
	//i2c_init(this->i2c_d, 0, I2C_400KHz_SPEED);
	//delay(I2CDELAY);
}

void HardwareI2C::end()
{
}

void HardwareI2C::setSpeed(boolean _fast)
{

}

/* WRITE ******************************************/
int8_t HardwareI2C::write(uint8_t address, uint8_t len, uint8_t *tx_buffer)
{

	uint8_t ret = i2c_write(this->i2c_d, address, tx_buffer, len);
	
	//uint8_t ret = i2c_Write(this->i2c_d, address, len, data);
	#ifdef DELAYI2C
		delay(I2CDELAY);
	#endif
	return ret;
}

int8_t HardwareI2C::write(uint8_t address, uint8_t registerAddress, uint8_t databyte)
{
	//uint8_t ret = i2c_8bitaddr_write(this->i2c_d, address, registerAddress, databyte);
	//uint8_t ret = i2c_write(this->i2c_d, address, registerAddress, databyte);
	
	uint8_t ibuff[2];

	ibuff[0] = registerAddress;
	ibuff[1] = databyte;
	
	uint8_t ret = i2c_write(this->i2c_d, address, ibuff, 2);
	
	#ifdef DELAYI2C
		delay(I2CDELAY);
	#endif

	return ret;
}

int8_t HardwareI2C::write(uint8_t address, uint16_t registerAddress, uint8_t databyte)
{
	uint8_t ibuff[3];

	ibuff[0] = (uint8_t)(registerAddress >> 8);
	ibuff[1] = (uint8_t)(registerAddress & 0xFF);
	ibuff[2] = (uint8_t)databyte;
		
	uint8_t ret = i2c_write(this->i2c_d, address, ibuff, 3);
	//uint8_t ret = i2c_write(this->i2c_d, address, registerAddress, databyte);
	
	#ifdef DELAYI2C
		delay(I2CDELAY);
	#endif

	return ret;
}

/* READ *******************************************/

int8_t HardwareI2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
	uint8_t ibuff[1];

	ibuff[0] = (uint8_t)registerAddress;
	
	uint8_t ret = i2c_read(this->i2c_d, address, ibuff, 1, dataBuffer, numberBytes);
	//uint8_t ret = i2c_8bitaddr_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
	//uint8_t ret = i2c_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
#ifdef DELAYI2C
	delay(I2CDELAY);
#endif

	return ret;
}

int8_t HardwareI2C::read(uint8_t address, uint16_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
	uint8_t ibuff[2];

	//preparing I2C buffer
	ibuff[0]=(uint8_t)(registerAddress >> 8);
	ibuff[1]=(uint8_t)(registerAddress & 0xFF);
	
	uint8_t ret = i2c_read(this->i2c_d, address, ibuff, 2, dataBuffer, numberBytes);
	//uint8_t ret = i2c_buffer_read(this->i2c_d, address, registerAddress, numberBytes, dataBuffer);
	
#ifdef DELAYI2C
	delay(I2CDELAY);
#endif

	return ret;
}

