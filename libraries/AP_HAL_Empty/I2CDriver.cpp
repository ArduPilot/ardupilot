
#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"

using namespace Empty;

void I2CDriver::begin() {}
void I2CDriver::end() {}
void I2CDriver::setTimeout(uint16_t ms) {}
void I2CDriver::setHighSpeed(bool active) {}

uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 1;} 
uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 1;}
uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 1;}

uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    memset(data, 0, len);
    return 0;
}
uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    *data = 0;
    return 1;
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    memset(data, 0, len);    
    return 1;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
uint8_t I2CDriver::readRegistersMultiple(uint8_t addr, uint8_t reg,
                                         uint8_t len, uint8_t count, 
                                         uint8_t* data)
{
    memset(data, 0, len*count);
    return 1;
}
#endif

uint8_t I2CDriver::lockup_count() {return 0;}
