
#include "I2CDriver.h"

using namespace Empty;

void EmptyI2CDriver::begin() {}
void EmptyI2CDriver::end() {}
void EmptyI2CDriver::setTimeout(uint16_t ms) {}
void EmptyI2CDriver::setHighSpeed(bool active) {}

uint8_t EmptyI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;} 
uint8_t EmptyI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 0;}
uint8_t EmptyI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}

uint8_t EmptyI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;}
uint8_t EmptyI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{return 0;}
uint8_t EmptyI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{return 0;}

uint8_t EmptyI2CDriver::lockup_count() {return 0;}
