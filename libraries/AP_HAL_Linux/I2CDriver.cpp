
#include <AP_HAL.h>
#include "I2CDriver.h"

using namespace Linux;

void LinuxI2CDriver::begin() {}
void LinuxI2CDriver::end() {}
void LinuxI2CDriver::setTimeout(uint16_t ms) {}
void LinuxI2CDriver::setHighSpeed(bool active) {}

uint8_t LinuxI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;} 
uint8_t LinuxI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 0;}
uint8_t LinuxI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}

uint8_t LinuxI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;}
uint8_t LinuxI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{return 0;}
uint8_t LinuxI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{return 0;}

uint8_t LinuxI2CDriver::lockup_count() {return 0;}
