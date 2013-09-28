
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "I2CDriver.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace Linux;

/*
  constructor
 */
LinuxI2CDriver::LinuxI2CDriver(AP_HAL::Semaphore* semaphore, const char *device) : 
    _semaphore(semaphore),
    _fd(-1),
    _device(device)
{
}

/*
  called from HAL class init()
 */
void LinuxI2CDriver::begin() 
{
    if (_fd != -1) {
        close(_fd);
    }
    _fd = open(_device, O_RDWR);
}

void LinuxI2CDriver::end() 
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }
}

/*
  tell the I2C library what device we want to talk to
 */
bool LinuxI2CDriver::set_address(uint8_t addr)
{
    if (_fd == -1) {
        return false;
    }
    if (_addr != addr) {
        ioctl(_fd, I2C_SLAVE, addr);
        _addr = addr;
    }
    return true;
}

void LinuxI2CDriver::setTimeout(uint16_t ms) 
{
    // unimplemented
}

void LinuxI2CDriver::setHighSpeed(bool active) 
{
    // unimplemented    
}

uint8_t LinuxI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::write(_fd, data, len) != len) {
        return 1;
    }
    return 0; // success
}


uint8_t LinuxI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                       uint8_t len, uint8_t* data)
{
    uint8_t buf[len+1];
    buf[0] = reg;
    if (len != 0) {
        memcpy(&buf[1], data, len);
    }
    return write(addr, len+1, buf);
}

uint8_t LinuxI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (!set_address(addr)) {
        return 1;
    }
    return i2c_smbus_write_byte_data(_fd, reg, val);
}

uint8_t LinuxI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    if (::read(_fd, data, len) != len) {
        return 1;
    }
    return 0;
}

uint8_t LinuxI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    // send the address to read from
    if (::write(_fd, &reg, 1) != 1) {
        return 1;
    }
    if (::read(_fd, data, len) != len) {
        return 1;
    }
    return 0;
}


uint8_t LinuxI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    if (!set_address(addr)) {
        return 1;
    }
    int32_t v = i2c_smbus_read_byte_data(_fd, reg);
    if (v == -1) {
        return 1;
    }
    *data = v & 0xFF;
    return 0;
}

uint8_t LinuxI2CDriver::lockup_count() 
{
    return 0;
}
#endif // CONFIG_HAL_BOARD
