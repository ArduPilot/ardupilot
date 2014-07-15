/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
  Uses the low level libmaple i2c library.
  Caution: requires fixes against the libmaple git master as of 2013-10-10
 */
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

// Flymaple I2CDriver

#include "I2CDriver.h"
#include "FlymapleWirish.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

// This is the instance of the libmaple I2C device to use
#define YUNEEC_I2C_DEVICE I2C1

YUNEECI2CDriver::YUNEECI2CDriver(AP_HAL::Semaphore* semaphore)
    : _semaphore(semaphore),
      _timeout_ms(0) 
{
}

void YUNEECI2CDriver::begin()
{
    _reset();
}

void YUNEECI2CDriver::end() {}

void YUNEECI2CDriver::setTimeout(uint16_t ms)
{
    _timeout_ms = ms;
}

void YUNEECI2CDriver::setHighSpeed(bool active) {}

uint8_t YUNEECI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    i2c_msg msgs[1];
    msgs[0].addr = addr;
    msgs[0].flags = 0; // Write
    msgs[0].length = len;
    msgs[0].data = data;
    return _transfer(msgs, 1);
} 

uint8_t YUNEECI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    return writeRegisters(addr, reg, 1, &val);
}

uint8_t YUNEECI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{
    uint8_t buffer[100];
    buffer[0] = reg;
    memcpy(buffer+1, data, len);

    i2c_msg msgs[1];
    msgs[0].addr = addr;
    msgs[0].flags = 0; // Write
    msgs[0].length = len + 1;
    msgs[0].data = buffer;
    return _transfer(msgs, 1);
}

uint8_t YUNEECI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    // For devices that do not honour normal register conventions (not on YUNEEC?)
    // Now read it
    i2c_msg msgs[1];
    msgs[0].addr = addr;
    msgs[0].flags = I2C_MSG_READ;
    msgs[0].length = len;
    msgs[0].data = data;
    return _transfer(msgs, 1);
}

uint8_t YUNEECI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    return readRegisters(addr, reg, 1, data);
}

uint8_t YUNEECI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{
    // We conduct a write of the register number we want followed by a read
    data[0] = reg; // Temporarily steal this for the write
    i2c_msg msgs[2];
    msgs[0].addr = addr;
    msgs[0].flags = 0; // Write
    msgs[0].length = 1;
    msgs[0].data = data;
    // Second transaction is a read
    msgs[1].addr = addr;
    msgs[1].flags = I2C_MSG_READ;
    msgs[1].length = len;
    msgs[1].data = data;
    return _transfer(msgs, 2);
}

uint8_t YUNEECI2CDriver::lockup_count() {
return _lockup_count;
}

uint8_t            YUNEECI2CDriver::_transfer(i2c_msg *msgs, uint16 num)
{
    // ALERT: patch to libmaple required for this to work else
    // crashes next line due to a bug in latest git libmaple see http://forums.leaflabs.com/topic.php?id=13458
    int32 result = i2c_master_xfer(I2C1, msgs, num, _timeout_ms);
    if (result != 0) {
        // Some sort of I2C bus fault, or absent device, reinitialise the bus
        _reset();
        if (!_ignore_errors) {
            _lockup_count++;
        }
    }
    return result != 0;
}

void YUNEECI2CDriver::_reset()
{
    i2c_disable(YUNEEC_I2C_DEVICE);
    i2c_master_enable(YUNEEC_I2C_DEVICE, I2C_FAST_MODE | I2C_BUS_RESET);
}

#endif
