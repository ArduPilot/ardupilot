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
 */
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

// Flymaple I2CDriver

#include "I2CDriver.h"
#include "FlymapleWirish.h"
#include <Wire.h>

using namespace AP_HAL_FLYMAPLE_NS;

static TwoWire twowire(5, 9); // Flymaple has non-standard SCL, SDA
void FLYMAPLEI2CDriver::begin() 
{
    twowire.begin();
}

void FLYMAPLEI2CDriver::end() {}

void FLYMAPLEI2CDriver::setTimeout(uint16_t ms) {}
void FLYMAPLEI2CDriver::setHighSpeed(bool active) {}

uint8_t FLYMAPLEI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    twowire.beginTransmission(addr);
    twowire.send(data, len);
    uint8_t result = twowire.endTransmission();
    return result;
} 

uint8_t FLYMAPLEI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    return writeRegisters(addr, reg, 1, &val);
}

uint8_t FLYMAPLEI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{
    twowire.beginTransmission(addr);
    twowire.send(reg);
    if (len)
	twowire.send(data, len);
    uint8_t result = twowire.endTransmission();
    return result;
}

uint8_t FLYMAPLEI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    uint8_t actual_len = twowire.requestFrom(addr, len);
    for (uint8_t i = 0; i < actual_len; i++)
	*data++ = twowire.receive();
    return actual_len != len;
}

uint8_t FLYMAPLEI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    return readRegisters(addr, reg, 1, data);
}

uint8_t FLYMAPLEI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{
    writeRegisters(addr, reg, 0, NULL); // Tell device which register we want
    return read(addr, len, data);
}

uint8_t FLYMAPLEI2CDriver::lockup_count() {return 0;}

#endif
