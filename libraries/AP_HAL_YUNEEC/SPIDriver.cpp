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

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

// Flymaple SPI driver
// Flymaple has no actual SPI devices, so this generic

#include "SPIDriver.h"
#include "FlymapleWirish.h"

using namespace AP_HAL_YUNEEC;

#define YUNEEC_SPI_CS_PIN 8
// Only one port so far:
#define YUNEEC_SPI_PORT 1
HardwareSPI spi(YUNEEC_SPI_PORT);


YUNEECSPIDeviceDriver::YUNEECSPIDeviceDriver()
{}

void YUNEECSPIDeviceDriver::init()
{
    spi.begin(SPI_9MHZ, MSBFIRST, SPI_MODE_0);
    digitalWrite(YUNEEC_SPI_CS_PIN, 1);
    pinMode(YUNEEC_SPI_CS_PIN, OUTPUT);
}

AP_HAL::Semaphore* YUNEECSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void YUNEECSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    cs_assert();
    if (rx == NULL) {
        for (uint16_t i = 0; i < len; i++) {
            transfer(tx[i]);
        }
    } else {
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = transfer(tx[i]);
        }
    }
    cs_release();
}


void YUNEECSPIDeviceDriver::cs_assert()
{
    digitalWrite(YUNEEC_SPI_CS_PIN, 0);
}

void YUNEECSPIDeviceDriver::cs_release()
{
    digitalWrite(YUNEEC_SPI_CS_PIN, 1);
}

uint8_t YUNEECSPIDeviceDriver::transfer (uint8_t data)
{
    return spi.transfer(data);
}

void YUNEECSPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
    spi.write(data, len);
}

YUNEECSPIDeviceManager::YUNEECSPIDeviceManager()
{
}

void YUNEECSPIDeviceManager::init(void *)
{
}

AP_HAL::SPIDeviceDriver* YUNEECSPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    _device.init(); // Defer this until GPIO pin 13 is set up else its a slave
    return &_device;
}

#endif
