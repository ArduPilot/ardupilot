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

#ifndef __AP_HAL_FLYMAPLE_SPIDRIVER_H__
#define __AP_HAL_FLYMAPLE_SPIDRIVER_H__

#include "AP_HAL_FLYMAPLE.h"
#include "Semaphores.h"

class HardwareSPI; // in libmaple

class AP_HAL_FLYMAPLE_NS::FLYMAPLESPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    FLYMAPLESPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    FLYMAPLESemaphore _semaphore;
};

class AP_HAL_FLYMAPLE_NS::FLYMAPLESPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    FLYMAPLESPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice, uint8_t index);
private:
    FLYMAPLESPIDeviceDriver _device;
};

#endif // __AP_HAL_FLYMAPLE_SPIDRIVER_H__
