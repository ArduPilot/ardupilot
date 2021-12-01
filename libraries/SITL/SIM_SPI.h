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
  Simulated spi buses and devices
*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "stdint.h"

namespace SITL {

class SPI {
public:
    SPI() {}

    void init();

    // update spi state
    void update(const class Aircraft &aircraft);

    int ioctl(uint8_t bus, uint8_t cs_pin, uint8_t ioctl_type, void *data);

    // the following must be identical to AP_HAL_SITL/SPIDevice.h
    struct spi_ioc_transfer {
        uint64_t tx_buf;
        uint64_t rx_buf;

        uint32_t len;
        // uint32_t speed_hz;

        // uint16_t delay_usecs;
        // uint8_t bits_per_word;
        // uint8_t cs_change;
    };

#define SPI_TRANSACTION_1LONG 17
#define SPI_TRANSACTION_2LONG 18
    // end "the following"

private:
    int ioctl_transaction(uint8_t bus, uint8_t cs_pin, uint8_t count, spi_ioc_transfer *data);

};

}
