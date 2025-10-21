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


#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#include "SIM_SPI.h"
#include "SIM_SPIDevice.h"

#include "SIM_RAMTRON_FM25V02.h"
#include "SIM_JEDEC_MX25L3206E.h"

#include <signal.h>

using namespace SITL;

#if AP_SIM_RAMTRON_FM25V02_ENABLED
static RAMTRON_FM25V02 ramtron_FM25V02;  // 32kB 2-byte-addressing
#endif
#if AP_SIM_JEDEC_MX25L3206E_ENABLED
static JEDEC_MX25L3206E jedec_MX25L3206E;
#endif

struct spi_device_at_cs_pin {
    uint8_t bus;
    uint8_t cs_pin;
    SPIDevice &device;
} spi_devices[] {
#if AP_SIM_RAMTRON_FM25V02_ENABLED
    { 0, 0, ramtron_FM25V02 },
#endif
#if AP_SIM_JEDEC_MX25L3206E_ENABLED
    { 1, 0, jedec_MX25L3206E },
#endif
};

void SPI::init()
{
    for (auto &i : spi_devices) {
        i.device.init();
    }

    // sanity check the spi_devices structure to ensure we don't have
    // two devices at the same address on the same bus:
    for (uint8_t i=0; i<ARRAY_SIZE(spi_devices)-1; i++) {
        const auto &dev_i = spi_devices[i];
        for (uint8_t j=i+1; j<ARRAY_SIZE(spi_devices); j++) {
            const auto &dev_j = spi_devices[j];
            if (dev_i.bus == dev_j.bus &&
                dev_i.cs_pin == dev_j.cs_pin) {
                AP_HAL::panic("Two devices on the same cs_pin on the same bus");
            }
        }
    }
}

void SPI::update(const class Aircraft &aircraft)
{
    for (auto daa : spi_devices) {
        daa.device.update(aircraft);
    }
}

int SPI::ioctl_transaction(uint8_t bus, uint8_t cs_pin, uint8_t count, spi_ioc_transfer *data)
{
    for (auto &dev_at_cs_pin : spi_devices) {
        if (dev_at_cs_pin.cs_pin != cs_pin) {
            continue;
        }
        if (dev_at_cs_pin.bus != bus) {
            continue;
        }
        return dev_at_cs_pin.device.rdwr(count, data);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unhandled spi message: bus=%u cs_pin=%u\n", bus, cs_pin);
    return -1;
}

int SPI::ioctl(uint8_t bus, uint8_t cs_pin, uint8_t ioctl_type, void *data)
{
    uint8_t count;
    switch (ioctl_type) {
    case SPI_TRANSACTION_1LONG:
        count = 1;
        break;
    case SPI_TRANSACTION_2LONG:
        count = 2;
        break;
    default:
        AP_HAL::panic("Bad transaction type");
    }

    return ioctl_transaction(bus, cs_pin, count, (spi_ioc_transfer*)data);
}
