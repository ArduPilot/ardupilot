/*
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/SPIDevice.h>

#include "DeviceBus.h"
#include "Semaphores.h"

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#endif

namespace Zephyr {

class SPIDevice : public AP_HAL::SPIDevice {
public:
    enum class DeviceId : uint8_t {
        IMU0,
        IMU1,
        IMU2,
        BARO,
        FRAM,
    };

    SPIDevice(const char *name, DeviceId id, uint8_t bus, uint8_t cs_index,
              uint32_t lowspeed_hz, uint32_t highspeed_hz);

    bool set_speed(AP_HAL::Device::Speed speed) override;

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /* In-place full duplex. MUST be overridden: AP_HAL::Device's default is not,
     * and ChibiOS overrides both forms. */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override;

    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    AP_HAL::Semaphore *get_semaphore() override
    {
        // bus semaphore is shared between devices on the same bus,
        // matching AP_HAL_ChibiOS
        return _bus != nullptr ? &_bus->semaphore : &_semaphore;
    }

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec) override;

private:
    const char *_name;
    DeviceId _id;
    DeviceBus *_bus = nullptr;
    uint32_t _speed_hz;
    uint32_t _lowspeed_hz;
    uint32_t _highspeed_hz;
    Semaphore _semaphore;

#ifdef __ZEPHYR__
    const struct spi_dt_spec *_spec = nullptr;
#ifdef __ZEPHYR__
    /* PERSISTENT config, deliberately not a stack temporary: Zephyr's SPI API keeps
     * a pointer to it for the life of the transfer. */
    struct spi_config _cfg[2] {};
    uint8_t _cfg_idx = 0;
    uint32_t _cfg_freq = 0;      // frequency currently in _cfg[_cfg_idx]
    bool _cfg_init = false;
    /* spi_is_ready_dt() walks the device + CS-GPIO ready flags; at ~7.8k
       transfers/s it measured 6.2% of ALL PC samples (PCSR, 2026-08-08,
       370 Hz build). Readiness never regresses once true, so cache it. */
    bool _bus_ready = false;
#endif
#endif
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
    uint8_t get_count() override;
    const char *get_device_name(uint8_t idx) override;
};

}  // namespace Zephyr
