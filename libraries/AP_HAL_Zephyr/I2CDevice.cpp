/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "I2CDevice.h"
#include "hwdef.h"

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/sys_io.h>
#endif

using namespace Zephyr;

#if defined(CONFIG_SOC_MIMXRT1176_CM7) || defined(CONFIG_SOC_SERIES_IMXRT11XX)
/* Hard-reset an LPI2C controller after an abandoned transfer: the peripheral can
 * hold SDA low indefinitely, which wedges every other device on the bus. */
#define AP_LPI2C_MCR_OFFSET   0x10U
#define AP_LPI2C_MCR_MEN      (1U << 0)   /* master enable */
#define AP_LPI2C_MCR_RST      (1U << 1)   /* software reset */

static const struct device *i2c_device_for_bus(uint8_t bus);

/* One entry per LPI2C the devicetree declares, matched by DEVICE pointer rather
 * than bus index - the two do not correspond. */
#define AP_LPI2C_ENTRY(label)                                             \
    { DEVICE_DT_GET(DT_NODELABEL(label)), DT_REG_ADDR(DT_NODELABEL(label)) },

static void ap_lpi2c_hard_reset(uint8_t bus)
{
    static const struct {
        const struct device *dev;
        uint32_t base;
    } lpi2c_map[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c1), okay)
        AP_LPI2C_ENTRY(lpi2c1)
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c2), okay)
        AP_LPI2C_ENTRY(lpi2c2)
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c3), okay)
        AP_LPI2C_ENTRY(lpi2c3)
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c4), okay)
        AP_LPI2C_ENTRY(lpi2c4)
#endif
    };

    const struct device *dev = i2c_device_for_bus(bus);
    if (dev == nullptr) {
        return;
    }
    for (const auto &entry : lpi2c_map) {
        if (entry.dev != dev) {
            continue;
        }
        const uint32_t mcr = entry.base + AP_LPI2C_MCR_OFFSET;
        sys_write32(AP_LPI2C_MCR_RST, mcr);   /* assert reset: flushes FIFOs */
        sys_write32(0U, mcr);                 /* release */
        sys_write32(AP_LPI2C_MCR_MEN, mcr);   /* re-enable master */
        return;
    }
}
#else
static void ap_lpi2c_hard_reset(uint8_t) {}
#endif


#ifdef __ZEPHYR__
/* ArduPilot I2C bus index → Zephyr controller, generated from hwdef.dat's
   I2C_ORDER by zephyr_hwdef.py. Boards without I2C_ORDER (or with the
   controllers disabled in DTS) fall through to nullptr. */
static const struct device *i2c_device_for_bus(uint8_t bus)
{
#ifdef HAL_I2C_DT_DEVICE_LOOKUP
    HAL_I2C_DT_DEVICE_LOOKUP(bus)
#else
    (void)bus;
#endif
    return nullptr;
}
#endif

I2CDevice::I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, uint32_t timeout_ms) :
    _bus(bus),
    _address(address),
    _bus_clock(bus_clock),
    _timeout_ms(timeout_ms)
{
    set_device_bus(bus);
    set_device_address(address);
    _bus_handle = DeviceBus::get_bus(bus, (uint8_t)AP_HAL::Device::BUS_TYPE_I2C);

#ifdef __ZEPHYR__
    _dev = i2c_device_for_bus(bus);
    if (_dev != nullptr && device_is_ready(_dev)) {
        (void)set_speed(AP_HAL::Device::SPEED_HIGH);
    }
#endif
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    if (_bus_handle == nullptr) {
        return nullptr;
    }
    return _bus_handle->register_periodic_callback(period_usec, cb, this);
}

bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                         uint32_t period_usec)
{
    if (_bus_handle == nullptr) {
        return false;
    }
    return _bus_handle->adjust_timer(h, period_usec);
}

bool I2CDevice::set_speed(AP_HAL::Device::Speed speed)
{
#ifdef __ZEPHYR__
    if (_dev == nullptr || !device_is_ready(_dev)) {
        return false;
    }

    uint32_t bitrate = _bus_clock;
    if (speed == AP_HAL::Device::SPEED_LOW) {
        bitrate = 100000U;
    }

    uint32_t cfg = I2C_MODE_CONTROLLER;
    if (bitrate <= 100000U) {
        cfg |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
    } else {
        cfg |= I2C_SPEED_SET(I2C_SPEED_FAST);
    }

    return i2c_configure(_dev, cfg) == 0;
#else
    (void)speed;
    return true;
#endif
}



bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef __ZEPHYR__
    if (_dev == nullptr || !device_is_ready(_dev)) {
        return false;
    }

    const uint8_t attempts = (_retries == 0U) ? 1U : (uint8_t)(_retries + 1U);
    for (uint8_t i = 0; i < attempts; i++) {
        int ret = 0;

        if (send != nullptr && send_len > 0U && recv != nullptr && recv_len > 0U) {
            if (_split_transfers) {
                ret = i2c_write(_dev, send, send_len, _address);
                if (ret == 0) {
                    ret = i2c_read(_dev, recv, recv_len, _address);
                }
            } else {
                ret = i2c_write_read(_dev, _address, send, send_len, recv, recv_len);
            }
        } else if (send != nullptr && send_len > 0U) {
            ret = i2c_write(_dev, send, send_len, _address);
        } else if (recv != nullptr && recv_len > 0U) {
            ret = i2c_read(_dev, recv, recv_len, _address);
        } else {
            ret = 0;
        }

        if (ret == 0) {
            return true;
        }
    }

    /* Every attempt failed. DISARM THE PERIPHERAL before giving up, or the next
     * transfer inherits a controller still mid-transaction. */
    ap_lpi2c_hard_reset(_bus);
    /* re-apply THIS device's requested clock, not an unconditional FAST:
       a 100 kHz device (e.g. the INA2xx battery backend) would otherwise
       find its bus silently reverted to 400 kHz after any hiccup */
    {
        uint32_t cfg = I2C_MODE_CONTROLLER;
        cfg |= (_bus_clock <= 100000U) ? I2C_SPEED_SET(I2C_SPEED_STANDARD)
                                       : I2C_SPEED_SET(I2C_SPEED_FAST);
        (void)i2c_configure(_dev, cfg);
    }

    return false;
#else
    (void)send;
    (void)send_len;
    (void)recv;
    (void)recv_len;
    return true;
#endif
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        if (!transfer(&first_reg, 1U, recv, recv_len)) {
            return false;
        }
        recv += recv_len;
    }
    return true;
}

AP_HAL::I2CDevice *I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                                     uint32_t bus_clock,
                                                     bool use_smbus,
                                                     uint32_t timeout_ms)
{
    (void)use_smbus;

    AP_HAL::I2CDevice *dev = NEW_NOTHROW I2CDevice(bus, address, bus_clock, timeout_ms);
    return dev;
}

uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return 0x0FU;
}

uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return 0x0EU;
}

uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    return 0x01U;
}


#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
