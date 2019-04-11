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
 */
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"
#include "driver/i2c.h"

using namespace ESP32;

#ifndef HAL_I2C_INTERNAL_MASK
#define HAL_I2C_INTERNAL_MASK 0
#endif

i2c_config_t i2c_bus_config[1] = {{
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)26,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)25,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        400000
    }
};

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_config)];

I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_config); i++) {
        businfo[i].bus = i;
        i2c_param_config((i2c_port_t)i, &i2c_bus_config[i]);
        i2c_driver_install((i2c_port_t)i, I2C_MODE_MASTER, 0, 0, 0);
        i2c_filter_enable((i2c_port_t)i, 7);
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    _retries(2),
    _address(address),
    bus(I2CDeviceManager::businfo[busnum])
{
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
}

I2CDevice::~I2CDevice()
{
    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (send_len != 0 && send != nullptr) {
        //tx with optional rx (after tx)
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, (uint8_t*)send, send_len, true);
    }
    if (recv_len != 0 && recv != nullptr) {
        //rx only or rx after tx
        //rx separated from tx by (re)start
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, (uint8_t *)recv, recv_len, I2C_MASTER_LAST_NACK);
    }
    i2c_master_stop(cmd);
    bool result = (i2c_master_cmd_begin((i2c_port_t)bus.bus, cmd, portMAX_DELAY) == ESP_OK);
    i2c_cmd_link_delete(cmd);
    return result;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_config)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(i2c_bus_config)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    return get_bus_mask() & HAL_I2C_INTERNAL_MASK;;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~HAL_I2C_INTERNAL_MASK;
}
