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
#include <AP_HAL_ESP32/Semaphores.h>
#include "Scheduler.h"

using namespace ESP32;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

I2CBusDesc i2c_bus_desc[] = { HAL_ESP32_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            businfo[i].sw_handle.sda = i2c_bus_desc[i].sda;
            businfo[i].sw_handle.scl = i2c_bus_desc[i].scl;
            //TODO make modular
            businfo[i].sw_handle.speed = I2C_SPEED_FAST;
            businfo[i].soft = true;
            i2c_init(&(businfo[i].sw_handle));
        } else {
            i2c_config_t i2c_bus_config;
            i2c_bus_config.mode = I2C_MODE_MASTER;
            i2c_bus_config.sda_io_num = i2c_bus_desc[i].sda;
            i2c_bus_config.scl_io_num = i2c_bus_desc[i].scl;
            i2c_bus_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.master.clk_speed = i2c_bus_desc[i].speed;
            i2c_port_t p = i2c_bus_desc[i].port;
            businfo[i].port = p;
            businfo[i].bus_clock = i2c_bus_desc[i].speed;
            businfo[i].soft = false;
            i2c_param_config(p, &i2c_bus_config);
            i2c_driver_install(p, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
            i2c_filter_enable(p, 7);
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    _retries(10),
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
    if (!bus.semaphore.check_owner()) {
        printf("I2C: not owner of 0x%x\n", (unsigned)get_bus_id());
        return false;
    }

    bool result = false;
    if (bus.soft) {
        uint8_t flag_wr = (recv_len == 0 || recv == nullptr) ? I2C_NOSTOP : 0;
        if (send_len != 0 && send != nullptr) {
            //tx with optional rx (after tx)
            i2c_write_bytes(&bus.sw_handle,
                            _address,
                            send,
                            send_len,
                            flag_wr);
        }
        if (recv_len != 0 && recv != nullptr) {
            //rx only or rx after tx
            //rx separated from tx by (re)start
            i2c_read_bytes(&bus.sw_handle,
                           _address,
                           (uint8_t *)recv, recv_len, 0);
        }
        result = true; //TODO check all
    } else {
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

        TickType_t timeout = 1 + 16L * (send_len + recv_len) * 1000 / bus.bus_clock / portTICK_PERIOD_MS;
        for (int i = 0; !result && i < _retries; i++) {
            result = (i2c_master_cmd_begin(bus.port, cmd, timeout) == ESP_OK);
            if (!result) {
                i2c_reset_tx_fifo(bus.port);
                i2c_reset_rx_fifo(bus.port);
            }
        }

        i2c_cmd_link_delete(cmd);
    }

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
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
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
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    uint32_t result = 0;
    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1u << i);
        }
    }
    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~get_bus_mask_internal();
}
