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
            i2c_master_bus_config_t i2c_bus_config = {};
            i2c_bus_config.i2c_port = i2c_bus_desc[i].port;
            i2c_bus_config.sda_io_num = i2c_bus_desc[i].sda;
            i2c_bus_config.scl_io_num = i2c_bus_desc[i].scl;
            i2c_bus_config.flags.enable_internal_pullup = true;
            i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            i2c_bus_config.glitch_ignore_cnt = 7;
            businfo[i].port = i2c_bus_desc[i].port;
            businfo[i].bus_clock = i2c_bus_desc[i].speed;
            businfo[i].soft = false;
            esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &businfo[i].bus_handle);
            if (err != ESP_OK) {
                printf("I2C: i2c_new_master_bus failed on port %d: %d\n", (int)businfo[i].port, (int)err);
            }
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(10),
    _address(address),
    _bus_clock(bus_clock),
    _timeout_ms(timeout_ms)
{
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
    if (!bus.soft) {
        _add_device();
    }
}

I2CDevice::~I2CDevice()
{
    if (!bus.soft) {
        _rm_device();
    }
    free(pname);
}

void I2CDevice::set_address(uint8_t address)
{
    if (_address != address) {
        _rm_device();
        _address = address;
        _add_device();
    }
}

void I2CDevice::_add_device()
{
    if (_dev_handle != nullptr || bus.soft || bus.bus_handle == nullptr) {
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = _address;
    const uint32_t clk = (_bus_clock != 0) ? _bus_clock : bus.bus_clock;
    dev_cfg.scl_speed_hz = clk;

    esp_err_t err = i2c_master_bus_add_device(bus.bus_handle, &dev_cfg, &_dev_handle);
    if (err != ESP_OK) {
        printf("I2C: i2c_master_bus_add_device failed for %s: %d\n", pname ? pname : "", (int)err);
        _dev_handle = nullptr;
    }
}

void I2CDevice::_rm_device()
{
    if (_dev_handle != nullptr && !bus.soft) {
        i2c_master_bus_rm_device(_dev_handle);
        _dev_handle = nullptr;
    }
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
        if (_dev_handle == nullptr) {
            return false;
        }

        const bool has_tx = (send_len != 0 && send != nullptr);
        const bool has_rx = (recv_len != 0 && recv != nullptr);
        if (!has_tx && !has_rx) {
            return false;
        }

        const uint32_t clk = (_bus_clock != 0) ? _bus_clock : (bus.bus_clock ? bus.bus_clock : 100000U);
        uint32_t timeout_ms = 1 + 16L * (send_len + recv_len) * 1000 / clk;
        timeout_ms = MAX(timeout_ms, _timeout_ms);
        timeout_ms = MIN(timeout_ms, 20U);
        for (int i = 0; !result && i < _retries; i++) {
            esp_err_t err = ESP_FAIL;

            if (has_tx && has_rx) {
                err = i2c_master_transmit_receive(_dev_handle, send, send_len, recv, recv_len, (int)timeout_ms);
            } else if (has_tx) {
                err = i2c_master_transmit(_dev_handle, send, send_len, (int)timeout_ms);
            } else {
                err = i2c_master_receive(_dev_handle, recv, recv_len, (int)timeout_ms);
            }

            if (err == ESP_OK) {
                result = true;
            } else if (err == ESP_ERR_TIMEOUT) {
                // Reset bus in case of timeout
                i2c_master_bus_reset(bus.bus_handle);
            }
        
        }

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

AP_HAL::I2CDevice *
I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                 uint32_t bus_clock,
                                 bool use_smbus,
                                 uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        return nullptr;
    }
    return NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms);
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
