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

#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>

#if AP_HAL_SPI_ENABLED

#include <AP_Math/AP_Math.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

using namespace ESP32;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

//#define SPIDEBUG 1

#ifdef HAL_ESP32_SPI_BUSES
SPIBusDesc bus_desc[] = {HAL_ESP32_SPI_BUSES};
#endif  // HAL_ESP32_SPI_BUSES

#ifdef HAL_ESP32_SPI_DEVICES
SPIDeviceDesc device_desc[] = {HAL_ESP32_SPI_DEVICES};
#endif  // HAL_ESP32_SPI_DEVICES

#ifdef HAL_ESP32_SPI_BUSES
SPIBus::SPIBus(uint8_t _bus):
    DeviceBus(Scheduler::SPI_PRIORITY), bus(_bus)
{

#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    spi_bus_config_t config = {
        .mosi_io_num = bus_desc[_bus].mosi,
        .miso_io_num = bus_desc[_bus].miso,
        .sclk_io_num = bus_desc[_bus].sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(bus_desc[_bus].host, &config, bus_desc[_bus].dma_ch);
}
#endif  // HAL_ESP32_SPI_BUSES


#ifdef HAL_ESP32_SPI_DEVICES
SPIDevice::SPIDevice(SPIBus &_bus, SPIDeviceDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{

#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    set_device_bus(bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);

    spi_device_interface_config_t cfg_low;
    memset(&cfg_low, 0, sizeof(cfg_low));
    cfg_low.mode = _device_desc.mode;
    cfg_low.clock_speed_hz = _device_desc.lspeed;
    cfg_low.spics_io_num = -1;
    cfg_low.queue_size = 5;
    spi_bus_add_device(bus_desc[_bus.bus].host, &cfg_low, &low_speed_dev_handle);

    if (_device_desc.hspeed != _device_desc.lspeed) {
        spi_device_interface_config_t cfg_high;
        memset(&cfg_high, 0, sizeof(cfg_high));
        cfg_high.mode = _device_desc.mode;
        cfg_high.clock_speed_hz = _device_desc.hspeed;
        cfg_high.spics_io_num = -1;
        cfg_high.queue_size = 5;
        spi_bus_add_device(bus_desc[_bus.bus].host, &cfg_high, &high_speed_dev_handle);
    }


    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name, 0, (unsigned)device_desc.device);
    printf("spi device constructed %s\n", pname);
}
#endif  // HAL_ESP32_SPI_DEVICES

SPIDevice::~SPIDevice()
{
    free(pname);
}

spi_device_handle_t SPIDevice::current_handle()
{
    if (speed == AP_HAL::Device::SPEED_HIGH && high_speed_dev_handle != nullptr) {
        return high_speed_dev_handle;
    }
    return low_speed_dev_handle;
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed _speed)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    speed = _speed;
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!send || !recv) {
        // simplest cases
        transfer_fullduplex(send, recv, recv_len?recv_len:send_len);
        return true;
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    transfer_fullduplex(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(uint8_t *send_recv, uint32_t len)
{
    return transfer_fullduplex(send_recv, send_recv, len);
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    acquire_bus(true);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len*8;
    t.tx_buffer = send;
    t.rxlength = len*8;
    t.rx_buffer = recv;
    spi_device_transmit(current_handle(), &t);
    acquire_bus(false);
    return true;
}

void SPIDevice::acquire_bus(bool accuire)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (accuire) {
        spi_device_acquire_bus(current_handle(), portMAX_DELAY);
        gpio_set_level(device_desc.cs, 0);
    } else {
        gpio_set_level(device_desc.cs, 1);
        spi_device_release_bus(current_handle());
    }
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
#ifdef SPIDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

#ifdef HAL_ESP32_SPI_DEVICES
AP_HAL::SPIDevice *
SPIDeviceManager::get_device_ptr(const char *name)
{
#ifdef SPIDEBUG
    printf("%s:%d %s\n", __PRETTY_FUNCTION__, __LINE__, name);
#endif
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
        if (strcmp(device_desc[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_desc)) {
        return nullptr;
    }
    SPIDeviceDesc &desc = device_desc[i];

#ifdef SPIDEBUG
    printf("%s:%d 222\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }

#ifdef SPIDEBUG
    printf("%s:%d 333\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (busp == nullptr) {
        // create a new one
        busp = NEW_NOTHROW SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;
        // deassert all CSes on the bus
        for (i = 0; i<ARRAY_SIZE(device_desc); i++) {
            SPIDeviceDesc &curr_desc = device_desc[i];
            if (desc.bus == curr_desc.bus) {
                esp_rom_gpio_pad_select_gpio(curr_desc.cs);
                gpio_set_direction(curr_desc.cs, GPIO_MODE_OUTPUT);
                gpio_set_level(curr_desc.cs, 1);
            }
        }
        buses = busp;
    }

#ifdef SPIDEBUG
    printf("%s:%d 444\n", __PRETTY_FUNCTION__, __LINE__);
#endif

    return NEW_NOTHROW SPIDevice(*busp, desc);
}
#endif  // HAL_ESP32_SPI_DEVICES

#endif  // AP_HAL_SPI_ENABLED
