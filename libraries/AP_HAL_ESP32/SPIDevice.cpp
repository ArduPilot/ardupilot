#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

using namespace ESP32;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

ESP32::SPIDesc SPIDeviceManager::device_table[] = {
    {
        .name = "MPU9250",
        .device = 1,
        .bus = 2,
        .cs_gpio = 5,
        .mode = 0,
        .lowspeed = 2 * MHZ,
        .highspeed = 8 * MHZ,

    },
    {
        .name = "BMP280",
        .device = 2,
        .bus = 2,
        .cs_gpio = 26,
        .mode = 3,
        .lowspeed =  1 * MHZ,
        .highspeed = 1 * MHZ,
    }
};


//two available spi buses : 1 and 2, device 0 is connected to internal flash
spi_bus_config_t bus_config[3] = {
    {},
    {},
    {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    },
};

SPIBus::SPIBus(uint8_t _bus):
    DeviceBus(Scheduler::SPI_PRIORITY), bus(_bus)
{
    spi_bus_initialize((spi_host_device_t)_bus, &bus_config[_bus], _bus);
}

SPIDevice::SPIDevice(SPIBus &_bus, SPIDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
    set_device_bus(bus.bus);
    set_device_address(_device_desc.device);
    set_speed(AP_HAL::Device::SPEED_LOW);
    gpio_pad_select_gpio((gpio_num_t)device_desc.cs_gpio);
    gpio_set_direction((gpio_num_t)device_desc.cs_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)device_desc.cs_gpio, 1);

    spi_device_interface_config_t cfg_low;
    memset(&cfg_low, 0, sizeof(cfg_low));
    cfg_low.mode = _device_desc.mode;
    cfg_low.clock_speed_hz = _device_desc.lowspeed;
    cfg_low.spics_io_num = -1;
    cfg_low.queue_size = 5;
    spi_bus_add_device((spi_host_device_t)bus.bus, &cfg_low, &low_speed_dev_handle);

    if (_device_desc.highspeed != _device_desc.lowspeed) {
        spi_device_interface_config_t cfg_high;
        memset(&cfg_high, 0, sizeof(cfg_high));
        cfg_high.mode = _device_desc.mode;
        cfg_high.clock_speed_hz = _device_desc.highspeed;
        cfg_high.spics_io_num = -1;
        cfg_high.queue_size = 5;
        spi_bus_add_device((spi_host_device_t)bus.bus, &cfg_high, &high_speed_dev_handle);
    }


    asprintf(&pname, "SPI:%s:%u:%u",
             device_desc.name, 0, (unsigned)device_desc.device);
    printf("spi device constructed %s\n", pname);
}

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
    speed = _speed;
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
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

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
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
    if (accuire) {
        spi_device_acquire_bus(current_handle(), portMAX_DELAY);
        gpio_set_level((gpio_num_t)device_desc.cs_gpio, 0);
    } else {
        gpio_set_level((gpio_num_t)device_desc.cs_gpio, 1);
        spi_device_release_bus(current_handle());
    }
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }
    SPIDesc &desc = device_table[i];

    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;
        buses = busp;
    }
    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*busp, desc));

}

