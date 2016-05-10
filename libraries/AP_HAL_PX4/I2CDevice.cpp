#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>

namespace PX4 {

I2CDevice::PX4_I2C::PX4_I2C(uint8_t bus)
    : I2C("AP_I2C", "/dev/api2c", bus, 0, 400000UL)
{
    init();
}

bool I2CDevice::PX4_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                                     uint8_t *recv, uint32_t recv_len)
{
    set_address(address);
    return transfer(send, send_len, recv, recv_len) == OK;
}

I2CDevice::I2CDevice(uint8_t bus, uint8_t address)
    : _device(bus)
    , _address(address)
{
}

I2CDevice::~I2CDevice()
{
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    return _device.do_transfer(_address, send, send_len, recv, recv_len);
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    while (times > 0) {
        if (!transfer(nullptr, 0, recv, recv_len)) {
            return false;
        }

        recv += recv_len;
        times--;
    }
    return true;
}

int I2CDevice::get_fd()
{
    return -1;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &semaphore;
}

I2CDeviceManager::I2CDeviceManager()
{
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    return dev;
}

}
