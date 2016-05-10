#pragma once

#include <vector>

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <drivers/device/i2c.h>
#include <arch/board/board.h>
#include "AP_HAL_PX4.h"
#include "board_config.h"

namespace PX4 {

class I2CDevice : public AP_HAL::I2CDevice {
public:
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    /* AP_HAL::I2CDevice implementation */
    I2CDevice(uint8_t bus, uint8_t address);

    ~I2CDevice();

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override { _retries = retries; }

    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::set_speed(): Empty implementation, not supported. */
    bool set_speed(enum Device::Speed speed) override { return true; }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle *register_periodic_callback(uint32_t period_usec, AP_HAL::MemberProc) override
    {
        /* Not implemented yet */
        return nullptr;
    };

    /* See AP_HAL::Device::get_fd() */
    int get_fd() override;

protected:
    class PX4_I2C : public device::I2C {
    public:
        PX4_I2C(uint8_t bus);
        bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);
    };

    PX4_I2C _device;
    uint8_t _address;
    uint8_t _retries = 0;

private:
    // we use an empty semaphore as the underlying I2C class already has a semaphore
    Empty::Semaphore semaphore;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    I2CDeviceManager();

    /* AP_HAL::I2CDeviceManager implementation */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) override;
};
}
