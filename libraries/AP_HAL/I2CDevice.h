#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

class I2CDevice : public Device {
public:
    virtual ~I2CDevice() { }

    /*
     * Change device address. Note that this is the 7 bit address, it
     * does not include the bit for read/write.
     */
    virtual void set_address(uint8_t address) = 0;

    /* set number of retries on transfers */
    virtual void set_retries(uint8_t retries) = 0;

    /* Device implementation */

    /* See Device::set_speed() */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* See Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

    /* See Device::register_periodic_callback() */
    virtual Device::PeriodicHandle *register_periodic_callback(
        uint32_t period_usec, MemberProc) override = 0;

    /* See Device::get_fd() */
    virtual int get_fd() override = 0;
};

class I2CDeviceManager {
public:
    /* Get a device handle */
    virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) = 0;
};

}
