#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/WSPIDevice.h>
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "FreeRTOS.h"
#include "semphr.h"

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)

namespace RP {

class WSPIDevice : public AP_HAL::WSPIDevice {
public:
    WSPIDevice(SemaphoreHandle_t lock);

    void set_cmd_header(const CommandHeader& cmd_hdr) override;

    bool is_busy() override;
    
    // ArduPilot uses this for JEDEC ID and flash operations
    bool transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len) override;
    
    AP_HAL::Semaphore* get_semaphore() override { return &_semaphore; }

    bool set_speed(Speed speed) override { return true; }

    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) override
    {
        return nullptr;
    }
    bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) override
    {
        return false;
    }

private:
    SemaphoreHandle_t _lock;        // FreeRTOS mutex
    HAL_Semaphore _semaphore;   // ArduPilot wrapper
    CommandHeader _current_cmd_hdr; // Save the current command
};

class WSPIDeviceManager : public AP_HAL::WSPIDeviceManager {
public:
    WSPIDeviceManager();
    AP_HAL::OwnPtr<AP_HAL::WSPIDevice> get_device(const char *name) override;

    uint8_t get_count() const override { return 1; }
    const char *get_device_name(uint8_t idx) const override { return "qspi_flash"; }

private:
    SemaphoreHandle_t _flash_mutex;
};

} // namespace RP
#endif // #if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
