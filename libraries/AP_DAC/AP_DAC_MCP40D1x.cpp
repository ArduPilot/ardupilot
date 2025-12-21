/*
  DAC backend for MCP40D1x I2C DACs
 */
#include "AP_DAC_config.h"

#if AP_DAC_MCP40D1X_ENABLED

#include "AP_DAC.h"
#include "AP_DAC_MCP40D1x.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/I2CDevice.h>

// common registers

#define MCP40D1x_ADDR 0x2E
#define REG_MCP40D1x_NOP 0x00
#define REG_MCP40D1x_WIPER_DEFAULT 0x3F

#ifndef AP_DAC_MCP40D1X_CONVERSION_EQ
#define AP_DAC_MCP40D1X_CONVERSION_EQ(vo,vr) (vo * 127 / vr)
#endif

extern const AP_HAL::HAL &hal;

void AP_DAC_MCP40D1x::init(void)
{
    if (params.bus_address <= 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x must be >0");
        return;
    }
    if (!dev) {
        dev = hal.i2c_mgr->get_device_ptr(params.bus, params.bus_address, 100000, true, 20);
    }

    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x device is null at %u:%", unsigned(params.bus), unsigned(params.bus_address));
        return;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t wiper_default = 0;
    if (!dev->read_registers(REG_MCP40D1x_NOP, &wiper_default, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x not found at %u:%x", unsigned(params.bus), unsigned(params.bus_address));
        delete dev;
        return;
    }
    initialized = true;
}

void AP_DAC_MCP40D1x::update(void)
{
    set_voltage(0, params.voltage);
}

// set voltage for a channel
bool AP_DAC_MCP40D1x::set_voltage(uint8_t chan, float v)
{
    if (!initialized) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MCP40D1x not initialized");
        return false;
    }

    // convert voltage to 7-bit value
    uint8_t wiper = uint8_t(roundf(AP_DAC_MCP40D1X_CONVERSION_EQ(v, params.voltage_reference)));
    if (wiper > INT8_MAX) {
        wiper = INT8_MAX;
    }

    // Currently only suitable for AP_Periph as called from the main thread
    WITH_SEMAPHORE(dev->get_semaphore());

    // write to data register first
    if (!dev->write_register(REG_MCP40D1x_NOP, wiper, true)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MCP40D1x wiper write fail: %x", unsigned(wiper));
        return false;
    }

    return true;
}

#endif // AP_DAC_MCP40D1X_ENABLED
