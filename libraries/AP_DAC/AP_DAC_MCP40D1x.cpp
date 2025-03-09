/*
  DAC backend for MCP40D1x I2C DACs
 */
#include "AP_DAC_config.h"

#if AP_DAC_ENABLED

#include "AP_DAC.h"
#include "AP_DAC_MCP40D1x.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/I2CDevice.h>

// common registers

#define MCP40D1x_ADDR 0x2E
#define REG_MCP40D1x_NOP 0x00
#define REG_MCP40D1x_WIPER_DEFAULT 0x3F

extern const AP_HAL::HAL &hal;

void AP_DAC_MCP40D1x::init(void)
{
    if (params.bus_address <= 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x address is 0");
        return;
    }
    dev = std::move(hal.i2c_mgr->get_device(params.bus, params.bus_address));
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x device is null at %u:%", unsigned(params.bus), unsigned(params.bus_address));
        return;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);

    uint8_t val;
    if (!register_read(REG_MCP40D1x_NOP, val) || val != 0x3F) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MCP40D1x not found 0x%x at %u:%u", unsigned(val), unsigned(params.bus), unsigned(params.bus_address));
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MCP40D1x found");

    dev->set_retries(1);
}

void AP_DAC_MCP40D1x::update(void)
{
    set_voltage(0, volts);
}
// set voltage for a channel
bool AP_DAC_MCP40D1x::set_voltage(uint8_t chan, float v)
{
    // convert voltage to 7-bit value
    uint8_t wiper = uint8_t(v * 127.0f / params.voltage_reference);
    if (wiper > 127) {
        wiper = 127;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // write to data register first
    if (!register_write(REG_MCP40D1x_NOP, wiper)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MCP40D1x wiper write fail");
        return false;
    }

    return true;
}

bool AP_DAC_MCP40D1x::register_read(uint8_t reg, uint8_t &val)
{
    uint8_t v;
    if (!dev->read_registers(reg, (uint8_t*)&v, sizeof(v))) {
        return false;
    }
    val = v;
    return true;
}

bool AP_DAC_MCP40D1x::register_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] { reg, val };
    return dev->transfer(buf, sizeof(buf), nullptr, 0);
}

#endif // AP_DAC_ENABLED
