/*
  DAC backend for TIx3204 I2C DACs
 */
#include "AP_DAC_config.h"

#if AP_DAC_TIX3204_ENABLED

#include "AP_DAC.h"
#include "AP_DAC_TIx3204.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/I2CDevice.h>

// common registers
#define REG_TIx3204_NOP 0x00
#define REG_TIx3204_COMMON_CONFIG 0x1F

#define REG_TIx3204_CHAN_BASE 0x01 // offset to first channel
#define REG_TIx3204_CHAN_SKIP 6 // 6 registers per channel

// per channel config registers
#define REG_TIx3204_OFS_MARGIN_HIGH      0x00
#define REG_TIx3204_OFS_MARGIN_LOW       0x01
#define REG_TIx3204_OFS_VOUT_CMP_CONFIG  0x02
#define REG_TIx3204_OFS_IOUT_MISC_CONFIG 0x03
#define REG_TIx3204_OFS_CMP_MODE_CONFIG  0x04
#define REG_TIx3204_OFS_FUNC_CONFIG      0x05

// data registers
#define REG_TIx3204_DAC_DATA(chan) ((chan)+0x19)

extern const AP_HAL::HAL &hal;

void AP_DAC_TIx3204::init(void)
{
    if (params.bus_address <= 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TIx3204 address is 0");
        return;
    }
    dev = std::move(hal.i2c_mgr->get_device(params.bus, params.bus_address));
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TIx3204 device is null at %u:%", unsigned(params.bus), unsigned(params.bus_address));
        return;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);

    uint16_t val = 17;
    if (!register_read(REG_TIx3204_NOP, val) || val != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TIx3204 not found 0x%x at %u:%u", unsigned(val), unsigned(params.bus), unsigned(params.bus_address));
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TIx3204 found");

    dev->set_retries(1);
}

// set voltage for a channel
bool AP_DAC_TIx3204::set_voltage(uint8_t chan, float v)
{
    if (chan >= 4) {
        return false;
    }

    // convert voltage to 12-bit value
    uint16_t val = uint16_t(v * 4095.0f / params.voltage_reference);
    if (val > 4095) {
        val = 4095;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // write to data register first
    if (!register_write(REG_TIx3204_DAC_DATA(chan), val<<4)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TIx3204 data write fail");
        return false;
    }

    if (!configured[chan]) {
        uint16_t config;
        if (!register_read(REG_TIx3204_COMMON_CONFIG, config)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TIx3204 chan config read %u fail", unsigned(chan));
            return false;
        }
        const uint8_t config_shift = chan*3;
        const uint16_t config_mask = 0x7 << config_shift;
        const uint16_t config_val = 0x1; // enable Vout
        config = (config & ~config_mask) | (config_val << config_shift);
        if (!register_write(REG_TIx3204_COMMON_CONFIG, config)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TIx3204 chan config %u fail", unsigned(chan));
            return false;
        }
        configured[chan] = true;
    }
    return true;
}

bool AP_DAC_TIx3204::register_read(uint8_t reg, uint16_t &val)
{
    uint16_t v;
    if (!dev->read_registers(reg, (uint8_t*)&v, sizeof(v))) {
        return false;
    }
    val = be16toh(v);
    return true;
}

bool AP_DAC_TIx3204::register_write(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] { reg, uint8_t(val >> 8), uint8_t(val) };
    return dev->transfer(buf, sizeof(buf), nullptr, 0);
}

#endif // AP_DAC_TIX3204_ENABLED
