#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA3221_ENABLED

#include "AP_BattMonitor_INA3221.h"

#include <AP_HAL/utility/sparse-endian.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

#define INA3221_DEBUGGING 0

#if INA3221_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {printf("INA3221: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
// #define debug(fmt, args ...)  do {gcs().send_text(MAV_SEVERITY_INFO, "INA3221: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#ifndef HAL_BATTMON_INA3221_BUS
#define HAL_BATTMON_INA3221_BUS 0
#endif

#ifndef HAL_BATTMON_INA3221_ADDR
#define HAL_BATTMON_INA3221_ADDR 64
#endif

#ifndef HAL_BATTMON_INA3221_SHUNT_OHMS
#define HAL_BATTMON_INA3221_SHUNT_OHMS 0.001
#endif

#define HAL_BATTMON_INA3221_CONV_TIME_140US 0b000
#define HAL_BATTMON_INA3221_CONV_TIME_204US 0b001
#define HAL_BATTMON_INA3221_CONV_TIME_332US 0b010
#define HAL_BATTMON_INA3221_CONV_TIME_588US 0b011
#define HAL_BATTMON_INA3221_CONV_TIME_1100US 0b100
#define HAL_BATTMON_INA3221_CONV_TIME_2116US 0b101
#define HAL_BATTMON_INA3221_CONV_TIME_4156US 0b110
#define HAL_BATTMON_INA3221_CONV_TIME_8244US 0b111

#define HAL_BATTMON_INA3221_AVG_MODE_1 0b000
#define HAL_BATTMON_INA3221_AVG_MODE_4 0b001
#define HAL_BATTMON_INA3221_AVG_MODE_16 0b010
#define HAL_BATTMON_INA3221_AVG_MODE_64 0b011
#define HAL_BATTMON_INA3221_AVG_MODE_128 0b100
#define HAL_BATTMON_INA3221_AVG_MODE_256 0b101
#define HAL_BATTMON_INA3221_AVG_MODE_512 0b110
#define HAL_BATTMON_INA3221_AVG_MODE_1024 0b111

#ifndef HAL_BATTMON_INA3221_SHUNT_CONV_TIME_SEL
#define HAL_BATTMON_INA3221_SHUNT_CONV_TIME_SEL HAL_BATTMON_INA3221_CONV_TIME_8244US
#endif

#ifndef HAL_BATTMON_INA3221_BUS_CONV_TIME_SEL
#define HAL_BATTMON_INA3221_BUS_CONV_TIME_SEL HAL_BATTMON_INA3221_CONV_TIME_8244US
#endif

#ifndef HAL_BATTMON_INA3221_AVG_MODE_SEL
#define HAL_BATTMON_INA3221_AVG_MODE_SEL HAL_BATTMON_INA3221_AVG_MODE_1024
#endif

struct AP_BattMonitor_INA3221::AddressDriver AP_BattMonitor_INA3221::address_driver[HAL_BATTMON_INA3221_MAX_DEVICES];
uint8_t AP_BattMonitor_INA3221::address_driver_count;

const AP_Param::GroupInfo AP_BattMonitor_INA3221::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 22, AP_BattMonitor_INA3221, i2c_bus, HAL_BATTMON_INA3221_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address. If this is zero then probe list of supported addresses
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 23, AP_BattMonitor_INA3221, i2c_address, HAL_BATTMON_INA3221_ADDR),

    // @Param: CHANNEL
    // @DisplayName: INA3221 channel
    // @Description: INA3221 channel to return data for
    // @Range: 1 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("CHANNEL", 24, AP_BattMonitor_INA3221, channel, 1),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

extern const AP_HAL::HAL &hal;

AP_BattMonitor_INA3221::AP_BattMonitor_INA3221(
    AP_BattMonitor &mon,
    AP_BattMonitor::BattMonitor_State &mon_state,
    AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

bool AP_BattMonitor_INA3221::AddressDriver::read_register(uint8_t addr, uint16_t &ret)
{
    if (!dev->transfer(&addr, 1, (uint8_t*)&ret, 2)) {
        return false;
    }
    ret = be16toh(ret);
    return true;
}

bool AP_BattMonitor_INA3221::AddressDriver::write_register(uint8_t addr, uint16_t val)
{
    uint8_t buf[3] { addr, uint8_t(val >> 8), uint8_t(val & 0xFF) };

    return dev->transfer(buf, sizeof(buf), nullptr, 0);
}

#define REG_CONFIGURATION 0x00
#define REG_MANUFACTURER_ID 0xFE
#define REG_DIE_ID 0xFF

bool AP_BattMonitor_INA3221::AddressDriver::write_config(void)
{
    // update device configuration
    union {
        struct PACKED {
            uint16_t mode : 3;
            uint16_t shunt_voltage_conversiontime : 3;
            uint16_t bus_voltage_conversiontime : 3;
            uint16_t averaging_mode : 3;
            uint16_t ch1_enable : 1;
            uint16_t ch2_enable : 1;
            uint16_t ch3_enable : 1;
            uint16_t reset : 1;
        } bits;
        uint16_t word;
    } configuration {{
        0b111, // continuous operation
        HAL_BATTMON_INA3221_SHUNT_CONV_TIME_SEL,
        HAL_BATTMON_INA3221_BUS_CONV_TIME_SEL,
        HAL_BATTMON_INA3221_AVG_MODE_SEL,
        // dynamically enable channels to not waste time converting unused data
        (channel_mask & (1 << 1)) != 0, // enable ch1?
        (channel_mask & (1 << 2)) != 0, // enable ch2?
        (channel_mask & (1 << 3)) != 0, // enable ch3?
        0b0,  // don't reset...
    }};

    if (!write_register(REG_CONFIGURATION, configuration.word)) {
        return false;
    }

    dev_channel_mask = channel_mask; // what's actually in the device now

    return true;
}

void AP_BattMonitor_INA3221::init()
{
    uint8_t dev_address = i2c_address.get();
    uint8_t dev_bus = i2c_bus.get();
    uint8_t dev_channel = channel.get();

    if ((dev_channel < 1) || (dev_channel > 3)) {
        debug("INA3221: nonexistent channel");
        return;
    }

    debug("INA3221: probe ch%d@0x%02x on bus %u", dev_channel, dev_address, dev_bus);

    // check to see if we already have the underlying driver reading the bus:
    for (uint8_t i=0; i<address_driver_count; i++) {
        AddressDriver *d = &address_driver[i];
        if (!d->dev) {
            continue;
        }
        if (d->address != dev_address) {
            continue;
        }
        if (d->bus != dev_bus) {
            continue;
        }
        debug("Reusing INA3221 driver @0x%02x on bus %u", dev_address, dev_bus);
        address_driver_state = NEW_NOTHROW AddressDriver::StateList;
        if (address_driver_state == nullptr) {
            return;
        }
        address_driver_state->channel = dev_channel;
        address_driver_state->next = d->statelist;
        d->statelist = address_driver_state;
        d->channel_mask |= (1 << dev_channel);
        return;
    }

    if (address_driver_count == ARRAY_SIZE(address_driver)) {
        debug("INA3221: out of address drivers");
        return;
    }

    AddressDriver *d = &address_driver[address_driver_count];
    d->dev = hal.i2c_mgr->get_device_ptr(i2c_bus, i2c_address, 100000, true, 20);
    if (!d->dev) {
        return;
    }

    d->bus = i2c_bus;
    d->address = i2c_address;

    WITH_SEMAPHORE(d->dev->get_semaphore());

    // check manufacturer_id
    uint16_t manufacturer_id;
    if (!d->read_register(REG_MANUFACTURER_ID, manufacturer_id)) {
        debug("read register (%u (0x%02x)) failed", REG_MANUFACTURER_ID, REG_MANUFACTURER_ID);
        return;
    }
    if (manufacturer_id != 0x5449) {  // 8.6.1 p24
        debug("Bad manufacturer_id: 0x%02x", manufacturer_id);
        return;
    }

    uint16_t die_id;
    if (!d->read_register(REG_DIE_ID, die_id)) {
        debug("Bad die: 0x%02x", manufacturer_id);
        return;
    }
    if (die_id != 0x3220) {  // 8.6.1 p24
        return;
    }

    d->channel_mask = (1 << dev_channel);
    if (!d->write_config()) {
        return;
    }

    address_driver_state = NEW_NOTHROW AddressDriver::StateList;
    if (address_driver_state == nullptr) {
        return;
    }
    address_driver_state->channel = dev_channel;
    address_driver_state->next = d->statelist;
    d->statelist = address_driver_state;

    debug("Found INA3221 ch%d@0x%02x on bus %u", dev_channel, dev_address, dev_bus);

    address_driver_count++;

    d->register_timer();
}

void AP_BattMonitor_INA3221::AddressDriver::register_timer(void)
{
    dev->register_periodic_callback(
        100000,
        FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA3221::AddressDriver::timer, void));
}

void AP_BattMonitor_INA3221::AddressDriver::timer(void)
{
    bool healthy = true;

    if (channel_mask != dev_channel_mask) {
        if (write_config()) { // update enabled channels
            return; // data is now out of date, read it next time
        }
        // continue on to reading if update failed so health gets cleared
        healthy = false;
    }

    for (uint8_t i=1; i<=3; i++) {
        if ((channel_mask & (1U<<i)) == 0) {
            continue;
        }
        const uint8_t channel_offset = (i-1)*2;
        const uint8_t reg_shunt = 1 + channel_offset;
        const uint8_t reg_bus = 2 + channel_offset;

        uint16_t shunt_val;
        if (!read_register(reg_shunt, shunt_val)) {
            healthy = false;
            shunt_val = 0;
        }
        uint16_t bus_val;
        if (!read_register(reg_bus, bus_val)) {
            healthy = false;
            bus_val = 0;
        }

        // 2s complement number with 3 lowest bits not used, 1 count is 8mV
        const float bus_voltage = ((int16_t)bus_val >> 3)*8e-3;
        // 2s complement number with 3 lowest bits not used, 1 count is 40uV
        const float shunt_voltage = ((int16_t)shunt_val >> 3)*40e-6;
        const float shunt_resistance = HAL_BATTMON_INA3221_SHUNT_OHMS;
        const float shunt_current = shunt_voltage/shunt_resistance; // I = V/R

        // transfer readings to state
        for (auto *state = statelist; state != nullptr; state = state->next) {
            if (state->channel != i) {
                continue;
            }
            WITH_SEMAPHORE(state->sem);

            // calculate time since last data read
            const uint32_t tnow = AP_HAL::micros();
            const uint32_t dt_us = tnow - state->last_time_micros;

            state->healthy = healthy;
            state->voltage = bus_voltage;
            state->current_amps = shunt_current;

            // update current drawn since last reading for read to accumulate
            if (state->last_time_micros != 0 && dt_us < 2000000) {
                const float mah = calculate_mah(state->current_amps, dt_us);
                state->delta_mah += mah;
                state->delta_wh  += 0.001 * mah * state->voltage;
            }

            state->last_time_micros = tnow;
        }
    }
}

void AP_BattMonitor_INA3221::read()
{
    if (address_driver_state == nullptr) {
        return;
    }

    WITH_SEMAPHORE(address_driver_state->sem);
    // copy state data to front-end under semaphore
    _state.healthy = address_driver_state->healthy;
    _state.voltage = address_driver_state->voltage;
    _state.current_amps = address_driver_state->current_amps;
    _state.consumed_mah += address_driver_state->delta_mah;
    _state.consumed_wh += address_driver_state->delta_wh;
    _state.last_time_micros = address_driver_state->last_time_micros;

    address_driver_state->delta_mah = 0;
    address_driver_state->delta_wh = 0;
}

#endif  // AP_BATTERY_INA3221_ENABLED
