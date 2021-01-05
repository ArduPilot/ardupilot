#include "AP_BattMonitor_INA3221.h"

#include <AP_HAL/utility/sparse-endian.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

#define INA3221_DEBUGGING 1

#if INA3221_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {printf("INA3221: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
// #define debug(fmt, args ...)  do {gcs().send_text(MAV_SEVERITY_INFO, "INA3221: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

AP_BattMonitor_INA3221::AP_BattMonitor_INA3221(
    AP_BattMonitor &mon,
    AP_BattMonitor::BattMonitor_State &mon_state,
    AP_BattMonitor_Params &params,
    AP_BattMonitor::Type type) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
}

bool AP_BattMonitor_INA3221::read_register(uint8_t addr, uint16_t &ret)
{
    if (!_dev->transfer(&addr, 1, (uint8_t*)&ret, 2)) {
        return false;
    }
    ret = be16toh(ret);
    return true;
}

bool AP_BattMonitor_INA3221::write_register(uint8_t addr, uint16_t val)
{
    uint8_t buf[3] { addr, uint8_t(val >> 8), uint8_t(val & 0xFF) };

    return _dev->transfer(buf, sizeof(buf), nullptr, 0);
}

#define REG_CONFIGURATION 0x00
#define REG_MANUFACTURER_ID 0xFE
#define REG_DIE_ID 0xFF
bool AP_BattMonitor_INA3221::init_at_address(uint8_t init_address)
{
    _dev = std::move(hal.i2c_mgr->get_device(_params._i2c_bus, init_address, 100000, true, 20));
    if (!_dev) {
        return false;
    }

    debug("INA3221: probe @0x%02x on bus %u", init_address, _params._i2c_bus.get());

    WITH_SEMAPHORE(_dev->get_semaphore());

    // check manufacturer_id
    uint16_t manufacturer_id;
    if (!read_register(REG_MANUFACTURER_ID, manufacturer_id)) {
        debug("read register (%u (0x%02x)) failed", REG_MANUFACTURER_ID, REG_MANUFACTURER_ID);
        return false;
    }
    if (manufacturer_id != 0x5449) {  // 8.6.1 p24
        debug("Bad manufacturer_id: 0x%02x", manufacturer_id);
        return false;
    }

    uint16_t die_id;
    if (!read_register(REG_DIE_ID, die_id)) {
        debug("Bad die: 0x%02x", manufacturer_id);
        return false;
    }
    if (die_id != 0x3220) {  // 8.6.1 p24
        return false;
    }

    debug("Found INA3221 @0x%02x on bus %u", init_address, _params._i2c_bus.get());

    // reset the device:
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
    } configuration {
            0b111, // continuous operation
            0b111, // 8ms conversion time for shunt voltage
            0b111, // 8ms conversion time for bus voltage
            0b111, // 1024 samples / average
            0b1,  // enable ch1
            0b1,  // enable ch2
            0b1,  // enable ch3
            0b0    // don't reset...
                };

    if (!write_register(REG_CONFIGURATION, configuration.word)) {
        return false;
    }

    _dev->register_periodic_callback(
        100000,
        FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA3221::_timer, void));

    return true;
}

void AP_BattMonitor_INA3221::_timer(void)
{
    uint8_t reg_shunt;
    uint8_t reg_bus;
    switch (_type) {
    case AP_BattMonitor::Type::INA3221_CH1:
        reg_shunt = 1;
        reg_bus = 2;
        break;
    case AP_BattMonitor::Type::INA3221_CH2:
        reg_shunt = 3;
        reg_bus = 4;
        break;
    case AP_BattMonitor::Type::INA3221_CH3:
        reg_shunt = 5;
        reg_bus = 6;
        break;
    default:
        return;
    }
    uint16_t shunt_voltage;
    if (!read_register(reg_shunt, shunt_voltage)) {
        return;
    }
    uint16_t bus_voltage;
    if (!read_register(reg_bus, bus_voltage)) {
        return;
    }

    //transfer readings to front end:
    WITH_SEMAPHORE(_sem);
    _state.voltage = bus_voltage;
    _state.current_amps = shunt_voltage * 0.56f;
    _state.last_time_micros = AP_HAL::micros();
}

void AP_BattMonitor_INA3221::init()
{
    // check to see which address we should start probing at:
    uint8_t probe_address = 64;
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().get_type(i) != _type) {
            // different type...
            continue;
        }
        const AP_BattMonitor_INA3221 *driver = ((AP_BattMonitor_INA3221*)AP::battery().drivers[i]);
        if (driver != nullptr) {
            if (driver->address >= probe_address) {
                probe_address = driver->address + 1;
            }
            continue;
        }
        break;
    }

    // now look for the next one:
    for (; probe_address <= 67; probe_address++) {
        if (!init_at_address(probe_address)) {
            continue;
        }
        break;
    }
}

void AP_BattMonitor_INA3221::read()
{
    static uint32_t last_print;
    if (AP_HAL::millis() - last_print > 5000) {
        last_print = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_INFO, "%u: voltage:%f current:%f\n", (unsigned)_state.last_time_micros, _state.voltage, _state.current_amps);
    }
}
