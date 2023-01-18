#include "AP_BattMonitor_INA260.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_BATTMON_INA260_ENABLED

extern const AP_HAL::HAL& hal;

/*INA260 registers*/
#define REG_CONFIG       0x00
#define REG_CURRENT      0x01
#define REG_BUS_VOLTAGE  0x02
#define REG_POWER        0x03
#define REG_MASK         0x06
#define REG_ALERT        0x07
#define REG_MANUF_ID     0xFE
#define REG_DIE_ID       0xFF

#define REG_CONFIG_RESET      0x8000
#define REG_CONFIG_DEFAULT    0x6127
#define REG_MANUF_ID_NUMBER   0x5449
#define REG_DEVICE_ID_NUMBER  0x227

/*I2C bus and address*/
#ifndef HAL_BATTMON_INA260_BUS
#define HAL_BATTMON_INA260_BUS  0                          //try changing bus value to 1,2 or 3 if 0 doesn't work
#endif
#ifndef HAL_BATTMON_INA260_ADDR
#define HAL_BATTMON_INA260_ADDR 64                         //default address 0x40 = 64 when A1 and A0 are GND
#endif

const AP_Param::GroupInfo AP_BattMonitor_INA260::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 25, AP_BattMonitor_INA260, i2c_bus, HAL_BATTMON_INA260_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 26, AP_BattMonitor_INA260, i2c_address, HAL_BATTMON_INA260_ADDR),

    AP_GROUPEND
};

AP_BattMonitor_INA260::AP_BattMonitor_INA260(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_INA260::init(void)
{
    dev = hal.i2c_mgr->get_device(i2c_bus, i2c_address, 100000, false, 20);        //100kHz freq. and 20ms timeout
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "device fail");
        return;
    }
    else{
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "device pass");
    }
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA260::timer, void));
}

void AP_BattMonitor_INA260::configure(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    int16_t config = 0;
    //check configuration register
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) ||
        !write_word(REG_CONFIG, REG_CONFIG_DEFAULT) ||
        !read_word(REG_CONFIG, config) ||
        config != REG_CONFIG_DEFAULT) {
        return;
    }

    //check manufacturer id and device id details
    int16_t manuf_id = 0;
    int16_t die_id = 0;
    if(!read_word(REG_MANUF_ID,manuf_id) || !read_word(REG_DIE_ID,die_id)){
        return;
    }
    else{
        if((manuf_id!=REG_MANUF_ID_NUMBER) || ((die_id>>4)!=REG_DEVICE_ID_NUMBER)){
            return;
        }
    }

    if(!write_word(REG_CONFIG, REG_CONFIG_RESET))  //reset
    {
        return;
    }
    
    configured = true;
    
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA260::read(void)
{

    WITH_SEMAPHORE(accumulate.sem);
    _state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.voltage/ accumulate.count;
    _state.current_amps = accumulate.current/ accumulate.count;
    accumulate.voltage = 0;
    accumulate.current = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;
    
    // update total current drawn since startup
    update_consumed(_state, dt_us);

    _state.last_time_micros = tnow;
}

/*
 read word from register
 returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA260::read_word(const uint8_t reg, int16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = int16_t(be16toh(uint16_t(data)));

    return true;
}

/*
  write word to a register, byte swapped
  returns true if write was successful, false if failed
*/
bool AP_BattMonitor_INA260::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

void AP_BattMonitor_INA260::timer(void)
{
    // allow for power-on after boot
    if (!configured) {
        uint32_t now = AP_HAL::millis();
        if (now - last_configure_ms > 200) {
            // try contacting the device at 5Hz
            last_configure_ms = now;
            configure();
        }
        if (!configured) {
            // waiting for the device to respond
            return;
        }
    }
    else{
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "configured");               //only for debugging
    }

    int16_t bus_voltage, current;

    if (!read_word(REG_BUS_VOLTAGE, bus_voltage) ||
        !read_word(REG_CURRENT, current)) {
        failed_reads++;
        if (failed_reads > 10) {
            // device has disconnected, we need to reconfigure it
            configured = false;
            failed_reads = 0;
            return;
        }
        
    }
    else{
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "read success");             //only for debugging
    }

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.voltage += bus_voltage*voltage_LSB;
    accumulate.current += current*current_LSB;
    accumulate.count++;
}

#endif // HAL_BATTMON_INA260_ENABLED
