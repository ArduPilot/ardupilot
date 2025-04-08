#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_ENABLED

#include "AP_BattMonitor_SMBus.h"

#define AP_BATTMONITOR_SMBUS_PEC_POLYNOME 0x07 // Polynome for CRC generation

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_SMBus::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 10, AP_BattMonitor_SMBus, _bus, 0),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 11, AP_BattMonitor_SMBus, _address, AP_BATTMONITOR_SMBUS_I2C_ADDR),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_BattMonitor_SMBus::AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                                           AP_BattMonitor::BattMonitor_State &mon_state,
                                           AP_BattMonitor_Params &params,
                                           uint8_t i2c_bus)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    _bus.set_default(i2c_bus);
    _params._serial_number.set(AP_BATT_SERIAL_NUMBER_DEFAULT);
    _params._pack_capacity.set(0);
}

void AP_BattMonitor_SMBus::init(void)
{
    _dev = hal.i2c_mgr->get_device_ptr(_bus, _address, 100000, true, 20);
    
    if (_dev) {
        timer_handle = _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus::timer, void));
    }
}

// return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_SMBus::get_cycle_count(uint16_t &cycles) const
{
    if (!_has_cycle_count) {
        return false;
    }
    cycles = _cycle_count;
    return true;
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_SMBus::read(void)
{
    // nothing to be done here for actually interacting with the battery
    // however we can use this to set any parameters that need to be set

    if (_serial_number != _params._serial_number) {
        _params._serial_number.set_and_notify(_serial_number);
    }

    if (_full_charge_capacity != _params._pack_capacity) {
        _params._pack_capacity.set_and_notify(_full_charge_capacity);
    }
}

// reads the pack full charge capacity
// returns if we already knew the pack capacity
void AP_BattMonitor_SMBus::read_full_charge_capacity(void)
{
    if (_full_charge_capacity != 0) {
        return;
    }

    if (read_word(BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY, _full_charge_capacity)) {
        _full_charge_capacity *= get_capacity_scaler();
    }
}

// reads the remaining capacity
// which will only be read if we know the full charge capacity (accounting for battery degradation)
void AP_BattMonitor_SMBus::read_remaining_capacity(void)
{
    int32_t capacity = _params._pack_capacity;

    if (capacity <= 0) {
        return;
    }

    uint16_t data;
    if (read_word(BATTMONITOR_SMBUS_REMAINING_CAPACITY, data)) {
        _state.consumed_mah = MAX(0, capacity - (data * get_capacity_scaler()));
    }
}

// reads the temperature word from the battery
void AP_BattMonitor_SMBus::read_temp(void)
{
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_TEMP, data)) {
        _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
        return;
    }
    _has_temperature = true;

    _state.temperature_time = AP_HAL::millis();
    _state.temperature = KELVIN_TO_C(0.1f * data);
}

// reads the serial number if it's not already known
// returns if the serial number was already known
void AP_BattMonitor_SMBus::read_serial_number(void)
{
    // don't recheck the serial number if we already have it
    if (_serial_number != -1) {
        return;
    }

    uint16_t data;
    if (read_word(BATTMONITOR_SMBUS_SERIAL, data)) {
        _serial_number = data;
    }
}

// reads the battery's cycle count
void AP_BattMonitor_SMBus::read_cycle_count()
{
    // only read cycle count once
    if (_has_cycle_count) {
        return;
    }
    _has_cycle_count = read_word(BATTMONITOR_SMBUS_CYCLE_COUNT, _cycle_count);
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus::read_word(uint8_t reg, uint16_t& data) const
{
    // buffer to hold results (1 extra byte returned holding PEC)
    const uint8_t read_size = 2 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];    // buffer to hold results

    // read the appropriate register from the device
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // check PEC
    if (_pec_supported) {
        const uint8_t pec = get_PEC(_address, reg, true, buff, 2);
        if (pec != buff[2]) {
            return false;
        }
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus::read_block(uint8_t reg, uint8_t* data, uint8_t len) const
{
    // get length
    uint8_t bufflen;
    // read byte (first byte indicates the number of bytes in the block)
    if (!_dev->read_registers(reg, &bufflen, 1)) {
        return 0;
    }

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > len) {
        return 0;
    }

    // buffer to hold results (2 extra byte returned holding length and PEC)
    const uint8_t read_size = bufflen + 1 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];

    // read bytes
    if (!_dev->read_registers(reg, buff, read_size)) {
        return 0;
    }

    // check PEC
    if (_pec_supported) {
        const uint8_t pec = get_PEC(_address, reg, true, buff, bufflen+1);
        if (pec != buff[bufflen+1]) {
            return 0;
        }
    }

    // copy data (excluding length & PEC)
    memcpy(data, &buff[1], bufflen);

    // return success
    return bufflen;
}

/// get_PEC - calculate packet error correction code of buffer
uint8_t AP_BattMonitor_SMBus::get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const
{
    // exit immediately if no data
    if (len == 0) {
        return 0;
    }

    // prepare temp buffer for calculating crc
    uint8_t tmp_buff[len+3];
    tmp_buff[0] = i2c_addr << 1;
    tmp_buff[1] = cmd;
    tmp_buff[2] = tmp_buff[0] | (uint8_t)reading;
    memcpy(&tmp_buff[3],buff,len);

    // initialise crc to zero
    uint8_t crc = 0;
    uint8_t shift_reg = 0;
    bool do_invert;

    // for each byte in the stream
    for (uint8_t i=0; i<sizeof(tmp_buff); i++) {
        // load next data byte into the shift register
        shift_reg = tmp_buff[i];
        // for each bit in the current byte
        for (uint8_t j=0; j<8; j++) {
            do_invert = (crc ^ shift_reg) & 0x80;
            crc <<= 1;
            shift_reg <<= 1;
            if(do_invert) {
                crc ^= AP_BATTMONITOR_SMBUS_PEC_POLYNOME;
            }
        }
    }

    // return result
    return crc;
}

#endif  // AP_BATTERY_SMBUS_ENABLED
