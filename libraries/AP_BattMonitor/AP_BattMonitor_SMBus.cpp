#include "AP_BattMonitor_SMBus.h"

#define AP_BATTMONITOR_SMBUS_PEC_POLYNOME 0x07 // Polynome for CRC generation

AP_BattMonitor_SMBus::AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                                           AP_BattMonitor::BattMonitor_State &mon_state,
                                           AP_BattMonitor_Params &params,
                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
        : AP_BattMonitor_Backend(mon, mon_state, params),
        _dev(std::move(dev))
{
    _params._serial_number = AP_BATT_SERIAL_NUMBER_DEFAULT;
    _params._pack_capacity = 0;
}

void AP_BattMonitor_SMBus::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus::timer, void));
    }
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
// returns true if the read was successful, or if we already knew the pack capacity
bool AP_BattMonitor_SMBus::read_full_charge_capacity(void)
{
    uint16_t data;

    if (_full_charge_capacity != 0) {
        return true;
    } else if (read_word(BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY, data)) {
        _full_charge_capacity = data;
        return true;
    }
    return false;
}

// reads the remaining capacity
// returns true if the read was succesful, which is only considered to be the
// we know the full charge capacity
bool AP_BattMonitor_SMBus::read_remaining_capacity(void)
{
    int32_t capacity = _params._pack_capacity;

    if (capacity > 0) {
        uint16_t data;
        if (read_word(BATTMONITOR_SMBUS_REMAINING_CAPACITY, data)) {
            _state.consumed_mah = MAX(0, capacity - data);
            return true;
        }
    }

    return false;
}

// reads the temperature word from the battery
// returns true if the read was successful
bool AP_BattMonitor_SMBus::read_temp(void)
{
    uint16_t data;
    if (read_word(BATTMONITOR_SMBUS_TEMP, data)) {
        _state.temperature_time = AP_HAL::millis();
        _state.temperature = ((float)(data - 2731)) * 0.1f;
        return true;
    }

    return false;
}

// reads the serial number if it's not already known
// returns true if the read was successful or the number was already known
bool AP_BattMonitor_SMBus::read_serial_number(void) {
    uint16_t data;

    // don't recheck the serial number if we already have it
    if (_serial_number != -1) {
        return true;
    } else if (read_word(BATTMONITOR_SMBUS_SERIAL, data)) {
        _serial_number = data;
        return true;
    }

    return false;
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
        const uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, 2);
        if (pec != buff[2]) {
            return false;
        }
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
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

