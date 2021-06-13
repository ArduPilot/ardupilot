#include "AP_BattMonitor_SMBus.h"
#include <stdio.h>
#include <AP_InternalError/AP_InternalError.h>

#define AP_BATTMONITOR_SMBUS_PEC_POLYNOME 0x07 // Polynome for CRC generation

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_SMBus::var_info[] = {

    // Param indexes must be between 10 and 19 to avoid conflict with other battery monitor param tables loaded by pointer

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

    // Param indexes must be between 10 and 19 to avoid conflict with other battery monitor param tables loaded by pointer

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
    _params._serial_number = AP_BATT_SERIAL_NUMBER_DEFAULT;
    _params._pack_capacity = 0;
}

void AP_BattMonitor_SMBus::init(void)
{
    _dev = hal.i2c_mgr->get_device(_bus, _address, 100000, true, 20);
    
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

#if HAL_SMART_BATTERY_INFO_ENABLED
// returns true if product name can be filled in from either manufacturer name and device name
bool AP_BattMonitor_SMBus::get_product_name(char *product_name, uint8_t buflen) const
{
    if ((0 < _manufacturer_name_len) || (0 < _device_name_len)) {
        snprintf(product_name, buflen, "%s_%s", _manufacturer_name, _device_name);
        return true;
    }

    // ensure null termination when data is unavailable
    if (buflen > 0 ) {
        product_name[0] = '\0';
    }
    return false;
}

// return true if design_capacity in mAh (capacity when newly manufactured) can be provided and fills it in
bool AP_BattMonitor_SMBus::get_design_capacity_mah(float &design_capacity) const
{
    if (_design_capacity_mah == 0) {
        return false;
    }

    design_capacity = _design_capacity_mah;
    return true;
}

// returns true if the full charge capacity in mAh (accounting for battery degradation) can be provided and fills it in
bool AP_BattMonitor_SMBus::get_full_charge_capacity_mah(float &full_charge_capacity) const
{
    if (_full_charge_capacity_mah == 0) {
        return false;
    }

    full_charge_capacity = _full_charge_capacity_mah;
    return true;
}

// returns true if the design voltage in volts (maximum charging voltage) can be provided and fill it in
bool AP_BattMonitor_SMBus::get_design_voltage(float &design_voltage) const
{
    if (_design_voltage_mv == 0) {
        return false;
    }

    design_voltage = _design_voltage_mv * 0.001;
    return true;
}

// returns true if the manufacture date can be provided and fills it in
bool AP_BattMonitor_SMBus::get_manufacture_date(char *manufacture_date, uint8_t buflen) const
{
    if (!_has_manufacture_date) {
        // ensure null termination when data is unavailable
        if (buflen > 0 ) {
            manufacture_date[0] = '\0';
        }
        return false;
    }

    snprintf(manufacture_date, buflen, "%u/%u/%u", _manufacture_date[0], _manufacture_date[1], _manufacture_date[2] + 1980);
    return true;
}
#endif

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_SMBus::read(void)
{
    // nothing to be done here for actually interacting with the battery
    // however we can use this to set any parameters that need to be set

    if (_serial_number != _params._serial_number) {
        _params._serial_number.set_and_notify(_serial_number);
    }

    if (_full_charge_capacity_mah != _params._pack_capacity) {
        _params._pack_capacity.set_and_notify(_full_charge_capacity_mah);
    }
}

// reads the pack full charge capacity
// returns if we already knew the pack capacity
void AP_BattMonitor_SMBus::read_full_charge_capacity(void)
{
    if (_full_charge_capacity_mah != 0) {
        return;
    }

    if (read_word(BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY, _full_charge_capacity_mah)) {
        _full_charge_capacity_mah *= get_capacity_scaler();
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

#if HAL_SMART_BATTERY_INFO_ENABLED
// reads the pack design voltage (maximum charging voltage)
// returns if we already knew the pack design voltage
void AP_BattMonitor_SMBus::read_design_voltage(void)
{
    if (_design_voltage_mv != 0) {
        return;
    }
    read_word(BATTMONITOR_SMBUS_DESIGN_VOLTAGE, _design_voltage_mv);
}

// reads the design capacity (capacity when newly manufactured)
// returns if we already knew the design capacity
void AP_BattMonitor_SMBus::read_design_capacity(void)
{
    if (_design_capacity_mah != 0) {
        return;
    }
    read_word(BATTMONITOR_SMBUS_DESIGN_CAPACITY, _design_capacity_mah);
    _design_capacity_mah *= get_capacity_scaler();
}

// returns number of characters written to name_out if the given register read of a character array was successful else returns 0
uint8_t AP_BattMonitor_SMBus::read_name(AP_BattMonitor_SMBus::BATTMONITOR_SMBUS reg_name, char * name_out, uint8_t buflen)
{
    if (name_out == nullptr || buflen == 0) {
        return 0;
    }
   
    // Check the ENUM to be sure it is a character array Product_Name or Device_Name
    if (reg_name == AP_BattMonitor_SMBus::BATTMONITOR_SMBUS::BATTMONITOR_SMBUS_MANUFACTURE_NAME ||
        reg_name == AP_BattMonitor_SMBus::BATTMONITOR_SMBUS::BATTMONITOR_SMBUS_DEVICE_NAME) {

        uint8_t name[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER+1];
        const uint8_t name_len = read_block(reg_name, name, true);

        if (name_len) {
            const uint8_t len = MIN(name_len, buflen);
            strncpy_noterm(name_out, (char*)name, len);
            return len;
        }
    } else {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    return 0;
}

// return the manufacturer name and return if already known
void AP_BattMonitor_SMBus::read_manufacturer_name(void)
{
    if (_manufacturer_name_len > 0) {
        return;
    }
    _manufacturer_name_len = read_name(BATTMONITOR_SMBUS_MANUFACTURE_NAME, _manufacturer_name, ARRAY_SIZE(_manufacturer_name));
}

// read the decvice name and return if already known
void AP_BattMonitor_SMBus::read_device_name(void)
{
    if (_device_name_len > 0) {
        return;
    }
    _device_name_len = read_name(BATTMONITOR_SMBUS_DEVICE_NAME, _device_name, ARRAY_SIZE(_device_name));
}

// read the manufacture date and return if already known
void AP_BattMonitor_SMBus::read_manufacture_date(void) 
{
    if (_has_manufacture_date) {
        return;
    }

    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_MANUFACTURE_DATE, data)) {
        _has_manufacture_date = false;
        return;
    }
    // Note this doesn't check for bad inputs ie Day = 0 or Month = 15
    _manufacture_date[0] = data % 32;            // day
    _manufacture_date[1] = (data >> 5) % 16 ;    // month
    _manufacture_date[2] = (data >> 9);          // year since 1980

    _has_manufacture_date = true;
}

#endif // HAL_SMART_BATTERY_INFO_ENABLED

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

// read_block - returns number of characters read if successful, zero if unsuccessful, if append_zero true
// appends '\0' to the end of the array
uint8_t AP_BattMonitor_SMBus::read_block(uint8_t reg, uint8_t* data, bool append_zero) const
{
    // get length
    uint8_t bufflen;
    // read byte (first byte indicates the number of bytes in the block)
    if (!_dev->read_registers(reg, &bufflen, 1)) {
        return 0;
    }

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > SMBUS_READ_BLOCK_MAXIMUM_TRANSFER) {
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
        uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
        if (pec != buff[bufflen+1]) {
            return 0;
        }
    }

    // copy data (excluding PEC)
    memcpy(data, &buff[1], bufflen);

    // optionally add zero to end
    if (append_zero) {
        data[bufflen] = '\0';
        ++bufflen;
    }

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
