#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Maxell.h"
#include <utility>

extern const AP_HAL::HAL& hal;

#include <AP_HAL/AP_HAL.h>


#define BATTMONITOR_SMBUS_MAXELL_VOLTAGE    0x09    // voltage register
#define BATTMONITOR_SMBUS_MAXELL_CURRENT    0x0a    // current register
#define BATTMONITOR_SMBUS_MAXELL_SPECIFICATION_INFO    0x1a    // specification info
#define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_NAME  0x20    // manufacturer name

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    0x20   // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.

#define SMBUS_PEC_POLYNOME  0x07 // Polynome for CRC generation

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_MAXELL_TEMP      0x08    // temperature register
 * #define BATTMONITOR_SMBUS_MAXELL_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_MAXELL_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_DATE      0x1b    // manufacturer date
 * #define BATTMONITOR_SMBUS_MAXELL_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE6         0x3a    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE5         0x3b    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE4         0x3c    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE3         0x3d    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE2         0x3e    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE1         0x3f    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_STATUS         0x50    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Maxell::AP_BattMonitor_SMBus_Maxell(AP_BattMonitor &mon, uint8_t instance,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, instance, mon_state)
    , _dev(std::move(dev))
{
    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_Maxell::timer, void));
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_SMBus_Maxell::read()
{
    // nothing to do - all done in timer()
}

void AP_BattMonitor_SMBus_Maxell::timer()
{
	// check if PEC is supported
    if (!check_pec_support()) {
        return;
    }

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_MAXELL_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_MAXELL_CURRENT, data)) {
        _state.current_amps = -(float)((int16_t)data) / 1000.0f;
        _state.last_time_micros = tnow;
    }
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus_Maxell::read_word(uint8_t reg, uint16_t& data) const
{
    // buffer to hold results (1 extra byte returned holding PEC)
    const uint8_t read_size = 2 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];    // buffer to hold results

    // read three bytes and place in last three bytes of buffer
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

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_Maxell::read_block(uint8_t reg, uint8_t* data, bool append_zero) const
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
    }

    // return success
    return bufflen;
}

// check if PEC supported with the version value in SpecificationInfo() function
// returns true once PEC is confirmed as working or not working
bool AP_BattMonitor_SMBus_Maxell::check_pec_support()
{
    // exit immediately if we have already confirmed pec support
    if (_pec_confirmed) {
        return true;
    }

    // specification info
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_MAXELL_SPECIFICATION_INFO, data)) {
        return false;
    }

    // extract version
    uint8_t version = (data & 0xF0) >> 4;

    // version less than 0011b (i.e. 3) do not support PEC
    if (version < 3) {
        _pec_supported = false;
        _pec_confirmed = true;
        return true;
    }

    // check manufacturer name
    uint8_t buff[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER + 1];
    if (read_block(BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_NAME, buff, true)) {
        // Hitachi maxell batteries do not support PEC
        if (strcmp((char*)buff, "Hitachi maxell") == 0) {
            _pec_supported = false;
            _pec_confirmed = true;
            return true;
        }
    }

    // assume all other batteries support PEC
	_pec_supported = true;
	_pec_confirmed = true;
	return true;
}

/// get_PEC - calculate packet error correction code of buffer
uint8_t AP_BattMonitor_SMBus_Maxell::get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const
{
    // exit immediately if no data
    if (len == 0) {
        return 0;
    }

    // prepare temp buffer for calcing crc
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
                crc ^= SMBUS_PEC_POLYNOME;
            }
        }
    }

    // return result
    return crc;
}
