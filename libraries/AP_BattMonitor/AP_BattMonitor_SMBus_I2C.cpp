#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_I2C.h"
#include <utility>

extern const AP_HAL::HAL& hal;

#include <AP_HAL/AP_HAL.h>

#define BATTMONITOR_SMBUS_TEMP      0x08    // temperature register
#define BATTMONITOR_SMBUS_VOLTAGE   0x09    // voltage register
#define BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY  0x10    // full capacity register
#define BATTMONITOR_SMBUS_BATTERY_STATUS        0x16    // battery status register including alarms
#define BATTMONITOR_SMBUS_DESIGN_CAPACITY       0x18    // design capacity register
#define BATTMONITOR_SMBUS_DESIGN_VOLTAGE        0x19    // design voltage register
#define BATTMONITOR_SMBUS_SERIALNUM             0x1c    // serial number register
#define BATTMONITOR_SMBUS_MANUFACTURE_NAME      0x20    // manufacturer name
#define BATTMONITOR_SMBUS_DEVICE_NAME           0x21    // device name
#define BATTMONITOR_SMBUS_DEVICE_CHEMISTRY      0x22    // device chemistry
#define BATTMONITOR_SMBUS_MANUFACTURE_INFO      0x25    // manufacturer info including cell voltage
#define BATTMONITOR_SMBUS_CELL_VOLTAGE          0x28    // cell voltage register
#define BATTMONITOR_SMBUS_CURRENT               0x2a    // current register

// Constructor
AP_BattMonitor_SMBus_I2C::AP_BattMonitor_SMBus_I2C(AP_BattMonitor &mon, uint8_t instance,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, instance, mon_state)
    , _dev(std::move(dev))
{
    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_I2C::timer, void));
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_SMBus_I2C::read()
{
    // nothing to do - all done in timer()
}

void AP_BattMonitor_SMBus_I2C::timer()
{
    uint16_t data;
    uint8_t buff[4];
    uint32_t tnow = AP_HAL::micros();

    // read voltage
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read current
    if (read_block(BATTMONITOR_SMBUS_CURRENT, buff, 4, false) == 4) {
        _state.current_amps = (float)((int32_t)((uint32_t)buff[3]<<24 | (uint32_t)buff[2]<<16 | (uint32_t)buff[1]<<8 | (uint32_t)buff[0])) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus_I2C::read_word(uint8_t reg, uint16_t& data) const
{
    uint8_t buff[3];    // buffer to hold results

    // read three bytes and place in last three bytes of buffer
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // check PEC
    uint8_t pec = get_PEC(BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, 2);
    if (pec != buff[2]) {
        return false;
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_I2C::read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const
{
    uint8_t buff[max_len+2];    // buffer to hold results (2 extra byte returned holding length and PEC)

    // read bytes
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return 0;
    }

    // get length
    uint8_t bufflen = buff[0];

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > max_len) {
        return 0;
    }

    // check PEC
    uint8_t pec = get_PEC(BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
    if (pec != buff[bufflen+1]) {
        return 0;
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

#define SMBUS_PEC_POLYNOME  0x07 // Polynome for CRC generation

/// get_PEC - calculate packet error correction code of buffer
uint8_t AP_BattMonitor_SMBus_I2C::get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const
{
    // exit immediately if no data
    if (len <= 0) {
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

