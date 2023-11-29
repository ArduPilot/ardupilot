/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Device.h"

#include <stdio.h>

/*
  using checked registers allows a device check that a set of critical
  register values don't change at runtime. This is useful on key
  sensors (such as IMUs) which may experience brownouts or other
  issues in flight

  To use register checking call setup_checked_registers() once to
  allocate the space for the checked register values. The set the
  checked flag on any write_register() calls that you want protected.

  Periodically (say at 50Hz) you should then call
  check_next_register(). If that returns false then the sensor has had
  a corrupted register value. Marking the sensor as unhealthy is
  appropiate. The bad value will be corrected
 */

/*
  setup nregs checked registers
 */
bool AP_HAL::Device::setup_checked_registers(uint8_t nregs, uint8_t frequency)
{
    if (_checked.regs != nullptr) {
        delete[] _checked.regs;
        _checked.n_allocated = 0;
        _checked.n_set = 0;
        _checked.next = 0;
    }
    _checked.regs = new struct checkreg[nregs];
    if (_checked.regs == nullptr) {
        return false;
    }
    _checked.n_allocated = nregs;
    _checked.frequency = frequency;
    _checked.counter = 0;
    return true;
}

void AP_HAL::Device::set_device_type(uint8_t devtype) {
    _bus_id.devid_s.devtype = devtype;
}


bool AP_HAL::Device::read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    first_reg |= _read_flag;
    return transfer_bank(bank, &first_reg, 1, recv, recv_len);
}

bool AP_HAL::Device::write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg, val };
    if (checked) {
        set_checked_register(bank, reg, val);
    }
    return transfer_bank(bank, buf, sizeof(buf), nullptr, 0);
}

/*
  set value of one checked register
 */
void AP_HAL::Device::set_checked_register(uint8_t reg, uint8_t val)
{
    set_checked_register(0, reg, val);
}

void AP_HAL::Device::set_checked_register(uint8_t bank, uint8_t reg, uint8_t val)
{
    if (_checked.regs == nullptr) {
        return;
    }
    struct checkreg *regs = _checked.regs;
    for (uint8_t i=0; i<_checked.n_set; i++) {
        if (regs[i].regnum == reg && regs[i].bank == bank) {
            regs[i].value = val;
            return;
        }
    }
    if (_checked.n_set == _checked.n_allocated) {
        printf("Not enough checked registers for reg 0x%02x on device 0x%x\n",
               (unsigned)reg, (unsigned)get_bus_id());
        return;
    }
    regs[_checked.n_set].bank = bank;
    regs[_checked.n_set].regnum = reg;
    regs[_checked.n_set].value = val;
    _checked.n_set++;
}

/*
  check one register value
 */
bool AP_HAL::Device::check_next_register(void)
{
    if (_checked.n_set == 0) {
        return true;
    }
    if (++_checked.counter < _checked.frequency) {
        return true;
    }
    _checked.counter = 0;

    struct checkreg &reg = _checked.regs[_checked.next];
    uint8_t v, v2;

    if (_bank_select) {
        if (!_bank_select(reg.bank)) {
            // Cannot set bank
#if 0
            printf("Device 0x%x set bank 0x%02x\n",
                   (unsigned)get_bus_id(),
                   (unsigned)reg.bank);
#endif
            _checked.last_reg_fail = reg;
            return false;
        }
    }

    if ((!read_registers(reg.regnum, &v, 1) || v != reg.value) &&
        (!read_registers(reg.regnum, &v2, 1) || v2 != reg.value)) {
        // a register has changed value unexpectedly. Try changing it back
        // and re-check it next time
#if 0
        printf("Device 0x%x fixing 0x%02x 0x%02x -> 0x%02x\n",
               (unsigned)get_bus_id(),
               (unsigned)reg.regnum, (unsigned)v, (unsigned)reg.value);
#endif
        write_register(reg.regnum, reg.value);
        _checked.last_reg_fail = reg;
        _checked.last_reg_fail.value = v;
        return false;
    }
    _checked.next = (_checked.next+1) % _checked.n_set;
    return true;
}

/*
  check one register value, returning information on the failure
 */
bool AP_HAL::Device::check_next_register(struct checkreg &fail)
{
    if (check_next_register()) {
        return true;
    }
    fail = _checked.last_reg_fail;
    return false;
}


bool AP_HAL::Device::write_register(uint8_t reg, uint8_t val, bool checked)
{
    uint8_t buf[2] = { reg, val };
    if (checked) {
        set_checked_register(reg, val);
    }
    bool result = transfer(buf, sizeof(buf), nullptr, 0);
    if (_register_rw_callback && result) {
        _register_rw_callback(reg, &val, 1, true);
    }
    return result;
}


bool AP_HAL::Device::read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    uint8_t read_reg = first_reg;
    first_reg |= _read_flag;
    bool result = transfer(&first_reg, 1, recv, recv_len);
    if (_register_rw_callback != nullptr && result) {
        _register_rw_callback(read_reg, recv, recv_len, false);
    }
    return result;
}

bool AP_HAL::Device::transfer_bank(uint8_t bank, const uint8_t *send, uint32_t send_len,
                        uint8_t *recv, uint32_t recv_len)
{
    if (_bank_select) {
        if (!_bank_select(bank)) {
            return false;
        }
    }
    return transfer(send, send_len, recv, recv_len);
}


/**
 * Some devices connected on the I2C or SPI bus require a bit to be set on
 * the register address in order to perform a read operation. This sets a
 * flag to be used by #read_registers(). The flag's default value is zero.
 */
void AP_HAL::Device::set_read_flag(uint8_t flag)
{
    _read_flag = flag;
}


/**
 * make a bus id given bus type, bus number, bus address and
 * device type This is for use by devices that do not use one of
 * the standard HAL Device types, such as UAVCAN devices
 */
uint32_t AP_HAL::Device::make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype) {
    union DeviceId d {};
    d.devid_s.bus_type = bus_type;
    d.devid_s.bus = bus;
    d.devid_s.address = address;
    d.devid_s.devtype = devtype;
    return d.devid;
}

/**
 * return a new bus ID for the same bus connection but a new device type.
 * This is used for auxiliary bus connections
 */
uint32_t AP_HAL::Device::change_bus_id(uint32_t old_id, uint8_t devtype) {
    union DeviceId d;
    d.devid = old_id;
    d.devid_s.devtype = devtype;
    return d.devid;
}

/**
 * return bus ID with a new devtype
 */
uint32_t AP_HAL::Device::get_bus_id_devtype(uint8_t devtype) const {
    return change_bus_id(get_bus_id(), devtype);
}

/**
 * get bus type
 */
enum AP_HAL::Device::BusType AP_HAL::Device::devid_get_bus_type(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.bus_type;
}

uint8_t AP_HAL::Device::devid_get_bus(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.bus;
}

uint8_t AP_HAL::Device::devid_get_address(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.address;
}

uint8_t AP_HAL::Device::devid_get_devtype(uint32_t dev_id) {
    union DeviceId d;
    d.devid = dev_id;
    return d.devid_s.devtype;
}
