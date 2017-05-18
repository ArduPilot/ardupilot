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
  approriate. The bad value will be corrected
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

/*
  set value of one checked register
 */
void AP_HAL::Device::set_checked_register(uint8_t reg, uint8_t val)
{
    if (_checked.regs == nullptr) {
        return;
    }
    struct checkreg *regs = _checked.regs;
    for (uint8_t i=0; i<_checked.n_set; i++) {
        if (regs[i].regnum == reg) {
            regs[i].value = val;
            return;
        }
    }
    if (_checked.n_set == _checked.n_allocated) {
        printf("Not enough checked registers for reg 0x%02x on device 0x%x\n",
               (unsigned)reg, (unsigned)get_bus_id());
        return;
    }
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
    uint8_t v;
    if (!read_registers(reg.regnum, &v, 1) || v != reg.value) {
        // a register has changed value unexpectedly. Try changing it back
        // and re-check it next time
#if 0
        printf("Device 0x%x fixing 0x%02x 0x%02x -> 0x%02x\n",
               (unsigned)get_bus_id(),
               (unsigned)reg.regnum, (unsigned)v, (unsigned)reg.value);
#endif
        write_register(reg.regnum, reg.value);
        return false;
    }
    _checked.next = (_checked.next+1) % _checked.n_set;
    return true;
}
