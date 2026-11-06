/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulator for ERDI TECH serial rangefinders (4-byte "5C" stream)

  VB series (e.g. LRF22VB, millimetres):
./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:erdi_vb --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 49
param set RNGFND1_MAX 19.9
reboot

  H series (e.g. LRF50H, centimetres):
./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:erdi_h --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 50
param set RNGFND1_MAX 50
reboot

graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance

arm throttle
rc 3 1600
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_RF_ERDI_ENABLED

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_ERDI : public SerialRangeFinder {
public:

    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;

    uint16_t reading_interval_ms() const override { return 20; }

protected:

    // raw distance value for an altitude, including the module's
    // out-of-range value
    virtual uint16_t raw_for_alt(float alt_m) const = 0;
};

// LRF22VB: millimetres, reports 20000 when out of range
class RF_ERDI_VB : public RF_ERDI {
public:
    static SerialRangeFinder *create() { return NEW_NOTHROW RF_ERDI_VB(); }
protected:
    uint16_t raw_for_alt(float alt_m) const override {
        const float mm = alt_m * 1000;
        return (mm >= 20000 || mm < 0) ? 20000 : uint16_t(mm);
    }
};

// LRF50H: centimetres, reports 65535 when out of range
class RF_ERDI_H : public RF_ERDI {
public:
    static SerialRangeFinder *create() { return NEW_NOTHROW RF_ERDI_H(); }
protected:
    uint16_t raw_for_alt(float alt_m) const override {
        const float cm = alt_m * 100;
        return (cm > 5000 || cm < 0) ? 0xFFFF : uint16_t(cm);
    }
};

}

#endif  // AP_SIM_RF_ERDI_ENABLED
