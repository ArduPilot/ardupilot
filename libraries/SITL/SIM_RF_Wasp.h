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
  Simulator for the Wasp serial rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:wasp --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 18
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600
*/

#pragma once

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_Wasp : public SerialRangeFinder {
public:

    static SerialRangeFinder *create() { return NEW_NOTHROW RF_Wasp(); }

    void update(float range) override;

    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;

    // 20Hz; if Wasp driver doesn't get a reading each time its update
    // is called it goes NoData
    uint16_t reading_interval_ms() const override { return 50; }

private:

    void check_configuration();

    struct {
        bool go;
        char format[16]; // e.g. ASCII
        char baud[5]; // low or high
        char lbe[7]; // big or little
        int frq;
        int aut;
        int mavg;
        int medf;
        int avg;
        int auv;
    } config;

    const struct {
        const char *name;
        char *value;
    } string_configs[3] {
        { "FMT", config.format },
        { "BAUD", config.baud },
        { "LBE", config.lbe },
    };
    const struct {
        const char *name;
        int *value;
    } integer_configs[6] {
        { "FRQ", &config.frq },
        { "AUT", &config.aut },
        { "MAVG", &config.mavg },
        { "MEDF", &config.medf },
        { "AVG", &config.avg },
        { "AUV", &config.auv },
    };

    char _buffer[256]; // from-autopilot
    uint8_t _buflen;

};

}
