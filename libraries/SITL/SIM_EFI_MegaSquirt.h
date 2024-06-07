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
  simulate MegaSquirt EFI system

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --serial5=sim:megasquirt --speedup=1
param set SERIAL5_PROTOCOL 24
param set SIM_EFI_TYPE 1
param set EFI_TYPE 1
reboot
status EFI_STATUS

./Tools/autotest/autotest.py --gdb --debug build.Plane test.Plane.MegaSquirt

*/

#pragma once

#include <AP_HAL/utility/Socket_native.h>
#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class EFI_MegaSquirt : public SerialDevice {
public:

    using SerialDevice::SerialDevice;

    void update();

private:
    void send_table();

    struct PACKED {
        uint16_t size;
        uint8_t command;
        uint8_t CANid;
        uint8_t table;
        uint16_t table_offset;
        uint16_t table_size;
        uint32_t crc;
    } r_command;
    uint8_t *buf = (uint8_t *)&r_command;
    uint8_t ofs;
    
    struct PACKED {
        uint16_t uptime_s;
        uint16_t pulseWidth1_us;
        uint16_t pulseWidth2_us;
        uint16_t rpm;
        int16_t advance_cdeg;
        int8_t squirt;
        int8_t engine_status;
        uint8_t afr_target1;
        uint8_t afr_target2;
        uint8_t wbo2_en1;
        uint8_t wbo2_en2;
        int16_t baro_hPa;
        int16_t map_hPa;
        int16_t mat_cF;
        int16_t ct_cF;
        int16_t throttle_pos;
        int16_t afr1;
        int16_t afr2;
        int16_t knock;
        int16_t egocor1;
        int16_t egocor2;
        int16_t aircor;
        int16_t warmcor;
        int16_t accel_enrich;
        int16_t tps_fuel_cut;
        int16_t baroCorrection;
        int16_t gammaEnrich;
        int16_t ve1;
        int16_t ve2;
        int16_t iacstep;
        int16_t cold_adv_deg;
        int16_t TPSdot;
        int16_t MAPdot;
        int16_t dwell;
        int16_t MAF;
        uint8_t fuelload;
        uint8_t pad[128-67];
        uint16_t fuelPressure;
    } table7;

    float tps;
};

}
