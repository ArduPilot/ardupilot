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
  simulate Edge Autonomy EFI system

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --serial5=sim:efi_edge --speedup=1
param set SERIAL5_PROTOCOL 28
param set SIM_EFI_TYPE 9
param set EFI_TYPE 7
param set SCR_ENABLE 1
param set EFI_EDGE_ENABLE 1
reboot
status EFI_STATUS

./Tools/autotest/autotest.py --gdb --debug build.Plane test.Plane.EFIEdge

*/

#pragma once

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class EFI_Edge : public SerialDevice {
public:

    using SerialDevice::SerialDevice;

    void update();

private:

    void update_receive();
    void update_send();
    void update_engine_model();

    void send_packet(uint8_t type, const uint8_t *payload, uint8_t len);
    void send_telem1();
    void send_telem2();

    void init();
    bool init_done;

    uint8_t receive_buf[32];
    uint8_t receive_buf_ofs;

    // telemetry period in 50ms units (default 2 = 10Hz)
    uint8_t telem_period = 2;
    uint32_t last_telem_send_ms;

    bool sw_kill;

    // telem1 payload (type 6, 10 bytes) - Table 7.1-7
    class PACKED Telem1 {
    public:
        uint16_t total_fuel_consumed;  // 0.1 kg
        uint16_t fuel_since_restart;   // grams
        uint16_t engine_work_time;     // 0.01 hour
        uint16_t remaining_fuel;       // grams
        uint16_t analog_input;         // 2mV (max 6V)
    };
    static_assert(sizeof(Telem1) == 10, "Telem1 size");

    // telem2 payload (type 7, 60 bytes) - Table 7.1-8
    class PACKED Telem2 {
    public:
        uint32_t time_raw;        // 0:  0.01s units
        uint16_t duct_i;          // 4:  mA
        uint16_t thrl_i;          // 6:  mA
        uint16_t fp_i;            // 8:  mA
        int8_t   ecu_t;           // 10: degC
        int8_t   gen_t;           // 11: degC
        uint8_t  duct_pos;        // 12: %
        uint8_t  thr_pos;         // 13: %
        uint16_t fuel_flow;       // 14: g/h
        uint16_t sys_status;      // 16: bit2=sw_kill, bit4=hw_kill
        uint8_t  thr_source;      // 18
        uint16_t pulse_w;         // 19: 0.000666 ms
        uint16_t rpm;             // 21
        uint16_t advance;         // 23: 0.1 degrees
        uint8_t  eng_status;      // 25
        int16_t  baro;            // 26: 0.1 kPa
        int16_t  map_p;           // 28: 0.1 kPa
        int16_t  mat;             // 30: 0.05555 degC (0.1 degF)
        int16_t  coolant;         // 32: 0.05555 degC (0.1 degF)
        int16_t  tps;             // 34: 0.1%
        int16_t  afr1;            // 36: 0.1 AFR
        int16_t  ms_volt;         // 38: 0.1V
        int16_t  warmup_enr;     // 40: Warmup Enrichment, %
        int16_t  bar_corr;       // 42: Barometer Correction, %
        int16_t  ve_curr1;       // 44: VE value in use, 0.1%
        uint16_t ign_i;          // 46: Ignition current, mA
        uint16_t inj_i;          // 48: Injector current, mA
        uint8_t  reserved[10];   // 50-59
    };
    static_assert(sizeof(Telem2) == 60, "Telem2 size");

    // engine model
    struct {
        float engine_temperature;  // degC
        uint32_t last_update_ms;
    } engine;
};

}
