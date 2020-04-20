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
  simulate EMU EFI system
*/

#pragma once

#include <SITL/SITL.h>
#include <AP_HAL/utility/Socket.h>

namespace SITL {

class EFI_EMUECU {
public:
    void update();

private:
    void send_json();
    SocketAPM sock{false};

    uint32_t last_send_ms;
    bool connected;

    void efi_printf(const char *fmt, ...);

    void config_dump();

    struct {
        uint8_t  version;   // int
        uint16_t thr_min;   // us thr input for 0%
        uint16_t thr_start; // us thr input for start
        uint16_t thr_max;   // us thr input for 100%
        uint16_t pwm0_min;  // us throttle
        uint16_t pwm0_max;  // us
        uint16_t pwm1_min;  // us starter?
        uint16_t pwm1_max;  // us
        uint8_t  auto_start; // attempts
        uint16_t rpm_limit; // rpm

        uint8_t  capacity;  // cc
        uint16_t inj_open;  // us
        uint16_t inj_close; // us
        uint16_t inj_flow;  // g/min

        uint16_t idle_rpm;      // rpm
        uint16_t dwell_time_ms; // ms
        uint16_t start_time_ms; // ms

        // tables and calibration
#if 0
        int16_t  a0cal[A_TAB_SIZE]; // 100*degrees
        int16_t  a1cal[A_TAB_SIZE]; // 100*degrees
        inj_ticks_t inj_map[MAP_ROWS][MAP_COLS];   // [throttle  ][rpm] ticks (16us)
        int16_t  ign_adv[MAP_COLS]; // TODO
#endif
        uint16_t checksum;
    } config;

    enum class ECU_State {
        INIT = 0,
        PRIME,
        STOPPED,
        CRANK,
        START,
        RUNNING,
    };

    struct {
        ECU_State state;
        float throttle_in;
        float throttle_out;
        uint16_t rpm;
        int16_t cht;   // -? - ~100
        int16_t iat;   // -? - 80
        uint32_t egt;  // 0 - 1024
        int16_t ecut;  // -? -
        uint32_t baro; // 0 - ~101300pa
        uint16_t humidity;
        // internal vars
        float pt_c;
        uint8_t starts;
        uint16_t engine_stop_ms;
        uint16_t engine_start_ms;
        uint16_t engine_prime_ms;
        // outputs
        uint8_t inj_ticks;
        uint16_t thr_in;
        uint16_t pwm0_out;
        uint16_t pwm1_out;
    } status;
};

}
