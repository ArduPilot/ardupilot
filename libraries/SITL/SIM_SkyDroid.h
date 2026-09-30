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
  Simulator for SkyDroid gimbal.  This one class simulates every model in
  SkyDroid's "TOP protocol" gimbal camera family: SkyDroid have confirmed the
  gimbal-control commands are identical across models, so the ONLY thing the
  simulated model name changes is the "MOD" response.  Registered under two
  device names (see SITL_State_common.cpp) purely so a test can check the
  driver behaves identically regardless of which model answers.

  Roll is deliberately not controllable here, matching the real hardware:
  SkyDroid have confirmed roll is self-stabilized with no control command on
  any model, so GAR/GSR are absorbed and ignored like any other unimplemented
  command.

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:skydroid --speedup=1

param set MNT1_TYPE 15       # skydroid
param set SERIAL5_PROTOCOL 8 # gimbal
reboot

Use --serial5=sim:skydroid_c13 to have the gimbal report itself as a "C13"
instead of a "C11".  Its control behaviour is identical.

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SKYDROID_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"

namespace SITL {

class SkyDroid : public Mount {
public:

    // model_name is returned verbatim in response to the "MOD" command (e.g. "C11",
    // "C13").  It affects nothing else - control behaviour is model-independent
    SkyDroid(const char *model_name) :
        _model_name(model_name) {}

    void update(const Aircraft &aircraft) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    const char *_model_name;

    // input accumulation buffer; also used as working buffer by handle_packet()
    static constexpr uint8_t PACKETLEN_MAX = 28;
    uint8_t _buf[PACKETLEN_MAX];
    uint8_t _buflen;

    uint32_t _last_attitude_ms;     // time of last attitude packet sent

    // last GSY/GSP speed values received (signed 8bit wire units).  These
    // individual-axis speed commands are the only ones that actually move the real
    // gimbal (GAM/GSM/GAY/GAP are all confirmed silently ignored on real hardware),
    // so they're the only ones simulated here
    int8_t _commanded_yaw_speed_lsb;
    int8_t _commanded_pitch_speed_lsb;

    // true from the moment a "PTZ" 0x05 ("center") command is received until the
    // next GSY/GSP speed command arrives - see update()'s use of this to simulate
    // the gimbal's own one-shot centering behaviour (AP_Mount_SkyDroid::
    // send_target_neutral()/send_target_retracted() send this instead of driving
    // GSY/GSP themselves, so without this the simulated gimbal would never move
    // for RETRACT/NEUTRAL and the autotest's neutral-position check would fail)
    bool _centering;

    // read and dispatch incoming packets from autopilot
    void update_input();

    // scan forward from search_start_pos for '#' and move it to _buf[0]
    void move_preamble_in_buffer(uint8_t search_start_pos);

    // send gimbal attitude packet to the driver
    void send_attitude();

    // dispatch a complete packet beginning at _buf[0], data_len data bytes
    void handle_packet(uint8_t data_len);

    // build and send a response packet.  SkyDroid's control address is always 'U'
    // (UDP-only device, no separate UART/network interface ambiguity)
    void send_packet(char addr2, const char id[3], bool write, const uint8_t *data, uint8_t len);

    // encode a uint16 as 4 uppercase ASCII hex chars
    static void uint16_to_hex4(uint16_t val, uint8_t buf[4]);

    // convert a nibble (0-15) to an uppercase ASCII hex character
    static uint8_t hex2char(uint8_t nibble) {
        return nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10);
    }
};

}  // namespace SITL

#endif  // AP_SIM_SKYDROID_ENABLED
