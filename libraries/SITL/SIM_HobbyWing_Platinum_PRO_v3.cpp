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
  Simulator for the HobbyWing Platinum Pro v3

 - frame gap is used to strengthen protocol integrity in ArduPilot
 - packet is emitted every 20ms

*/

#include "SIM_Aircraft.h"

#include "SIM_HobbyWing_Platinum_PRO_v3.h"
#include "SITL.h"
#include "SITL_Input.h"

#include <AP_HAL/utility/sparse-endian.h>

using namespace SITL;

HobbyWing_Platinum_PRO_v3::HobbyWing_Platinum_PRO_v3(uint8_t _servo_channel_number) : SerialDevice::SerialDevice(), servo_channel_number{_servo_channel_number}
{
}

void HobbyWing_Platinum_PRO_v3::update(const Aircraft &aircraft, const sitl_input &input)
{
    // only sent every 20ms
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms < 20) {
        return;
    }
    last_send_ms = now_ms;

    uint16_t pwm = 0;
    const uint16_t servo_channel_offset = servo_channel_number - 1;
    if (servo_channel_offset < ARRAY_SIZE(input.servos)) {
        pwm = input.servos[servo_channel_offset];
    }

    SITL::SIM* sitl = AP::sitl();
    const int16_t rpm = sitl->state.rpm[servo_channel_offset];
    // ::printf("sitl rpm=%d\n", rpm);

    if (rpm == 0) {
        // never sent with zero rpm
        return;
    }

    const struct Packet {
        uint8_t header; // 0x9B
        uint8_t seqno_high;  // 24-bit "package number"
        uint8_t seqno_mid;
        uint8_t seqno_low;
        uint16_t throttle_req_pwm;
        uint16_t throttle_out_pwm;
        uint16_t rpm;
    } packet {
        0x9B,  // MAGIC!
        uint8_t((counter >> 16) & 0xff),
        uint8_t((counter >> 8) & 0xff),
        uint8_t(counter & 0xff),
        htobe16(pwm - 1000),  // throttle request
        htobe16(pwm - 1000),   // throttle output
        htobe16(commutation_time_from_rpm(rpm))
    };

    write_to_autopilot((char*)&packet, sizeof(packet));
}

uint32_t HobbyWing_Platinum_PRO_v3::commutation_time_from_rpm(int16_t rpm) const
{
    if (rpm == 0) {
        abort();
    }

    // convert from rpm to a commutation interval (in microseconds),
    // assuming 28 poles / 14 pole-pairs
    return 60000000.0 / (rpm * 14 * 0.5);
}

HobbyWing_Platinum_PRO_v3 *HobbyWing_Platinum_PRO_v3s::create(uint8_t servo_channel_number)
{
    if (num_escs >= ARRAY_SIZE(escs)) {
        return nullptr;
    }
    escs[num_escs] = new HobbyWing_Platinum_PRO_v3(servo_channel_number);  // starting at 1, typically 3 for throttle
    if (escs[num_escs] == nullptr) {
        return nullptr;
    }
    return escs[num_escs++];
}
