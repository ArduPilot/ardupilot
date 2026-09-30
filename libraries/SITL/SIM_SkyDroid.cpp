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
  Simulator for SkyDroid gimbal
*/

#include "SIM_config.h"
#include <AP_Common/AP_Common.h>

#if AP_SIM_SKYDROID_ENABLED

#include "SIM_SkyDroid.h"
#include "SIM_Aircraft.h"
#include <AP_Math/AP_Math.h>
#include <errno.h>

using namespace SITL;

void SkyDroid::update(const Aircraft &aircraft)
{
    Matrix3f gimbal_dcm;
    gimbal.get_dcm(gimbal_dcm);
    const Vector3f vehicle_rate_gimbal = gimbal_dcm.transposed() * aircraft.get_dcm() * aircraft.get_gyro();

    // Confirmed on real hardware: GAM/GSM/GAY/GAP are silently ignored, only the
    // individual-axis GSY/GSP speed commands actually move the gimbal, and they are
    // genuinely proportional (unlike PTZ's fixed-speed jog, which was tried first and
    // found not to move yaw at all).  Real-world calibration: see
    // AP_MOUNT_SKYDROID_AXIS_DPS_PER_LSB in AP_Mount_SkyDroid.cpp for the
    // 0.5deg/s-per-LSB scale this mirrors, matching the protocol doc and SkyDroid's
    // own RCSDK.  GSY's sign is also inverted vs the doc on real hardware; this
    // simulation reproduces that same inversion so it cancels out correctly against
    // the driver's compensating negation in send_target_rates(), exactly like the
    // real gimbal does
    const float dps_per_lsb = 0.5f;
    Vector3f ja;
    gimbal.get_joint_angles(ja);
    float pitch_rate;
    float yaw_rate;
    if (_centering) {
        // simulate the gimbal's own one-shot "center" response to "PTZ" 0x05 - drive
        // pitch/yaw toward zero using the same simple P-controller approach the real
        // driver uses for closed-loop control elsewhere, since we have no real
        // hardware data on how a real gimbal's own centering actually moves
        constexpr float gain = 10.0f;
        pitch_rate = -ja.y * gain;
        yaw_rate = -ja.z * gain;
    } else {
        pitch_rate = radians(_commanded_pitch_speed_lsb * dps_per_lsb);
        yaw_rate = -radians(_commanded_yaw_speed_lsb * dps_per_lsb);
    }

    // roll is left entirely to the simulated gimbal's own stabilization, matching the
    // real hardware: SkyDroid have confirmed roll is self-stabilized with no control
    // command on any model in this family
    gimbal.set_demanded_rates(Vector3f(
        vehicle_rate_gimbal.x,
        vehicle_rate_gimbal.y + pitch_rate,
        vehicle_rate_gimbal.z + yaw_rate));

    gimbal.update(aircraft);
    update_input();

    // send attitude at 10 Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_attitude_ms >= 100) {
        _last_attitude_ms = now_ms;
        send_attitude();
    }
}

void SkyDroid::send_attitude()
{
    // Report actual GimbalSim joint angles.
    // joint_angles.y = pitch (negative = down), .z = azimuth.
    // Wire encoding: pitch_cd = pitch_deg * 100, yaw_cd = yaw_deg * 100 (no sign flip)
    Vector3f ja;
    gimbal.get_joint_angles(ja);
    const int16_t yaw_cd   = (int16_t)(degrees(ja.z) * 100.0f);
    const int16_t pitch_cd = (int16_t)(degrees(ja.y) * 100.0f);
    const int16_t roll_cd  = (int16_t)(degrees(ja.x) * 100.0f);

    uint8_t data[12];
    uint16_to_hex4((uint16_t)yaw_cd,   &data[0]);
    uint16_to_hex4((uint16_t)pitch_cd, &data[4]);
    uint16_to_hex4((uint16_t)roll_cd,  &data[8]);

    // attitude data is always sent with identifier "GAC", distinct from the
    // "GAA" enable/rate-request command the driver sends to ask for it
    send_packet('G', "GAC", false, data, sizeof(data));
}

/*
  read bytes from autopilot into _buf, then scan for complete packets.
  Packet format:
    [0]    '#'
    [1]    'T' or 't'
    [2]    'P' or 'p'
    [3]    'U' (addr1, always UDP/external-control for SkyDroid)
    [4]    addr2 ('G','D','M')
    [5]    data_len as a single ASCII hex char
    [6]    'r' or 'w'
    [7..9] 3-char command ID
    [10..10+data_len-1]  data bytes
    [10+data_len..11+data_len]  2-byte CRC
  Total packet length = 12 + data_len
*/
void SkyDroid::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i = search_start_pos; i < _buflen; i++) {
        if (_buf[i] == '#') {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(_buf, &_buf[i], _buflen - i);
    _buflen -= i;
}

void SkyDroid::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&_buf[_buflen], ARRAY_SIZE(_buf) - _buflen - 1);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
        return;
    }
    _buflen += n;

    while (_buflen >= 3) {
        // search for '#' at the start
        if (_buf[0] != '#') {
            move_preamble_in_buffer(1);
            continue;
        }
        if (_buf[1] != 'T' && _buf[1] != 't') {
            move_preamble_in_buffer(1);
            continue;
        }
        if (_buf[2] != 'P' && _buf[2] != 'p') {
            move_preamble_in_buffer(1);
            continue;
        }

        // need at least 6 bytes to read the data_len field
        if (_buflen < 6) {
            break;
        }

        // parse data length from ASCII hex char at [5]
        uint8_t data_len;
        if (!hex_char_to_nibble(_buf[5], data_len)) {
            // invalid data length — discard '#'
            move_preamble_in_buffer(1);
            continue;
        }

        const uint8_t pkt_len = 12 + data_len;
        if (pkt_len > PACKETLEN_MAX) {
            move_preamble_in_buffer(1);
            continue;
        }

        // wait for the full packet
        if (_buflen < pkt_len) {
            break;
        }

        // verify and dispatch the packet
        handle_packet(data_len);
        move_preamble_in_buffer(pkt_len);
    }
}

void SkyDroid::handle_packet(uint8_t data_len)
{
    // verify CRC
    const uint8_t pkt_len = 12 + data_len;
    const uint8_t crc = crc_sum_of_bytes(_buf, pkt_len - 2);
    const uint8_t expected_hi = hex2char((crc >> 4) & 0x0f);
    const uint8_t expected_lo = hex2char(crc & 0x0f);
    if (_buf[pkt_len - 2] != expected_hi || _buf[pkt_len - 1] != expected_lo) {
        return;
    }

    // ID is at bytes [7..9]
    const char *id = (const char*)&_buf[7];

    if (strncmp(id, "GAA", 3) == 0) {
        // attitude streaming enable/rate request
        send_attitude();

    } else if (strncmp(id, "VER", 3) == 0) {
        // version response begins with a literal 'V' (see AP_Mount_SkyDroid::gimbal_version_analyse)
        const uint8_t data[] { 'V', '1', '.', '0', '.', '0' };
        send_packet('D', "VER", false, data, sizeof(data));

    } else if (strncmp(id, "SDC", 3) == 0) {
        // card present: 5 hex chars remaining, 5 hex chars total, not all zero
        const uint8_t data[] { '0','0','0','1','0', '0','0','0','2','0' };
        send_packet('D', "SDC", false, data, sizeof(data));

    } else if (strncmp(id, "MOD", 3) == 0) {
        // model name response is raw ASCII text, e.g. "C13"
        send_packet('D', "MOD", false, (const uint8_t*)_model_name, strlen(_model_name));

    } else if (strncmp(id, "GSY", 3) == 0 && data_len >= 2) {
        // individual-axis yaw speed command - confirmed on real hardware to be the
        // only thing that actually moves yaw: signed 8bit hex value, LSB units
        // calibrated in update() above.  A real speed command supersedes any
        // in-progress centering - see _centering's comment
        _centering = false;
        uint32_t tmp;
        if (hex_chars_to_uint32((const char*)&_buf[10], 2, tmp)) {
            _commanded_yaw_speed_lsb = (int8_t)tmp;
        }

    } else if (strncmp(id, "GSP", 3) == 0 && data_len >= 2) {
        // individual-axis pitch speed command, same as GSY above
        _centering = false;
        uint32_t tmp;
        if (hex_chars_to_uint32((const char*)&_buf[10], 2, tmp)) {
            _commanded_pitch_speed_lsb = (int8_t)tmp;
        }

    } else if (strncmp(id, "PTZ", 3) == 0 && data_len >= 2) {
        // discrete gimbal control.  Only 0x05 ("center") is simulated - see
        // _centering's comment and update()'s use of it.  Follow/lock (0x06/0x07)
        // and the jog codes (0x00-0x04) are not simulated: this driver only ever
        // sends follow/lock (see AP_Mount_SkyDroid::set_gimbal_lock()), which has no
        // observable effect on the simulated gimbal's motion either way
        uint32_t tmp;
        if (hex_chars_to_uint32((const char*)&_buf[10], 2, tmp) && tmp == 0x05) {
            _centering = true;
        }
    }
    // Everything else is absorbed silently, deliberately reproducing the real
    // hardware's behaviour: the combined and absolute-angle commands (GAM, GSM, GAY,
    // GAP) are confirmed silently ignored on real hardware, and roll (GAR, GSR) has
    // no control command at all - SkyDroid have confirmed roll is self-stabilized.
    // FAE, FAI, CAP, REC, DZM and TIM are also absorbed here
}

void SkyDroid::send_packet(char addr2, const char id[3], bool write, const uint8_t *data, uint8_t len)
{
    const uint8_t total = 12 + len;
    if (total > PACKETLEN_MAX) {
        return;
    }

    uint8_t pkt[PACKETLEN_MAX];
    uint8_t ofs = 0;

    pkt[ofs++] = '#';
    pkt[ofs++] = 'T';
    pkt[ofs++] = 'P';
    pkt[ofs++] = 'U';   // SkyDroid always replies to the 'U' (UDP/external-control) address
    pkt[ofs++] = (uint8_t)addr2;
    pkt[ofs++] = hex2char(len & 0x0f);   // data length as single ASCII hex char
    pkt[ofs++] = write ? 'w' : 'r';
    pkt[ofs++] = (uint8_t)id[0];
    pkt[ofs++] = (uint8_t)id[1];
    pkt[ofs++] = (uint8_t)id[2];

    for (uint8_t i = 0; i < len; i++) {
        pkt[ofs++] = data[i];
    }

    // checksum: byte sum of all preceding bytes, encoded as 2 uppercase ASCII hex chars
    const uint8_t crc = crc_sum_of_bytes(pkt, ofs);
    pkt[ofs++] = hex2char((crc >> 4) & 0x0f);
    pkt[ofs++] = hex2char(crc & 0x0f);

    write_to_autopilot((const char*)pkt, ofs);
}

void SkyDroid::uint16_to_hex4(uint16_t val, uint8_t buf[4])
{
    buf[0] = hex2char((val >> 12) & 0x0f);
    buf[1] = hex2char((val >>  8) & 0x0f);
    buf[2] = hex2char((val >>  4) & 0x0f);
    buf[3] = hex2char((val      ) & 0x0f);
}

#endif  // AP_SIM_SKYDROID_ENABLED
