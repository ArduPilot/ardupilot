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
  Simulator for Viewpro gimbal
*/

#include "SIM_config.h"

#if AP_SIM_VIEWPRO_ENABLED

#include "SIM_Viewpro.h"
#include "SIM_Aircraft.h"
#include <AP_Math/AP_Math.h>
#include <errno.h>

using namespace SITL;

// scalar from AP_Mount_Viewpro.cpp
#define VIEWPRO_DEG_TO_OUTPUT  (65536.0f / 360.0f)

void Viewpro::update(const Aircraft &aircraft)
{
    // Drive GimbalSim toward the angles commanded via A1 packets.
    // The A1 wire encoding is: pitch_raw = -pitch_deg * DEG_TO_OUTPUT,
    // yaw_raw = yaw_deg * DEG_TO_OUTPUT.  Invert to recover target radians.
    {
        static constexpr float OUTPUT_TO_RAD = (360.0f / 65536.0f) * (float)DEG_TO_RAD;
        const float target_pitch_rad = -_target_pitch_raw * OUTPUT_TO_RAD;
        const float target_yaw_rad   =  _target_yaw_raw   * OUTPUT_TO_RAD;

        Vector3f ja;
        gimbal.get_joint_angles(ja);

        // P-gain: drives gimbal to commanded angle with ~0.1 s time constant.
        // joint_angles.y = pitch (negative = down), .z = azimuth.
        Matrix3f gimbal_dcm;
        gimbal.get_dcm(gimbal_dcm);
        const Vector3f vehicle_rate_gimbal = gimbal_dcm.transposed() * aircraft.get_dcm() * aircraft.get_gyro();

        static constexpr float GAIN = 10.0f;
        gimbal.set_demanded_rates(Vector3f(
            vehicle_rate_gimbal.x,
            vehicle_rate_gimbal.y + (target_pitch_rad - ja.y) * GAIN,
            vehicle_rate_gimbal.z + (target_yaw_rad   - ja.z) * GAIN));
    }

    gimbal.update(aircraft);
    update_input();

    // send attitude at 10 Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_attitude_ms >= 100) {
        _last_attitude_ms = now_ms;
        send_t1_f1_b1_d1();
    }
}

/*
  read bytes from the autopilot and process them byte-by-byte through the
  Viewpro protocol state machine.

  The driver discards header bytes without storing them.  After all three
  header bytes are consumed the internal packet buffer starts fresh:
    _buf[0]   = length+frame_counter byte  (bits 0-5 = body_length)
    _buf[1]   = frame_id byte
    _buf[2+]  = data bytes
    _buf[N]   = CRC byte (last)

  body_length = 1(length byte) + 1(frame_id) + data_len + 1(crc)
  => data_len = body_length - 3
*/
void Viewpro::update_input()
{
    // read new bytes into a scratch buffer; process one at a time
    uint8_t scratch[128];
    const ssize_t n = read_from_autopilot((char*)scratch, sizeof(scratch));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        const uint8_t b = scratch[i];

        switch (_parse_state) {
        case ParseState::HEADER1:
            if (b == HEADER1) {
                _buflen = 0;
                _parse_state = ParseState::HEADER2;
            }
            break;

        case ParseState::HEADER2:
            if (b == HEADER2) {
                _buflen = 0;
                _parse_state = ParseState::HEADER3;
            } else {
                _buflen = 0;
                _parse_state = ParseState::HEADER1;
            }
            break;

        case ParseState::HEADER3:
            if (b == HEADER3) {
                _buflen = 0;
                _parse_state = ParseState::LENGTH;
            } else {
                _buflen = 0;
                _parse_state = ParseState::HEADER1;
            }
            break;

        case ParseState::LENGTH: {
            const uint8_t body_length = b & 0x3F;
            if (body_length < 3) {
                _buflen = 0;
                _parse_state = ParseState::HEADER1;
                break;
            }
            _data_len = body_length - 3;
            _buf[_buflen++] = b;    // _buf[0] = length+fc byte
            _parse_state = ParseState::FRAMEID;
            break;
        }

        case ParseState::FRAMEID:
            _data_bytes_received = 0;
            _buf[_buflen++] = b;    // _buf[1] = frame_id
            _parse_state = (_data_len > 0) ? ParseState::DATA : ParseState::CHECKSUM;
            break;

        case ParseState::DATA:
            if (_buflen < PACKETLEN_MAX) {
                _buf[_buflen++] = b;
            }
            _data_bytes_received++;
            if (_data_bytes_received >= _data_len) {
                _parse_state = ParseState::CHECKSUM;
            }
            break;

        case ParseState::CHECKSUM: {
            if (_buflen < PACKETLEN_MAX) {
                _buf[_buflen++] = b;
            }
            // verify CRC: XOR of _buf[0.._buflen-2] (all bytes except the crc byte itself)
            const uint8_t expected_crc = crc_xor_of_bytes(_buf, _buflen - 1);
            if (expected_crc == b) {
                dispatch_packet();
            }
            _buflen = 0;
            _parse_state = ParseState::HEADER1;
            break;
        }
        }
    }
}

void Viewpro::dispatch_packet()
{
    // _buf[1] = frame_id; _buf[2+] = data (data_start = 2)
    const FrameId frame_id = (FrameId)_buf[1];

    switch (frame_id) {
    case FrameId::HANDSHAKE:
        send_t1_f1_b1_d1();
        break;

    case FrameId::U:
        // _buf[2] = data[0] = CommConfigCmd
        if (_data_len >= 1) {
            send_v_response((CommConfigCmd)_buf[2]);
        }
        break;

    case FrameId::A1:
        // _buf layout: [0]=length_fc, [1]=frame_id, [2]=servo_status,
        //              [3..4]=yaw big-endian, [5..6]=pitch big-endian
        // Store raw int16 values so the driver round-trips them without
        // any additional float rounding.
        if (_data_len >= 5) {
            _target_yaw_raw   = (int16_t)(((uint16_t)_buf[3] << 8) | _buf[4]);
            _target_pitch_raw = (int16_t)(((uint16_t)_buf[5] << 8) | _buf[6]);
        }
        break;
    case FrameId::C1:
    case FrameId::C2:
    case FrameId::E1:
    case FrameId::E2:
    case FrameId::M_AHRS:
    case FrameId::HEARTBEAT:
        // absorb; gimbal physics is updated via update()
        break;

    default:
        break;
    }
}

/*
  send T1_F1_B1_D1 (0x40) packet with current gimbal attitude.

  Driver reads angles using _msg_buff_data_start=2:
    data[22]     = tracking status (bits 3-4; 0 = stopped)
    data[23]     = servo_status(upper 4 bits) | roll_12bit_MSByte(lower 4 bits)
    data[24]     = roll LSByte   → roll_deg = 12-bit * 180/4095 - 90
    data[25..26] = yaw int16 big-endian  → yaw_deg = val * 360/65536
    data[27..28] = pitch int16 big-endian  → pitch_deg = val * 360/65536  (driver negates)
    data[29]     = image sensor (bits 0-2; stored as val+1; 0→EO1)
    data[32]     = recording status (bits 0-2; 0=stopped)
    data[33..34] = rangefinder × 0.1 m
    data[39..40] = zoom × 0.1
*/
void Viewpro::send_t1_f1_b1_d1()
{
    static constexpr uint8_t DATA_LEN = 41;
    // databuff: frame_id (1 byte) + data (DATA_LEN bytes)
    uint8_t databuff[1 + DATA_LEN] {};
    databuff[0] = (uint8_t)FrameId::T1_F1_B1_D1;

    uint8_t *data = &databuff[1];

    // Report actual GimbalSim joint angles.
    // joint_angles.y = pitch (negative = down), .z = azimuth.
    // Wire encoding: pitch_out = -pitch_deg * DEG_TO_OUTPUT (driver negates on read).
    //                yaw_out   =  yaw_deg   * DEG_TO_OUTPUT.
    // roll: 12-bit value where 2048 (midpoint of 0..4095) ≈ 0 deg (actual = +0.022 deg,
    //       within the neutral_tol_deg=0.05 used by MountViewPro).
    // Pitch clamp to ±16380 (±89.978 deg): at exactly ±90 deg (raw ±16384) the float32
    // from_euler quaternion with the small roll (~0.022 deg) has DCM[2][0] > 1.0 in
    // float64 (Python), triggering pymavlink gimbal-lock → 180 deg instead of ±90 deg.
    Vector3f ja;
    gimbal.get_joint_angles(ja);
    const int16_t yaw_out   = (int16_t)(degrees(ja.z) * VIEWPRO_DEG_TO_OUTPUT);
    const int16_t pitch_out = (int16_t)constrain_int16(
        (int16_t)(-degrees(ja.y) * VIEWPRO_DEG_TO_OUTPUT),
        -16380, 16380);
    const uint16_t roll_out = 2048U;

    // bytes 0..21: T1/F1 fields — leave as zero
    data[22] = 0x00;    // tracking stopped

    data[23] = (uint8_t)((roll_out >> 8) & 0x0F);   // lower nibble = roll MSB; upper nibble = 0 (servo status)
    data[24] = (uint8_t)(roll_out & 0xFF);

    data[25] = (uint8_t)((uint16_t)yaw_out >> 8);
    data[26] = (uint8_t)((uint16_t)yaw_out & 0xFF);

    data[27] = (uint8_t)((uint16_t)pitch_out >> 8);
    data[28] = (uint8_t)((uint16_t)pitch_out & 0xFF);

    data[29] = 0x00;    // image sensor bits 0-2 = 0 → driver stores as EO1
    data[32] = 0x00;    // recording stopped
    data[33] = 0x00;    // rangefinder MSB
    data[34] = 0x00;    // rangefinder LSB
    data[39] = 0x00;    // zoom MSB
    data[40] = 0x0A;    // zoom LSB: 10 × 0.1 = 1.0× zoom

    send_packet(databuff, sizeof(databuff));
}

/*
  send V (0x02) response to a U communication config command.

  Firmware version response (QUERY_FIRMWARE_VER = 0xD0):
    data[0] = 0xD0 (echo of CommConfigCmd)
    data[1] = 'S'  prefix
    data[2..9] = "yyyymmdd" date string
    driver reads: major=atoi(data[4..5]), minor=atoi(data[6..7]), patch=atoi(data[8..9])
    Using "20220301": major=22, minor=03, patch=01

  Model name response (QUERY_MODEL = 0xE4):
    data[0] = 0xE4
    data[1+] = model name bytes
    driver: memcpy(_model_name, &data[1], data_bytes_received-1)
*/
void Viewpro::send_v_response(CommConfigCmd cmd)
{
    uint8_t databuff[32] {};
    uint8_t total_len;

    databuff[0] = (uint8_t)FrameId::V;

    switch (cmd) {
    case CommConfigCmd::QUERY_FIRMWARE_VER:
        databuff[1] = (uint8_t)cmd;
        databuff[2] = 'S';          // prefix character
        databuff[3] = 'y';
        databuff[4] = 's';
        databuff[5] = '2';          // major hi: "20"
        databuff[6] = '0';
        databuff[7] = '2';          // minor hi: "22"
        databuff[8] = '2';
        databuff[9] = '0';          // patch hi: "01"
        databuff[10] = '1';
        total_len = 11;
        break;

    case CommConfigCmd::QUERY_MODEL:
        databuff[1] = (uint8_t)cmd;
        databuff[2] = 'S';
        databuff[3] = 'I';
        databuff[4] = 'M';
        databuff[5] = '_';
        databuff[6] = 'V';
        databuff[7] = 'P';
        // pad to 10 name bytes so the driver's fixed-length copy does not
        // reach the CRC byte; databuff is zero-initialised so bytes 8-11 = 0
        total_len = 12;
        break;

    default:
        return;
    }

    send_packet(databuff, total_len);
}

/*
  build and write a complete Viewpro packet to the autopilot.
  databuff[0] = frame_id, databuff[1+] = data payload.

  Wire format:
    0x55 0xAA 0xDC [length_fc] [databuff...] [crc]
  where:
    length_fc bits 6-7 = frame counter (0-3, increments per packet)
    length_fc bits 0-5 = databuff_len + 2  (= body_length)
    crc = XOR of [length_fc .. last_databuff_byte]
*/
void Viewpro::send_packet(const uint8_t *databuff, uint8_t databuff_len)
{
    // total wire bytes: 3 (header) + 1 (length_fc) + databuff_len + 1 (crc)
    const uint8_t wire_len = 5 + databuff_len;
    if (wire_len > (uint8_t)(PACKETLEN_MAX + 4)) {
        return;
    }

    uint8_t pkt[PACKETLEN_MAX + 4];
    uint8_t ofs = 0;

    pkt[ofs++] = HEADER1;
    pkt[ofs++] = HEADER2;
    pkt[ofs++] = HEADER3;

    _frame_counter = (_frame_counter + 1) & 0x03;
    const uint8_t body_length = (uint8_t)(databuff_len + 2);
    pkt[ofs++] = (uint8_t)((_frame_counter << 6) | (body_length & 0x3F));

    for (uint8_t i = 0; i < databuff_len; i++) {
        pkt[ofs++] = databuff[i];
    }

    // CRC: XOR of bytes [3]..[ofs-1] (length_fc through last data byte)
    pkt[ofs] = crc_xor_of_bytes(&pkt[3], ofs - 3);
    ofs++;

    write_to_autopilot((const char*)pkt, ofs);
}


#endif  // AP_SIM_VIEWPRO_ENABLED
