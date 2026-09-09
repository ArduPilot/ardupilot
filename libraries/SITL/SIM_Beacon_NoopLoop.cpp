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
  Simulator for the NoopLoop (LinkTrack) UWB beacon system
*/

#include "SIM_Beacon_NoopLoop.h"

#if AP_SIM_NOOPLOOP_ENABLED

#include <AP_HAL/AP_HAL.h>

using namespace SITL;

// NODE_FRAME2 layout (tag position + anchor ranges); offsets and
// scalings must match AP_Beacon/AP_Beacon_Nooploop.cpp:
#define NOOPLOOP_HEADER                     0x55
#define NOOPLOOP_FUNCTION_MARK_NODE_FRAME2  0x04
#define NOOPLOOP_NF2_SYSTIME                6    // 4 bytes, system time (ms)
#define NOOPLOOP_NF2_PRECISION_X            10   // 1 byte each x,y,z, m*100
#define NOOPLOOP_NF2_POSX                   13   // 3 bytes, ENU x (East) m*1000
#define NOOPLOOP_NF2_POSY                   16   // 3 bytes, ENU y (North) m*1000
#define NOOPLOOP_NF2_POSZ                   19   // 3 bytes, ENU z (Up) m*1000
#define NOOPLOOP_NF2_VALID_NODES            118  // 1 byte, number of node blocks
#define NOOPLOOP_NF2_NODE_BLOCK             119  // start of first node block
#define NOOPLOOP_NF2_NODE_BLOCK_LEN         13   // bytes per node block

// SETTING_FRAME0 layout (anchor positions):
#define NOOPLOOP_HEADER2                       0x54
#define NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0  0x00
#define NOOPLOOP_SF0_SZ                        128  // total frame length
#define NOOPLOOP_SF0_A0                        37   // offset of first anchor
#define NOOPLOOP_SF0_ANCHOR_LEN                9    // 3 x 3-byte position

// anchors arranged in a rectangle around the beacon origin (NED metres):
Vector3f Beacon_NoopLoop::anchor_position(uint8_t i) const
{
    switch (i) {
    case 0: return Vector3f{ 25,  25, 0};  // NE
    case 1: return Vector3f{-25,  25, 0};  // SE
    case 2: return Vector3f{-25, -25, 0};  // SW
    case 3: return Vector3f{ 25, -25, 0};  // NW
    }
    return Vector3f{};
}

void Beacon_NoopLoop::put_int24_le(uint8_t *buf, int32_t value_mm)
{
    buf[0] = value_mm & 0xff;
    buf[1] = (value_mm >> 8) & 0xff;
    buf[2] = (value_mm >> 16) & 0xff;
}

void Beacon_NoopLoop::send_data(const Vector3f &pos_ned)
{
    // the driver discards NODE_FRAME2 frames until it has the anchor
    // positions, so (re)send those each cycle ahead of the position:
    send_setting_frame0();
    send_node_frame2(pos_ned);
}

void Beacon_NoopLoop::send_setting_frame0()
{
    uint8_t buf[NOOPLOOP_SF0_SZ] {};

    buf[0] = NOOPLOOP_HEADER2;
    buf[1] = NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0;

    for (uint8_t i=0; i<num_anchors; i++) {
        const Vector3f pos = anchor_position(i);
        uint8_t *p = &buf[NOOPLOOP_SF0_A0 + i*NOOPLOOP_SF0_ANCHOR_LEN];
        // anchor position is carried as ENU millimetres:
        put_int24_le(&p[0], int32_t(pos.y * 1000));   // East
        put_int24_le(&p[3], int32_t(pos.x * 1000));   // North
        put_int24_le(&p[6], int32_t(-pos.z * 1000));  // Up
    }

    // the checksum is the modulo-256 sum of all preceding bytes:
    uint8_t crc = 0;
    for (uint16_t i=0; i<NOOPLOOP_SF0_SZ-1; i++) {
        crc += buf[i];
    }
    buf[NOOPLOOP_SF0_SZ-1] = crc;

    write_to_autopilot((const char*)buf, sizeof(buf));
}

void Beacon_NoopLoop::send_node_frame2(const Vector3f &pos_ned)
{
    const uint16_t frame_len = NOOPLOOP_NF2_NODE_BLOCK + num_anchors*NOOPLOOP_NF2_NODE_BLOCK_LEN + 1;
    uint8_t buf[NOOPLOOP_NF2_NODE_BLOCK + num_anchors*NOOPLOOP_NF2_NODE_BLOCK_LEN + 1] {};

    buf[0] = NOOPLOOP_HEADER;
    buf[1] = NOOPLOOP_FUNCTION_MARK_NODE_FRAME2;
    buf[2] = frame_len & 0xff;
    buf[3] = (frame_len >> 8) & 0xff;

    // system time (ms):
    const uint32_t now_ms = AP_HAL::millis();
    buf[NOOPLOOP_NF2_SYSTIME+0] = now_ms & 0xff;
    buf[NOOPLOOP_NF2_SYSTIME+1] = (now_ms >> 8) & 0xff;
    buf[NOOPLOOP_NF2_SYSTIME+2] = (now_ms >> 16) & 0xff;
    buf[NOOPLOOP_NF2_SYSTIME+3] = (now_ms >> 24) & 0xff;

    // position precision in each axis (m*100); report 0.1m:
    buf[NOOPLOOP_NF2_PRECISION_X+0] = 10;
    buf[NOOPLOOP_NF2_PRECISION_X+1] = 10;
    buf[NOOPLOOP_NF2_PRECISION_X+2] = 10;

    // tag (vehicle) position, ENU millimetres:
    put_int24_le(&buf[NOOPLOOP_NF2_POSX], int32_t(pos_ned.y * 1000));   // East
    put_int24_le(&buf[NOOPLOOP_NF2_POSY], int32_t(pos_ned.x * 1000));   // North
    put_int24_le(&buf[NOOPLOOP_NF2_POSZ], int32_t(-pos_ned.z * 1000));  // Up

    // distance from the tag to each anchor:
    buf[NOOPLOOP_NF2_VALID_NODES] = num_anchors;
    for (uint8_t i=0; i<num_anchors; i++) {
        uint8_t *block = &buf[NOOPLOOP_NF2_NODE_BLOCK + i*NOOPLOOP_NF2_NODE_BLOCK_LEN];
        const float dist = (pos_ned - anchor_position(i)).length();
        block[1] = i;                                  // node id
        put_int24_le(&block[2], int32_t(dist * 1000)); // distance (mm)
    }

    // the checksum is the modulo-256 sum of all preceding bytes:
    uint8_t crc = 0;
    for (uint16_t i=0; i<frame_len-1U; i++) {
        crc += buf[i];
    }
    buf[frame_len-1] = crc;

    write_to_autopilot((const char*)buf, frame_len);
}

#endif  // AP_SIM_NOOPLOOP_ENABLED
