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
  Driver for Hi-Link HLK-LD2451 24 GHz mmWave vehicle-detection radar.

  Serial Communication Protocol v1.03 (Hi-Link / Shenzhen Hilink Electronics)

  UART: 115200 baud, 8N1, 3.3 V logic

  Data reporting frame layout
  ─────────────────────────────────────────────────────────────────────────
  Header   : F4 F3 F2 F1                                (4 bytes)
  Length   : [LL HH] little-endian                      (2 bytes)
               = 2 + N_targets × BYTES_PER_TARGET
  count    : number of targets [0-3]                    (1 byte)
  alarm    : 0x01 = at least one target approaching     (1 byte)
  Per target (5 bytes each):
    angle  : raw byte; actual_deg = raw - 0x80          (1 byte, -128..+127°)
    dist   : metres, unsigned                            (1 byte,  0..255 m)
    dir    : 0x00 = moving away, 0x01 = approaching     (1 byte)
    speed  : km/h, unsigned                              (1 byte)
    snr    : signal-to-noise 0–100                       (1 byte)
  Tail     : F8 F7 F6 F5                                (4 bytes)

  Config command frame (not currently used by this driver):
    Header : FD FC FB FA
    Tail   : 04 03 02 01
*/

#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_HLK_LD2451_ENABLED

#include "AP_Proximity_Backend_Serial.h"

// maximum targets the sensor can report
#define HLK_LD2451_MAX_TARGETS        3
#define HLK_LD2451_BYTES_PER_TARGET   5

// frame delimiters
#define HLK_LD2451_HEADER_LEN         4
#define HLK_LD2451_LEN_FIELD_SIZE     2
#define HLK_LD2451_PAYLOAD_OVERHEAD   2   // count byte + alarm byte
#define HLK_LD2451_TAIL_LEN           4

// max total frame = header(4) + len(2) + overhead(2) + 3×5 + tail(4)
#define HLK_LD2451_MAX_FRAME_LEN      (HLK_LD2451_HEADER_LEN +      \
                                       HLK_LD2451_LEN_FIELD_SIZE +   \
                                       HLK_LD2451_PAYLOAD_OVERHEAD + \
                                       HLK_LD2451_MAX_TARGETS * HLK_LD2451_BYTES_PER_TARGET + \
                                       HLK_LD2451_TAIL_LEN)

// sensor limits
#define HLK_LD2451_DIST_MAX_M   200.0f
#define HLK_LD2451_DIST_MIN_M     0.5f

// data timeout
#define HLK_LD2451_TIMEOUT_MS   1000

class AP_Proximity_HLK_LD2451 : public AP_Proximity_Backend_Serial
{
public:
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    void update() override;

    float distance_max_m() const override { return HLK_LD2451_DIST_MAX_M; }
    float distance_min_m() const override { return HLK_LD2451_DIST_MIN_M; }

private:
    void read_sensor_data();
    void parse_frame();

    uint8_t  _buf[HLK_LD2451_MAX_FRAME_LEN];
    uint16_t _buf_len;
    uint16_t _payload_bytes_expected;

    uint32_t _last_packet_ms;

    enum class ParseState : uint8_t {
        HEADER,
        LENGTH,
        PAYLOAD,
    } _state = ParseState::HEADER;
};

#endif  // AP_PROXIMITY_HLK_LD2451_ENABLED
