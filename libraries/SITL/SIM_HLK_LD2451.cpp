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
  Simulator for the Hi-Link HLK-LD2451 24 GHz mmWave radar proximity sensor.
*/

#include "SIM_config.h"

#if AP_SIM_HLK_LD2451_ENABLED

#include "SIM_HLK_LD2451.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

using namespace SITL;

// ─────────────────────────────────────────────────────────────────────────────
// Static constant definitions
// ─────────────────────────────────────────────────────────────────────────────

// Body-frame probe angles: +45° (front-right) and -30° (front-left)
const float PS_HLK_LD2451::PROBE_ANGLES_DEG[PS_HLK_LD2451::NUM_TARGETS] = { 45.0f, -30.0f };

// Fallback synthetic distances when no SITL obstacle is found within range.
// Target 0: oscillates 6 ± 3 m  (3 – 9 m)
// Target 1: oscillates 10 ± 2 m (8 – 12 m)
const float PS_HLK_LD2451::SYNTH_BASE_DIST_M[PS_HLK_LD2451::NUM_TARGETS] = {  6.0f, 10.0f };
const float PS_HLK_LD2451::SYNTH_AMPLITUDE_M[PS_HLK_LD2451::NUM_TARGETS] = {  3.0f,  2.0f };

// Both synthetic targets oscillate with this period so they are out of phase
// by the angular offset of their probe angles, giving varied but predictable
// motion for driver regression tests.
const float PS_HLK_LD2451::SYNTH_PERIOD_S = 5.0f;

// Sensor maximum range (mirrors HLK_LD2451_DIST_MAX_M in the driver header)
const float PS_HLK_LD2451::DIST_MAX_M = 200.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Frame constants  (Serial Communication Protocol v1.03)
// ─────────────────────────────────────────────────────────────────────────────

static const uint8_t FRAME_HEADER[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
static const uint8_t FRAME_TAIL[4]   = { 0xF8, 0xF7, 0xF6, 0xF5 };

static const uint8_t HEADER_LEN     = 4;
static const uint8_t LEN_FIELD_SIZE = 2;
static const uint8_t PAYLOAD_OH     = 2;  // count byte + alarm byte
static const uint8_t BYTES_PER_TGT  = 5;
static const uint8_t TAIL_LEN       = 4;

// ─────────────────────────────────────────────────────────────────────────────
// packet_for_location()
// ─────────────────────────────────────────────────────────────────────────────

uint32_t PS_HLK_LD2451::packet_for_location(const Location &location,
                                             uint8_t *data,
                                             uint8_t buflen)
{
    // Maximum frame size: header(4) + len(2) + overhead(2) + 2×5 + tail(4) = 22
    const uint8_t MAX_FRAME = HEADER_LEN + LEN_FIELD_SIZE +
                              PAYLOAD_OH + NUM_TARGETS * BYTES_PER_TGT +
                              TAIL_LEN;
    if (buflen < MAX_FRAME) {
        return 0;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const float    now_s  = now_ms * 1e-3f;

    // Time elapsed since the last frame (used to derive speed from dDistance)
    float dt_s = 0.1f;  // default 10 Hz interval
    if (_prev_packet_ms != 0) {
        const uint32_t elapsed_ms = now_ms - _prev_packet_ms;
        if (elapsed_ms > 0 && elapsed_ms < 2000) {
            dt_s = elapsed_ms * 1e-3f;
        }
    }
    _prev_packet_ms = now_ms;

    // ── Build per-target data ────────────────────────────────────────────────
    struct TargetData {
        float   dist_m;
        float   angle_deg;
        bool    approaching;
        uint8_t speed_kmh;
        uint8_t snr;
        bool    valid;
    } targets[NUM_TARGETS];

    for (uint8_t i = 0; i < NUM_TARGETS; i++) {
        const float angle_deg = PROBE_ANGLES_DEG[i];

        // Try the SITL obstacle map first.
        float dist_m = measure_distance_at_angle_bf(location, angle_deg);

        if (dist_m <= 0.0f || dist_m > DIST_MAX_M) {
            // No real obstacle found – use a sine-wave fallback target so the
            // driver always has data to parse.  Each target uses a slightly
            // different phase so they oscillate independently.
            const float phase = (float)i * float(M_PI) / NUM_TARGETS;
            const float t     = (now_s / SYNTH_PERIOD_S) * float(2.0 * M_PI) + phase;
            dist_m = SYNTH_BASE_DIST_M[i] + SYNTH_AMPLITUDE_M[i] * sinf(t);
        }

        // Clamp to valid sensor range.
        dist_m = constrain_float(dist_m, 0.5f, DIST_MAX_M);

        // Derive speed and direction from previous distance.
        const float delta_m     = dist_m - _prev_dist_m[i];
        const bool  approaching = (delta_m < 0.0f);

        // Speed in km/h: |Δd / Δt| converted from m/s to km/h
        float speed_ms  = (dt_s > 0.0f) ? fabsf(delta_m) / dt_s : 0.0f;
        float speed_kmh = speed_ms * 3.6f;
        if (speed_kmh > 255.0f) {
            speed_kmh = 255.0f;
        }

        // SNR: closer targets have stronger returns; 5-95 range.
        // Linear: 95 at 0.5 m, 5 at 200 m.
        float snr_f = 95.0f - 90.0f * (dist_m - 0.5f) / (DIST_MAX_M - 0.5f);
        snr_f = constrain_float(snr_f, 5.0f, 95.0f);

        targets[i].dist_m     = dist_m;
        targets[i].angle_deg  = angle_deg;
        targets[i].approaching = approaching;
        targets[i].speed_kmh  = static_cast<uint8_t>(speed_kmh);
        targets[i].snr        = static_cast<uint8_t>(snr_f);
        targets[i].valid      = true;

        _prev_dist_m[i] = dist_m;
    }

    // ── Count valid targets and set alarm ───────────────────────────────────
    uint8_t count = 0;
    bool    alarm = false;
    for (uint8_t i = 0; i < NUM_TARGETS; i++) {
        if (targets[i].valid) {
            count++;
            if (targets[i].approaching) {
                alarm = true;
            }
        }
    }

    // ── Serialise into the LD2451 binary frame ──────────────────────────────
    uint8_t *p = data;

    // Header
    memcpy(p, FRAME_HEADER, HEADER_LEN);
    p += HEADER_LEN;

    // Length field: 2 (overhead) + count × 5
    const uint16_t payload_len = PAYLOAD_OH + count * BYTES_PER_TGT;
    p[0] = static_cast<uint8_t>(payload_len & 0xFF);
    p[1] = static_cast<uint8_t>(payload_len >> 8);
    p += LEN_FIELD_SIZE;

    // Payload: count + alarm
    *p++ = count;
    *p++ = alarm ? 0x01u : 0x00u;

    // Per-target payload
    for (uint8_t i = 0; i < NUM_TARGETS; i++) {
        if (!targets[i].valid) {
            continue;
        }

        // angle_raw: actual_degrees = raw - 0x80
        // Clamp to int8 range before encoding.
        const float angle_clamped = constrain_float(targets[i].angle_deg, -128.0f, 127.0f);
        const int8_t  angle_signed = static_cast<int8_t>(angle_clamped);
        *p++ = static_cast<uint8_t>(angle_signed + 0x80);

        // Distance: metres, unsigned byte (clamped 0-255)
        const float d = constrain_float(targets[i].dist_m, 0.0f, 255.0f);
        *p++ = static_cast<uint8_t>(d);

        // Direction
        *p++ = targets[i].approaching ? 0x01u : 0x00u;

        // Speed
        *p++ = targets[i].speed_kmh;

        // SNR
        *p++ = targets[i].snr;
    }

    // Tail
    memcpy(p, FRAME_TAIL, TAIL_LEN);
    p += TAIL_LEN;

    return static_cast<uint32_t>(p - data);
}

#endif  // AP_SIM_HLK_LD2451_ENABLED
