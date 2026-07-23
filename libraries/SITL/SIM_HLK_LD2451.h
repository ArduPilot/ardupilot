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

  Generates valid LD2451 UART frames and feeds them to a SITL serial port at
  10 Hz, allowing the real AP_Proximity_HLK_LD2451 driver to run unchanged
  inside SITL.

  Frame format (Serial Communication Protocol v1.03):
    Header  : F4 F3 F2 F1                               (4 bytes)
    Length  : [LL HH] little-endian = 2 + N_targets × 5 (2 bytes)
    count   : number of targets [0-3]                    (1 byte)
    alarm   : 0x01 if any target is approaching          (1 byte)
    Per target (5 bytes each):
      angle_raw : actual_deg = raw - 0x80               (1 byte, -128..+127°)
      dist_m    : unsigned metres                        (1 byte)
      direction : 0x00 = moving away, 0x01 = approaching (1 byte)
      speed_kmh : unsigned km/h                          (1 byte)
      snr       : signal-to-noise 0-100                  (1 byte)
    Tail    : F8 F7 F6 F5                               (4 bytes)

  Two synthetic targets are generated:
    - One at +45° body-frame (front-right quadrant)
    - One at -30° body-frame (front-left quadrant)

  Each target distance is derived from the SITL obstacle map via
  measure_distance_at_angle_bf().  If no obstacle is found within the
  sensor's 200 m range the target falls back to a sine-wave that oscillates
  the distance over time, exercising the driver's motion-detection logic.

  Typical sim_vehicle.py usage:
    --serial5=sim:hlk-ld2451

  Required ArduPilot parameters:
    SERIAL5_PROTOCOL 11   (Proximity)
    PRX1_TYPE <LD2451 type index>
    Reboot
*/

#pragma once

#include "SIM_config.h"

#ifndef AP_SIM_HLK_LD2451_ENABLED
#define AP_SIM_HLK_LD2451_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_HLK_LD2451_ENABLED

#include "SIM_SerialProximitySensor.h"

namespace SITL {

class PS_HLK_LD2451 : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

    // Build one complete LD2451 data frame from the current SITL location.
    // Returns the number of bytes written into |data|, or 0 on error.
    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    // LD2451 UART: 115200 baud, 8N1
    uint32_t device_baud() const override { return 115200; }

    // 10 Hz output rate
    uint16_t reading_interval_ms() const override { return 100; }

private:

    // Number of synthetic scan angles
    static const uint8_t NUM_TARGETS = 2;

    // Body-frame probe angles (degrees).  Positive = clockwise from forward.
    static const float PROBE_ANGLES_DEG[NUM_TARGETS];

    // Fallback synthetic distance parameters (used when no SITL obstacle found)
    static const float SYNTH_BASE_DIST_M[NUM_TARGETS];  // centre distance (m)
    static const float SYNTH_AMPLITUDE_M[NUM_TARGETS];   // oscillation amplitude (m)
    static const float SYNTH_PERIOD_S;                   // oscillation period (s)

    // Maximum sensor range (metres) - matches HLK_LD2451_DIST_MAX_M
    static const float DIST_MAX_M;

    // Previous measured distances used to compute relative speed
    float _prev_dist_m[NUM_TARGETS];
    // Timestamp of the previous packet_for_location() call (ms)
    uint32_t _prev_packet_ms;
};

}  // namespace SITL

#endif  // AP_SIM_HLK_LD2451_ENABLED
