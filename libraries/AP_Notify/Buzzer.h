/*
  Buzzer driver
*/
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
#pragma once

#include "NotifyDevice.h"

class Buzzer: public NotifyDevice
{
public:
    /// Constructor
    Buzzer() {}

    /// init - initialise the buzzer
    bool init(void) override;

    /// update - updates buzzer according to timed_updated.  Should be called at 50Hz
    void update() override;

private:

    /// on - turns the buzzer on or off
    void on(bool on_off);

    // Patterns - how many beeps will be played; read from
    // left-to-right, each bit represents 100ms
    static const uint32_t    SINGLE_BUZZ = 0b10000000000000000000000000000000UL;
    static const uint32_t    DOUBLE_BUZZ = 0b10100000000000000000000000000000UL;
    static const uint32_t    ARMING_BUZZ = 0b11111111111111111111111111111100UL; // 3s
    static const uint32_t      BARO_BUZZ = 0b10101010100000000000000000000000UL;
    static const uint32_t        EKF_BAD = 0b11101101010000000000000000000000UL;
    static const uint32_t      INIT_GYRO = 0b10010010010010010010000000000000UL;
    static const uint32_t   PRE_ARM_GOOD = 0b10101011111111110000000000000000UL;

    /// play_pattern - plays the defined buzzer pattern
    void play_pattern(const uint32_t pattern);

    /// buzzer_flag_type - bitmask of current state and ap_notify states we track
    struct buzzer_flag_type {
        uint8_t on                  : 1;    // 1 if the buzzer is currently on
        uint8_t arming              : 1;    // 1 if we are beginning the arming process
        uint8_t armed               : 1;    // 0 = disarmed, 1 = armed
        uint8_t failsafe_battery    : 1;    // 1 if battery failsafe has triggered
        uint8_t ekf_bad             : 1;    // 1 if ekf position has gone bad
        uint8_t gyro_calibrated     : 1;    // 1 if calibrating gyro
        uint8_t pre_arm_check       : 1;    // 1 if pre-arm check has passed
    } _flags;

    uint32_t _pattern;           // current pattern
    uint8_t _pin;
    uint32_t _pattern_start_time;

    // enforce minumum 100ms interval between patterns:
    const uint16_t _pattern_start_interval_time_ms = 32*100 + 100;

    void update_playing_pattern();
    void update_pattern_to_play();

};
