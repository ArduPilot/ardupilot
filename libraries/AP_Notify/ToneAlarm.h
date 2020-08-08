/*
 *  ToneAlarm driver
 */
/*
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "NotifyDevice.h"
#include "MMLPlayer.h"

// wait 2 seconds before assuming a tone is done and continuing the continuous tone
#define AP_NOTIFY_TONEALARM_MAX_TONE_LENGTH_MS 2000
#define AP_NOTIFY_TONEALARM_TONE_BUF_SIZE 100

class AP_ToneAlarm: public NotifyDevice {
public:
    /// init - initialised the tone alarm
    bool init(void) override;

    /// update - updates led according to timed_updated.  Should be called at 50Hz
    void update() override;

    // handle a PLAY_TUNE message
    void handle_play_tune(const mavlink_message_t &msg) override;

    // play_tune - play tone specified by the provided string of notes
    void play_tune(const char *tune) override;

private:
    /// play_tune - play one of the pre-defined tunes
    void play_tone(const uint8_t tone_index);

    // stop_cont_tone - stop playing the currently playing continuous tone
    void stop_cont_tone();

    // check_cont_tone - check if we should begin playing a continuous tone
    void check_cont_tone();

    // timer task - runs at 1khz
    void _timer_task();

    /// tonealarm_type - bitmask of states we track
    struct tonealarm_type {
        uint16_t armed                 : 1;    // 0 = disarmed, 1 = armed
        uint16_t failsafe_battery      : 1;    // 1 if battery failsafe
        uint16_t parachute_release     : 1;    // 1 if parachute is being released
        uint16_t pre_arm_check         : 1;    // 0 = failing checks, 1 = passed
        uint16_t failsafe_radio        : 1;    // 1 if radio failsafe
        uint16_t failsafe_gcs          : 1;    // 1 if gcs failsafe
        uint16_t vehicle_lost          : 1;    // 1 if lost copter tone requested
        uint16_t compass_cal_running   : 1;    // 1 if compass calibration is running
        uint16_t waiting_for_throw     : 1;    // 1 if waiting for copter throw launch
        uint16_t leak_detected         : 1;    // 1 if leak detected
        uint16_t powering_off          : 1;    // 1 if smart battery is powering off
    } flags;
    bool _have_played_ready_tone : 1;

    int8_t _cont_tone_playing;
    int8_t _tone_playing;
    uint32_t _tone_beginning_ms;

    struct Tone {
        const char *str;
        const uint8_t continuous : 1;
    };

    const static Tone _tones[];

    HAL_Semaphore _sem;
    MMLPlayer _mml_player;
    char _tone_buf[AP_NOTIFY_TONEALARM_TONE_BUF_SIZE];
};
