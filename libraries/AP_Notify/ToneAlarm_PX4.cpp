/*
  ToneAlarm PX4 driver
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "ToneAlarm_PX4.h"
#include "AP_Notify.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <drivers/drv_tone_alarm.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

const ToneAlarm_PX4::Tone ToneAlarm_PX4::_tones[] {
    #define AP_NOTIFY_PX4_TONE_QUIET_NEG_FEEDBACK 0
    { "MFT200L4<<<B#A#2", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_NEG_FEEDBACK 1
    { "MFT100L4>B#A#2P8B#A#2", false },
    #define AP_NOTIFY_PX4_TONE_QUIET_NEU_FEEDBACK 2
    { "MFT200L4<B#", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_NEU_FEEDBACK 3
    { "MFT100L4>B#", false },
    #define AP_NOTIFY_PX4_TONE_QUIET_POS_FEEDBACK 4
    { "MFT200L4<A#B#", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_POS_FEEDBACK 5
    { "MFT100L4>A#B#", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_READY_OR_FINISHED 6
    { "MFT100L4>G#6A#6B#4", false },
    #define AP_NOTIFY_PX4_TONE_QUIET_READY_OR_FINISHED 7
    { "MFT200L4<G#6A#6B#4", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_ATTENTION_NEEDED 8
    { "MFT100L4>B#B#B#B#", false },
    #define AP_NOTIFY_PX4_TONE_QUIET_ARMING_WARNING 9
    { "MNT75L1O2G", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_WP_COMPLETE 10
    { "MFT200L8G>C3", false },
    #define AP_NOTIFY_PX4_TONE_LOUD_LAND_WARNING_CTS 11
    { "MBT200L2A-G-A-G-A-G-", true },
    #define AP_NOTIFY_PX4_TONE_LOUD_VEHICLE_LOST_CTS 12
    { "MBT200>B#1", true },
    #define AP_NOTIFY_PX4_TONE_LOUD_BATTERY_ALERT_CTS 13
    { "MBNT255>B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8B#8", true },
    #define AP_NOTIFY_PX4_TONE_QUIET_COMPASS_CALIBRATING_CTS 14
    { "MBNT255<C16P2", true },
    #define AP_NOTIFY_PX4_TONE_WAITING_FOR_THROW 15
    { "MBNT90L4O2A#O3DFN0N0N0", true},
    #define AP_NOTIFY_PX4_TONE_LOUD_1 16
    { "MFT100L8>B", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_2 17
    { "MFT100L8>BB", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_3 18
    { "MFT100L8>BBB", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_4 19
    { "MFT100L8>BBBB", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_5 20
    { "MFT100L8>BBBBB", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_6 21
    { "MFT100L8>BBBBBB", false},
    #define AP_NOTIFY_PX4_TONE_LOUD_7 22
    { "MFT100L8>BBBBBBB", false},
    #define AP_NOTIFY_PX4_TONE_TUNING_START 23
    { "MFT100L20>C#D#", false},
    #define AP_NOTIFY_PX4_TONE_TUNING_SAVE 24
    { "MFT100L10DBDB>", false},
    #define AP_NOTIFY_PX4_TONE_TUNING_ERROR 25
    { "MFT100L10>BBBBBBBB", false},
};

bool ToneAlarm_PX4::init()
{
    // open the tone alarm device
    _tonealarm_fd = open(TONEALARM0_DEVICE_PATH, O_WRONLY);
    if (_tonealarm_fd == -1) {
        hal.console->printf("ToneAlarm_PX4: Unable to open " TONEALARM0_DEVICE_PATH);
        return false;
    }
    
    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    flags.armed = AP_Notify::flags.armed;
    flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    flags.pre_arm_check = 1;
    _cont_tone_playing = -1;
    return true;
}

// play_tune - play one of the pre-defined tunes
void ToneAlarm_PX4::play_tone(const uint8_t tone_index)
{
    uint32_t tnow_ms = AP_HAL::millis();
    const Tone &tone_requested = _tones[tone_index];

    if(tone_requested.continuous) {
        _cont_tone_playing = tone_index;
    }

    _tone_playing = tone_index;
    _tone_beginning_ms = tnow_ms;

    play_string(tone_requested.str);
}

void ToneAlarm_PX4::play_string(const char *str) {
    write(_tonealarm_fd, str, strlen(str) + 1);
}

void ToneAlarm_PX4::stop_cont_tone() {
    if(_cont_tone_playing == _tone_playing) {
        play_string("");
        _tone_playing = -1;
    }
    _cont_tone_playing = -1;
}

void ToneAlarm_PX4::check_cont_tone() {
    uint32_t tnow_ms = AP_HAL::millis();
    // if we are supposed to be playing a continuous tone,
    // and it was interrupted, and the interrupting tone has timed out,
    // resume the continuous tone

    if (_cont_tone_playing != -1 && _tone_playing != _cont_tone_playing && tnow_ms-_tone_beginning_ms > AP_NOTIFY_PX4_MAX_TONE_LENGTH_MS) {
        play_tone(_cont_tone_playing);
    }
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void ToneAlarm_PX4::update()
{
    // exit immediately if we haven't initialised successfully
    if (_tonealarm_fd == -1) {
        return;
    }

    // exit if buzzer is not enabled
    if (pNotify->buzzer_enabled() == false) {
        return;
    }

    check_cont_tone();

    if (AP_Notify::flags.compass_cal_running != flags.compass_cal_running) {
        if(AP_Notify::flags.compass_cal_running) {
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_COMPASS_CALIBRATING_CTS);
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_POS_FEEDBACK);
        } else {
            if(_cont_tone_playing == AP_NOTIFY_PX4_TONE_QUIET_COMPASS_CALIBRATING_CTS) {
                stop_cont_tone();
            }
        }
    }
    flags.compass_cal_running = AP_Notify::flags.compass_cal_running;

    if (AP_Notify::events.compass_cal_canceled) {
        play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEU_FEEDBACK);
        return;
    }

    if (AP_Notify::events.initiated_compass_cal) {
        play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEU_FEEDBACK);
        return;
    }

    if (AP_Notify::events.compass_cal_saved) {
        play_tone(AP_NOTIFY_PX4_TONE_QUIET_READY_OR_FINISHED);
        return;
    }

    if (AP_Notify::events.compass_cal_failed) {
        play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEG_FEEDBACK);
        return;
    }

    // don't play other tones if compass cal is running
    if (AP_Notify::flags.compass_cal_running) {
        return;
    }

    // notify the user when autotune or mission completes
    if (AP_Notify::flags.armed && (AP_Notify::events.autotune_complete || AP_Notify::events.mission_complete)) {
        play_tone(AP_NOTIFY_PX4_TONE_LOUD_READY_OR_FINISHED);
    }

    //notify the user when autotune fails
    if (AP_Notify::flags.armed && (AP_Notify::events.autotune_failed)) {
        play_tone(AP_NOTIFY_PX4_TONE_LOUD_NEG_FEEDBACK);
    }

    // notify the user when a waypoint completes
    if (AP_Notify::events.waypoint_complete) {
        play_tone(AP_NOTIFY_PX4_TONE_LOUD_WP_COMPLETE);
    }

    // notify the user when their mode change was successful
    if (AP_Notify::events.user_mode_change) {
        if (AP_Notify::flags.armed) {
            play_tone(AP_NOTIFY_PX4_TONE_LOUD_NEU_FEEDBACK);
        } else {
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEU_FEEDBACK);
        }
    }

    // notify the user when their mode change failed
    if (AP_Notify::events.user_mode_change_failed) {
        if (AP_Notify::flags.armed) {
            play_tone(AP_NOTIFY_PX4_TONE_LOUD_NEG_FEEDBACK);
        } else {
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEG_FEEDBACK);
        }
    }

    // failsafe initiated mode change
    if(AP_Notify::events.failsafe_mode_change) {
        play_tone(AP_NOTIFY_PX4_TONE_LOUD_ATTENTION_NEEDED);
    }

    // notify the user when arming fails
    if (AP_Notify::events.arming_failed) {
        play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEG_FEEDBACK);
    }

    // notify the user when RC contact is lost
    if (flags.failsafe_radio != AP_Notify::flags.failsafe_radio) {
        flags.failsafe_radio = AP_Notify::flags.failsafe_radio;
        if (flags.failsafe_radio) {
            // armed case handled by events.failsafe_mode_change
            if (!AP_Notify::flags.armed) {
                play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEG_FEEDBACK);
            }
        } else {
            if (AP_Notify::flags.armed) {
                play_tone(AP_NOTIFY_PX4_TONE_LOUD_POS_FEEDBACK);
            } else {
                play_tone(AP_NOTIFY_PX4_TONE_QUIET_POS_FEEDBACK);
            }
        }
    }

    // notify the user when pre_arm checks are passing
    if (flags.pre_arm_check != AP_Notify::flags.pre_arm_check) {
        flags.pre_arm_check = AP_Notify::flags.pre_arm_check;
        if (flags.pre_arm_check) {
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_READY_OR_FINISHED);
        }
    }

    // check if arming status has changed
    if (flags.armed != AP_Notify::flags.armed) {
        flags.armed = AP_Notify::flags.armed;
        if (flags.armed) {
            // arming tune
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_ARMING_WARNING);
        }else{
            // disarming tune
            play_tone(AP_NOTIFY_PX4_TONE_QUIET_NEU_FEEDBACK);
            stop_cont_tone();
        }
    }

    // check if battery status has changed
    if (flags.failsafe_battery != AP_Notify::flags.failsafe_battery) {
        flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
        if (flags.failsafe_battery) {
            // battery warning tune
            play_tone(AP_NOTIFY_PX4_TONE_LOUD_BATTERY_ALERT_CTS);
        }
    }

    // check parachute release
    if (flags.parachute_release != AP_Notify::flags.parachute_release) {
        flags.parachute_release = AP_Notify::flags.parachute_release;
        if (flags.parachute_release) {
            // parachute release warning tune
            play_tone(AP_NOTIFY_PX4_TONE_LOUD_ATTENTION_NEEDED);
        }
    }

    // lost vehicle tone
    if (flags.vehicle_lost != AP_Notify::flags.vehicle_lost) {
        flags.vehicle_lost = AP_Notify::flags.vehicle_lost;
        if (flags.vehicle_lost) {
            play_tone(AP_NOTIFY_PX4_TONE_LOUD_VEHICLE_LOST_CTS);
        } else {
            stop_cont_tone();
        }
    }

    // waiting to be thrown vehicle tone
    if (flags.waiting_for_throw != AP_Notify::flags.waiting_for_throw) {
        flags.waiting_for_throw = AP_Notify::flags.waiting_for_throw;
        if (flags.waiting_for_throw) {
            play_tone(AP_NOTIFY_PX4_TONE_WAITING_FOR_THROW);
        } else {
            stop_cont_tone();
        }
    }

    if (AP_Notify::events.tune_started) {
        play_tone(AP_NOTIFY_PX4_TONE_TUNING_START);
        AP_Notify::events.tune_started = 0;        
    }
    if (AP_Notify::events.tune_next) {
        // signify which parameter in the set is starting
        play_tone(AP_NOTIFY_PX4_TONE_LOUD_1 + (AP_Notify::events.tune_next-1));
        AP_Notify::events.tune_next = 0;        
    }
    if (AP_Notify::events.tune_save) {
        play_tone(AP_NOTIFY_PX4_TONE_TUNING_SAVE);
        AP_Notify::events.tune_save = 0;
    }
    if (AP_Notify::events.tune_error) {
        play_tone(AP_NOTIFY_PX4_TONE_TUNING_ERROR);
        AP_Notify::events.tune_error = 0;
    }
}


/*
  handle a PLAY_TUNE message
*/
void ToneAlarm_PX4::handle_play_tune(mavlink_message_t *msg)
{
    // decode mavlink message
    mavlink_play_tune_t packet;
    
    mavlink_msg_play_tune_decode(msg, &packet);

    play_string(packet.tune);
}


#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
