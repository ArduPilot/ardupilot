/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */

#include "AP_OSD.h"

#if OSD_ENABLED

#include "AP_OSD_MAX7456.h"
#ifdef WITH_SITL_OSD
#include "AP_OSD_SITL.h"
#endif
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <utility>
#include <AP_Notify/AP_Notify.h>

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: OSD type
    // @Description: OSD type
    // @Values: 0:None,1:MAX7456
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_OSD, osd_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _CHAN
    // @DisplayName: Screen switch transmitter channel
    // @Description: This sets the channel used to switch different OSD screens.
    // @Values: 0:Disable,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("_CHAN", 2, AP_OSD, rc_channel, 0),

    // @Group: 1_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[0], "1_", 3, AP_OSD, AP_OSD_Screen),

    // @Group: 2_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[1], "2_", 4, AP_OSD, AP_OSD_Screen),

    // @Group: 3_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[2], "3_", 5, AP_OSD, AP_OSD_Screen),

    // @Group: 4_
    // @Path: AP_OSD_Screen.cpp
    AP_SUBGROUPINFO(screen[3], "4_", 6, AP_OSD, AP_OSD_Screen),

    // @Param: _SW_METHOD
    // @DisplayName: Screen switch method
    // @Description: This sets the method used to switch different OSD screens.
    // @Values: 0: switch to next screen if channel value was changed,
    //          1: select screen based on pwm ranges specified for each screen,
    //          2: switch to next screen after low to high transition and every 1s while channel value is high
    // @User: Standard
    AP_GROUPINFO("_SW_METHOD", 7, AP_OSD, sw_method, AP_OSD::TOGGLE),

    // @Param: _OPTIONS
    // @DisplayName: OSD Options
    // @Description: This sets options that change the display
    // @Bitmask: 0:UseDecimalPack, 1:InvertedWindPointer, 2:InvertedAHRoll
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 8, AP_OSD, options, OPTION_DECIMAL_PACK),

    // @Param: _FONT
    // @DisplayName: OSD Font
    // @Description: This sets which OSD font to use. It is an integer from 0 to the number of fonts available
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_FONT", 9, AP_OSD, font_num, 0),

    // @Param: _V_OFFSET
    // @DisplayName: OSD vertical offset
    // @Description: Sets vertical offset of the osd inside image
    // @Range: 0 31
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_V_OFFSET", 10, AP_OSD, v_offset, 16),

    // @Param: _H_OFFSET
    // @DisplayName: OSD horizontal offset
    // @Description: Sets horizontal offset of the osd inside image
    // @Range: 0 63
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_H_OFFSET", 11, AP_OSD, h_offset, 32),

    // @Param: _W_RSSI
    // @DisplayName: RSSI warn level (in %)
    // @Description: Set level at which RSSI item will flash
    // @Range: 0 99
    // @User: Standard
    AP_GROUPINFO("_W_RSSI", 12, AP_OSD, warn_rssi, 30),

    // @Param: _W_NSAT
    // @DisplayName: NSAT warn level
    // @Description: Set level at which NSAT item will flash
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("_W_NSAT", 13, AP_OSD, warn_nsat, 9),

    // @Param: _W_BATVOLT
    // @DisplayName: BAT_VOLT warn level
    // @Description: Set level at which BAT_VOLT item will flash
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_W_BATVOLT", 14, AP_OSD, warn_batvolt, 10.0f),

    // @Param: _UNITS
    // @DisplayName: Display Units
    // @Description: Sets the units to use in displaying items
    // @Values: 0:Metric,1:Imperial,2:SI,3:Aviation
    // @User: Standard
    AP_GROUPINFO("_UNITS", 15, AP_OSD, units, 0),

    // @Param: _MSG_TIME
    // @DisplayName: Message display duration in seconds
    // @Description: Sets message duration seconds
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("_MSG_TIME", 16, AP_OSD, msgtime_s, 10),

    // @Param: _ARM_SCR
    // @DisplayName: Arm screen
    // @Description: Screen to be shown on Arm event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_ARM_SCR", 17, AP_OSD, arm_scr, 0),

    // @Param: _DSARM_SCR
    // @DisplayName: Disarm screen
    // @Description: Screen to be shown on disarm event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_DSARM_SCR", 18, AP_OSD, disarm_scr, 0),

    // @Param: _FS_SCR
    // @DisplayName: Failsafe screen
    // @Description: Screen to be shown on failsafe event. Zero to disable the feature.
    // @Range: 0 4
    // @User: Standard
    AP_GROUPINFO("_FS_SCR", 19, AP_OSD, failsafe_scr, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// singleton instance
AP_OSD *AP_OSD::_singleton;

AP_OSD::AP_OSD()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OSD must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
    // default first screen enabled
    screen[0].enabled = 1;
#ifdef WITH_SITL_OSD
    osd_type.set_default(2);
#endif

#ifdef HAL_OSD_TYPE_DEFAULT
    osd_type.set_default(HAL_OSD_TYPE_DEFAULT);
#endif
    previous_pwm_screen = -1;
    _singleton = this;
}

void AP_OSD::init()
{
    switch ((enum osd_types)osd_type.get()) {
    case OSD_NONE:
    default:
        break;

    case OSD_MAX7456: {
        AP_HAL::OwnPtr<AP_HAL::Device> spi_dev = std::move(hal.spi->get_device("osd"));
        if (!spi_dev) {
            break;
        }
        backend = AP_OSD_MAX7456::probe(*this, std::move(spi_dev));
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started MAX7456 OSD\n");
        break;
    }

#ifdef WITH_SITL_OSD
    case OSD_SITL: {
        backend = AP_OSD_SITL::probe(*this);
        if (backend == nullptr) {
            break;
        }
        hal.console->printf("Started SITL OSD\n");
        break;
    }
#endif
    }
    if (backend != nullptr) {
        // create thread as higher priority than IO
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OSD::osd_thread, void), "OSD", 1024, AP_HAL::Scheduler::PRIORITY_IO, 1);
    }
}

void AP_OSD::osd_thread()
{
    while (true) {
        hal.scheduler->delay(100);
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    backend->clear();

    if (!_disable) {
        stats();
        update_current_screen();

        screen[current_screen].set_backend(backend);
        screen[current_screen].draw();
    }

    backend->flush();
}

//update maximums and totals
void AP_OSD::stats()
{
    uint32_t now = AP_HAL::millis();
    if (!AP_Notify::flags.armed) {
        last_update_ms = now;
        return;
    }

    // flight distance
    uint32_t delta_ms = now - last_update_ms;
    last_update_ms = now;

    AP_AHRS &ahrs = AP::ahrs();
    Vector2f v = ahrs.groundspeed_vector();
    float speed = v.length();
    if (speed < 0.178) {
        speed = 0.0;
    }
    float dist_m = (speed * delta_ms)*0.001;
    last_distance_m += dist_m;

    // maximum ground speed
    max_speed_mps = fmaxf(max_speed_mps,speed);

    // maximum distance
    Location loc;
    if (ahrs.get_position(loc) && ahrs.home_is_set()) {
        const Location &home_loc = ahrs.get_home();
        float distance = home_loc.get_distance(loc);
        max_dist_m = fmaxf(max_dist_m, distance);
    }

    // maximum altitude
    float alt;
    AP::ahrs().get_relative_position_D_home(alt);
    alt = -alt;
    max_alt_m = fmaxf(max_alt_m, alt);
    // maximum current
    AP_BattMonitor &battery = AP::battery();
    float amps;
    if (battery.current_amps(amps)) {
        max_current_a = fmaxf(max_current_a, amps);
    }
}


//Thanks to minimosd authors for the multiple osd screen idea
void AP_OSD::update_current_screen()
{
    // Switch on ARM/DISARM event
    if (AP_Notify::flags.armed) {
        if (!was_armed && arm_scr > 0 && arm_scr <= AP_OSD_NUM_SCREENS && screen[arm_scr-1].enabled) {
            current_screen = arm_scr-1;
        }
        was_armed = true;
    } else if (was_armed) {
        if (disarm_scr > 0 && disarm_scr <= AP_OSD_NUM_SCREENS && screen[disarm_scr-1].enabled) {
            current_screen = disarm_scr-1;
        }
        was_armed = false;
    }

    // Switch on failsafe event
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery) {
        if (!was_failsafe && failsafe_scr > 0 && failsafe_scr <= AP_OSD_NUM_SCREENS && screen[failsafe_scr-1].enabled) {
            pre_fs_screen = current_screen;
            current_screen = failsafe_scr-1;
        }
        was_failsafe = true;
    } else if (was_failsafe) {
        if (screen[pre_fs_screen].enabled) {
            current_screen = pre_fs_screen;
        }
        was_failsafe = false;
    }

    if (rc_channel == 0) {
        return;
    }

    RC_Channel *channel = RC_Channels::rc_channel(rc_channel-1);
    if (channel == nullptr) {
        return;
    }

    int16_t channel_value = channel->get_radio_in();
    switch (sw_method) {
    //switch to next screen if channel value was changed
    default:
    case TOGGLE:
        if (previous_channel_value == 0) {
            //do not switch to the next screen just after initialization
            previous_channel_value = channel_value;
        }
        if (abs(channel_value-previous_channel_value) > 200) {
            if (switch_debouncer) {
                next_screen();
                previous_channel_value = channel_value;
            } else {
                switch_debouncer = true;
                return;
            }
        }
        break;
    //select screen based on pwm ranges specified
    case PWM_RANGE:
        for (int i=0; i<AP_OSD_NUM_SCREENS; i++) {
            if (screen[i].enabled && screen[i].channel_min <= channel_value && screen[i].channel_max > channel_value && previous_pwm_screen != i) {
                current_screen = previous_pwm_screen = i;
                break;
            }
        }
        break;
    //switch to next screen after low to high transition and every 1s while channel value is high
    case AUTO_SWITCH:
        if (channel_value > channel->get_radio_trim()) {
            if (switch_debouncer) {
                uint32_t now = AP_HAL::millis();
                if (now - last_switch_ms > 1000) {
                    next_screen();
                    last_switch_ms = now;
                }
            } else {
                switch_debouncer = true;
                return;
            }
        } else {
            last_switch_ms = 0;
        }
        break;
    }
    switch_debouncer = false;
}

//select next avaliable screen, do nothing if all screens disabled
void AP_OSD::next_screen()
{
    uint8_t t = current_screen;
    do {
        t = (t + 1)%AP_OSD_NUM_SCREENS;
    } while (t != current_screen && !screen[t].enabled);
    current_screen = t;
}

// set navigation information for display
void AP_OSD::set_nav_info(NavInfo &navinfo)
{
    // do this without a lock for now
    nav_info = navinfo;
}

AP_OSD *AP::osd() {
    return AP_OSD::get_singleton();
}

#endif // OSD_ENABLED
