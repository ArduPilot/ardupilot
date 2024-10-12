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

#if OSD_ENABLED || OSD_PARAM_ENABLED

#include "AP_OSD_MAX7456.h"
#ifdef WITH_SITL_OSD
#include "AP_OSD_SITL.h"
#endif
#include "AP_OSD_MSP.h"
#include "AP_OSD_MSP_DisplayPort.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <utility>
#include <AP_Notify/AP_Notify.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Camera/AP_RunCam.h>

// macro for easy use of var_info2
#define AP_SUBGROUPINFO2(element, name, idx, thisclazz, elclazz) { name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info2 }, AP_PARAM_FLAG_NESTED_OFFSET, idx, AP_PARAM_GROUP }

const AP_Param::GroupInfo AP_OSD::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: OSD type
    // @Description: OSD type. TXONLY makes the OSD parameter selection available to other modules even if there is no native OSD support on the board, for instance CRSF.
    // @Values: 0:None,1:MAX7456,2:SITL,3:MSP,4:TXONLY,5:MSP_DISPLAYPORT
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_OSD, osd_type, 0, AP_PARAM_FLAG_ENABLE),

#if OSD_ENABLED
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
    // @Values: 0: switch to next screen if channel value was changed, 1: select screen based on pwm ranges specified for each screen, 2: switch to next screen after low to high transition and every 1s while channel value is high, 3: switches to next screen if the sticks in the next position: roll - middle, pitch - high, throttle - middle, yaw - left. Keeps toggling to next screen every 1s while sticks in mentioned positions
    // @User: Standard
    AP_GROUPINFO("_SW_METHOD", 7, AP_OSD, sw_method, AP_OSD::TOGGLE),

    // @Param: _OPTIONS
    // @DisplayName: OSD Options
    // @Description: This sets options that change the display
    // @Bitmask: 0:UseDecimalPack, 1:InvertedWindArrow, 2:InvertedAHRoll, 3:Convert feet to miles at 5280ft instead of 10000ft, 4:DisableCrosshair, 5:TranslateArrows, 6:AviationStyleAH, 7:Prefix LQ with RF Mode
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
    // @Description: Set level at which RSSI item will flash (in positive % or negative dBm values as applicable). 30% or -100dBm are defaults.
    // @Range: -128 100
    // @User: Standard
    AP_GROUPINFO("_W_RSSI", 12, AP_OSD, warn_rssi, AP_OSD_WARN_RSSI_DEFAULT),

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

#if OSD_PARAM_ENABLED
    // @Param: _BTN_DELAY
    // @DisplayName: Button delay
    // @Description: Debounce time in ms for stick commanded parameter navigation.
    // @Range: 0 3000
    // @User: Advanced
    AP_GROUPINFO("_BTN_DELAY", 20, AP_OSD, button_delay_ms, 300),
#endif
#if AP_TERRAIN_AVAILABLE
    // @Param: _W_TERR
    // @DisplayName: Terrain warn level
    // @Description: Set level below which TER_HGT item will flash. -1 disables.
    // @Range: -1 3000
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_W_TERR", 23, AP_OSD, warn_terr, -1),
#endif

    // @Param: _W_AVGCELLV
    // @DisplayName: AVGCELLV warn level
    // @Description: Set level at which AVGCELLV item will flash
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_W_AVGCELLV", 24, AP_OSD, warn_avgcellvolt, 3.6f),

   // @Param: _CELL_COUNT
    // @DisplayName: Battery cell count
    // @Description: Used for average cell voltage display. -1 disables, 0 uses cell count autodetection for well charged LIPO/LIION batteries at connection, other values manually select cell count used.
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_CELL_COUNT", 25, AP_OSD, cell_count, -1),

    // @Param: _W_RESTVOLT
    // @DisplayName: RESTVOLT warn level
    // @Description: Set level at which RESTVOLT item will flash
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_W_RESTVOLT", 26, AP_OSD, warn_restvolt, 10.0f),
       
    // @Param: _W_ACRVOLT
    // @DisplayName: Avg Cell Resting Volt warn level
    // @Description: Set level at which ACRVOLT item will flash
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_W_ACRVOLT", 31, AP_OSD, warn_avgcellrestvolt, 3.6f),

#if AP_OSD_EXTENDED_LNK_STATS
    // @Param: _W_LQ
    // @DisplayName: RC link quality warn level (in %)
    // @Description: Set level at which RC_LQ item will flash (%)
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_W_LQ", 33, AP_OSD, warn_lq, 50),

    // @Param: _W_SNR
    // @DisplayName: RC link SNR warn level (in %)
    // @Description: Set level at which RC_SNR item will flash (in db)
    // @Range: -20 10
    // @User: Standard
    AP_GROUPINFO("_W_SNR", 34, AP_OSD, warn_snr, 0),
#endif

#if HAL_OSD_SIDEBAR_ENABLE
    // @Param: _SB_H_OFS
    // @DisplayName: Sidebar horizontal offset
    // @Description: Extends the spacing between the sidebar elements by this amount of columns. Positive values increases the width to the right of the screen.
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("_SB_H_OFS", 35, AP_OSD, sidebar_h_offset, 0),

    // @Param: _SB_V_EXT
    // @DisplayName: Sidebar vertical extension
    // @Description: Increase of vertical length of the sidebar itens by this amount of lines. Applied equally both above and below the default setting.
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("_SB_V_EXT", 36, AP_OSD, sidebar_v_ext, 0),
#endif // HAL_OSD_SIDEBAR_ENABLE

#endif //osd enabled
#if OSD_PARAM_ENABLED
    // @Group: 5_
    // @Path: AP_OSD_ParamScreen.cpp
    AP_SUBGROUPINFO(param_screen[0], "5_", 21, AP_OSD, AP_OSD_ParamScreen),

    // @Group: 6_
    // @Path: AP_OSD_ParamScreen.cpp
    AP_SUBGROUPINFO(param_screen[1], "6_", 22, AP_OSD, AP_OSD_ParamScreen),
#endif

#if OSD_ENABLED
    // additional tables to go beyond 63 limit
    AP_SUBGROUPINFO2(screen[0], "1_", 27, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[1], "2_", 28, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[2], "3_", 29, AP_OSD, AP_OSD_Screen),
    AP_SUBGROUPINFO2(screen[3], "4_", 30, AP_OSD, AP_OSD_Screen),
#endif

    // @Param: _TYPE2
    // @DisplayName: OSD type 2
    // @Description: OSD type 2. TXONLY makes the OSD parameter selection available to other modules even if there is no native OSD support on the board, for instance CRSF.
    // @Values: 0:None,1:MAX7456,2:SITL,3:MSP,4:TXONLY,5:MSP_DISPLAYPORT
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_TYPE2", 32, AP_OSD, osd_type2, 0),

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
#if OSD_ENABLED
    // force first screen enabled
    screen[0].enabled.set_and_default(1);
    previous_pwm_screen = -1;
#endif
#ifdef WITH_SITL_OSD
    osd_type.set_default(OSD_SITL);
#endif

#ifdef HAL_OSD_TYPE_DEFAULT
    osd_type.set_default(HAL_OSD_TYPE_DEFAULT);
#endif
    _singleton = this;
}

void AP_OSD::init()
{
    const AP_OSD::osd_types types[OSD_MAX_INSTANCES] = {
        osd_types(osd_type.get()),
        osd_types(osd_type2.get())
    };
    for (uint8_t instance = 0; instance < OSD_MAX_INSTANCES; instance++) {
        if (init_backend(types[instance], instance)) {
            _backend_count++;
        }
    }
    if (_backend_count > 0) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OSD::osd_thread, void), "OSD", 1280, AP_HAL::Scheduler::PRIORITY_IO, 1);
    }
}

bool AP_OSD::init_backend(const AP_OSD::osd_types type, const uint8_t instance)
{
    // check if we can run this backend instance in parallel with backend instance 0
    if (instance > 0) {
        if (_backends[0] && !_backends[0]->is_compatible_with_backend_type(type)) {
            return false;
        }
    }
    switch (type) {
    case OSD_NONE:
    case OSD_TXONLY:
    default:
        break;

    case OSD_MAX7456: {
#ifdef HAL_WITH_SPI_OSD
        AP_HAL::OwnPtr<AP_HAL::Device> spi_dev = std::move(hal.spi->get_device("osd"));
        if (!spi_dev) {
            break;
        }
#if HAL_WITH_OSD_BITMAP
        _backends[instance] = AP_OSD_MAX7456::probe(*this, std::move(spi_dev));
#endif
        if (_backends[instance] == nullptr) {
            break;
        }
        DEV_PRINTF("Started MAX7456 OSD\n");
#endif
        break;
    }

#ifdef WITH_SITL_OSD
    case OSD_SITL: {
        _backends[instance] = AP_OSD_SITL::probe(*this);
        if (_backends[instance] == nullptr) {
            break;
        }
        DEV_PRINTF("Started SITL OSD\n");
        break;
    }
#endif
    case OSD_MSP: {
        _backends[instance] = AP_OSD_MSP::probe(*this);
        if (_backends[instance] == nullptr) {
            break;
        }
        DEV_PRINTF("Started MSP OSD\n");
        break;
    }
#if HAL_WITH_MSP_DISPLAYPORT
    case OSD_MSP_DISPLAYPORT: {
        _backends[instance] = AP_OSD_MSP_DisplayPort::probe(*this);
        if (_backends[instance] == nullptr) {
            break;
        }
        DEV_PRINTF("Started MSP DisplayPort OSD\n");
        break;
    }
#endif
    }
#if OSD_ENABLED
    if (_backends[instance] != nullptr) {
        // populate the fonts lookup table
        _backends[instance]->init_symbol_set(AP_OSD_AbstractScreen::symbols_lookup_table, AP_OSD_NUM_SYMBOLS);
        return true;
    }
#endif
    return false;
}

#if OSD_ENABLED
void AP_OSD::osd_thread()
{
    // initialize thread specific code once
    for (uint8_t instance = 0; instance < _backend_count; instance++) {
        _backends[instance]->osd_thread_run_once();
    }


    while (true) {
        hal.scheduler->delay(100);
        if (!_disable) {
            update_stats();
            update_current_screen();
        }
        update_osd();
    }
}

void AP_OSD::update_osd()
{
    for (uint8_t instance = 0; instance < _backend_count; instance++) {
        _backends[instance]->clear();

        if (!_disable) {
            get_screen(current_screen).set_backend(_backends[instance]);
            // skip drawing for MSP OSD backends to save some resources
            if (_backends[instance]->get_backend_type() != OSD_MSP) {
                get_screen(current_screen).draw();
            }
        }

        _backends[instance]->flush();
    }
}

//update maximums and totals
void AP_OSD::update_stats()
{
    // allow other threads to consume stats info
    WITH_SEMAPHORE(_sem);

    uint32_t now = AP_HAL::millis();
    if (!AP_Notify::flags.armed) {
        _stats.last_update_ms = now;
        return;
    }

    // flight distance
    uint32_t delta_ms = now - _stats.last_update_ms;
    _stats.last_update_ms = now;

    Vector2f v;
    Location loc {};
    Location home_loc;
    bool home_is_set;
    bool have_airspeed_estimate;
    float alt;
    float aspd_mps = 0.0f;
    {
        // minimize semaphore scope
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        v = ahrs.groundspeed_vector();
        home_is_set = ahrs.get_location(loc) && ahrs.home_is_set();
        if (home_is_set) {
            home_loc = ahrs.get_home();
        }
        ahrs.get_relative_position_D_home(alt);
        have_airspeed_estimate = ahrs.airspeed_estimate(aspd_mps);
    }
    float speed = v.length();
    if (speed < 0.178) {
        speed = 0.0;
    }
    float dist_m = (speed * delta_ms)*0.001;
    _stats.last_distance_m += dist_m;

    // maximum ground speed
    _stats.max_speed_mps = fmaxf(_stats.max_speed_mps,speed);

    // maximum distance
    if (home_is_set) {
        float distance = home_loc.get_distance(loc);
        _stats.max_dist_m = fmaxf(_stats.max_dist_m, distance);
    }

    // maximum altitude
    alt = -alt;
    _stats.max_alt_m = fmaxf(_stats.max_alt_m, alt);
#if AP_BATTERY_ENABLED
    // maximum current
    AP_BattMonitor &battery = AP::battery();
    float amps;
    if (battery.current_amps(amps)) {
        _stats.max_current_a = fmaxf(_stats.max_current_a, amps);
    }
    // minimum voltage
    float voltage = battery.voltage();
    if (voltage > 0) {
        _stats.min_voltage_v = fminf(_stats.min_voltage_v, voltage);
    }
#endif
#if AP_RSSI_ENABLED
    // minimum rssi
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        _stats.min_rssi = fminf(_stats.min_rssi, ap_rssi->read_receiver_rssi());
    }
#endif
    // max airspeed either true or synthetic
    if (have_airspeed_estimate) {
        _stats.max_airspeed_mps = fmaxf(_stats.max_airspeed_mps, aspd_mps);
    }
#if HAL_WITH_ESC_TELEM
    // max esc temp
    AP_ESC_Telem& telem = AP::esc_telem();
    int16_t highest_temperature = 0;
    telem.get_highest_temperature(highest_temperature);
    _stats.max_esc_temp = MAX(_stats.max_esc_temp, highest_temperature);
#endif
}

//Thanks to minimosd authors for the multiple osd screen idea
void AP_OSD::update_current_screen()
{
    // Switch on ARM/DISARM event
    if (AP_Notify::flags.armed) {
        if (!was_armed && arm_scr > 0 && arm_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(arm_scr-1).enabled) {
            current_screen = arm_scr-1;
        }
        was_armed = true;
    } else if (was_armed) {
        if (disarm_scr > 0 && disarm_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(disarm_scr-1).enabled) {
            current_screen = disarm_scr-1;
        }
        was_armed = false;
    }

    // Switch on failsafe event
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery) {
        if (!was_failsafe && failsafe_scr > 0 && failsafe_scr <= AP_OSD_NUM_DISPLAY_SCREENS && get_screen(failsafe_scr-1).enabled) {
            pre_fs_screen = current_screen;
            current_screen = failsafe_scr-1;
        }
        was_failsafe = true;
    } else if (was_failsafe) {
        if (get_screen(pre_fs_screen).enabled) {
            current_screen = pre_fs_screen;
        }
        was_failsafe = false;
    }

    if (rc_channel == 0 && sw_method != STICKS_INPUT) {
        return;
    }

#if AP_RC_CHANNEL_ENABLED
    int16_t channel_value = 0;
    RC_Channel *channel = nullptr;

    if (sw_method != STICKS_INPUT) {
        channel = RC_Channels::rc_channel(rc_channel-1);
        if (channel == nullptr) {
            return;
        }

        channel_value = channel->get_radio_in();
    }

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
            if (get_screen(i).enabled && get_screen(i).channel_min <= channel_value && get_screen(i).channel_max > channel_value) {
                if (previous_pwm_screen == i) {
                    break;
                } else {
                current_screen = previous_pwm_screen = i;
                }
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
    case STICKS_INPUT:
        if (!AP_Notify::flags.armed && !AP::runcam()->in_menu()) {
            auto &rc_channels = rc();
            auto &roll = rc_channels.get_roll_channel();
            auto &pitch = rc_channels.get_pitch_channel();
            auto &yaw = rc_channels.get_yaw_channel();
            auto &throttle = rc_channels.get_throttle_channel();
            if (roll.get_aux_switch_pos() == RC_Channel::AuxSwitchPos::MIDDLE &&
                pitch.get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH &&
                yaw.get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW &&
                throttle.get_aux_switch_pos() == RC_Channel::AuxSwitchPos::MIDDLE) {
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
        }
        break;
    }
    switch_debouncer = false;
#endif  // AP_RC_CHANNEL_ENABLED
}

//select next avaliable screen, do nothing if all screens disabled
void AP_OSD::next_screen()
{
    uint8_t t = current_screen;
    do {
        t = (t + 1)%AP_OSD_NUM_SCREENS;
    } while (t != current_screen && !get_screen(t).enabled);
    current_screen = t;
}

// set navigation information for display
void AP_OSD::set_nav_info(NavInfo &navinfo)
{
    // do this without a lock for now
    nav_info = navinfo;
}

// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool AP_OSD::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
#if OSD_PARAM_ENABLED
    // currently in the OSD menu, do not allow arming
    if (!is_readonly_screen()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "In OSD menu");
        return false;
    }
#endif  

    //check if second backend was requested by user but not instantiated
    if (osd_type.get() != OSD_NONE && _backend_count == 1 && osd_type2.get() != OSD_NONE) {
        hal.util->snprintf(failure_msg, failure_msg_len, "OSD_TYPE2 not compatible with first OSD");
        return false; 
    }

    // if we got this far everything must be ok
    return true;
}

#endif // OSD_ENABLED

// handle OSD parameter configuration
#if HAL_GCS_ENABLED
void AP_OSD::handle_msg(const mavlink_message_t &msg, const GCS_MAVLINK& link)
{
    bool found = false;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_OSD_PARAM_CONFIG: {
        mavlink_osd_param_config_t packet;
        mavlink_msg_osd_param_config_decode(&msg, &packet);
#if OSD_PARAM_ENABLED
        for (uint8_t i = 0; i < AP_OSD_NUM_PARAM_SCREENS; i++) {
            if (packet.osd_screen == i + AP_OSD_NUM_DISPLAY_SCREENS + 1) {
                param_screen[i].handle_write_msg(packet, link);
                found = true;
            }
        }
#endif
        // send back an error
        if (!found) {
            mavlink_msg_osd_param_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_SCREEN);
        }
    }
        break;
    case MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG: {
        mavlink_osd_param_show_config_t packet;
        mavlink_msg_osd_param_show_config_decode(&msg, &packet);
#if OSD_PARAM_ENABLED
        for (uint8_t i = 0; i < AP_OSD_NUM_PARAM_SCREENS; i++) {
            if (packet.osd_screen == i + AP_OSD_NUM_DISPLAY_SCREENS + 1) {
                param_screen[i].handle_read_msg(packet, link);
                found = true;
            }
        }
#endif
        // send back an error
        if (!found) {
            mavlink_msg_osd_param_show_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_SCREEN,
                nullptr, OSD_PARAM_NONE, 0, 0, 0);
        }
    }
        break;
    default:
        break;
    }
}
#endif

AP_OSD *AP::osd() {
    return AP_OSD::get_singleton();
}

#endif // OSD_ENABLED || OSD_PARAM_ENABLED
