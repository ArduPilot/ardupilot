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
/*
  parameter settings for one screen
 */
#include "AP_OSD.h"
#include "AP_OSD_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>
#include <limits.h>
#include <ctype.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if OSD_PARAM_ENABLED

const AP_Param::GroupInfo AP_OSD_ParamScreen::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable screen
    // @Description: Enable this screen
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OSD_ParamScreen, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CHAN_MIN
    // @DisplayName: Transmitter switch screen minimum pwm
    // @Description: This sets the PWM lower limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_OSD_ParamScreen, channel_min, 900),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter switch screen maximum pwm
    // @Description: This sets the PWM upper limit for this screen
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_OSD_ParamScreen, channel_max, 2100),

    // @Group: PARAM1
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[0], "PARAM1", 4, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM2
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[1], "PARAM2", 5, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM3
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[2], "PARAM3", 6, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM4
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[3], "PARAM4", 7, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM5
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[4], "PARAM5", 8, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM6
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[5], "PARAM6", 9, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM7
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[6], "PARAM7", 10, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM8
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[7], "PARAM8", 11, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Group: PARAM9
    // @Path: AP_OSD_ParamSetting.cpp
    AP_SUBGROUPINFO(params[8], "PARAM9", 12, AP_OSD_ParamScreen, AP_OSD_ParamSetting),

    // @Param: SAVE_X
    // @DisplayName: SAVE_X
    // @Description: Horizontal position of Save button on screen
    // @Range: 0 25
    // @User: Advanced
    AP_GROUPINFO("SAVE_X", 13, AP_OSD_ParamScreen, save_x, 23),

    // @Param: SAVE_Y
    // @DisplayName: SAVE_Y
    // @Description: Vertical position of Save button on screen
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("SAVE_Y", 14, AP_OSD_ParamScreen, save_y, 11),

    AP_GROUPEND
};

#define OSD_HOLD_BUTTON_PRESS_DELAY 100
#define OSD_HOLD_BUTTON_PRESS_COUNT 18

#define OSD_PARAM_DEBUG 0

#if OSD_PARAM_DEBUG
static const char* event_names[5] = {
        "NONE", "MENU_ENTER", "MENU_UP", "MENU_DOWN", "IN_MENU_EXIT"
};
#define debug(fmt, args ...) do { hal.console->printf("OSD: " fmt, args); } while (0)
#else
#define debug(fmt, args ...)
#endif

static const AP_OSD_ParamSetting::Initializer PARAM_DEFAULTS[AP_OSD_NUM_PARAM_SCREENS][AP_OSD_ParamScreen::NUM_PARAMS] {
#if APM_BUILD_COPTER_OR_HELI
    {
        { 1, { 102, 0, 4033 }, OSD_PARAM_NONE },            // ATC_RAT_RLL_P
        { 2, { 102, 0, 129  }, OSD_PARAM_NONE },            // ATC_RAT_RLL_D
        { 3, { 102, 0, 705  }, OSD_PARAM_NONE },            // ATC_RAT_RLL_FLTD
        { 4, { 102, 0, 4034 }, OSD_PARAM_NONE },            // ATC_RAT_PIT_P
        { 5, { 102, 0, 130  }, OSD_PARAM_NONE },            // ATC_RAT_PIT_D
        { 6, { 102, 0, 706  }, OSD_PARAM_NONE },            // ATC_RAT_PIT_FLTD
        { 7, { 102, 0, 4035 }, OSD_PARAM_NONE },            // ATC_RAT_YAW_P
        { 8, { 102, 0, 131  }, OSD_PARAM_NONE },            // ATC_RAT_YAW_D
        { 9, { 102, 0, 643  }, OSD_PARAM_NONE }             // ATC_RAT_YAW_FLTE
    },
    {
        { 1, { 3, 0, 231 }, OSD_PARAM_NONE },               // INS_LOG_BAT_OPT
        { 2, { 3, 0, 167 }, OSD_PARAM_NONE },               // INS_LOG_BAT_MASK
        { 3, { 60, 0, 0  }, OSD_PARAM_NONE },               // LOG_BITMASK
        { 4, { 3, 0, 18  }, OSD_PARAM_NONE },               // INS_GYRO_FILT
        { 5, { 102, 0, 6 }, OSD_PARAM_NONE },               // ATC_THR_MIX_MAN
        { 6, { 102, 0, 5 }, OSD_PARAM_NONE },               // ATC_THR_MIX_MAX
        { 7, { 6, 0, 25041 }, OSD_PARAM_AUX_FUNCTION },     // RC7_OPTION
        { 8, { 6, 0, 25105 }, OSD_PARAM_AUX_FUNCTION },     // RC8_OPTION
        { 9, { 36, 0, 1047 }, OSD_PARAM_FAILSAFE_ACTION_2 } // BATT_FS_LOW_ACT
    }
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    {
        { 1, { 232, 0, 265 }, OSD_PARAM_NONE },             // RLL_RATE_FF
        { 2, { 232, 0, 4041 }, OSD_PARAM_NONE },            // RLL_RATE_P
        { 3, { 232, 0, 73 }, OSD_PARAM_NONE },              // RLL_RATE_I
        { 4, { 233, 0, 267 }, OSD_PARAM_NONE },             // PTCH_RATE_FF
        { 5, { 233, 0, 4043 }, OSD_PARAM_NONE },            // PTCH_RATE_P
        { 6, { 233, 0, 75 }, OSD_PARAM_NONE },              // PTCH_RATE_I
        { 7, { 233, 0, 6 }, OSD_PARAM_NONE },               // PTCH2SRV_RLL
        { 8, { 199, 0, 1 }, OSD_PARAM_NONE },               // TUNE_PARAM
        { 9, { 199, 0, 320 }, OSD_PARAM_NONE }              // TUNE_RANGE
    },
    {
        { 1, { 185, 0, 0 }, OSD_PARAM_NONE },               // TRIM_THROTTLE
        { 2, { 155, 0, 0 }, OSD_PARAM_NONE },               // TRIM_ARSPD_CM
        { 3, { 4, 0, 1094 }, OSD_PARAM_NONE },              // SERVO_AUTO_TRIM
        { 4, { 120, 0, 0 }, OSD_PARAM_NONE},                // ARSPD_FBW_MIN
        { 5, { 121, 0, 0 }, OSD_PARAM_NONE },               // ARSPD_FBW_MAX
        { 6, { 156, 0, 0 }, OSD_PARAM_NONE },               // ALT_HOLD_RTL
        { 7, { 140, 2, 8 }, OSD_PARAM_NONE },               // AHRS_TRIM_Y
        { 8, { 182, 0, 0 }, OSD_PARAM_NONE },               // THR_MAX
        { 9, { 189, 0, 0 }, OSD_PARAM_NONE }                // THR_SLEWRATE
    }
#else
    {
        { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 }, { 8 }, { 9 },
    },
    {
        { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 }, { 8 }, { 9 }
    }
#endif
};

AP_OSD_ParamScreen::AP_OSD_ParamScreen(uint8_t index)
{
    for (uint8_t i = 0; i < NUM_PARAMS; i++) {
        params[i] = PARAM_DEFAULTS[index][i];
    }
    AP_Param::setup_object_defaults(this, var_info);
}

AP_OSD_ParamSetting* AP_OSD_ParamScreen::get_setting(uint8_t param_idx)
{
    if (param_idx >= NUM_PARAMS) {
        return nullptr;
    }
    params[param_idx].update(); // make sure we are fresh
    return &params[param_idx];
}

#if OSD_ENABLED
void AP_OSD_ParamScreen::draw_parameter(uint8_t number, uint8_t x, uint8_t y)
{
    bool param_blink = false;
    bool value_blink = false;
    const bool selected = number == _selected_param;

    switch(_menu_state) {
    case MenuState::PARAM_SELECT:
        param_blink = selected;
        break;
    case MenuState::PARAM_VALUE_MODIFY:
        value_blink = selected;
        break;
    case MenuState::PARAM_PARAM_MODIFY:
        param_blink = value_blink = selected;
        break;
    }

    if (number >= NUM_PARAMS + 1) {
        backend->write(x, y, param_blink, "%s", _requires_save ? "  SAVE" : "REBOOT");
        return;
    }

    AP_OSD_ParamSetting& setting = params[number-1];
    setting.update();

    ap_var_type type = setting._param_type;
    AP_Param* p =  setting._param;
    if (p != nullptr) {
        // grab the name of the parameter
        char name[17];
        setting.copy_name(name, 17);

        const AP_OSD_ParamSetting::ParamMetadata* metadata = setting.get_custom_metadata();

        uint16_t value_pos = 19;
        backend->write(x, y, param_blink, "%s:", name);

        switch (type) {
        case AP_PARAM_INT8: {
            int8_t val = ((AP_Int8*)p)->get();
            if (metadata != nullptr && val >= 0 && val < metadata->values_max) {
                backend->write(value_pos, y, value_blink, "%s", metadata->values[val]);
            } else {
                backend->write(value_pos, y, value_blink, "%hhd", val);
            }
            break;
        }
        case AP_PARAM_INT16: {
            int16_t val = ((AP_Int16*)p)->get();
            if (metadata != nullptr && val >= 0 && val < metadata->values_max) {
                backend->write(value_pos, y, value_blink, "%s", metadata->values[val]);
            } else {
                backend->write(value_pos, y, value_blink, "%hd", val);
            }
            break;
        }
        case AP_PARAM_INT32: {
           int32_t val = ((AP_Int16*)p)->get();
            if (metadata != nullptr && val >= 0 && val < metadata->values_max) {
                backend->write(value_pos, y, value_blink, "%s", metadata->values[val]);
            } else {
                backend->write(value_pos, y, value_blink, "%d", (signed)val);
            }
            break;
        }
        case AP_PARAM_FLOAT: {
            const float val = ((AP_Float*)p)->get();
            // cope with really small value
            if (val < 0.01 && !is_zero(val)) {
                backend->write(value_pos, y, value_blink, "%.4f", val);
            } else if (val < 0.001 && !is_zero(val)) {
                backend->write(value_pos, y, value_blink, "%.5f", val);
            } else {
                backend->write(value_pos, y, value_blink, "%.3f", val);
            }
            break;
        }
        case AP_PARAM_VECTOR3F:
        case AP_PARAM_NONE:
        case AP_PARAM_GROUP:
            break;
        }
    }
}

// modify the selected parameter number
void AP_OSD_ParamScreen::modify_parameter(uint8_t number, Event ev)
{
    if (number > NUM_PARAMS) {
        return;
    }

    const AP_OSD_ParamSetting& setting = params[number-1];
    AP_Param* p = setting._param;

    if (p == nullptr || p->is_read_only()) {
        return;
    }

    _requires_save |= 1 << (number-1);

    const float incr = setting._param_incr * ((ev == Event::MENU_DOWN) ? -1.0f : 1.0f);
    const int32_t incr_int = int32_t(roundf(incr));
    const int32_t max_int = int32_t(roundf(setting._param_max));
    const int32_t min_int = int32_t(roundf(setting._param_min));

    switch (setting._param_type) {
        // there is no way to validate the ranges, so as a rough guess prevent
        // integer types going below -1;
        case AP_PARAM_INT8: {
            AP_Int8* param = (AP_Int8*)p;
            param->set(constrain_int16(param->get() + incr_int, min_int, max_int));
            break;
        }
        case AP_PARAM_INT16: {
            AP_Int16* param = (AP_Int16*)p;
            param->set(constrain_int16(param->get() + incr_int, min_int, max_int));
            break;
        }
        case AP_PARAM_INT32: {
            AP_Int32* param = (AP_Int32*)p;
            param->set(constrain_int32(param->get() + incr_int, min_int, max_int));
            break;
        }
        case AP_PARAM_FLOAT: {
            AP_Float* param = (AP_Float*)p;
            param->set(constrain_float(param->get() + incr, setting._param_min, setting._param_max));
            break;
        }
        case AP_PARAM_VECTOR3F:
        case AP_PARAM_NONE:
        case AP_PARAM_GROUP:
            break;
    }

}

// modify which parameter is configured for the given selection
void AP_OSD_ParamScreen::modify_configured_parameter(uint8_t number, Event ev)
{
    if (number > NUM_PARAMS) {
        return;
    }

    _requires_save |= 1 << (number-1);

    AP_OSD_ParamSetting& setting = params[number-1];
    AP_Param* param;

    if (ev == Event::MENU_DOWN) {
        param = AP_Param::next_scalar(&setting._current_token, &setting._param_type);
    } else {
        // going backwards is somewhat convoluted as the param code is geared for going forward
        ap_var_type type = AP_PARAM_NONE, prev_type = AP_PARAM_NONE, prev_prev_type = AP_PARAM_NONE;
        AP_Param::ParamToken token {}, prev_token, prev_prev_token;

        for (param = AP_Param::first(&token, &type);
            param && (setting._current_token.key != token.key
                || setting._current_token.idx != token.idx
                || setting._current_token.group_element != token.group_element);
            param = AP_Param::next_scalar(&token, &type)) {
            prev_prev_token = prev_token;
            prev_prev_type = prev_type;
            prev_token = token;
            prev_type = type;
        }
        if (param != nullptr) {
            param = AP_Param::next_scalar(&prev_prev_token, &prev_prev_type);
            setting._current_token = prev_prev_token;
            setting._param_type = prev_prev_type;
        }
    }

    if (param != nullptr) {
        // update the stored index
        setting._param_group.set(setting._current_token.group_element);
        setting._param_key.set(AP_Param::get_persistent_key(setting._current_token.key));
        setting._param_idx.set(setting._current_token.idx);
        setting._param = param;
        setting._type.set(OSD_PARAM_NONE);
        // force update() to refresh the token
        setting._current_token.key = 0;
        setting._current_token.idx = 0;
        setting._current_token.group_element = 0;
    }
}

// return radio values as LOW, MIDDLE, HIGH
// this function uses different threshold values to RC_Chanel::get_channel_pos()
// to avoid glitching on the stick travel
RC_Channel::AuxSwitchPos AP_OSD_ParamScreen::get_channel_pos(uint8_t rcmapchan) const
{
    const RC_Channel* chan = rc().channel(rcmapchan-1);
    if (chan == nullptr) {
        return RC_Channel::AuxSwitchPos::LOW;
    }

    const uint16_t in = chan->get_radio_in();
    if (in <= 900 || in >= 2200) {
        return RC_Channel::AuxSwitchPos::LOW;
    }

    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = chan->get_reverse() && rc().option_is_enabled(RC_Channels::Option::ALLOW_SWITCH_REV);

    if (in < RC_Channel::AUX_PWM_TRIGGER_LOW) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
    } else if (in > RC_Channel::AUX_PWM_TRIGGER_HIGH) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::LOW : RC_Channel::AuxSwitchPos::HIGH;
    } else {
        return RC_Channel::AuxSwitchPos::MIDDLE;
    }
}

// map rc input to an event
AP_OSD_ParamScreen::Event AP_OSD_ParamScreen::map_rc_input_to_event() const
{
    const RC_Channel::AuxSwitchPos throttle = get_channel_pos(AP::rcmap()->throttle());
    const RC_Channel::AuxSwitchPos yaw = get_channel_pos(AP::rcmap()->yaw());
    const RC_Channel::AuxSwitchPos roll = get_channel_pos(AP::rcmap()->roll());
    const RC_Channel::AuxSwitchPos pitch = get_channel_pos(AP::rcmap()->pitch());

    Event result = Event::NONE;

    if (yaw != RC_Channel::AuxSwitchPos::MIDDLE || throttle != RC_Channel::AuxSwitchPos::LOW) {
        return result;
    }

    if (pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::LOW) {
        result = Event::MENU_EXIT;
    } else if (pitch == RC_Channel::AuxSwitchPos::MIDDLE && roll == RC_Channel::AuxSwitchPos::HIGH) {
        result = Event::MENU_ENTER;
    } else if (pitch == RC_Channel::AuxSwitchPos::LOW && roll == RC_Channel::AuxSwitchPos::MIDDLE) {
        result = Event::MENU_UP;
    } else if (pitch == RC_Channel::AuxSwitchPos::HIGH && roll == RC_Channel::AuxSwitchPos::MIDDLE) {
        result = Event::MENU_DOWN;
    } else {
        // OSD option has not changed so assume stick re-centering
        result = Event::NONE;
    }
    return result;
}

// update the state machine when disarmed
void AP_OSD_ParamScreen::update_state_machine()
{
    const uint32_t now = AP_HAL::millis();
    if ((now - _transition_start_ms) < _transition_timeout_ms) {
        return;
    }

    const Event ev = map_rc_input_to_event();
    // only take action on transitions
    if (ev == Event::NONE && ev == _last_rc_event) {
        return;
    }

    debug("update_state_machine(%s)\n", event_names[int(ev)]);

    _transition_start_ms = now;
    if (ev == _last_rc_event) {
        _transition_timeout_ms = OSD_HOLD_BUTTON_PRESS_DELAY;
        _transition_count++;
    } else {
        _transition_timeout_ms = osd->button_delay_ms;
        _transition_count = 0;
    }
    _last_rc_event = ev;

    // if we were armed then there is no selected parameter - so find one
    if (_selected_param == 0) {
        _selected_param = 1;
        for (uint8_t i = 0; i < NUM_PARAMS && !params[_selected_param-1].enabled; i++) {
            _selected_param++;
        }
    }

    switch (ev) {
    case Event::MENU_ENTER:
        switch(_menu_state) {
        case MenuState::PARAM_SELECT:
            if (_selected_param == SAVE_PARAM) {
                if (_transition_count >= OSD_HOLD_BUTTON_PRESS_COUNT) {
                    save_parameters();
                    hal.scheduler->reboot(false);
                } else {
                    save_parameters();
                }
            } else {
                _menu_state = MenuState::PARAM_VALUE_MODIFY;
            }
            break;
        case MenuState::PARAM_VALUE_MODIFY:
            if (_transition_count >= OSD_HOLD_BUTTON_PRESS_COUNT) {
                _menu_state = MenuState::PARAM_PARAM_MODIFY;
            }
            break;
        case MenuState::PARAM_PARAM_MODIFY:
            break;
        }
        break;
    case Event::MENU_UP:
        switch (_menu_state) {
        case MenuState::PARAM_SELECT:
            _selected_param--;
            if (_selected_param < 1) {
                _selected_param = SAVE_PARAM;
            }
            // skip over parameters that are not enabled
            for (uint8_t i = 0; i < NUM_PARAMS + 1 && (_selected_param != SAVE_PARAM && !params[_selected_param-1].enabled); i++) {
                _selected_param--;
                if (_selected_param < 1) {
                    _selected_param = SAVE_PARAM;
                }
            }
            // repeat at the standard rate
            _transition_timeout_ms = osd->button_delay_ms;
            break;
        case MenuState::PARAM_VALUE_MODIFY:
            modify_parameter(_selected_param, ev);
            break;
        case MenuState::PARAM_PARAM_MODIFY:
            modify_configured_parameter(_selected_param, ev);
            break;
        }
        break;
    case Event::MENU_DOWN:
        switch (_menu_state) {
        case MenuState::PARAM_SELECT:
            _selected_param++;
            if (_selected_param > SAVE_PARAM) {
                _selected_param = 1;
            }
            // skip over parameters that are not enabled
            for (uint8_t i = 0; i < NUM_PARAMS + 1 && (_selected_param != SAVE_PARAM && !params[_selected_param-1].enabled); i++) {
                _selected_param++;
                if (_selected_param > SAVE_PARAM) {
                    _selected_param = 1;
                }
            }
            // repeat at the standard rate
            _transition_timeout_ms = osd->button_delay_ms;
            break;
        case MenuState::PARAM_VALUE_MODIFY:
            modify_parameter(_selected_param, ev);
            break;
        case MenuState::PARAM_PARAM_MODIFY:
            modify_configured_parameter(_selected_param, ev);
            break;
        }
        break;
    case Event::MENU_EXIT:
        switch(_menu_state) {
        case MenuState::PARAM_SELECT:
            break;
        case MenuState::PARAM_VALUE_MODIFY:
            _menu_state = MenuState::PARAM_SELECT;
            break;
        case MenuState::PARAM_PARAM_MODIFY:
            _menu_state = MenuState::PARAM_VALUE_MODIFY;
            break;
        }
        break;
    case Event::NONE:
        break;
    }
}

#if HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT
void AP_OSD_ParamScreen::draw(void)
{
    if (!enabled || !backend) {
        return;
    }

    // first update the state machine
    if (!AP::arming().is_armed()) {
        update_state_machine();
    } else {
        _selected_param = 0;
    }

    for (uint8_t i = 0; i < NUM_PARAMS; i++) {
        AP_OSD_ParamSetting n = params[i];
        if (n.enabled) {
            draw_parameter(n._param_number, n.xpos, n.ypos);
        }
    }
    // the save button
    draw_parameter(SAVE_PARAM, save_x, save_y);
}
#endif

#endif // OSD_ENABLED

// save all of the parameters
void AP_OSD_ParamScreen::save_parameters()
{
    if (!_requires_save) {
        return;
    }

    for (uint8_t i = 0; i < NUM_PARAMS; i++) {
        if (params[i].enabled && (_requires_save & (1 << i))) {
            AP_Param* p = params[i]._param;
            if (p != nullptr) {
                p->save();
            }
            params[i].save_as_new();
        }
    }
    _requires_save = 0;
}

// handle OSD configuration messages
#if HAL_GCS_ENABLED
void AP_OSD_ParamScreen::handle_write_msg(const mavlink_osd_param_config_t& packet, const class GCS_MAVLINK& link)
{
    // request out of range - return an error
    if (packet.osd_index < 1 || packet.osd_index > AP_OSD_ParamScreen::NUM_PARAMS) {
        mavlink_msg_osd_param_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_PARAMETER_INDEX);
        return;
    }
    // set the parameter
    bool ret = params[packet.osd_index - 1].set_by_name(packet.param_id, packet.config_type, packet.min_value, packet.max_value, packet.increment);
    mavlink_msg_osd_param_config_reply_send(link.get_chan(), packet.request_id, ret ? OSD_PARAM_SUCCESS : OSD_PARAM_INVALID_PARAMETER);
}

// handle OSD show configuration messages
void AP_OSD_ParamScreen::handle_read_msg(const mavlink_osd_param_show_config_t& packet, const class GCS_MAVLINK& link)
{
    // request out of range - return an error
    if (packet.osd_index < 1 || packet.osd_index > AP_OSD_ParamScreen::NUM_PARAMS) {
        mavlink_msg_osd_param_show_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_PARAMETER_INDEX,
            nullptr, OSD_PARAM_NONE, 0, 0, 0);
        return;
    }
    // get the parameter and make sure it is fresh
    AP_OSD_ParamSetting& param = params[packet.osd_index - 1];
    param.update();

    // check for bad things
    if (param._param == nullptr) {
        mavlink_msg_osd_param_show_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_INVALID_PARAMETER_INDEX,
            nullptr, OSD_PARAM_NONE, 0, 0, 0);
        return;
    }
    // get the name and send back the details
    char buf[AP_MAX_NAME_SIZE+1];
    param._param->copy_name_token(param._current_token, buf, AP_MAX_NAME_SIZE);
    buf[AP_MAX_NAME_SIZE] = 0;
    mavlink_msg_osd_param_show_config_reply_send(link.get_chan(), packet.request_id, OSD_PARAM_SUCCESS,
        buf, param._type, param._param_min, param._param_max, param._param_incr);
}
#endif

#endif // OSD_PARAM_ENABLED
