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
 *       RC_Channel.cpp - class for one RC channel input
 */

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"
#include <GCS_MAVLink/GCS.h>


#include <AP_BoardConfig/AP_BoardConfig.h>

#include <AP_VideoTX/AP_VideoTX.h>

#define SWITCH_DEBOUNCE_TIME_MS  200

const AP_Param::GroupInfo RC_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  1, RC_Channel, radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 2, RC_Channel, radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  3, RC_Channel, radio_max, 1900),

    // @Param: REVERSED
    // @DisplayName: RC reversed
    // @Description: Reverse channel input. Set to 0 for normal operation. Set to 1 to reverse this input channel.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    AP_GROUPINFO("REVERSED",  4, RC_Channel, reversed, 0),

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: PWM dead zone in microseconds around trim or bottom
    // @Units: PWM
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("DZ",   5, RC_Channel, dead_zone, 0),

    // @Param: OPTION
    // @DisplayName: RC input option
    // @Description: Function assigned to this RC channel
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 94:VTX Power, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 94:VTX Power, 97:Windvane home heading direction offset, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 106:Disable Airspeed Use, 201:Roll, 202:Pitch, 203:Walking Height, 207:MainSail, 208:Flap, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 11:Fence, 16:ModeAuto, 24:Auto Mission Reset, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 52: ModeACRO, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 94:VTX Power, 95:FBWA taildragger takeoff mode, 96:trigger re-reading of mode switch, 98: ModeTraining, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 106:Disable Airspeed Use, 208:Flap, 209: Forward Throttle, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Blimp}: 0:Do Nothing, 18:Land, 41:ArmDisarm, 46:RC Override Enable, 65:GPS Disable, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2
    // @User: Standard
    AP_GROUPINFO_FRAME("OPTION",  6, RC_Channel, option, 0, AP_PARAM_FRAME_COPTER|AP_PARAM_FRAME_ROVER|AP_PARAM_FRAME_PLANE|AP_PARAM_FRAME_BLIMP),

    AP_GROUPEND
};


// constructor
RC_Channel::RC_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void RC_Channel::set_range(uint16_t high)
{
    type_in = RC_CHANNEL_TYPE_RANGE;
    high_in = high;
}

void RC_Channel::set_angle(uint16_t angle)
{
    type_in = RC_CHANNEL_TYPE_ANGLE;
    high_in = angle;
}

void RC_Channel::set_default_dead_zone(int16_t dzone)
{
    dead_zone.set_default(abs(dzone));
}

bool RC_Channel::get_reverse(void) const
{
    return bool(reversed.get());
}

// read input from hal.rcin or overrides
bool RC_Channel::update(void)
{
    if (has_override() && !rc().ignore_overrides()) {
        radio_in = override_value;
    } else if (rc().has_had_rc_receiver() && !rc().ignore_receiver()) {
        radio_in = hal.rcin->read(ch_in);
    } else {
        return false;
    }

    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range();
    } else {
        //RC_CHANNEL_TYPE_ANGLE
        control_in = pwm_to_angle();
    }

    return true;
}

// recompute control values with no deadzone
// When done this way the control_in value can be used as servo_out
// to give the same output as input
void RC_Channel::recompute_pwm_no_deadzone()
{
    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range_dz(0);
    } else {
        //RC_CHANNEL_ANGLE
        control_in = pwm_to_angle_dz(0);
    }
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t RC_Channel::get_control_mid() const
{
    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        int16_t r_in = (radio_min.get() + radio_max.get())/2;

        if (reversed) {
            r_in = radio_max.get() - (r_in - radio_min.get());
        }

        int16_t radio_trim_low  = radio_min + dead_zone;

        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t RC_Channel::pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim) const
{
    int16_t radio_trim_high = _trim + _dead_zone;
    int16_t radio_trim_low  = _trim - _dead_zone;

    int16_t reverse_mul = (reversed?-1:1);

    // don't allow out of range values
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (r_in > radio_trim_high && radio_max != radio_trim_high) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    } else if (r_in < radio_trim_low && radio_trim_low != radio_min) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t RC_Channel::pwm_to_angle_dz(uint16_t _dead_zone) const
{
    return pwm_to_angle_dz_trim(_dead_zone, radio_trim);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t RC_Channel::pwm_to_angle() const
{
	return pwm_to_angle_dz(dead_zone);
}


/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t RC_Channel::pwm_to_range_dz(uint16_t _dead_zone) const
{
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (reversed) {
	    r_in = radio_max.get() - (r_in - radio_min.get());
    }

    int16_t radio_trim_low  = radio_min + _dead_zone;

    if (r_in > radio_trim_low) {
        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    }
    return 0;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t RC_Channel::pwm_to_range() const
{
    return pwm_to_range_dz(dead_zone);
}


int16_t RC_Channel::get_control_in_zero_dz(void) const
{
    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        return pwm_to_range_dz(0);
    }
    return pwm_to_angle_dz(0);
}

// ------------------------------------------

float RC_Channel::norm_input() const
{
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < radio_trim) {
        if (radio_min >= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        if (radio_max <= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

float RC_Channel::norm_input_dz() const
{
    int16_t dz_min = radio_trim - dead_zone;
    int16_t dz_max = radio_trim + dead_zone;
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

// return a normalised input for a channel, in range -1 to 1,
// ignores trim and deadzone
float RC_Channel::norm_input_ignore_trim() const
{
    // sanity check min and max to avoid divide by zero
    if (radio_max <= radio_min) {
        return 0.0f;
    }
    const float ret = (reversed ? -2.0f : 2.0f) * (((float)(radio_in - radio_min) / (float)(radio_max - radio_min)) - 0.5f);
    return constrain_float(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t RC_Channel::percent_input() const
{
    if (radio_in <= radio_min) {
        return reversed?100:0;
    }
    if (radio_in >= radio_max) {
        return reversed?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (reversed) {
        ret = 100 - ret;
    }
    return ret;
}

/*
  return true if input is within deadzone of trim
*/
bool RC_Channel::in_trim_dz() const
{
    return is_bounded_int32(radio_in, radio_trim - dead_zone, radio_trim + dead_zone);
}

void RC_Channel::set_override(const uint16_t v, const uint32_t timestamp_ms)
{
    if (!rc().gcs_overrides_enabled()) {
        return;
    }

    last_override_time = timestamp_ms != 0 ? timestamp_ms : AP_HAL::millis();
    override_value = v;
    rc().new_override_received();
}

void RC_Channel::clear_override()
{
    last_override_time = 0;
    override_value = 0;
}

bool RC_Channel::has_override() const
{
    if (override_value == 0) {
        return false;
    }

    uint32_t override_timeout_ms;
    if (!rc().get_override_timeout_ms(override_timeout_ms)) {
        // timeouts are disabled
        return true;
    }

    if (override_timeout_ms == 0) {
        // overrides are explicitly disabled by a zero value
        return false;
    }

    return (AP_HAL::millis() - last_override_time < override_timeout_ms);
}

/*
  perform stick mixing on one channel
  This type of stick mixing reduces the influence of the auto
  controller as it increases the influence of the users stick input,
  allowing the user full deflection if needed
 */
int16_t RC_Channel::stick_mixing(const int16_t servo_in)
{
    float ch_inf = (float)(radio_in - radio_trim);
    ch_inf = fabsf(ch_inf);
    ch_inf = MIN(ch_inf, 400.0f);
    ch_inf = ((400.0f - ch_inf) / 400.0f);

    int16_t servo_out = servo_in;
    servo_out *= ch_inf;
    servo_out += control_in;

    return servo_out;
}

//
// support for auxillary switches:
//

void RC_Channel::reset_mode_switch()
{
    switch_state.current_position = -1;
    switch_state.debounce_position = -1;
    read_mode_switch();
}

// read a 6 position switch
bool RC_Channel::read_6pos_switch(int8_t& position)
{
    // calculate position of 6 pos switch
    const uint16_t pulsewidth = get_radio_in();
    if (pulsewidth <= RC_MIN_LIMIT_PWM || pulsewidth >= RC_MAX_LIMIT_PWM) {
        return false;  // This is an error condition
    }

    if      (pulsewidth < 1231) position = 0;
    else if (pulsewidth < 1361) position = 1;
    else if (pulsewidth < 1491) position = 2;
    else if (pulsewidth < 1621) position = 3;
    else if (pulsewidth < 1750) position = 4;
    else position = 5;

    if (!debounce_completed(position)) {
        return false;
    }

    return true;
}

void RC_Channel::read_mode_switch()
{
    int8_t position;
    if (read_6pos_switch(position)) {
        // set flight mode and simple mode setting
        mode_switch_changed(modeswitch_pos_t(position));
    }
}

bool RC_Channel::debounce_completed(int8_t position) 
{
    // switch change not detected
    if (switch_state.current_position == position) {
        // reset debouncing
         switch_state.debounce_position = position;
    } else {
        // switch change detected
        const uint32_t tnow_ms = AP_HAL::millis();

        // position not established yet
        if (switch_state.debounce_position != position) {
            switch_state.debounce_position = position;
            switch_state.last_edge_time_ms = tnow_ms;
        } else if (tnow_ms - switch_state.last_edge_time_ms >= SWITCH_DEBOUNCE_TIME_MS) {
            // position estabilished; debounce completed
            switch_state.current_position = position;
            return true;
        }
    }

    return false;
}

/*
  read an aux channel. Return true if a switch has changed
 */
bool RC_Channel::read_aux()
{
    const AP_AuxFunc::Function _option = (AP_AuxFunc::Function)option.get();
    if (_option == AP_AuxFunc::Function::DO_NOTHING) {
        // may wish to add special cases for other "AUXSW" things
        // here e.g. RCMAP_ROLL etc once they become options
        return false;
    } else if (_option == AP_AuxFunc::Function::VTX_POWER) {
        int8_t position;
        if (read_6pos_switch(position)) {
            AP::vtx().change_power(position);
            return true;
        }
        return false;
    }

    AP_AuxFunc::SwitchPos new_position;
    if (!read_3pos_switch(new_position)) {
        return false;
    }

    if (!debounce_completed((int8_t)new_position)) {
        return false;
    }

#if !HAL_MINIMIZE_FEATURES
    // announce the change to the GCS:
    const char *aux_string = AP_AuxFunc::string_for_function(_option);
    if (aux_string != nullptr) {
        const char *temp =  nullptr;
        switch (new_position) {
        case AP_AuxFunc::SwitchPos::HIGH:
            temp = "HIGH";
            break;
        case AP_AuxFunc::SwitchPos::MIDDLE:
            temp = "MIDDLE";
            break;
        case AP_AuxFunc::SwitchPos::LOW:
            temp = "LOW";
            break;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "%s %s", aux_string, temp);
    }
#endif

    // debounced; undertake the action:
    AP::auxfunc().run_function(_option, new_position, AP_AuxFunc::TriggerSource::RC);
    return true;
}

void RC_Channel::init_aux()
{
    AP_AuxFunc::SwitchPos position;
    if (!read_3pos_switch(position)) {
        position = AP_AuxFunc::SwitchPos::LOW;
    }
    const AP_AuxFunc::Function function = (AP_AuxFunc::Function)option.get();
    if (!AP::auxfunc().init_function(function, position)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_BoardConfig::config_error("Failed to init: RC%u_OPTION: %u",
                           (unsigned)(this->ch_in+1), (unsigned)function);
#endif
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to init: RC%u_OPTION: %u\n",
                           (unsigned)(this->ch_in+1), (unsigned)function);
    }
}

// read_3pos_switch
bool RC_Channel::read_3pos_switch(AP_AuxFunc::SwitchPos &ret) const
{
    const uint16_t in = get_radio_in();
    if (in <= RC_MIN_LIMIT_PWM || in >= RC_MAX_LIMIT_PWM) {
        return false;
    }
    
    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = reversed && rc().switch_reverse_allowed();
    
    if (in < AUX_SWITCH_PWM_TRIGGER_LOW) {
        ret = switch_reversed ? AP_AuxFunc::SwitchPos::HIGH : AP_AuxFunc::SwitchPos::LOW;
    } else if (in > AUX_SWITCH_PWM_TRIGGER_HIGH) {
        ret = switch_reversed ? AP_AuxFunc::SwitchPos::LOW : AP_AuxFunc::SwitchPos::HIGH;
    } else {
        ret = AP_AuxFunc::SwitchPos::MIDDLE;
    }
    return true;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
AP_AuxFunc::SwitchPos RC_Channel::get_aux_switch_pos() const
{
    AP_AuxFunc::SwitchPos position = AP_AuxFunc::SwitchPos::LOW;
    UNUSED_RESULT(read_3pos_switch(position));

    return position;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
AP_AuxFunc::SwitchPos RC_Channels::get_channel_pos(const uint8_t rcmapchan) const
{
    const RC_Channel* chan = rc().channel(rcmapchan-1);
    return chan != nullptr ? chan->get_aux_switch_pos() : AP_AuxFunc::SwitchPos::LOW;
}

RC_Channel *RC_Channels::find_channel_for_option(const AP_AuxFunc::Function option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((AP_AuxFunc::Function)c->option.get() == option) {
            return c;
        }
    }
    return nullptr;
}

// duplicate_options_exist - returns true if any options are duplicated
bool RC_Channels::duplicate_options_exist()
{
    uint8_t auxsw_option_counts[256] = {};
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        const RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        const uint16_t option = c->option.get();
        if (option >= sizeof(auxsw_option_counts)) {
            continue;
        }
        auxsw_option_counts[option]++;
    }

    for (uint16_t i=0; i<sizeof(auxsw_option_counts); i++) {
        if (i == 0) { // MAGIC VALUE! This is AUXSW_DO_NOTHING
            continue;
        }
        if (auxsw_option_counts[i] > 1) {
            return true;
        }
    }
   return false;
}
