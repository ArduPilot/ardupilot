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

#include <AC_Avoidance/AC_Avoid.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_LandingGear/AP_LandingGear.h>

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
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw, 38:ADSB-Avoidance, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 43:InvertedFlight, 44:Winch Enable, 45:WinchControl, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66: Relay5, 67: Relay6
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 7:Save WP, 9:Camera Trigger, 16:Auto, 19:Gripper, 28:Relay On/Off, 30:Lost Rover Sound, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm, 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66: Relay5, 67: Relay6
    // @Values{Plane}: 0:Do Nothing, 9:Camera Trigger, 28:Relay On/Off, 29:Landing Gear, 34:Relay2 On/Off, 30:Lost Plane Sound, 35:Relay3 On/Off, 36:Relay4 On/Off, 41:ArmDisarm, 43:InvertedFlight, 46:RC Override Enable, 58:Clear Waypoints, 62:Compass Learn, 64: Reverse Throttle, 65:GPS Disable, 66: Relay5, 67: Relay6
    // @User: Standard
    AP_GROUPINFO_FRAME("OPTION",  6, RC_Channel, option, 0, AP_PARAM_FRAME_COPTER|AP_PARAM_FRAME_ROVER|AP_PARAM_FRAME_PLANE),

    AP_GROUPEND
};


// constructor
RC_Channel::RC_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void
RC_Channel::set_range(uint16_t high)
{
    type_in = RC_CHANNEL_TYPE_RANGE;
    high_in = high;
}

void
RC_Channel::set_angle(uint16_t angle)
{
    type_in = RC_CHANNEL_TYPE_ANGLE;
    high_in = angle;
}

void
RC_Channel::set_default_dead_zone(int16_t dzone)
{
    dead_zone.set_default(abs(dzone));
}

bool
RC_Channel::get_reverse(void) const
{
    return bool(reversed.get());
}

// read input from hal.rcin or overrides
bool
RC_Channel::update(void)
{
    if (has_override() && !(*RC_Channels::options & RC_IGNORE_OVERRIDES)) {
        radio_in = override_value;
    } else if (!(*RC_Channels::options & RC_IGNORE_RECEIVER)) {
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
void
RC_Channel::recompute_pwm_no_deadzone()
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
int16_t
RC_Channel::pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim) const
{
    int16_t radio_trim_high = _trim + _dead_zone;
    int16_t radio_trim_low  = _trim - _dead_zone;

    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in > radio_trim_high && radio_max != radio_trim_high) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(radio_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    } else if (radio_in < radio_trim_low && radio_trim_low != radio_min) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t _dead_zone) const
{
    return pwm_to_angle_dz_trim(_dead_zone, radio_trim);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle() const
{
	return pwm_to_angle_dz(dead_zone);
}


/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t
RC_Channel::pwm_to_range_dz(uint16_t _dead_zone) const
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
int16_t
RC_Channel::pwm_to_range() const
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

float
RC_Channel::norm_input() const
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

float
RC_Channel::norm_input_dz() const
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

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t
RC_Channel::percent_input() const
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
  Return true if the channel is at trim and within the DZ
*/
bool RC_Channel::in_trim_dz() const
{
    return is_bounded_int32(radio_in, radio_trim - dead_zone, radio_trim + dead_zone);
}

void RC_Channel::set_override(const uint16_t v, const uint32_t timestamp_us)
{
    if (!rc().gcs_overrides_enabled()) {
        return;
    }
    // this UINT16_MAX stuff should really, really be in the
    // mavlink packet handling code.  It can be moved once that
    // code is in the GCS_MAVLink class!
    if (v == UINT16_MAX) {
        return;
    }
    last_override_time = timestamp_us != 0 ? timestamp_us : AP_HAL::millis();
    override_value = v;
    RC_Channels::has_new_overrides = true;
}

void RC_Channel::clear_override()
{
    last_override_time = 0;
    override_value = 0;
}

bool RC_Channel::has_override() const
{
    if (override_value <= 0) {
        return false;
    }

    const float override_timeout_ms = RC_Channels::override_timeout->get() * 1e3f;
    return is_positive(override_timeout_ms) && ((AP_HAL::millis() - last_override_time) < (uint32_t)override_timeout_ms);
}

//
// support for auxillary switches:
//
#define MODE_SWITCH_DEBOUNCE_TIME_MS  200

uint32_t RC_Channel::old_switch_positions;
RC_Channel::modeswitch_state_t RC_Channel::mode_switch_state;

void RC_Channel::reset_mode_switch()
{
    mode_switch_state.last_position = -1;
    mode_switch_state.debounced_position = -1;
    read_mode_switch();
}

void RC_Channel::read_mode_switch()
{
    // calculate position of flight mode switch
    const uint16_t pulsewidth = get_radio_in();
    if (pulsewidth <= 900 || pulsewidth >= 2200) {
        return;  // This is an error condition
    }

    modeswitch_pos_t position;
    if      (pulsewidth < 1231) position = 0;
    else if (pulsewidth < 1361) position = 1;
    else if (pulsewidth < 1491) position = 2;
    else if (pulsewidth < 1621) position = 3;
    else if (pulsewidth < 1750) position = 4;
    else position = 5;

    if (mode_switch_state.last_position == position) {
        // nothing to do
        return;
    }

    const uint32_t tnow_ms = AP_HAL::millis();
    if (position != mode_switch_state.debounced_position) {
        mode_switch_state.debounced_position = position;
        // store time that switch last moved
        mode_switch_state.last_edge_time_ms = tnow_ms;
        return;
    }

    if (tnow_ms - mode_switch_state.last_edge_time_ms < MODE_SWITCH_DEBOUNCE_TIME_MS) {
        // still in debounce
        return;
    }

    // set flight mode and simple mode setting
    mode_switch_changed(position);

    // set the last switch position.  This marks the
    // transition as complete, even if the mode switch actually
    // failed.  This prevents the vehicle changing modes
    // unexpectedly some time later.
    mode_switch_state.last_position = position;
}

//
// support for auxillary switches:
//

// init_aux_switch_function - initialize aux functions
void RC_Channel::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
    case RC_OVERRIDE_ENABLE:
    case AVOID_PROXIMITY:
        do_aux_function(ch_option, ch_flag);
        break;
    // the following functions to not need to be initialised:
    case RELAY:
    case RELAY2:
    case RELAY3:
    case RELAY4:
    case RELAY5:
    case RELAY6:
    case CAMERA_TRIGGER:
    case LOST_VEHICLE_SOUND:
    case DO_NOTHING:
    case CLEAR_WP:
    case COMPASS_LEARN:
    case LANDING_GEAR:
        break;
    case GRIPPER:
    case SPRAYER:
    case GPS_DISABLE:
        do_aux_function(ch_option, ch_flag);
        break;
    default:
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to initialise RC function (%u)", ch_option);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("RC function (%u) initialisation not handled", ch_option);
#endif
        break;
    }
}

void RC_Channel::read_aux()
{
    const aux_func_t _option = (aux_func_t)option.get();
    if (_option == DO_NOTHING) {
        // may wish to add special cases for other "AUXSW" things
        // here e.g. RCMAP_ROLL etc once they become options
        return;
    }
    aux_switch_pos_t new_position;
    if (!read_3pos_switch(new_position)) {
        return;
    }
    const aux_switch_pos_t old_position = old_switch_position();
    if (new_position == old_position) {
        debounce.count = 0;
        return;
    }
    if (debounce.new_position != new_position) {
        debounce.new_position = new_position;
        debounce.count = 0;
    }
    // a value of 2 means we need 3 values in a row with the same
    // value to activate
    if (debounce.count++ < 2) {
        return;
    }

    // debounced; undertake the action:
    do_aux_function(_option, new_position);
    set_old_switch_position(new_position);
}


void RC_Channel::do_aux_function_avoid_proximity(const aux_switch_pos_t ch_flag)
{
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    switch (ch_flag) {
    case HIGH:
        avoid->proximity_avoidance_enable(true);
        break;
    case MIDDLE:
        // nothing
        break;
    case LOW:
        avoid->proximity_avoidance_enable(false);
        break;
    }
}

void RC_Channel::do_aux_function_camera_trigger(const aux_switch_pos_t ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return;
    }
    if (ch_flag == HIGH) {
        camera->take_picture();
    }
}

void RC_Channel::do_aux_function_clear_wp(const aux_switch_pos_t ch_flag)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    if (ch_flag == HIGH) {
        mission->clear();
    }
}

void RC_Channel::do_aux_function_relay(const uint8_t relay, bool val)
{
    AP_ServoRelayEvents *servorelayevents = AP::servorelayevents();
    if (servorelayevents == nullptr) {
        return;
    }
    servorelayevents->do_set_relay(relay, val);
}

void RC_Channel::do_aux_function_sprayer(const aux_switch_pos_t ch_flag)
{
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return;
    }

    sprayer->run(ch_flag == HIGH);
    // if we are disarmed the pilot must want to test the pump
    sprayer->test_pump((ch_flag == HIGH) && !hal.util->get_soft_armed());
}

void RC_Channel::do_aux_function_gripper(const aux_switch_pos_t ch_flag)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return;
    }

    switch(ch_flag) {
    case LOW:
        gripper->release();
//        copter.Log_Write_Event(DATA_GRIPPER_RELEASE);
        break;
    case MIDDLE:
        // nothing
        break;
    case HIGH:
        gripper->grab();
//        copter.Log_Write_Event(DATA_GRIPPER_GRAB);
        break;
    }
}

void RC_Channel::do_aux_function_lost_vehicle_sound(const aux_switch_pos_t ch_flag)
{
    switch (ch_flag) {
    case HIGH:
        AP_Notify::flags.vehicle_lost = true;
        break;
    case MIDDLE:
        // nothing
        break;
    case LOW:
        AP_Notify::flags.vehicle_lost = false;
        break;
    }
}

void RC_Channel::do_aux_function_rc_override_enable(const aux_switch_pos_t ch_flag)
{
    switch (ch_flag) {
    case HIGH: {
        rc().set_gcs_overrides_enabled(true);
        break;
    }
    case MIDDLE:
        // nothing
        break;
    case LOW: {
        rc().set_gcs_overrides_enabled(false);
        break;
    }
    }
}

void RC_Channel::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
    case CAMERA_TRIGGER:
        do_aux_function_camera_trigger(ch_flag);
        break;

    case GRIPPER:
        do_aux_function_gripper(ch_flag);
        break;

    case RC_OVERRIDE_ENABLE:
        // Allow or disallow RC_Override
        do_aux_function_rc_override_enable(ch_flag);
        break;

    case AVOID_PROXIMITY:
        do_aux_function_avoid_proximity(ch_flag);
        break;

    case RELAY:
        do_aux_function_relay(0, ch_flag == HIGH);
        break;
    case RELAY2:
        do_aux_function_relay(1, ch_flag == HIGH);
        break;
    case RELAY3:
        do_aux_function_relay(2, ch_flag == HIGH);
        break;
    case RELAY4:
        do_aux_function_relay(3, ch_flag == HIGH);
        break;
    case RELAY5:
        do_aux_function_relay(4, ch_flag == HIGH);
        break;
    case RELAY6:
        do_aux_function_relay(5, ch_flag == HIGH);
        break;
    case CLEAR_WP:
        do_aux_function_clear_wp(ch_flag);
        break;

    case SPRAYER:
        do_aux_function_sprayer(ch_flag);
        break;

    case LOST_VEHICLE_SOUND:
        do_aux_function_lost_vehicle_sound(ch_flag);
        break;

    case COMPASS_LEARN:
        if (ch_flag == HIGH) {
            Compass &compass = AP::compass();
            compass.set_learn_type(Compass::LEARN_INFLIGHT, false);
        }
        break;

    case LANDING_GEAR: {
        AP_LandingGear *lg = AP_LandingGear::instance();
        if (lg == nullptr) {
            break;
        }
        switch (ch_flag) {
        case LOW:
            lg->set_position(AP_LandingGear::LandingGear_Deploy);
            break;
        case MIDDLE:
            // nothing
            break;
        case HIGH:
            lg->set_position(AP_LandingGear::LandingGear_Retract);
            break;
        }
        break;
    }

    case GPS_DISABLE:
        AP::gps().force_disable(ch_flag == HIGH);
        break;
        
    default:
        gcs().send_text(MAV_SEVERITY_INFO, "Invalid channel option (%u)", ch_option);
        break;
    }
}

void RC_Channel::init_aux()
{
    aux_switch_pos_t position;
    if (!read_3pos_switch(position)) {
        position = aux_switch_pos_t::LOW;
    }
    init_aux_function((aux_func_t)option.get(), position);
}

// read_3pos_switch
bool RC_Channel::read_3pos_switch(RC_Channel::aux_switch_pos_t &ret) const
{
    const uint16_t in = get_radio_in();
    if (in <= 900 or in >= 2200) {
        return false;
    }
    if (in < AUX_PWM_TRIGGER_LOW) {
        ret = LOW;
    } else if (in > AUX_PWM_TRIGGER_HIGH) {
        ret = HIGH;
    } else {
        ret = MIDDLE;
    }
    return true;
}

RC_Channel *RC_Channels::find_channel_for_option(const RC_Channel::aux_func_t option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::aux_func_t)c->option.get() == option) {
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
