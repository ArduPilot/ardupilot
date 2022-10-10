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
#include <AP_Camera/AP_Camera.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_Torqeedo/AP_Torqeedo.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

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
    // @Values{Copter}: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 13:Super Simple Mode, 14:Acro Trainer, 15:Sprayer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount1, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 37:Throw, 38:ADSB Avoidance En, 39:PrecLoiter, 40:Proximity Avoidance, 41:ArmDisarm (4.1 and lower), 42:SmartRTL, 43:InvertedFlight, 44:Winch Enable, 46:RC Override Enable, 47:User Function 1, 48:User Function 2, 49:User Function 3, 52:Acro, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 60:ZigZag, 61:ZigZag SaveWP, 62:Compass Learn, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 68:Stabilize, 69:PosHold, 70:AltHold, 71:FlowHold, 72:Circle, 73:Drift, 75:SurfaceTrackingUpDown, 76:Standby Mode, 78:RunCam Control, 79:RunCam OSD Control, 80:VisOdom Align, 81:Disarm, 83:ZigZag Auto, 84:Air Mode, 85:Generator, 90:EKF Pos Source, 94:VTX Power, 99:AUTO RTL, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 151:Turtle, 152:simple heading reset, 153:ArmDisarm (4.2 and higher), 154:ArmDisarm with AirMode  (4.2 and higher), 158:Optflow Calibration, 159:Force Flying, 161:Turbine Start(heli), 162:FFT Tune, 163:Mount Lock, 164:Pause Stream Logging, 165:Arm/Emergency Motor Stop, 212:Mount1 Roll, 213:Mount1 Pitch, 214:Mount1 Yaw, 215:Mount2 Roll, 216:Mount2 Pitch, 217:Mount2 Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Rover}: 0:Do Nothing, 4:RTL, 5:Save Trim (4.1 and lower), 7:Save WP, 9:Camera Trigger, 11:Fence, 16:Auto, 19:Gripper, 24:Auto Mission Reset, 27:Retract Mount1, 28:Relay On/Off, 30:Lost Rover Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 40:Proximity Avoidance, 41:ArmDisarm (4.1 and lower), 42:SmartRTL, 46:RC Override Enable, 50:LearnCruise, 51:Manual, 52:Acro, 53:Steering, 54:Hold, 55:Guided, 56:Loiter, 57:Follow, 58:Clear Waypoints, 59:Simple Mode, 62:Compass Learn, 63:Sailboat Tack, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 74:Sailboat motoring 3pos, 78:RunCam Control, 79:RunCam OSD Control, 80:Viso Align, 81:Disarm, 90:EKF Pos Source, 94:VTX Power, 97:Windvane home heading direction offset, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 106:Disable Airspeed Use, 153:ArmDisarm (4.2 and higher), 155: set steering trim to current servo and RC, 156:Torqeedo Clear Err, 162:FFT Tune, 163:Mount Lock, 164:Pause Stream Logging, 165:Arm/Emergency Motor Stop, 201:Roll, 202:Pitch, 207:MainSail, 208:Flap, 211:Walking Height, 212:Mount1 Roll, 213:Mount1 Pitch, 214:Mount1 Yaw, 215:Mount2 Roll, 216:Mount2 Pitch, 217:Mount2 Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Plane}: 0:Do Nothing, 4:ModeRTL, 9:Camera Trigger, 11:Fence, 16:ModeAuto, 22:Parachute Release, 24:Auto Mission Reset, 27:Retract Mount1, 28:Relay On/Off, 29:Landing Gear, 30:Lost Plane Sound, 31:Motor Emergency Stop, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 38:ADSB Avoidance En, 41:ArmDisarm (4.1 and lower), 43:InvertedFlight, 46:RC Override Enable, 51:ModeManual, 52: ModeACRO, 55:ModeGuided, 56:ModeLoiter, 58:Clear Waypoints, 62:Compass Learn, 64:Reverse Throttle, 65:GPS Disable, 66:Relay5 On/Off, 67:Relay6 On/Off, 72:ModeCircle, 77:ModeTakeoff, 78:RunCam Control, 79:RunCam OSD Control, 81:Disarm, 82:QAssist 3pos, 84:Air Mode, 85:Generator, 86: Non Auto Terrain Follow Disable, 87:Crow Select, 88:Soaring Enable, 89:Landing Flare, 90:EKF Pos Source, 91:Airspeed Ratio Calibration, 92:FBWA, 94:VTX Power, 95:FBWA taildragger takeoff mode, 96:trigger re-reading of mode switch, 98: ModeTraining, 100:KillIMU1, 101:KillIMU2, 102:Camera Mode Toggle, 105:GPS Disable Yaw, 106:Disable Airspeed Use, 107: EnableFixedWingAutotune, 108: ModeQRTL, 150: CRUISE, 153:ArmDisarm (4.2 and higher), 154:ArmDisarm with Quadplane AirMode (4.2 and higher), 155: set roll pitch and yaw trim to current servo and RC, 157: Force FS Action to FBWA, 158:Optflow Calibration, 160:Weathervane Enable, 162:FFT Tune, 163:Mount Lock, 164:Pause Stream Logging, 165:Arm/Emergency Motor Stop, 208:Flap, 209: Forward Throttle, 210: Airbrakes, 212:Mount1 Roll, 213:Mount1 Pitch, 214:Mount1 Yaw, 215:Mount2 Roll, 216:Mount2 Pitch, 217:Mount2 Yaw, 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @Values{Blimp}: 0:Do Nothing, 18:Land, 46:RC Override Enable, 65:GPS Disable, 81:Disarm, 90:EKF Pos Source, 100:KillIMU1, 101:KillIMU2, 153:ArmDisarm, 164:Pause Stream Logging
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
    type_in = ControlType::RANGE;
    high_in = high;
}

void RC_Channel::set_angle(uint16_t angle)
{
    type_in = ControlType::ANGLE;
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

    if (type_in == ControlType::RANGE) {
        control_in = pwm_to_range();
    } else {
        // ControlType::ANGLE
        control_in = pwm_to_angle();
    }

    return true;
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t RC_Channel::get_control_mid() const
{
    if (type_in == ControlType::RANGE) {
        int16_t r_in = (radio_min.get() + radio_max.get())/2;

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
    if (type_in == ControlType::RANGE) {
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


/*
   return trues if input is within deadzone of min
*/
bool RC_Channel::in_min_dz() const
{
    return radio_in < radio_min + dead_zone;
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
float RC_Channel::stick_mixing(const float servo_in)
{
    float ch_inf = (float)(radio_in - radio_trim);
    ch_inf = fabsf(ch_inf);
    ch_inf = MIN(ch_inf, 400.0f);
    ch_inf = ((400.0f - ch_inf) / 400.0f);

    float servo_out = servo_in;
    servo_out *= ch_inf;
    servo_out += control_in;

    return servo_out;
}

//
// support for auxiliary switches:
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

//
// support for auxiliary switches:
//

// init_aux_switch_function - initialize aux functions
void RC_Channel::init_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::ARMDISARM:
    case AUX_FUNC::CAMERA_TRIGGER:
    case AUX_FUNC::CLEAR_WP:
    case AUX_FUNC::COMPASS_LEARN:
    case AUX_FUNC::DISARM:
    case AUX_FUNC::DO_NOTHING:
    case AUX_FUNC::LANDING_GEAR:
    case AUX_FUNC::LOST_VEHICLE_SOUND:
    case AUX_FUNC::RELAY:
    case AUX_FUNC::RELAY2:
    case AUX_FUNC::RELAY3:
    case AUX_FUNC::RELAY4:
    case AUX_FUNC::RELAY5:
    case AUX_FUNC::RELAY6:
    case AUX_FUNC::VISODOM_ALIGN:
    case AUX_FUNC::EKF_LANE_SWITCH:
    case AUX_FUNC::EKF_YAW_RESET:
    case AUX_FUNC::GENERATOR: // don't turn generator on or off initially
    case AUX_FUNC::EKF_POS_SOURCE:
    case AUX_FUNC::TORQEEDO_CLEAR_ERR:
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
    case AUX_FUNC::VTX_POWER:
    case AUX_FUNC::OPTFLOW_CAL:
    case AUX_FUNC::TURBINE_START:
    case AUX_FUNC::MOUNT1_ROLL:
    case AUX_FUNC::MOUNT1_PITCH:
    case AUX_FUNC::MOUNT1_YAW:
    case AUX_FUNC::MOUNT2_ROLL:
    case AUX_FUNC::MOUNT2_PITCH:
    case AUX_FUNC::MOUNT2_YAW:
        break;
    case AUX_FUNC::AVOID_ADSB:
    case AUX_FUNC::AVOID_PROXIMITY:
    case AUX_FUNC::FENCE:
    case AUX_FUNC::GPS_DISABLE:
    case AUX_FUNC::GPS_DISABLE_YAW:
    case AUX_FUNC::GRIPPER:
    case AUX_FUNC::KILL_IMU1:
    case AUX_FUNC::KILL_IMU2:
    case AUX_FUNC::MISSION_RESET:
    case AUX_FUNC::MOTOR_ESTOP:
    case AUX_FUNC::RC_OVERRIDE_ENABLE:
    case AUX_FUNC::RUNCAM_CONTROL:
    case AUX_FUNC::RUNCAM_OSD_CONTROL:
    case AUX_FUNC::SPRAYER:
    case AUX_FUNC::DISABLE_AIRSPEED_USE:
    case AUX_FUNC::FFT_NOTCH_TUNE:
#if HAL_MOUNT_ENABLED
    case AUX_FUNC::RETRACT_MOUNT1:
    case AUX_FUNC::MOUNT_LOCK:
#endif
    case AUX_FUNC::LOG_PAUSE:
    case AUX_FUNC::ARM_EMERGENCY_STOP:

        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
    default:
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to init: RC%u_OPTION: %u\n",
                           (unsigned)(this->ch_in+1), (unsigned)ch_option);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_BoardConfig::config_error("Failed to init: RC%u_OPTION: %u",
                           (unsigned)(this->ch_in+1), (unsigned)ch_option);
#endif
        break;
    }
}

#if !HAL_MINIMIZE_FEATURES

const RC_Channel::LookupTable RC_Channel::lookuptable[] = {
    { AUX_FUNC::SAVE_WP,"SaveWaypoint"},
    { AUX_FUNC::CAMERA_TRIGGER,"CameraTrigger"},
    { AUX_FUNC::RANGEFINDER,"Rangefinder"},
    { AUX_FUNC::FENCE,"Fence"},
    { AUX_FUNC::SPRAYER,"Sprayer"},
    { AUX_FUNC::PARACHUTE_ENABLE,"ParachuteEnable"},
    { AUX_FUNC::PARACHUTE_RELEASE,"ParachuteRelease"},
    { AUX_FUNC::PARACHUTE_3POS,"Parachute3Position"},
    { AUX_FUNC::MISSION_RESET,"MissionReset"},
    { AUX_FUNC::RETRACT_MOUNT1,"RetractMount1"},
    { AUX_FUNC::RELAY,"Relay1"},
    { AUX_FUNC::MOTOR_ESTOP,"MotorEStop"},
    { AUX_FUNC::MOTOR_INTERLOCK,"MotorInterlock"},
    { AUX_FUNC::RELAY2,"Relay2"},
    { AUX_FUNC::RELAY3,"Relay3"},
    { AUX_FUNC::RELAY4,"Relay4"},
    { AUX_FUNC::PRECISION_LOITER,"PrecisionLoiter"},
    { AUX_FUNC::AVOID_PROXIMITY,"AvoidProximity"},
    { AUX_FUNC::WINCH_ENABLE,"WinchEnable"},
    { AUX_FUNC::WINCH_CONTROL,"WinchControl"},
    { AUX_FUNC::CLEAR_WP,"ClearWaypoint"},
    { AUX_FUNC::COMPASS_LEARN,"CompassLearn"},
    { AUX_FUNC::SAILBOAT_TACK,"SailboatTack"},
    { AUX_FUNC::GPS_DISABLE,"GPSDisable"},
    { AUX_FUNC::GPS_DISABLE_YAW,"GPSDisableYaw"},
    { AUX_FUNC::DISABLE_AIRSPEED_USE,"DisableAirspeedUse"},
    { AUX_FUNC::RELAY5,"Relay5"},
    { AUX_FUNC::RELAY6,"Relay6"},
    { AUX_FUNC::SAILBOAT_MOTOR_3POS,"SailboatMotor"},
    { AUX_FUNC::SURFACE_TRACKING,"SurfaceTracking"},
    { AUX_FUNC::RUNCAM_CONTROL,"RunCamControl"},
    { AUX_FUNC::RUNCAM_OSD_CONTROL,"RunCamOSDControl"},
    { AUX_FUNC::VISODOM_ALIGN,"VisOdomAlign"},
    { AUX_FUNC::AIRMODE, "AirMode"},
    { AUX_FUNC::EKF_POS_SOURCE,"EKFPosSource"},
    { AUX_FUNC::CAM_MODE_TOGGLE,"CamModeToggle"},
    { AUX_FUNC::GENERATOR,"Generator"},
    { AUX_FUNC::ARSPD_CALIBRATE,"Calibrate Airspeed"},
    { AUX_FUNC::TORQEEDO_CLEAR_ERR, "Torqeedo Clear Err"},
    { AUX_FUNC::EMERGENCY_LANDING_EN, "Emergency Landing"},
    { AUX_FUNC::WEATHER_VANE_ENABLE, "Weathervane"},
    { AUX_FUNC::TURBINE_START, "Turbine Start"},
    { AUX_FUNC::FFT_NOTCH_TUNE, "FFT Notch Tuning"},
    { AUX_FUNC::MOUNT_LOCK, "MountLock"},
    { AUX_FUNC::LOG_PAUSE, "Pause Stream Logging"},
};

/* lookup the announcement for switch change */
const char *RC_Channel::string_for_aux_function(AUX_FUNC function) const     
{
     for (const struct LookupTable &entry : lookuptable) {
        if (entry.option == function) {
            return entry.announcement;
        }
     }
     return nullptr;
}

#endif // HAL_MINIMIZE_FEATURES

/*
  read an aux channel. Return true if a switch has changed
 */
bool RC_Channel::read_aux()
{
    const aux_func_t _option = (aux_func_t)option.get();
    if (_option == AUX_FUNC::DO_NOTHING) {
        // may wish to add special cases for other "AUXSW" things
        // here e.g. RCMAP_ROLL etc once they become options
        return false;
    } else if (_option == AUX_FUNC::VTX_POWER) {
        int8_t position;
        if (read_6pos_switch(position)) {
            AP::vtx().change_power(position);
            return true;
        }
        return false;
    }

    AuxSwitchPos new_position;
    if (!read_3pos_switch(new_position)) {
        return false;
    }

    if (!debounce_completed((int8_t)new_position)) {
        return false;
    }

#if !HAL_MINIMIZE_FEATURES
    // announce the change to the GCS:
    const char *aux_string = string_for_aux_function(_option);
    if (aux_string != nullptr) {
        const char *temp =  nullptr;
        switch (new_position) {
        case AuxSwitchPos::HIGH:
            temp = "HIGH";           
            break;
        case AuxSwitchPos::MIDDLE:
            temp = "MIDDLE";
            break;
        case AuxSwitchPos::LOW:
            temp = "LOW";          
            break;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "%s %s", aux_string, temp);
    }
#endif

    // debounced; undertake the action:
    run_aux_function(_option, new_position, AuxFuncTriggerSource::RC);
    return true;
}


void RC_Channel::do_aux_function_armdisarm(const AuxSwitchPos ch_flag)
{
    // arm or disarm the vehicle
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        break;
    }
}

void RC_Channel::do_aux_function_avoid_adsb(const AuxSwitchPos ch_flag)
{
#if HAL_ADSB_ENABLED
    AP_Avoidance *avoidance = AP::ap_avoidance();
    if (avoidance == nullptr) {
        return;
    }
    AP_ADSB *adsb = AP::ADSB();
    if (adsb == nullptr) {
        return;
    }
    if (ch_flag == AuxSwitchPos::HIGH) {
        // try to enable AP_Avoidance
        if (!adsb->enabled() || !adsb->healthy()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB not available");
            return;
        }
        avoidance->enable();
        AP::logger().Write_Event(LogEvent::AVOIDANCE_ADSB_ENABLE);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Enabled");
        return;
    }

    // disable AP_Avoidance
    avoidance->disable();
    AP::logger().Write_Event(LogEvent::AVOIDANCE_ADSB_DISABLE);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Disabled");
#endif
}

void RC_Channel::do_aux_function_avoid_proximity(const AuxSwitchPos ch_flag)
{
#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        avoid->proximity_avoidance_enable(true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        avoid->proximity_avoidance_enable(false);
        break;
    }
#endif // !APM_BUILD_ArduPlane
}

void RC_Channel::do_aux_function_camera_trigger(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return;
    }
    if (ch_flag == AuxSwitchPos::HIGH) {
        camera->take_picture();
    }
}

void RC_Channel::do_aux_function_runcam_control(const AuxSwitchPos ch_flag)
{
#if HAL_RUNCAM_ENABLED
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            runcam->start_recording();
            break;
        case AuxSwitchPos::MIDDLE:
            runcam->osd_option();
            break;
        case AuxSwitchPos::LOW:
            runcam->stop_recording();
            break;
    }
#endif
}

void RC_Channel::do_aux_function_runcam_osd_control(const AuxSwitchPos ch_flag)
{
#if HAL_RUNCAM_ENABLED
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            runcam->enter_osd();
            break;
        case AuxSwitchPos::MIDDLE:
        case AuxSwitchPos::LOW:
            runcam->exit_osd();
            break;
    }
#endif
}

// enable or disable the fence
void RC_Channel::do_aux_function_fence(const AuxSwitchPos ch_flag)
{
#if AP_FENCE_ENABLED
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    fence->enable(ch_flag == AuxSwitchPos::HIGH);
#endif
}

void RC_Channel::do_aux_function_clear_wp(const AuxSwitchPos ch_flag)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    if (ch_flag == AuxSwitchPos::HIGH) {
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

#if HAL_GENERATOR_ENABLED
void RC_Channel::do_aux_function_generator(const AuxSwitchPos ch_flag)
{
    AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::LOW:
        generator->stop();
        break;
    case AuxSwitchPos::MIDDLE:
        generator->idle();
        break;
    case AuxSwitchPos::HIGH:
        generator->run();
        break;
    }
}
#endif

void RC_Channel::do_aux_function_sprayer(const AuxSwitchPos ch_flag)
{
#if HAL_SPRAYER_ENABLED
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return;
    }

    sprayer->run(ch_flag == AuxSwitchPos::HIGH);
    // if we are disarmed the pilot must want to test the pump
    sprayer->test_pump((ch_flag == AuxSwitchPos::HIGH) && !hal.util->get_soft_armed());
#endif // HAL_SPRAYER_ENABLED
}

void RC_Channel::do_aux_function_gripper(const AuxSwitchPos ch_flag)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return;
    }

    switch(ch_flag) {
    case AuxSwitchPos::LOW:
        gripper->release();
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::HIGH:
        gripper->grab();
        break;
    }
}

void RC_Channel::do_aux_function_lost_vehicle_sound(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP_Notify::flags.vehicle_lost = true;
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP_Notify::flags.vehicle_lost = false;
        break;
    }
}

void RC_Channel::do_aux_function_rc_override_enable(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH: {
        rc().set_gcs_overrides_enabled(true);
        break;
    }
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW: {
        rc().set_gcs_overrides_enabled(false);
        break;
    }
    }
}

void RC_Channel::do_aux_function_mission_reset(const AuxSwitchPos ch_flag)
{
    if (ch_flag != AuxSwitchPos::HIGH) {
        return;
    }
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    mission->reset();
}

void RC_Channel::do_aux_function_fft_notch_tune(const AuxSwitchPos ch_flag)
{
#if HAL_GYROFFT_ENABLED
    AP_GyroFFT *fft = AP::fft();
    if (fft == nullptr) {
        return;
    }

    switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            fft->start_notch_tune();
            break;
        case AuxSwitchPos::MIDDLE:
        case AuxSwitchPos::LOW:
            fft->stop_notch_tune();
            break;
    }
#endif
}

bool RC_Channel::run_aux_function(aux_func_t ch_option, AuxSwitchPos pos, AuxFuncTriggerSource source)
{
#if AP_SCRIPTING_ENABLED
    rc().set_aux_cached(ch_option, pos);
#endif
    const bool ret = do_aux_function(ch_option, pos);

    // @LoggerMessage: AUXF
    // @Description: Auxiliary function invocation information
    // @Field: TimeUS: Time since system startup
    // @Field: function: ID of triggered function
    // @Field: pos: switch position when function triggered
    // @Field: source: source of auxiliary function invocation
    // @Field: result: true if function was successful
    AP::logger().Write(
        "AUXF",
        "TimeUS,function,pos,source,result",
        "s#---",
        "F----",
        "QHBBB",
        AP_HAL::micros64(),
        uint16_t(ch_option),
        uint8_t(pos),
        uint8_t(source),
        uint8_t(ret)
        );
    return ret;
}

bool RC_Channel::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
    case AUX_FUNC::CAMERA_TRIGGER:
        do_aux_function_camera_trigger(ch_flag);
        break;

    case AUX_FUNC::FENCE:
        do_aux_function_fence(ch_flag);
        break;

    case AUX_FUNC::GRIPPER:
        do_aux_function_gripper(ch_flag);
        break;

    case AUX_FUNC::RC_OVERRIDE_ENABLE:
        // Allow or disallow RC_Override
        do_aux_function_rc_override_enable(ch_flag);
        break;

    case AUX_FUNC::AVOID_PROXIMITY:
        do_aux_function_avoid_proximity(ch_flag);
        break;

    case AUX_FUNC::RELAY:
        do_aux_function_relay(0, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY2:
        do_aux_function_relay(1, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY3:
        do_aux_function_relay(2, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY4:
        do_aux_function_relay(3, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY5:
        do_aux_function_relay(4, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY6:
        do_aux_function_relay(5, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::RUNCAM_CONTROL:
        do_aux_function_runcam_control(ch_flag);
        break;

    case AUX_FUNC::RUNCAM_OSD_CONTROL:
        do_aux_function_runcam_osd_control(ch_flag);
        break;

    case AUX_FUNC::CLEAR_WP:
        do_aux_function_clear_wp(ch_flag);
        break;
    case AUX_FUNC::MISSION_RESET:
        do_aux_function_mission_reset(ch_flag);
        break;

    case AUX_FUNC::AVOID_ADSB:
        do_aux_function_avoid_adsb(ch_flag);
        break;

    case AUX_FUNC::FFT_NOTCH_TUNE:
        do_aux_function_fft_notch_tune(ch_flag);
        break;

#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::GENERATOR:
        do_aux_function_generator(ch_flag);
        break;
#endif

    case AUX_FUNC::SPRAYER:
        do_aux_function_sprayer(ch_flag);
        break;

    case AUX_FUNC::LOST_VEHICLE_SOUND:
        do_aux_function_lost_vehicle_sound(ch_flag);
        break;

    case AUX_FUNC::ARMDISARM:
        do_aux_function_armdisarm(ch_flag);
        break;

    case AUX_FUNC::DISARM:
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        }
        break;

    case AUX_FUNC::COMPASS_LEARN:
        if (ch_flag == AuxSwitchPos::HIGH) {
            Compass &compass = AP::compass();
            compass.set_learn_type(Compass::LEARN_INFLIGHT, false);
        }
        break;

    case AUX_FUNC::LANDING_GEAR: {
        AP_LandingGear *lg = AP_LandingGear::get_singleton();
        if (lg == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            lg->set_position(AP_LandingGear::LandingGear_Deploy);
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            lg->set_position(AP_LandingGear::LandingGear_Retract);
            break;
        }
        break;
    }

    case AUX_FUNC::GPS_DISABLE:
        AP::gps().force_disable(ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::GPS_DISABLE_YAW:
        AP::gps().set_force_disable_yaw(ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::DISABLE_AIRSPEED_USE: {
#if AP_AIRSPEED_ENABLED
        AP_Airspeed *airspeed = AP::airspeed();
        if (airspeed == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            airspeed->force_disable_use(true);
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            airspeed->force_disable_use(false);
            break;
        }
#endif
        break;
    }

    case AUX_FUNC::MOTOR_ESTOP:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH: {
            SRV_Channels::set_emergency_stop(true);
            break;
        }
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::LOW: {
            SRV_Channels::set_emergency_stop(false);
            break;
        }
        }
        break;

    case AUX_FUNC::VISODOM_ALIGN:
#if HAL_VISUALODOM_ENABLED
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP_VisualOdom *visual_odom = AP::visualodom();
            if (visual_odom != nullptr) {
                visual_odom->align_sensor_to_vehicle();
            }
        }
#endif
        break;

    case AUX_FUNC::EKF_POS_SOURCE:
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // low switches to primary source
            AP::ahrs().set_posvelyaw_source_set(0);
            break;
        case AuxSwitchPos::MIDDLE:
            // middle switches to secondary source
            AP::ahrs().set_posvelyaw_source_set(1);
            break;
        case AuxSwitchPos::HIGH:
            // high switches to tertiary source
            AP::ahrs().set_posvelyaw_source_set(2);
            break;
        }
        break;

    case AUX_FUNC::OPTFLOW_CAL: {
#if AP_OPTICALFLOW_ENABLED
        AP_OpticalFlow *optflow = AP::opticalflow();
        if (optflow == nullptr) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "OptFlow Cal: failed sensor not enabled");
            break;
        }
        if (ch_flag == AuxSwitchPos::HIGH) {
            optflow->start_calibration();
        } else {
            optflow->stop_calibration();
        }
#endif
        break;
    }

#if !HAL_MINIMIZE_FEATURES
    case AUX_FUNC::KILL_IMU1:
        AP::ins().kill_imu(0, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::KILL_IMU2:
        AP::ins().kill_imu(1, ch_flag == AuxSwitchPos::HIGH);
        break;
#endif // HAL_MINIMIZE_FEATURES

    case AUX_FUNC::CAM_MODE_TOGGLE: {
        // Momentary switch to for cycling camera modes
        AP_Camera *camera = AP_Camera::get_singleton();
        if (camera == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // nothing
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            camera->cam_mode_toggle();
            break;
        }
        break;
    }

    case AUX_FUNC::RETRACT_MOUNT1: {
#if HAL_MOUNT_ENABLED
        AP_Mount *mount = AP::mount();
        if (mount == nullptr) {
            break;
        }
        switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                mount->set_mode(0,MAV_MOUNT_MODE_RETRACT);
                break;
            case AuxSwitchPos::MIDDLE:
                // nothing
                break;
            case AuxSwitchPos::LOW:
                mount->set_mode_to_default(0);
                break;
        }
#endif
        break;
    }

    case AUX_FUNC::MOUNT_LOCK: {
#if HAL_MOUNT_ENABLED
        AP_Mount *mount = AP::mount();
        if (mount == nullptr) {
            break;
        }
        mount->set_yaw_lock(ch_flag == AuxSwitchPos::HIGH);
#endif
        break;
    }

    case AUX_FUNC::LOG_PAUSE: {
        AP_Logger *logger = AP_Logger::get_singleton();
        switch (ch_flag) {
           case AuxSwitchPos::LOW:
               logger->log_pause(false);
               break;
           case AuxSwitchPos::MIDDLE:
                // nothing
                break;
           case AuxSwitchPos::HIGH:
               logger->log_pause(true);
               break;
           }
        break;
    }

    case AUX_FUNC::ARM_EMERGENCY_STOP: {
        switch (ch_flag) {
           case AuxSwitchPos::HIGH:
               // request arm, disable emergency motor stop
               SRV_Channels::set_emergency_stop(false);
               AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
               break;
           case AuxSwitchPos::MIDDLE:
               // disable emergency motor stop
               SRV_Channels::set_emergency_stop(false);
               break;
           case AuxSwitchPos::LOW:
               // enable emergency motor stop
               SRV_Channels::set_emergency_stop(true);
               break;
        }
        break;
    }

    case AUX_FUNC::EKF_LANE_SWITCH:
        // used to test emergency lane switch
        AP::ahrs().check_lane_switch();
        break;

    case AUX_FUNC::EKF_YAW_RESET:
        // used to test emergency yaw reset
        AP::ahrs().request_yaw_reset();
        break;

    // clear torqeedo error
    case AUX_FUNC::TORQEEDO_CLEAR_ERR: {
#if HAL_TORQEEDO_ENABLED
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP_Torqeedo *torqeedo = AP_Torqeedo::get_singleton();
            if (torqeedo != nullptr) {
                torqeedo->clear_motor_error();
            }
        }
#endif
        break;
    }

    // do nothing for these functions
    case AUX_FUNC::MOUNT1_ROLL:
    case AUX_FUNC::MOUNT1_PITCH:
    case AUX_FUNC::MOUNT1_YAW:
    case AUX_FUNC::MOUNT2_ROLL:
    case AUX_FUNC::MOUNT2_PITCH:
    case AUX_FUNC::MOUNT2_YAW:
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
        break;

    default:
        gcs().send_text(MAV_SEVERITY_INFO, "Invalid channel option (%u)", (unsigned int)ch_option);
        return false;
    }

    return true;
}

void RC_Channel::init_aux()
{
    AuxSwitchPos position;
    if (!read_3pos_switch(position)) {
        position = AuxSwitchPos::LOW;
    }
    init_aux_function((aux_func_t)option.get(), position);
}

// read_3pos_switch
bool RC_Channel::read_3pos_switch(RC_Channel::AuxSwitchPos &ret) const
{
    const uint16_t in = get_radio_in();
    if (in <= RC_MIN_LIMIT_PWM || in >= RC_MAX_LIMIT_PWM) {
        return false;
    }
    
    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = reversed && rc().switch_reverse_allowed();
    
    if (in < AUX_SWITCH_PWM_TRIGGER_LOW) {
        ret = switch_reversed ? AuxSwitchPos::HIGH : AuxSwitchPos::LOW;
    } else if (in > AUX_SWITCH_PWM_TRIGGER_HIGH) {
        ret = switch_reversed ? AuxSwitchPos::LOW : AuxSwitchPos::HIGH;
    } else {
        ret = AuxSwitchPos::MIDDLE;
    }
    return true;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
RC_Channel::AuxSwitchPos RC_Channel::get_aux_switch_pos() const
{
    AuxSwitchPos position = AuxSwitchPos::LOW;
    UNUSED_RESULT(read_3pos_switch(position));

    return position;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
RC_Channel::AuxSwitchPos RC_Channels::get_channel_pos(const uint8_t rcmapchan) const
{
    const RC_Channel* chan = rc().channel(rcmapchan-1);
    return chan != nullptr ? chan->get_aux_switch_pos() : RC_Channel::AuxSwitchPos::LOW;
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

// convert option parameter from old to new
void RC_Channels::convert_options(const RC_Channel::aux_func_t old_option, const RC_Channel::aux_func_t new_option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::aux_func_t)c->option.get() == old_option) {
            c->option.set_and_save((int16_t)new_option);
        }
    }
}
