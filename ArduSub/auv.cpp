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

#include "Sub.h"
#include "auv.h"

// Parameter table
const AP_Param::GroupInfo AUV_Control::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: AUV control enable
    // @Description: Enable dedicated AUV control surface allocation (always active when FRAME_CONFIG=8)
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AUV_Control, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TAIL_TYPE
    // @DisplayName: AUV tail geometry type
    // @Description: Tail fin layout. 0 for cruciform (+) with 2 horizontal elevators and 2 vertical rudders; 1 for X-tail (x) with 4 diagonal fins.
    // @Values: 0:Cruciform,1:X_Tail
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 2, AUV_Control, _tail_type, TAIL_CRUCIFORM),

    // @Param: FIN_MAX
    // @DisplayName: AUV fin maximum deflection angle
    // @Description: Maximum physical deflection angle of control-surface fins in degrees
    // @Range: 10 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("FIN_MAX", 3, AUV_Control, _fin_max_deg, 30.0f),

    // @Param: SPD_MIN
    // @DisplayName: AUV minimum hydrodynamic control speed
    // @Description: Minimum forward speed below which control surface authority is insufficient. Depth integrator is inhibited below this speed.
    // @Range: 0.1 2.0
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("SPD_MIN", 4, AUV_Control, _speed_min, 0.3f),

    // @Param: SPD_NOM
    // @DisplayName: AUV nominal cruise speed
    // @Description: Nominal design speed for hydrodynamic gain scaling.
    // @Range: 0.5 5.0
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("SPD_NOM", 5, AUV_Control, _speed_nom, 1.5f),

    // @Param: SPD_SCL_MIN
    // @DisplayName: AUV speed scaling minimum limit
    // @Description: Minimum gain scaler applied at high speeds
    // @Range: 0.2 1.0
    // @User: Advanced
    AP_GROUPINFO("SPD_SCL_MIN", 6, AUV_Control, _speed_scale_min, 0.5f),

    // @Param: SPD_SCL_MAX
    // @DisplayName: AUV speed scaling maximum limit
    // @Description: Maximum gain scaler applied at low speeds
    // @Range: 1.0 4.0
    // @User: Advanced
    AP_GROUPINFO("SPD_SCL_MAX", 7, AUV_Control, _speed_scale_max, 2.0f),

    // @Param: PITCH_MAX
    // @DisplayName: AUV maximum depth-control pitch angle
    // @Description: Maximum pitch angle demanded during depth change in degrees
    // @Range: 5 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 8, AUV_Control, _pitch_max_deg, 25.0f),

    // @Param: PITCH_GAIN
    // @DisplayName: AUV depth error to pitch angle gain
    // @Description: Gain converting depth tracking error to demanded pitch angle (deg per meter of depth error)
    // @Range: 0.5 20.0
    // @User: Standard
    AP_GROUPINFO("PITCH_GAIN", 9, AUV_Control, _pitch_gain, 5.0f),

    // @Param: CRUISE_THR
    // @DisplayName: AUV cruise forward throttle in depth-holding modes
    // @Description: Forward throttle percentage applied in depth-holding and autonomous modes to maintain hydrodynamic control
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("CRUISE_THR", 10, AUV_Control, _cruise_throttle, 30.0f),

    // @Param: ROLL_DIFF
    // @DisplayName: AUV differential elevator roll compensation
    // @Description: Enable differential elevator deflection to counter roll torque
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ROLL_DIFF", 11, AUV_Control, _roll_diff, 0),

    AP_GROUPEND
};

AUV_Control::AUV_Control()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AUV_Control::is_enabled() const
{
    return (_enable.get() != 0) || sub.motors.is_auv_frame();
}

void AUV_Control::init()
{
    if (!is_enabled()) {
        return;
    }

    // Configure default functions for channels 1..5 if not set
    SRV_Channels::set_default_function(CH_1, SRV_Channel::k_motor1);
    SRV_Channels::set_default_function(CH_2, SRV_Channel::k_elevator);
    SRV_Channels::set_default_function(CH_3, SRV_Channel::k_elevator);
    SRV_Channels::set_default_function(CH_4, SRV_Channel::k_rudder);
    SRV_Channels::set_default_function(CH_5, SRV_Channel::k_rudder);

    // Set angle range in centidegrees
    const float fin_max_cd = _fin_max_deg.get() * 100.0f;
    SRV_Channels::set_angle(SRV_Channel::k_elevator, fin_max_cd);
    SRV_Channels::set_angle(SRV_Channel::k_rudder, fin_max_cd);
}

float AUV_Control::get_speed_scaler() const
{
    float speed = 0.0f;
    Vector3f vel;
    if (AP::ahrs().get_velocity_NED(vel)) {
        speed = vel.length();
    } else {
        // Fallback open-loop estimation: speed proportional to forward thrust
        speed = fabsf(sub.motors.get_forward()) * _speed_nom.get();
    }

    if (speed < 0.05f) {
        return 1.0f; // Nominal scaling at very low or zero speed
    }

    const float spd_nom = _speed_nom.get();
    const float spd_min = MAX(_speed_min.get(), 0.1f);
    const float eff_spd = MAX(speed, spd_min);

    return constrain_float(spd_nom / eff_spd, _speed_scale_min.get(), _speed_scale_max.get());
}

void AUV_Control::output_control_surfaces()
{
    if (!is_enabled()) {
        return;
    }

    // When disarmed, center servos to neutral (0 scaled = SERVOn_TRIM)
    if (!sub.motors.armed()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0.0f);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0.0f);
        return;
    }

    // Demands from rate controller or pilot sticks [-1.0, 1.0]
    const float pitch_in = sub.motors.get_pitch() + sub.motors.get_pitch_ff();
    const float yaw_in   = sub.motors.get_yaw() + sub.motors.get_yaw_ff();
    const float roll_in  = sub.motors.get_roll() + sub.motors.get_roll_ff();

    // Calculate hydrodynamic dynamic-pressure speed scaler
    const float scaler = get_speed_scaler();

    // Scale to centidegrees
    const float fin_max_cd = _fin_max_deg.get() * 100.0f;
    const float pitch_cd = constrain_float(pitch_in * scaler * fin_max_cd, -fin_max_cd, fin_max_cd);
    const float yaw_cd   = constrain_float(yaw_in * scaler * fin_max_cd, -fin_max_cd, fin_max_cd);

    if (_tail_type.get() == TAIL_CRUCIFORM) {
        // Standard cruciform (+): horizontal elevators, vertical rudders
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_cd);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_cd);

        // Optional differential elevator for roll trim
        if (_roll_diff.get() && SRV_Channels::function_assigned(SRV_Channel::k_elevon_left)) {
            const float roll_cd = constrain_float(roll_in * scaler * fin_max_cd * 0.5f, -fin_max_cd, fin_max_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left, pitch_cd + roll_cd);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right, pitch_cd - roll_cd);
        }
    } else if (_tail_type.get() == TAIL_X) {
        // X-Tail (x): 4 diagonal fins at 45 degrees
        // Fin 1: top-right, Fin 2: bottom-right, Fin 3: bottom-left, Fin 4: top-left
        const float fin_tr = 0.7071f * (pitch_cd + yaw_cd);
        const float fin_br = 0.7071f * (pitch_cd - yaw_cd);
        const float fin_bl = 0.7071f * (-pitch_cd - yaw_cd);
        const float fin_tl = 0.7071f * (-pitch_cd + yaw_cd);

        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left, fin_tl);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right, fin_tr);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left, fin_bl);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right, fin_br);
    }
}

bool AUV_Control::calculate_depth_pitch_cd(float target_z_cm, float curr_z_cm, float &pitch_cd)
{
    if (!is_enabled()) {
        return false;
    }

    // Depth tracking error (NED frame: target - current)
    // When target depth is deeper than current, error_z_m > 0 -> pitch down (negative pitch)
    const float error_z_m = (target_z_cm - curr_z_cm) * 0.01f;

    const float max_pitch_cd = _pitch_max_deg.get() * 100.0f;
    pitch_cd = constrain_float(-error_z_m * _pitch_gain.get() * 100.0f, -max_pitch_cd, max_pitch_cd);

    return true;
}
