// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_Backend.h>

// update_targets_from_rc - updates angle targets using input from receiver
void AP_Mount_Backend::update_targets_from_rc()
{
#define rc_ch(i) RC_Channel::rc_channel(i-1)

    uint8_t roll_rc_in = _frontend.state[_instance]._roll_rc_in;
    uint8_t tilt_rc_in = _frontend.state[_instance]._tilt_rc_in;
    uint8_t pan_rc_in = _frontend.state[_instance]._pan_rc_in;

    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if (_frontend._joystick_speed) {
        // allow pilot speed position input to come directly from an RC_Channel
        if (roll_rc_in && rc_ch(roll_rc_in)) {
            _angle_ef_target_rad.x += rc_ch(roll_rc_in)->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
            constrain_float(_angle_ef_target_rad.x, radians(_frontend.state[_instance]._roll_angle_min*0.01f), radians(_frontend.state[_instance]._roll_angle_max*0.01f));
        }
        if (tilt_rc_in && (rc_ch(tilt_rc_in))) {
            _angle_ef_target_rad.y += rc_ch(tilt_rc_in)->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
            constrain_float(_angle_ef_target_rad.y, radians(_frontend.state[_instance]._tilt_angle_min*0.01f), radians(_frontend.state[_instance]._tilt_angle_max*0.01f));
        }
        if (pan_rc_in && (rc_ch(pan_rc_in))) {
            _angle_ef_target_rad.z += rc_ch(pan_rc_in)->norm_input_dz() * 0.0001f * _frontend._joystick_speed;
            constrain_float(_angle_ef_target_rad.z, radians(_frontend.state[_instance]._pan_angle_min*0.01f), radians(_frontend.state[_instance]._pan_angle_max*0.01f));
        }
    } else {
        // allow pilot position input to come directly from an RC_Channel
        if (roll_rc_in && (rc_ch(roll_rc_in))) {
            _angle_ef_target_rad.x = angle_input_rad(rc_ch(roll_rc_in), _frontend.state[_instance]._roll_angle_min, _frontend.state[_instance]._roll_angle_max);
        }
        if (tilt_rc_in && (rc_ch(tilt_rc_in))) {
            _angle_ef_target_rad.y = angle_input_rad(rc_ch(tilt_rc_in), _frontend.state[_instance]._tilt_angle_min, _frontend.state[_instance]._tilt_angle_max);
        }
        if (pan_rc_in && (rc_ch(pan_rc_in))) {
            _angle_ef_target_rad.z = angle_input_rad(rc_ch(pan_rc_in), _frontend.state[_instance]._pan_angle_min, _frontend.state[_instance]._pan_angle_max);
        }
    }
}

// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t AP_Mount_Backend::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

// returns the angle (radians) that the RC_Channel input is receiving
float AP_Mount_Backend::angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
void AP_Mount_Backend::calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad, bool calc_tilt, bool calc_pan)
{
    float GPS_vector_x = (target.lng-_frontend._current_loc.lng)*cosf(ToRad((_frontend._current_loc.lat+target.lat)*0.00000005f))*0.01113195f;
    float GPS_vector_y = (target.lat-_frontend._current_loc.lat)*0.01113195f;
    float GPS_vector_z = (target.alt-_frontend._current_loc.alt);                 // baro altitude(IN CM) should be adjusted to known home elevation before take off (Set altimeter).
    float target_distance = 100.0f*pythagorous2(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // initialise all angles to zero
    angles_to_target_rad.zero();

    // tilt calcs
    if (calc_tilt) {
        angles_to_target_rad.y = atan2f(GPS_vector_z, target_distance);
    }

    // pan calcs
    if (calc_pan) {
        angles_to_target_rad.z = atan2f(GPS_vector_x, GPS_vector_y);
    }
}
