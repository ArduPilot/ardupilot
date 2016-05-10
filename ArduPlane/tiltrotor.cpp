// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */


/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = (tilt.max_rate_dps.get() * plane.G_Dt) / 90.0f;
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_motor_tilt, 1000 * tilt.current_tilt);

    // setup tilt compensation
    motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&QuadPlane::tilt_compensate, void, float *, uint8_t));
}

/*
  update motor tilt
 */
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
        return;
    }

    // default to inactive
    tilt.motors_active = false;

    // the maximum rate of throttle change
    float max_change = (tilt.max_rate_dps.get() * plane.G_Dt) / 90.0f;
    
    if (!in_vtol_mode() && !assisted_flight) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
        tiltrotor_slew(1);

        float new_throttle = plane.channel_throttle->get_servo_out()*0.01f;
        if (tilt.current_tilt < 1) {
            tilt.current_throttle = constrain_float(new_throttle,
                                                    tilt.current_throttle-max_change,
                                                    tilt.current_throttle+max_change);
        } else {
            tilt.current_throttle = new_throttle;
        }
        if (!hal.util->get_soft_armed()) {
            tilt.current_throttle = 0;
        } else {
            motors->output_motor_mask(tilt.current_throttle, (uint8_t)tilt.tilt_mask.get());
            // prevent motor shutdown
            tilt.motors_active = true;
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    tilt.current_throttle = constrain_float(motors->get_throttle(),
                                            tilt.current_throttle-max_change,
                                            tilt.current_throttle+max_change);
    
    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 3 strategies we will use:

      1) in QSTABILIZE or QHOVER the angle will be set to zero. This
         enables these modes to be used as a safe recovery mode.

      2) in fixed wing assisted flight or velocity controlled modes we
         will set the angle based on the demanded forward throttle,
         with a maximum tilt given by Q_TILT_MAX. This relies on
         Q_VFWD_GAIN being set

      3) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */
    if (plane.control_mode == QSTABILIZE ||
        plane.control_mode == QHOVER) {
        tiltrotor_slew(0);
        return;
    }

    if (assisted_flight &&
        transition_state >= TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        tiltrotor_slew(1);
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
        float settilt = constrain_float(plane.channel_throttle->get_servo_out() / 50.0f, 0, 1);
        tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
    }
}


/*
  compensate for tilt in a set of motor outputs

  Compensation is of two forms. The first is to apply _tilt_factor,
  which is a compensation for the reduces vertical thrust when
  tilted. This is supplied by set_motor_tilt_factor().

  The second compensation is to use equal thrust on all tilted motors
  when _tilt_equal_thrust is true. This is used when the motors are
  tilted by a large angle to prevent the roll and yaw controllers from
  causing instability. Typically this would be used when the motors
  are tilted beyond 45 degrees. At this angle it is assumed that roll
  control can be achieved using fixed wing control surfaces and yaw
  control with the remaining multicopter motors (eg. tricopter tail).

  By applying _tilt_equal_thrust the tilted motors effectively become
  a single pitch control motor.
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    float tilt_factor;
    if (tilt.current_tilt > 0.98f) {
        tilt_factor = 1.0 / cosf(radians(0.98f*90));
    } else {
        tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
    }

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    uint8_t mask = tilt.tilt_mask;
    
    // apply _tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (mask & (1U<<i)) {
            thrust[i] *= tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (mask & (1U<<i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the tilted motors then reduce all
    // motors to keep them in proportion to the original thrust. This
    // helps maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}
