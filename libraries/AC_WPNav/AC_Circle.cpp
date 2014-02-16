/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_Circle.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] PROGMEM = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _last_update(0),
    _angle(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// set_circle_center in cm from home
void AC_Circle::set_center(const Vector3f& position)
{
    _center = position;

	// To-Do: set target position, angle, etc so that copter begins circle from closest point to stopping point
	_pos_control.set_pos_target(_inav.get_position());

	// To-Do: set _pos_control speed and accel

    // calculate velocities
    calc_velocities();
}

/// init_center in cm from home using stopping point and projecting out based on the copter's heading
void AC_Circle::init_center()
{
    Vector3f stopping_point;

    // get reasonable stopping point
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);

    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + _radius * _ahrs.cos_yaw();
    _center.y = stopping_point.y + _radius * _ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // update pos_control target to stopping point
    _pos_control.set_pos_target(stopping_point);

    // calculate velocities
    calc_velocities();
}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update) / 1000.0f;

    // update circle position at 10hz
    if (dt > 0.095f) {

        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _last_update = now;

        // ramp up angular velocity to maximum
        if (_rate >= 0) {
            if (_angular_vel < _angular_vel_max) {
                _angular_vel += _angular_accel * dt;
                _angular_vel = constrain_float(_angular_vel, 0, _angular_vel_max);
            }
        }else{
            if (_angular_vel > _angular_vel_max) {
                _angular_vel += _angular_accel * dt;
                _angular_vel = constrain_float(_angular_vel, _angular_vel_max, 0);
            }
        }

        // update the target angle and total angle traveled
        float angle_change = _angular_vel * dt;
        _angle += angle_change;
        _angle = wrap_PI(_angle);
        _angle_total += angle_change;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if (_radius != 0.0) {
            // calculate target position
            Vector3f target;
            target.x = _center.x + _radius * cosf(-_angle);
            target.y = _center.y - _radius * sinf(-_angle);
            target.z = _pos_control.get_alt_target();

            // update position controller target
            _pos_control.set_pos_target(target);

            // heading is 180 deg from vehicles target position around circle
            _yaw = wrap_PI(_angle-PI) * AC_CIRCLE_DEGX100;
        }else{
            // set target position to center
            Vector3f target;
            target.x = _center.x;
            target.y = _center.y;
            target.z = _pos_control.get_alt_target();

            // update position controller target
            _pos_control.set_pos_target(target);

            // heading is same as _angle but converted to centi-degrees
            _yaw = _angle * AC_CIRCLE_DEGX100;
        }

        // trigger position controller on next update
        _pos_control.trigger_xy();
    }

    // run loiter's position to velocity step
    _pos_control.update_pos_controller(false);
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities()
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        _angular_vel_max = ToRad(_rate);
        _angular_accel = _angular_vel_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
        _angle = wrap_PI(_ahrs.yaw-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = min(_pos_control.get_speed_xy(), safe_sqrt(0.5f*_pos_control.get_accel_xy()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrain_float(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = _pos_control.get_accel_xy()/_radius;
        if (_rate < 0.0f) {
            _angular_accel = -_angular_accel;
        }
    }

    // initialise other variables
    _angle_total = 0;
    _angular_vel = 0;
}
