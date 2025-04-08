#pragma once

#include <AP_Common/AP_Common.h>

class SplineCurve {

public:

    // set maximum speed and acceleration
    void set_speed_accel(float speed_xy, float speed_up, float speed_down,
                         float accel_xy, float accel_z);

    // set origin and destination using position vectors (offset from EKF origin)
    // origin_vel is vehicle velocity at origin (in NEU frame)
    // destination_vel is vehicle velocity at destination (in NEU frame)
    void set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel);

    // move target location along track from origin to destination
    // target_pos is updated with the target position from EKF origin in NEU frame
    // target_vel is updated with the target velocity in NEU frame
    void advance_target_along_track(float dt, Vector3f &target_pos, Vector3f &target_vel);

    // returns true if vehicle has reached destination
    bool reached_destination() const WARN_IF_UNUSED { return _reached_destination; }

    // returns the unscaled destination velocity vector
    const Vector3f& get_destination_vel() WARN_IF_UNUSED { return _destination_vel; }

    // returns maximum speed at origin
    float get_origin_speed_max() const WARN_IF_UNUSED { return _origin_speed_max; }

    // get or set maximum speed at destination
    float get_destination_speed_max() const WARN_IF_UNUSED { return _destination_speed_max; }
    void set_destination_speed_max(float destination_speed_max) { _destination_speed_max = MIN(_destination_speed_max, destination_speed_max); }

private:

    // calculate the spline delta time for a given delta distance
    // returns the spline position and velocity and maximum speed and acceleration the vehicle can travel without exceeding acceleration limits
    void calc_dt_speed_max(float time, float distance_delta, float &spline_dt, Vector3f &target_pos, Vector3f &spline_vel_unit, float &speed_max, float &accel_max);

    // recalculate hermite_spline_solution grid
    void update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel);

    // calculate target position and velocity from given spline time
    // time is a value from 0 to 1
    // position is updated with target position as an offset from EKF origin in NEU frame
    // velocity is updated with the unscaled velocity
    // relies on set_origin_and_destination having been called to update_solution
    void calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity, Vector3f &acceleration, Vector3f &jerk);

    // interval variables
    Vector3f    _origin;                // origin offset (in NEU frame) from EKF
    Vector3f    _destination;           // destination offset (in NEU frame) from EKF
    Vector3f    _origin_vel;            // the target velocity vector in NEU frame at the origin of the spline segment
    Vector3f    _destination_vel;       // the target velocity vector in NEU frame at the destination point of the spline segment
    Vector3f    _hermite_solution[4];   // array describing path between origin and destination
    float       _time;                  // current spline time (between 0 and 1) between origin and destination
    float       _speed_xy;              // maximum horizontal speed
    float       _speed_up;              // maximum speed upwards
    float       _speed_down;            // maximum speed downwards
    float       _accel_xy;              // maximum horizontal acceleration
    float       _accel_z;               // maximum vertical acceleration
    float       _origin_speed_max;      // maximum speed at origin
    float       _destination_speed_max; // maximum speed at destination
    bool        _reached_destination;   // true once vehicle has reached destination
    bool        _zero_length;           // true if spline is zero length
};
