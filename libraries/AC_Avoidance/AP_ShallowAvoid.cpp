#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#include <AP_Math/curve_fitting.h>
#include "AP_ShallowAvoid.h"

static constexpr float OA_SHALLOW_TIMEOUT_MS = 3000;      
static constexpr float OA_SHALLOW_SAMP_DIST_MIN = 5.0f;

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_ShallowAvoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable SpeedDecider
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ShallowAvoid, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SAM_DST
    // @DisplayName: Shallow avoidance sample distance in real time
    // @Description: Shallow avoidance will sample this many meters for model in real time
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAM_DST", 1, AP_ShallowAvoid, _sample_distance, 10),

    // @Param: PRE_DST
    // @DisplayName: Shallow Avoidance predict distance in real time
    // @Description: Shallow Avoidance will predict this may meters in real time
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PRE_DST", 2, AP_ShallowAvoid, _predict_distance, 10),

    // @Param: ANG_MAX
    // @DisplayName: Shallow Avoidance constrain angle maximum
    // @Description: Shallow Avoidance will constrain mission angle error 
    // @Units: deg
    // @Range: 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANG_MAX", 3, AP_ShallowAvoid, _max_lean_angle, 30),

    // @Param: DPT_MIN
    // @DisplayName: Shallow Avoidance water depth minmum
    // @Description: Shallow Avoidance will constrain water depth for mission
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DPT_MIN", 4, AP_ShallowAvoid, _min_water_depth, 1.0),

    // @Param: RADIUS_MIN
    // @DisplayName: Shallow Avoidance shallow radius
    // @Description: Shallow Avoidance will look this meters around current waypoint
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS_MIN", 5, AP_ShallowAvoid, _min_water_radius, 20),

    // @Param: SLOPE_MIN
    // @DisplayName: Shallow Avoidance shallow slope minmum
    // @Description: Shallow Avoidance constrain shallow slope
    // @Units: deg
    // @Range: 1 75
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLOPE_MIN", 6, AP_ShallowAvoid, _min_water_slope, 1),

    // @Param: SAMNUM_MIN
    // @DisplayName: Shallow Avoidance sample number
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAMNUM_MIN", 7, AP_ShallowAvoid, _sample_num_min, 10),

    AP_GROUPEND
};

AP_ShallowAvoid::AP_ShallowAvoid()
{
    // load default parameters from eeprom
    AP_Param::setup_object_defaults(this, var_info); 
}


// Return true if shallow detected and can't reach destination 
bool AP_ShallowAvoid::update(const Location &current_loc, const Location& origin, const Location& destination, const Vector2f &ground_speed_vec, const float dt)
{
    if (!_enable) {
        return false;
    }

    // get ground course
    float ground_course_deg;
    if (ground_speed_vec.length_squared() < sq(0.2f)) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    // Get current mission velocity derivation angle base on current mission line
    const float bearing_to_dest = origin.get_bearing_to(destination) * 0.01f;
    const float lean_angle = wrap_180(ground_course_deg - bearing_to_dest);

    // Convert start and end to offsets from EKF origin
    Vector2f current_NE,origin_NE,destination_NE;
    if (!current_loc.get_vector_xy_from_origin_NE(current_NE) ||
        !origin.get_vector_xy_from_origin_NE(origin_NE) ||
        !destination.get_vector_xy_from_origin_NE(destination_NE)) {
        return false;
    }

    // If vehicle already reach destination,return true
    float mission_distance = origin.get_distance(destination);
    if (current_NE == destination_NE || mission_distance < 1e-3f) {
        return true;
    }
    current_NE *= 0.01f;destination_NE *= 0.01f;origin_NE *= 0.01f;

    _last_update_ms = AP_HAL::millis();

    // record sonar data into queue
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr || !rangefinder->has_data_orient(ROTATION_PITCH_270) || is_zero(dt)) {
        return false;
    }

    // Timeout called check
    if (!is_active()) { 
        _sample_points.clear();
        _last_avoid_flag = false;
    }

    // Special consideration if two waypoints close 
    const float wp_distance = (destination_NE - origin_NE).length();
    if (_last_avoid_flag == true) {
        _last_avoid_flag = false;
        if (wp_distance <= _min_water_radius) {
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Continuous Shallow Warnning");
            return true;
        }
    }
    
    // Push back sensor data into dequeue
    const bool sensor_healthy = (rangefinder->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good);
    const float water_depth_m = rangefinder->distance_orient(ROTATION_PITCH_270,true);
    const float speed = AP::ahrs().groundspeed();
    const float sample_dist = (water_depth_m < _min_water_depth) ? OA_SHALLOW_SAMP_DIST_MIN : _sample_distance;
    std::size_t nw = MAX(sample_dist / (speed * dt), _sample_num_min);
    if (sensor_healthy && (fabsf(lean_angle) <= _max_lean_angle || water_depth_m < _min_water_depth) && speed >= 1.0f) {
        if (_sample_points.size() >= nw) {
            _sample_points.pop_front();
        }
        _sample_points.push_back(water_depth_m);
    } else {
        _sample_points.clear();
        return false;
    }
 
    // Fitting model
    float error_square = 0.0;
    std::deque<float> _coef;
    if (_sample_points.size() < nw) {return false;}

    // Use algebra method to solve least square
    _coef = std::move(least_square(_sample_points, dt, &error_square));
    
   // Predict and shallow check
   const float predict_time = dt * nw + MAX(_predict_distance / speed , 1.0f);
   const float predict_depth = evaluate_polynomial(_coef, predict_time);
   if (predict_depth <= _min_water_depth && _coef[1] <= -tanf(radians(_min_water_slope))) {
        _last_avoid_flag = true;
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Shallow Warnning");
        return true;
   }

    return false;
}

// true if update has been called recently
bool AP_ShallowAvoid::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < OA_SHALLOW_TIMEOUT_MS);
}




