#include "AP_NPFG.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

int32_t AP_NPFG::nav_roll_cd(void) const {
    return 0;
}

float AP_NPFG::lateral_acceleration(void) const {
    return 0.0;
}

int32_t AP_NPFG::nav_bearing_cd(void) const {
    return 0;
}

int32_t AP_NPFG::bearing_error_cd(void) const {
    return 0;
}

int32_t AP_NPFG::target_bearing_cd(void) const {
    return 0;
}

float AP_NPFG::crosstrack_error(void) const {
    return 0.0;
}

float AP_NPFG::crosstrack_error_integrator(void) const {
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius) const {
    return 0.0;
}

float AP_NPFG::turn_distance(float wp_radius, float turn_angle) const {
    return 0.0;
}

float AP_NPFG::loiter_radius(const float radius) const {
    return 0.0;
}

void AP_NPFG::update_waypoint(const class Location &prev_WP, const class Location &next_WP, float dist_min) {
}

void AP_NPFG::update_loiter(const class Location &center_WP, float radius, int8_t loiter_direction) {
}

void AP_NPFG::update_heading_hold(int32_t navigation_heading_cd) {
}

void AP_NPFG::update_level_flight(void) {
}

bool AP_NPFG::reached_loiter_target(void) {
    return false;
}

void AP_NPFG::set_data_is_stale(void) {
}

bool AP_NPFG::data_is_stale(void) const {
    return false;
}

void AP_NPFG::set_reverse(bool reverse) {
}
