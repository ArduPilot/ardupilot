// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Navigation.h
/// @brief   generic navigation controller interface

/*
  This defines a generic interface for navigation controllers. Each
  specific controller should be a subclass of this generic
  interface. All variables used by controllers should be in their
  own class.
 */
#pragma once

#include <AP_Common/AP_Common.h>

class AP_Navigation {
public:
	// return the desired roll angle in centi-degrees to move towards
	// the target waypoint
	virtual int32_t nav_roll_cd(void) const = 0;

	// return the desired lateral acceleration in m/s/s to move towards
	// the target waypoint
	virtual float lateral_acceleration(void) const = 0;

	// note: all centi-degree values returned in AP_Navigation should
	// be wrapped at -18000 to 18000 in centi-degrees.

	// return the tracking bearing that the navigation controller is
	// using in centi-degrees. This is used to display an arrow on
	// ground stations showing the effect of the cross-tracking in the
	// controller
	virtual int32_t nav_bearing_cd(void) const = 0;

	// return the difference between the vehicles current course and
	// the nav_bearing_cd() in centi-degrees. A positive value means
	// the vehicle is tracking too far to the left of the correct
	// bearing.
	virtual int32_t bearing_error_cd(void) const = 0;

	// return the target bearing in centi-degrees. This is the bearing
	// from the vehicles current position to the target waypoint. This
	// should be calculated in the update_*() functions below.
	virtual int32_t target_bearing_cd(void) const = 0;

	// return the crosstrack error in meters. This is the distance in
	// the X-Y plane that we are off the desired track
    virtual float crosstrack_error(void) const = 0;
    virtual float crosstrack_error_integrator(void) const { return 0; }

	
	// return the distance in meters at which a turn should commence
	// to allow the vehicle to neatly move to the next track in the
	// mission when approaching a waypoint. Assumes 90 degree turn
	virtual float turn_distance(float wp_radius) const = 0;

	// return the distance in meters at which a turn should commence
	// to allow the vehicle to neatly move to the next track in the
	// mission when approaching a waypoint
	virtual float turn_distance(float wp_radius, float turn_angle) const = 0;

	// update the internal state of the navigation controller, given
	// the previous and next waypoints. This is the step function for
	// navigation control for path following between two points.  This
	// function is called at regular intervals (typically 10Hz). The
	// main flight code will call an output function (such as
	// nav_roll_cd()) after this function to ask for the new required
	// navigation attitude/steering.
	virtual void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP) = 0;

	// update the internal state of the navigation controller for when
	// the vehicle has been commanded to circle about a point.  This
	// is the step function for navigation control for circling.  This
	// function is called at regular intervals (typically 10Hz). The
	// main flight code will call an output function (such as
	// nav_roll_cd()) after this function to ask for the new required
	// navigation attitude/steering.
	virtual void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction) = 0;

	// update the internal state of the navigation controller, given a
	// fixed heading. This is the step function for navigation control
	// for a fixed heading.  This function is called at regular
	// intervals (typically 10Hz). The main flight code will call an
	// output function (such as nav_roll_cd()) after this function to
	// ask for the new required navigation attitude/steering.
	virtual void update_heading_hold(int32_t navigation_heading_cd) = 0;

	// update the internal state of the navigation controller for
	// level flight on the current heading. This is the step function
	// for navigation control for level flight.  This function is
	// called at regular intervals (typically 10Hz). The main flight
	// code will call an output function (such as nav_roll_cd()) after
	// this function to ask for the new required navigation
	// attitude/steering.
	virtual void update_level_flight(void) = 0;

	// return true if we have reached the target loiter location. This
	// may be a fuzzy decision, depending on internal navigation
	// parameters. For example the controller may return true only
	// when on the circular path around the waypoint, and not when
	// tracking towards the center. This function is only valid when
	// the update_loiter() method is used
	virtual bool reached_loiter_target(void) = 0;

	// notify Navigation controller that a new waypoint has just been
	// processed. This means that until we handle an update_XXX() function
	// the data is stale with old navigation information.
    virtual void set_data_is_stale(void) = 0;

    // return true if a new waypoint has been processed by mission
    // controller but the navigation controller still has old stale data
    // from previous waypoint navigation handling. This gets cleared on
    // every update_XXXXXX() call.
    virtual bool data_is_stale(void) const = 0;

    virtual void set_reverse(bool reverse) = 0;

	// add new navigation controllers to this enum. Users can then
	// select which navigation controller to use by setting the
	// NAV_CONTROLLER parameter
	enum ControllerType {
	    CONTROLLER_DEFAULT      = 0,
		CONTROLLER_L1           = 1
	};
};
