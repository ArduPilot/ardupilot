#pragma once

#include "AR_WPNav.h"

class AR_WPNav_OA : public AR_WPNav {
public:

    // re-use parent's constructor
    using AR_WPNav::AR_WPNav;

    // update navigation
    void update(float dt) override;

    // set desired location and (optionally) next_destination
    // next_destination should be provided if known to allow smooth cornering
    bool set_desired_location(const Location &destination, Location next_destination = Location(), bool oa_state = false) override WARN_IF_UNUSED;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    bool reached_destination() const override;

    // get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_oa_origin() const override;

    // get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_oa_destination() const override;

    // return the heading (in centi-degrees) to the next waypoint accounting for OA, (used by sailboats)
    float oa_wp_bearing_cd() const override;

private:

    // update distance and bearing from vehicle's current position to destination
    void update_oa_distance_and_bearing_to_destination();

    // object avoidance variables
    bool _oa_active;                // true if we should use alternative destination to avoid obstacles
    Location _origin_oabak;         // backup of _origin so it can be restored when oa completes
    Location _destination_oabak;    // backup of _desitnation so it can be restored when oa completes
    Location _oa_origin;            // intermediate origin during avoidance
    Location _oa_destination;       // intermediate destination during avoidance
    float _oa_distance_to_destination; // OA (object avoidance) distance from vehicle to _oa_destination in meters
    float _oa_wp_bearing_cd;        // OA adjusted heading to _oa_destination in centi-degrees
};
