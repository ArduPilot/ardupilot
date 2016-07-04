#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

class AP_Avoidance;

class AvoidanceHandler {
public:

    // enter is called once when starting this avoidance action
    virtual bool enter(class AP_Avoidance::Obstacle &threat, class AvoidanceHandler *old_handler);
    // update is called perdiodically while undertaking this avoidance
    // action
    virtual bool update() { return true; };
    // exit is called when the problem is either resolved or being
    // handed off to a different handler
    virtual void exit() { }

    // mav_avoidance_action returns the closest entry in the MAV
    // enumeration MAV_COLLISION_ACTION that this avoidance handler
    // corresponds to
    virtual MAV_COLLISION_ACTION mav_avoidance_action() const = 0;

    // name returns a human-readable string for this avoidance handler
    virtual const char * name() const = 0;

    // v1 is NED
    static Vector3f perpendicular_xyz(const Location &p1, const Vector3f &v1, const Location &p2);
    static Vector2f perpendicular_xy(const Location &p1, const Vector3f &v1, const Location &p2);

protected:

    AP_Avoidance::Obstacle *_threat;
    virtual void internal_error();

    // lowest height guided avoidance will send the aircraft, in metres
    const uint8_t _minimum_guided_height = 10;

    bool new_destination_perpendicular(Vector3f &newdest_neu, const AP_AHRS &_ahrs, const uint8_t _minimum_avoid_height, const float wp_speed_xy, const float wp_speed_z);

private:

    // speed below which we will fly directly away from a threat
    // rather than perpendicular to its velocity:
    const uint8_t _low_velocity_threshold = 1; // metres/second

};

// Ignore any collision threat
class AvoidanceHandler_NONE : public AvoidanceHandler {

public:

    bool update() { return true; };

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_NONE;
    }

    const char *name() const { return "NONE"; }

private:

};

// Report any collision threat
class AvoidanceHandler_REPORT : public AvoidanceHandler {

public:

    bool update() { return true; };

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_REPORT;
    }

    const char *name() const { return "REPORT"; }

private:

};
