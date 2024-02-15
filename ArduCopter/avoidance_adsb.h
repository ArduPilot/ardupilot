#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Copter-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Copter to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Copter : public AP_Avoidance {
public:

    using AP_Avoidance::AP_Avoidance;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Avoidance_Copter);

private:
    // helper function to set modes and always succeed
    void set_mode_else_try_RTL_else_LAND(Mode::Number mode);

    // get minimum limit altitude allowed on descend
    int32_t get_altitude_minimum() const;

protected:
    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(RecoveryAction recovery_action) override;

    // check flight mode is avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // vertical avoidance handler
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // horizontal avoidance handler
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // perpendicular (3 dimensional) avoidance handler
    bool handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // control mode before avoidance began
    Mode::Number prev_control_mode = Mode::Number::RTL;
};
