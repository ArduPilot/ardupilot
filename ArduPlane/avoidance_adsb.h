#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Plane-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Plane to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Plane : public AP_Avoidance {
public:
    AP_Avoidance_Plane(AP_AHRS &ahrs, class AP_ADSB &adsb)
        : AP_Avoidance(ahrs, adsb)
    {
    }

    /* Do not allow copies */
    AP_Avoidance_Plane(const AP_Avoidance_Plane &other) = delete;
    AP_Avoidance_Plane &operator=(const AP_Avoidance_Plane&) = delete;

protected:
    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(uint8_t recovery_action) override;

    // check flight mode is avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // vertical avoidance handler
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // horizontal avoidance handler
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // control mode before avoidance began
    enum Mode::Number prev_control_mode_number = Mode::Number::RTL;
};
