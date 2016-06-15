// this file is #included in avoidance_handler.h

#include "defines.h"

// Avoid a collision by flying at right-angles to the other aircraft's
// velocity
class AvoidanceHandler_PERPENDICULAR : public AvoidanceHandler__ModeChange {

public:

    AvoidanceHandler_PERPENDICULAR(const AP_AHRS &ahrs) :
        AvoidanceHandler__ModeChange(),
        _ahrs(ahrs)
        { }

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
    }

    const char *name() const override { return "PERPENDICULAR"; }

    bool update() override;

protected:

    FlightMode mode() const override { return GUIDED; };

    bool new_destination(Location &newdest);

private:

    const AP_AHRS &_ahrs;

    uint32_t _last_wp_update;

    Location guided_wp_loc;

    bool do_avoid_navigation();

    const uint8_t _minimum_avoid_height = 20; // metres
};
