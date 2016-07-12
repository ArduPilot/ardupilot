// this file is #included in avoidance_handler.h

#include "defines.h"

// Avoid a collision by flying at right-angles to the other aircraft's
// velocity
class AvoidanceHandler_PERPENDICULAR : public AvoidanceHandler__AVOID {

public:

    AvoidanceHandler_PERPENDICULAR(const AP_AHRS &ahrs) :
        AvoidanceHandler__AVOID(ahrs)
        { }

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
    }

    const char *name() const override { return "PERPENDICULAR"; }

protected:

    bool new_destination(Vector3f &newdest_neu) override;

private:

};
