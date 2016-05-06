// this file is #included in avoidance_handler.h

// Avoid by standing very still and hoping the other aircraft avoids
// us
class AvoidanceHandler_HOVER : public AvoidanceHandler__ModeChange {

public:

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_HOVER;
    }

    const char *name() const override { return "HOVER"; }

protected:

    control_mode_t mode() const override { return RTL; }

private:

};
