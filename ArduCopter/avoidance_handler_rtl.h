// this file is #included in avoidance_handler.h

// Avoid a collision by running for home
class AvoidanceHandler_RTL : public AvoidanceHandler__ModeChange {

public:

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_RTL;
    }

    const char *name() const override { return "RTL"; }

protected:

    control_mode_t mode() const override { return RTL; }

private:

};
