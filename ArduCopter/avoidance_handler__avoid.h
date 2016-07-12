// this file is #included in avoidance_handler.h

#include "defines.h"

class AvoidanceHandler__AVOID : public AvoidanceHandler__ModeChange {

public:

    AvoidanceHandler__AVOID(const AP_AHRS &ahrs) :
        AvoidanceHandler__ModeChange(),
        _ahrs(ahrs) { }

    bool update() override;

protected:

    const AP_AHRS &_ahrs;

    // new_destination - return new target destination
    // (distance from home, in cm, NEU)
    virtual bool new_destination(Vector3f &newdest_neu) = 0;

    control_mode_t mode() const override { return AVOID; }

    // lowest height avoidance will send the aircraft, in metres
    static const uint8_t _minimum_avoid_height = 10;

private:

    uint32_t _last_wp_update;

    bool do_avoid_navigation();
};
