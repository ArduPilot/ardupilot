// this file is #included in avoidance_handler.h

#include "defines.h"

// Avoid a collision by following the TCAS protocol
class AvoidanceHandler_TCAS : public AvoidanceHandler__AVOID {

public:

    AvoidanceHandler_TCAS(const AP_AHRS &ahrs) :
        AvoidanceHandler__AVOID(ahrs)
        { }

    MAV_COLLISION_ACTION mav_avoidance_action() const override {
        return MAV_COLLISION_ACTION_TCAS;
    }

    const char *name() const { return "TCAS"; }

protected:

    bool new_destination(Vector3f &newdest_neu) override;

private:

    // different types of TCAS resolution
    typedef enum {
        tcas_resolution_descend = 2,
        tcas_resolution_ascend = 3,
        tcas_resolution_neutral = 4, // not technically a TCAS resolution... FIXME
    } tcas_resolution_t ;

    // returns the action we should take based on the TCAS algorithm
    tcas_resolution_t tcas_resolution();

    // returns an identifier for this aircraft corresponding to
    // supplied SRC.  For example, for if the source is mavlink then
    // the identifier would be this aircraft's mavlink src id
    uint32_t my_src_id(const MAV_COLLISION_SRC src) const;

};
