#include "avoidance_handler.h"

#include "Copter.h"

uint32_t AvoidanceHandler_TCAS::my_src_id(const MAV_COLLISION_SRC src) const
{
    switch (src) {
    case MAV_COLLISION_SRC_ADSB:
        // if we are actively broadcasting ADSB then we should have an ID.  Return that here
        return 0; // should we return MAX_UINT32/2 here?
    case MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT:
        return mavlink_system.sysid;
    case MAV_COLLISION_SRC_ENUM_END:
        break;
    }
    return 0;
}

AvoidanceHandler_TCAS::tcas_resolution_t AvoidanceHandler_TCAS::tcas_resolution()
{
    if (_threat == nullptr) {
        // why where we called?!
        return tcas_resolution_descend;
    }

    Location my_loc;
    if (!_ahrs.get_position(my_loc)) {
        // we should not get to here!  If we don't know our position
        // we probably can't know if there are any threats, for starters!
        internal_error();
        return tcas_resolution_descend;
    }
    const uint32_t my_alt = my_loc.alt;
    const uint32_t threat_alt = _threat->_location.alt;
    // ::fprintf(stderr, "heights: me=%d threat=%d\n", my_alt, threat_alt);
   if ((int)(my_alt/100) == (int)(threat_alt/100)) {
       // the aircraft are within 1m of each other.  Treat this as
       // equal

       // we now compare src ids.  Note thatif these are coming from
       // different sources (consider ADSB and MAVLink sources), then
       // this comparison doesn't make a great deal of sense.
       if (_threat->src_id < my_src_id(_threat->src)) {
           return tcas_resolution_ascend;
       } else if (_threat->src_id > my_src_id(_threat->src)) {
           return tcas_resolution_descend;
       }
       // We have the same src id as the threat.  That's probably bad.
       // Flip a coin as to whether to go up or down.
       return ((my_alt % 2 == 0) ? tcas_resolution_descend : tcas_resolution_ascend);
   }

    if (my_loc.alt > _threat->_location.alt) {
        return tcas_resolution_ascend;
    }
    return tcas_resolution_descend;
}

// execute TCAS avoidance
// Traffic Collision Avoidance System, if you were wondering
// Nice writeup: http://wiki.paparazziuav.org/wiki/MultiUAV
bool AvoidanceHandler_TCAS::new_destination(Vector3f &newdest_neu)
{
    tcas_resolution_t rr = tcas_resolution();
    // apparently we haven't gone up or down far enough yet...
    // dest here is NEU!
    const Vector3f dest = copter.wp_nav.get_wp_destination();
    float delta_cm = 1000.00;
    if (rr == tcas_resolution_descend) {
        delta_cm = -delta_cm;
    }
    // ::fprintf(stderr, "tcas_resolution=%d delta_cm=%f\n", rr, delta_cm);
    float new_alt_cm = 0;
    Vector3f my_pos;
    if (_ahrs.get_relative_position_NED(my_pos)) {
        new_alt_cm = -my_pos[2]*100.0f + delta_cm;
    } else {
        // We don't know our current height.  How did we get here?!
        internal_error();
    }
    if (new_alt_cm < _minimum_guided_height * 100) {
        new_alt_cm = _minimum_guided_height * 100.0f;
    }
    newdest_neu[0] = dest[0];
    newdest_neu[1] = dest[1];
    newdest_neu[2] = new_alt_cm;

    return true;
}
