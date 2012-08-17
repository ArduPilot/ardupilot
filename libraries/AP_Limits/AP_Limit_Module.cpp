// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos


#include <AP_Limit_Module.h>

extern const prog_char_t *get_module_name(enum moduleid i) {

    switch (i) {
    case AP_LIMITS_GPSLOCK:
        return PSTR("GPSLock Limit");
        break;
    case AP_LIMITS_GEOFENCE:
        return PSTR("Geofence Limit");
        break;
    case AP_LIMITS_ALTITUDE:
        return PSTR("Altitude Limit");
        break;
    default:
        return PSTR("ERROR");
        break;
    }
}

AP_Limit_Module::AP_Limit_Module(enum moduleid i) {
    _id = i;
    _next = NULL;
}

bool AP_Limit_Module::init() {

    _triggered = false;
    return true;
};

moduleid AP_Limit_Module::get_module_id() {
    return(_id);
}

bool AP_Limit_Module::enabled() {
    return(_enabled);
}

bool AP_Limit_Module::required() {
    return(_required);
}

AP_Limit_Module *AP_Limit_Module::next() {
    return(_next);
}

void AP_Limit_Module::link(AP_Limit_Module *m) {
    _next = m;
}

bool AP_Limit_Module::triggered() {
    return(_triggered);
}

void AP_Limit_Module::action() {
    // do nothing
}







