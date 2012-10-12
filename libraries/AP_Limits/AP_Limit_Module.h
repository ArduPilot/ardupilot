// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/// @brief	Imposes limits on location (geofence), altitude and other parameters
///         Each breach will trigger an action or set of actions to recover.
//          Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef __AP_LIMIT_MODULE_H__
#define __AP_LIMIT_MODULE_H__

#include <AP_Common.h>
#include <AP_Param.h>

// The module IDs are defined as powers of 2, to make a bit-field
enum moduleid {
    // not a module - used to set the "zero" value
    AP_LIMITS_NULLMODULE = 0,
    // a GPS-lock-required limit
    AP_LIMITS_GPSLOCK    = (1 << 0),
    // fence within a set of coordinates
    AP_LIMITS_GEOFENCE   = (1 << 1),
    // altitude limits
    AP_LIMITS_ALTITUDE   = (1 << 2)
};

extern const prog_char_t *get_module_name(enum moduleid i);

// an integer type big enough to fit a bit field for all modules.
// We have 3 modules, so 8-bit int is enough.
typedef uint8_t LimitModuleBits;

class AP_Limit_Module {

public:
    // initialize a new module
    AP_Limit_Module(enum moduleid id);
    // initialize self
    bool        init();

    virtual moduleid                get_module_id();
    virtual bool                    enabled();
    virtual bool                    required();

    // return next module in linked list
    virtual AP_Limit_Module *       next();

    // link the next module in the linked list
    virtual void                    link(AP_Limit_Module *m);

    // trigger check function
    virtual bool                    triggered();

    // recovery action
    virtual void                    action();

protected:
    // often exposed as a MAVLink parameter
    AP_Int8                         _enabled;

    AP_Int8                         _required;
    AP_Int8                         _triggered;

private:
    enum moduleid                   _id;
    AP_Limit_Module *               _next;
};


#endif // __AP_LIMIT_MODULE_H__
