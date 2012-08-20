// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef AP_LIMIT_MODULE_H_
#define AP_LIMIT_MODULE_H_

#include <AP_Common.h>
#include <AP_Param.h>

// The module IDs are defined as powers of 2, to make a bit-field
enum moduleid {
    AP_LIMITS_NULLMODULE    =       0,                          // not a module - used to set the "zero" value
    AP_LIMITS_GPSLOCK               =       (1 << 0),           // a GPS-lock-required limit
    AP_LIMITS_GEOFENCE              =       (1 << 1),           // fence within a set of coordinates
    AP_LIMITS_ALTITUDE              =       (1 << 2)            // altitude limits
};

extern const prog_char_t *      get_module_name(enum moduleid i);

// an integer type big enough to fit a bit field for all modules. We have 3 modules, so 8-bit int is enough.
typedef uint8_t LimitModuleBits;

class AP_Limit_Module {

public:
    AP_Limit_Module(enum moduleid id);     // initialize a new module


    bool        init();                                                                 // initialize self

    virtual moduleid                get_module_id();
    virtual bool                    enabled();
    virtual bool                    required();
    virtual AP_Limit_Module *       next();      // return next module in linked list

    virtual void                    link(AP_Limit_Module *m);             // link the next module in the linked list

    virtual bool                    triggered();             // trigger check function

    virtual void                    action();              // recovery action

protected:
    AP_Int8                         _enabled;     // often exposed as a MAVLink parameter
    AP_Int8                         _required;
    AP_Int8                         _triggered;

private:
    enum moduleid                   _id;
    AP_Limit_Module *               _next;
};


#endif /* AP_LIMIT_MODULE_H_ */
