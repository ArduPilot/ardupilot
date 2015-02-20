// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))


// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT       (1<<0)          // 1 = Relative
                                                        // altitude

// Controller modes
// ----------------

enum ControlMode {
    MANUAL=0,
    STOP=1,
    SCAN=2,
    AUTO=10,
    INITIALISING=16
};

enum ServoType {
    SERVO_TYPE_POSITION=0,
    SERVO_TYPE_ONOFF=1
};

#endif // _DEFINES_H

