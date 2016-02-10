// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

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
    SERVO_TEST=3,
    AUTO=10,
    INITIALISING=16
};

enum ServoType {
    SERVO_TYPE_POSITION=0,
    SERVO_TYPE_ONOFF=1,
    SERVO_TYPE_CR=2
};

#endif // _DEFINES_H

