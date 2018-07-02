#pragma once

#include <stdint.h>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>

#include "defines.h"

#define MODE_NEXT_HEADING_UNKNOWN   99999.0f    // used to indicate to set_desired_location method that next leg's heading is unknown

// pre-define ModeRTL so Auto can appear higher in this file
class ModeRTL;

class Mode
{
public:

    // Auto Pilot modes
    // ----------------
    enum Number {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
        QAUTOTUNE     = 22,
    };

    // Constructor
    Mode();

    // enter this mode, returns false if we failed to enter
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns short text name (up to 4 bytes)
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    //
    // attributes for mavlink system status reporting
    //

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // true if heading is controlled
    virtual bool attitude_stabilized() const { return true; }

    //
    // navigation methods
    //

    // return distance (in meters) to destination
    virtual float get_distance_to_destination() const { return 0.0f; }

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }
};


class ModeAcro : public Mode
{
public:
    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name4() const override { return "RTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name4() const override { return "FBWB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name4() const override { return "CRUS"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name4() const override { return "QSTB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name4() const override { return "QHOV"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name4() const override { return "QLOT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name4() const override { return "QLND"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name4() const override { return "QRTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQAutotune : public Mode
{
public:

    uint32_t mode_number() const override { return QAUTOTUNE; }
    const char *name4() const override { return "QAUTOTUNE"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};
