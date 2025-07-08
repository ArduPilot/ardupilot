#pragma once

#include "Blimp.h"
class Parameters;
class ParametersG2;

class GCS_Blimp;

class Mode
{

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        LAND =          0,  // currently just stops moving
        MANUAL =        1,  // manual control
        VELOCITY =      2,  // velocity mode
        LOITER =        3,  // loiter mode (position hold)
        RTL =           4,  // rtl
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // child classes should override these methods
    virtual bool init(bool ignore_checks)
    {
        return true;
    }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const
    {
        return false;
    }
    virtual bool has_user_takeoff(bool must_navigate) const
    {
        return false;
    }
    virtual bool in_guided_mode() const
    {
        return false;
    }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;

    virtual bool is_landing() const
    {
        return false;
    }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const
    {
        return false;
    }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc)
    {
        return false;
    };
    virtual int32_t wp_bearing() const
    {
        return 0;
    }
    virtual float wp_distance_m() const
    {
        return 0.0f;
    }
    virtual float crosstrack_error() const
    {
        return 0.0f;
    }

    void update_navigation();

    // pilot input processing
    void get_pilot_input(Vector3f &pilot, float &yaw);

protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    Fins *&motors;
    Loiter *&loiter;
    RC_Channel *&channel_right;
    RC_Channel *&channel_front;
    RC_Channel *&channel_up;
    RC_Channel *&channel_yaw;
    float &G_Dt;

public:
    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Blimp &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    const char *name() const override
    {
        return "MANUAL";
    }
    const char *name4() const override
    {
        return "MANU";
    }

    Mode::Number number() const override { return Mode::Number::MANUAL; }

private:

};

class ModeVelocity : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "VELOCITY";
    }
    const char *name4() const override
    {
        return "VELY";
    }

    Mode::Number number() const override { return Mode::Number::VELOCITY; }

private:

};

class ModeLoiter : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "LOITER";
    }
    const char *name4() const override
    {
        return "LOIT";
    }

    Mode::Number number() const override { return Mode::Number::LOITER; }

private:
    Vector3f target_pos;
    float target_yaw;
};

class ModeLand : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return false;
    };
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    const char *name() const override
    {
        return "LAND";
    }
    const char *name4() const override
    {
        return "LAND";
    }

    Mode::Number number() const override { return Mode::Number::LAND; }

private:

};

class ModeRTL : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "RTL";
    }
    const char *name4() const override
    {
        return "RTL";
    }

    Mode::Number number() const override { return Mode::Number::RTL; }

};
