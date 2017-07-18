#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>  // for MAV_SEVERITY
#include "defines.h"

class Mode
{
public:

    // Constructor
    Mode();

    // enter this mode, returns false if we failed to enter
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual uint32_t mode_number() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // calculates the amount of throttle that should be output based
    // on things like proximity to corners and current speed
    virtual void calc_throttle(float target_speed);

    // called to determine where the vehicle should go next, and how it should get there
    virtual void update_navigation() { }  // most modes don't navigate

    //
    // attributes of the mode
    //

    // return if in non-manual mode : AUTO, GUIDED, RTL
    virtual bool is_autopilot_mode() const { return false; }

    // returns true if steering is directly controlled by RC
    virtual bool manual_steering() const { return false; }

    // returns true if the throttle is controlled automatically
    virtual bool auto_throttle() { return is_autopilot_mode(); }

    // return true if throttle should be supressed in event of a
    // FAILSAFE_EVENT_THROTTLE
    virtual bool failsafe_throttle_suppress() const { return true; }

    //
    // attributes for mavlink system status reporting
    //

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // true if heading is controlled
    virtual bool attitude_stabilized() const { return true; }

    // Navigation control variables
    // The instantaneous desired lateral acceleration in m/s/s
    float lateral_acceleration;

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // calculate steering angle given a desired lateral acceleration
    virtual void calc_nav_steer();

    // calculate desired lateral acceleration using current location and target held in next_WP
    virtual void calc_lateral_acceleration();

    // calculate desired lateral acceleration
    void calc_lateral_acceleration(const struct Location &last_wp, const struct Location &next_WP);

    // calculate pilot input to nudge throttle up or down
    int16_t calc_throttle_nudge();

    // references to avoid code churn:
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;  // TODO : Pointer reference ?
    class RC_Channel *&channel_throttle;
    class AP_Mission &mission;
};


class ModeAuto : public Mode
{
public:

    uint32_t mode_number() const override { return AUTO; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void calc_throttle(float target_speed) override;
    void update_navigation() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

protected:

    bool _enter() override;
    void _exit() override;
    void calc_nav_steer() override;
    void calc_lateral_acceleration() override;

private:

    bool check_trigger(void);

    // this is set to true when auto has been triggered to start
    bool auto_triggered;
};


class ModeGuided : public Mode
{
public:

    uint32_t mode_number() const override { return GUIDED; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void update_navigation() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

    enum GuidedMode {
        Guided_WP,
        Guided_Angle,
        Guided_Velocity
    };

    // Guided
    GuidedMode guided_mode;  // stores which GUIDED mode the vehicle is in

protected:

    bool _enter() override;
};


class ModeHold : public Mode
{
public:

    uint32_t mode_number() const override { return HOLD; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool attitude_stabilized() const override { return false; }
};


class ModeManual : public Mode
{
public:

    uint32_t mode_number() const override { return MANUAL; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool manual_steering() const override { return true; }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};


class ModeLearning : public ModeManual
{
public:

    uint32_t mode_number() const override { return LEARNING; }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
};


class ModeRTL : public Mode
{
public:

    uint32_t mode_number() const override { return RTL; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void update_navigation() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }
    bool failsafe_throttle_suppress() const override { return false; }

protected:

    bool _enter() override;
};


class ModeSteering : public Mode
{
public:

    uint32_t mode_number() const override { return STEERING; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
};

class ModeInitializing : public Mode
{
public:

    uint32_t mode_number() const override { return INITIALISING; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
};
