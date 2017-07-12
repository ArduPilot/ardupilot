#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>  // for MAV_SEVERITY
#include "defines.h"

class Mode
{
public:

    Mode();

    // returns false if we failed to enter this mode
    bool enter();
    // perform any cleanups required:
    void exit();

    // called at the main loop rate:
    virtual void update() = 0;

    // returns true if steering calculations should be logged:
    bool should_log_steering() const { return !manual_steering(); }

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // called to determine where the vehicle should go next, and how
    // it should get there
    virtual void update_navigation() { }  // most modes don't navigate

    // returns a unique number specific to this mode
    virtual uint32_t mode_number() const = 0;
    // return the mode number for mavlink msg
    uint32_t mavlink_custom_mode() const { return mode_number(); }  // for heartbeat.custom_mode
    // return if in non-manual mode : AUTO, GUIDED, RTL
    virtual bool is_autopilot_mode() const { return false; }

    virtual bool angular_rate_control() const { return true; }
    virtual bool attitude_stabilized() const { return true; }
    // returns true if the throttle is controlled automatically
    virtual bool auto_throttle() { return is_autopilot_mode(); }
    // returns true if the vehicle can attempt to turn on spot
    virtual bool allow_pivot_steering() const { return false; }
    // returns true if steering is directly controlled by RC:
    virtual bool manual_steering() const { return false; }

    // return true if throttle should be supressed in event of a
    // FAILSAFE_EVENT_THROTTLE
    virtual bool failsafe_throttle_suppress() const { return true; }

    // Navigation control variables
    // The instantaneous desired lateral acceleration in m/s/s
    float lateral_acceleration;

    // calculates the amount of throttle that should be output based
    // on things like proximity to corners and current speed
    virtual void calc_throttle(float target_speed);

protected:

    virtual void calc_nav_steer();

    virtual void calc_lateral_acceleration();
    void calc_lateral_acceleration(const struct Location &last_wp, const struct Location &next_WP);
    // subclasses override this to do what they need to do:
    virtual bool _enter() { return true; }
    // subclasses override this to do what they need to do:
    virtual void _exit() { return; }

    // references to avoid code churn:
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;  // TODO : Pointer reference ?
    class RC_Channel *&channel_throttle;
    class AP_Mission &mission;


private:

    virtual bool stickmixing_enabled() const { return true; }
};


class ModeAuto : public Mode
{
public:

    void update() override;

    uint32_t mode_number() const override { return AUTO; }
    bool is_autopilot_mode() const override { return true; }

    bool allow_pivot_steering() const override { return true; }

    void calc_nav_steer() override;
    bool failsafe_throttle_suppress() const override { return false; }

    void calc_throttle(float target_speed) override;
    void calc_lateral_acceleration() override;

protected:

    bool _enter() override;
    void _exit() override;

    void update_navigation() override;

private:

    bool check_trigger(void);

    // this is set to true when auto has been triggered to start
    bool auto_triggered;
};

class ModeGuided : public Mode
{
public:

    void update() override;

    uint32_t mode_number() const override { return GUIDED; }
    bool is_autopilot_mode() const override { return true; }

    bool allow_pivot_steering() const override { return true; }

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
    void update_navigation() override;
};


class ModeHold : public Mode
{
public:

    void update() override;

    uint32_t mode_number() const override { return HOLD; }
    bool angular_rate_control() const override { return false; }
    bool attitude_stabilized() const override { return false; }
};


class ModeManual : public Mode
{
public:

    void update() override;
    bool manual_steering() const override { return true; }

    uint32_t mode_number() const override { return MANUAL; }
    bool has_manual_input() const override { return true; }
    bool angular_rate_control() const override { return false; }
    bool attitude_stabilized() const override { return false; }
};


class ModeLearning : public ModeManual
{
public:

    uint32_t mode_number() const override { return LEARNING; }
    bool has_manual_input() const override { return true; }
};


class ModeRTL : public Mode
{
public:

    void update() override;

    uint32_t mode_number() const override { return RTL; }
    bool is_autopilot_mode() const override { return true; }

    bool allow_pivot_steering() const override { return true; }

    bool failsafe_throttle_suppress() const override { return false; }

protected:

    bool _enter() override;
    void update_navigation() override;
};

class ModeSteering : public Mode
{
public:

    void update() override;

    uint32_t mode_number() const override { return STEERING; }
    bool has_manual_input() const override { return true; }
};



class ModeInitializing : public Mode
{
public:

    void update() override { }

    uint32_t mode_number() const override { return INITIALISING; }

    bool has_manual_input() const override { return true; }
    bool stickmixing_enabled() const override { return false; }
    bool angular_rate_control() const override { return false; }
    bool attitude_stabilized() const override { return false; }
};
