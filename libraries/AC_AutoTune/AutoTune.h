/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AutoTune_config.h"
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

class AC_AttitudeControl;
class AC_PosControl;
class AP_AHRS_View;
class AP_InertialNav;
class AP_Motors;

class AutoTune;
class AutoTune_Backend
{
public:
    AutoTune_Backend(
        AutoTune& _frontend,
        AC_AttitudeControl& _attitude_control,
        AC_PosControl& _pos_control,
        AP_AHRS_View& _ahrs_view,
        AP_InertialNav& _inertial_nav,
        AP_Motors& _motors
    );

    virtual void run() = 0;
    virtual void save_tuning_gains() {};
    virtual void stop() = 0;
    virtual void reset() {};
    virtual bool init_internals(bool use_poshold) { return true; };
    virtual bool run_previous_mode() { return false; };

    virtual void update_switch_pos(const RC_Channel::AuxSwitchPos ch_flag) {};

protected:
    AutoTune &frontend;

    AC_AttitudeControl &attitude_control;
    AC_PosControl &pos_control;
    AP_AHRS_View &ahrs_view;
    AP_InertialNav &inertial_nav;
    AP_Motors &motors;
};

class AutoTune
{
    friend class AutoTune_Backend;
    friend class AC_AutoTune;
    friend class AC_AutoTune_Multi;
    friend class AC_AutoTune_Heli;
    friend class AP_Quicktune;

public:

    AutoTune();

    CLASS_NO_COPY(AutoTune);

    void setup(AC_AttitudeControl &_attitude_control,
               AC_PosControl &_pos_control,
               AP_AHRS_View &_ahrs_view,
               AP_InertialNav &_inertial_nav,
               AP_Motors &_motors);

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *backend_var_info;

    bool enabled() const;

    // Backend wrapper functions
    void save_tuning_gains();
    void stop();
    void reset();
    virtual void run();
    bool init_internals(bool use_poshold);
    bool run_previous_mode();
    void update_switch_pos(const RC_Channel::AuxSwitchPos ch_flag);

protected:

    //
    // methods that must be supplied by the vehicle specific subclass
    //
    virtual bool init() = 0;

    // get pilot input for desired climb rate
    virtual float get_pilot_desired_climb_rate_cms() const = 0;

    // get pilot input for designed roll and pitch, and yaw rate
    virtual void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) = 0;

    // init pos controller Z velocity and accel limits
    virtual void init_z_limits() = 0;

#if HAL_LOGGING_ENABLED
    // log PIDs at full rate for during twitch
    virtual void log_pids() = 0;
#endif

    // return true if we have a good position estimate
    virtual bool position_ok() = 0;

private:

    AutoTune_Backend *backend;

    enum class TuneType : uint8_t {
        NONE = 0,
        MULTI = 1,
        HELI = 2,
        QUICKTUNE = 3,
    };
    AP_Enum<TuneType> type;

};

