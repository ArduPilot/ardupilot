
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

#include "AutoTune.h"

#include "AC_AutoTune.h"
#include "AC_AutoTune_Multi.h"
#include "AC_AutoTune_Heli.h"
#include "AC_AutoTune_Quicktune.h"

// Backend constructor
AutoTune_Backend::AutoTune_Backend(AutoTune & _frontend,
                                   AC_AttitudeControl &_attitude_control,
                                   AC_PosControl &_pos_control,
                                   AP_AHRS_View &_ahrs_view,
                                   AP_InertialNav &_inertial_nav,
                                   AP_Motors &_motors):
    frontend(_frontend),
    attitude_control(_attitude_control),
    pos_control(_pos_control),
    ahrs_view(_ahrs_view),
    inertial_nav(_inertial_nav),
    motors(_motors)
{
}

const AP_Param::GroupInfo AutoTune::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Autotune type
    // @Description: Type of auto tune
    // @Values: 0:None, 1:Multi, 2:Heli, 3:QuickTune
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AutoTune, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: _
    // @Path: AC_AutoTune_Multi.cpp, AC_AutoTune_Heli.cpp, AC_AutoTune_Quicktune.cpp
    AP_SUBGROUPVARPTR(backend, "_", 2, AutoTune, backend_var_info),

    AP_GROUPEND
};

const AP_Param::GroupInfo *AutoTune::backend_var_info;

AutoTune::AutoTune()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Setup function allocates the backend
void AutoTune::setup(AC_AttitudeControl &_attitude_control,
                     AC_PosControl &_pos_control,
                     AP_AHRS_View &_ahrs_view,
                     AP_InertialNav &_inertial_nav,
                     AP_Motors &_motors)
{
    switch ((TuneType)type) {
        case TuneType::MULTI:
            backend = NEW_NOTHROW AC_AutoTune_Multi(*this, _attitude_control, _pos_control, _ahrs_view, _inertial_nav, _motors);
            break;

#if defined(FRAME_CONFIG) && (FRAME_CONFIG==HELI_FRAME)
        case TuneType::HELI:
            backend = NEW_NOTHROW AC_AutoTune_Heli(*this, _attitude_control, _pos_control, _ahrs_view, _inertial_nav, _motors);
            break;
#endif

        case TuneType::QUICKTUNE:
            backend = NEW_NOTHROW AP_Quicktune(*this, _attitude_control, _pos_control, _ahrs_view, _inertial_nav, _motors);
            break;

        case TuneType::NONE:
        default:
            break;
    }

    // Load backend specific params
    if (backend != nullptr) {
        AP_Param::load_object_from_eeprom(backend, backend_var_info);
    }

}

bool AutoTune::enabled() const
{
    if (backend == nullptr) {
        return false;
    }
    return type != TuneType::NONE;
}

// Backend wrapper functions
bool AutoTune::init_internals(bool use_poshold)
{
    if (backend == nullptr) {
        return false;
    }
    return backend->init_internals(use_poshold);
}

void AutoTune::save_tuning_gains()
{
    if (backend == nullptr) {
        return;
    }
    backend->save_tuning_gains();
}

void AutoTune::stop()
{
    if (backend == nullptr) {
        return;
    }
    backend->stop();
}

void AutoTune::reset()
{
    if (backend == nullptr) {
        return;
    }
    backend->reset();
}

void AutoTune::run()
{
    if (backend == nullptr) {
        return;
    }
    backend->run();
}

bool AutoTune::run_previous_mode()
{
    if (backend == nullptr) {
        return false;
    }
    return backend->run_previous_mode();
}

void AutoTune::update_switch_pos(const RC_Channel::AuxSwitchPos ch_flag)
{
    if (backend == nullptr) {
        return;
    }
    backend->update_switch_pos(ch_flag);
}

