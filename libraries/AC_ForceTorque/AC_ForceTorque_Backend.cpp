#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AC_ForceTorque.h"
#include "AC_ForceTorque_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AC_ForceTorque_Backend::AC_ForceTorque_Backend(ForceTorque::ForceTorque_State &_state, AC_ForceTorque_Params &_params) :
        state(_state),
		params(_params)
{
    _backend_type = type();
}

ForceTorque::Status AC_ForceTorque_Backend::status() const {
    if (type() == ForceTorque::Type::NONE) {
        // turned off at runtime?
        return ForceTorque::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AC_ForceTorque_Backend::has_data() const {
    return ((state.status != ForceTorque::Status::NotConnected) &&
            (state.status != ForceTorque::Status::NoData));
}

// update status based on force and torque measurement
void AC_ForceTorque_Backend::update_status()
{
    // check force and torque max or min range
    if (state.force_N.x > get_max_force_N() || state.force_N.y > get_max_force_N() || state.force_N.z > get_max_force_N() || state.torque_Nm.x > get_max_torque_Nm() || state.torque_Nm.y > get_max_torque_Nm() || state.torque_Nm.z > get_max_torque_Nm()) {
        set_status(ForceTorque::Status::OutOfRangeHigh);
    } else if (state.force_N.x < get_min_force_N() || state.force_N.y < get_min_force_N() || state.force_N.z < get_min_force_N() || state.torque_Nm.x < get_min_torque_Nm() || state.torque_Nm.y < get_min_torque_Nm() || state.torque_Nm.z < get_min_torque_Nm()) {
        set_status(ForceTorque::Status::OutOfRangeLow);
    } else {
        set_status(ForceTorque::Status::Good);
    }
}

// set status and update valid count
void AC_ForceTorque_Backend::set_status(ForceTorque::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == ForceTorque::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}