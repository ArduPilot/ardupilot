#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Vtx_Backend.h"


extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_Vtx_Backend::AP_Vtx_Backend(AP_Vtx_Backend::Vtx_State &_state, AP_Vtx_Params &_params) :
    state(_state),
    params(_params)
{
    _status=AP_Vtx_Backend::Status::not_connected;
}

// true if vtx is returning data
bool AP_Vtx_Backend::has_data() const
{
    return ((_status != AP_Vtx_Backend::Status::not_connected) &&
            (_status != AP_Vtx_Backend::Status::idle));
}

