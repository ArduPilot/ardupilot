#include "AP_CustomRotations_config.h"

#if AP_CUSTOMROTATIONS_ENABLED

#include "AP_CustomRotations.h"

#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

const AP_Param::GroupInfo AP_CustomRotations::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Enable Custom rotations
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables custom rotations
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_CustomRotations, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: 1_
    // @Path: AP_CustomRotations_params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 2, AP_CustomRotations, AP_CustomRotation_params),

    // @Group: 2_
    // @Path: AP_CustomRotations_params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 3, AP_CustomRotations, AP_CustomRotation_params),

    AP_GROUPEND
};

AP_CustomRotation::AP_CustomRotation(AP_CustomRotation_params &_params):
    params(_params)
{
    init();
}

void AP_CustomRotation::init()
{
    m.from_euler(radians(params.roll),radians(params.pitch),radians(params.yaw));
    q.from_rotation_matrix(m);
}

AP_CustomRotations::AP_CustomRotations()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_CustomRotations must be singleton");
    }
#endif
    singleton = this;
}

void AP_CustomRotations::init()
{
    if (enable == 0) {
        return;
    }

    // make sure all custom rotations are allocated
    for (uint8_t i = 0; i < NUM_CUST_ROT; i++) {
        AP_CustomRotation* rot = get_rotation(Rotation(i + ROTATION_CUSTOM_1));
        if (rot == nullptr) {
            AP_BoardConfig::allocation_error("Custom Rotations");
        }
    }
}

void AP_CustomRotations::convert(Rotation r, float roll, float pitch, float yaw)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    if (!rot->params.roll.configured() && !rot->params.pitch.configured() && !rot->params.yaw.configured()) {
        rot->params.roll.set_and_save(roll);
        rot->params.pitch.set_and_save(pitch);
        rot->params.yaw.set_and_save(yaw);
        rot->init();
    }
}

void AP_CustomRotations::set(Rotation r, float roll, float pitch, float yaw)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    rot->params.roll.set(roll);
    rot->params.pitch.set(pitch);
    rot->params.yaw.set(yaw);
    rot->init();
}

void AP_CustomRotations::from_rotation(Rotation r, QuaternionD& q)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    q = rot->q.todouble();
}

void AP_CustomRotations::from_rotation(Rotation r, Quaternion& q)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    q = rot->q;
}

void AP_CustomRotations::rotate(Rotation r, Vector3d& v)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    v = (rot->m * v.tofloat()).todouble();
}

void AP_CustomRotations::rotate(Rotation r, Vector3f& v)
{
    AP_CustomRotation* rot = get_rotation(r);
    if (rot == nullptr) {
        return;
    }
    v = rot->m * v;
}

AP_CustomRotation* AP_CustomRotations::get_rotation(Rotation r)
{
    if (r < ROTATION_CUSTOM_1 || r >= ROTATION_CUSTOM_END) {
        INTERNAL_ERROR(AP_InternalError::error_t::bad_rotation);
        return nullptr;
    }
    const uint8_t index = r - ROTATION_CUSTOM_1;
    if (rotations[index] == nullptr) {
        rotations[index] = NEW_NOTHROW AP_CustomRotation(params[index]);
         // make sure param is enabled if custom rotation is used
        enable.set_and_save_ifchanged(1);
    }
    return rotations[index];
}

// singleton instance
AP_CustomRotations *AP_CustomRotations::singleton;

namespace AP {

AP_CustomRotations &custom_rotations()
{
    return *AP_CustomRotations::get_singleton();
}

}

#endif  // AP_CUSTOMROTATIONS_ENABLED
