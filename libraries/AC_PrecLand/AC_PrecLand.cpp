#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: YAW_ALIGN
    // @DisplayName: Sensor yaw alignment
    // @Description: Yaw angle from body x-axis to sensor x-axis.
    // @Range: 0 360
    // @Increment: 1
    // @User: Advanced
    // @Units: Centi-degrees
    AP_GROUPINFO("YAW_ALIGN",    2, AC_PrecLand, _yaw_align, 0),

    // @Param: LAND_OFS_X
    // @DisplayName: Land offset forward
    // @Description: Desired landing position of the camera forward of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters
    AP_GROUPINFO("LAND_OFS_X",    3, AC_PrecLand, _land_ofs_cm_x, 0),

    // @Param: LAND_OFS_Y
    // @DisplayName: Land offset right
    // @Description: desired landing position of the camera right of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: Centimeters
    AP_GROUPINFO("LAND_OFS_Y",    4, AC_PrecLand, _land_ofs_cm_y, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav) :
    _ahrs(ahrs),
    _inav(inav),
    _last_update_ms(0),
    _last_backend_los_meas_ms(0),
    _backend(NULL)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN || CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    _attitude_history.push_back(_ahrs.get_rotation_body_to_ned());
    
    // run backend update
    if (_backend != NULL && _enabled) {
        // read from sensor
        _backend->update();

        Vector3f vehicleVelocityNED = _inav.get_velocity()*0.01f;
        vehicleVelocityNED.z = -vehicleVelocityNED.z;

        if (target_acquired()) {
            // EKF prediction step
            float dt;
            Vector3f targetDelVel;
            _ahrs.getCorrectedDeltaVelocityNED(targetDelVel, dt);
            targetDelVel = -targetDelVel;

            _ekf_x.predict(dt, targetDelVel.x, 0.5f*dt);
            _ekf_y.predict(dt, targetDelVel.y, 0.5f*dt);
        }

        if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
            // we have a new, unique los measurement
            _last_backend_los_meas_ms = _backend->los_meas_time_ms();

            Vector3f target_vec_unit_body;
            _backend->get_los_body(target_vec_unit_body);

            // Apply sensor yaw alignment rotation
            float sin_yaw_align = sinf(radians(_yaw_align*0.01f));
            float cos_yaw_align = cosf(radians(_yaw_align*0.01f));
            Matrix3f Rz = Matrix3f(
                cos_yaw_align, -sin_yaw_align, 0,
                sin_yaw_align, cos_yaw_align, 0,
                0, 0, 1
            );

            Vector3f target_vec_unit_ned = _attitude_history.front() * Rz * target_vec_unit_body;

            bool target_vec_valid = target_vec_unit_ned.z > 0.0f;

            if (target_vec_valid && rangefinder_alt_valid && rangefinder_alt_cm > 0.0f) {
                float alt = MAX(rangefinder_alt_cm*0.01f, 0.0f);
                float dist = alt/target_vec_unit_ned.z;
                Vector3f targetPosRelMeasNED = Vector3f(target_vec_unit_ned.x*dist, target_vec_unit_ned.y*dist, alt);

                float xy_pos_var = sq(targetPosRelMeasNED.z*(0.01f + 0.01f*_ahrs.get_gyro().length()) + 0.02f);
                if (!target_acquired()) {
                    // reset filter state
                    if (_inav.get_filter_status().flags.horiz_pos_rel) {
                        _ekf_x.init(targetPosRelMeasNED.x, xy_pos_var, -vehicleVelocityNED.x, sq(2.0f));
                        _ekf_y.init(targetPosRelMeasNED.y, xy_pos_var, -vehicleVelocityNED.y, sq(2.0f));
                    } else {
                        _ekf_x.init(targetPosRelMeasNED.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(targetPosRelMeasNED.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                } else {
                    float NIS_x = _ekf_x.getPosNIS(targetPosRelMeasNED.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(targetPosRelMeasNED.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(targetPosRelMeasNED.x, xy_pos_var);
                        _ekf_y.fusePos(targetPosRelMeasNED.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }
        }
    }
}

bool AC_PrecLand::target_acquired() const
{
    return (AP_HAL::millis()-_last_update_ms) < 2000;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    Vector3f land_ofs_ned_cm = _ahrs.get_rotation_body_to_ned() * Vector3f(_land_ofs_cm_x,_land_ofs_cm_y,0);

    ret.x = _ekf_x.getPos()*100.0f + _inav.get_position().x + land_ofs_ned_cm.x;
    ret.y = _ekf_y.getPos()*100.0f + _inav.get_position().y + land_ofs_ned_cm.y;
    return true;
}

bool AC_PrecLand::get_target_position_relative_cm(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }

    Vector3f land_ofs_ned_cm = _ahrs.get_rotation_body_to_ned() * Vector3f(_land_ofs_cm_x,_land_ofs_cm_y,0);

    ret.x = _ekf_x.getPos()*100.0f + land_ofs_ned_cm.x;
    ret.y = _ekf_y.getPos()*100.0f + land_ofs_ned_cm.y;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative_cms(Vector2f& ret) const
{
    if (!target_acquired()) {
        return false;
    }
    ret.x = _ekf_x.getVel()*100.0f;
    ret.y = _ekf_y.getVel()*100.0f;
    return true;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != NULL) {
        _backend->handle_msg(msg);
    }
}
