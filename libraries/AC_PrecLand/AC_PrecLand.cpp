#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"
#include "AC_PrecLand_SITL_Gazebo.h"
#include "AC_PrecLand_SITL.h"

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
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock, 3:SITL_Gazebo, 4:SITL
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: YAW_ALIGN
    // @DisplayName: Sensor yaw alignment
    // @Description: Yaw angle from body x-axis to sensor x-axis.
    // @Range: 0 360
    // @Increment: 1
    // @User: Advanced
    // @Units: cdeg
    AP_GROUPINFO("YAW_ALIGN",    2, AC_PrecLand, _yaw_align, 0),

    // @Param: LAND_OFS_X
    // @DisplayName: Land offset forward
    // @Description: Desired landing position of the camera forward of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_X",    3, AC_PrecLand, _land_ofs_cm_x, 0),

    // @Param: LAND_OFS_Y
    // @DisplayName: Land offset right
    // @Description: desired landing position of the camera right of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_Y",    4, AC_PrecLand, _land_ofs_cm_y, 0),

    // @Param: EST_TYPE
    // @DisplayName: Precision Land Estimator Type
    // @Description: Specifies the estimation method to be used
    // @Values: 0:RawSensor, 1:KalmanFilter
    // @User: Advanced
    AP_GROUPINFO("EST_TYPE",    5, AC_PrecLand, _estimator_type, 1),

    // @Param: ACC_P_NSE
    // @DisplayName: Kalman Filter Accelerometer Noise
    // @Description: Kalman Filter Accelerometer Noise, higher values weight the input from the camera more, accels less
    // @Range: 0.5 5
    // @User: Advanceds
    AP_GROUPINFO("ACC_P_NSE", 6, AC_PrecLand, _accel_noise, 2.5f),

    // @Param: CAM_POS_X
    // @DisplayName: Camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: CAM_POS_Y
    // @DisplayName: Camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: CAM_POS_Z
    // @DisplayName: Camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("CAM_POS", 7, AC_PrecLand, _cam_offset, 0.0f),

    // @Param: BUS
    // @DisplayName: Sensor Bus
    // @Description: Precland sensor bus for I2C sensors.
    // @Values: -1:DefaultBus,0:InternalI2C,1:ExternalI2C
    // @User: Advanced
    AP_GROUPINFO("BUS",    8, AC_PrecLand, _bus, -1),
    
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS_NavEKF& ahrs) :
    _ahrs(ahrs),
    _last_update_ms(0),
    _target_acquired(false),
    _last_backend_los_meas_ms(0),
    _backend(nullptr)
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
    if (_backend != nullptr) {
        return;
    }

    // default health to false
    _backend = nullptr;
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
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case PRECLAND_TYPE_SITL_GAZEBO:
            _backend = new AC_PrecLand_SITL_Gazebo(*this, _backend_state);
            break;
        case PRECLAND_TYPE_SITL:
            _backend = new AC_PrecLand_SITL(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != nullptr) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    // append current velocity and attitude correction into history buffer
    struct inertial_data_frame_s inertial_data_newest;
    _ahrs.getCorrectedDeltaVelocityNED(inertial_data_newest.correctedVehicleDeltaVelocityNED, inertial_data_newest.dt);
    inertial_data_newest.Tbn = _ahrs.get_rotation_body_to_ned();
    Vector3f curr_vel;
    nav_filter_status status;
    if (!_ahrs.get_velocity_NED(curr_vel) || !_ahrs.get_filter_status(status)) {
        inertial_data_newest.inertialNavVelocityValid = false;
    } else {
        inertial_data_newest.inertialNavVelocityValid = status.flags.horiz_vel;
    }
    curr_vel.z = -curr_vel.z;  // NED to NEU
    inertial_data_newest.inertialNavVelocity = curr_vel;

    _inertial_history.push_back(inertial_data_newest);

    // update estimator of target position
    if (_backend != nullptr && _enabled) {
        _backend->update();
        run_estimator(rangefinder_alt_cm*0.01f, rangefinder_alt_valid);
    }
}

bool AC_PrecLand::target_acquired()
{
    _target_acquired = _target_acquired && (AP_HAL::millis()-_last_update_ms) < 2000;
    return _target_acquired;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    Vector2f curr_pos;
    if (!_ahrs.get_relative_position_NE_origin(curr_pos)) {
        return false;
    }
    ret.x = (_target_pos_rel_out_NE.x + curr_pos.x) * 100.0f;   // m to cm
    ret.y = (_target_pos_rel_out_NE.y  + curr_pos.y) * 100.0f;  // m to cm
    return true;
}

bool AC_PrecLand::get_target_position_relative_cm(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_pos_rel_out_NE*100.0f;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative_cms(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_vel_rel_out_NE*100.0f;
    return true;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != nullptr) {
        _backend->handle_msg(msg);
    }
}

//
// Private methods
//

void AC_PrecLand::run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    const struct inertial_data_frame_s& inertial_data_delayed = _inertial_history.front();

    switch (_estimator_type) {
        case ESTIMATOR_TYPE_RAW_SENSOR: {
            // Return if there's any invalid velocity data
            for (uint8_t i=0; i<_inertial_history.size(); i++) {
                const struct inertial_data_frame_s& inertial_data = _inertial_history.peek(i);
                if (!inertial_data.inertialNavVelocityValid) {
                    _target_acquired = false;
                    return;
                }
            }

            // Predict
            if (target_acquired()) {
                _target_pos_rel_est_NE.x -= inertial_data_delayed.inertialNavVelocity.x * inertial_data_delayed.dt;
                _target_pos_rel_est_NE.y -= inertial_data_delayed.inertialNavVelocity.y * inertial_data_delayed.dt;
                _target_vel_rel_est_NE.x = -inertial_data_delayed.inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed.inertialNavVelocity.y;
            }

            // Update if a new LOS measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                _target_pos_rel_est_NE.x = _target_pos_rel_meas_NED.x;
                _target_pos_rel_est_NE.y = _target_pos_rel_meas_NED.y;
                _target_vel_rel_est_NE.x = -inertial_data_delayed.inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed.inertialNavVelocity.y;

                _last_update_ms = AP_HAL::millis();
                _target_acquired = true;
            }

            // Output prediction
            if (target_acquired()) {
                run_output_prediction();
            }
            break;
        }
        case ESTIMATOR_TYPE_KALMAN_FILTER: {
            // Predict
            if (target_acquired()) {
                const float& dt = inertial_data_delayed.dt;
                const Vector3f& vehicleDelVel = inertial_data_delayed.correctedVehicleDeltaVelocityNED;

                _ekf_x.predict(dt, -vehicleDelVel.x, _accel_noise*dt);
                _ekf_y.predict(dt, -vehicleDelVel.y, _accel_noise*dt);
            }

            // Update if a new LOS measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                float xy_pos_var = sq(_target_pos_rel_meas_NED.z*(0.01f + 0.01f*_ahrs.get_gyro().length()) + 0.02f);
                if (!target_acquired()) {
                    // reset filter state
                    if (inertial_data_delayed.inertialNavVelocityValid) {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, -inertial_data_delayed.inertialNavVelocity.x, sq(2.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, -inertial_data_delayed.inertialNavVelocity.y, sq(2.0f));
                    } else {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                    _target_acquired = true;
                } else {
                    float NIS_x = _ekf_x.getPosNIS(_target_pos_rel_meas_NED.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(_target_pos_rel_meas_NED.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(_target_pos_rel_meas_NED.x, xy_pos_var);
                        _ekf_y.fusePos(_target_pos_rel_meas_NED.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                        _target_acquired = true;
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }

            // Output prediction
            if (target_acquired()) {
                _target_pos_rel_est_NE.x = _ekf_x.getPos();
                _target_pos_rel_est_NE.y = _ekf_y.getPos();
                _target_vel_rel_est_NE.x = _ekf_x.getVel();
                _target_vel_rel_est_NE.y = _ekf_y.getVel();

                run_output_prediction();
            }
            break;
        }
    }
}

bool AC_PrecLand::retrieve_los_meas(Vector3f& target_vec_unit_body)
{
    if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
        _last_backend_los_meas_ms = _backend->los_meas_time_ms();
        _backend->get_los_body(target_vec_unit_body);

        // Apply sensor yaw alignment rotation
        float sin_yaw_align = sinf(radians(_yaw_align*0.01f));
        float cos_yaw_align = cosf(radians(_yaw_align*0.01f));
        Matrix3f Rz = Matrix3f(
            cos_yaw_align, -sin_yaw_align, 0,
            sin_yaw_align, cos_yaw_align, 0,
            0, 0, 1
        );

        target_vec_unit_body = Rz*target_vec_unit_body;
        return true;
    } else {
        return false;
    }
}

bool AC_PrecLand::construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    Vector3f target_vec_unit_body;
    if (retrieve_los_meas(target_vec_unit_body)) {
        const struct inertial_data_frame_s& inertial_data_delayed = _inertial_history.front();

        Vector3f target_vec_unit_ned = inertial_data_delayed.Tbn * target_vec_unit_body;
        bool target_vec_valid = target_vec_unit_ned.z > 0.0f;
        bool alt_valid = (rangefinder_alt_valid && rangefinder_alt_m > 0.0f) || (_backend->distance_to_target() > 0.0f);
        if (target_vec_valid && alt_valid) {
            float dist, alt;
            if (_backend->distance_to_target() > 0.0f) {
                dist = _backend->distance_to_target();
                alt = dist * target_vec_unit_ned.z;
            } else {
                alt = MAX(rangefinder_alt_m, 0.0f);
                dist = alt / target_vec_unit_ned.z;
            }

            // Compute camera position relative to IMU
            Vector3f accel_body_offset = AP::ins().get_imu_pos_offset(_ahrs.get_primary_accel_index());
            Vector3f cam_pos_ned = inertial_data_delayed.Tbn * (_cam_offset.get() - accel_body_offset);

            // Compute target position relative to IMU
            _target_pos_rel_meas_NED = Vector3f(target_vec_unit_ned.x*dist, target_vec_unit_ned.y*dist, alt) + cam_pos_ned;
            return true;
        }
    }
    return false;
}

void AC_PrecLand::run_output_prediction()
{
    _target_pos_rel_out_NE = _target_pos_rel_est_NE;
    _target_vel_rel_out_NE = _target_vel_rel_est_NE;

    // Predict forward from delayed time horizon
    for (uint8_t i=1; i<_inertial_history.size(); i++) {
        const struct inertial_data_frame_s& inertial_data = _inertial_history.peek(i);
        _target_vel_rel_out_NE.x -= inertial_data.correctedVehicleDeltaVelocityNED.x;
        _target_vel_rel_out_NE.y -= inertial_data.correctedVehicleDeltaVelocityNED.y;
        _target_pos_rel_out_NE.x += _target_vel_rel_out_NE.x * inertial_data.dt;
        _target_pos_rel_out_NE.y += _target_vel_rel_out_NE.y * inertial_data.dt;
    }

    const Matrix3f& Tbn = _inertial_history.peek(_inertial_history.size()-1).Tbn;
    Vector3f accel_body_offset = AP::ins().get_imu_pos_offset(_ahrs.get_primary_accel_index());

    // Apply position correction for CG offset from IMU
    Vector3f imu_pos_ned = Tbn * accel_body_offset;
    _target_pos_rel_out_NE.x += imu_pos_ned.x;
    _target_pos_rel_out_NE.y += imu_pos_ned.y;

    // Apply position correction for body-frame horizontal camera offset from CG, so that vehicle lands lens-to-target
    Vector3f cam_pos_horizontal_ned = Tbn * Vector3f(_cam_offset.get().x, _cam_offset.get().y, 0);
    _target_pos_rel_out_NE.x -= cam_pos_horizontal_ned.x;
    _target_pos_rel_out_NE.y -= cam_pos_horizontal_ned.y;

    // Apply velocity correction for IMU offset from CG
    Vector3f vel_ned_rel_imu = Tbn * (_ahrs.get_gyro() % (-accel_body_offset));
    _target_vel_rel_out_NE.x -= vel_ned_rel_imu.x;
    _target_vel_rel_out_NE.y -= vel_ned_rel_imu.y;

    // Apply land offset
    Vector3f land_ofs_ned_m = _ahrs.get_rotation_body_to_ned() * Vector3f(_land_ofs_cm_x,_land_ofs_cm_y,0) * 0.01f;
    _target_pos_rel_out_NE.x += land_ofs_ned_m.x;
    _target_pos_rel_out_NE.y += land_ofs_ned_m.y;
}
