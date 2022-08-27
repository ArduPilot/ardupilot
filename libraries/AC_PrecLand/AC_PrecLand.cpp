#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"
#include "AC_PrecLand_SITL_Gazebo.h"
#include "AC_PrecLand_SITL.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
 # define AC_PRECLAND_ORIENT_DEFAULT Rotation::ROTATION_NONE
#else
 # define AC_PRECLAND_ORIENT_DEFAULT Rotation::ROTATION_PITCH_270
#endif

static const uint32_t EKF_INIT_TIME_MS = 2000; // EKF initialisation requires this many milliseconds of good sensor data
static const uint32_t EKF_INIT_SENSOR_MIN_UPDATE_MS = 500; // Sensor must update within this many ms during EKF init, else init will fail
static const uint32_t LANDING_TARGET_TIMEOUT_MS = 2000; // Sensor must update within this many ms, else prec landing will be switched off
static const uint32_t LANDING_TARGET_LOST_TIMEOUT_MS = 180000; // Target will be considered as "lost" if the last known location of the target is more than this many ms ago
static const float    LANDING_TARGET_LOST_DIST_THRESH_M  = 30; // If the last known location of the landing target is beyond this many meters, then we will consider it lost

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled
    // @Description: Precision Land enabled/disabled
    // @Values: 0:Disabled, 1:Enabled
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
    // @Range: 0 36000
    // @Increment: 10
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
    // @User: Advanced
    AP_GROUPINFO("ACC_P_NSE", 6, AC_PrecLand, _accel_noise, 2.5f),

    // @Param: CAM_POS_X
    // @DisplayName: Camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: CAM_POS_Y
    // @DisplayName: Camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: CAM_POS_Z
    // @DisplayName: Camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("CAM_POS", 7, AC_PrecLand, _cam_offset, 0.0f),

    // @Param: BUS
    // @DisplayName: Sensor Bus
    // @Description: Precland sensor bus for I2C sensors.
    // @Values: -1:DefaultBus,0:InternalI2C,1:ExternalI2C
    // @User: Advanced
    AP_GROUPINFO("BUS",    8, AC_PrecLand, _bus, -1),

    // @Param: LAG
    // @DisplayName: Precision Landing sensor lag
    // @Description: Precision Landing sensor lag, to cope with variable landing_target latency
    // @Range: 0.02 0.250
    // @Increment: 1
    // @Units: s
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("LAG", 9, AC_PrecLand, _lag, 0.02f), // 20ms is the old default buffer size (8 frames @ 400hz/2.5ms)

    // @Param: XY_DIST_MAX
    // @DisplayName: Precision Landing maximum distance to target before descending
    // @Description: The vehicle will not start descending if the landing target is detected and it is further than this many meters away. Set 0 to always descend.
    // @Range: 0 10
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("XY_DIST_MAX", 10, AC_PrecLand, _xy_max_dist_desc, 2.5f),
    // @Param: STRICT
    // @DisplayName: PrecLand strictness
    // @Description: How strictly should the vehicle land on the target if target is lost
    // @Values: 0: Land Vertically (Not strict), 1: Retry Landing(Normal Strictness), 2: Do not land (just Hover) (Very Strict)
    AP_GROUPINFO("STRICT", 11, AC_PrecLand, _strict, 1),

    // @Param: RET_MAX
    // @DisplayName: PrecLand Maximum number of retires for a failed landing
    // @Description: PrecLand Maximum number of retires for a failed landing. Set to zero to disable landing retry.
    // @Range: 0 10
    // @Increment: 1
    AP_GROUPINFO("RET_MAX", 12, AC_PrecLand, _retry_max, 4),

    // @Param: TIMEOUT
    // @DisplayName: PrecLand retry timeout
    // @Description: Time for which vehicle continues descend even if target is lost. After this time period, vehicle will attemp a landing retry depending on PLND_STRICT parameter.
    // @Range: 0 20
    // @Units: s
    AP_GROUPINFO("TIMEOUT", 13, AC_PrecLand, _retry_timeout_sec, 4),

    // @Param: RET_BEHAVE
    // @DisplayName: PrecLand retry behaviour
    // @Description: Prec Land will do the action selected by this parameter if a retry to a landing is needed
    // @Values: 0: Go to the last location where landing target was detected, 1: Go towards the approximate location of the detected landing target
    AP_GROUPINFO("RET_BEHAVE", 14, AC_PrecLand, _retry_behave, 0),

    // @Param: ALT_MIN
    // @DisplayName: PrecLand minimum alt for retry
    // @Description: Vehicle will continue landing vertically even if target is lost below this height. This needs a rangefinder to work. Set to zero to disable this.
    // @Range: 0 5
    // @Units: m
    AP_GROUPINFO("ALT_MIN", 15, AC_PrecLand, _sensor_min_alt, 0.75),

    // @Param: ALT_MAX
    // @DisplayName: PrecLand maximum alt for retry
    // @Description: Vehicle will continue landing vertically until this height if target is not found. Below this height if landing target is not found, landing retry/failsafe might be attempted. This needs a rangefinder to work. Set to zero to disable this.
    // @Range: 0 50
    // @Units: m
    AP_GROUPINFO("ALT_MAX", 16, AC_PrecLand, _sensor_max_alt, 8),

    // @Param: OPTIONS
    // @DisplayName: Precision Landing Extra Options
    // @Description: Precision Landing Extra Options
    // @Bitmask: 0: Moving Landing Target
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 17, AC_PrecLand, _options, 0),

    // @Param: ORIENT
    // @DisplayName: Camera Orientation
    // @Description: Orientation of camera/sensor on body
    // @Values: 0:Forward, 4:Back, 25:Down
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FRAME("ORIENT", 18, AC_PrecLand, _orient, AC_PRECLAND_ORIENT_DEFAULT, AP_PARAM_FRAME_ROVER),

    AP_GROUPEND
};

// Default constructor.
AC_PrecLand::AC_PrecLand()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AC_PrecLand must be singleton");
    }
    _singleton = this;

    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation of landing controllers
// update_rate_hz should be the rate at which the update method will be called in hz
void AC_PrecLand::init(uint16_t update_rate_hz)
{
    // exit immediately if init has already been run
    if (_backend != nullptr) {
        return;
    }

    // init as target TARGET_NEVER_SEEN, we will update this later
    _current_target_state = TargetState::TARGET_NEVER_SEEN;

    // default health to false
    _backend = nullptr;
    _backend_state.healthy = false;

    // create inertial history buffer
    // constrain lag parameter to be within bounds
    _lag.set(constrain_float(_lag, 0.02f, 0.25f));

    // calculate inertial buffer size from lag and minimum of main loop rate and update_rate_hz argument
    const uint16_t inertial_buffer_size = MAX((uint16_t)roundf(_lag * MIN(update_rate_hz, AP::scheduler().get_loop_rate_hz())), 1);

    // instantiate ring buffer to hold inertial history, return on failure so no backends are created
    _inertial_history = new ObjectArray<inertial_data_frame_s>(inertial_buffer_size);
    if (_inertial_history == nullptr) {
        return;
    }

    // instantiate backend based on type parameter
    switch ((Type)(_type.get())) {
        // no type defined
        case Type::NONE:
        default:
            return;
        // companion computer
        case Type::COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
        case Type::IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case Type::SITL_GAZEBO:
            _backend = new AC_PrecLand_SITL_Gazebo(*this, _backend_state);
            break;
        case Type::SITL:
            _backend = new AC_PrecLand_SITL(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != nullptr) {
        _backend->init();
    }

    _approach_vector_body.x = 1;
    _approach_vector_body.rotate(_orient);
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    // exit immediately if not enabled
    if (_backend == nullptr || _inertial_history == nullptr) {
        return;
    }

    // append current velocity and attitude correction into history buffer
    struct inertial_data_frame_s inertial_data_newest;
    const auto &_ahrs = AP::ahrs();
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

    inertial_data_newest.time_usec = AP_HAL::micros64();
    _inertial_history->push_force(inertial_data_newest);

    const float rangefinder_alt_m = rangefinder_alt_cm*0.01f;  //cm to meter

    // update estimator of target position
    if (_backend != nullptr && _enabled) {
        _backend->update();
        run_estimator(rangefinder_alt_m, rangefinder_alt_valid);
    }

    // check the status of the landing target location
    check_target_status(rangefinder_alt_m, rangefinder_alt_valid);

    const uint32_t now = AP_HAL::millis();
    if (now - last_log_ms > 40) {  // 25Hz
        last_log_ms = now;
        Write_Precland();
    }
}

// check the status of the target
void AC_PrecLand::check_target_status(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    if (target_acquired()) {
        // target in sight
        _current_target_state = TargetState::TARGET_FOUND;
        // early return because we already know the status
        return;
    }

    // target not in sight
    if (_current_target_state == TargetState::TARGET_FOUND ||
               _current_target_state == TargetState::TARGET_RECENTLY_LOST) {
        // we had target in sight, but not any more, i.e we have lost the target
        _current_target_state = TargetState::TARGET_RECENTLY_LOST;
    } else {
        // we never had the target in sight
        _current_target_state = TargetState::TARGET_NEVER_SEEN;
    }

    // We definitely do not have the target in sight
    // check if the precision landing sensor is supposed to be in range
    // this needs a valid rangefinder to work
    if (!check_if_sensor_in_range(rangefinder_alt_m, rangefinder_alt_valid)) {
        // Target is not in range (vehicle is either too high or too low). Vehicle will not be attempting any sort of landing retries during this period
        _current_target_state = TargetState::TARGET_OUT_OF_RANGE;
        return;
    }

    if (_current_target_state == TargetState::TARGET_RECENTLY_LOST) {
        // check if it's nearby/found recently, else the status will be demoted to "TARGET_LOST"
        Vector2f curr_pos;
        if (AP::ahrs().get_relative_position_NE_origin(curr_pos)) {
            const float dist_to_last_target_loc_xy = (curr_pos - Vector2f{_last_target_pos_rel_origin_NED.x, _last_target_pos_rel_origin_NED.y}).length();
            const float dist_to_last_loc_xy = (curr_pos - Vector2f{_last_vehicle_pos_NED.x, _last_vehicle_pos_NED.y}).length();
            if ((AP_HAL::millis() - _last_valid_target_ms) > LANDING_TARGET_LOST_TIMEOUT_MS) {
                // the target has not been seen for a long time
                // might as well consider it as "never seen"
                _current_target_state = TargetState::TARGET_NEVER_SEEN;
                return;
            }

            if ((dist_to_last_target_loc_xy > LANDING_TARGET_LOST_DIST_THRESH_M) || (dist_to_last_loc_xy > LANDING_TARGET_LOST_DIST_THRESH_M)) {
                // the last known location of target is too far away
                _current_target_state = TargetState::TARGET_NEVER_SEEN;
                return;
            }
        }
    }
}

// Check if the landing target is supposed to be in sight based on the height of the vehicle from the ground
// This needs a valid rangefinder to work, if the min/max parameters are non zero
bool AC_PrecLand::check_if_sensor_in_range(float rangefinder_alt_m, bool rangefinder_alt_valid) const
{
    if (is_zero(_sensor_max_alt) && is_zero(_sensor_min_alt)) {
        // no sensor limits have been specified, assume sensor is always in range
        return true;
    }

    if (!rangefinder_alt_valid) {
        // rangefinder isn't healthy. We might be at a very high altitude
        return false;
    }

    if (rangefinder_alt_m > _sensor_max_alt && !is_zero(_sensor_max_alt)) {
        // this prevents triggering a retry when we are too far away from the target
        return false;
    }

    if (rangefinder_alt_m < _sensor_min_alt && !is_zero(_sensor_min_alt)) {
        // this prevents triggering a retry when we are very close to the target
        return false;
    }

    // target should be in range
    return true;
}

bool AC_PrecLand::target_acquired()
{
    if ((AP_HAL::millis()-_last_update_ms) > LANDING_TARGET_TIMEOUT_MS) {
        if (_target_acquired) {
            // just lost the landing target, inform the user. This message will only be sent once everytime target is lost
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PrecLand: Target Lost");
        }
        // not had a sensor update since a long time
        // probably lost the target
        _estimator_initialized = false;
        _target_acquired = false;
    }
    return _target_acquired;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    Vector2f curr_pos;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos)) {
        return false;
    }
    ret.x = (_target_pos_rel_out_NE.x + curr_pos.x) * 100.0f;   // m to cm
    ret.y = (_target_pos_rel_out_NE.y  + curr_pos.y) * 100.0f;  // m to cm
    return true;
}

void AC_PrecLand::get_target_position_measurement_cm(Vector3f& ret)
{
    ret = _target_pos_rel_meas_NED*100.0f;
    return;
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

// get the absolute velocity of the vehicle
void AC_PrecLand::get_target_velocity_cms(const Vector2f& vehicle_velocity_cms, Vector2f& target_vel_cms)
{
    if (!(_options & PLND_OPTION_MOVING_TARGET)) {
        // the target should not be moving
        target_vel_cms.zero();
        return;
    }
    if ((EstimatorType)_estimator_type.get() == EstimatorType::RAW_SENSOR) {
        // We do not predict the velocity of the target in this case
        // assume velocity to be zero
        target_vel_cms.zero();
        return;
    }
    Vector2f target_vel_rel_cms;
    if (!get_target_velocity_relative_cms(target_vel_rel_cms)) {
        // Don't know where the target is
        // assume velocity to be zero
        target_vel_cms.zero();
        return;
    }
    // return the absolute velocity
    target_vel_cms  = target_vel_rel_cms + vehicle_velocity_cms;
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // run backend update
    if (_backend != nullptr) {
        _backend->handle_msg(packet, timestamp_ms);
    }
}

//
// Private methods
//

void AC_PrecLand::run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    const struct inertial_data_frame_s *inertial_data_delayed = (*_inertial_history)[0];

    switch ((EstimatorType)_estimator_type.get()) {
        case EstimatorType::RAW_SENSOR: {
            // Return if there's any invalid velocity data
            for (uint8_t i=0; i<_inertial_history->available(); i++) {
                const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
                if (!inertial_data->inertialNavVelocityValid) {
                    _target_acquired = false;
                    return;
                }
            }

            // Predict
            if (target_acquired()) {
                _target_pos_rel_est_NE.x -= inertial_data_delayed->inertialNavVelocity.x * inertial_data_delayed->dt;
                _target_pos_rel_est_NE.y -= inertial_data_delayed->inertialNavVelocity.y * inertial_data_delayed->dt;
                _target_vel_rel_est_NE.x = -inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed->inertialNavVelocity.y;
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                if (!_estimator_initialized) {
                    gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Target Found");
                    _estimator_initialized = true;
                }
                _target_pos_rel_est_NE.x = _target_pos_rel_meas_NED.x;
                _target_pos_rel_est_NE.y = _target_pos_rel_meas_NED.y;
                _target_vel_rel_est_NE.x = -inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed->inertialNavVelocity.y;

                _last_update_ms = AP_HAL::millis();
                _target_acquired = true;
            }

            // Output prediction
            if (target_acquired()) {
                run_output_prediction();
            }
            break;
        }
        case EstimatorType::KALMAN_FILTER: {
            // Predict
            if (target_acquired() || _estimator_initialized) {
                const float& dt = inertial_data_delayed->dt;
                const Vector3f& vehicleDelVel = inertial_data_delayed->correctedVehicleDeltaVelocityNED;

                _ekf_x.predict(dt, -vehicleDelVel.x, _accel_noise*dt);
                _ekf_y.predict(dt, -vehicleDelVel.y, _accel_noise*dt);
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                float xy_pos_var = sq(_target_pos_rel_meas_NED.z*(0.01f + 0.01f*AP::ahrs().get_gyro().length()) + 0.02f);
                if (!_estimator_initialized) {
                    // Inform the user landing target has been found
                    gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Target Found");
                    // start init of EKF. We will let the filter consume the data for a while before it available for consumption
                    // reset filter state
                    if (inertial_data_delayed->inertialNavVelocityValid) {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, -inertial_data_delayed->inertialNavVelocity.x, sq(2.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, -inertial_data_delayed->inertialNavVelocity.y, sq(2.0f));
                    } else {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                    _estimator_init_ms = AP_HAL::millis();
                    // we have initialized the estimator but will not use the values for sometime so that EKF settles down
                    _estimator_initialized = true;
                } else {
                    float NIS_x = _ekf_x.getPosNIS(_target_pos_rel_meas_NED.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(_target_pos_rel_meas_NED.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(_target_pos_rel_meas_NED.x, xy_pos_var);
                        _ekf_y.fusePos(_target_pos_rel_meas_NED.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }

            // check EKF was properly initialized when the sensor detected a landing target
            check_ekf_init_timeout();

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


// check if EKF got the time to initialize when the landing target was first detected
// Expects sensor to update within EKF_INIT_SENSOR_MIN_UPDATE_MS milliseconds till EKF_INIT_TIME_MS milliseconds have passed
// after this period landing target estimates can be used by vehicle
void AC_PrecLand::check_ekf_init_timeout()
{
    if (!target_acquired() && _estimator_initialized) {
        // we have just got the target in sight
        if (AP_HAL::millis()-_last_update_ms > EKF_INIT_SENSOR_MIN_UPDATE_MS) {
            // we have lost the target, not enough readings to initialize the EKF
            _estimator_initialized = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PrecLand: Init Failed");
        } else if (AP_HAL::millis()-_estimator_init_ms > EKF_INIT_TIME_MS) {
            // the target has been visible for a while, EKF should now have initialized to a good value
            _target_acquired = true;
            gcs().send_text(MAV_SEVERITY_INFO, "PrecLand: Init Complete");
        }
    }
}

bool AC_PrecLand::retrieve_los_meas(Vector3f& target_vec_unit_body)
{
    if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
        _last_backend_los_meas_ms = _backend->los_meas_time_ms();
        _backend->get_los_body(target_vec_unit_body);
        if (!is_zero(_yaw_align)) {
            // Apply sensor yaw alignment rotation
            target_vec_unit_body.rotate_xy(radians(_yaw_align*0.01f));
        }


        // rotate vector based on sensor oriention to get correct body frame vector
        if (_orient != ROTATION_PITCH_270) {
            // by default, the vector is constructed downwards in body frame
            // hence, we do not do any rotation if the orientation is downwards
            // if it is some other orientation, we first bring the vector to forward
            // and then we rotate it to desired orientation
            // because the rotations are measured with respect to a vector pointing towards front in body frame
            // for eg, if orientation is back, i.e., ROTATION_YAW_180, 
            // the vector is first brought to front and then rotation by YAW 180 to take it to the back of vehicle
            target_vec_unit_body.rotate(ROTATION_PITCH_90); // bring vector to front
            target_vec_unit_body.rotate(_orient);           // rotate it to desired orientation
        }

        return true;
    } else {
        return false;
    }
}

bool AC_PrecLand::retrieve_local_offset_ned_meas(Vector3f& target_local_offset_ned)
{
    // Try to get target position from local FRD target position
    if (_backend->have_los_meas_local_frd()) { //We have local offset measurement
        if(_backend->los_meas_local_frd_time_ms() != _last_backend_los_meas_ms){ // Measurement not processed yet
            
            Vector3f _target_local_frd;
            const AP_AHRS &_ahrs = AP::ahrs();
             
            if (_backend->get_los_local_frd_target(_target_local_frd)) {// Get offset NED position                
                _target_local_frd.rotate_xy(_ahrs.yaw);
                target_local_offset_ned = _target_local_frd;
                _last_backend_los_meas_ms = _backend->los_meas_local_frd_time_ms();  
                return true;
            }
        }   
        return false; // We have measurement, but it is not updated.
    }

    // Try to get relative target position directly
    if (_backend->have_los_meas_local_offset_ned()) { //We have local offset measurement
        if(_backend->los_meas_local_offset_ned_time_ms() != _last_backend_los_meas_ms){ // Measurement not processed yet
            
            if (_backend->get_los_local_offset_ned_target(target_local_offset_ned)) {// Get offset NED position                
               
                _last_backend_los_meas_ms = _backend->los_meas_local_offset_ned_time_ms();
                return true;
            }
        }   
        return false; // We have measurement, but it is not updated.
    }
    
    // Try to get relative position from absolute NED measurements
    if (_backend->have_los_meas_local_ned()){ //We have local absolute measurement
        if(_backend->los_meas_local_ned_time_ms() != _last_backend_los_meas_ms){ // Measurement not processed yet
           
            Vector3f _target_local_ned;
            const AP_AHRS &_ahrs = AP::ahrs();
            Vector3f _vehicle_pos_NED;
            if (_backend->get_los_local_ned_target(_target_local_ned) &&
                (_ahrs.get_relative_position_NED_origin(_vehicle_pos_NED))) {// Get offset NED position 
                
                _last_backend_los_meas_ms = _backend->los_meas_local_ned_time_ms();
                target_local_offset_ned = _target_local_ned - _vehicle_pos_NED;
                return true;
            } 
        }       
        return false; // But it is not updated
    }
    return false;
}

bool AC_PrecLand::construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    // Process NED data if provided 
    if (retrieve_local_offset_ned_meas(_target_pos_rel_meas_NED)){

        _last_target_pos_rel_origin_NED = _target_pos_rel_meas_NED;
        const AP_AHRS &_ahrs = AP::ahrs();
        Vector3f _vehicle_pos_NED;
        if (_ahrs.get_relative_position_NED_origin(_vehicle_pos_NED)) {
                _last_vehicle_pos_NED = _vehicle_pos_NED;
            }
        return true;
    }

    Vector3f target_vec_unit_body;
    if (retrieve_los_meas(target_vec_unit_body)) {
        const struct inertial_data_frame_s *inertial_data_delayed = (*_inertial_history)[0];

        const bool target_vec_valid = target_vec_unit_body.projected(_approach_vector_body).dot(_approach_vector_body) > 0.0f;
        const Vector3f target_vec_unit_ned = inertial_data_delayed->Tbn * target_vec_unit_body;
        const Vector3f approach_vector_NED = inertial_data_delayed->Tbn * _approach_vector_body;
        const bool alt_valid = (rangefinder_alt_valid && rangefinder_alt_m > 0.0f) || (_backend->distance_to_target() > 0.0f);
        if (target_vec_valid && alt_valid) {
            // distance to target and distance to target along approach vector
            float dist_to_target, dist_to_target_along_av;
            // figure out ned camera orientation w.r.t its offset
            Vector3f cam_pos_ned;
            if (!_cam_offset.get().is_zero()) {
                // user has specifed offset for camera
                // take its height into account while calculating distance
                cam_pos_ned = inertial_data_delayed->Tbn * _cam_offset;
            }
            if (_backend->distance_to_target() > 0.0f) {
                // sensor has provided distance to landing target
                dist_to_target = _backend->distance_to_target();
            } else {
                // sensor only knows the horizontal location of the landing target
                // rely on rangefinder for the vertical target
                dist_to_target_along_av = MAX(rangefinder_alt_m - cam_pos_ned.projected(approach_vector_NED).length(), 0.0f);
                dist_to_target = dist_to_target_along_av / target_vec_unit_ned.projected(approach_vector_NED).length();
            }

            // Compute camera position relative to IMU
            const Vector3f accel_pos_ned = inertial_data_delayed->Tbn * AP::ins().get_imu_pos_offset(AP::ahrs().get_primary_accel_index());
            const Vector3f cam_pos_ned_rel_imu = cam_pos_ned - accel_pos_ned;

            // Compute target position relative to IMU
            _target_pos_rel_meas_NED = (target_vec_unit_ned * dist_to_target) + cam_pos_ned_rel_imu;

            // store the current relative down position so that if we need to retry landing, we know at this height landing target can be found
            const AP_AHRS &_ahrs = AP::ahrs();
            Vector3f pos_NED;
            if (_ahrs.get_relative_position_NED_origin(pos_NED)) {
                _last_target_pos_rel_origin_NED.z = pos_NED.z;
                _last_vehicle_pos_NED = pos_NED;
            }
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
    for (uint8_t i=1; i<_inertial_history->available(); i++) {
        const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
        _target_vel_rel_out_NE.x -= inertial_data->correctedVehicleDeltaVelocityNED.x;
        _target_vel_rel_out_NE.y -= inertial_data->correctedVehicleDeltaVelocityNED.y;
        _target_pos_rel_out_NE.x += _target_vel_rel_out_NE.x * inertial_data->dt;
        _target_pos_rel_out_NE.y += _target_vel_rel_out_NE.y * inertial_data->dt;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    const Matrix3f& Tbn = (*_inertial_history)[_inertial_history->available()-1]->Tbn;
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

    // store the landing target as a offset from current position. This is used in landing retry
    Vector2f last_target_loc_rel_origin_2d;
    get_target_position_cm(last_target_loc_rel_origin_2d);
    _last_target_pos_rel_origin_NED.x = last_target_loc_rel_origin_2d.x * 0.01f;
    _last_target_pos_rel_origin_NED.y = last_target_loc_rel_origin_2d.y * 0.01f;

    // record the last time there was a target output
    _last_valid_target_ms = AP_HAL::millis();
}

// Write a precision landing entry
void AC_PrecLand::Write_Precland()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    Vector3f target_pos_meas;
    Vector2f target_pos_rel;
    Vector2f target_vel_rel;
    get_target_position_relative_cm(target_pos_rel);
    get_target_velocity_relative_cms(target_vel_rel);
    get_target_position_measurement_cm(target_pos_meas);

    const struct log_Precland pkt {
        LOG_PACKET_HEADER_INIT(LOG_PRECLAND_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : healthy(),
        target_acquired : target_acquired(),
        pos_x           : target_pos_rel.x,
        pos_y           : target_pos_rel.y,
        vel_x           : target_vel_rel.x,
        vel_y           : target_vel_rel.y,
        meas_x          : target_pos_meas.x,
        meas_y          : target_pos_meas.y,
        meas_z          : target_pos_meas.z,
        last_meas       : last_backend_los_meas_ms(),
        ekf_outcount    : ekf_outlier_count(),
        estimator       : (uint8_t)_estimator_type
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// singleton instance
AC_PrecLand *AC_PrecLand::_singleton;

namespace AP {

AC_PrecLand *ac_precland()
{
    return AC_PrecLand::get_singleton();
}

}
