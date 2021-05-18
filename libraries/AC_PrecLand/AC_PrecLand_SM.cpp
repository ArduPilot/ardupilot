//
// Created by khancyr on 11/02/2021.
//
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include "AC_PrecLand_SM.h"

const AP_Param::GroupInfo AC_PrecLand_SM::var_info[] = {

        // @Param: ENABLED
        // @DisplayName: Precision Land SM enabled/disabled
        // @Description: Precision Land SM enabled/disabled
        // @Values: 0:Disabled, 1:Enabled
        // @User: Advanced
        AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand_SM, _enabled, 0, AP_PARAM_FLAG_ENABLE),

        // @Param: SRCH_ALT
        // @DisplayName: Search for beacon starting alt
        // @Description: Search for beacon starting alt
        // @Units: cm
        // @Range: 1000 3000
        // @User: Advanced
        AP_GROUPINFO("SRCH_ALT", 1, AC_PrecLand_SM, _search_start_alt, 1500),

        // @Param: SRCH_S_ALT
        // @DisplayName: Search for beacon stoping alt
        // @Description: Search for beacon stoping alt
        // @Units: cm
        // @Range: 500 1000
        // @User: Advanced
        AP_GROUPINFO("SRCH_S_ALT", 2, AC_PrecLand_SM, _search_stop_alt, 1000),

        // @Param: DESC_S_ALT
        // @DisplayName: Descend and alignate stop alt
        // @Description: Descend and alignate stop alt. We should be alignate correctly at this point or we will retry. Further descend won't be stopped.
        // @Units: cm
        // @Range: 100 200
        // @User: Advanced
        AP_GROUPINFO("DESC_S_ALT", 3, AC_PrecLand_SM, _descend_stop_alt, 200),

        // @Param: RETRY
        // @DisplayName: Number of RETRY
        // @Description: Number of RETRY
        // @User: Advanced
        AP_GROUPINFO("RETRY", 4, AC_PrecLand_SM, _retry_max, 3),

        // @Param: EM_BHV
        // @DisplayName: BEHAVIOR in case of battery failsafe.
        // @Description: BEHAVIOR in case of battery failsafe.
        // @Bitmask: 0:No changes, 1:Disable SM, 2: Disable Retry
        // @User: Advanced
        AP_GROUPINFO("EM_BHV", 5, AC_PrecLand_SM, _emergency_behavior, 0),

        // @Param: D_SPEED
        // @DisplayName: Descend speed max.
        // @Description: Descend speed max.
        // @Units: cm/s
        // @Range: 30 200
        // @Increment: 10
        // @User: Advanced
        AP_GROUPINFO("D_SPEED", 6, AC_PrecLand_SM, _descend_speed_max, 75),

        // @Param: F_ALT
        // @DisplayName: Failure altitude to reach before do safe landing.
        // @Description: Failure altitude to reach before do safe landing.
        // @Units: cm
        // @Range: 200 500
        // @User: Advanced
        AP_GROUPINFO("F_ALT", 7, AC_PrecLand_SM, _failure_alt, 300),

        // @Param: F_POSN
        // @DisplayName: Failure position oriented on North in m.
        // @Description: Position that will be reach for safe landing in case of precland failure. This a position in m on North of the home point
        // @Units: m
        // @Range: 2 5
        // @User: Advanced
        AP_GROUPINFO("F_POSN", 8, AC_PrecLand_SM, _failure_posn, 2),

        AP_GROUPEND
};

AC_PrecLand_SM::AC_PrecLand_SM(AC_PosControl& pos_control, AC_Loiter& loiter_nav, AC_WPNav& wp_nav,
        AC_PrecLand& precland)
        :
        _pos_control(pos_control),
        _loiter_nav(loiter_nav),
        _wp_nav(wp_nav),
        _precland(precland)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_PrecLand_SM::reset()
{
    _state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    _retry_count = 0;
    _in_failure = false;
    _wp_nav_is_active = false;
    _initialised = false;
    _is_final_landing = false;
    _wp_nav.reset_radius();
    _wp_nav.reset_rangefinder_use();
    _is_emergency = false;
}

void AC_PrecLand_SM::init()
{
    if (!_enabled.get() || !_precland.enabled() || !AP::arming().is_armed()) {
        return;
    }
    reset();
    if (!AP::ahrs().get_position(_init_location)) {
        _init_location = AP::ahrs().get_home();  // default to home in case of issue ...
    }
    _initialised = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Initialise PLSM");
    search_start();
}

void AC_PrecLand_SM::run()
{
    if (!_enabled.get() || !_initialised || !_precland.enabled()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_log_ms > 40) {  // 25Hz
        last_log_ms = now;
        write_log();
    }

    if (_is_emergency && _emergency_behavior & EM_BHV_DISABLE_SM) {
        return;
    }
    if (!AP::arming().is_armed()) {
        if (_state == PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL) {
            gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Landed on target");
            _state = PLD_STATE::PRECISION_LAND_STATE_SUCCESS;
        } else {
            if (_state != PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE
                    && _state != PLD_STATE::PRECISION_LAND_STATE_SUCCESS) {
                _state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
                gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Unexpected Land");
            }
        }
    }

    switch (_state) {
    case PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE:
    case PLD_STATE::PRECISION_LAND_STATE_SUCCESS:break;
    case PLD_STATE::PRECISION_LAND_STATE_SEARCH:search_run();
        break;
    case PLD_STATE::PRECISION_LAND_STATE_DESCEND:descend_run();
        break;
    case PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL:descend_final_run();
        break;
    case PLD_STATE::PRECISION_LAND_STATE_RETRY:retry_run();
        break;
    case PLD_STATE::PRECISION_LAND_STATE_FAILED:failed_run();
        break;
    }
}

void AC_PrecLand_SM::search_start()
{
    _state = PLD_STATE::PRECISION_LAND_STATE_SEARCH;

    _wp_nav.set_radius(WP_RADIUS);
    _wp_nav.use_rangefinder(false);
    _wp_nav.wp_and_spline_init();

    if (_precland.target_acquired()) {
        Vector2f target_pos;
        if (_precland.get_target_position_cm(target_pos)) {
            _wp_nav.set_wp_destination(
                    Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get() - 500)), false);
            _wp_nav.get_wp_destination_loc(_search_target);
        }
    } else {
        _search_target = _init_location;
        _search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
        _wp_nav.set_wp_destination_loc(_search_target);
    }
    _first_pass = true;
    _doing_first_pass = false;
    _wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Search start");
}

void AC_PrecLand_SM::search_run()
{
    if (_alt_above_ground_cm >= _search_start_alt.get() || _doing_first_pass) {
        if (_wp_nav.reached_wp_destination()) {
            _doing_first_pass = false;
        }
        return;
    }

    // if we are under the stop alt, we get up
    if (_alt_above_ground_cm < _search_stop_alt) {
        // get up to stop alt This unsure we always do the same patern and garanty the same descend phase
        if (_first_pass) {
            _search_target.set_alt_cm((_search_start_alt.get() - 500), Location::AltFrame::ABOVE_HOME);
            _wp_nav.set_wp_destination_loc(_search_target);
            _doing_first_pass = true;
            _first_pass = false;
            return;
        } else {
            // inform user
            _state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
            failed_start();
            return;
        }
    }

    _first_pass = false;
    _doing_first_pass = false;
    // if we lock the target, go on the target
    if (_precland.target_acquired()) {
        if (_alt_above_ground_cm > _search_stop_alt + 1000) {
            Vector2f target_pos;
            if (_precland.get_target_position_cm(target_pos)) {
                _wp_nav.set_wp_destination(
                        Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_stop_alt.get())), false);
            }
        } else {
            descend_start();
            return;
        }
    } else {
        // search the target
        // circle to stop alt
    }
}

void AC_PrecLand_SM::descend_start()
{
    gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Descend start");
    _state = PLD_STATE::PRECISION_LAND_STATE_DESCEND;

    _avg_filter.reset();
    _avg_pos_error = 0.0f;
    _wp_nav.reset_radius();
    _wp_nav.reset_rangefinder_use();
    _loiter_nav.init_target();
    _pos_control.init_xy_controller();
    _last_target_time = AP_HAL::millis();
    _max_descend_speed = MIN(-_pos_control.get_max_speed_down(), _descend_speed_max);
    _wp_nav_is_active = false;
}

void AC_PrecLand_SM::calculate_slowdown()
{
    // we get back using loiter_nav for tighter locking on target. We will just slowdown the descend according to the pos error
    // recenter and descend
    if (_precland.target_acquired()) {
        _last_target_time = AP_HAL::millis();
        _avg_pos_error = _avg_filter.apply(_pos_control.get_pos_error_xy());
    } else {
        if (AP_HAL::millis() - _last_target_time > 500) {  //timeout of 0.5sec for launching a retry
            // inform user
            gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Lost target for more than 0.5s");
            _state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
            failed_start();
            return;
        }
    }

    float land_slowdown = 0.0f;
    if (_avg_pos_error > _acceptable_error_cm) {
        // slowdown max at 2 * _acceptable_error_cm
        land_slowdown = MAX(0.0f, _avg_pos_error * (_max_descend_speed / (_acceptable_error_cm * 2.0f)));
    }
    _precland_sm_climb_rate = MIN(-MIN_DESCENT_SPEED, -_max_descend_speed + land_slowdown);
}
void AC_PrecLand_SM::descend_run()
{
    calculate_slowdown();
    if (_alt_above_ground_cm <= _descend_stop_alt.get()) {
        if (_avg_pos_error <= _acceptable_error_cm) {
            descend_final_start();
            return;
        } else {
            // try to give 1s on stationnary to alignate better
            if (_descend_timer == 0) {
                _descend_timer = AP_HAL::millis();
            }
            if (AP_HAL::millis() - _descend_timer > 1000) {
                // inform user
                gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Failed to lock target accuratly");
                _state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
                failed_start();
                return;
            } else {
                _precland_sm_climb_rate = 0.1f; // slightly positive to ensure a little thrust up to stop inertia !
                return;
            }
        }
    }

}

void AC_PrecLand_SM::descend_final_start()
{
    gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Final descend start");
    _state = PLD_STATE::PRECISION_LAND_STATE_DESCEND_FINAL;
    _max_descend_speed = MIN(-_pos_control.get_max_speed_down(), _descend_speed_max) * 0.5f;
}

void AC_PrecLand_SM::descend_final_run()
{
    if (_alt_above_ground_cm <= (_descend_stop_alt * 0.25f) || _alt_above_ground_cm <= 35.0f) {
        _is_final_landing = true;  // unsure smooth landing by the land controler
    }
    calculate_slowdown();
}

void AC_PrecLand_SM::failed_start()
{
    gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Failed start");
    _is_final_landing = false;
    bool allow_retry = !(_is_emergency && _emergency_behavior & EM_BHV_DISABLE_RETRY);
    if (allow_retry && _retry_count < _retry_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Retry %d", _retry_count);
        _retry_count++;
        retry_start();
        return;
    }
    _state = PLD_STATE::PRECISION_LAND_STATE_FAILED;
    _search_target = _init_location;
    _search_target.set_alt_cm(_failure_alt, Location::AltFrame::ABOVE_HOME);
    _search_target.offset(_failure_posn, 0);  // set safe land at _failure_posn of north of the beacon.
    _wp_nav.set_wp_destination_loc(_search_target);
    _wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "PLSM: Failed to retry going to safe position");
}

void AC_PrecLand_SM::failed_run()
{
    // case retry count failure, land at X m from the home without precland
    if (_wp_nav.reached_wp_destination()) {
        // else do normal land
        _in_failure = true;
        _wp_nav_is_active = false;
        _state = PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    }
}

void AC_PrecLand_SM::retry_start()
{
    _state = PLD_STATE::PRECISION_LAND_STATE_RETRY;

    _wp_nav.use_rangefinder(false);
    _wp_nav.wp_and_spline_init();

    if (_precland.target_acquired()) {
        Vector2f target_pos;
        if (_precland.get_target_position_cm(target_pos)) {
            _wp_nav.set_wp_destination(
                    Vector3f(target_pos.x, target_pos.y, static_cast<float>(_search_start_alt.get())), false);
            _wp_nav.get_wp_destination_loc(_search_target);
        }
    } else {
        _search_target = _init_location;
        _search_target.set_alt_cm(static_cast<int32_t>(_search_start_alt), Location::AltFrame::ABOVE_HOME);
        _wp_nav.set_wp_destination_loc(_search_target);
    }
    _wp_nav_is_active = true;
    gcs().send_text(MAV_SEVERITY_INFO, "retry_start");
}

void AC_PrecLand_SM::retry_run()
{
    if (_wp_nav.reached_wp_destination()) {
        search_start();
    }
}

void AC_PrecLand_SM::write_log()
{
    struct log_Precland_SM pkt = {
            LOG_PACKET_HEADER_INIT(LOG_PRECLAND_SM_MSG),
            .time_us               = AP_HAL::micros64(),
            .state                 =  static_cast<uint8_t>(_state),
            .alt_above_ground      =  _alt_above_ground_cm,
            .climb_rate            =  _precland_sm_climb_rate,
            .avg_pos_error         =  _avg_pos_error,
            .retry                 = _retry_count,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

