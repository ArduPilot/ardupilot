//
// Created by khancyr on 11/02/2021.
//

#pragma once
#include <AP_Param/AP_Param.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_PrecLand/AC_PrecLand.h>
#include <Filter/AverageFilter.h>

class AC_PrecLand_SM {
public:
    AC_PrecLand_SM(AC_PosControl &pos_control, AC_Loiter &loiter_nav, AC_WPNav &wp_nav, AC_PrecLand &precland);
    void reset();
    void init();
    void run();
    bool is_enable() const {
        return _enabled;
    }
    bool is_active() const {
        return _initialised && _state != PLD_STATE::PRECISION_LAND_STATE_NOT_ACTIVE;
    }

    bool wp_nav_is_active() const {
        return _wp_nav_is_active;
    }

    float get_desired_cmb_rate() const {
        return _precland_sm_climb_rate;
    }

    bool is_final_landing() const {
        return _is_final_landing;
    }

    bool in_failure() const {
        return _in_failure;
    }

    void set_alt_above_ground_cm(int32_t alt_above_ground_cm) {
        _alt_above_ground_cm = alt_above_ground_cm;
    }

    void set_emergency_flag(bool flag) {
        _is_emergency = flag;
    }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];
private:
    enum class PLD_STATE: uint8_t {
        PRECISION_LAND_STATE_NOT_ACTIVE = 0,
        PRECISION_LAND_STATE_SEARCH,
        PRECISION_LAND_STATE_DESCEND,
        PRECISION_LAND_STATE_DESCEND_FINAL,
        PRECISION_LAND_STATE_RETRY,
        PRECISION_LAND_STATE_SUCCESS,
        PRECISION_LAND_STATE_FAILED,
    };
    bool _initialised;
    int32_t _alt_above_ground_cm;
    float _precland_sm_climb_rate;
    bool _wp_nav_is_active;
    PLD_STATE _state;
    AP_Int8 _enabled;
    AP_Int8 _retry_max;
    AP_Int16 _emergency_behavior;
    AP_Int16 _descend_speed_max;
    AP_Int16 _failure_alt;
    AP_Float _failure_posn;
    bool _is_emergency;
    // for EM_BHV parameter
    static constexpr uint8_t EM_BHV_NO_CHANGES = (1<<0);
    static constexpr uint8_t EM_BHV_DISABLE_SM = (1<<1);
    static constexpr uint8_t EM_BHV_DISABLE_RETRY = (1<<2);

    AC_PosControl &_pos_control;
    AC_Loiter &_loiter_nav;
    AC_WPNav &_wp_nav;
    AC_PrecLand &_precland;

    void write_log();
    uint32_t last_log_ms;  // last time we logged

    Location _init_location;
    /*******************/
    void search_start();
    void search_run();
    static constexpr uint32_t WP_RADIUS = 50; // cm
    AP_Int32 _search_start_alt; // in cm
    AP_Int32 _search_stop_alt;
    Location _search_target;
    bool _first_pass;
    bool _doing_first_pass;
    /********************/
    void calculate_slowdown();
    void descend_start();
    void descend_run();
    static constexpr uint8_t AVG_FILTER_SIZE = 100;
    AverageFilter<float, float, AVG_FILTER_SIZE> _avg_filter;
    float _avg_pos_error; //cm
    uint32_t _last_target_time;
    AP_Int32 _descend_stop_alt; //cm
    float _acceptable_error_cm = 15; //TODO param ?
    float _max_descend_speed;
    uint32_t _descend_timer;
    static constexpr float MIN_DESCENT_SPEED = 10.0f;

    void descend_final_start();
    void descend_final_run();
    bool _is_final_landing;

    void failed_start();
    void failed_run();
    uint8_t _retry_count;
    bool _in_failure;

    void retry_start();
    void retry_run();

};

