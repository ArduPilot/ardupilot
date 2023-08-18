/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "AP_OSD_config.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_config.h>
#include <AP_OLC/AP_OLC.h>
#include <AP_MSP/msp.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RPM/AP_RPM_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AC_Fence/AC_Fence_config.h>

class AP_OSD_Backend;
class AP_MSP;

#define AP_OSD_NUM_DISPLAY_SCREENS 4
#if OSD_PARAM_ENABLED
#define AP_OSD_NUM_PARAM_SCREENS 2
#else
#define AP_OSD_NUM_PARAM_SCREENS 0
#endif
#define AP_OSD_NUM_SCREENS (AP_OSD_NUM_DISPLAY_SCREENS + AP_OSD_NUM_PARAM_SCREENS)

#define PARAM_INDEX(key, idx, group) (uint32_t(uint32_t(key) << 23 | uint32_t(idx) << 18 | uint32_t(group)))
#define PARAM_TOKEN_INDEX(token) PARAM_INDEX(AP_Param::get_persistent_key(token.key), token.idx, token.group_element)

#define AP_OSD_NUM_SYMBOLS 91
#define OSD_MAX_INSTANCES 2
/*
  class to hold one setting
 */
class AP_OSD_Setting
{
public:
    AP_Int8 enabled;
    AP_Int8 xpos;
    AP_Int8 ypos;

    AP_OSD_Setting(bool enabled = 0, uint8_t x = 0, uint8_t y = 0);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    const float default_enabled;
    const float default_xpos;
    const float default_ypos;
};

class AP_OSD;

class AP_OSD_AbstractScreen
{
    friend class AP_OSD;
public:
    // constructor
    AP_OSD_AbstractScreen() {}
    virtual void draw(void) {}

    void set_backend(AP_OSD_Backend *_backend);

    AP_Int8 enabled;
    AP_Int16 channel_min;
    AP_Int16 channel_max;

protected:
    bool check_option(uint32_t option);
#if HAL_WITH_MSP_DISPLAYPORT
    virtual uint8_t get_txt_resolution() const {
        return 0;
    }
    virtual uint8_t get_font_index() const {
        return 0;
    }
#endif
    enum unit_type {
        ALTITUDE=0,
        SPEED=1,
        VSPEED=2,
        DISTANCE=3,
        DISTANCE_LONG=4,
        TEMPERATURE=5,
        UNIT_TYPE_LAST=6,
    };

    char u_icon(enum unit_type unit);
    float u_scale(enum unit_type unit, float value);

    AP_OSD_Backend *backend;
    AP_OSD *osd;

    static uint8_t symbols_lookup_table[AP_OSD_NUM_SYMBOLS];
};

#if OSD_ENABLED
/*
  class to hold one screen of settings
 */
class AP_OSD_Screen : public AP_OSD_AbstractScreen
{
public:
    // constructor
    AP_OSD_Screen();

    // skip the drawing if we are not using a font based backend. This saves a lot of flash space when
    // using the MSP OSD system on boards that don't have a MAX7456
#if HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT
    void draw(void) override;
#endif

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

#if HAL_WITH_MSP_DISPLAYPORT
    uint8_t get_txt_resolution() const override {
        return txt_resolution;
    }
    uint8_t get_font_index() const override {
        return font_index;
    }
#endif
private:
    friend class AP_MSP;
    friend class AP_MSP_Telem_Backend;
    friend class AP_MSP_Telem_DJI;

    static const uint8_t message_visible_width = 26;
    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    static constexpr float ah_max_pitch = DEG_TO_RAD * 20;
    //typical fpv camera has 80deg vertical field of view, 16 row of chars
    static constexpr float ah_pitch_rad_to_char = 16.0f/(DEG_TO_RAD * 80);

    enum class VoltageType {
        VOLTAGE,
        RESTING_VOLTAGE,
        AVG_CELL,
        RESTING_CELL,
    };

    AP_OSD_Setting altitude{true, 23, 8};
    AP_OSD_Setting bat_volt{true, 24, 1};
    AP_OSD_Setting rssi{true, 1, 1};
    AP_OSD_Setting link_quality{false,1,1};
    AP_OSD_Setting restvolt{false, 24, 2};
    AP_OSD_Setting avgcellvolt{false, 24, 3};
    AP_OSD_Setting avgcellrestvolt{false, 24, 4};
    AP_OSD_Setting current{true, 25, 2};
    AP_OSD_Setting batused{true, 23, 3};
    AP_OSD_Setting sats{true, 1, 3};
    AP_OSD_Setting fltmode{true, 2, 8};
    AP_OSD_Setting message{true, 2, 6};
    AP_OSD_Setting gspeed{true, 2, 14};
    AP_OSD_Setting horizon{true, 14, 8};
    AP_OSD_Setting home{true, 14, 1};
    AP_OSD_Setting throttle{true, 24, 11};
    AP_OSD_Setting heading{true, 13, 2};
    AP_OSD_Setting compass{true, 15, 3};
    AP_OSD_Setting wind{false, 2, 12};
    AP_OSD_Setting aspeed{false, 2, 13};
    AP_OSD_Setting aspd1;
    AP_OSD_Setting aspd2;
    AP_OSD_Setting vspeed{true, 24, 9};
#if AP_RPM_ENABLED
    AP_OSD_Setting rrpm{false, 2, 2};
#endif
#if HAL_WITH_ESC_TELEM
    AP_OSD_Setting esc_temp {false, 24, 13};
    AP_OSD_Setting esc_rpm{false, 22, 12};
    AP_OSD_Setting esc_amps{false, 24, 14};
#endif
    AP_OSD_Setting gps_latitude{true, 9, 13};
    AP_OSD_Setting gps_longitude{true, 9, 14};
    AP_OSD_Setting roll_angle;
    AP_OSD_Setting pitch_angle;
    AP_OSD_Setting temp;
#if BARO_MAX_INSTANCES > 1
    AP_OSD_Setting btemp;
#endif
    AP_OSD_Setting hdop;
    AP_OSD_Setting waypoint;
    AP_OSD_Setting xtrack_error;
    AP_OSD_Setting dist{false,22,11};
    AP_OSD_Setting stat{false,0,0};
    AP_OSD_Setting flightime{false, 23, 10};
    AP_OSD_Setting climbeff{false,0,0};
    AP_OSD_Setting eff{false, 22, 10};
    AP_OSD_Setting atemp;
    AP_OSD_Setting bat2_vlt;
    AP_OSD_Setting bat2used;
    AP_OSD_Setting current2;
    AP_OSD_Setting clk;
    AP_OSD_Setting callsign;
    AP_OSD_Setting vtx_power;
    AP_OSD_Setting hgt_abvterr{false, 23, 7};
    AP_OSD_Setting fence{false, 14, 9};
    AP_OSD_Setting rngf;
#if HAL_PLUSCODE_ENABLE
    AP_OSD_Setting pluscode;
#endif
    AP_OSD_Setting sidebars{false, 4, 5};

    // MSP OSD only
    AP_OSD_Setting crosshair;
    AP_OSD_Setting home_dist{true, 1, 1};
    AP_OSD_Setting home_dir{true, 1, 1};
    AP_OSD_Setting power{true, 1, 1};
    AP_OSD_Setting cell_volt{true, 1, 1};
    AP_OSD_Setting batt_bar{true, 1, 1};
    AP_OSD_Setting arming{true, 1, 1};

#if HAL_WITH_MSP_DISPLAYPORT
    // Per screen HD resolution options (currently supported only by DisplayPort)
    AP_Int8 txt_resolution;
    AP_Int8 font_index;
#endif

    void draw_altitude(uint8_t x, uint8_t y);
    void draw_bat_volt(uint8_t instance,VoltageType  type,uint8_t x, uint8_t y);
    void draw_bat_volt(uint8_t x, uint8_t y);
    void draw_avgcellvolt(uint8_t x, uint8_t y);
    void draw_avgcellrestvolt(uint8_t x, uint8_t y);
    void draw_restvolt(uint8_t x, uint8_t y);
    void draw_rssi(uint8_t x, uint8_t y);
    void draw_link_quality(uint8_t x, uint8_t y);
    void draw_current(uint8_t x, uint8_t y);
    void draw_current(uint8_t instance, uint8_t x, uint8_t y);
    void draw_batused(uint8_t x, uint8_t y);
    void draw_batused(uint8_t instance, uint8_t x, uint8_t y);
    void draw_sats(uint8_t x, uint8_t y);
    void draw_fltmode(uint8_t x, uint8_t y);
    void draw_message(uint8_t x, uint8_t y);
    void draw_gspeed(uint8_t x, uint8_t y);
    void draw_horizon(uint8_t x, uint8_t y);
    void draw_home(uint8_t x, uint8_t y);
    void draw_throttle(uint8_t x, uint8_t y);
    void draw_heading(uint8_t x, uint8_t y);
#if AP_RPM_ENABLED
    void draw_rrpm(uint8_t x, uint8_t y);
#endif
#ifdef HAL_OSD_SIDEBAR_ENABLE
    void draw_sidebars(uint8_t x, uint8_t y);
#endif
    void draw_compass(uint8_t x, uint8_t y);
    void draw_wind(uint8_t x, uint8_t y);
    void draw_aspeed(uint8_t x, uint8_t y);
    void draw_aspd1(uint8_t x, uint8_t y);
    void draw_aspd2(uint8_t x, uint8_t y);
    void draw_vspeed(uint8_t x, uint8_t y);
#if HAL_PLUSCODE_ENABLE
    void draw_pluscode(uint8_t x, uint8_t y);
#endif
    //helper functions
    void draw_speed(uint8_t x, uint8_t y, float angle_rad, float magnitude);
    void draw_distance(uint8_t x, uint8_t y, float distance);
    char get_arrow_font_index (int32_t angle_cd);
#if HAL_WITH_ESC_TELEM
    void draw_esc_temp(uint8_t x, uint8_t y);
    void draw_esc_rpm(uint8_t x, uint8_t y);
    void draw_esc_amps(uint8_t x, uint8_t y);
#endif
    void draw_gps_latitude(uint8_t x, uint8_t y);
    void draw_gps_longitude(uint8_t x, uint8_t y);
    void draw_roll_angle(uint8_t x, uint8_t y);
    void draw_pitch_angle(uint8_t x, uint8_t y);
    void draw_temp(uint8_t x, uint8_t y);
#if BARO_MAX_INSTANCES > 1
    void draw_btemp(uint8_t x, uint8_t y);
#endif
    void draw_hdop(uint8_t x, uint8_t y);
    void draw_waypoint(uint8_t x, uint8_t y);
    void draw_xtrack_error(uint8_t x, uint8_t y);
    void draw_dist(uint8_t x, uint8_t y);
    void draw_stat(uint8_t x, uint8_t y);
    void draw_flightime(uint8_t x, uint8_t y);
    void draw_climbeff(uint8_t x, uint8_t y);
    void draw_eff(uint8_t x, uint8_t y);
    void draw_atemp(uint8_t x, uint8_t y);
    void draw_bat2_vlt(uint8_t x, uint8_t y);
    void draw_bat2used(uint8_t x, uint8_t y);
    void draw_clk(uint8_t x, uint8_t y);
    void draw_callsign(uint8_t x, uint8_t y);
    void draw_current2(uint8_t x, uint8_t y);
    void draw_vtx_power(uint8_t x, uint8_t y);
    void draw_hgt_abvterr(uint8_t x, uint8_t y);
#if AP_FENCE_ENABLED
    void draw_fence(uint8_t x, uint8_t y);
#endif
    void draw_rngf(uint8_t x, uint8_t y);

    struct {
        bool load_attempted;
        const char *str;
    } callsign_data;
};
#endif // OSD_ENABLED

#if OSD_PARAM_ENABLED
/*
  class to hold one setting
 */
class AP_OSD_ParamSetting
{
public:

    AP_Int8 enabled;
    AP_Int8 xpos;
    AP_Int8 ypos;

    // configured index.
    AP_Int32 _param_group;
    AP_Int16 _param_key;
    AP_Int8  _param_idx;
    // metadata
    AP_Float _param_min;
    AP_Float _param_max;
    AP_Float _param_incr;
    AP_Int8 _type;

    // parameter number
    uint8_t _param_number;
    AP_Param* _param;
    ap_var_type _param_type;
    AP_Param::ParamToken _current_token;

    // structure to contain setting constraints for important settings
    struct ParamMetadata {
        float min_value;
        float max_value;
        float increment;
        uint8_t values_max;
        const char** values;
    };
    // compact structure used to hold default values for static initialization
    struct Initializer {
        uint8_t index;
        AP_Param::ParamToken token;
        int8_t type;
    };

    static const ParamMetadata _param_metadata[];

    AP_OSD_ParamSetting() {};
    AP_OSD_ParamSetting(uint8_t param_number);
    AP_OSD_ParamSetting(const Initializer& initializer);

    // initialize the setting from the configured information
    void update();
    // grab the parameter name
    void copy_name(char* name, size_t len) const {
        _param->copy_name_token(_current_token, name, len);
        if (len > 16) name[16] = 0;
    }
    // copy the name converting FOO_BAR_BAZ to FooBarBaz
    void copy_name_camel_case(char* name, size_t len) const;
    // set the ranges from static metadata
    bool set_from_metadata();
    bool set_by_name(const char* name, uint8_t config_type, float pmin=0, float pmax=0, float pincr=0);
    void guess_ranges(bool force = false);
    void save_as_new();

    const ParamMetadata* get_custom_metadata() const {
        return (_type > 0 ? &_param_metadata[_type - 1] : nullptr);
    }

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    float default_enabled;
    float default_ypos;
    float default_param_group;
    float default_param_idx;
    float default_param_key;
    float default_type;

};

/*
  class to hold one screen of settings
 */
class AP_OSD_ParamScreen : public AP_OSD_AbstractScreen
{
public:
    AP_OSD_ParamScreen(uint8_t index);

    enum class Event {
        NONE,
        MENU_ENTER,
        MENU_UP,
        MENU_DOWN,
        MENU_EXIT
    };

    enum class MenuState {
        PARAM_SELECT,
        PARAM_VALUE_MODIFY,
        PARAM_PARAM_MODIFY
    };

    static const uint8_t NUM_PARAMS = 9;
    static const uint8_t SAVE_PARAM = NUM_PARAMS + 1;

#if OSD_ENABLED && (HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT)
    void draw(void) override;
#endif
#if HAL_GCS_ENABLED
    void handle_write_msg(const mavlink_osd_param_config_t& packet, const class GCS_MAVLINK& link);
    void handle_read_msg(const mavlink_osd_param_show_config_t& packet, const class GCS_MAVLINK& link);
#endif
    // get a setting and associated metadata
    AP_OSD_ParamSetting* get_setting(uint8_t param_idx);
    // Save button co-ordinates
    AP_Int8 save_x;
    AP_Int8 save_y;

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_OSD_ParamSetting params[NUM_PARAMS];

    void save_parameters();
#if OSD_ENABLED
    void update_state_machine();
    void draw_parameter(uint8_t param_number, uint8_t x, uint8_t y);
    void modify_parameter(uint8_t number, Event ev);
    void modify_configured_parameter(uint8_t number, Event ev);

#if AP_RC_CHANNEL_ENABLED
    Event map_rc_input_to_event() const;
    RC_Channel::AuxSwitchPos get_channel_pos(uint8_t rcmapchan) const;
#endif

    uint8_t _selected_param = 1;
    MenuState _menu_state = MenuState::PARAM_SELECT;
    Event _last_rc_event = Event::NONE;

    // start time of the current button press
    uint32_t _transition_start_ms;
    // timeout of the current button press
    uint32_t _transition_timeout_ms;
    // number of consecutive times the current transition has happened
    uint32_t _transition_count;
#endif
    uint16_t _requires_save;
};

#endif // OSD_PARAM_ENABLED

class AP_OSD
{
public:
    friend class AP_OSD_Screen;
    friend class AP_MSP;
    friend class AP_MSP_Telem_Backend;
    friend class AP_OSD_ParamScreen;
    //constructor
    AP_OSD();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OSD);

    // get singleton instance
    static AP_OSD *get_singleton()
    {
        return _singleton;
    }

    // init - perform required initialisation
    void init();

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    enum osd_types {
        OSD_NONE=0,
        OSD_MAX7456=1,
        OSD_SITL=2,
        OSD_MSP=3,
        OSD_TXONLY=4,
        OSD_MSP_DISPLAYPORT=5
    };

    bool init_backend(const osd_types type, const uint8_t instance);

    enum switch_method {
        TOGGLE=0,
        PWM_RANGE=1,
        AUTO_SWITCH=2,
    };

    AP_Int8 osd_type;
    AP_Int8 osd_type2; // additional backend active in parallel
    AP_Int8 font_num;
    AP_Int32 options;

#if OSD_ENABLED
    AP_Int8 rc_channel;
    AP_Int8 sw_method;

    AP_Int8 v_offset;
    AP_Int8 h_offset;

    AP_Int8 warn_rssi;
    AP_Int8 warn_nsat;
    AP_Int32 warn_terr;
    AP_Float warn_avgcellvolt;
    AP_Float max_battery_voltage;
    AP_Int8 cell_count;
    AP_Float warn_restvolt;
    AP_Float warn_avgcellrestvolt;
    AP_Float warn_batvolt;
    AP_Float warn_bat2volt;
    AP_Int8 msgtime_s;
    AP_Int8 arm_scr;
    AP_Int8 disarm_scr;
    AP_Int8 failsafe_scr;
    AP_Int32 button_delay_ms;

    enum {
        OPTION_DECIMAL_PACK = 1U<<0,
        OPTION_INVERTED_WIND = 1U<<1,
        OPTION_INVERTED_AH_ROLL = 1U<<2,
        OPTION_IMPERIAL_MILES = 1U<<3,
        OPTION_DISABLE_CROSSHAIR = 1U<<4,
        OPTION_BF_ARROWS = 1U<<5,
    };

    enum {
        UNITS_METRIC=0,
        UNITS_IMPERIAL=1,
        UNITS_SI=2,
        UNITS_AVIATION=3,
        UNITS_LAST=4,
    };

    AP_Int8 units;

    AP_OSD_Screen screen[AP_OSD_NUM_DISPLAY_SCREENS];

    struct NavInfo {
        float wp_distance;
        int32_t wp_bearing;
        float wp_xtrack_error;
        uint16_t wp_number;
    };

    struct StatsInfo {
        uint32_t last_update_ms;
        float last_distance_m;
        float max_dist_m;
        float max_alt_m;
        float max_speed_mps;
        float max_airspeed_mps;
        float max_current_a;
        float avg_current_a;
        float min_voltage_v = FLT_MAX;
        float min_rssi = FLT_MAX;   // 0-1
        int16_t max_esc_temp;
    };

    void set_nav_info(NavInfo &nav_info);
    const volatile StatsInfo& get_stats_info() const {return _stats;};
    // disable the display
    void disable() {
        _disable = true;
    }
    // enable the display
    void enable() {
        _disable = false;
    }

    AP_OSD_AbstractScreen& get_screen(uint8_t idx) {
#if OSD_PARAM_ENABLED
        if (idx >= AP_OSD_NUM_DISPLAY_SCREENS) {
            return param_screen[idx - AP_OSD_NUM_DISPLAY_SCREENS];
        }
#endif
        return screen[idx];
    }

    // Check whether arming is allowed
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;
    bool is_readonly_screen() const { return current_screen < AP_OSD_NUM_DISPLAY_SCREENS; }
    // get the current screen
    uint8_t get_current_screen() const { return current_screen; };
#endif // OSD_ENABLED
#if OSD_PARAM_ENABLED
    AP_OSD_ParamScreen param_screen[AP_OSD_NUM_PARAM_SCREENS] { 0, 1 };
    // return a setting for use by TX based OSD
    AP_OSD_ParamSetting* get_setting(uint8_t screen_idx, uint8_t param_idx) {
        if (screen_idx >= AP_OSD_NUM_PARAM_SCREENS) {
            return nullptr;
        }
        return param_screen[screen_idx].get_setting(param_idx);
    }
#endif
    // handle OSD parameter configuration
#if HAL_GCS_ENABLED
    void handle_msg(const mavlink_message_t &msg, const class GCS_MAVLINK& link);
#endif

    // allow threads to lock against OSD update
    HAL_Semaphore &get_semaphore(void) {
        return _sem;
    }

private:
    void osd_thread();
#if OSD_ENABLED
    void update_osd();
    void update_stats();
    void update_current_screen();
    void next_screen();

    //variables for screen switching
    uint8_t current_screen;
    uint16_t previous_channel_value;
    bool switch_debouncer;
    uint32_t last_switch_ms;
    struct NavInfo nav_info;
    int8_t previous_pwm_screen;
    int8_t pre_fs_screen;
    bool was_armed;
    bool was_failsafe;
    bool _disable;

    StatsInfo _stats;
#endif
    AP_OSD_Backend *_backends[OSD_MAX_INSTANCES];
    uint8_t _backend_count;

    static AP_OSD *_singleton;
    // multi-thread access support
    HAL_Semaphore _sem;
};

namespace AP
{
AP_OSD *osd();
};
