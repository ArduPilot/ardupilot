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

/*
  control of internal combustion engines (starter, ignition and choke)
 */
#pragma once

#include <AP_Param/AP_Param.h>

class AP_ICEngine {
public:
    // constructor
    AP_ICEngine(const class AP_RPM &_rpm);

    static const struct AP_Param::GroupInfo var_info[];

    // update engine state. Should be called at 10Hz or more
    void update(void);

    // check for throttle override
    bool throttle_override(float &percent);

    enum ICE_State {
        ICE_OFF=0,
        ICE_START_HEIGHT_DELAY=1,
        ICE_START_DELAY=2,
        ICE_STARTING=3,
        ICE_RUNNING=4
    };

    // get current engine control state
    ICE_State get_state(void) const { return state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay);

    // update min throttle for idle governor
    void update_idle_governor(int8_t &min_throttle);

    static AP_ICEngine *get_singleton() { return _singleton; }

private:
    static AP_ICEngine *_singleton;

    const class AP_RPM &rpm;

    enum ICE_State state;

    // filter for RPM value
    LowPassFilterFloat _rpm_filter;
    float filtered_rpm_value;

    // enable library
    AP_Int8 enable;

    // channel for pilot to command engine start, 0 for none
    AP_Int8 start_chan;

    // min pwm on start channel for engine stop
    AP_Int16 start_chan_min_pwm;
    
    // which RPM instance to use
    AP_Int8 rpm_instance;
    
    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;
    
    // pwm values 
    AP_Int16 pwm_ignition_on;
    AP_Int16 pwm_ignition_off;
    AP_Int16 pwm_starter_on;
    AP_Int16 pwm_starter_off;
    
    // RPM above which engine is considered to be running
    AP_Int32 rpm_threshold;

    // time when we started the starter
    uint32_t starter_start_time_ms;

    // time when we last ran the starter
    uint32_t starter_last_run_ms;

    // throttle percentage for engine start
    AP_Int8 start_percent;

    // throttle percentage for engine idle
    AP_Int8 idle_percent;

    // Idle Controller RPM setpoint
    AP_Int16 idle_rpm;

    // Idle Controller RPM deadband
    AP_Int16 idle_db;

    // Idle Controller Slew Rate
    AP_Float idle_slew;
    
    // height when we enter ICE_START_HEIGHT_DELAY
    float initial_height;

    // height change required to start engine
    float height_required;

    // we are waiting for valid height data
    bool height_pending:1;

    // idle governor
    float idle_governor_integrator;

    enum class Options : uint16_t {
        DISABLE_IGNITION_RC_FAILSAFE=(1U<<0),
        DISABLE_REDLINE_GOVERNOR = (1U << 1),
    };
    AP_Int16 options;

    // start_chan debounce
    uint16_t start_chan_last_value = 1500;
    uint32_t start_chan_last_ms;

    // redline rpm
    AP_Int32 redline_rpm;
    struct {
        bool flag;
        float governor_integrator;
        float throttle_percentage;
    } redline;
};


namespace AP {
    AP_ICEngine *ice();
};
