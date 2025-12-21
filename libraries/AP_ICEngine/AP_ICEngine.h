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

#include "AP_ICEngine_config.h"

#if AP_ICENGINE_ENABLED

#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>
#include <AP_RPM/AP_RPM_config.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Relay/AP_Relay_config.h>
#include <RC_Channel/RC_Channel.h>

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine_TCA9554.h"
#endif

class AP_ICEngine {
public:
    // constructor
    AP_ICEngine();

    static const struct AP_Param::GroupInfo var_info[];

    // One time init call
    void init();

    // update engine state. Should be called at 10Hz or more
    void update(void);

    // check for throttle override
    bool throttle_override(float &percent, const float base_throttle);

    enum ICE_State {
        ICE_DISABLED = -1,
        ICE_OFF=0,
        ICE_START_HEIGHT_DELAY=1,
        ICE_START_DELAY=2,
        ICE_STARTING=3,
        ICE_RUNNING=4
    };

    // get current engine control state
    ICE_State get_state(void) const { return !enable?ICE_DISABLED:state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay, uint32_t flags);

    // do we have throttle while disarmed enabled?
    bool allow_throttle_while_disarmed(void) const;

    // Handle incoming aux function trigger
    void do_aux_function(const RC_Channel::AuxFuncTrigger &trigger);

#if AP_RELAY_ENABLED
    // Needed for param conversion from relay numbers to functions
    bool get_legacy_ignition_relay_index(int8_t &num);
#endif

    int8_t get_min_throttle_pct() const { return min_throttle_pct; }

    static AP_ICEngine *get_singleton() { return _singleton; }

private:
    static AP_ICEngine *_singleton;

    void set_ignition(bool on);
    void set_starter(bool on);

    enum ICE_State state;

    // Minimum throttle for idle (from idle_percent or idle governor)
    int8_t min_throttle_pct;

#if AP_RPM_ENABLED
    // update min throttle for idle governor
    void update_idle_governor(int8_t &min_throttle);

    // filter for RPM value
    LowPassFilterConstDtFloat _rpm_filter;
    float filtered_rpm_value;
#endif

    // enable library
    AP_Int8 enable;

    // min pwm on start channel for engine stop
    AP_Int16 start_chan_min_pwm;

#if AP_RPM_ENABLED
    // which RPM instance to use
    AP_Int8 rpm_instance;
#endif
    
    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;

    // max crank retry
    AP_Int8 max_crank_retry;
    int8_t crank_retry_ct;
    
#if AP_RPM_ENABLED
    // RPM above which engine is considered to be running
    AP_Int32 rpm_threshold;
#endif

    // time when we started the starter
    uint32_t starter_start_time_ms;

    // time when we last ran the starter
    uint32_t starter_last_run_ms;

    // time when we last had an uncommanded engine stop
    uint32_t last_uncommanded_stop_ms;

    // throttle percentage for engine start
    AP_Int8 start_percent;

    // throttle percentage for engine idle
    AP_Int8 idle_percent;

#if AP_RPM_ENABLED
    // Idle Controller RPM setpoint
    AP_Int16 idle_rpm;

    // Idle Controller RPM deadband
    AP_Int16 idle_db;

    // Idle Controller Slew Rate
    AP_Float idle_slew;
#endif

    // height when we enter ICE_START_HEIGHT_DELAY
    float initial_height;

    // height change required to start engine
    float height_required;

    // we are waiting for valid height data
    bool height_pending;

    bool allow_single_start_while_disarmed;

    // idle governor
    float idle_governor_integrator;

    enum class Options : uint16_t {
        DISABLE_IGNITION_RC_FAILSAFE = (1U << 0),
        DISABLE_REDLINE_GOVERNOR     = (1U << 1),
        THROTTLE_WHILE_DISARMED      = (1U << 2),
        NO_RUNNING_WHILE_DISARMED    = (1U << 3),
        CRANK_DIR_REVERSE            = (1U << 4),
    };
    AP_Int16 options;

    bool option_set(Options option) const {
        return (options & uint16_t(option)) != 0;
    }

    // Last aux function value
    RC_Channel::AuxSwitchPos aux_pos = RC_Channel::AuxSwitchPos::MIDDLE;

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
    AP_ICEngine_TCA9554 tca9554_starter;
#endif

#if AP_RPM_ENABLED
    // redline rpm
    AP_Int32 redline_rpm;
    struct {
        bool flag;
        float governor_integrator;
        float throttle_percentage;
    } redline;
#endif

    // Param conversion function and flag
    void param_conversion();
    AP_Int8 param_format_version;
};


namespace AP {
    AP_ICEngine *ice();
};

#endif  // AP_ICENGINE_ENABLED
