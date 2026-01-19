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

#include "AP_ICEngine.h"

#if AP_ICENGINE_ENABLED

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Relay/AP_Relay.h>
#include "AP_ICEngine.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ICEngine::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable ICEngine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ICEngine, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: START_CHAN
    // @DisplayName: Input channel for engine start
    // @Description: This is an RC input channel for requesting engine start. Engine will try to start when channel is at or above 1700. Engine will stop when channel is at or below 1300. Between 1301 and 1699 the engine will not change state unless a MAVLink command or mission item commands a state change, or the vehicle is disarmed. See ICE_STARTCHN_MIN parameter to change engine stop PWM value and/or to enable debouncing of the START_CH to avoid accidental engine kills due to noise on channel.
    // @Legacy: 4.5 param
    // @User: Standard
    // @Values: 0:None,1:Chan1,2:Chan2,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16

    // 1 was START_CHAN

    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @User: Standard
    // @Units: s
    // @Range: 0.1 5
    AP_GROUPINFO("STARTER_TIME", 2, AP_ICEngine, starter_time, 3),

    // @Param: START_DELAY
    // @DisplayName: Time to wait between starts
    // @Description: Delay between start attempts
    // @User: Standard
    // @Units: s
    // @Range: 1 10
    AP_GROUPINFO("START_DELAY", 3, AP_ICEngine, starter_delay, 2),

#if AP_RPM_ENABLED
    // @Param: RPM_THRESH
    // @DisplayName: RPM threshold
    // @Description: This is the measured RPM above which the engine is considered to be running
    // @User: Standard
    // @Range: 100 100000
    AP_GROUPINFO("RPM_THRESH", 4, AP_ICEngine, rpm_threshold, 100),
#endif

    // @Param: PWM_IGN_ON
    // @DisplayName: PWM value for ignition on
    // @Description: This is the value sent to the ignition channel when on
    // @Legacy: 4.5 param
    // @User: Standard
    // @Range: 1000 2000

    // 5 was PWM_IGN_ON

    // @Param: PWM_IGN_OFF
    // @DisplayName: PWM value for ignition off
    // @Description: This is the value sent to the ignition channel when off
    // @Legacy: 4.5 param
    // @User: Standard
    // @Range: 1000 2000

    // 6 was PWM_IGN_OFF

    // @Param: PWM_STRT_ON
    // @DisplayName: PWM value for starter on
    // @Description: This is the value sent to the starter channel when on
    // @Legacy: 4.5 param
    // @User: Standard
    // @Range: 1000 2000

    // 7 was PWM_STRT_ON

    // @Param: PWM_STRT_OFF
    // @DisplayName: PWM value for starter off
    // @Description: This is the value sent to the starter channel when off
    // @Legacy: 4.5 param
    // @User: Standard
    // @Range: 1000 2000

    // 8 was PWM_STRT_OFF

#if AP_RPM_ENABLED
    // @Param: RPM_CHAN
    // @DisplayName: RPM instance channel to use
    // @Description: This is which of the RPM instances to use for detecting the RPM of the engine
    // @User: Standard
    // @Values: 0:None,1:RPM1,2:RPM2
    AP_GROUPINFO("RPM_CHAN",  9, AP_ICEngine, rpm_instance, 0),
#endif

    // @Param: START_PCT
    // @DisplayName: Throttle percentage for engine start
    // @Description: This is the percentage throttle output for engine start
    // @User: Standard
    // @Range: 0 100
    AP_GROUPINFO("START_PCT", 10, AP_ICEngine, start_percent, 5),

    // @Param: IDLE_PCT
    // @DisplayName: Throttle percentage for engine idle
    // @Description: This is the minimum percentage throttle output while running, this includes being disarmed, but not safe
    // @User: Standard
    // @Range: 0 100
    AP_GROUPINFO("IDLE_PCT", 11, AP_ICEngine, idle_percent, 0),

#if AP_RPM_ENABLED
    // @Param: IDLE_RPM
    // @DisplayName: RPM Setpoint for Idle Governor
    // @Description: This configures the RPM that will be commanded by the idle governor. Set to -1 to disable
    // @User: Advanced
    AP_GROUPINFO("IDLE_RPM", 12, AP_ICEngine, idle_rpm, -1),

    // @Param: IDLE_DB
    // @DisplayName: Deadband for Idle Governor
    // @Description: This configures the deadband that is tolerated before adjusting the idle setpoint
    AP_GROUPINFO("IDLE_DB", 13, AP_ICEngine, idle_db, 50),

    // @Param: IDLE_SLEW
    // @DisplayName: Slew Rate for idle control
    // @Description: This configures the slewrate used to adjust the idle setpoint in percentage points per second
    AP_GROUPINFO("IDLE_SLEW", 14, AP_ICEngine, idle_slew, 1),
#endif

    // @Param: OPTIONS
    // @DisplayName: ICE options
    // @Description: Options for ICE control. The Disable ignition in RC failsafe option will cause the ignition to be set off on any R/C failsafe. If Throttle while disarmed is set then throttle control will be allowed while disarmed for planes when in MANUAL mode. If disable while disarmed is set the engine will not start while the vehicle is disarmed unless overriden by the MAVLink DO_ENGINE_CONTROL command.
    // @Bitmask: 0:Disable ignition in RC failsafe,1:Disable redline governor,2:Throttle control in MANUAL while disarmed with safety off,3:Disable while disarmed,4:Crank direction Reverse
    AP_GROUPINFO("OPTIONS", 15, AP_ICEngine, options, 0),

    // @Param: STARTCHN_MIN
    // @DisplayName: Input channel for engine start minimum PWM
    // @Description: This is a minimum PWM value for engine start channel for an engine stop to be commanded. Setting this value will avoid RC input glitches with low PWM values from causing an unwanted engine stop. A value of zero means any PWM above 800 and below 1300 triggers an engine stop. To stop the engine start channel must above the larger of this value and 800 and below 1300.
    // @User: Standard
    // @Range: 0 1300
    AP_GROUPINFO("STARTCHN_MIN", 16, AP_ICEngine, start_chan_min_pwm, 0),

#if AP_RPM_ENABLED
    // @Param: REDLINE_RPM
    // @DisplayName: RPM of the redline limit for the engine
    // @Description: Maximum RPM for the engine provided by the manufacturer. A value of 0 disables this feature. See ICE_OPTIONS to enable or disable the governor.
    // @User: Advanced
    // @Range: 0 2000000
    // @Units: RPM
    AP_GROUPINFO("REDLINE_RPM", 17, AP_ICEngine, redline_rpm, 0),
#endif

    // 18 was IGNITION_RLY

    // Hidden param used as a flag for param conversion
    // This allows one time conversion while allowing user to flash between versions with and without converted params
    AP_GROUPINFO_FLAGS("FMT_VER", 19, AP_ICEngine, param_format_version, 0, AP_PARAM_FLAG_HIDDEN),

    // @Param: STRT_MX_RTRY
    // @DisplayName: Maximum number of retries
    // @Description: If set 0 then there is no limit to retrials. If set to a value greater than 0 then the engine will retry starting the engine this many times before giving up.
    // @User: Standard
    // @Range: 0 127
    AP_GROUPINFO("STRT_MX_RTRY", 20, AP_ICEngine, max_crank_retry, 0),

    AP_GROUPEND
};

// constructor
AP_ICEngine::AP_ICEngine()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ICEngine must be singleton");
    }
    _singleton = this;

#if AP_RPM_ENABLED
    // ICEngine runs at 10Hz
    _rpm_filter.set_cutoff_frequency(10, 0.5f);
#endif
}

// One time init call
void AP_ICEngine::init()
{
    // Configure starter and ignition outputs as range type
    SRV_Channels::set_range(SRV_Channel::k_starter, 1);
    SRV_Channels::set_range(SRV_Channel::k_ignition, 1);

    // Set default PWM endpoints to 1000 to 2000
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_starter, 1000, 2000);
    SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_ignition, 1000, 2000);

    // Convert params
    param_conversion();
}

// PARAMETER_CONVERSION - Added: Aug 2024
void AP_ICEngine::param_conversion()
{
    if (!enable || (param_format_version == 1)) {
        // not enabled or conversion has already been done
        return;
    }

    // Set format version so the conversion is not done again
    param_format_version.set_and_save(1);

    AP_Param::ConversionInfo info;
    if (!AP_Param::find_key_by_pointer(this, info.old_key)) {
        return;
    }

    // Conversion table giving the old on and off pwm parameter indexes and the function for both starter and ignition
    const struct convert_table {
        uint32_t element[2];
        SRV_Channel::Function fuction;
    } conversion_table[] = {
        { {450, 514}, SRV_Channel::k_starter },  // PWM_STRT_ON, PWM_STRT_OFF
        { {322, 386}, SRV_Channel::k_ignition }, // PWM_IGN_ON, PWM_IGN_OFF
    };

    // All PWM values were int16
    info.type = AP_PARAM_INT16;

    for (const auto & elem : conversion_table) {
        // Use the original default values if params are not saved
        uint16_t pwm_on = 2000;
        uint16_t pwm_off = 1000;

        // Get param values if configured
        AP_Int16 param_value;
        info.old_group_element = elem.element[0];
        if (AP_Param::find_old_parameter(&info, &param_value)) {
            pwm_on = param_value;
        }
        info.old_group_element = elem.element[1];
        if (AP_Param::find_old_parameter(&info, &param_value)) {
            pwm_off = param_value;
        }

        // Save as servo endpoints, note that save_output_min_max will set reversed if needed
        SRV_Channels::save_output_min_max(elem.fuction, pwm_off, pwm_on);
    }

    // Convert to new RC option
    AP_Int8 start_chan;
    info.type = AP_PARAM_INT8;
    info.old_group_element = 66;
    if (AP_Param::find_old_parameter(&info, &start_chan) && (start_chan > 0)) {
        RC_Channel *chan = rc().channel(start_chan-1);
        if (chan != nullptr) {
            chan->option.set_and_save((int16_t)RC_Channel::AUX_FUNC::ICE_START_STOP);
        }
    }
}

// Handle incoming aux function
void AP_ICEngine::do_aux_function(const RC_Channel::AuxFuncTrigger &trigger)
{
    // If triggered from RC apply start chan min
    if (trigger.source == RC_Channel::AuxFuncTrigger::Source::RC) {
        RC_Channel *chan = rc().channel(trigger.source_index);
        if ((chan != nullptr) && (chan->get_radio_in() < start_chan_min_pwm)) {
            return;
        }
    }

    aux_pos = trigger.pos;
}

/*
  update engine state
 */
void AP_ICEngine::update(void)
{
    if (!enable) {
        return;
    }

    bool should_run = false;
    uint32_t now = AP_HAL::millis();


    if ((state == ICE_START_HEIGHT_DELAY) && (aux_pos == RC_Channel::AuxSwitchPos::HIGH)) {
        // user is overriding the height start delay and asking for
        // immediate start. Put into ICE_OFF so that the logic below
        // can start the engine now
        state = ICE_OFF;
    }

    if ((state == ICE_OFF) && (aux_pos == RC_Channel::AuxSwitchPos::HIGH)) {
        should_run = true;
    } else if (aux_pos == RC_Channel::AuxSwitchPos::LOW) {
        should_run = false;

        // clear the single start flag now that we will be stopping the engine
        if (state != ICE_OFF) {
            allow_single_start_while_disarmed = false;
        }
    } else if (state != ICE_OFF) {
        should_run = true;
    }

    if (option_set(Options::DISABLE_IGNITION_RC_FAILSAFE) && AP_Notify::flags.failsafe_radio) {
        // user has requested ignition kill on RC failsafe
        should_run = false;
    }

    if (option_set(Options::NO_RUNNING_WHILE_DISARMED)) {
        if (hal.util->get_soft_armed()) {
            // clear the disarmed start flag, as we are now armed, if we disarm again we expect the engine to stop
            allow_single_start_while_disarmed = false;
        } else {
            // check if we are blocking disarmed starts
            if (!allow_single_start_while_disarmed) {
                should_run = false;
            }
        }
    }

#if HAL_PARACHUTE_ENABLED
    // Stop on parachute deployment
    AP_Parachute *parachute = AP::parachute();
    if ((parachute != nullptr) && parachute->release_initiated()) {
        should_run = false;
    }
#endif

    // Stop on emergency stop
    if (SRV_Channels::get_emergency_stop()) {
        // Throttle is already forced to 0 in this case, ignition should also be stopped.
        // Starter should not run.
        should_run = false;
    }

    // switch on current state to work out new state
    switch (state) {
    case ICE_DISABLED:
        return;
    case ICE_OFF:
        if (should_run) {
            state = ICE_START_DELAY;
        }
        crank_retry_ct = 0;
        // clear the last uncommanded stop time, we only care about tracking
        // the last one since the engine was started
        last_uncommanded_stop_ms = 0;
        break;

    case ICE_START_HEIGHT_DELAY: {
        Vector3f pos;
        if (!should_run) {
            state = ICE_OFF;
        } else if (AP::ahrs().get_relative_position_NED_origin_float(pos)) {
            if (height_pending) {
                height_pending = false;
                initial_height = -pos.z;
            } else if ((-pos.z) >= initial_height + height_required) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting height reached %.1f",
                                                 (double)(-pos.z - initial_height));
                state = ICE_STARTING;
            }
        }
        break;
    }

    case ICE_START_DELAY:
        if (!should_run) {
            state = ICE_OFF;
        } else if (now - starter_last_run_ms >= starter_delay*1000) {
            // check if we should retry starting the engine
            if (max_crank_retry <= 0 || crank_retry_ct < max_crank_retry) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting engine");
                state = ICE_STARTING;
                crank_retry_ct++;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Engine max crank attempts reached");
                // Mark the last run now so we don't send this message every loop
                starter_last_run_ms = now;
            }
        }
        break;

    case ICE_STARTING:
        if (!should_run) {
            state = ICE_OFF;
        } else if (now - starter_start_time_ms >= starter_time*1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine running");
            state = ICE_RUNNING;
        }
        break;

    case ICE_RUNNING:
        if (!should_run) {
            state = ICE_OFF;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Stopped engine");
        }
#if AP_RPM_ENABLED
        else if (rpm_instance > 0) {
            // check RPM to see if still running
            float rpm_value;
            if (!AP::rpm()->get_rpm(rpm_instance-1, rpm_value) ||
                rpm_value < rpm_threshold) {
                // engine has stopped when it should be running
                state = ICE_START_DELAY;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Uncommanded engine stop");
                if (last_uncommanded_stop_ms != 0 &&
                        now - last_uncommanded_stop_ms > 3*(starter_time + starter_delay)*1000) {
                    // if it has been a long enough time since the last uncommanded stop
                    // (3 times the time between start attempts) then reset the retry count
                    crank_retry_ct = 0;
                }
                last_uncommanded_stop_ms = now;
            }
        }
#endif
        break;
    }

    if (!hal.util->get_soft_armed()) {
        if (state == ICE_START_HEIGHT_DELAY) {
            // when disarmed we can be waiting for takeoff
            Vector3f pos;
            if (AP::ahrs().get_relative_position_NED_origin_float(pos)) {
                // reset initial height while disarmed
                initial_height = -pos.z;
            }
        } else if (idle_percent <= 0 && !allow_throttle_while_disarmed()) {
            // force ignition off when disarmed
            state = ICE_OFF;
        }
    }

#if AP_RPM_ENABLED
    // check against redline RPM
    float rpm_value;
    if (rpm_instance > 0 && redline_rpm > 0 && AP::rpm()->get_rpm(rpm_instance-1, rpm_value)) {
        // update the filtered RPM value
        filtered_rpm_value =  _rpm_filter.apply(rpm_value);
        if (!redline.flag && filtered_rpm_value > redline_rpm) {
            // redline governor is off. rpm is too high. enable the governor
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine: Above redline RPM");
            redline.flag = true;
        } else if (redline.flag && filtered_rpm_value < redline_rpm * 0.9f) {
            // redline governor is on. rpm is safely below. disable the governor
            redline.flag = false;
            // reset redline governor
            redline.throttle_percentage = 0.0f;
            redline.governor_integrator = 0.0f;
        }
    } else {
        redline.flag = false;
    }
#endif // AP_RPM_ENABLED

    /* now set output channels */
    switch (state) {
    case ICE_DISABLED:
        return;
    case ICE_OFF:
        set_ignition(false);
        set_starter(false);
        starter_start_time_ms = 0;
        break;

    case ICE_START_HEIGHT_DELAY:
    case ICE_START_DELAY:
        set_ignition(true);
        set_starter(false);
        break;

    case ICE_STARTING:
        set_ignition(true);
        set_starter(true);

        if (starter_start_time_ms == 0) {
            starter_start_time_ms = now;
        }
        starter_last_run_ms = now;
        break;

    case ICE_RUNNING:
        set_ignition(true);
        set_starter(false);
        starter_start_time_ms = 0;
        break;
    }
}


/*
  check for throttle override. This allows the ICE controller to force
  the correct starting throttle when starting the engine and maintain idle when disarmed

  base_throttle is the throttle before the disarmed override
  check. This allows for throttle control while disarmed
 */
bool AP_ICEngine::throttle_override(float &percentage, const float base_throttle)
{
    if (!enable) {
        return false;
    }

    min_throttle_pct = idle_percent.get();
    #if AP_RPM_ENABLED
        update_idle_governor(min_throttle_pct);
    #endif // AP_RPM_ENABLED

    if (state == ICE_RUNNING &&
        min_throttle_pct > 0 &&
        min_throttle_pct < 100 &&
        min_throttle_pct > percentage)
    {
        percentage = min_throttle_pct;
        if (allow_throttle_while_disarmed() && !hal.util->get_soft_armed()) {
            percentage = MAX(percentage, base_throttle);
        }
        return true;
    }

    if (state == ICE_STARTING || state == ICE_START_DELAY) {
        percentage = start_percent.get();
        return true;
    } else if (state != ICE_RUNNING && hal.util->get_soft_armed()) {
        percentage = 0;
        return true;
    }

#if AP_RPM_ENABLED
    if (redline.flag && !option_set(Options::DISABLE_REDLINE_GOVERNOR)) {
        // limit the throttle from increasing above what the current output is
        if (redline.throttle_percentage < 1.0f) {
            redline.throttle_percentage = percentage;
        }
        if (percentage < redline.throttle_percentage - redline.governor_integrator) {
            // the throttle before the override is much lower than what the integrator is at
            // reset the integrator
            redline.governor_integrator = 0;
            redline.throttle_percentage = percentage;
        } else if (percentage < redline.throttle_percentage) {
            // the throttle is below the integrator set point
            // remove the difference from the integrator
            redline.governor_integrator -= redline.throttle_percentage - percentage;
            redline.throttle_percentage = percentage;
        } else if (filtered_rpm_value > redline_rpm) {
            // reduce the throttle if still over the redline RPM
            const float redline_setpoint_step = idle_slew * AP::scheduler().get_loop_period_s();
            redline.governor_integrator += redline_setpoint_step;
        }
        percentage = redline.throttle_percentage - redline.governor_integrator;
        return true;
    }
#endif // AP_RPM_ENABLED

    // if THROTTLE_WHILE_DISARMED is set then we use the base_throttle, allowing the pilot to control throttle while disarmed
    if (allow_throttle_while_disarmed() && !hal.util->get_soft_armed() &&
        base_throttle > percentage) {
        percentage = base_throttle;
        return true;
    }

    return false;
}

/*
  handle DO_ENGINE_CONTROL messages via MAVLink or mission
*/
bool AP_ICEngine::engine_control(float start_control, float cold_start, float height_delay, uint32_t flags)
{
    if (!enable) {
        return false;
    }

    // always update the start while disarmed flag
    allow_single_start_while_disarmed = (flags & ENGINE_CONTROL_OPTIONS_ALLOW_START_WHILE_DISARMED) != 0;

    if (start_control <= 0) {
        state = ICE_OFF;
        return true;
    }
    if (state == ICE_RUNNING || state == ICE_START_DELAY || state == ICE_STARTING) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine: already running");
        return false;
    }

    // get starter control channel
    if (aux_pos == RC_Channel::AuxSwitchPos::LOW) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine: start control disabled by aux function");
        return false;
    }

    if (height_delay > 0) {
        height_pending = true;
        initial_height = 0;
        height_required = height_delay;
        state = ICE_START_HEIGHT_DELAY;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Takeoff height set to %.1fm", (double)height_delay);
        return true;
    }
    state = ICE_STARTING;
    return true;
}

#if AP_RPM_ENABLED
/*
  Update low throttle limit to ensure steady idle for IC Engines
  return a new min_throttle value
*/
void AP_ICEngine::update_idle_governor(int8_t &min_throttle)
{
    if (!enable) {
        return;
    }
    const int8_t min_throttle_base = min_throttle;

    // Initialize idle point to start_percent on the first run
    static bool idle_point_initialized = false;
    if (!idle_point_initialized) {
        idle_governor_integrator = start_percent.get();
        idle_point_initialized = true;
    }
    AP_RPM *ap_rpm = AP::rpm();
    if (!ap_rpm || rpm_instance == 0 || !ap_rpm->healthy(rpm_instance-1)) {
        return;
    }

    // Check to make sure we have an enabled IC Engine, EFI Instance and that the idle governor is enabled
    if (get_state() != AP_ICEngine::ICE_RUNNING || idle_rpm < 0) {
        idle_point_initialized = false;
        return;
    }

    // get current RPM feedback
    float rpmv;

    // Double Check to make sure engine is really running
    if (!ap_rpm->get_rpm(rpm_instance-1, rpmv) || rpmv < 1) {
        // Reset idle point to the default value when the engine is stopped
        idle_point_initialized = false;
        return;
    }

    // Override
    min_throttle = roundf(idle_governor_integrator);

    // Calculate Error in system
    int32_t error = idle_rpm - rpmv;

    bool underspeed = error > 0;

    // Don't adjust idle point when we're within the deadband
    if (abs(error) < idle_db) {
        return;
    }

    // Don't adjust idle point if the commanded throttle is above the
    // current idle throttle setpoint and the RPM is above the idle
    // RPM setpoint (Normal flight)
    if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > min_throttle && !underspeed) {
        return;
    }

    // Calculate the change per loop to achieve the desired slew rate of 1 percent per second
    static const float idle_setpoint_step = idle_slew * AP::scheduler().get_loop_period_s();

    // Update Integrator
    if (underspeed) {
        idle_governor_integrator += idle_setpoint_step;
    } else {
        idle_governor_integrator -= idle_setpoint_step;
    }

    idle_governor_integrator = constrain_float(idle_governor_integrator, min_throttle_base, 40.0f);

    min_throttle = roundf(idle_governor_integrator);
}
#endif // AP_RPM_ENABLED

/*
  set ignition state
 */
void AP_ICEngine::set_ignition(bool on)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_ignition, on ? 1.0 : 0.0);

#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->set(AP_Relay_Params::FUNCTION::IGNITION, on);
    }
#endif // AP_RELAY_ENABLED

}

/*
  set starter state
 */
void AP_ICEngine::set_starter(bool on)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_starter, on ? 1.0 : 0.0);

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
    tca9554_starter.set_starter(on, option_set(Options::CRANK_DIR_REVERSE));
#endif

#if AP_RELAY_ENABLED
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->set(AP_Relay_Params::FUNCTION::ICE_STARTER, on);
    }
#endif // AP_RELAY_ENABLED
}


bool AP_ICEngine::allow_throttle_while_disarmed() const
{
    return option_set(Options::THROTTLE_WHILE_DISARMED) &&
        hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

#if AP_RELAY_ENABLED
bool AP_ICEngine::get_legacy_ignition_relay_index(int8_t &num) 
{
    // PARAMETER_CONVERSION - Added: Dec-2023
    if (!enable || !AP_Param::get_param_by_index(this, 18, AP_PARAM_INT8, &num)) {
        return false;
    }
    // convert to zero indexed
    num -= 1;
    return true;
}
#endif

// singleton instance. Should only ever be set in the constructor.
AP_ICEngine *AP_ICEngine::_singleton;
namespace AP {
AP_ICEngine *ice() {
        return AP_ICEngine::get_singleton();
    }
}

#endif  // AP_ICENGINE_ENABLED
