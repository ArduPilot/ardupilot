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


#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include <RC_Channel/RC_Channel.h>
#include <Filter/LowPassFilter.h>
#include "AP_ICEngine.h"
#include <AP_RPM/AP_RPM.h>

extern const AP_HAL::HAL& hal;

#define AP_ICENGINE_START_CHAN_DEBOUNCE_MS          300

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
    // @User: Standard
    // @Values: 0:None,1:Chan1,2:Chan2,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    AP_GROUPINFO("START_CHAN", 1, AP_ICEngine, start_chan, 0),

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

    // @Param: RPM_THRESH
    // @DisplayName: RPM threshold
    // @Description: This is the measured RPM above which the engine is considered to be running
    // @User: Standard
    // @Range: 100 100000
    AP_GROUPINFO("RPM_THRESH", 4, AP_ICEngine, rpm_threshold, 100),

    // @Param: PWM_IGN_ON
    // @DisplayName: PWM value for ignition on
    // @Description: This is the value sent to the ignition channel when on
    // @User: Standard
    // @Range: 1000 2000
    AP_GROUPINFO("PWM_IGN_ON", 5, AP_ICEngine, pwm_ignition_on, 2000),

    // @Param: PWM_IGN_OFF
    // @DisplayName: PWM value for ignition off
    // @Description: This is the value sent to the ignition channel when off
    // @User: Standard
    // @Range: 1000 2000
    AP_GROUPINFO("PWM_IGN_OFF", 6, AP_ICEngine, pwm_ignition_off, 1000),

    // @Param: PWM_STRT_ON
    // @DisplayName: PWM value for starter on
    // @Description: This is the value sent to the starter channel when on
    // @User: Standard
    // @Range: 1000 2000
    AP_GROUPINFO("PWM_STRT_ON", 7, AP_ICEngine, pwm_starter_on, 2000),

    // @Param: PWM_STRT_OFF
    // @DisplayName: PWM value for starter off
    // @Description: This is the value sent to the starter channel when off
    // @User: Standard
    // @Range: 1000 2000
    AP_GROUPINFO("PWM_STRT_OFF", 8, AP_ICEngine, pwm_starter_off, 1000),

    // @Param: RPM_CHAN
    // @DisplayName: RPM instance channel to use
    // @Description: This is which of the RPM instances to use for detecting the RPM of the engine
    // @User: Standard
    // @Values: 0:None,1:RPM1,2:RPM2
    AP_GROUPINFO("RPM_CHAN",  9, AP_ICEngine, rpm_instance, 0),

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

    // @Param: OPTIONS
    // @DisplayName: ICE options
    // @Description: Options for ICE control
    // @Bitmask: 0:DisableIgnitionRCFailsafe,1:DisableRedlineRPMGovernor
    AP_GROUPINFO("OPTIONS", 15, AP_ICEngine, options, 0),

    // @Param: STARTCHN_MIN
    // @DisplayName: Input channel for engine start minimum PWM
    // @Description: This is a minimum PWM value for engine start channel for an engine stop to be commanded. Setting this value will avoid RC input glitches with low PWM values from causing an unwanted engine stop. A value of zero means any PWM below 1300 triggers an engine stop.
    // @User: Standard
    // @Range: 0 1300
    AP_GROUPINFO("STARTCHN_MIN", 16, AP_ICEngine, start_chan_min_pwm, 0),

    // @Param: REDLINE_RPM
    // @DisplayName: RPM of the redline limit for the engine
    // @Description: Maximum RPM for the engine provided by the manufacturer. A value of 0 disables this feature. See ICE_OPTIONS to enable or disable the governor.
    // @User: Advanced
    // @Range: 0 2000000
    // @Units: RPM
    AP_GROUPINFO("REDLINE_RPM", 17, AP_ICEngine, redline_rpm, 0),

    AP_GROUPEND
};


// constructor
AP_ICEngine::AP_ICEngine(const AP_RPM &_rpm) :
    rpm(_rpm)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ICEngine must be singleton");
    }
    _singleton = this;

    _rpm_filter.set_cutoff_frequency(1 / AP::scheduler().get_loop_period_s(), 0.5f);
}

/*
  update engine state
 */
void AP_ICEngine::update(void)
{
    if (!enable) {
        return;
    }

    uint16_t cvalue = 1500;
    RC_Channel *c = rc().channel(start_chan-1);
    if (c != nullptr && rc().has_valid_input()) {
        // get starter control channel
        cvalue = c->get_radio_in();

        if (cvalue < start_chan_min_pwm) {
            cvalue = start_chan_last_value;
        }

        // snap the input to either 1000, 1500, or 2000
        // this is useful to compare a debounce changed value
        // while ignoring tiny noise
        if (cvalue >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
            cvalue = 2000;
        } else if ((cvalue > 800) && (cvalue <= RC_Channel::AUX_PWM_TRIGGER_LOW)) {
            cvalue = 1300;
        } else {
            cvalue = 1500;
        }
    }

    bool should_run = false;
    uint32_t now = AP_HAL::millis();


    // debounce timer to protect from spurious changes on start_chan rc input
    // If the cached value is the same, reset timer
    if (start_chan_last_value == cvalue) {
        start_chan_last_ms = now;
    } else if (now - start_chan_last_ms >= AP_ICENGINE_START_CHAN_DEBOUNCE_MS) {
        // if it has changed, and stayed changed for the duration, then use that new value
        start_chan_last_value = cvalue;
    }


    if (state == ICE_OFF && start_chan_last_value >= RC_Channel::AUX_PWM_TRIGGER_HIGH) {
        should_run = true;
    } else if (start_chan_last_value <= RC_Channel::AUX_PWM_TRIGGER_LOW) {
        should_run = false;
    } else if (state != ICE_OFF) {
        should_run = true;
    }

    if ((options & uint16_t(Options::DISABLE_IGNITION_RC_FAILSAFE)) && AP_Notify::flags.failsafe_radio) {
        // user has requested ignition kill on RC failsafe
        should_run = false;
    }

    float rpm_value;

    // switch on current state to work out new state
    switch (state) {
    case ICE_OFF:
        if (should_run) {
            state = ICE_START_DELAY;
        }
        break;

    case ICE_START_HEIGHT_DELAY: {
        Vector3f pos;
        if (!should_run) {
            state = ICE_OFF;
        } else if (AP::ahrs().get_relative_position_NED_origin(pos)) {
            if (height_pending) {
                height_pending = false;
                initial_height = -pos.z;
            } else if ((-pos.z) >= initial_height + height_required) {
                gcs().send_text(MAV_SEVERITY_INFO, "Starting height reached %.1f",
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
            gcs().send_text(MAV_SEVERITY_INFO, "Starting engine");
            state = ICE_STARTING;
        }
        break;

    case ICE_STARTING:
        if (!should_run) {
            state = ICE_OFF;
        } else if (now - starter_start_time_ms >= starter_time*1000) {
            state = ICE_RUNNING;
        }
        break;

    case ICE_RUNNING:
        if (!should_run) {
            state = ICE_OFF;
            gcs().send_text(MAV_SEVERITY_INFO, "Stopped engine");
        } else if (rpm_instance > 0) {
            // check RPM to see if still running
            if (!rpm.get_rpm(rpm_instance-1, rpm_value) ||
                rpm_value < rpm_threshold) {
                // engine has stopped when it should be running
                state = ICE_START_DELAY;
            }
        }
        break;
    }

    if (!hal.util->get_soft_armed()) {
        if (state == ICE_START_HEIGHT_DELAY) {
            // when disarmed we can be waiting for takeoff
            Vector3f pos;
            if (AP::ahrs().get_relative_position_NED_origin(pos)) {
                // reset initial height while disarmed
                initial_height = -pos.z;
            }
        } else if (idle_percent <= 0) { // check if we should idle
            // force ignition off when disarmed
            state = ICE_OFF;
        }
    }

    // check against redline RPM
    if (rpm_instance > 0 && redline_rpm > 0 && rpm.get_rpm(rpm_instance-1, rpm_value)) {
        // update the filtered RPM value
        filtered_rpm_value =  _rpm_filter.apply(rpm_value);
        if (!redline.flag && filtered_rpm_value > redline_rpm) {
            // redline governor is off. rpm is too high. enable the governor
            gcs().send_text(MAV_SEVERITY_INFO, "Engine: Above redline RPM");
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

    /* now set output channels */
    switch (state) {
    case ICE_OFF:
        SRV_Channels::set_output_pwm(SRV_Channel::k_ignition, pwm_ignition_off);
        SRV_Channels::set_output_pwm(SRV_Channel::k_starter,  pwm_starter_off);
        starter_start_time_ms = 0;
        break;

    case ICE_START_HEIGHT_DELAY:
    case ICE_START_DELAY:
        SRV_Channels::set_output_pwm(SRV_Channel::k_ignition, pwm_ignition_on);
        SRV_Channels::set_output_pwm(SRV_Channel::k_starter,  pwm_starter_off);
        break;

    case ICE_STARTING:
        SRV_Channels::set_output_pwm(SRV_Channel::k_ignition, pwm_ignition_on);
        SRV_Channels::set_output_pwm(SRV_Channel::k_starter,  pwm_starter_on);
        if (starter_start_time_ms == 0) {
            starter_start_time_ms = now;
        }
        starter_last_run_ms = now;
        break;

    case ICE_RUNNING:
        SRV_Channels::set_output_pwm(SRV_Channel::k_ignition, pwm_ignition_on);
        SRV_Channels::set_output_pwm(SRV_Channel::k_starter,  pwm_starter_off);
        starter_start_time_ms = 0;
        break;
    }
}


/*
  check for throttle override. This allows the ICE controller to force
  the correct starting throttle when starting the engine and maintain idle when disarmed
 */
bool AP_ICEngine::throttle_override(float &percentage)
{
    if (!enable) {
        return false;
    }

    if (state == ICE_RUNNING &&
        idle_percent > 0 &&
        idle_percent < 100 &&
        idle_percent > percentage)
    {
        percentage = idle_percent;
        return true;
    }

    if (state == ICE_STARTING || state == ICE_START_DELAY) {
        percentage = start_percent.get();
        return true;
    }

    if (redline.flag && !(options & uint16_t(Options::DISABLE_REDLINE_GOVERNOR))) {
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
    return false;
}


/*
  handle DO_ENGINE_CONTROL messages via MAVLink or mission
*/
bool AP_ICEngine::engine_control(float start_control, float cold_start, float height_delay)
{
    if (start_control <= 0) {
        state = ICE_OFF;
        return true;
    }
    RC_Channel *c = rc().channel(start_chan-1);
    if (c != nullptr && rc().has_valid_input()) {
        // get starter control channel
        uint16_t cvalue = c->get_radio_in();
        if (cvalue >= start_chan_min_pwm && cvalue <= RC_Channel::AUX_PWM_TRIGGER_LOW) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine: start control disabled");
            return false;
        }
    }
    if (height_delay > 0) {
        height_pending = true;
        initial_height = 0;
        height_required = height_delay;
        state = ICE_START_HEIGHT_DELAY;
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff height set to %.1fm", (double)height_delay);
        return true;
    }
    state = ICE_STARTING;
    return true;
}

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

    // Initialize idle point to min_throttle on the first run
    static bool idle_point_initialized = false;
    if (!idle_point_initialized) {
        idle_governor_integrator = min_throttle;
        idle_point_initialized = true;
    }
    AP_RPM *ap_rpm = AP::rpm();
    if (!ap_rpm || rpm_instance == 0 || !ap_rpm->healthy(rpm_instance-1)) {
        return;
    }

    // Check to make sure we have an enabled IC Engine, EFI Instance and that the idle governor is enabled
    if (get_state() != AP_ICEngine::ICE_RUNNING || idle_rpm < 0) {
        return;
    }

    // get current RPM feedback
    float rpmv;

    // Double Check to make sure engine is really running
    if (!ap_rpm->get_rpm(rpm_instance-1, rpmv) || rpmv < 1) {
        // Reset idle point to the default value when the engine is stopped
        idle_governor_integrator = min_throttle;
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


// singleton instance. Should only ever be set in the constructor.
AP_ICEngine *AP_ICEngine::_singleton;
namespace AP {
AP_ICEngine *ice() {
        return AP_ICEngine::get_singleton();
    }
}
