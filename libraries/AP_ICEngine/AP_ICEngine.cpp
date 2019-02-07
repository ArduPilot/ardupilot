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
    // @Description: This is an RC input channel for requesting engine start. Engine will try to start when channel is at or above 1700. Engine will stop when channel is at or below 1300. Between 1301 and 1699 the engine will not change state unless a MAVLink command or mission item commands a state change, or the vehicle is disamed.
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

    // @Param: TEMP_PIN
    // @DisplayName: Temperature analog feedback pin
    // @Description: Temperature analog feedback pin. This is used to sample the engine temperature.
    // @User: Advanced
    AP_GROUPINFO("TEMP_PIN", 11, AP_ICEngine, temperature.pin, -1),

    // @Param: TEMP_SCALER
    // @DisplayName: Temperature scaler
    // @Description: Temperature scaler to apply to analog input to convert voltage to degrees C
    // @User: Advanced
    AP_GROUPINFO("TEMP_SCALER", 12, AP_ICEngine, temperature.scaler, 1),

    // @Param: TEMP_MAX
    // @DisplayName: Temperature overheat
    // @Description: Temperature limit that is considered overheating. When above this temperature the starting and throttle will be limited/inhibited. Use 0 to disable.
    // @User: Advanced
    // @Units: degC
    AP_GROUPINFO("TEMP_MAX", 13, AP_ICEngine, temperature.max, 0),

    // @Param: TEMP_MIN
    // @DisplayName: Temperature minimum
    // @Description: Temperature minimum that is considered too cold to run the engine. While under this temp the throttle will be inhibited. Use 0 to disable.
    // @User: Advanced
    // @Units: degC
    AP_GROUPINFO("TEMP_MIN", 14, AP_ICEngine, temperature.min, 0),

    // @Param: TEMP_RMETRIC
    // @DisplayName: Temperature is Ratiometric
    // @Description: This parameter sets whether an analog temperature is ratiometric. Most analog analog sensors are ratiometric, meaning that their output voltage is influenced by the supply voltage.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("TEMP_RMETRIC", 15, AP_ICEngine, temperature.ratiometric, 1),

    // @Param: TEMP_OFFSET
    // @DisplayName: Temperature voltage offset
    // @Description: Offset in volts for analog sensor.
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("TEMP_OFFSET", 16, AP_ICEngine, temperature.offset, 0),

    // @Param: TEMP_FUNC
    // @DisplayName: Temperature sensor function
    // @Description: Control over what function is used to calculate temperature. For a linear function, the temp is (voltage-offset)*scaling. For a inverted function the temp is (offset-voltage)*scaling. For a hyperbolic function the temp is scaling/(voltage-offset).
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("TEMP_FUNC", 17, AP_ICEngine, temperature.function, 0),

    AP_GROUPEND    
};


// constructor
AP_ICEngine::AP_ICEngine(const AP_RPM &_rpm) :
    rpm(_rpm)
{
    AP_Param::setup_object_defaults(this, var_info);
    temperature.source = hal.analogin->channel(temperature.pin);
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
    if (c != nullptr) {
        // get starter control channel
        cvalue = c->get_radio_in();
    }

    update_temperature();

    bool should_run = false;
    const uint32_t now = AP_HAL::millis();

    if (too_hot()) {
        should_run = false;
    } else if (state == ICE_OFF && cvalue >= 1700) {
        should_run = true;
    } else if (cvalue <= 1300) {
        should_run = false;
    } else if (state != ICE_OFF) {
        should_run = true;
    }


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
            if (!rpm.healthy(rpm_instance-1) ||
                rpm.get_rpm(rpm_instance-1) < rpm_threshold) {
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
        } else {
            // force ignition off when disarmed
            state = ICE_OFF;
        }
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

void AP_ICEngine::update_temperature()
{
    if (!temperature.is_valid()) {
        temperature.value = -999;
        return;
    }

    temperature.source->set_pin(temperature.pin);

    float v;
    if (temperature.ratiometric) {
        v = temperature.source->voltage_average_ratiometric();
    } else {
        v = temperature.source->voltage_average();
    }

    switch ((AP_ICEngine::Temperature_Function)temperature.function.get()) {
    case Temperature_Function::FUNCTION_LINEAR:
        temperature.value = (v - temperature.offset) * temperature.scaler;
        break;

    case Temperature_Function::FUNCTION_INVERTED:
        temperature.value = (temperature.offset - v) * temperature.scaler;
        break;

    case Temperature_Function::FUNCTION_HYPERBOLA:
        if (v <= temperature.offset) {
            temperature.value = -999;
        } else {
            temperature.value = temperature.scaler / (v - temperature.offset);
        }
        break;
    }

    if (isinf(temperature.value)) {
        temperature.value = -999;
    }
}

bool AP_ICEngine::too_hot() const
{
    if (!temperature.is_valid() ||
        temperature.max <= 0 ||
        temperature.max < temperature.min) {
        // disabled or invalid or not configured correctly
        return false;
    }
    return (temperature.value > temperature.max);
}

bool AP_ICEngine::too_cold() const
{
    if (!temperature.is_valid() ||
        temperature.min <= 0 ||
        temperature.min > temperature.max) {
        // disabled or invalid or not configured correctly
        return false;
    }
    return (temperature.value < temperature.min);
}

bool AP_ICEngine::get_temperature(float& value) const
{
    if (!temperature.is_valid() || temperature.value < -500) {
        return false;
    }
    value = temperature.value;
    return true;
}

/*
  check for throttle override. This allows the ICE controller to force
  the correct starting throttle when starting the engine
 */
bool AP_ICEngine::throttle_override(uint8_t &percentage)
{
    if (!enable) {
        return false;
    }
    if (too_cold() || too_hot()) {
        percentage = 0;
        return true;
    }
    if (state == ICE_STARTING || state == ICE_START_DELAY) {
        percentage = (uint8_t)start_percent.get();
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
    if (c != nullptr) {
        // get starter control channel
        if (c->get_radio_in() <= 1300) {
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
