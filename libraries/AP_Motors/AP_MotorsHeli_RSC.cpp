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

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsHeli_RSC.h"
#include <AP_RPM/AP_RPM.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_RSC::var_info[] = {

    // @Param: SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: Throttle (HeliRSC Servo) output in percent to the external motor governor when motor interlock enabled (throttle hold off).
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SETPOINT", 1, AP_MotorsHeli_RSC, _rsc_setpoint, AP_MOTORS_HELI_RSC_SETPOINT),

    // @Param: MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: Selects the type of rotor speed control used to determine throttle output to the HeliRSC servo channel when motor interlock is enabled (throttle hold off). RC Passthrough sends the input from the RC Motor Interlock channel as throttle output.  External Gov SetPoint sends the RSC SetPoint parameter value as throttle output.  Throttle Curve uses the 5 point throttle curve to determine throttle output based on the collective output.  AutoThrottle requires a rotor speed sensor, contains an advanced autothrottle governor and is primarily for piston and turbine engines. WARNING: Throttle ramp time and throttle curve MUST be tuned properly using Throttle Curve mode before using AutoThrottle
    // @Values: 1:RC Passthrough, 2:External Gov SetPoint, 3:Throttle Curve, 4:AutoThrottle
    // @User: Standard
    AP_GROUPINFO("MODE", 2, AP_MotorsHeli_RSC, _rsc_mode, (int8_t)ROTOR_CONTROL_MODE_PASSTHROUGH),

    // @Param: RAMP_TIME
    // @DisplayName: Throttle Ramp Time
    // @Description: Time in seconds for throttle output (HeliRSC servo) to ramp from ground idle (RSC_IDLE) to flight idle throttle setting when motor interlock is enabled (throttle hold off).
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RAMP_TIME", 3, AP_MotorsHeli_RSC, _ramp_time, AP_MOTORS_HELI_RSC_RAMP_TIME),

    // @Param: RUNUP_TIME
    // @DisplayName: Rotor Runup Time
    // @Description: Actual time in seconds for the main rotor to reach full speed after motor interlock is enabled (throttle hold off). Must be at least one second longer than the Throttle Ramp Time that is set with RSC_RAMP_TIME. WARNING: For AutoThrottle users with piston and turbine engines it is VERY important to know how long it takes to warm up your engine and reach full rotor speed when throttle switch is turned ON. This timer should be set for at least the amount of time it takes to get your helicopter to full flight power, ready for takeoff. Failure to heed this warning could result in the auto-takeoff mode attempting to lift up into hover before the engine has reached full power, and subsequent loss of control
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RUNUP_TIME", 4, AP_MotorsHeli_RSC, _runup_time, AP_MOTORS_HELI_RSC_RUNUP_TIME),

    // @Param: CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Percentage of normal rotor speed where flight is no longer possible. However currently the rotor runup/rundown is estimated using the RSC_RUNUP_TIME parameter.   Estimated rotor speed increases/decreases between 0 (rotor stopped) to 1 (rotor at normal speed) in the RSC_RUNUP_TIME in seconds. This parameter should be set so that the estimated rotor speed goes below critical in approximately 3 seconds.  So if you had a 10 second runup time then set RSC_CRITICAL to 70%.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRITICAL", 5, AP_MotorsHeli_RSC, _critical_speed, AP_MOTORS_HELI_RSC_CRITICAL),

    // @Param: IDLE
    // @DisplayName: Throttle Output at Idle
    // @Description: Throttle output (HeliRSC Servo) in percent while armed but motor interlock is disabled (throttle hold on). FOR COMBUSTION ENGINES. Sets the engine ground idle throttle percentage with clutch disengaged. This must be set to zero for electric helicopters under most situations. If the ESC has an autorotation window this can be set to keep the autorotation window open in the ESC. Consult the operating manual for your ESC to set it properly for this purpose
    // @Range: 0 50
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IDLE", 6, AP_MotorsHeli_RSC, _idle_output, AP_MOTORS_HELI_RSC_IDLE_DEFAULT),

    // @Param: SLEWRATE
    // @DisplayName: Throttle Slew Rate
    // @Description: This controls the maximum rate at which the throttle output (HeliRSC servo) can change, as a percentage per second. A value of 100 means the throttle can change over its full range in one second. A value of zero gives unlimited slew rate.
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 7, AP_MotorsHeli_RSC, _power_slewrate, 0),

    // @Param: THRCRV_0
    // @DisplayName: Throttle Curve at 0% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 0 percent collective is defined by H_COL_MIN. Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to -2 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_0", 8, AP_MotorsHeli_RSC, _thrcrv[0], AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT),

    // @Param: THRCRV_25
    // @DisplayName: Throttle Curve at 25% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 25% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_25", 9, AP_MotorsHeli_RSC, _thrcrv[1], AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT),

    // @Param: THRCRV_50
    // @DisplayName: Throttle Curve at 50% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 50% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_50", 10, AP_MotorsHeli_RSC, _thrcrv[2], AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT),

    // @Param: THRCRV_75
    // @DisplayName: Throttle Curve at 75% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 75% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_75", 11, AP_MotorsHeli_RSC, _thrcrv[3], AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT),

    // @Param: THRCRV_100
    // @DisplayName: Throttle Curve at 100% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to +10 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_100", 12, AP_MotorsHeli_RSC, _thrcrv[4], AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT),

    // Indices 13 thru 16 have been re-assigned and should not be used in the future

    // @Param: GOV_RANGE
    // @DisplayName: Governor Operational Range
    // @Description: RPM range +/- governor rpm reference setting where governor is operational. If speed sensor fails or rpm falls outside of this range, the governor will disengage and return to throttle curve. Recommended range is 100
    // @Range: 50 200
    // @Units: RPM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GOV_RANGE", 17, AP_MotorsHeli_RSC, _governor_range, AP_MOTORS_HELI_RSC_GOVERNOR_RANGE_DEFAULT),

    // Index 18 was renamed from AROT_PCT to AROT_IDLE

    // @Param: CLDWN_TIME
    // @DisplayName: Cooldown Time
    // @Description: Will provide a fast idle for engine cooldown by raising the Ground Idle speed setting by 50% for the number of seconds the timer is set for. A setting of zero disables the fast idle. This feature will only apply after the runup complete has been declared. This will not extend the time before ground idle is declared, which triggers engine shutdown for autonomous landings.
    // @Range: 0 120
    // @Units: s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CLDWN_TIME", 19, AP_MotorsHeli_RSC, _cooldown_time, 0),

    // @Param: GOV_COMP
    // @DisplayName: Governor Torque Compensator
    // @Description: Adjusts the autothrottle governor torque compensator that determines how fast the governor will adjust the base torque reference to compensate for changes in density altitude. If RPM is low or high by more than 2-5 RPM, increase this setting by 1% at a time until the governor speed matches your RPM setting. Setting the compensator too high can result in surging and throttle "hunting". Do not make large adjustments at one time
    // @Range: 0 70
    // @Units: %
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GOV_COMP", 20, AP_MotorsHeli_RSC, _governor_compensator, 25),

    // @Param: GOV_DROOP
    // @DisplayName: Governor Droop Compensator
    // @Description: AutoThrottle governor droop response under load, normal settings of 0-50%. Higher value is quicker response to large speed changes due to load but may cause surging. Adjust this to be as aggressive as possible without getting surging or RPM over-run when the governor responds to large load changes on the rotor system
    // @Range: 0 100
    // @Units: %
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GOV_DROOP", 21, AP_MotorsHeli_RSC, _governor_droop_response, 25),

    // @Param: GOV_FF
    // @DisplayName: Governor Feedforward
    // @Description: Feedforward governor gain to throttle response during sudden loading/unloading of the rotor system. If RPM drops excessively during full collective climb with the droop response set correctly, increase the governor feedforward.
    // @Range: 0 100
    // @Units: %
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GOV_FF", 22, AP_MotorsHeli_RSC, _governor_ff, 50),

    // @Param: GOV_RPM
    // @DisplayName: Rotor RPM Setting
    // @Description: Main rotor RPM that governor maintains when engaged
    // @Range: 800 3500
    // @Units: RPM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GOV_RPM", 23, AP_MotorsHeli_RSC, _governor_rpm, 1500),

    // @Param: GOV_TORQUE
    // @DisplayName: Governor Torque Limiter
    // @Description: Adjusts the engine's percentage of torque rise on autothrottle during ramp-up to governor speed. The torque rise will determine how fast the rotor speed will ramp up when rotor speed reaches 50% of the rotor RPM setting. The sequence of events engaging the governor is as follows: Throttle ramp time will engage the clutch and start the main rotor turning. The collective should be at flat pitch and the throttle curve set to provide at least 50% of normal RPM at flat pitch. The autothrottle torque limiter will automatically activate and start accelerating the main rotor. If the autothrottle consistently fails to accelerate the main rotor during ramp-in due to engine tune or other factors, then increase the torque limiter setting. NOTE: throttle ramp time and throttle curve should be tuned using RSC_MODE Throttle Curve before using RSC_MODE AutoThrottle
    // @Range: 10 60
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_TORQUE", 24, AP_MotorsHeli_RSC, _governor_torque, 30),

    // @Param: AROT_ENG_T
    // @DisplayName: Time for in-flight power re-engagement
    // @Description: amount of seconds to move throttle output from idle to throttle curve position during manual autorotations
    // @Range: 0 10
    // @Units: %
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("AROT_ENG_T", 25, AP_MotorsHeli_RSC, _rsc_arot_engage_time, AP_MOTORS_HELI_RSC_AROT_ENGAGE_TIME),

    // @Param: AROT_MN_EN
    // @DisplayName: Enable Manual Autorotations
    // @Description: Allows you to enable (1) or disable (0) the manual autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AROT_MN_EN", 26, AP_MotorsHeli_RSC, _rsc_arot_man_enable, 0),

    // @Param: AROT_IDLE
    // @DisplayName: Idle Throttle Percentage during Autorotation
    // @Description: Idle throttle used for all RSC modes.  For external governors, this would be set to signal it to enable fast spool-up, when bailing out of an autorotation.  Set 0 to disable. If also using a tail rotor of type DDVP with external governor then this value must lie within the autorotation window of both governors.
    // @Range: 0 40
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("AROT_IDLE", 27, AP_MotorsHeli_RSC, _arot_idle_output, AP_MOTORS_HELI_RSC_AROT_IDLE),

    AP_GROUPEND
};

// init_servo - servo initialization on start-up
void AP_MotorsHeli_RSC::init_servo()
{
    // setup RSC on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn, _default_channel);

}

// set_power_output_range
// TODO: Look at possibly calling this at a slower rate.  Doesn't need to be called every cycle.
void AP_MotorsHeli_RSC::set_throttle_curve()
{
    float thrcrv[5];
    // Ensure user inputs are within parameter limits
    // Scale throttle curve parameters
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i] = constrain_float(_thrcrv[i] * 0.01f, 0.0f, 1.0f);
    }
    // Calculate the spline polynomials for the throttle curve
    splinterp5(thrcrv,_thrcrv_poly);
}

// output - update value to send to ESC/Servo
void AP_MotorsHeli_RSC::output(RotorControlState state)
{
    // _rotor_RPM available to the RSC output
#if AP_RPM_ENABLED
    const AP_RPM *rpm = AP_RPM::get_singleton();
    if (rpm != nullptr) {
        if (!rpm->get_rpm(0, _rotor_rpm)) {
            // No valid RPM data
            _rotor_rpm = -1;
        }
    } else {
        // No RPM because pointer is null
        _rotor_rpm = -1;
    }
#else
    _rotor_rpm = -1;
#endif

    float dt;
    uint64_t now = AP_HAL::micros64();
    float last_control_output = _control_output;

    if (_last_update_us == 0) {
        _last_update_us = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-6f * (now - _last_update_us);
        _last_update_us = now;
    }

    switch (state) {
    case ROTOR_CONTROL_STOP:
        // set rotor ramp to decrease speed to zero, this happens instantly inside update_rotor_ramp()
        update_rotor_ramp(0.0f, dt);

        // control output forced to zero
        _control_output = 0.0f;

        // governor is forced to disengage status and reset outputs
        governor_reset();
        _autothrottle = false;
        _governor_fault = false;
        //turbine start flag on
        _starting = true;
        _autorotating = false;
        _bailing_out = false;
        _gov_bailing_out = false;

        // ensure _idle_throttle not set to invalid value
        _idle_throttle = get_idle_output();
        break;

    case ROTOR_CONTROL_IDLE:
        // set rotor ramp to decrease speed to zero
        update_rotor_ramp(0.0f, dt);

        // set rotor control speed to engine idle and ensure governor is reset, if used
        governor_reset();
        _autothrottle = false;
        _governor_fault = false;
        if (_in_autorotation) {
            // if in autorotation, set the output to idle for autorotation. This will tell an external governor to use fast ramp for spool up.
            // if autorotation idle is set to zero then default to the RSC idle value.
            if (_arot_idle_output == 0) {
                _idle_throttle = get_idle_output();
            } else {
                _idle_throttle = constrain_float( get_arot_idle_output(), 0.0f, 0.4f);
            }
            if (!_autorotating) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Autorotation");
                _autorotating =true;
            }
        } else {
            if (_autorotating) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Autorotation Stopped");
                _autorotating =false;
            }
            // set rotor control speed to idle speed parameter, this happens instantly and ignores ramping
            if (_turbine_start && _starting == true ) {
                _idle_throttle += 0.001f;
                if (_control_output >= 1.0f) {
                    _idle_throttle = get_idle_output();
                    gcs().send_text(MAV_SEVERITY_INFO, "Turbine startup");
                    _starting = false;
                }
            } else {
                if (_cooldown_time > 0) {
                    _idle_throttle = get_idle_output() * 1.5f;
                    _fast_idle_timer += dt;
                    if (_fast_idle_timer > (float)_cooldown_time) {
                        _fast_idle_timer = 0.0f;
                    }
                } else {
                    _idle_throttle = get_idle_output();
                }
            }
            // this resets the bailout feature if the aircraft has landed.
            _use_bailout_ramp = false;
            _bailing_out = false;
            _gov_bailing_out = false;
        }
        _control_output = _idle_throttle;
        break;

    case ROTOR_CONTROL_ACTIVE:
        // set main rotor ramp to increase to full speed
        update_rotor_ramp(1.0f, dt);

        // ensure _idle_throttle not set to invalid value due to premature switch out of turbine start
        if (_starting) {
            _idle_throttle = get_idle_output();
        }
        // if turbine engine started without using start sequence, set starting flag just to be sure it can't be triggered when back in idle
        _starting = false;
        _autorotating = false;

        if ((_control_mode == ROTOR_CONTROL_MODE_PASSTHROUGH) || (_control_mode == ROTOR_CONTROL_MODE_SETPOINT)) {
            // set control rotor speed to ramp slewed value between idle and desired speed
            _control_output = _idle_throttle + (_rotor_ramp_output * (_desired_speed - _idle_throttle));
        } else if (_control_mode == ROTOR_CONTROL_MODE_THROTTLECURVE) {
            // throttle output from throttle curve based on collective position
            float throttlecurve = calculate_throttlecurve(_collective_in);
            _control_output = _idle_throttle + (_rotor_ramp_output * (throttlecurve - _idle_throttle));
        } else if (_control_mode == ROTOR_CONTROL_MODE_AUTOTHROTTLE) {
            autothrottle_run();
        }
        break;
    }

    // update rotor speed run-up estimate
    update_rotor_runup(dt);

    if (_power_slewrate > 0) {
        // implement slew rate for throttle
        float max_delta = dt * _power_slewrate * 0.01f;
        _control_output = constrain_float(_control_output, last_control_output-max_delta, last_control_output+max_delta);
    }

    // output to rsc servo
    write_rsc(_control_output);
}

// update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
void AP_MotorsHeli_RSC::update_rotor_ramp(float rotor_ramp_input, float dt)
{
    int8_t ramp_time;
    int8_t bailout_time;
    // sanity check ramp time and enable bailout if set
    if (_ramp_time <= 0) {
        ramp_time = 1;
    } else {
        ramp_time = _ramp_time;
    }

    if (_rsc_arot_engage_time <= 0) {
        bailout_time = 1;
    } else {
        bailout_time = _rsc_arot_engage_time;
    }

    // ramp output upwards towards target
    if (_rotor_ramp_output < rotor_ramp_input) {
        if (_use_bailout_ramp) {
            if (!_bailing_out) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "bailing_out");
                _bailing_out = true;
                if (_control_mode == ROTOR_CONTROL_MODE_AUTOTHROTTLE) {_gov_bailing_out = true;}
            }
            _rotor_ramp_output += (dt / bailout_time);
        } else {
            _rotor_ramp_output += (dt / ramp_time);
        }
        if (_rotor_ramp_output > rotor_ramp_input) {
            _rotor_ramp_output = rotor_ramp_input;
            _bailing_out = false;
            _use_bailout_ramp = false;
        }
    } else {
        // ramping down happens instantly
        _rotor_ramp_output = rotor_ramp_input;
    }
}

// update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
void AP_MotorsHeli_RSC::update_rotor_runup(float dt)
{
    int8_t runup_time = _runup_time;
    // sanity check runup time
    runup_time = MAX(_ramp_time+1,runup_time);

    // adjust rotor runup when bailing out
    if (_use_bailout_ramp) {
        // maintain same delta as set in parameters
        runup_time = _runup_time-_ramp_time+1;
    }

    // protect against divide by zero
    runup_time = MAX(1,runup_time);

    // ramp speed estimate towards control out
    float runup_increment = dt / runup_time;
    if (_rotor_runup_output < _rotor_ramp_output) {
        _rotor_runup_output += runup_increment;
        if (_rotor_runup_output > _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    } else {
        _rotor_runup_output -= runup_increment;
        if (_rotor_runup_output < _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }
    // if in autorotation, don't let rotor_runup_output go less than critical speed to keep
    // runup complete flag from being set to false
    if (_autorotating && !rotor_speed_above_critical()) {
        _rotor_runup_output = get_critical_speed();
    }

    // update run-up complete flag

    // if control mode is disabled, then run-up complete always returns true
    if ( _control_mode == ROTOR_CONTROL_MODE_DISABLED ) {
        _runup_complete = true;
        return;
    }

    // if rotor ramp and runup are both at full speed, then run-up has been completed
    if (!_runup_complete && (_rotor_ramp_output >= 1.0f) && (_rotor_runup_output >= 1.0f)) {
        _runup_complete = true;
    }
    // if rotor speed is less than critical speed, then run-up is not complete
    // this will prevent the case where the target rotor speed is less than critical speed
    if (_runup_complete && !rotor_speed_above_critical()) {
        _runup_complete = false;
    }
    // if rotor estimated speed is zero, then spooldown has been completed
    if (get_rotor_speed() <= 0.0f) {
        _spooldown_complete = true;
    } else {
        _spooldown_complete = false;
    }
}

// get_rotor_speed - gets rotor speed either as an estimate, or (ToDO) a measured value
float AP_MotorsHeli_RSC::get_rotor_speed() const
{
    // if no actual measured rotor speed is available, estimate speed based on rotor runup scalar.
    return _rotor_runup_output;
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1
void AP_MotorsHeli_RSC::write_rsc(float servo_out)
{
    if (_control_mode == ROTOR_CONTROL_MODE_DISABLED) {
        // do not do servo output to avoid conflicting with other output on the channel
        // ToDo: We should probably use RC_Channel_Aux to avoid this problem
        return;
    } else {
        SRV_Channels::set_output_scaled(_aux_fn, servo_out * 1000);
    }
}

// Return mask of output channels which the RSC is outputting on
uint32_t AP_MotorsHeli_RSC::get_output_mask() const
{
    if (_control_mode == ROTOR_CONTROL_MODE_DISABLED) {
        return 0;
    }
    return SRV_Channels::get_output_channel_mask(_aux_fn);
}

// calculate_throttlecurve - uses throttle curve and collective input to determine throttle setting
float AP_MotorsHeli_RSC::calculate_throttlecurve(float collective_in)
{
    const float inpt = collective_in * 4.0f + 1.0f;
    uint8_t idx = constrain_int16(int8_t(collective_in * 4), 0, 3);
    const float a = inpt - (idx + 1.0f);
    const float b = (idx + 1.0f) - inpt + 1.0f;
    float throttle = _thrcrv_poly[idx][0] * a + _thrcrv_poly[idx][1] * b + _thrcrv_poly[idx][2] * (powf(a,3.0f) - a) / 6.0f + _thrcrv_poly[idx][3] * (powf(b,3.0f) - b) / 6.0f;

    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return throttle;

}

// autothrottle_run - calculate throttle output for governor controlled throttle
void AP_MotorsHeli_RSC::autothrottle_run()
{
    float throttlecurve = calculate_throttlecurve(_collective_in);
    float const torque_ref_error_rpm = 2.0f;

    // if the desired governor RPM is zero, use the throttle curve only and exit
    if (_governor_rpm == 0) {
        _control_output = _idle_throttle + (_rotor_ramp_output * (throttlecurve - _idle_throttle));
        return;
    }

    // autothrottle main power loop with governor
    if (_governor_engage && !_governor_fault) {
        // calculate droop - difference between actual and desired speed
        float governor_droop = ((float)_governor_rpm - _rotor_rpm) * _governor_droop_response * 0.0001f;
        _governor_output = governor_droop + ((throttlecurve - _governor_torque_reference) * _governor_ff * 0.01);
        if (_rotor_rpm < ((float)_governor_rpm - torque_ref_error_rpm)) {
            _governor_torque_reference += get_governor_compensator();   // torque compensator
        } else if (_rotor_rpm > ((float)_governor_rpm + torque_ref_error_rpm)) {
            _governor_torque_reference -= get_governor_compensator();
        }
        // throttle output uses droop + torque compensation to maintain proper rotor speed
        _control_output = constrain_float((_governor_torque_reference + _governor_output), (get_idle_output() * 1.5f), 1.0f);
        // governor and speed sensor fault detection - must maintain RPM within governor range
        // speed fault detector will allow a fault to persist for 200 contiguous governor updates
        // this is failsafe for bad speed sensor or severely mis-adjusted governor
        if ((_rotor_rpm <= (_governor_rpm - _governor_range)) || (_rotor_rpm >= (_governor_rpm + _governor_range))) {
            _governor_fault_count++;
            if (_governor_fault_count > 200) {
                governor_reset();
                _governor_fault = true;
                if (_rotor_rpm >= (_governor_rpm + _governor_range)) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Governor Fault: Rotor Overspeed");
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Governor Fault: Rotor Underspeed");
                }
            }
        } else {
            _governor_fault_count = 0;   // reset fault count if the fault doesn't persist
        }
    } else if (!_governor_engage && !_governor_fault) {
        // if governor is not engaged and rotor is overspeeding by more than governor range due to 
        // misconfigured throttle curve or stuck throttle, set a fault and governor will not operate
        if (_rotor_rpm > (_governor_rpm + _governor_range) && !_gov_bailing_out) {
            _governor_fault = true;
            governor_reset();
            gcs().send_text(MAV_SEVERITY_WARNING, "Governor Fault: Rotor Overspeed");
            _governor_output = 0.0f;

        // when performing power recovery from autorotation, this waits for user to load rotor in order to 
        // engage the governor
        } else if (_rotor_rpm > _governor_rpm && _gov_bailing_out) {
            _governor_output = 0.0f;

            // torque rise limiter accelerates rotor to the reference speed
            // this limits the max torque rise the governor could call for from the main power loop
        } else if (_rotor_rpm >= (_governor_rpm * 0.5f)) {
            float torque_limit = (get_governor_torque() * get_governor_torque());
            _governor_output = (_rotor_rpm / (float)_governor_rpm) * torque_limit;
            if (_rotor_rpm >= ((float)_governor_rpm - torque_ref_error_rpm)) {
                _governor_engage = true;
                _autothrottle = true;
                _gov_bailing_out = false;
                gcs().send_text(MAV_SEVERITY_NOTICE, "Governor Engaged");
            }
        } else {
            // temporary use of throttle curve and ramp timer to accelerate rotor to governor min torque rise speed
            _governor_output = 0.0f;
        }
        _control_output = constrain_float(_idle_throttle + (_rotor_ramp_output * (throttlecurve + _governor_output - _idle_throttle)), 0.0f, 1.0f);
        _governor_torque_reference = _control_output;  // increment torque setting to be passed to main power loop
    } else {
        // failsafe - if governor has faulted use throttle curve
        _control_output = _idle_throttle + (_rotor_ramp_output * (throttlecurve - _idle_throttle));
    }
}

// governor_reset - disengages governor and resets outputs
void AP_MotorsHeli_RSC::governor_reset()
{
    _governor_output = 0.0f;
    _governor_torque_reference = 0.0f;
    _governor_engage = false;
    _governor_fault_count = 0;   // reset fault count when governor reset
}
