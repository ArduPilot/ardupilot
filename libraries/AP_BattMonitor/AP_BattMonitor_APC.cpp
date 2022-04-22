#include "AP_BattMonitor_APC.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

# define AP_BATT_VOLT_FREQ_DEFAULT 100.0f
# define AP_BATT_VOLT_MAXVOLT_DEFAULT 5.0f
# define AP_BATT_VOLT_PWMLOW_DEFAULT 100
# define AP_BATT_VOLT_PWMUP_DEFAULT 20000
/*
    "battery" monitor for Analog to PWM Convertor chip(APC).
    
    It is an analog signal to PWM signal converter, which is equivalent to an ADC output by PWM signal.
 
    This chip linearly converts the analog voltage from the lower to upper limit voltage into a PWM signal with a duty cycle of 0% to 100%.
    (like GP9303T, lower limit voltage is 0V and upper limit voltage is 5V and PWM signal with a duty cycle of 0% to 100%)

    The actual voltage value can be scaled according to the peripheral circuit and adjusted by "VOLT_MULT".

    The ratio of PWM conversion voltage parameters is "VLT_PWMFreq"

    Actual Voltage: VOLT_MULT * volt_pulse_width * VLT_MaxVolt * VLT_PWMFreq 
 */

const AP_Param::GroupInfo AP_BattMonitor_APC::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the PWM input pin that should be used for voltage monitoring.
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor_APC, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: VLT_PWMFreq
    // @DisplayName: Voltage PWM Frequency
    // @Description: PWM frequency for APC conversion
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("VLT_PWMFREQ", 2, AP_BattMonitor_APC, _volt_pwmfreq, AP_BATT_VOLT_FREQ_DEFAULT),

    // @Param: VLT_MaxVolt
    // @DisplayName: Voltage Max Voltage
    // @Description: Maximum voltage of APC input
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_MAXVOLT", 3, AP_BattMonitor_APC, _volt_maxvolt, AP_BATT_VOLT_MAXVOLT_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 4, AP_BattMonitor_APC, _volt_offset, 0),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin to the actual battery's voltage (pin_voltage * VOLT_MULT). 
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 5, AP_BattMonitor_APC, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: VLT_PWMLOW
    // @DisplayName: Lower limit voltage PWM duty cycle values
    // @Description: This is used to set the minimum duty cycle value of the PWM that in the normal range
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("VLT_PWMLOW", 6, AP_BattMonitor_APC, _volt_pwm_low, AP_BATT_VOLT_PWMLOW_DEFAULT),
    // @Param: VLT_PWMUP
    // @DisplayName: Upper limit voltagePWM duty cycle values
    // @Description: This is used to set the maximum duty cycle value of the PWM that in the normal range
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("VLT_PWMUP", 7, AP_BattMonitor_APC, _volt_pwm_up, AP_BATT_VOLT_PWMUP_DEFAULT),  

    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_APC::AP_BattMonitor_APC(AP_BattMonitor& mon, 
                                       AP_BattMonitor::BattMonitor_State& mon_state, 
                                       AP_BattMonitor_Params& params): 
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    _state.voltage = 1.0; // show a fixed voltage of 1v
}

/*
  read - read the "voltage"
*/
void AP_BattMonitor_APC::read()
{
    // this copes with changing the pin at runtime
    if (!volt_pin_pwm_source.set_pin(_volt_pin, "APC_Voltage")) {
        _state.healthy = false;
        return;
    }

    uint32_t volt_pulse_width_us = volt_pin_pwm_source.get_pwm_us();

    const uint32_t volt_pwm_low = _volt_pwm_low;
    const uint32_t volt_pwm_up = _volt_pwm_up;

    uint32_t now_us = AP_HAL::micros();

    // check for invalid pulse
    if (volt_pulse_width_us <= volt_pwm_low || volt_pulse_width_us >= volt_pwm_up) {
        _state.healthy = false;
        return;
    }

    volt_pulse_width_us = constrain_uint32(volt_pulse_width_us, volt_pwm_low, volt_pwm_up);

    _state.voltage = (float)volt_pulse_width_us / (float)hz_to_usec(_volt_pwmfreq) * _volt_maxvolt * _volt_multiplier;

    _state.last_time_micros = now_us;

    _state.healthy = true;
}

