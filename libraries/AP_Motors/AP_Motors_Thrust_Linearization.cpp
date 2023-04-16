
#include "AP_Motors_Thrust_Linearization.h"
#include "AP_Motors_Class.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AP_MOTORS_BATT_VOLT_FILT_HZ 0.5 // battery voltage filtered at 0.5hz

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // Example does not instantiate baro so cannot do density compensation
    #define AP_MOTORS_DENSITY_COMP 0
#else
    #ifndef AP_MOTORS_DENSITY_COMP
        #define AP_MOTORS_DENSITY_COMP 1
    #endif
#endif


Thrust_Linearization::Thrust_Linearization(AP_Motors& _motors) :
    motors(_motors),
    lift_max(1.0)
{
    // setup battery voltage filtering
    batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    batt_voltage_filt.reset(1.0);
}

// converts desired thrust to linearized actuator output in a range of 0~1
float Thrust_Linearization::thrust_to_actuator(float thrust_in) const
{
    thrust_in = constrain_float(thrust_in, 0.0, 1.0);
    return spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float Thrust_Linearization::actuator_to_thrust(float actuator) const
{
    actuator = (actuator - spin_min) /  (spin_max - spin_min);
    return constrain_float(remove_thrust_curve_and_volt_scaling(actuator), 0.0, 1.0);
}

// apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
float Thrust_Linearization::apply_thrust_curve_and_volt_scaling(float thrust) const
{
    float battery_scale = 1.0;
    if (is_positive(batt_voltage_filt.get())) {
        battery_scale = 1.0 / batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return lift_max * thrust * battery_scale;
    }
    float throttle_ratio = ((thrust_curve_expo - 1.0) + safe_sqrt((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo) + 4.0 * thrust_curve_expo * lift_max * thrust)) / (2.0 * thrust_curve_expo);
    return constrain_float(throttle_ratio * battery_scale, 0.0, 1.0);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float Thrust_Linearization::remove_thrust_curve_and_volt_scaling(float throttle) const
{
    float battery_scale = 1.0;
    if (is_positive(batt_voltage_filt.get())) {
        battery_scale = 1.0 / batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return  throttle / (lift_max * battery_scale);
    }
    float thrust = ((throttle / battery_scale) * (2.0 * thrust_curve_expo)) - (thrust_curve_expo - 1.0);
    thrust = (thrust * thrust) - ((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo));
    thrust /=  4.0 * thrust_curve_expo * lift_max;
    return constrain_float(thrust, 0.0, 1.0);
}

// update_lift_max from battery voltage - used for voltage compensation
void Thrust_Linearization::update_lift_max_from_batt_voltage()
{
    // sanity check battery_voltage_min is not too small
    // if disabled or misconfigured exit immediately
    float _batt_voltage_resting_estimate = AP::battery().voltage_resting_estimate(batt_idx);
    if ((batt_voltage_max <= 0) || (batt_voltage_min >= batt_voltage_max) || (_batt_voltage_resting_estimate < 0.25 * batt_voltage_min)) {
        batt_voltage_filt.reset(1.0);
        lift_max = 1.0;
        return;
    }

    batt_voltage_min.set(MAX(batt_voltage_min, batt_voltage_max * 0.6));

    // contrain resting voltage estimate (resting voltage is actual voltage with sag removed based on current draw and resistance)
    _batt_voltage_resting_estimate = constrain_float(_batt_voltage_resting_estimate, batt_voltage_min, batt_voltage_max);

    // filter at 0.5 Hz
    float batt_voltage_flittered = batt_voltage_filt.apply(_batt_voltage_resting_estimate / batt_voltage_max, motors.get_dt());

    // calculate lift max
    float thrust_curve_expo = constrain_float(curve_expo, -1.0, 1.0);
    lift_max = batt_voltage_flittered * (1 - thrust_curve_expo) + thrust_curve_expo * batt_voltage_flittered * batt_voltage_flittered;
}

// return gain scheduling gain based on voltage and air density
float Thrust_Linearization::get_compensation_gain() const
{
    // avoid divide by zero
    if (get_lift_max() <= 0.0) {
        return 1.0;
    }

    float ret = 1.0 / get_lift_max();

#if AP_MOTORS_DENSITY_COMP == 1
    // air density ratio is increasing in density / decreasing in altitude
    const float air_density_ratio = AP::baro().get_air_density_ratio();
    if (air_density_ratio > 0.3 && air_density_ratio < 1.5) {
        ret *= 1.0 / constrain_float(air_density_ratio, 0.5, 1.25);
    }
#endif
    return ret;
}
