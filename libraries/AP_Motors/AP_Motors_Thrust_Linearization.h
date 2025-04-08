#pragma once

#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter.h>

class AP_Motors;
class Thrust_Linearization {
friend class AP_MotorsMulticopter;
friend class AP_MotorsMulticopter_test;
friend class AP_MotorsHeli_Single;
public:
    Thrust_Linearization(AP_Motors& _motors);

    // Apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
    float apply_thrust_curve_and_volt_scaling(float thrust) const;

    // Inverse of above
    float remove_thrust_curve_and_volt_scaling(float throttle) const;

    // Converts desired thrust to linearized actuator output in a range of 0~1
    float thrust_to_actuator(float thrust_in) const;

    // Inverse of above
    float actuator_to_thrust(float actuator) const;

    // Update_lift_max_from_batt_voltage - used for voltage compensation
    void update_lift_max_from_batt_voltage();

    // return gain scheduling gain based on voltage and air density
    float get_compensation_gain() const;

    // Get spin min parameter value
    float get_spin_min() const { return spin_min.get(); }

    // Get spin max parameter value
    float get_spin_max() const { return spin_max.get(); }

    // Get spin max parameter value
    int8_t get_battery_index() const { return batt_idx.get(); }

    // Get min battery voltage
    float get_battery_min_voltage() const { return batt_voltage_min.get(); }

    // Get lift max
    float get_lift_max() const { return lift_max; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Float curve_expo;       // curve used to linearize pwm to thrust conversion.  set to 0 for linear and 1 for second order approximation
    AP_Float spin_min;         // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float spin_max;         // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Int8  batt_idx;         // battery index used for compensation
    AP_Float batt_voltage_max; // maximum voltage used to scale lift
    AP_Float batt_voltage_min; // minimum voltage used to scale lift

private:
    float               lift_max;          // maximum lift ratio from battery voltage
    float               throttle_limit;    // ratio of throttle limit between hover and maximum
    LowPassFilterFloat  batt_voltage_filt; // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max

    AP_Motors& motors;
};
