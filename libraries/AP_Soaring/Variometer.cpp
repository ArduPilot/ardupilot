/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#include "Variometer.h"

#include <AP_Logger/AP_Logger.h>

Variometer::Variometer(const AP_Vehicle::FixedWing &parms, PolarParams &polarParams) :
    _aparm(parms),
    _polarParams(polarParams)
{
}

void Variometer::update(const float thermal_bank)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    _ahrs.get_relative_position_D_home(alt);
    alt = -alt;

    float aspd = 0;
    if (!_ahrs.airspeed_estimate(aspd)) {
            aspd = _aparm.airspeed_cruise_cm * 0.01f;
    }

    float aspd_filt = _sp_filter.apply(aspd);

    // Constrained airspeed.
    const float minV = sqrtf(_polarParams.K/1.5);
    _aspd_filt_constrained = aspd_filt>minV ? aspd_filt : minV;

    tau = calculate_circling_time_constant(radians(thermal_bank));

    float dt = (float)(AP_HAL::micros64() - _prev_update_time)/1e6;

    // Logic borrowed from AP_TECS.cpp
    // Update and average speed rate of change
    // Get DCM
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Calculate speed rate of change
    float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
    // take 5 point moving average
    float dsp = _vdot_filter.apply(temp);

    // Now we need to high-pass this signal to remove bias.
    _vdotbias_filter.set_cutoff_frequency(1/(20*tau));
    float dsp_bias = _vdotbias_filter.apply(temp, dt);
    
    float dsp_cor = dsp - dsp_bias;


    Vector3f velned;

    float raw_climb_rate = 0.0f;
    if (_ahrs.get_velocity_NED(velned)) {
        // if possible use the EKF vertical velocity
        raw_climb_rate = -velned.z;
    }
    
    _climb_filter.set_cutoff_frequency(1/(3*tau));
    float smoothed_climb_rate = _climb_filter.apply(raw_climb_rate, dt);

    // Compute still-air sinkrate
    float roll = _ahrs.roll;
    float sinkrate = calculate_aircraft_sinkrate(roll);

    reading = raw_climb_rate + dsp_cor*_aspd_filt_constrained/GRAVITY_MSS + sinkrate;
    
    // Update filters.

    float filtered_reading = _trigger_filter.apply(reading, dt);

    _audio_filter.apply(reading, dt);

    _stf_filter.apply(reading, dt);

    _prev_update_time = AP_HAL::micros64();

    _expected_thermalling_sink = calculate_aircraft_sinkrate(radians(thermal_bank));

// @LoggerMessage: VAR
// @Vehicles: Plane
// @Description: Variometer data
// @Field: TimeUS: Time since system startup
// @Field: aspd_raw: always zero
// @Field: aspd_filt: filtered and constrained airspeed
// @Field: alt: AHRS altitude
// @Field: roll: AHRS roll
// @Field: raw: estimated air vertical speed
// @Field: filt: low-pass filtered air vertical speed
// @Field: cl: raw climb rate
// @Field: fc: filtered climb rate
// @Field: exs: expected sink rate relative to air in thermalling turn
// @Field: dsp: average acceleration along X axis
// @Field: dspb: detected bias in average acceleration along X axis
    AP::logger().WriteStreaming("VAR", "TimeUS,aspd_raw,aspd_filt,alt,roll,raw,filt,cl,fc,exs,dsp,dspb", "Qfffffffffff",
                       AP_HAL::micros64(),
                       (double)0.0,
                       (double)_aspd_filt_constrained,
                       (double)alt,
                       (double)roll,
                       (double)reading,
                       (double)filtered_reading,
                       (double)_raw_climb_rate,
                       (double)smoothed_climb_rate,
                       (double)_expected_thermalling_sink,
                       (double)dsp,
                       (double)dsp_bias);
}


float Variometer::calculate_aircraft_sinkrate(float phi) const
{
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    CL0 = _polarParams.K / (_aspd_filt_constrained * _aspd_filt_constrained);

    C1 = _polarParams.CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = _polarParams.B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    float cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
    
    return _aspd_filt_constrained * (C1 + C2 / (cosphi * cosphi));
}

float Variometer::calculate_circling_time_constant(float thermal_bank)
{
    // Calculate a time constant to use to filter quantities over a full thermal orbit.
    // This is used for rejecting variation in e.g. climb rate, or estimated climb rate
    // potential, as the aircraft orbits the thermal.
    // Use the time to circle - variations at the circling frequency then have a gain of 25%
    // and the response to a step input will reach 64% of final value in three orbits.
    return 2*M_PI*_aspd_filt_constrained/(GRAVITY_MSS*tanf(thermal_bank));
}
