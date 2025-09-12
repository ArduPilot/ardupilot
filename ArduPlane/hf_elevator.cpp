/*
  high frequency elevator support.

  This allows you to split the elevator output into high and low
  frequency components, and mix elevator into he flaperon output
 */

#include "Plane.h"

#if AP_PLANE_HF_ELEVATOR_ENABLED

const AP_Param::GroupInfo Plane::HighFreqElevator::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable high frequency elevator
    // @Description: The high frequency elevator support allows splitting elevator output into high and low frequency components
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, HighFreqElevator, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: High frequency elevator frequency
    // @Description: Cutoff frequency of the high pass filter applied to the high frequency elevator demand.
    // @Units: Hz
    // @Range: 1.0 10.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FREQ", 2, HighFreqElevator, hf_elevator_freq, 2.0f),

    // @Param: PCT
    // @DisplayName: High frequency elevator percentage
    // @Description: Percentage of the high pass filtered elevator signal sent to the high frequency elevator. The remainder will be sent to he low frequency elevator.
    // @Units: pct
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PCT", 3, HighFreqElevator, hf_elevator_pct, 0),

    // @Param: FLAP_HF
    // @DisplayName: Gain from high frequency elevator to flap
    // @Description: Percentage of the high pass filtered elevator signal sent to the flaperon channels. set to 0 to disable.
    // @Units: pct
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLAP_HF", 4, HighFreqElevator, hf_elev_flap_mix_gain_pct, 0),

    // @Param: FLAP_LF
    // @DisplayName: Gain from low frequency elevator to flap
    // @Description: Percentage of the low pass filtered elevator signal sent to the flaperon channels. set to 0 to disable.
    // @Units: pct
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLAP_LF", 5, HighFreqElevator, lf_elev_flap_mix_gain_pct, 0),

    AP_GROUPEND
};


/*
  calculate high frequency and low frequency components of elevator
  output
*/
void Plane::HighFreqElevator::update(void)
{
    if (enable != 1 || hf_elevator_pct < 1) {
        // output normal elevator on LF channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator_lf, SRV_Channels::get_output_scaled(SRV_Channel::k_elevator));
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator_hf, 0.0f);
        lpf.reset();
        flaperon_mix = 0;
        return;
    }
    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);

    if (!is_equal((float)lpf.get_cutoff_freq(), (float)hf_elevator_freq)) {
        lpf.set_cutoff_frequency(plane.scheduler.get_loop_rate_hz(), (float)hf_elevator_freq);
    }

    float elevator_lf = lpf.apply(elevator);
    float elevator_hf = elevator - elevator_lf;

    // The allocation of the HF signal between high and low frequency elevators is adjustable to allow
    // some of the high freqency sactivity to be sent to the low frequency elevator. This enables the
    // actuator workoad to be balanced between the two channels.
    const float hf_fraction = 0.01 * (float)hf_elevator_pct;
    elevator_lf += elevator_hf * (1.0 - hf_fraction);
    elevator_hf *= hf_fraction;

    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator_lf, elevator_lf);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator_hf, elevator_hf);

    // calculate flaperon mix
    const float elevator_hf_mix = 0.01 * float(hf_elev_flap_mix_gain_pct) * elevator_hf;
    const float elevator_lf_mix = 0.01 * float(lf_elev_flap_mix_gain_pct) * elevator_lf;
    flaperon_mix = elevator_hf_mix + elevator_lf_mix;
}

#endif //  AP_PLANE_HF_ELEVATOR_ENABLED

