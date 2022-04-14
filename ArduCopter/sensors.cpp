#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.update();
    rangefinder_up_state.update();

    wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#if MODE_CIRCLE_ENABLED
    circle_nav->set_rangefinder_alt(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#endif
#if HAL_PROXIMITY_ENABLED
    g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#endif

#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret)
{
    if (!rangefinder_alt_ok()) {
        return false;
    }
    ret = rangefinder_state.alt_cm_filt.get();
    float inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    ret += inertial_alt_cm - rangefinder_state.inertial_alt_cm;
    return true;
}
