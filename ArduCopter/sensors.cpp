#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED && AP_RANGEFINDER_ENABLED
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
#if RANGEFINDER_ENABLED == ENABLED && AP_RANGEFINDER_ENABLED
    rangefinder.update();

    rangefinder_state.update();
    rangefinder_up_state.update();

#if HAL_PROXIMITY_ENABLED
    if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    }
#endif

#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return rangefinder_state.enabled_and_healthy();
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return rangefinder_up_state.enabled_and_healthy();
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

// helper function to get inertially interpolated rangefinder height.
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
#if RANGEFINDER_ENABLED == ENABLED && AP_RANGEFINDER_ENABLED
    return rangefinder_state.get_rangefinder_height_interpolated_cm(ret);
#else
    return false;
#endif
}
