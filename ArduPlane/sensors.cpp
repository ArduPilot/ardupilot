#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{

    // notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight
    float height;
#if AP_TERRAIN_AVAILABLE
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // use the best available alt estimate via baro above home
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    rangefinder_height_update();
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    airspeed.update(should_log(MASK_LOG_IMU));

    // we calculate airspeed errors (and thus target_airspeed_cm) even
    // when airspeed is disabled as TECS may be using synthetic
    // airspeed for a quadplane transition
    calc_airspeed_errors();
    
    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }

    // low pass filter speed scaler, with 1Hz cutoff, at 10Hz
    const float speed_scaler = calc_speed_scaler();
    const float cutoff_Hz = 2.0;
    const float dt = 0.1;
    surface_speed_scaler += calc_lowpass_alpha_dt(dt, cutoff_Hz) * (speed_scaler - surface_speed_scaler);
}

/*
  update RPM sensors
 */
void Plane::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        logger.Write_RPM(rpm_sensor);
    }
}
