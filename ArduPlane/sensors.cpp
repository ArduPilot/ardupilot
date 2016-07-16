// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>

void Plane::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    } else {
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

void Plane::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init();
#endif
}

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED

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
        if (flight_stage == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
            flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
            flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
            // ensure the rangefinder is powered-on when land alt is higher than home altitude.
            // This is done using the target alt which we know is below us and we are sinking to it
            height = height_above_target();
        } else {
            // otherwise just use the best available baro estimate above home.
            height = relative_altitude();
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    rangefinder.update();

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();

    rangefinder_height_update();
#endif
}

/*
  calibrate compass
*/
void Plane::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

/*
    Accel calibration
*/
void Plane::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }

        // set target_airspeed_cm and airspeed_error
        calc_airspeed_errors();

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }

        // check for airspeed hardware failure but wait until:
        // a few seconds after a launch or until after an auto takeoff completes.
        bool stages_to_ignore = (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF ||
                flight_stage == AP_SpdHgtControl::FLIGHT_LAND_PREFLARE ||
                flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL ||
                flight_stage == AP_SpdHgtControl::FLIGHT_VTOL);

        if (is_flying() &&
            !(control_mode == AUTO && stages_to_ignore) &&
            AP_HAL::millis() - started_flying_ms > 10000) {

            bool airspeed_data_validity;

            // perform airspeed hardware failure self-check. Fail when error > abs(target*25%)
            float trashf; Vector2f trash2f; Vector3f trash3f;
            float true_airspeed_variance;
            if (ahrs.get_variances(trashf,trashf,trashf,trash3f, true_airspeed_variance, trash2f)) {
                airspeed_data_validity = true_airspeed_variance < 1.0f;
            } else {
                const float error_threshold = target_airspeed_cm * 0.002f; // cmd to m conversion * 20% error.
                airspeed_data_validity = fabsf(airspeed_error) < error_threshold;
            }

            bool is_failure_detected = airspeed.self_check(airspeed_data_validity);
            if (is_failure_detected &&
                (airspeed.get_fail_action_mask() & AP_Airspeed::FAILURE_ACTION_RTL)) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ALERT, "Airspeed failure: setting mode to RTL");
                set_mode(RTL);
            }
        }
    }

    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}

void Plane::zero_airspeed(bool in_startup)
{
    airspeed.calibrate(in_startup);
    read_airspeed();
    // update barometric calibration with new airspeed supplied temperature
    barometer.update_calibration();
    gcs_send_text(MAV_SEVERITY_INFO,"Airspeed calibration started");
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Plane::read_battery(void)
{
    battery.read();
    compass.set_current(battery.current_amps());

    if (!usb_connected && 
        hal.util->get_soft_armed() &&
        battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
    
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Plane::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

/*
  update RPM sensors
 */
void Plane::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.healthy(0) || rpm_sensor.healthy(1)) {
        if (should_log(MASK_LOG_RC)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}
