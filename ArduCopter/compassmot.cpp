#include "Copter.h"

/*
  compass/motor interference calibration
 */

// setup_compassmot - sets compass's motor interference parameters
MAV_RESULT Copter::mavlink_compassmot(const GCS_MAVLINK &gcs_chan)
{
#if FRAME_CONFIG == HELI_FRAME
    // compassmot not implemented for tradheli
    return MAV_RESULT_UNSUPPORTED;
#else
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base[COMPASS_MAX_INSTANCES];           // compass vector when throttle is zero
    Vector3f motor_compensation[COMPASS_MAX_INSTANCES];     // final compensation to be stored to eeprom
    float    throttle_pct = 0.0f;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    current_amps_min = 3.0f;   // minimum current for an effective reading
    float    interference_pct[COMPASS_MAX_INSTANCES]{};       // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint32_t last_send_time;
    bool     updated = false;           // have we updated the compensation vector at least once
    uint8_t  command_ack_start = command_ack_counter;
    uint16_t motor_num = 0;
    uint32_t motor_mask = motors->get_motor_mask();
    bool update_motor = false;

    // exit immediately if we are already in compassmot
    if (ap.compass_mot) {
        // ignore restart messages
        return MAV_RESULT_TEMPORARILY_REJECTED;
    } else {
        ap.compass_mot = true;
    }

    // check compass is enabled
    if (!AP::compass().available()) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "CMOT: compass disabled");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // disable motor compensation before checking health
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    // give compass a little time to settle
    hal.scheduler->delay(100);

    // check compass health
    compass.read();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        if (!compass.healthy(i)) {
            gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "CMOT: check compass");
            ap.compass_mot = false;
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }

    // check if radio is calibrated
    if (!arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "CMOT: RC not calibrated");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // check throttle is at zero
    read_radio();
    if (channel_throttle->get_control_in() != 0) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "CMOT: throttle not zero");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // check we are landed
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "CMOT: not landed");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // disable cpu failsafe
    failsafe_disable();

    float current;

    // default compensation type to use current if possible
    if (battery.current_amps(current)) {
        if (compass.per_motor_compensation_enabled()) {
            comp_type = AP_COMPASS_MOT_COMP_PER_MOTOR;
            current_amps_min = 1.0f;
        } else {
            comp_type = AP_COMPASS_MOT_COMP_CURRENT;
        }
    } else {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // send back initial ACK
    mavlink_msg_command_ack_send(gcs_chan.get_chan(), MAV_CMD_PREFLIGHT_CALIBRATION,0,
                                 0, 0, 0, 0);

    // flash leds
    AP_Notify::flags.esc_calibration = true;

    // warn user we are starting calibration
    gcs_chan.send_text(MAV_SEVERITY_INFO, "Starting calibration");

    // inform what type of compensation we are attempting
    if (comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Current");
    } else if (comp_type == AP_COMPASS_MOT_COMP_PER_MOTOR) {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Current per-motor");
    } else {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Throttle");
    }

    // disable throttle failsafe
    g.failsafe_throttle.set(FS_THR_DISABLED);

    // get initial compass readings
    compass.read();

    // store initial x,y,z compass values
    // initialise interference percentage
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass_base[i] = compass.get_field(i);
        interference_pct[i] = 0.0f;
    }

    EXPECT_DELAY_MS(5000);

    hal.scheduler->delay(100);
    // enable motors and pass through throttle
    motors->output_min();  // output lowest possible value to motors
    motors->armed(true);
    hal.util->set_soft_armed(true);

    // initialise run time
    last_run_time = millis();
    last_send_time = millis();

    // main run while there is no user input and the compass is healthy
    while (command_ack_start == command_ack_counter && compass.healthy() && motors->armed()) {
        EXPECT_DELAY_MS(5000);

        // 50hz loop
        if (millis() - last_run_time < 20) {
            hal.scheduler->delay(5);
            continue;
        }
        last_run_time = millis();

        // read radio input
        read_radio();

        // calculate scaling for throttle
        throttle_pct = (float)channel_throttle->get_control_in() * 0.001f;
        throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

        // pass through throttle to motors
        SRV_Channels::cork();
        if (comp_type == AP_COMPASS_MOT_COMP_PER_MOTOR) {
            motors->set_throttle_passthrough_per_motor(throttle_pct, motor_num);
        } else {
            motors->set_throttle_passthrough_for_esc_calibration(throttle_pct);
        }
        SRV_Channels::push();

        // read some compass values
        compass.read();

        // read current
        battery.read();

        // record maximum throttle
        throttle_pct_max = MAX(throttle_pct_max, throttle_pct);

        if (!battery.current_amps(current)) {
            current = 0;
        }
        current_amps_max = MAX(current_amps_max, current);

        // if throttle is near zero, update base x,y,z values
        if (!is_positive(throttle_pct)) {
            for (uint8_t i=0; i<compass.get_count(); i++) {
                compass_base[i] = compass_base[i] * 0.99f + compass.get_field(i) * 0.01f;
            }
            update_motor = true;
        } else {
            if (update_motor) {
                if (updated) {
                    for (uint8_t i=0; i<compass.get_count(); i++) {
                        compass.per_motor_set_compensation(i, motor_num, motor_compensation[i]);
                        motor_compensation[i].zero();
                        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibrated motor %u", motor_num);
                    }
                    motor_mask &= ~(1U<<motor_num);
                    if (!motor_mask) {
                        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibrated all motors");
                    }
                }
                const uint16_t prev_motor = motor_num;
                do {
                    // turn off the previous motor
                    motor_num = (motor_num + 1) % AP_MOTORS_MAX_NUM_MOTORS;
                    throttle_pct_max = throttle_pct;
                    current_amps_max = current;
                } while (!motors->is_motor_enabled(motor_num));
                motors->set_throttle_passthrough_per_motor(0.0f, prev_motor);
                update_motor = false;
                updated = false;
            }
            for (uint8_t i=0; i<compass.get_count(); i++) {

                // calculate diff from compass base and scale with throttle
                Vector3f motor_impact = compass.get_field(i) - compass_base[i];

                // throttle based compensation
                if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                    // scale by throttle
                    Vector3f motor_impact_scaled = motor_impact / throttle_pct;
                    // adjust the motor compensation to negate the impact
                    motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                } else {
                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if (current >= current_amps_min) {
                        Vector3f motor_impact_scaled = motor_impact / current;
                        motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled * 0.01f;
                        updated = true;
                    }
                }

                // calculate interference percentage at full throttle as % of total mag field
                if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                    // interference is impact@fullthrottle / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() / (float)arming.compass_magfield_expected() * 100.0f;
                } else {
                    // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() * (current_amps_max/throttle_pct_max) / (float)arming.compass_magfield_expected() * 100.0f;
                }
            }
        }

        if (AP_HAL::millis() - last_send_time > 500) {
            last_send_time = AP_HAL::millis();
            mavlink_msg_compassmot_status_send(gcs_chan.get_chan(),
                                               channel_throttle->get_control_in(),
                                               current,
                                               interference_pct[0],
                                               motor_compensation[0].x,
                                               motor_compensation[0].y,
                                               motor_compensation[0].z);
#if HAL_WITH_ESC_TELEM
            // send ESC telemetry to monitor ESC and motor temperatures
            AP::esc_telem().send_esc_telemetry_mavlink(gcs_chan.get_chan());
#endif
        }
    }

    // stop motors
    motors->output_min();
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // set and save motor compensation
    if (comp_type == AP_COMPASS_MOT_COMP_PER_MOTOR && !motor_mask) {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibration successful");
    } else if (comp_type != AP_COMPASS_MOT_COMP_PER_MOTOR && updated) {
        compass.motor_compensation_type(comp_type);
        for (uint8_t i=0; i<compass.get_count(); i++) {
            compass.set_motor_compensation(i, motor_compensation[i]);
        }
        compass.save_motor_compensation();
        // display success message
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibration successful");
    } else {
        // compensation vector never updated, report failure
        gcs_chan.send_text(MAV_SEVERITY_NOTICE, "Failed");
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    // re-enable cpu failsafe
    failsafe_enable();

    // re-enable failsafes
    g.failsafe_throttle.load();

    // flag we have completed
    ap.compass_mot = false;

    return MAV_RESULT_ACCEPTED;
#endif  // FRAME_CONFIG != HELI_FRAME
}
