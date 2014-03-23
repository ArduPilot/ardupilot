// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  compass/motor interference calibration
 */

// setup_compassmot - sets compass's motor interference parameters
static uint8_t mavlink_compassmot(mavlink_channel_t chan)
{
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base;              // compass vector when throttle is zero
    Vector3f motor_impact;              // impact of motors on compass vector
    Vector3f motor_impact_scaled;       // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation;        // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct = 0.0f;   // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint32_t last_send_time;
    bool     updated = false;           // have we updated the compensation vector at least once
    uint8_t  command_ack_start = command_ack_counter;

    // exit immediately if we are already in compassmot
    if (ap.compass_mot) {
        // ignore restart messages
        return 1;
    }else{
        ap.compass_mot = true;
    }

    // check compass is enabled
    if (!g.compass_enabled) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("compass disabled\n"));
        ap.compass_mot = false;
        return 1;
    }

    // check compass health
    compass.read();
    if (!compass.healthy(0)) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("check compass"));
        ap.compass_mot = false;
        return 1;
    }

    // check if radio is calibrated
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("RC not calibrated"));
        ap.compass_mot = false;
        return 1;
    }

    // check throttle is at zero
    read_radio();
    if (g.rc_3.control_in != 0) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("thr not zero"));
        ap.compass_mot = false;
        return 1;
    }

    // check we are landed
    if (!ap.land_complete) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Not landed"));
        ap.compass_mot = false;
        return 1;
    }

    // disable cpu failsafe
    failsafe_disable();

    // initialise compass
    init_compass();

    // default compensation type to use current if possible
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // send back initial ACK
    mavlink_msg_command_ack_send(chan, MAV_CMD_PREFLIGHT_CALIBRATION,0);

    // flash leds
    AP_Notify::flags.esc_calibration = true;

    // warn user we are starting calibration
    gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("STARTING CALIBRATION"));

    // inform what type of compensation we are attempting
    if (comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("CURRENT"));
    } else{
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("THROTTLE"));
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = FS_BATT_DISABLED;
    g.failsafe_gps_enabled = FS_GPS_DISABLED;

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    compass.set_motor_compensation(Vector3f(0,0,0));

    // get initial compass readings
    last_run_time = millis();
    while ( millis() - last_run_time < 500 ) {
        compass.accumulate();
    }
    compass.read();

    // store initial x,y,z compass values
    compass_base = compass.get_field();

    // initialise motor compensation
    motor_compensation = Vector3f(0,0,0);

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();
    last_send_time = millis();

    // main run while there is no user input and the compass is healthy
    while (command_ack_start == command_ack_counter && compass.healthy(0) && motors.armed()) {
        // 50hz loop
        if (millis() - last_run_time < 20) {
            // grab some compass values
            compass.accumulate();
            hal.scheduler->delay(5);
            continue;
        }
        last_run_time = millis();

        // read radio input
        read_radio();
        
        // pass through throttle to motors
        motors.throttle_pass_through();
        
        // read some compass values
        compass.read();
        
        // read current
        read_battery();
        
        // calculate scaling for throttle
        throttle_pct = (float)g.rc_3.control_in / 1000.0f;
        throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);
        
        // if throttle is near zero, update base x,y,z values
        if (throttle_pct <= 0.0f) {
            compass_base = compass_base * 0.99f + compass.get_field() * 0.01f;
            
            // causing printing to happen as soon as throttle is lifted
        } else {

            // calculate diff from compass base and scale with throttle
            motor_impact = compass.get_field() - compass_base;
            
            // throttle based compensation
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                // scale by throttle
                motor_impact_scaled = motor_impact / throttle_pct;
                
                // adjust the motor compensation to negate the impact
                motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                updated = true;
            } else {
                // current based compensation if more than 3amps being drawn
                motor_impact_scaled = motor_impact / battery.current_amps();
                
                // adjust the motor compensation to negate the impact if drawing over 3amps
                if( battery.current_amps() >= 3.0f ) {
                    motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                }
            }

            // calculate interference percentage at full throttle as % of total mag field
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                // interference is impact@fullthrottle / mag field * 100
                interference_pct = motor_compensation.length() / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
            }else{
                // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
                interference_pct = motor_compensation.length() * (current_amps_max/throttle_pct_max) / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
            }

            // record maximum throttle and current
            throttle_pct_max = max(throttle_pct_max, throttle_pct);
            current_amps_max = max(current_amps_max, battery.current_amps());

        }
        if (hal.scheduler->millis() - last_send_time > 500) {
            last_send_time = hal.scheduler->millis();
            mavlink_msg_compassmot_status_send(chan, 
                                               g.rc_3.control_in,
                                               battery.current_amps(),
                                               interference_pct,
                                               motor_compensation.x,
                                               motor_compensation.y,
                                               motor_compensation.z);
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // set and save motor compensation
    if (updated) {
        compass.motor_compensation_type(comp_type);
        compass.set_motor_compensation(motor_compensation);
        compass.save_motor_compensation();
        // display success message
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Calibration Successful!"));
    } else {
        // compensation vector never updated, report failure
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Failed!"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    // re-enable cpu failsafe
    failsafe_enable();

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_battery_enabled.load();
    g.failsafe_gps_enabled.load();

    // flag we have completed
    ap.compass_mot = false;

    return 0;
}

