// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    // reset glitch protection to use new baro alt
    baro_glitch.reset();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    barometer.read();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    // run glitch protection and update AP_Notify if home has been initialised
    baro_glitch.check_alt();
    bool report_baro_glitch = (baro_glitch.glitching() && !ap.usb_connected && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    if (AP_Notify::flags.baro_glitching != report_baro_glitch) {
        if (baro_glitch.glitching()) {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_BARO_GLITCH);
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_ERROR_RESOLVED);
        }
        AP_Notify::flags.baro_glitching = report_baro_glitch;
    }
}

#if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
   sonar.init();
}
#endif

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    // exit immediately if sonar is disabled
    if (!sonar_enabled || !sonar.healthy()) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

// initialise compass
static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

// initialise optical flow sensor
static void init_optflow()
{
#if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // initialise sensor and display error on failure
    optflow.init();
    if (!optflow.healthy()) {
        cliSerial->print_P(PSTR("Failed to Init OptFlow\n"));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }
#endif      // OPTFLOW == ENABLED
}

// called at 100hz but data from sensor only arrives at 20 Hz
#if OPTFLOW == ENABLED
static void update_optflow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        if (should_log(MASK_LOG_OPTFLOW)) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

/* Voltage scaling of PIDs
 * How it works:
 * - Data from eCalc suggests that thrust is roughly linear in voltage
 * - We use the voltage from the monitoring pin, before it is multiplied by BATT_VOLT_MULT.
 *      This makes the scaling robust to parameter changes by users.
 * - We store the maximum voltage that we have seen since startup, and use the voltage
 *      relative to this value for scaling.
 * - We use the following 3 constants to configure the scaler range:
 *      - VSCALE_REFERENCE_VOLTAGE is set to the voltage we expect the battery to droop to
 *           under load. I've set this to 3.7, as 3.7 is the nominal voltage of a LIPO.
 *           This corresponds the the voltage at which the scaler is 1.0. If set equal
 *           to VSCALE_MAX_VOLTAGE, the scaler will start at 1.0 and go up from there.
 *      - VSCALE_MAX_VOLTAGE is set to the maximum no-load voltage of the battery
 *           This ratio of this value and VSCALE_REFERENCE_VOLTAGE is used to determine the
 *           minimum value of the scaler - the value that it will have when the copter is
 *           about to start flying
 *      - VSCALE_MIN_VOLTAGE is set to the typical "empty" voltage of the battery. This
 *           value is used to constrain the maximum value of the scaler.
 * - NOTE: a low-pass filter is used before saving the maximum voltage. It is not the
 * intention of the author to use this in the output. The PIDs do not care about noise
 * in the scaler. However, if we were to use this to scale THR_MID, we would want to
 * store a separate, filtered scaler for that purpose only.
 */
#define VSCALE_REFERENCE_VOLTAGE 3.7f
#define VSCALE_MAX_VOLTAGE 4.2f
#define VSCALE_MIN_VOLTAGE 3.0f

#define VSCALE_SCALER_MAX VSCALE_REFERENCE_VOLTAGE/VSCALE_MIN_VOLTAGE
#define VSCALE_SCALER_MIN VSCALE_REFERENCE_VOLTAGE/VSCALE_MAX_VOLTAGE
static void update_pid_scaling(void)
{
    float scaler = 1.0f;
    static uint32_t tlast = 0.0f;
    static float voltage_pin_lpf = 0.0f;
    static float voltage_pin_lpf_max = 0.0f;

    uint8_t monitoring = battery.monitoring();

    // leave scaler at 1.0f if we are not monitoring voltage
    if(g.pidvoltscale && (monitoring == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT || monitoring == AP_BATT_MONITOR_VOLTAGE_ONLY)) {
        // Get dt for filter
        uint32_t tnow = hal.scheduler->micros();
        float dt = (tnow - tlast) * 1.0E-6f;
        tlast = tnow;

        // Get pin voltage from battMonitor:
        float voltage_pin = battery.unscaled_pin_voltage();

        // Find and store maximum voltage since startup, reject noise.
        if(tlast != 0 && dt < 2.0f) {
            voltage_pin_lpf += (voltage_pin - voltage_pin_lpf) * dt * .2;
            voltage_pin_lpf_max = max(voltage_pin_lpf, voltage_pin_lpf_max);
        }

        // equal to VSCALE_SCALER_MIN when vmax/v = 1.0f, increasing for smaller values of v
        scaler = VSCALE_SCALER_MIN * voltage_pin_lpf_max / voltage_pin;

        // Sanity check:
        // isinf, isnan checks for zero values of voltage_pin_lpf_max and voltage_pin
        // scaler < VSCALE_SCALER_MIN constrains scaler
        // scaler > 2.0f*VSCALE_SCALER_MAX prevents problems if the wire on the voltage sensor is loose
        if(isinf(scaler) || isnan(scaler) || scaler < VSCALE_SCALER_MIN || scaler > 2.0f*VSCALE_SCALER_MAX) {
            scaler = VSCALE_SCALER_MIN;
        } else if(scaler > VSCALE_SCALER_MAX) {
            scaler = VSCALE_SCALER_MAX;
        }
    }
    attitude_control.set_pid_scaler(scaler);
    pos_control.set_pid_scaler(scaler);
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        compass.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
    update_pid_scaling();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

#if EPM_ENABLED == ENABLED
// epm update - moves epm pwm output back to neutral after grab or release is completed
void epm_update()
{
    epm.update();
}
#endif
