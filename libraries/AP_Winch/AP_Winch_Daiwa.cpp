#include <AP_Winch/AP_Winch_Daiwa.h>

#if AP_WINCH_DAIWA_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#define AP_WINCH_DAIWA_STUCK_TIMEOUT_MS 1000    // winch is considered stuck if unmoving for this many milliseconds
#define AP_WINCH_DAIWA_STUCK_CENTER_MS  1000    // stuck protection outputs zero rate for this many milliseconds
#define AP_WINCH_DAIWA_STUCK_LENGTH_MIN 0.1     // stuck protection active when line length is more than this many meters
#define AP_WINCH_DAIWA_STUCK_RATE_MIN   0.2     // stuck protection active when desired rate is at least this value (+ve or -ve)

extern const AP_HAL::HAL& hal;

const char* AP_Winch_Daiwa::send_text_prefix = "Winch:";

// true if winch is healthy
bool AP_Winch_Daiwa::healthy() const
{
    // healthy if we have received data within 3 seconds
    return (AP_HAL::millis() - data_update_ms < 3000);
}

void AP_Winch_Daiwa::init()
{
    // call superclass init
    AP_Winch_Backend::init();

    // initialise serial connection to winch
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Winch, 0);
}

void AP_Winch_Daiwa::update()
{
    // return immediately if no servo is assigned to control the winch
    if (!SRV_Channels::function_assigned(SRV_Channel::k_winch)) {
        return;
    }

    // read latest data from winch
    read_data_from_winch();

    // read pilot input
    read_pilot_desired_rate();

    // send outputs to winch
    control_winch();

    // update_user
    update_user();
}

//send generator status
void AP_Winch_Daiwa::send_status(const GCS_MAVLINK &channel)
{
    // prepare status bitmask
    uint32_t status_bitmask = 0;
    if (healthy()) {
        status_bitmask |= MAV_WINCH_STATUS_HEALTHY;
    }
    if (latest.thread_end) {
        status_bitmask |= MAV_WINCH_STATUS_FULLY_RETRACTED;
    }
    if (latest.moving > 0) {
        status_bitmask |= MAV_WINCH_STATUS_MOVING;
    }
    if (latest.clutch > 0) {
        status_bitmask |= MAV_WINCH_STATUS_CLUTCH_ENGAGED;
    }

    // convert speed percentage to absolute speed
    const float speed_ms = fabsf(config.rate_max) * (float)latest.speed_pct * 0.01f;

    // send status
    mavlink_msg_winch_status_send(
        channel.get_chan(),
        AP_HAL::micros64(),
        latest.line_length,
        speed_ms,
        (float)latest.tension_corrected * 0.01f,
        latest.voltage,
        latest.current,
        latest.motor_temp,
        status_bitmask);
}

#if HAL_LOGGING_ENABLED
// write log
void AP_Winch_Daiwa::write_log()
{
    AP::logger().Write_Winch(
            healthy(),
            latest.thread_end,
            latest.moving,
            latest.clutch,
            (uint8_t)config.control_mode,
            config.length_desired,
            get_current_length(),
            config.rate_desired,
            latest.tension_corrected,
            latest.voltage,
            constrain_int16(latest.motor_temp, INT8_MIN, INT8_MAX));
}
#endif

// read incoming data from winch and update intermediate and latest structures
void AP_Winch_Daiwa::read_data_from_winch()
{
    // return immediately if serial port is not configured
    if (uart == nullptr) {
        return;
    }

    // read any available characters from serial port and send to GCS
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t b = uart->read();

        if (ishexa(b)) {
            // add digits to buffer
            buff[buff_len] = b;
            buff_len++;
            if (buff_len >= ARRAY_SIZE(buff)) {
                buff_len = 0;
                parse_state = ParseState::WAITING_FOR_TIME;
            }
        } else if (b == ',' || b == '\r') {
            // comma or carriage return signals end of current value
            // parse number received and empty buffer
            buff[buff_len] = '\0';
            long int value = (int32_t)strtol(buff, nullptr, 16);
            buff_len = 0;
            switch (parse_state) {
            case ParseState::WAITING_FOR_TIME:
                intermediate.time_ms = (uint32_t)value;
                parse_state = ParseState::WAITING_FOR_SPOOL;
                break;
            case ParseState::WAITING_FOR_SPOOL:
                intermediate.line_length = (int32_t)value * line_length_correction_factor;
                parse_state = ParseState::WAITING_FOR_TENSION1;
                break;
            case ParseState::WAITING_FOR_TENSION1:
                intermediate.tension_uncorrected = (uint16_t)value;
                parse_state = ParseState::WAITING_FOR_TENSION2;
                break;
            case ParseState::WAITING_FOR_TENSION2:
                intermediate.tension_corrected = (uint16_t)value;
                parse_state = ParseState::WAITING_FOR_THREAD_END;
                break;
            case ParseState::WAITING_FOR_THREAD_END:
                intermediate.thread_end = (value > 0);
                parse_state = ParseState::WAITING_FOR_MOVING;
                break;
            case ParseState::WAITING_FOR_MOVING:
                intermediate.moving = constrain_int32(value, 0, UINT8_MAX);
                parse_state = ParseState::WAITING_FOR_CLUTCH;
                break;
            case ParseState::WAITING_FOR_CLUTCH:
                intermediate.clutch = constrain_int32(value, 0, UINT8_MAX);
                parse_state = ParseState::WAITING_FOR_SPEED;
                break;
            case ParseState::WAITING_FOR_SPEED:
                intermediate.speed_pct = constrain_int32(value, 0, UINT8_MAX);
                parse_state = ParseState::WAITING_FOR_VOLTAGE;
                break;
            case ParseState::WAITING_FOR_VOLTAGE:
                intermediate.voltage = (float)value * 0.1f;
                parse_state = ParseState::WAITING_FOR_CURRENT;
                break;
            case ParseState::WAITING_FOR_CURRENT:
                intermediate.current = (float)value * 0.1f;
                parse_state = ParseState::WAITING_FOR_PCB_TEMP;
                break;
            case ParseState::WAITING_FOR_PCB_TEMP:
                intermediate.pcb_temp = (float)value * 0.1f;
                parse_state = ParseState::WAITING_FOR_MOTOR_TEMP;
                break;
            case ParseState::WAITING_FOR_MOTOR_TEMP:
                intermediate.motor_temp = (float)value * 0.1f;
                // successfully parsed a complete message
                latest = intermediate;
                data_update_ms = AP_HAL::millis();
                parse_state = ParseState::WAITING_FOR_TIME;
                break;
            }
        } else {
            // line feed or unexpected characters
            buff_len = 0;
            parse_state = ParseState::WAITING_FOR_TIME;
        }
    }
}

// update pwm outputs to control winch
void AP_Winch_Daiwa::control_winch()
{
    const uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - control_update_ms) * 0.001f;
    if (dt > 1.0f) {
        dt = 0.0f;
    }
    control_update_ms = now_ms;

    // if real doing any control output trim value
    if (config.control_mode == AP_Winch::ControlMode::RELAXED) {
        // if not doing any control output release clutch and move winch to trim
        SRV_Channels::set_output_limit(SRV_Channel::k_winch_clutch, SRV_Channel::Limit::MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_winch, 0);

        // rate used for acceleration limiting reset to zero
        set_previous_rate(0.0f);
        return;
    }

    // release clutch
    SRV_Channels::set_output_limit(SRV_Channel::k_winch_clutch, SRV_Channel::Limit::MIN);

    // if doing position control, calculate position error to desired rate
    if ((config.control_mode == AP_Winch::ControlMode::POSITION) && healthy()) {
        float position_error = config.length_desired - latest.line_length;
        config.rate_desired = constrain_float(position_error * config.pos_p, -config.rate_max, config.rate_max);
    }

    // apply acceleration limited to rate
    float rate_limited = get_rate_limited_by_accel(config.rate_desired, dt);

    // apply stuck protection to rate
    rate_limited = get_stuck_protected_rate(now_ms, rate_limited);

    // use linear interpolation to calculate output to move winch at desired rate
    float scaled_output = 0;
    if (!is_zero(rate_limited)) {
        scaled_output = linear_interpolate(output_dz, 1000, fabsf(rate_limited), 0, config.rate_max) * (is_positive(rate_limited) ? 1.0f : -1.0f);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_winch, scaled_output);
}

// returns the rate which may be modified to unstick the winch
// if the winch stops, the rate is temporarily set to zero
// now_ms should be set to the current system time
// rate should be the rate used to calculate the final PWM output to the winch
float AP_Winch_Daiwa::get_stuck_protected_rate(uint32_t now_ms, float rate)
{
    // exit immediately if stuck protection disabled
    if (!option_enabled(AP_Winch::Options::RetryIfStuck)) {
        return rate;
    }

    // check for timeout
    bool timeout = (now_ms - stuck_protection.last_update_ms) > 1000;
    stuck_protection.last_update_ms = now_ms;

    // check if winch is nearly fully pulled in
    const bool near_thread_start = (latest.line_length < AP_WINCH_DAIWA_STUCK_LENGTH_MIN) && is_negative(rate);

    // check if rate is near zero (winch may not move with very low desired rates)
    const bool rate_near_zero = fabsf(rate) < AP_WINCH_DAIWA_STUCK_RATE_MIN;

    // return rate unchanged if this protection has not been called recently or winch is unhealthy
    // or if winch is moving, desired rate is near zero or winch has stopped at thread start or thread end
    if (timeout || !healthy() || latest.moving || rate_near_zero || near_thread_start || latest.thread_end) {
        // notify user when winch becomes unstuck
        if (option_enabled(AP_Winch::Options::VerboseOutput) && (stuck_protection.stuck_start_ms != 0) && (stuck_protection.user_notified)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s unstuck", send_text_prefix);
        }
        // reset stuck protection state
        stuck_protection.stuck_start_ms = 0;
        return rate;
    }

    // winch is healthy, with non-zero requested rate but not moving
    // record when winch became stuck
    if (stuck_protection.stuck_start_ms == 0) {
        stuck_protection.stuck_start_ms = now_ms;
        stuck_protection.user_notified = false;
    }

    // if stuck for between 1 to 2 seconds return zero rate
    const uint32_t stuck_time_ms = (now_ms - stuck_protection.stuck_start_ms);
    if (stuck_time_ms > AP_WINCH_DAIWA_STUCK_TIMEOUT_MS) {
        // notify user
        if (!stuck_protection.user_notified) {
            stuck_protection.user_notified = true;
            if (option_enabled(AP_Winch::Options::VerboseOutput)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s stuck", send_text_prefix);
            }
        }

        // return zero rate for 1 second
        if (stuck_time_ms <= (AP_WINCH_DAIWA_STUCK_TIMEOUT_MS+AP_WINCH_DAIWA_STUCK_CENTER_MS)) {
            return 0;
        }

        // rate has been set to zero for 1 sec so release and restart stuck detection
        stuck_protection.stuck_start_ms = 0;

        // rate used for acceleration limiting also reset to zero
        set_previous_rate(0.0f);

        // update user
        if (option_enabled(AP_Winch::Options::VerboseOutput)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s retrying", send_text_prefix);
        }
    }

    // give winch more time to start moving
    return rate;
}

// update user with changes to winch state via send text messages
void AP_Winch_Daiwa::update_user()
{
    // exit immediately if verbose output disabled
    if (!option_enabled(AP_Winch::Options::VerboseOutput)) {
        return;
    }

    // send updates at no more than 2hz
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - user_update.last_ms < 500) {
        return;
    }
    bool update_sent = false;

    // health change
    const bool now_healthy = healthy();
    if (user_update.healthy != now_healthy) {
        user_update.healthy = now_healthy;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %shealthy", send_text_prefix, now_healthy ? "" : "not ");
        update_sent = true;
    }

    // thread end
    if (latest.thread_end && (user_update.thread_end != latest.thread_end)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s thread end", send_text_prefix);
        update_sent = true;
    }
    user_update.thread_end = latest.thread_end;

    // moving state
    if (user_update.moving != latest.moving) {
        // 0:stopped, 1:retracting line, 2:extending line, 3:clutch engaged, 4:zero reset
        static const char* moving_str[] = {"stopped", "raising", "lowering", "free spinning", "zero reset"};
        if (latest.moving < ARRAY_SIZE(moving_str)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s", send_text_prefix, moving_str[latest.moving]);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s move state unknown", send_text_prefix);
        }
        update_sent = true;
    }
    user_update.moving = latest.moving;

    // clutch state
    if (user_update.clutch != latest.clutch) {
        // 0:clutch off, 1:clutch engaged weakly, 2:clutch engaged strongly, motor can spin freely
        static const char* clutch_str[] = {"off", "weak", "strong (free)"};
        if (user_update.clutch < ARRAY_SIZE(clutch_str)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s clutch %s", send_text_prefix, clutch_str[latest.moving]);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s clutch state unknown", send_text_prefix);
        }
        update_sent = true;
    }
    user_update.clutch = latest.clutch;

    // length in meters
    const float latest_line_length_rounded = roundf(latest.line_length);
    if (!is_equal(user_update.line_length, latest_line_length_rounded)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %dm", send_text_prefix, (int)latest_line_length_rounded);
        update_sent = true;
    }
    user_update.line_length = latest_line_length_rounded;

    // record time message last sent to user
    if (update_sent) {
        user_update.last_ms = now_ms;
    }
}

#endif  // AP_WINCH_DAIWA_ENABLED
