#include <AP_Winch/AP_Winch_Daiwa.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

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
    if (uart != nullptr) {
        // always use baudrate of 115200
        uart->begin(115200);
    }
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

        if ((b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f')) {
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
    const float rate_limited = get_rate_limited_by_accel(config.rate_desired, dt);

    // use linear interpolation to calculate output to move winch at desired rate
    float scaled_output = 0;
    if (!is_zero(rate_limited)) {
        scaled_output = linear_interpolate(output_dz, 1000, fabsf(rate_limited), 0, config.rate_max) * (is_positive(rate_limited) ? 1.0f : -1.0f);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_winch, scaled_output);
}
