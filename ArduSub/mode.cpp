#include "Sub.h"

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(sub.g),
    g2(sub.g2),
    inertial_nav(sub.inertial_nav),
    ahrs(sub.ahrs),
    motors(sub.motors),
    channel_roll(sub.channel_roll),
    channel_pitch(sub.channel_pitch),
    channel_throttle(sub.channel_throttle),
    channel_yaw(sub.channel_yaw),
    channel_forward(sub.channel_forward),
    channel_lateral(sub.channel_lateral),
    position_control(&sub.pos_control),
    attitude_control(&sub.attitude_control),
    G_Dt(sub.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Sub::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::ALT_HOLD:
        ret = &mode_althold;
        break;
    case Mode::Number::SURFTRAK:
        ret = &mode_surftrak;
        break;
    case Mode::Number::POSHOLD:
        ret = &mode_poshold;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::SURFACE:
        ret = &mode_surface;
        break;
    case Mode::Number::MOTOR_DETECT:
        ret = &mode_motordetect;
        break;
    default:
        break;
    }

    return ret;
}


// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// Some modes can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Sub::set_mode(Mode::Number mode, ModeReason reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num((Mode::Number)mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    if (new_flightmode->requires_GPS() &&
        !sub.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!sub.control_check_barometer() && // maybe use ekf_alt_ok() instead?
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(false)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode
    prev_control_mode = control_mode;

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
#if HAL_LOGGING_ENABLED
    logger.Write_Mode((uint8_t)control_mode, reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);

    // update notify object
    notify_flight_mode();

    // return success
    return true;
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(Mode::Number old_control_mode, Mode::Number new_control_mode)
{
    // stop mission when we leave auto mode
    if (old_control_mode == Mode::Number::AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    }
}

bool Sub::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return sub.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Sub::update_flight_mode()
{
    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(Mode *&old_flightmode, Mode *&new_flightmode){
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Sub::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}


// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
void Mode::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo.set(1.0f);
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the vehicle back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle > sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-sub.aparm.angle_max);
            } else if (roll_angle < -sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+sub.aparm.angle_max);
            }

            if (pitch_angle > sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-sub.aparm.angle_max);
            } else if (pitch_angle < -sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+sub.aparm.angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        } else {
            float acro_level_mix = constrain_float(1-MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.x)-fabsf(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.y)-fabsf(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.z)-fabsf(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}


bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return sub.set_mode(mode, reason);
}

GCS_Sub &Mode::gcs()
{
    return sub.gcs();
}
