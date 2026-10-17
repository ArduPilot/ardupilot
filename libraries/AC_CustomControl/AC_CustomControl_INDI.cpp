#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

// Tikhonov term of the inversion relative to the largest singular value (section 9)
static constexpr float INVERSION_TIKHONOV = 1.0e-3f;
// identification increment window (section 4, n_w = 10 ms)
static constexpr float IDENT_WINDOW_S = 0.010f;
// the motor-speed feedback handles the quad-X frame only, as in PR 21699:
// four motors whose normalised roll and pitch factors are +-0.5 (see
// AP_MotorsMatrix::normalise_rpy_factors) in the quad-X numbering
static constexpr uint8_t RPM_NUM_MOTORS = 4;
static constexpr float RPM_FRAME_FACTOR = 0.5f;
// a factor counts as matching within this much (normalised factors are
// exactly +-0.5 on quad-X; anything else differs by at least 0.1)
static constexpr float RPM_FRAME_TOLERANCE = 0.01f;

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_INDI::var_info[] = {
    // @Param: G_RP
    // @DisplayName: INDI roll and pitch effectiveness prior
    // @Description: Angular acceleration expected per unit of roll or pitch command before anything is learned. Class figures: about 400 for multirotors under 1 kg, 120 for 1 to 15 kg. Only the band 0.25x to 4x of this value can be identified.
    // @Units: rad/s/s
    // @Range: 20 2000
    // @User: Advanced
    AP_GROUPINFO("G_RP", 1, AC_CustomControl_INDI, _g_prior_rp, 400.0f),

    // @Param: G_YAW
    // @DisplayName: INDI yaw effectiveness prior
    // @Description: Angular acceleration expected per unit of yaw command before anything is learned, typically a tenth to a twentieth of the roll figure
    // @Units: rad/s/s
    // @Range: 2 500
    // @User: Advanced
    AP_GROUPINFO("G_YAW", 2, AC_CustomControl_INDI, _g_prior_yaw, 25.0f),

    // @Param: G_SIGMA
    // @DisplayName: INDI prior relative uncertainty
    // @Description: How far the true effectiveness is expected to be from the prior, as a fraction. Sets the initial covariance and so how aggressively the first strokes update the estimate.
    // @Range: 0.1 1.5
    // @User: Advanced
    AP_GROUPINFO("G_SIGMA", 3, AC_CustomControl_INDI, _g_rel_sigma, 0.7f),

    // @Param: RAT_GAIN
    // @DisplayName: INDI rate loop gain
    // @Description: Desired angular acceleration per unit of rate error. Vehicle independent once the effectiveness is identified.
    // @Units: 1/s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("RAT_GAIN", 4, AC_CustomControl_INDI, _rate_gain, 20.0f),

    // @Param: ANG_BW
    // @DisplayName: INDI attitude loop bandwidth
    // @Description: Nominal attitude loop P gain, desired rate per unit of attitude error. Over-driven by up to ODRV_MAX while the effectiveness is uncertain.
    // @Units: 1/s
    // @Range: 2 15
    // @User: Advanced
    AP_GROUPINFO("ANG_BW", 5, AC_CustomControl_INDI, _ang_bw, 6.0f),

    // @Param: ODRV_MAX
    // @DisplayName: INDI launch over-drive
    // @Description: Maximum multiplier on the attitude loop bandwidth while the effectiveness is uncertain, producing larger strokes for faster identification
    // @Range: 1 4
    // @User: Advanced
    AP_GROUPINFO("ODRV_MAX", 6, AC_CustomControl_INDI, _overdrive_max, 2.5f),

    // @Param: ACT_TAU
    // @DisplayName: INDI actuator lag time constant
    // @Description: First-order lag of the motors in response to a command change
    // @Units: s
    // @Range: 0 0.1
    // @User: Advanced
    AP_GROUPINFO("ACT_TAU", 7, AC_CustomControl_INDI, _act_tau, 0.010f),

    // @Param: ACT_DLY
    // @DisplayName: INDI actuator and sensor delay
    // @Description: Pure delay from a command change to its effect appearing in the filtered gyro, including ESC latency and the gyro filter delay
    // @Units: s
    // @Range: 0 0.05
    // @User: Advanced
    AP_GROUPINFO("ACT_DLY", 8, AC_CustomControl_INDI, _act_delay, 0.005f),

    // @Param: DU_MIN
    // @DisplayName: INDI minimum stroke
    // @Description: Smallest change in normalised command that is used for identification, and the amplitude of excitation pulses
    // @Range: 0.01 0.2
    // @User: Advanced
    AP_GROUPINFO("DU_MIN", 9, AC_CustomControl_INDI, _du_min, 0.05f),

    // @Param: PULSE
    // @DisplayName: INDI excitation pulses
    // @Description: Allow short command pulses of DU_MIN amplitude on an axis that the flight has not excited enough to identify. Pulses are never sent while on the ground.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PULSE", 10, AC_CustomControl_INDI, _pulses, 1),

    // @Param: PULSE_ALT
    // @DisplayName: INDI pulse minimum height
    // @Description: Height above home below which no excitation pulse is sent even once airborne. 0 allows pulses from liftoff, which a vertical takeoff needs if the axes are to be identified in the first few hundred milliseconds.
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("PULSE_ALT", 11, AC_CustomControl_INDI, _pulse_alt, 0.0f),

    // @Param: ACT_SRC
    // @DisplayName: INDI actuator state source
    // @Description: Where the actuator state the incremental law builds on comes from. 0 models it from the command with ACT_DLY and ACT_TAU. 1 measures it from motor speed (the RPM feedback of the earlier INDI custom controller, ArduPilot pull request 21699) when every motor reports RPM through ESC telemetry and the frame is quad X; otherwise, and while a measurement is first checked against the model, the model is used. Needs RPM at the loop rate, as bidirectional DShot gives it; serial ESC telemetry is too slow.
    // @Values: 0:Modelled,1:Motor speed when available
    // @User: Advanced
    AP_GROUPINFO("ACT_SRC", 12, AC_CustomControl_INDI, _act_src, 0),

    // @Param: THR2RTRSPD
    // @DisplayName: INDI motor speed at full throttle
    // @Description: Motor speed at full throttle, used with ACT_SRC 1 to turn measured motor speed into a fraction of full thrust (thrust taken as proportional to the square of speed). As THR2RTRSPD in ArduPilot pull request 21699, whose default it keeps. An error here scales the measured actuator state, which the identified effectiveness then absorbs.
    // @Units: rad/s
    // @Range: 100 10000
    // @User: Advanced
    AP_GROUPINFO("THR2RTRSPD", 13, AC_CustomControl_INDI, _throttle2motor_speed, 3251.0f),

    AP_GROUPEND
};

AC_CustomControl_INDI::AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
    init_indi();
}

void AC_CustomControl_INDI::init_indi()
{
    AP_INDI::Params p;
    p.g_prior[0] = _g_prior_rp;
    p.g_prior[1] = _g_prior_rp;
    p.g_prior[2] = _g_prior_yaw;
    p.prior_rel_sigma = _g_rel_sigma;
    p.rate_gain = _rate_gain;
    p.overdrive_max = _overdrive_max;
    p.act_tau_s = _act_tau;
    p.act_delay_s = _act_delay;
    p.sensor_filt_hz = AP::ins().get_gyro_filter_hz();
    p.du_min = _du_min;
    p.window_s = IDENT_WINDOW_S;
    p.tikhonov = INVERSION_TIKHONOV;
    p.pulses_enabled = _pulses != 0;
    _indi.init(_dt, p);
}

Vector3f AC_CustomControl_INDI::update()
{
    bool flying = false;
    switch (_motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // on the ground: start the next flight from the prior
        reset();
        return Vector3f();
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // throttle pinned at its minimum means the vehicle is sitting on
        // the ground after touchdown (or the pilot has cut it): nothing
        // measured there says anything about the airframe
        flying = !_motors->limit.throttle_lower;
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    // attitude error and feedforward rate from the stock attitude target,
    // as the PID backend does
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
    Vector3f attitude_error;
    float thrust_angle_rad, thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad, thrust_error_angle_rad);
    const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    AP_INDI::Inputs in;
    for (uint8_t i = 0; i < AP_INDI::NUM_AXES; i++) {
        // fixed nominal bandwidth, over-driven while uncertain
        in.rate_target[i] = _ang_bw * _indi.get_att_gain_scale(i) * attitude_error[i] + ang_vel_body_feedforward[i];
    }
    in.gyro = _ahrs->get_gyro_latest();
    // the main rate controller has already run this loop; on axes handed
    // back to it these are the commands the motors will see
    in.u_applied = Vector3f(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
    in.saturated[0] = _motors->limit.roll;
    in.saturated[1] = _motors->limit.pitch;
    in.saturated[2] = _motors->limit.yaw;
    // health of the gyro the attitude estimate actually uses
    in.gyro_healthy = AP::ins().get_gyro_health(AP::ahrs().get_primary_gyro_index());
    in.flying = flying;

    // may be airborne: the landing detector says so. It clears at takeoff
    // intent, before the vehicle actually leaves, so the controller
    // itself decides launch from the first stroke that gets a response.
    const AP_Vehicle *vehicle = AP::vehicle();
    const bool likely_flying = vehicle != nullptr && vehicle->get_likely_flying();
    in.launched = flying && likely_flying;
    in.ident_allowed = in.launched;

    Vector3f pos_ned;
    float alt_m = 0;
    if (_ahrs->get_relative_position_NED_home(pos_ned)) {
        alt_m = -pos_ned.z;
    }
    in.pulses_allowed = in.launched && alt_m >= _pulse_alt;

    if (_act_src == 1) {
        in.u_measured_valid = motor_speed_actuator_state(in.u_measured);
    }

    _indi.update(in);
    return _indi.get_output();
}

/*
  Motor-speed feedback. get_motor_speed() and the quad-X allocation below
  come from AC_CustomControl_INDI::get_motor_speed() and
  calculate_torque_thrust_est() in ArduPilot/ardupilot pull request 21699
  by Emre Saldiran (2022, rebased by Michelle Rossouw in 2025), itself
  from AC_INDI_Control in pull request 20511. Changes from that code:
  - each motor's RPM is read with AP_ESC_Telem::get_rpm() on its own
    output channel: get_motor_frequencies_hz() now takes a channel mask
    and returns only the ESCs that reported, packed, so indexing its
    result by channel reads the wrong motor once one ESC is missing;
  - the SITL block that overwrote the telemetry with zeros is gone, as
    SITL now supplies RPM through ESC telemetry (SIM_ESC_TELEM, with
    SIM_VIB_MOT_MAX setting the speed at full command);
  - #ifdef HAL_WITH_ESC_TELEM, always true since the macro is always
    defined, is #if;
  - the torque estimate is not formed or filtered. PR 21699 computed the
    torque from speed squared with arm length l, thrust coefficient k_T
    and torque ratio k, and its scale_torque_cmd() took a torque back to
    roll, pitch and yaw commands; composed, l, k_T and k cancel and the
    command is half the allocation row times the squared speed fraction.
    The 0.1 factor scale_torque_cmd() applied to yaw belonged to its
    torque command path and is not part of the measurement. Its first-
    order filter (TRQEST_FILT) is replaced by AP_INDI's own filter on the
    actuator state, matched to the gyro filter
*/

// get current rotation speed of each motor in rad/s
bool AC_CustomControl_INDI::get_motor_speed(void)
{
    float motor_speed_hz[4] = {0.0f, 0.0f, 0.0f, 0.0f};

#if HAL_WITH_ESC_TELEM
    // map esc servo order to quad x frame order
    float temp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i=0; i < 4; i++) {
        uint8_t chan;
        SRV_Channel::Function function = SRV_Channels::get_motor_function(i);
        float rpm;
        if (!SRV_Channels::find_channel(function, chan) || !AP::esc_telem().get_rpm(chan, rpm)) {
            return false;
        }
        temp[i] = rpm * (1.0f / 60.0f);
    }

    // change quad x frame order so that motor order starts at the front
    // right motor and goes front right, back right, back left, front left
    // (PR 21699's comment read counter clockwise; the order is clockwise
    // seen from above, which its allocation matrix assumes)
    motor_speed_hz[0] = temp[0];
    motor_speed_hz[1] = temp[3];
    motor_speed_hz[2] = temp[1];
    motor_speed_hz[3] = temp[2];
#else
    return false;
#endif
    for (uint8_t i=0; i < 4; i++) {
        _motor_speed_meas_radps[i] = motor_speed_hz[i] * M_2PI;
    }
    return true;
}

bool AC_CustomControl_INDI::motor_speed_actuator_state(Vector3f &u_measured)
{
    // quad X only, as in PR 21699: check the mixer's factors against the
    // layout its allocation matrix assumes, motors 1 to 4 being front
    // right, back left, front left, back right, and no fifth motor
    static const float roll_sign[RPM_NUM_MOTORS] = { -1, 1, 1, -1 };
    static const float pitch_sign[RPM_NUM_MOTORS] = { 1, -1, 1, -1 };
    bool quad_x = is_zero(_motors->get_roll_factor(RPM_NUM_MOTORS)) && is_zero(_motors->get_pitch_factor(RPM_NUM_MOTORS));
    for (uint8_t i = 0; i < RPM_NUM_MOTORS; i++) {
        quad_x &= fabsf(_motors->get_roll_factor(i) - roll_sign[i] * RPM_FRAME_FACTOR) < RPM_FRAME_TOLERANCE;
        quad_x &= fabsf(_motors->get_pitch_factor(i) - pitch_sign[i] * RPM_FRAME_FACTOR) < RPM_FRAME_TOLERANCE;
    }
    if (!quad_x) {
        if (!_act_src_warned) {
            _act_src_warned = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INDI: motor speed feedback needs quad X, using model");
        }
        return false;
    }
    if (!get_motor_speed() || !is_positive(_throttle2motor_speed.get())) {
        return false;
    }

    // control allocation matrix for quad-x frame type, PR 21699's rows
    // for roll, pitch and yaw with l and k factored out, motor order front
    // right, back right, back left, front left
    static const float control_alloc_G1[3][4] = {
        {-1.0f, -1.0f,  1.0f,  1.0f},
        { 1.0f, -1.0f, -1.0f,  1.0f},
        { 1.0f, -1.0f,  1.0f, -1.0f}};

    // squared speed as a fraction of full speed: the thrust fraction, and
    // the mixer's per-motor output, for thrust proportional to speed squared
    const float inv_full_sq = 1.0f / sq(_throttle2motor_speed.get());
    float motor_speed_meas_sq[4];
    for (uint8_t i=0; i < 4; i++) {
        motor_speed_meas_sq[i] = sq(_motor_speed_meas_radps[i]) * inv_full_sq;
    }

    // each motor carries +-RPM_FRAME_FACTOR of the roll, pitch and yaw
    // command in the mixer, so the command is that factor times the
    // signed sum of the thrust fractions (PR 21699's torque estimate
    // times its scale_torque_cmd() factors, 2 / (4 k_T l w_max^2) for roll
    // and pitch and 2 / (4 k_T k w_max^2) for yaw, gives the same)
    float cmd[3] = {0.0f, 0.0f, 0.0f};
    for (uint8_t i=0; i < 3; i++) {
        for (uint8_t j=0; j < 4; j++) {
            cmd[i] += control_alloc_G1[i][j] * motor_speed_meas_sq[j];
        }
        cmd[i] *= RPM_FRAME_FACTOR;
    }
    u_measured = Vector3f(cmd[0], cmd[1], cmd[2]);
    return true;
}

// restart from the prior, taking the current motor commands as the
// starting actuator state so the first increment is small
void AC_CustomControl_INDI::reset(void)
{
    init_indi();
    _indi.reset(Vector3f(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw()));
}

#endif // AP_COPTER_CUSTOMCONTROL_INDI_ENABLED
