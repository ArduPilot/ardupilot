#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_INDI_ENABLED

#include "AP_CustomControl_INDI.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

// below this fraction of the reference airspeed the surfaces are too weak
// to identify from, and the dynamic pressure scaling is floored there
static constexpr float IDENT_MIN_ASPD_FRAC = 0.5f;
// the loop rate the identification needs; below it the differentiator
// window outlasts the excitation pulse (SPEC.md, rates)
static constexpr float MIN_LOOP_RATE_HZ = 200.0f;
// identification starts this long after the gear has let go: the TAKEOFF
// rotation on the SITL plane (a pitch-up to the climb attitude and the
// loop's own pull-back) is over in about a second
static constexpr uint32_t LAUNCH_SETTLE_MS = 1000;
// the coordination offsets use the bank angle clamped as the stock
// controllers clamp it: beyond 80 degrees tan(bank) runs away
static constexpr float BANK_LIMIT_RAD = radians(80.0f);
// a fixed-wing counts as launched once it is this far above the home
// height: on the runway the gear holds roll and yaw, so a stroke there
// measures the ground and not the airframe (SITL plane: every ground-roll
// estimate landed on the lower band edge), and below 2 m the baro cannot
// tell a bounce from flight
static constexpr float LAUNCH_MIN_HEIGHT_M = 2.0f;
// Tikhonov term of the inversion relative to the largest singular value (section 9)
static constexpr float INVERSION_TIKHONOV = 1.0e-3f;
// identification increment window (section 4, n_w = 10 ms)
static constexpr float IDENT_WINDOW_S = 0.010f;
// full surface deflection in the scaled servo output units
static constexpr float SURFACE_FULL_SCALE = 4500.0f;

const AP_Param::GroupInfo AP_CustomControl_INDI::var_info[] = {
    // @Param: G_RLL
    // @DisplayName: INDI aileron effectiveness prior
    // @Description: Roll acceleration expected per unit of aileron command at ASPD_REF before anything is learned; scaled with the square of airspeed in flight. Default is the SITL plane model (2 kg foam, 1.88 m span) at 13 m/s: q S b Cl_da times the 0.8 of full deflection a 1100-1900 servo reaches. A real airframe must be measured; only 0.25x to 4x of this value can be identified.
    // @Units: rad/s/s
    // @Range: 10 1000
    // @User: Advanced
    AP_GROUPINFO("G_RLL", 1, AP_CustomControl_INDI, _g_roll, 18.0f),

    // @Param: G_PIT
    // @DisplayName: INDI elevator effectiveness prior
    // @Description: Pitch acceleration expected per unit of elevator command at ASPD_REF, see G_RLL. Default is the SITL plane model, q S c Cm_de
    // @Units: rad/s/s
    // @Range: 10 1000
    // @User: Advanced
    AP_GROUPINFO("G_PIT", 2, AP_CustomControl_INDI, _g_pitch, 9.0f),

    // @Param: G_YAW
    // @DisplayName: INDI rudder effectiveness prior
    // @Description: Yaw acceleration expected per unit of rudder command at ASPD_REF, see G_RLL. Default is the SITL plane model, q S b Cn_dr
    // @Units: rad/s/s
    // @Range: 2 300
    // @User: Advanced
    AP_GROUPINFO("G_YAW", 3, AP_CustomControl_INDI, _g_yaw, 7.0f),

    // @Param: ASPD_REF
    // @DisplayName: INDI reference airspeed
    // @Description: Airspeed at which the effectiveness priors apply
    // @Units: m/s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("ASPD_REF", 4, AP_CustomControl_INDI, _aspd_ref, 13.0f),

    // @Param: G_SIGMA
    // @DisplayName: INDI prior relative uncertainty
    // @Description: How far the true effectiveness is expected to be from the prior, as a fraction
    // @Range: 0.1 1.5
    // @User: Advanced
    AP_GROUPINFO("G_SIGMA", 5, AP_CustomControl_INDI, _g_rel_sigma, 1.0f),

    // @Param: RAT_GAIN
    // @DisplayName: INDI rate loop gain
    // @Description: Desired angular acceleration per unit of rate error
    // @Units: 1/s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("RAT_GAIN", 6, AP_CustomControl_INDI, _rate_gain, 15.0f),

    // @Param: ANG_BW
    // @DisplayName: INDI attitude loop bandwidth
    // @Description: Nominal attitude loop P gain, desired rate per unit of attitude error, over-driven by up to ODRV_MAX while the effectiveness is uncertain
    // @Units: 1/s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("ANG_BW", 7, AP_CustomControl_INDI, _ang_bw, 4.0f),

    // @Param: ODRV_MAX
    // @DisplayName: INDI launch over-drive
    // @Description: Maximum multiplier on the attitude loop bandwidth while the effectiveness is uncertain
    // @Range: 1 4
    // @User: Advanced
    AP_GROUPINFO("ODRV_MAX", 8, AP_CustomControl_INDI, _overdrive_max, 2.5f),

    // @Param: ACT_TAU
    // @DisplayName: INDI servo lag time constant
    // @Description: First-order lag of the control surfaces in response to a command change
    // @Units: s
    // @Range: 0 0.2
    // @User: Advanced
    AP_GROUPINFO("ACT_TAU", 9, AP_CustomControl_INDI, _act_tau, 0.040f),

    // @Param: ACT_DLY
    // @DisplayName: INDI servo and sensor delay
    // @Description: Pure delay from a command change to its effect appearing in the filtered gyro
    // @Units: s
    // @Range: 0 0.1
    // @User: Advanced
    AP_GROUPINFO("ACT_DLY", 10, AP_CustomControl_INDI, _act_delay, 0.010f),

    // @Param: DU_MIN
    // @DisplayName: INDI minimum stroke
    // @Description: Smallest change in normalised command used for identification, and the amplitude of excitation pulses
    // @Range: 0.01 0.2
    // @User: Advanced
    AP_GROUPINFO("DU_MIN", 11, AP_CustomControl_INDI, _du_min, 0.05f),

    // @Param: PULSE
    // @DisplayName: INDI excitation pulses
    // @Description: Allow short surface pulses of DU_MIN amplitude on an axis the flight has not excited enough to identify
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PULSE", 12, AP_CustomControl_INDI, _pulses, 1),

    // @Param: RAT_MAX
    // @DisplayName: INDI attitude loop rate limit
    // @Description: Largest roll or pitch rate the attitude loop asks of the rate loop. A large bank demand otherwise asks for whatever ANG_BW times the error gives, and a slow servo then overshoots: the SITL plane rolled through 90 degrees answering a 60 degree demand. The stock controllers limit the same way (RLL2SRV_RMAX, PTCH2SRV_RMAX_UP).
    // @Units: deg/s
    // @Range: 20 300
    // @User: Advanced
    AP_GROUPINFO("RAT_MAX", 13, AP_CustomControl_INDI, _rate_max, 60.0f),

    AP_GROUPEND
};

AP_CustomControl_INDI::AP_CustomControl_INDI(AP_CustomControl& frontend, float dt) :
    AP_CustomControl_Backend(frontend, dt)
{
    _q_scale = 1.0f;
    _was_flying = false;
    _airborne_ms = 0;
    _airborne_latched = false;
    AP_Param::setup_object_defaults(this, var_info);
    init_indi();
}

void AP_CustomControl_INDI::init_indi()
{
    AP_INDI::Params p;
    p.g_prior[0] = _g_roll;
    p.g_prior[1] = _g_pitch;
    p.g_prior[2] = _g_yaw;
    p.prior_rel_sigma = _g_rel_sigma;
    // with the lag compensated the rate loop is second order: omega_dd +
    // (1/tau - L) omega_d + (K/tau) omega = (K/tau) omega_target, where L is
    // the airframe's own rate damping (negative, -3 to -12 1/s on the SITL
    // plane at 27 m/s). The cap K = 2/tau keeps the damping above 0.35 with
    // no aerodynamic damping at all and above 0.5 with the plane's pitch
    // damping; the SITL plane's 0.14 s servo limit-cycled the elevator at
    // 15 1/s only while the loop ran at 50 Hz with a 60 ms differentiator
    p.rate_gain = MIN(_rate_gain.get(), 2.0f / MAX(_act_tau.get(), 0.001f));
    p.overdrive_max = _overdrive_max;
    p.act_tau_s = _act_tau;
    p.act_delay_s = _act_delay;
    p.sensor_filt_hz = AP::ins().get_gyro_filter_hz();
    p.du_min = _du_min;
    p.window_s = IDENT_WINDOW_S;
    p.tikhonov = INVERSION_TIKHONOV;
    p.pulses_enabled = _pulses != 0;
    _indi.init(_dt, p);
    // axes outside the mask stay with the stock controllers: on a plane
    // the rudder is a damper, its strokes in a turn are the coordination
    // offset and not probes, and its identification at launch is the
    // least trustworthy of the three
    for (uint8_t i = 0; i < AP_INDI::NUM_AXES; i++) {
        if (!(_frontend.get_mask() & (1U << i))) {
            _indi.disable_axis(i);
        }
    }
    // the identification is designed for 400 Hz (SPEC.md, rates): at the
    // plane's default 50 Hz the derivative window is 140 ms and a 30 ms
    // pulse spans two steps, and the SITL plane identified at a third of
    // its effectiveness. Say so once rather than fail quietly.
    if (_dt > 1.0f / MIN_LOOP_RATE_HZ && !_rate_warned) {
        _rate_warned = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INDI: loop %.0f Hz, needs %.0f (SCHED_LOOP_RATE)", double(1.0f / _dt), double(MIN_LOOP_RATE_HZ));
    }
}

void AP_CustomControl_INDI::update()
{
    const AP_AHRS &ahrs = AP::ahrs();
    const bool flying = _frontend.is_flying && !_frontend.ground_mode;
    if (!flying && _was_flying) {
        // back on the ground: the next flight starts from the prior
        reset();
    }
    _was_flying = flying;

    // dynamic pressure relative to the reference. The commands the loop
    // works on are the surface deflections scaled by this, so the
    // effectiveness it identifies is airspeed independent; the limit the
    // loop may command scales with it too.
    float aspd;
    if (!ahrs.airspeed_EAS(aspd)) {
        aspd = _aspd_ref;
    }
    const float ref = MAX(_aspd_ref.get(), 1.0f);
    const float aspd_floored = MAX(aspd, IDENT_MIN_ASPD_FRAC * ref);
    _q_scale = sq(aspd_floored / ref);
    _indi.set_command_limit(_q_scale);

    // fixed-gain attitude loop on the targets Plane provides, over-driven
    // while the effectiveness is uncertain. In a banked turn the body
    // pitch and yaw rates are not zero: the turn rate g*tan(bank)/V
    // projects onto them as the same coordination offsets the stock
    // AP_PitchController and AP_YawController apply. Without them the
    // yaw loop drives the rudder to its stop against every turn (full
    // rudder for the whole of a 57 degree loiter on the SITL plane) and
    // the pitch loop carries a steady error of several degrees.
    const float bank = constrain_float(ahrs.get_roll_rad(), -BANK_LIMIT_RAD, BANK_LIMIT_RAD);
    const float tas = MAX(aspd * ahrs.get_EAS2TAS(), IDENT_MIN_ASPD_FRAC * ref);
    const float lateral_accel = tanf(bank) * GRAVITY_MSS * cosf(ahrs.get_pitch_rad());
    const float pitch_coord = fabsf(lateral_accel / tas * sinf(bank));
    const float yaw_coord = GRAVITY_MSS / tas * sinf(bank);
    const float roll_err = radians(wrap_180(get_roll_target_deg() - ahrs.get_roll_deg()));
    const float pitch_err = radians(wrap_180(get_pitch_target_deg() - ahrs.get_pitch_deg()));
    AP_INDI::Inputs in;
    const float rate_max = radians(MAX(_rate_max.get(), 1.0f));
    in.rate_target.x = constrain_float(_ang_bw * _indi.get_att_gain_scale(0) * roll_err, -rate_max, rate_max);
    in.rate_target.y = constrain_float(_ang_bw * _indi.get_att_gain_scale(1) * pitch_err, -rate_max, rate_max) + pitch_coord;
    in.rate_target.z = yaw_coord;
    in.gyro = ahrs.get_gyro_latest();
    // the stock controllers have already written the surfaces this loop;
    // on axes handed back to them that is what the plant will see
    in.u_applied = Vector3f(SRV_Channels::get_output_scaled(SRV_Channel::k_aileron),
                            SRV_Channels::get_output_scaled(SRV_Channel::k_elevator),
                            SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)) * (_q_scale / SURFACE_FULL_SCALE);
    const Vector3f &last = _indi.get_output();
    for (uint8_t i = 0; i < AP_INDI::NUM_AXES; i++) {
        in.saturated[i] = fabsf(last[i]) >= 0.999f * _q_scale;
    }
    in.gyro_healthy = AP::ins().get_gyro_health(ahrs.get_primary_gyro_index());
    // off the ground: Plane's is_flying() is true for the whole takeoff
    // roll, so height above home decides when the gear has let go
    float pos_d = 0;
    ahrs.get_relative_position_D_home(pos_d);
    const bool airborne = flying && (-pos_d >= LAUNCH_MIN_HEIGHT_M);
    // ... and is held only before the first time it gets there: the
    // landing flare passes back through the same 2 m, and treating the
    // vehicle as held there dropped the incremental law in the last second
    // before touchdown (SITL: the elevator eased off while the nose fell
    // 20 degrees below the flare attitude). Pulses and the divergence
    // watch do follow the height: in the flare a stall pins the elevator
    // against a nose that keeps falling, which the watch cannot tell from
    // an inversion asking for more than the actuators have
    if (!flying) {
        _airborne_latched = false;
    } else if (airborne) {
        _airborne_latched = true;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (!airborne) {
        _airborne_ms = 0;
    } else if (_airborne_ms == 0) {
        _airborne_ms = now_ms;
    }
    // the takeoff rotation is a pitch manoeuvre of several rad/s^2 that
    // the loop itself drives; a probe read inside it is read inside the
    // manoeuvre, so identification waits for the launch transient to pass
    const bool settled = airborne && now_ms - _airborne_ms >= LAUNCH_SETTLE_MS;
    in.flying = flying;
    in.held = flying && !_airborne_latched;
    in.launched = airborne;
    in.ident_allowed = settled && aspd >= IDENT_MIN_ASPD_FRAC * ref;
    in.pulses_allowed = settled;

    _indi.update(in);

    // back to surface deflection and out to the servos, except on axes
    // handed back to the stock controllers
    const Vector3f u_scaled = _indi.get_output() / _q_scale;
    const uint8_t fallback = _indi.get_fallback_mask();
    const uint32_t mask = _frontend.get_mask();
    if ((mask & (1U << 0)) && !(fallback & (1U << 0))) {
        _frontend.set_output_scaled(SRV_Channel::k_aileron, constrain_float(u_scaled.x, -1.0f, 1.0f) * SURFACE_FULL_SCALE);
    }
    if ((mask & (1U << 1)) && !(fallback & (1U << 1))) {
        _frontend.set_output_scaled(SRV_Channel::k_elevator, constrain_float(u_scaled.y, -1.0f, 1.0f) * SURFACE_FULL_SCALE);
    }
    if ((mask & (1U << 2)) && !(fallback & (1U << 2))) {
        _frontend.set_output_scaled(SRV_Channel::k_rudder, constrain_float(u_scaled.z, -1.0f, 1.0f) * SURFACE_FULL_SCALE);
    }
}

// restart from the prior, taking the current surface outputs as the
// starting actuator state so the first increment is small
void AP_CustomControl_INDI::reset(void)
{
    init_indi();
    Vector3f u(SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) / SURFACE_FULL_SCALE,
               SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / SURFACE_FULL_SCALE,
               SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / SURFACE_FULL_SCALE);
    _indi.reset(u * _q_scale);
}

#endif // AP_PLANE_CUSTOMCONTROL_INDI_ENABLED
