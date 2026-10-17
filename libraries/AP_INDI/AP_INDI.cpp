#include "AP_INDI.h"

#if AP_INDI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// static constexpr members passed by reference (the MIN/MAX macros) need a
// definition outside the class under C++11
constexpr uint8_t AP_INDI::NUM_AXES;
constexpr uint8_t AP_INDI::NUM_ACT;
constexpr uint8_t AP_INDI::SG_TAPS;
constexpr uint8_t AP_INDI::SG_DELAY;
constexpr uint8_t AP_INDI::MAX_DELAY;
constexpr uint8_t AP_INDI::MAX_WINDOW;
constexpr uint8_t AP_INDI::ALIGN_LAGS;
constexpr int8_t AP_INDI::ALIGN_HALF;
constexpr int8_t AP_INDI::ALIGN_ADJ_MIN;
constexpr int8_t AP_INDI::ALIGN_ADJ_MAX;

// milliseconds without an accepted sample after which the over-drive
// factor starts decaying back to one (SPEC.md section 6)
static constexpr uint32_t ACCEPT_HOLD_MS = 200;
// time constant of that decay
static constexpr float ODRIVE_DECAY_TC_S = 0.1f;
// a stale axis: no accepted sample for this long while manoeuvring (section 8)
static constexpr uint32_t STALL_MS = 2000;
// two stale events inside this window fall the axis back
static constexpr uint32_t STALL_WINDOW_MS = 10000;
// gyro unhealthy for longer than this falls all axes back
static constexpr uint32_t GYRO_BAD_MS = 500;
// innovation growing for this many consecutive saturated steps is divergence
// an innovation that keeps growing against a pinned command for this long
// is diverging: about two time constants of a slow servo, so a flare or a
// gust that the actuator needs a moment to answer does not count (as five
// loop steps it was 12.5 ms at 400 Hz, and fired on the landing flare)
static constexpr float DIVERGE_S = 0.1f;
// a measured actuator state is checked against the modelled one, both
// low-passed over ACT_MISMATCH_TC_S as a mean |difference| and a mean
// amplitude. The check can only catch gross faults: a correct measurement
// of an actuator four times slower than modelled disagreed with the model
// by 64 % of the amplitude in the unit tests' square wave, more than a
// measurement scaled 2x (50 %), so a scale error is not detectable this
// way and is a configuration matter (the loop tolerates 0 to 2x with a
// steady rate offset). A measurement with an axis reversed disagreed by
// 140 to 200 %, one stuck at zero by 100 %: above ACT_MISMATCH_REL the
// measurement is not a lagged copy of the command and is ignored for the
// rest of the flight. Below ACT_MISMATCH_FLOOR of the command range both
// signals are too small to judge (strokes under du_min do not count either)
static constexpr float ACT_MISMATCH_REL = 0.9f;
static constexpr float ACT_MISMATCH_FLOOR = 0.02f;
static constexpr float ACT_MISMATCH_TC_S = 0.5f;
// a measurement is flown on only after it has agreed with the model for
// this long while both carried a signal above the floor: flown at once, a
// reversed measurement turned the loop's feedback positive and diverged
// in the unit tests before the averaged check could reject it. One filter
// time constant of agreement is the shortest that check can judge
static constexpr float ACT_QUALIFY_S = ACT_MISMATCH_TC_S;
// the inversion never divides by less than this fraction of the prior:
// the loop is stable only while the estimate is at least half the truth,
// and an estimate biased low by disturbed windows (the SITL plane's pitch
// walked from 16 to 4 against a prior of 9 in one manoeuvre) would raise
// the loop gain past that; at 0.6 the loop ran 1.4x hotter than designed
// and overshot a 60 degree bank demand by 35 degrees. A weaker airframe is
// flown up to 25 % more sluggishly than it could be, which is the safe
// side; the identified value is still logged and judged against the band
static constexpr float INVERSION_PRIOR_FLOOR = 0.8f;
// excitation pulse shape (section 6 step 4): width, and how long a
// natural stroke on an axis postpones a pulse there. Pulses on the
// unconverged axes follow each other round-robin with no gap: the 3x3
// regression attributes each stroke to its axis, and a gap only
// lengthens the time to the identification targets.
static constexpr uint32_t PULSE_WIDTH_MS = 30;
static constexpr uint32_t PULSE_WAIT_MS = 100;
// a window whose innovation exceeds this many noise sigmas is a gust (section 8)
static constexpr float GUST_SIGMA = 3.0f;
// before an axis is converged, a probe window whose response is outside
// this range of what the prior predicts (either sign) is a disturbance,
// not evidence about the airframe: slightly wider than the 0.25x..4x band
// a cross-axis command may be credited with at most this fraction of the
// axis's own prior effectiveness (AP_INDI_Ident::Config::coupling_max):
// roll, pitch and yaw commands are the actuators here, and no airframe
// is cross-coupled by more than half of its own axis
static constexpr float COUPLING_REL_MAX = 0.5f;
// a probe window's stroke may exceed the pulse's own largest increment by
// this factor before the window is the loop's and not the pulse's: the
// pulse itself comes through the inversion at up to 1.25 times its size
// (the inversion floor of 0.8 of the prior), and the loop's ordinary
// corrections during a pulse add a fraction of that again
static constexpr float PROBE_STROKE_MAX = 2.0f;
static constexpr float PROBE_RATIO_MIN = 0.2f;
static constexpr float PROBE_RATIO_MAX = 5.0f;
// alignment adjustment: strokes needed before a decision, how much the
// off-zero peak must exceed the zero-lag correlation, and the step
static constexpr uint16_t ALIGN_MIN_STROKES = 40;
static constexpr float ALIGN_PEAK_RATIO = 1.5f;
// rate target above which the vehicle counts as manoeuvring, rad/s
static constexpr float MANOEUVRE_RATE = 0.1f;
// a clear stroke that produces less than this fraction of the angular
// acceleration the prior predicts is a vehicle that cannot rotate (still
// on the ground, held), not a weak one: the band's lower edge is 0.25
static constexpr float GROUND_RESPONSE_FRAC = 0.05f;
// logging: full rate for this long after launch, then 10 Hz
static constexpr uint32_t LOG_FULL_RATE_MS = 2000;
static constexpr uint32_t LOG_SLOW_PERIOD_MS = 100;
// relative sigma at which the schedule reaches nominal bandwidth
static constexpr float S_CONV = 0.10f;

void AP_INDI::init(float dt, const Params &p)
{
    _p = p;
    _dt = MAX(dt, 1.0e-4f);

    _delay_samples = constrain_int16(roundf(_p.act_delay_s / _dt), 0, MAX_DELAY - 1);
    _window = constrain_int16(roundf(_p.window_s / _dt), 2, MAX_WINDOW);
    _diverge_steps = MAX(uint16_t(roundf(DIVERGE_S / _dt)), uint16_t(2));
    _align_adj = 0;
    if (is_positive(_p.sensor_filt_hz)) {
        _u_lpf.set_cutoff_frequency(1.0f / _dt, _p.sensor_filt_hz);
    }

    // identifier on the three virtual actuators, diagonal prior
    float prior[NUM_AXES][AP_INDI_Ident::MAX_ACTUATORS] {};
    float prior_rel_sigma[NUM_AXES];
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        prior[i][i] = _p.g_prior[i];
        prior_rel_sigma[i] = _p.prior_rel_sigma;
    }
    _ident_cfg = AP_INDI_Ident::Config();
    _ident_cfg.du_min = stroke_gate();
    _ident_cfg.coupling_max = COUPLING_REL_MAX;
    _ident.init(NUM_ACT, prior, prior_rel_sigma, _ident_cfg);

    reset(Vector3f());
}

void AP_INDI::reset(const Vector3f &u_current)
{
    _u = u_current;
    _u_f = u_current;
    _u_aligned = u_current;
    _u_lpf.reset(u_current);
    _u_measured.zero();
    _act_source = ActuatorSource::MODEL;
    _act_measured_rejected = false;
    _act_agree_s = 0;
    _gyro_hist.clear(Vector3f());
    _u_hist.clear(u_current);
    _uf_hist.clear(u_current);
    _pulse_hist.clear(-1);
    _wd_hist.clear(Vector3f());
    _omega_dot.zero();
    _omega_dot_valid = false;

    _launch_ms = 0;
    _fallback_mask = _disabled_mask;
    _triggers = 0;
    _sat_mask = 0;
    _out_saturated = false;
    _gust_rejects = 0;
    _gyro_bad_since_ms = 0;
    _pulse_axis = -1;
    _pulse_end_ms = 0;
    _pulse_count = 0;
    _pulse_next_axis = 0;
    _align_last_ms = 0;
    _align_samples = 0;
    _last_log_ms = 0;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        _last_accept_ms[i] = 0;
        _sign_flipped[i] = false;
        _att_scale[i] = 1.0f;
        _last_innov_abs[i] = 0;
        _innov_grow_count[i] = 0;
        _stale_ms[i][0] = 0;
        _stale_ms[i][1] = 0;
        _pulse_sign[i] = 1.0f;
        _pulse_requested[i] = false;
        _last_natural_ms[i] = 0;
        _act_mismatch[i] = 0;
        _act_amplitude[i] = 0;
        // a mirrored prior is not carried over: a fresh engage starts
        // from the configured sign and must re-prove it
        _ident.restart_axis(i, false);
        if (_ident.get_prior(i, i) * _p.g_prior[i] < 0) {
            _ident.restart_axis(i, true);
        }
    }
    for (uint8_t l = 0; l < ALIGN_LAGS; l++) {
        _align_corr[l] = 0;
    }
}

void AP_INDI::update(const Inputs &in)
{
    // zero is "not launched" in the timers below, so never let it be zero
    const uint32_t now_ms = MAX(AP_HAL::millis(), 1U);

    // 1. angular acceleration from the gyro history. Savitzky-Golay
    // first derivative, 7 taps: coefficients (-3,-2,-1,0,1,2,3)/28 per
    // sample, estimating the slope at the centre tap, SG_DELAY samples ago.
    _gyro_hist.push(in.gyro);
    if (_gyro_hist.count >= SG_TAPS) {
        Vector3f sum;
        for (uint8_t k = 0; k < SG_TAPS; k++) {
            // newest sample has weight +3, oldest -3
            const float w = float(int8_t(SG_DELAY) - int8_t(k));
            sum += _gyro_hist.at(k) * w;
        }
        _omega_dot = sum / (28.0f * _dt);
        _omega_dot_valid = true;
    }

    // 2. actuator model on the command issued last step. On an axis that
    // has fallen back the plant saw the fallback controller's command,
    // not ours, so that is what enters the model.
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (_fallback_mask & (1U << i)) {
            _u[i] = in.u_applied[i];
        }
    }
    update_actuator_model(in);
    _sat_mask = 0;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (in.saturated[i]) {
            _sat_mask |= (1U << i);
        }
    }

    // 3. gyro health gate
    if (!in.gyro_healthy) {
        if (_gyro_bad_since_ms == 0) {
            _gyro_bad_since_ms = now_ms;
            _triggers |= TRIG_GYRO;
        } else if (now_ms - _gyro_bad_since_ms > GYRO_BAD_MS && _fallback_mask != 0x7) {
            set_fallback(0xFF, TRIG_GYRO, "gyro unhealthy");
        }
        // hold the last command: nothing measured can be trusted
        _omega_dot_valid = false;
        write_log();
        return;
    }
    _gyro_bad_since_ms = 0;

    // 4. identification, schedule and pulses
    update_identification(in, now_ms);
    update_schedule(now_ms);
    update_pulses(in, now_ms);
    update_alignment(now_ms);

    // 5. the inversion
    control_law(in);

    write_log();
}

void AP_INDI::update_actuator_model(const Inputs &in)
{
    // pure delay on the command, then first-order lag. The model always
    // runs, so it can take over from a measurement at any step
    _u_hist.push(_u);
    const Vector3f &u_delayed = _u_hist.at(_delay_samples);
    if (_p.act_tau_s <= _dt) {
        _u_f = u_delayed;
    } else {
        _u_f += (u_delayed - _u_f) * (_dt / _p.act_tau_s);
    }

    // a measured actuator state, when offered, is what the actuators
    // actually did, lag and delay included, and is preferred to the
    // model; unless it has disagreed with the model for long enough to
    // be reversed, misordered or stuck (ACT_MISMATCH_REL)
    Vector3f u_state = _u_f;
    _act_source = ActuatorSource::MODEL;
    if (in.u_measured_valid) {
        _u_measured = in.u_measured;
        const float alpha = MIN(_dt / ACT_MISMATCH_TC_S, 1.0f);
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            _act_mismatch[i] += (fabsf(in.u_measured[i] - _u_f[i]) - _act_mismatch[i]) * alpha;
            _act_amplitude[i] += (MAX(fabsf(in.u_measured[i]), fabsf(_u_f[i])) - _act_amplitude[i]) * alpha;
            const float allowed = MAX(ACT_MISMATCH_REL * _act_amplitude[i], ACT_MISMATCH_FLOOR * _u_limit);
            if (!_act_measured_rejected && _act_mismatch[i] > allowed) {
                _act_measured_rejected = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INDI: actuator measurement disagrees on axis %u, using model", unsigned(i));
            }
        }
        // agreement only counts while some axis carries a signal to judge
        bool signal = false;
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            signal |= _act_amplitude[i] > ACT_MISMATCH_FLOOR * _u_limit;
        }
        if (signal && _act_agree_s < ACT_QUALIFY_S) {
            _act_agree_s += _dt;
        }
        if (_act_measured_rejected) {
            _act_source = ActuatorSource::REJECTED;
        } else if (_act_agree_s >= ACT_QUALIFY_S) {
            u_state = in.u_measured;
            _act_source = ActuatorSource::MEASURED;
        }
    }

    // the gyro the acceleration comes from is low-passed; pass the
    // actuator state through the same filter so the two compare like
    // for like
    Vector3f u_matched = u_state;
    if (is_positive(_p.sensor_filt_hz)) {
        u_matched = _u_lpf.apply(u_state);
    }
    // align with the acceleration estimate, which refers to SG_DELAY
    // samples ago plus whatever the runtime alignment check added
    _uf_hist.push(u_matched);
    _pulse_hist.push(_pulse_axis);
    _u_aligned = _uf_hist.at(uint8_t(int8_t(SG_DELAY) + _align_adj));
}

bool AP_INDI::window_has_pulse(uint8_t axis) const
{
    // was a pulse on this axis active at any step the increment spans,
    // allowing for the actuator delay between command and the model
    const uint8_t base = uint8_t(int8_t(SG_DELAY) + _align_adj);
    for (uint8_t k = 0; k <= _window + _delay_samples; k++) {
        if (_pulse_hist.at(base + k) == int8_t(axis)) {
            return true;
        }
    }
    return false;
}

void AP_INDI::disable_axis(uint8_t axis)
{
    if (axis < NUM_AXES) {
        _disabled_mask |= (1U << axis);
        _fallback_mask |= (1U << axis);
    }
}

void AP_INDI::set_command_limit(float limit)
{
    // never below a twentieth of full travel: the loop must keep some
    // authority, and the stroke gates must not collapse to zero
    _u_limit = MAX(limit, 0.05f);
    // the identifier's gate follows the stroke size
    _ident_cfg.du_min = stroke_gate();
    _ident.set_stroke_min(_ident_cfg.du_min);
}

float AP_INDI::stroke_gate() const
{
    const float window_s = float(_window) * _dt;
    float through = 1.0f;
    if (_p.act_tau_s > window_s) {
        through = 1.0f - expf(-window_s / _p.act_tau_s);
    }
    return 0.5f * stroke_min() * through;
}

bool AP_INDI::stroke_is_monotonic(uint8_t act) const
{
    // every step of the aligned actuator over the window moves the same
    // way (or not at all), so the increment is one stroke, not a reversal
    const uint8_t base = uint8_t(int8_t(SG_DELAY) + _align_adj);
    float dir = 0;
    for (uint8_t k = 0; k < _window; k++) {
        const float step = _uf_hist.at(base + k)[act] - _uf_hist.at(base + k + 1)[act];
        if (is_zero(step)) {
            continue;
        }
        if (is_zero(dir)) {
            dir = step;
        } else if (dir * step < 0) {
            return false;
        }
    }
    return true;
}

void AP_INDI::update_identification(const Inputs &in, uint32_t now_ms)
{
    if (!in.flying) {
        // back on the ground: the next liftoff is a new launch
        _launch_ms = 0;
        _wd_hist.clear(Vector3f());
        return;
    }
    if (!_omega_dot_valid || !in.ident_allowed) {
        return;
    }

    _wd_hist.push(_omega_dot);
    if (_wd_hist.count <= _window) {
        return;
    }

    // increments over the window of the aligned actuator state and the
    // acceleration estimate, SPEC.md section 4
    const uint8_t base = uint8_t(int8_t(SG_DELAY) + _align_adj);
    const Vector3f du_v = _uf_hist.at(base) - _uf_hist.at(base + _window);
    const Vector3f dwd = _wd_hist.at(0) - _wd_hist.at(_window);
    float du[AP_INDI_Ident::MAX_ACTUATORS] {};
    for (uint8_t j = 0; j < NUM_ACT; j++) {
        du[j] = du_v[j];
    }
    bool any_saturated = false;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        any_saturated |= in.saturated[i];
    }

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (_fallback_mask & (1U << i)) {
            continue;
        }
        if (_ident.get_state(i) == AP_INDI_Ident::AxisState::FAILED) {
            continue;
        }
        if (!stroke_is_monotonic(i)) {
            continue;
        }
        // until converged an axis learns only from windows in which its
        // own actuator made a stroke: the identifier's gate is on the norm
        // over all actuators, so a window carried by another axis's stroke
        // would otherwise feed this one a response of nothing (on the SITL
        // quad that started the launch clock on the ground and walked the
        // estimate to the band edge before the vehicle could rotate)
        if (_ident.get_state(i) < AP_INDI_Ident::AxisState::CONVERGED &&
            fabsf(du_v[i]) < _ident_cfg.du_min) {
            continue;
        }
        if (fabsf(du_v[i]) >= _ident_cfg.du_min &&
            fabsf(dwd[i]) < GROUND_RESPONSE_FRAC * fabsf(_p.g_prior[i] * du_v[i])) {
            // no response to speak of to a real stroke: the vehicle is
            // constrained (on the ground, held), skip
            continue;
        }

        // gust rejection once the fit is trusted: a window the fit cannot
        // explain within GUST_SIGMA noise sigmas is a disturbance
        if (_ident.get_state(i) >= AP_INDI_Ident::AxisState::USABLE) {
            float predicted = 0;
            for (uint8_t j = 0; j < NUM_ACT; j++) {
                predicted += _ident.get_effectiveness(i, j) * du[j];
            }
            const float noise = _ident.get_noise_sigma(i);
            if (is_positive(noise) && fabsf(dwd[i] - predicted) > GUST_SIGMA * noise &&
                fabsf(dwd[i]) > fabsf(predicted)) {
                _gust_rejects++;
                _triggers |= TRIG_GUST;
                continue;
            }
        }

        // a deliberate pulse is the only stroke whose cause is known. Until
        // the axis is converged, only such windows feed the identifier: the
        // loop's own strokes while it counters a disturbance correlate the
        // wrong way with the response and would bias the estimate low.
        // Once converged, the explained-fraction and gust gates take over.
        // a window is a probe only if the pulse can account for its stroke:
        // the pulse's own increment in a window is at most twice the gate
        // (the gate is set at half of it), and a window with more than
        // PROBE_STROKE_MAX times that is the loop's own manoeuvre with the
        // pulse buried in it, whose response is to whatever the loop was
        // answering (the SITL plane's takeoff rotation read as a reversed
        // elevator, with the loop pulling back against a rising nose)
        const bool probe = window_has_pulse(i) &&
                           fabsf(du_v[i]) <= PROBE_STROKE_MAX * 2.0f * _ident_cfg.du_min;
        if (!probe && fabsf(du_v[i]) >= stroke_min()) {
            _last_natural_ms[i] = now_ms;
        }
        if (_p.pulses_enabled && !probe && _ident.get_state(i) < AP_INDI_Ident::AxisState::CONVERGED) {
            continue;
        }
        if (probe && _ident.get_state(i) < AP_INDI_Ident::AxisState::CONVERGED &&
            fabsf(du_v[i]) >= _ident_cfg.du_min) {
            const float ratio = dwd[i] / (_p.g_prior[i] * du_v[i]);
            if (fabsf(ratio) < PROBE_RATIO_MIN || fabsf(ratio) > PROBE_RATIO_MAX) {
                _gust_rejects++;
                _triggers |= TRIG_GUST;
                continue;
            }
        }
        if (_ident.update(i, du, dwd[i], any_saturated, probe)) {
            _last_accept_ms[i] = now_ms;
            // the launch clock starts at the first stroke that produces a
            // response: that is the moment the vehicle can actually rotate
            if (_launch_ms == 0 && in.launched) {
                _launch_ms = now_ms;
                for (uint8_t k = 0; k < NUM_AXES; k++) {
                    _last_accept_ms[k] = now_ms;
                }
            }
        }

        if (_ident.get_state(i) == AP_INDI_Ident::AxisState::FAILED) {
            switch (_ident.get_fail_reason(i)) {
            case AP_INDI_Ident::FailReason::SIGN:
                if (!_sign_flipped[i]) {
                    // the sign-reversal response: mirror the prior once
                    _sign_flipped[i] = true;
                    _ident.restart_axis(i, true);
                    _last_accept_ms[i] = now_ms;
                    _triggers |= TRIG_SIGN_FLIP;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INDI: axis %u sign reversed", unsigned(i));
                } else {
                    set_fallback(i, TRIG_SIGN_FAIL, "sign failed twice");
                }
                break;
            case AP_INDI_Ident::FailReason::BAND:
                set_fallback(i, TRIG_BAND_FAIL, "out of band");
                break;
            case AP_INDI_Ident::FailReason::NONE:
                break;
            }
        }

        // stall: manoeuvring but nothing accepted for STALL_MS
        const bool manoeuvring = fabsf(in.rate_target[i]) > MANOEUVRE_RATE || fabsf(du_v[i]) > stroke_min();
        if (_launch_ms != 0 && manoeuvring && now_ms - _last_accept_ms[i] > STALL_MS) {
            _last_accept_ms[i] = now_ms;  // one stale event per STALL_MS
            _triggers |= TRIG_STALL;
            // a stall before convergence means the estimate never got
            // its evidence: hand the axis back. Once converged the
            // estimate is simply frozen; in cruise the gust gate rejects
            // most windows (its noise sigma comes from the launch) so a
            // stall there is the normal state, not a fault, and the
            // divergence trigger covers an actuator that degrades
            const bool converged = _ident.get_state(i) >= AP_INDI_Ident::AxisState::CONVERGED;
            if (_stale_ms[i][0] != 0 && now_ms - _stale_ms[i][0] < STALL_WINDOW_MS && !converged) {
                set_fallback(i, TRIG_STALL, "stalled twice");
            } else {
                _stale_ms[i][1] = _stale_ms[i][0];
                _stale_ms[i][0] = now_ms;
                _pulse_requested[i] = true;
            }
        }
    }

    // cruise once every axis still being identified has converged: the
    // recursion then forgets slowly and gives no more band verdicts, since
    // the launch evidence has been judged and the natural windows of
    // cruise carry the closed-loop bias (SPEC.md, convergence targets)
    bool cruise = true;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if ((_fallback_mask & (1U << i)) == 0 &&
            _ident.get_state(i) < AP_INDI_Ident::AxisState::CONVERGED) {
            cruise = false;
        }
    }
    _ident.set_cruise(cruise);
}

void AP_INDI::update_schedule(uint32_t now_ms)
{
    // over-drive the attitude loop while the primary ratio is uncertain:
    // scale = clip(s_i / S_CONV, 1, k_od), held while samples are being
    // accepted and decayed to one when acceptance stops
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        float target = 1.0f;
        const bool accepting = _launch_ms != 0 && (now_ms - _last_accept_ms[i]) < ACCEPT_HOLD_MS;
        if (accepting && !(_fallback_mask & (1U << i))) {
            float s = _ident.get_rel_sigma(i);
            if (_ident.get_state(i) == AP_INDI_Ident::AxisState::PRIOR || !is_positive(s)) {
                s = _p.prior_rel_sigma;
            }
            target = constrain_float(s / S_CONV, 1.0f, MAX(_p.overdrive_max, 1.0f));
            _att_scale[i] = target;
        } else {
            _att_scale[i] += (target - _att_scale[i]) * constrain_float(_dt / ODRIVE_DECAY_TC_S, 0, 1);
        }
    }
}

void AP_INDI::update_pulses(const Inputs &in, uint32_t now_ms)
{
    if (_pulse_axis >= 0 && now_ms >= _pulse_end_ms) {
        _pulse_axis = -1;
    }
    if (!_p.pulses_enabled || !in.pulses_allowed || !in.flying || !in.launched || _pulse_axis >= 0) {
        return;
    }
    // round-robin over the axes that still need excitation: not yet
    // converged (or stale), not fallen back, and not excited by the flight
    // itself within the last PULSE_WAIT_MS. Before the first response a
    // vehicle that may be airborne is probed straight away; a pulse the
    // ground absorbs costs nothing.
    for (uint8_t n = 0; n < NUM_AXES; n++) {
        const uint8_t i = (_pulse_next_axis + n) % NUM_AXES;
        if (_fallback_mask & (1U << i)) {
            continue;
        }
        const bool unconverged = _ident.get_state(i) < AP_INDI_Ident::AxisState::CONVERGED;
        const bool natural = _last_natural_ms[i] != 0 && now_ms - _last_natural_ms[i] < PULSE_WAIT_MS;
        if ((unconverged && !natural) || _pulse_requested[i]) {
            _pulse_requested[i] = false;
            _pulse_axis = i;
            // a pulse must outlast the actuator: hold it for two time
            // constants (86 % of the stroke reached) when the lag is longer
            // than the nominal width, which a 30 ms pulse into a 0.14 s
            // servo would otherwise leave at a fifth of its height
            _pulse_end_ms = now_ms + MAX(PULSE_WIDTH_MS, uint32_t(2000.0f * _p.act_tau_s));
            _pulse_next_axis = (i + 1) % NUM_AXES;
            // alternate sign so successive pulses cancel in attitude
            _pulse_sign[i] = -_pulse_sign[i];
            _pulse_count++;
            return;
        }
    }
}

void AP_INDI::update_alignment(uint32_t now_ms)
{
    // cross-correlate the increment of each axis's own command with its
    // acceleration increment at lags around the current alignment. A
    // consistent peak off zero means the actuator model is early or late
    // by that many samples; move the alignment to the peak. Decisions are
    // taken at 10 Hz once enough strokes have been seen, so the first
    // ones fall inside the launch window where a wrong alignment biases
    // the identification most.
    if (_launch_ms == 0 || _wd_hist.count <= _window) {
        return;
    }
    const Vector3f dwd = _wd_hist.at(0) - _wd_hist.at(_window);
    bool stroke = false;
    for (uint8_t l = 0; l < ALIGN_LAGS; l++) {
        const int8_t lag = int8_t(l) - ALIGN_HALF;
        const int16_t base = int16_t(SG_DELAY) + _align_adj + lag;
        if (base < 0 || base + _window >= int16_t(_uf_hist.capacity)) {
            continue;
        }
        float c = 0;
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            const float du_i = _uf_hist.at(base)[i] - _uf_hist.at(base + _window)[i];
            if (fabsf(du_i) >= _ident_cfg.du_min) {
                stroke = true;
            }
            // sign of the prior so a correctly signed response counts positive
            const float sgn = is_negative(_ident.get_prior(i, i)) ? -1.0f : 1.0f;
            c += sgn * du_i * dwd[i];
        }
        _align_corr[l] += c;
    }
    if (stroke) {
        _align_samples++;
    }
    if (now_ms - _align_last_ms < 100 || _align_samples < ALIGN_MIN_STROKES) {
        return;
    }
    _align_last_ms = now_ms;
    uint8_t best = ALIGN_HALF;
    for (uint8_t l = 0; l < ALIGN_LAGS; l++) {
        if (_align_corr[l] > _align_corr[best]) {
            best = l;
        }
    }
    if (best != ALIGN_HALF && _align_corr[best] > ALIGN_PEAK_RATIO * MAX(_align_corr[ALIGN_HALF], 0.0f)) {
        // a positive lag means the response arrived later than modelled.
        // One sample at a time: a shift changes the pairing the next
        // correlation is built on, and a larger step chases itself.
        const int8_t step = (best > ALIGN_HALF) ? 1 : -1;
        const int16_t adj = int16_t(_align_adj) + step;
        _align_adj = int8_t(constrain_int16(adj, ALIGN_ADJ_MIN, ALIGN_ADJ_MAX));
        _triggers |= TRIG_ALIGN;
    }
    for (uint8_t l = 0; l < ALIGN_LAGS; l++) {
        _align_corr[l] = 0;
    }
    _align_samples = 0;
}

void AP_INDI::build_G(Matrix3f &G) const
{
    Vector3f *rows[NUM_AXES] = { &G.a, &G.b, &G.c };
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        for (uint8_t j = 0; j < NUM_ACT; j++) {
            float g;
            if (i == j) {
                // the live estimate once its sign is confirmed; before
                // that the prior, so the first few samples cannot swing
                // the inversion gain while they are still being judged
                const float prior = _ident.get_prior(i, j);
                if (_ident.get_state(i) >= AP_INDI_Ident::AxisState::SIGN_OK) {
                    g = _ident.get_effectiveness(i, j);
                    // same sign as the prior (a mirrored prior carries the
                    // flip), magnitude never below the floor
                    if (fabsf(g) < INVERSION_PRIOR_FLOOR * fabsf(prior)) {
                        g = INVERSION_PRIOR_FLOOR * prior;
                    }
                } else {
                    g = prior;
                }
                if (!is_positive(fabsf(g))) {
                    g = _p.g_prior[i];
                }
            } else if (_ident.get_state(i) >= AP_INDI_Ident::AxisState::USABLE &&
                       _ident.coupling_significant(i, j)) {
                g = _ident.get_effectiveness(i, j);
            } else {
                // coupling is used only once the row is trusted and the
                // coupling itself stands clear of zero: an uncertain one
                // is a fit to whatever the other loops were doing
                g = 0;
            }
            (*rows[i])[j] = g;
        }
    }
}

void AP_INDI::control_law(const Inputs &in)
{
    // desired angular acceleration from the rate error, limited to half
    // the prior authority so an optimistic prior cannot saturate (section 6)
    Vector3f nu;
    // the authority scales with the command range: a fixed-wing at twice
    // its reference airspeed has four times the acceleration per unit of
    // command, and a limit set at the reference would let the loop use
    // an eighth of what it has
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        const float limit = 0.5f * fabsf(_p.g_prior[i]) * _u_limit;
        nu[i] = constrain_float(_p.rate_gain * (in.rate_target[i] - in.gyro[i]), -limit, limit);
    }
    // an excitation pulse is asked for as the acceleration a du_min stroke
    // of the prior would give. Added to the demand, the loop produces and
    // holds the stroke for the pulse width; added to the command instead,
    // the loop would read the response as an error and cancel it within
    // one actuator delay, leaving almost nothing to identify from.
    if (_pulse_axis >= 0) {
        nu[_pulse_axis] += _pulse_sign[_pulse_axis] * stroke_min() * fabsf(_p.g_prior[_pulse_axis]);
    }

    const Vector3f innovation = _omega_dot_valid ? (nu - _omega_dot) : nu;

    // divergence watch: the innovation growing for DIVERGE_STEPS steps
    // while our own command is pinned at its limit means the inversion
    // is asking for more than the actuators have, in the wrong direction
    const bool watch = in.launched && _launch_ms != 0;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (_fallback_mask & (1U << i)) {
            _innov_grow_count[i] = 0;
            _last_innov_abs[i] = 0;
            continue;
        }
        const float a = fabsf(innovation[i]);
        if (watch && _out_saturated && a > _last_innov_abs[i]) {
            if (++_innov_grow_count[i] >= _diverge_steps && _fallback_mask != 0x7) {
                set_fallback(0xFF, TRIG_DIVERGENCE, "diverging");
            }
        } else {
            _innov_grow_count[i] = 0;
        }
        _last_innov_abs[i] = a;
    }

    // regularised inversion: du = (G'G + lambda I)^-1 G' innovation, with
    // lambda a fraction of the largest diagonal of G'G (section 9)
    Matrix3f G;
    build_G(G);
    Matrix3f Gt = G.transposed();
    Matrix3f GtG = Gt * G;
    float max_diag = MAX(GtG.a.x, MAX(GtG.b.y, GtG.c.z));
    const float lambda = MAX(_p.tikhonov, 1.0e-6f) * MAX(max_diag, 1.0e-6f);
    GtG.a.x += lambda;
    GtG.b.y += lambda;
    GtG.c.z += lambda;
    Matrix3f inv;
    Vector3f du;
    if (GtG.inverse(inv)) {
        du = inv * (Gt * innovation);
    }

    // a vehicle held by the ground shows no response, so the increment
    // would integrate the command onto a stop; while held the command is
    // the inversion of the demand alone, a proportional law like the one
    // the stock controllers run on the runway (the SITL plane released
    // with full up elevator and rotated at 80 deg/s before this)
    Vector3f u = in.held ? du : (_u_aligned + du);
    _out_saturated = false;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (_fallback_mask & (1U << i)) {
            // not ours to command: report what the fallback applies
            u[i] = in.u_applied[i];
            continue;
        }
        if (fabsf(u[i]) >= _u_limit) {
            _out_saturated = true;
        }
        u[i] = constrain_float(u[i], -_u_limit, _u_limit);
    }
    _u = u;
}

void AP_INDI::set_fallback(uint8_t axis, Trigger why, const char *reason)
{
    const uint8_t before = _fallback_mask;
    if (axis == 0xFF) {
        _fallback_mask = 0x7;
    } else if (axis < NUM_AXES) {
        _fallback_mask |= (1U << axis);
    }
    if (_fallback_mask != before) {
        _triggers |= why;
        if (axis == 0xFF) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "INDI: %s, fallback all axes", reason);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "INDI: axis %u %s, fallback", unsigned(axis), reason);
        }
    }
}

void AP_INDI::write_log()
{
#if HAL_LOGGING_ENABLED
    // the unit tests run without a logger; the documentation scanner in
    // Tools/autotest only recognises logger.Write(), hence the reference
    AP_Logger *logger_ptr = AP_Logger::get_singleton();
    if (logger_ptr == nullptr) {
        // unit tests and vehicles without logging
        return;
    }
    AP_Logger &logger = *logger_ptr;
    const uint32_t now_ms = AP_HAL::millis();
    const bool full_rate = _launch_ms != 0 && (now_ms - _launch_ms) < LOG_FULL_RATE_MS;
    if (!full_rate && now_ms - _last_log_ms < LOG_SLOW_PERIOD_MS) {
        return;
    }
    _last_log_ms = now_ms;
    const uint64_t now_us = AP_HAL::micros64();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        // @LoggerMessage: INDG
        // @Description: INDI control effectiveness estimate for one axis
        // @Field: TimeUS: Time since system startup
        // @Field: I: axis, 0 roll 1 pitch 2 yaw
        // @Field: G0: effectiveness of the roll command on this axis
        // @Field: G1: effectiveness of the pitch command on this axis
        // @Field: G2: effectiveness of the yaw command on this axis
        // @Field: RS: relative standard deviation of the primary ratio
        // @Field: NS: estimated measurement noise
        // @Field: St: identification state
        // @FieldValueEnum: St: AP_INDI_Ident::AxisState
        // @Field: N: accepted samples
        // @Field: UM: measured actuator state on this axis, last valid value
        // @Field: AM: low-passed disagreement between measured and modelled actuator state, relative to their amplitude
        // @Field: Src: actuator state source
        // @FieldValueEnum: Src: AP_INDI::ActuatorSource
        logger.Write("INDG", "TimeUS,I,G0,G1,G2,RS,NS,St,N,UM,AM,Src", "s#EEE-E-----", "F-----------", "QBfffffBIffB",
                                    now_us, i,
                                    _ident.get_effectiveness(i, 0),
                                    _ident.get_effectiveness(i, 1),
                                    _ident.get_effectiveness(i, 2),
                                    _ident.get_rel_sigma(i),
                                    _ident.get_noise_sigma(i),
                                    uint8_t(_ident.get_state(i)),
                                    _ident.get_accepted_count(i),
                                    _u_measured[i],
                                    _act_mismatch[i] / MAX(_act_amplitude[i], ACT_MISMATCH_FLOOR * _u_limit),
                                    uint8_t(_act_source));
    }

    // a log message holds at most 16 fields, so Trg carries the

    // alignment event and Aln the current adjustment

    // @LoggerMessage: INDS
    // @Description: INDI controller state, triggers and outputs
    // @Field: TimeUS: Time since system startup
    // @Field: TMs: milliseconds since identification was enabled at launch, 0 before
    // @Field: SR: attitude loop over-drive scale, roll
    // @Field: SP: attitude loop over-drive scale, pitch
    // @Field: SY: attitude loop over-drive scale, yaw
    // @Field: UR: roll command out
    // @Field: UP: pitch command out
    // @Field: UY: yaw command out
    // @Field: WDR: measured roll angular acceleration
    // @Field: WDP: measured pitch angular acceleration
    // @Field: WDY: measured yaw angular acceleration
    // @Field: Fb: fallback axis mask
    // @Field: Trg: triggers seen since the previous record
    // @FieldBitmaskEnum: Trg: AP_INDI::Trigger
    // @Field: Pls: excitation pulses so far
    // @Field: Sat: mixer limit flags, bit 0 roll, 1 pitch, 2 yaw
    // @Field: Aln: alignment delay adjustment, samples
    logger.Write("INDS", "TimeUS,TMs,SR,SP,SY,UR,UP,UY,WDR,WDP,WDY,Fb,Trg,Pls,Sat,Aln", "ss------EEE-----", "FC--------------", "QIfffffffffBBHBb",
                                now_us,
                                _launch_ms == 0 ? 0U : (now_ms - _launch_ms),
                                _att_scale[0], _att_scale[1], _att_scale[2],
                                _u.x, _u.y, _u.z,
                                _omega_dot.x, _omega_dot.y, _omega_dot.z,
                                _fallback_mask, _triggers, _pulse_count, _sat_mask, _align_adj);
    _triggers = 0;
#endif
}

#endif // AP_INDI_ENABLED
