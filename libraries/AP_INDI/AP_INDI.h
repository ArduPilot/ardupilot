#pragma once

#include "AP_INDI_config.h"

#if AP_INDI_ENABLED

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include "AP_INDI_Ident.h"

/*
  Incremental Nonlinear Dynamic Inversion (INDI) body-rate loop with
  online identification of the control effectiveness matrix G.

  The loop works on three "virtual actuators": the normalised roll,
  pitch and yaw torque commands (-1..1) that a vehicle's mixer turns
  into motor or surface outputs. G is therefore 3x3 with a diagonal
  prior; cross-coupling is learned as off-diagonal terms.

  Each control step:
    1. angular acceleration is estimated from the gyro with a 7-tap
       Savitzky-Golay first-derivative filter (3 samples of delay)
    2. the previous commands are passed through the actuator model
       (pure delay plus first-order lag) and aligned to the same
       instant as the acceleration estimate
    3. the identifier is fed increments of both over a short window
    4. the increment du = G^-1 (omega_dot_desired - omega_dot_measured)
       is added to the aligned actuator state to give the new command

  The attitude loop is the caller's; this class only supplies the
  over-drive scale it should apply to its bandwidth while G is
  uncertain. Fallback triggers (SPEC.md section 8) are detected here
  and reported as a per-axis mask; the caller decides what flies the
  vehicle on a fallen-back axis.

  A caller may offer a measured actuator state (on a multirotor, from
  motor speed) in place of step 2's model; it is used once it has
  agreed with the model and ignored if it stops agreeing (SPEC.md,
  measured actuator state).

  This is an attempt to improve on earlier, unmerged INDI work for
  ArduPilot by Emre Saldiran (ArduPilot/ardupilot pull requests 20511
  and 21699), adding online identification, more modelling and a test
  suite. INDI itself is published work; SPEC.md, prior art, gives the
  pull requests and papers.

  See SPEC.md in this directory for the full design.
 */
class AP_INDI
{
public:
    static constexpr uint8_t NUM_AXES = AP_INDI_Ident::NUM_AXES;
    static constexpr uint8_t NUM_ACT = 3;       // virtual actuators: roll, pitch, yaw command
    static constexpr uint8_t SG_TAPS = 7;       // Savitzky-Golay derivative taps
    static constexpr uint8_t SG_DELAY = 3;      // its group delay in samples, (SG_TAPS-1)/2
    static constexpr uint8_t MAX_DELAY = 32;    // longest actuator delay in samples
    static constexpr uint8_t MAX_WINDOW = 16;   // longest identification window in samples

    struct Params {
        float g_prior[NUM_AXES];     // diagonal effectiveness prior, rad/s^2 per unit command
        float prior_rel_sigma;       // trust in the prior, relative sigma
        float rate_gain;             // omega_dot_des = rate_gain * (rate_target - rate), 1/s
        float overdrive_max;         // k_od: attitude bandwidth multiplier while uncertain
        float act_tau_s;             // actuator first-order lag time constant
        float act_delay_s;           // actuator plus sensor path pure delay
        float sensor_filt_hz;        // low-pass the gyro carries; applied to the command too so both match. 0 for none
        float du_min;                // smallest increment that counts as a stroke (normalised)
        float window_s;              // identification increment window
        float tikhonov;              // inversion regularisation relative to largest singular value
        bool pulses_enabled;         // allow excitation pulses
    };

    // bit positions of the triggers in the logged mask
    enum Trigger : uint8_t {
        TRIG_SIGN_FLIP  = 1 << 0,   // axis sign mirrored after a wrong-sign fit
        TRIG_SIGN_FAIL  = 1 << 1,   // second wrong-sign fit, axis fallen back
        TRIG_BAND_FAIL  = 1 << 2,   // estimate left its band repeatedly, axis fallen back
        TRIG_STALL      = 1 << 3,   // no accepted sample while manoeuvring, pulse injected
        TRIG_DIVERGENCE = 1 << 4,   // growing innovation with saturated output, all axes fallen back
        TRIG_GYRO       = 1 << 5,   // gyro unhealthy, identification frozen
        TRIG_GUST       = 1 << 6,   // window rejected as a disturbance
        TRIG_ALIGN      = 1 << 7,   // actuator alignment delay adjusted
    };

    struct Inputs {
        Vector3f gyro;                  // body rates, rad/s, bias removed
        Vector3f rate_target;           // desired body rates, rad/s
        Vector3f u_applied;             // command the fallback controller is applying this step, per axis; used on fallen-back axes
        bool saturated[NUM_AXES];       // mixer hit a limit on this axis with the previous output
        bool gyro_healthy;
        bool flying;                    // motors unrestricted and not sitting on the ground
        bool launched;                  // may be airborne: allows pulses; the launch clock starts at the first measured response
        bool ident_allowed;             // enough thrust for the actuators to be effective
        bool pulses_allowed;            // altitude and phase allow excitation pulses
        // the vehicle is known to be held by the ground (a fixed-wing on its
        // runway roll): the incremental law has no actuator state to build
        // on and commands the inversion of the demand alone, so it cannot
        // wind the surfaces onto their stops before release
        bool held = false;
        // actuator state measured on the vehicle, in the same units as the
        // command (a multirotor backend derives it from motor speed, after
        // the RPM feedback of the earlier INDI work, see SPEC.md prior art).
        // While valid it replaces the modelled delay and lag; the model
        // keeps running and takes over whenever the measurement is missing
        // or persistently disagrees with it
        Vector3f u_measured;
        bool u_measured_valid = false;
    };

    // where the actuator state the law builds on came from, per step
    enum class ActuatorSource : uint8_t {
        MODEL = 0,      // modelled delay and first-order lag on the command (also while a measurement qualifies)
        MEASURED = 1,   // the caller's measurement (Inputs::u_measured)
        REJECTED = 2,   // a measurement is offered but was rejected this flight
    };

    AP_INDI() {}
    CLASS_NO_COPY(AP_INDI);

    void init(float dt, const Params &p);

    // restart from the prior with the given current actuator command
    // so the first increment is taken from where the vehicle actually is
    void reset(const Vector3f &u_current);

    // one control step. The new command is read back with get_output().
    void update(const Inputs &in);

    const Vector3f &get_output() const { return _u; }

    // multiplier for the caller's attitude-loop bandwidth on an axis
    float get_att_gain_scale(uint8_t axis) const
    {
        return axis < NUM_AXES ? _att_scale[axis] : 1.0f;
    }

    // axes on which the caller must fly the fallback controller
    uint8_t get_fallback_mask() const { return _fallback_mask; }

    // magnitude the command may reach. 1 for a mixer that takes -1..1
    // directly; a fixed-wing caller that scales commands by dynamic
    // pressure sets it every step so saturation is judged correctly
    // the command range the loop may use, 1 for a command in units of
    // full actuator travel. A fixed-wing backend scales its commands by
    // dynamic pressure and passes that scale here, so the minimum stroke
    // and the excitation pulse stay a fixed fraction of surface travel.
    void set_command_limit(float limit);
    // an axis the backend leaves to the stock controller (not in its axis
    // mask): never identified, pulsed or commanded, its applied command
    // enters the model as on a fallen-back axis, and no trigger fires on it
    void disable_axis(uint8_t axis);

    ActuatorSource get_actuator_source() const { return _act_source; }

    // time the vehicle left the ground with identification enabled; 0 if not launched
    uint32_t get_launch_ms() const { return _launch_ms; }

    const AP_INDI_Ident &ident() const { return _ident; }

    // log INDG and INDS; rate limited internally
    void write_log();

private:
    // ring buffer of fixed capacity, newest sample at index 0 of at()
    template <typename T, uint8_t CAP>
    struct Ring {
        static constexpr uint8_t capacity = CAP;
        T buf[CAP];
        uint8_t head = 0;
        uint8_t count = 0;
        void clear(const T &v)
        {
            for (uint8_t i = 0; i < CAP; i++) {
                buf[i] = v;
            }
            head = 0;
            count = 0;
        }
        void push(const T &v)
        {
            head = (head + 1) % CAP;
            buf[head] = v;
            if (count < CAP) {
                count++;
            }
        }
        // sample n steps back, n = 0 is the newest
        const T &at(uint8_t n) const
        {
            return buf[(head + CAP - (n % CAP)) % CAP];
        }
    };

    void update_actuator_model(const Inputs &in);
    void update_identification(const Inputs &in, uint32_t now_ms);
    void update_schedule(uint32_t now_ms);
    void update_pulses(const Inputs &in, uint32_t now_ms);
    void update_alignment(uint32_t now_ms);
    void control_law(const Inputs &in);
    void set_fallback(uint8_t axis, Trigger why, const char *reason);
    bool stroke_is_monotonic(uint8_t act) const;
    // smallest increment that counts as a stroke, in command units
    float stroke_min() const { return _p.du_min * _u_limit; }
    // the identifier's gate on one window's increment of the lag-filtered
    // command: half of what a du_min stroke leaves in a window once the
    // actuator lag has spread it (all of it for a lag shorter than the
    // window, 1 - exp(-window/tau) for a slower actuator; a 0.14 s servo
    // passes a quarter of a stroke through a 40 ms window)
    float stroke_gate() const;
    bool window_has_pulse(uint8_t axis) const;
    void build_G(Matrix3f &G) const;

    Params _p;
    float _dt;
    float _u_limit = 1.0f;
    uint8_t _delay_samples;
    uint8_t _window;            // increment window in samples
    uint16_t _diverge_steps;    // divergence horizon in samples (DIVERGE_S)
    int8_t _align_adj;          // alignment delay correction found at runtime, samples

    AP_INDI_Ident _ident;
    AP_INDI_Ident::Config _ident_cfg;

    Ring<Vector3f, SG_TAPS> _gyro_hist;
    Ring<Vector3f, MAX_DELAY> _u_hist;              // commands, for the pure delay
    Ring<Vector3f, SG_DELAY + 8 + MAX_WINDOW> _uf_hist;  // lagged commands, for alignment and increments
    Ring<int8_t, SG_DELAY + 8 + MAX_WINDOW> _pulse_hist;  // axis pulsed at each step, -1 none; aligned like _uf_hist
    Ring<Vector3f, MAX_WINDOW + 1> _wd_hist;        // angular acceleration estimates
    Vector3f _u;                // last command out
    Vector3f _u_f;              // actuator model state
    LowPassFilter2pVector3f _u_lpf;  // matched to the gyro's low-pass
    Vector3f _u_aligned;        // actuator state at the instant of the acceleration estimate
    Vector3f _u_measured;       // last valid measured actuator state, for the log
    float _act_mismatch[NUM_AXES] {};  // low-passed |measured - modelled| per axis
    float _act_amplitude[NUM_AXES] {}; // low-passed max(|measured|, |modelled|) per axis
    ActuatorSource _act_source = ActuatorSource::MODEL;
    bool _act_measured_rejected = false;  // a measurement disagreed with the model; ignored until reset
    float _act_agree_s = 0;            // time the measurement has agreed with the model while carrying a signal
    Vector3f _omega_dot;        // measured angular acceleration (delayed by SG_DELAY)
    bool _omega_dot_valid;

    uint32_t _launch_ms;
    uint32_t _last_accept_ms[NUM_AXES];
    bool _sign_flipped[NUM_AXES];
    uint8_t _fallback_mask;
    uint8_t _disabled_mask = 0; // axes the backend never controls, kept in _fallback_mask across resets
    uint8_t _triggers;          // latched since the last log write
    uint8_t _sat_mask;          // mixer limit flags from the caller, for the log
    bool _out_saturated;        // our own command hit +-1 last step
    uint16_t _gust_rejects;
    float _att_scale[NUM_AXES];

    // divergence watch
    float _last_innov_abs[NUM_AXES];
    uint8_t _innov_grow_count[NUM_AXES];

    // gyro health
    uint32_t _gyro_bad_since_ms;

    // stall watch
    uint32_t _stale_ms[NUM_AXES][2];   // last two stale events per axis

    // excitation pulses, injected as desired angular acceleration so
    // the loop carries them instead of cancelling them
    int8_t _pulse_axis;         // -1 when none active
    uint32_t _pulse_end_ms;
    float _pulse_sign[NUM_AXES];
    bool _pulse_requested[NUM_AXES];
    uint16_t _pulse_count;
    uint8_t _pulse_next_axis;   // round-robin position
    uint32_t _last_natural_ms[NUM_AXES];  // last stroke the flight itself produced on the axis

    // alignment correlation at lags -4..4 samples around the current one
    static constexpr uint8_t ALIGN_LAGS = 9;
    static constexpr int8_t ALIGN_HALF = 4;
    static constexpr int8_t ALIGN_ADJ_MIN = -int8_t(SG_DELAY);  // never ahead of the acceleration estimate
    static constexpr int8_t ALIGN_ADJ_MAX = 6;
    float _align_corr[ALIGN_LAGS];
    uint16_t _align_samples;
    uint32_t _align_last_ms;

    uint32_t _last_log_ms;
};

#endif // AP_INDI_ENABLED
