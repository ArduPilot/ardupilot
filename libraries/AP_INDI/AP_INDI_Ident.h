#pragma once

#include "AP_INDI_config.h"

#if AP_INDI_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>

/*
  Online identification of the control effectiveness matrix G for an
  INDI rate loop: the angular acceleration produced per unit of
  actuator command, one row per body axis.

  Each row is estimated independently by recursive least squares on
  increments, so inertia, damping and trim never enter the regression:

      delta_omega_dot_i = g_i . delta_u + e_i

  The caller supplies, per control step during an actuator stroke, the
  change in lag-compensated command of every actuator and the matching
  change in angular acceleration on the axis. Time alignment of the two
  is the caller's job.

  Internally the state is kept as ratios to the prior so that every
  coefficient is of order one in single precision. For actuators with a
  non-zero prior the ratio is g_j / prior_j; for actuators with a zero
  prior (no coupling expected) it is g_j / g_ref, where g_ref is the
  largest |prior| on the axis. The "primary" actuator of an axis is the
  one with the largest |prior|; its ratio carries the sign test, the
  band check and the relative-sigma convergence measure.

  A reversed actuator is caught two ways: the fitted primary ratio
  going confidently negative (3 sigma), and, faster, a run of
  sign_fail_count consecutive samples whose measured response opposes
  the prior's sign by more than the noise. The second is what stops a
  reversed axis within a few windows, before the covariance has
  shrunk enough for the first.

  See SPEC.md in this directory for the full design.
 */
class AP_INDI_Ident
{
public:
    static constexpr uint8_t MAX_ACTUATORS = 8;
    static constexpr uint8_t NUM_AXES = 3;

    struct Config {
        float lambda_launch = 0.995f;      // forgetting factor while converging
        float lambda_cruise = 0.9995f;     // forgetting factor once converged
        float du_min = 0.05f;              // minimum |delta_u| for a sample to count (normalised)
        float band_lo = 0.25f;             // allowed primary ratio band, lower edge
        float band_hi = 4.0f;              // allowed primary ratio band, upper edge
        float rel_sigma_floor = 0.02f;     // floor on covariance diagonal, relative
        float rel_sigma_usable = 0.25f;    // relative sigma below which an axis is usable
        float rel_sigma_converged = 0.10f; // relative sigma below which an axis is converged
        float sign_margin_sigma = 3.0f;    // sigma margin required to confirm the sign
        float explained_min = 0.5f;        // fraction of |delta_omega_dot| the fit must explain once usable
        float offdiag_rel_sigma = 0.3f;    // prior relative sigma for zero-prior actuators
        float noise_rel_init = 0.02f;      // initial guess of measurement noise, as a fraction of g_ref; pessimistic so one sample cannot swing the estimate
        float sign_resp_min = 0.02f;       // response, as a fraction of g_ref, below which the fast sign test ignores a sample
        float noise_rel_floor = 0.005f;    // floor on the estimated measurement noise
        uint8_t noise_window = 15;         // samples over which the noise estimate averages
        uint8_t noise_prior_weight = 5;    // samples' worth of weight the initial noise guess carries against the first residuals
        uint8_t max_clamps = 2;            // clamps on one axis before it is marked failed
        uint8_t band_min_samples = 10;     // no band verdict before this many accepted samples, while the estimate is still raw
        float band_margin = 0.35f;         // the unclamped shadow estimate must be this far beyond an edge, relative, to fail the axis
        // the shadow must also be beyond the edge by this many of its own standard
        // deviations before the band verdict is given (3: a 0.3% false verdict)
        float band_sigmas = 3.0f;
        uint8_t sign_fail_count = 6;       // consecutive probe samples opposing the prior's sign that fail the axis
        // an actuator with a zero prior is never credited with more than this
        // fraction of the axis's own reference effectiveness: the regressors move
        // together in a coordinated manoeuvre and least squares splits the response
        // among them arbitrarily (a SITL plane's pitch row credited the rudder with
        // 88 rad/s^2 per unit against an elevator of 6; a quad's yaw row credited
        // roll with 48 against a yaw of 6). With motors as actuators every motor
        // yaws as much as the reference one, so the identifier's own bound is the
        // reference itself; a controller with axis commands as actuators sets it
        // lower (AP_INDI: half, no airframe is cross-coupled by more than that)
        float coupling_max = 1.0f;
        // the fast sign test reads a window only while the other actuators' strokes
        // sum to at most this fraction of the primary's own stroke, so the verdict
        // rests on the raw response and not on the coupling estimates: at half, with
        // couplings bounded to half of the reference, the other strokes account for
        // at most a quarter of the primary's predicted response, which is the test's
        // own margin. With motors as actuators every stroke moves all of them and
        // the test never applies; the slow test on the estimate carries the sign
        // verdict there
        float sign_other_frac = 0.5f;
    };

    enum class AxisState : uint8_t {
        PRIOR = 0,      // no accepted samples yet
        SIGN_OK = 1,    // sign of the primary ratio confirmed
        USABLE = 2,     // relative sigma below rel_sigma_usable
        CONVERGED = 3,  // relative sigma below rel_sigma_converged
        FAILED = 4,     // sign test failed or band left repeatedly
    };

    // why an axis entered AxisState::FAILED; the caller's response
    // differs (mirror the prior for SIGN, fall back for BAND)
    enum class FailReason : uint8_t {
        NONE = 0,
        SIGN = 1,       // primary ratio confidently negative
        BAND = 2,       // primary ratio clamped max_clamps times
    };

    AP_INDI_Ident() {}
    CLASS_NO_COPY(AP_INDI_Ident);

    // prior[axis][actuator] is the effectiveness prior in rad/s^2 per
    // unit command; prior_rel_sigma[axis] is how much it is trusted.
    // change the stroke gate after init (the command scale changed)
    void set_stroke_min(float du_min) { _cfg.du_min = du_min; }
    void init(uint8_t num_actuators,
              const float prior[NUM_AXES][MAX_ACTUATORS],
              const float prior_rel_sigma[NUM_AXES],
              const Config &cfg);

    // feed one increment sample for one axis. Returns true if the sample
    // was accepted and the estimate updated. probe says the stroke was
    // deliberate excitation rather than the loop's own reaction; only
    // probe samples may fail the sign test, since a loop countering a
    // disturbance also produces strokes that oppose the response.
    bool update(uint8_t axis, const float delta_u[MAX_ACTUATORS], float delta_omega_dot, bool saturated, bool probe = true);

    // restart an axis from its prior, optionally with the sign of the
    // prior mirrored (the sign-reversal response)
    void restart_axis(uint8_t axis, bool mirror);

    // switch all axes between the launch and cruise forgetting factors
    void set_cruise(bool cruise)
    {
        _cruise = cruise;
    }

    // current effectiveness estimate, rad/s^2 per unit command
    float get_effectiveness(uint8_t axis, uint8_t actuator) const;

    // relative standard deviation of the primary ratio on an axis
    float get_rel_sigma(uint8_t axis) const;

    // whether a cross-axis estimate is distinguishable from zero by the
    // sign-test margin (sign_margin_sigma standard deviations)
    bool coupling_significant(uint8_t axis, uint8_t actuator) const;

    // estimated measurement noise on an axis, rad/s^2
    float get_noise_sigma(uint8_t axis) const;

    AxisState get_state(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].state : AxisState::FAILED;
    }
    FailReason get_fail_reason(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].fail_reason : FailReason::NONE;
    }
    // the prior currently in use (mirrored after restart_axis(axis, true))
    float get_prior(uint8_t axis, uint8_t actuator) const
    {
        return (axis < NUM_AXES && actuator < _n) ? _axis[axis].prior[actuator] : 0;
    }
    uint8_t get_clamp_count(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].clamp_count : 0;
    }
    uint32_t get_accepted_count(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].accepted : 0;
    }
    uint8_t get_primary(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].primary : 0;
    }

private:
    struct Axis {
        float prior[MAX_ACTUATORS];          // effectiveness prior per actuator
        float theta[MAX_ACTUATORS];          // ratio state, see class comment
        float theta_raw;                     // primary ratio without the band clamp, for the band verdict
        // covariance of theta in UD factorised form, P = U D U': U is unit
        // upper triangular, of which only the part above the diagonal is
        // stored, and D is diagonal
        float U[MAX_ACTUATORS][MAX_ACTUATORS];
        float D[MAX_ACTUATORS];
        float P0_diag[MAX_ACTUATORS];        // initial covariance diagonal, cap on growth
        float g_ref;                         // largest |prior| on the axis
        float r;                             // estimated measurement noise variance, normalised units
        uint8_t primary;                     // actuator with the largest |prior|
        uint8_t clamp_count;
        uint8_t sign_disagree;               // consecutive samples whose response opposed the prior's sign
        uint32_t accepted;
        AxisState state;
        FailReason fail_reason;
    };

    void reset_axis(Axis &a, float prior_rel_sigma);
    void update_state(Axis &a);
    // diagonal element j of the covariance P = U D U'
    float covariance(const Axis &a, uint8_t j) const;
    // hold the diagonal of P between the floor and the cap
    void bound_covariance(Axis &a) const;

    Config _cfg;
    Axis _axis[NUM_AXES];
    float _prior_rel_sigma[NUM_AXES];
    uint8_t _n;
    bool _cruise;
};

#endif // AP_INDI_ENABLED
