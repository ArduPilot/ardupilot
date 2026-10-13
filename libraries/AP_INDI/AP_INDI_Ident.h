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
        float noise_rel_init = 0.1f;       // initial guess of measurement noise, as a fraction of g_ref
        float noise_rel_floor = 0.005f;    // floor on the estimated measurement noise
        uint8_t noise_window = 50;         // samples over which the noise estimate averages
        uint8_t max_clamps = 2;            // clamps on one axis before it is marked failed
    };

    enum class AxisState : uint8_t {
        PRIOR = 0,      // no accepted samples yet
        SIGN_OK = 1,    // sign of the primary ratio confirmed
        USABLE = 2,     // relative sigma below rel_sigma_usable
        CONVERGED = 3,  // relative sigma below rel_sigma_converged
        FAILED = 4,     // sign test failed or band left repeatedly
    };

    AP_INDI_Ident() {}
    CLASS_NO_COPY(AP_INDI_Ident);

    // prior[axis][actuator] is the effectiveness prior in rad/s^2 per
    // unit command; prior_rel_sigma[axis] is how much it is trusted.
    void init(uint8_t num_actuators,
              const float prior[NUM_AXES][MAX_ACTUATORS],
              const float prior_rel_sigma[NUM_AXES],
              const Config &cfg);

    // feed one increment sample for one axis. Returns true if the sample
    // was accepted and the estimate updated.
    bool update(uint8_t axis, const float delta_u[MAX_ACTUATORS], float delta_omega_dot, bool saturated);

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

    // estimated measurement noise on an axis, rad/s^2
    float get_noise_sigma(uint8_t axis) const;

    AxisState get_state(uint8_t axis) const
    {
        return axis < NUM_AXES ? _axis[axis].state : AxisState::FAILED;
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
        float P[MAX_ACTUATORS][MAX_ACTUATORS]; // covariance of theta
        float P0_diag[MAX_ACTUATORS];        // initial covariance diagonal, cap on growth
        float g_ref;                         // largest |prior| on the axis
        float r;                             // estimated measurement noise variance, normalised units
        uint8_t primary;                     // actuator with the largest |prior|
        uint8_t clamp_count;
        uint32_t accepted;
        AxisState state;
    };

    void reset_axis(Axis &a, float prior_rel_sigma);
    void update_state(Axis &a);

    Config _cfg;
    Axis _axis[NUM_AXES];
    float _prior_rel_sigma[NUM_AXES];
    uint8_t _n;
    bool _cruise;
};

#endif // AP_INDI_ENABLED
