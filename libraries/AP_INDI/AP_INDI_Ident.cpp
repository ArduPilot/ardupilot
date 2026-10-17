#include "AP_INDI_Ident.h"

#if AP_INDI_ENABLED

#include <AP_Math/AP_Math.h>

// static constexpr members passed by reference (the MIN macro) need a
// definition outside the class under C++11
constexpr uint8_t AP_INDI_Ident::MAX_ACTUATORS;
constexpr uint8_t AP_INDI_Ident::NUM_AXES;

void AP_INDI_Ident::init(uint8_t num_actuators,
                         const float prior[NUM_AXES][MAX_ACTUATORS],
                         const float prior_rel_sigma[NUM_AXES],
                         const Config &cfg)
{
    _cfg = cfg;
    _n = MIN(num_actuators, MAX_ACTUATORS);
    _cruise = false;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        Axis &a = _axis[i];
        _prior_rel_sigma[i] = prior_rel_sigma[i];
        a.g_ref = 0;
        a.primary = 0;
        for (uint8_t j = 0; j < MAX_ACTUATORS; j++) {
            a.prior[j] = (j < _n) ? prior[i][j] : 0;
            if (fabsf(a.prior[j]) > a.g_ref) {
                a.g_ref = fabsf(a.prior[j]);
                a.primary = j;
            }
        }
        reset_axis(a, _prior_rel_sigma[i]);
    }
}

void AP_INDI_Ident::reset_axis(Axis &a, float prior_rel_sigma)
{
    a.clamp_count = 0;
    a.sign_disagree = 0;
    a.accepted = 0;
    a.state = AxisState::PRIOR;
    a.fail_reason = FailReason::NONE;
    a.r = sq(_cfg.noise_rel_init);
    for (uint8_t j = 0; j < MAX_ACTUATORS; j++) {
        for (uint8_t k = 0; k < MAX_ACTUATORS; k++) {
            a.U[j][k] = 0;
        }
        a.D[j] = 0;
        if (j >= _n || is_zero(a.g_ref)) {
            a.theta[j] = 0;
            a.P0_diag[j] = 0;
            continue;
        }
        if (!is_zero(a.prior[j])) {
            // ratio to a non-zero prior starts at exactly one
            a.theta[j] = 1.0f;
            a.theta_raw = 1.0f;
            a.P0_diag[j] = sq(prior_rel_sigma);
        } else {
            // no coupling expected, but allow it to be learned
            a.theta[j] = 0;
            a.P0_diag[j] = sq(_cfg.offdiag_rel_sigma);
        }
        a.D[j] = a.P0_diag[j];
    }
}

void AP_INDI_Ident::restart_axis(uint8_t axis, bool mirror)
{
    if (axis >= NUM_AXES) {
        return;
    }
    Axis &a = _axis[axis];
    if (mirror) {
        for (uint8_t j = 0; j < _n; j++) {
            a.prior[j] = -a.prior[j];
        }
    }
    reset_axis(a, _prior_rel_sigma[axis]);
}

bool AP_INDI_Ident::update(uint8_t axis, const float delta_u[MAX_ACTUATORS], float delta_omega_dot, bool saturated, bool probe)
{
    if (axis >= NUM_AXES || _n == 0) {
        return false;
    }
    Axis &a = _axis[axis];
    if (a.state == AxisState::FAILED || is_zero(a.g_ref) || saturated) {
        return false;
    }

    // stroke amplitude gate
    float du_norm_sq = 0;
    for (uint8_t j = 0; j < _n; j++) {
        du_norm_sq += sq(delta_u[j]);
    }
    if (du_norm_sq < sq(_cfg.du_min)) {
        return false;
    }

    // regressor and measurement scaled to order one
    float phi[MAX_ACTUATORS];
    for (uint8_t j = 0; j < _n; j++) {
        phi[j] = is_zero(a.prior[j]) ? delta_u[j] : delta_u[j] * a.prior[j] / a.g_ref;
    }
    const float y = delta_omega_dot / a.g_ref;

    float y_hat = 0;
    for (uint8_t j = 0; j < _n; j++) {
        y_hat += a.theta[j] * phi[j];
    }

    // once the fit is trusted, reject windows the command does not explain
    if (a.state >= AxisState::USABLE) {
        if (y * y_hat < 0 || fabsf(y_hat) < _cfg.explained_min * fabsf(y)) {
            return false;
        }
    }

    // fast sign test: the primary actuator moved by a clear stroke and the
    // axis accelerated the wrong way by a clear margin. The margin is the
    // configured noise floor and a quarter of the response the prior
    // predicts, not the running noise estimate: a wrong prior inflates
    // that estimate by design, which would mute this test exactly when
    // it is needed.
    // The raw response is judged, and only in a window the primary's own
    // stroke dominates: subtracting the other actuators' predicted share
    // made the verdict hinge on the coupling estimates, which a coordinated
    // manoeuvre can drive far from the truth (both false reversals seen in
    // SITL, a quad's yaw and a plane's pitch, were read through a coupling
    // of several times the axis's own effectiveness)
    const float phi_p = phi[a.primary];
    float phi_other = 0;
    for (uint8_t j = 0; j < _n; j++) {
        if (j != a.primary) {
            phi_other += fabsf(phi[j]);
        }
    }
    if (probe &&
        fabsf(phi_p) >= _cfg.du_min &&
        phi_other <= _cfg.sign_other_frac * fabsf(phi_p) &&
        fabsf(y) > _cfg.sign_resp_min &&
        fabsf(y) > 0.25f * fabsf(phi_p)) {
        if (y * phi_p < 0) {
            if (++a.sign_disagree >= _cfg.sign_fail_count) {
                a.state = AxisState::FAILED;
                a.fail_reason = FailReason::SIGN;
                return true;
            }
        } else {
            a.sign_disagree = 0;
        }
    }

    const float lambda = _cruise ? _cfg.lambda_cruise : _cfg.lambda_launch;

    // weighted RLS: k = P phi / (lambda r + phi' P phi), where r is the
    // measurement noise variance. Without r the covariance would only be
    // known up to the noise scale and the gain would be wrong.
    // P is held as P = U D U' and updated by Bierman's method (G. J.
    // Bierman, Factorization Methods for Discrete Sequential Estimation,
    // 1977, chapter V) with measurement variance lambda r, then D is
    // divided by lambda. That is exactly the recursion
    // P = (P - P phi phi' P / (lambda r + phi' P phi)) / lambda, but each
    // element of D is only ever scaled by a ratio of positive sums, so P
    // stays positive definite whatever the rounding, and the bounds keep
    // it so (bound_covariance()). Clamping the diagonal of P element by
    // element did not: under coordinated strokes P went indefinite and
    // every later sample was refused (SPEC.md, Numerics, f32).
    float f[MAX_ACTUATORS];  // U' phi
    float g[MAX_ACTUATORS];  // D U' phi
    float denom = lambda * a.r;
    for (uint8_t j = 0; j < _n; j++) {
        f[j] = phi[j];
        for (uint8_t i = 0; i < j; i++) {
            f[j] += a.U[i][j] * phi[i];
        }
        g[j] = a.D[j] * f[j];
        denom += f[j] * g[j];
    }
    // positive by construction; this refuses a non-finite regressor
    // before it reaches the state
    if (!is_positive(denom)) {
        return false;
    }
    // the update also accumulates P phi, with the covariance as it was
    // before this sample, for the gain
    float Pphi[MAX_ACTUATORS];
    float alpha = lambda * a.r;
    for (uint8_t j = 0; j < _n; j++) {
        const float alpha_prev = alpha;
        alpha += f[j] * g[j];
        a.D[j] *= alpha_prev / alpha;
        Pphi[j] = g[j];
        const float mu = -f[j] / alpha_prev;
        for (uint8_t i = 0; i < j; i++) {
            const float u = a.U[i][j];
            a.U[i][j] = u + Pphi[i] * mu;
            Pphi[i] += u * g[j];
        }
    }
    for (uint8_t j = 0; j < _n; j++) {
        a.D[j] /= lambda;
    }
    bound_covariance(a);

    const float innovation = y - y_hat;
    for (uint8_t j = 0; j < _n; j++) {
        a.theta[j] += (Pphi[j] / denom) * innovation;
        if (is_zero(a.prior[j])) {
            a.theta[j] = constrain_float(a.theta[j], -_cfg.coupling_max, _cfg.coupling_max);
        }
    }
    // the same update on the unclamped shadow of the primary ratio, with
    // the innovation taken against the shadow's own prediction: fed the
    // clamped estimate's innovation it would keep every clamp correction
    // as a permanent offset and fail an axis whose estimate sits inside
    // the band
    const float innovation_raw = innovation - (a.theta_raw - a.theta[a.primary]) * phi[a.primary];
    a.theta_raw += (Pphi[a.primary] / denom) * innovation_raw;

    // noise estimate from the product of the prior and posterior
    // residuals, which is unbiased for r whatever the parameter
    // uncertainty. Growth is limited to 4x per sample (a doubling of the
    // noise standard deviation) so a wrong prior is absorbed by the
    // parameters over several samples rather than written off as noise
    // in one step. The averaging window ramps up to noise_window, with
    // the initial guess given noise_prior_weight samples of weight: the
    // first residual of a flight is often a liftoff transient, and
    // letting it replace the guess outright hands the next window the
    // whole innovation.
    float e_post = y;
    for (uint8_t j = 0; j < _n; j++) {
        e_post -= a.theta[j] * phi[j];
    }
    const float r_sample = MAX(innovation * e_post, 0.0f);
    const uint32_t support = a.accepted + 1 + _cfg.noise_prior_weight;
    const float w = 1.0f / float(MIN(support, uint32_t(_cfg.noise_window)));
    a.r = MIN((1.0f - w) * a.r + w * r_sample, 4.0f * a.r);
    a.r = MAX(a.r, sq(_cfg.noise_rel_floor));

    // keep the primary ratio inside its band. A negative ratio is left
    // to the sign test rather than clamped up to the band, otherwise a
    // reversed actuator would be reported as merely weak. The band verdict
    // comes from the unclamped shadow: an airframe just inside an edge
    // has an estimate that rests on it half the time (0.3x of the prior
    // against a 0.25x edge, say) and must not be failed for that, while
    // one well outside drives the shadow past the edge by a clear margin
    // even after the noise estimate has absorbed the model error and the
    // clamped value no longer moves. No verdict before band_min_samples:
    // until then a single disturbed window can have put the shadow there.
    float &tp = a.theta[a.primary];
    if (tp > _cfg.band_hi || (tp >= 0 && tp < _cfg.band_lo)) {
        tp = constrain_float(tp, _cfg.band_lo, _cfg.band_hi);
        a.clamp_count++;
    } else if (tp < -_cfg.band_hi) {
        tp = -_cfg.band_hi;
    }
    // ... and the shadow must also sit beyond the edge by band_sigmas of
    // its own standard deviation: a fixed margin alone fails an axis whose
    // first few windows were noisy (the SITL quad's yaw, truth 0.5x of the
    // prior, was failed at the 0.25x edge 600 ms after launch on a shadow
    // that had not yet settled).
    // The verdict is only given in the launch phase: it judges the airframe
    // against its class on the launch evidence. In cruise the natural
    // windows carry the closed-loop bias, and a shadow that drifts to an
    // edge there (the SITL plane's pitch, 100 s into a flight it was
    // tracking well) is a fit gone stale, not a different airframe; the
    // clamp keeps the estimate in band, the inversion floor carries the
    // loop, and a degraded actuator is the divergence watch's case
    const bool judged = !_cruise;
    const float sigma = sqrtf(covariance(a, a.primary));
    if (judged && a.accepted >= _cfg.band_min_samples &&
        ((a.theta_raw > _cfg.band_hi * (1.0f + _cfg.band_margin) &&
          a.theta_raw - _cfg.band_hi > _cfg.band_sigmas * sigma) ||
         (a.theta_raw >= 0 && a.theta_raw < _cfg.band_lo * (1.0f - _cfg.band_margin) &&
          _cfg.band_lo - a.theta_raw > _cfg.band_sigmas * sigma))) {
        a.state = AxisState::FAILED;
        a.fail_reason = FailReason::BAND;
        return true;
    }

    a.accepted++;
    update_state(a);
    return true;
}

void AP_INDI_Ident::update_state(Axis &a)
{
    const float sigma = sqrtf(covariance(a, a.primary));
    const float tp = a.theta[a.primary];

    // sign test: the primary ratio must be positive by a clear margin.
    // A confident negative means the actuator acts the wrong way.
    if (tp < -_cfg.sign_margin_sigma * sigma) {
        a.state = AxisState::FAILED;
        a.fail_reason = FailReason::SIGN;
        return;
    }
    if (tp < _cfg.sign_margin_sigma * sigma) {
        a.state = AxisState::PRIOR;
        return;
    }
    const float rel_sigma = sigma / tp;
    if (rel_sigma <= _cfg.rel_sigma_converged) {
        a.state = AxisState::CONVERGED;
    } else if (rel_sigma <= _cfg.rel_sigma_usable) {
        a.state = AxisState::USABLE;
    } else {
        a.state = AxisState::SIGN_OK;
    }
}

float AP_INDI_Ident::covariance(const Axis &a, uint8_t j) const
{
    // P_jj = sum over k >= j of U_jk^2 D_k, with U_jj = 1
    float p = a.D[j];
    for (uint8_t k = j + 1; k < _n; k++) {
        p += sq(a.U[j][k]) * a.D[k];
    }
    return p;
}

void AP_INDI_Ident::bound_covariance(Axis &a) const
{
    // the floor: an estimate is never more certain than rel_sigma_floor.
    // Each short P_jj is raised by adding the shortfall to D_j, which adds
    // a positive semidefinite term to P. P_jj depends only on D_k for
    // k >= j, so working from the last element back each is set once
    const float floor_var = sq(_cfg.rel_sigma_floor);
    float diag[MAX_ACTUATORS] {};
    for (int8_t j = _n - 1; j >= 0; j--) {
        diag[j] = covariance(a, j);
        if (diag[j] < floor_var) {
            a.D[j] += floor_var - diag[j];
            diag[j] = floor_var;
        }
    }
    // the cap: the forgetting factor grows P in directions the data do not
    // excite, and the prior variance is the most it may grow to. P becomes
    // S P S with S diagonal, s_j = sqrt(cap_j / P_jj) where P_jj is over
    // the cap and one elsewhere, which keeps the correlations and positive
    // definiteness and in factorised form is D_j *= s_j^2,
    // U_ij *= s_i / s_j. The cap is never below the floor, which wins
    // where a prior variance is configured below it
    float s[MAX_ACTUATORS];
    bool capped = false;
    for (uint8_t j = 0; j < _n; j++) {
        const float cap = MAX(a.P0_diag[j], floor_var);
        s[j] = 1;
        if (diag[j] > cap) {
            s[j] = sqrtf(cap / diag[j]);
            capped = true;
        }
    }
    if (!capped) {
        return;
    }
    for (uint8_t j = 0; j < _n; j++) {
        a.D[j] *= sq(s[j]);
        for (uint8_t i = 0; i < j; i++) {
            a.U[i][j] *= s[i] / s[j];
        }
    }
}

bool AP_INDI_Ident::coupling_significant(uint8_t axis, uint8_t actuator) const
{
    if (axis >= NUM_AXES || actuator >= _n) {
        return false;
    }
    const Axis &a = _axis[axis];
    const float sigma = sqrtf(covariance(a, actuator));
    return fabsf(a.theta[actuator]) > _cfg.sign_margin_sigma * sigma;
}

float AP_INDI_Ident::get_effectiveness(uint8_t axis, uint8_t actuator) const
{
    if (axis >= NUM_AXES || actuator >= _n) {
        return 0;
    }
    const Axis &a = _axis[axis];
    if (is_zero(a.prior[actuator])) {
        return a.theta[actuator] * a.g_ref;
    }
    return a.theta[actuator] * a.prior[actuator];
}

float AP_INDI_Ident::get_noise_sigma(uint8_t axis) const
{
    if (axis >= NUM_AXES) {
        return 0;
    }
    return sqrtf(_axis[axis].r) * _axis[axis].g_ref;
}

float AP_INDI_Ident::get_rel_sigma(uint8_t axis) const
{
    if (axis >= NUM_AXES) {
        return 0;
    }
    const Axis &a = _axis[axis];
    const float tp = fabsf(a.theta[a.primary]);
    if (!is_positive(tp)) {
        return 0;
    }
    return sqrtf(covariance(a, a.primary)) / tp;
}

#endif // AP_INDI_ENABLED
