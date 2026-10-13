#include "AP_INDI_Ident.h"

#if AP_INDI_ENABLED

#include <AP_Math/AP_Math.h>

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
    a.accepted = 0;
    a.state = AxisState::PRIOR;
    a.r = sq(_cfg.noise_rel_init);
    for (uint8_t j = 0; j < MAX_ACTUATORS; j++) {
        for (uint8_t k = 0; k < MAX_ACTUATORS; k++) {
            a.P[j][k] = 0;
        }
        if (j >= _n || is_zero(a.g_ref)) {
            a.theta[j] = 0;
            a.P0_diag[j] = 0;
            continue;
        }
        if (!is_zero(a.prior[j])) {
            // ratio to a non-zero prior starts at exactly one
            a.theta[j] = 1.0f;
            a.P0_diag[j] = sq(prior_rel_sigma);
        } else {
            // no coupling expected, but allow it to be learned
            a.theta[j] = 0;
            a.P0_diag[j] = sq(_cfg.offdiag_rel_sigma);
        }
        a.P[j][j] = a.P0_diag[j];
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

bool AP_INDI_Ident::update(uint8_t axis, const float delta_u[MAX_ACTUATORS], float delta_omega_dot, bool saturated)
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

    const float lambda = _cruise ? _cfg.lambda_cruise : _cfg.lambda_launch;

    // weighted RLS: k = P phi / (lambda r + phi' P phi), where r is the
    // measurement noise variance. Without r the covariance would only be
    // known up to the noise scale and the gain would be wrong.
    float Pphi[MAX_ACTUATORS];
    float phiPphi = 0;
    for (uint8_t j = 0; j < _n; j++) {
        Pphi[j] = 0;
        for (uint8_t k = 0; k < _n; k++) {
            Pphi[j] += a.P[j][k] * phi[k];
        }
        phiPphi += phi[j] * Pphi[j];
    }
    const float innovation = y - y_hat;

    const float denom = lambda * a.r + phiPphi;
    if (!is_positive(denom)) {
        return false;
    }
    for (uint8_t j = 0; j < _n; j++) {
        a.theta[j] += (Pphi[j] / denom) * innovation;
    }

    // noise estimate from the product of the prior and posterior
    // residuals, which is unbiased for r whatever the parameter
    // uncertainty. Growth is limited to 4x per sample (a doubling of the
    // noise standard deviation) so a wrong prior is absorbed by the
    // parameters over several samples rather than written off as noise
    // in one step. The averaging window ramps from 1 to noise_window so
    // the pessimistic initial guess is replaced quickly.
    float e_post = y;
    for (uint8_t j = 0; j < _n; j++) {
        e_post -= a.theta[j] * phi[j];
    }
    const float r_sample = MAX(innovation * e_post, 0.0f);
    const float w = 1.0f / float(MIN(a.accepted + 1, uint32_t(_cfg.noise_window)));
    a.r = MIN((1.0f - w) * a.r + w * r_sample, 4.0f * a.r);
    a.r = MAX(a.r, sq(_cfg.noise_rel_floor));

    // P = (P - k phi' P) / lambda, with the result kept symmetric and bounded.
    // TODO: move to a UD factorised update (spec section 9) before flight use.
    for (uint8_t j = 0; j < _n; j++) {
        for (uint8_t k = j; k < _n; k++) {
            float v = (a.P[j][k] - Pphi[j] * Pphi[k] / denom) / lambda;
            if (j == k) {
                v = constrain_float(v, sq(_cfg.rel_sigma_floor), a.P0_diag[j]);
            }
            a.P[j][k] = v;
            a.P[k][j] = v;
        }
    }

    // keep the primary ratio inside its band
    float &tp = a.theta[a.primary];
    if (tp < _cfg.band_lo || tp > _cfg.band_hi) {
        tp = constrain_float(tp, _cfg.band_lo, _cfg.band_hi);
        a.clamp_count++;
        if (a.clamp_count >= _cfg.max_clamps) {
            a.state = AxisState::FAILED;
            return true;
        }
    }

    a.accepted++;
    update_state(a);
    return true;
}

void AP_INDI_Ident::update_state(Axis &a)
{
    const float sigma = sqrtf(a.P[a.primary][a.primary]);
    const float tp = a.theta[a.primary];

    // sign test: the primary ratio must be positive by a clear margin.
    // A confident negative means the actuator acts the wrong way.
    if (tp < -_cfg.sign_margin_sigma * sigma) {
        a.state = AxisState::FAILED;
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
    return sqrtf(a.P[a.primary][a.primary]) / tp;
}

#endif // AP_INDI_ENABLED
