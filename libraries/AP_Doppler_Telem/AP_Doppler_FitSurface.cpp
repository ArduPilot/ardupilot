#include "AP_Doppler_FitSurface.h"

#include <cmath>

void AP_Doppler_FitSurface::BeamSample::init(float gamma, float beta)
{
    gamma_rad = gamma;
    beta_rad = beta;
    cos_gamma = cosf(gamma_rad);
    sin_gamma = sinf(gamma_rad);
    cos_beta  = cosf(beta_rad);
    sin_beta  = sinf(beta_rad);
    valid = false;
    distance_m = 0.0f;
    vec_sensor.zero();
}

void AP_Doppler_FitSurface::BeamSample::update_from_distance(float distance_m_in, bool valid_in)
{
    valid = valid_in;
    if (!valid || !is_positive(distance_m_in)) {
        valid = false;
        distance_m = 0.0f;
        vec_sensor.zero();
        return;
    }

    distance_m = distance_m_in;

    vec_sensor.x = distance_m * cos_gamma;
    vec_sensor.y = distance_m * sin_gamma * cos_beta;
    vec_sensor.z = distance_m * sin_gamma * sin_beta;
}

AP_Doppler_FitSurface::AP_Doppler_FitSurface(AP_Doppler_Telem& doppler,
                                             const BeamGeometry& beam_a,
                                             const BeamGeometry& beam_b,
                                             const BeamGeometry& beam_c,
                                             const BeamGeometry& beam_d,
                                             float max_residual_m,
                                             enum Rotation sensor_to_body_rot) :
    _doppler(doppler),
    _sensor_to_body_rot(sensor_to_body_rot),
    _max_residual_m(max_residual_m)
{
    _beam_a.init(beam_a.gamma_rad, beam_a.beta_rad);
    _beam_b.init(beam_b.gamma_rad, beam_b.beta_rad);
    _beam_c.init(beam_c.gamma_rad, beam_c.beta_rad);
    _beam_d.init(beam_d.gamma_rad, beam_d.beta_rad);

    reset_solution();
}

void AP_Doppler_FitSurface::reset_solution()
{
    _healthy = false;
    _beam_count = 0;
    _normal_sensor.zero();
    _normal_body.zero();
    _plane_d_sensor = 0.0f;
    _residual_m = 0.0f;
}

bool AP_Doppler_FitSurface::update()
{
    reset_solution();

    if (!read_beams()) {
        return false;
    }

    if (!fit_plane_from_valid_points()) {
        return false;
    }

    update_body_frame_outputs();
    return _healthy;
}

bool AP_Doppler_FitSurface::read_beams()
{
    DVL_U_Msg msg_a {};
    DVL_U_Msg msg_b {};
    DVL_U_Msg msg_c {};
    DVL_U_Msg msg_d {};

    //时间对齐？？？？？？？？？？？？？？？？？？？？？？？？？？？？

    const bool got_a = _doppler.get_ua_msg(msg_a);
    const bool got_b = _doppler.get_ub_msg(msg_b);
    const bool got_c = _doppler.get_uc_msg(msg_c);
    const bool got_d = _doppler.get_ud_msg(msg_d);

    _beam_a.update_from_distance(got_a ? msg_a.distance_m : 0.0f, got_a && msg_a.valid);
    _beam_b.update_from_distance(got_b ? msg_b.distance_m : 0.0f, got_b && msg_b.valid);
    _beam_c.update_from_distance(got_c ? msg_c.distance_m : 0.0f, got_c && msg_c.valid);
    _beam_d.update_from_distance(got_d ? msg_d.distance_m : 0.0f, got_d && msg_d.valid);

    _beam_count =
        (_beam_a.valid ? 1U : 0U) +
        (_beam_b.valid ? 1U : 0U) +
        (_beam_c.valid ? 1U : 0U) +
        (_beam_d.valid ? 1U : 0U);

    return (_beam_count >= 3U);
}

bool AP_Doppler_FitSurface::fit_plane_from_valid_points()
{
    Vector3f pts[4];
    uint8_t idx = 0;

    if (_beam_a.valid) { pts[idx++] = _beam_a.vec_sensor; }
    if (_beam_b.valid) { pts[idx++] = _beam_b.vec_sensor; }
    if (_beam_c.valid) { pts[idx++] = _beam_c.vec_sensor; }
    if (_beam_d.valid) { pts[idx++] = _beam_d.vec_sensor; }

    if (idx < 3U) {
        return false;
    }

    if (idx == 3U) {
        return fit_plane_3points(pts, idx);
    }

    return fit_plane_4points_ls(pts);
}

bool AP_Doppler_FitSurface::fit_plane_3points(const Vector3f* pts, uint8_t count)
{
    if (count != 3U) {
        return false;
    }

    const Vector3f v1 = pts[1] - pts[0];
    const Vector3f v2 = pts[2] - pts[0];
    Vector3f n = v1 % v2;

    const float n_len = n.length();
    if (!is_positive(n_len)) {
        return false;
    }

    n /= n_len;

    // 与你原始代码一致：约束法向量在传感器x方向为正
    if (n.x < 0.0f) {
        n = -n;
    }

    _normal_sensor = n;
    _plane_d_sensor = -(_normal_sensor * pts[0]);
    float d_error = fabs(_plane_d_sensor - _plane_d_last_sensor);
    _plane_d_last_sensor = _plane_d_sensor;
    if (d_error > AP_DOPPLER_FITSURFACE_DISTANCEERROR) {
        return false;
    }

    _healthy = true;
    return true;
}

bool AP_Doppler_FitSurface::fit_plane_4points_ls(const Vector3f* pts)
{
    const Vector3f centroid = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

    const Vector3f q0 = pts[0] - centroid;
    const Vector3f q1 = pts[1] - centroid;
    const Vector3f q2 = pts[2] - centroid;
    const Vector3f q3 = pts[3] - centroid;

    float C[3][3] {};
    const Vector3f q[4] = { q0, q1, q2, q3 };

    for (uint8_t i = 0; i < 4U; i++) {
        C[0][0] += q[i].x * q[i].x;
        C[0][1] += q[i].x * q[i].y;
        C[0][2] += q[i].x * q[i].z;

        C[1][0] += q[i].y * q[i].x;
        C[1][1] += q[i].y * q[i].y;
        C[1][2] += q[i].y * q[i].z;

        C[2][0] += q[i].z * q[i].x;
        C[2][1] += q[i].z * q[i].y;
        C[2][2] += q[i].z * q[i].z;
    }

    Vector3f n;
    if (!smallest_eigenvector_sym_3x3(C, n)) {
        return false;
    }

    if (n.x < 0.0f) {
        n = -n;
    }

    _normal_sensor = n;
    _plane_d_sensor = -(_normal_sensor * centroid);
    float d_error = fabs(_plane_d_sensor - _plane_d_last_sensor);
    _plane_d_last_sensor = _plane_d_sensor;
    if (d_error > AP_DOPPLER_FITSURFACE_DISTANCEERROR) {
        return false;
    }

    const float e0 = (_normal_sensor * pts[0]) + _plane_d_sensor;
    const float e1 = (_normal_sensor * pts[1]) + _plane_d_sensor;
    const float e2 = (_normal_sensor * pts[2]) + _plane_d_sensor;
    const float e3 = (_normal_sensor * pts[3]) + _plane_d_sensor;

    _residual_m = sqrtf((sq(e0) + sq(e1) + sq(e2) + sq(e3)) * 0.25f);

    _healthy = (_residual_m <= _max_residual_m);
    return _healthy;
}

void AP_Doppler_FitSurface::update_body_frame_outputs()
{
    _normal_body = _normal_sensor;
    _normal_body.rotate(_sensor_to_body_rot);

    const float nlen = _normal_body.length();
    if (!is_positive(nlen)) {
        _healthy = false;
        return;
    }
    _normal_body /= nlen;
}

bool AP_Doppler_FitSurface::smallest_eigenvector_sym_3x3(const float A_in[3][3], Vector3f& eigvec)
{
    float A[3][3];
    float V[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    for (uint8_t r = 0; r < 3U; r++) {
        for (uint8_t c = 0; c < 3U; c++) {
            A[r][c] = A_in[r][c];
        }
    }

    // Jacobi for symmetric 3x3
    for (uint8_t iter = 0; iter < 12U; iter++) {
        uint8_t p = 0;
        uint8_t q = 1;

        float max_offdiag = fabsf(A[0][1]);
        const float a02 = fabsf(A[0][2]);
        const float a12 = fabsf(A[1][2]);

        if (a02 > max_offdiag) {
            max_offdiag = a02;
            p = 0;
            q = 2;
        }
        if (a12 > max_offdiag) {
            max_offdiag = a12;
            p = 1;
            q = 2;
        }

        if (max_offdiag < 1.0e-6f) {
            break;
        }

        const float app = A[p][p];
        const float aqq = A[q][q];
        const float apq = A[p][q];

        const float tau = (aqq - app) / (2.0f * apq);
        const float t = (tau >= 0.0f) ?
                        (1.0f / (tau + sqrtf(1.0f + tau * tau))) :
                        (-1.0f / (-tau + sqrtf(1.0f + tau * tau)));
        const float c = 1.0f / sqrtf(1.0f + t * t);
        const float s = t * c;

        // rotate A
        for (uint8_t k = 0; k < 3U; k++) {
            if (k == p || k == q) {
                continue;
            }

            const float Akp = A[k][p];
            const float Akq = A[k][q];

            A[k][p] = c * Akp - s * Akq;
            A[p][k] = A[k][p];

            A[k][q] = s * Akp + c * Akq;
            A[q][k] = A[k][q];
        }

        const float new_app = c * c * app - 2.0f * s * c * apq + s * s * aqq;
        const float new_aqq = s * s * app + 2.0f * s * c * apq + c * c * aqq;

        A[p][p] = new_app;
        A[q][q] = new_aqq;
        A[p][q] = 0.0f;
        A[q][p] = 0.0f;

        // rotate V
        for (uint8_t k = 0; k < 3U; k++) {
            const float Vkp = V[k][p];
            const float Vkq = V[k][q];

            V[k][p] = c * Vkp - s * Vkq;
            V[k][q] = s * Vkp + c * Vkq;
        }
    }

    uint8_t min_idx = 0;
    if (A[1][1] < A[min_idx][min_idx]) {
        min_idx = 1;
    }
    if (A[2][2] < A[min_idx][min_idx]) {
        min_idx = 2;
    }

    eigvec.x = V[0][min_idx];
    eigvec.y = V[1][min_idx];
    eigvec.z = V[2][min_idx];

    const float len = eigvec.length();
    if (!is_positive(len)) {
        return false;
    }

    eigvec /= len;
    return true;
}
