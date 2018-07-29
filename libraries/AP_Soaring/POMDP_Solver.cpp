// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "POMDP_Solver.h"
#include <GCS_MAVLink/GCS.h>

#define EKF_FAST_MATH

#ifdef EKF_FAST_MATH
static union
{
    float d;
    int i;
} _eco;
#define EXP_A 12102203 /* int(1<<23/math.log(2)) */
#define EXP_C 0 /* see text for choice of c values */

// Adapted from Schraudolph, "A Fast, Compact Approximation if the Exponential Function",
// Tech Report UDSIA-07-98
#define fastexp(y) (_eco.i = EXP_A*(y)+(1065353216 - EXP_C), _eco.d)
/*
in the above fastexp macro:
values of x around -88 to -89 can result in NaN,
values below about -89 are not valid:
x                                    hex value                                 hex value
-88.0    exp(x) = 6.0546014852e-39   41edc4      fastexp(x) = 5.0357061614e-40     57bc0
-88.5    exp(x) = 3.6723016101e-39   27fce2      fastexp(x) = NaN               ffa92680
-89.0    exp(x) = 2.2273639090e-39   1840fc      fastexp(x) = -2.7225029733e+38 ff4cd180
so we check that x is than 88 to avoid this.
(Note we also assume here that x is always negative, which is the case when used in a gaussian)
*/

#define EXP(x) ( (x) > -88.0f ? fastexp(x) : 0.0 )
#else
#define EXP(x) expf(x)
#endif

#define fastarctan(x) ( M_PI_4*(x) - (x)*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x)) )

PomdpSolver::PomdpSolver()
{
    fill_random_array();
    _i_ptr = 0;
    _s_ptr = 0;
}


// This as mostly a cut and paste of the _get_rate_out() from AP_RollController.cpp
// so that we can model the aileron PID controller
float PomdpSolver::_get_rate_out(float dt, float aspeed, float eas2tas, float achieved_rate, float desired_rate)
{
    // Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
    float ki_rate = gains.I * gains.tau;
    float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau - gains.D, 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    // Limit the demanded roll rate
    if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = -gains.rmax;
    }
    else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }
#pragma GCC diagnostic pop
    float scaler = 2.0;

    if (aspeed > 0.0001f)
    {
        scaler = _scaling_speed / aspeed;
    }
    scaler = constrain_float(scaler, 0.5f, 2.0f);
    float rate_error = (desired_rate - achieved_rate) * scaler;

    // Multiply roll rate error by _ki_rate, apply scaler and integrate
    // Scaler is applied before integrator so that integrator state relates directly to aileron deflection
    // This means aileron trim offset doesn't change as the value of scaler changes with airspeed
    if (ki_rate > 0) 
    {
        // only integrate if airspeed above min value.
        if (aspeed > float(_aparm.airspeed_min))
        {
            float integrator_delta = rate_error * ki_rate * dt * scaler;
            // prevent the integrator from increasing if surface defln demand is above the upper limit
            if (_last_out < -45)
            {
                integrator_delta = MAX(integrator_delta, 0);
            }
            else if (_last_out > 45)
            {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                integrator_delta = MIN(integrator_delta, 0);
            }
            _pid_I += integrator_delta;
        }
    }
    else
    {
        _pid_I = 0;
    }

    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_I = constrain_float(_pid_I, -intLimScaled, intLimScaled);

    // Calculate the demanded control surface deflection
    // Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
    // path, but want a 1/speed^2 scaler applied to the rate error path. 
    // This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_D = rate_error * gains.D * scaler;
    _pid_P = desired_rate * kp_ff * scaler;
    _pid_FF = desired_rate * k_ff * scaler;
    _pid_desired = desired_rate;

    _last_out = _pid_FF + _pid_P + _pid_D + _pid_I;

    //  constrain
    return constrain_float(_last_out, -45, 45);
}


void PomdpSolver::set_pid_gains(float P, float I, float D, float FF, float tau, float imax, float rmax, float scaling_speed)
{
    gains.P = P;
    gains.I = I;
    gains.D = D;
    gains.FF = FF;
    gains.tau = tau;
    gains.imax = imax;
    gains.rmax = rmax;
    _scaling_speed = scaling_speed;
}


void PomdpSolver::set_polar(float poly_a, float poly_b, float poly_c)
{
    _poly_a = poly_a;
    _poly_b = poly_b;
    _poly_c = poly_c;
}


void PomdpSolver::generate_action_paths(float v0, float eas2tas, float psi0, float roll0, float roll_rate0, float current_action, int pomdp_k, int nactions, float* action,
    float t_step, float t_hori, float I_moment, float k_aileron, float k_roll_damping, float c_lp, int extend)
{
    _v0 = v0;
    int k = int(t_hori * pomdp_k);
    _t_hori = t_hori;
    _t_step = t_step;

    if (extend > 1)
    {
        k = int(extend * t_hori * pomdp_k);

        if (k > MAX_ACTION_SAMPLES)
        {
            k = MAX_ACTION_SAMPLES;
            _t_hori = k / (float)pomdp_k;
        }
        else
        {
            _t_hori = extend * t_hori;
        }
        _t_step = extend * t_step;
    }

    _actions = action;
    _eas2tas = eas2tas;
    _psi0 = psi0;
    _roll0 = roll0;
    _roll_rate0 = roll_rate0;
    _I_moment = I_moment;
    _k_aileron = k_aileron;
    _k_roll_damping = k_roll_damping;
    _c_lp = c_lp;
    _extend = extend;
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "gact k:%d", k);
    _k = k;
    _nactions = nactions;
    _prev_action = current_action;
}

void PomdpSolver::generate_action(int m, float v0, float eas2tas, float psi0, float roll0, float roll_rate0, float current_action, int k, int nactions, float* action,
    float t_step, float t_hori, float I_moment, float k_aileron, float k_roll_damping, float c_lp, int j0, int j1)
{
    const int rate_x = 10;
    const float g = -9.80665;
    float dt = t_hori / (k * rate_x);
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "gact %f %f %f %f %f %f", v0, psi0, roll0, roll_rate0, action[0], action[1]);
    float px = 0.0f;
    float py = 0.0f;
    float psi = psi0;
    float theta_cmd = current_action;
    float theta = roll0;
    float theta_rate = roll_rate0;
    float t = dt;

    if (j0 == 0)
    {
        _pid_I = 0;
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "])", m + 1);
        _action_path_x[m][0] = px;
        _action_path_y[m][0] = py;
        _action_path_psi[m][0] = psi;
        _action_path_theta[m][0] = theta;
    }
    else {
        px = _action_path_x[m][j0];
        py = _action_path_y[m][j0];
        psi = _action_path_psi[m][j0];
        theta = _action_path_theta[m][j0];
        theta_rate = _theta_rate;
        t = _t;
    }

    for (int j = j0; j < k && j < j1; j++)
    {	
        for (int i = 0; i < rate_x; i++)
        {
            if (t > t_step)
            {
                theta_cmd = action[m];
            }
            else
            {
                theta_cmd = current_action;
            }

            float C_lp = -c_lp * (theta_rate) / (2 * v0);
            float desired_rate = (theta_cmd - theta) / gains.tau;
            float aileron_out = _get_rate_out(dt, v0, eas2tas, theta_rate, desired_rate) / 45.0f;
            float theta_acc = (aileron_out * k_aileron - k_roll_damping * C_lp) / I_moment;
            theta_rate += theta_acc * dt;
            theta += theta_rate * dt;
            psi -= dt * (g * tanf((theta * M_PI) / 180.0f) / v0);
            px += dt * v0 * sinf(psi);
            py += dt * v0 * cosf(psi);
            t += dt;
        }

        _action_path_x[m][j + 1] = px;
        _action_path_y[m][j + 1] = py;
        _action_path_psi[m][j + 1] = psi;
        _action_path_theta[m][j + 1] = theta;
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%3u j:%2d m:%2d %f %f %f %f", _slice_count, j+1, m, px, py, theta_cmd, t);
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "[%f, %f],", px,py);
    }
        
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "a%d = np.array([", m + 1);
    _theta_rate = theta_rate;
    _t = t;
    _log_j = 0; //start logging new actions
    _new_actions = true;
}


void PomdpSolver::log_actions(uint64_t thermal_id)
{
    if (_new_actions && _log_j < _k + 1)
    {
        for (int m = 0; m < _nactions; m++) 
        {
            DataFlash_Class::instance()->Log_Write("POMA", "TimeUS,id,m,j,x,y,roll",
                "QQBBfff",
                AP_HAL::micros64(),
                thermal_id,
                m, _log_j,
                (double)_action_path_x[m][_log_j],
                (double)_action_path_y[m][_log_j],
                (double)_action_path_theta[m][_log_j]
            );
        }
        _log_j++;
    }
    else
    {
        _new_actions = false;
    }
}


void PomdpSolver::init_step(int max_loops, int n,
    const VectorN<float, 4> &x0, const MatrixN<float, 4> &p0, const MatrixN<float, 4> &q0, float r0,
    float weights[4], bool max_lift)
{
    _n = n;
    _inv_n = 1.0f / (float)n;

    for (int i = 0; i < 4; i++)
    {
        _x0[i] = x0[i];
        _weights[i] = weights[i];	
    }

    _p0 = p0;
    _q0 = q0;
    _r0 = r0;
    cholesky44(_p0.getarray(),_chol_p0);
    _max_lift = max_lift;
    _dt = _t_hori / ((float)_k);
    _k_t_step = int(_t_step / _dt);
    _therm_x = _x0[3];
    _therm_y = _x0[2];
    _best_action = 0;
    _i = 0;
    _j = 0;
    _m = 0;
    _Q[0] = 0;
    _running = true;
    _loop = 0;
    _max_loops = max_loops;
    _generate_actions = true;
    _start_action_loop = true;
    _start_sample_loop = true;
    _slice_count = 0;
}


float PomdpSolver::sink_polar(float aspd, float poly_a, float poly_b, float poly_c, float roll)
{
    float netto_rate;
    float phi = (roll * M_PI) / 180.0f;
    float cosphi;
    cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
    netto_rate = (poly_a * aspd* aspd + poly_b * aspd + poly_c) / cosphi;
    return netto_rate;
}


void PomdpSolver::inner_loop()
{
    float px1 = _action_path_x[_m][_j];
    float py1 = _action_path_y[_m][_j];
    float rx = px1 - _x;
    float ry = py1 - _y;
    float z = _w * EXP(-(rx * rx + ry * ry) / (_r * _r));

    if (_max_lift)
    {
        _total_lift += z + sink_polar(_v0, _poly_a, _poly_b, _poly_c, _action_path_theta[_m][_j]);
    }

    _ekf.update(z, py1 - _py0, px1 - _px0);
    _px0 = px1;
    _py0 = py1;
}


void PomdpSolver::sample_loop()
{
    float s[4];
    if (_n > 1)
    {
        multivariate_normal(s, _mean, _chol_p0);
        samp_count++;
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Sample: %f %f %f %f", s[0], s[1], s[2], s[3]);
        _w = _x0[0] + s[0];
        _r = _x0[1] + s[1];
        _x = _therm_x + s[3]; // Note: state vector index 3 = East = x 
        _y = _therm_y + s[2]; // Note: state vector index 2 = North = y
    }
    else {
        _w = _x0[0];
        _r = _x0[1];
        _x = _therm_x;
        _y = _therm_y;
    }

    _total_lift = 0;
    _px0 = 0;
    _py0 = 0;

    _ekf.reset(_x0, _p0, _q0, _r0);
}


void PomdpSolver::action_loop()
{
    if (_max_lift)
    {
        _Q[_m] = 0;
    }
    else
    {
        _Q[_m] = 0;
    }

    if (_n <= 1) {
        _s[0][0] = _x0[0];
        _s[0][1] = _x0[1];
        _s[0][2] = 0;
        _s[0][3] = 0;
    }
}


int PomdpSolver::update()
{
    samp_count = 0;
    _slice_count++;
    _solve_time = AP_HAL::micros64();
    if (_generate_actions)
    {
        if (_m >= _nactions)
        {
            _generate_actions = false;
            _m = 0;
            return 0;
        }

        int j1 = MIN(_j + ACTION_GENERATION_STEPS_PER_LOOP, _k);
        generate_action(_m, _v0, _eas2tas, _psi0, _roll0, _roll_rate0, _prev_action, _k, _nactions, _actions,
            _t_step, _t_hori, _I_moment, _k_aileron, _k_roll_damping, _c_lp, _j, j1);
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%lluus %3u loop:%6lluus j:%2d m:%2d", _solve_time, _slice_count, AP_HAL::micros64() - _solve_time, _j, _m);
        _j += ACTION_GENERATION_STEPS_PER_LOOP;
        if (_j >= _k)
        {
            _j = 0;
            _m++;

            if (_m >= _nactions)
            {
                _generate_actions = false;
                _m = 0;
            }
        }
        
        return 0;
    }

    if (_start_action_loop)
    {
        // action loop init
        action_loop();
        _start_action_loop = false;
    }

    if (_start_sample_loop)
    {
        // sample loop init
        sample_loop();
        _start_sample_loop = false;
    }
    
    _loop = 0;
    while (_loop < _max_loops)
    {
        // inner loop body
        inner_loop();
        // GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%02u loop:%02d i:%02d j:%02d m:%02d ip%03u sp:%03u", _slice_count, _loop, _i, _j, _m, _i_ptr, _s_ptr);
        _loop++;
        _j++;
        
        if (_j >= _k)
        {
            if (_max_lift)
            {
                // maximizing lift = minimizing the negative of the lift
                _Q[_m] += - _total_lift; // *_inv_n;
            }
            else
            {
                _Q[_m] += (_weights[0] * _ekf.P(0,0)
                           + _weights[1] * _ekf.P(1,1)
                           + _weights[2] * _ekf.P(2,2)
                           + _weights[3] * _ekf.P(3,3)) * 1.0 / _n;
            }
            
            _j = 0;
            _i++;
            
            if (_i >= _n)
            {
                
                if (_max_lift && _Q[_m] < _Q[_best_action])
                {
                    _best_action = _m;
                }
                else if (_Q[_m] < _Q[_best_action])
                {
                    _best_action = _m;
                }
                
                _i = 0;
                _m++;
                
                if (_m >= _nactions)
                {
                    _running = false;
                    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Solver finished: %d %lluus", _slice_count, AP_HAL::micros64() - _solve_time);
                    _solve_time = AP_HAL::micros64();
                    return samp_count;
                }
                
                action_loop();
            }
            
            sample_loop();
        }
    }

    return samp_count;
}


void PomdpSolver::run_exp_test(unsigned n)
{
    for(unsigned i=0; i < n; i++)
    {
        _dummy[0] = expf(_s[i % MAX_GAUSS_SAMPLES][0]);
        _dummy[1] = expf(_s[i % MAX_GAUSS_SAMPLES][1]);
        _dummy[2] = expf(_s[i % MAX_GAUSS_SAMPLES][2]);
        _dummy[3] = expf(_s[i % MAX_GAUSS_SAMPLES][3]);
    }
}


void PomdpSolver::run_fast_exp_test(unsigned n)
{
    for(unsigned i=0; i < n; i++)
    {
        _dummy[0] = EXP(_s[i % MAX_GAUSS_SAMPLES][0]);
        _dummy[1] = EXP(_s[i % MAX_GAUSS_SAMPLES][1]);
        _dummy[2] = EXP(_s[i % MAX_GAUSS_SAMPLES][2]);
        _dummy[3] = EXP(_s[i % MAX_GAUSS_SAMPLES][3]);
    }
}


void PomdpSolver::fill_random_array()
{
    float cov[4][4];
    float mean[4] = { 0, 0, 0, 0 };
    cov[0][0] = 1;
    cov[1][1] = 1;
    cov[2][2] = 1;
    cov[3][3] = 1;
    multivariate_normal_fill(_s, mean, cov, MAX_GAUSS_SAMPLES);
}


void PomdpSolver::run_rnd_test(unsigned n)
{
    for (unsigned i = 0; i < n; i++)
    {
        xorshift128();
    }
}


void PomdpSolver::run_multivariate_normal_sample_test(unsigned n)
{
    float L[4][4];
    float mean[4] = { 0, 0, 0, 0 };
    L[0][0] = 1;
    L[1][1] = 1;
    L[2][2] = 1;
    L[3][3] = 1;
    float s[4];

    for (unsigned i = 0; i < n; i++) {
        multivariate_normal(s,mean,L);
    }
}


void PomdpSolver::run_trig_box_muller_test(unsigned n)
{
    float y1, y2;
    for (unsigned i = 0; i < n; i++) {
        trig_box_muller(&y1,&y2);
    }
}


void PomdpSolver::run_polar_box_muller_test(unsigned n)
{
    float y1, y2;

    for (unsigned i = 0; i < n; i++)
    {
        polar_box_muller(&y1,&y2);
    }
}


void PomdpSolver::run_ekf_test(unsigned n)
{
    VectorN<float, 4> X = (const float[]) {2.5,100,0,0};
    MatrixN<float, 4> P = (const float[]) { 1,100,1000,1000 };
    MatrixN<float, 4> Q = (const float[]) { 0.0025,1,2,2};
    float R = 0.024;
    _ekf.reset(X,P,Q,R);

    for (unsigned i = 0; i < n; i++)
    {
        _ekf.update(0.1, 1.0, 2.0);
    }
}


void PomdpSolver::run_loop_test(unsigned n, bool max_lift)
{
    VectorN<float, 4> X = (const float[]) { 2.5, 100, 0, 0 };
    MatrixN<float, 4> P = (const float[]) { 1, 100, 1000, 1000 };
    MatrixN<float, 4> Q = (const float[]) { 0.0025, 1, 2, 2 };
    float R = 0.024;
    _ekf.reset(X, P, Q, R);
    _w = X[0];
    _r = X[1];
    _y = X[2];
    _x = X[3];
    _max_lift = max_lift;
    _m = 0;
    _j = 0;
    _action_path_x[_m][_j] = 1.0;
    _action_path_y[_m][_j] = 2.0;

    for (unsigned i = 0; i < n; i++)
    {
        inner_loop();
        _px0 = 0;
        _py0 = 0;
    }
}


void PomdpSolver::update_random_buffer(unsigned n, MatrixN<float, 4> &cov, bool reset)
{
    unsigned div = MIN(MAX_GAUSS_SAMPLES - _s_ptr, n);
    unsigned rem = n - div;
    float mean[4] = { 0, 0, 0, 0 };
    MatrixN<float, 4> &p = cov;

    if (_running) {
        p = _p0;
    }

    multivariate_normal_fill(_s, mean, p.getarray(), div, _s_ptr);
    
    if (rem > 0) {
        multivariate_normal_fill(_s, mean, p.getarray(), rem, 0);
    }
    
    if (reset) {
        _i_ptr = _s_ptr;
    }

    _s_ptr = (_s_ptr + n) % MAX_GAUSS_SAMPLES;
}

void PomdpSolver::update_test() {
    update_test_counter++;
}


