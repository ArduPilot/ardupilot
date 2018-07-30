// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "ExtendedKalmanFilter.h"
#include "random.h"
#include <AP_Math/matrixN.h>


#define MAX_ACTIONS 10
#define MAX_ACTION_SAMPLES 100
#define ACTION_GENERATION_STEPS_PER_LOOP 7
class PomdpSolver
{
    struct {
        float airspeed_min;
    } _aparm;

    struct {
        float P;
        float I;
        float D;
        float FF;
        float tau;
        float imax;
        float rmax;
    } gains;

    float _scaling_speed;
    ExtendedKalmanFilter _ekf;
    float _Q[MAX_ACTIONS]; // quality of action
    float _s[MAX_GAUSS_SAMPLES][4]; // Gaussian samples
    unsigned _s_ptr = 0;
    unsigned _i_ptr = 0;
    float _action_path_x[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_y[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_psi[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    float _action_path_theta[MAX_ACTIONS][MAX_ACTION_SAMPLES + 1];
    int _prev_action;
    int _n_sample; // number of samples per action trajectory
    int _n_step; // number of action steps (_i_step)
    int _n_actions; // numer of actions (_i_action)
    float _action[MAX_ACTIONS];
    float _t_step;
    float _t_hori;
    VectorN<float, 4> _x0;
    MatrixN<float, 4> _p0;
    MatrixN<float, 4> _q0;
    float _r0;
    float _chol_p0[4][4];
    float _weights[4];
    float _mean[4] = { 0, 0, 0, 0 };
    bool _mode_exploit;
    float _dt;
    int _k_t_step;
    float _therm_x;
    float _therm_y;
    int _best_action;
    float _v0;
    float _poly_a;
    float _poly_b;
    float _poly_c;
    float _total_lift;
    float _px0;
    float _py0;
    float _px1;
    float _py1;
    int _i_sample; // Index of the current sample
    int _i_step;   // Index of the current action step
    int _i_action; // Index of the current action
    float _w ;
    float _r ;
    float _x ;
    float _y ;
    bool _running = false;
    int _max_loops;
    bool _start_sample_loop;
    bool _start_action_loop;
    float _pid_P;
    float _pid_I;
    float _pid_D;
    float _pid_FF;
    float _pid_desired;
    float _last_out;
    int _log_j;
    bool _new_actions = false;
    void inner_loop();
    void sample_loop();
    void action_loop();
    float _get_rate_out(float dt, float aspeed, float eas2tas, float achieved_rate, float desired_rate);
    float _dummy[4];
    int _slice_count = 0;
    uint64_t _solve_time = 0;
    float _eas2tas;
    float _psi0;
    float _roll0;
    float _roll_rate0;
    float _I_moment;
    float _k_aileron;
    float _k_roll_damping;
    float _c_lp;
    int _extend;
    float* _actions;
    bool _generate_actions = false;
    float _theta_rate;
    float _t;
    

public:
    PomdpSolver(); 
    void set_pid_gains(float P, float I, float D, float FF, float tau, float imax, float rmax, float scaling_speed);
    void set_polar(float poly_a, float poly_b, float poly_c);
    void generate_action_paths(float v0, float eas2tas, float psi0, float roll0, float roll_rate0, float current_action, int pomdp_k, int nactions, float* action,
        float t_step, float t_hori, float I_moment, float k_aileron, float k_roll_damping, float c_lp, int extend);
    void generate_action(int i_action, float v0, float eas2tas, float psi0, float roll0, float roll_rate0, float current_action, int pomdp_k, float* action,
        float t_step, float t_hori, float I_moment, float k_aileron, float k_roll_damping, float c_lp, int step_start, int step_end);
    void log_actions(uint64_t thermal_id);
    float sink_polar(float aspd, float poly_a, float poly_b, float poly_c, float roll);
    void init_step(int max_loops, int n,const VectorN<float, 4> &x0, 
                   const MatrixN<float, 4> &p0, const MatrixN<float, 4> &q0, float r0,
                   float weights[4], bool max_lift);
    void update();
    int get_best_action() { return _best_action;  }
    float get_action_path_x(int a, int k) { return _action_path_x[a][k]; }
    float get_action_path_y(int a, int k) { return _action_path_y[a][k]; }
    float get_action_path_theta(int a, int k) { return _action_path_theta[a][k]; }
    float get_action_path_psi(int a, int k) { return _action_path_psi[a][k]; }
    float get_action_v0() { return _v0; }
    float get_action_Q(int i) { return _Q[i]; }

    bool running() { return _running;  }
    void run_exp_test(unsigned n);
    void run_fast_exp_test(unsigned n);
    void fill_random_array();
    void run_rnd_test(unsigned n);
    void run_multivariate_normal_sample_test(unsigned n);
    void run_trig_box_muller_test(unsigned n);
    void run_polar_box_muller_test(unsigned n);
    void run_ekf_test(unsigned n);
    void run_loop_test(unsigned n, bool max_lift);
    void update_random_buffer(unsigned n, MatrixN<float, 4> &cov, bool reset);
    void update_test();
    unsigned update_test_counter = 0;
    int actions_generated() { return _generate_actions ? _i_action : _n_actions; }
};