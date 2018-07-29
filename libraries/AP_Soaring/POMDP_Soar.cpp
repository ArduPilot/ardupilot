// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include <GCS_MAVLink/GCS.h>
#include "AP_Soaring.h"
#include "POMDP_Soar.h"

// ArduSoar parameters
const AP_Param::GroupInfo POMDSoarAlgorithm::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the POMDSoar algorithm on?
    // @Description: If 1, the soaring controller uses the POMDSoar algorithm. If 0, the soaring controller uses the ArduSoar algorithm.
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, POMDSoarAlgorithm, pomdp_on, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: N
    // @DisplayName: Number of samples per action trajectory used by POMDSoar
    // @Description: Number of samples per action trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("N", 2, POMDSoarAlgorithm, pomdp_n, 10),

    // @Param: K
    // @DisplayName: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Description: Number of POMDP sample points per 1 second of an action's trajectory used by POMDSoar.
    // @Units: samples
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("K", 3, POMDSoarAlgorithm, pomdp_k, 5),

    // @Param: HORI
    // @DisplayName: POMDP planning horizon used by POMDSoar.
    // @Description: POMDP planning horizon used by POMDSoar.
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("HORI", 4, POMDSoarAlgorithm, pomdp_hori, 4.0),

    // @Param: STEP_T
    // @DisplayName:POMDP planning step solve time
    // @Description: The amount of computation time the POMDP solver has for computing the next action
    // @Units: seconds
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("STEP", 5, POMDSoarAlgorithm, pomdp_step_t, 1),

    // @Param: LOOP
    // @DisplayName: Number of POMDP solver's inner loop executions per planning step
    // @Description: Number of POMDP solver's inner loop executions per planning step (see also the STEP_T parameter)
    // @Units:
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("LOOP", 6, POMDSoarAlgorithm, pomdp_loop_load, 1),

    // @Param: ROLL1
    // @DisplayName: POMDP's maximum commanded roll angle.
    // @Description: Maximum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("ROLL1", 7, POMDSoarAlgorithm, pomdp_roll1, 15),

    // @Param: ROLL2
    // @DisplayName: POMDP's minimum commanded roll angle.
    // @Description: Minimum commanded roll angle in the POMDP used by POMDSoar.
    // @Units: degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("ROLL2", 8, POMDSoarAlgorithm, pomdp_roll2, 45),

    // @Param: RRATE
    // @DisplayName: The sailplane UAV's roll rate increment used by POMDSoar
    // @Description: The sailplane UAV's roll rate increment used by POMDSoar.
    // @Units: degrees/second
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("RRATE", 9, POMDSoarAlgorithm, pomdp_roll_rate, 75),

    // @Param: N_ACT
    // @DisplayName: POMDP number of actions
    // @Description: Number of actions in the POMDP used by POMDSoar. The roll angle input commands corresponding to actions are endpoints of (N_ACT-1) equal intervals between ROLL2 and ROLL1 (inclusive).
    // @Units: seconds
    // @Range: 1 254
    // @User: Advanced
    AP_GROUPINFO("N_ACT", 10, POMDSoarAlgorithm, pomdp_n_actions, 2),

    // @Param: I_MOMENT
    // @DisplayName: I-moment coefficient
    // @Description: Airframe-specific I-moment coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("I_MOM", 11, POMDSoarAlgorithm, I_moment, 0.00257482),

    // @Param: K_AILERON
    // @DisplayName: Aileron K coefficient
    // @Description: Airframe-specific aileron K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units: seconds
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("K_AIL", 12, POMDSoarAlgorithm, k_aileron, 1.44833047),

    // @Param: K_ROLLDAMP
    // @DisplayName: Roll dampening K coefficient
    // @Description: Airframe-specific roll-dampening K coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("RLLDMP", 13, POMDSoarAlgorithm, k_roll_damping, 0.41073589),

    // @Param: ROLL_CLP
    // @DisplayName: CLP coefficient
    // @Description: Airframe-specific CLP coefficient used by POMDSoar to model the trajectory corresponding to a given commanded roll angle.
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("RLLCLP", 14, POMDSoarAlgorithm, c_lp, -1.12808702679),

    // @Param: POLY_A
    // @DisplayName: Sink polynomial coefficient a
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_A", 15, POMDSoarAlgorithm, poly_a, -0.03099261),

    // @Param: POLY_B
    // @DisplayName: Sink polynomial coefficient b
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_B", 16, POMDSoarAlgorithm, poly_b, 0.44731854),

    // @Param: POLY_C
    // @DisplayName: Sink polynomial coefficient c
    // @Description: a*x^2 + b*x + c sink polynomial for netto vario correction
    // @Units:
    // @Range: -10000 10000
    // @User: Advanced
    AP_GROUPINFO("POLY_C", 17, POMDSoarAlgorithm, poly_c, -2.30292972),

    // @Param: TH
    // @DisplayName: POMDSoar's threshold on tr(P) for switching between explore and max-lift modes.
    // @Description: POMDSoar's threshold on the P matrix trace for switching between explore and max-lift modes.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("PTH", 18, POMDSoarAlgorithm, pomdp_pth, 50),

    // @Param: NORM
    // @DisplayName: Normalize the P matrix trace when solving the POMDP
    // @Description: Normalize the trace of the P matrix used for switching between explore and max-lift modes in POMDSoar. 0 = no normalizing, 1 = normalize tr(P)
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("NORM", 19, POMDSoarAlgorithm, pomdp_norm_pth, 0),

    // @Param: EXT
    // @DisplayName: Enable action duration extension in POMDSoar's max-lift mode compared to the explore mode
    // @Description: 0 = off, > 1 = multiplicative factor by which to extend action duration in max-lift compared to the explore mode.
    // @Units:
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("EXT", 20, POMDSoarAlgorithm, pomdp_extend, 0),

    // @Param: PLN
    // @DisplayName: Enable deterministic trajectory planning mode for the POMDP
    // @Description: Enable deterministic trajectory planning mode for the POMDP. 0 = off, 1 on.
    // @Units:
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("PLN", 21, POMDSoarAlgorithm, pomdp_plan_mode, 0),

    AP_GROUPEND
};

POMDSoarAlgorithm::POMDSoarAlgorithm(const SoaringController *sc, AP_RollController &rollController, AP_Float &scaling_speed) :
    _sc(sc),
    _gains(rollController.get_gains()),
    _scaling_speed(scaling_speed)
{
    AP_Param::setup_object_defaults(this, var_info);
    _prev_pomdp_update_time = AP_HAL::micros64();
}


void POMDSoarAlgorithm::init_actions(POMDP_Mode mode)
{
    _n_actions = MIN(pomdp_n_actions, MAX_ACTIONS);
    float max_roll = fmax(pomdp_roll1 * _sign, pomdp_roll2 * _sign);
    float min_roll = fmin(pomdp_roll1 * _sign, pomdp_roll2 * _sign);

    switch (mode) {
        case POMDP_MODE_EXPLOIT:
        {
            float new_max_roll = _pomdp_roll_cmd + pomdp_roll_rate;
            float new_min_roll = _pomdp_roll_cmd - pomdp_roll_rate;

            if (new_max_roll > max_roll)
            {
                new_max_roll = max_roll;
                new_min_roll = max_roll - 2 * pomdp_roll_rate;
            }

            if (new_min_roll < min_roll)
            {
                new_min_roll = min_roll;
                new_max_roll = min_roll + 2 * pomdp_roll_rate;
            }

            float roll = new_max_roll;
            float roll_rate = (new_max_roll - new_min_roll) / (_n_actions - 1);
            for (int i = 0; i < _n_actions; i++) {
                _roll_cmds[i] = roll;
                //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
                roll -= roll_rate;
            }
        }
        case POMDP_MODE_EXPLORE:
        {
            float roll = max_roll;
            float roll_rate = (max_roll - min_roll) / (_n_actions - 1);
            for (int i = 0; i < _n_actions; i++)
            {
                _roll_cmds[i] = roll;
                //gcs().send_text(MAV_SEVERITY_INFO, "Action[%d] %f", i, (double)_roll_cmds[i]);
                roll -= roll_rate;
            }
        }
    }

    _prev_n_actions = _n_actions;
}


void POMDSoarAlgorithm::init_thermalling()
{
    Vector2f ground_vector = _sc->_ahrs.groundspeed_vector();
    float head_sin = ground_vector.y / ground_vector.length();
    float head_cos = ground_vector.x / ground_vector.length();

    float xprod = _sc->_ekf.X[3] * head_cos - _sc->_ekf.X[2] * head_sin;
    _sign = xprod <= 0 ? -1.0 : 1.0;
    _pomdp_roll_cmd = pomdp_roll1 * _sign;
    init_actions(POMDP_MODE_EXPLORE);
    float aspd = _sc->get_aspd();
    float eas2tas = _sc->get_eas2tas();
    // This assumes that SoaringController called get_position(_prev_update_location) right before this call to init_thermalling
    _pomdp_wp = _sc->_prev_update_location;
    _sc->get_relative_position_wrt_home(_pomdp_vecNE);
    //printf("_pomdp_wp = %f %f\n", ((double)_pomdp_wp.lat) * 1e-7, ((double)_pomdp_wp.lng) * 1e-7);
    float hdx, hdy;
    _sc->get_heading_estimate(&hdx, &hdy);
    float wind_corrected_heading = atan2f(hdy, hdx);

    // Prepare everything necessary for generating action trajectories (by modelling trajectories resulting from various commanded roll angles)
    _solver.set_pid_gains(_gains.P, _gains.I, _gains.D, _gains.FF, _gains.tau, _gains.imax, _gains.rmax, _scaling_speed);
    _solver.set_polar(float(poly_a), float(poly_b), float(poly_c));
    _n_action_samples = pomdp_hori * pomdp_k;

    _solver.generate_action_paths(aspd, eas2tas, wind_corrected_heading, degrees(_sc->get_roll()),
        degrees(_sc->get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
        pomdp_step_t, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), 0);
    _m = 0;
    _solver.init_step(pomdp_loop_load, pomdp_n, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, false);
    // This assumes that SoaringController updated _thermal_start_time_us right before this call to init_thermalling
    _prev_pomdp_update_time = _sc->_thermal_start_time_us;
    _prev_pomdp_wp = _sc->_prev_update_location;
    _pomdp_active = true;
    _pomdp_mode = POMDP_MODE_EXPLORE;
    _prev_pomdp_action = _sign > 0 ? _n_actions - 1 : 0;
}


float POMDSoarAlgorithm::assess_thermalability(uint8_t exit_mode)
{
    float thermalability = -1e6;
    float aspd = _sc->get_aspd();

    if (exit_mode == 1)
    {
        float expected_thermalling_sink = _sc->correct_netto_rate(0.0f, radians(_pomdp_roll_cmd), aspd);
        float dist_sqr = _sc->_ekf.X[2] * _sc->_ekf.X[2] + _sc->_ekf.X[3] * _sc->_ekf.X[3];
        thermalability = (_sc->_ekf.X[0] * expf(-dist_sqr / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink;
    }
    else if (exit_mode == 2)
    {
        int n_samps = 20;
        float theta_step = fmax(pomdp_roll1, pomdp_roll2) / (float)n_samps;
        float theta = 0;

        for (int i = 0; i < n_samps; i++)
        {
            float expected_thermalling_sink = _sc->correct_netto_rate(0.0f, radians(theta), aspd);
            float r = (aspd * aspd) / (GRAVITY_MSS * tanf(radians(theta)));
            thermalability = MAX(thermalability, (_sc->_ekf.X[0] * expf(-(r*r) / powf(_sc->_ekf.X[1], 2))) - expected_thermalling_sink);
            theta += theta_step;
        }
    }

    return thermalability;
}


bool POMDSoarAlgorithm::are_computations_in_progress()
{
    return _pomdp_active;
}


void POMDSoarAlgorithm::update_solver()
{
    if (_solver.running())
    {
        uint64_t start_time = AP_HAL::micros64();
        _solver.update();
        _pomp_loop_time = AP_HAL::micros64() - start_time;
        //gcs().send_text(MAV_SEVERITY_INFO, "slice time: %lluus  samps: %d", _pomp_loop_time, samps);
        _pomp_loop_min_time = (_pomp_loop_min_time > _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_min_time;
        _pomp_loop_max_time = (_pomp_loop_max_time < _pomp_loop_time) ? _pomp_loop_time : _pomp_loop_max_time;
        _pomp_loop_av_time = _pomp_loop_av_time * 0.9 + _pomp_loop_time * 0.1;
    }
}


void POMDSoarAlgorithm::update_solver_test()
{
    _solver.update_test();
}


void POMDSoarAlgorithm::stop_computations()
{
    _pomdp_active = false;
}


bool POMDSoarAlgorithm::is_set_to_continue_past_thermal_locking_period()
{
    return (pomdp_pth > 0);
}


int POMDSoarAlgorithm::get_curr_mode()
{
    return _pomdp_mode;
}


uint64_t POMDSoarAlgorithm::get_latest_pomdp_solve_time()
{
    return _pomdp_solve_time;
}


float POMDSoarAlgorithm::get_action()
{
    return _pomdp_roll_cmd * 100;
}


void POMDSoarAlgorithm::send_test_out_msg(mavlink_channel_t chan)
{
    /*
    0: pomdp_wp.lat (y0)
    4: pomdp_wp.lng (x0)
    8:  v0
    10: psi0
    12: x1,y1,psi1,x2,y2,psi2,x3,y3,psi3,x4,y4,psi4
    36: x1,y1,psi1,x2,y2,psi2,x3,y3,psi3,x4,y4,psi4
    60: a1
    61: a2
    62-63: 2 bytes spare
    */
    // uint8_t *_debug_out_bytes = (uint8_t *)&_debug_out[0];
    // uint8_t *a1 = &_debug_out_bytes[60];
    // uint8_t *a2 = &_debug_out_bytes[61];

    // if (_pomdp_active && _m < _solver.actions_generated())
    // {
    //     _debug_out_mode = 0;
    //     int32_t *pos_ptr = (int32_t *)(&_debug_out[0]);
    //     pos_ptr[0] = _pomdp_wp.lat;
    //     pos_ptr[1] = _pomdp_wp.lng;

    //     int16_t *v0 = (int16_t *)&_debug_out_bytes[8];
    //     *v0 = (int16_t)(_solver.get_action_v0() * 256);

    //     int16_t *psi0 = (int16_t *)&_debug_out_bytes[10];
    //     *psi0 = (int16_t)(_solver.get_action_path_psi(0, 0) * 256);

    //     int16_t *action1 = (int16_t *)&_debug_out_bytes[12];
    //     int16_t *action2 = (int16_t *)&_debug_out_bytes[36];

    //     int k_step = _n_action_samples / 4;
    //     *a1 = _m;
    //     int k = k_step;

    //     for (int i = 0; i < 12; i += 3)
    //     {
    //         action1[i] = (int16_t)(_solver.get_action_path_x(_m, k) * 256);
    //         action1[i + 1] = (int16_t)(_solver.get_action_path_y(_m, k) * 256);
    //         action1[i + 2] = (int16_t)(_solver.get_action_path_psi(_m, k) * 256);
    //         k += k_step;
    //     }

    //     _m++;

    //     if (_m < _n_actions && _m < _solver.actions_generated())
    //     {
    //         *a2 = _m;
    //         k = k_step;

    //         for (int i = 0; i < 12; i += 3) {
    //             action2[i] = (int16_t)(_solver.get_action_path_x(_m, k) * 256);
    //             action2[i + 1] = (int16_t)(_solver.get_action_path_y(_m, k) * 256);
    //             action2[i + 2] = (int16_t)(_solver.get_action_path_psi(_m, k) * 256);
    //             k += k_step;
    //         }
    //     }
    //     else
    //     {
    //         *a2 = 255;
    //     }

    //     _m++;

    //     if (_m >= _n_actions)
    //     {
    //         _m = 0;
    //     }
    // }
    // else
    // {
    //     _debug_out_mode = 0;
    //     *a1 = 255; // action = 255 to signal no more action data
    //     *a2 = 255; // action = 255 to signal no more action data
    // }

    // mavlink_msg_soar_test_out_send(
    //     chan,
    //     _debug_out_mode,
    //     _debug_out); //<field name = "debug" type = "float[16]">Debug results< / field>
}


void POMDSoarAlgorithm::update_thermalling(const Location &current_loc)
{
    if (!_solver.running())
    {
        _pomdp_solve_time = AP_HAL::micros64() - _prev_pomdp_update_time;

        float eas2tas = _sc->get_eas2tas();
        float aspd = _sc->get_aspd();
        uint8_t action = (uint8_t)_solver.get_best_action();
        _pomdp_roll_cmd = _roll_cmds[action];

        _sc->get_relative_position_wrt_home(_pomdp_vecNE);
        float hdx, hdy;
        _sc->get_heading_estimate(&hdx, &hdy);
        float wind_corrected_heading = atan2f(hdy, hdx);
        //gcs().send_text(MAV_SEVERITY_INFO, "head %f %f %f", hdx, hdy, wind_corrected_heading);
        float n[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

        // Normalise the trace of covariance matrix, if required.
        if (pomdp_norm_pth)
        {
            n[0] = fabsf(_sc->_ekf.X[0]) > 0.0f ? fabsf(_sc->_ekf.X[0]) : 1.0f;
            n[1] = fabsf(_sc->_ekf.X[1]) > 0.0f ? fabsf(_sc->_ekf.X[1]) : 1.0f;
            n[2] = n[1];
            n[3] = n[1];
        }

        float trP = _sc->_ekf.P(0, 0) / n[0] + _sc->_ekf.P(1, 1) / n[1] + _sc->_ekf.P(2, 2) / n[2] + _sc->_ekf.P(3, 3) / n[3];

        // Update the correct mode (exploit vs explore) based on EKF covariance trace
        _pomdp_mode = trP < pomdp_pth && pomdp_pth > 0.0f ? POMDP_MODE_EXPLOIT : POMDP_MODE_EXPLORE;
        
        // Initialise actions accordingly.
        init_actions(_pomdp_mode);

        // Determine appropriate number of action samples. Planning horizon in seconds times times samples/second.
        _n_action_samples = pomdp_hori * pomdp_k;

        if (_pomdp_mode==POMDP_MODE_EXPLOIT)
        {
            // Extend horizon in exploitation mode.
            _n_action_samples = MIN(MAX_ACTION_SAMPLES, int(pomdp_hori * pomdp_k * pomdp_extend));
        }

        //  Determine appropriate number of samples.
        //  pompd_n = "Number of samples per action trajectory" - how is this different from n_action_samples?
        int n_samples = pomdp_n;
        float step_w = 1.0f;

        if (_pomdp_mode==POMDP_MODE_EXPLOIT && pomdp_plan_mode)
        {
            // "Enable deterministic trajectory planning mode for the POMDP"
            n_samples = 1;
            step_w = 1.0f / pomdp_n;
        }

        _solver.generate_action_paths(aspd, eas2tas, wind_corrected_heading, degrees(_sc->get_roll()), degrees(_sc->get_rate()), _pomdp_roll_cmd, pomdp_k, _n_actions, _roll_cmds,
            pomdp_step_t * step_w, pomdp_hori, float(I_moment), float(k_aileron), float(k_roll_damping), float(c_lp), pomdp_extend);
        _m = 0;
        _solver.init_step(pomdp_loop_load, n_samples, _sc->_ekf.X, _sc->_ekf.P, _sc->_ekf.Q, _sc->_ekf.R, _weights, _pomdp_mode==POMDP_MODE_EXPLOIT);

        _prev_pomdp_update_time = AP_HAL::micros64();
        _prev_pomdp_action = action;

        DataFlash_Class::instance()->Log_Write("POMP", "TimeUS,n,k,action,x,y,sign,lat,lng,rlat,rlng,roll,mode,Q", "QQHHBfffLLLLfB",
            AP_HAL::micros64(),
            pomdp_n,
            pomdp_k,
            action,
            (double)_pomdp_vecNE.y,
            (double)_pomdp_vecNE.x,
            (double)_sign,
            current_loc.lat,
            current_loc.lng,
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_pomdp_roll_cmd,
            (uint8_t)_pomdp_mode,
            (double)_solver.get_action_Q(action));
        DataFlash_Class::instance()->Log_Write("POMT", "TimeUS,loop_min,loop_max,loop_time,solve_time,load,n,k", "QQQQQQHHH",
            AP_HAL::micros64(),
            _pomp_loop_min_time,
            _pomp_loop_max_time,
            _pomp_loop_time,
            _pomdp_solve_time,
            pomdp_loop_load,
            pomdp_n,
            pomdp_k
        );
        DataFlash_Class::instance()->Log_Write("PDBG", "TimeUS,lat,lng,v0,psi,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6", "QLLffffffffffffff",
            AP_HAL::micros64(),
            _pomdp_wp.lat,
            _pomdp_wp.lng,
            (double)_solver.get_action_v0(),
            (double)_solver.get_action_path_psi(action, 0),
            (double)_solver.get_action_path_x(action, 1),
            (double)_solver.get_action_path_y(action, 1),
            (double)_solver.get_action_path_x(action, 2),
            (double)_solver.get_action_path_y(action, 2),
            (double)_solver.get_action_path_x(action, 3),
            (double)_solver.get_action_path_y(action, 3),
            (double)_solver.get_action_path_x(action, 4),
            (double)_solver.get_action_path_y(action, 4),
            (double)_solver.get_action_path_x(action, 5),
            (double)_solver.get_action_path_y(action, 5),
            (double)_solver.get_action_path_x(action, 6),
            (double)_solver.get_action_path_y(action, 6)
        );

        _pomdp_wp = current_loc;
        _prev_pomdp_wp = current_loc;
        _pomp_loop_min_time = (unsigned long)1e12;
        _pomp_loop_max_time = 0;
    }
}

bool POMDSoarAlgorithm::ok_to_stop()
{
    return (!_solver.running() && is_set_to_continue_past_thermal_locking_period());
}


void POMDSoarAlgorithm::run_tests()
{
    int8_t test_id = 1;
    uint64_t start_time = AP_HAL::micros64();
    uint64_t total_time   = 0;
    uint64_t total_time_2 = 0;

    switch (test_id) {
    case 1:
        _solver.run_exp_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring exp test: %llu us", total_time);
        break;
    case 2:
        _solver.run_fast_exp_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fast exp test: %llu us", total_time);
        break;
    case 3:
        _solver.fill_random_array();
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %llu us", total_time);
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring fill rnd array: %llu us", total_time);
        break;
    case 4:
        _solver.run_rnd_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring rnd test: %llu us", total_time);
        break;
    case 5:
        _solver.run_ekf_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring EKF test: %llu us", total_time);
        break;
    case 6:
        _solver.run_loop_test(1000, false);
        total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_loop_test(1000, true);
        total_time_2 = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring loop test: %llu us ML: %llu us", total_time, total_time_2);
        break;
    case 7:
        _solver.run_multivariate_normal_sample_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring multivariate_normal test: %llu us", total_time);
        break;
    case 8:
        // test #8 is run by the SoaringController class itself
        break;
    case 9:
        _solver.run_trig_box_muller_test(1000);
        total_time = AP_HAL::micros64() - start_time;
        start_time = AP_HAL::micros64();
        _solver.run_polar_box_muller_test(1000);
        total_time_2 = AP_HAL::micros64() - start_time;
        gcs().send_text(MAV_SEVERITY_INFO, "Soaring box-muller trig: %llu us polar: %llu us", total_time, total_time_2);
        break;
    }

    _prev_run_timing_test = test_id;
}