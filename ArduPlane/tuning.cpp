// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/*
  tables of tuning sets
 */
const uint8_t AP_Tuning_Plane::tuning_set_q_rate_roll_pitch[] = { TUNING_Q_RATE_ROLL_KD, TUNING_Q_RATE_ROLL_KPI,
                                                                  TUNING_Q_RATE_PITCH_KD, TUNING_Q_RATE_PITCH_KPI};
const uint8_t AP_Tuning_Plane::tuning_set_q_rate_roll[] =       { TUNING_Q_RATE_ROLL_KD, TUNING_Q_RATE_ROLL_KPI };
const uint8_t AP_Tuning_Plane::tuning_set_q_rate_pitch[] =      { TUNING_Q_RATE_PITCH_KD, TUNING_Q_RATE_PITCH_KPI };

// macro to prevent getting the array length wrong
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

// list of tuning sets
const AP_Tuning_Plane::tuning_set AP_Tuning_Plane::tuning_sets[] = {
    { TUNING_SET_Q_RATE_ROLL_PITCH, TUNING_ARRAY(tuning_set_q_rate_roll_pitch) },
    { TUNING_SET_Q_RATE_ROLL,       TUNING_ARRAY(tuning_set_q_rate_roll) },
    { TUNING_SET_Q_RATE_PITCH,      TUNING_ARRAY(tuning_set_q_rate_pitch) },
    { TUNING_SET_NONE, 0, nullptr }
};

/*
  table of tuning names
 */
const AP_Tuning_Plane::tuning_name AP_Tuning_Plane::tuning_names[] = {
    { TUNING_Q_RATE_ROLL_KPI, "Q_RateRollPI" },
    { TUNING_Q_RATE_ROLL_KD,  "Q_RateRollD" },
    { TUNING_Q_RATE_PITCH_KPI,"Q_RatePitchPI" },
    { TUNING_Q_RATE_PITCH_KD, "Q_RatePitchD" },
    { TUNING_NONE, nullptr }
};

/*
  get a pointer to an AP_Float for a parameter, or NULL on fail
 */
AP_Float *AP_Tuning_Plane::get_param_pointer(uint8_t parm)
{
    switch (parm) {
    case TUNING_RLL_P:
        return &plane.rollController.kP();

    case TUNING_RLL_I:
        return &plane.rollController.kI();

    case TUNING_RLL_D:
        return &plane.rollController.kD();

    case TUNING_RLL_FF:
        return &plane.rollController.kFF();

    case TUNING_PIT_P:
        return &plane.pitchController.kP();

    case TUNING_PIT_I:
        return &plane.pitchController.kI();

    case TUNING_PIT_D:
        return &plane.pitchController.kD();

    case TUNING_PIT_FF:
        return &plane.pitchController.kFF();
        
    default:
        break;
    }
        
    if (!plane.quadplane.available()) {
        // quadplane tuning options not available
        return nullptr;
    }
    
    switch(parm) {

    case TUNING_Q_RATE_ROLL_KPI:
        // use P for initial value when tuning PI
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_Q_RATE_ROLL_KP:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_Q_RATE_ROLL_KI:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kI();

    case TUNING_Q_RATE_ROLL_KD:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kD();

    case TUNING_Q_RATE_PITCH_KPI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_Q_RATE_PITCH_KP:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_Q_RATE_PITCH_KI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kI();

    case TUNING_Q_RATE_PITCH_KD:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kD();

    case TUNING_Q_RATE_YAW_KPI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_Q_RATE_YAW_KP:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_Q_RATE_YAW_KI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kI();

    case TUNING_Q_RATE_YAW_KD:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kD();

    case TUNING_Q_ANG_ROLL_KP:
        return &plane.quadplane.attitude_control->get_angle_roll_p().kP();

    case TUNING_Q_ANG_PITCH_KP:
        return &plane.quadplane.attitude_control->get_angle_pitch_p().kP();

    case TUNING_Q_ANG_YAW_KP:
        return &plane.quadplane.attitude_control->get_angle_yaw_p().kP();

    case TUNING_Q_PXY_P:
        return &plane.quadplane.p_pos_xy.kP();

    case TUNING_Q_PZ_P:
        return &plane.quadplane.p_alt_hold.kP();

    case TUNING_Q_VXY_P:
        return &plane.quadplane.pi_vel_xy.kP();

    case TUNING_Q_VXY_I:
        return &plane.quadplane.pi_vel_xy.kI();

    case TUNING_Q_VZ_P:
        return &plane.quadplane.p_vel_z.kP();

    case TUNING_Q_AZ_P:
        return &plane.quadplane.pid_accel_z.kP();

    case TUNING_Q_AZ_I:
        return &plane.quadplane.pid_accel_z.kI();

    case TUNING_Q_AZ_D:
        return &plane.quadplane.pid_accel_z.kD();
        
    default:
        break;
    }
    return nullptr;
}


/*
  save a parameter
 */
void AP_Tuning_Plane::save_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_Q_RATE_ROLL_KPI:
        save_value(TUNING_Q_RATE_ROLL_KP);
        save_value(TUNING_Q_RATE_ROLL_KI);
        break;
    case TUNING_Q_RATE_PITCH_KPI:
        save_value(TUNING_Q_RATE_PITCH_KP);
        save_value(TUNING_Q_RATE_PITCH_KI);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->save();
        }
        break;
    }
}

/*
  set a parameter
 */
void AP_Tuning_Plane::set_value(uint8_t parm, float value)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_Q_RATE_ROLL_KPI:
        set_value(TUNING_Q_RATE_ROLL_KP, value);
        set_value(TUNING_Q_RATE_ROLL_KI, value);
        break;
    case TUNING_Q_RATE_PITCH_KPI:
        set_value(TUNING_Q_RATE_PITCH_KP, value);
        set_value(TUNING_Q_RATE_PITCH_KI, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->set_and_notify(value);
        }
        break;
    }
}

/*
  reload a parameter
 */
void AP_Tuning_Plane::reload_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_Q_RATE_ROLL_KPI:
        reload_value(TUNING_Q_RATE_ROLL_KP);
        reload_value(TUNING_Q_RATE_ROLL_KI);
        break;
    case TUNING_Q_RATE_PITCH_KPI:
        reload_value(TUNING_Q_RATE_PITCH_KP);
        reload_value(TUNING_Q_RATE_PITCH_KI);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->load();
        }
        break;
    }
}
