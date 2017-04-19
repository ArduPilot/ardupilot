#include "Plane.h"

/*
  the vehicle class has its own var table for TUNE_PARAM so it can
  have separate parameter docs for the list of available parameters
 */
const AP_Param::GroupInfo AP_Tuning_Plane::var_info[] = {
    // @Param: PARAM
    // @DisplayName: Transmitter tuning parameter or set of parameters
    // @Description: This sets which parameter or set of parameters will be tuned. Values greater than 100 indicate a set of parameters rather than a single parameter. Parameters less than 50 are for QuadPlane vertical lift motors only.
    // @Values: 0:None,1:RateRollPI,2:RateRollP,3:RateRollI,4:RateRollD,5:RatePitchPI,6:RatePitchP,7:RatePitchI,8:RatePitchD,9:RateYawPI,10:RateYawP,11:RateYawI,12:RateYawD,13:AngleRollP,14:AnglePitchP,15:AngleYawP,16:PosXYP,17:PosZP,18:VelXYP,19:VelXYI,20:VelZP,21:AccelZP,22:AccelZI,23:AccelZD,50:FixedWingRollP,51:FixedWingRollI,52:FixedWingRollD,53:FixedWingRollFF,54:FixedWingPitchP,55:FixedWingPitchI,56:FixedWingPitchD,57:FixedWingPitchFF,101:Set_RateRollPitch,102:Set_RateRoll,103:Set_RatePitch,104:Set_RateYaw,105:Set_AngleRollPitch,106:Set_VelXY,107:Set_AccelZ
    // @User: Standard
    AP_GROUPINFO("PARAM", 1, AP_Tuning_Plane, parmset, 0),

    // the rest of the parameters are from AP_Tuning
    AP_NESTEDGROUPINFO(AP_Tuning, 0),

    AP_GROUPEND
};


/*
  tables of tuning sets
 */
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll_pitch[] = { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI,
                                                                TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI};
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll[] =       { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_pitch[] =      { TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_yaw[] =        { TUNING_RATE_YAW_P, TUNING_RATE_YAW_I, TUNING_RATE_YAW_D };
const uint8_t AP_Tuning_Plane::tuning_set_ang_roll_pitch[] =  { TUNING_ANG_ROLL_P, TUNING_ANG_PITCH_P };
const uint8_t AP_Tuning_Plane::tuning_set_vxy[] =             { TUNING_VXY_P, TUNING_VXY_I };
const uint8_t AP_Tuning_Plane::tuning_set_az[] =              { TUNING_AZ_P, TUNING_AZ_I, TUNING_AZ_D };

// macro to prevent getting the array length wrong
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

// list of tuning sets
const AP_Tuning_Plane::tuning_set AP_Tuning_Plane::tuning_sets[] = {
    { TUNING_SET_RATE_ROLL_PITCH, TUNING_ARRAY(tuning_set_rate_roll_pitch) },
    { TUNING_SET_RATE_ROLL,       TUNING_ARRAY(tuning_set_rate_roll) },
    { TUNING_SET_RATE_PITCH,      TUNING_ARRAY(tuning_set_rate_pitch) },
    { TUNING_SET_RATE_YAW,        TUNING_ARRAY(tuning_set_rate_yaw) },
    { TUNING_SET_ANG_ROLL_PITCH,  TUNING_ARRAY(tuning_set_ang_roll_pitch) },
    { TUNING_SET_VXY,             TUNING_ARRAY(tuning_set_vxy) },
    { TUNING_SET_AZ,              TUNING_ARRAY(tuning_set_az) },
    { 0, 0, nullptr }
};

/*
  table of tuning names
 */
const AP_Tuning_Plane::tuning_name AP_Tuning_Plane::tuning_names[] = {
    { TUNING_RATE_ROLL_PI, "RateRollPI" },
    { TUNING_RATE_ROLL_P,  "RateRollP" },
    { TUNING_RATE_ROLL_I,  "RateRollI" },
    { TUNING_RATE_ROLL_D,  "RateRollD" },
    { TUNING_RATE_PITCH_PI,"RatePitchPI" },
    { TUNING_RATE_PITCH_P, "RatePitchP" },
    { TUNING_RATE_PITCH_I, "RatePitchI" },
    { TUNING_RATE_PITCH_D, "RatePitchD" },
    { TUNING_RATE_YAW_PI,  "RateYawPI" },
    { TUNING_RATE_YAW_P,   "RateYawP" },
    { TUNING_RATE_YAW_I,   "RateYawI" },
    { TUNING_RATE_YAW_D,   "RateYawD" },
    { TUNING_ANG_ROLL_P,   "AngRollP" },
    { TUNING_ANG_PITCH_P,  "AngPitchP" },
    { TUNING_ANG_YAW_P,    "AngYawP" },
    { TUNING_PXY_P,        "PXY_P" },
    { TUNING_PZ_P,         "PZ_P" },
    { TUNING_VXY_P,        "VXY_P" },
    { TUNING_VXY_I,        "VXY_I" },
    { TUNING_VZ_P,         "VZ_P" },
    { TUNING_AZ_P,         "RateAZ_P" },
    { TUNING_AZ_I,         "RateAZ_I" },
    { TUNING_AZ_D,         "RateAZ_D" },
    { TUNING_RLL_P,        "RollP" },
    { TUNING_RLL_I,        "RollI" },
    { TUNING_RLL_D,        "RollD" },
    { TUNING_RLL_FF,       "RollFF" },
    { TUNING_PIT_P,        "PitchP" },
    { TUNING_PIT_I,        "PitchI" },
    { TUNING_PIT_D,        "PitchD" },
    { TUNING_PIT_FF,       "PitchFF" },
    { TUNING_NONE, nullptr }
};

/*
  get a pointer to an AP_Float for a parameter, or nullptr on fail
 */
AP_Float *AP_Tuning_Plane::get_param_pointer(uint8_t parm)
{
    if (parm < TUNING_FIXED_WING_BASE && !plane.quadplane.available()) {
        // quadplane tuning options not available
        return nullptr;
    }
    
    switch(parm) {

    case TUNING_RATE_ROLL_PI:
        // use P for initial value when tuning PI
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_P:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_I:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kI();

    case TUNING_RATE_ROLL_D:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kD();

    case TUNING_RATE_PITCH_PI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_P:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_I:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kI();

    case TUNING_RATE_PITCH_D:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kD();

    case TUNING_RATE_YAW_PI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_P:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_I:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kI();

    case TUNING_RATE_YAW_D:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kD();

    case TUNING_ANG_ROLL_P:
        return &plane.quadplane.attitude_control->get_angle_roll_p().kP();

    case TUNING_ANG_PITCH_P:
        return &plane.quadplane.attitude_control->get_angle_pitch_p().kP();

    case TUNING_ANG_YAW_P:
        return &plane.quadplane.attitude_control->get_angle_yaw_p().kP();

    case TUNING_PXY_P:
        return &plane.quadplane.p_pos_xy.kP();

    case TUNING_PZ_P:
        return &plane.quadplane.p_alt_hold.kP();

    case TUNING_VXY_P:
        return &plane.quadplane.pi_vel_xy.kP();

    case TUNING_VXY_I:
        return &plane.quadplane.pi_vel_xy.kI();

    case TUNING_VZ_P:
        return &plane.quadplane.p_vel_z.kP();

    case TUNING_AZ_P:
        return &plane.quadplane.pid_accel_z.kP();

    case TUNING_AZ_I:
        return &plane.quadplane.pid_accel_z.kI();

    case TUNING_AZ_D:
        return &plane.quadplane.pid_accel_z.kD();

    // fixed wing tuning parameters
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
    case TUNING_RATE_ROLL_PI:
        save_value(TUNING_RATE_ROLL_P);
        save_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        save_value(TUNING_RATE_PITCH_P);
        save_value(TUNING_RATE_PITCH_I);
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
    case TUNING_RATE_ROLL_PI:
        set_value(TUNING_RATE_ROLL_P, value);
        set_value(TUNING_RATE_ROLL_I, value);
        break;
    case TUNING_RATE_PITCH_PI:
        set_value(TUNING_RATE_PITCH_P, value);
        set_value(TUNING_RATE_PITCH_I, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            if (!(param_bit & have_set)) {
                // first time this param has been set by tuning. We
                // need to see if a reversion value is available in
                // FRAM, and if not then save one
                float current_value = f->get();
                if (!f->load()) {
                    // there is no value in FRAM, set one
                    f->set_and_save(current_value);
                }
                have_set |= param_bit;
            }
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
    case TUNING_RATE_ROLL_PI:
        reload_value(TUNING_RATE_ROLL_P);
        reload_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        reload_value(TUNING_RATE_PITCH_P);
        reload_value(TUNING_RATE_PITCH_I);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            // only reload if we have set this parameter at some point
            if (param_bit & have_set) {
                f->load();
            }
        }
        break;
    }
}

/*
  return current controller error
 */
float AP_Tuning_Plane::controller_error(uint8_t parm)
{
    if (!plane.quadplane.available()) {
        return 0;
    }

    // in general a good tune will have rmsP significantly greater
    // than rmsD. Otherwise it is too easy to push D too high while
    // tuning a quadplane and end up with D dominating
    const float max_P_D_ratio = 3.0f;

    if (plane.quadplane.motors->get_throttle() < 0.1f) {
        // don't report stale errors if not running VTOL motors
        return 0;
    }
        
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
    case TUNING_RATE_ROLL_P:
    case TUNING_RATE_ROLL_I:
        return plane.quadplane.attitude_control->control_monitor_rms_output_roll();

    case TUNING_RATE_ROLL_D: {
        // special case for D term to keep it well below P
        float rms_P = plane.quadplane.attitude_control->control_monitor_rms_output_roll_P();
        float rms_D = plane.quadplane.attitude_control->control_monitor_rms_output_roll_D();
        if (rms_P < rms_D * max_P_D_ratio) {
            return max_P_D_ratio;
        }
        return rms_P+rms_D;
    }
        
        
    case TUNING_RATE_PITCH_PI:
    case TUNING_RATE_PITCH_P:
    case TUNING_RATE_PITCH_I:
        return plane.quadplane.attitude_control->control_monitor_rms_output_pitch();

    case TUNING_RATE_PITCH_D: {
        // special case for D term to keep it well below P
        float rms_P = plane.quadplane.attitude_control->control_monitor_rms_output_pitch_P();
        float rms_D = plane.quadplane.attitude_control->control_monitor_rms_output_pitch_D();
        if (rms_P < rms_D * max_P_D_ratio) {
            return max_P_D_ratio;
        }
        return rms_P+rms_D;
    }
        
    case TUNING_RATE_YAW_PI:
    case TUNING_RATE_YAW_P:
    case TUNING_RATE_YAW_I:
    case TUNING_RATE_YAW_D:
        return plane.quadplane.attitude_control->control_monitor_rms_output_yaw();

    default:
        // no special handler
        return 0;
    }
}
