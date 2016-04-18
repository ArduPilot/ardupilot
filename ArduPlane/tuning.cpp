// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

const AP_Param::GroupInfo Tuning::var_info[] = {

    // @Param: CHAN
    // @DisplayName: Transmitter tuning channel
    // @Description: This sets the channel for transmitter tuning 
    // @Values: 0:Disable,1:Chan1,2:Chan3,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO_FLAGS("CHAN", 1, Tuning, channel, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PARM
    // @DisplayName: Transmitter tuning parameter
    // @Description: This sets which parameter or combination of parameters will be tuned
    // @Values: 0:None,1:QuadRateRollPitch_PI,2:QuadRateRollPitch_P,3:QuadRateRollPitch_I,4:QuadRateRollPitch_D,5:QuadRATE_ROLL_PI,6:QuadRateRoll_P,7:QuadRateRoll_I,8:QuadRateRoll_D,9:QuadRatePitch_PI,10:QuadRatePitch_P,11:QuadRatePitch_I,12:QuadRatePitch_D,13:QuadRateYaw_PI,14:QuadRateYaw_P,15:QuadRateYaw_I,16:QuadRateYaw_D,17:QuadAngleRoll_P,18:QuadAnglePitch_P,19:QuadAngleYaw_P,20:QuadPXY_P,21:QuadPZ_P,22:QuadVXY_P,23:QuadVXY_I,24:QuadVZ_P,25:QuadAZ_P,26:QuadAZ_I,27:QuadAZ_D,28:Roll_P,29:Roll_I,30:Roll_D,31:Roll_FF,32:Pitch_P,33:Pitch_I,34:Pitch_D,35:Pitch_FF
    // @User: Standard
    AP_GROUPINFO("PARM", 2, Tuning, parm, 0),

    // @Param: MIN
    // @DisplayName: Transmitter tuning minimum value
    // @Description: This sets the minimum value
    // @User: Standard
    AP_GROUPINFO("MIN", 3, Tuning, minimum, 0),

    // @Param: MAX
    // @DisplayName: Transmitter tuning maximum value
    // @Description: This sets the maximum value
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tuning, maximum, 0),
    
    AP_GROUPEND
};

/*
  constructor
 */
Tuning::Tuning(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  check for changed tuning input
 */
void Tuning::check_input(void)
{
    if (channel <= 0 || parm <= 0) {
        // disabled
        return;
    }

    // only adjust values at 4Hz
    uint32_t now = AP_HAL::millis();
    if (now - last_check_ms < 250) {
        return;
    }
    last_check_ms = now;

    if (channel > hal.rcin->num_channels()) {
        return;
    }
    
    RC_Channel *chan = RC_Channel::rc_channel(channel-1);
    if (chan == nullptr) {
        return;
    }
    uint8_t input = chan->percent_input();
    if (input == last_input_pct) {
        // no change
        return;
    }
    last_input_pct = input;

    float tuning_value = minimum + (maximum-minimum)*(input*0.01f);

    Log_Write_Parameter_Tuning((uint8_t)parm.get(), tuning_value, minimum, maximum);

    switch((enum tuning_func)parm.get()) {

    case TUNING_RLL_P:
        plane.rollController.kP(tuning_value);
        break;

    case TUNING_RLL_I:
        plane.rollController.kI(tuning_value);
        break;

    case TUNING_RLL_D:
        plane.rollController.kD(tuning_value);
        break;

    case TUNING_RLL_FF:
        plane.rollController.kFF(tuning_value);
        break;

    case TUNING_PIT_P:
        plane.pitchController.kP(tuning_value);
        break;

    case TUNING_PIT_I:
        plane.pitchController.kI(tuning_value);
        break;

    case TUNING_PIT_D:
        plane.pitchController.kD(tuning_value);
        break;

    case TUNING_PIT_FF:
        plane.pitchController.kFF(tuning_value);
        break;
        
    default:
        break;
    }
        
    if (!plane.quadplane.available()) {
        // quadplane tuning options not available
        return;
    }
    
    switch((enum tuning_func)parm.get()) {

    case TUNING_Q_RATE_ROLL_PITCH_KPI:
        plane.quadplane.attitude_control->get_rate_roll_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_roll_pid().kI(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_PITCH_KP:
        plane.quadplane.attitude_control->get_rate_roll_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_PITCH_KI:
        plane.quadplane.attitude_control->get_rate_roll_pid().kI(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_PITCH_KD:
        plane.quadplane.attitude_control->get_rate_roll_pid().kD(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_KPI:
        plane.quadplane.attitude_control->get_rate_roll_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_roll_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_KP:
        plane.quadplane.attitude_control->get_rate_roll_pid().kP(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_KI:
        plane.quadplane.attitude_control->get_rate_roll_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_ROLL_KD:
        plane.quadplane.attitude_control->get_rate_roll_pid().kD(tuning_value);
        break;

    case TUNING_Q_RATE_PITCH_KPI:
        plane.quadplane.attitude_control->get_rate_pitch_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_PITCH_KP:
        plane.quadplane.attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_Q_RATE_PITCH_KI:
        plane.quadplane.attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_PITCH_KD:
        plane.quadplane.attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    case TUNING_Q_RATE_YAW_KPI:
        plane.quadplane.attitude_control->get_rate_yaw_pid().kP(tuning_value);
        plane.quadplane.attitude_control->get_rate_yaw_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_YAW_KP:
        plane.quadplane.attitude_control->get_rate_yaw_pid().kP(tuning_value);
        break;

    case TUNING_Q_RATE_YAW_KI:
        plane.quadplane.attitude_control->get_rate_yaw_pid().kI(tuning_value);
        break;

    case TUNING_Q_RATE_YAW_KD:
        plane.quadplane.attitude_control->get_rate_yaw_pid().kD(tuning_value);
        break;

    case TUNING_Q_ANG_ROLL_KP:
        plane.quadplane.attitude_control->get_angle_roll_p().kP(tuning_value);
        break;

    case TUNING_Q_ANG_PITCH_KP:
        plane.quadplane.attitude_control->get_angle_pitch_p().kP(tuning_value);
        break;

    case TUNING_Q_ANG_YAW_KP:
        plane.quadplane.attitude_control->get_angle_yaw_p().kP(tuning_value);
        break;

    case TUNING_Q_PXY_P:
        plane.quadplane.p_pos_xy.kP(tuning_value);
        break;

    case TUNING_Q_PZ_P:
        plane.quadplane.p_alt_hold.kP(tuning_value);
        break;

    case TUNING_Q_VXY_P:
        plane.quadplane.pi_vel_xy.kP(tuning_value);
        break;

    case TUNING_Q_VXY_I:
        plane.quadplane.pi_vel_xy.kI(tuning_value);
        break;

    case TUNING_Q_VZ_P:
        plane.quadplane.p_vel_z.kP(tuning_value);
        break;

    case TUNING_Q_AZ_P:
        plane.quadplane.pid_accel_z.kP(tuning_value);
        break;

    case TUNING_Q_AZ_I:
        plane.quadplane.pid_accel_z.kI(tuning_value);
        break;

    case TUNING_Q_AZ_D:
        plane.quadplane.pid_accel_z.kD(tuning_value);
        break;
        
    default:
        return;
    }
}

/*
  log a tuning change
 */
void Tuning::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_low, float tune_high)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_low     : tune_low,
        tuning_high    : tune_high
    };

    plane.DataFlash.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}
