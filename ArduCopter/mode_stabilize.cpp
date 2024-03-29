#include "Copter.h"
#include "mycontroller_usercode.h"
#define PI 3.14159265359

float human_roll_command        = 0.0;
float human_pitch_command       = 0.0;
float human_yaw_rate_command    = 0.0;
float human_throttle_command    = 0.0;
float human_des_yaw_command     = 0.0;

float TRO_target_roll_angle     = 0.0;     //  [-45 45] degrees
float TRO_target_pitch_angle    = 0.0;     //  [-45 45] degrees
float TRO_target_yaw_rate       = 0.0;     //  [-20 20] degrees/sec
float TRO_target_throttle       = 0.0;     //  [0 1] 0.38 - is for hovering condition almost

Vector3f quad_pos(0.0,0.0,0.0);
Vector3f quad_vel(0.0,0.0,0.0);
Vector3f quad_pos_des(0.0,0.0,0.0);

float quad_roll         = 0.0;
float quad_pitch        = 0.0;
float quad_yaw          = 0.0;
float quad_roll_dot     = 0.0;
float quad_pitch_dot    = 0.0;
float quad_yaw_dot      = 0.0;

float quad_yaw_des  = 0.0;

Vector3f rpy_des(0.0,0.0,0.0);

float human_xd_dot      = 0.0;
float human_yd_dot      = 0.0;
float human_zd_dot      = 0.0;
float human_psid_dot      = 0.0;

Vector3f qc_1(0.0,0.0,0.0);
Vector3f qc_2(0.0,0.0,0.0);

Vector3f qc_1_dot(0.0,0.0,0.0);
Vector3f qc_2_dot(0.0,0.0,0.0);

Vector3f qp(1.0,0.0,0.0);
Vector3f qc_1_fil(0.0,0.0,-1.0);
Vector3f qc_1_dot_fil(0.0,0.0,0.0);
Vector3f qc_1_des(0.0,0.0,-1.0);

float fil_qc_11_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_12_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_13_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_qc_11_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_12_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_13_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

Vector3f u1_POS(0.0,0.0,0.0);
Vector3f u1_PAC(0.0,0.0,0.0);
Vector3f u1_CAC1(0.0,0.0,0.0);
Vector3f u1(0.0,0.0,0.0);

float mq                = 1.236;
float gravity_acc       = 9.81;

Vector3f b1_quad(1.0,0.0,0.0);
Vector3f b_1_des(1.0,0.0,0.0);
Vector3f b_2_des(0.0,1.0,0.0);
Vector3f b_3_des(0.0,0.0,1.0);
Vector3f b_1_c(1.0,0.0,0.0);

Vector3f yaw_current_vector(1.0,0.0,0.0);
Vector3f yaw_desired_vector(1.0,0.0,0.0);
Vector3f error_yaw_vector(1.0,0.0,0.0);

float third_value_of_error_yaw_vector       = 0.0;
float third_value_of_error_yaw_vector_old   = 0.0;
float third_value_of_error_yaw_vector_dot   = 0.0;
float third_value_of_error_yaw_vector_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_phi_des_array[]       = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_theta_des_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

Vector3f quad_pos_ini_inertial(0.0,0.0,0.0);

float H_x_dot_command   = 0.0;
float H_y_dot_command   = 0.0;
float H_z_dot_command   = 0.0;
float H_yaw_rate__      = 0.0;

////////////////// PD gains

float kp_x       = 5.0;
float kd_x       = 3.3;

float kp_y       = 5.0;
float kd_y       = 3.3;

float kp_z       = 12.0;
float kd_z       = 4.0;

int arming_state_variable = 0;
float ch_6_state = 0.0;
/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{

    pilot_input_command();
    quad_states();

    ch_6_state = RC_Channels::get_radio_in(CH_6);
    // hal.console->printf("%2.3f\n", ch_6_state);

    if (ch_6_state < 1500.0)
    {
        quad_pos_des[0] =  inertial_nav.get_position_neu_cm().x / 100.0;
        quad_pos_des[1] =  -inertial_nav.get_position_neu_cm().y / 100.0;
        quad_pos_des[2] =  inertial_nav.get_position_neu_cm().z / 100.0;

        human_des_yaw_command       = quad_yaw;    // degrees [0 360]

    }

    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    
    float pilot_desired_throttle = get_pilot_desired_throttle();

    // hal.console->printf("%3.3f,",  qp[0]);

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }


    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }


//////////////////////////////////////////////////////////////////////////////////
/////////////////////        Human commands              /////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    // target_roll              [-4500 to 4500]
    // target_pitch             [-4500 to 4500]
    // target_yaw_rate          [-20250 to 20250]
    // pilot_desired_throttle   [0 to 1]

    human_roll_command          = target_roll;
    human_pitch_command         = target_pitch;
    human_yaw_rate_command      = target_yaw_rate;
    human_throttle_command      = pilot_desired_throttle;




    // if (arming_state_variable == 0){
    //     if(motors->armed()){
    //         quad_pos_des[0] =  inertial_nav.get_position_neu_cm().x / 100.0;
    //         quad_pos_des[1] =  inertial_nav.get_position_neu_cm().y / 100.0;
    //         // quad_pos_des[2] =  inertial_nav.get_position_neu_cm().z / 100.0;

    //         human_des_yaw_command       = quad_yaw;    // degrees [0 360]
    //         arming_state_variable       = 1;
    //     }
    // }

/////////////////////        Quad 1 position controller        /////////////////////////

    hal.console->printf("%2.3f,%2.3f | ", quad_pos[0], quad_pos_des[0]);
    hal.console->printf("%2.3f,%2.3f\n",  quad_pos[1], quad_pos_des[1]);
    // hal.console->printf("%2.3f,%2.3f\n",quad_pos[2],quad_pos_des[2]);

    // hal.console->printf("X - (%2.2f,%2.2f,%2.2f) | ",quad_pos[0],quad_pos[1],quad_pos[2]);
    // hal.console->printf("X_dot - (%2.2f,%2.2f,%2.2f) | ",quad_vel[0],quad_vel[1],quad_vel[2]);
    // hal.console->printf("Xd - (%2.2f,%2.2f,%2.2f)\n",quad_pos_des[0],quad_pos_des[1],quad_pos_des[2]);

    Vector3f e_quad_pos = quad_pos_des - quad_pos;
    Vector3f e_quad_vel =              - quad_vel;

    u1_POS[0]      = kp_x * e_quad_pos[0] + kd_x * e_quad_vel[0];
    u1_POS[1]      = kp_y * e_quad_pos[1] + kd_y * e_quad_vel[1];
    u1_POS[2]      = kp_z * e_quad_pos[2] + kd_z * e_quad_vel[2];

    // hal.console->printf("%2.3f,%2.3f,%2.3f\n", u1_POS[0], u1_POS[1], u1_POS[2]);
    // hal.console->printf("%2.3f, %2.3f\n", quad_pos[2], quad_pos_des[2]);

/////////////        summation of all the control inputs        //////////////////
    float coeficiant_constant_hover_value = 0.38*2;
    Vector3f gravity_compensation_vec(0.0, 0.0, coeficiant_constant_hover_value*mq*gravity_acc);

    u1                  = u1_POS + u1_CAC1 + gravity_compensation_vec;

    // u1[0]               = 5;
    // u1[1]               = 5;
    // u1[2]               = 15;

    if (u1[2] < 0.0){u1[2] = 0.0;}

    float u1_norm       = norm_of_vector(u1);

    b_3_des[0]          = u1[0] / u1_norm;
    b_3_des[1]          = u1[1] / u1_norm;
    b_3_des[2]          = u1[2] / u1_norm;
    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", b_3_des[0], b_3_des[1], b_3_des[2]);
    // human_des_yaw_command = 10.0;
    b_1_c[0]            = cosf((human_des_yaw_command/180.0*PI));
    b_1_c[1]            = sinf((human_des_yaw_command/180.0*PI));
    b_1_c[2]            = 0.0;
    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", b_1_c[0], b_1_c[1], b_1_c[2]);

    b_2_des             = cross_product(b_3_des, b_1_c);
    b_1_des             = cross_product(b_2_des, b_3_des);

    // hal.console->printf("Human Yaw des -> %3.3f \n", human_des_yaw_command);

    // hal.console->printf("b_1_des -> %3.3f, %3.3f, %3.3f, |\n", b_1_des[0], b_1_des[1], b_1_des[2]);
    // hal.console->printf("b_2_des -> %3.3f, %3.3f, %3.3f, |\n", b_2_des[0], b_2_des[1], b_2_des[2]);
    // hal.console->printf("b_3_des -> %3.3f, %3.3f, %3.3f, |\n", b_3_des[0], b_3_des[1], b_3_des[2]);


    Matrix3f R_quad_attitude(
            b_1_des[0], b_2_des[0], b_3_des[0],
            b_1_des[1], b_2_des[1], b_3_des[1],
            b_1_des[2], b_2_des[2], b_3_des[2]
            );

    rpy_des = Rotation_matrix_to_Euler_angle(R_quad_attitude);
    // hal.console->printf("%3.3f Yaw_des -> ", human_des_yaw_command);
    // hal.console->printf("RPY_des ->  %3.3f, %3.3f, %3.3f \n", rpy_des[0], rpy_des[1], rpy_des[2]);
    // hal.console->printf("Quad_RPY -> %3.3f, %3.3f, %3.3f, |\n", quad_roll, quad_pitch, quad_yaw);
    // hal.console->printf("%3.3f, %3.3f \n", quad_yaw, rpy_des[2]);

    if (u1_norm > 2*mq*gravity_acc) { u1_norm = 2*mq*gravity_acc; }
    if (u1_norm < 0.0){u1_norm = 0.0; }

    float u1_scaled = u1_norm / (2*mq*gravity_acc);
    // hal.console->printf("%3.3f \n", u1_scaled);

/////////////////////        yaw angle controller        /////////////////////////

    yaw_current_vector[0]   = cosf((PI/2.0) - (quad_yaw/180.0*PI));
    yaw_current_vector[1]   = sinf((PI/2.0) - (quad_yaw/180.0*PI));
    yaw_current_vector[2]   = 0.0;

    yaw_desired_vector[0]   = cosf((PI/2.0) - (human_des_yaw_command/180.0*PI));
    yaw_desired_vector[1]   = sinf((PI/2.0) - (human_des_yaw_command/180.0*PI));
    yaw_desired_vector[2]   = 0.0;

    // yaw_desired_vector[0]   = cosf( -(rpy_des[2]/180.0*PI) + (PI/2.0));
    // yaw_desired_vector[1]   = sinf( -(rpy_des[2]/180.0*PI) + (PI/2.0));
    // yaw_desired_vector[2]   = 0.0;

    // hal.console->printf("%3.3f, %3.3f,\n", quad_yaw, human_des_yaw_command);

    error_yaw_vector                    = Matrix_vector_mul(hatmap(yaw_current_vector), yaw_desired_vector);
    third_value_of_error_yaw_vector     = error_yaw_vector[2];
    third_value_of_error_yaw_vector_dot = (third_value_of_error_yaw_vector - third_value_of_error_yaw_vector_old);

    third_value_of_error_yaw_vector_old = third_value_of_error_yaw_vector;

    float third_value_of_error_yaw_vector_dot_fil = 200*simple_fil_low_pos(5, third_value_of_error_yaw_vector_dot_array, third_value_of_error_yaw_vector_dot);

    // float error_yaw_vector_norm = norm_of_vector(error_yaw_vector);
    // hal.console->printf("%3.3f, %3.3f\n", third_value_of_error_yaw_vector, third_value_of_error_yaw_vector_dot_fil);

    if (third_value_of_error_yaw_vector_dot_fil >  3.0) { third_value_of_error_yaw_vector_dot_fil = 3.0; }
    if (third_value_of_error_yaw_vector_dot_fil < -3.0) { third_value_of_error_yaw_vector_dot_fil = -3.0; }

    if (third_value_of_error_yaw_vector >  1.0) { third_value_of_error_yaw_vector = 1.0; }
    if (third_value_of_error_yaw_vector < -1.0) { third_value_of_error_yaw_vector = -1.0; }

//////////////////////////////////////////////////////////////////////////////////
/////////////////////        Final control inputs        /////////////////////////
//////////////////////////////////////////////////////////////////////////////////

    float phi_fil      = simple_fil_low_pos(3, fil_phi_des_array, rpy_des[0]);
    float theta_fil    = simple_fil_low_pos(3, fil_theta_des_array, rpy_des[1]);

    TRO_target_throttle       = u1_scaled;                              //  [0 1] 0.38 - is for hovering condition almost
    TRO_target_roll_angle     = phi_fil;                                //  [-45 45] degrees
    TRO_target_pitch_angle    = -theta_fil;                              //  [-45 45] degrees
    TRO_target_yaw_rate       = 20.0 * third_value_of_error_yaw_vector + 3.0 * third_value_of_error_yaw_vector_dot_fil;     //  [-20 20] degrees/sec
    // TRO_target_yaw_rate       = human_yaw_rate_command/1000.0;       //  [-20 20] degrees/sec    

    // TRO_target_throttle       = 0.0;                                    //  [0 1] 0.38 - is for hovering condition almost
    // TRO_target_roll_angle     = 0.0;                                    //  [-45 45] degrees
    // TRO_target_pitch_angle    = 0.0;                                    //  [-45 45] degrees
    // TRO_target_yaw_rate       = 0.0;

    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", quad_yaw, human_des_yaw_command, error_yaw_vector_norm);

//////////////////////////////////////////////////////////////////////////////////
////////////        final altitude and attitude controller    ////////////////////
//////////////////////////////////////////////////////////////////////////////////
/////////////////////        set limits on the inputs       /////////////////////////
    TRO_target_roll_angle       = limit_on_desired_angles(TRO_target_roll_angle);
    TRO_target_pitch_angle      = limit_on_desired_angles(TRO_target_pitch_angle);
    TRO_target_yaw_rate         = limit_on_yaw_rate(TRO_target_yaw_rate);
    TRO_target_throttle         = limit_on_thurst_val(TRO_target_throttle);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(TRO_target_roll_angle*100, TRO_target_pitch_angle*100, TRO_target_yaw_rate*1000);

    // output pilot's throttle
    attitude_control->set_throttle_out(TRO_target_throttle, true, g.throttle_filt);
}


void ModeStabilize::pilot_input_command()
{
    H_yaw_rate__      = human_yaw_rate_command / 1000.0;      // -20.25 to 20.25
    float dt_yaw            = 1.0/100.0;
    human_des_yaw_command   = wrap_360(human_des_yaw_command - H_yaw_rate__*dt_yaw);
    // float offsetted_val_yaw = 1.0;
    // if (H_yaw_rate__ > offsetted_val_yaw)
    // {
    //     human_des_yaw_command   = wrap_360(human_des_yaw_command - (H_yaw_rate__-offsetted_val_yaw)*dt_yaw);
    // }

    // if(H_yaw_rate__ < -offsetted_val_yaw)
    // {
    //     human_des_yaw_command   = wrap_360(human_des_yaw_command - (H_yaw_rate__+offsetted_val_yaw)*dt_yaw);
    // }

    H_y_dot_command   = human_roll_command  / 100.0;          // -45.0 to 45.0
    float dt_y              = 1.0/12000.0;
    float offsetted_val_y   = 2.0;
    if (H_y_dot_command > offsetted_val_y)
    {
        quad_pos_des[1]         = quad_pos_des[1] - (H_y_dot_command-offsetted_val_y)*dt_y;
    }
    if (H_y_dot_command < -offsetted_val_y)
    {
        quad_pos_des[1]         = quad_pos_des[1] - (H_y_dot_command+offsetted_val_y)*dt_y;
    }

    H_x_dot_command     = human_pitch_command  / 100.0;
    float dt_x              = 1.0/12000.0;
    float offsetted_val_x   = 2.0;
    if (H_x_dot_command > offsetted_val_x)
    {
        quad_pos_des[0]         = quad_pos_des[0] - (H_x_dot_command-offsetted_val_x)*dt_x;
    }
    if (H_x_dot_command < -offsetted_val_x)
    {
        quad_pos_des[0]         = quad_pos_des[0] - (H_x_dot_command+offsetted_val_x)*dt_x;
    }

    H_z_dot_command     = (double)channel_throttle->get_control_in()-500.0;

    float dt_z              = 1.0/120000.0;
    float offsetted_val_z   = 50;
    if (H_z_dot_command > offsetted_val_z)
    {
        quad_pos_des[2]         = quad_pos_des[2] + (H_z_dot_command-offsetted_val_z)*dt_z;
    }
    if (H_z_dot_command < -offsetted_val_z)
    {
        quad_pos_des[2]         = quad_pos_des[2] + (H_z_dot_command+offsetted_val_z)*dt_z;
    }

    // hal.console->printf("%2.3f, %2.3f\n",H_z_dot_command, quad_pos_des[2]);
    // hal.console->printf("%2.3f\n", quad_pos_des[2]);
    
    




}

void ModeStabilize::quad_states(){

    // position in body reference frame
    quad_pos[0] =  inertial_nav.get_position_neu_cm().x / 100.0;
    quad_pos[1] =  -inertial_nav.get_position_neu_cm().y / 100.0;
    quad_pos[2] =  inertial_nav.get_position_neu_cm().z / 100.0;

    // linear velocity in inertial frame of reference
    quad_vel[0] =  inertial_nav.get_velocity_neu_cms().x /100.0;
    quad_vel[1] =  -inertial_nav.get_velocity_neu_cms().y /100.0;
    quad_vel[2] =  inertial_nav.get_velocity_neu_cms().z /100.0;

    quad_roll        =        (ahrs.roll_sensor)  / 100.0;     // degrees 
    quad_pitch       =       -(ahrs.pitch_sensor) / 100.0;     // degrees 
    quad_yaw         =  360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    quad_roll_dot    =        (ahrs.get_gyro().x);             // degrees/second
    quad_pitch_dot   =       -(ahrs.get_gyro().y);             // degrees/second    
    quad_yaw_dot     =       -(ahrs.get_gyro().z);             // degrees/second

}

float ModeStabilize::limit_on_desired_angles(float angle)
{
    float max_angle_value = 45.0;
    // float max_angle_value = 20.0;
    if (angle > max_angle_value)
    {
        angle = max_angle_value;
    }

    if (angle < -max_angle_value)
    {
        angle = -max_angle_value;
    }
    return angle;
}

float ModeStabilize::limit_on_yaw_rate(float angle_dot)
{
    float value = 20.0;
    if (angle_dot > value)
    {
        angle_dot = value;
    }

    if (angle_dot < -value)
    {
        angle_dot = -value;
    }
    return angle_dot;
}

float ModeStabilize::limit_on_thurst_val(float val)
{
    float max_value = 1.0;
    float min_value = 0.0;

    if (val > max_value)
    {
        val = max_value;
    }

    if (val < min_value)
    {
        val = min_value;
    }
    return val;
}

float ModeStabilize::norm_of_vector(Vector3f v)
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float ModeStabilize::simple_fil_low_pos(int iteration, float array[], float current_value)
{
    for (int h = iteration-1; h >= 0; h--)
    {
        if (h >= 1)
        {
            array[h] = array[h-1];
        }
        else if (h == 0)
        {
            array[0] = current_value;
        }
    }
    float array_sum = 0.0;
    for (int h = 0; h < iteration; h++)
    {
        array_sum += array[h];
    }

    float filterd_value = array_sum/iteration;

    return filterd_value;
}

Vector3f ModeStabilize::cross_product(Vector3f v1, Vector3f v2)
{
    return Matrix_vector_mul(hatmap(v1), v2);
}


Vector3f ModeStabilize::Rotation_matrix_to_Euler_angle(Matrix3f R)
{

    float sy = sqrtf(R[0][0] * R[0][0] +  R[1][0] * R[1][0]);
    float ph_des = atan2f(R[2][1] , R[2][2]);
    float th_des = atan2f(-R[2][0], sy);
    float ps_des = atan2f(R[1][0], R[0][0]);


    Vector3f des_roll_pitch_yaw(ph_des*180.0/PI, th_des*180.0/PI, ps_des*180.0/PI);

    return des_roll_pitch_yaw;
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f ModeStabilize::hatmap(Vector3f v){
    Matrix3f R (
               0,    -v[2],      v[1],
               v[2],    0,     -v[0],
               -v[1],      v[0],       0);
    return R;
}