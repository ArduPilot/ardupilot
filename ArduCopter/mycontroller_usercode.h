#pragma once

extern Vector3f quad_vel;
extern Vector3f quad_pos;
extern Vector3f quad_pos_des;
extern float quad_yaw_des;

extern float human_des_yaw_command;

extern float quad_roll;
extern float quad_pitch;
extern float quad_yaw;

extern float quad_roll_dot;
extern float quad_pitch_dot;
extern float quad_yaw_dot;

extern float human_x_des_command;
extern float human_y_des_command;
extern float human_z_des_command;
extern float human_ps_des_dot_command;

extern float human_xd_dot;
extern float human_yd_dot;
extern float human_zd_dot;
extern float human_psid_dot;

extern Vector3f qc_1;
extern Vector3f qc_2;
extern Vector3f qc_1_dot;

extern Vector3f qc_2;
extern Vector3f qp;

extern Vector3f u1_POS;
extern Vector3f u1_PAC;
extern Vector3f u1_CAC1;