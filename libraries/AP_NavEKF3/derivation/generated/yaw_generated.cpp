// calculate 321 yaw observation matrix - option A
const ftype SA0 = 2*q0;
const ftype SA1 = 2*q1;
const ftype SA2 = SA0*q3 + SA1*q2;
const ftype SA3 = -2*powf(q2, 2) - 2*powf(q3, 2) + 1;
const ftype SA4 = powf(SA3, -2);
const ftype SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
const ftype SA6 = 1.0F/SA3;
const ftype SA7 = 2*SA5*SA6;
const ftype SA8 = 4*SA2*SA4;


H_YAW[0] = SA7*q3;
H_YAW[1] = SA7*q2;
H_YAW[2] = SA5*(SA1*SA6 + SA8*q2);
H_YAW[3] = SA5*(SA0*SA6 + SA8*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 321 yaw observation matrix - option B
const ftype SB0 = 2*q0;
const ftype SB1 = 2*q1;
const ftype SB2 = SB0*q3 + SB1*q2;
const ftype SB3 = powf(SB2, -2);
const ftype SB4 = -2*powf(q2, 2) - 2*powf(q3, 2) + 1;
const ftype SB5 = 1.0F/(SB3*powf(SB4, 2) + 1);
const ftype SB6 = SB3*SB4;
const ftype SB7 = 2*SB5*SB6;
const ftype SB8 = 4/SB2;


H_YAW[0] = SB7*q3;
H_YAW[1] = SB7*q2;
H_YAW[2] = -SB5*(-SB1*SB6 - SB8*q2);
H_YAW[3] = -SB5*(-SB0*SB6 - SB8*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 312 yaw observation matrix - option A
const ftype SA0 = 2*q0;
const ftype SA1 = 2*q2;
const ftype SA2 = SA0*q3 - SA1*q1;
const ftype SA3 = -2*powf(q1, 2) - 2*powf(q3, 2) + 1;
const ftype SA4 = powf(SA3, -2);
const ftype SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
const ftype SA6 = 1.0F/SA3;
const ftype SA7 = 2*SA5*SA6;
const ftype SA8 = 4*SA2*SA4;


H_YAW[0] = SA7*q3;
H_YAW[1] = SA5*(-SA1*SA6 + SA8*q1);
H_YAW[2] = -SA7*q1;
H_YAW[3] = SA5*(SA0*SA6 + SA8*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


// calculate 312 yaw observation matrix - option B
const ftype SB0 = 2*q0;
const ftype SB1 = -SB0*q3 + 2*q1*q2;
const ftype SB2 = powf(SB1, -2);
const ftype SB3 = 2*powf(q1, 2) + 2*powf(q3, 2) - 1;
const ftype SB4 = 1.0F/(SB2*powf(SB3, 2) + 1);
const ftype SB5 = SB2*SB3;
const ftype SB6 = 2*SB4*SB5;
const ftype SB7 = 4/SB1;


H_YAW[0] = -SB6*q3;
H_YAW[1] = -SB4*(-2*SB5*q2 + SB7*q1);
H_YAW[2] = SB6*q1;
H_YAW[3] = -SB4*(SB0*SB5 + SB7*q3);
H_YAW[4] = 0;
H_YAW[5] = 0;
H_YAW[6] = 0;
H_YAW[7] = 0;
H_YAW[8] = 0;
H_YAW[9] = 0;
H_YAW[10] = 0;
H_YAW[11] = 0;
H_YAW[12] = 0;
H_YAW[13] = 0;
H_YAW[14] = 0;
H_YAW[15] = 0;
H_YAW[16] = 0;
H_YAW[17] = 0;
H_YAW[18] = 0;
H_YAW[19] = 0;
H_YAW[20] = 0;
H_YAW[21] = 0;
H_YAW[22] = 0;
H_YAW[23] = 0;


