// calculate 321 yaw observation matrix - option A
const ftype SA0 = 2*q3;
const ftype SA1 = 2*q2;
const ftype SA2 = SA0*q0 + SA1*q1;
const ftype SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
const ftype SA4 = 1.0/sq(SA3);
const ftype SA5 = 1.0F/(sq(SA2)*SA4 + 1);
const ftype SA6 = 1.0F/SA3;
const ftype SA7 = SA2*SA4;
const ftype SA8 = 2*SA7;
const ftype SA9 = 2*SA6;


H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
H_YAW[1] = SA5*(SA1*SA6 - SA8*q1);
H_YAW[2] = SA5*(SA1*SA7 + SA9*q1);
H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
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
const ftype SB3 = 1.0/sq(SB2);
const ftype SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
const ftype SB5 = 1.0F/(SB3*sq(SB4) + 1);
const ftype SB6 = 1.0F/SB2;
const ftype SB7 = SB3*SB4;
const ftype SB8 = 2*SB7;
const ftype SB9 = 2*SB6;


H_YAW[0] = -SB5*(SB0*SB6 - SB8*q3);
H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
H_YAW[3] = -SB5*(-SB0*SB7 - SB9*q3);
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
const ftype SA0 = 2*q3;
const ftype SA1 = 2*q2;
const ftype SA2 = SA0*q0 - SA1*q1;
const ftype SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
const ftype SA4 = 1.0/sq(SA3);
const ftype SA5 = 1.0F/(sq(SA2)*SA4 + 1);
const ftype SA6 = 1.0F/SA3;
const ftype SA7 = SA2*SA4;
const ftype SA8 = 2*SA7;
const ftype SA9 = 2*SA6;


H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
H_YAW[1] = SA5*(-SA1*SA6 + SA8*q1);
H_YAW[2] = SA5*(-SA1*SA7 - SA9*q1);
H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);
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
const ftype SB1 = 2*q1;
const ftype SB2 = -SB0*q3 + SB1*q2;
const ftype SB3 = 1.0/sq(SB2);
const ftype SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
const ftype SB5 = 1.0F/(SB3*sq(SB4) + 1);
const ftype SB6 = 1.0F/SB2;
const ftype SB7 = SB3*SB4;
const ftype SB8 = 2*SB7;
const ftype SB9 = 2*SB6;


H_YAW[0] = -SB5*(-SB0*SB6 + SB8*q3);
H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
H_YAW[3] = -SB5*(SB0*SB7 + SB9*q3);
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


