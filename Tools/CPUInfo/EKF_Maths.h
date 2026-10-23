#pragma once

/*
  data structure for measuring speed of EKF mag fusion code
 */
#include <AP_Math/AP_Math.h>
#include <AP_Math/ftype.h>
#include <stdint.h>

class EKF_Maths {
public:
    EKF_Maths() {}

    typedef ftype Vector2[2];
    typedef ftype Vector3[3];
    typedef ftype Vector4[4];
    typedef ftype Vector5[5];
    typedef ftype Vector6[6];
    typedef ftype Vector7[7];
    typedef ftype Vector8[8];
    typedef ftype Vector9[9];
    typedef ftype Vector10[10];
    typedef ftype Vector11[11];
    typedef ftype Vector13[13];
    typedef ftype Vector14[14];
    typedef ftype Vector15[15];
    typedef ftype Vector22[22];
    typedef ftype Vector23[23];
    typedef ftype Vector24[24];
    typedef ftype Vector25[25];
    typedef ftype Vector28[28];
    typedef ftype Matrix3[3][3];
    typedef ftype Matrix24[24][24];
    typedef ftype Matrix34_50[34][50];
    typedef uint32_t Vector_u32_50[50];

    struct state_elements {
        Vector3f    angErr;         // 0..2
        Vector3f    velocity;       // 3..5
        Vector3f    position;       // 6..8
        Vector3f    gyro_bias;      // 9..11
        Vector3f    gyro_scale;     // 12..14
        float       accel_zbias;    // 15
        Vector3f    earth_magfield; // 16..18
        Vector3f    body_magfield;  // 19..21
        Vector2f    wind_vel;       // 22..23
        Quaternion  quat;           // 24..27
    };

    union {
        Vector28 statesArray;
        struct state_elements stateStruct;
    };

    struct {
        ftype q0;
        ftype q1;
        ftype q2;
        ftype q3;
        ftype magN;
        ftype magE;
        ftype magD;
        ftype magXbias;
        ftype magYbias;
        ftype magZbias;
        Matrix3f DCM;
        Vector3f MagPred;
        ftype R_MAG;
        Vector9 SH_MAG;
    } mag_state;

    Vector3f innovMag;
    Vector3f varInnovMag;

    Matrix24 P;
    Vector28 Kfusion;

    void init(void);
    float test(void);
};

