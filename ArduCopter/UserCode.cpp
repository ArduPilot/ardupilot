#include "Copter.h"
#include <AP_HAL/HAL.h>
#include "mycontroller_usercode.h"

#define PI 3.14159265359

float CAM_roll     = 0.0;
float CAM_pitch    = 0.0;

float PAMD_roll     = 0.0;
float PAMD_pitch    = 0.0;
float PAMD_yaw      = 0.0;

char cable_attitude[]       = "50000_50000";
char payload_attitude[]       = "500000_500000&500000";

char CAM_roll_char[]            = "50000";
char CAM_pitch_char[]           = "50000";

char PAMD_roll_char[]       = "500000";
char PAMD_pitch_char[]      = "500000";
char PAMD_yaw_char[]        = "500000";

float quad_roll             = 0.0;
float quad_pitch            = 0.0;
float quad_yaw              = 0.0;

Vector3f qc_2(0.0,0.0,-1.0);
Vector3f qp(1.0,0.0,0.0);

int CAM_device_port     = 4;
int PAMD_device_port    = 2;

//           parameter prefix 
// UART4  -  SERIAL3_           - GPS      - GPS1       
// UART8 -   SERIAL4_           - SERIAL4  - GPS2       

// USART2 -  SERIAL1_             TELEM1     TELEM1     
// USART3 -  SERIAL2_             TELEM2     TELEM2     


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    hal.serial(PAMD_device_port)->begin(230400);        // telemetry 2 port Pixhawk Cube Orange - PAMD device
    hal.serial(CAM_device_port)->begin(230400);         // GPS 2       port Pixhawk Cube Orange - CAM  device
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

    quad_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    quad_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    quad_yaw         = 360.0 - (ahrs.yaw_sensor)   / 100.0;     // degrees 

    // hal.console->printf("Roll -> %f\n",quad_roll);

    get_CAM_device_Data();
    get_PAMD_device_Data();
    // hal.console->printf("Hi Pratik from Ardupilot \n");

}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here    
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::get_PAMD_device_Data()

{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(PAMD_device_port)->read());

    while (hal.serial(PAMD_device_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(PAMD_device_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    payload_attitude[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    payload_attitude[index] = '\0';
                    receiving_data = false;
                    new_data = false;
                    index = 0;
                }
            }
            else if (temp == startChar)
            {
                receiving_data = true;
                index = 0; 
            }
        }

        // hal.console->printf("attitude -> %s\n",payload_attitude);

        for (int i = 0; i < 7; i++)
        {
                PAMD_roll_char[i]       = payload_attitude[i];
                PAMD_pitch_char[i]      = payload_attitude[i+7];
                PAMD_yaw_char[i]        = payload_attitude[i+14];
        }

        int PAMD_roll_int       = atoi(PAMD_roll_char);
        int PAMD_pitch_int      = atoi(PAMD_pitch_char);
        int PAMD_yaw_int        = atoi(PAMD_yaw_char);

        PAMD_roll  =  (float)((PAMD_roll_int  - 500000.0) / 100.0);
        PAMD_pitch =  (float)((PAMD_pitch_int - 500000.0) / 100.0);
        PAMD_yaw   =  (float)((PAMD_yaw_int - 500000.0) / 100.0);

        if (PAMD_roll > 60.0){
            PAMD_roll = 60.0;
        }
        if (PAMD_roll < -60.0){
            PAMD_roll = -60.0;
        }

        if (PAMD_pitch > 60.0){
            PAMD_pitch = 60.0;
        }
        if (PAMD_pitch < -60.0){
            PAMD_pitch = -60.0;
        }

        if (PAMD_yaw > 180.0){
            PAMD_yaw = 180.0;
        }
        if (PAMD_yaw < -180.0){
            PAMD_yaw = -180.0;
        }

        // hal.console->printf("%3.3f,", PAMD_roll);
        // hal.console->printf("%3.3f,", PAMD_pitch);
        // hal.console->printf("%3.3f\n", PAMD_yaw);

        Vector3f rpy_vector(PAMD_roll*PI/180.0,PAMD_pitch*PI/180.0,PAMD_yaw*PI/180.0);
        Matrix3f R_payload(eulerAnglesToRotationMatrix(rpy_vector));
        Vector3f e_3(0,0,1.0);
        qp = Matrix_vector_mul(R_payload,e_3);

        hal.console->printf("%3.3f,",  qp[0]);
        hal.console->printf("%3.3f,",  qp[1]);
        hal.console->printf("%3.3f\n", qp[2]);

}

void Copter::get_CAM_device_Data()
{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(3)->read());

    while (hal.serial(CAM_device_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(CAM_device_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    cable_attitude[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    cable_attitude[index] = '\0';
                    receiving_data = false;
                    new_data = false;
                    index = 0;
                }
            }
            else if (temp == startChar)
            {
                receiving_data = true;
                index = 0; 
            }
            // hal.console->printf("attitude from while loop-> %s\n",cable_attitude);
        }

        // hal.console->printf("attitude -> %s\n",cable_attitude);
        // hal.console->printf("Hi Pratik from Ardupilot \n");

        for (int i = 0; i < 11; i++)
        {
            if (i < 5)  
            {
                CAM_roll_char[i]                = cable_attitude[i];
            } else if (i >= 6 && i < 11 )
            {
                CAM_pitch_char[i - 6]           = cable_attitude[i];
            }
        }

        int encoder_roll_int      = atoi(CAM_roll_char);
        int encoder_pitch_int     = atoi(CAM_pitch_char);

        CAM_roll  =  (float)((encoder_roll_int  - 50000.0) / 100.0);
        CAM_pitch =  (float)((encoder_pitch_int - 50000.0) / 100.0); 

        // hal.console->printf("%3.3f,", CAM_roll);
        // hal.console->printf("%3.3f\n", CAM_pitch);

        if (CAM_roll > 60.0){
            CAM_roll = 60.0;
        }
        if (CAM_roll < -60.0){
            CAM_roll = -60.0;
        }

        if (CAM_pitch > 60.0){
            CAM_pitch = 60.0;
        }
        if (CAM_pitch < -60.0){
            CAM_pitch = -60.0;
        }

        // hal.console->printf("ENCO-> [%3.3f,%3.3f] | ",CAM_roll,CAM_pitch);

        // hal.console->printf("%3.3f,", CAM_roll);
        // hal.console->printf("%3.3f\n", CAM_pitch);

        // hal.serial(2)->printf("%3.3f,", CAM_roll);
        // hal.serial(2)->printf("%3.3f\n", CAM_pitch);

        // hal.console->printf("CAM_device_data -> %f,%f\n",CAM_roll,CAM_pitch);

        Vector3f rpy(quad_roll*PI/180.0,quad_pitch*PI/180.0,quad_yaw*PI/180.0);
        Vector3f e_3_neg(0,0,-1);
        Matrix3f R(eulerAnglesToRotationMatrix(rpy));

        // Calculate rotation about pitch axis of CAM device
        Matrix3f CAM_R_y (
                cosf(CAM_pitch*PI/180),    0,      sinf(CAM_pitch*PI/180),
                0,               1,      0,
                -sinf(CAM_pitch*PI/180),   0,      cosf(CAM_pitch*PI/180)
                );

        // Calculate rotation about roll axis of CAM device
        Matrix3f CAM_R_x (
                1,       0,              0,
                0,       cosf(CAM_roll*PI/180),   -sinf(CAM_roll*PI/180),
                0,       sinf(CAM_roll*PI/180),   cosf(CAM_roll*PI/180)
                );

        qc_2 = Matrix_vector_mul(R,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
        qc_2 = sat_q(qc_2);

}

Vector3f Copter::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f Copter::hatmap(Vector3f v){
    Matrix3f R (
               0,         -v[2],      v[1],
               v[2],         0 ,     -v[0],
               -v[1],      v[0],       0);
    return R;
}

Matrix3f Copter::eulerAnglesToRotationMatrix(Vector3f rpy){
     // Calculate rotation about x axis
    Matrix3f R_x (
               1,       0,              0,
               0,       cosf(rpy[0]),   -sinf(rpy[0]),
               0,       sinf(rpy[0]),   cosf(rpy[0])
               );

    // Calculate rotation about y axis
    Matrix3f R_y (
               cosf(rpy[1]),    0,      sinf(rpy[1]),
               0,               1,      0,
               -sinf(rpy[1]),   0,      cosf(rpy[1])
               );

    // Calculate rotation about z axis
    Matrix3f R_z (
               cosf(rpy[2]),    -sinf(rpy[2]),      0,
               sinf(rpy[2]),    cosf(rpy[2]),       0,
               0,               0,                  1);

    // Combined rotation matrix
    Matrix3f R = R_z * R_y * R_x;
    return R;
}

Vector3f Copter::sat_q(Vector3f vec){
    float sat_lim = 1;
    for (int ii=0; ii<3; ii++){
        if (vec[ii] > sat_lim){
            vec[ii] = sat_lim;
        }
        if (vec[ii] < -sat_lim){
            vec[ii] = -sat_lim;
        }
    }
    return vec;
}

Vector3f Copter::sat_q_dot(Vector3f vec){
    float sat_lim = 3;
    for (int ii=0; ii<3; ii++){
        if (vec[ii] > sat_lim){
            vec[ii] = sat_lim;
        }
        if (vec[ii] < -sat_lim){
            vec[ii] = -sat_lim;
        }
    }
    return vec;
}