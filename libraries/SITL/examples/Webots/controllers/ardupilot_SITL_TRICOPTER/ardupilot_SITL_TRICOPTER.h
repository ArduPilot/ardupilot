#ifndef ARDUPILOT_SITL_TRICOPTER_H
#define ARDUPILOT_SITL_TRICOPTER_H

// #define DEBUG_MOTORS
// #define DEBUG_SENSORS
// #define DEBUG_USE_KB
// #define DEBUG_INPUT_DATA

/* Thrust linearisation is no longer optional: update_controls() always commands
   omega = sqrt(u) * omega_max so that thrust is linear in ArduPilot's output.
   The matching .parm files set MOT_THST_EXPO 0. */

/* The tricopter worlds carry an "emitter_plugin" Emitter and the
   sitl_physics_env plugin, so wind forces can be applied. */
#define WIND_SIMULATION

#define VEHICLE_DRAG_FACTOR 0.001

// # of simulation steps between two image frames.
#define CAMERA_FRAME_RATE_FACTOR  50

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))


enum data_type {
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR4F,
    };

struct vector4f 
{
    float w,x,y,z;
};

    
typedef struct vector4f VECTOR4F;

struct {
        double timestamp;
        VECTOR4F motors;
        VECTOR4F wind; 
        /*
         struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
        } wind;
        */
        } state, last_state;



// table to aid parsing of JSON sensor data
struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;

} keytable[2] = {
        //{ "", "timestamp", &state.timestamp, DATA_DOUBLE },
        { "", "eng",    &state.motors, DATA_VECTOR4F }, /*right,left,servo,back*/
        { "", "wnd",    &state.wind, DATA_VECTOR4F }    /*speed,x,y,z in Ardupilot AXIS*/    
};


/*
        w: wind speed
        x , y, z: wind direction.
*/
VECTOR4F   wind_webots_axis;

#endif  /* ARDUPILOT_SITL_TRICOPTER_H */
