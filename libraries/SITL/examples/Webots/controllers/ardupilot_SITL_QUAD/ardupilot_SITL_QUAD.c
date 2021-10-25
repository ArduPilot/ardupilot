/*
 * File: ardupilot_SITL_QUAD.c
 * Date: 29 July 2019
 * Description: integration with ardupilot SITL simulation.
 * Author: M.S.Hefny (HefnySco)
 * Modifications:
 *  - Blocking sockets
 *  - Advance simulation time only when receive motor data.
 */


/*
  Data is sent in format:
  
  {"timestamp": 1561043647.7598028, 
            "vehicle.imu": {"timestamp": 1561043647.7431362, 
                    "angular_velocity": [-8.910427823138889e-06, 1.6135254554683343e-06, 0.0005768465343862772], 
                    "linear_acceleration": [-0.06396577507257462, 0.22235631942749023, 9.807276725769043], 
                    "magnetic_field": [23662.052734375, 2878.55859375, -53016.55859375]}, 
                    "vehicle.gps": {"timestamp": 1561043647.7431362, "x": -0.0027823783457279205, "y": -0.026340210810303688, "z": 0.159392312169075}, 
                    "vehicle.velocity": {"timestamp": 1561043647.7431362, "linear_velocity": [-6.0340113122947514e-05, -2.264878639834933e-05, 9.702569059300004e-07], 
                    "angular_velocity": [-8.910427823138889e-06, 1.6135254554683343e-06, 0.0005768465343862772], 
                    "world_linear_velocity": [-5.9287678595865145e-05, -2.5280191039200872e-05, 8.493661880493164e-07]}, 
                    "vehicle.pose": {"timestamp": 1561043647.7431362, "x": -0.0027823783457279205, "y": -0.026340210810303688, "z": 0.159392312169075, "yaw": 0.04371734336018562, "pitch": 0.0065115075558424, "roll": 0.022675735875964165}}
*/


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include "ardupilot_SITL_QUAD.h"
#include "sockets.h"
#include "sensors.h"



#define MOTOR_NUM 4

static WbDeviceTag motors[MOTOR_NUM];

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag emitter;


static double _linear_velocity[3] = {0.0,0.0,0.0};
static double v[MOTOR_NUM];
int port;
float dragFactor = VEHICLE_DRAG_FACTOR;

static int timestep;

#ifdef DEBUG_SENSORS
FILE *fptr;
#endif

#ifdef DEBUG_USE_KB
/*
// Code used tp simulae motors using keys to make sure that sensors directions and motor torques and thrusts are all correct.
// You can start this controller and use telnet instead of SITL to start the simulator.
Then you can use Keyboard to emulate motor input.
*/
static float factor = 1.0f;
static float offset = 0.0f;
void process_keyboard ()
{
  switch (wb_keyboard_get_key()) 
  {
    case 'Q':  // Q key -> up & left
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      v[3] = 0.0;
      break;

    case 'Y':
      v[0] = v[0] + 0.01;
      v[2] = v[2] - 0.01;
      break;

    case 'H':
      v[0] = v[0] - 0.01;
      v[2] = v[2] + 0.01;
      break;

    case 'G':
      v[1] = v[1] + 0.01;
      v[3] = v[3] - 0.01;
      break;

    case 'J':
      v[1] = v[1] - 0.01;
      v[3] = v[3] + 0.01;
      break;

    case 'E':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] += 0.00002;
      }
      break;

    case 'W':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] += 0.0002;
      }
      break;

    case 'S':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] -= 0.0002;
      }
      break;
  
    case 'A':
      v[1] = v[1] + 0.001;
      v[3] = v[3] + 0.001;
      v[0] = v[0] - 0.001;
      v[2] = v[2] - 0.001;
      break;

    case 'D':
      v[1] = v[1] - 0.001;
      v[3] = v[3] - 0.001;
      v[0] = v[0] + 0.001;
      v[2] = v[2] + 0.001;
      break;

    
  }

  for (int i=0; i< MOTOR_NUM; ++i)
  {
    
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], factor* v[i] + offset); 

    
  }
  
  printf ("Motors Internal %f %f %f %f\n", v[0],v[1],v[2],v[3]);
  
}
#endif


/*
// apply motor thrust.
*/
void update_controls()
{
  /*
      1 N = 101.97162129779 grams force
      Thrust = t1 * |omega| * omega - t2 * |omega| * V
      Where t1 and t2 are the constants specified in the thrustConstants field,
      omega is the motor angular velocity 
      and V is the component of the linear velocity of the center of thrust along the shaft axis.

      if Vehicle mass = 1 Kg. and we want omega = 1.0 to hover
      then (mass / 0.10197) / (4 motors) = t1

    LINEAR_THRUST
      we also want throttle to be linear with thrust so we use sqrt to calculate omega from input.
      Check this doc: https://docs.google.com/spreadsheets/d/1eR4Fb6cgaTb-BHUKJbhAXPzyX0ZLtUcEE3EY-wQYvM8/edit?usp=sharing
   */
#ifndef DEBUG_USE_KB
  //static float factor = 2.1f;
  //static float offset = 0.0f;
  
  
  //static float factor = 1.0f;
  static float offset = 0.0f;
  
  static float v[4];
  // pls check https://docs.google.com/spreadsheets/d/1eR4Fb6cgaTb-BHUKJbhAXPzyX0ZLtUcEE3EY-wQYvM8/edit?usp=sharing
  static float factorDyn[11] = {
            3.6f, // 0.0
            3.6f, // 0.1
            4.6f, // 0.2
            4.1f, // 0.3
            4.1f, // 0.4
            3.9f, // 0.5
            3.9f, // 0.6
            3.8f, // 0.7
            3.7f, // 0.8 
            3.6f, // 0.9 
            3.4f  // 1.0
          };
  //#define LINEAR_THRUST



#ifdef LINEAR_THRUST
  v[0] = sqrt(state.motors.w) * factor + offset;
  v[1] = sqrt(state.motors.x) * factor + offset;
  v[2] = sqrt(state.motors.y) * factor + offset;
  v[3] = sqrt(state.motors.z) * factor + offset;
#else  
  v[0] = (state.motors.w ) * factorDyn[(int)(10 * state.motors.w)] + offset;
  v[1] = (state.motors.x ) * factorDyn[(int)(10 * state.motors.x)]  + offset;
  v[2] = (state.motors.y ) * factorDyn[(int)(10 * state.motors.y)]  + offset;
  v[3] = (state.motors.z ) * factorDyn[(int)(10 * state.motors.z)]  + offset;
#endif //LINEAR_THRUST

  for ( int i=0; i<4; ++i)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 
  }

  #ifdef DEBUG_MOTORS
  printf ("RAW    F:%f  R:%f  B:%f  L:%f\n", state.motors.w, state.motors.x, state.motors.y, state.motors.z);
  printf ("Factors    F:%f  \n", factorDyn[(int)(10 * state.motors.w)]);
  printf ("Motors F:%f  R:%f  B:%f  L:%f\n", v[0], v[1], v[2], v[3]);
  
  #endif //DEBUG_MOTORS

#endif //DEBUG_USE_KB

  #ifdef WIND_SIMULATION
  /*
    Drag: Fd = ½ ρ Cd A v²

    Fd is drag force in Newtons
    ρ is the density of air in kg/m³
    Cd is the drag coefficient
    A is the cross section of our quad in m³ in the direction of movement
    v is the velocity in m/s
  */
  
  wind_webots_axis.x =  state.wind.x - linear_velocity[0];
  wind_webots_axis.z = -state.wind.y - linear_velocity[2];   // "-state.wind.y" as angle 90 wind is from EAST.
  wind_webots_axis.y =  state.wind.z - linear_velocity[1];
  

  wind_webots_axis.x = dragFactor * wind_webots_axis.x * abs(wind_webots_axis.x);
  wind_webots_axis.z = dragFactor * wind_webots_axis.z * abs(wind_webots_axis.z);
  wind_webots_axis.y = dragFactor * wind_webots_axis.y * abs(wind_webots_axis.y);

  wb_emitter_send(emitter, &wind_webots_axis, sizeof(VECTOR4F));
  
  #ifdef DEBUG_WIND
  printf("wind sitl: %f %f %f %f\n",state.wind.w, state.wind.x, state.wind.y, state.wind.z);
  printf("wind ctrl: (dragFactor) %f %f %f %f %f\n",dragFactor, wind_webots_axis.w, wind_webots_axis.x, wind_webots_axis.y, wind_webots_axis.z);
  #endif

  #endif
}


// data example: [my_controller_SITL] {"engines": [0.000, 0.000, 0.000, 0.000]}
// the JSON parser is directly inspired by https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Morse.cpp
bool parse_controls(const char *json)
{
    #ifdef DEBUG_INPUT_DATA
    printf("%s\n", json);
    #endif
    
    for (uint16_t i=0; i < ARRAY_SIZE(keytable); i++) {
        struct keytable *key;
        key = &keytable[i];
        // look for section header 
        const char *p = strstr(json, key->section);
        if (!p) {
            // we don't have this sensor
            continue;
        }
        p += strlen(key->section)+1;

        // find key inside section
        p = strstr(p, key->key);
        if (!p) {
            fprintf(stderr,"Failed to find key %s/%s DATA:%s\n", key->section, key->key, json);
            return false;
        }

        p += strlen(key->key)+3;
        
        switch (key->type) 
        {
          case DATA_FLOAT:
              *((float *)key->ptr) = atof(p);
              #ifdef DEBUG_INPUT_DATA
              printf("GOT  %s/%s\n", key->section, key->key);
              #endif
              break;

          case DATA_DOUBLE:
              *((double *)key->ptr) = atof(p);
              #ifdef DEBUG_INPUT_DATA
              printf("GOT  %s/%s\n", key->section, key->key);
              #endif
              break;

          case DATA_VECTOR4F: {
              VECTOR4F *v = (VECTOR4F *)key->ptr;
              if (sscanf(p, "[%f, %f, %f, %f]", &(v->w), &(v->x), &(v->y), &(v->z)) != 4) {
                  fprintf(stderr,"Failed to parse Vector3f for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else
              {
                  #ifdef DEBUG_INPUT_DATA
                  printf("GOT  %s/%s\n[%f, %f, %f, %f]\n ", key->section, key->key,v->w,v->x,v->y,v->z);
                  #endif
              }
              
              break;
              }
        }
    }
    return true;
}

void run ()
{

    char send_buf[1000]; 
    char command_buffer[1000];
    fd_set rfds;
    
    // calculate initial sensor values.
    wb_robot_step(timestep);
    
    while (true) 
    {
        #ifdef DEBUG_USE_KB
        process_keyboard();
        #endif

        if (fd == 0) 
        {
          // if no socket wait till you get a socket
            fd = socket_accept(sfd);
            if (fd < 0)
              break;
        }
         
        
        // trigget ArduPilot to send motor data 
        getAllSensors ((char *)send_buf, gyro,accelerometer,compass,gps, inertialUnit);

        #ifdef DEBUG_SENSORS
        //printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        if (strlen (pBug)> 5)
        {
        // fprintf(fptr, "%s\n",pBug);
        }
        #endif
         
        
        if (write(fd,send_buf,strlen(send_buf)) <= 0)
        {
          fprintf (stderr,"Send Data Error\n");
        }

        if (fd) 
        {
          FD_ZERO(&rfds);
          FD_SET(fd, &rfds);
          struct timeval tv;
          tv.tv_sec = 0.05;
          tv.tv_usec = 0;
          int number = select(fd + 1, &rfds, NULL, NULL, &tv);
          if (number != 0) 
          {
            // there is a valid connection
                int n = recv(fd, (char *)command_buffer, 1000, 0);

                if (n < 0) {
        #ifdef _WIN32
                  int e = WSAGetLastError();
                  if (e == WSAECONNABORTED)
                    fprintf(stderr, "Connection aborted.\n");
                  else if (e == WSAECONNRESET)
                    fprintf(stderr, "Connection reset.\n");
                  else
                    fprintf(stderr, "Error reading from socket: %d.\n", e);
        #else
                  if (errno)
                    fprintf(stderr, "Error reading from socket: %d.\n", errno);
        #endif
                  break;
                }
                if (n==0)
                {
                  break;
                }
                if (n > 0)
                {
                  command_buffer[n] = 0;
                  if (parse_controls (command_buffer))
                  {
                    update_controls();
                    //https://cyberbotics.com/doc/reference/robot#wb_robot_step
                    // this is used to force webots not to execute untill it receives feedback from simulator.
                    wb_robot_step(timestep);
                  }

                }
          }
          
        }
    }
    
    socket_cleanup();
}


bool initialize (int argc, char *argv[])
{
  
  fd_set rfds;
  #ifdef DEBUG_SENSORS
  fptr = fopen ("/tmp/log.txt","w");
  #endif
  port = 5599;  // default port
  for (int i = 0; i < argc; ++i)
  {
      if (strcmp (argv[i],"-p")==0)
      { // specify port for SITL.
        if (argc > i+1 )
        {
          port = atoi (argv[i+1]);
          printf("socket port %d\n",port);
        }
      }
      else if (strcmp (argv[i],"-df")==0)
      { // specify drag functor used to simulate air resistance.
        if (argc > i+1 )
        {
          dragFactor = atof (argv[i+1]);
          printf("drag Factor %f\n",dragFactor);
        }
        else
        {
          fprintf(stderr,"Missing drag factor value.\n");
          return false;
        }
        
      }
  }
    
    
  sfd = create_socket_server(port);
  
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  timestep = (int)wb_robot_get_basic_time_step();
  timestep_scale = timestep * 1000.0;
  printf("timestep_scale: %f \n", timestep_scale);
  
  
  // keybaard
  #ifdef DEBUG_USE_KB
  wb_keyboard_enable(timestep);
  #endif


  // inertialUnit
  inertialUnit = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(inertialUnit, timestep);

  // gyro
  gyro = wb_robot_get_device("gyro1");
  wb_gyro_enable(gyro, timestep);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer1");
  wb_accelerometer_enable(accelerometer, timestep);
  
  // compass
  compass = wb_robot_get_device("compass1");
  wb_compass_enable(compass, timestep);

  // gps
  gps = wb_robot_get_device("gps1");
  wb_gps_enable(gps, timestep);

  // camera
  camera = wb_robot_get_device("camera1");
   wb_camera_enable(camera, CAMERA_FRAME_RATE_FACTOR * timestep);

  #ifdef WIND_SIMULATION
  // emitter
  emitter = wb_robot_get_device("emitter_plugin");
  #endif

  // names of motor should be the same as name of motor in the robot.
  const char *MOTOR_NAMES[] = {"motor1", "motor2", "motor3", "motor4"};
  
  // get motor device tags
  for (int i = 0; i < MOTOR_NUM; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    v[i] = 0.0f;
  }
  
  FD_ZERO(&rfds);
  FD_SET(sfd, &rfds);

  // init linear_velocity untill we receive valid data from Supervisor.
  linear_velocity = &_linear_velocity[0] ;


  return true;
}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{

  

  if (initialize( argc, argv))
  {
  
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
    run();
  }

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

  return 0;
}
