
/*
 * File: ardupilot_SITL_TRICOPTER.c
 * Date: 18 Aug 2019
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include "ardupilot_SITL_TRICOPTER.h"
#include "sockets.h"
#include "sensors.h"


#define MOTOR_NUM 3

static WbDeviceTag motors[MOTOR_NUM];
static WbDeviceTag servo;
static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag receiver;

#ifdef WIND_SIMULATION
static WbDeviceTag emitter;
#endif
static double _linear_velocity[3] = {0.0,0.0,0.0};
static double northDirection = 1;
static double v[MOTOR_NUM];
static double servo_value = 0;
#ifdef DEBUG_USE_KB
static double servo_value_extra = 0;
#endif

int port;

int timestep;


#ifdef DEBUG_USE_KB
/*
// Code used tp simulae motors using keys to make sure that sensors directions and motor torques and thrusts are all correct.
// You can start this controller and use telnet instead of SITL to start the simulator.
Then you can use Keyboard to emulate motor input.
*/
void process_keyboard ()
{
  switch (wb_keyboard_get_key()) 
  {
    case 'Q':  // Q key -> up & left
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      servo_value_extra = 0.0;
      break;

    case 'Y':
      v[0] = v[0] + 0.01;
      v[1] = v[1] + 0.01;
      v[2] = v[2] - 0.02;
      break;

    case 'H':
      v[0] = v[0] - 0.01;
      v[1] = v[1] - 0.01;
      v[2] = v[2] + 0.02;
      break;

    case 'G':
      v[0] = v[0] + 0.01;
      v[1] = v[1] - 0.01;
      break;

    case 'J':
      v[0] = v[0] - 0.01;
      v[1] = v[1] + 0.01;
      break;

    case 'W':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] += 0.01;
      }
      break;

    case 'S':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] -= 0.01;
      }
      break;
  
    case 'A':
      servo_value_extra = servo_value_extra + 0.01;
      break;

    case 'D':
      servo_value_extra = servo_value_extra - 0.01;
      break;

    
  }

  for (int i=0; i< MOTOR_NUM; ++i)
  {
    if (v[i] <=0) v[i]=0;
    if (v[i] >=600) v[i]=10;

    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 
  }

  wb_motor_set_position (servo, servo_value_extra);
  wb_motor_set_velocity (servo, 100);
  
  
  printf ("Motors Internal right:%f left:%f back:%f servo:%f\n", v[0],v[1],v[2],servo_value);
  
}
#endif

// Read all messages and empty the Q and keep last value as the valid one.
static void read_incoming_messages() 
{
   // read while queue not empty
   while (wb_receiver_get_queue_length(receiver) > 0) {
     // I'm only expecting ascii messages
     double * data = (double *) wb_receiver_get_data(receiver);
     _linear_velocity[0] = data[0];
     _linear_velocity[1] = data[1];
     _linear_velocity[2] = data[2];
     northDirection      = data[3];
     //printf("RAW Data [%f, %f, %f]\n", linear_velocity[0], linear_velocity[2], linear_velocity[1]);
    
     wb_receiver_next_packet(receiver);
   }
 }

/*
// apply motor thrust.
*/
void update_controls()
{
  /*
      1 N = 101.9716213 grams force
      Thrust = t1 * |omega| * omega - t2 * |omega| * V
      Where t1 and t2 are the constants specified in the thrustConstants field,
      omega is the motor angular velocity 
      and V is the component of the linear velocity of the center of thrust along the shaft axis.

      if Vehicle mass = 1 Kg. and we want omega = 1.0 to hover
      then (mass / 0.10197) / (4 motors) = t1

    LINEAR_THRUST
      we also want throttle to be linear with thrust so we use sqrt to calculate omega from input.
   */
  static float factor = 1.0f;
  static float offset = 0.0f;
  static float v[MOTOR_NUM];
  
#ifdef LINEAR_THRUST
  v[0] = sqrt(state.motors.w ) * factor + offset;
  v[1] = sqrt(state.motors.x ) * factor + offset;
  v[2] = sqrt(state.motors.z ) * factor + offset;
#else  
  v[0] = (state.motors.w ) * factor + offset;
  v[1] = (state.motors.x ) * factor + offset;
  v[2] = (state.motors.z ) * factor + offset;
#endif

  servo_value = -state.motors.y ;

  for ( int i=0; i<MOTOR_NUM; ++i)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 
  }

#ifdef DEBUG_USE_KB
  wb_motor_set_position(servo, servo_value + servo_value_extra);
#else
  wb_motor_set_position(servo, servo_value);
#endif
  wb_motor_set_velocity(servo, 1000); 

  #ifdef DEBUG_MOTORS
  printf ("RAW    R:%f L:%f SRV:%f B:%f\n", state.motors.w, state.motors.x, state.motors.y, state.motors.z);
  printf ("Motors R:%f L:%f SRV:%f B:%f\n", v[0], v[1], servo_value, v[2]);
  #endif


#ifdef WIND_SIMULATION
  
  double linear_speed = sqrt(linear_velocity[0] * linear_velocity[0] + linear_velocity[1] * linear_velocity[1] + linear_velocity[2] * linear_velocity[2]);
  wind_webots_axis.w =  state.wind.w + 0.01 * linear_speed * linear_speed;
  
  if (northDirection == 1)
  {
    wind_webots_axis.x =  state.wind.x - linear_velocity[0];
    wind_webots_axis.z = -state.wind.y - linear_velocity[2];   // "-state.wind.y" as angle 90 wind is from EAST.
    wind_webots_axis.y =  state.wind.z - linear_velocity[1];
  }
  else
  { // as in pyramids and any open map street world.
    wind_webots_axis.x =  state.wind.y  - linear_velocity[0]; // always add "linear_velocity" as there is no axis transformation here.
    wind_webots_axis.z =  -state.wind.x - linear_velocity[2];
    wind_webots_axis.y =  state.wind.z  - linear_velocity[1];
  }

  wb_emitter_send(emitter, &wind_webots_axis, sizeof(VECTOR4F));

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
            printf("Failed to find key %s/%s DATA:%s\n", key->section, key->key, json);
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
                  printf("Failed to parse Vector3f for %s %s/%s\n",p,  key->section, key->key);
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

#define TIME_DIV 1000.0
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

        
        
        read_incoming_messages();
        
        
        // trigget ArduPilot to send motor data
        getAllSensors ((char *)send_buf, northDirection, gyro,accelerometer,compass,gps, inertialUnit);

        #ifdef DEBUG_SENSORS
        printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
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
          
  port = 5599;  // default port
  for (int i = 0; i < argc; ++i)
  {
        if (strcmp (argv[i],"-p")==0)
        {
          if (argc > i+1 )
          {
            port = atoi (argv[i+1]);
            printf("socket port %d\n",port);
          }
        }
        else if (strcmp (argv[i],"-df")==0)
        { 
          //TODO: to be implemented later. use same logic as in Quad file.

        }
  }
    
  sfd = create_socket_server(port);
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  timestep = (int)wb_robot_get_basic_time_step();
  
  // init receiver from Supervisor
  receiver = wb_robot_get_device("receiver_main");
  if (receiver ==0)
  {
    fprintf(stderr,"Receiver not found\n");
    fprintf(stderr,"EXIT with error\n");
    return false;
  }

  // read robot number and set receiver channel accordingly
  const char * customData = wb_robot_get_custom_data();
  if (customData != NULL)
  {
    int channel = atoi(customData);
    wb_receiver_set_channel(receiver,channel);
    wb_receiver_enable(receiver,timestep);
    printf("Receiver Channel at %d \n", channel);
  }
  else
  {
    fprintf(stderr, "MISSING Channel NO. in Custom Data");
    return false;
  }
  
  
  
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
  const char *MOTOR_NAMES[] = {"motor1", "motor2", "motor3"};
  
  // get motor device tags
  for (int i = 0; i < MOTOR_NUM; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    v[i] = 0.0f;
  }
  
  // tricopter servo name
  servo = wb_robot_get_device("servo_tail");

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
