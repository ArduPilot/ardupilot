/*
 * File: ardupilot_SITL_QUAD.c
 * Date: 29 July 2019
 * Description: integration with ardupilot SITL simulation.
 * Author: M.S.Hefny (HefnySco)
 * Modifications:
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
static WbNodeRef world_info;

static const double *northDirection;
static double v[MOTOR_NUM];
int port;
float drafFactor = VEHICLE_DRAG_FACTOR;

static int timestep;


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
      v[1] = v[1] + 0.01;
      v[3] = v[3] + 0.01;
      v[0] = v[0] - 0.01;
      v[2] = v[2] - 0.01;
      break;

    case 'D':
      v[1] = v[1] - 0.01;
      v[3] = v[3] - 0.01;
      v[0] = v[0] + 0.01;
      v[2] = v[2] + 0.01;
      break;

    
  }

  for (int i=0; i< MOTOR_NUM; ++i)
  {
    if (v[i] <=0) v[i]=0;
    if (v[i] >=600) v[i]=10;

    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 

    
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
  static float v[4];
  
#ifdef LINEAR_THRUST
  v[0] = sqrt(state.motors.w ) * factor + offset;
  v[1] = sqrt(state.motors.x ) * factor + offset;
  v[2] = sqrt(state.motors.y ) * factor + offset;
  v[3] = sqrt(state.motors.z ) * factor + offset;
#else  
  v[0] = (state.motors.w ) * factor + offset;
  v[1] = (state.motors.x ) * factor + offset;
  v[2] = (state.motors.y ) * factor + offset;
  v[3] = (state.motors.z ) * factor + offset;
#endif

  for ( int i=0; i<4; ++i)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 
  }

  #ifdef DEBUG_MOTORS
  printf ("RAW    F:%f  R:%f  B:%f  L:%f\n", state.motors.w, state.motors.x, state.motors.y, state.motors.z);
  printf ("Motors F:%f  R:%f  B:%f  L:%f\n", v[0], v[1], v[2], v[3]);
  #endif


  #ifdef WIND_SIMULATION
  /*
    Drag: Fd = ½ ρ Cd A v²

    Fd is drag force in Newtons
    ρ is the density of air in kg/m³
    Cd is the drag coefficient
    A is the cross section of our quad in m³ in the direction of movement
    v is the velocity in m/s
  */
  if (northDirection[0] == 1)
  {
    wind_webots_axis.x =  state.wind.x - linear_velocity[0];
    wind_webots_axis.z = -state.wind.y - linear_velocity[2];   // "-state.wind.y" as angle 90 wind is from EAST.
    wind_webots_axis.y =  state.wind.z - linear_velocity[1];
  }
  else
  { // as in pyramids and any open map street world.
    wind_webots_axis.x =  state.wind.y - linear_velocity[0]; // always add "linear_velocity" as there is no axis transformation here.
    wind_webots_axis.z = -state.wind.x - linear_velocity[2];
    wind_webots_axis.y =  state.wind.z - linear_velocity[1];
  }

  wind_webots_axis.x = drafFactor * wind_webots_axis.x * abs(wind_webots_axis.x);
  wind_webots_axis.z = drafFactor * wind_webots_axis.z * abs(wind_webots_axis.z);
  wind_webots_axis.y = drafFactor * wind_webots_axis.y * abs(wind_webots_axis.y);

  wb_emitter_send(emitter, &wind_webots_axis, sizeof(VECTOR4F));
  
  #ifdef DEBUG_WIND
  printf("wind sitl: %f %f %f %f\n",state.wind.w, state.wind.x, state.wind.y, state.wind.z);
  printf("wind ctrl: (drafFactor) %f %f %f %f %f\n",drafFactor, wind_webots_axis.w, wind_webots_axis.x, wind_webots_axis.y, wind_webots_axis.z);
  #endif

  #endif
}


// data example: [my_controller_SITL] {"engines": [0.000, 0.000, 0.000, 0.000]}
// the JSON parser is directly inspired by https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Morse.cpp
bool parse_controls(const char *json)
{
    //state.timestamp = 1.0;
    #ifdef DEBUG_INPUT_DATA
    printf("%s\n", json);
    #endif
    
    for (uint16_t i=0; i < ARRAY_SIZE(keytable); i++) {
        struct keytable *key;
        key = &keytable[i];
        //printf("search   %s/%s\n", key->section, key->key);
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
            printf("Failed to find key %s/%s\n", key->section, key->key);
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


void run ()
{

    char send_buf[1000]; //1000 just a safe margin
    char command_buffer[200];
    fd_set rfds;
    while (wb_robot_step(timestep) != -1) 
    {
        #ifdef DEBUG_USE_KB
        process_keyboard();
        #endif

        if (fd == 0) 
        {
          // if no socket wait till you get a socket
            fd = socket_accept(sfd);
            if (fd > 0)
              socket_set_non_blocking(fd);
            else if (fd < 0)
              break;
        }
         
        getAllSensors ((char *)send_buf, northDirection, gyro,accelerometer,compass,gps, inertialUnit);

        #ifdef DEBUG_SENSORS
        printf("%s\n",send_buf);
        #endif
        
        if (write(fd,send_buf,strlen(send_buf)) <= 0)
        {
          printf ("Send Data Error\n");
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
                
                int n = recv(fd, (char *)command_buffer, 200, 0);
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
                if (command_buffer[0] == 'e')
                {
                  break;
                }
                if (n > 0)
                {

                  //printf("Received %d bytes:\n", n);
                  command_buffer[n] = 0;
                  parse_controls (command_buffer);
                  update_controls();
                  

                }
          }
          
        }
    }
    
    socket_cleanup();
}


void initialize (int argc, char *argv[])
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
          if (argc > i+1 )
          {
            drafFactor = atof (argv[i+1]);
            printf("drag Factor %f\n",drafFactor);
          }
        }
    }
    
    
  sfd = create_socket_server(port);
  
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  // Get WorldInfo Node.
  WbNodeRef root, node;
  WbFieldRef children, field;
  int n, i;
  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");
  n = wb_supervisor_field_get_count(children);
  printf("This world contains %d nodes:\n", n);
  for (i = 0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(children, i);
    if (wb_supervisor_node_get_type(node) == WB_NODE_WORLD_INFO)
    {
      world_info = node; 
      break;
    }
  }

  node = wb_supervisor_field_get_mf_node(children, 0);
  field = wb_supervisor_node_get_field(node, "northDirection");
  northDirection = wb_supervisor_field_get_sf_vec3f(field);
  
  if (northDirection[0] == 1)
  {
    printf ("Axis Default Directions\n");
  }

  printf("WorldInfo.northDirection = %g %g %g\n\n", northDirection[0], northDirection[1], northDirection[2]);



  
  // get Self Node
  self_node = wb_supervisor_node_get_self();

  
  // keybaard
  timestep = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(timestep);



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
   wb_camera_enable(camera, timestep);

  #ifdef WIND_SIMULATION
  // emitter
  emitter = wb_robot_get_device("emitter_plugin");
  #endif
  
  const char *MOTOR_NAMES[] = {"motor1", "motor2", "motor3", "motor4"};
  
  // get motor device tags
  for (i = 0; i < MOTOR_NUM; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    v[i] = 0.0f;
    //assert(motors[i]);
  }
  
  FD_ZERO(&rfds);
  FD_SET(sfd, &rfds);
}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{

  

  initialize( argc, argv);
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
    run();


    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

  return 0;
}
