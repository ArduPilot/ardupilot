/*
 * File:          ardupilot_SITL_ROV.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <webots/vehicle/car.h>

#include <webots/vehicle/driver.h>
#include "ardupilot_SITL_ROVER.h"
#include "sockets.h"
#include "sensors.h"


#define MOTOR_NUM 2

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag car;
static WbNodeRef world_info;

static const double *northDirection;


const float max_speed = 27; //m/s 

static double v[MOTOR_NUM];
int port;

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
      break;

    case 'W':
      v[1] += 0.01;
      break;

    case 'S':
      v[1] -= 0.01;
      break;
  
    case 'A':
      v[0] = v[0] + 0.01;
      break;

    case 'D':
      v[0] = v[0] - 0.01;
      break;

    
  }

  wbu_driver_set_cruising_speed (v[1]);
  wbu_driver_set_steering_angle (v[0]);
  
  printf ("Motors Internal %f %f\n", v[0],v[1]);
  
}
#endif




/*
// apply motor thrust.
*/
void update_controls()
{
  float cruise_speed = state.rover.y * max_speed * 3.6f;
  float steer_angle =  state.rover.x  * 0.7f;
  wbu_driver_set_cruising_speed (cruise_speed + v[1]);
  wbu_driver_set_steering_angle (steer_angle + v[0]);
  
  #ifdef DEBUG_MOTORS
  printf("cruise speed: %f steering angle: %f\n", cruise_speed, steer_angle);
  #endif
}



bool parse_controls(const char *json)
{
    //state.timestamp = 1.0;
    #ifdef DEBUG_INPUT_DATA
    printf("%s\n", json);
    #endif
    
    for (uint16_t i=0; i < ARRAY_SIZE(keytableROV); i++) {
        struct keytableROV *key;
        key = &keytableROV[i];
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
          case DATA_VECTOR2F:
          {
              VECTOR2F *v = (VECTOR2F *)key->ptr;
              if (sscanf(p, "[%f, %f]", &(v->x), &(v->y)) != 2) {
                  printf("Failed to parse Vector2f for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else
              {
                  #ifdef DEBUG_INPUT_DATA
                  printf("GOT  %s/%s [%f, %f]\n ", key->section, key->key,v->x,v->y);
                  #endif
              }
              break;
          }
          case DATA_VECTOR4F: {
              VECTOR4F *v = (VECTOR4F *)key->ptr;
              if (sscanf(p, "[%f, %f, %f, %f]", &(v->w), &(v->x), &(v->y), &(v->z)) != 4) {
                  printf("Failed to parse Vector4f for %s %s/%s\n",p,  key->section, key->key);
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
          }
        }
    }
    
    
  sfd = create_socket_server(port);
  
  
  

  /* necessary to initialize webots stuff */
  wb_robot_init();
  wbu_driver_init (); 
  

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

  printf("\n");
  node = wb_supervisor_field_get_mf_node(children, 0);
  field = wb_supervisor_node_get_field(node, "northDirection");
  northDirection = wb_supervisor_field_get_sf_vec3f(field);
  
  if (northDirection[0] == 1)
  {
    printf ("Axis Default Directions");
  }

  printf("WorldInfo.northDirection = %g %g %g\n\n", northDirection[0], northDirection[1], northDirection[2]);



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


  car = wb_robot_get_device ("rover");

  FD_ZERO(&rfds);
  FD_SET(sfd, &rfds);
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


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
  run();

  /* Enter your cleanup code here */

  wbu_driver_cleanup();
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
