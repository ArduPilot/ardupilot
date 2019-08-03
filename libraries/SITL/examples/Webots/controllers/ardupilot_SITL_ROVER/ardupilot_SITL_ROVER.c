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
#include <webots/vehicle/driver.h>
#include <webots/supervisor.h>
#include "ardupilot_SITL_ROVER.h"
#include "sockets.h"
#include "sensors.h"


#define WHEELS 4

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag wheels[4];

int port;

static int timestep;

/*
// apply motor thrust.
*/
void update_controls()
{
  
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
         
        getAllSensors ((char *)send_buf, gyro,accelerometer,compass,gps, inertialUnit);

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


  /*
  const char *MOTOR_NAMES[] = {"motor1", "motor2", "motor3", "motor4"};
  
  // get motor device tags
  int i;
  for (i = 0; i < MOTOR_NUM; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    v[i] = 0.0f;
    //assert(motors[i]);
  }
  */

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

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
