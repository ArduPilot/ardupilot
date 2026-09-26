/*
 * File:          ardupilot_SITL_ROV.c
 * Date:          July 2019
 * Description: integration with ardupilot SITL simulation.
 * Author: M.S.Hefny (HefnySco)
 * Modifications:
 *  - Blocking sockets
 *  - Advance simulation time only when receive motor data.
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include "ardupilot_SITL_ROVER.h"
#include "sitl_link.h"
#include "sensors.h"




#define MOTOR_NUM 2

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;


/*
  Full-scale commands.  SIM_Webots.cpp sends steering and throttle as -1..1;
  these map them to the Webots driver library's steering angle and cruising
  speed.  Override with controllerArgs "-sa <rad>" and "-ms <m/s>", and keep
  rover.parm's CRUISE_THROTTLE and steering gains in step if you do.
*/
static double max_speed = 27.0;          /* m/s at full throttle */
static double max_steering_angle = 0.7;  /* rad at full steering, +ve = right */

static double v[MOTOR_NUM];
int port;

static int timestep;



#ifdef DEBUG_USE_KB
/*
// Code used tp simulae motors using keys to make sure that sensors directions and motor torques and thrusts are all correct.
// You can start this controller and use telnet instead of SITL to start the simulator.
Then you can use Keyboard to emulate motor input.
*/
void process_keyboard (void)
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
void update_controls(void)
{
  const double cruise_speed = state.rover.y * max_speed * 3.6;   /* km/h */
  const double steer_angle  = state.rover.x * max_steering_angle;
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
            printf("Failed to find key %s/%s DATA:%s\n", key->section, key->key, json);
            return false;
        }

        p += strlen(key->key)+3;
        
        switch (key->type) 
        {
          case DATA_FLOAT:
              *((float *)key->ptr) = strtof(p, NULL);
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



/*
  Motors off, tail servo centred, wheels stopped: what the vehicle should do
  until a newly connected SITL says otherwise.
*/
static void reset_controls(void)
{
  memset(&state, 0, sizeof(state));
  update_controls();
}

void run (void)
{
    char send_buf[1200];
    bool reconnecting = false;

    vehicle_pose_save();

    // calculate initial sensor values.
    if (wbu_driver_step() == -1) {
      return;
    }

    while (true)
    {
        #ifdef DEBUG_USE_KB
        process_keyboard();
        #endif

        if (!sitl_link_connected())
        {
          if (!sitl_link_accept()) {
            break;
          }
          if (reconnecting) {
            /* SITL was restarted: start the new session from where the world
               placed the vehicle, not wherever the last one left it */
            reset_controls();
            vehicle_pose_restore();
            if (wbu_driver_step() == -1) {
              break;
            }
          }
          reconnecting = true;
        }

        // trigger ArduPilot to send motor data
        getAllSensors ((char *)send_buf, gyro,accelerometer,compass,gps, inertialUnit, "");

        #ifdef DEBUG_SENSORS
        printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        #endif

        if (sitl_link_exchange(send_buf, parse_controls) != SITL_LINK_CONTROLS) {
          /* nothing yet: re-send this frame, or wait for a reconnect */
          continue;
        }

        update_controls();
        /* this is used to force webots not to execute until it receives
           feedback from the simulator.  With the driver library,
           wbu_driver_step() replaces wb_robot_step(): it steps the robot and
           also updates the car's wheel speeds, brakes and lights.
           https://cyberbotics.com/doc/automobile/driver-library */
        if (wbu_driver_step() == -1) {
          break;
        }
    }
    sitl_link_close();
}


/* value following argv[i], or NULL (with an error) if there is none */
static const char *arg_value(int argc, char *argv[], int i)
{
  if (i + 1 < argc) {
    return argv[i + 1];
  }
  fprintf(stderr, "Missing value for %s.\n", argv[i]);
  return NULL;
}

bool initialize (int argc, char *argv[])
{
  port = 5599;  // default port
  for (int i = 0; i < argc; ++i)
  {
    const char *value = NULL;
    if (strcmp(argv[i], "-p") == 0)
    { // port SITL connects to
      if ((value = arg_value(argc, argv, i)) == NULL) {
        return false;
      }
      port = atoi(value);
    }
    else if (strcmp(argv[i], "-ms") == 0)
    { // speed at full throttle, m/s
      if ((value = arg_value(argc, argv, i)) == NULL) {
        return false;
      }
      max_speed = strtod(value, NULL);
    }
    else if (strcmp(argv[i], "-sa") == 0)
    { // steering angle at full steering, rad
      if ((value = arg_value(argc, argv, i)) == NULL) {
        return false;
      }
      max_steering_angle = strtod(value, NULL);
    }
  }
  printf("port %d, max speed %.2f m/s, max steering angle %.3f rad\n",
         port, max_speed, max_steering_angle);

  if (!sitl_link_open(port)) {
    return false;
  }

  /* wbu_driver_init() replaces wb_robot_init() for vehicles driven through the
     Webots driver library; calling both is outside the documented use */
  wbu_driver_init();

  /*
    The old code walked WorldInfo looking for a "northDirection" field.  That
    field was removed in Webots R2022a when WorldInfo.coordinateSystem replaced
    it, so the lookup returns NULL on every supported version and the value was
    only ever printed.  The axis convention now comes from the world's
    coordinateSystem, which sensors.c documents.
  */

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
  wb_camera_enable(camera, CAMERA_FRAME_RATE_FACTOR * timestep);

  return true;
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* initialize() only fails before wbu_driver_init(), so only clean up after
     it succeeded */
  if (initialize(argc, argv)) {
    run();
    /* replaces wb_robot_cleanup(), like wbu_driver_init() replaces wb_robot_init() */
    wbu_driver_cleanup();
  }

  return 0;
}
