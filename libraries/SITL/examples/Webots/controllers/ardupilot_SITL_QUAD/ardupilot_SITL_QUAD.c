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
  Wire protocol, line-delimited JSON in both directions.

  webots -> SITL, one line per simulation step:

  {"ts": 1561043647.759803,
   "vehicle.imu": {"av": [...], "la": [...], "mf": [...]},
   "vehicle.gps": {"x": ..., "y": ..., "z": ...},
   "vehicle.velocity": {"wlv": [...]},
   "vehicle.pose": {"x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ...},
   "rpm": [...]}

  SITL -> webots:

  {"pwm": [16 values, 1000..2000], "wnd": [speed, north, east, down]}
*/


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include "ardupilot_SITL_QUAD.h"
#include "sitl_link.h"
#include "sensors.h"
#include "rotor_speed.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



#define MOTOR_NUM 4

static WbDeviceTag motors[MOTOR_NUM];

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag emitter;

/*
  ArduPilot SERVOn feeding each Webots motor device.  motors[i] is named
  MOTOR_NAMES[i] in the world file, and is driven by SITL output channel
  MOTOR_SERVO_CHANNEL[i] (zero based, so 2 == SERVO3).
*/
static const int MOTOR_SERVO_CHANNEL[MOTOR_NUM] = { 2, 0, 3, 1 };

/* rotor angular velocity setpoint, rad/s, indexed like motors[] */
static double rotor_velocity[MOTOR_NUM];

/* estimated rotor shaft speed, rad/s, slewing towards rotor_velocity (rotor_speed.h) */
static double rotor_speed[MOTOR_NUM];

/* maximum rotor angular velocity, rad/s, read from the world's RotationalMotor */
static double motor_max_velocity[MOTOR_NUM];

/* fastest a rotor can change speed, rad/s^2: its RotationalMotor's maxTorque
   (see rotor_speed.h) */
static double motor_max_accel[MOTOR_NUM];

/* servo channels covered by the "rpm" array sent to SITL */
#define RPM_CHANNELS 4

int port;
float dragFactor = VEHICLE_DRAG_FACTOR;

static int timestep;

#ifdef DEBUG_SENSORS
FILE *fptr;
#endif

/**
// apply motor thrust.
*/
void update_controls(void)
{
  /*
      Webots' Propeller node computes

          Thrust = t1 * |omega| * omega - t2 * |omega| * V
          Torque = c1 * |omega| * omega - c2 * |omega| * V

      where t1/t2 and c1/c2 are the thrustConstants and torqueConstants fields,
      omega is the motor angular velocity and V is the component of the linear
      velocity of the centre of thrust along the shaft axis.
      See https://cyberbotics.com/doc/reference/propeller

      Thrust is therefore quadratic in omega.  ArduPilot's mixer assumes thrust
      is linear in its 0..1 output once MOT_THST_EXPO is 0, so we linearise here
      by commanding omega = sqrt(u) * omega_max.  That makes

          Thrust(u) = t1 * omega_max^2 * u

      exactly linear in u.  The .parm files shipped alongside set MOT_THST_EXPO 0
      to match; if you raise MOT_THST_EXPO you are asking ArduPilot to compensate
      for a propeller curve that this controller has already removed.

      This replaces the old `factorDyn` lookup table, which indexed
      factorDyn[10 * (int)u] -- and (int)u is 0 for every u below 1.0, so the
      eleven-entry curve only ever yielded its first and last entries.
   */
  for (int i = 0; i < MOTOR_NUM; ++i) {
    const int ch = MOTOR_SERVO_CHANNEL[i];
    double u = (state.motors.v[ch] - 1000.0) * 0.001;   /* 1000..2000 -> 0..1 */

    if (u < 0.0) {
      u = 0.0;
    } else if (u > 1.0) {
      u = 1.0;
    }

    rotor_velocity[i] = sqrt(u) * motor_max_velocity[i];

    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], rotor_velocity[i]);
  }
  /* update_controls() runs once per wb_robot_step(), so this advances the
     estimate to the end of the step the next sensor frame is sampled at */
  rotor_speed_update(rotor_speed, rotor_velocity, motor_max_accel, MOTOR_NUM,
                     timestep * 0.001);


  #ifdef WIND_SIMULATION
  /*
    Drag: Fd = ½ ρ Cd A v²

    Fd is drag force in Newtons
    ρ is the density of air in kg/m³
    Cd is the drag coefficient
    A is the cross section of our quad in m² in the direction of movement
    v is the velocity in m/s

    v here is the airspeed, i.e. wind velocity minus vehicle velocity, so that
    the force falls to zero once the vehicle is carried along with the air mass.
    The vehicle velocity used to come from a `linear_velocity` pointer that was
    initialised to a zero array "until we receive valid data from Supervisor"
    and then never reassigned, so drag was previously computed against a
    stationary vehicle.

    state.wind holds ArduPilot's earth-frame wind vector, NED:
      state.wind.w = speed, .x = north, .y = east, .z = down.
    The worlds are WorldInfo.coordinateSystem "NUE", so North/Up/East.
  */
  const double *vehicle_vel = wb_gps_get_speed_vector(gps);   /* NUE, m/s */

  wind_webots_axis.x =  state.wind.x - vehicle_vel[0];   /* north */
  wind_webots_axis.y = -state.wind.z - vehicle_vel[1];   /* up, from NED down */
  wind_webots_axis.z =  state.wind.y - vehicle_vel[2];   /* east */

  /* fabsf, not abs: abs() is int abs(int), which truncated every apparent wind
     below 1 m/s to exactly zero and quantised everything above it. */
  wind_webots_axis.x = dragFactor * wind_webots_axis.x * fabsf(wind_webots_axis.x);
  wind_webots_axis.y = dragFactor * wind_webots_axis.y * fabsf(wind_webots_axis.y);
  wind_webots_axis.z = dragFactor * wind_webots_axis.z * fabsf(wind_webots_axis.z);

  wind_webots_axis.w = 0.0f;   /* unused by the physics plugin, but keep it defined */

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
            // we don't have this section
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

          case DATA_VECTOR4F: {
             VECTOR4F *v = (VECTOR4F *)key->ptr;
              if (sscanf(p, "[%f, %f, %f, %f]", &(v->w), &(v->x), &(v->y), &(v->z)) != 4) {
                  fprintf(stderr,"Failed to parse Vector3f for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else {
                  #ifdef DEBUG_INPUT_DATA
                  printf("GOT  %s/%s\n[%f, %f, %f, %f]\n ", key->section, key->key,v->w,v->x,v->y,v->z);
                  #endif
              }
            break;
            }

          case DATA_VECTOR16F: {
              VECTOR16F *v = (VECTOR16F *)key->ptr;
              if (sscanf(p, "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]", &(v->v[0]), &(v->v[1]), &(v->v[2]), &(v->v[3])
                  , &(v->v[4]), &(v->v[5]), &(v->v[6]), &(v->v[7])
                  , &(v->v[8]), &(v->v[9]), &(v->v[10]), &(v->v[11])
                  , &(v->v[12]), &(v->v[13]), &(v->v[14]), &(v->v[15])
                ) != 16) {
                  printf("Failed to parse DATA_VECTOR16F for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else {
                  #ifdef DEBUG_INPUT_DATA
                  printf("GOT  %s/%s\n[%f, %f, %f, %f]\n ", key->section, key->key, (float)v->v[0], (float)v->v[1], (float)v->v[2],  (float)v->v[3]);
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
  /* the vehicle is put back at rest, rotors stopped */
  memset(rotor_speed, 0, sizeof(rotor_speed));
  update_controls();
}

void run (void)
{
    char send_buf[1200];
    char rpm_buf[160];
    bool reconnecting = false;

    vehicle_pose_save();

    // calculate initial sensor values.
    if (wb_robot_step(timestep) == -1) {
      return;
    }

    while (true)
    {

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
            if (wb_robot_step(timestep) == -1) {
              break;
            }
          }
          reconnecting = true;
        }

        // trigger ArduPilot to send motor data
        rotor_speed_format_rpm(rpm_buf, sizeof(rpm_buf), rotor_speed,
                               MOTOR_SERVO_CHANNEL, MOTOR_NUM, RPM_CHANNELS);
        getAllSensors ((char *)send_buf, gyro,accelerometer,compass,gps, inertialUnit, rpm_buf);

        #ifdef DEBUG_SENSORS
        printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        #endif

        if (sitl_link_exchange(send_buf, parse_controls) != SITL_LINK_CONTROLS) {
          /* nothing yet: re-send this frame, or wait for a reconnect */
          continue;
        }

        update_controls();
        //https://cyberbotics.com/doc/reference/robot#wb_robot_step
        // this is used to force webots not to execute untill it receives feedback from simulator.
        if (wb_robot_step(timestep) == -1) {
          break;
        }
    }
    sitl_link_close();
}


bool initialize (int argc, char *argv[])
{
  #ifdef DEBUG_SENSORS
  fptr = fopen ("/tmp/log.txt","w");
  #endif
  port = 5599;  // default port
  double motor_velocity_cap = 0.0;   // 0 == use the world's maxVelocity

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
      { // specify drag factor used to simulate air resistance.
        if (argc > i+1 )
        {
          dragFactor = strtof (argv[i+1], NULL);
          printf("drag Factor %f\n",dragFactor);
        }
        else
        {
          fprintf(stderr,"Missing drag factor value.\n");
          return false;
        }

      }
      else if (strcmp (argv[i],"-mv")==0)
      { // cap the rotor angular velocity below the world's maxVelocity.
        if (argc > i+1 )
        {
          motor_velocity_cap = strtod (argv[i+1], NULL);
          printf("motor velocity cap %f rad/s\n", motor_velocity_cap);
        }
        else
        {
          fprintf(stderr,"Missing motor velocity cap value.\n");
          return false;
        }
      }
  }


  if (!sitl_link_open(port)) {
    return false;
  }

  /* necessary to initialize webots stuff */
  wb_robot_init();

  timestep = (int)wb_robot_get_basic_time_step();


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
    rotor_velocity[i] = 0.0;
    rotor_speed[i] = 0.0;

    motor_max_accel[i] = wb_motor_get_max_torque(motors[i]);
    motor_max_velocity[i] = wb_motor_get_max_velocity(motors[i]);
    if (motor_velocity_cap > 0.0 && motor_velocity_cap < motor_max_velocity[i]) {
      motor_max_velocity[i] = motor_velocity_cap;
    }
    printf("%s: max rotor velocity %.1f rad/s (%.0f rpm)\n",
           MOTOR_NAMES[i], motor_max_velocity[i],
           motor_max_velocity[i] * 60.0 / (2.0 * M_PI));

    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], 0.0);
  }

  return true;
}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* initialize() only fails before wb_robot_init(), so only clean up after it
     succeeded */
  if (initialize(argc, argv)) {
    run();
    wb_robot_cleanup();
  }

  return 0;
}
