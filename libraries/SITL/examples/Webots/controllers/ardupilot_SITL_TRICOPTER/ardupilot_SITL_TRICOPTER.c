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
  Wire protocol, line-delimited JSON in both directions.

  webots -> SITL, one line per simulation step:

  {"ts": 1561043647.759803,
   "vehicle.imu": {"av": [...], "la": [...], "mf": [...]},
   "vehicle.gps": {"x": ..., "y": ..., "z": ...},
   "vehicle.velocity": {"wlv": [...]},
   "vehicle.pose": {"x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ...},
   "rpm": [...]}

  SITL -> webots, from SIM_Webots.cpp output_tricopter():

  {"eng": [right, left, tail_servo, back], "wnd": [speed, north, east, down]}

  The three engine entries are already scaled 0..1 and the servo entry -0.5..0.5.
*/


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include "ardupilot_SITL_TRICOPTER.h"
#include "sitl_link.h"
#include "sensors.h"
#include "rotor_speed.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



#define MOTOR_NUM 3

/* tail servo deflection, radians, at full stick.  SIM_Webots.cpp sends the
   servo channel as -0.5..0.5, so this is the scale applied to that. */
#define TAIL_SERVO_SCALE_RAD 1.0

static WbDeviceTag motors[MOTOR_NUM];
static WbDeviceTag servo;
static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;

#ifdef WIND_SIMULATION
static WbDeviceTag emitter;
#endif

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

/*
  ArduPilot SERVOn feeding each Webots motor device, zero based: motor1 is the
  right rotor on SERVO1, motor2 the left on SERVO2 and motor3 the back one on
  SERVO4 (SIM_Webots.cpp output_tricopter()).  SERVO3 drives nothing.
*/
static const int MOTOR_SERVO_CHANNEL[MOTOR_NUM] = { 0, 1, 3 };

/* tail servo's own maxVelocity, rad/s, read from the world's RotationalMotor.
   Used to be a hard-coded 1000, which spammed a warning on every control step
   once the world's servo maxVelocity was tuned down below that. */
static double servo_max_velocity;

static double servo_value = 0;
#ifdef DEBUG_USE_KB
static double servo_value_extra = 0;
static double v_kb[MOTOR_NUM];
#endif

int port;
float dragFactor = VEHICLE_DRAG_FACTOR;

int timestep;


#ifdef DEBUG_USE_KB
/*
// Code used to simulate motors using keys to make sure that sensor directions and motor torques and thrusts are all correct.
// You can start this controller and use telnet instead of SITL to start the simulator.
// Then you can use the keyboard to emulate motor input.
*/
void process_keyboard (void)
{
  switch (wb_keyboard_get_key())
  {
    case 'Q':  // Q key -> up & left
      v_kb[0] = 0.0;
      v_kb[1] = 0.0;
      v_kb[2] = 0.0;
      servo_value_extra = 0.0;
      break;

    case 'Y':
      v_kb[0] = v_kb[0] + 0.01;
      v_kb[1] = v_kb[1] + 0.01;
      v_kb[2] = v_kb[2] - 0.02;
      break;

    case 'H':
      v_kb[0] = v_kb[0] - 0.01;
      v_kb[1] = v_kb[1] - 0.01;
      v_kb[2] = v_kb[2] + 0.02;
      break;

    case 'G':
      v_kb[0] = v_kb[0] + 0.01;
      v_kb[1] = v_kb[1] - 0.01;
      break;

    case 'J':
      v_kb[0] = v_kb[0] - 0.01;
      v_kb[1] = v_kb[1] + 0.01;
      break;

    case 'W':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v_kb[i] += 0.01;
      }
      break;

    case 'S':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v_kb[i] -= 0.01;
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
    if (v_kb[i] <= 0) v_kb[i] = 0;
    if (v_kb[i] >= 1) v_kb[i] = 1;

    rotor_velocity[i] = sqrt(v_kb[i]) * motor_max_velocity[i];
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], rotor_velocity[i]);
  }

  wb_motor_set_position (servo, servo_value_extra);
  wb_motor_set_velocity (servo, 100);


  printf ("Motors Internal right:%f left:%f back:%f servo:%f\n", v_kb[0],v_kb[1],v_kb[2],servo_value);

}
#endif


/*
// apply motor thrust.
*/
void update_controls(void)
{
  /*
      Webots' Propeller node computes

          Thrust = t1 * |omega| * omega - t2 * |omega| * V

      so thrust is quadratic in the rotor speed.  ArduPilot's mixer wants thrust
      linear in its 0..1 output once MOT_THST_EXPO is 0, so command
      omega = sqrt(u) * omega_max and the product is linear in u.
      See https://cyberbotics.com/doc/reference/propeller

      state.motors is the "eng" array: w = right, x = left, y = tail servo,
      z = back, already scaled by SIM_Webots.cpp.
   */
  const double u[MOTOR_NUM] = { state.motors.w, state.motors.x, state.motors.z };

  for (int i = 0; i < MOTOR_NUM; ++i)
  {
    double ui = u[i];
    if (ui < 0.0) {
      ui = 0.0;
    } else if (ui > 1.0) {
      ui = 1.0;
    }

    rotor_velocity[i] = sqrt(ui) * motor_max_velocity[i];

    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], rotor_velocity[i]);
  }
  /* update_controls() runs once per wb_robot_step(), so this advances the
     estimate to the end of the step the next sensor frame is sampled at */
  rotor_speed_update(rotor_speed, rotor_velocity, motor_max_accel, MOTOR_NUM,
                     timestep * 0.001);

  servo_value = -state.motors.y * TAIL_SERVO_SCALE_RAD;

#ifdef DEBUG_USE_KB
  wb_motor_set_position(servo, servo_value + servo_value_extra);
#else
  wb_motor_set_position(servo, servo_value);
#endif
  wb_motor_set_velocity(servo, servo_max_velocity);

  #ifdef DEBUG_MOTORS
  printf ("RAW    R:%f L:%f SRV:%f B:%f\n", state.motors.w, state.motors.x, state.motors.y, state.motors.z);
  printf ("Motors R:%f L:%f SRV:%f B:%f\n", rotor_velocity[0], rotor_velocity[1], servo_value, rotor_velocity[2]);
  #endif


#ifdef WIND_SIMULATION
  /*
    Drag: Fd = ½ ρ Cd A v², where v is the airspeed (wind velocity minus
    vehicle velocity) so that the force decays to zero once the vehicle is
    carried along with the air mass.

    The vehicle velocity used to arrive over a Receiver from a Supervisor and
    was read into a `linear_velocity` pointer that stayed pointing at a zero
    array, so drag was computed against a stationary vehicle.  The GPS device
    reports the same thing directly and needs no Supervisor.

    state.wind is ArduPilot's earth-frame wind vector, NED:
      .w = speed, .x = north, .y = east, .z = down.
    The worlds are WorldInfo.coordinateSystem "NUE", so North/Up/East.
  */
  const double *vehicle_vel = wb_gps_get_speed_vector(gps);   /* NUE, m/s */

  wind_webots_axis.x =  state.wind.x - vehicle_vel[0];   /* north */
  wind_webots_axis.y = -state.wind.z - vehicle_vel[1];   /* up, from NED down */
  wind_webots_axis.z =  state.wind.y - vehicle_vel[2];   /* east */

  /* fabsf, not abs: abs() is int abs(int) and truncates every apparent wind
     below 1 m/s to exactly zero. */
  wind_webots_axis.x = dragFactor * wind_webots_axis.x * fabsf(wind_webots_axis.x);
  wind_webots_axis.y = dragFactor * wind_webots_axis.y * fabsf(wind_webots_axis.y);
  wind_webots_axis.z = dragFactor * wind_webots_axis.z * fabsf(wind_webots_axis.z);

  wind_webots_axis.w = 0.0f;

  wb_emitter_send(emitter, &wind_webots_axis, sizeof(VECTOR4F));
#endif
}


// data example: [my_controller_SITL] {"eng": [0.000, 0.000, 0.000, 0.000]}
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
                  fprintf(stderr,"Failed to parse Vector4f for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else {
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

  // keyboard
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

  // tricopter servo name
  servo = wb_robot_get_device("servo_tail");
  servo_max_velocity = wb_motor_get_max_velocity(servo);

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
