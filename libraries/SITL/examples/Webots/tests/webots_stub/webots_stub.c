/* Stub implementations: enough to link the controllers for a compile check.
   Nothing here simulates anything. */
#include "webots/robot.h"
#include "webots/gyro.h"
#include "webots/accelerometer.h"
#include "webots/compass.h"
#include "webots/inertial_unit.h"
#include "webots/gps.h"
#include "webots/motor.h"
#include "webots/camera.h"
#include "webots/emitter.h"
#include "webots/receiver.h"
#include "webots/keyboard.h"
#include "webots/supervisor.h"
#include "webots/vehicle/car.h"
#include "webots/vehicle/driver.h"

static const double zero3[3] = {0.0, 0.0, 0.0};
static const double zero4[4] = {0.0, 1.0, 0.0, 0.0};

void wb_robot_init(void) {}
int wb_robot_step(int ms) { (void)ms; return -1; }
void wb_robot_cleanup(void) {}
bool wb_robot_get_supervisor(void) { return false; }
WbDeviceTag wb_robot_get_device(const char *name) { (void)name; return 1; }
double wb_robot_get_basic_time_step(void) { return 1.0; }
double wb_robot_get_time(void) { return 0.0; }
const char *wb_robot_get_custom_data(void) { return ""; }

void wb_gyro_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
const double *wb_gyro_get_values(WbDeviceTag t) { (void)t; return zero3; }
void wb_accelerometer_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
const double *wb_accelerometer_get_values(WbDeviceTag t) { (void)t; return zero3; }
void wb_compass_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
const double *wb_compass_get_values(WbDeviceTag t) { (void)t; return zero3; }
void wb_inertial_unit_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag t) { (void)t; return zero3; }
void wb_gps_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
const double *wb_gps_get_values(WbDeviceTag t) { (void)t; return zero3; }
double wb_gps_get_speed(WbDeviceTag t) { (void)t; return 0.0; }
const double *wb_gps_get_speed_vector(WbDeviceTag t) { (void)t; return zero3; }
void wb_motor_set_position(WbDeviceTag t, double p) { (void)t; (void)p; }
void wb_motor_set_velocity(WbDeviceTag t, double v) { (void)t; (void)v; }
double wb_motor_get_max_velocity(WbDeviceTag t) { (void)t; return 400.0; }
double wb_motor_get_max_torque(WbDeviceTag t) { (void)t; return 5000.0; }
void wb_camera_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
int wb_emitter_send(WbDeviceTag t, const void *d, int s) { (void)t; (void)d; return s; }
void wb_receiver_enable(WbDeviceTag t, int ms) { (void)t; (void)ms; }
void wb_receiver_set_channel(WbDeviceTag t, int c) { (void)t; (void)c; }
int wb_receiver_get_queue_length(WbDeviceTag t) { (void)t; return 0; }
const void *wb_receiver_get_data(WbDeviceTag t) { (void)t; return zero3; }
void wb_receiver_next_packet(WbDeviceTag t) { (void)t; }
void wb_keyboard_enable(int ms) { (void)ms; }
int wb_keyboard_get_key(void) { return -1; }
WbNodeRef wb_supervisor_node_get_root(void) { return 0; }
WbFieldRef wb_supervisor_node_get_field(WbNodeRef n, const char *f) { (void)n; (void)f; return 0; }
int wb_supervisor_node_get_type(WbNodeRef n) { (void)n; return 0; }
int wb_supervisor_field_get_count(WbFieldRef f) { (void)f; return 0; }
WbNodeRef wb_supervisor_field_get_mf_node(WbFieldRef f, int i) { (void)f; (void)i; return 0; }
const double *wb_supervisor_field_get_sf_vec3f(WbFieldRef f) { (void)f; return zero3; }
const double *wb_supervisor_field_get_sf_rotation(WbFieldRef f) { (void)f; return zero4; }
void wb_supervisor_field_set_sf_vec3f(WbFieldRef f, const double v[3]) { (void)f; (void)v; }
void wb_supervisor_field_set_sf_rotation(WbFieldRef f, const double v[4]) { (void)f; (void)v; }
WbNodeRef wb_supervisor_node_get_self(void) { return 0; }
void wb_supervisor_node_reset_physics(WbNodeRef n) { (void)n; }
double wbu_car_get_right_steering_angle(void) { return 0.0; }
void wbu_driver_init(void) {}
int wbu_driver_step(void) { return -1; }
void wbu_driver_cleanup(void) {}
void wbu_driver_set_cruising_speed(double s) { (void)s; }
void wbu_driver_set_steering_angle(double a) { (void)a; }
