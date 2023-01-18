

#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/compass.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/camera.h>


WbNodeRef self_node;
double *linear_velocity;
double timestep_scale;
char pBug[1024];
void getInertia (const WbDeviceTag inertialUnit, char *buf);
void getLinearVelocity (WbNodeRef nodeRef, char * buf);
void getCompass (const WbDeviceTag compass, char *buf);
void getAcc (const WbDeviceTag accelerometer, char *buf);
void getGyro (const WbDeviceTag gyro, char *buf);
void getGPS (const WbDeviceTag gps, char *buf);
void getAllSensors (char *buf, WbDeviceTag gyro, WbDeviceTag accelerometer, WbDeviceTag compass, const WbDeviceTag gps, const WbDeviceTag inertial_unit);