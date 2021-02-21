

#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/compass.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/camera.h>




void getInertia (const WbDeviceTag inertialUnit, const double *northDirection, char *buf);
void getLinearVelocity (WbNodeRef nodeRef, const double *northDirection, char * buf);
void getCompass (const WbDeviceTag compass, const double *northDirection, char *buf);
void getAcc (const WbDeviceTag accelerometer, const double *northDirection, char *buf);
void getGyro (const WbDeviceTag gyro, const double *northDirection, char *buf);
void getGPS (const WbDeviceTag gps, const double *northDirection, char *buf);
void getAllSensors (char *buf, const double *northDirection, WbDeviceTag gyro, WbDeviceTag accelerometer, WbDeviceTag compass, const WbDeviceTag gps, WbDeviceTag inertial_unit);