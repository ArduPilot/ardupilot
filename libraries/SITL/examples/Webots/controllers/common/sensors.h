/*
  sensor packing shared by the ArduPilot Webots controllers
*/

#ifndef ARDUPILOT_SITL_SENSORS_H
#define ARDUPILOT_SITL_SENSORS_H

#include <stdbool.h>
#include <stddef.h>
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/compass.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/camera.h>

void getInertia(const WbDeviceTag inertialUnit, char *buf, size_t buflen);
void getLinearVelocity(const WbDeviceTag gps, char *buf, size_t buflen);
void getCompass(const WbDeviceTag compass, char *buf, size_t buflen);
void getAcc(const WbDeviceTag accelerometer, char *buf, size_t buflen);
void getGyro(const WbDeviceTag gyro, char *buf, size_t buflen);
void getGPS(const WbDeviceTag gps, char *buf, size_t buflen);
/* extra_json is appended verbatim inside the top-level object; pass "" for none.
   The controllers use it to report rotor speeds as "\"rpm\": [...]".
   Returns false if the frame does not fit in buflen bytes. */
bool getAllSensors(char *buf, size_t buflen, WbDeviceTag gyro, WbDeviceTag accelerometer,
                   WbDeviceTag compass, const WbDeviceTag gps,
                   const WbDeviceTag inertial_unit, const char *extra_json);

#endif  /* ARDUPILOT_SITL_SENSORS_H */
