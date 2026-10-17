#ifndef WEBOTS_STUB_MOTOR_H
#define WEBOTS_STUB_MOTOR_H
#include "types.h"
void wb_motor_set_position(WbDeviceTag tag, double pos);
void wb_motor_set_velocity(WbDeviceTag tag, double vel);
double wb_motor_get_max_velocity(WbDeviceTag tag);
double wb_motor_get_max_torque(WbDeviceTag tag);
#endif
