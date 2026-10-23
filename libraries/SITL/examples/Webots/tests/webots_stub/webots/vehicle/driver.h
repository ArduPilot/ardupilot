#ifndef WEBOTS_STUB_DRIVER_H
#define WEBOTS_STUB_DRIVER_H
#include "../types.h"
void wbu_driver_init(void);
int wbu_driver_step(void);
void wbu_driver_cleanup(void);
void wbu_driver_set_cruising_speed(double speed);
void wbu_driver_set_steering_angle(double angle);
#endif
