#ifndef WEBOTS_STUB_ROBOT_H
#define WEBOTS_STUB_ROBOT_H
#include "types.h"
void wb_robot_init(void);
int wb_robot_step(int ms);
void wb_robot_cleanup(void);
bool wb_robot_get_supervisor(void);
WbDeviceTag wb_robot_get_device(const char *name);
double wb_robot_get_basic_time_step(void);
double wb_robot_get_time(void);
const char *wb_robot_get_custom_data(void);
#endif
