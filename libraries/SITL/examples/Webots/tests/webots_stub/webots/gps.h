#ifndef WEBOTS_STUB_GPS_H
#define WEBOTS_STUB_GPS_H
#include "types.h"
void wb_gps_enable(WbDeviceTag tag, int ms);
const double *wb_gps_get_values(WbDeviceTag tag);
double wb_gps_get_speed(WbDeviceTag tag);
const double *wb_gps_get_speed_vector(WbDeviceTag tag);
#endif
