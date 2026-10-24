#ifndef WEBOTS_STUB_ACCELEROMETER_H
#define WEBOTS_STUB_ACCELEROMETER_H
#include "types.h"
void wb_accelerometer_enable(WbDeviceTag tag, int ms);
const double *wb_accelerometer_get_values(WbDeviceTag tag);
#endif
