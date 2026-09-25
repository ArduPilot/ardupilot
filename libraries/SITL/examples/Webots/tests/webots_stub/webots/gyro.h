#ifndef WEBOTS_STUB_GYRO_H
#define WEBOTS_STUB_GYRO_H
#include "types.h"
void wb_gyro_enable(WbDeviceTag tag, int ms);
const double *wb_gyro_get_values(WbDeviceTag tag);
#endif
