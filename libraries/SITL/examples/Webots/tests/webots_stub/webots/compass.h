#ifndef WEBOTS_STUB_COMPASS_H
#define WEBOTS_STUB_COMPASS_H
#include "types.h"
void wb_compass_enable(WbDeviceTag tag, int ms);
const double *wb_compass_get_values(WbDeviceTag tag);
#endif
