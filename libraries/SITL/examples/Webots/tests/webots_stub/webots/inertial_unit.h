#ifndef WEBOTS_STUB_IU_H
#define WEBOTS_STUB_IU_H
#include "types.h"
void wb_inertial_unit_enable(WbDeviceTag tag, int ms);
const double *wb_inertial_unit_get_roll_pitch_yaw(WbDeviceTag tag);
#endif
