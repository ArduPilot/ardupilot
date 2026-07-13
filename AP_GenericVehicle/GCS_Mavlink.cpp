#include "GCS_Mavlink.h"

#if HAL_GCS_ENABLED

#include "AP_GenericVehicle.h"
/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

#endif  // HAL_GCS_ENABLED
