
#include "AP_IMU_Shim.h"

uint16_t  AP_IMU_Shim::_count; // number of samples captured
uint32_t  AP_IMU_Shim::_first_sample_time_micros; // time first sample began (equal to the last sample time of the previous iteration)
uint32_t  AP_IMU_Shim::_last_sample_time_micros;  // time that the latest sample was captured
