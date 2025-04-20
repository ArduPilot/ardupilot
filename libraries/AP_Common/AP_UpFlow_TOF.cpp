#include "AP_UpFlow_TOF.h"

static UPFLOW_TOF UPFLOW_TOF_DATA;
static UPFLOW_TOF* upflow_tof_data = &UPFLOW_TOF_DATA;

UPFLOW_TOF* get_upflow_tof()
{
    return upflow_tof_data;
}