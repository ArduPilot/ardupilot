#include "AP_UpFlow_TOF.h"

static UPFLOW_TOF UPFLOW_TOF_DATA;

UPFLOW_TOF* get_upflow_tof()
{
    return &UPFLOW_TOF_DATA;
}