#include "AP_UpFlow_TOF.h"

static UPFLOW_TOF UPFLOW_TOF_DATA;

/* UPFLOW_TOF* get_upflow_tof_ptr()
{
    return &UPFLOW_TOF_DATA;
} */

UPFLOW_TOF get_upflow_tof()
{
    return UPFLOW_TOF_DATA;
}

void set_upflow_tof( UPFLOW_TOF data )
{
    UPFLOW_TOF_DATA = data;
}
