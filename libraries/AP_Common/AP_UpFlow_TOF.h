#pragma once

#include <stdint.h> 

typedef struct{
    uint16_t   	ground_distance;        //reserved, always 999
    uint8_t    	tof_valid;              //0 for not valid, 0x64 (100) for 100% valid.
    bool        if_opt_ok;
}UPFLOW_TOF;

UPFLOW_TOF* get_upflow_tof();