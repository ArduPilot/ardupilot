#pragma once

#include <stdint.h>

/*
  structure passed in giving servo positions as PWM values in
  microseconds
*/
struct sitl_input {
    uint16_t servos[32];
    struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
    } wind;
};

