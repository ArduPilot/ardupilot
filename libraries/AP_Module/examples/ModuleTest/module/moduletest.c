/*
  very simple example module
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "../../../AP_Module_Structures.h"

void hook_setup_start(uint64_t time_us)
{
    printf("setup_start called\n");
}

void hook_setup_complete(uint64_t time_us)
{
    printf("setup_complete called\n");
}


#define degrees(x) (x * 180.0 / M_PI)

void hook_AHRS_update(const struct AHRS_state *state)
{
    static uint64_t last_print_us;
    if (state->time_us - last_print_us < 1000000UL) {
        return;
    }
    last_print_us = state->time_us;
    // print euler angles once per second
    printf("AHRS_update (%.1f,%.1f,%.1f)\n",
           degrees(state->eulers[0]),
           degrees(state->eulers[1]),
           degrees(state->eulers[2]));
}
