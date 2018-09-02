/*
  very simple example module
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <AP_Module_Structures.h>

void ap_hook_setup_start(uint64_t time_us)
{
    printf("setup_start called\n");
}

void ap_hook_setup_complete(uint64_t time_us)
{
    printf("setup_complete called\n");
}


#define degrees(x) (x * 180.0 / M_PI)

void ap_hook_AHRS_update(const struct AHRS_state *state)
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

void ap_hook_gyro_sample(const struct gyro_sample *state)
{
    static uint64_t last_print_us;
    if (state->time_us - last_print_us < 1000000UL) {
        return;
    }
    last_print_us = state->time_us;
    // print gyro rates once per second
    printf("gyro (%.1f,%.1f,%.1f)\n",
           degrees(state->gyro[0]),
           degrees(state->gyro[1]),
           degrees(state->gyro[2]));    
}

void ap_hook_accel_sample(const struct accel_sample *state)
{
    static uint64_t last_print_us;
    static uint32_t counter;
    static uint32_t fsync_count;
    counter++;
    if (state->fsync_set) {
        fsync_count++;
    }
    if (state->time_us - last_print_us < 1000000UL) {
        return;
    }
    last_print_us = state->time_us;
    // print accels once per second
    printf("accel (%.1f,%.1f,%.1f) %lu %lu\n",
           state->accel[0],
           state->accel[1],
           state->accel[2],
           (unsigned long)counter,
           (unsigned long)fsync_count);
}
