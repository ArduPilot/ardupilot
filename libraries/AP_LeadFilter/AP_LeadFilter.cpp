/*
 *       AP_LeadFilter.cpp - GPS lag remover library for Arduino
 *       Code by Jason Short. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and / or
 *               modify it under the terms of the GNU Lesser General Public
 *               License as published by the Free Software Foundation; either
 *               version 2.1 of the License, or (at your option) any later version.
 *
 */
#include "AP_LeadFilter.h"

// setup the control preferences
int32_t
AP_LeadFilter::get_position(int32_t pos, int16_t vel, float lag_in_seconds)
{
    // assumes a 1 second delay in the GPS
    int16_t accel_contribution = (vel - _last_velocity) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    _last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}
