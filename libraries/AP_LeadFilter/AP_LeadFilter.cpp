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

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "AP_LeadFilter.h"

/*
 *  int32_t
 *  AP_LeadFilter::get_position(int32_t pos, int16_t vel)
 *  {
 *       vel = (_last_velocity + vel) / 2;
 *       pos += vel;
 *       pos += (vel - _last_velocity);
 *       _last_velocity = vel;
 *       return pos;
 *  }
 */

// setup the control preferences
int32_t
AP_LeadFilter::get_position(int32_t pos, int16_t vel)
{
    // assumes a 1 second delay in the GPS
    int16_t accel_contribution = (vel - _last_velocity) * _lag * _lag;
    int16_t vel_contribution = vel * _lag;

    // store velocity for next iteration
    _last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}
