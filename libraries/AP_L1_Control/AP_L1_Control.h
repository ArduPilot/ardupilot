// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_L1_Control.h
/// @brief   L1 Control algorithm utilities

/*
 *  L1_Control algorithm utilities
 *  Functions to generate a L1 reference point.
 *  Brandon Jones 2013
 *
 *  calc_L1_circ and calc_L1_line based on:
 *  Ducard, G.; Kulling, K.C.; Geering, H.P.; , "A simple and adaptive on-line path planning system
 *  for a UAV," Control & Automation, 2007. MED '07. Mediterranean Conference on , vol., no., pp.1-6,
 *  27-29 June 2007 */

#ifndef AP_L1_CONTROL_H
#define AP_L1_CONTROL_H

#include <AP_Math.h>


//Convert a 2D vector from latitude and longitude to planar coordinates based on a reference point
Vector2f        geo2planar(const Vector2f &ref, const Vector2f &wp);

//Convert a 2D vector from planar coordinates to latitude and longitude based on a reference point
Vector2f        planar2geo(const Vector2f &ref, const Vector2f &wp);


/*Calculate a reference point for L1 control based on a circle.
 *  L1:  Reference length, smaller is equivalent to higher gain [meters]
 *  turn_radius: Radius of the circle [meters]
 *  turn_center_loc: Center of the circle
 *  current_loc: Current location of vehicle
 *  L1_ref: Generated Reference Point  */
void        calc_L1_circ(  uint8_t                  L1,
                           const uint8_t            turn_radius,
                           const struct Location &  turn_center_loc,
                           const struct Location &  current_loc,
                           struct Location &        L1_ref);

/*Calculate a reference point for L1 control based on a line.
 *  L1: Reference length, smaller is equivalent to higher gain [meters]
 *  A: First point in line
 *  B: Second point in line
 *  current_loc: Current location of vehicle
 *  L1_ref: Generated Reference Point */
void        calc_L1_line(  uint8_t                  L1,
                           const struct Location &  A,
                           const struct Location &  B,
                           const struct Location &  current_loc,
                           struct Location &        L1_ref);


#endif //AP_L1_CONTROL_H