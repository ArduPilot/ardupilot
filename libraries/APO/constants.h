/*
 * constants.h
 *
 *  Created on: Apr 7, 2011
 *      Author: nkgentry
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "math.h"

const float scale_deg = 1e7; // scale of integer degrees/ degree
const float scale_m = 1e3; // scale of integer meters/ meter
const float rEarth = 6371000; // radius of earth, meters
const float rad2Deg = 180 / M_PI; // radians to degrees
const float deg2Rad = M_PI / 180; // degrees to radians
const float rad2DegInt = rad2Deg * scale_deg; // radians to degrees x 1e7
const float degInt2Rad = deg2Rad / scale_deg; // degrees x 1e7 to radians

#define MAV_MODE_FAILSAFE MAV_MODE_TEST3

#endif /* CONSTANTS_H_ */
// vim:ts=4:sw=4:expandtab
