// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

///
/// @file       AP_Common.h
/// @brief		Common definitions and utility routines for the ArduPilot
///				libraries.
///

#ifndef __AP_COMMON_H__
#define __AP_COMMON_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wlogical-op"
#pragma GCC diagnostic ignored "-Wredundant-decls"

// Make some dire warnings into errors
//
// Some warnings indicate questionable code; rather than let
// these slide, we force them to become errors so that the
// developer has to find a safer alternative.
//
//#pragma GCC diagnostic error "-Wfloat-equal"

// The following is strictly for type-checking arguments to printf_P calls
// in conjunction with a suitably modified Arduino IDE; never define for
// production as it generates bad code.
//
#if PRINTF_FORMAT_WARNING_DEBUG
 # undef PSTR
 # define PSTR(_x)               _x             // help the compiler with printf_P
 # define float double                  // silence spurious format warnings for %f
#endif

#define FPSTR(s) (wchar_t *)(s)

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi
// @}


////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored               0: Absolute,	1: Relative
/// bit 1: Chnage Alt between WP            0: Gradually,	1: ASAP
/// bit 2:
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home					0: No,          1: Yes
/// bit 5:
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

struct Location {
    uint8_t id;                                                 ///< command id
    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
    uint8_t p1;                                                 ///< param 1
    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

//@}

////////////////////////////////////////////////////////////////////////////////
/// @name	Conversions
///
/// Conversion macros and factors.
///
//@{


/*  Product IDs for all supported products follow */

#define AP_PRODUCT_ID_NONE                      0x00    // Hardware in the loop
#define AP_PRODUCT_ID_APM1_1280         0x01    // APM1 with 1280 CPUs
#define AP_PRODUCT_ID_APM1_2560         0x02    // APM1 with 2560 CPUs
#define AP_PRODUCT_ID_SITL                      0x03    // Software in the loop
#define AP_PRODUCT_ID_APM2ES_REV_C4 0x14        // APM2 with MPU6000ES_REV_C4
#define AP_PRODUCT_ID_APM2ES_REV_C5     0x15    // APM2 with MPU6000ES_REV_C5
#define AP_PRODUCT_ID_APM2ES_REV_D6     0x16    // APM2 with MPU6000ES_REV_D6
#define AP_PRODUCT_ID_APM2ES_REV_D7     0x17    // APM2 with MPU6000ES_REV_D7
#define AP_PRODUCT_ID_APM2ES_REV_D8     0x18    // APM2 with MPU6000ES_REV_D8
#define AP_PRODUCT_ID_APM2_REV_C4       0x54    // APM2 with MPU6000_REV_C4
#define AP_PRODUCT_ID_APM2_REV_C5       0x55    // APM2 with MPU6000_REV_C5
#define AP_PRODUCT_ID_APM2_REV_D6       0x56    // APM2 with MPU6000_REV_D6
#define AP_PRODUCT_ID_APM2_REV_D7       0x57    // APM2 with MPU6000_REV_D7
#define AP_PRODUCT_ID_APM2_REV_D8       0x58    // APM2 with MPU6000_REV_D8
#define AP_PRODUCT_ID_APM2_REV_D9       0x59    // APM2 with MPU6000_REV_D9

#endif // _AP_COMMON_H
