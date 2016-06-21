// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
///
/// @file       AP_Common.h
/// @brief		Common definitions and utility routines for the ArduPilot
///				libraries.
///

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// used to pack structures
#define PACKED __attribute__((__packed__))

// used to mark a function that may be unused in some builds
#define UNUSED_FUNCTION __attribute__((unused))

// this can be used to optimize individual functions
#define OPTIMIZE(level) __attribute__((optimize(level)))

// sometimes we need to prevent inlining to prevent large stack usage
#define NOINLINE __attribute__((noinline))

#define FMT_PRINTF(a,b) __attribute__((format(printf, a, b)))
#define FMT_SCANF(a,b) __attribute__((format(scanf, a, b)))

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define LOCATION_ALT_MAX_M  83000   // maximum altitude (in meters) that can be fit into Location structure's alt field

/*
  check if bit bitnumber is set in value, returned as a
  bool. Bitnumber starts at 0 for the first bit
 */
#define BIT_IS_SET(value, bitnumber) (((value) & (1U<<(bitnumber))) != 0)

// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

template <typename T, size_t N>
char (&_ARRAY_SIZE_HELPER(T (&_arr)[N]))[N];

template <typename T>
char (&_ARRAY_SIZE_HELPER(T (&_arr)[0]))[0];

#define ARRAY_SIZE(_arr) sizeof(_ARRAY_SIZE_HELPER(_arr))

// @}


////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored               0: Absolute,	1: Relative
/// bit 1: Chnage Alt between WP            0: Gradually,	1: ASAP
/// bit 2: Direction of loiter command      0: Clockwise	1: Counter-Clockwise
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home					0: No,          1: Yes
/// bit 5: Loiter crosstrack reference      0: WP center    1: Tangent exit point
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

struct PACKED Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relateive to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
};

struct PACKED Location {
    union {
        Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

/*
  home states. Used to record if user has overridden home position.
*/
enum HomeState {
    HOME_UNSET,                 // home is unset, no GPS positions yet received
    HOME_SET_NOT_LOCKED,        // home is set to EKF origin or armed location (can be moved)
    HOME_SET_AND_LOCKED         // home has been set by user, cannot be moved except by user initiated do-set-home command
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
#define AP_PRODUCT_ID_SITL              0x03    // Software in the loop
#define AP_PRODUCT_ID_PX4               0x04    // PX4 on NuttX
#define AP_PRODUCT_ID_PX4_V2            0x05    // PX4 FMU2 on NuttX
#define AP_PRODUCT_ID_PX4_V4            0x06    // PX4 FMU4 on NuttX
#define AP_PRODUCT_ID_L3G4200D          0x101   // Linux with L3G4200D and ADXL345
#define AP_PRODUCT_ID_PIXHAWK_FIRE_CAPE 0x102   // Linux with the PixHawk Fire Cape
#define AP_PRODUCT_ID_MPU9250           0x103   // MPU9250
#define AP_PRODUCT_ID_VRBRAIN           0x150   // VRBRAIN on NuttX

/*
  Return true if value is between lower and upper bound inclusive.
  False otherwise.
*/
bool is_bounded_int32(int32_t value, int32_t lower_bound, int32_t upper_bound);

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include <AP_HAL_QURT/replace.h>
#endif

/*
  useful debugging macro for SITL
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SITL_printf(fmt, args ...) do { ::printf("%s(%u): " fmt, __FILE__, __LINE__, ##args); } while(0)
#else
#define SITL_printf(fmt, args ...)
#endif

