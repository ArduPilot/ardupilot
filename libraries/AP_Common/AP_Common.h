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

#ifndef __AP_COMMON_H__
#define __AP_COMMON_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wlogical-op"
#pragma GCC diagnostic ignored "-Wredundant-decls"

// used to pack structures
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define PACKED
#else
#define PACKED __attribute__((__packed__))
#endif

// this can be used to optimize individual functions
#define OPTIMIZE(level) __attribute__((optimize(level)))

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

/*
  check if bit bitnumber is set in value, returned as a
  bool. Bitnumber starts at 0 for the first bit
 */
#define BIT_IS_SET(value, bitnumber) (((value) & (1U<<(bitnumber))) != 0)

// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

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
/// bit 5:
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

struct PACKED Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relateive to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
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
#define AP_PRODUCT_ID_SITL              0x03    // Software in the loop
#define AP_PRODUCT_ID_PX4               0x04    // PX4 on NuttX
#define AP_PRODUCT_ID_PX4_V2            0x05    // PX4 FMU2 on NuttX
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
#define AP_PRODUCT_ID_FLYMAPLE          0x100   // Flymaple with ITG3205, ADXL345, HMC5883, BMP085
#define AP_PRODUCT_ID_L3G4200D          0x101   // Linux with L3G4200D and ADXL345
#define AP_PRODUCT_ID_PIXHAWK_FIRE_CAPE 0x102   // Linux with the PixHawk Fire Cape
#define AP_PRODUCT_ID_MPU9250           0x103   // MPU9250
#define AP_PRODUCT_ID_VRBRAIN           0x150   // VRBRAIN on NuttX

// map from kbaud rate to baudrate
uint32_t map_baudrate(int16_t rate);

#endif // _AP_COMMON_H
