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

#ifndef _AP_COMMON_H
#define _AP_COMMON_H

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "wiring.h"
#endif
// ... and remove some of their stupid macros
#undef round
#undef abs

// prog_char_t is used as a wrapper type for prog_char, which is
// a character stored in flash. By using this wrapper type we can
// auto-detect at compile time if a call to a string function is using
// a flash-stored string or not
typedef struct {
    char c;
} prog_char_t;

#include <stdint.h>
#include "c++.h" // c++ additions
//#include "AP_Vector.h"
//#include "AP_Loop.h"

////////////////////////////////////////////////////////////////////////////////
/// @name	Warning control
//@{
//
// Turn on/off warnings of interest.
//
// These warnings are normally suppressed by the Arduino IDE,
// but with some minor hacks it's possible to have warnings
// emitted.  This helps greatly when diagnosing subtle issues.
//
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
#else
// This is a workaround for GCC bug c++/34734.
//
// The C++ compiler normally emits many spurious warnings for the use
// of PSTR (even though it generates correct code).  This workaround
// has an equivalent effect but avoids the warnings, which otherwise
// make finding real issues difficult.
//
 #ifdef DESKTOP_BUILD
  # undef PROGMEM
  # define PROGMEM __attribute__(())
 #else
  # undef PROGMEM
  # define PROGMEM __attribute__(( section(".progmem.data") ))
 #endif

 # undef PSTR
  /* Need const type for progmem - new for avr-gcc 4.6 */
  # if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 5 
 # define PSTR(s) (__extension__({static const prog_char __c[] PROGMEM = (s); \
                                  (const prog_char_t *)&__c[0]; }))
  #else
 # define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                                  (prog_char_t *)&__c[0]; }))
  #endif
#endif

// a varient of PSTR() for progmem strings passed to %S in printf()
// this gets the gcc __format__ checking right
#define FPSTR(s) (wchar_t *)(s)


static inline int        strcasecmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcasecmp_P(str1, (const prog_char *)pstr);
}

static inline int        strcmp_P(const char *str1, const prog_char_t *pstr)
{
    return strcmp_P(str1, (const prog_char *)pstr);
}

static inline size_t        strlen_P(const prog_char_t *pstr)
{
    return strlen_P((const prog_char *)pstr);
}

static inline void *      memcpy_P(void *dest, const prog_char_t *src, size_t n)
{
    return memcpy_P(dest, (const prog_char *)src, n);
}

// strlcat_P() in AVR libc seems to be broken
static inline size_t        strlcat_P(char *d, const prog_char_t *s, size_t bufsize)
{
    size_t          len1 = strlen(d);
    size_t          len2 = strlen_P(s);
    size_t          ret = len1 + len2;

    if (len1+len2 >= bufsize) {
        if (bufsize < (len1+1)) {
            return ret;
        }
        len2 = bufsize - (len1+1);
    }
    if (len2 > 0) {
        memcpy_P(d+len1, s, len2);
        d[len1+len2] = 0;
    }
    return ret;
}

static inline char *      strncpy_P(char *buffer, const prog_char_t *pstr, size_t buffer_size)
{
    return strncpy_P(buffer, (const prog_char *)pstr, buffer_size);
}


// read something the size of a pointer. This makes the menu code more
// portable
static inline uintptr_t        pgm_read_pointer(const void *s)
{
    if (sizeof(uintptr_t) == sizeof(uint16_t)) {
        return (uintptr_t)pgm_read_word(s);
    } else {
        union {
            uintptr_t p;
            uint8_t a[sizeof(uintptr_t)];
        } u;
        uint8_t        i;
        for (i=0; i< sizeof(uintptr_t); i++) {
            u.a[i] = pgm_read_byte(i + (const prog_char *)s);
        }
        return u.p;
    }
}

//@}


///
/// @name Macros
/// @{

/// Define a constant string in program memory.  This is a little more obvious
/// and less error-prone than typing the declaration out by hand.  It's required
/// when passing PROGMEM strings to static object constructors because the PSTR
/// hack can't be used at global scope.
///
#define PROGMEM_STRING(_v, _s)  static const char _v[] PROGMEM = _s

#define ToRad(x) (x*0.01745329252)      // *pi/180
#define ToDeg(x) (x*57.2957795131)      // *180/pi
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

/// XXX this should probably be replaced with radians()/degrees(), but their
/// inclusion in wiring.h makes doing that here difficult.
#define ToDeg(x) (x*57.2957795131)      // *180/pi
#define ToRad(x) (x*0.01745329252)      // *pi/180

//@}

#ifdef DESKTOP_BUILD
// used to report serious errors in autotest
 # define SITL_debug(fmt, args ...)  fprintf(stdout, "%s:%u " fmt, __FUNCTION__, __LINE__, ## args)
#else
 # define SITL_debug(fmt, args ...)
#endif

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
