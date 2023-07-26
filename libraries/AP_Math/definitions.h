#pragma once

#include <cmath>

#include <AP_HAL/AP_HAL_Boards.h>

#ifdef M_PI
# undef M_PI
#endif
#define M_PI      (3.141592653589793238462643383279502884)

#ifdef M_PI_2
# undef M_PI_2
#endif
#define M_PI_2    (M_PI / 2)

#define M_GOLDEN  1.6180339f

#define M_2PI         (M_PI * 2)

// MATH_CHECK_INDEXES modifies some objects (e.g. SoloGimbalEKF) to
// include more debug information.  It is also used by some functions
// to add extra code for debugging purposes. If you wish to activate
// this, do it here or as part of the top-level Makefile -
// e.g. Tools/Replay/Makefile
#ifndef MATH_CHECK_INDEXES
  #define MATH_CHECK_INDEXES 0
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// Centi-degrees to radians
#define DEGX100 5729.57795f

// GPS Specific double precision conversions
// The precision here does matter when using the wsg* functions for converting
// between LLH and ECEF coordinates.
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
static const double DEG_TO_RAD_DOUBLE = asin(1) / 90;
static const double RAD_TO_DEG_DOUBLE = 1 / DEG_TO_RAD_DOUBLE;
#endif

#define RadiansToCentiDegrees(x) (static_cast<float>(x) * RAD_TO_DEG * static_cast<float>(100))

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

// convert a longitude or latitude point to meters or centimeters.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M     0.011131884502145034
#define LATLON_TO_M_INV 89.83204953368922
#define LATLON_TO_CM    1.1131884502145034

// Semi-major axis of the Earth, in meters.
static const double WGS84_A = 6378137.0;

//Inverse flattening of the Earth
static const double WGS84_IF = 298.257223563;

// The flattening of the Earth
static const double WGS84_F = ((double)1.0 / WGS84_IF);

// Semi-minor axis of the Earth in meters
static const double WGS84_B = (WGS84_A * (1 - WGS84_F));

// Eccentricity of the Earth
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
static const double WGS84_E = (sqrt(2 * WGS84_F - WGS84_F * WGS84_F));
#endif

#define C_TO_KELVIN(temp) (temp + 273.15f)
#define KELVIN_TO_C(temp) (temp - 273.15f)
#define F_TO_KELVIN(temp) C_TO_KELVIN(((temp - 32) * 5/9))

#define M_PER_SEC_TO_KNOTS 1.94384449f
#define KNOTS_TO_M_PER_SEC (1/M_PER_SEC_TO_KNOTS)

#define KM_PER_HOUR_TO_M_PER_SEC 0.27777778f

// Gas Constant is from Aerodynamics for Engineering Students, Third Edition, E.L.Houghton and N.B.Carruthers
#define ISA_GAS_CONSTANT 287.26f
#define ISA_LAPSE_RATE 0.0065f

// Standard Sea Level values
// Ref: https://en.wikipedia.org/wiki/Standard_sea_level
#define SSL_AIR_DENSITY         1.225f // kg/m^3
#define SSL_AIR_PRESSURE 101325.01576f // Pascal
#define SSL_AIR_TEMPERATURE    288.15f // K

#define INCH_OF_H2O_TO_PASCAL 248.84f

/*
  use AP_ prefix to prevent conflict with OS headers, such as NuttX
  clock.h
 */
#define AP_NSEC_PER_SEC   1000000000ULL
#define AP_NSEC_PER_USEC  1000ULL
#define AP_USEC_PER_SEC   1000000ULL
#define AP_USEC_PER_MSEC  1000ULL
#define AP_MSEC_PER_SEC   1000ULL
#define AP_SEC_PER_HOUR   (3600ULL)
#define AP_MSEC_PER_HOUR  (AP_SEC_PER_HOUR * AP_MSEC_PER_SEC)
#define AP_SEC_PER_WEEK   (7ULL * 86400ULL)
#define AP_MSEC_PER_WEEK  (AP_SEC_PER_WEEK * AP_MSEC_PER_SEC)

// speed and distance conversions
#define KNOTS_TO_METERS_PER_SECOND 0.51444
#define FEET_TO_METERS 0.3048

// Convert amps milliseconds to milliamp hours
// Amp.millisec to milliAmp.hour = 1/1E3(ms->s) * 1/3600(s->hr) * 1000(A->mA)
#define AMS_TO_MAH 0.000277777778f

// Amps microseconds to milliamp hours
#define AUS_TO_MAH 0.0000002778f

// kg/m^3 to g/cm^3
#define KG_PER_M3_TO_G_PER_CM3(x) (0.001 * x)
