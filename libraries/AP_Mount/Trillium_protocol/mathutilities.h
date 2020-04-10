#ifndef MATHUTILITIES_H
#define MATHUTILITIES_H

#include "Constants.h"

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

//! 1D interpolation
#define interpolate(ref, ind0, ind1, dep0, dep1) (((ref)-(ind0))*((dep1)-(dep0))/((ind1)-(ind0)) + (dep0))

//! Add two angles together accounting for circular wrap
double addAngles(double first, double second);

//! Subtract one angle from another, account for circular wrap
double subtractAngles(double left, double right);

//! Adjust an angle for circular wrap
double wrapAngle(double angle);

//! Adjust an angle for circular wrap with the wrap point at -270/+90
double wrapAngle90(double angle);

//! Apply a first order low pass filter
double firstOrderFilter(double prev, double sig, double tau, double sampleTime);

//! Add two angles together accounting for circular wrap
float addAnglesf(float first, float second);

//! Subtract one angle from another, account for circular wrap
float subtractAnglesf(float left, float right);

//! Adjust an angle for circular wrap
float wrapAnglef(float angle);

//! Adjust an angle for circular wrap with the wrap point at -270/+90
float wrapAngle90f(float angle);

//! Adjust an angle for circular wrap with the wrap point at 0/360
float wrapAngle360f(float angle);

//! Fast sine approximation
float fastSin(float angle);

//! Fast cosine approximation
float fastCos(float angle);

//! Fast square root approximation
float fastISqrt(float x);

//! Fast square root approximation
float fastSqrt(float x);

//! Apply a first order low pass filter
float firstOrderFilterf(float prev, float sig, float tau, float sampleTime);

//! Apply a rate of change limit
float rateOfChangeLimitf(float prev, float value, float limit, float sampleTime);

//! Use GPS time information to compute the Gregorian calendar date and time with respect to UTC.
void computeDateAndTimeFromWeekAndItow(uint16_t week, uint32_t itow, uint8_t leapsecs, uint16_t* pyear, uint8_t* pmonth, uint8_t* pday, uint8_t* phour, uint8_t* pmin, uint8_t* psec);

//! Use GPS time information to compute the Gregorian calendar date.
void computeDateFromWeekAndItow(uint16_t week, uint32_t itow, uint16_t* pyear, uint8_t* pmonth, uint8_t* pday);

//! Compute hours, minutes, and seconds from GPS time of week
void computeTimeFromItow(uint32_t itow, uint8_t* hour, uint8_t* min, uint8_t* second);

//! Use Gregorian date information to compute GPS style time information.
void computeWeekAndItow(uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds, int16_t milliseconds, uint16_t* pweek, uint32_t* pitow);

//! Test the date conversion logic
int testDateConversion(void);

// C++ compilers: don't mangle us
#ifdef __cplusplus
}
#endif

#endif // MATHUTILITIES_H
