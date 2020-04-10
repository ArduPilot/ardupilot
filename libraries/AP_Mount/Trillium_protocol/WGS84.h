#ifndef WGS84_H
#define WGS84_H

/*!
 * \file World Geographic System 1984 (WGS84) datum.
 *
 * WGS-84 is an ellipsoid that is defined by the sem-major axis (equatorial
 * radius) and the flattening. All other components are derived from these two.
 */

#include <math.h>

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

//! Semi-major axis (radius of the equator) in meters
#define datum_semiMajorAxis 6378137.0

//! flattening
#define datum_flattening (1.0 / 298.257223563)

//! Semi-minor axis (polar radius) in meters
#define datum_semiMinorAxis (datum_semiMajorAxis * (1.0 - datum_flattening))

//! inverse flattening
#define datum_inverseFlattenning (1.0 / datum_flattening)

//! First eccentricity squared
#define datum_eSquared (1.0 - (datum_semiMinorAxis * datum_semiMinorAxis) / (datum_semiMajorAxis * datum_semiMajorAxis))

//! First eccentricity
#define datum_e sqrt(datum_eSquared)

//! Second eccentricity squared
#define datum_eSecondSquared ((datum_semiMajorAxis * datum_semiMajorAxis) / (datum_semiMinorAxis * datum_semiMinorAxis) - 1.0)

//! Arithmetic mean radius in meters
#define datum_meanRadius ((2.0 * datum_semiMajorAxis + datum_semiMinorAxis) / 3.0)

//! Earth rate of rotation in radians per second, based on mean sidereal day
#define datum_earthAngularRate 7.292115E-5

//! Gravity at the equator
#define datum_equatorialGravity 9.7803267714

//! Return the gravity at the surface of the ellipsoid
double gravity(double latitude);

//! Return the gravity at the surface of the ellipsoid from the sin() if the latitude
double gravityFromSinLat(double sinlat);

//! Compute the East-West radius of curvature at a specific latitude
double radiusOfEWCurv(double latitude);

//! Compute the East-West radius of curvature from the sin() of the latitude
double radiusOfEWCurvFromSinLat(double sinLat);

// C++ compilers: don't mangle us
#ifdef __cplusplus
}
#endif

#endif // WGS84_H
