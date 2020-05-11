#ifndef EARTHPOSITION_H
#define EARTHPOSITION_H

#include "Types.h"

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

enum
{
    ECEFX,
    ECEFY,
    ECEFZ,
    NECEF
};

enum
{
    LAT,
    LON,
    ALT,
    NLLA
};

enum
{
    NORTH,
    EAST,
    DOWN,
    NNED
};


/*! A structure to represent the trigonmetric values of the latitude and longitude */
typedef struct
{
    double sinLat;
    double sinLon;
    double cosLat;
    double cosLon;

}llaTrig_t;

//! Compute the trigonmetric quantities of latitude and longitude
const llaTrig_t* llaToTrig(const double lla[NLLA], llaTrig_t* trig);

//! Convert a LLA position to Earth centered Earth fixed.
void llaToECEF(const double lla[NLLA], double ecef[NECEF]);

//! Convert a LLA position to Earth centered Earth fixed, and return the lla trig values
void llaToECEFandTrig(const double lla[NLLA], double ecef[NECEF], llaTrig_t* trig);

//! Convert a LLA position represented by trigonometric values to ECEF
void llaTrigToECEF(double alt, double ecef[NECEF], const llaTrig_t* trig);

//! Convert the ECEF position to LLA
void ecefToLLA(const double ecef[NECEF], double lla[NLLA]);

//! Convert the ECEF position to LLA, with LLA trig
void ecefToLLAandTrig(const double ecef[NECEF], double lla[NLLA], llaTrig_t* trig);

//! Convert an array of geodetic coordinates to spherical geocentric coordinates.
void geodeticToGeocentric(const double geodetic[NLLA], double spherical[NLLA]);

//! Perform Earth position tests
BOOL testEarthPosition(void);

// C++ compilers: don't mangle us
#ifdef __cplusplus
}
#endif

#endif // EARTHPOSITION_H
