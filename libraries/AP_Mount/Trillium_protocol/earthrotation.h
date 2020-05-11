#ifndef EARTHROTATION_H
#define EARTHROTATION_H

#include "dcm.h"
#include "earthposition.h"

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

//! Convert an NED vector to ECEF
void nedToECEF(const double ned[NNED], double ecef[NECEF], const double lla[NLLA]);

//! Convert an NED vector to NED
void ecefToNED(const double ecef[NECEF], double ned[NNED], const double lla[NLLA]);

//! Convert an NED vector to ECEF using trig data
void nedToECEFtrig(const double ned[NNED], double ecef[NECEF], const llaTrig_t* trig);

//! Convert an ECEF vector to NED using trig data
void ecefToNEDtrig(const double ecef[NECEF], double ned[NNED], const llaTrig_t* trig);

//! Convert an NED vector to ECEF
void nedToECEFf(const float ned[NNED], float ecef[NECEF], const double lla[NLLA]);

//! Convert an ECEF vector to NED
void ecefToNEDf(const float ecef[NECEF], float ned[NNED], const double lla[NLLA]);

//! Convert an NED vector to ECEF using trig data
void nedToECEFtrigf(const float ned[NNED], float ecef[NECEF], const llaTrig_t* trig);

//! Convert an ECEF vector to NED using trig data
void ecefToNEDtrigf(const float ecef[NECEF], float ned[NNED], const llaTrig_t* trig);

//! Fill out a dcm that rotates from NED to ECEF
void nedToECEFdcm(DCM_t* dcm, const llaTrig_t* trig);

//! Fill out a dcm that rotates from ECEF to NED
void ecefToNEDdcm(DCM_t* dcm, const llaTrig_t* trig);

//! Compute the gravity vector in the ECEF frame given the strength of gravity.
void gravityToECEF(float gravityDown, float gravityEcef[NECEF], const llaTrig_t* trig);

//! Test earth rotations
BOOL testEarthRotation(void);

// C++ compilers: don't mangle us
#ifdef __cplusplus
}
#endif

#endif // EARTHROTATION_H
