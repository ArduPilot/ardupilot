#include "earthrotation.h"
#include "Constants.h"
#include <math.h>


/*!
 * Convert a vector in North, East, Down to Earth Centered Earth Fixed.
 * \param ned is the NED vector to convert
 * \param ecef receives the equivalent ECEF vector
 * \param lla is the position in latitude and longitude in radians.
 */
void nedToECEF(const double ned[NNED], double ecef[NECEF], const double lla[NLLA])
{
    llaTrig_t trig;
    nedToECEFtrig(ned, ecef, llaToTrig(lla, &trig));
}


/*!
 * Convert a vector in Earth Centered Earth Fixed to North, East, Down.
 * \param ecef is the ECEF vector to convert
 * \param ned receives the equivalent NED vector
 * \param lla is the position in latitude and longitude in radians.
 */
void ecefToNED(const double ecef[NECEF], double ned[NNED], const double lla[NLLA])
{
    llaTrig_t trig;
    ecefToNEDtrig(ecef, ned, llaToTrig(lla, &trig));
}


/*!
 * Convert a vector in North, East, Down to Earth Centered Earth Fixed. The
 * conversion is done using precomputed trig values for latitude and longitude.
 * \param ned is the NED vector to convert
 * \param ecef receives the equivalent ECEF vector. ecef can be the same memory
 *        as ned for rotation in place.
 * \param trig are the precomputed trig values that depend on latitude and longitude
 */
void nedToECEFtrig(const double ned[NNED], double ecef[NECEF], const llaTrig_t* trig)
{
    // Doing it this way allows ned and ecef to point to the same array
    double x = -ned[NORTH]*trig->sinLat*trig->cosLon - ned[EAST]*trig->sinLon - ned[DOWN]*trig->cosLat*trig->cosLon;
    double y = -ned[NORTH]*trig->sinLat*trig->sinLon + ned[EAST]*trig->cosLon - ned[DOWN]*trig->cosLat*trig->sinLon;
    double z =  ned[NORTH]*trig->cosLat                                       - ned[DOWN]*trig->sinLat;

    ecef[ECEFX] = x;
    ecef[ECEFY] = y;
    ecef[ECEFZ] = z;

}// nedToECEFtrig


/*!
 * Convert a vector in Earth Centered Earth Fixed to North, East, Down. The
 * conversion is done using precomputed trig values for latitude and longitude.
 * \param ecef is the ECEF vector to convert
 * \param ned receives the equivalent NED vector. ned can be the same memory
 *        as ecef for rotation in place.
 * \param trig are the precomputed trig values that depend on latitude and longitude
 */
void ecefToNEDtrig(const double ecef[NECEF], double ned[NNED], const llaTrig_t* trig)
{
    // Doing it this way allows ned and ecef to point to the same array
    double x = -ecef[ECEFX]*trig->sinLat*trig->cosLon - ecef[ECEFY]*trig->sinLat*trig->sinLon + ecef[ECEFZ]*trig->cosLat;
    double y = -ecef[ECEFX]*trig->sinLon              + ecef[ECEFY]*trig->cosLon;
    double z = -ecef[ECEFX]*trig->cosLat*trig->cosLon - ecef[ECEFY]*trig->cosLat*trig->sinLon - ecef[ECEFZ]*trig->sinLat;

    ned[NORTH] = x;
    ned[EAST]  = y;
    ned[DOWN]  = z;

}// ecefToNEDtrig


/*!
 * Convert a vector in North, East, Down to Earth Centered Earth Fixed.
 * \param ned is the NED vector to convert
 * \param ecef receives the equivalent ECEF vector
 * \param lla is the position in latitude and longitude in radians.
 */
void nedToECEFf(const float ned[NNED], float ecef[NECEF], const double lla[NLLA])
{
    llaTrig_t trig;
    nedToECEFtrigf(ned, ecef, llaToTrig(lla, &trig));
}


/*!
 * Convert a vector in Earth Centered Earth Fixed to North, East, Down.
 * \param ecef is the ECEF vector to convert
 * \param ned receives the equivalent NED vector
 * \param lla is the position in latitude and longitude in radians.
 */
void ecefToNEDf(const float ecef[NECEF], float ned[NNED], const double lla[NLLA])
{
    llaTrig_t trig;
    ecefToNEDtrigf(ecef, ned, llaToTrig(lla, &trig));
}


/*!
 * Convert a vector in North, East, Down to Earth Centered Earth Fixed. The
 * conversion is done using precomputed trig values for latitude and longitude.
 * \param ned is the NED vector to convert
 * \param ecef receives the equivalent ECEF vector. ecef can be the same memory
 *        as ned for rotation in place.
 * \param trig are the precomputed trig values that depend on latitude and longitude
 */
void nedToECEFtrigf(const float ned[NNED], float ecef[NECEF], const llaTrig_t* trig)
{
    // Doing it this way allows ned and ecef to point to the same array
    double x = (float)(-ned[NORTH]*trig->sinLat*trig->cosLon - ned[EAST]*trig->sinLon - ned[DOWN]*trig->cosLat*trig->cosLon);
    double y = (float)(-ned[NORTH]*trig->sinLat*trig->sinLon + ned[EAST]*trig->cosLon - ned[DOWN]*trig->cosLat*trig->sinLon);
    double z = (float)(ned[NORTH]*trig->cosLat                                        - ned[DOWN]*trig->sinLat);

    ecef[ECEFX] = x;
    ecef[ECEFY] = y;
    ecef[ECEFZ] = z;

}// nedToECEFtrigf


/*!
 * Convert a vector in Earth Centered Earth Fixed to North, East, Down. The
 * conversion is done using precomputed trig values for latitude and longitude.
 * \param ecef is the ECEF vector to convert
 * \param ned receives the equivalent NED vector. ned can be the same memory
 *        as ecef for rotation in place.
 * \param trig are the precomputed trig values that depend on latitude and longitude
 */
void ecefToNEDtrigf(const float ecef[NECEF], float ned[NNED], const llaTrig_t* trig)
{
    float x = (float)(-ecef[ECEFX]*trig->sinLat*trig->cosLon - ecef[ECEFY]*trig->sinLat*trig->sinLon + ecef[ECEFZ]*trig->cosLat);
    float y = (float)(-ecef[ECEFX]*trig->sinLon              + ecef[ECEFY]*trig->cosLon);
    float z = (float)(-ecef[ECEFX]*trig->cosLat*trig->cosLon - ecef[ECEFY]*trig->cosLat*trig->sinLon - ecef[ECEFZ]*trig->sinLat);

    ned[NORTH] = x;
    ned[EAST]  = y;
    ned[DOWN]  = z;

}// ecefToNEDtrigf


/*!
 * Fill out a dcm that rotates from NED to ECEF
 * \param dcm is filled out with the rotation
 * \param trig are the LLA trig values that go into the rotation
 */
void nedToECEFdcm(DCM_t* dcm, const llaTrig_t* trig)
{
    float* dcmdata = dcm->data;

    dcmdata[0] = (float)(-trig->sinLat*trig->cosLon); dcmdata[1] = (float)(-trig->sinLon); dcmdata[2] = (float)(-trig->cosLat*trig->cosLon);
    dcmdata[3] = (float)(-trig->sinLat*trig->sinLon); dcmdata[4] = (float)( trig->cosLon); dcmdata[5] = (float)(-trig->cosLat*trig->sinLon);
    dcmdata[6] = (float)( trig->cosLat);              dcmdata[7] =  0.0f;                  dcmdata[8] = (float)(-trig->sinLat);

}// nedToECEFdcm


/*!
 * Fill out a dcm that rotates from ECEF to NED
 * \param dcm is filled out with the rotation
 * \param trig are the LLA trig values that go into the rotation
 */
void ecefToNEDdcm(DCM_t* dcm, const llaTrig_t* trig)
{
    float* dcmdata = dcm->data;

    // Notice the indices are transposed compared to above
    dcmdata[0] = (float)(-trig->sinLat*trig->cosLon); dcmdata[3] = (float)(-trig->sinLon); dcmdata[6] = (float)(-trig->cosLat*trig->cosLon);
    dcmdata[1] = (float)(-trig->sinLat*trig->sinLon); dcmdata[4] = (float)( trig->cosLon); dcmdata[7] = (float)(-trig->cosLat*trig->sinLon);
    dcmdata[2] = (float)( trig->cosLat);              dcmdata[5] =  0.0f;                  dcmdata[8] = (float)(-trig->sinLat);

}// ecefToNEDdcm


/*!
 * Compute the gravity vector in the ECEF frame given the strength of gravity
 * in the NED frame. Gravity in NED has one component, DOWN, which is positive.
 * \param gravityDown is the strenght of gravity in m/s/s.
 * \param gravityEcef receives the gravity vector in ECEF.
 * \param trig are the LLA trig values that go into the rotation.
 */
void gravityToECEF(float gravityDown, float gravityEcef[NECEF], const llaTrig_t* trig)
{
    // This is the NED to ECEF rotation assuming NORTH and EAST are zero.
    gravityEcef[ECEFX] = (float)(-gravityDown*trig->cosLat*trig->cosLon);
    gravityEcef[ECEFY] = (float)(-gravityDown*trig->cosLat*trig->sinLon);
    gravityEcef[ECEFZ] = (float)(-gravityDown*trig->sinLat);

}// gravityToECEF


/*!
 * Perform Earth rotation tests, converting between NED and ECEF representations
 * \return TRUE if the tests pass
 */
BOOL testEarthRotation(void)
{
    float error = 0.0;
    double posLLA[NLLA] = {0, 0, 0};
    float velNED[NNED] = {1.0f, 2.0f, 3.0f};
    float velECEF[NECEF];
    llaTrig_t trig;
    stackAllocateDCM(dcm);

    llaToTrig(posLLA, &trig);
    nedToECEFtrigf(velNED, velECEF, &trig);
    error += fabsf(-3.0f - velECEF[ECEFX]);
    error += fabsf(2.0f - velECEF[ECEFY]);
    error += fabsf(1.0f - velECEF[ECEFZ]);

    nedToECEFdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velNED, velECEF);
    error += fabsf(-3.0f - velECEF[ECEFX]);
    error += fabsf(2.0f - velECEF[ECEFY]);
    error += fabsf(1.0f - velECEF[ECEFZ]);

    ecefToNEDtrigf(velECEF, velNED, &trig);
    error += fabsf(1.0f - velNED[NORTH]);
    error += fabsf(2.0f - velNED[EAST]);
    error += fabsf(3.0f - velNED[DOWN]);

    ecefToNEDdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velECEF, velNED);
    error += fabsf(1.0f - velNED[NORTH]);
    error += fabsf(2.0f - velNED[EAST]);
    error += fabsf(3.0f - velNED[DOWN]);

    // New position and velocity
    posLLA[LON] = PId;
    velNED[NORTH] = -1.0f;
    velNED[EAST] = -2.0f;
    velNED[DOWN] = -3.0f;
    llaToTrig(posLLA, &trig);
    nedToECEFtrigf(velNED, velECEF, &trig);
    error += fabsf(-3.0f - velECEF[ECEFX]);
    error += fabsf(2.0f - velECEF[ECEFY]);
    error += fabsf(-1.0f - velECEF[ECEFZ]);

    nedToECEFdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velNED, velECEF);
    error += fabsf(-3.0f - velECEF[ECEFX]);
    error += fabsf(2.0f - velECEF[ECEFY]);
    error += fabsf(-1.0f - velECEF[ECEFZ]);

    ecefToNEDtrigf(velECEF, velNED, &trig);
    error += fabsf(-1.0f - velNED[NORTH]);
    error += fabsf(-2.0f - velNED[EAST]);
    error += fabsf(-3.0f - velNED[DOWN]);

    ecefToNEDdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velECEF, velNED);
    error += fabsf(-1.0f - velNED[NORTH]);
    error += fabsf(-2.0f - velNED[EAST]);
    error += fabsf(-3.0f - velNED[DOWN]);

    // New position and velocity
    posLLA[LAT] = PId/4.0f;
    posLLA[LON] = -3.0*PId/4.0;
    velNED[NORTH] = 1.0;
    velNED[EAST] = 0.0;
    velNED[DOWN] = 0.0;

    llaToTrig(posLLA, &trig);
    nedToECEFtrigf(velNED, velECEF, &trig);
    error += fabsf(0.5f - velECEF[ECEFX]);
    error += fabsf(0.5f - velECEF[ECEFY]);
    error += fabsf(0.70710678118655f - velECEF[ECEFZ]);

    nedToECEFdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velNED, velECEF);
    error += fabsf(0.5f - velECEF[ECEFX]);
    error += fabsf(0.5f - velECEF[ECEFY]);
    error += fabsf(0.70710678118655f - velECEF[ECEFZ]);

    ecefToNEDtrigf(velECEF, velNED, &trig);
    error += fabsf(1.0f - velNED[NORTH]);
    error += fabsf(0.0f - velNED[EAST]);
    error += fabsf(0.0f - velNED[DOWN]);

    ecefToNEDdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velECEF, velNED);
    error += fabsf(1.0f - velNED[NORTH]);
    error += fabsf(0.0f - velNED[EAST]);
    error += fabsf(0.0f - velNED[DOWN]);

    // New velocity
    velNED[NORTH] = 0.0f;
    velNED[EAST] = 1.0f;
    velNED[DOWN] = 0.0f;
    nedToECEFtrigf(velNED, velECEF, &trig);
    error += fabsf(0.70710678118655f - velECEF[ECEFX]);
    error += fabsf(-.70710678118655f - velECEF[ECEFY]);
    error += fabsf(0.0f - velECEF[ECEFZ]);

    nedToECEFdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velNED, velECEF);
    error += fabsf(0.70710678118655f - velECEF[ECEFX]);
    error += fabsf(-.70710678118655f - velECEF[ECEFY]);
    error += fabsf(0.0f - velECEF[ECEFZ]);

    ecefToNEDtrigf(velECEF, velNED, &trig);
    error += fabsf(0.0f - velNED[NORTH]);
    error += fabsf(1.0f - velNED[EAST]);
    error += fabsf(0.0f - velNED[DOWN]);

    ecefToNEDdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velECEF, velNED);
    error += fabsf(0.0f - velNED[NORTH]);
    error += fabsf(1.0f - velNED[EAST]);
    error += fabsf(0.0f - velNED[DOWN]);

    // New velocity
    velNED[NORTH] = 0.0f;
    velNED[EAST] = 0.0f;
    velNED[DOWN] = 1.0f;
    nedToECEFtrigf(velNED, velECEF, &trig);
    error += fabsf(0.5f - velECEF[ECEFX]);
    error += fabsf(0.5f - velECEF[ECEFY]);
    error += fabsf(-.70710678118655f - velECEF[ECEFZ]);

    nedToECEFdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velNED, velECEF);
    error += fabsf(0.5f - velECEF[ECEFX]);
    error += fabsf(0.5f - velECEF[ECEFY]);
    error += fabsf(-.70710678118655f - velECEF[ECEFZ]);

    ecefToNEDtrigf(velECEF, velNED, &trig);
    error += fabsf(0.0f - velNED[NORTH]);
    error += fabsf(0.0f - velNED[EAST]);
    error += fabsf(1.0f - velNED[DOWN]);

    ecefToNEDdcm(&dcm, &trig);
    dcmApplyRotation(&dcm, velECEF, velNED);
    error += fabsf(0.0f - velNED[NORTH]);
    error += fabsf(0.0f - velNED[EAST]);
    error += fabsf(1.0f - velNED[DOWN]);

    if(error < 0.0001f)
        return TRUE;
    else
        return FALSE;

}// testEarthRotation
