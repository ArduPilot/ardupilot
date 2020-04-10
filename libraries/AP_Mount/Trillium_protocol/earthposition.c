#include "earthposition.h"
#include "WGS84.h"
#include "Constants.h"
#include <math.h>


/*!
 * Compute the trigonmetric quantities of latitude and longitude
 * \param lla is the latitude, longitude, altitude array in rad, rad, meters.
 * \param trig receives the sin and cosine of latitude and longitude.
 * \return a const pointer to trig.
 */
const llaTrig_t* llaToTrig(const double lla[NLLA], llaTrig_t* trig)
{
    trig->cosLat = cos(lla[LAT]);
    trig->cosLon = cos(lla[LON]);
    trig->sinLat = sin(lla[LAT]);
    trig->sinLon = sin(lla[LON]);
    return trig;

}// llaToTrig


/*!
 * Convert a LLA position to Earth centered Earth fixed.
 * \param lla is the latitude, longitude, altitude array in rad, rad, meters.
 * \param ecef receives the X, Y, Z ECEF coordinates in meters.
 */
void llaToECEF(const double lla[NLLA], double ecef[NECEF])
{
    llaTrig_t trig;
    llaToECEFandTrig(lla, ecef, &trig);

}// llaToECEF


/*!
 * Convert a LLA position to Earth centered Earth fixed, and return the lla trig values.
 * \param lla is the latitude, longitude, altitude array in rad, rad, meters.
 * \param ecef receives the X, Y, Z ECEF coordinates in meters.
 * \param trig receives the lla trigonmetric values.
 */
void llaToECEFandTrig(const double lla[NLLA], double ecef[NECEF], llaTrig_t* trig)
{
    llaTrigToECEF(lla[ALT], ecef, llaToTrig(lla, trig));

}// llaToECEF_WithTrig


/*!
 * Convert a LLA position represented by trigonometric values to Earth
 * centered Earth fixed.
 * \param alt is the altitude of the LLA position with respect to the
 *        WGS-84 ellipsoid in meters.
 * \param ecef is a 3 element array that receives the X, Y, Z ECEF
 *        coordinates in meters.
 * \param trig points to the lla trigonometric values.
 */
void llaTrigToECEF(double alt, double ecef[NECEF], const llaTrig_t* trig)
{
    // Radius of East-West curvature in meters
    double Rc = radiusOfEWCurvFromSinLat(trig->sinLat);

    // PosECEF position data
    ecef[ECEFX] = (Rc+alt)*trig->cosLat*trig->cosLon;
    ecef[ECEFY] = (Rc+alt)*trig->cosLat*trig->sinLon;
    ecef[ECEFZ] = (Rc*(1.0 - datum_eSquared) + alt)*trig->sinLat;

}// llaTrigToECEF


/*!
 * Convert the ECEF (earth centered, earth fixed) position to geodetic
 * (latitude, longitude, altitude) position.  This conversion is not exact
 * and provides centimeter accuracy for heights below 1000km.  See
 * Browning, B. 1976. Transformation from spatial to geographical
 * coordinates. Survey Review XXIII: page 323-327.
 * \param ecef is the X, Y, Z ECEF position in meters.
 * \param lla receives the LLA position in radians, radians, meters.
 */
void ecefToLLA(const double ecef[NECEF], double lla[NLLA])
{
    llaTrig_t trig;
    ecefToLLAandTrig(ecef, lla, &trig);

}// ecefToLLA


/*!
 * Convert the ECEF (earth centered, earth fixed) position to geodetic
 * (latitude, longitude, altitude) position, and latitude and longitude
 * trigonometric values.  This conversion is not exact
 * and provides centimeter accuracy for heights below 1000km.  See
 * Browning, B. 1976. Transformation from spatial to geographical
 * coordinates. Survey Review XXIII: page 323-327.
 * \param ecef is the X, Y, Z ECEF position in meters.
 * \param lla receives the LLA position in radians, radians, meters.
 * \param trig receives the latitude and longitude trig values.*/
void ecefToLLAandTrig(const double ecef[NECEF], double lla[NLLA], llaTrig_t* trig)
{
    double psquared = ecef[ECEFX]*ecef[ECEFX] + ecef[ECEFY]*ecef[ECEFY];

    if(psquared == 0.0)
    {
        // We are on the Earth rotation axis, we could be
        // in the center, or at one of the poles
        lla[ALT] = fabs(ecef[ECEFZ]) - datum_semiMinorAxis;
        lla[LON] = 0.0;
        trig->cosLon = 1.0;
        trig->sinLon = 0.0;

        if(ecef[ECEFZ] == 0.0)
        {
            lla[LAT] = 0.0;
            trig->cosLat = 1.0;
            trig->sinLat = 0.0;

        }// if center of the earth
        else if(ecef[ECEFZ] > 0.0)
        {
            lla[LAT] = PId/2.0;
            trig->cosLat = 0.0;
            trig->sinLat = 1.0;

        }// else if north pole
        else
        {
            lla[LAT] = -PId/2.0;
            trig->cosLat = 0.0;
            trig->sinLat = -1.0;
        }

    }// if on the earth axis
    else
    {
        // distance from axis of rotation
        double p = sqrt(psquared);

        double zeta = atan2(ecef[ECEFZ]*datum_semiMajorAxis, p*datum_semiMinorAxis);
        double SinZeta = sin(zeta);
        double CosZeta = cos(zeta);

        // Latitude
        double num = ecef[ECEFZ] + datum_eSecondSquared*datum_semiMinorAxis*SinZeta*SinZeta*SinZeta;
        double den = p - datum_eSquared*datum_semiMajorAxis*CosZeta*CosZeta*CosZeta;

        // hypotenuse should never be zero
        double hyp = sqrt(num*num + den*den);
        lla[LAT] = atan2(num, den);
        trig->sinLat = num/hyp;
        trig->cosLat = den/hyp;

        // Since we had to compute num and den I belive the above code is faster
        // than calling sin and cos. The trade is one sqrt versus two trig
        // trig->sinLat = sin(lla[LAT]);
        // trig->cosLat = cos(lla[LAT]);

        // Longitude
        lla[LON] = atan2(ecef[ECEFY], ecef[ECEFX]);

        // Altitude is calculated differently at the poles, in order to avoid the singularity
        if(fabs(trig->cosLat) > 0.001)
            lla[ALT] = (p / trig->cosLat) - radiusOfEWCurvFromSinLat(trig->sinLat);
        else
            lla[ALT] = fabs(ecef[ECEFZ]) - datum_semiMinorAxis;

        // We do this just to fill out the trig. This is a lot cheaper
        // than later calling the sin() or cos() function. p will not be zero
        trig->cosLon = ecef[ECEFX]/p;
        trig->sinLon = ecef[ECEFY]/p;

    }// else if not on earth axis

}// ecefToLLA_withTrig


/*!
 * Convert an array of geodetic coordinates (latitude, longitude, altitude)
 * to spherical geocentric coordinates (latitude', longitude, radius).
 * Geodetic coordinates are the familiar latitude, longitude, and altitude
 * in which the locally level plane (tangent to the ellipsoid) has North,
 * East, -Down axes parallel to Latitude, Longitude, Altitude axes.  Because
 * WGS84 is an ellipsoid (not a sphere) the origin of the geodetic
 * coordinate system is not at the center of the earth, and moves along the
 * equatorial plane as latitude changes.  In contrast the geocentric 
 * coordinates have their origin at the center of the Earth.
 * 
 * \param geodetic is 3 element array of geodetic coordinates in latitude(rad),
 *        longitude(rad), altitude(m) with respect to the WGS-84 ellipsoid.
 * \param spherical is a 3 element array that receives the equivalent spherical
 *        coordinates in latitude'(rad), longitude(rad), radius(m).  spherical
 *        can refer to the same array as geodetic.
 */
void geodeticToGeocentric(const double geodetic[NLLA], double spherical[NLLA])
{
    // sin of the latitude
    double sinLat = sin(geodetic[LAT]);

    // Radius of East-West curvature in meters
    double Rc = radiusOfEWCurvFromSinLat(sinLat);

    // distance from the axis of rotation
    double p = (Rc + geodetic[ALT]) * cos(geodetic[LAT]);

    // distance from the plane of the equator
    double z = (Rc * (1 - datum_eSquared) + geodetic[ALT]) * sinLat;

    // radius in meters of the spherical coordinates
    spherical[ALT] = sqrt(p*p + z*z);

    // geocentric latitude in spherical coordinates
    spherical[LAT] = asin(z / spherical[ALT]);

    // Longitude is the same in both coordinate systems.
    spherical[LON] = geodetic[LON];
            
}// geodeticToGeocentric


/*!
 * Perform Earth position tests, converting between LLA and ECEF representations
 * \return TRUE if the tests pass
 */
BOOL testEarthPosition(void)
{
    double error = 0.0;
    double posECEF[3] = {datum_semiMajorAxis, 0.0, 0.0};
    double posLLA[3];
    llaTrig_t trig;

    ecefToLLAandTrig(posECEF, posLLA, &trig);
    error += fabs(0.0 - posLLA[LAT])*datum_meanRadius;
    error += fabs(0.0 - posLLA[LON])*datum_meanRadius;
    error += fabs(0.0 - posLLA[ALT]);
    error += fabs(1.0 - trig.cosLat);
    error += fabs(1.0 - trig.cosLon);
    error += fabs(0.0 - trig.sinLat);
    error += fabs(0.0 - trig.sinLon);

    llaToECEFandTrig(posLLA, posECEF, &trig);
    error += fabs(datum_semiMajorAxis - posECEF[0]);
    error += fabs(0.0 - posECEF[1]);
    error += fabs(0.0 - posECEF[2]);
    error += fabs(1.0 - trig.cosLat);
    error += fabs(1.0 - trig.cosLon);
    error += fabs(0.0 - trig.sinLat);
    error += fabs(0.0 - trig.sinLon);

    posECEF[0] = 0.0;
    posECEF[1] = datum_semiMajorAxis;
    ecefToLLAandTrig(posECEF, posLLA, &trig);
    error += fabs(0.0 - posLLA[LAT])*datum_meanRadius;
    error += fabs(PId/2.0 - posLLA[LON])*datum_meanRadius;
    error += fabs(0.0 - posLLA[ALT]);
    error += fabs(1.0 - trig.cosLat);
    error += fabs(0.0 - trig.cosLon);
    error += fabs(0.0 - trig.sinLat);
    error += fabs(1.0 - trig.sinLon);

    llaToECEFandTrig(posLLA, posECEF, &trig);
    error += fabs(datum_semiMajorAxis - posECEF[1]);
    error += fabs(0.0 - posECEF[0]);
    error += fabs(0.0 - posECEF[2]);
    error += fabs(1.0 - trig.cosLat);
    error += fabs(0.0 - trig.cosLon);
    error += fabs(0.0 - trig.sinLat);
    error += fabs(1.0 - trig.sinLon);

    posECEF[0] = 0.0;
    posECEF[1] = 0.0;
    posECEF[2] = datum_semiMajorAxis;
    ecefToLLAandTrig(posECEF, posLLA, &trig);
    error += fabs(PId/2.0 - posLLA[LAT])*datum_meanRadius;
    error += fabs(0.0 - posLLA[LON])*datum_meanRadius;
    error += fabs((datum_semiMajorAxis - datum_semiMinorAxis) - posLLA[ALT]);
    error += fabs(0.0 - trig.cosLat);
    error += fabs(1.0 - trig.cosLon);
    error += fabs(1.0 - trig.sinLat);
    error += fabs(0.0 - trig.sinLon);

    llaToECEFandTrig(posLLA, posECEF, &trig);
    error += fabs(datum_semiMajorAxis - posECEF[2]);
    error += fabs(0.0 - posECEF[1]);
    error += fabs(0.0 - posECEF[0]);
    error += fabs(0.0 - trig.cosLat);
    error += fabs(1.0 - trig.cosLon);
    error += fabs(1.0 - trig.sinLat);
    error += fabs(0.0 - trig.sinLon);

    posLLA[LAT] = PId/4.0;
    posLLA[LON] = -3.0*PId/4.0;
    posLLA[ALT] = 255.0;
    llaToECEFandTrig(posLLA, posECEF, &trig);
    ecefToLLAandTrig(posECEF, posLLA, &trig);
    error += fabs(PId/4.0 -  posLLA[LAT])*datum_meanRadius;
    error += fabs(-3.0*PId/4.0 -  posLLA[LON])*datum_meanRadius;
    error += fabs(255.0 - posLLA[ALT]);
    error += fabs(0.707106781186548 - trig.cosLat);
    error += fabs(-0.707106781186548 - trig.cosLon);
    error += fabs(0.707106781186548 - trig.sinLat);
    error += fabs(-0.707106781186548 - trig.sinLon);

    if(error < 0.0001)
        return TRUE;
    else
        return FALSE;

}// testEarthPosition
