/*!
 *  \file GelocateTelemetry.h
 *  \brief Information needed to locate the gimbals imagery on a map.
 *
 *  This module defines the GeolocateTelemetry_t structure which gives all the
 *  information needed to project the gimbals line of sight (typically until it
 *  intersects a terrain model). In addition this module gives the necessary
 *  functions to encode/decode this information to/from an OrionPacket. Note
 *  that the GEOLOCATE_TELEMETRY packet encodes the minimum amount of
 *  information; however the structure contains the position, velocity, and
 *  attitude data in multiple redundant forms, for the convenience of anyone
 *  who receives this data. The DecodeGeolocateTelemetry() function fills out
 *  the redundant data.
 */

#ifndef GEOLOCATETELEMETRY_H_
#define GEOLOCATETELEMETRY_H_

#include "earthposition.h"
#include "quaternion.h"
#include "Constants.h"
#include "OrionPublicPacket.h"
#include "OrionPublicPacketShim.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

//! The information needed to determine location of gimbal image
typedef struct
{
    // The basic gelocation data that is transmitted and received in the packet
    GeolocateTelemetryCore_t base;

	// Data below this point are not transmitted but instead are constructed from what is above

	//! Year of the date
	UInt16 Year;

	//! Month of the year, Jan == 1, Dec == 12
	UInt8  Month;

	//! Day of the month, from 1 to 31
	UInt8  Day;

	//! Hour of the day, from 0 to 23
	UInt8  Hour;

	//! Minute of the hour, from 0 to 59
	UInt8  Minute;

	//! Second of the minute, from 0 to 59
	UInt8  Second;

	//! Trigonometric information about the LLA position
	llaTrig_t llaTrig;

	//! Position in ECEF meters
	double posECEF[NECEF];

	//! Velocity in ECEF meters per second
	float velECEF[NECEF];

	//! Euler attitude of the gimbal (roll, pitch, yaw) in radians
	float gimbalEuler[NUM_AXES];

    //! The DCM of the gimbal (body to nav NED)
	structAllocateDCM(gimbalDcm);

    //! Quaternion attitude of the camera (body to nav NED)
	float cameraQuat[NQUATERNION];

	//! Euler attitude of the camera (roll, pitch, yaw) in radians
	float cameraEuler[NUM_AXES];

    //! The DCM of the camera (body to nav NED)
	structAllocateDCM(cameraDcm);

	//! Slant range to target in meters
	float slantRange;

    //! Image position in ECEF and LLA coordinates
    double imagePosECEF[NECEF];
    double imagePosLLA[NLLA];

}GeolocateTelemetry_t;

//! Number of entries to keep in the geolocate telemetry buffer
#define GEOLOCATE_BUFFER_SIZE 100

/*! A buffer of geolocate telemetry data */
typedef struct
{
	GeolocateTelemetry_t geobuf[GEOLOCATE_BUFFER_SIZE];
	uint32_t in;
	uint32_t holding;
	
}GeolocateBuffer_t;

//! Create a GeolocateTelemetry packet
void FormGeolocateTelemetry(OrionPkt_t *pPkt, const GeolocateTelemetry_t *pGeo);

//! Decode a GeolocateTelemetry packet
BOOL DecodeGeolocateTelemetry(const OrionPkt_t *pPkt, GeolocateTelemetry_t *pGeo);

//! Convert a GeolocateTelemetryCore_t structure to a GeolocateTelemetry_t
void ConvertGeolocateTelemetryCore(const GeolocateTelemetryCore_t *pCore, GeolocateTelemetry_t *pGeo);

//! Offset an image location according to a user click
BOOL offsetImageLocation(const GeolocateTelemetry_t *geo, const double imagePosLLA[NLLA], float ydev, float zdev, double newPosLLA[NLLA]);

//! Get the terrain intersection based on the current telemetry
BOOL getTerrainIntersection(const GeolocateTelemetry_t *pGeo, float (*getElevationHAE)(double, double), double PosLLA[NLLA], double *pRange);

//! Get the velocity of the terrain intersection
BOOL getImageVelocity(const GeolocateBuffer_t* buf, uint32_t dt, float imageVel[NNED]);

//! Push a new geolocate telemetry into a geolocate buffer
void pushGeolocateBuffer(GeolocateBuffer_t* buf, const GeolocateTelemetry_t* geo);

//! Copy a geolocate structure, which cannot be done with simple assignment due to the DCM pointers
void copyGeolocateTelemetry(const GeolocateTelemetry_t* source, GeolocateTelemetry_t* dest);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // GEOLOCATETELEMETRY_H_
