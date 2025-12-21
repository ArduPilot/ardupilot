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
/*
  support for serial connected SBG INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

/*!
 * Clock status and UTC time status definitions.
 */
#define SBG_ECOM_CLOCK_STATUS_SHIFT			(1u)					/*!< Shift used to extract the clock status part. */
#define SBG_ECOM_CLOCK_STATUS_MASK			(0x000Fu)				/*!< Mask used to keep only the clock status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_SHIFT		(6u)					/*!< Shift used to extract the clock UTC status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_MASK		(0x000Fu)				/*!< Mask used to keep only the clock UTC status part. */

/*!
 * Clock status enum.
 */
typedef enum _SbgEComClockStatus
{
	SBG_ECOM_CLOCK_ERROR			= 0,							/*!< An error has occurred on the clock estimation. */
	SBG_ECOM_CLOCK_FREE_RUNNING		= 1,							/*!< The clock is only based on the internal crystal. */
	SBG_ECOM_CLOCK_STEERING			= 2,							/*!< A PPS has been detected and the clock is converging to it. */
	SBG_ECOM_CLOCK_VALID			= 3								/*!< The clock has converged to the PPS and is within 500ns. */
} SbgEComClockStatus;

/*!
 * Status for the UTC time data.
 */
typedef enum _SbgEComClockUtcStatus
{
	SBG_ECOM_UTC_INVALID			= 0,							/*!< The UTC time is not known, we are just propagating the UTC time internally. */
	SBG_ECOM_UTC_NO_LEAP_SEC		= 1,							/*!< We have received valid UTC time information but we don't have the leap seconds information. */
	SBG_ECOM_UTC_VALID				= 2								/*!< We have received valid UTC time data with valid leap seconds. */
} SbgEComClockUtcStatus;

static inline SbgEComClockStatus sbgEComLogUtcGetClockStatus(uint16_t status) {
	return (SbgEComClockStatus)((status >> SBG_ECOM_CLOCK_STATUS_SHIFT) & SBG_ECOM_CLOCK_STATUS_MASK);
}
static inline SbgEComClockUtcStatus sbgEComLogUtcGetClockUtcStatus(uint16_t status) {
	return (SbgEComClockUtcStatus)((status >> SBG_ECOM_CLOCK_UTC_STATUS_SHIFT) & SBG_ECOM_CLOCK_UTC_STATUS_MASK);
}


typedef struct PACKED _SbgLogUtcData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< UTC time and clock status information */
	uint16_t	year;						/*!< Year for example: 2013. */
	int8_t      month;						/*!< Month in year [1 .. 12]. */
	int8_t      day;						/*!< Day in month [1 .. 31]. */
	int8_t      hour;						/*!< Hour in day [0 .. 23]. */
	int8_t      minute;						/*!< Minute in hour [0 .. 59]. */
	int8_t      second;						/*!< Second in minute [0 .. 60]. (60 is used only when a leap second is added) */
	int32_t     nanoSecond;					/*!< Nanosecond of current second in ns. */
	uint32_t    gpsTimeOfWeek;				/*!< GPS time of week in ms. */
} SbgLogUtcData;



/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_VEL message.
 */
typedef struct PACKED _SbgLogGpsVel
{
	uint32_t		timeStamp;				/*!< Time in us since the sensor power up. */
	uint32_t		status;					/*!< GPS velocity status, type and bitmask. */
	uint32_t		timeOfWeek;				/*!< GPS time of week in ms. */
	float			velocity[3];			/*!< GPS North, East, Down velocity in m.s^-1. */
	float			velocityAcc[3];			/*!< GPS North, East, Down velocity 1 sigma accuracy in m.s^-1. */
	float			course;					/*!< Track ground course in degrees. */
	float			courseAcc;				/*!< Course accuracy in degrees. */
} SbgLogGpsVel;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_POS message.
 */
typedef struct PACKED _SbgLogGpsPos
{
	uint32_t		timeStamp;				/*!< Time in us since the sensor power up. */
	uint32_t		status;					/*!< GPS position status, type and bitmask. */
	uint32_t		timeOfWeek;				/*!< GPS time of week in ms. */
	double			latitude;				/*!< Latitude in degrees, positive north. */
	double			longitude;				/*!< Longitude in degrees, positive east. */
	double			altitude;				/*!< Altitude above Mean Sea Level in meters. */
	float			undulation;				/*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
	float			latitudeAccuracy;		/*!< 1 sigma latitude accuracy in meters. */
	float			longitudeAccuracy;		/*!< 1 sigma longitude accuracy in meters. */
	float			altitudeAccuracy;		/*!< 1 sigma altitude accuracy in meters. */
	uint8_t			numSvUsed;				/*!< Number of space vehicles used to compute the solution (since version 1.4). */
	uint16_t		baseStationId;			/*!< Base station id for differential corrections (0-4095). Set to 0xFFFF if differential corrections are not used (since version 1.4). */
	uint16_t		differentialAge;		/*!< Differential correction age in 0.01 seconds. Set to 0XFFFF if differential corrections are not used (since version 1.4). */
} SbgLogGpsPos;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_HDT message.
 */
typedef struct PACKED _SbgLogGpsHdt
{
	uint32_t		timeStamp;				/*!< Time in us since the sensor power up. */
	uint16_t		status;					/*!< GPS HDT status, type and bitmask. */
	uint32_t		timeOfWeek;				/*!< GPS time of week in ms. */
	float			heading;				/*!< GPS true heading in degrees. */
	float			headingAccuracy;		/*!< 1 sigma GPS true heading accuracy in degrees. */
	float			pitch;					/*!< GPS pitch angle measured from the master to the rover in degrees. */
	float			pitchAccuracy;			/*!< 1 signa GPS pitch angle accuarcy in degrees. */
} SbgLogGpsHdt;



/*!
 * Structure that stores data for the SBG_ECOM_LOG_IMU_DATA message.
 */
typedef struct PACKED _SbgLogImuData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< IMU status bitmask. */
	float		accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
	float		gyroscopes[3];				/*!< X, Y, Z gyroscopes in rad.s^-1. */
	float		temperature;				/*!< Internal temperature in C. */	
	float		deltaVelocity[3];			/*!< X, Y, Z delta velocity in m.s^-2. */
	float		deltaAngle[3];				/*!< X, Y, Z delta angle in rad.s^-1. */
} SbgLogImuData;

/*!
 * Structure that stores the data for SBG_ECOM_LOG_FAST_IMU_DATA message
 */
typedef struct PACKED _SbgLogFastImuData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< IMU status bitmask. */
	float		accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
	float		gyroscopes[3];				/*!< X, Y, Z gyroscopes in rad.s^-1. */
} SbgLogFastImuData;

/*!
 * Log inertial data status mask definitions
 */
#define SBG_ECOM_IMU_COM_OK                 (0x00000001u << 0)      /*!< Set to 1 if the communication with the IMU is ok. */
#define SBG_ECOM_IMU_STATUS_BIT             (0x00000001u << 1)      /*!< Set to 1 if the IMU passes general Built in Tests (calibration, CPU, ...). */

#define SBG_ECOM_IMU_ACCEL_X_BIT            (0x00000001u << 2)      /*!< Set to 1 if the accelerometer X passes Built In Test. */
#define SBG_ECOM_IMU_ACCEL_Y_BIT            (0x00000001u << 3)      /*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define SBG_ECOM_IMU_ACCEL_Z_BIT            (0x00000001u << 4)      /*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define SBG_ECOM_IMU_GYRO_X_BIT             (0x00000001u << 5)      /*!< Set to 1 if the gyroscope X passes Built In Test. */
#define SBG_ECOM_IMU_GYRO_Y_BIT             (0x00000001u << 6)      /*!< Set to 1 if the gyroscope Y passes Built In Test. */
#define SBG_ECOM_IMU_GYRO_Z_BIT             (0x00000001u << 7)      /*!< Set to 1 if the gyroscope Z passes Built In Test. */

#define SBG_ECOM_IMU_ACCELS_IN_RANGE        (0x00000001u << 8)      /*!< Set to 1 if all accelerometers are within operating range. */
#define SBG_ECOM_IMU_GYROS_IN_RANGE         (0x00000001u << 9)      /*!< Set to 1 if all gyroscopes are within operating range. */
#define SBG_ECOM_IMU_GYROS_USE_HIGH_SCALE   (0x00000001u << 10)     /*!< Set if the gyroscope scale factor range is high. Applicable only for SBG_ECOM_LOG_IMU_SHORT logs. */

/*!
 * Standard gyroscope scale factor.
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_STD                     (67108864.0f)

/*!
 * High range gyroscope scale factor derived from 10000 degrees per second maximum range.
 *
 * Calculation: (2^31 - 1) / (10000 * π / 180)
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_HIGH                    (12304174.0f)

/*!
 * Maximum value for the standard scale factor, in radians per second.
 *
 * Approximately equivalent to 1833 °/s.
 */
#define SBG_ECOM_LOG_IMU_GYRO_SCALE_STD_MAX_RAD             ((float)INT32_MAX / SBG_ECOM_LOG_IMU_GYRO_SCALE_STD)

/*!
 * Standard accelerometer scale factor.
 */
#define SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD                    (1048576.0f)

/*!
 * Standard temperature scale factor.
 */
#define SBG_ECOM_LOG_IMU_TEMP_SCALE_STD                     (256.0f)

/*!
 * Structure that stores data for the SBG_ECOM_LOG_IMU_SHORT message.
 *
 * This message is sent asynchronously and must be used for post processing.
 *
 * The delta angle values are scaled based on the gyroscopes output. If any output exceeds
 * a predefined limit, the scale factor switches from standard to high range to prevent saturation.
 */
typedef struct PACKED _SbgEComLogImuShort
{
    uint32_t    timeStamp;                  /*!< Time in us since the sensor power up. */
    uint16_t    status;                     /*!< IMU status bitmask. */
    int32_t     deltaVelocity[3];           /*!< X, Y, Z delta velocity. Unit is 1048576 LSB for 1 m.s^-2. */
    int32_t     deltaAngle[3];              /*!< X, Y, Z delta angle. Unit is either 67108864 LSB for 1 rad.s^-1 (standard) or 12304174 LSB for 1 rad.s^-1 (high range), managed automatically. */
    int16_t     temperature;                /*!< IMU average temperature. Unit is 256 LSB for 1°C. */
} SbgEComLogImuShort;



#define SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK         (0x0000000Fu)       /*!< Mask used to keep only the clock status part. */

/*!
 * Solution filter mode enum.
 */
typedef enum _SbgEComSolutionMode
{
    SBG_ECOM_SOL_MODE_UNINITIALIZED         = 0,                    /*!< The Kalman filter is not initialized and the returned data are all invalid. */
    SBG_ECOM_SOL_MODE_VERTICAL_GYRO         = 1,                    /*!< The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. */
    SBG_ECOM_SOL_MODE_AHRS                  = 2,                    /*!< A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. */
    SBG_ECOM_SOL_MODE_NAV_VELOCITY          = 3,                    /*!< The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. */
    SBG_ECOM_SOL_MODE_NAV_POSITION          = 4                     /*!< Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided. */
} SbgEComSolutionMode;

/*!
 * EKF computed orientation using euler angles.
 */
typedef struct PACKED _SbgLogEkfEulerData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	float		euler[3];					/*!< Roll, Pitch and Yaw angles in rad. */
	float		eulerStdDev[3];				/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
	uint32_t	status;						/*!< EKF solution status bitmask and enum. */
} SbgLogEkfEulerData;

/*!
 * EFK computed orientation using quaternion.
 */
typedef struct PACKED _SbgLogEkfQuatData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	float		quaternion[4];				/*!< Orientation quaternion stored in W, X, Y, Z form. */
	float		eulerStdDev[3];				/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
	uint32_t	status;						/*!< EKF solution status bitmask and enum. */
} SbgLogEkfQuatData;

/*!
 * EFK computed navigation data.
 */
typedef struct PACKED _SbgLogEkfNavData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	float		velocity[3];				/*!< North, East, Down velocity in m.s^-1. */
	float		velocityStdDev[3];			/*!< North, East, Down velocity 1 sigma standard deviation in m.s^-1. */
	double		position[3];				/*!< Latitude, Longitude in degrees positive North and East.
										 	Altitude above Mean Sea Level in meters. */
	float		undulation;					/*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
	float		positionStdDev[3];			/*!< Latitude, longitude and altitude 1 sigma standard deviation in meters. */
	uint32_t	status;						/*!< EKF solution status bitmask and enum. */
} SbgLogEkfNavData;

//----------------------------------------------------------------------//
//- Log Air Data status definitions                                    -//
//----------------------------------------------------------------------//

/*!
 * Air Data sensor status mask definitions
 */
#define SBG_ECOM_AIR_DATA_TIME_IS_DELAY             (0x0001u << 0)      /*!< Set to 1 if the timeStamp field represents a delay instead of an absolute timestamp. */
#define SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID        (0x0001u << 1)      /*!< Set to 1 if the pressure field is filled and valid. */
#define SBG_ECOM_AIR_DATA_ALTITUDE_VALID            (0x0001u << 2)      /*!< Set to 1 if the barometric altitude field is filled and valid. */
#define SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID       (0x0001u << 3)      /*!< Set to 1 if the differential pressure field is filled and valid. */
#define SBG_ECOM_AIR_DATA_AIRPSEED_VALID            (0x0001u << 4)      /*!< Set to 1 if the true airspeed field is filled and valid. */
#define SBG_ECOM_AIR_DATA_TEMPERATURE_VALID         (0x0001u << 5)      /*!< Set to 1 if the output air temperature field is filled and valid. */

/*!
 * Log structure for AirData.
 */
typedef struct PACKED _SbgLogAirData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up OR measurement delay in us. */
	uint16_t	status;						/*!< Airdata sensor status bitmask. */
	float		pressureAbs;				/*!< Raw absolute pressure measured by the barometer sensor in Pascals. */
	float		altitude;					/*!< Altitude computed from barometric altimeter in meters and positive upward. */
	float		pressureDiff;				/*!< Raw differential pressure measured by the pitot tube in Pascal. */
	float		trueAirspeed;				/*!< True airspeed measured by a pitot tube in m.s^-1 and positive forward. */
	float		airTemperature;				/*!< Outside air temperature in C that could be used to compute true airspeed from differential pressure. */
} SbgLogAirData;

/*!
 * Log magnetometer data status mask definitions
 */
#define SBG_ECOM_MAG_MAG_X_BIT          (0x00000001u << 0)      /*!< Set to 1 if the magnetometer X passes Built In Test. */
#define SBG_ECOM_MAG_MAG_Y_BIT          (0x00000001u << 1)      /*!< Set to 1 if the magnetometer Y passes Built In Test. */
#define SBG_ECOM_MAG_MAG_Z_BIT          (0x00000001u << 2)      /*!< Set to 1 if the magnetometer Z passes Built In Test. */

#define SBG_ECOM_MAG_ACCEL_X_BIT        (0x00000001u << 3)      /*!< Set to 1 if the accelerometer X passes Built In Test. */
#define SBG_ECOM_MAG_ACCEL_Y_BIT        (0x00000001u << 4)      /*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define SBG_ECOM_MAG_ACCEL_Z_BIT        (0x00000001u << 5)      /*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define SBG_ECOM_MAG_MAGS_IN_RANGE      (0x00000001u << 6)      /*!< Set to 1 if all magnetometers are within operating range. */
#define SBG_ECOM_MAG_ACCELS_IN_RANGE    (0x00000001u << 7)      /*!< Set to 1 if all accelerometers are within operating range. */

#define SBG_ECOM_MAG_CALIBRATION_OK     (0x00000001u << 8)      /*!< Set to 1 if the magnetometers seems to be calibrated. */

/*!
 * Structure that stores data for the SBG_ECOM_LOG_MAG message.
 */
typedef struct PACKED _SbgLogMag
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< Magnetometer status bitmask. */
	float		magnetometers[3];			/*!< X, Y, Z magnetometer data in A.U. */
	float		accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
} SbgLogMag;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_MAG_CALIB message.
 */
typedef struct PACKED _SbgLogMagCalib
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	reserved;					/*!< Reserved for future use. */
	uint8_t		magData[16];				/*!< Magnetometers calibration data. */
} SbgLogMagCalib;


typedef struct PACKED _SbgLogStatusData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	generalStatus;				/*!< General status bitmask and enums. */
	uint16_t	reserved1;					/*!< Reserved status field for future use */
	uint32_t	comStatus;					/*!< Communication status bitmask and enums. */
	uint32_t	aidingStatus;				/*!< Aiding equipments status bitmask and enums. */
	uint32_t	reserved2;					/*!< Reserved status field for future use. */
	uint16_t	reserved3;					/*!< Reserved status field for future use. */
	uint32_t	uptime;						/*!< System uptime in seconds. */
} SbgLogStatusData;

/*!
 * Helper structure to retrieve device info.
 */
#define SBG_ECOM_INFO_PRODUCT_CODE_LENGTH			(32)
typedef struct PACKED _SbgEComDeviceInfo
{
	uint8_t		productCode[SBG_ECOM_INFO_PRODUCT_CODE_LENGTH];	/*!< Human readable Product Code. */
	uint32_t	serialNumber;				/*!< Device serial number */
	uint32_t	calibationRev;				/*!< Calibration data revision */
	uint16_t	calibrationYear;			/*!< Device Calibration Year */
	uint8_t		calibrationMonth;			/*!< Device Calibration Month */
	uint8_t		calibrationDay;				/*!< Device Calibration Day */
	uint32_t	hardwareRev;				/*!< Device hardware revision */
	uint32_t	firmwareRev;				/*!< Firmware revision */
} SbgEComDeviceInfo;



/*!
 *	Generic errors definitions for SBG Systems projects.
 */
typedef enum _SbgErrorCode : uint16_t
{
	SBG_NO_ERROR = 0,						/*!< The operation was successfully executed. */
	SBG_ERROR,								/*!< We have a generic error. */
	SBG_NULL_POINTER,						/*!< A pointer is null. */
	SBG_INVALID_CRC,						/*!< The received frame has an invalid CRC. */
	SBG_INVALID_FRAME,						/*!< The received frame is invalid <br> */
											/*!<	We have received an unexpected frame (not the cmd we are waiting for or with an invalid data size.<br> */
											/*!<	This could be caused by a desync between questions and answers.<br> */
											/*!<	You should flush the serial port to fix this. */
	SBG_TIME_OUT,							/*!< We have started to receive a frame but not the end. */
	SBG_WRITE_ERROR,						/*!< All bytes hasn't been written. */
	SBG_READ_ERROR,							/*!< All bytes hasn't been read. */
	SBG_BUFFER_OVERFLOW,					/*!< A buffer is too small to contain so much data. */
	SBG_INVALID_PARAMETER,					/*!< An invalid parameter has been found. */
	SBG_NOT_READY,							/*!< A device isn't ready (Rx isn't ready for example). */
	SBG_MALLOC_FAILED,						/*!< Failed to allocate a buffer. */
	SGB_CALIB_MAG_NOT_ENOUGH_POINTS,		/*!< Not enough points were available to perform magnetometers calibration. */
	SBG_CALIB_MAG_INVALID_TAKE,				/*!< The calibration procedure could not be properly executed due to insufficient precision. */
	SBG_CALIB_MAG_SATURATION,				/*!< Saturation were detected when attempt to calibrate magnetos. */
	SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE,	/*!< 2D calibration procedure could not be performed. */

	SBG_DEVICE_NOT_FOUND,					/*!< A device couldn't be founded or opened PC only error code */
	SBG_OPERATION_CANCELLED,				/*!< An operation was cancelled.  PC only error code*/
	SBG_NOT_CONTINUOUS_FRAME,				/*!< We have received a frame that isn't a continuous one. PC only error code*/

	SBG_INCOMPATIBLE_HARDWARE,				/*!< Hence valid; the command cannot be executed because of hardware incompatibility */
	SBG_INVALID_VERSION						/*!< Incompatible version */
} SbgErrorCode;


typedef struct PACKED _SbgEComACK
{
	uint8_t	ackMsg;
	uint8_t	ackMsgClass;
	SbgErrorCode ackErrorCode;
} SbgEComAck;

/*!
 * Defintion of all the settings actions available.
 */
typedef enum _SbgEComSettingsAction
{
	SBG_ECOM_REBOOT_ONLY 				= 0,		/*!< Only reboot the device. */
	SBG_ECOM_SAVE_SETTINGS				= 1,		/*!< Save the settings to non-volatile memory and then reboot the device. */
	SBG_ECOM_RESTORE_DEFAULT_SETTINGS	= 2			/*!< Restore default settings, save them to non-volatile memory and reboot the device. */
} SbgEComSettingsAction;


typedef enum _SbgEComGpsPosType
{
	SBG_ECOM_POS_NO_SOLUTION		= 0,							/*!< No valid solution available. */
	SBG_ECOM_POS_UNKNOWN_TYPE		= 1,							/*!< An unknown solution type has been computed. */
	SBG_ECOM_POS_SINGLE				= 2,							/*!< Single point solution position. */
	SBG_ECOM_POS_PSRDIFF			= 3,							/*!< Standard Pseudorange Differential Solution (DGPS). */
	SBG_ECOM_POS_SBAS				= 4,							/*!< SBAS satellite used for differential corrections. */
	SBG_ECOM_POS_OMNISTAR			= 5,							/*!< Omnistar VBS Position (L1 sub-meter). */
	SBG_ECOM_POS_RTK_FLOAT			= 6,							/*!< Floating RTK ambiguity solution (20 cms RTK). */
	SBG_ECOM_POS_RTK_INT			= 7,							/*!< Integer RTK ambiguity solution (2 cms RTK). */
	SBG_ECOM_POS_PPP_FLOAT			= 8,							/*!< Precise Point Positioning with float ambiguities. */
	SBG_ECOM_POS_PPP_INT			= 9,							/*!< Precise Point Positioning with fixed ambiguities. */
	SBG_ECOM_POS_FIXED				= 10							/*!< Fixed location solution position. */
} SbgEComGpsPosType;

#define SBG_ECOM_GPS_POS_TYPE_SHIFT			(6u)					/*!< Shift used to extract the GPS position type part. */
#define SBG_ECOM_GPS_POS_TYPE_MASK			(0x0000003Fu)			/*!< Mask used to keep only the GPS position type part. */



/*!
 *	Define if the onboard magnetic calibration should acquiere points for a 3D or 2D calibration.
 */
typedef enum _SbgEComMagCalibMode
{
	SBG_ECOM_MAG_CALIB_MODE_2D			= 1,				/*!< Tell the device that the magnetic calibration will be performed with limited motions.
																 This calibration mode is only designed to be used when roll and pitch motions are less than  5.
																 To work correctly, the device should be rotated through at least a full circle. */
	SBG_ECOM_MAG_CALIB_MODE_3D			= 2					/*!< Tell the device to start a full 3D magnetic calibration procedure.
																 The 3D magnetic calibration offers the best accuracy but needs at least motion of  30 on the roll and pitch angles. */
} SbgEComMagCalibMode;


/*!
 *	Used to select the expected dynamics during the magnetic calibration.
 */
typedef enum _SbgEComMagCalibBandwidth
{
	SBG_ECOM_MAG_CALIB_LOW_BW		= 0,					/*!< Tell the device that low dynamics will be observed during the magnetic calibration process. */
	SBG_ECOM_MAG_CALIB_MEDIUM_BW	= 1,					/*!< Tell the device that normal dynamics will be observed during the magnetic calibration process. */
	SBG_ECOM_MAG_CALIB_HIGH_BW		= 2						/*!< Tell the device that high dynamics will be observed during the magnetic calibration process. */
} SbgEComMagCalibBandwidth;

/*!
 *	General quality indicator of an onboard magnetic calibration.
 */
typedef enum _SbgEComMagCalibQuality
{
	SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL	= 0,					/*!< All acquired points fit very well on a unit sphere after the calibration. */
	SBG_ECOM_MAG_CALIB_QUAL_GOOD	= 1,					/*!< Small deviations of the magnetic field norm have been detected. The magnetic calibration should although provide accurate heading. */
	SBG_ECOM_MAG_CALIB_QUAL_POOR	= 2,					/*!< Large deviations of the magnetic field norm have been detected. It may come from external magnetic distortions during the calibration. */
	SBG_ECOM_MAG_CALIB_QUAL_INVALID	= 3						/*!< No valid magnetic calibration has been computed. It could comes from too much magnetic disturbances, insufficient or invalid motions. */
} SbgEComMagCalibQuality;

/*!
 *	Confidence indicator on results of an onbard magnetic calibration.
 */
typedef enum _SbgEComMagCalibConfidence
{
	SBG_ECOM_MAG_CALIB_TRUST_HIGH	= 0,					/*!< Reported quality indicator can be trusted as enough remarkable magnetic field points have been acquired. */
	SBG_ECOM_MAG_CALIB_TRUST_MEDIUM	= 1,					/*!< Few remarkable magnetic field points have been used to compute the magnetic calibration leading to a medium confidence in reported quality indicators. */
	SBG_ECOM_MAG_CALIB_TRUST_LOW	= 2						/*!< Even if the quality indicator could report an excellent calibration,
																 The data set used to compute the magnetic calibration was not meaningful enough to compute meaningful quality indicators.
																 This calibration should be used carefully. */
} SbgEComMagCalibConfidence;

#define SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS	(0x0001u)	/*!< Not enough valid magnetic points have been acquired. */
#define SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS	(0x0002u)	/*!< Unable to compute a magnetic calibration due to magnetic interferences or incorrect data set distribution. */
#define SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE		(0x0004u)	/*!< For a 3D calibration: not enough motion on X axis. For a 2D calibration; too much motion on X axis. */
#define SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE		(0x0008u)	/*!< For a 3D calibration: not enough motion on Y axis. For a 2D calibration; too much motion on Y axis. */
#define SBG_ECOM_MAG_CALIB_Z_MOTION_ISSUE		(0x0010u)	/*!< For a 3D or 2D calibration: not enough motion on Z axis. */
#define SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE		(0x0020u)	/*!< For a 3D calibration: the alignment between the magnetometers and the inertial frame seems to be invalid. */ 


/*!
 * Helper structure to retrieve onboard magnetic calibration results.
 */
typedef struct PACKED _SbgEComMagCalibResults
{
	SbgEComMagCalibQuality		quality;					/*!< General magnetic calibration quality indicator. */
	SbgEComMagCalibConfidence	confidence;					/*!< Confidence indicator that should be read to interpret the quality indicator. */
	uint16_t					advancedStatus;				/*!< Set of bit masks used to report advanced information on the magnetic calibration status.*/

	float						beforeMeanError;			/*!< Mean magnetic field norm error observed before calibration. */
	float						beforeStdError;				/*!< Standard deviation of the magnetic field norm error observed before calibration. */
	float						beforeMaxError;				/*!< Maximum magnetic field norm error observed before calibration. */

	float						afterMeanError;				/*!< Mean magnetic field norm error observed after calibration. */
	float						afterStdError;				/*!< Standard deviation of the magnetic field norm error observed after calibration. */
	float						afterMaxError;				/*!< Maximum magnetic field norm error observed after calibration. */

	float						meanAccuracy;				/*!< Mean expected heading accuracy in radians. */
	float						stdAccuracy;				/*!< Standard deviation of the expected heading accuracy in radians. */
	float						maxAccuracy;				/*!< Maximum expected heading accuracy in radians. */

	uint16_t					numPoints;					/*!< Number of magnetic field points stored internally and used to compute the magnetic calibration. */
	uint16_t					maxNumPoints;				/*!< Maximum number of magnetic field points that can be stored internally. */
	float						offset[3];					/*!< Computed Hard Iron correction vector offset. */
	float						matrix[9];					/*!< Computed Hard & Soft Iron correction matrix. */
} SbgEComMagCalibResults;


typedef struct _SbgEComSetMagCal
{
	float						offset[3];					/*!< Computed Hard Iron correction vector offset. */
	float						matrix[9];					/*!< Computed Hard & Soft Iron correction matrix. */
} SbgEComSetMagCal;


/*!
 * Enum that defines all the message classes available.
 */
typedef enum _SbgEComClass
{
	SBG_ECOM_CLASS_LOG_ECOM_0			= 0x00,			/*!< Class that contains sbgECom protocol input/output log messages. */

	SBG_ECOM_CLASS_LOG_ECOM_1			= 0x01,			/*!< Class that contains special sbgECom output messages that handle high frequency output */

	SBG_ECOM_CLASS_LOG_NMEA_0			= 0x02,			/*!< Class that contains NMEA (and NMEA like) output logs. <br>
															 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_NMEA_1			= 0x03,			/*!< Class that contains proprietary NMEA (and NMEA like) output logs. <br>
															 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_THIRD_PARTY_0	= 0x04,			/*!< Class that contains third party output logs.
															Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_CMD_0			= 0x10			/*!< Class that contains sbgECom protocol commands */
} SbgEComClass;

//----------------------------------------------------------------------//
//- Definition of all messages id for sbgECom                          -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the available ECom output logs from the sbgECom library.
 */
typedef enum _SbgEComLog
{
    SBG_ECOM_LOG_STATUS                     = 1,        /*!< Status general, clock, com aiding, solution, heave */

    SBG_ECOM_LOG_UTC_TIME                   = 2,        /*!< Provides UTC time reference */

    SBG_ECOM_LOG_IMU_DATA                   = 3,        /*!< DEPRECATED: Synchronous IMU measurements (time aligned to UTC - NEVER use for Post Processing). */

    SBG_ECOM_LOG_MAG                        = 4,        /*!< Magnetic data with associated accelerometer on each axis */
    SBG_ECOM_LOG_MAG_CALIB                  = 5,        /*!< Magnetometer calibration data (raw buffer) */

    SBG_ECOM_LOG_EKF_EULER                  = 6,        /*!< Includes roll, pitch, yaw and their accuracies on each axis */
    SBG_ECOM_LOG_EKF_QUAT                   = 7,        /*!< Includes the 4 quaternions values */
    SBG_ECOM_LOG_EKF_NAV                    = 8,        /*!< Position and velocities in NED coordinates with the accuracies on each axis */

    SBG_ECOM_LOG_SHIP_MOTION                = 9,        /*!< Heave, surge and sway and accelerations on each axis. */

    SBG_ECOM_LOG_GPS1_VEL                   = 13,       /*!< GPS velocities from primary or secondary GPS receiver */
    SBG_ECOM_LOG_GPS1_POS                   = 14,       /*!< GPS positions from primary or secondary GPS receiver */
    SBG_ECOM_LOG_GPS1_HDT                   = 15,       /*!< GPS true heading from dual antenna system */

    SBG_ECOM_LOG_GPS2_VEL                   = 16,       /*!< GPS 2 velocity log data. */
    SBG_ECOM_LOG_GPS2_POS                   = 17,       /*!< GPS 2 position log data. */
    SBG_ECOM_LOG_GPS2_HDT                   = 18,       /*!< GPS 2 true heading log data. */

    SBG_ECOM_LOG_ODO_VEL                    = 19,       /*!< Provides odometer velocity */

    SBG_ECOM_LOG_EVENT_A                    = 24,       /*!< Event markers sent when events are detected on sync in A pin */
    SBG_ECOM_LOG_EVENT_B                    = 25,       /*!< Event markers sent when events are detected on sync in B pin */
    SBG_ECOM_LOG_EVENT_C                    = 26,       /*!< Event markers sent when events are detected on sync in C pin */
    SBG_ECOM_LOG_EVENT_D                    = 27,       /*!< Event markers sent when events are detected on sync in D pin */
    SBG_ECOM_LOG_EVENT_E                    = 28,       /*!< Event markers sent when events are detected on sync in E pin */

    SBG_ECOM_LOG_DVL_BOTTOM_TRACK           = 29,       /*!< Doppler Velocity Log for bottom tracking data. */
    SBG_ECOM_LOG_DVL_WATER_TRACK            = 30,       /*!< Doppler Velocity log for water layer data. */

    SBG_ECOM_LOG_GPS1_RAW                   = 31,       /*!< GPS 1 raw data for post processing. */

    SBG_ECOM_LOG_SHIP_MOTION_HP             = 32,       /*!< Return delayed ship motion such as surge, sway, heave. */

    SBG_ECOM_LOG_AIR_DATA                   = 36,       /*!< Air Data aiding such as barometric altimeter and true air speed. */

    SBG_ECOM_LOG_USBL                       = 37,       /*!< Raw USBL position data for sub-sea navigation. */

    SBG_ECOM_LOG_GPS2_RAW                   = 38,       /*!< GPS 2 raw data for post processing. */


    SBG_ECOM_LOG_IMU_SHORT                  = 44,       /*!< Asynchronous IMU measurements output at the IMU rate and to use for Post Processing with Qinertia. */

    SBG_ECOM_LOG_EVENT_OUT_A                = 45,       /*!< Event marker used to timestamp each generated Sync Out A signal. */
    SBG_ECOM_LOG_EVENT_OUT_B                = 46,       /*!< Event marker used to timestamp each generated Sync Out B signal. */

    SBG_ECOM_LOG_DEPTH                      = 47,       /*!< Depth sensor measurement log used for sub-sea navigation. */
    SBG_ECOM_LOG_DIAG                       = 48,       /*!< Diagnostic log. */

    SBG_ECOM_LOG_RTCM_RAW                   = 49,       /*!< RTCM raw data. */

    SBG_ECOM_LOG_GPS1_SAT                   = 50,       /*!< GPS 1 Satellite data. */
    SBG_ECOM_LOG_GPS2_SAT                   = 51,       /*!< GNSS2 Satellite data. */

    SBG_ECOM_LOG_EKF_ROT_ACCEL_BODY         = 52,       /*!< INS body rotation rate and lateral acceleration (bias, earth rotation and gravity free compensated). */
    SBG_ECOM_LOG_EKF_ROT_ACCEL_NED          = 53,       /*!< INS North/East/Down rotation rate and lateral acceleration (bias, earth rotation and gravity free compensated). */
    SBG_ECOM_LOG_EKF_VEL_BODY               = 54,       /*!< INS X,Y,Z body velocity and standard deviation. */

    SBG_ECOM_LOG_SESSION_INFO               = 55,       /*!< Session information, including device information and current settings. */

    SBG_ECOM_LOG_EKF_DEBUG                  = 56,       /*!< EKF debug data. */

    SBG_ECOM_LOG_PTP_STATUS                 = 57,       /*!< PTP status. */

    SBG_ECOM_LOG_VELOCITY_1                 = 58,       /*!< Generic velocity 1 log. */

    SBG_ECOM_LOG_VIB_MON_FFT                = 59,       /*!< Vibration monitoring FFT data for post processing. */
    SBG_ECOM_LOG_VIB_MON_REPORT             = 60,       /*!< Vibration monitoring report information. */

    SBG_ECOM_LOG_ECOM_NUM_MESSAGES                      /*!< Helper definition to know the number of ECom messages */
} SbgEComLog;

/*!
 * Enum that defines all the available ECom output logs in the class SBG_ECOM_CLASS_LOG_ECOM_1
 */
typedef enum _SbgEComLog1MsgId
{
	SBG_ECOM_LOG_FAST_IMU_DATA 				= 0,		/*!< Provides accelerometers, gyroscopes, time and status at 1KHz rate. */
	SBG_ECOM_LOG_ECOM_1_NUM_MESSAGES					/*!< Helper definition to know the number of ECom messages */
} SbgEComLog1;


/*!
 * Enum that defines all the available commands for the sbgECom library.
 */
typedef enum _SbgEComCmd
{
	/* Acknowledge */
	SBG_ECOM_CMD_ACK			 			= 0,		/*!< Acknowledge */

	/* Special settings commands */
	SBG_ECOM_CMD_SETTINGS_ACTION 			= 1,		/*!< Performs various settings actions */
	SBG_ECOM_CMD_IMPORT_SETTINGS 			= 2,		/*!< Imports a new settings structure to the sensor */
	SBG_ECOM_CMD_EXPORT_SETTINGS 			= 3,		/*!< Export the whole configuration from the sensor */

	/* Device info */
	SBG_ECOM_CMD_INFO 						= 4,		/*!< Get basic device information */

	/* Sensor parameters */
	SBG_ECOM_CMD_INIT_PARAMETERS 			= 5,		/*!< Initial configuration */
	SBG_ECOM_CMD_SET_MOTION_PROFILE 		= 6,		/*!< Set a new motion profile */
	SBG_ECOM_CMD_MOTION_PROFILE_ID	 		= 7,		/*!< Get motion profile information */
	SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM	= 8,		/*!< Sensor alignment and lever arm on vehicle configuration */
	SBG_ECOM_CMD_AIDING_ASSIGNMENT 			= 9,		/*!< Aiding assignments such as RTCM / GPS / Odometer configuration */

	/* Magnetometer configuration */
	SBG_ECOM_CMD_MAGNETOMETER_SET_MODEL 	= 10,		/*!< Set a new magnetometer error model */
	SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID	 	= 11,		/*!< Get magnetometer error model information */
	SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE 	= 12,		/*!< Magnetometer aiding rejection mode */
	SBG_ECOM_CMD_SET_MAG_CALIB 				= 13,		/*!< Set magnetic soft and hard Iron calibration data */

	/* Magnetometer onboard calibration */
	SBG_ECOM_CMD_START_MAG_CALIB			= 14,		/*!< Start / reset internal magnetic field logging for calibration. */
	SBG_ECOM_CMD_COMPUTE_MAG_CALIB			= 15,		/*!< Compute a magnetic calibration based on previously logged data. */

	/* GNSS configuration */
	SBG_ECOM_CMD_GNSS_1_SET_MODEL 			= 16,		/*!< Set a new GNSS model */
	SBG_ECOM_CMD_GNSS_1_MODEL_ID 			= 17,		/*!< Get GNSS model information */
	SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT = 18,		/*!< GNSS installation configuration (lever arm, antenna alignments) */
	SBG_ECOM_CMD_GNSS_1_REJECT_MODES 		= 19,		/*!< GNSS aiding rejection modes configuration. */

	/* Odometer configuration */
	SBG_ECOM_CMD_ODO_CONF 					= 20,		/*!< Odometer gain, direction configuration */
	SBG_ECOM_CMD_ODO_LEVER_ARM 				= 21,		/*!< Odometer installation configuration (lever arm) */
	SBG_ECOM_CMD_ODO_REJECT_MODE 			= 22,		/*!< Odometer aiding rejection mode configuration. */

	/* Interfaces configuration */
	SBG_ECOM_CMD_UART_CONF 					= 23,		/*!< UART interfaces configuration */
	SBG_ECOM_CMD_CAN_BUS_CONF 				= 24,		/*!< CAN bus interface configuration */
	SBG_ECOM_CMD_CAN_OUTPUT_CONF			= 25,		/*!< CAN identifiers configuration */
		
	/* Events configuration */
	SBG_ECOM_CMD_SYNC_IN_CONF 				= 26,		/*!< Synchronization inputs configuration */
	SBG_ECOM_CMD_SYNC_OUT_CONF 				= 27,		/*!< Synchronization outputs configuration */

	/* Output configuration */
	SBG_ECOM_CMD_NMEA_TALKER_ID 			= 29,		/*!< NMEA talker ID configuration */
	SBG_ECOM_CMD_OUTPUT_CONF 				= 30,		/*!< Output configuration */
	SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF 	= 31,		/*!< Legacy serial output mode configuration */

	/* Avanced configuration */
	SBG_ECOM_CMD_ADVANCED_CONF 				= 32,		/*!< Advanced settings configuration */

	/* Features related commands */
	SBG_ECOM_CMD_FEATURES					= 33,		/*!< Retrieve device features */

	/* Licenses related commands */
	SBG_ECOM_CMD_LICENSE_APPLY				= 34,		/*!< Upload and apply a new license */
		
	/* Message class output switch */
	SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE		= 35,		/*!< Enable/disable the output of an entire class */

	/* Ethernet configuration */
	SBG_ECOM_CMD_ETHERNET_CONF				= 36,		/*!< Set/get Ethernet configuration such as DHCP mode and IP address. */
	SBG_ECOM_CMD_ETHERNET_INFO				= 37,		/*!< Return the current IP used by the device. */

	/* Misc. */
	SBG_ECOM_LOG_ECOM_NUM_CMDS							/*!< Helper definition to know the number of commands */
} SbgEComCmd;


#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED

