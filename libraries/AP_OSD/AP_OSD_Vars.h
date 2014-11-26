#ifndef __AP_OSD_VARS_H__
#define __AP_OSD_VARS_H__

#include <AP_HAL.h>
class AP_OSD_Vars
{
public:
	AP_OSD_Vars();

	//pisition
	uint8_t		_panSpeed_XY[2];
	uint8_t		_panThrottle_XY[2];
	uint8_t		_panVehicleAlt_XY[2];
	uint8_t		_panPitch_XY[2];
	uint8_t		_panRoll_XY[2];
	uint8_t		_panHomeDir_XY[2];
	uint8_t		_panHomeDist_XY[2];
	uint8_t		_panMode_XY[2];
	uint8_t		_panTime_XY[2];
	uint8_t		_panHorizon_XY[2];
	uint8_t		_panGPSSats_XY[2];
	uint8_t		_panGPSCoord_XY[2];
	uint8_t		_panBatteryVol_XY[2];
	uint8_t		_panBatteryCurrent_XY[2];
	uint8_t		_panBatteryPercent_XY[2];
	uint8_t		_panBatteryConsume_XY[2];
	uint8_t		_panWPDir_XY[2];
	uint8_t		_panWPDist_XY[2];
	uint8_t		_panRSSI_XY[2];
	
	// screen display variable
	float		_groundSpeed;
	uint16_t	_throttle;
	float		_altitude;
	int8_t		_pitch;
	int8_t		_roll;
	int32_t		_homeDirection;	// Arrow direction pointing to home (1-16 to CW loop)
	int32_t		_homeDistance;
	uint8_t		_flyMode;
	float		_startTime;
	uint8_t		_GPSSats;
	float		_GPSLongitude;
	float		_GPSLatitude;
	float		_GPSLongitudePrint;
	float		_GPSLatitudePrint;
	float		_BatteryVol;
	float		_BatteryCurrent;
	uint8_t		_BatteryPercent;
	float		_BatteryConsum;
	int32_t		_WPDirection;
	int32_t		_WPDistance;
	float		_heading;
	int16_t		_rssiScale;
};

#endif //  __AP_OSD_VARS_H__