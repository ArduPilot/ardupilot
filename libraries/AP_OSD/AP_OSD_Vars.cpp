#include "AP_OSD_Vars.h"
#include <AP_Math.h>

AP_OSD_Vars::AP_OSD_Vars()
:_groundSpeed(0.0),
 _throttle(0),
 _altitude(0.0),
 _pitch(0),
 _roll(0),
 _homeDirection(0),
 _homeDistance(0),
 _flyMode(0),
 _startTime(0),
 _GPSSats(0),
 _GPSLongitude(0.0),
 _GPSLatitude(0.0),
 _GPSLongitudePrint(0.0),
 _GPSLatitudePrint(0.0),
 _BatteryVol(0.0),
 _BatteryCurrent(0.0),
 _BatteryPercent(0),
 _WPDirection(0),
 _WPDistance(0)
{
	_panSpeed_XY[0] = 1; 
	_panSpeed_XY[1] = 8;

	_panThrottle_XY[0] = 1;
	_panThrottle_XY[1] = 2;

	_panVehicleAlt_XY[0] = 22;
	_panVehicleAlt_XY[1] = 8;

	_panPitch_XY[0] = 8;
	_panPitch_XY[1] = 2;

	_panRoll_XY[0] = 14;
	_panRoll_XY[1] = 2;

	_panHomeDir_XY[0] = 22;
	_panHomeDir_XY[1] = 2;

	_panHomeDist_XY[0] = 22;
	_panHomeDist_XY[1] = 3;

	_panRSSI_XY[0] = 23;
	_panRSSI_XY[1] = 4;

	_panMode_XY[0] = 22;
	_panMode_XY[1] = 5;

	_panTime_XY[0] = 23;
	_panTime_XY[1] = 6;

	_panHorizon_XY[0] = 8;
	_panHorizon_XY[1] = 6;

	_panGPSSats_XY[0] = 1;
	_panGPSSats_XY[1] = 10;

	_panGPSCoord_XY[0] = 1;
	_panGPSCoord_XY[1] = 11;

	

	_panBatteryVol_XY[0] = 22;
	_panBatteryVol_XY[1] = 9;

	_panBatteryCurrent_XY[0] = 22;
	_panBatteryCurrent_XY[1] = 10;

	_panBatteryConsume_XY[0] = 19;
	_panBatteryConsume_XY[1] = 11;

	_panBatteryPercent_XY[0] = 22;
	_panBatteryPercent_XY[1] = 12;

	_panWPDir_XY[0] = 1;
	_panWPDir_XY[1] = 4;

	_panWPDist_XY[0] = 2;
	_panWPDist_XY[1] = 5;

	_heading = 0.0;
}

//void AP_OSD_Vars::update()
//{
//	_BatteryVol = battery.voltage();
//	_BatteryCurrent = battery.current_amps() * 100;
//	_BatteryPercent = battery.capacity_remaining_pct();
//
//	const Location &loc = gps.location(0);
//	_GPSSats = gps.num_sats(0);
//	_GPSLongitude = loc.lng;
//	_GPSLatitude = loc.lat;
//
//	_groundSpeed = gps.ground_speed();
//	//float heading = (ahrs.yaw_sensor / 100) % 360;
//	_throttle = g.rc_3.servo_out/10;
//	_altitude = current_loc.alt / 100.0f;
//	//float climbRate = climb_rate / 100.0f;
//
//	_pitch = ahrs.pitch;
//	_roll = ahrs.roll;
//	//int8_t yaw = ahrs.yaw;
//
//	_WPDirection = wp_bearing;
//	_WPDistance = wp_distance;
//	//uint8_t wayPointNum = mission.get_current_nav_index();
//
//	_flyMode = control_mode;
//}