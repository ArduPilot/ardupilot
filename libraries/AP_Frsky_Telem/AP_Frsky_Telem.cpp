/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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
   FRSKY Telemetry library
   for the moment it only handle hub port telemetry
   the sport reference are only here to simulate the frsky module and use opentx simulator. it will eventually be removed
*/

#include <AP_Frsky_Telem.h>

extern const AP_HAL::HAL& hal;

void AP_Frsky_Telem::init(AP_HAL::UARTDriver *port, uint8_t frsky_type)
{
    if (port == NULL) {
	return;
    }
    _port = port;    
    _port->begin(9600);
    _initialised = true;    
}

void AP_Frsky_Telem::frsky_send_byte(uint8_t value)
{
    const uint8_t x5E[] = { 0x5D, 0x3E };
    const uint8_t x5D[] = { 0x5D, 0x3D };
    switch (value) {
    case 0x5E:
	_port->write( x5E, sizeof(x5E));
	break;
	
    case 0x5D:
	_port->write( x5D, sizeof(x5D));
	break;
	
    default:
	_port->write(&value, sizeof(value));
	break;
    }
}


/**
 * Sends a 0x5E start/stop byte.
 */
void AP_Frsky_Telem::frsky_send_hub_startstop()
{
    static const uint8_t c = 0x5E;
    _port->write(&c, sizeof(c));
}

/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_Telem::frsky_send_data(uint8_t id, int16_t data)
{
    /* Cast data to unsigned, because signed shift might behave incorrectly */
    uint16_t udata = data;

    frsky_send_hub_startstop();
    frsky_send_byte(id);

    frsky_send_byte(udata); /* LSB */
    frsky_send_byte(udata >> 8); /* MSB */

}

/**
 * Sends frame 1 (every 200ms):
 *  barometer altitude,  battery voltage & current
 */
void AP_Frsky_Telem::frsky_send_frame1(uint8_t mode)
{
    struct Location loc;
    float battery_amps = _battery.current_amps();
    float baro_alt = 0; // in meters
    bool posok = _ahrs.get_position(loc);
    if  (posok) {
	baro_alt = loc.alt * 0.01f; // convert to meters
        if (!loc.flags.relative_alt) {
            baro_alt -= _ahrs.get_home().alt * 0.01f; // subtract home if set
        }
    }
    const AP_GPS &gps = _ahrs.get_gps();

    // GPS status is sent as num_sats*10 + status, to fit into a
    // uint8_t
    uint8_t T2 = gps.num_sats() * 10 + gps.status();
  
    frsky_send_data(FRSKY_ID_TEMP1, mode);
    frsky_send_data(FRSKY_ID_TEMP2, T2);

    /*
      Note that this isn't actually barometric altitdue, it is the
      AHRS estimate of altitdue above home.
     */
    uint16_t baro_alt_meters = (uint16_t)baro_alt;
    uint16_t baro_alt_cm = (baro_alt - baro_alt_meters) * 100;
    frsky_send_data(FRSKY_ID_BARO_ALT_BP, baro_alt_meters);
    frsky_send_data(FRSKY_ID_BARO_ALT_AP, baro_alt_cm);
  
    frsky_send_data(FRSKY_ID_FUEL, roundf(_battery.capacity_remaining_pct()));
 
    frsky_send_data(FRSKY_ID_VFAS, roundf(_battery.voltage() * 10.0f));
    frsky_send_data(FRSKY_ID_CURRENT, (battery_amps < 0) ? 0 : roundf(battery_amps * 10.0f));
  
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
float  AP_Frsky_Telem::frsky_format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/**
 * Sends frame 2 (every 1000ms):
 * course(heading), latitude, longitude, ground speed, GPS altitude
 */
void  AP_Frsky_Telem::frsky_send_frame2()
{

    // we send the heading based on the ahrs instead of GPS course which is not very usefull
    uint16_t course_in_degrees = (_ahrs.yaw_sensor / 100) % 360;
    frsky_send_data(FRSKY_ID_GPS_COURS_BP, course_in_degrees);

    const AP_GPS &gps = _ahrs.get_gps();
    bool posok = (gps.status() >= 3);
    if  (posok){
	// send formatted frame 
	float lat = 0, lon = 0,  alt = 0, speed= 0;
	char lat_ns = 0, lon_ew = 0;
	Location loc = gps.location();//get gps instance 0
	
	lat = frsky_format_gps(fabsf(loc.lat/10000000.0));
	uint16_t latdddmm = lat;
	uint16_t latmmmm = (lat - latdddmm) * 10000;
	lat_ns = (loc.lat < 0) ? 'S' : 'N';
	
	lon = frsky_format_gps(fabsf(loc.lng/10000000.0));
	uint16_t londddmm = lon;
	uint16_t lonmmmm = (lon - londddmm) * 10000;
	lon_ew = (loc.lng < 0) ? 'W' : 'E';

	alt = loc.alt / 100;
	uint16_t alt_gps_meters = alt;
	uint16_t alt_gps_cm = (alt - alt_gps_meters) * 100;

	speed = gps.ground_speed ();
	uint16_t speed_in_meter = speed;
	uint16_t speed_in_centimeter = (speed - speed_in_meter) * 100;
  
  
	frsky_send_data(FRSKY_ID_GPS_LAT_BP, latdddmm);
	frsky_send_data(FRSKY_ID_GPS_LAT_AP, latmmmm);
	frsky_send_data(FRSKY_ID_GPS_LAT_NS, lat_ns);

	frsky_send_data(FRSKY_ID_GPS_LONG_BP, londddmm);
	frsky_send_data(FRSKY_ID_GPS_LONG_AP, lonmmmm);
	frsky_send_data(FRSKY_ID_GPS_LONG_EW, lon_ew);
  
	frsky_send_data(FRSKY_ID_GPS_SPEED_BP, speed_in_meter);
	frsky_send_data(FRSKY_ID_GPS_SPEED_AP, speed_in_centimeter);
  
	frsky_send_data(FRSKY_ID_GPS_ALT_BP, alt_gps_meters);
	frsky_send_data(FRSKY_ID_GPS_ALT_AP, alt_gps_cm);
  
    }
}


/*
  check for input bytes from sport
 */
void AP_Frsky_Telem::check_sport_input(void)
{
    
}


/*
  send telemetry frames. Should be called at 50Hz. The high rate is to
  allow this code to poll for serial bytes coming from the receiver
  for the SPort protocol
 */
void AP_Frsky_Telem::send_frames(uint8_t control_mode, enum FrSkyProtocol protocol)
{
    if (!_initialised) {
        return;
    }
    
    if (protocol == FrSkySPORT) {
        // check for sport bytes
        check_sport_input();
    }
    
    uint32_t now = hal.scheduler->millis();

    // send frame1 every 200ms
    if (now - _last_frame1_ms > 200) {
        _last_frame1_ms = now;
        frsky_send_frame1(control_mode);        
    }

    // send frame2 every second
    if (now - _last_frame2_ms > 1000) {
        _last_frame2_ms = now;
        frsky_send_frame2();
    }
}
