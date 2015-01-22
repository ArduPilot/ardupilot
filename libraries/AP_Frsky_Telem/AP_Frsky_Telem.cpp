// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
*/
#include <AP_Frsky_Telem.h>
extern const AP_HAL::HAL& hal;

void AP_Frsky_Telem::init(AP_HAL::UARTDriver *port, enum FrSkyProtocol protocol)
{
    if (port == NULL) {
        return;
    }
    _port = port;    
    _protocol = protocol;
    if (_protocol == FrSkySPORT) {
        _port->begin(57600);
        _gps_call = 0;
        _fas_call = 0;
        _vario_call = 0 ;
        _various_call = 0 ;
        _gps_data_ready = false;
        _battery_data_ready = false;
        _baro_data_ready = false;
        _mode_data_ready = false;
        _sats_data_ready = false;
        _sport_status = 0;
        hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&AP_Frsky_Telem::sport_tick));
    } else {
        // if this is D-port then spec says 9600 baud
        _port->begin(9600);
        _initialised = true;    
    }
}


/* 
   simple crc implementation for FRSKY telem S-PORT
*/
void AP_Frsky_Telem::calc_crc (uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0x00ff;
    _crc += _crc >> 8; //0-0FF
    _crc &= 0x00ff;
}

/*
 * send the crc at the end of the S-PORT frame
 */
void AP_Frsky_Telem::send_crc() 
{
    frsky_send_byte(0x00ff-_crc);
    _crc = 0;
}


/*
  send 1 byte and do the byte stuffing Frsky stuff 
  This can send more than 1 byte eventually
*/
void AP_Frsky_Telem::frsky_send_byte(uint8_t value)
{
    if (_protocol == FrSkyDPORT) {
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
    } else { 
        //SPORT
        calc_crc(value);
        const uint8_t x7E[] = { 0x7D, 0x5E };
        const uint8_t x7D[] = { 0x7D, 0x5D };
        switch (value) {
        case 0x7E:
            _port->write( x7E, sizeof(x7E));
            break;
            
        case 0x7D:
            _port->write( x7D, sizeof(x7D));
            break;
            
        default:
            _port->write(&value, sizeof(value));
            break;
        }
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

/*
  add sport protocol for frsky tx module 
*/
void AP_Frsky_Telem::frsky_send_sport_prim()
{
    static const uint8_t c = 0x10;
    frsky_send_byte(c);
}


/**
 * Sends one data id/value pair.
 */
void  AP_Frsky_Telem::frsky_send_data(uint8_t id, int16_t data)
{
    static const uint8_t zero = 0x0;

    /* Cast data to unsigned, because signed shift might behave incorrectly */
    uint16_t udata = data;

    if (_protocol == FrSkySPORT) {
        frsky_send_sport_prim();
        frsky_send_byte(id);
        frsky_send_byte(zero);
    } else {
        frsky_send_hub_startstop();
        frsky_send_byte(id);
    }

    frsky_send_byte(udata); /* LSB */
    frsky_send_byte(udata >> 8); /* MSB */

    if (_protocol == FrSkySPORT) {
        //Sport expect 32 bits data but we use only 16 bits data, so we send 0 for MSB
        frsky_send_byte(zero);
        frsky_send_byte(zero);
        send_crc();
    }
}

/*
 * frsky_send_baro_meters : send altitude in Meters based on ahrs estimate
 */
void AP_Frsky_Telem::calc_baro_alt()
{
    struct Location loc;
    float baro_alt = 0; // in meters
    bool posok = _ahrs.get_position(loc);
    if  (posok) {
        baro_alt = loc.alt * 0.01f; // convert to meters
        if (!loc.flags.relative_alt) {
            baro_alt -= _ahrs.get_home().alt * 0.01f; // subtract home if set
        }
    }
    /*
      Note that this isn't actually barometric altitude, it is the
      inertial nav estimate of altitude above home.
    */
    _baro_alt_meters = (int16_t)baro_alt;
    _baro_alt_cm = (baro_alt - abs(_baro_alt_meters)) * 100;
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
float  AP_Frsky_Telem::frsky_format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare lattitude and longitude information stored in member variables
 */

void AP_Frsky_Telem::calc_gps_position()
{
    _course_in_degrees = (_ahrs.yaw_sensor / 100) % 360;

    const AP_GPS &gps = _ahrs.get_gps();
    float lat;
    float lon ;
    float alt ;
    float speed;
    _pos_gps_ok = (gps.status() >= 3);
    if (_pos_gps_ok) {
        Location loc = gps.location();//get gps instance 0
    
        lat = frsky_format_gps(fabsf(loc.lat/10000000.0));
        _latdddmm = lat;
        _latmmmm = (lat - _latdddmm) * 10000;
        _lat_ns = (loc.lat < 0) ? 'S' : 'N';
        
        lon = frsky_format_gps(fabsf(loc.lng/10000000.0));
        _londddmm = lon;
        _lonmmmm = (lon - _londddmm) * 10000;
        _lon_ew = (loc.lng < 0) ? 'W' : 'E';
        
        alt = loc.alt * 0.01f;
        _alt_gps_meters = (int16_t)alt;
        _alt_gps_cm = (alt - abs(_alt_gps_meters)) * 100;
        
        speed = gps.ground_speed();
        _speed_in_meter = speed;
        _speed_in_centimeter = (speed - _speed_in_meter) * 100;
    } else {
        _latdddmm = 0;
        _latmmmm = 0;
        _lat_ns = 0;
        _londddmm = 0;
        _lonmmmm = 0;
        _alt_gps_meters = 0;
        _alt_gps_cm = 0;
        _speed_in_meter = 0;
        _speed_in_centimeter = 0;
    }
}

/*
 * prepare battery information stored in member variables
 */
void AP_Frsky_Telem::calc_battery ()
{
    _batt_remaining = roundf(_battery.capacity_remaining_pct());
    _batt_volts = roundf(_battery.voltage() * 10.0f);
    _batt_amps = roundf(_battery.current_amps() * 10.0f);
}

/*
 * prepare sats information stored in member variables
 */
void AP_Frsky_Telem::calc_gps_sats ()
{
    // GPS status is sent as num_sats*10 + status, to fit into a uint8_t
    const AP_GPS &gps = _ahrs.get_gps();
    gps_sats = gps.num_sats() * 10 + gps.status();
}

/*
 * send number of gps satellite and gps status eg: 73 means 7 satellite and 3d lock
 */
void AP_Frsky_Telem::send_gps_sats ()
{
    frsky_send_data(FRSKY_ID_TEMP2, gps_sats);
}

/*
 * send control_mode as Temperature 1 (TEMP1)
 */
void AP_Frsky_Telem::send_mode (void)
{
    frsky_send_data(FRSKY_ID_TEMP1, _mode);
}

/*
 * send barometer altitude integer part . Initialize baro altitude
 */
void AP_Frsky_Telem::send_baro_alt_m (void)
{
    frsky_send_data(FRSKY_ID_BARO_ALT_BP, _baro_alt_meters);
}

/*
 * send barometer altitude decimal part
 */
void AP_Frsky_Telem::send_baro_alt_cm (void)
{
    frsky_send_data(FRSKY_ID_BARO_ALT_AP, _baro_alt_cm);
}

/*
 * send battery remaining
 */
void AP_Frsky_Telem::send_batt_remain (void)
{
    frsky_send_data(FRSKY_ID_FUEL, _batt_remaining);
}

/*
 * send battery voltage 
 */
void AP_Frsky_Telem::send_batt_volts (void)
{
    frsky_send_data(FRSKY_ID_VFAS, _batt_volts);
}

/*
 * send current consumptiom 
 */
void AP_Frsky_Telem::send_current (void)
{
    frsky_send_data(FRSKY_ID_CURRENT, _batt_amps);
}

/*
 * send heading in degree based on AHRS and not GPS 
 */
void AP_Frsky_Telem::send_heading (void)
{
    frsky_send_data(FRSKY_ID_GPS_COURS_BP, _course_in_degrees);
}

/*
 * send gps lattitude degree and minute integer part; Initialize gps info
 */
void AP_Frsky_Telem::send_gps_lat_dd (void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_BP, _latdddmm);
}

/*
 * send gps lattitude minutes decimal part 
 */
void AP_Frsky_Telem::send_gps_lat_mm (void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_AP, _latmmmm);
}

/*
 * send gps North / South information 
 */
void AP_Frsky_Telem::send_gps_lat_ns (void)
{
    frsky_send_data(FRSKY_ID_GPS_LAT_NS, _lat_ns);
}

/*
 * send gps longitude degree and minute integer part 
 */
void AP_Frsky_Telem::send_gps_lon_dd (void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_BP, _londddmm);
}

/*
 * send gps longitude minutes decimal part 
 */
void AP_Frsky_Telem::send_gps_lon_mm (void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_AP, _lonmmmm);
}

/*
 * send gps East / West information 
 */
void AP_Frsky_Telem::send_gps_lon_ew (void)
{
    frsky_send_data(FRSKY_ID_GPS_LONG_EW, _lon_ew);
}

/*
 * send gps speed integer part
 */
void AP_Frsky_Telem::send_gps_speed_meter (void)
{
    frsky_send_data(FRSKY_ID_GPS_SPEED_BP, _speed_in_meter);
}

/*
 * send gps speed decimal part
 */
void AP_Frsky_Telem::send_gps_speed_cm (void)
{
    frsky_send_data(FRSKY_ID_GPS_SPEED_AP, _speed_in_centimeter);
}

/*
 * send gps altitude integer part
 */
void AP_Frsky_Telem::send_gps_alt_meter (void)
{
    frsky_send_data(FRSKY_ID_GPS_ALT_BP, _alt_gps_meters);
}

/*
 * send gps altitude decimals
 */
void AP_Frsky_Telem::send_gps_alt_cm (void)
{
    frsky_send_data(FRSKY_ID_GPS_ALT_AP, _alt_gps_cm);
}

/*
 * send frame 1 every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 */

void AP_Frsky_Telem::send_hub_frame()
{
    uint32_t now = hal.scheduler->millis();

    // send frame1 every 200ms
    if (now - _last_frame1_ms > 200) {
        _last_frame1_ms = now;
        calc_gps_sats();
        send_gps_sats();
        send_mode();
        
        calc_battery();
        send_batt_remain();
        send_batt_volts();
        send_current();
        
        calc_baro_alt();
        send_baro_alt_m();
        send_baro_alt_cm();
    }
    // send frame2 every second
    if (now - _last_frame2_ms > 1000) {
        _last_frame2_ms = now;
        send_heading();
        calc_gps_position();
        if (_pos_gps_ok) {
            send_gps_lat_dd();
            send_gps_lat_mm();
            send_gps_lat_ns();
            send_gps_lon_dd();
            send_gps_lon_mm();
            send_gps_lon_ew();
            send_gps_speed_meter();
            send_gps_speed_cm();
            send_gps_alt_meter();
            send_gps_alt_cm();
        }
    }
}


/*
  send telemetry frames. Should be called at 50Hz. The high rate is to
  allow this code to poll for serial bytes coming from the receiver
  for the SPort protocol
*/
void AP_Frsky_Telem::send_frames(uint8_t control_mode)
{
    if (!_initialised) {
        return;
    }
    
    if (_protocol == FrSkySPORT) {
        if (!_mode_data_ready) {
            _mode=control_mode;
            _mode_data_ready = true;
        }
        if (!_baro_data_ready) { 
            calc_baro_alt();
            _baro_data_ready = true;
        }
        if (!_gps_data_ready) { 
            calc_gps_position();
            _gps_data_ready = true;
        }
        if (!_sats_data_ready) {
            calc_gps_sats();
            _sats_data_ready = true;
        }
        if (!_battery_data_ready) {
            calc_battery();
            _battery_data_ready = true;
        }
    } else {
        _mode=control_mode;
        send_hub_frame();
    }
}


void AP_Frsky_Telem::sport_tick(void)
{
    if (!_initialised) {
        _port->begin(57600);
        _initialised = true;
    }

    int16_t numc;
    numc = _port->available();

    // check if available is negative
    if (numc < 0) {
        return;
    }
    
    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }
    
    for (int16_t i = 0; i < numc; i++) {  
        int16_t readbyte = _port->read();
        if (_sport_status == 0) {
            if  (readbyte == SPORT_START_FRAME) {
                _sport_status = 1;
            } 
        } else {
            switch (readbyte) {
            case DATA_ID_FAS: 
                if (_battery_data_ready) {
                    switch (_fas_call) {
                    case 0:
                        send_batt_volts();
                        break;
                    case 1:
                        send_current();
                        break;
                    }
                    _fas_call++;
                    if (_fas_call > 1) {
                        _fas_call = 0;
                    }
                    _battery_data_ready = false;
                }
                break;
            case DATA_ID_GPS:
                if (_gps_data_ready) {
                    switch (_gps_call) {
                    case 0:
                        send_gps_lat_dd();
                        break;
                    case 1:
                        send_gps_lat_mm();
                        break;
                    case 2:
                        send_gps_lat_ns();
                        break;
                    case 3:
                        send_gps_lon_dd();
                        break;
                    case 4:
                        send_gps_lon_mm();
                        break;
                    case 5:
                        send_gps_lon_ew();
                        break;
                    case 6:
                        send_gps_speed_meter();
                        break;
                    case 7:
                        send_gps_speed_cm();
                        break;
                    case 8:
                        send_gps_alt_meter();
                        break;
                    case 9:
                        send_gps_alt_cm();
                        break;
                    case 10:
                        send_heading();
                        break;
                    }
               
                    _gps_call++;
                    if (_gps_call > 10) {
                        _gps_call = 0;
                        _gps_data_ready = false;
                    }
                }
                break;
            case DATA_ID_VARIO:
                if (_baro_data_ready) {
                    switch (_vario_call) {
                    case 0 :
                        send_baro_alt_m();
                        break;
                    case 1:
                        send_baro_alt_cm();
                        break;
                    }
                    _vario_call ++;
                    if (_vario_call > 1) {
                        _vario_call = 0;
                        _baro_data_ready = false;
                    }
                }
                break;
            case DATA_ID_SP2UR:
                switch (_various_call) {
                case 0 :
                    if ( _sats_data_ready ) {
                        send_gps_sats();
                        _sats_data_ready = false;
                    }
                    break;
                case 1:
                    if ( _mode_data_ready ) {
                        send_mode();
                        _mode_data_ready = false;
                    }
                    break;
                }
                _various_call++;
                if (_various_call > 1) {
                    _various_call = 0;
                }
                break;
            }
            _sport_status = 0;
        }
    }
}
