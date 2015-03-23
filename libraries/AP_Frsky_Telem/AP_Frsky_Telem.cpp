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

#include <AP_Frsky_Telem.h>

extern const AP_HAL::HAL& hal;
extern uint8_t textId, g_severity;

void AP_Frsky_Telem::init (AP_HAL::UARTDriver *port, uint8_t frsky_type)
{
    if (port == NULL) {
        return;
    }

    _port = port;

    if (frsky_type == FrSkyDPORT) {
        _port->begin (9600);
    } else {
        _port->begin (57600);
        ignore_rx_chars = 0;
    }
    _initialised = true;
}

// Send one byte and do byte stuffing (D-Port receivers only)
void AP_Frsky_Telem::frsky_send_byte (uint8_t b)
{
    switch (b) {
        case 0x5D:
        case 0x5E:
            _port->write (    0x5D);
            _port->write (b ^ 0x60);
            break;
        default:
            _port->write (b);
            break;
    }
}

// Send one data id/value pair (D-Port receivers only)
void AP_Frsky_Telem::frsky_send_data(uint8_t id, int16_t data)
{
    // Cast data to unsigned, because signed shift might behave incorrectly
    uint16_t udata = data;

    _port->write (0x5E);            // Send a 0x5E start/stop byte
    frsky_send_byte (id);
    frsky_send_byte (udata);        // LSB
    frsky_send_byte (udata >> 8);   // MSB
}

// Send one byte, do byte stuffing and crc calculation (S-Port receivers only)
void AP_Frsky_Telem::frsky_send_char (uint8_t b)
{
    switch (b) {
        case 0x7D:
        case 0x7E:
            _port->write (    0x7D);
            _port->write (b ^ 0x20);
            ignore_rx_chars++;
            break;
        default:
            _port->write (b);
            break;
    }

    ignore_rx_chars++;

    crc += b;                    // checksum calculation is based on data before byte-stuffing
    crc += (crc >> 8);
    crc &= 0xFF;
}

// Send one data id/value pair (S-Port receivers only)
void AP_Frsky_Telem::frsky_send_value (uint16_t valueid, uint32_t value)
{
    crc = 0;
    frsky_send_char (FRSKY_DATA_FRAME);
    frsky_send_char ((valueid >>  0) & 0xFF);
    frsky_send_char ((valueid >>  8) & 0xFF);
    frsky_send_char ((value   >>  0) & 0xFF);
    frsky_send_char ((value   >>  8) & 0xFF);
    frsky_send_char ((value   >> 16) & 0xFF);
    frsky_send_char ((value   >> 24) & 0xFF);
    frsky_send_char (0xFF - crc);
}

// Send frame 1 (every 200ms): barometer altitude, battery voltage + current (D-Port receivers only)
void AP_Frsky_Telem::frsky_send_frame1 (uint8_t mode)
{
    struct Location loc;
    float battery_amps = _battery.current_amps();
    float baro_alt = 0; // in meters
    bool posok = _ahrs.get_position(loc);
    if (posok) {
        baro_alt = loc.alt * 0.01f; // convert to meters
        if (!loc.flags.relative_alt) {
            baro_alt -= _ahrs.get_home().alt * 0.01f; // subtract home if set
        }
    }
    const AP_GPS &gps = _ahrs.get_gps ();

    // GPS status is sent as num_sats*10 + status, to fit into a uint8_t
    uint8_t T2 = gps.num_sats() * 10 + gps.status();

    frsky_send_data (FRSKY_ID_TEMP1, mode);
    frsky_send_data (FRSKY_ID_TEMP2, T2);

    // Note that this isn't actually barometric altitdue, it is the AHRS estimate of altitdue above home
    uint16_t baro_alt_meters = (uint16_t)baro_alt;
    uint16_t baro_alt_cm = (baro_alt - baro_alt_meters) * 100;
    frsky_send_data (FRSKY_ID_BARO_ALT_BP, baro_alt_meters);
    frsky_send_data (FRSKY_ID_BARO_ALT_AP, baro_alt_cm);

    frsky_send_data (FRSKY_ID_FUEL, roundf(_battery.capacity_remaining_pct()));

    frsky_send_data (FRSKY_ID_VFAS, roundf(_battery.voltage() * 10.0f));
    frsky_send_data (FRSKY_ID_CURRENT, (battery_amps < 0) ? 0 : roundf(battery_amps * 10.0f));
}

// Format decimal latitude/longitude to the required degrees/minutes (D-Port receivers only)
float AP_Frsky_Telem::frsky_format_gps (float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

// Send frame 2 (every 1000ms): course(heading), latitude, longitude, ground speed, GPS altitude (D-Port receivers only)
void AP_Frsky_Telem::frsky_send_frame2 (void)
{
    // we send the heading based on the ahrs instead of GPS course which is not very usefull
    uint16_t course_in_degrees = (_ahrs.yaw_sensor / 100) % 360;
    frsky_send_data (FRSKY_ID_GPS_COURS_BP, course_in_degrees);

    const AP_GPS &gps = _ahrs.get_gps ();
    bool posok = (gps.status() >= 3);
    if  (posok) {
        // send formatted frame
        float lat = 0, lon = 0,  alt = 0, speed= 0;
        char lat_ns = 0, lon_ew = 0;
        Location loc = gps.location();//get gps instance 0

        lat = frsky_format_gps (fabsf(loc.lat/10000000.0));
        uint16_t latdddmm = lat;
        uint16_t latmmmm = (lat - latdddmm) * 10000;
        lat_ns = (loc.lat < 0) ? 'S' : 'N';

        lon = frsky_format_gps (fabsf(loc.lng/10000000.0));
        uint16_t londddmm = lon;
        uint16_t lonmmmm = (lon - londddmm) * 10000;
        lon_ew = (loc.lng < 0) ? 'W' : 'E';

        alt = loc.alt / 100;
        uint16_t alt_gps_meters = alt;
        uint16_t alt_gps_cm = (alt - alt_gps_meters) * 100;

        speed = gps.ground_speed ();
        uint16_t speed_in_meter = speed;
        uint16_t speed_in_centimeter = (speed - speed_in_meter) * 100;

        frsky_send_data (FRSKY_ID_GPS_LAT_BP, latdddmm);
        frsky_send_data (FRSKY_ID_GPS_LAT_AP, latmmmm);
        frsky_send_data (FRSKY_ID_GPS_LAT_NS, lat_ns);

        frsky_send_data (FRSKY_ID_GPS_LONG_BP, londddmm);
        frsky_send_data (FRSKY_ID_GPS_LONG_AP, lonmmmm);
        frsky_send_data (FRSKY_ID_GPS_LONG_EW, lon_ew);

        frsky_send_data (FRSKY_ID_GPS_SPEED_BP, speed_in_meter);
        frsky_send_data (FRSKY_ID_GPS_SPEED_AP, speed_in_centimeter);

        frsky_send_data (FRSKY_ID_GPS_ALT_BP, alt_gps_meters);
        frsky_send_data (FRSKY_ID_GPS_ALT_AP, alt_gps_cm);
    }
}

// Check for input bytes from S.Port and answer accordingly
void AP_Frsky_Telem::check_sport_input(uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle)
{
    static uint8_t state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
    static uint8_t state_sensor = 0;
    uint16_t hdop;

    while (_port->available()) {
        uint8_t b = _port->read ();
        switch (state_serial) {
            case FRSKY_STATE_WAIT_TX_DONE:             // because RX and TX are connected, we need to ignore bytes transmitted by us
                if (--ignore_rx_chars == 0)
                    state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
                break;
            case FRSKY_STATE_WAIT_POLL_REQUEST:        // we get polled ~ every 12 ms (83 Hz)
                if (b == FRSKY_POLL_REQUEST)
                    state_serial = FRSKY_STATE_WAIT_SENSOR_ID;
                break;
            case FRSKY_STATE_WAIT_SENSOR_ID:
                if (b == FRSKY_SENSOR_ID) {
                    state_serial = FRSKY_STATE_WAIT_TX_DONE;
                    switch (state_sensor++) {          // send one value every second poll (24 ms)
                        case 0:     // Vfas (Battery voltage [V])
                            frsky_send_value (FRSKY_ID2_VOLTAGE, _battery.voltage()*100);
                            break;
                        case 1:     // Curr (Battery current [A])
                            frsky_send_value (FRSKY_ID2_CURRENT, _battery.current_amps()*10);
                            break;
                        case 2:     // Fuel (Flightmode as defined in defines.h + Simple mode + Armed/Disarmed)
                            frsky_send_value (FRSKY_ID2_FUEL, (control_mode<<3) | (simple_mode<<1) | _ahrs.get_armed());
                            break;
                        case 3:     // Hdg (Heading, [°])
                            frsky_send_value (FRSKY_ID2_HEADING, _ahrs.yaw_sensor);
                            break;
                        case 4:     // AccX (HDOP)
                            hdop = _ahrs.get_gps().get_hdop() / 4;
							if (hdop > 255) {
								hdop = 255;
							}
                            frsky_send_value (FRSKY_ID2_ACCX, hdop*100);
                            break;
                        case 5:     // AccY (Roll angle [°])
							frsky_send_value (FRSKY_ID2_ACCY, _ahrs.roll_sensor);
                            break;
                        case 6:     // AccZ (Pitch angle [°])
							frsky_send_value (FRSKY_ID2_ACCZ, _ahrs.pitch_sensor);
                            break;
                        case 7:     // RPM (Throttle [%] + Altitude mode, = ^v)
                            frsky_send_value (FRSKY_ID2_RPM, (throttle<<2) | alt_mode);
                            break;
                        case 8:     // VSpd (Vertical speed [m/s])
                            frsky_send_value (FRSKY_ID2_VARIO, (int32_t) climbrate);
                            break;
                        case 9:     // Alt (Baro height above ground [m])
                            if (_ahrs.get_armed()==true) {
                                frsky_send_value (FRSKY_ID2_ALTITUDE, altitude);
                            } else {
                                frsky_send_value (FRSKY_ID2_ALTITUDE, 0);
                            }
                            break;
                        case 10:    // GAlt (GPS height above ground [m])
                            if (_ahrs.get_armed()==true && _ahrs.get_gps().status()>=3) {
                                frsky_send_value (FRSKY_ID2_GPS_ALT, _ahrs.get_gps().location().alt);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_ALT, 0);
                            }
                            break;
                        case 11:    // GPS Latitude
                            if (_ahrs.get_armed()==true && _ahrs.get_gps().status()>=3) {
                                uint32_t pos;
                                if (_ahrs.get_gps().location().lat > 0) {
                                    pos = 0;                // north latitude: id 00
                                } else {
                                    pos = 1 << 30;          // south latitude: id 01
                                }
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, pos | abs(_ahrs.get_gps().location().lat)/100*6);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, 0);
                            }
                            break;
                        case 12:    // GPS Longitude
                            if (_ahrs.get_armed()==true && _ahrs.get_gps().status()>=3) {
                                uint32_t pos;
                                if (_ahrs.get_gps().location().lng > 0) {
                                    pos = 2 << 30;          // east longitude: id 10
                                } else {
                                    pos = 3 << 30;          // west longitude: id 11
                                }
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, pos | abs(_ahrs.get_gps().location().lng)/100*6);
                            } else {
                                frsky_send_value (FRSKY_ID2_GPS_LON_LAT, 0);
                            }
                            break;
                        case 13:    // Spd (Ground speed [km/h])
                            if (_ahrs.get_armed()==true && _ahrs.get_gps().status()>=3) {
                                frsky_send_value (FRSKY_ID2_SPEED, _ahrs.get_gps().ground_speed()*543*3.6);
                            } else {
                                frsky_send_value (FRSKY_ID2_SPEED, 0);
                            }
                            break;
                        case 14:    // T1 (Number of satellites used for navigation + Lock Mode as defined in AP_GPS.h)
                            frsky_send_value (FRSKY_ID2_T1, ((_ahrs.get_gps().num_sats()&0x1F)<<3) | (_ahrs.get_gps().status()&0x07));
                            break;
                        case 15:    // T2 (Message ID + severity as defined in GCS_MAVLink.h)
							// text messages are sent multiple times to make sure they really arrive (even on radio glitches)
							#define TX_REPEATS 4
                            static uint8_t  tx_cnt=TX_REPEATS;
                            static uint32_t val=0;
                            if (tx_cnt > 0 && textId > 0) {
								if (tx_cnt == TX_REPEATS) {
                                	val = (textId<<3) | (g_severity&0x07);
                                }
                                tx_cnt--;
                            }
                            frsky_send_value (FRSKY_ID2_T2, val);
                            if (tx_cnt == 0) {
                               	textId     = 0;
                               	g_severity = 0;
                               	val        = 0;
                               	tx_cnt     = TX_REPEATS;
                            }
                            state_sensor = 0;			// Reset state counter on last state
                            break;
                    }
                } else {
                    state_serial = FRSKY_STATE_WAIT_POLL_REQUEST;
                }
                break;
            }
        }
}

/*
  Send telemetry frames. Should be called at 100Hz. The high rate is to
  allow this code to poll for serial bytes coming from the receiver
  for the SPort protocol which is polling us every ~ 12ms
  climbrate in cm/s based on filtered data - positive = up
 */
void AP_Frsky_Telem::send_frames (uint8_t control_mode, uint8_t simple_mode, int32_t altitude, int8_t alt_mode, float climbrate, int16_t throttle, enum FrSkyProtocol protocol)
{
    if (!_initialised) {
        return;
    }

    if (protocol == FrSkySPORT) {
        // check for sport bytes
        check_sport_input (control_mode, simple_mode, altitude, alt_mode, climbrate, throttle);
    } else {
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
}
