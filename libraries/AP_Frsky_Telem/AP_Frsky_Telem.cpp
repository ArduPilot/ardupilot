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
#include "AP_Frsky_Telem.h"
extern const AP_HAL::HAL& hal;

//constructor
AP_Frsky_Telem::AP_Frsky_Telem(AP_AHRS &ahrs, const AP_BattMonitor &battery) :
    _ahrs(ahrs),
    _battery(battery),
    _port(NULL),
    _protocol(),
    _crc(0),
    _control_mode(),
    _gps()
    {}

/*
 * init - perform required initialisation
 */
void AP_Frsky_Telem::init(const AP_SerialManager &serial_manager, uint8_t *control_mode)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_D; // FrSky D protocol (D-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort; // FrSky SPort protocol (X-receivers)
    }

    _control_mode = control_mode;

    if (_port != NULL) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort(void)
{
    // variables keeping track of which data to send
    static uint8_t fas_call = 1;
    static uint8_t gps_call = 1;
    static uint8_t vario_call = 1;
    static uint8_t various_call = 1;

    while (_port->available()) {
        uint8_t readbyte = _port->read();
        if (readbyte == START_STOP_SPORT) { // byte 0x7E is the header of each poll request
            readbyte = _port->read();
            switch(readbyte) {
                case SENSOR_ID_FAS:
                    switch (fas_call) {
                        case 1:
                            send_data(DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                            break;
                        case 2:
                            send_data(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                            break;
                        case 3:
                            send_data(DATA_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
                            break;
                    }
                    if (fas_call++ > 3) fas_call = 0;
                    break;
                case SENSOR_ID_GPS:
                    switch (gps_call) {
                        case 1:
                            calc_gps_position(); // gps data is not recalculated until all of it has been sent
                            send_data(DATA_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
                            break;
                        case 2:
                            send_data(DATA_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
                            break;
                        case 3:
                            send_data(DATA_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
                            break;
                        case 4:
                            send_data(DATA_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
                            break;
                        case 5:
                            send_data(DATA_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
                            break;
                        case 6:
                            send_data(DATA_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
                            break;
                        case 7:
                            send_data(DATA_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
                            break;
                        case 8:
                            send_data(DATA_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
                            break;
                        case 9:
                            send_data(DATA_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
                            break;
                        case 10:
                            send_data(DATA_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimals
                            break;
                        case 11:
                            send_data(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
                            break;
                    }
                    if (gps_call++ > 11) gps_call = 1;
                    break;
                case SENSOR_ID_VARIO:
                    switch (vario_call) {
                        case 1 :
                            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
                            send_data(DATA_ID_BARO_ALT_BP, _gps.alt_nav_meters); // send altitude integer part
                            break;
                        case 2:
                            send_data(DATA_ID_BARO_ALT_AP, _gps.alt_nav_cm); // send altitude decimal part
                            break;
                        }
                    if (vario_call++ > 2) vario_call = 1;
                    break;    
                case SENSOR_ID_SP2UR:
                    switch (various_call) {
                        case 1 :
                            send_data(DATA_ID_TEMP2, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                            break;
                        case 2:
                            send_data(DATA_ID_TEMP1, (uint16_t)*_control_mode); // send flight mode
                            break;
                    }
                    if (various_call++ > 2) various_call = 1;
                    break;
            }
        }
    }
}

/*
 * send frame1 and frame2 telemetry data
 * one frame (frame1) is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 * a second frame (frame2) is sent every second (1000ms) with gps position data, and ahrs.yaw_sensor heading (instead of GPS heading)
 * for FrSky D protocol (D-receivers)
 */
void AP_Frsky_Telem::send_D(void)
{
    static uint32_t last_200ms_frame = 0;
    static uint32_t last_1000ms_frame = 0;

    uint32_t now = AP_HAL::millis();
    // send frame1 every 200ms
    if (now - last_200ms_frame > 200) {
        last_200ms_frame = now;
        send_data(DATA_ID_TEMP2, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        send_data(DATA_ID_TEMP1, (uint16_t)*_control_mode); // send flight mode
        send_data(DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
        send_data(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        send_data(DATA_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
        calc_nav_alt();
        send_data(DATA_ID_BARO_ALT_BP, _gps.alt_nav_meters); // send nav altitude integer part
        send_data(DATA_ID_BARO_ALT_AP, _gps.alt_nav_cm); // send nav altitude decimal part
    }
    // send frame2 every second
    if (now - last_1000ms_frame > 1000) {
        last_1000ms_frame = now;
        send_data(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (_ahrs.get_gps().status() >= 3) {
            send_data(DATA_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
            send_data(DATA_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
            send_data(DATA_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
            send_data(DATA_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
            send_data(DATA_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
            send_data(DATA_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
            send_data(DATA_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
            send_data(DATA_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
            send_data(DATA_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
            send_data(DATA_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimal part
        }
    }
}

/*
 * tick - main call to send data to the receiver (called by scheduler at 1kHz)
 */
void AP_Frsky_Telem::tick(void)
{
    static bool initialised_uart = false; // true when we have detected the protocol and UART has been initialised
    // check UART has been initialised
    if (!initialised_uart) {
        // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
        if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                    // FrSky D protocol (D-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_D_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        } else {                                                                        // FrSky SPort protocol (X-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        }
        initialised_uart = true;
    }

    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                        // FrSky D protocol (D-receivers)
        send_D();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort) {             // FrSky SPort protocol (X-receivers)
        send_SPort();
    }
}

/* 
 * build up the frame's crc
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_crc(void) 
{
    send_byte(0xFF - _crc);
    _crc = 0;
}


/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_Telem::send_byte(uint8_t byte)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) { // FrSky D protocol (D-receivers)
        if (byte == START_STOP_D) {
            _port->write(0x5D);
            _port->write(0x3E);
        } else if (byte == BYTESTUFF_D) {
            _port->write(0x5D);
            _port->write(0x3D);
        } else {
            _port->write(byte);
        }
    } else { // FrSky SPort protocol (X-receivers)
        if (byte == START_STOP_SPORT) {
            _port->write(0x7D);
            _port->write(0x5E);
        } else if (byte == BYTESTUFF_SPORT) {
            _port->write(0x7D);
            _port->write(0x5D);
        } else {
            _port->write(byte);
        }
        calc_crc(byte);
    }
}

/*
 * send one frame of FrSky data
 */
void  AP_Frsky_Telem::send_data(uint16_t id, uint32_t data)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) { // FrSky D protocol (D-receivers)
        _port->write(START_STOP_D);    // send a 0x5E start byte
        uint8_t *bytes = (uint8_t*)&id;
        send_byte(bytes[0]);
        bytes = (uint8_t*)&data;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]); // MSB
    } else { // FrSky SPort protocol (X-receivers)
        send_byte(0x10); // DATA_FRAME
        uint8_t *bytes = (uint8_t*)&id;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]); // MSB
        bytes = (uint8_t*)&data;
        send_byte(bytes[0]); // LSB
        send_byte(bytes[1]);
        send_byte(bytes[2]);
        send_byte(bytes[3]); // MSB
        send_crc();
    }
}

/*
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_nav_alt(void)
{
    Location current_loc;
    float current_height; // in centimeters above home
    if  (_ahrs.get_position(current_loc)) {
        if (current_loc.flags.relative_alt) {
            current_height = current_loc.alt*0.01f;
        } else {
            current_height = (current_loc.alt - _ahrs.get_home().alt)*0.01f;
        }
    } else {
        current_height = 0;
    }
    _gps.alt_nav_meters = (int16_t)current_height;
    _gps.alt_nav_cm = (current_height - _gps.alt_nav_meters) * 100;
} 

/*
 * format the decimal latitude/longitude to the required degrees/minutes
 * for FrSky D and SPort protocols
 */
float AP_Frsky_Telem::format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare gps data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_gps_position(void)
{
    float lat;
    float lon;
    float alt;
    float speed;

    if (_ahrs.get_gps().status() >= 3) {
        Location loc = _ahrs.get_gps().location(); //get gps instance 0
        lat = format_gps(fabsf(loc.lat/10000000.0f));
        _gps.latdddmm = lat;
        _gps.latmmmm = (lat - _gps.latdddmm) * 10000;
        _gps.lat_ns = (loc.lat < 0) ? 'S' : 'N';

        lon = format_gps(fabsf(loc.lng/10000000.0f));
        _gps.londddmm = lon;
        _gps.lonmmmm = (lon - _gps.londddmm) * 10000;
        _gps.lon_ew = (loc.lng < 0) ? 'W' : 'E';

        alt = loc.alt * 0.01f;
        _gps.alt_gps_meters = (int16_t)alt;
        _gps.alt_gps_cm = (alt - _gps.alt_gps_meters) * 100;

        speed = _ahrs.get_gps().ground_speed();
        _gps.speed_in_meter = speed;
        _gps.speed_in_centimeter = (speed - _gps.speed_in_meter) * 100;
    } else {
        _gps.latdddmm = 0;
        _gps.latmmmm = 0;
        _gps.lat_ns = 0;
        _gps.londddmm = 0;
        _gps.lonmmmm = 0;
        _gps.alt_gps_meters = 0;
        _gps.alt_gps_cm = 0;
        _gps.speed_in_meter = 0;
        _gps.speed_in_centimeter = 0;
    }
}
