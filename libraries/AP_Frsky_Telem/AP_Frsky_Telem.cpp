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

#include <stdio.h> // for use of sprintf

#include "AP_Frsky_Telem.h"
extern const AP_HAL::HAL& hal;

//constructor
AP_Frsky_Telem::AP_Frsky_Telem(const AP_InertialNav &inav, const AP_AHRS &ahrs, const AP_BattMonitor &battery, const RangeFinder &rng) :
    _inav(inav),
    _ahrs(ahrs),
    _rng(rng),
    _battery(battery),

    _port(NULL),
    _protocol(),
    _crc(0),

    _params(),
    _mav(),

    _control_sensors_present(0),
    _control_sensors_enabled(0),
    _control_sensors_health(0),

    _home_distance(0),
    _home_bearing(0),
    _control_mode(0),
    _simple_mode(0),
    _land_complete(true),
    _gps()
    {}

/*
 * init - perform required initialisation
 */
void AP_Frsky_Telem::init(const AP_SerialManager &serial_manager, const uint8_t mav_type, const char *firmware_str, const char *frame_config_str, float fs_batt_voltage, float fs_batt_mah)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_DPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_DPort; // protocol for D-receivers
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_SPort; // legacy Smart Port protocol (for X-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_XPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_XPort; // new Smart Port protocol (for X-receivers)
        // add firmware and frame info to message queue
        char buf[50];
		sprintf(buf, "%s %s", firmware_str, frame_config_str);
        queue_message(buf);
		// save main parameters locally
        _params.mav_type = mav_type; // frame type (see MAV_TYPE in Mavlink definition file common.h)
		_params.fs_batt_voltage = fs_batt_voltage; // failsafe battery voltage in volts
		_params.fs_batt_mah = fs_batt_mah; // failsafe reserve capacity in mAh
		_params.batt_capacity_mah = _battery.pack_capacity_mah(); // battery capacity in mAh as configured by user
    }

    if (_port != NULL) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
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
        if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) {     // for DPort protocol
            _port->begin(AP_SERIALMANAGER_FRSKY_DPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        } else {                                                            // for SPort and XPort protocols
            _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        }
        initialised_uart = true;
    }

    if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) {     // for DPort protocol
        send_DPort();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FRSky_SPort) { // for SPort protocol
        send_SPort();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FRSky_XPort) { // for XPort protocol
        send_XPort();
    }
}

/*
 * send data
 * (for XPort protocol)
 */
void AP_Frsky_Telem::send_XPort(void)
{
    static bool params_sent = false; // set to true once we are done sending the AP parameters
	static bool chunk_sent = true; // set to false whenever we are not done sending a chunk of mavlink message
    static bool send_attitude = false;
    static bool send_latitude = false;
    static uint32_t timer_gps_latlng = 0;
    static uint32_t timer_gps_status = 0;
    static uint32_t timer_batt = 0;
    static uint32_t timer_status = 0;
    static uint32_t timer_home = 0;
    static uint32_t timer_velandrng = 0;

    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    static uint8_t new_byte = 0;
    while (_port->available()) { 
        prev_byte = new_byte;
        new_byte = _port->read();
    }

    if ((prev_byte == 0x7E) && (new_byte == SENSOR_28)) { // byte 0x7E is the header of each poll request
        if (!params_sent) { // have an initialisation phase where some AP parameters are sent
            frsky_send_data(0x1007, calc_params(&params_sent));
        } else { // once initialisation is complete, send data periodically
            if (send_attitude) { // skip other data, send attitude only this iteration
                send_attitude = false; // next iteration, check if we should send something other than attitude
            } else { // check if there's other data to send than attitude
                send_attitude = true; // next iteration, send attitude b/c it needs frequent updates to remain smooth
                // build mavlink message queue - this must run each time to keep monitoring new statustext messages as well as control_sensors and ekf_status flag changes
                mavlink_statustext_check();
                control_sensors_check();
                ekf_status_check();
                // if there's any message in the queue, start sending them chunk by chunk; three times each chunk
                if ((_mav.msg_sent_index != _mav.msg_queued_index) || (!chunk_sent)) {
					frsky_send_data(0x1000, get_next_msg_chunk(&chunk_sent));
                    return;
                }
                // send other sensor data if it's time for them, and reset the corresponding timer if sent
                uint32_t now = hal.scheduler->millis();
		    	// if it's time, send GPS longitude then latitude shortly after
                if ((now - timer_gps_latlng) > 1000) { // if we have at least a 2D fix
                    frsky_send_data(0x0800, calc_gps_latlng(&send_latitude));
                    if (!send_latitude) { // we've cycled and sent one each of longitude then latitude, so reset the timer
                        timer_gps_latlng = hal.scheduler->millis();
                    }
                    return;
                } else if ((now - timer_gps_status) > 1000) {
                    frsky_send_data(0x1002, calc_gps_status());
                    timer_gps_status = hal.scheduler->millis();
                    return;
                } else if ((now - timer_batt) > 1000) {
                    frsky_send_data(0x1003, calc_batt());
                    timer_batt = hal.scheduler->millis();
                    return;
                } else if ((now - timer_status) > 500) {
                    frsky_send_data(0x1001, calc_status());
                    timer_status = hal.scheduler->millis();
                    return;
                } else if ((now - timer_home) > 500) {
                    frsky_send_data(0x1004, calc_home());
                    timer_home = hal.scheduler->millis();
                    return;
                } else if ((now - timer_velandrng) > 500) {
                    frsky_send_data(0x1005, calc_velandrng());
                    timer_velandrng = hal.scheduler->millis();
                    return;
                }
            }
            // if nothing else needed to be sent, send attitude data
            frsky_send_data(0x1006, calc_attitude());
        }
    }
}

/*
 * grabs one byte of the mavlink statustext message to be transmitted, returns 0 if the end of the message is reached
 * (for XPort protocol)
 */
uint8_t AP_Frsky_Telem::get_next_msg_byte(void)
{
    static uint8_t index = 0;
    uint8_t character = _mav.message_data[_mav.msg_sent_index].text[index++];

    if (!character) { // we've reached the end of the message (string terminated by '\0')
        index = 0;
        _mav.msg_sent_index = (_mav.msg_sent_index + 1) % MSG_BUFFER_LENGTH;
    }
    return character;
}

/*
 * grabs one "chunk" (4 bytes) of the mavlink statustext message to be transmitted
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::get_next_msg_chunk(bool *chunk_sent)
{
    static uint32_t chunk = 0; // a "chunk" (four characters/bytes) at a time of the mavlink message to be sent
    static uint8_t repeats = 3; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
	
    if (repeats == 3) {
        chunk = 0;
        uint8_t character = get_next_msg_byte();
        if (character) {
            chunk |= character<<24;
            character = get_next_msg_byte();
            if (character) {
                chunk |= character<<16;
                character = get_next_msg_byte();
                if (character) {
                    chunk |= character<<8;
                    character = get_next_msg_byte();
                    if (character) {
                        chunk |= character;
                    }
                }
            }
        }
    }
    repeats--;
    if (repeats <= 0) {
        repeats = 3;
        *chunk_sent = true;
    }
    return chunk;
}

/*
 * add latest mavlink_statustext message to message cue, normally passed as statustext mavlink messages to the GCS, for transmission through FrSky link
 * should have the mavlink library fill the queue instead using queue_message(str)
 * (for XPort protocol)
 */
void AP_Frsky_Telem::mavlink_statustext_check(void)
{
    if (GCS_MAVLINK::gcs_message_flag) {
        GCS_MAVLINK::gcs_message_flag = 0;
        _mav.message_data[_mav.msg_queued_index] = GCS_MAVLINK::gcs_message_data;
        _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
    }
}

/*
 * add control_sensors information to message cue, normally passed as sys_status mavlink messages to the GCS, for transmission through FrSky link
 * (for XPort protocol)
 */
void AP_Frsky_Telem::control_sensors_check(void)
{
    static uint32_t control_sensors_timer = 0;
    uint32_t now = hal.scheduler->millis();
    if ((now - control_sensors_timer) > 5000) { // prevent repeating any system_status messages unless 5 seconds have passed
        uint32_t _control_sensors_flags = (_control_sensors_health ^ _control_sensors_enabled) & _control_sensors_present;
        // only one error is reported at a time (in order of preference). Same setup as Mission Planner.
        if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_GPS) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad GPS Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_GYRO) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad Gyro Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_ACCEL) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad Accel Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_MAG) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad Compass Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad Baro Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_LASER_POSITION) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad LiDAR Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad OptFlow Health");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_TERRAIN) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad or No Terrain Data");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_GEOFENCE) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Geofence Breach");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        } else if ((_control_sensors_flags & MAV_SYS_STATUS_AHRS) > 0) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Bad AHRS");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            control_sensors_timer = now;
        }
    }
}

/*
 * add ekf_status information to message cue, normally passed as ekf_status_report mavlink messages to the GCS, for transmission through FrSky link
 * (for XPort protocol)
 */
void AP_Frsky_Telem::ekf_status_check(void)
{
    static uint32_t ekf_status_timer = 0;
    mavlink_ekf_status_report_t ekf_status_report;
    _ahrs.get_ekf_status_report(ekf_status_report);

    uint32_t now = hal.scheduler->millis();
    if ((now - ekf_status_timer) > 10000) { // prevent repeating any ekf_status_report messages unless 10 seconds have passed
        // multiple errors can be reported at a time. Same setup as Mission Planner.
        if (ekf_status_report.velocity_variance >= 1) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Error velocity variance");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            ekf_status_timer = now;
        }

        if (ekf_status_report.pos_horiz_variance >= 1) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Error pos horiz variance");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            ekf_status_timer = now;
        }

        if (ekf_status_report.pos_vert_variance >= 1) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Error pos vert variance");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            ekf_status_timer = now;
        }

        if (ekf_status_report.compass_variance >= 1) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Error compass variance");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            ekf_status_timer = now;
        }

        if (ekf_status_report.terrain_alt_variance >= 1) {
            sprintf(_mav.message_data[_mav.msg_queued_index].text, "Error terrain alt variance");
            _mav.msg_queued_index = (_mav.msg_queued_index + 1) % MSG_BUFFER_LENGTH;
            ekf_status_timer = now;
        }
    }
}


/*
 * prepare parameter data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_params(bool *params_sent)
{
    static uint8_t paramID = 0;
    static uint8_t repeats = 3;
    uint32_t params = 0; 

    paramID++;
    switch(paramID) {
    case 1:
        params = _params.mav_type; // frame type (see MAV_TYPE in Mavlink definition file common.h)
        break;
    case 2:
        params = (uint32_t)roundf(_params.fs_batt_voltage * 100.0f); // battery failsafe voltage in centivolts
        break;
    case 3:
        params = (uint32_t)roundf(_params.fs_batt_mah); // battery failsafe capacity in mAh
        break;
    case 4:
        params = (uint32_t)roundf(_params.batt_capacity_mah); // battery pack capacity in mAh
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    params = (paramID << 24) | (params & 0xFFFFFF);

    if (paramID >= 4) {
        paramID = 0;
        repeats--;
        if (repeats <= 0) {
            *params_sent = true;
        }
	}

    return params;
}

/*
 * prepare gps latitude/longitude data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_gps_latlng(bool *send_latitude)
{
    const Location &loc = _ahrs.get_gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)
    uint32_t latlng = 0;

    // alternate between latitude and longitude
    if (*send_latitude == true) {
        if (loc.lat < 0) {
            latlng = ((abs(loc.lat)/100)*6) | 0x40000000;
        } else {
            latlng = ((abs(loc.lat)/100)*6);
        }
        *send_latitude = false;
    } else {
        if (loc.lng < 0) {
            latlng = ((abs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            latlng = ((abs(loc.lng)/100)*6) | 0x80000000;
        }
        *send_latitude = true;
    }
    return latlng;
}

/*
 * prepare gps status data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_gps_status(void)
{
    uint8_t status;
	uint8_t num_sats;
    uint32_t gps = 0;

    if (_ahrs.get_gps().status() > 3) {
        status = 3;
    } else {
        status = _ahrs.get_gps().status();
    }

    if (_ahrs.get_gps().num_sats() > 15) {
        num_sats = 15;
    } else {
        num_sats = _ahrs.get_gps().num_sats();
    }

    gps = (uint8_t)(num_sats & 0xF); // number of GPS satellites visible
    gps |= (uint8_t)(status & 0x3)<<4; // GPS receiver status (i.e., NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D >= 3)
    gps |= (uint16_t)(_ahrs.get_gps().get_hdop() & 0x3FFF)<<6; // GPS horizontal dilution of precision in cm
    return gps;
}

/*
 * prepare battery data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_batt(void)
{
    uint32_t batt = 0;

    batt = (((uint16_t)roundf(_battery.voltage() * 10.0f)) & 0x1FF); // battery voltage in decivolts
    batt |= (((uint16_t)roundf(_battery.current_amps() * 10.0f)) & 0x1FF)<<9; // battery current draw in deciamps
    batt |= (((uint8_t)roundf(_battery.capacity_remaining_pct())) & 0x7F)<<18; // battery percentage remaining
    return batt;
}

/*
 * prepare various status data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_status(void)
{
    uint32_t status = 0;

    status = (uint8_t)(_control_mode & 0x1F); // flight mode number
    status |= (uint8_t)(_simple_mode & 0x3)<<5; // simple/super simple modes flag
    status |= (uint8_t)(AP_Notify::flags.armed & 0x1)<<7; // armed flag
    status |= (uint8_t)(_land_complete & 0x1)<<8; // land complete flag
    status |= (uint8_t)(AP_Notify::flags.failsafe_battery & 0x1)<<9; // battery failsafe flag
    status |= (uint8_t)(AP_Notify::flags.ekf_bad & 0x1)<<10; // bad ekf flag
    return status;
}

/*
 * prepare home related data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_home(void)
{
    uint32_t home = 0;

    if ((_params.mav_type == MAV_TYPE_FIXED_WING)||(_params.mav_type == MAV_TYPE_GROUND_ROVER)) { // if plane or rover
        struct Location loc;
        float rel_alt = 0; // in centimeters above home
        bool posok = _ahrs.get_position(loc);
        if  (posok) {
            rel_alt = loc.alt;
            if (!loc.flags.relative_alt) {
                rel_alt -= _ahrs.get_home().alt; // subtract home if set
            }
        }
        home = ((uint16_t)rel_alt & 0x7FFF); // altitude between vehicle and home location in centimeters
    } else { // if copter
        home = ((uint16_t)_inav.get_altitude() & 0x7FFF); // altitude between vehicle and home location in centimeters
    }
    home |= (((uint8_t)roundf(_home_bearing * 0.00333f)) & 0x7F)<<15; // angle between vehicle and home location (relative to North) in 3 degree increments
    home |= (((uint16_t)roundf(_home_distance * 0.01f)) & 0x3FF)<<22; // distance between vehicle and home location in meters
    return home;
}

/*
 * prepare velocity and range data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_velandrng(void)
{
    uint32_t velandrng = 0;

    velandrng = (((int8_t)roundf(_inav.get_velocity_z() * 0.1f)) & 0xFF); // vertical velocity in dm/s
    velandrng |= (((uint8_t)roundf(_inav.get_velocity_xy() * 0.1f)) & 0xFF)<<8; // horizontal velocity in dm/s
    velandrng |= ((uint16_t)_rng.distance_cm() & 0xFFF)<<16; // rangefinder measurement in cm
    return velandrng;
}

/*
 * prepare attitude data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_attitude(void)
{
    uint32_t attitude = 0;

    attitude = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & 0x7FF); // roll from [-18000;18000] centidegrees to unsigned .2 degree increments
    attitude |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & 0x3FF)<<11; // pitch from [-18000;18000] centidegrees to unsigned .2 degree increments
    attitude |= ((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f) & 0x7FF)<<21; // yaw from [0;36000] centidegrees to .2 degree increments (already unsigned)
    return attitude;
}

/*
 * send data
 * (for SPort protocol)
 */
void AP_Frsky_Telem::send_SPort(void)
{
    // variables keeping track of which data to send
    // for SPort protocol
    static uint8_t fas_call = 1;
    static uint8_t gps_call = 1;
    static uint8_t vario_call = 1;
    static uint8_t various_call = 1;

    while (_port->available()) {
        uint8_t readbyte = _port->read();
        if (readbyte == 0x7E) { // byte 0x7E is the header of each poll request
            readbyte = _port->read();
            switch(readbyte) {
                case SENSOR_FAS:
                    switch (fas_call) {
                        case 1:
                            frsky_send_data(FRSKY_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                            break;
                        case 2:
                            frsky_send_data(FRSKY_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                            break;
                        case 3:
                            frsky_send_data(FRSKY_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
                            break;
                    }
                    if (fas_call++ > 3) fas_call = 0;
                    break;
                case SENSOR_GPS:
                    switch (gps_call) {
                        case 1:
                            calc_gps_position(); // gps data is not recalculated until all of it has been sent
                            frsky_send_data(FRSKY_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
                            break;
                        case 2:
                            frsky_send_data(FRSKY_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
                            break;
                        case 3:
                            frsky_send_data(FRSKY_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
                            break;
                        case 4:
                            frsky_send_data(FRSKY_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
                            break;
                        case 5:
                            frsky_send_data(FRSKY_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
                            break;
                        case 6:
                            frsky_send_data(FRSKY_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
                            break;
                        case 7:
                            frsky_send_data(FRSKY_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
                            break;
                        case 8:
                            frsky_send_data(FRSKY_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
                            break;
                        case 9:
                            frsky_send_data(FRSKY_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
                            break;
                        case 10:
                            frsky_send_data(FRSKY_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimals
                            break;
                        case 11:
                            frsky_send_data(FRSKY_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
                            break;
                    }
                    if (gps_call++ > 11) gps_call = 1;
                    break;
                case SENSOR_VARIO:
                    switch (vario_call) {
                        case 1 :
                            frsky_send_data(FRSKY_ID_BARO_ALT_BP, (uint16_t)roundf(_inav.get_altitude() * 0.01f)); // send altitude integer part
                            break;
                        case 2:
                            frsky_send_data(FRSKY_ID_BARO_ALT_AP, ((uint16_t)_inav.get_altitude())%100); // send altitude decimal part
                            break;
                        }
                    if (vario_call++ > 2) vario_call = 1;
                    break;    
                case SENSOR_SP2UR:
                    switch (various_call) {
                        case 1 :
                            frsky_send_data(FRSKY_ID_TEMP2, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                            break;
                        case 2:
                            frsky_send_data(FRSKY_ID_TEMP1, (uint16_t)_control_mode); // send flight mode
                            break;
                    }
                    if (various_call++ > 2) various_call = 1;
                    break;
            }
        }
    }
}

/*
 * send frame1 and frame2
 * one frame (frame1) is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 * a second frame (frame2) is sent every second (1000ms) with gps position data, and ahrs.yaw_sensor heading (instead of GPS heading)
 * (for DPort protocol)
 */
void AP_Frsky_Telem::send_DPort(void)
{
    static uint32_t last_200ms_frame = 0;
    static uint32_t last_1000ms_frame = 0;

    uint32_t now = hal.scheduler->millis();
    // send frame1 every 200ms
    if (now - last_200ms_frame > 200) {
        last_200ms_frame = now;
        frsky_send_data(FRSKY_ID_TEMP2, (uint16_t)(_ahrs.get_gps().num_sats() * 10 + _ahrs.get_gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        frsky_send_data(FRSKY_ID_TEMP1, (uint16_t)_control_mode); // send flight mode
        frsky_send_data(FRSKY_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
        frsky_send_data(FRSKY_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        frsky_send_data(FRSKY_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
        frsky_send_data(FRSKY_ID_BARO_ALT_BP, (uint16_t)roundf(_inav.get_altitude() * 0.01f)); // send barometer altitude integer part
        frsky_send_data(FRSKY_ID_BARO_ALT_AP, ((uint16_t)_inav.get_altitude())%100); // send barometer altitude decimal part
    }
    // send frame2 every second
    if (now - last_1000ms_frame > 1000) {
        last_1000ms_frame = now;
        frsky_send_data(FRSKY_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (_ahrs.get_gps().status() >= 3) {
            frsky_send_data(FRSKY_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
            frsky_send_data(FRSKY_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
            frsky_send_data(FRSKY_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
            frsky_send_data(FRSKY_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
            frsky_send_data(FRSKY_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
            frsky_send_data(FRSKY_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
            frsky_send_data(FRSKY_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
            frsky_send_data(FRSKY_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
            frsky_send_data(FRSKY_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
            frsky_send_data(FRSKY_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimals
        }
    }
}

/*
 * format the decimal latitude/longitude to the required degrees/minutes
 * (for DPort and SPort protocols)
 */
float AP_Frsky_Telem::format_gps(float dec)
{
    uint8_t dm_deg = (uint8_t) dec;
    return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/*
 * prepare gps data
 * (for DPort and SPort protocols)
 */
void AP_Frsky_Telem::calc_gps_position(void)
{
    float lat;
    float lon ;
    float alt ;
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
        _gps.alt_gps_cm = (alt - abs(_gps.alt_gps_meters)) * 100;
        
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

/* 
 * build up the frame's crc
 * (for SPort and XPort protocols)
 */
void AP_Frsky_Telem::frsky_calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * (for SPort and XPort protocols)
 */
void AP_Frsky_Telem::frsky_send_crc(void) 
{
    frsky_send_byte(0xFF - _crc);
    _crc = 0;
}

/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_Telem::frsky_send_byte(uint8_t byte)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) { // protocol for D-receivers
        if (byte == 0x5E) {
            _port->write(0x5D);
            _port->write(0x3E);
        } else if (byte == 0x5D) {
            _port->write(0x5D);
            _port->write(0x3D);
        } else {
            _port->write(byte);
        }
    } else { // Smart Port protocol (for X-receivers)
        if (byte == 0x7E) {
            _port->write(0x7D);
            _port->write(0x5E);
        } else if (byte == 0x7D) {
            _port->write(0x7D);
            _port->write(0x5D);
        } else {
            _port->write(byte);
        }
        frsky_calc_crc(byte);
    }
}

/*
 * send one frame of FrSky data
 */
void  AP_Frsky_Telem::frsky_send_data(uint16_t id, uint32_t value)
{
    if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) { // protocol for D-receivers
        _port->write(0x5E);    // send a 0x5E start byte
        uint8_t *bytes = (uint8_t*)&id;
        frsky_send_byte(bytes[0]);
        bytes = (uint8_t*)&value;
        frsky_send_byte(bytes[0]); // LSB
        frsky_send_byte(bytes[1]); // MSB
    } else { // Smart Port protocol (for X-receivers)
        frsky_send_byte(0x10); // DATA_FRAME
        uint8_t *bytes = (uint8_t*)&id;
        frsky_send_byte(bytes[0]); // LSB
        frsky_send_byte(bytes[1]); // MSB
        bytes = (uint8_t*)&value;
        frsky_send_byte(bytes[0]); // LSB
        frsky_send_byte(bytes[1]);
        frsky_send_byte(bytes[2]);
        frsky_send_byte(bytes[3]); // MSB
        frsky_send_crc();
    }
}
