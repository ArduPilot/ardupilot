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
AP_Frsky_Telem::AP_Frsky_Telem(const AP_InertialNav& inav, const AP_AHRS &ahrs, const AP_BattMonitor &battery, const RangeFinder &rng) :
	_inav(inav),
	_ahrs(ahrs),
	_rng(rng),
    _battery(battery),
	
    _port(NULL),
    _protocol(),
    _crc(0),
	
	_gps(),
	_mav(),
	
	_control_sensors_present(0),
	_control_sensors_enabled(0),
	_control_sensors_health(0),
	
	_home_distance(0),
	_home_bearing(0),
	_control_mode(0),
	_simple_mode(0),
	_land_complete(true)
	{}

// init - perform require initialisation including detecting which protocol to use (changes to the serial port parameter while turned on won't be effective; need to reset the unit for changes to be effective)
void AP_Frsky_Telem::init(const AP_SerialManager& serial_manager)
{
	// check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_DPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_DPort; // protocol for D-receivers
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_SPort; // former Smart.Port protocol (for X-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FRSky_XPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FRSky_XPort; // new Smart.Port protocol (for X-receivers)
    }

    if (_port != NULL) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
		_port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

/*
  tick - main call to send updates to transmitter (called by scheduler at 1kHz)
*/
void AP_Frsky_Telem::tick(void)
{
	static bool initialised_uart = false; // true when we have detected the protocol and UART has been initialised
	// check UART has been initialised
	if (!initialised_uart) {
		// initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
		if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) { 	// for DPort protocol
			_port->begin(AP_SERIALMANAGER_FRSKY_DPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
		} else {															// for SPort and XPort protocols
			_port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
		}
		initialised_uart = true;
	}

	if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) { 	// for DPort protocol
		send_hub_frame();
		return;
	}
	
	static bool polled = false; // true if the receiver is polling a sensor
	// variables keeping track of which data to send
	// for SPort protocol
	static uint8_t fas_call = 1;
	static uint8_t gps_call = 1;
	static uint8_t vario_call = 1;
	static uint8_t various_call = 1;
	// for XPort protocol
	static uint32_t msg_chunk; // contains a "chunk" (four characters/bytes) at a time of the mavlink message to be sent
	static uint8_t repeats = 3; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
	static uint8_t oneHz_call = 1;
	static uint8_t oneover2n_call = 1;
		
	// build mavlink message queue - this must run fast to keep monitoring new statustext messages and flag changes
	if (_protocol == AP_SerialManager::SerialProtocol_FRSky_XPort) {	// for XPort protocol only
		mavlink_statustext_check();
		control_sensors_check();
		ekf_status_check();
	}
	
	while (_port->available()) {
		uint8_t readbyte = _port->read();
		if (!polled) {
			if (readbyte == 0x7E) { // byte 0x7E is the header of each poll request
                polled = true;
			}
        } else {
			if (_protocol == AP_SerialManager::SerialProtocol_FRSky_SPort) { // for SPort protocol
				hal.console->print_P(PSTR("SPort"));
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
			} else { // for XPort protocol
				switch(readbyte) {
					case SENSOR_8: // grab the next message chunk in the queue if it exists and send it repeats times
						if ((_mav.msgNum_sent != _mav.msgNum_current) || (repeats < 3)) {
							if (repeats != 0) {
								if (repeats == 3) {
									msg_chunk = get_next_msg_chunk();
								}
								frsky_send_data(0x1000, msg_chunk);
								repeats--;
							}
							if (repeats == 0) {
								repeats = 3;
							}
						}
						break;
					case SENSOR_9:
						if (oneHz_call) {
							switch (oneHz_call) { // both sent at 1-3 Hz
								case 1:
									frsky_send_data(0x1002, calc_gps());
									break;
								case 2:
									frsky_send_data(0x1003, calc_batt());
									break;
							}
							oneHz_call++;
						}
						break;
					case SENSOR_10:
						switch (oneover2n_call) { // each sent at 1/2n max rate, where n is the number of cases (lowest rate the sensor is reliably polled)
							case 1:
								frsky_send_data(0x1001, calc_status());
								break;
							case 3:
								frsky_send_data(0x1004, calc_home());
								break;
							case 5:
								frsky_send_data(0x1005, calc_velandrng());
								break;
						}
						if (oneover2n_call++ > 5) oneover2n_call = 1;
						break;
					case SENSOR_11:	// sent at max rate (~25-30Hz depending on number of sensors responding)
						frsky_send_data(0x1006, calc_attitude());
						break;
					case SENSOR_28: // this sensor call is used as the 1Hz sync byte (don't use it to send data)
						oneHz_call = 1; // sensor calls are cycled at approximately 1Hz (but sometimes up to 3Hz for some reason)
						break;
				}
			}
			polled = false;
		}
	}
}

/*
 * grabs one byte of the mavlink statustext message to be transmitted, returns 0 if the end of the message is reached
 * (for XPort protocol)
 */
uint8_t AP_Frsky_Telem::get_next_msg_byte(void)
{
	static uint8_t charIdx = 0;
	uint8_t res = _mav.msg_data[_mav.msgNum_sent].text[charIdx++];
	
	if (!res) { //atEOM = !res; equal to 1 if we've reached the end of the message (string terminated by '\0')
		charIdx = 0;
		_mav.msgNum_sent = (_mav.msgNum_sent + 1) % TEXT_NUM_BUFFERS;
	}
	return res;
}

/*
 * grabs one "chunk" (4 bytes) of the mavlink statustext message to be transmitted
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::get_next_msg_chunk(void)
{
	uint32_t res = 0;
	uint8_t ch = get_next_msg_byte();
	
	if (ch) {
		res |= ch<<24;
		ch = get_next_msg_byte();
		if (ch) {
			res |= ch<<16;
			ch = get_next_msg_byte();
			if (ch) {
				res |= ch<<8;
				ch = get_next_msg_byte();
				if (ch) {
					res |= ch;
				}
			}
		}
	}
	return res;
}

/*
 * add mavlink_statustext messages to message cue, normally passed as status text mavlink messages to the GCS, for transmission through FrSky link
 * (for XPort protocol)
 */
void AP_Frsky_Telem::mavlink_statustext_check(void)
{
	if (GCS_MAVLINK::gcs_message_flag) {
		GCS_MAVLINK::gcs_message_flag = 0;
		_mav.msg_data[_mav.msgNum_current] = GCS_MAVLINK::gcs_message_data;
		_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
	}
}

/*
 * add control_sensors information to message cue, normally passed as system_status mavlink messages to the GCS, for transmission through FrSky link
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
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad GPS Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_GYRO) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad Gyro Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_ACCEL) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad Accel Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_3D_MAG) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad Compass Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad Baro Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_LASER_POSITION) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad LiDAR Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad OptFlow Health");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_TERRAIN) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad or No Terrain Data");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_GEOFENCE) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Geofence Breach");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			control_sensors_timer = now;
		} else if ((_control_sensors_flags & MAV_SYS_STATUS_AHRS) > 0) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Bad AHRS");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
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
	uint32_t now = hal.scheduler->millis();
	if ((now - ekf_status_timer) > 10000) { // prevent repeating any ekf_status_report messages unless 10 seconds have passed
        // multiple errors can be reported at a time. Same setup as Mission Planner.
		if (NavEKF::ekf_status_report.velocity_variance >= 1) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Error velocity variance");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			ekf_status_timer = now;
		}

		if (NavEKF::ekf_status_report.pos_horiz_variance >= 1) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Error compass variance");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			ekf_status_timer = now;
		}

		if (NavEKF::ekf_status_report.pos_vert_variance >= 1) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Error pos horiz variance");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			ekf_status_timer = now;
		}

		if (NavEKF::ekf_status_report.compass_variance >= 1) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Error compass variance");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			ekf_status_timer = now;
		}

		if (NavEKF::ekf_status_report.terrain_alt_variance >= 1) {
			sprintf(_mav.msg_data[_mav.msgNum_current].text, "Error terrain alt variance");
			_mav.msgNum_current = (_mav.msgNum_current + 1) % TEXT_NUM_BUFFERS;
			ekf_status_timer = now;
		}
	}
	/*
	flags found in AP_NavEKF/AP_Nav_Common.h | the flags are not used as of Copter3.3
	NavEKF::ekf_status_report.flags.attitude				// 0 - true if attitude estimate is valid
	NavEKF::ekf_status_report.flags.horiz_vel				// 1 - true if horizontal velocity estimate is valid
	NavEKF::ekf_status_report.flags.vert_vel				// 2 - true if the vertical velocity estimate is valid
	NavEKF::ekf_status_report.flags.horiz_pos_rel			// 3 - true if the relative horizontal position estimate is valid
	NavEKF::ekf_status_report.flags.horiz_pos_abs			// 4 - true if the absolute horizontal position estimate is valid
	NavEKF::ekf_status_report.flags.vert_pos				// 5 - true if the vertical position estimate is valid
	NavEKF::ekf_status_report.flags.terrain_alt				// 6 - true if the terrain height estimate is valid
	NavEKF::ekf_status_report.flags.const_pos_mode			// 7 - true if we are in const position mode
	NavEKF::ekf_status_report.flags.pred_horiz_pos_rel		// 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
	NavEKF::ekf_status_report.flags.pred_horiz_pos_abs		// 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
	*/
}

/* 
 * build up the frame's crc
 * (for SPort and XPort protocols)
 */
void AP_Frsky_Telem::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

/*
 * send the frame's crc at the end of the frame
 * (for SPort and XPort protocols)
 */
void AP_Frsky_Telem::send_crc(void) 
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
    } else { // Smart.Port protocol (for X-receivers)
		if (byte == 0x7E) {
			_port->write(0x7D);
			_port->write(0x5E);
		} else if (byte == 0x7D) {
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
void  AP_Frsky_Telem::frsky_send_data(uint16_t id, uint32_t value)
{
	if (_protocol == AP_SerialManager::SerialProtocol_FRSky_DPort) { // protocol for D-receivers
		_port->write(0x5E);	// send a 0x5E start byte
        uint8_t *bytes = (uint8_t*)&id;
		frsky_send_byte(bytes[0]);
		bytes = (uint8_t*)&value;
		frsky_send_byte(bytes[0]); // LSB
		frsky_send_byte(bytes[1]); // MSB
	} else { // Smart.Port protocol (for X-receivers)
		frsky_send_byte(0x10); // DATA_FRAME
		uint8_t *bytes = (uint8_t*)&id;
		frsky_send_byte(bytes[0]); // LSB
		frsky_send_byte(bytes[1]); // MSB
		bytes = (uint8_t*)&value;
		frsky_send_byte(bytes[0]); // LSB
		frsky_send_byte(bytes[1]);
		frsky_send_byte(bytes[2]);
		frsky_send_byte(bytes[3]); // MSB
		send_crc();
	}
}

/*
 * prepare various status data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_status(void)
{
	uint32_t status;
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
	uint32_t home;
	home = ((uint16_t)_inav.get_altitude() & 0x7FFF); // altitude between vehicle and home location in centimeters
	home |= (uint8_t)(_home_bearing & 0x7F)<<15; // angle between vehicle and home location (relative to North) in 3 degree increments
	home |= (uint16_t)(_home_distance & 0x3FF)<<22; // distance between vehicle and home location in meters
	return home;
}

/*
 * prepare gps data
 * (for XPort protocol)
 */

uint32_t AP_Frsky_Telem::calc_gps(void)
{
	uint32_t gps;
	gps = (uint8_t)(_ahrs.get_gps().num_sats() & 0xF); // number of GPS satellites visible
	gps |= (uint8_t)(_ahrs.get_gps().status() & 0x3)<<4; // GPS receiver status (i.e., NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D >= 3)
	gps |= (uint16_t)(_ahrs.get_gps().get_hdop() & 0x3FFF)<<6; // GPS horizontal dilution of precision in cm
	return gps;
}

/*
 * prepare battery data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_batt(void)
{
	uint32_t batt;
	batt = (((uint16_t)roundf(_battery.voltage() * 10.0f)) & 0x1FF); // battery voltage in decivolts
	batt |= (((uint16_t)roundf(_battery.current_amps() * 10.0f)) & 0x1FF)<<9; // battery current draw in deciamps
	batt |= (((uint8_t)roundf(_battery.capacity_remaining_pct())) & 0x7F)<<18; // battery percentage remaining
	return batt;
}

/*
 * prepare velocity and range data
 * (for XPort protocol)
 */
uint32_t AP_Frsky_Telem::calc_velandrng(void)
{
	uint32_t velandrng;
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
	uint32_t attitude;
	attitude = (((uint16_t)roundf((_ahrs.roll_sensor + 18000)* 0.05f)) & 0x7FF); // roll in .2 degree increments
	attitude |= (((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f)) & 0x3FF)<<11; // pitch in .2 degree increments
	attitude |= (((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f)) & 0x7FF)<<21; // yaw in .2 degree increments
	return attitude;
}


/*
 * send frame1 and frame2
 * one frame (frame1) is sent every 200ms with baro alt, nb sats, batt volts and amp, control_mode
 * a second frame (frame2) is sent every second (1000ms) with gps position data, and ahrs.yaw_sensor heading (instead of GPS heading)
 * (for DPort protocol)
 */
void AP_Frsky_Telem::send_hub_frame(void)
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
float AP_Frsky_Telem::frsky_format_gps(float dec)
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
    
        lat = frsky_format_gps(fabsf(loc.lat/10000000.0f));
        _gps.latdddmm = lat;
        _gps.latmmmm = (lat - _gps.latdddmm) * 10000;
        _gps.lat_ns = (loc.lat < 0) ? 'S' : 'N';
        
        lon = frsky_format_gps(fabsf(loc.lng/10000000.0f));
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
