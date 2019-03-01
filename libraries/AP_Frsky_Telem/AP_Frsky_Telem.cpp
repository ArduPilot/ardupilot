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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

ObjectArray<mavlink_statustext_t> AP_Frsky_Telem::_statustext_queue(FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY);

/*
 * init - perform required initialisation
 */
void AP_Frsky_Telem::init(const uint32_t *ap_valuep)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_D; // FrSky D protocol (D-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort; // FrSky SPort protocol (X-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough; // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
        // make frsky_telemetry available to GCS_MAVLINK (used to queue statustext messages from GCS_MAVLINK)
        // add firmware and frame info to message queue
        const char* _frame_string = gcs().frame_string();
        if (_frame_string == nullptr) {
            queue_message(MAV_SEVERITY_INFO, AP::fwversion().fw_string);
        } else {
            char firmware_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
            snprintf(firmware_buf, sizeof(firmware_buf), "%s %s", AP::fwversion().fw_string, _frame_string);
            queue_message(MAV_SEVERITY_INFO, firmware_buf);
        }
        // save main parameters locally
        if (ap_valuep == nullptr) { // ap bit-field
            _ap.value = 0x2000; // set "initialised" to 1 for rover and plane
            _ap.valuep = &_ap.value;
        } else {
            _ap.valuep = ap_valuep;
        }
    }
    
    if (_port != nullptr) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::tick, void));
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}


/*
 * send telemetry data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort_Passthrough(void)
{
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

    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    for (int16_t i = 0; i < numc; i++) {
        prev_byte = _passthrough.new_byte;
        _passthrough.new_byte = _port->read();
    }

    if ((prev_byte == START_STOP_SPORT) && (_passthrough.new_byte == SENSOR_ID_28)) { // byte 0x7E is the header of each poll request
        if (_passthrough.send_attiandrng) { // skip other data, send attitude (roll, pitch) and range only this iteration
            _passthrough.send_attiandrng = false; // next iteration, check if we should send something other
        } else { // send other sensor data if it's time for them, and reset the corresponding timer if sent
            _passthrough.send_attiandrng = true; // next iteration, send attitude b/c it needs frequent updates to remain smooth
            uint32_t now = AP_HAL::millis();
            if ((now - _passthrough.params_timer) >= 1000) {
                send_uint32(DIY_FIRST_ID+7, calc_param());
                _passthrough.params_timer = AP_HAL::millis();
                return;
            }
            // build message queue for sensor_status_flags
            check_sensor_status_flags();
            // build message queue for ekf_status
            check_ekf_status();
            // if there's any message in the queue, start sending them chunk by chunk; three times each chunk
            if (get_next_msg_chunk()) {
                send_uint32(DIY_FIRST_ID, _msg_chunk.chunk);
                return;
            }
            if ((now - _passthrough.ap_status_timer) >= 500) {
                if (((*_ap.valuep) & AP_INITIALIZED_FLAG) > 0) {  // send ap status only once vehicle has been initialised
                    send_uint32(DIY_FIRST_ID+1, calc_ap_status());
                    _passthrough.ap_status_timer = AP_HAL::millis();
                }
                return;
            }
            if ((now - _passthrough.batt_timer) >= 1000) {
                send_uint32(DIY_FIRST_ID+3, calc_batt(0));
                _passthrough.batt_timer = AP_HAL::millis();
                return;
            }
            if (AP::battery().num_instances() > 1) {
                if ((now - _passthrough.batt_timer2) >= 1000) {
                    send_uint32(DIY_FIRST_ID+8, calc_batt(1));
                    _passthrough.batt_timer2 = AP_HAL::millis();
                    return;
                }
            }
            if ((now - _passthrough.gps_status_timer) >= 1000) {
                send_uint32(DIY_FIRST_ID+2, calc_gps_status());
                _passthrough.gps_status_timer = AP_HAL::millis();
                return;
            }
            if ((now - _passthrough.home_timer) >= 500) {
                send_uint32(DIY_FIRST_ID+4, calc_home());
                _passthrough.home_timer = AP_HAL::millis();
                return;
            }
            if ((now - _passthrough.velandyaw_timer) >= 500) {
                send_uint32(DIY_FIRST_ID+5, calc_velandyaw());
                _passthrough.velandyaw_timer = AP_HAL::millis();
                return;
            }
            if ((now - _passthrough.gps_latlng_timer) >= 1000) {
                send_uint32(GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
                if (!_passthrough.send_latitude) { // we've cycled and sent one each of longitude then latitude, so reset the timer
                    _passthrough.gps_latlng_timer = AP_HAL::millis();
                }
                return;
            }
        }
        // if nothing else needed to be sent, send attitude (roll, pitch) and range data
        send_uint32(DIY_FIRST_ID+6, calc_attiandrng());
    }
}

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort(void)
{
    const AP_AHRS &_ahrs = AP::ahrs();

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
        if (_SPort.sport_status == false) {
            if  (readbyte == START_STOP_SPORT) {
                _SPort.sport_status = true;
            }
        } else {
            const AP_BattMonitor &_battery = AP::battery();
            switch(readbyte) {
                case SENSOR_ID_FAS:
                    switch (_SPort.fas_call) {
                        case 0:
                            send_uint32(DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                            break;
                        case 1:
                            send_uint32(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                            break;
                        case 2:
                            send_uint32(DATA_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
                            break;
                    }
                    if (_SPort.fas_call++ > 2) _SPort.fas_call = 0;
                    break;
                case SENSOR_ID_GPS:
                    switch (_SPort.gps_call) {
                        case 0:
                            calc_gps_position(); // gps data is not recalculated until all of it has been sent
                            send_uint32(DATA_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
                            break;
                        case 1:
                            send_uint32(DATA_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
                            break;
                        case 2:
                            send_uint32(DATA_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
                            break;
                        case 3:
                            send_uint32(DATA_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
                            break;
                        case 4:
                            send_uint32(DATA_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
                            break;
                        case 5:
                            send_uint32(DATA_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
                            break;
                        case 6:
                            send_uint32(DATA_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
                            break;
                        case 7:
                            send_uint32(DATA_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
                            break;
                        case 8:
                            send_uint32(DATA_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
                            break;
                        case 9:
                            send_uint32(DATA_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimals
                            break;
                        case 10:
                            send_uint32(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
                            break;
                    }
                    if (_SPort.gps_call++ > 10) _SPort.gps_call = 0;
                    break;
                case SENSOR_ID_VARIO:
                    switch (_SPort.vario_call) {
                        case 0 :
                            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
                            send_uint32(DATA_ID_BARO_ALT_BP, _gps.alt_nav_meters); // send altitude integer part
                            break;
                        case 1:
                            send_uint32(DATA_ID_BARO_ALT_AP, _gps.alt_nav_cm); // send altitude decimal part
                            break;
                        }
                    if (_SPort.vario_call++ > 1) _SPort.vario_call = 0;
                    break;    
                case SENSOR_ID_SP2UR:
                    switch (_SPort.various_call) {
                        case 0 :
                            send_uint32(DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                            break;
                        case 1:
                            send_uint32(DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
                            break;
                    }
                    if (_SPort.various_call++ > 1) _SPort.various_call = 0;
                    break;
            }
            _SPort.sport_status = false;
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
    const AP_AHRS &_ahrs = AP::ahrs();
    const AP_BattMonitor &_battery = AP::battery();

    uint32_t now = AP_HAL::millis();
    // send frame1 every 200ms
    if (now - _D.last_200ms_frame >= 200) {
        _D.last_200ms_frame = now;
        send_uint16(DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        send_uint16(DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
        send_uint16(DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
        send_uint16(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        send_uint16(DATA_ID_CURRENT, (uint16_t)roundf(_battery.current_amps() * 10.0f)); // send current consumption
        calc_nav_alt();
        send_uint16(DATA_ID_BARO_ALT_BP, _gps.alt_nav_meters); // send nav altitude integer part
        send_uint16(DATA_ID_BARO_ALT_AP, _gps.alt_nav_cm); // send nav altitude decimal part
    }
    // send frame2 every second
    if (now - _D.last_1000ms_frame >= 1000) {
        _D.last_1000ms_frame = now;
        send_uint16(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (AP::gps().status() >= 3) {
            send_uint16(DATA_ID_GPS_LAT_BP, _gps.latdddmm); // send gps lattitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LAT_AP, _gps.latmmmm); // send gps lattitude minutes decimal part
            send_uint16(DATA_ID_GPS_LAT_NS, _gps.lat_ns); // send gps North / South information
            send_uint16(DATA_ID_GPS_LONG_BP, _gps.londddmm); // send gps longitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LONG_AP, _gps.lonmmmm); // send gps longitude minutes decimal part
            send_uint16(DATA_ID_GPS_LONG_EW, _gps.lon_ew); // send gps East / West information
            send_uint16(DATA_ID_GPS_SPEED_BP, _gps.speed_in_meter); // send gps speed integer part
            send_uint16(DATA_ID_GPS_SPEED_AP, _gps.speed_in_centimeter); // send gps speed decimal part
            send_uint16(DATA_ID_GPS_ALT_BP, _gps.alt_gps_meters); // send gps altitude integer part
            send_uint16(DATA_ID_GPS_ALT_AP, _gps.alt_gps_cm); // send gps altitude decimal part
        }
    }
}

/*
 * tick - main call to send data to the receiver (called by scheduler at 1kHz)
 */
void AP_Frsky_Telem::tick(void)
{
    // check UART has been initialised
    if (!_initialised_uart) {
        // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
        if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                    // FrSky D protocol (D-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_D_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        } else {                                                                        // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
            _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
        }
        _initialised_uart = true;// true when we have detected the protocol and UART has been initialised
    }

    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                        // FrSky D protocol (D-receivers)
        send_D();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort) {             // FrSky SPort protocol (X-receivers)
        send_SPort();
    } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough) { // FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
        send_SPort_Passthrough();
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
 * send one uint32 frame of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  AP_Frsky_Telem::send_uint32(uint16_t id, uint32_t data)
{
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

/*
 * send one uint16 frame of FrSky data - for FrSky D protocol (D-receivers)
 */
void  AP_Frsky_Telem::send_uint16(uint16_t id, uint16_t data)
{
    _port->write(START_STOP_D);    // send a 0x5E start byte
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(bytes[0]);
    bytes = (uint8_t*)&data;
    send_byte(bytes[0]); // LSB
    send_byte(bytes[1]); // MSB
}

/*
 * grabs one "chunk" (4 bytes) of the queued message to be transmitted
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_Telem::get_next_msg_chunk(void)
{
    if (_statustext_queue.empty()) {
        return false;
    }

    if (_msg_chunk.repeats == 0) { // if it's the first time get_next_msg_chunk is called for a given chunk
        uint8_t character = 0;
        _msg_chunk.chunk = 0; // clear the 4 bytes of the chunk buffer

        for (int i = 3; i > -1 && _msg_chunk.char_index < sizeof(_statustext_queue[0]->text); i--) {
            character = _statustext_queue[0]->text[_msg_chunk.char_index++];

            if (!character) {
                break;
            }

            _msg_chunk.chunk |= character << i * 8;
        }

        if (!character || (_msg_chunk.char_index == sizeof(_statustext_queue[0]->text))) { // we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
            _msg_chunk.char_index = 0; // reset index to get ready to process the next message
            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            _msg_chunk.chunk |= (_statustext_queue[0]->severity & 0x4)<<21;
            _msg_chunk.chunk |= (_statustext_queue[0]->severity & 0x2)<<14;
            _msg_chunk.chunk |= (_statustext_queue[0]->severity & 0x1)<<7;
        }
    }

    if (_msg_chunk.repeats++ > 2) { // repeat each message chunk 3 times to ensure transmission
        _msg_chunk.repeats = 0;
        if (_msg_chunk.char_index == 0) { // if we're ready for the next message
            _statustext_queue.remove(0);
        }
    }
    return true;
}

/*
 * add message to message cue for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::queue_message(MAV_SEVERITY severity, const char *text)
{
    mavlink_statustext_t statustext{};

    statustext.severity = severity;
    strncpy(statustext.text, text, sizeof(statustext.text));

    // The force push will ensure comm links do not block other comm links forever if they fail.
    // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
    // block but not until the buffer fills up.
    _statustext_queue.push_force(statustext);
}

/*
 * add sensor_status_flags information to message cue, normally passed as sys_status mavlink messages to the GCS, for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::check_sensor_status_flags(void)
{
    uint32_t now = AP_HAL::millis();

    const uint32_t _sensor_status_flags = sensor_status_flags();

    if ((now - check_sensor_status_timer) >= 5000) { // prevent repeating any system_status messages unless 5 seconds have passed
        // only one error is reported at a time (in order of preference). Same setup and displayed messages as Mission Planner.
        if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_GPS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad GPS Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_GYRO) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Gyro Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_ACCEL) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Accel Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_MAG) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Compass Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Baro Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_LASER_POSITION) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad LiDAR Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad OptFlow Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_TERRAIN) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad or No Terrain Data");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_GEOFENCE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Geofence Breach");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_AHRS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad AHRS");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "No RC Receiver");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_LOGGING) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Logging");
            check_sensor_status_timer = now;
        }
    }
}

/*
 * add innovation variance information to message cue, normally passed as ekf_status_report mavlink messages to the GCS, for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::check_ekf_status(void)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    // get variances
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    if (_ahrs.get_variances(velVar, posVar, hgtVar, magVar, tasVar, offset)) {
        uint32_t now = AP_HAL::millis();
        if ((now - check_ekf_status_timer) >= 10000) { // prevent repeating any ekf_status message unless 10 seconds have passed
            // multiple errors can be reported at a time. Same setup as Mission Planner.
            if (velVar >= 1) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error velocity variance");
                check_ekf_status_timer = now;
            }
            if (posVar >= 1) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error pos horiz variance");
                check_ekf_status_timer = now;
            }
            if (hgtVar >= 1) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error pos vert variance");
                check_ekf_status_timer = now;
            }
            if (magVar.length() >= 1) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error compass variance");
                check_ekf_status_timer = now;
            }
            if (tasVar >= 1) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error terrain alt variance");
                check_ekf_status_timer = now;
            }
        }
    }
}
      
/*
 * prepare parameter data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_param(void)
{
    const AP_BattMonitor &_battery = AP::battery();

    uint32_t param = 0;

    // cycle through paramIDs
    if (_paramID >= 5) {
        _paramID = 0;
    }
    _paramID++;
    switch(_paramID) {
    case 1:
        param = gcs().frame_type(); // see MAV_TYPE in Mavlink definition file common.h
        break;
    case 2: // was used to send the battery failsafe voltage
    case 3: // was used to send the battery failsafe capacity in mAh
        break;
    case 4:
        param = (uint32_t)roundf(_battery.pack_capacity_mah(0)); // battery pack capacity in mAh
        break;
    case 5:
        param = (uint32_t)roundf(_battery.pack_capacity_mah(1)); // battery pack capacity in mAh
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    param = (_paramID << PARAM_ID_OFFSET) | (param & PARAM_VALUE_LIMIT);
    
    return param;
}

/*
 * prepare gps latitude/longitude data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_latlng(bool *send_latitude)
{
    uint32_t latlng;
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if ((*send_latitude) == true) {
        if (loc.lat < 0) {
            latlng = ((labs(loc.lat)/100)*6) | 0x40000000;
        } else {
            latlng = ((labs(loc.lat)/100)*6);
        }
        (*send_latitude) = false;
    } else {
        if (loc.lng < 0) {
            latlng = ((labs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            latlng = ((labs(loc.lng)/100)*6) | 0x80000000;
        }
        (*send_latitude) = true;
    }
    return latlng;
}

/*
 * prepare gps status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_status(void)
{
    const AP_GPS &gps = AP::gps();

    uint32_t gps_status;

    // number of GPS satellites visible (limit to 15 (0xF) since the value is stored on 4 bits)
    gps_status = (gps.num_sats() < GPS_SATS_LIMIT) ? gps.num_sats() : GPS_SATS_LIMIT;
    // GPS receiver status (limit to 0-3 (0x3) since the value is stored on 2 bits: NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D or GPS_OK_FIX_3D_DGPS or GPS_OK_FIX_3D_RTK_FLOAT or GPS_OK_FIX_3D_RTK_FIXED = 3)
    gps_status |= ((gps.status() < GPS_STATUS_LIMIT) ? gps.status() : GPS_STATUS_LIMIT)<<GPS_STATUS_OFFSET;
    // GPS horizontal dilution of precision in dm
    gps_status |= prep_number(roundf(gps.get_hdop() * 0.1f),2,1)<<GPS_HDOP_OFFSET; 
    // GPS receiver advanced status (0: no advanced fix, 1: GPS_OK_FIX_3D_DGPS, 2: GPS_OK_FIX_3D_RTK_FLOAT, 3: GPS_OK_FIX_3D_RTK_FIXED)
    gps_status |= ((gps.status() > GPS_STATUS_LIMIT) ? gps.status()-GPS_STATUS_LIMIT : 0)<<GPS_ADVSTATUS_OFFSET;
    // Altitude MSL in dm
    const Location &loc = gps.location();
    gps_status |= prep_number(roundf(loc.alt * 0.1f),2,2)<<GPS_ALTMSL_OFFSET; 
    return gps_status;
}

/*
 * prepare battery data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_batt(uint8_t instance)
{
    const AP_BattMonitor &_battery = AP::battery();

    uint32_t batt;
    
    // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    batt = (((uint16_t)roundf(_battery.voltage(instance) * 10.0f)) & BATT_VOLTAGE_LIMIT);
    // battery current draw in deciamps
    batt |= prep_number(roundf(_battery.current_amps(instance) * 10.0f), 2, 1)<<BATT_CURRENT_OFFSET; 
    // battery current drawn since power on in mAh (limit to 32767 (0x7FFF) since value is stored on 15 bits)
    batt |= ((_battery.consumed_mah(instance) < BATT_TOTALMAH_LIMIT) ? ((uint16_t)roundf(_battery.consumed_mah(instance)) & BATT_TOTALMAH_LIMIT) : BATT_TOTALMAH_LIMIT)<<BATT_TOTALMAH_OFFSET;
    return batt;
}

/*
 * prepare various autopilot status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_ap_status(void)
{
    uint32_t ap_status;

    // IMU temperature: offset -19, 0 means temp =< 19°, 63 means temp => 82°
    uint8_t imu_temp = (uint8_t) roundf(constrain_float(AP::ins().get_temperature(0), AP_IMU_TEMP_MIN, AP_IMU_TEMP_MAX) - AP_IMU_TEMP_MIN);

    // control/flight mode number (limit to 31 (0x1F) since the value is stored on 5 bits)
    ap_status = (uint8_t)((gcs().custom_mode()+1) & AP_CONTROL_MODE_LIMIT);
    // simple/super simple modes flags
    ap_status |= (uint8_t)((*_ap.valuep) & AP_SSIMPLE_FLAGS)<<AP_SSIMPLE_OFFSET;
    // is_flying flag
    ap_status |= (uint8_t)(((*_ap.valuep) & AP_LANDCOMPLETE_FLAG) ^ AP_LANDCOMPLETE_FLAG);
    // armed flag
    ap_status |= (uint8_t)(AP_Notify::flags.armed)<<AP_ARMED_OFFSET;
    // battery failsafe flag
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery)<<AP_BATT_FS_OFFSET;
    // bad ekf flag
    ap_status |= (uint8_t)(AP_Notify::flags.ekf_bad)<<AP_EKF_FS_OFFSET;
    // IMU temperature
    ap_status |= imu_temp << AP_IMU_TEMP_OFFSET;
    return ap_status;
}

/*
 * prepare home position related data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_home(void)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    uint32_t home = 0;
    Location loc;
    float _relative_home_altitude = 0;
    if (_ahrs.get_position(loc)) {            
        // check home_loc is valid
        const Location &home_loc = _ahrs.get_home();
        if (home_loc.lat != 0 || home_loc.lng != 0) {
            // distance between vehicle and home_loc in meters
            home = prep_number(roundf(home_loc.get_distance(loc)), 3, 2);
            // angle from front of vehicle to the direction of home_loc in 3 degree increments (just in case, limit to 127 (0x7F) since the value is stored on 7 bits)
            home |= (((uint8_t)roundf(get_bearing_cd(loc,home_loc) * 0.00333f)) & HOME_BEARING_LIMIT)<<HOME_BEARING_OFFSET;
        }
        // altitude between vehicle and home_loc
        _relative_home_altitude = loc.alt;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            _relative_home_altitude -= _ahrs.get_home().alt;
        }
    }
    // altitude above home in decimeters
    home |= prep_number(roundf(_relative_home_altitude * 0.1f), 3, 2)<<HOME_ALT_OFFSET;
    return home;
}

/*
 * prepare velocity and yaw data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_velandyaw(void)
{
    AP_AHRS &_ahrs = AP::ahrs();

    uint32_t velandyaw;
    Vector3f velNED {};

    // if we can't get velocity then we use zero for vertical velocity
    _ahrs.get_velocity_NED(velNED);
    // vertical velocity in dm/s
    velandyaw = prep_number(roundf(-velNED.z * 10), 2, 1);
    // horizontal velocity in dm/s (use airspeed if available and enabled - even if not used - otherwise use groundspeed)
    const AP_Airspeed *aspeed = _ahrs.get_airspeed();
    if (aspeed && aspeed->enabled()) {        
        velandyaw |= prep_number(roundf(aspeed->get_airspeed() * 10), 2, 1)<<VELANDYAW_XYVEL_OFFSET;
    } else { // otherwise send groundspeed estimate from ahrs
        velandyaw |= prep_number(roundf(_ahrs.groundspeed() * 10), 2, 1)<<VELANDYAW_XYVEL_OFFSET;
    }
    // yaw from [0;36000] centidegrees to .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    velandyaw |= ((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f) & VELANDYAW_YAW_LIMIT)<<VELANDYAW_YAW_OFFSET;
    return velandyaw;
}

/*
 * prepare attitude (roll, pitch) and range data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_attiandrng(void)
{
    const AP_AHRS &_ahrs = AP::ahrs();
    const RangeFinder *_rng = RangeFinder::get_singleton();

    uint32_t attiandrng;

    // roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    attiandrng = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ATTIANDRNG_ROLL_LIMIT);
    // pitch from [-9000;9000] centidegrees to unsigned .2 degree increments [0;900] (just in case, limit to 1023 (0x3FF) since the value is stored on 10 bits)
    attiandrng |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & ATTIANDRNG_PITCH_LIMIT)<<ATTIANDRNG_PITCH_OFFSET;
    // rangefinder measurement in cm
    attiandrng |= prep_number(_rng ? _rng->distance_cm_orient(ROTATION_PITCH_270) : 0, 3, 1)<<ATTIANDRNG_RNGFND_OFFSET;
    return attiandrng;
}

/*
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t AP_Frsky_Telem::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10240)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 127000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}

/*
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_nav_alt(void)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    Location loc;
    float current_height = 0; // in centimeters above home
    if (_ahrs.get_position(loc)) {
        current_height = loc.alt*0.01f;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            current_height -= _ahrs.get_home().alt*0.01f;
        }
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

    if (AP::gps().status() >= 3) {
        const Location &loc = AP::gps().location(); //get gps instance 0
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

        speed = AP::gps().ground_speed();
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

uint32_t AP_Frsky_Telem::sensor_status_flags() const
{
    uint32_t present;
    uint32_t enabled;
    uint32_t health;
    gcs().get_sensor_status_flags(present, enabled, health);

    return ~health & enabled & present;
}
