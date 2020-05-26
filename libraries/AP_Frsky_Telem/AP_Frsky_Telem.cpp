/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

AP_Frsky_Telem *AP_Frsky_Telem::singleton;

AP_Frsky_Telem::AP_Frsky_Telem(bool _external_data) : AP_RCTelemetry(TIME_SLOT_MAX),
    use_external_data(_external_data)
{
    singleton = this;
}

AP_Frsky_Telem::~AP_Frsky_Telem(void)
{
    singleton = nullptr;
}

/*
  setup ready for passthrough telem
 */
void AP_Frsky_Telem::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )
    set_scheduler_entry(TEXT, 35, 28);          // 0x5000 status text (dynamic)
    set_scheduler_entry(ATTITUDE, 50, 38);      // 0x5006 Attitude and range (dynamic)
    set_scheduler_entry(GPS_LAT, 550, 280);     // 0x800 GPS lat
    set_scheduler_entry(GPS_LON, 550, 280);     // 0x800 GPS lon
    set_scheduler_entry(VEL_YAW, 400, 250);     // 0x5005 Vel and Yaw
    set_scheduler_entry(AP_STATUS, 700, 500);   // 0x5001 AP status
    set_scheduler_entry(GPS_STATUS, 700, 500);  // 0x5002 GPS status
    set_scheduler_entry(HOME, 400, 500);        // 0x5004 Home
    set_scheduler_entry(BATT_2, 1300, 500);     // 0x5008 Battery 2 status
    set_scheduler_entry(BATT_1, 1300, 500);     // 0x5008 Battery 1 status
    set_scheduler_entry(PARAM, 1700, 1000);     // 0x5007 parameters
}

/*
 * init - perform required initialisation
 */
bool AP_Frsky_Telem::init()
{
    if (use_external_data) {
        return AP_RCTelemetry::init();
    }

    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_D, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_D; // FrSky D protocol (D-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort; // FrSky SPort protocol (X-receivers)
    } else if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough; // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
        AP_RCTelemetry::init();
    }

    if (_port != nullptr) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::loop, void),
                                          "FrSky",
                                          1024, AP_HAL::Scheduler::PRIORITY_RCIN, 1)) {
            return false;
        }
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        return true;
    }

    return false;
}

void AP_Frsky_Telem::adjust_packet_weight(bool queue_empty)
{
    if (!queue_empty) {
        _scheduler.packet_weight[TEXT] = 45;     // messages
        _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
    } else {
        _scheduler.packet_weight[TEXT] = 5000;   // messages
        _scheduler.packet_weight[ATTITUDE] = 45;     // attitude
    }
}

// WFQ scheduler
bool AP_Frsky_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    bool packet_ready = false;
    switch (idx) {
        case TEXT:
            packet_ready = !queue_empty;
            break;
        case AP_STATUS:
            packet_ready = gcs().vehicle_initialised();
            break;
        case BATT_2:
            packet_ready = AP::battery().num_instances() > 1;
            break;
        default:
            packet_ready = true;
            break;
    }

    return packet_ready;
}

/*
 * WFQ scheduler
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
        case TEXT: // 0x5000 status text
            if (get_next_msg_chunk()) {
                send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID, _msg_chunk.chunk);
            }
            break;
        case ATTITUDE: // 0x5006 Attitude and range
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+6, calc_attiandrng());
            break;
        case GPS_LAT: // 0x800 GPS lat
            // sample both lat and lon at the same time
            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
            _passthrough.gps_lng_sample = calc_gps_latlng(&_passthrough.send_latitude);
            // force the scheduler to select GPS lon as packet that's been waiting the most
            // this guarantees that gps coords are sent at max 
            // _scheduler.avg_polling_period*number_of_downlink_sensors time separation
            _scheduler.packet_timer[GPS_LON] = _scheduler.packet_timer[GPS_LAT] - 10000;
            break;
        case GPS_LON: // 0x800 GPS lon
            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, _passthrough.gps_lng_sample); // gps longitude
            break;
        case VEL_YAW: // 0x5005 Vel and Yaw
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+5, calc_velandyaw());
            break;
        case AP_STATUS: // 0x5001 AP status
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+1, calc_ap_status());
            break;
        case GPS_STATUS: // 0x5002 GPS Status
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+2, calc_gps_status());
            break;
        case HOME: // 0x5004 Home
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+4, calc_home());
            break;
        case BATT_2: // 0x5008 Battery 2 status
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+8, calc_batt(1));
            break;
        case BATT_1: // 0x5003 Battery 1 status
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+3, calc_batt(0));
            break;
        case PARAM: // 0x5007 parameters
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+7, calc_param());
            break;
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
    if (prev_byte == FRAME_HEAD) {
        if (_passthrough.new_byte == SENSOR_ID_27) { // byte 0x7E is the header of each poll request
            run_wfq_scheduler();
        }
    }
}

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort(void)
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

    if (numc == 0) {
        // no serial data to process do bg tasks
        if (_SPort.vario_refresh) {
            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
            _SPort.vario_refresh = false;
        }
        if (_SPort.gps_refresh) {
            calc_gps_position(); // gps data is not recalculated until all of it has been sent
            _SPort.gps_refresh = false;
        }
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_SPort.sport_status == false) {
            if  (readbyte == FRAME_HEAD) {
                _SPort.sport_status = true;
            }
        } else {
            const AP_BattMonitor &_battery = AP::battery();
            switch(readbyte) {
                case SENSOR_ID_VARIO:   // Sensor ID  0
                    switch (_SPort.vario_call) {
                        case 0:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters); // send altitude integer part
                            break;
                        case 1:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm); // send altitude decimal part
                            break;
                        case 2:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VARIO, _SPort_data.vario_vspd); // send vspeed m/s
                            _SPort.vario_refresh = true;
                            break;
                    }
                    if (++_SPort.vario_call > 2) {
                        _SPort.vario_call = 0;
                    } 
                    break;    
                case SENSOR_ID_FAS: // Sensor ID  2
                    switch (_SPort.fas_call) {
                        case 0:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                            break;
                        case 1:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                            break;
                        case 2:
                            {
                                float current;
                                if (!_battery.current_amps(current)) {
                                    current = 0;
                                }
                                send_sport_frame(SPORT_DATA_FRAME, DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption
                                break;
                            }                        
                            break;
                    }
                    if (++_SPort.fas_call > 2) {
                        _SPort.fas_call = 0;
                    }
                    break;
                case SENSOR_ID_GPS: // Sensor ID  3
                    switch (_SPort.gps_call) {
                        case 0:
                            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
                            break;
                        case 1:
                            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
                            break;
                        case 2:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
                            break;
                        case 3:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
                            break;
                        case 4:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
                            break;
                        case 5:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimals
                            break;
                        case 6:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_COURS_BP, _SPort_data.yaw); // send heading in degree based on AHRS and not GPS
                            _SPort.gps_refresh = true;
                            break;
                    }
                    if (++_SPort.gps_call > 6) {
                        _SPort.gps_call = 0;
                    }
                    break;
                case SENSOR_ID_SP2UR: // Sensor ID  6
                    switch (_SPort.various_call) {
                        case 0 :
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                            break;
                        case 1:
                            send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
                            break;
                    }
                    if (++_SPort.various_call > 1) {
                        _SPort.various_call = 0;
                    }
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
    const AP_BattMonitor &_battery = AP::battery();
    uint32_t now = AP_HAL::millis();
    // send frame1 every 200ms
    if (now - _D.last_200ms_frame >= 200) {
        _D.last_200ms_frame = now;
        send_uint16(DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
        send_uint16(DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
        send_uint16(DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
        send_uint16(DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
        float current;
        if (!_battery.current_amps(current)) {
            current = 0;
        }
        send_uint16(DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption        
        calc_nav_alt();
        send_uint16(DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters); // send nav altitude integer part
        send_uint16(DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm); // send nav altitude decimal part
    }
    // send frame2 every second
    if (now - _D.last_1000ms_frame >= 1000) {
        _D.last_1000ms_frame = now;
        AP_AHRS &_ahrs = AP::ahrs();
        send_uint16(DATA_ID_GPS_COURS_BP, (uint16_t)((_ahrs.yaw_sensor / 100) % 360)); // send heading in degree based on AHRS and not GPS
        calc_gps_position();
        if (AP::gps().status() >= 3) {
            send_uint16(DATA_ID_GPS_LAT_BP, _SPort_data.latdddmm); // send gps lattitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LAT_AP, _SPort_data.latmmmm); // send gps lattitude minutes decimal part
            send_uint16(DATA_ID_GPS_LAT_NS, _SPort_data.lat_ns); // send gps North / South information
            send_uint16(DATA_ID_GPS_LONG_BP, _SPort_data.londddmm); // send gps longitude degree and minute integer part
            send_uint16(DATA_ID_GPS_LONG_AP, _SPort_data.lonmmmm); // send gps longitude minutes decimal part
            send_uint16(DATA_ID_GPS_LONG_EW, _SPort_data.lon_ew); // send gps East / West information
            send_uint16(DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
            send_uint16(DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
            send_uint16(DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
            send_uint16(DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimal part
        }
    }
}

/*
  thread to loop handling bytes
 */
void AP_Frsky_Telem::loop(void)
{
    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                    // FrSky D protocol (D-receivers)
        _port->begin(AP_SERIALMANAGER_FRSKY_D_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
    } else {                                                                        // FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
        _port->begin(AP_SERIALMANAGER_FRSKY_SPORT_BAUD, AP_SERIALMANAGER_FRSKY_BUFSIZE_RX, AP_SERIALMANAGER_FRSKY_BUFSIZE_TX);
    }
    _port->set_unbuffered_writes(true);

    while (true) {
        hal.scheduler->delay(1);
        if (_protocol == AP_SerialManager::SerialProtocol_FrSky_D) {                        // FrSky D protocol (D-receivers)
            send_D();
        } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort) {             // FrSky SPort protocol (X-receivers)
            send_SPort();
        } else if (_protocol == AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough) { // FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
            send_SPort_Passthrough();
        }
    }
}

/*
  send 1 byte and do byte stuffing
*/
void AP_Frsky_Telem::send_byte(uint8_t byte)
{
    if (byte == START_STOP_D) {
        _port->write(0x5D);
        _port->write(0x3E);
    } else if (byte == BYTESTUFF_D) {
        _port->write(0x5D);
        _port->write(0x3D);
    } else {
        _port->write(byte);
    }
}

/*
 * send an 8 bytes SPort frame of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  AP_Frsky_Telem::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    if (use_external_data) {
        external_data.frame = frame;
        external_data.appid = appid;
        external_data.data = data;
        external_data.pending = true;
        return;
    }

    uint8_t buf[8];

    buf[0] = frame;
    buf[1] = appid & 0xFF;
    buf[2] = appid >> 8;
    memcpy(&buf[3], &data, 4);

    uint16_t sum = 0;
    for (uint8_t i=0; i<sizeof(buf)-1; i++) {
        sum += buf[i];
        sum += sum >> 8;
        sum &= 0xFF;              
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    buf[7] = (uint8_t)sum;

    // perform byte stuffing per SPort spec
    uint8_t len = 0;
    uint8_t buf2[sizeof(buf)*2+1];

    for (uint8_t i=0; i<sizeof(buf); i++) {
        uint8_t c = buf[i];
        if (c == FRAME_DLE || buf[i] == FRAME_HEAD) {
            buf2[len++] = FRAME_DLE;
            buf2[len++] = c ^ FRAME_XOR;
        } else {
            buf2[len++] = c;
        }
    }
#ifndef HAL_BOARD_SITL
    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will overwrite the next
      polling frame.
      SPort poll-to-pool period is 11.65ms, a frame takes 1.38ms
      this leaves us with up to 10ms to respond but to play it safe we
      allow no more than 7500us
     */
    uint64_t tend = _port->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > 7500) {
        // we've been too slow in responding
        return;
    }
#endif
    _port->write(buf2, len);
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
    if (!_statustext.available) {
        WITH_SEMAPHORE(_statustext.sem);
        if (!_statustext.queue.pop(_statustext.next)) {
            return false;
        }
        _statustext.available = true;
    }

    if (_msg_chunk.repeats == 0) { // if it's the first time get_next_msg_chunk is called for a given chunk
        uint8_t character = 0;
        _msg_chunk.chunk = 0; // clear the 4 bytes of the chunk buffer

        for (int i = 3; i > -1 && _msg_chunk.char_index < sizeof(_statustext.next.text); i--) {
            character = _statustext.next.text[_msg_chunk.char_index++];

            if (!character) {
                break;
            }

            _msg_chunk.chunk |= character << i * 8;
        }

        if (!character || (_msg_chunk.char_index == sizeof(_statustext.next.text))) { // we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
            _msg_chunk.char_index = 0; // reset index to get ready to process the next message
            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            _msg_chunk.chunk |= (_statustext.next.severity & 0x4)<<21;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x2)<<14;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x1)<<7;
        }
    }

    // repeat each message chunk 3 times to ensure transmission
    // on slow links reduce the number of duplicate chunks
    uint8_t extra_chunks = 2;

    if (_scheduler.avg_packet_rate < 20) {
        // with 3 or more extra frsky sensors on the bus
        // send messages only once
        extra_chunks = 0;
    } else if (_scheduler.avg_packet_rate < 30) {
        // with 1 or 2 extra frsky sensors on the bus
        // send messages twice
        extra_chunks = 1;
    }
    
    if (_msg_chunk.repeats++ > extra_chunks ) {
        _msg_chunk.repeats = 0;
        if (_msg_chunk.char_index == 0) {
            // we're ready for the next message
            _statustext.available = false;
        }
    }
    return true;
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
    float current, consumed_mah;
    if (!_battery.current_amps(current, instance)) {
        current = 0;
    }
    if (!_battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = 0;
    }
    
    // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    batt = (((uint16_t)roundf(_battery.voltage(instance) * 10.0f)) & BATT_VOLTAGE_LIMIT);
    // battery current draw in deciamps
    batt |= prep_number(roundf(current * 10.0f), 2, 1)<<BATT_CURRENT_OFFSET;
    // battery current drawn since power on in mAh (limit to 32767 (0x7FFF) since value is stored on 15 bits)
    batt |= ((consumed_mah < BATT_TOTALMAH_LIMIT) ? ((uint16_t)roundf(consumed_mah) & BATT_TOTALMAH_LIMIT) : BATT_TOTALMAH_LIMIT)<<BATT_TOTALMAH_OFFSET;
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
    ap_status |= (uint8_t)(gcs().simple_input_active())<<AP_SIMPLE_OFFSET;
    ap_status |= (uint8_t)(gcs().supersimple_input_active())<<AP_SSIMPLE_OFFSET;
    // is_flying flag
    ap_status |= (uint8_t)(AP_Notify::flags.flying) << AP_FLYING_OFFSET;
    // armed flag
    ap_status |= (uint8_t)(AP_Notify::flags.armed)<<AP_ARMED_OFFSET;
    // battery failsafe flag
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery)<<AP_BATT_FS_OFFSET;
    // bad ekf flag
    ap_status |= (uint8_t)(AP_Notify::flags.ekf_bad)<<AP_EKF_FS_OFFSET;
    // IMU temperature
    ap_status |= imu_temp << AP_IMU_TEMP_OFFSET;
    //hal.console->printf("flying=%d\n",AP_Notify::flags.flying);    
    //hal.console->printf("ap_status=%08X\n",ap_status);    
    return ap_status;
}

/*
 * prepare home position related data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_home(void)
{
    uint32_t home = 0;
    Location loc;
    Location home_loc;
    bool get_position;
    float _relative_home_altitude = 0;

    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        get_position = _ahrs.get_position(loc);
        home_loc = _ahrs.get_home();
    }

    if (get_position) {            
        // check home_loc is valid
        if (home_loc.lat != 0 || home_loc.lng != 0) {
            // distance between vehicle and home_loc in meters
            home = prep_number(roundf(home_loc.get_distance(loc)), 3, 2);
            // angle from front of vehicle to the direction of home_loc in 3 degree increments (just in case, limit to 127 (0x7F) since the value is stored on 7 bits)
            home |= (((uint8_t)roundf(loc.get_bearing_to(home_loc) * 0.00333f)) & HOME_BEARING_LIMIT)<<HOME_BEARING_OFFSET;
        }
        // altitude between vehicle and home_loc
        _relative_home_altitude = loc.alt;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            _relative_home_altitude -= home_loc.alt;
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
    float vspd = get_vspeed_ms();
    // vertical velocity in dm/s
    uint32_t velandyaw = prep_number(roundf(vspd * 10), 2, 1);
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
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
    const RangeFinder *_rng = RangeFinder::get_singleton();

    uint32_t attiandrng;
    AP_AHRS &_ahrs = AP::ahrs();
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
 * get vertical speed from ahrs, if not available fall back to baro climbrate, units is m/s
 * for FrSky D and SPort protocols
 */
float AP_Frsky_Telem::get_vspeed_ms(void)
{

    {// release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v;
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }

    auto &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}

/*
 * prepare altitude between vehicle and home location data
 * for FrSky D and SPort protocols
 */
void AP_Frsky_Telem::calc_nav_alt(void)
{
    _SPort_data.vario_vspd = (int32_t)(get_vspeed_ms()*100); //convert to cm/s
    
    Location loc;
    float current_height = 0; // in centimeters above home
    
    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    if (_ahrs.get_position(loc)) {
        current_height = loc.alt*0.01f;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            current_height -= _ahrs.get_home().alt*0.01f;
        }
    }

    _SPort_data.alt_nav_meters = (int16_t)current_height;
    _SPort_data.alt_nav_cm = (current_height - _SPort_data.alt_nav_meters) * 100;
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
        _SPort_data.latdddmm = lat;
        _SPort_data.latmmmm = (lat - _SPort_data.latdddmm) * 10000;
        _SPort_data.lat_ns = (loc.lat < 0) ? 'S' : 'N';

        lon = format_gps(fabsf(loc.lng/10000000.0f));
        _SPort_data.londddmm = lon;
        _SPort_data.lonmmmm = (lon - _SPort_data.londddmm) * 10000;
        _SPort_data.lon_ew = (loc.lng < 0) ? 'W' : 'E';

        alt = loc.alt * 0.01f;
        _SPort_data.alt_gps_meters = (int16_t)alt;
        _SPort_data.alt_gps_cm = (alt - _SPort_data.alt_gps_meters) * 100;

        speed = AP::gps().ground_speed();
        _SPort_data.speed_in_meter = speed;
        _SPort_data.speed_in_centimeter = (speed - _SPort_data.speed_in_meter) * 100;
    } else {
        _SPort_data.latdddmm = 0;
        _SPort_data.latmmmm = 0;
        _SPort_data.lat_ns = 0;
        _SPort_data.londddmm = 0;
        _SPort_data.lonmmmm = 0;
        _SPort_data.alt_gps_meters = 0;
        _SPort_data.alt_gps_cm = 0;
        _SPort_data.speed_in_meter = 0;
        _SPort_data.speed_in_centimeter = 0;
    }

    AP_AHRS &_ahrs = AP::ahrs();
    _SPort_data.yaw = (uint16_t)((_ahrs.yaw_sensor / 100) % 360); // heading in degree based on AHRS and not GPS    
}

/*
  fetch Sport data for an external transport, such as FPort
 */
bool AP_Frsky_Telem::_get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data)
{
    run_wfq_scheduler();
    if (!external_data.pending) {
        return false;
    }
    frame = external_data.frame;
    appid = external_data.appid;
    data = external_data.data;
    external_data.pending = false;
    return true;
}

/*
  fetch Sport data for an external transport, such as FPort
 */
bool AP_Frsky_Telem::get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data)
{
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_Frsky_Telem object then try to allocate one
        new AP_Frsky_Telem(true);
        // initialize the passthrough scheduler
        if (singleton) {
            singleton->init();
        }
    }
    if (!singleton) {
        return false;
    }
    return singleton->_get_telem_data(frame, appid, data);
}

namespace AP {
    AP_Frsky_Telem *frsky_telem() {
        return AP_Frsky_Telem::get_singleton();
    }
};
