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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_Common/Location.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include "AP_Frsky_Telem.h"
#include "AP_Frsky_Parameters.h"

#include <stdio.h>
#include <math.h>

#define SPORT_TX_PACKET_DUPLICATES          1   // number of duplicates packets we send (fport only)
#define SPORT_RX_PACKET_DISCARD_DUPLICATE   1   // should we discard duplicate incoming packets

extern const AP_HAL::HAL& hal;

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
using namespace MAVLITE;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

AP_Frsky_Telem *AP_Frsky_Telem::singleton;

AP_Frsky_Telem::AP_Frsky_Telem(bool _external_data) : AP_RCTelemetry(TIME_SLOT_MAX),
    use_external_data(_external_data)
{
    singleton = this;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    _frsky_parameters = &AP::vehicle()->frsky_parameters;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
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
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    set_scheduler_entry(MAV, 35, 25);           // mavlite
    // initialize sport sensor IDs
    set_sport_sensor_id(_frsky_parameters->_uplink_id, _SPort_bidir.uplink_sensor_id);
    set_sport_sensor_id(_frsky_parameters->_dnlink1_id, _SPort_bidir.downlink1_sensor_id);
    set_sport_sensor_id(_frsky_parameters->_dnlink2_id, _SPort_bidir.downlink2_sensor_id);
    // initialize sport
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Telem::process_sport_rx_queue, void));
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
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

/*
    dinamically change scheduler priorities based on queue sizes
 */
void AP_Frsky_Telem::adjust_packet_weight(bool queue_empty)
{
    /*
        When queues are empty set a low priority (high weight), when queues
        are not empty set a higher priority (low weight) based on the following
        relative priority order: mavlite > status text > attitude.
     */
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!_SPort_bidir.tx_packet_buffer.is_empty()) {
        _scheduler.packet_weight[MAV] = 30;        // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;     // messages
            _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;   // messages
            _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
        }
    } else {
        _scheduler.packet_weight[MAV] = 5000;      // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;     // messages
            _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;   // messages
            _scheduler.packet_weight[ATTITUDE] = 45;     // attitude
        }
    }
#else   //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!queue_empty) {
        _scheduler.packet_weight[TEXT] = 45;     // messages
        _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
    } else {
        _scheduler.packet_weight[TEXT] = 5000;   // messages
        _scheduler.packet_weight[ATTITUDE] = 45;     // attitude
    }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}

// WFQ scheduler
bool AP_Frsky_Telem::is_packet_ready(uint8_t idx, bool queue_empty)
{
    bool packet_ready = false;
    switch (idx) {
        case TEXT:
            packet_ready = !queue_empty;
            break;
        case GPS_LAT:
        case GPS_LON:
            // force gps coords to use sensor 0x1B, always send when used with external data
            packet_ready = use_external_data || (_passthrough.new_byte == SENSOR_ID_27);
            break;
        case AP_STATUS:
            packet_ready = gcs().vehicle_initialised();
            break;
        case BATT_2:
            packet_ready = AP::battery().num_instances() > 1;
            break;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        case MAV:
            packet_ready = !_SPort_bidir.tx_packet_buffer.is_empty();
            break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
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
            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
            _passthrough.gps_lng_sample = calc_gps_latlng(_passthrough.send_latitude);
            // force the scheduler to select GPS lon as packet that's been waiting the most
            // this guarantees that lat and lon packets are sent as consecutive packets
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
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        case MAV: // mavlite
            process_sport_tx_queue();
            break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL            
    }
}

/*
 * true if we need to respond to the last polling byte
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_Telem::is_passthrough_byte()
{
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    return _passthrough.new_byte == SENSOR_ID_27 || _passthrough.new_byte == _SPort_bidir.downlink1_sensor_id || _passthrough.new_byte == _SPort_bidir.downlink2_sensor_id ;
#else   //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    return _passthrough.new_byte == SENSOR_ID_27;
#endif  //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
bool AP_Frsky_Telem::parse_sport_rx_data() 
{
    return FRSky::parse_sport_telemetry_data(_passthrough.new_byte, _SPort_bidir.parse_state);
}
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

/*
 * send telemetry data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort_Passthrough(void)
{
    const uint16_t numc = MIN(_port->available(), 1024U);

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }
    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    for (uint16_t i = 0; i < numc; i++) {
        prev_byte = _passthrough.new_byte;
        _passthrough.new_byte = _port->read();
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        if (parse_sport_rx_data()) {
            queue_sport_rx_packet();
        }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    }
    // check if we should respond to this polling byte
    if (prev_byte == FRAME_HEAD && is_passthrough_byte()) {
        run_wfq_scheduler();
    }
}

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_Telem::send_SPort(void)
{
    const uint16_t numc = MIN(_port->available(), 1024U);

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

    for (uint16_t i = 0; i < numc; i++) {
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
                            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
                            break;
                        case 1:
                            send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
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

    FRSky::send_sport_frame(_port, frame, appid, data);
}

/*
 * send 2 bytes of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  AP_Frsky_Telem::send_uint16(uint16_t appid, uint16_t data)
{
    FRSky::send_uint16(_port, appid, data);
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

        for (uint8_t i = 0; i < 4 && _msg_chunk.char_index < sizeof(_statustext.next.text); i++) {
            character = _statustext.next.text[_msg_chunk.char_index++];

            if (!character) {
                break;
            }

            _msg_chunk.chunk |= character << (3-i) * 8;
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
        // with 3 or more extra FrSky sensors on the bus
        // send messages only once
        extra_chunks = 0;
    } else if (_scheduler.avg_packet_rate < 30) {
        // with 1 or 2 extra FrSky sensors on the bus
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
    uint8_t last_param = AP::battery().num_instances() > 1 ? BATT_CAPACITY_2 : BATT_CAPACITY_1;

    // cycle through paramIDs
    if (_passthrough.paramID >= last_param) {
        _passthrough.paramID = 0;
    }

    _passthrough.paramID++;
    switch (_passthrough.paramID) {
        case FRAME_TYPE:
            param = gcs().frame_type(); // see MAV_TYPE in Mavlink definition file common.h
            break;
        case BATT_FS_VOLTAGE:           // was used to send the battery failsafe voltage, lend slot to next param
        case BATT_FS_CAPACITY:          // was used to send the battery failsafe capacity in mAh, lend slot to next param
        case BATT_CAPACITY_1:
            _passthrough.paramID = 4;
            param = (uint32_t)roundf(_battery.pack_capacity_mah(0)); // battery pack capacity in mAh
            break;
        case BATT_CAPACITY_2:
            param = (uint32_t)roundf(_battery.pack_capacity_mah(1)); // battery pack capacity in mAh
            break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    param = (_passthrough.paramID << PARAM_ID_OFFSET) | (param & PARAM_VALUE_LIMIT);

    return param;
}

/*
 * prepare gps latitude/longitude data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_latlng(bool &send_latitude)
{
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if (send_latitude == true) {
        send_latitude = false;
        if (loc.lat < 0) {
            return ((labs(loc.lat)/100)*6) | 0x40000000;
        } else {
            return ((labs(loc.lat)/100)*6);
        }
    } else {
        send_latitude = true;
        if (loc.lng < 0) {
            return ((labs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            return ((labs(loc.lng)/100)*6) | 0x80000000;
        }
    }
}

/*
 * prepare gps status data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_Telem::calc_gps_status(void)
{
    const AP_GPS &gps = AP::gps();
    // number of GPS satellites visible (limit to 15 (0xF) since the value is stored on 4 bits)
    uint32_t gps_status = (gps.num_sats() < GPS_SATS_LIMIT) ? gps.num_sats() : GPS_SATS_LIMIT;
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

    float current, consumed_mah;
    if (!_battery.current_amps(current, instance)) {
        current = 0;
    }
    if (!_battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = 0;
    }

    // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    uint32_t batt = (((uint16_t)roundf(_battery.voltage(instance) * 10.0f)) & BATT_VOLTAGE_LIMIT);
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
    // IMU temperature: offset -19, 0 means temp =< 19°, 63 means temp => 82°
    uint8_t imu_temp = (uint8_t) roundf(constrain_float(AP::ins().get_temperature(0), AP_IMU_TEMP_MIN, AP_IMU_TEMP_MAX) - AP_IMU_TEMP_MIN);

    // control/flight mode number (limit to 31 (0x1F) since the value is stored on 5 bits)
    uint32_t ap_status = (uint8_t)((gcs().custom_mode()+1) & AP_CONTROL_MODE_LIMIT);
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
    bool got_position = false;
    float _relative_home_altitude = 0;

    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        got_position = _ahrs.get_position(loc);
        home_loc = _ahrs.get_home();
    }

    if (got_position) {
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

    AP_AHRS &_ahrs = AP::ahrs();
    // roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    uint32_t attiandrng = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ATTIANDRNG_ROLL_LIMIT);
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

    {
        // release semaphore as soon as possible
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
    if (AP::gps().status() >= 3) {
        const Location &loc = AP::gps().location(); //get gps instance 0
        float lat = format_gps(fabsf(loc.lat/10000000.0f));
        _SPort_data.latdddmm = lat;
        _SPort_data.latmmmm = (lat - _SPort_data.latdddmm) * 10000;
        _SPort_data.lat_ns = (loc.lat < 0) ? 'S' : 'N';

        float lon = format_gps(fabsf(loc.lng/10000000.0f));
        _SPort_data.londddmm = lon;
        _SPort_data.lonmmmm = (lon - _SPort_data.londddmm) * 10000;
        _SPort_data.lon_ew = (loc.lng < 0) ? 'W' : 'E';

        float alt = loc.alt * 0.01f;
        _SPort_data.alt_gps_meters = (int16_t)alt;
        _SPort_data.alt_gps_cm = (alt - _SPort_data.alt_gps_meters) * 100;

        float speed = AP::gps().ground_speed();
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
    if (singleton == nullptr) {
        return false;
    }
    return singleton->_get_telem_data(frame, appid, data);
}

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
/*
  allow external transports (e.g. FPort), to supply telemetry data
 */
bool AP_Frsky_Telem::_set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
{
    // queue only Uplink packets
    if (frame == SPORT_UPLINK_FRAME || frame == SPORT_UPLINK_FRAME_RW) {
        const FRSky::sport_packet_t sp {
            0x0D,
            frame,
            appid,
            data
        };

        _SPort_bidir.rx_packet_buffer.push_force(sp);
        return true;
    }
    return false;
}

/*
  allow external transports (e.g. FPort), to supply telemetry data
 */
bool AP_Frsky_Telem::set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
{
    if (!singleton && !hal.util->get_soft_armed()) {
        // if telem data is requested when we are disarmed and don't
        // yet have a AP_Frsky_Telem object then try to allocate one
        new AP_Frsky_Telem(true);
        if (singleton) {
            singleton->init();
        }
    }
    if (singleton == nullptr) {
        return false;
    }
    return singleton->_set_telem_data(frame, appid, data);
}


/*
 * Queue uplink packets in the sport rx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::queue_sport_rx_packet()
{
    if (!FRSky::should_process_sport_packet(_SPort_bidir.parse_state.telemetry_rx_buffer, _SPort_bidir.last_packet)) {
        return;
    }

    // queue only Uplink packets
    if (_SPort_bidir.parse_state.telemetry_rx_buffer[0] == _SPort_bidir.uplink_sensor_id && _SPort_bidir.parse_state.telemetry_rx_buffer[1] == SPORT_UPLINK_FRAME) {
        const FRSky::sport_packet_t sp {
            _SPort_bidir.parse_state.telemetry_rx_buffer[0],
            _SPort_bidir.parse_state.telemetry_rx_buffer[1],
            le16toh_ptr(&_SPort_bidir.parse_state.telemetry_rx_buffer[2]),
            le32toh_ptr(&_SPort_bidir.parse_state.telemetry_rx_buffer[4])
        };

        _SPort_bidir.rx_packet_buffer.push_force(sp);
    }
}

/*
 * Process the sport rx queue
 * Extract up to 1 mavlite message from the queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::process_sport_rx_queue()
{
    FRSky::sport_packet_t packet;
    uint8_t loop_count = 0; // prevent looping forever
    while (_SPort_bidir.rx_packet_buffer.pop(packet) && loop_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {
        for (uint8_t i=0; i<6; i++) {
            mavlite_rx_parse(packet.raw[i+2],i, _mavlite.rx_message, _mavlite.rx_status);
        }

        if (_mavlite.rx_status.parse_state == MAVLITE::ParseState::MESSAGE_RECEIVED) {
            mavlite_process_message(_mavlite.rx_message);
            break; // process only 1 mavlite message each call
        }
    }
}

/*
 * Process the sport tx queue
 * pop and send 1 sport packet
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::process_sport_tx_queue()
{
    FRSky::sport_packet_t packet;

    if (!_SPort_bidir.tx_packet_buffer.peek(packet)) {
        return;
    }

    // when using fport repeat each packet to account for
    // fport packet loss (around 15%)
    if (!use_external_data || _SPort_bidir.tx_packet_duplicates++ == SPORT_TX_PACKET_DUPLICATES) {
        _SPort_bidir.tx_packet_buffer.pop();
        _SPort_bidir.tx_packet_duplicates = 0;
    }
    send_sport_frame(SPORT_DOWNLINK_FRAME, packet.appid, packet.data);
}


/*
 * Handle the COMMAND_LONG mavlite message
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::mavlite_handle_command_long(const mavlite_message_t &rxmsg)
{
    mavlink_command_long_t mav_command_long {};

    uint8_t cmd_options;
    float params[7] {};

    mavlite_msg_get_uint16(mav_command_long.command, rxmsg, 0);
    mavlite_msg_get_uint8(cmd_options, rxmsg, 2);
    uint8_t param_count = bit8_unpack(cmd_options, 3, 0);               // first 3 bits
    mav_command_long.confirmation = bit8_unpack(cmd_options, 5, 3);     // last 5 bits

    for (uint8_t cmd_idx=0; cmd_idx<param_count; cmd_idx++) {
        mavlite_msg_get_float(params[cmd_idx], rxmsg, 3+(4*cmd_idx));   // base offset is 3, relative offset is 4*cmd_idx
    }

    mav_command_long.param1 = params[0];
    mav_command_long.param2 = params[1];
    mav_command_long.param3 = params[2];
    mav_command_long.param4 = params[3];
    mav_command_long.param5 = params[4];
    mav_command_long.param6 = params[5];
    mav_command_long.param7 = params[6];

    MAV_RESULT mav_result = MAV_RESULT_FAILED;
    // filter allowed commands
    switch (mav_command_long.command) {
        //case MAV_CMD_ACCELCAL_VEHICLE_POS:
        case MAV_CMD_DO_SET_MODE:
            if (AP::vehicle()->set_mode(mav_command_long.param1, ModeReason::FRSKY_COMMAND)) {
                mav_result = MAV_RESULT_ACCEPTED;
            }
            break;
        //case MAV_CMD_DO_SET_HOME:
        case MAV_CMD_DO_FENCE_ENABLE:
            mav_result = mavlite_handle_command_do_fence_enable((uint16_t)mav_command_long.param1);
            break;
        //case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        //case MAV_CMD_DO_START_MAG_CAL:
        //case MAV_CMD_DO_ACCEPT_MAG_CAL:
        //case MAV_CMD_DO_CANCEL_MAG_CAL:
        //case MAV_CMD_START_RX_PAIR:
        //case MAV_CMD_DO_DIGICAM_CONFIGURE:
        //case MAV_CMD_DO_DIGICAM_CONTROL:
        //case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        //case MAV_CMD_DO_GRIPPER:
        //case MAV_CMD_DO_MOUNT_CONFIGURE:
        //case MAV_CMD_DO_MOUNT_CONTROL:
        //case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        //case MAV_CMD_DO_SET_ROI_SYSID:
        //case MAV_CMD_DO_SET_ROI_LOCATION:
        //case MAV_CMD_DO_SET_ROI:
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (is_equal(mav_command_long.param1,0.0f) && is_equal(mav_command_long.param2,0.0f) && is_equal(mav_command_long.param3,1.0f)) {
                mav_result = mavlite_handle_command_preflight_calibration_baro();
            }
            break;
        //case MAV_CMD_BATTERY_RESET:
        //case MAV_CMD_PREFLIGHT_UAVCAN:
        //case MAV_CMD_FLASH_BOOTLOADER:
        //case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
        //case MAV_CMD_GET_HOME_POSITION:
        //case MAV_CMD_PREFLIGHT_STORAGE:
        //case MAV_CMD_SET_MESSAGE_INTERVAL:
        //case MAV_CMD_GET_MESSAGE_INTERVAL:
        //case MAV_CMD_REQUEST_MESSAGE:
        //case MAV_CMD_DO_SET_SERVO:
        //case MAV_CMD_DO_REPEAT_SERVO:
        //case MAV_CMD_DO_SET_RELAY:
        //case MAV_CMD_DO_REPEAT_RELAY:
        //case MAV_CMD_DO_FLIGHTTERMINATION:
        //case MAV_CMD_COMPONENT_ARM_DISARM:
        //case MAV_CMD_FIXED_MAG_CAL_YAW:
        default:
            mav_result = MAV_RESULT_UNSUPPORTED;
            break;
    }
    mavlite_init(_mavlite.tx_message, _mavlite.tx_status);
    _mavlite.tx_message.msgid = MAVLINK_MSG_ID_COMMAND_ACK;
    mavlite_msg_set_uint16(_mavlite.tx_message, mav_command_long.command, 0);
    mavlite_msg_set_uint8(_mavlite.tx_message, (uint8_t)mav_result, 2);
    mavlite_send_message(_mavlite.tx_message, _mavlite.tx_status);
}

MAV_RESULT AP_Frsky_Telem::mavlite_handle_command_preflight_calibration_baro()
{
    if (AP_Notify::flags.armed) {
        return MAV_RESULT_DENIED;
    }
    // fast barometer calibration
    gcs().send_text(MAV_SEVERITY_INFO, "Updating barometer calibration");
    AP::baro().update_calibration();
    gcs().send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");

    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr) {
        airspeed->calibrate(false);
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Frsky_Telem::mavlite_handle_command_do_fence_enable(uint16_t param1)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    switch (param1) {
        case 0:
            fence->enable(false);
            return MAV_RESULT_ACCEPTED;
        case 1:
            fence->enable(true);
            return MAV_RESULT_ACCEPTED;
        default:
            return MAV_RESULT_FAILED;
    }
}

/*
 * Handle the PARAM_REQUEST_READ mavlite message
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::mavlite_handle_param_request_read(const mavlite_message_t &rxmsg)
{
    float param_value;
    char param_name[AP_MAX_NAME_SIZE+1];
    mavlite_msg_get_string(param_name, rxmsg, 0);
    // find existing param
    if (!AP_Param::get(param_name,param_value)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Param read failed (%s)", param_name);
        return;
    }
    mavlite_init(_mavlite.tx_message, _mavlite.tx_status);
    _mavlite.tx_message.msgid = MAVLINK_MSG_ID_PARAM_VALUE;
    mavlite_msg_set_float(_mavlite.tx_message, param_value, 0);
    mavlite_msg_set_string(_mavlite.tx_message, param_name, 4);
    mavlite_send_message(_mavlite.tx_message, _mavlite.tx_status);
}

/*
 * Handle the PARAM_SET mavlite message
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::mavlite_handle_param_set(const mavlite_message_t &rxmsg)
{
    float param_value;
    char param_name[AP_MAX_NAME_SIZE+1];
    // populate packet with mavlite payload
    mavlite_msg_get_float(param_value, rxmsg, 0);
    mavlite_msg_get_string(param_name, rxmsg, 4);
    // find existing param so we can get the old value
    enum ap_var_type var_type;
    // set parameter
    AP_Param *vp;
    uint16_t parameter_flags = 0;
    vp = AP_Param::find(param_name, &var_type, &parameter_flags);
    if (vp == nullptr || isnan(param_value) || isinf(param_value)) {
        return;
    }
    if ((parameter_flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) || vp->is_read_only()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Param write denied (%s)", param_name);
    } else if (!AP_Param::set_and_save(param_name, param_value)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Param write failed (%s)", param_name);
    }
    // let's read back the last value, either the readonly one or the updated one
    if (!AP_Param::get(param_name,param_value)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Param read failed (%s)", param_name);
        return;
    }
    mavlite_init(_mavlite.tx_message, _mavlite.tx_status);
    _mavlite.tx_message.msgid = MAVLINK_MSG_ID_PARAM_VALUE;
    mavlite_msg_set_float(_mavlite.tx_message, param_value, 0);
    mavlite_msg_set_string(_mavlite.tx_message, param_name, 4);
    mavlite_send_message(_mavlite.tx_message, _mavlite.tx_status);
}

/*
 * Process an incoming mavlite message
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_Telem::mavlite_process_message(const mavlite_message_t &rxmsg)
{
    switch (rxmsg.msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            mavlite_handle_param_request_read(rxmsg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            mavlite_handle_param_set(rxmsg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            mavlite_handle_command_long(rxmsg);
            break;
    }
}

/*
 * Send a mavlite message
 * Message is chunked in sport packets pushed in the tx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_Telem::mavlite_send_message(mavlite_message_t &txmsg, mavlite_status_t &status)
{
    // let's check if there's enough room to send it
    if (_SPort_bidir.tx_packet_buffer.space() < MAVLITE_MSG_SPORT_PACKETS_COUNT(txmsg.len)) {
        return false;
    }
    // prevent looping forever
    uint8_t packet_count = 0;
    while (status.parse_state != MAVLITE::ParseState::MESSAGE_RECEIVED && packet_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {
        FRSky::sport_packet_t packet {};
        for (uint8_t i=0; i<6; i++) {
            mavlite_tx_encode(packet.raw[i+2],i, txmsg, status);
        }
        if (status.parse_state == MAVLITE::ParseState::ERROR) {
            break;
        }
        // queue the packet for sending
        _SPort_bidir.tx_packet_buffer.push(packet);
    }
    status.parse_state = MAVLITE::ParseState::IDLE;
    return true;
}

/*
 * Utility method to apply constraints in changing sensor id values
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_Telem::set_sport_sensor_id(AP_Int8 param_idx, uint8_t &sensor)
{
    int8_t idx = param_idx.get();

    if (idx == -1) {
        // disable this sensor
        sensor = 0xFF;
        return true;
    }
    sensor = FRSky::get_sport_sensor_id(idx);
    return true;
}
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL


namespace AP
{
AP_Frsky_Telem *frsky_telem()
{
    return AP_Frsky_Telem::get_singleton();
}
};
