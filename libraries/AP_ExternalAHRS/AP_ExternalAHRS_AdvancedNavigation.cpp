/*
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
  support for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ADNAV_ENABLED
#include "AP_ExternalAHRS_AdvancedNavigation.h"
#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define AN_TIMEOUT 5000 //ms
#define AN_MAXIMUM_PACKET_PERIODS 50
#define AN_GNSS_PACKET_RATE 5
#define AN_AIRSPEED_AIDING_FREQUENCY 20

#define an_packet_pointer(packet) packet->header
#define an_packet_size(packet) (packet->length + AN_PACKET_HEADER_SIZE)*sizeof(uint8_t)
#define an_packet_crc(packet) ((packet->header[4]<<8) | packet->header[3])

extern const AP_HAL::HAL &hal;

/*
    Packet for requesting a Advanced Navigation Device to send its
    device information (ANPP Packet 3) a single time.
        0x9a-0xd1   - Header see ANPP Documentation for more info
        0x03        - Request Packet 3 (Device Info Packet)
*/
static const uint8_t request_an_info[] {0x9a, 0x01, 0x01, 0x93, 0xd1, 0x03};

/*
    Packet for requesting a Advanced Navigation Device to set its
    Packet Timer Period to 1000Hz
        0xa0-0x6c   - Header see ANPP Documentation for more info
        0x01        - True - Permanent effect
        0x01        - True - UTC Sync
        0xe8,0x03   - Packet Timer Period ms (uint16_t) (1000Hz)
*/
static const uint8_t timer_period_1000_hz[] {0xa0, 0xb4, 0x04, 0x3c, 0x6c, 0x01, 0x01, 0xe8, 0x03};


/*
 * Function to decode ANPP Packets from raw buffer in the decoder structure
 * Returns false for a buffer error
 */
int AP_ExternalAHRS_AdvancedNavigation_Decoder::decode_packet(uint8_t* out_buffer, size_t buf_size)
{
    uint16_t decode_iterator = 0;
    uint8_t header_lrc, length;
    uint16_t crc;

    // Iterate through buffer until no more headers could be in buffer
    while (decode_iterator + AN_PACKET_HEADER_SIZE <= _buffer_length) {
        header_lrc = _buffer[decode_iterator++];

        // Is this the start of a valid header?
        if (header_lrc == calculate_header_lrc(&_buffer[decode_iterator])) {
            decode_iterator++; // skip ID as it is unused (-Werror=unused-but-set-variable)
            length = _buffer[decode_iterator++];
            crc = _buffer[decode_iterator++];
            crc |= _buffer[decode_iterator++] << 8;

            // If the packet length is over the edge of the buffer
            if (decode_iterator + length > _buffer_length) {
                decode_iterator -= AN_PACKET_HEADER_SIZE;
                break;
            }

            // If the crc matches then a valid packet has been identified.
            if (crc == crc16_ccitt(&_buffer[decode_iterator], length, 0xFFFF)) {

                // Protect from buffer overflow.
                if ((size_t) (length + AN_PACKET_HEADER_SIZE) > buf_size) {
                    return false;
                }

                // Save the data into the output buffer.
                memcpy(out_buffer, &_buffer[decode_iterator - AN_PACKET_HEADER_SIZE], AN_PACKET_HEADER_SIZE + length * sizeof(uint8_t));

                decode_iterator += length;
                _packets_decoded++;
                _bytes_decoded += length + AN_PACKET_HEADER_SIZE;
                break;
            } else { // Invalid packet for given header
                decode_iterator -= (AN_PACKET_HEADER_SIZE - 1);
                _crc_errors++;
                _bytes_discarded++;
            }
        } else { // Invalid Header
            _lrc_errors++;
            _bytes_discarded++;
        }
    }
    // If there is still buffer to be decoded.
    if (decode_iterator < _buffer_length) {
        // Ensure that the iterator isn't invalid
        if (decode_iterator > 0) {
            // move the unparsed memory to the beginning of the buffer.
            memmove(&_buffer[0], &_buffer[decode_iterator], (_buffer_length - decode_iterator) * sizeof(uint8_t));
            _buffer_length -= decode_iterator;
            _complete = false;
            return true;
        }
    } else {
        _buffer_length = 0;
    }

    _complete = true;
    return true;
}


// constructor
AP_ExternalAHRS_AdvancedNavigation::AP_ExternalAHRS_AdvancedNavigation(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    dal (AP::dal())
{
    auto &sm = AP::serialmanager();
    _uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS no UART");
        return;
    }

    _baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    _last_vel_sd = new AN_VELOCITY_STANDARD_DEVIATION;
    _last_satellites = new AN_SATELLITES;

    // don't offer IMU by default, at 50Hz it is too slow
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));


    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AdvancedNavigation::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    uint32_t tstart = AP_HAL::millis();

    while (!_last_device_info_pkt_ms)
    {
        const uint32_t tnow = AP_HAL::millis();
        if (tnow - tstart >= AN_TIMEOUT)
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Advanced Navigation Device Unresponsive");
            tstart = tnow;
        }
        hal.scheduler->delay(50);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised: %s", get_name());
}

void AP_ExternalAHRS_AdvancedNavigation::update()
{
    get_packets();
}

void AP_ExternalAHRS_AdvancedNavigation::update_thread(void)
{
    _uart->begin(_baudrate);

    while (true) {
        // Request data. If error occurs notify.
        if (!request_data()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Request Data Error");
        }

        // Sleep the scheduler
        hal.scheduler->delay_microseconds(1000);

        // Send Aiding data to the device
        if (option_is_set(AP_ExternalAHRS::OPTIONS::AN_ARSP_AID)) {
            if (!send_airspeed_aiding()) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ExternalAHRS: Unable to send airspeed aiding.");
            }
        }

        // Collect the requested packets from the UART manager
        if (!get_packets()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Error Receiving Packets");
        }
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::get_packets(void)
{
    if (_uart == nullptr) {
        return false;
    }
    // Ensure that this section is completed in a single thread.
    WITH_SEMAPHORE(_sem);

    // guard for no data on uart.
    if (_uart->available() <= 0) {
        return true;
    }

    // receive packets from the UART into the decoder
    _decoder.receive(_uart->read(_decoder.pointer(), _decoder.size()));

    if (_decoder.bytes_received() > 0) {
        // Decode all packets in the buffer
        while (!_decoder.is_complete()) {
            // decode a packet into the message buffer
            if (!_decoder.decode_packet(_msg.buffer, sizeof(_msg.buffer))) {
                return false;
            }
            handle_packet();
        }
    }
    return true;
}

bool AP_ExternalAHRS_AdvancedNavigation::request_data(void)
{

    // Update device info every 20 secs
    if ((AP_HAL::millis() - _last_device_info_pkt_ms > 20000) || (_last_device_info_pkt_ms == 0)) {
        if (_uart->txspace() < sizeof(request_an_info)) {
            return false;
        }
        _uart->write(request_an_info, sizeof(request_an_info));
    }

    // Don't send a packet request unless the device is healthy
    if (_current_rate != get_rate() && healthy()) {
        if (!sendPacketRequest()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Failure to send packet request");
        }
    }

    // check for updates to environment that require INS filter updates
    if (_fly_forward != in_fly_forward() || _gnss_disable != gnss_is_disabled()){
        _fly_forward = in_fly_forward();
        _gnss_disable = gnss_is_disabled();

        // Select AdNav vehicle for current flight mode.
        vehicle_type_e vehicle_type = vehicle_type_3d_aircraft;
        if (_fly_forward == true) {
            vehicle_type = vehicle_type_fixed_wing_plane;
        }

        set_filter_options(!_gnss_disable, vehicle_type);
    }
    return true;
}

int8_t AP_ExternalAHRS_AdvancedNavigation::get_port(void) const
{
    if (_uart == nullptr) {
        return -1;
    }
    return _port_num;
};

// Get model/type name
const char* AP_ExternalAHRS_AdvancedNavigation::get_name() const
{
    if ((AP_HAL::millis() - _last_pkt_ms) > AN_TIMEOUT) {
        static char buf[30];
        hal.util->snprintf(buf, 30, "AdNav: TIMEOUT... %8ums", (unsigned int) (AP_HAL::millis() - _last_pkt_ms));
        return buf;
    }

    if (_last_device_info_pkt_ms == 0) {
        return "AdNav No Connection...";
    }

    switch (_device_id) {
    case 0:
        return "Uninitialized Device ID";
        break;
    case device_id_spatial:
        return "AdNav Spatial";
        break;
    case device_id_orientus:
    case device_id_orientus_v3:
        return "AdNav Orientus";
        break;
    case device_id_spatial_fog:
        return "AdNav Spatial FOG";
        break;
    case device_id_spatial_dual:
        return "AdNav Spatial Dual";
        break;
    case device_id_ilu:
        return "AdNav Interface Logging Unit";
        break;
    case device_id_air_data_unit:
        return "AdNav Air Data Unit";
        break;
    case device_id_spatial_fog_dual:
        return "AdNav Spatial FOG Dual";
        break;
    case device_id_motus:
        return "AdNav Motus";
        break;
    case device_id_gnss_compass:
        return "AdNav GNSS Compass";
        break;
    case device_id_certus:
        return "AdNav Certus";
        break;
    case device_id_aries:
        return "AdNav Aries";
        break;
    case device_id_boreas_d90:
    case device_id_boreas_d90_fpga:
    case device_id_boreas_coil:
        return "AdNav Boreas";
        break;
    case device_id_certus_mini_a:
        return "AdNav Certus Mini A";
        break;
    case device_id_certus_mini_n:
        return "AdNav Certus Mini N";
        break;
    case device_id_certus_mini_d:
        return "AdNav Certus Mini D";
        break;
    default:
        return "Unknown AdNav Device ID";
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return ((now - _last_state_pkt_ms) < 500);
}

bool AP_ExternalAHRS_AdvancedNavigation::initialised(void) const
{
    if (get_gnss_capability()) {
        return _last_state_pkt_ms != 0 && _last_device_info_pkt_ms != 0 && _last_raw_gnss_pkt_ms !=0;
    } else {
        return _last_state_pkt_ms != 0 && _last_device_info_pkt_ms != 0;
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // Add failure messages
    if (AP_HAL::millis() - _last_pkt_ms > AN_TIMEOUT) {
        hal.util->snprintf(failure_msg, failure_msg_len, "DEVICE TIMEOUT Last Packet %8ums ago", (unsigned int) (AP_HAL::millis() - _last_pkt_ms));
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Device unhealthy");
        return false;
    }
    if (_device_status.system.b.gnss_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "GNSS Failure");
        return false;
    }
    if (_device_status.system.b.system_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "System Failure");
        return false;
    }
    if (_device_status.system.b.accelerometer_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Accelerometer Failure");
        return false;
    }
    if (_device_status.system.b.gyroscope_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Gyroscope Failure");
        return false;
    }
    if (_device_status.system.b.magnetometer_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Magnetometer Failure");
        return false;
    }
    if (_device_status.system.b.pressure_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Barometer Failure");
        return false;
    }
    if ((_device_status.filter.b.gnss_fix_type < 1) && (get_gnss_capability())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "No GPS lock");
        return false;
    }
    if (!_device_status.filter.b.orientation_filter_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Orientation Filter Not Initialised");
        return false;
    }
    if (!_device_status.filter.b.ins_filter_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "INS Filter Not Initialised");
        return false;
    }
    if (!_device_status.filter.b.heading_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Heading Filter Not Initialised");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_AdvancedNavigation::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (_last_state_pkt_ms == 0) {
        return;
    }
    status.flags.initalized = true;
    if (!healthy()) {
        return;
    }

    status.flags.vert_pos = true;
    status.flags.attitude = true;
    status.flags.vert_vel = true;

    if (_device_status.filter.b.gnss_fix_type > gnss_fix_none) {
        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }

    if (_device_status.filter.b.gnss_fix_type > gnss_fix_2d) {
        status.flags.gps_quality_good = true;
    }

}

void AP_ExternalAHRS_AdvancedNavigation::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 5; // represents hz value data is posted at
    const float pos_gate = 5; // represents hz value data is posted at
    const float hgt_gate = 5; // represents hz value data is posted at
    const float mag_var =  5;
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       norm(_last_vel_sd->sd[0],
                                            _last_vel_sd->sd[1],
                                            _last_vel_sd->sd[2])/vel_gate,
                                       norm(_gnss_sd.x,
                                            _gnss_sd.y)/pos_gate,
                                       _gnss_sd.z/hgt_gate,
                                       mag_var,
                                       0,
                                       0);
}

/*
 * Function to request data packets from a Advanced Navigation Device at the current rate.
 */
bool AP_ExternalAHRS_AdvancedNavigation::sendPacketRequest()
{
    // Set the device to use a period timer of 1000Hz
    // See ANPP Packet Rates for more info
    if (_uart->txspace() < sizeof(timer_period_1000_hz)) {
        return false;
    }
    _uart->write(timer_period_1000_hz, sizeof(timer_period_1000_hz));

    // Update the current rate
    _current_rate = get_rate();

    const AN_PACKET_PERIODS periods{
        permanent: true,
        clear_existing_packet_periods: true,
        periods: {
            AN_PERIOD{
                id: packet_id_system_state,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: packet_id_velocity_standard_deviation,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: packet_id_raw_sensors,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: packet_id_raw_gnss,
                packet_period: (uint32_t) 1.0e3 / AN_GNSS_PACKET_RATE
            },
            AN_PERIOD{
                id: packet_id_satellites,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            }
        }
    };

    AN_PACKET packet;

    // load the AN_PACKETS_PERIOD Into the payload.
    packet.payload.packet_periods = periods;
    packet.update_checks(packet_id_packet_periods, packet.getPeriodsLength(periods));

    // Check for space in the tx buffer
    if (_uart->txspace() < packet.packet_size()) {
        return false;
    }
    _uart->write(packet.raw_pointer(), packet.packet_size());

    return true;
}

/*
 * Function that returns the gps capability of the connected AdNav device.
 */
bool AP_ExternalAHRS_AdvancedNavigation::get_gnss_capability(void) const
{
    switch (_device_id) {
    case device_id_orientus:
    case device_id_orientus_v3:
    case device_id_air_data_unit:
    case device_id_motus:
    case device_id_certus_mini_a:
        return false;
    default:
        return true;
    }
}

/*
 * Function that returns the barometric capability of the connected AdNav device.
 */
bool AP_ExternalAHRS_AdvancedNavigation::get_baro_capability(void) const
{
    switch (_device_id) {
    case device_id_air_data_unit:
    case device_id_orientus:
    case device_id_orientus_v3:
    case device_id_gnss_compass:
    case device_id_certus_mini_a:
        return false;
    case device_id_motus:
        // Motus versions prior to 2.3 didn't have a barometer enabled.
        if (_hardware_rev < 2300) {
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

bool AP_ExternalAHRS_AdvancedNavigation::set_filter_options(bool gnss_en, vehicle_type_e vehicle_type, bool permanent)
{

    AN_FILTER_OPTIONS options_packet;

    options_packet.permanent = permanent;
    options_packet.vehicle_type = vehicle_type;
    options_packet.internal_gnss_enabled = gnss_en;
    options_packet.magnetometers_enabled = true;
    options_packet.atmospheric_altitude_enabled = true;
    options_packet.velocity_heading_enabled = true;
    options_packet.reversing_detection_enabled = false;
    options_packet.motion_analysis_enabled = false;
    options_packet.automatic_magnetic_calibration_enabled = true;
    options_packet.dual_antenna_disabled = false;
    // set reserved packets to 0
    memset(options_packet.reserved, 0, sizeof(options_packet.reserved));

    return set_filter_options(options_packet);
}

bool AP_ExternalAHRS_AdvancedNavigation::set_filter_options(const AN_FILTER_OPTIONS options_packet)
{
    AN_PACKET packet;

    packet.payload.filter_options = options_packet;
    packet.update_checks(packet_id_filter_options, sizeof(packet.payload.filter_options));

    // Check for space in the tx buffer
    if (_uart->txspace() < packet.packet_size()) {
        return false;
    }
    _uart->write(packet.raw_pointer(), packet.packet_size());

    return true;
}

// Returns true on successful transmit of packet or if called prior to requirement of sending packet.
bool AP_ExternalAHRS_AdvancedNavigation::send_airspeed_aiding(){

    uint32_t now = AP_HAL::millis();
    const AP_DAL_Airspeed *arsp = dal.airspeed();
    const AP_DAL_Baro &bar = dal.baro();
    uint32_t period_airspeed_aiding_ms = 1000/AN_AIRSPEED_AIDING_FREQUENCY;
    if (now < _last_ext_air_data_sent_ms + period_airspeed_aiding_ms) {
        return true;
    }

    if (arsp == nullptr || !arsp->healthy() || !bar.healthy(bar.get_primary())) {
        return false;
    }

    AN_PACKET packet;
    float airspeed = arsp->get_airspeed();

    _last_ext_air_data_sent_ms = now;

    packet.payload.ext_air_data.true_airspeed = airspeed * dal.get_EAS2TAS();
    packet.payload.ext_air_data.airspeed_delay = (AP_HAL::millis() - arsp->last_update_ms()) / 1000;
    packet.payload.ext_air_data.airspeed_standard_deviation = get_airspeed_error(airspeed);
    packet.payload.ext_air_data.barometric_altitude =  bar.get_altitude();
    packet.payload.ext_air_data.baro_delay = (AP_HAL::millis() - bar.get_last_update()) / 1000;
    packet.payload.ext_air_data.barometric_standard_deviation = get_pressure_error();
    packet.payload.ext_air_data.flags.b.airspeed_set_valid = true;
    packet.payload.ext_air_data.flags.b.barometric_altitude_set_valid = true;
    packet.update_checks(packet_id_external_air_data, sizeof(packet.payload.ext_air_data));

    // Check for space in the tx buffer
    if (_uart->txspace() < packet.packet_size()) {
        return false;
    }
    _uart->write(packet.raw_pointer(), packet.packet_size());

    return true;
}

// Method to return estimate of airspeed error based upon the current airspeed given error at 20m/s.
float AP_ExternalAHRS_AdvancedNavigation::get_airspeed_error(float airspeed)
{
    float nominal_pressure = 0.5 * sq(airspeed);
    float adj_airspeed = sqrt(2 * nominal_pressure + get_pressure_error());
    return adj_airspeed - airspeed;
}

// Method to give an estimate of the pressure error from the parameter for airspeed error at 20m/s
float AP_ExternalAHRS_AdvancedNavigation::get_pressure_error(void)
{
    return (0.5*(sq(20+airspeed_err_20ms()))) - (0.5*sq(20));
}

void AP_ExternalAHRS_AdvancedNavigation::handle_packet()
{
    // get current time
    uint32_t now = AP_HAL::millis();
    _last_pkt_ms = now;

    // Update depending on received packet.
    switch (_msg.packet.id) {
    case packet_id_device_information: {
        _last_device_info_pkt_ms = now;
        _device_id = _msg.packet.payload.device_info.device_id;
        _hardware_rev = _msg.packet.payload.device_info.hardware_revision;
        break;
    }
    case packet_id_system_state: {
        _last_state_pkt_ms = now;
        // Save the status
        _device_status.system.r = _msg.packet.payload.system_state.status.system.r;
        _device_status.filter.r = _msg.packet.payload.system_state.status.filter.r;
        {
            WITH_SEMAPHORE(state.sem);

            state.accel = Vector3f{
                    _msg.packet.payload.system_state.body_acceleration[0],
                    _msg.packet.payload.system_state.body_acceleration[1],
                    _msg.packet.payload.system_state.body_acceleration[2]
            };

            state.gyro = Vector3f{
                _msg.packet.payload.system_state.angular_velocity[0],
                _msg.packet.payload.system_state.angular_velocity[1],
                _msg.packet.payload.system_state.angular_velocity[2]
            };

            state.have_velocity = true;
            state.velocity = Vector3f{
                _msg.packet.payload.system_state.velocity_ned[0],
                _msg.packet.payload.system_state.velocity_ned[1],
                _msg.packet.payload.system_state.velocity_ned[2]
            };

            if (get_gnss_capability()) {
                state.have_location = true;
                const double latlon_scale = degrees(1) * 1.0e7;
                state.location = Location{
                    (int32_t) (_msg.packet.payload.system_state.llh[0] * latlon_scale),
                    (int32_t) (_msg.packet.payload.system_state.llh[1] * latlon_scale),
                    (int32_t) (_msg.packet.payload.system_state.llh[2] *1.0e2),
                    Location::AltFrame::ABSOLUTE
                };
                if (!state.have_origin) {
                    state.origin = state.location;
                    state.have_origin = true;
                }
                state.last_location_update_us = AP_HAL::micros();
            }

            state.have_quaternion = true;
            state.quat.from_euler(
                _msg.packet.payload.system_state.rph[0],
                _msg.packet.payload.system_state.rph[1],
                _msg.packet.payload.system_state.rph[2]
            );
        }
        break;
    }
    case packet_id_velocity_standard_deviation: {
        // save packet to be used for external gps.
        *_last_vel_sd = _msg.packet.payload.velocity_standard_deviation;
        break;
    }

    case packet_id_raw_sensors: {
        AP_ExternalAHRS::ins_data_message_t ins{
            accel: Vector3f{
                    _msg.packet.payload.raw_sensors.accelerometers[0],
                    _msg.packet.payload.raw_sensors.accelerometers[1],
                    _msg.packet.payload.raw_sensors.accelerometers[2]
                },

            gyro: Vector3f{
                    _msg.packet.payload.raw_sensors.gyroscopes[0],
                    _msg.packet.payload.raw_sensors.gyroscopes[1],
                    _msg.packet.payload.raw_sensors.gyroscopes[2]
                },
            temperature: _msg.packet.payload.raw_sensors.imu_temperature
        };
        AP::ins().handle_external(ins);


#if AP_COMPASS_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::mag_data_message_t mag {
            field: Vector3f{
                    _msg.packet.payload.raw_sensors.magnetometers[0],
                    _msg.packet.payload.raw_sensors.magnetometers[1],
                    _msg.packet.payload.raw_sensors.magnetometers[2]
                },
        };
        AP::compass().handle_external(mag);
#endif
#if AP_BARO_EXTERNALAHRS_ENABLED
        if (get_baro_capability()) {
            AP_ExternalAHRS::baro_data_message_t baro{
                instance: 0,
            pressure_pa: _msg.packet.payload.raw_sensors.pressure,
            temperature: _msg.packet.payload.raw_sensors.pressure_temperature
            };
            AP::baro().handle_external(baro);
        };
#endif
    }
    break;

    case packet_id_raw_gnss: {
        // Save the standard deviations for status report
        _gnss_sd = Vector3f{
            _msg.packet.payload.raw_gnss.llh_standard_deviation[0],
            _msg.packet.payload.raw_gnss.llh_standard_deviation[1],
            _msg.packet.payload.raw_gnss.llh_standard_deviation[2]
        };

        AP_ExternalAHRS::gps_data_message_t gps;

        const uint32_t unix_sec = _msg.packet.payload.raw_gnss.unix_time;
        const uint32_t unix_usec = _msg.packet.payload.raw_gnss.unix_microseconds;
        uint8_t fix = 0;

        switch (_msg.packet.payload.raw_gnss.flags.b.fix_type) {
        case gnss_fix_none:
            fix = GPS_FIX_TYPE_NO_FIX;
            break;
        case gnss_fix_2d:
            fix = GPS_FIX_TYPE_2D_FIX;
            break;
        case gnss_fix_3d:
            fix = GPS_FIX_TYPE_3D_FIX;
            break;
        case gnss_fix_sbas:
        case gnss_fix_differential:
            fix = GPS_FIX_TYPE_DGPS;
            break;
        case gnss_fix_omnistar:
            fix = GPS_FIX_TYPE_PPP;
            break;
        case gnss_fix_rtk_float:
            fix = GPS_FIX_TYPE_RTK_FLOAT;
            break;
        case gnss_fix_rtk_fixed:
            fix = GPS_FIX_TYPE_RTK_FIXED;
            break;
        default:
            break;
        }

        const uint32_t leapseconds = 18U;
        const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds;
        const uint32_t epoch_seconds = unix_sec - epoch;
        gps.gps_week = epoch_seconds / AP_SEC_PER_WEEK;
        const uint32_t t_ms = unix_usec / 1000U;
        // round time to nearest 200ms
        gps.ms_tow = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);

        gps.fix_type = fix;
        gps.satellites_in_view = (uint8_t) (_last_satellites->beidou_satellites + _last_satellites->galileo_satellites
                                            + _last_satellites->glonass_satellites + _last_satellites->gps_satellites + _last_satellites->sbas_satellites);

        gps.horizontal_pos_accuracy = (float) norm(_msg.packet.payload.raw_gnss.llh_standard_deviation[0], _msg.packet.payload.raw_gnss.llh_standard_deviation[1]);
        gps.vertical_pos_accuracy = _msg.packet.payload.raw_gnss.llh_standard_deviation[2];

        gps.hdop = _last_satellites->hdop;
        gps.vdop = _last_satellites->vdop;

        gps.latitude = (int32_t) (degrees(_msg.packet.payload.raw_gnss.llh[0]) * 1.0e7);
        gps.longitude = (int32_t) (degrees(_msg.packet.payload.raw_gnss.llh[1]) * 1.0e7);
        gps.msl_altitude = (int32_t) (_msg.packet.payload.raw_gnss.llh[2] * 1.0e2);

        gps.ned_vel_north = _msg.packet.payload.raw_gnss.velocity_ned[0];
        gps.ned_vel_east = _msg.packet.payload.raw_gnss.velocity_ned[1];
        gps.ned_vel_down = _msg.packet.payload.raw_gnss.velocity_ned[2];

        gps.horizontal_vel_accuracy = (float) norm(
                                          _last_vel_sd->sd[0],
                                          _last_vel_sd->sd[1],
                                          _last_vel_sd->sd[2]
                                      );

        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(gps, instance);
        }
    }
    break;

    case packet_id_satellites: {
        // save packet to be used for external gps.
        *_last_satellites = _msg.packet.payload.satellites;
    }
    break;

    default: {

    }
    break;
    }
}

#endif // AP_EXTERNAL_AHRS_ADNAV_ENABLED

