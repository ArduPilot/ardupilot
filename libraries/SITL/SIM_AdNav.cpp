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
  Simulation for a Advanced Navigation External AHRS System
*/

#include "SIM_AdNav.h"

using namespace SITL;

AdNav::AdNav() :
    SerialDevice::SerialDevice()
{
    an_decoder_initialise(&_an_decoder);
}
/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

void AdNav::update()
{
    if(!init_sitl_pointer()) {
        return;
    }

    // Receive Packets
    receive_packets();

    // Get Current Time
    uint32_t now = AP_HAL::micros();

    // If we need to send packets do so now
    if (now - _last_pkt_sent_us >= _packet_period_us) {
        _last_pkt_sent_us = now;
        send_state_pkt(); // Send ANPP.20
        send_vel_sd_pkt(); // Send ANPP.25
        send_raw_sensors_pkt(); // Send ANPP.28
    }

    if (now - _last_gnss_sent_us >= _gnss_period_us) {
        _last_gnss_sent_us = now;
        send_raw_gnss_pkt(); // Send ANPP.29
        send_sat_pkt(); // Send ANPP.30
    }
}

void AdNav::receive_packets(){
    ssize_t bytes_received = 0;
    an_packet_t* an_packet;

    if ((bytes_received = read_from_autopilot((char*) an_decoder_pointer(&_an_decoder), an_decoder_size(&_an_decoder))) > 0)
    {
        an_decoder_increment(&_an_decoder, bytes_received);

        while((an_packet = an_packet_decode(&_an_decoder)) != NULL)
        {
            switch(an_packet->id){
                // If we have been sent a periods packet
                case AN_PACKET_ID_PACKET_PERIODS:
                    send_acknowledge(an_packet_crc(an_packet), AN_PACKET_ID_PACKET_PERIODS);
                    packet_periods_packet_t packet_periods;
                    decode_packet_periods_packet(&packet_periods, an_packet);
                    _packet_period_us = packet_periods.packet_periods->period * 1.0e3;
                    break;
                case AN_PACKET_ID_REQUEST_PACKET:
                    // This is usually where any packet ID can be requested
                    // However only ID:3 Device info is ever requested by the module
                    // So this will return a Device info packet
                    send_device_info_pkt();
                    break;
                case AN_PACKET_ID_TIMER_PERIODS:
                    // do nothing
                    break;
                default:
                    printf("AN_DEVICE_SIM: Unknown AN Packet %u\n", unsigned(an_packet->id));
                    break;
            }
        }
    }
}

void AdNav::send_packet(an_packet_t* an_packet){
    // Encode the packet.
    an_packet_encode(an_packet);

    // Write the packet to the autopilot
    write_to_autopilot((char *) an_packet_pointer(an_packet), an_packet_size(an_packet));

    // Free the packet memory
    an_packet_free(&an_packet);
}

/*
    Function to send an Acknowledgement Packet.
        CRC - CRC of the packet being acknowledged
        ID  - ID of the packet being acknowledged
 */
void AdNav::send_acknowledge(uint16_t crc, uint8_t id){
    acknowledge_packet_t ack_pkt;

    memset(&ack_pkt, 0, sizeof(ack_pkt));
    ack_pkt.acknowledge_result = 0; // success
    ack_pkt.packet_crc = crc;
    ack_pkt.packet_id = id;

    send_packet(encode_acknowledge_packet(&ack_pkt));
}

/*
    Function to send a Device Info Packet.
 */
void AdNav::send_device_info_pkt(){
    device_information_packet_t dev_info;

    memset(&dev_info, 0, sizeof(dev_info));
    dev_info.device_id = 26; // Certus

    send_packet(encode_device_information_packet(&dev_info));
}

/*
    Function to send a System State Packet.
 */
void AdNav::send_state_pkt(){
    const auto &fdm = _sitl->state;
    
    struct timeval tv;
    simulation_timeval(&tv);

    system_state_packet_t system_state_packet;
    memset(&system_state_packet, 0, sizeof(system_state_packet));

    system_state_packet.unix_time_seconds = tv.tv_sec;
    system_state_packet.microseconds = tv.tv_usec;
    system_state_packet.system_status.r = false; // no AN Device Errors
    system_state_packet.filter_status.b.orientation_filter_initialised = true;
    system_state_packet.filter_status.b.ins_filter_initialised = true;
    system_state_packet.filter_status.b.heading_initialised = true;
    system_state_packet.filter_status.b.gnss_fix_type = 7; // RTK Fixed
    system_state_packet.latitude = fdm.latitude * DEG_TO_RAD_DOUBLE;
    system_state_packet.longitude = fdm.longitude * DEG_TO_RAD_DOUBLE;
    system_state_packet.height = fdm.altitude;
    system_state_packet.velocity[0] = fdm.speedN;
    system_state_packet.velocity[1] = fdm.speedE;
    system_state_packet.velocity[2] = fdm.speedD;
    system_state_packet.body_acceleration[0] = fdm.xAccel;
    system_state_packet.body_acceleration[1] = fdm.yAccel;
    system_state_packet.body_acceleration[2] = fdm.zAccel;
    system_state_packet.g_force = 1; // Unused
    system_state_packet.orientation[0] = radians(fdm.rollDeg);
    system_state_packet.orientation[1] = radians(fdm.pitchDeg);
    system_state_packet.orientation[2] = radians(fdm.yawDeg);
    system_state_packet.angular_velocity[0] = radians(fdm.rollRate);
    system_state_packet.angular_velocity[1] = radians(fdm.pitchRate);
    system_state_packet.angular_velocity[2] = radians(fdm.yawRate);
    system_state_packet.standard_deviation[0] = 0.8;
    system_state_packet.standard_deviation[1] = 0.8;
    system_state_packet.standard_deviation[2] = 1.2;

    // fdm doesn't contain SD values for LLH

    send_packet(encode_system_state_packet(&system_state_packet));
}

/*
    Function to send a Velocity Standard Deviation Packet.
 */
void AdNav::send_vel_sd_pkt(){
    // FDM Does not contain any Velocity SD Information so send 0's instead.
    velocity_standard_deviation_packet_t vel_sd;
    memset(&vel_sd, 0, sizeof(vel_sd));
    send_packet(encode_velocity_standard_deviation_packet(&vel_sd));

}

/*
    Function to send a Raw Sensors Packet.
 */
void AdNav::send_raw_sensors_pkt(){
    const auto& fdm = _sitl->state;

    raw_sensors_packet_t raw_sensors;
    memset(&raw_sensors, 0, sizeof(raw_sensors));

    raw_sensors.accelerometers[0] = fdm.xAccel;
    raw_sensors.accelerometers[1] = fdm.yAccel;
    raw_sensors.accelerometers[2] = fdm.zAccel;
    raw_sensors.gyroscopes[0] = radians(fdm.rollRate);
    raw_sensors.gyroscopes[1] = radians(fdm.pitchRate);
    raw_sensors.gyroscopes[2] = radians(fdm.yawRate);
    raw_sensors.magnetometers[0] = fdm.bodyMagField[0];
    raw_sensors.magnetometers[1] = fdm.bodyMagField[1];
    raw_sensors.magnetometers[2] = fdm.bodyMagField[2];
    raw_sensors.imu_temperature = 25; //fixed
    
    float pressure, temp_k;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude+rand_float()*0.25, pressure, temp_k);
    raw_sensors.pressure = pressure;
    raw_sensors.pressure_temperature = KELVIN_TO_C(temp_k);

    send_packet(encode_raw_sensors_packet(&raw_sensors));
}

/*
    Function to send a Raw GNSS Packet.
 */
void AdNav::send_raw_gnss_pkt(){
    const auto& fdm = _sitl->state;

    raw_gnss_packet_t raw_gnss;
    memset(&raw_gnss, 0, sizeof(raw_gnss));

    struct timeval tv;
    simulation_timeval(&tv);

    raw_gnss.unix_time_seconds = tv.tv_sec;
    raw_gnss.microseconds = tv.tv_usec;
    raw_gnss.position[0] = fdm.latitude  * DEG_TO_RAD_DOUBLE;
    raw_gnss.position[1] = fdm.longitude  * DEG_TO_RAD_DOUBLE;
    raw_gnss.position[2] = fdm.altitude;
    raw_gnss.velocity[0] = fdm.speedN;
    raw_gnss.velocity[1] = fdm.speedE;
    raw_gnss.velocity[2] = fdm.speedD;
    raw_gnss.heading = radians(fdm.heading);
    raw_gnss.flags.b.heading_valid = 1;
    raw_gnss.flags.b.fix_type = 7; // rtk fixed
    raw_gnss.position_standard_deviation[0] = 0.8;
    raw_gnss.position_standard_deviation[1] = 0.8;
    raw_gnss.position_standard_deviation[2] = 1.2;

    send_packet(encode_raw_gnss_packet(&raw_gnss));
}

/*
    Function to send a Raw Satellites Packet.
 */
void AdNav::send_sat_pkt(){
    satellites_packet_t sat_pkt;
    memset(&sat_pkt, 0, sizeof(sat_pkt));

    // Values taken from a GNSS Compass in Newcastle AU.
    sat_pkt.hdop = 0.5;
    sat_pkt.vdop = 0.6;
    sat_pkt.beidou_satellites = 8;
    sat_pkt.galileo_satellites = 6;
    sat_pkt.gps_satellites = 7;
    sat_pkt.sbas_satellites = 3;

    send_packet(encode_satellites_packet(&sat_pkt));
}

/*
    Function to encode an Acknowledge Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet){
    an_packet_t* an_packet = an_packet_allocate(4, AN_PACKET_ID_ACKNOWLEDGE);
    if(an_packet != NULL){
        memcpy(&an_packet->data[0], &acknowledge_packet->packet_id, sizeof(uint8_t));
        memcpy(&an_packet->data[1], &acknowledge_packet->packet_crc, sizeof(uint16_t));
        memcpy(&an_packet->data[3], &acknowledge_packet->acknowledge_result, sizeof(uint8_t));
    }
    return an_packet;
}

/*
    Function to encode a Device Information Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_device_information_packet(device_information_packet_t* device_information_packet){
    an_packet_t* an_packet = an_packet_allocate(24, AN_PACKET_ID_DEVICE_INFO);
    if (an_packet != NULL)
    {
        memcpy(&an_packet->data[0], &device_information_packet->software_version, sizeof(uint32_t));
		memcpy(&an_packet->data[4], &device_information_packet->device_id, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &device_information_packet->hardware_revision, sizeof(uint32_t));
		memcpy(&an_packet->data[12], &device_information_packet->serial_number[0], 3 * sizeof(uint32_t));
    }
    return an_packet;
}

/*
    Function to encode a System State Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_system_state_packet(system_state_packet_t* system_state_packet){
    an_packet_t* an_packet = an_packet_allocate(100, AN_PACKET_ID_SYSTEM_STATE);
    if (an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &system_state_packet->system_status, sizeof(uint16_t));
		memcpy(&an_packet->data[2], &system_state_packet->filter_status, sizeof(uint16_t));
		memcpy(&an_packet->data[4], &system_state_packet->unix_time_seconds, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &system_state_packet->microseconds, sizeof(uint32_t));
		memcpy(&an_packet->data[12], &system_state_packet->latitude, sizeof(double));
		memcpy(&an_packet->data[20], &system_state_packet->longitude, sizeof(double));
		memcpy(&an_packet->data[28], &system_state_packet->height, sizeof(double));
		memcpy(&an_packet->data[36], &system_state_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[48], &system_state_packet->body_acceleration[0], 3 * sizeof(float));
		memcpy(&an_packet->data[60], &system_state_packet->g_force, sizeof(float));
		memcpy(&an_packet->data[64], &system_state_packet->orientation[0], 3 * sizeof(float));
		memcpy(&an_packet->data[76], &system_state_packet->angular_velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[88], &system_state_packet->standard_deviation[0], 3 * sizeof(float));
	}
    return an_packet;
}

/*
    Function to encode a Velocity Standard Deviation Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_velocity_standard_deviation_packet(velocity_standard_deviation_packet_t* velocity_standard_deviation_packet){
    an_packet_t* an_packet = an_packet_allocate(12, AN_PACKET_ID_VELOCITY_STANDARD_DEVIATION);
    if (an_packet != NULL)
    {
        memcpy(&an_packet->data[0], &velocity_standard_deviation_packet->standard_deviation[0], 3 * sizeof(float));
    }
    return an_packet;
}

/*
    Function to encode a Raw Sensors Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_raw_sensors_packet(raw_sensors_packet_t* raw_sensors_packet){
    an_packet_t* an_packet = an_packet_allocate(48, AN_PACKET_ID_RAW_SENSORS);
    if (an_packet != NULL)
    {
        memcpy(&an_packet->data[0], &raw_sensors_packet->accelerometers[0], 3 * sizeof(float));
		memcpy(&an_packet->data[12], &raw_sensors_packet->gyroscopes[0], 3 * sizeof(float));
		memcpy(&an_packet->data[24], &raw_sensors_packet->magnetometers[0], 3 * sizeof(float));
		memcpy(&an_packet->data[36], &raw_sensors_packet->imu_temperature, sizeof(float));
		memcpy(&an_packet->data[40], &raw_sensors_packet->pressure, sizeof(float));
		memcpy(&an_packet->data[44], &raw_sensors_packet->pressure_temperature, sizeof(float));
    }
    return an_packet;
}

/*
    Function to encode a Raw GNSS Packet into a an_packet.
 */
an_packet_t* AdNav::encode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet){
	an_packet_t* an_packet = an_packet_allocate(74, AN_PACKET_ID_RAW_GNSS);
	if (an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &raw_gnss_packet->unix_time_seconds, sizeof(uint32_t));
		memcpy(&an_packet->data[4], &raw_gnss_packet->microseconds, sizeof(uint32_t));
		memcpy(&an_packet->data[8], &raw_gnss_packet->position[0], 3 * sizeof(double));
		memcpy(&an_packet->data[32], &raw_gnss_packet->velocity[0], 3 * sizeof(float));
		memcpy(&an_packet->data[44], &raw_gnss_packet->position_standard_deviation[0], 3 * sizeof(float));
		memcpy(&an_packet->data[56], &raw_gnss_packet->tilt, sizeof(float));
		memcpy(&an_packet->data[60], &raw_gnss_packet->heading, sizeof(float));
		memcpy(&an_packet->data[64], &raw_gnss_packet->tilt_standard_deviation, sizeof(float));
		memcpy(&an_packet->data[68], &raw_gnss_packet->heading_standard_deviation, sizeof(float));
		memcpy(&an_packet->data[72], &raw_gnss_packet->flags.r, sizeof(uint16_t));
	}
	return an_packet;
}

/*
    Function to encode a Satellites Packet into an an_packet structure.
 */
an_packet_t* AdNav::encode_satellites_packet(satellites_packet_t* satellites_packet){
    an_packet_t* an_packet = an_packet_allocate(13, AN_PACKET_ID_SATELLITES);
    if (an_packet != NULL)
    {
        memcpy(&an_packet->data[0], &satellites_packet->hdop, sizeof(float));
		memcpy(&an_packet->data[4], &satellites_packet->vdop, sizeof(float));
		memcpy(&an_packet->data[8], &satellites_packet->gps_satellites, 5 * sizeof(uint8_t));
    }
    return an_packet;
}

/*
    Function to decode a Packet Periods Packet into an an_packet structure.
 */
int AdNav::decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet){
	if(an_packet->id == AN_PACKET_ID_PACKET_PERIODS && (an_packet->length - 2) % 5 == 0)
	{
		int i;
		int packet_periods_count = (an_packet->length - 2) / 5;
		packet_periods_packet->permanent = an_packet->data[0];
		packet_periods_packet->clear_existing_packets = an_packet->data[1];
		for(i = 0; i < AN_MAXIMUM_PACKET_PERIODS; i++)
		{
			if(i < packet_periods_count)
			{
				packet_periods_packet->packet_periods[i].packet_id = an_packet->data[2 + 5 * i];
				memcpy(&packet_periods_packet->packet_periods[i].period, &an_packet->data[2 + 5 * i + 1], sizeof(uint32_t));
			}
			else memset(&packet_periods_packet->packet_periods[i], 0, sizeof(packet_period_t));
		}
		return 0;
	}
	else return 1;
}

/*
 * Function to calculate a 4 byte LRC
 */
uint8_t AdNav::calculate_header_lrc(uint8_t* data)
{
    return ((data[0] + data[1] + data[2] + data[3]) ^ 0xFF) + 1;
}

/*
 * Function to dynamically allocate an an_packet
 */
an_packet_t* AdNav::an_packet_allocate(uint8_t length, uint8_t id)
{
    an_packet_t* an_packet = (an_packet_t*) malloc(sizeof(an_packet_t) + length * sizeof(uint8_t));
    if (an_packet != NULL) {
        an_packet->id = id;
        an_packet->length = length;
    }
    return an_packet;
}

/*
 * Function to free an an_packet
 */
void AdNav::an_packet_free(an_packet_t** an_packet)
{
    free(*an_packet);
    *an_packet = NULL;
}

/*
 * Initialise the decoder
 */
void AdNav::an_decoder_initialise(an_decoder_t* an_decoder)
{
    an_decoder->buffer_length = 0;
    an_decoder->packets_decoded = 0;
    an_decoder->bytes_decoded = 0;
    an_decoder->bytes_discarded = 0;
    an_decoder->lrc_errors = 0;
    an_decoder->crc_errors = 0;
}

/*
 * Function to decode an_packets from raw data
 * Returns a pointer to the packet decoded or NULL if no packet was decoded
 */
an_packet_t* AdNav::an_packet_decode(an_decoder_t* an_decoder)
{
    uint16_t decode_iterator = 0;
    an_packet_t* an_packet = NULL;
    uint8_t header_lrc, id, length;
    uint16_t crc;

    while (decode_iterator + AN_PACKET_HEADER_SIZE <= an_decoder->buffer_length) {
        header_lrc = an_decoder->buffer[decode_iterator++];
        if (header_lrc == calculate_header_lrc(&an_decoder->buffer[decode_iterator])) {
            id = an_decoder->buffer[decode_iterator++];
            length = an_decoder->buffer[decode_iterator++];
            crc = an_decoder->buffer[decode_iterator++];
            crc |= an_decoder->buffer[decode_iterator++] << 8;

            if (decode_iterator + length > an_decoder->buffer_length) {
                decode_iterator -= AN_PACKET_HEADER_SIZE;
                break;
            }

            if (crc == crc16_ccitt(&an_decoder->buffer[decode_iterator], length, 0xFFFF)) {
                an_packet = an_packet_allocate(length, id);
                if (an_packet != NULL) {
                    memcpy(an_packet->header, &an_decoder->buffer[decode_iterator - AN_PACKET_HEADER_SIZE], AN_PACKET_HEADER_SIZE * sizeof(uint8_t));
                    memcpy(an_packet->data, &an_decoder->buffer[decode_iterator], length * sizeof(uint8_t));
                }
                decode_iterator += length;
                an_decoder->packets_decoded++;
                an_decoder->bytes_decoded += length + AN_PACKET_HEADER_SIZE;
                break;
            } else {
                decode_iterator -= (AN_PACKET_HEADER_SIZE - 1);
                an_decoder->crc_errors++;
                an_decoder->bytes_discarded++;
            }
        } else {
            an_decoder->lrc_errors++;
            an_decoder->bytes_discarded++;
        }
    }
    if (decode_iterator < an_decoder->buffer_length) {
        if (decode_iterator > 0) {
            memmove(&an_decoder->buffer[0], &an_decoder->buffer[decode_iterator], (an_decoder->buffer_length - decode_iterator) * sizeof(uint8_t));
            an_decoder->buffer_length -= decode_iterator;
        }
    } else {
        an_decoder->buffer_length = 0;
    }

    return an_packet;
}

/*
 * Function to encode an an_packet
 */
void AdNav::an_packet_encode(an_packet_t* an_packet)
{
    uint16_t crc;
    an_packet->header[1] = an_packet->id;
    an_packet->header[2] = an_packet->length;
    crc = crc16_ccitt(an_packet->data, an_packet->length, 0xFFFF);
    memcpy(&an_packet->header[3], &crc, sizeof(uint16_t));
    an_packet->header[0] = calculate_header_lrc(&an_packet->header[1]);
}
