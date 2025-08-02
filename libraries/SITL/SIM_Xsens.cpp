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
    Simulate Xsens GNSS/INS devices
    
    Usage:
    PARAMS:
        param set AHRS_EKF_TYPE 11
        param set EAHRS_TYPE 11  
        param set SERIAL4_PROTOCOL 36
        param set SERIAL4_BAUD 115200
        param set EAHRS_RATE 100
    sim_vehicle.py -v Copter -A "--serial2=sim:Xsens" --console --map -DG
*/

#include "SIM_Xsens.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_Common/NMEA.h>

using namespace SITL;

Xsens::Xsens() :
    SerialDevice::SerialDevice(),
    last_mtdata2_us(0),
    last_gnss_pkt_us(0),
    packet_counter(0),
    current_state(DeviceState::CONFIG_MODE),
    output_configured(false),
    alignment_configured(false),
    gnss_fix_type(3),           // 3D fix
    gnss_num_satellites(12),    // Typical satellite count
    gnss_horizontal_accuracy(2000), // 2m in mm
    gnss_vertical_accuracy(3000),   // 3m in mm
    gnss_speed_accuracy(100),       // 0.1m/s in mm/s
    gnss_hdop(120),                 // 1.2 * 100
    gnss_vdop(180),                 // 1.8 * 100
    rx_buffer_pos(0)
{
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

void Xsens::write_uint8(uint8_t *buffer, uint8_t value)
{
    buffer[0] = value;
}

void Xsens::write_uint16_be(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

void Xsens::write_uint32_be(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

void Xsens::write_int32_be(uint8_t *buffer, int32_t value)
{
    write_uint32_be(buffer, (uint32_t)value);
}

void Xsens::write_float_be(uint8_t *buffer, float value)
{
    union {
        float f;
        uint32_t u;
    } converter;
    converter.f = value;
    write_uint32_be(buffer, converter.u);
}

void Xsens::write_double_fp1632(uint8_t *buffer, double value)
{
    // FP16.32 format: 32-bit fractional part followed by 16-bit integer part
    int64_t fixed_point = (int64_t)(value * 4294967296.0); // 2^32
    uint32_t fractional_part = (uint32_t)(fixed_point & 0xFFFFFFFF);
    int16_t integer_part = (int16_t)(fixed_point >> 32);
    
    write_uint32_be(buffer, fractional_part);
    write_uint16_be(buffer + 4, (uint16_t)integer_part);
}

void Xsens::update_gnss_state()
{
    const auto &fdm = _sitl->state;
    
    // Simulate varying GNSS conditions based on altitude and movement
    float altitude_factor = 1.0f;
    if (fdm.altitude < 10.0f) {
        altitude_factor = 0.7f; // Reduced performance at low altitude
    }
    
    float speed = sqrtf(fdm.speedN * fdm.speedN + fdm.speedE * fdm.speedE);
    float movement_factor = 1.0f;
    if (speed > 20.0f) {
        movement_factor = 0.9f; // Slightly reduced performance at high speed
    }
    
    // Simulate occasional fix type variations
    uint32_t now = AP_HAL::millis();
    if ((now / 10000) % 30 == 0) { // Every 5 minutes, brief degradation
        gnss_fix_type = 2; // 2D fix
        gnss_num_satellites = 4 + (now % 4);
        gnss_horizontal_accuracy = 5000; // 5m
        gnss_vertical_accuracy = 10000;  // 10m
        gnss_hdop = 300; // 3.0
        gnss_vdop = 500; // 5.0
    } else {
        gnss_fix_type = 3; // 3D fix
        gnss_num_satellites = 8 + (now % 8);
        gnss_horizontal_accuracy = (uint32_t)(2000 / altitude_factor / movement_factor); // Base 2m accuracy
        gnss_vertical_accuracy = (uint32_t)(3000 / altitude_factor / movement_factor);   // Base 3m accuracy
        gnss_speed_accuracy = (uint32_t)(100 / movement_factor);
        gnss_hdop = (uint16_t)(120 / altitude_factor); // 1.2 base
        gnss_vdop = (uint16_t)(180 / altitude_factor); // 1.8 base
    }
}

void Xsens::calculate_gps_time_from_utc(uint16_t year, uint8_t month, uint8_t day,
                                       uint8_t hour, uint8_t minute, uint8_t second,
                                       int32_t nano, uint16_t &gps_week, uint32_t &ms_tow)
{
    // Simple GPS time calculation (should match the driver implementation)
    // Days in each month (non-leap year)
    static const uint16_t days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    // Calculate total days since GPS epoch (January 6, 1980)
    uint32_t total_days = 0;
    
    // Add days for complete years since 1980
    for (uint16_t y = 1980; y < year; y++) {
        bool is_leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        total_days += is_leap ? 366 : 365;
    }
    
    // Add days for complete months in the current year
    for (uint8_t m = 1; m < month; m++) {
        total_days += days_in_month[m];
        // Add extra day for February in leap years
        if (m == 2 && year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
            total_days++;
        }
    }
    
    // Add days in the current month
    total_days += day - 1; // -1 because day 1 = 0 days elapsed
    
    // GPS epoch was January 6, 1980, so subtract 5 days (Jan 1-5, 1980)
    if (total_days >= 5) {
        total_days -= 5;
    } else {
        gps_week = 0;
        ms_tow = 0;
        return; // Date is before GPS epoch
    }
    
    // Calculate GPS week
    gps_week = (total_days / 7) % 1024; // Handle 1024-week rollover
    
    // Calculate milliseconds time of week
    uint32_t days_in_current_week = total_days % 7;
    uint32_t seconds_in_week = days_in_current_week * 86400 + // days to seconds
                               hour * 3600 +                  // hours to seconds
                               minute * 60 +                  // minutes to seconds
                               second;                         // seconds
    
    ms_tow = seconds_in_week * 1000 + nano / 1000000; // Convert to milliseconds and add nanoseconds
}

void Xsens::create_xbus_message(uint8_t *buffer, uint8_t bid, uint8_t mid, uint16_t length, uint8_t *payload)
{
    buffer[0] = XBUS_PREAMBLE;
    buffer[1] = bid;
    buffer[2] = mid;
    
    if (length < 255) {
        buffer[3] = length;
        if (payload && length > 0) {
            memcpy(&buffer[4], payload, length);
        }
    } else {
        buffer[3] = LENGTH_EXTENDER_BYTE;
        buffer[4] = (length >> 8) & 0xFF;
        buffer[5] = length & 0xFF;
        if (payload && length > 0) {
            memcpy(&buffer[6], payload, length);
        }
    }
}

void Xsens::insert_checksum(uint8_t *message, uint16_t total_length)
{
    uint8_t checksum = 0;
    for (int i = 1; i < total_length - 1; i++) {
        checksum += message[i];
    }
    checksum = 0x100 - checksum;
    message[total_length - 1] = checksum;
}

bool Xsens::verify_checksum(const uint8_t *message, uint16_t total_length)
{
    uint8_t checksum = 0;
    for (int i = 1; i < total_length; i++) {
        checksum += message[i];
    }
    return (checksum & 0xFF) == 0;
}

uint8_t Xsens::get_message_id(const uint8_t *message)
{
    return message[2];
}

uint8_t Xsens::get_payload_length(const uint8_t *message)
{
    if (message[3] != LENGTH_EXTENDER_BYTE) {
        return message[3];
    } else {
        return (message[4] << 8) | message[5];
    }
}

const uint8_t* Xsens::get_const_pointer_to_payload(const uint8_t* xbus_message) const
{
    if (xbus_message[3] == LENGTH_EXTENDER_BYTE) {
        return xbus_message + 6;
    } else {
        return xbus_message + 4;
    }
}

void Xsens::send_goto_config_ack()
{
    // Reply: FA FF 31 00 D0
    uint8_t ack_msg[5];
    create_xbus_message(ack_msg, XBUS_MASTERDEVICE, XsMessageId::GotoConfigAck, 0, nullptr);
    insert_checksum(ack_msg, 5);
    write_to_autopilot((const char *)ack_msg, 5);
    printf("Xsens: Sent GotoConfigAck\n");
}

void Xsens::send_output_config_ack(const uint8_t *original_payload, uint8_t original_length)
{
    // Parse the original SetOutputConfig message and create response
    uint8_t response_msg[128];
    uint8_t response_payload[120];
    int response_idx = 0;
    
    // Parse original payload to create response
    for (int i = 0; i < original_length; i += 4) {
        if (i + 3 >= original_length) break;
        
        uint16_t xdi = (original_payload[i] << 8) | original_payload[i + 1];
        uint16_t requested_rate = (original_payload[i + 2] << 8) | original_payload[i + 3];
        
        // Copy XDI
        response_payload[response_idx++] = original_payload[i];
        response_payload[response_idx++] = original_payload[i + 1];
        
        // Set response rate based on XDI type
        uint16_t response_rate;
        switch (xdi) {
            case XDI::PACKET_COUNTER:
            case XDI::SAMPLE_TIME_FINE: 
            case XDI::UTC_TIME:
            case XDI::STATUS_WORD:
                response_rate = 0xFFFF; // Special markers
                break;
            case XDI::GNSSPVTDATA:
                response_rate = 4; // Always 4Hz for GNSS PVT
                break;
            default:
                response_rate = requested_rate; // Echo back the requested rate
                break;
        }
        
        response_payload[response_idx++] = (response_rate >> 8) & 0xFF;
        response_payload[response_idx++] = response_rate & 0xFF;
    }
    
    create_xbus_message(response_msg, XBUS_MASTERDEVICE, XsMessageId::OutputConfig, response_idx, response_payload);
    
    uint16_t total_length = 4 + response_idx + 1;
    insert_checksum(response_msg, total_length);
    write_to_autopilot((const char *)response_msg, total_length);
    
    output_configured = true;
    printf("Xsens: Sent OutputConfig response (%d bytes)\n", response_idx);
}

void Xsens::send_alignment_rotation_ack()
{
    // Reply: FA FF ED 00 14
    uint8_t ack_msg[5];
    create_xbus_message(ack_msg, XBUS_MASTERDEVICE, XsMessageId::AlignmentRotationAck, 0, nullptr);
    insert_checksum(ack_msg, 5);
    write_to_autopilot((const char *)ack_msg, 5);
    
    alignment_configured = true;
    printf("Xsens: Sent AlignmentRotationAck\n");
}

void Xsens::send_goto_measurement_ack()
{
    // Reply: FA FF 11 00 F0
    uint8_t ack_msg[5];
    create_xbus_message(ack_msg, XBUS_MASTERDEVICE, XsMessageId::GotoMeasurementAck, 0, nullptr);
    insert_checksum(ack_msg, 5);
    write_to_autopilot((const char *)ack_msg, 5);
    
    current_state = DeviceState::MEASUREMENT_MODE;
    printf("Xsens: Sent GotoMeasurementAck - Now in MEASUREMENT mode\n");
}

void Xsens::send_gnsspvt_packet()
{
    const auto &fdm = _sitl->state;
    
    uint8_t mtdata2_msg[256];
    uint8_t payload[200];
    int payload_idx = 0;
    
    // Update GNSS state before sending
    update_gnss_state();
    
    // Packet Counter (XDI 0x1020)
    write_uint16_be(&payload[payload_idx], XDI::PACKET_COUNTER);
    payload_idx += 2;
    payload[payload_idx++] = 2; // Size
    write_uint16_be(&payload[payload_idx], packet_counter++);
    payload_idx += 2;
    
    // Sample Time Fine (XDI 0x1060)
    write_uint16_be(&payload[payload_idx], XDI::SAMPLE_TIME_FINE);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    write_uint32_be(&payload[payload_idx], AP_HAL::micros());
    payload_idx += 4;
    
    // UTC Time (XDI 0x1010)
    write_uint16_be(&payload[payload_idx], XDI::UTC_TIME);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    
    struct timeval tv;
    simulation_timeval(&tv);
    struct tm *utc_tm = gmtime(&tv.tv_sec);
    
    write_uint32_be(&payload[payload_idx], tv.tv_usec * 1000); // Nanoseconds
    payload_idx += 4;
    write_uint16_be(&payload[payload_idx], utc_tm->tm_year + 1900); // Year
    payload_idx += 2;
    payload[payload_idx++] = utc_tm->tm_mon + 1; // Month (1-12)
    payload[payload_idx++] = utc_tm->tm_mday;    // Day
    payload[payload_idx++] = utc_tm->tm_hour;    // Hour
    payload[payload_idx++] = utc_tm->tm_min;     // Minute
    payload[payload_idx++] = utc_tm->tm_sec;     // Second
    payload[payload_idx++] = 0x07; // Flags (valid time)
    
    // Status Word (XDI 0xE020)
    write_uint16_be(&payload[payload_idx], XDI::STATUS_WORD);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    uint32_t status = 0x0004; // GNSS fix bit set
    write_uint32_be(&payload[payload_idx], status);
    payload_idx += 4;
    
    // GNSS PVT Data (XDI 0x7010) - 94 bytes
    write_uint16_be(&payload[payload_idx], XDI::GNSSPVTDATA);
    payload_idx += 2;
    payload[payload_idx++] = 94; // Size
    
    // Calculate GPS time
    uint16_t gps_week;
    uint32_t ms_tow;
    calculate_gps_time_from_utc(utc_tm->tm_year + 1900, utc_tm->tm_mon + 1, utc_tm->tm_mday,
                               utc_tm->tm_hour, utc_tm->tm_min, utc_tm->tm_sec,
                               tv.tv_usec * 1000, gps_week, ms_tow);
    
    write_uint32_be(&payload[payload_idx], ms_tow);           // iTOW
    payload_idx += 4;
    write_uint16_be(&payload[payload_idx], utc_tm->tm_year + 1900); // year
    payload_idx += 2;
    write_uint8(&payload[payload_idx], utc_tm->tm_mon + 1);   // month
    payload_idx += 1;
    write_uint8(&payload[payload_idx], utc_tm->tm_mday);      // day
    payload_idx += 1;
    write_uint8(&payload[payload_idx], utc_tm->tm_hour);      // hour
    payload_idx += 1;
    write_uint8(&payload[payload_idx], utc_tm->tm_min);       // min
    payload_idx += 1;
    write_uint8(&payload[payload_idx], utc_tm->tm_sec);       // sec
    payload_idx += 1;
    write_uint8(&payload[payload_idx], 0x07);                 // valid flags
    payload_idx += 1;
    write_uint32_be(&payload[payload_idx], 1000000);          // tAcc (1ms)
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], tv.tv_usec * 1000); // nano
    payload_idx += 4;
    write_uint8(&payload[payload_idx], gnss_fix_type);        // fixType
    payload_idx += 1;
    write_uint8(&payload[payload_idx], 0x01);                 // flags (gnssFixOK)
    payload_idx += 1;
    write_uint8(&payload[payload_idx], gnss_num_satellites);  // numSv
    payload_idx += 1;
    write_uint8(&payload[payload_idx], 0);                    // reserved
    payload_idx += 1;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.longitude * 1e7)); // lon
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.latitude * 1e7));  // lat
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.altitude * 1000)); // height (mm)
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.altitude * 1000)); // hMSL (mm)
    payload_idx += 4;
    write_uint32_be(&payload[payload_idx], gnss_horizontal_accuracy); // hAcc (mm)
    payload_idx += 4;
    write_uint32_be(&payload[payload_idx], gnss_vertical_accuracy);   // vAcc (mm)
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.speedN * 1000)); // velN (mm/s)
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.speedE * 1000)); // velE (mm/s)
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(fdm.speedD * 1000)); // velD (mm/s)
    payload_idx += 4;
    
    float ground_speed = sqrtf(fdm.speedN * fdm.speedN + fdm.speedE * fdm.speedE);
    write_int32_be(&payload[payload_idx], (int32_t)(ground_speed * 1000)); // gSpeed (mm/s)
    payload_idx += 4;
    
    float heading = atan2f(fdm.speedE, fdm.speedN) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    write_int32_be(&payload[payload_idx], (int32_t)(heading * 1e5)); // headMot (deg * 1e-5)
    payload_idx += 4;
    
    write_uint32_be(&payload[payload_idx], gnss_speed_accuracy); // sAcc (mm/s)
    payload_idx += 4;
    write_uint32_be(&payload[payload_idx], 180000);             // headAcc (deg * 1e-5) = 1.8 deg
    payload_idx += 4;
    write_int32_be(&payload[payload_idx], (int32_t)(heading * 1e5)); // headVeh (deg * 1e-5)
    payload_idx += 4;
    
    // DOP values (* 0.01)
    write_uint16_be(&payload[payload_idx], gnss_hdop + gnss_vdop); // gDop
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], gnss_hdop + 50);        // pDop
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], 100);                  // tDop (1.0)
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], gnss_vdop);            // vDop
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], gnss_hdop);            // hDop
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], gnss_hdop / 2);        // nDop
    payload_idx += 2;
    write_uint16_be(&payload[payload_idx], gnss_hdop / 2);        // eDop
    payload_idx += 2;
    
    // Create and send the complete MTData2 message
    create_xbus_message(mtdata2_msg, XBUS_MASTERDEVICE, XsMessageId::MtData2, payload_idx, payload);
    
    // Calculate total message length (header + payload + checksum)
    uint16_t total_length = 4 + payload_idx + 1;
    insert_checksum(mtdata2_msg, total_length);
    
    write_to_autopilot((const char *)mtdata2_msg, total_length);
}

void Xsens::send_mtdata2_packet()
{
    const auto &fdm = _sitl->state;
    
    uint8_t mtdata2_msg[256];
    uint8_t payload[252];
    int payload_idx = 0;
    
    // Packet Counter (XDI 0x1020)
    write_uint16_be(&payload[payload_idx], XDI::PACKET_COUNTER);
    payload_idx += 2;
    payload[payload_idx++] = 2; // Size
    write_uint16_be(&payload[payload_idx], packet_counter++);
    payload_idx += 2;
    
    // Sample Time Fine (XDI 0x1060)
    write_uint16_be(&payload[payload_idx], XDI::SAMPLE_TIME_FINE);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    write_uint32_be(&payload[payload_idx], AP_HAL::micros());
    payload_idx += 4;
    
    // UTC Time (XDI 0x1010)
    write_uint16_be(&payload[payload_idx], XDI::UTC_TIME);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    
    struct timeval tv;
    simulation_timeval(&tv);
    struct tm *utc_tm = gmtime(&tv.tv_sec);
    
    write_uint32_be(&payload[payload_idx], tv.tv_usec * 1000); // Nanoseconds
    payload_idx += 4;
    write_uint16_be(&payload[payload_idx], utc_tm->tm_year + 1900); // Year
    payload_idx += 2;
    payload[payload_idx++] = utc_tm->tm_mon + 1; // Month (1-12)
    payload[payload_idx++] = utc_tm->tm_mday;    // Day
    payload[payload_idx++] = utc_tm->tm_hour;    // Hour
    payload[payload_idx++] = utc_tm->tm_min;     // Minute
    payload[payload_idx++] = utc_tm->tm_sec;     // Second
    payload[payload_idx++] = 0x07; // Flags (valid time)
    
    // Temperature (XDI 0x0810)
    write_uint16_be(&payload[payload_idx], XDI::TEMPERATURE);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    float temp = AP_Baro::get_temperatureC_for_alt_amsl(fdm.altitude);
    write_float_be(&payload[payload_idx], temp);
    payload_idx += 4;
    
    // Quaternion (XDI 0x2010) - w,x,y,z format
    write_uint16_be(&payload[payload_idx], XDI::QUATERNION);
    payload_idx += 2;
    payload[payload_idx++] = 16; // Size
    write_float_be(&payload[payload_idx], fdm.quaternion.q1);     // w
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.quaternion.q2);     // x
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.quaternion.q3);     // y
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.quaternion.q4);     // z
    payload_idx += 4;
    
    // Acceleration (XDI 0x4020)
    write_uint16_be(&payload[payload_idx], XDI::ACCELERATION);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    const float accel_noise = 0.01f;
    write_float_be(&payload[payload_idx], fdm.xAccel + rand_float() * accel_noise);
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.yAccel + rand_float() * accel_noise);
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.zAccel + rand_float() * accel_noise);
    payload_idx += 4;
    
    // Rate of Turn (XDI 0x8020)
    write_uint16_be(&payload[payload_idx], XDI::RATE_OF_TURN);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    const float gyro_noise = 0.001;
    write_float_be(&payload[payload_idx], radians(fdm.rollRate + rand_float() * gyro_noise));
    payload_idx += 4;
    write_float_be(&payload[payload_idx], radians(fdm.pitchRate + rand_float() * gyro_noise));
    payload_idx += 4;
    write_float_be(&payload[payload_idx], radians(fdm.yawRate + rand_float() * gyro_noise));
    payload_idx += 4;
    
    // Magnetic Field (XDI 0xC020) - in arbitrary units
    write_uint16_be(&payload[payload_idx], XDI::MAGNETIC_FIELD);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    const float gauss_to_au = 1.0f / 49.0f; // Inverse of conversion factor
    write_float_be(&payload[payload_idx], fdm.bodyMagField.x * 0.001f * gauss_to_au);
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.bodyMagField.y * 0.001f * gauss_to_au);
    payload_idx += 4;
    write_float_be(&payload[payload_idx], fdm.bodyMagField.z * 0.001f * gauss_to_au);
    payload_idx += 4;
    
    // Status Word (XDI 0xE020)
    write_uint16_be(&payload[payload_idx], XDI::STATUS_WORD);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    uint32_t status = 0x0004; // GNSS fix bit set
    write_uint32_be(&payload[payload_idx], status);
    payload_idx += 4;
    
    // Barometric Pressure (XDI 0x3010)
    write_uint16_be(&payload[payload_idx], XDI::BAROMETRIC_PRESSURE);
    payload_idx += 2;
    payload[payload_idx++] = 4; // Size
    const float pressure_Pa = AP_Baro::get_pressure_for_alt_amsl(fdm.altitude);
    uint32_t pressure_pa = (uint32_t)(pressure_Pa + rand_float() * 100);
    write_uint32_be(&payload[payload_idx], pressure_pa);
    payload_idx += 4;
    
    // Latitude/Longitude (XDI 0x5042) - FP1632 format
    write_uint16_be(&payload[payload_idx], XDI::LAT_LON);
    payload_idx += 2;
    payload[payload_idx++] = 12; // Size
    write_double_fp1632(&payload[payload_idx], fdm.latitude);
    payload_idx += 6;
    write_double_fp1632(&payload[payload_idx], fdm.longitude);
    payload_idx += 6;
    
    // Altitude Ellipsoid (XDI 0x5022) - FP1632 format
    write_uint16_be(&payload[payload_idx], XDI::ALTITUDE_ELLIPSOID);
    payload_idx += 2;
    payload[payload_idx++] = 6; // Size
    write_double_fp1632(&payload[payload_idx], fdm.altitude);
    payload_idx += 6;
    
    // Velocity XYZ (XDI 0xD012) - FP1632 format
    write_uint16_be(&payload[payload_idx], XDI::VELOCITY_XYZ);
    payload_idx += 2;
    payload[payload_idx++] = 18; // Size
    write_double_fp1632(&payload[payload_idx], fdm.speedN);
    payload_idx += 6;
    write_double_fp1632(&payload[payload_idx], fdm.speedE);
    payload_idx += 6;
    write_double_fp1632(&payload[payload_idx], fdm.speedD);
    payload_idx += 6;
    
    // Create and send the complete MTData2 message
    create_xbus_message(mtdata2_msg, XBUS_MASTERDEVICE, XsMessageId::MtData2, payload_idx, payload);
    
    // Calculate total message length (header + payload + checksum)
    uint16_t total_length = 4 + payload_idx + 1;
    insert_checksum(mtdata2_msg, total_length);
    
    write_to_autopilot((const char *)mtdata2_msg, total_length);
}

void Xsens::handle_message(const uint8_t *data, uint8_t length)
{
    if (length < 4) {
        printf("Xsens: Invalid message length %u\n", length);
        return;
    }
    
    uint8_t msg_id = get_message_id(data);
    uint8_t payload_length = get_payload_length(data);
    printf("Xsens: Received message ID 0x%02X, payload length %u\n", msg_id, payload_length);
    
    switch (msg_id) {
        case XsMessageId::GotoConfig:
            printf("Xsens: Going to config mode\n");
            current_state = DeviceState::CONFIG_MODE;
            output_configured = false;
            alignment_configured = false;
            send_goto_config_ack();
            break;
            
        case XsMessageId::SetOutputConfig:
            printf("Xsens: Setting output config\n");
            if (payload_length > 0) {
                const uint8_t* payload = get_const_pointer_to_payload(data);
                send_output_config_ack(payload, payload_length);
            }
            break;
            
        case XsMessageId::SetAlignmentRotation:
            printf("Xsens: Setting alignment rotation\n");
            send_alignment_rotation_ack();
            break;
            
        case XsMessageId::GotoMeasurement:
            printf("Xsens: Going to measurement mode\n");
            send_goto_measurement_ack();
            break;
            
        default:
            printf("Xsens: Ignoring message ID 0x%02X\n", msg_id);
            break;
    }
}

void Xsens::process_incoming_data()
{
    char receive_buf[64];
    ssize_t n = read_from_autopilot(receive_buf, sizeof(receive_buf));
    if (n <= 0) {
        return;
    }
    
    // Add received data to buffer
    for (ssize_t i = 0; i < n && rx_buffer_pos < RX_BUFFER_SIZE; i++) {
        rx_buffer[rx_buffer_pos++] = receive_buf[i];
    }
    
    // Process complete messages
    size_t search_pos = 0;
    while (search_pos < rx_buffer_pos) {
        // Look for preamble
        while (search_pos < rx_buffer_pos && rx_buffer[search_pos] != XBUS_PREAMBLE) {
            search_pos++;
        }
        
        if (search_pos >= rx_buffer_pos) {
            break; // No preamble found
        }
        
        // Check if we have enough data for a complete message
        if (search_pos + 4 > rx_buffer_pos) {
            break; // Not enough data for header
        }
        
        uint8_t payload_length = get_payload_length(&rx_buffer[search_pos]);
        uint16_t total_msg_length = 4 + payload_length + 1; // Header + payload + checksum
        
        if (search_pos + total_msg_length > rx_buffer_pos) {
            break; // Not enough data for complete message
        }
        
        // Verify checksum
        if (verify_checksum(&rx_buffer[search_pos], total_msg_length)) {
            handle_message(&rx_buffer[search_pos], total_msg_length);
        } else {
            printf("Xsens: Checksum verification failed\n");
        }
        
        search_pos += total_msg_length;
    }
    
    // Remove processed data from buffer
    if (search_pos > 0) {
        memmove(rx_buffer, &rx_buffer[search_pos], rx_buffer_pos - search_pos);
        rx_buffer_pos -= search_pos;
    }
}

/*
  send Xsens data
 */
void Xsens::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    
    // Process incoming configuration commands
    process_incoming_data();
    
    // Send data when in measurement mode
    if (current_state == DeviceState::MEASUREMENT_MODE) {
        uint32_t now = AP_HAL::micros();
        
        // Send IMU/INS data at 100Hz
        if (now - last_mtdata2_us >= 10000) { // 100Hz = 10ms intervals
            last_mtdata2_us = now;
            send_mtdata2_packet();
        }
        
        // Send GNSS data at 5Hz 
        if (now - last_gnss_pkt_us >= 200000) { // 5Hz = 200ms intervals
            last_gnss_pkt_us = now;
            send_gnsspvt_packet();
        }
    }
}