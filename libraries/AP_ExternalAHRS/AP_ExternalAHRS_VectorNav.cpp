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

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_VECTORNAV_ENABLED

#include "AP_ExternalAHRS_VectorNav.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;


/*
TYPE::VN_AHRS configures 2 packets: high-rate IMU and mid-rate EKF
Header for IMU packet
    $VNWRG,75,3,16,01,0721*D415
    Common group (Group 1)
        TimeStartup
        AngularRate
        Accel
        Imu
        MagPres
Header for EKF packet
    $VNWRG,76,3,16,11,0001,0106*B36B
    Common group (Group 1)
        TimeStartup
    Attitude group (Group 4)
        Ypr
        Quaternion
        YprU
*/

struct PACKED VN_IMU_packet {
    static constexpr uint8_t header[]{0x01, 0x21, 0x07};
    uint64_t timeStartup;
    float gyro[3];
    float accel[3];
    float uncompAccel[3];
    float uncompAngRate[3];
    float mag[3];
    float temp;
    float pressure;
};
constexpr uint8_t VN_IMU_packet::header[];
constexpr uint8_t VN_IMU_LENGTH = sizeof(VN_IMU_packet) + sizeof(VN_IMU_packet::header) + 1 + 2; // Includes sync byte and CRC

struct PACKED VN_AHRS_ekf_packet {
    static constexpr uint8_t header[]{0x11, 0x01, 0x00, 0x06, 0x01};
    uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
};
constexpr uint8_t VN_AHRS_ekf_packet::header[];
constexpr uint8_t VN_AHRS_EKF_LENGTH = sizeof(VN_AHRS_ekf_packet) + sizeof(VN_AHRS_ekf_packet::header) + 1 + 2; // Includes sync byte and CRC

/*
TYPE::VN_INS configures 3 packets: high-rate IMU, mid-rate EKF, and 5Hz GNSS
Header for IMU packet
    $VNWRG,75,3,16,01,0721*D415
    Common group (Group 1)
        TimeStartup
        AngularRate
        Accel
        Imu
        MagPres
Header for EKF packet
    $VNWRG,76,3,16,31,0001,0106,0613*097A
    Common group (Group 1)
        TimeStartup
    Attitude group (Group 4)
        Ypr
        Quaternion
        YprU
    Ins group (Group 5)
        InsStatus
        PosLla
        VelNed
        PosU
        VelU
Header for GNSS packet
    $VNWRG,77,1,160,49,0003,26B8,0018*4FD9
    Common group (Group 1)
        TimeStartup
        TimeGps
    Gnss1 group (Group 3)
        NumSats
        GnssFix
        GnssPosLla
        GnssVelNed
        PosU1
        VelU1
        GnssDop
    Gnss2 group (Group 6)
        NumSats
        GnssFix
*/

union Ins_Status {
    uint16_t _value;
    struct {
        uint16_t mode : 2;
        uint16_t gnssFix : 1;
        uint16_t resv1 : 2;
        uint16_t imuErr : 1;
        uint16_t magPresErr : 1;
        uint16_t gnssErr : 1;
        uint16_t resv2 : 1;
        uint16_t gnssHeadingIns : 2;
    };
};

struct PACKED VN_INS_ekf_packet {
    static constexpr uint8_t header[]{0x31, 0x01, 0x00, 0x06, 0x01, 0x13, 0x06};
    uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
    uint16_t insStatus;
    double posLla[3];
    float velNed[3];
    float posU;
    float velU;
};
constexpr uint8_t VN_INS_ekf_packet::header[];
constexpr uint8_t VN_INS_EKF_LENGTH = sizeof(VN_INS_ekf_packet) + sizeof(VN_INS_ekf_packet::header) + 1 + 2; // Includes sync byte and CRC

struct PACKED VN_INS_gnss_packet {
    static constexpr uint8_t header[]{0x49, 0x03, 0x00, 0xB8, 0x26, 0x18, 0x00};
    uint64_t timeStartup;
    uint64_t timeGps;
    uint8_t numSats1;
    uint8_t fix1;
    double posLla1[3];
    float velNed1[3];
    float posU1[3];
    float velU1;
    float dop1[7];
    uint8_t numSats2;
    uint8_t fix2;
};
constexpr uint8_t VN_INS_gnss_packet::header[];
constexpr uint8_t VN_INS_GNSS_LENGTH = sizeof(VN_INS_gnss_packet) + sizeof(VN_INS_gnss_packet::header) + 1 + 2; // Includes sync byte and CRC

// constructor
AP_ExternalAHRS_VectorNav::AP_ExternalAHRS_VectorNav(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VectorNav ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    bufsize = MAX(MAX(MAX(VN_IMU_LENGTH, VN_INS_EKF_LENGTH), VN_INS_GNSS_LENGTH), VN_AHRS_EKF_LENGTH);

    pktbuf = NEW_NOTHROW uint8_t[bufsize];
    latest_ins_ekf_packet = NEW_NOTHROW VN_INS_ekf_packet;
    latest_ins_gnss_packet = NEW_NOTHROW VN_INS_gnss_packet;

    if (!pktbuf || !latest_ins_ekf_packet) {
        AP_BoardConfig::allocation_error("VectorNav ExternalAHRS");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_VectorNav::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("VectorNav Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VectorNav ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
#define SYNC_BYTE 0xFA
bool AP_ExternalAHRS_VectorNav::check_uart()
{
    if (!setup_complete) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    if (pktoffset < bufsize) {
        ssize_t nread = uart->read(&pktbuf[pktoffset], MIN(n, unsigned(bufsize-pktoffset)));
        if (nread <= 0) {
            return false;
        }
        pktoffset += nread;
    }

    bool match_header1 = false;
    bool match_header2 = false;
    bool match_header3 = false;
    bool match_header4 = false;

    if (pktbuf[0] != SYNC_BYTE) {
        goto reset;
    }


    match_header1 = (0 == memcmp(&pktbuf[1], VN_IMU_packet::header,  MIN(sizeof(VN_IMU_packet::header), unsigned(pktoffset - 1))));
    if (type == TYPE::VN_AHRS) {
        match_header2 = (0 == memcmp(&pktbuf[1], VN_AHRS_ekf_packet::header, MIN(sizeof(VN_AHRS_ekf_packet::header), unsigned(pktoffset - 1))));
    } else {
        match_header3 = (0 == memcmp(&pktbuf[1], VN_INS_ekf_packet::header,  MIN(sizeof(VN_INS_ekf_packet::header), unsigned(pktoffset - 1))));
        match_header4 = (0 == memcmp(&pktbuf[1], VN_INS_gnss_packet::header, MIN(sizeof(VN_INS_gnss_packet::header), unsigned(pktoffset - 1))));
    }
    if (!match_header1 && !match_header2 && !match_header3 && !match_header4) {
        goto reset;
    }

    if (match_header1 && pktoffset >= VN_IMU_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_IMU_LENGTH - 1, 0);
        if (crc == 0) {
            process_imu_packet(&pktbuf[sizeof(VN_IMU_packet::header) + 1]);
            memmove(&pktbuf[0], &pktbuf[VN_IMU_LENGTH], pktoffset - VN_IMU_LENGTH);
            pktoffset -= VN_IMU_LENGTH;
        } else {
            goto reset;
        }
    } else if (match_header2 && pktoffset >= VN_AHRS_EKF_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_AHRS_EKF_LENGTH - 1, 0);
        if (crc == 0) {
            process_ahrs_ekf_packet(&pktbuf[sizeof(VN_AHRS_ekf_packet::header) + 1]);
            memmove(&pktbuf[0], &pktbuf[VN_AHRS_EKF_LENGTH], pktoffset - VN_AHRS_EKF_LENGTH);
            pktoffset -= VN_AHRS_EKF_LENGTH;
        } else {
            goto reset;
        }
    } else if (match_header3 && pktoffset >= VN_INS_EKF_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_INS_EKF_LENGTH - 1, 0);
        if (crc == 0) {
            process_ins_ekf_packet(&pktbuf[sizeof(VN_INS_ekf_packet::header) + 1]);
            memmove(&pktbuf[0], &pktbuf[VN_INS_EKF_LENGTH], pktoffset - VN_INS_EKF_LENGTH);
            pktoffset -= VN_INS_EKF_LENGTH;
        } else {
            goto reset;
        }
    } else if (match_header4 && pktoffset >= VN_INS_GNSS_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_INS_GNSS_LENGTH - 1, 0);
        if (crc == 0) {
            process_ins_gnss_packet(&pktbuf[sizeof(VN_INS_gnss_packet::header) + 1]);
            memmove(&pktbuf[0], &pktbuf[VN_INS_GNSS_LENGTH], pktoffset - VN_INS_GNSS_LENGTH);
            pktoffset -= VN_INS_GNSS_LENGTH;
        } else {
            goto reset;
        }
    }
    return true;

reset:
    uint8_t *p = (uint8_t *)memchr(&pktbuf[1], SYNC_BYTE, pktoffset-1);
    if (p) {
        uint8_t newlen = pktoffset - (p - pktbuf);
        memmove(&pktbuf[0], p, newlen);
        pktoffset = newlen;
    } else {
        pktoffset = 0;
    }
    return true;
}

// Send command and wait for response
// Only run from thread! This blocks and retries until a non-error response is received
#define READ_REQUEST_RETRY_MS 500
void AP_ExternalAHRS_VectorNav::run_command(const char * fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    hal.util->vsnprintf(message_to_send, sizeof(message_to_send), fmt, ap);
    va_end(ap);

    uint32_t request_sent = 0;
    while (true) {
        hal.scheduler->delay(1);

        const uint32_t now = AP_HAL::millis();
        if (now - request_sent > READ_REQUEST_RETRY_MS) {
            nmea_printf(uart, "$%s", message_to_send);
            request_sent = now;
        }

        int16_t nbytes = uart->available();
        while (nbytes-- > 0) {
            char c = uart->read();
            if (decode(c)) {
                if (nmea.error_response && nmea.sentence_done) {
                    // Received a valid VNERR. Try to resend after the timeout length
                    break;
                }
                return;
            }
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_ExternalAHRS_VectorNav::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        nmea.checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        if (nmea.sentence_done) {
            return false;
        }
        if (nmea.term_is_checksum) {
            nmea.sentence_done = true;
            uint8_t checksum = 16 * char_to_hex(nmea.term[0]) + char_to_hex(nmea.term[1]);
            return ((checksum == nmea.checksum) && nmea.sentence_valid);
        }

        // null terminate and decode latest term
        nmea.term[nmea.term_offset] = 0;
        if (nmea.sentence_valid) {
            nmea.sentence_valid = decode_latest_term();
        }

        // move onto next term
        nmea.term_number++;
        nmea.term_offset = 0;
        nmea.term_is_checksum = (c == '*');
        return false;
    }

    case '$': // sentence begin
        nmea.sentence_valid = true;
        nmea.term_number = 0;
        nmea.term_offset = 0;
        nmea.checksum = 0;
        nmea.term_is_checksum = false;
        nmea.sentence_done = false;
        nmea.error_response = false;
        return false;
    }

    // ordinary characters are added to term
    if (nmea.term_offset < sizeof(nmea.term) - 1) {
        nmea.term[nmea.term_offset++] = c;
    }
    if (!nmea.term_is_checksum) {
        nmea.checksum ^= c;
    }

    return false;
}

// decode the most recently consumed term
// returns true if new term is valid
bool AP_ExternalAHRS_VectorNav::decode_latest_term()
{
    // Check the first two terms (In most cases header + reg number) that they match the sent
    // message. If not, the response is invalid.
    switch (nmea.term_number) {
        case 0:
            if (strncmp(nmea.term, "VNERR", nmea.term_offset) == 0) {
                nmea.error_response = true;  // Message will be printed on next term 
            } else if (strncmp(nmea.term, message_to_send, nmea.term_offset) != 0) {
                return false;
            }
            return true;
        case 1: 
            if (nmea.error_response) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "VectorNav received VNERR code: %s", nmea.term);
            } else if (strlen(message_to_send) > 6 && 
                       strncmp(nmea.term, &message_to_send[6], nmea.term_offset) != 0) {  // Start after "VNXXX,"
                return false;
            }
            return true;
        case 2: 
            if (strncmp(nmea.term, "VN-", 3) == 0) {
                // This term is the model number
                strncpy(model_name, nmea.term, sizeof(model_name));
            }
            return true;
        default:
            return true;
    }
}

void AP_ExternalAHRS_VectorNav::initialize() {
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    // Pause asynchronous communications to simplify packet finding
    run_command("VNASY,0");

    // Stop ASCII async outputs for both UARTs. If only active UART is disabled, we get a baudrate
    // overflow on the other UART when configuring binary outputs (reg 75 and 76) to both UARTs
    run_command("VNWRG,06,0,1");
    run_command("VNWRG,06,0,2");

    // Read Model Number Register, ID 1
    run_command("VNRRG,01");

    // Setup for messages respective model types (on both UARTs)
    if (strncmp(model_name, "VN-1", 4) == 0) {
        // VN-1X0
        type = TYPE::VN_AHRS;

        // These assumes unit is still configured at its default rate of 800hz
        run_command("VNWRG,75,3,%u,01,0721", unsigned(800 / get_rate()));
        run_command("VNWRG,76,3,16,11,0001,0106");
    } else {
        // Default to setup for sensors other than VN-100 or VN-110
        // This assumes unit is still configured at its default IMU rate of 400hz for VN-300, 800hz for others
        uint16_t imu_rate = 800;  // Default for everything but VN-300
        if (strncmp(model_name, "VN-300", 6) == 0) {
            imu_rate = 400;
        }
        if (strncmp(model_name, "VN-3", 4) == 0) {
            has_dual_gnss = true;
        }
        run_command("VNWRG,75,3,%u,01,0721", unsigned(imu_rate / get_rate()));
        run_command("VNWRG,76,3,%u,31,0001,0106,0613", unsigned(imu_rate / 50));
        run_command("VNWRG,77,3,%u,49,0003,26B8,0018", unsigned(imu_rate / 5));
    }

    // Resume asynchronous communications
    run_command("VNASY,1");
    setup_complete = true;
}

void AP_ExternalAHRS_VectorNav::update_thread() {
    initialize();
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

const char* AP_ExternalAHRS_VectorNav::get_name() const
{
    if (setup_complete) {
        return model_name;
    }
    return nullptr;
}

// Input data struct for EAHA logging message, used by both AHRS mode and INS mode
struct AP_ExternalAHRS_VectorNav::VNAT {
    uint64_t timeUs;
    float quat[4];
    float ypr[3];
    float yprU[3];
};


void AP_ExternalAHRS_VectorNav::write_vnat(const VNAT& data_to_log) const {

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: VNAT
    // @Description: VectorNav Attitude data
    // @Field: TimeUS: Time since system startup
    // @Field: Q1: Attitude quaternion 1
    // @Field: Q2: Attitude quaternion 2
    // @Field: Q3: Attitude quaternion 3
    // @Field: Q4: Attitude quaternion 4
    // @Field: Yaw: Yaw
    // @Field: Pitch: Pitch
    // @Field: Roll: Roll
    // @Field: YU: Yaw unceratainty
    // @Field: PU: Pitch uncertainty
    // @Field: RU: Roll uncertainty

    AP::logger().WriteStreaming("VNAT", "TimeUS,Q1,Q2,Q3,Q4,Yaw,Pitch,Roll,YU,PU,RU",
                       "s----dddddd", "F0000000000",
                       "Qffffffffff",
                       data_to_log.timeUs,
                       data_to_log.quat[0], data_to_log.quat[1], data_to_log.quat[2], data_to_log.quat[3],
                       data_to_log.ypr[0], data_to_log.ypr[1], data_to_log.ypr[2], 
                       data_to_log.yprU[0], data_to_log.yprU[1], data_to_log.yprU[2]);
#endif
}



// process INS mode INS packet
void AP_ExternalAHRS_VectorNav::process_imu_packet(const uint8_t *b)
{
    const struct VN_IMU_packet &pkt = *(struct VN_IMU_packet *)b;

    last_pkt1_ms = AP_HAL::millis();

    const bool use_uncomp = option_is_set(AP_ExternalAHRS::OPTIONS::VN_UNCOMP_IMU);

    {
        WITH_SEMAPHORE(state.sem);
        if (use_uncomp) {
            state.accel = Vector3f{pkt.uncompAccel[0], pkt.uncompAccel[1], pkt.uncompAccel[2]};
            state.gyro = Vector3f{pkt.uncompAngRate[0], pkt.uncompAngRate[1], pkt.uncompAngRate[2]};
        } else {
            state.accel = Vector3f{pkt.accel[0], pkt.accel[1], pkt.accel[2]};
            state.gyro = Vector3f{pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]};
        }
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pkt.pressure * 1e3;
        baro.temperature = pkt.temp;

        AP::baro().handle_external(baro);
    }
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = Vector3f{pkt.mag[0], pkt.mag[1], pkt.mag[2]};
        mag.field *= 1000; // to mGauss

        AP::compass().handle_external(mag);
    }
#endif

    {
        AP_ExternalAHRS::ins_data_message_t ins;

        ins.accel = state.accel;
        ins.gyro = state.gyro;
        ins.temperature = pkt.temp;

        AP::ins().handle_external(ins);
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: VNIM
    // @Description: VectorNav IMU data
    // @Field: TimeUS: Time since system startup
    // @Field: Temp: Temprature
    // @Field: Pres: Pressure
    // @Field: MX: Magnetic feild X-axis
    // @Field: MY: Magnetic feild Y-axis
    // @Field: MZ: Magnetic feild Z-axis
    // @Field: AX: Acceleration X-axis
    // @Field: AY: Acceleration Y-axis
    // @Field: AZ: Acceleration Z-axis
    // @Field: GX: Rotation rate X-axis
    // @Field: GY: Rotation rate Y-axis
    // @Field: GZ: Rotation rate Z-axis

    AP::logger().WriteStreaming("VNIM", "TimeUS,Temp,Pres,MX,MY,MZ,AX,AY,AZ,GX,GY,GZ",
                       "sdPGGGoooEEE", "F00000000000",
                       "Qfffffffffff",
                       AP_HAL::micros64(),
                       pkt.temp, pkt.pressure*1e3,
                       pkt.mag[0], pkt.mag[1], pkt.mag[2],
                       state.accel[0], state.accel[1], state.accel[2],
                       state.gyro[0], state.gyro[1], state.gyro[2],
                       state.quat[0], state.quat[1], state.quat[2], state.quat[3]);
#endif  // HAL_LOGGING_ENABLED
}

// process AHRS mode AHRS packet
void AP_ExternalAHRS_VectorNav::process_ahrs_ekf_packet(const uint8_t *b) {
    const struct VN_AHRS_ekf_packet &pkt = *(struct VN_AHRS_ekf_packet *)b;

    last_pkt2_ms = AP_HAL::millis();

    state.quat = Quaternion{pkt.quaternion[3], pkt.quaternion[0], pkt.quaternion[1], pkt.quaternion[2]};
    state.have_quaternion = true;

#if HAL_LOGGING_ENABLED
    VNAT data_to_log;
    data_to_log.timeUs = AP_HAL::micros64();
    memcpy(data_to_log.quat, pkt.quaternion, sizeof(pkt.quaternion));
    memcpy(data_to_log.ypr, pkt.ypr, sizeof(pkt.ypr));
    memcpy(data_to_log.yprU, pkt.yprU, sizeof(pkt.yprU));

    write_vnat(data_to_log);
#endif  // HAL_LOGGING_ENABLED
}

// process INS mode EKF packet
void AP_ExternalAHRS_VectorNav::process_ins_ekf_packet(const uint8_t *b) {
    const struct VN_INS_ekf_packet &pkt = *(struct VN_INS_ekf_packet *)b;

    last_pkt2_ms          = AP_HAL::millis();
    *latest_ins_ekf_packet = pkt;

    state.quat = Quaternion{pkt.quaternion[3], pkt.quaternion[0], pkt.quaternion[1], pkt.quaternion[2]};
    state.have_quaternion = true;

    state.velocity      = Vector3f{pkt.velNed[0], pkt.velNed[1], pkt.velNed[2]};
    state.have_velocity = true;

    state.location = Location{int32_t(pkt.posLla[0] * 1.0e7), int32_t(pkt.posLla[1] * 1.0e7), int32_t(pkt.posLla[2] * 1.0e2), Location::AltFrame::ABSOLUTE};
    state.last_location_update_us = AP_HAL::micros();
    state.have_location           = true;

#if HAL_LOGGING_ENABLED
    VNAT data_to_log;
    auto now =  AP_HAL::micros64();
    data_to_log.timeUs = now;
    memcpy(data_to_log.quat, pkt.quaternion, sizeof(pkt.quaternion));
    memcpy(data_to_log.ypr, pkt.ypr, sizeof(pkt.ypr));
    memcpy(data_to_log.yprU, pkt.yprU, sizeof(pkt.yprU));
    write_vnat(data_to_log);

    // @LoggerMessage: VNKF
    // @Description: VectorNav INS Kalman Filter data
    // @Field: TimeUS: Time since system startup
    // @Field: InsStatus: VectorNav INS health status
    // @Field: Lat: Latitude
    // @Field: Lon: Longitude
    // @Field: Alt: Altitude
    // @Field: VelN: Velocity Northing
    // @Field: VelE: Velocity Easting
    // @Field: VelD: Velocity Downing
    // @Field: PosU: Filter estimated position uncertainty
    // @Field: VelU: Filter estimated Velocity uncertainty

    AP::logger().WriteStreaming("VNKF", "TimeUS,InsStatus,Lat,Lon,Alt,VelN,VelE,VelD,PosU,VelU",
                       "s-ddmnnndn", "F000000000",
                       "QHdddfffff",
                       now,
                       pkt.insStatus,
                       pkt.posLla[0], pkt.posLla[1], pkt.posLla[2],
                       pkt.velNed[0], pkt.velNed[1], pkt.velNed[2],
                       pkt.posU, pkt.velU);
#endif  // HAL_LOGGING_ENABLED

}

// process INS mode GNSS packet
void AP_ExternalAHRS_VectorNav::process_ins_gnss_packet(const uint8_t *b) {
    const struct VN_INS_gnss_packet &pkt = *(struct VN_INS_gnss_packet *)b;
    AP_ExternalAHRS::gps_data_message_t gps;


    last_pkt3_ms          = AP_HAL::millis();
    *latest_ins_gnss_packet = pkt;
    
    // get ToW in milliseconds
    gps.gps_week           = pkt.timeGps / (AP_MSEC_PER_WEEK * 1000000ULL);
    gps.ms_tow             = (pkt.timeGps / 1000000ULL) % (60 * 60 * 24 * 7 * 1000ULL);
    gps.fix_type           = AP_GPS_FixType(pkt.fix1);
    gps.satellites_in_view = pkt.numSats1;

    gps.horizontal_pos_accuracy = pkt.posU1[0];
    gps.vertical_pos_accuracy   = pkt.posU1[2];
    gps.horizontal_vel_accuracy = pkt.velU1;

    gps.hdop = pkt.dop1[4];
    gps.vdop = pkt.dop1[3];

    gps.latitude     = pkt.posLla1[0] * 1.0e7;
    gps.longitude    = pkt.posLla1[1] * 1.0e7;
    gps.msl_altitude = pkt.posLla1[2] * 1.0e2;

    gps.ned_vel_north = pkt.velNed1[0];
    gps.ned_vel_east  = pkt.velNed1[1];
    gps.ned_vel_down  = pkt.velNed1[2];

    if (!state.have_origin && gps.fix_type >= AP_GPS_FixType::FIX_3D) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(pkt.posLla1[0] * 1.0e7), int32_t(pkt.posLla1[1] * 1.0e7),
                                int32_t(pkt.posLla1[2] * 1.0e2), Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }
    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_VectorNav::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_VectorNav::healthy(void) const
{
    const uint32_t now = AP_HAL::millis();
    return (now - last_pkt1_ms < 40) && (now - last_pkt2_ms < 500) && (type == TYPE::VN_AHRS ? true: now - last_pkt3_ms < 1000);
}

bool AP_ExternalAHRS_VectorNav::initialised(void) const
{
    if (!setup_complete) {
        return false;
    }
    return last_pkt1_ms != UINT32_MAX && last_pkt2_ms != UINT32_MAX && (type == TYPE::VN_AHRS ? true : last_pkt3_ms != UINT32_MAX);
}

bool AP_ExternalAHRS_VectorNav::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav unhealthy");
        return false;
    }
    if (type == TYPE::VN_INS) {
        if (latest_ins_gnss_packet->fix1 < 3) {
            hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS1 lock");
            return false;
        }
        if (has_dual_gnss && (latest_ins_gnss_packet->fix2 < 3)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS2 lock");
            return false;
        }
    }
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_VectorNav::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (healthy()) {
        if (type == TYPE::VN_AHRS) {
            status.flags.attitude = true;
        } else {
            status.flags.attitude = true;
            if (latest_ins_ekf_packet) {
                status.flags.vert_vel = true;
                status.flags.vert_pos = true;

                status.flags.horiz_vel          = true;
                status.flags.horiz_pos_rel      = true;
                status.flags.horiz_pos_abs      = true;
                status.flags.pred_horiz_pos_rel = true;
                status.flags.pred_horiz_pos_abs = true;
                status.flags.using_gps          = true;
            }
        }
    }
}

// get variances
bool AP_ExternalAHRS_VectorNav::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    const struct VN_INS_ekf_packet &pkt = *(struct VN_INS_ekf_packet *)latest_ins_ekf_packet;
    velVar = pkt.velU * vel_gate_scale;
    posVar = pkt.posU * pos_gate_scale;
    hgtVar = pkt.posU * hgt_gate_scale;
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_VECTORNAV_ENABLED
