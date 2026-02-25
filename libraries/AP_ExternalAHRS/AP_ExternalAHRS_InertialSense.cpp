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
  support for Inertial Sense INS
 */


#include <algorithm>
#include <cstdint>

#ifdef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#undef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#endif
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_InertialSense.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS_FixType.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include "data_sets.h"
#include "ISComm.h"

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_InertialSense *AP_ExternalAHRS_InertialSense::instance = nullptr;

AP_ExternalAHRS_InertialSense::AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
{
    printf("Inertial Sense ExternalAHRS created\r\n");
    hal.scheduler->delay(1000);

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = 921600; // sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    printf("Inertial Sense ExternalAHRS created: baudrate [%" PRIu32 "], port_num [%d]\n\n", baudrate, port_num);

    if (!uart) {
        printf("Inertial Sense ExternalAHRS no UART\r\n");
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Inertial Sense ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialSense::update_thread, void), "AHRS", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Inertial Sense failed to allocate ExternalAHRS update thread");
    }

    // don't offer IMU by default, the processing can take the main loop below minimum rate
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    hal.scheduler->delay(1000);
}

int8_t AP_ExternalAHRS_InertialSense::get_port(void) const {
    if (!uart) {
        return -1;
    }
    return port_num;
}

const char* AP_ExternalAHRS_InertialSense::get_name() const {
    // TODO: Return model name
    return "Inertial Sense";
}

bool AP_ExternalAHRS_InertialSense::healthy(void) const {
    uint32_t now = AP_HAL::millis();
    return _healthy && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::initialised(void) const {
    uint32_t now = AP_HAL::millis();
    return initialized && now - last_gps_pkt < 500 && now - last_filter_pkt < 100;
}

bool AP_ExternalAHRS_InertialSense::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense unhealthy\n");
        return false;
    }

    if(_fix_type < AP_GPS_FixType::FIX_3D) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Inertial Sense no GPS lock");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_InertialSense::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (initialised())
    {
        status.flags.initalized = true;
    }

    if (healthy())
    {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

bool AP_ExternalAHRS_InertialSense::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
    velVar = vel_cov * vel_gate_scale;
    posVar = pos_cov * pos_gate_scale;
    hgtVar = hgt_cov * hgt_gate_scale;
    tasVar = 0;
    return true;
}

int AP_ExternalAHRS_InertialSense::stop_message_broadcasting()
{
    printf("Inertial Sense ExternalAHRS stop_message_broadcasting\r\n");

    // Stop all broadcasts on the device
    // int ret = is_comm_stop_broadcasts_all_ports(port);
    int size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, NULL);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write stop broadcasts message\r\n");
        return -3;
    }

    // ret = is_comm_stop_broadcasts_current_port(port);
    size = is_comm_write_to_buf(buffer, sizeof(buffer), &comm, PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT, 0, 0, 0, NULL);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write stop broadcasts message\r\n");
        return -3;
    }

    return 0;
}

int AP_ExternalAHRS_InertialSense::enable_message_broadcasting()
{
    printf("Inertial Sense ExternalAHRS enable_message_broadcasting\r\n");

    int size;

    // Ask for INS message w/ update 8ms period (4ms source period x 2)
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INS_3, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get INS message\r\n");
        return -4;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_POS, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get GPS POS message\r\n");
        return -5;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_VEL, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get GPS VEL message\r\n");
        return -5;
    }

    // Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_GPS1_RTK_POS_MISC, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get RTK POS MISC message\r\n");
        return -5;
    }

    // Don't offer IMU by default, as it may drop the main loop below minimum rate
    // // Ask for IMU message at period of 20ms (1ms source period x 20).
    // size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_PIMU, 0, 0, imu_sample_duration);
    // {
    //     printf("Failed to encode and write get PIMU message\r\n");
    //     return -6;
    // }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_MAGNETOMETER, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get MAG message\r\n");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_BAROMETER, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get BARO message\r\n");
        return -6;
    }

    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_INL2_NED_SIGMA, 0, 0, 1);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get NL2_NED_SIGMA message\r\n");
        return -6;
    }

    // request a device info message
    size = is_comm_get_data_to_buf(buffer, sizeof(buffer), &comm, DID_DEV_INFO, 0, 0, 0);
    if(uart->write(buffer, size) != size) {
        printf("Failed to encode and write get BARO message\r\n");
        return -6;
    }

#ifdef LOG_PPD
    // Enable PPD data stream without disabling other messages
    rmc_t rmc;
    rmc.bits = RMC_PRESET_IMX_PPD;
    rmc.options = RMC_OPTIONS_PRESERVE_CTRL;
    if (is_comm_data(port, DID_RMC, sizeof(rmc_t), 0, (void*)&rmc) < sizeof(rmc_t)) {
        printf("Failed to encode and write get PPD message\r\n");
        return -5;
    }

    char *dir_path = "./isb_ppd";
    int ret = AP::FS().mkdir(dir_path);
    if (ret == -1) {
        if (errno == EEXIST) {
            printf("Directory - %s already exist\n", dir_path);
        } else {
            printf("Failed to create directory %s - %s\n", dir_path, strerror(errno));
        }
    }
#endif

    return 0;
}

int AP_ExternalAHRS_InertialSense::initialize()
{
    printf("Inertial Sense ExternalAHRS initialize\r\n");

    if (uart == nullptr) {
        return -1;
    }

    uart->begin(baudrate);

    is_comm_init(&comm, comm_buf, sizeof(comm_buf), NULL);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    instance = this;
    is_comm_register_isb_handler(&comm, &AP_ExternalAHRS_InertialSense::isbDataHandler);

#ifdef LOG_PPD
    char *file_path = "./isb_ppd/ppd.log";
    ppd_fd = AP::FS().open(file_path, O_WRONLY | O_CREAT | O_TRUNC);
    if (ppd_fd == -1) {
        printf("Open %s failed - %s\n", file_path, strerror(errno));
    }
#endif

    int error = 0;

    if ((error = stop_message_broadcasting()))
    {
        printf("Error: stop_message_broadcasting()\r\n");
        return error;
    }
    hal.scheduler->delay(500);

    if ((error = enable_message_broadcasting()))
    {
        printf("Error: enable_message_broadcasting\r\n");
        return error;
    }

    initialized = true;

    return 0;
}

void AP_ExternalAHRS_InertialSense::handleIns1Message(ins_1_t* ins)
{
    last_filter_pkt = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    Quaternion q;
    q.from_euler(ins->theta[0], ins->theta[1], ins->theta[2]);
    state.quat = q;
    state.have_quaternion = true;

    Vector3f uvw(ins->uvw[0], ins->uvw[1], ins->uvw[2]);
    state.velocity = q * uvw;
    state.have_velocity = true;

    state.location = Location{
        (int32_t)(ins->lla[0] * 1e7),
        (int32_t)(ins->lla[1] * 1e7),
        state.location.alt,
        Location::AltFrame::ABSOLUTE};
    state.have_location = true;
    state.last_location_update_us = AP_HAL::micros();

    switch((eGpsNavFixStatus)INS_STATUS_NAV_FIX_STATUS(ins->insStatus)) {
    case eGpsNavFixStatus::GPS_NAV_FIX_NONE:
        _fix_type = AP_GPS_FixType::NONE;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_3D:
        _fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FLOAT:
        _fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FIX:
        _fix_type = AP_GPS_FixType::RTK_FIXED;
        break;

    default:
        _fix_type = AP_GPS_FixType::NO_GPS;
    }

    if (_fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
    if((ins->hdwStatus & HDW_STATUS_ERROR_MASK) != HDW_STATUS_BIT_PASSED)
    {
        printf("Inertial Sense hardware failed!\n");
        _healthy = false;
    }
}

void AP_ExternalAHRS_InertialSense::handleIns2Message(ins_2_t* ins)
{
    last_filter_pkt = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    Quaternion q(ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3]);
    state.quat = q;
    state.have_quaternion = true;

    Vector3f uvw(ins->uvw[0], ins->uvw[1], ins->uvw[2]);
    state.velocity = q * uvw;
    state.have_velocity = true;

    state.location = Location{
        (int32_t)(ins->lla[0] * 1e7),
        (int32_t)(ins->lla[1] * 1e7),
        state.location.alt,
        Location::AltFrame::ABSOLUTE};
    state.have_location = true;
    state.last_location_update_us = AP_HAL::micros();

    switch((eGpsNavFixStatus)INS_STATUS_NAV_FIX_STATUS(ins->insStatus)) {
    case eGpsNavFixStatus::GPS_NAV_FIX_NONE:
        _fix_type = AP_GPS_FixType::NONE;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_3D:
        _fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FLOAT:
        _fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FIX:
        _fix_type = AP_GPS_FixType::RTK_FIXED;
        break;

    default:
        _fix_type = AP_GPS_FixType::NO_GPS;
    }

    if (_fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
    if((ins->hdwStatus & HDW_STATUS_ERROR_MASK) != HDW_STATUS_BIT_PASSED)
    {
        printf("Inertial Sense hardware failed!\n");
        _healthy = false;
    }
}


void AP_ExternalAHRS_InertialSense::handleIns3Message(ins_3_t* ins)
{
    last_filter_pkt = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    Quaternion q(ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3]);
    state.quat = q;
    state.have_quaternion = true;

    Vector3f uvw(ins->uvw[0], ins->uvw[1], ins->uvw[2]);
    state.velocity = q * uvw;
    state.have_velocity = true;

    state.location = Location{
        (int32_t)(ins->lla[0] * 1e7),
        (int32_t)(ins->lla[1] * 1e7),
        (int32_t)(ins->msl * 100),
        Location::AltFrame::ABSOLUTE};
    state.have_location = true;
    state.last_location_update_us = AP_HAL::micros();

    switch((eGpsNavFixStatus)INS_STATUS_NAV_FIX_STATUS(ins->insStatus)) {
    case eGpsNavFixStatus::GPS_NAV_FIX_NONE:
        _fix_type = AP_GPS_FixType::NONE;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_3D:
        _fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FLOAT:
        _fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case eGpsNavFixStatus::GPS_NAV_FIX_POSITIONING_RTK_FIX:
        _fix_type = AP_GPS_FixType::RTK_FIXED;
        break;

    default:
        _fix_type = AP_GPS_FixType::NO_GPS;
    }

    if (_fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
    if((ins->hdwStatus & HDW_STATUS_ERROR_MASK) != HDW_STATUS_BIT_PASSED)
    {
        printf("Inertial Sense hardware failed!\n");
        _healthy = false;
    }
}

void AP_ExternalAHRS_InertialSense::handleGpsPosMessage(gps_pos_t* pos)
{
    last_gps_pkt = AP_HAL::millis();

    AP_GPS_FixType fix_type = AP_GPS_FixType::NONE;

    int fix = pos->status & GPS_STATUS_FIX_MASK;
    switch(fix) {
    case GPS_STATUS_FIX_2D:
        fix_type = AP_GPS_FixType::FIX_2D;
        break;

    case GPS_STATUS_FIX_3D:
        fix_type = AP_GPS_FixType::FIX_3D;
        break;

    case GPS_STATUS_FIX_DGPS:
        fix_type = AP_GPS_FixType::DGPS;
        break;

    case GPS_STATUS_FIX_RTK_FLOAT:
        fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;

    case GPS_STATUS_FIX_RTK_FIX:
        fix_type = AP_GPS_FixType::RTK_FIXED;
        break;
    }

    int32_t latitude = (int32_t)(pos->lla[0] * 1e7);
    int32_t longitude = (int32_t)(pos->lla[1] * 1e7);
    int32_t msl_altitude = (int32_t)(pos->hMSL * 100);

    // TODO: combine the gps_sat_t, gps_pos_t, gps_vel_t messages.
    gps_data_msg.gps_week = (uint16_t)pos->week;
    gps_data_msg.ms_tow = pos->timeOfWeekMs;
    gps_data_msg.fix_type = fix_type;
    gps_data_msg.satellites_in_view = pos->satsUsed;
    gps_data_msg.horizontal_pos_accuracy = pos->hAcc;
    gps_data_msg.vertical_pos_accuracy = pos->vAcc;
    gps_data_msg.longitude = longitude;
    gps_data_msg.latitude = latitude;
    gps_data_msg.msl_altitude = msl_altitude;

    gps_data_msg.hdop = pos->hAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handleGpsVelMessage(gps_vel_t* vel)
{
    gps_data_msg.horizontal_vel_accuracy = vel->sAcc;
    gps_data_msg.ned_vel_north = vel->vel[0];
    gps_data_msg.ned_vel_east = vel->vel[1];
    gps_data_msg.ned_vel_down = vel->vel[2];

    if (gps_data_msg.fix_type >= AP_GPS_FixType::FIX_3D && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{
            gps_data_msg.latitude,
            gps_data_msg.longitude,
            gps_data_msg.msl_altitude,
            Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    uint8_t gps_instance;
    if (AP::gps().get_first_external_instance(gps_instance)) {
        AP::gps().handle_external(gps_data_msg, gps_instance);
    }

    gps_data_msg.vdop = vel->sAcc * 100;
}

void AP_ExternalAHRS_InertialSense::handleGpsRtkPosMiscMessage(gps_rtk_misc_t* misc)
{
    gps_data_msg.hdop = misc->hDop * 100;
    gps_data_msg.vdop = misc->vDop * 100;
}

void AP_ExternalAHRS_InertialSense::handlePimuMessage(pimu_t* pimu)
{
    last_imu_pkt = AP_HAL::millis();

    Vector3f accel;
    accel[0] = pimu->vel[0] / (pimu->dt * imu_sample_duration);
    accel[1] = pimu->vel[1] / (pimu->dt * imu_sample_duration);
    accel[2] = pimu->vel[2] / (pimu->dt * imu_sample_duration);

    Vector3f gyro;
    gyro[0] = pimu->theta[0] / (pimu->dt * imu_sample_duration);
    gyro[1] = pimu->theta[1] / (pimu->dt * imu_sample_duration);
    gyro[2] = pimu->theta[2] / (pimu->dt * imu_sample_duration);

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro;
    }

    AP_ExternalAHRS::ins_data_message_t ins {
        accel: accel,
        gyro: gyro,
        temperature: -300
    };
    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_InertialSense::handleMagnetometerMessage(magnetometer_t* _mag)
{
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f{_mag->mag[0], _mag->mag[1], _mag->mag[2]};

    // mag values have been normalized to 1, so we scale to something that can be calibrated

    // For SN510457, multiply DID_MAGNETOMETER by 0.4381111936 to scale back to the sensor raw output.
    mag.field *= 0.4381111936 * 1000;

    AP::compass().handle_external(mag);
#endif
}

void AP_ExternalAHRS_InertialSense::handleBarometerMessage(barometer_t* bar)
{
#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = bar->bar * 1e3;
    baro.temperature = bar->barTemp;

    AP::baro().handle_external(baro);
#endif
}

void AP_ExternalAHRS_InertialSense::handleInl2NedSigmaMessage(inl2_ned_sigma_t *sigmas)
{
    float pos_std = Vector3f(sigmas->StdPosNed[0], sigmas->StdPosNed[1], sigmas->StdPosNed[2]).length();
    float vel_std = Vector3f(sigmas->StdVelNed[0], sigmas->StdVelNed[1], sigmas->StdVelNed[2]).length();

    pos_cov = pos_std * pos_std;
    vel_cov = vel_std * vel_std;
    hgt_cov = pos_std * pos_std;
}

void AP_ExternalAHRS_InertialSense::handleDevInfoMessage(dev_info_t *dev_info)
{
    devInfoPopulateMissingHardware(dev_info);

    char *hardware_type;
    switch(dev_info->hardwareType) {
    case 1:
        hardware_type = "uINS";
        break;

    case 2:
        hardware_type = "EVB";
        break;

    case 3:
        hardware_type = "IMX";
        break;

    case 4:
        hardware_type = "GPX";
        break;

    default:
        hardware_type = "unknown";
    }

    printf("Inertial Sense:\n");
    printf("    manufacturer:       %s\n", dev_info->manufacturer);
    printf("    hardware type:      %s\n", hardware_type);
    printf("    hardware version:   %d.%d.%d.%d\n", dev_info->hardwareVer[0], dev_info->hardwareVer[1], dev_info->hardwareVer[2], dev_info->hardwareVer[3]);
    printf("    firmware version:   %d.%d.%d.%d\n", dev_info->firmwareVer[0], dev_info->firmwareVer[1], dev_info->firmwareVer[2], dev_info->firmwareVer[3]);
    printf("    serial number:      %" PRIu32 "\n", dev_info->serialNumber);
}

void AP_ExternalAHRS_InertialSense::handleBitMessage(bit_t* bit)
{
    if(bit->state != BIT_STATE_DONE)
        return;

    if(bit->hdwBitStatus & HDW_BIT_FAILED_MASK) {
        printf("Inertial Sense hardware built-in-test failed!\n");
        return;
    }

    if(bit->calBitStatus & CAL_BIT_FAILED_MASK) {
        printf("Inertial Sense calibration built-in-test failed!\n");
        return;
    }

    WITH_SEMAPHORE(sem);
    _healthy = true;
}

void AP_ExternalAHRS_InertialSense::update() {
    if (!check_uart()) {
        hal.scheduler->delay_microseconds(100);
    }
}

int AP_ExternalAHRS_InertialSense::parseIsbData(void* ctx, p_data_t* data, port_handle_t port) {
    switch (data->hdr.id)
    {
    case DID_INS_3:
        handleIns3Message((ins_3_t*)data->ptr);
        break;

    case DID_GPS1_POS:
        handleGpsPosMessage((gps_pos_t*)data->ptr);
        break;

    case DID_GPS1_VEL:
        handleGpsVelMessage((gps_vel_t*)data->ptr);
        break;

    case DID_MAGNETOMETER:
        handleMagnetometerMessage((magnetometer_t *)data->ptr);
        break;

    case DID_INL2_NED_SIGMA:
        handleInl2NedSigmaMessage((inl2_ned_sigma_t *)data->ptr);
        break;

    case DID_DEV_INFO:
        handleDevInfoMessage((dev_info_t*)data->ptr);
        break;

    case DID_BIT:
        handleBitMessage((bit_t*)data->ptr);
        break;

    default:
        break;
    }

    return 0;
}

bool AP_ExternalAHRS_InertialSense::check_uart() {
    if(!initialized) {
        printf("UART not initialized!\n");
        return false;
    }

    WITH_SEMAPHORE(sem);

    if(!uart->available())
        return false;

    auto len = uart->read(buffer, MIN(uart->available(), 1024u));
    is_comm_buffer_parse_messages(buffer, len, &comm);

#ifdef LOG_PPD
    AP::FS().write(ppd_fd, buffer, len);
    AP::FS().fsync(ppd_fd);
#endif

    return true;
}

void AP_ExternalAHRS_InertialSense::update_thread() {
    if(!initialized) {
        initialize();
    }

    while(true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

uint8_t AP_ExternalAHRS_InertialSense::num_gps_sensors(void) const {
    // TODO: query actual number of GPS units on module
    return 1;
}

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
