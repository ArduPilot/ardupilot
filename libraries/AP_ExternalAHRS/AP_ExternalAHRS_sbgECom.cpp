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
    param set AHRS_EKF_TYPE 11
    param set EAHRS_TYPE 8
    param set GPS1_TYPE 21
    param set GPS2_TYPE 21
    param set SERIAL3_BAUD 115
    param set SERIAL3_PROTOCOL 36

 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBGECOM_ENABLED

#include "AP_ExternalAHRS_sbgECom.h"

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <GCS_MAVLink/GCS.h>

#include <version/sbgVersion.h>

extern const AP_HAL::HAL &hal;
uint32_t last_packet_received_time_ms;
uint32_t last_sending_time_ms;

// constructor
AP_ExternalAHRS_sbgECom::AP_ExternalAHRS_sbgECom(AP_ExternalAHRS *_frontend,
                                                 AP_ExternalAHRS::state_t &_state) : AP_ExternalAHRS_backend(_frontend, _state)
{
    hal.scheduler->delay(1000); // 1ms, decrease if necessary
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom ExternalAHRS UART found (%d - %" PRIu32 ")", port_num, baudrate);

    initialize();
    // Open port in the thread
    uart->begin(baudrate);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom ExternalAHRS initialised");
}

void AP_ExternalAHRS_sbgECom::initialize()
{
    hal.scheduler->delay(1000); // 1ms, decrease if necessary
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Initialize sbgECom");

    // Logging
    sbgCommonLibSetLogCallback(printLogCallBack);

    SbgErrorCode error_code;

    sbgInterfaceZeroInit(&_sbg_interface);

    // Initialise the interface
    _sbg_interface.handle = uart;
    _sbg_interface.type = SBG_IF_TYPE_SERIAL;
    _sbg_interface.pReadFunc = readCallback;
    _sbg_interface.pWriteFunc = writeCallback;

    error_code = sbgEComInit(&_com_handle, &_sbg_interface);

    if (error_code == SBG_NO_ERROR)
    {
        sbgEComSetReceiveLogCallback(&_com_handle, onLogReceived, this);
    }
    else
    {
        SBG_LOG_ERROR(error_code, "sbgECom initialization failed (%d)", error_code);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom initialized");

    setup_complete = true;
}

const char *AP_ExternalAHRS_sbgECom::get_name() const
{
    return "sbgECom";
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_sbgECom::get_port(void) const
{
    if (!uart)
    {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_sbgECom::healthy(void) const
{
    uint32_t current_time_ms = AP_HAL::millis();
    return ((current_time_ms - last_packet_received_time_ms) < 50);
}

bool AP_ExternalAHRS_sbgECom::initialised(void) const
{
    if (!setup_complete)
    {
        return false;
    }

    return true;
}

bool AP_ExternalAHRS_sbgECom::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "sbgECom setup failed");
        return false;
    }
    if (!healthy())
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "sbgECom unhealthy");
        return false;
    }

    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_sbgECom::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (healthy())
    {
        if (state.have_location)
        {
            status.flags.vert_pos = true;
            status.flags.horiz_pos_rel = true;
            status.flags.horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
            status.flags.pred_horiz_pos_abs = true;
            status.flags.using_gps = true;
        }

        if (state.have_quaternion)
        {
            status.flags.attitude = true;
        }

        if (state.have_velocity)
        {
            status.flags.vert_vel = true;
            status.flags.horiz_vel = true;
        }
    }
}

// get variances
bool AP_ExternalAHRS_sbgECom::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return false;
}

void AP_ExternalAHRS_sbgECom::update()
{
    uint32_t now_ms = AP_HAL::millis();

    check_uart();

    if (now_ms - last_sending_time_ms > 100)
    {
        send_uart();
        last_sending_time_ms = now_ms;
    }
}

// /*
//   check the UART for more data
//   returns true if the function should be called again straight away
//  */
// #define SYNC_BYTE1 0xFF
// #define SYNC_BYTE2 0x5A
bool AP_ExternalAHRS_sbgECom::check_uart()
{
    bool ret = false;

    if (!setup_complete)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom setup not complete");
        return ret;
    }

    SbgErrorCode error_code;
    SbgEComProtocolPayload payload;
    uint8_t received_msg;
    uint8_t received_msg_class;

    sbgEComProtocolPayloadConstruct(&payload);
    error_code = sbgEComProtocolReceive2(&_com_handle.protocolHandle, &received_msg_class, &received_msg, &payload);

    if (error_code == SBG_NO_ERROR)
    {
        if (sbgEComMsgClassIsALog((SbgEComClass)received_msg_class))
        {
            error_code = sbgEComLogParse((SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg,
                                         sbgEComProtocolPayloadGetBuffer(&payload), sbgEComProtocolPayloadGetSize(&payload), &_log_data);

            if (error_code == SBG_NO_ERROR)
            {
                if (_com_handle.pReceiveLogCallback)
                {
                    error_code = _com_handle.pReceiveLogCallback(&_com_handle, (SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg,
                                                                 &_log_data, _com_handle.pUserArg);
                }

                sbgEComLogCleanup(&_log_data, (SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg);
            }
        }
        else
        {
            SBG_LOG_ERROR(error_code, "command received %d", error_code);
        }
    }

    sbgEComProtocolPayloadDestroy(&payload);

    if (error_code == SBG_NO_ERROR)
    {
        ret = true;
    }

    return ret;
}

bool AP_ExternalAHRS_sbgECom::send_uart()
{
    bool ret = false;

    if (!setup_complete)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom setup not complete");
        return ret;
    }

    SbgErrorCode error_code;

    error_code = sendAirDataLog(&_com_handle);

    if (error_code == SBG_NO_ERROR)
    {
        error_code = sendMagDataLog(&_com_handle);
    }

    if (error_code == SBG_NO_ERROR)
    {
        ret = true;
    }

    return ret;
}

void AP_ExternalAHRS_sbgECom::printLogCallBack(const char *file_name, const char *function_name, uint32_t line, const char *category,
                                               SbgDebugLogType log_type, SbgErrorCode error_code, const char *message)
{
    const char *base_name;

    assert(file_name);
    assert(function_name);
    assert(category);
    assert(message);

    base_name = strrchr(file_name, '/');

    if (!base_name)
    {
        base_name = file_name;
    }
    else
    {
        base_name++;
    }

    switch (log_type)
    {
    case SBG_DEBUG_LOG_TYPE_DEBUG:
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s:%" PRIu32 ": %s: %s", base_name, line, function_name, message);
        break;

    case SBG_DEBUG_LOG_TYPE_INFO:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s:%" PRIu32 ": %s: %s", base_name, line, function_name, message);
        break;

    case SBG_DEBUG_LOG_TYPE_WARNING:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s:%" PRIu32 ": %s: warn:%s: %s", base_name, line, function_name, sbgErrorCodeToString(error_code), message);
        break;

    case SBG_DEBUG_LOG_TYPE_ERROR:
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s:%" PRIu32 ": %s: err:%s: %s", base_name, line, function_name, sbgErrorCodeToString(error_code), message);
        break;
    }
}

SbgErrorCode AP_ExternalAHRS_sbgECom::onLogReceived(SbgEComHandle *handle, SbgEComClass msg_class, SbgEComMsgId msg,
                                                    const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    SBG_UNUSED_PARAMETER(handle);

    assert(ref_sbg_data);
    assert(user_arg);

    // SBG_LOG_DEBUG("Log received (id %d)", msg);
    last_packet_received_time_ms = AP_HAL::millis();

    if (msg_class == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        switch (msg)
        {
        case SBG_ECOM_LOG_IMU_SHORT:
            process_imu_packet(ref_sbg_data, user_arg);
            break;

        case SBG_ECOM_LOG_MAG:
            process_mag_packet(ref_sbg_data, user_arg);
            break;

        case SBG_ECOM_LOG_GPS1_POS:
            process_gnss_pos_packet(ref_sbg_data, user_arg);
            break;

        case SBG_ECOM_LOG_GPS1_VEL:
            process_gnss_vel_packet(ref_sbg_data, user_arg);
            break;

        case SBG_ECOM_LOG_EKF_QUAT:
            process_ekf_quat_packet(ref_sbg_data, user_arg);
            break;

        case SBG_ECOM_LOG_EKF_NAV:
            process_ekf_nav_packet(ref_sbg_data, user_arg);
            break;

        default:
            SBG_LOG_INFO("Received unhandled SBG message (class %u, id %u)", msg_class, msg);
            break;
        }
    }

    return SBG_NO_ERROR;
}

// send a packet to the serial port
SbgErrorCode AP_ExternalAHRS_sbgECom::writeCallback(SbgInterface *p_interface, const void *p_buffer, size_t bytes_to_write)
{
    AP_HAL::UARTDriver *uart;

    assert(p_interface);

    uart = (AP_HAL::UARTDriver *)p_interface->handle;

    // send data if possible
    uart->write((const uint8_t *)p_buffer, bytes_to_write);

    return SBG_NO_ERROR;
}

SbgErrorCode AP_ExternalAHRS_sbgECom::readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read)
{
    AP_HAL::UARTDriver *uart;

    assert(p_interface);

    uart = (AP_HAL::UARTDriver *)p_interface->handle;

    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0)
    {
        return SBG_NOT_READY;
    }

    ssize_t nread = uart->read((uint8_t *)p_buffer, MIN(n, bytes_to_read));
    if (nread <= 0)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sbgECom Read Callback - nread<=0");
        return SBG_NOT_READY;
    }
    *p_read_bytes = nread;

    return SBG_NO_ERROR;
}

SbgErrorCode AP_ExternalAHRS_sbgECom::sendMagDataLog(SbgEComHandle *handle)
{
    SbgErrorCode error_code = SBG_NO_ERROR;
    SbgEComLogMag mag_data_log;
    uint8_t output_buffer[64];
    SbgStreamBuffer output_stream;

    memset(&mag_data_log, 0x00, sizeof(mag_data_log));

    mag_data_log.timeStamp = AP_HAL::micros();

    Compass *compass = Compass::get_singleton();
    Vector3f mag_field = compass->get_field();

    mag_data_log.magnetometers[0] = mag_field[0];
    mag_data_log.magnetometers[1] = mag_field[1];
    mag_data_log.magnetometers[2] = mag_field[2];
    mag_data_log.status |= SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAGS_IN_RANGE | SBG_ECOM_MAG_CALIBRATION_OK;

    sbgStreamBufferInitForWrite(&output_stream, output_buffer, sizeof(output_buffer));

    error_code = sbgEComLogMagWriteToStream(&mag_data_log, &output_stream);

    if (error_code == SBG_NO_ERROR)
    {
        error_code = sbgEComProtocolSend(&handle->protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG,
                                         sbgStreamBufferGetLinkedBuffer(&output_stream), sbgStreamBufferGetLength(&output_stream));

        if (error_code != SBG_NO_ERROR)
        {
            SBG_LOG_ERROR(error_code, "Unable to send the Mag log %d", error_code);
        }
    }
    else
    {
        SBG_LOG_ERROR(error_code, "Unable to write the Mag payload. %d", error_code);
    }

    return error_code;
}

SbgErrorCode AP_ExternalAHRS_sbgECom::sendAirDataLog(SbgEComHandle *handle)
{
    SbgErrorCode error_code = SBG_NO_ERROR;
    SbgEComLogAirData air_data_log;
    uint8_t output_buffer[64];
    SbgStreamBuffer output_stream;

    memset(&air_data_log, 0x00, sizeof(air_data_log));

    air_data_log.timeStamp = 0;
    air_data_log.status |= SBG_ECOM_AIR_DATA_TIME_IS_DELAY;

    AP_Baro *barometer = AP_Baro::get_singleton();

    if (barometer != nullptr && barometer->num_instances() != 0)
    {
        air_data_log.pressureAbs = barometer->get_pressure();
        air_data_log.status |= SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID;

        air_data_log.altitude = barometer->get_altitude();
        air_data_log.status |= SBG_ECOM_AIR_DATA_ALTITUDE_VALID;

        air_data_log.airTemperature = barometer->get_temperature();
        air_data_log.status |= SBG_ECOM_AIR_DATA_TEMPERATURE_VALID;

        AP_Airspeed *airspeed = AP_Airspeed::get_singleton();

        if (airspeed != nullptr)
        {
            float temp;
            if (airspeed->get_temperature(temp) == true)
            {
                air_data_log.airTemperature = temp;
            }

            air_data_log.pressureDiff = airspeed->get_differential_pressure();
            air_data_log.status |= SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID;

            air_data_log.trueAirspeed = airspeed->get_airspeed();
            air_data_log.status |= SBG_ECOM_AIR_DATA_AIRPSEED_VALID;
        }

        sbgStreamBufferInitForWrite(&output_stream, output_buffer, sizeof(output_buffer));

        error_code = sbgEComLogAirDataWriteToStream(&air_data_log, &output_stream);

        if (error_code == SBG_NO_ERROR)
        {
            error_code = sbgEComProtocolSend(&handle->protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA,
                                            sbgStreamBufferGetLinkedBuffer(&output_stream), sbgStreamBufferGetLength(&output_stream));

            if (error_code != SBG_NO_ERROR)
            {
                SBG_LOG_ERROR(error_code, "Unable to send the AirData log %d", error_code);
            }
        }
        else
        {
            SBG_LOG_ERROR(error_code, "Unable to write the AirData payload. %d", error_code);
        }
    }

    return error_code;
}

void AP_ExternalAHRS_sbgECom::process_gnss_vel_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    AP_ExternalAHRS_sbgECom *instance = static_cast<AP_ExternalAHRS_sbgECom *>(user_arg);

    instance->gnss_data.horizontal_vel_accuracy = 0.0f;

    instance->gnss_data.ned_vel_north = ref_sbg_data->gpsVelData.velocity[0];
    instance->gnss_data.ned_vel_east = ref_sbg_data->gpsVelData.velocity[1];
    instance->gnss_data.ned_vel_down = ref_sbg_data->gpsVelData.velocity[2];

    AP::gps().handle_external(instance->gnss_data, 0);
}

void AP_ExternalAHRS_sbgECom::process_gnss_pos_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    AP_ExternalAHRS_sbgECom *instance = static_cast<AP_ExternalAHRS_sbgECom *>(user_arg);

    instance->gnss_data.gps_week = ref_sbg_data->gpsPosData.timeOfWeek / 1000;
    instance->gnss_data.ms_tow = ref_sbg_data->gpsPosData.timeOfWeek - (instance->gnss_data.gps_week * 1000);

    SbgEComGnssPosType posType = sbgEComLogGnssPosGetType(&ref_sbg_data->gpsPosData);
    switch (posType)
    {
    case SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION:
        instance->gnss_data.fix_type = AP_GPS_FixType::NONE;
        break;
    case SBG_ECOM_GNSS_POS_TYPE_SINGLE:
    case SBG_ECOM_GNSS_POS_TYPE_FIXED:
        instance->gnss_data.fix_type = AP_GPS_FixType::FIX_3D;
        break;
    case SBG_ECOM_GNSS_POS_TYPE_PSRDIFF:
    case SBG_ECOM_GNSS_POS_TYPE_SBAS:
    case SBG_ECOM_GNSS_POS_TYPE_OMNISTAR:
    case SBG_ECOM_GNSS_POS_TYPE_PPP_FLOAT:
    case SBG_ECOM_GNSS_POS_TYPE_PPP_INT:
        instance->gnss_data.fix_type = AP_GPS_FixType::DGPS;
        break;
    case SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT:
        instance->gnss_data.fix_type = AP_GPS_FixType::RTK_FLOAT;
        break;
    case SBG_ECOM_GNSS_POS_TYPE_RTK_INT:
        instance->gnss_data.fix_type = AP_GPS_FixType::RTK_FIXED;
        break;
    case SBG_ECOM_GNSS_POS_TYPE_UNKNOWN:
        instance->gnss_data.fix_type = AP_GPS_FixType::NO_GPS;
    }

    instance->gnss_data.hdop = 0.0f;
    instance->gnss_data.vdop = 0.0f;

    instance->gnss_data.satellites_in_view = ref_sbg_data->gpsPosData.numSvUsed;

    instance->gnss_data.horizontal_pos_accuracy = sqrt(ref_sbg_data->gpsPosData.latitudeAccuracy * ref_sbg_data->gpsPosData.latitudeAccuracy + ref_sbg_data->gpsPosData.longitudeAccuracy * ref_sbg_data->gpsPosData.longitudeAccuracy);
    instance->gnss_data.vertical_pos_accuracy = ref_sbg_data->gpsPosData.altitudeAccuracy;

    instance->gnss_data.latitude = ref_sbg_data->gpsPosData.latitude * 1.0e7;
    instance->gnss_data.longitude = ref_sbg_data->gpsPosData.longitude * 1.0e7;
    instance->gnss_data.msl_altitude = ref_sbg_data->gpsPosData.altitude * 1.0e7;

    AP::gps().handle_external(instance->gnss_data, 0);
}

void AP_ExternalAHRS_sbgECom::process_ekf_nav_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    Location location;
    Vector3f velocity;
    AP_ExternalAHRS_sbgECom *instance = static_cast<AP_ExternalAHRS_sbgECom *>(user_arg);

    location.lat = ref_sbg_data->ekfNavData.position[0] * 1.0e7;
    location.lng = ref_sbg_data->ekfNavData.position[1] * 1.0e7;
    location.alt = ref_sbg_data->ekfNavData.position[2] * 1.0e7;

    velocity[0] = ref_sbg_data->ekfNavData.velocity[0];
    velocity[1] = ref_sbg_data->ekfNavData.velocity[1];
    velocity[2] = ref_sbg_data->ekfNavData.velocity[2];

    if (instance->state.have_origin == false)
    {
        instance->state.origin = location;
        instance->state.have_origin = true;
    }

    instance->state.location = location;
    instance->state.have_location = true;
    instance->state.last_location_update_us = AP_HAL::micros();

    instance->state.velocity = velocity;
    instance->state.have_velocity = true;

    // SBG_LOG_DEBUG("EKF Nav received - %f %f %f - %f %f %f",
    //              location.lat * 1.0e-7,
    //              location.lng * 1.0e-7,
    //              location.alt * 1.0e-7,
    //              velocity[0],
    //              velocity[1],
    //              velocity[2]);
}

void AP_ExternalAHRS_sbgECom::process_ekf_quat_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    Quaternion quat;
    AP_ExternalAHRS_sbgECom *instance = static_cast<AP_ExternalAHRS_sbgECom *>(user_arg);

    quat.q1 = ref_sbg_data->ekfQuatData.quaternion[0];
    quat.q2 = ref_sbg_data->ekfQuatData.quaternion[1];
    quat.q3 = ref_sbg_data->ekfQuatData.quaternion[2];
    quat.q4 = ref_sbg_data->ekfQuatData.quaternion[3];

    instance->state.quat = quat;
    instance->state.have_quaternion = true;

    // SBG_LOG_DEBUG("EKF Quaternion received - %f %f %f %f",
    //              quat.q1,
    //              quat.q2,
    //              quat.q3,
    //              quat.q4);
}

void AP_ExternalAHRS_sbgECom::process_mag_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    AP_ExternalAHRS::mag_data_message_t mag;

    mag.field[0] = ref_sbg_data->magData.magnetometers[0];
    mag.field[1] = ref_sbg_data->magData.magnetometers[1];
    mag.field[2] = ref_sbg_data->magData.magnetometers[2];

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP::compass().handle_external(mag);
#endif

    // SBG_LOG_DEBUG("Mag Data received - %f %f %f",
    //              mag.field[0],
    //              mag.field[1],
    //              mag.field[2]);
}

void AP_ExternalAHRS_sbgECom::process_imu_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
    assert(ref_sbg_data);
    assert(user_arg);

    AP_ExternalAHRS::ins_data_message_t ins;
    AP_ExternalAHRS_sbgECom *instance = static_cast<AP_ExternalAHRS_sbgECom *>(user_arg);

    ins.accel[0] = sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 0);
    ins.accel[1] = sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 1);
    ins.accel[2] = sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 2);

    ins.gyro[0] = sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 0);
    ins.gyro[1] = sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 1);
    ins.gyro[2] = sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 2);

    ins.temperature = sbgEComLogImuShortGetTemperature(&ref_sbg_data->imuShort);

    instance->state.accel = ins.accel;
    instance->state.gyro = ins.gyro;

    AP::ins().handle_external(ins);

    // SBG_LOG_DEBUG("IMU Short received - %f %f %f - %f %f %f - %f",
    //              ins.accel[0],
    //              ins.accel[1],
    //              ins.accel[2],
    //              ins.gyro[0],
    //              ins.gyro[1],
    //              ins.gyro[2],
    //              ins.temperature);
}

#endif // AP_EXTERNAL_AHRS_SBGECOM_ENABLED
