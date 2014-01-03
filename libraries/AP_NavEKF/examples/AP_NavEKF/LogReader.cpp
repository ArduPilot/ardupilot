#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Airspeed.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <DataFlash.h>

#include "LogReader.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

extern const AP_HAL::HAL& hal;

LogReader::LogReader(AP_InertialSensor &_ins, AP_Baro_HIL &_baro, AP_Compass_HIL &_compass, GPS *&_gps) :
    fd(-1),
    ins(_ins),
    baro(_baro),
    compass(_compass),
    gps(_gps)
{}

bool LogReader::open_log(const char *logfile)
{
    fd = ::open(logfile, O_RDONLY);
    if (fd == -1) {
        return false;
    }
    return true;
}


struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
};

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    uint32_t wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
    float   altitude;
    uint32_t groundspeed_cm;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

bool LogReader::update(uint8_t &type)
{
    uint8_t hdr[3];
    if (::read(fd, hdr, 3) != 3) {
        return false;
    }
    if (hdr[0] != HEAD_BYTE1 || hdr[1] != HEAD_BYTE2) {
        return false;
    }

    if (hdr[2] == LOG_FORMAT_MSG) {
        struct log_Format &f = formats[num_formats];
        memcpy(&f, hdr, 3);
        if (::read(fd, &f.type, sizeof(f)-3) != sizeof(f)-3) {
            return false;
        }
        num_formats++;
        type = f.type;
        return true;
    }

    uint8_t i;
    for (i=0; i<num_formats; i++) {
        if (formats[i].type == hdr[2]) break;
    }
    if (i == num_formats) {
        return false;
    }
    const struct log_Format &f = formats[i];
    
    uint8_t data[f.length];
    memcpy(data, hdr, 3);
    if (::read(fd, &data[3], f.length-3) != f.length-3) {
        return false;
    }

    switch (f.type) {
    case LOG_IMU_MSG: {
        struct log_IMU msg;
        memcpy(&msg, data, sizeof(msg));
        wait_timestamp(msg.timestamp);
        ins.set_gyro(Vector3f(msg.gyro_x, msg.gyro_y, msg.gyro_z));
        ins.set_accel(Vector3f(msg.accel_x, msg.accel_y, msg.accel_z));
        break;
    }

    case LOG_GPS_MSG: {
        struct log_GPS msg;
        memcpy(&msg, data, sizeof(msg));
        wait_timestamp(msg.apm_time);
        gps->setHIL(msg.apm_time,
                    msg.latitude*1.0e-7f, 
                    msg.longitude*1.0e-7f, 
                    msg.altitude*1.0e-2f,
                    msg.ground_speed*1.0e-2f, 
                    msg.ground_course*1.0e-2f, 
                    0, 
                    msg.status==3?msg.num_sats:0);
        if (msg.status == 3 && ground_alt_cm == 0) {
            ground_alt_cm = msg.altitude;
        }
        baro.setHIL((ground_alt_cm + msg.rel_altitude)*1.0e-2f);
        break;
    }

    case LOG_COMPASS_MSG: {
        struct log_Compass msg;
        memcpy(&msg, data, sizeof(msg));
        wait_timestamp(msg.time_ms);
        //compass.setHIL(Vector3i(msg.mag_x - msg.offset_x, msg.mag_y - msg.offset_y, msg.mag_z - msg.offset_z));
        compass.setHIL(Vector3i(msg.mag_x, msg.mag_y, msg.mag_z));
        break;
    }

    case LOG_ATTITUDE_MSG: {
        struct log_Attitude msg;
        memcpy(&msg, data, sizeof(msg));
        wait_timestamp(msg.time_ms);
        attitude = Vector3f(msg.roll*0.01f, msg.pitch*0.01f, msg.yaw*0.01f);
        break;
    }

    case LOG_SIMSTATE_MSG: {
        struct log_AHRS msg;
        memcpy(&msg, data, sizeof(msg));
        wait_timestamp(msg.time_ms);
        sim_attitude = Vector3f(msg.roll*0.01f, msg.pitch*0.01f, msg.yaw*0.01f);
        break;
    }
    }

    type = f.type;

    return true;
}

void LogReader::wait_timestamp(uint32_t timestamp)
{
    hal.scheduler->stop_clock(timestamp);
}

bool LogReader::wait_type(uint8_t wtype)
{
    while (true) {
        uint8_t type;
        if (!update(type)) {
            return false;
        }
        if (wtype == type) {
            break;
        }
    }
    return true;
}
