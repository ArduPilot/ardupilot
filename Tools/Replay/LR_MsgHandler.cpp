#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

extern const AP_HAL::HAL& hal;

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f,
                             AP_Logger &_dataflash,
                             uint64_t &_last_timestamp_usec) :
    dataflash(_dataflash), last_timestamp_usec(_last_timestamp_usec),
    MsgHandler(_f) {
}

void LR_MsgHandler::wait_timestamp_usec(uint64_t timestamp)
{
    last_timestamp_usec = timestamp;
    hal.scheduler->stop_clock(timestamp);
}

void LR_MsgHandler::wait_timestamp(uint32_t timestamp)
{
    uint64_t usecs = timestamp*1000UL;
    wait_timestamp_usec(usecs);
}

void LR_MsgHandler::wait_timestamp_from_msg(uint8_t *msg)
{
    uint64_t time_us;
    uint32_t time_ms;

    if (field_value(msg, "TimeUS", time_us)) {
        // 64-bit timestamp present - great!
        wait_timestamp_usec(time_us);
    } else if (field_value(msg, "TimeMS", time_ms)) {
        // there is special rounding code that needs to be crossed in
        // wait_timestamp:
        wait_timestamp(time_ms);
    } else {
        ::printf("No timestamp on message");
    }
}



/*
 * subclasses to handle specific messages below here
*/

void LR_MsgHandler_AHR2::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, ahr2_attitude, "Roll", "Pitch", "Yaw");
}


void LR_MsgHandler_ARM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    uint8_t ArmState = require_field_uint8_t(msg, "ArmState");
    hal.util->set_soft_armed(ArmState);
    printf("Armed state: %u at %lu\n", 
           (unsigned)ArmState,
           (unsigned long)AP_HAL::millis());
}


void LR_MsgHandler_ARSP::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    airspeed.setHIL(require_field_float(msg, "Airspeed"),
		    require_field_float(msg, "DiffPress"),
		    require_field_float(msg, "Temp"));
}

void LR_MsgHandler_NKF1::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
}


void LR_MsgHandler_ATT::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, attitude, "Roll", "Pitch", "Yaw");
}

void LR_MsgHandler_CHEK::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    check_state.time_us = AP_HAL::micros64();
    attitude_from_msg(msg, check_state.euler, "Roll", "Pitch", "Yaw");
    check_state.euler *= radians(1);
    location_from_msg(msg, check_state.pos, "Lat", "Lng", "Alt");
    require_field(msg, "VN", check_state.velocity.x);
    require_field(msg, "VE", check_state.velocity.y);
    require_field(msg, "VD", check_state.velocity.z);
}


void LR_MsgHandler_BARO::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    uint32_t last_update_ms;
    if (!field_value(msg, "SMS", last_update_ms)) {
        last_update_ms = 0;
    }
    AP::baro().setHIL(0,
		require_field_float(msg, "Press"),
		require_field_int16_t(msg, "Temp") * 0.01f,
		require_field_float(msg, "Alt"),
		require_field_float(msg, "CRt"),
                last_update_ms);
}


#define DATA_ARMED                          10
#define DATA_DISARMED                       11

void LR_MsgHandler_Event::process_message(uint8_t *msg)
{
    uint8_t id = require_field_uint8_t(msg, "Id");
    if (id == DATA_ARMED) {
        hal.util->set_soft_armed(true);
        printf("Armed at %lu\n", 
               (unsigned long)AP_HAL::millis());
    } else if (id == DATA_DISARMED) {
        hal.util->set_soft_armed(false);
        printf("Disarmed at %lu\n", 
               (unsigned long)AP_HAL::millis());
    }
}


void LR_MsgHandler_GPS2::process_message(uint8_t *msg)
{
    update_from_msg_gps(1, msg);
}

void LR_MsgHandler_GPS_Base::update_from_msg_gps(uint8_t gps_offset, uint8_t *msg)
{
    uint64_t time_us;
    if (! field_value(msg, "TimeUS", time_us)) {
        uint32_t timestamp;
        require_field(msg, "T", timestamp);
        time_us = timestamp * 1000;
    }
    wait_timestamp_usec(time_us);

    Location loc;
    location_from_msg(msg, loc, "Lat", "Lng", "Alt");
    Vector3f vel;
    ground_vel_from_msg(msg, vel, "Spd", "GCrs", "VZ");

    uint8_t status = require_field_uint8_t(msg, "Status");
    uint8_t hdop = 0;
    if (! field_value(msg, "HDop", hdop) &&
        ! field_value(msg, "HDp", hdop)) {
        hdop = 20;
    }
    uint8_t nsats = 0;
    if (! field_value(msg, "NSats", nsats) &&
        ! field_value(msg, "numSV", nsats)) {
        field_not_found(msg, "NSats");
    }
    uint16_t GWk;
    uint32_t GMS;
    if (! field_value(msg, "GWk", GWk)) {
        field_not_found(msg, "GWk");
    }
    if (! field_value(msg, "GMS", GMS)) {
        field_not_found(msg, "GMS");
    }
    gps.setHIL(gps_offset,
               (AP_GPS::GPS_Status)status,
               AP_GPS::time_epoch_convert(GWk, GMS),
               loc,
               vel,
               nsats,
               hdop);
    if (status == AP_GPS::GPS_OK_FIX_3D && ground_alt_cm == 0) {
        ground_alt_cm = require_field_int32_t(msg, "Alt");
    }
}



void LR_MsgHandler_GPS::process_message(uint8_t *msg)
{
    update_from_msg_gps(0, msg);
}


void LR_MsgHandler_GPA_Base::update_from_msg_gpa(uint8_t gps_offset, uint8_t *msg)
{
    uint64_t time_us;
    require_field(msg, "TimeUS", time_us);
    wait_timestamp_usec(time_us);

    uint16_t vdop, hacc, vacc, sacc;
    require_field(msg, "VDop", vdop);
    require_field(msg, "HAcc", hacc);
    require_field(msg, "VAcc", vacc);
    require_field(msg, "SAcc", sacc);
    uint8_t have_vertical_velocity;
    if (! field_value(msg, "VV", have_vertical_velocity)) {
        have_vertical_velocity = !is_zero(gps.velocity(gps_offset).z);
    }
    uint32_t sample_ms;
    if (! field_value(msg, "SMS", sample_ms)) {
        sample_ms = 0;
    }

    gps.setHIL_Accuracy(gps_offset, vdop*0.01f, hacc*0.01f, vacc*0.01f, sacc*0.01f, have_vertical_velocity, sample_ms);
}

void LR_MsgHandler_GPA::process_message(uint8_t *msg)
{
    update_from_msg_gpa(0, msg);
}


void LR_MsgHandler_GPA2::process_message(uint8_t *msg)
{
    update_from_msg_gpa(1, msg);
}



void LR_MsgHandler_IMU2::process_message(uint8_t *msg)
{
  update_from_msg_imu(1, msg);
}


void LR_MsgHandler_IMU3::process_message(uint8_t *msg)
{
  update_from_msg_imu(2, msg);
}


void LR_MsgHandler_IMU_Base::update_from_msg_imu(uint8_t imu_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    uint8_t this_imu_mask = 1 << imu_offset;

    if (gyro_mask & this_imu_mask) {
        Vector3f gyro;
        require_field(msg, "Gyr", gyro);
        ins.set_gyro(imu_offset, gyro);
    }
    if (accel_mask & this_imu_mask) {
        Vector3f accel2;
        require_field(msg, "Acc", accel2);
        ins.set_accel(imu_offset, accel2);
    }
}


void LR_MsgHandler_IMU::process_message(uint8_t *msg)
{
    update_from_msg_imu(0, msg);
}

void LR_MsgHandler_IMT_Base::update_from_msg_imt(uint8_t imu_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    if (!use_imt) {
        return;
    }

    uint8_t this_imu_mask = 1 << imu_offset;

    float delta_time = 0;
    require_field(msg, "DelT", delta_time);
    ins.set_delta_time(delta_time);

    if (gyro_mask & this_imu_mask) {
        Vector3f d_angle;
        require_field(msg, "DelA", d_angle);
        float d_angle_dt;
        if (!field_value(msg, "DelaT", d_angle_dt)) {
            d_angle_dt = 0;
        }
        ins.set_delta_angle(imu_offset, d_angle, d_angle_dt);
    }
    if (accel_mask & this_imu_mask) {
        float dvt = 0;
        require_field(msg, "DelvT", dvt);
        Vector3f d_velocity;
        require_field(msg, "DelV", d_velocity);
        ins.set_delta_velocity(imu_offset, dvt, d_velocity);
    }
}

void LR_MsgHandler_IMT::process_message(uint8_t *msg)
{
  update_from_msg_imt(0, msg);
}

void LR_MsgHandler_IMT2::process_message(uint8_t *msg)
{
  update_from_msg_imt(1, msg);
}

void LR_MsgHandler_IMT3::process_message(uint8_t *msg)
{
  update_from_msg_imt(2, msg);
}

void LR_MsgHandler_MAG2::process_message(uint8_t *msg)
{
    update_from_msg_compass(1, msg);
}


void LR_MsgHandler_MAG_Base::update_from_msg_compass(uint8_t compass_offset, uint8_t *msg)
{
    wait_timestamp_from_msg(msg);

    Vector3f mag;
    require_field(msg, "Mag", mag);
    Vector3f mag_offset;
    require_field(msg, "Ofs", mag_offset);
    uint32_t last_update_usec;
    if (!field_value(msg, "S", last_update_usec)) {
        last_update_usec = AP_HAL::micros();
    }

    compass.setHIL(compass_offset, mag - mag_offset, last_update_usec);
    // compass_offset is which compass we are setting info for;
    // mag_offset is a vector indicating the compass' calibration...
    compass.set_offsets(compass_offset, mag_offset);
}



void LR_MsgHandler_MAG::process_message(uint8_t *msg)
{
    update_from_msg_compass(0, msg);
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

void LR_MsgHandler_MSG::process_message(uint8_t *msg)
{
    const uint8_t msg_text_len = 64;
    char msg_text[msg_text_len];
    require_field(msg, "Message", msg_text, msg_text_len);

    if (strncmp(msg_text, "ArduPlane", strlen("ArduPlane")) == 0) {
	vehicle = VehicleType::VEHICLE_PLANE;
	::printf("Detected Plane\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
	ahrs.set_fly_forward(true);
    } else if (strncmp(msg_text, "ArduCopter", strlen("ArduCopter")) == 0 ||
	       strncmp(msg_text, "APM:Copter", strlen("APM:Copter")) == 0) {
	vehicle = VehicleType::VEHICLE_COPTER;
	::printf("Detected Copter\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
	ahrs.set_fly_forward(false);
    } else if (strncmp(msg_text, "ArduRover", strlen("ArduRover")) == 0) {
	vehicle = VehicleType::VEHICLE_ROVER;
	::printf("Detected Rover\n");
	ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
	ahrs.set_fly_forward(true);
    }
}


void LR_MsgHandler_NTUN_Copter::process_message(uint8_t *msg)
{
    inavpos = Vector3f(require_field_float(msg, "PosX") * 0.01f,
		       require_field_float(msg, "PosY") * 0.01f,
		       0);
}


bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = { "GPS_TYPE", "AHRS_EKF_TYPE", "EK2_ENABLE", "EK3_ENABLE"
                                   "COMPASS_ORIENT", "COMPASS_ORIENT2",
                                   "COMPASS_ORIENT3", "LOG_FILE_BUFSIZE",
                                   "LOG_DISARMED"};
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return _set_parameter_callback(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];
    uint64_t time_us;

    if (field_value(msg, "TimeUS", time_us)) {
        wait_timestamp_usec(time_us);
    } else {
        // older logs can have a lot of FMT and PARM messages up the
        // front which don't have timestamps.  Since in Replay we run
        // AP_Logger's IO only when stop_clock is called, we can
        // overflow AP_Logger's ringbuffer.  This should force us to
        // do IO:
        hal.scheduler->stop_clock(last_timestamp_usec);
    }

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    if (globals.no_params || replay.check_user_param(parameter_name)) {
        printf("Not changing %s to %f\n", parameter_name, value);
    } else {
        set_parameter(parameter_name, value);
    }
}

void LR_MsgHandler_PM::process_message(uint8_t *msg)
{
    uint32_t new_logdrop;
    if (field_value(msg, "LogDrop", new_logdrop) &&
        new_logdrop != 0) {
        printf("PM.LogDrop: %u dropped at timestamp %lu\n", new_logdrop, last_timestamp_usec);
    }
}

void LR_MsgHandler_SIM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, sim_attitude, "Roll", "Pitch", "Yaw");
}
