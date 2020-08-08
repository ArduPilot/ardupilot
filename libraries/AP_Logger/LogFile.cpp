#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_GPS/AP_GPS.h>

#include "AP_Logger.h"
#include "AP_Logger_File.h"
#include "AP_Logger_MAVLink.h"
#include "LoggerMessageWriter.h"

extern const AP_HAL::HAL& hal;


/*
  write a structure format to the log - should be in frontend
 */
void AP_Logger_Backend::Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = s->msg_type;
    pkt.length = s->msg_len;
    strncpy_noterm(pkt.name, s->name, sizeof(pkt.name));
    strncpy_noterm(pkt.format, s->format, sizeof(pkt.format));
    strncpy_noterm(pkt.labels, s->labels, sizeof(pkt.labels));
}

/*
  Pack a LogStructure packet into a structure suitable to go to the logfile:
 */
void AP_Logger_Backend::Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_UNITS_MSG;
    pkt.time_us = AP_HAL::micros64();
    pkt.format_type = s->msg_type;
    strncpy_noterm(pkt.units, s->units, sizeof(pkt.units));
    strncpy_noterm(pkt.multipliers, s->multipliers, sizeof(pkt.multipliers));
}

/*
  write a structure format to the log
 */
bool AP_Logger_Backend::Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Fill_Format(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit definition
 */
bool AP_Logger_Backend::Write_Unit(const struct UnitStructure *s)
{
    struct log_Unit pkt{
        LOG_PACKET_HEADER_INIT(LOG_UNIT_MSG),
        time_us : AP_HAL::micros64(),
        type    : s->ID,
        unit    : { }
    };
    strncpy_noterm(pkt.unit, s->unit, sizeof(pkt.unit));

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit-multiplier definition
 */
bool AP_Logger_Backend::Write_Multiplier(const struct MultiplierStructure *s)
{
    const struct log_Format_Multiplier pkt{
        LOG_PACKET_HEADER_INIT(LOG_MULT_MSG),
        time_us      : AP_HAL::micros64(),
        type         : s->ID,
        multiplier   : s->multiplier,
    };

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write the units for a format to the log
 */
bool AP_Logger_Backend::Write_Format_Units(const struct LogStructure *s)
{
    struct log_Format_Units pkt;
    Fill_Format_Units(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const char *name, float value)
{
    struct log_Parameter pkt{
        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
        time_us : AP_HAL::micros64(),
        name  : {},
        value : value
    };
    strncpy_noterm(pkt.name, name, sizeof(pkt.name));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const AP_Param *ap,
                                            const AP_Param::ParamToken &token,
                                            enum ap_var_type type)
{
    char name[16];
    ap->copy_name_token(token, &name[0], sizeof(name), true);
    return Write_Parameter(name, ap->cast_to_float(type));
}

// Write an GPS packet
void AP_Logger::Write_GPS(uint8_t i, uint64_t time_us)
{
    const AP_GPS &gps = AP::gps();
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const struct Location &loc = gps.location(i);

    float yaw_deg=0, yaw_accuracy_deg=0;
    gps.gps_yaw_deg(i, yaw_deg, yaw_accuracy_deg);

    const struct log_GPS pkt = {
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPS_MSG+i)),
        time_us       : time_us,
        status        : (uint8_t)gps.status(i),
        gps_week_ms   : gps.time_week_ms(i),
        gps_week      : gps.time_week(i),
        num_sats      : gps.num_sats(i),
        hdop          : gps.get_hdop(i),
        latitude      : loc.lat,
        longitude     : loc.lng,
        altitude      : loc.alt,
        ground_speed  : gps.ground_speed(i),
        ground_course : gps.ground_course(i),
        vel_z         : gps.velocity(i).z,
        yaw           : yaw_deg,
        used          : (uint8_t)(gps.primary_sensor() == i)
    };
    WriteBlock(&pkt, sizeof(pkt));

    /* write auxiliary accuracy information as well */
    float hacc = 0, vacc = 0, sacc = 0;
    gps.horizontal_accuracy(i, hacc);
    gps.vertical_accuracy(i, vacc);
    gps.speed_accuracy(i, sacc);
    struct log_GPA pkt2{
        LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GPA_MSG+i)),
        time_us       : time_us,
        vdop          : gps.get_vdop(i),
        hacc          : (uint16_t)MIN((hacc*100), UINT16_MAX),
        vacc          : (uint16_t)MIN((vacc*100), UINT16_MAX),
        sacc          : (uint16_t)MIN((sacc*100), UINT16_MAX),
        yaw_accuracy  : yaw_accuracy_deg,
        have_vv       : (uint8_t)gps.have_vertical_velocity(i),
        sample_ms     : gps.last_message_time_ms(i),
        delta_ms      : gps.last_message_delta_time_ms(i)
    };
    WriteBlock(&pkt2, sizeof(pkt2));
}


// Write an RCIN packet
void AP_Logger::Write_RCIN(void)
{
    uint16_t values[14] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));
    const struct log_RCIN pkt{
        LOG_PACKET_HEADER_INIT(LOG_RCIN_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : values[0],
        chan2         : values[1],
        chan3         : values[2],
        chan4         : values[3],
        chan5         : values[4],
        chan6         : values[5],
        chan7         : values[6],
        chan8         : values[7],
        chan9         : values[8],
        chan10        : values[9],
        chan11        : values[10],
        chan12        : values[11],
        chan13        : values[12],
        chan14        : values[13]
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an SERVO packet
void AP_Logger::Write_RCOUT(void)
{
    const struct log_RCOUT pkt{
        LOG_PACKET_HEADER_INIT(LOG_RCOUT_MSG),
        time_us       : AP_HAL::micros64(),
        chan1         : hal.rcout->read(0),
        chan2         : hal.rcout->read(1),
        chan3         : hal.rcout->read(2),
        chan4         : hal.rcout->read(3),
        chan5         : hal.rcout->read(4),
        chan6         : hal.rcout->read(5),
        chan7         : hal.rcout->read(6),
        chan8         : hal.rcout->read(7),
        chan9         : hal.rcout->read(8),
        chan10        : hal.rcout->read(9),
        chan11        : hal.rcout->read(10),
        chan12        : hal.rcout->read(11),
        chan13        : hal.rcout->read(12),
        chan14        : hal.rcout->read(13)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RSSI packet
void AP_Logger::Write_RSSI()
{
    AP_RSSI *rssi = AP::rssi();
    if (rssi == nullptr) {
        return;
    }

    const struct log_RSSI pkt{
        LOG_PACKET_HEADER_INIT(LOG_RSSI_MSG),
        time_us       : AP_HAL::micros64(),
        RXRSSI        : rssi->read_receiver_rssi()
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Baro_instance(uint64_t time_us, uint8_t baro_instance, enum LogMessages type)
{
    AP_Baro &baro = AP::baro();
    float climbrate = baro.get_climb_rate();
    float drift_offset = baro.get_baro_drift_offset();
    float ground_temp = baro.get_ground_temperature();
    const struct log_BARO pkt{
        LOG_PACKET_HEADER_INIT(type),
        time_us       : time_us,
        altitude      : baro.get_altitude(baro_instance),
        pressure      : baro.get_pressure(baro_instance),
        temperature   : (int16_t)(baro.get_temperature(baro_instance) * 100 + 0.5f),
        climbrate     : climbrate,
        sample_time_ms: baro.get_last_update(baro_instance),
        drift_offset  : drift_offset,
        ground_temp   : ground_temp,
        healthy       : (uint8_t)baro.healthy(baro_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
void AP_Logger::Write_Baro(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const AP_Baro &baro = AP::baro();
    Write_Baro_instance(time_us, 0, LOG_BARO_MSG);
    if (baro.num_instances() > 1 && baro.healthy(1)) {
        Write_Baro_instance(time_us, 1, LOG_BAR2_MSG);
    }
    if (baro.num_instances() > 2 && baro.healthy(2)) {
        Write_Baro_instance(time_us, 2, LOG_BAR3_MSG);
    }
}

void AP_Logger::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro(imu_instance);
    const Vector3f &accel = ins.get_accel(imu_instance);
    const struct log_IMU pkt{
        LOG_PACKET_HEADER_INIT(type),
        time_us : time_us,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z,
        gyro_error  : ins.get_gyro_error_count(imu_instance),
        accel_error : ins.get_accel_error_count(imu_instance),
        temperature : ins.get_temperature(imu_instance),
        gyro_health : (uint8_t)ins.get_gyro_health(imu_instance),
        accel_health : (uint8_t)ins.get_accel_health(imu_instance),
        gyro_rate : ins.get_gyro_rate_hz(imu_instance),
        accel_rate : ins.get_accel_rate_hz(imu_instance),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an raw accel/gyro data packet
void AP_Logger::Write_IMU()
{
    uint64_t time_us = AP_HAL::micros64();

    const AP_InertialSensor &ins = AP::ins();

    Write_IMU_instance(time_us, 0, LOG_IMU_MSG);
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }

    Write_IMU_instance(time_us, 1, LOG_IMU2_MSG);

    if (ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) {
        return;
    }

    Write_IMU_instance(time_us, 2, LOG_IMU3_MSG);
}

// Write an accel/gyro delta time data packet
void AP_Logger::Write_IMUDT_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    float delta_t = ins.get_delta_time();
    float delta_vel_t = ins.get_delta_velocity_dt(imu_instance);
    float delta_ang_t = ins.get_delta_angle_dt(imu_instance);
    Vector3f delta_angle, delta_velocity;
    ins.get_delta_angle(imu_instance, delta_angle);
    ins.get_delta_velocity(imu_instance, delta_velocity);

    const struct log_IMUDT pkt{
        LOG_PACKET_HEADER_INIT(type),
        time_us : time_us,
        delta_time   : delta_t,
        delta_vel_dt : delta_vel_t,
        delta_ang_dt : delta_ang_t,
        delta_ang_x  : delta_angle.x,
        delta_ang_y  : delta_angle.y,
        delta_ang_z  : delta_angle.z,
        delta_vel_x  : delta_velocity.x,
        delta_vel_y  : delta_velocity.y,
        delta_vel_z  : delta_velocity.z
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_IMUDT(uint64_t time_us, uint8_t imu_mask)
{
    const AP_InertialSensor &ins = AP::ins();
    if (imu_mask & 1) {
        Write_IMUDT_instance(time_us, 0, LOG_IMUDT_MSG);
    }
    if ((ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) || !ins.use_gyro(1)) {
        return;
    }

    if (imu_mask & 2) {
        Write_IMUDT_instance(time_us, 1, LOG_IMUDT2_MSG);
    }

    if ((ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) || !ins.use_gyro(2)) {
        return;
    }

    if (imu_mask & 4) {
        Write_IMUDT_instance(time_us, 2, LOG_IMUDT3_MSG);
    }
}

void AP_Logger::Write_Vibration()
{
    uint64_t time_us = AP_HAL::micros64();
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f vibration = ins.get_vibration_levels();
    const struct log_Vibe pkt{
        LOG_PACKET_HEADER_INIT(LOG_VIBE_MSG),
        time_us     : time_us,
        vibe_x      : vibration.x,
        vibe_y      : vibration.y,
        vibe_z      : vibration.z,
        clipping_0  : ins.get_accel_clip_count(0),
        clipping_1  : ins.get_accel_clip_count(1),
        clipping_2  : ins.get_accel_clip_count(2)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Command(const mavlink_command_int_t &packet,
                              const MAV_RESULT result,
                              bool was_command_long)
{
    const struct log_MAVLink_Command pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAVLINK_COMMAND_MSG),
        time_us         : AP_HAL::micros64(),
        target_system   : packet.target_system,
        target_component: packet.target_component,
        frame           : packet.frame,
        command         : packet.command,
        current         : packet.current,
        autocontinue    : packet.autocontinue,
        param1          : packet.param1,
        param2          : packet.param2,
        param3          : packet.param3,
        param4          : packet.param4,
        x               : packet.x,
        y               : packet.y,
        z               : packet.z,
        result          : (uint8_t)result,
        was_command_long:was_command_long,
    };
    return WriteBlock(&pkt, sizeof(pkt));
}

bool AP_Logger_Backend::Write_Mission_Cmd(const AP_Mission &mission,
                                              const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_int_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink_int(cmd,mav_cmd);
    const struct log_Cmd pkt{
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        time_us         : AP_HAL::micros64(),
        command_total   : mission.num_commands(),
        sequence        : mav_cmd.seq,
        command         : mav_cmd.command,
        param1          : mav_cmd.param1,
        param2          : mav_cmd.param2,
        param3          : mav_cmd.param3,
        param4          : mav_cmd.param4,
        latitude        : mav_cmd.x,
        longitude       : mav_cmd.y,
        altitude        : mav_cmd.z,
        frame           : mav_cmd.frame
    };
    return WriteBlock(&pkt, sizeof(pkt));
}

bool AP_Logger_Backend::Write_EntireMission()
{
    // kick off asynchronous write:
    return _startup_messagewriter->writeentiremission();
}

// Write a text message to the log
bool AP_Logger_Backend::Write_Message(const char *message)
{
    struct log_Message pkt{
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };
    strncpy_noterm(pkt.msg, message, sizeof(pkt.msg));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uint8_t safety_and_armed = uint8_t(hal.util->safety_switch_state());
    if (hal.util->get_soft_armed()) {
        // encode armed state in bit 3
        safety_and_armed |= 1U<<2;
    }
    const struct log_POWR pkt{
        LOG_PACKET_HEADER_INIT(LOG_POWR_MSG),
        time_us : AP_HAL::micros64(),
        Vcc     : hal.analogin->board_voltage(),
        Vservo  : hal.analogin->servorail_voltage(),
        flags   : hal.analogin->power_status_flags(),
        accumulated_flags   : hal.analogin->accumulated_power_status_flags(),
        safety_and_arm : safety_and_armed
    };
    WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write an AHRS2 packet
void AP_Logger::Write_AHRS2()
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f euler;
    struct Location loc;
    Quaternion quat;
    if (!ahrs.get_secondary_attitude(euler) || !ahrs.get_secondary_position(loc) || !ahrs.get_secondary_quaternion(quat)) {
        return;
    }
    const struct log_AHRS pkt{
        LOG_PACKET_HEADER_INIT(LOG_AHR2_MSG),
        time_us : AP_HAL::micros64(),
        roll  : (int16_t)(degrees(euler.x)*100),
        pitch : (int16_t)(degrees(euler.y)*100),
        yaw   : (uint16_t)(wrap_360_cd(degrees(euler.z)*100)),
        alt   : loc.alt*1.0e-2f,
        lat   : loc.lat,
        lng   : loc.lng,
        q1    : quat.q1,
        q2    : quat.q2,
        q3    : quat.q3,
        q4    : quat.q4,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a POS packet
void AP_Logger::Write_POS()
{
    const AP_AHRS &ahrs = AP::ahrs();

    Location loc;
    if (!ahrs.get_position(loc)) {
        return;
    }
    float home, origin;
    ahrs.get_relative_position_D_home(home);
    const struct log_POS pkt{
        LOG_PACKET_HEADER_INIT(LOG_POS_MSG),
        time_us        : AP_HAL::micros64(),
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt*1.0e-2f,
        rel_home_alt   : -home,
        rel_origin_alt : ahrs.get_relative_position_D_origin(origin) ? -origin : quiet_nanf(),
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Radio(const mavlink_radio_t &packet)
{
    const struct log_Radio pkt{
        LOG_PACKET_HEADER_INIT(LOG_RADIO_MSG),
        time_us      : AP_HAL::micros64(),
        rssi         : packet.rssi,
        remrssi      : packet.remrssi,
        txbuf        : packet.txbuf,
        noise        : packet.noise,
        remnoise     : packet.remnoise,
        rxerrors     : packet.rxerrors,
        fixed        : packet.fixed
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_CameraInfo(enum LogMessages msg, const Location &current_loc, uint64_t timestamp_us)
{
    const AP_AHRS &ahrs = AP::ahrs();

    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    const struct log_Camera pkt{
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : timestamp_us?timestamp_us:AP_HAL::micros64(),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_Camera(const Location &current_loc, uint64_t timestamp_us)
{
    Write_CameraInfo(LOG_CAMERA_MSG, current_loc, timestamp_us);
}

// Write a Trigger packet
void AP_Logger::Write_Trigger(const Location &current_loc)
{
    Write_CameraInfo(LOG_TRIGGER_MSG, current_loc, 0);
}

// Write an attitude packet
void AP_Logger::Write_Attitude(const Vector3f &targets)
{
    const AP_AHRS &ahrs = AP::ahrs();

    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(ahrs.yaw_sensor),
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void AP_Logger::Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets)
{
    const struct log_Attitude pkt{
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)wrap_360_cd(targets.z),
        yaw             : (uint16_t)wrap_360_cd(ahrs.yaw_sensor),
        error_rp        : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw       : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Current_instance(const uint64_t time_us,
                                       const uint8_t battery_instance)
{
    AP_BattMonitor &battery = AP::battery();
    float temp;
    bool has_temp = battery.get_temperature(temp, battery_instance);
    float current, consumed_mah, consumed_wh;
    if (!battery.current_amps(current, battery_instance)) {
        current = quiet_nanf();
    }
    if (!battery.consumed_mah(consumed_mah, battery_instance)) {
        consumed_mah = quiet_nanf();
    }
    if (!battery.consumed_wh(consumed_wh, battery_instance)) {
        consumed_wh = quiet_nanf();
    }

    const struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_us             : time_us,
        instance            : battery_instance,
        voltage             : battery.voltage(battery_instance),
        voltage_resting     : battery.voltage_resting_estimate(battery_instance),
        current_amps        : current,
        current_total       : consumed_mah,
        consumed_wh         : consumed_wh,
        temperature         : (int16_t)(has_temp ? (temp * 100) : 0),
        resistance          : battery.get_resistance(battery_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));

    // individual cell voltages
    if (battery.has_cell_voltages(battery_instance)) {
        const AP_BattMonitor::cells &cells = battery.get_cell_voltages(battery_instance);
        struct log_Current_Cells cell_pkt{
            LOG_PACKET_HEADER_INIT(LOG_CURRENT_CELLS_MSG),
            time_us             : time_us,
            instance            : battery_instance,
            voltage             : battery.voltage(battery_instance)
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(cells.cells); i++) {
            cell_pkt.cell_voltages[i] = cells.cells[i] + 1;
        }
        WriteBlock(&cell_pkt, sizeof(cell_pkt));

        // check battery structure can hold all cells
        static_assert(ARRAY_SIZE(cells.cells) == (sizeof(cell_pkt.cell_voltages) / sizeof(cell_pkt.cell_voltages[0])),
                      "Battery cell number doesn't match in library and log structure");
    }
}

// Write an Current data packet
void AP_Logger::Write_Current()
{
    const uint64_t time_us = AP_HAL::micros64();
    const uint8_t num_instances = AP::battery().num_instances();
    for (uint8_t i = 0; i < num_instances; i++) {
        Write_Current_instance(time_us, i);
    }
}

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance, const enum LogMessages type)
{
    const Compass &compass = AP::compass();

    const Vector3f &mag_field = compass.get_field(mag_instance);
    const Vector3f &mag_offsets = compass.get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(mag_instance);
    const struct log_Compass pkt{
        LOG_PACKET_HEADER_INIT(type),
        time_us         : time_us,
        mag_x           : (int16_t)mag_field.x,
        mag_y           : (int16_t)mag_field.y,
        mag_z           : (int16_t)mag_field.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z,
        health          : (uint8_t)compass.healthy(mag_instance),
        SUS             : compass.last_update_usec(mag_instance)
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Compass packet
void AP_Logger::Write_Compass(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const Compass &compass = AP::compass();
    if (compass.get_count() > 0) {
        Write_Compass_instance(time_us, 0, LOG_COMPASS_MSG);
    }

    if (compass.get_count() > 1) {
        Write_Compass_instance(time_us, 1, LOG_COMPASS2_MSG);
    }

    if (compass.get_count() > 2) {
        Write_Compass_instance(time_us, 2, LOG_COMPASS3_MSG);
    }
}

// Write a mode packet.
bool AP_Logger_Backend::Write_Mode(uint8_t mode, const ModeReason reason)
{
    static_assert(sizeof(ModeReason) <= sizeof(uint8_t), "Logging expects the ModeReason to fit in 8 bits");
    const struct log_Mode pkt{
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_us  : AP_HAL::micros64(),
        mode     : mode,
        mode_num : mode,
        mode_reason : static_cast<uint8_t>(reason)
    };
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write ESC status messages
//   id starts from 0
//   rpm is eRPM (rpm * 100)
//   voltage is in centi-volts
//   current is in centi-amps
//   temperature is in centi-degrees Celsius
//   current_tot is in centi-amp hours
void AP_Logger::Write_ESC(uint8_t instance, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t esc_temp, uint16_t current_tot, int16_t motor_temp)
{
    const struct log_Esc pkt{
        LOG_PACKET_HEADER_INIT(uint8_t(LOG_ESC_MSG)),
        time_us     : time_us,
        instance    : instance,
        rpm         : rpm,
        voltage     : voltage,
        current     : current,
        esc_temp    : esc_temp,
        current_tot : current_tot,
        motor_temp  : motor_temp
    };
    WriteBlock(&pkt, sizeof(pkt));
}

/*
  write servo status from CAN servo
 */
void AP_Logger::Write_ServoStatus(uint64_t time_us, uint8_t id, float position, float force, float speed, uint8_t power_pct)
{
    const struct log_CSRV pkt {
        LOG_PACKET_HEADER_INIT(LOG_CSRV_MSG),
        time_us     : time_us,
        id          : id,
        position    : position,
        force       : force,
        speed       : speed,
        power_pct   : power_pct
    };
    WriteBlock(&pkt, sizeof(pkt));
}

/*
  write ESC status from CAN ESC
 */
void AP_Logger::Write_ESCStatus(uint64_t time_us, uint8_t id, uint32_t error_count, float voltage, float current, float temperature, int32_t rpm, uint8_t power_pct)
{
    const struct log_CESC pkt {
        LOG_PACKET_HEADER_INIT(LOG_CESC_MSG),
        time_us     : time_us,
        id          : id,
        error_count : error_count,
        voltage     : voltage,
        current     : current,
        temperature : temperature,
        rpm         : rpm,
        power_pct   : power_pct
    };
    WriteBlock(&pkt, sizeof(pkt));
}


// Write a Yaw PID packet
void AP_Logger::Write_PID(uint8_t msg_type, const PID_Info &info)
{
    const struct log_PID pkt{
        LOG_PACKET_HEADER_INIT(msg_type),
        time_us         : AP_HAL::micros64(),
        target          : info.target,
        actual          : info.actual,
        error           : info.error,
        P               : info.P,
        I               : info.I,
        D               : info.D,
        FF              : info.FF,
        Dmod            : info.Dmod
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Origin(uint8_t origin_type, const Location &loc)
{
    const struct log_ORGN pkt{
        LOG_PACKET_HEADER_INIT(LOG_ORGN_MSG),
        time_us     : AP_HAL::micros64(),
        origin_type : origin_type,
        latitude    : loc.lat,
        longitude   : loc.lng,
        altitude    : loc.alt
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_RPM(const AP_RPM &rpm_sensor)
{
    float rpm1 = -1, rpm2 = -1;

    rpm_sensor.get_rpm(0, rpm1);
    rpm_sensor.get_rpm(1, rpm2);

    const struct log_RPM pkt{
        LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
        time_us     : AP_HAL::micros64(),
        rpm1        : rpm1,
        rpm2        : rpm2
    };
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void AP_Logger::Write_Rate(const AP_AHRS_View *ahrs,
                                     const AP_Motors &motors,
                                     const AC_AttitudeControl &attitude_control,
                                     const AC_PosControl &pos_control)
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target();
    const struct log_Rate pkt_rate{
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_us         : AP_HAL::micros64(),
        control_roll    : degrees(rate_targets.x),
        roll            : degrees(ahrs->get_gyro().x),
        roll_out        : motors.get_roll(),
        control_pitch   : degrees(rate_targets.y),
        pitch           : degrees(ahrs->get_gyro().y),
        pitch_out       : motors.get_pitch(),
        control_yaw     : degrees(rate_targets.z),
        yaw             : degrees(ahrs->get_gyro().z),
        yaw_out         : motors.get_yaw(),
        control_accel   : (float)accel_target.z,
        accel           : (float)(-(ahrs->get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f),
        accel_out       : motors.get_throttle()
    };
    WriteBlock(&pkt_rate, sizeof(pkt_rate));
}

// Write visual odometry sensor data
void AP_Logger::Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence)
{
    const struct log_VisualOdom pkt_visualodom{
        LOG_PACKET_HEADER_INIT(LOG_VISUALODOM_MSG),
        time_us             : AP_HAL::micros64(),
        time_delta          : time_delta,
        angle_delta_x       : angle_delta.x,
        angle_delta_y       : angle_delta.y,
        angle_delta_z       : angle_delta.z,
        position_delta_x    : position_delta.x,
        position_delta_y    : position_delta.y,
        position_delta_z    : position_delta.z,
        confidence          : confidence
    };
    WriteBlock(&pkt_visualodom, sizeof(log_VisualOdom));
}

// Write visual position sensor data.  x,y,z are in meters, angles are in degrees
void AP_Logger::Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float pos_err, float ang_err, uint8_t reset_counter)
{
    const struct log_VisualPosition pkt_visualpos {
        LOG_PACKET_HEADER_INIT(LOG_VISUALPOS_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        pos_x           : x,
        pos_y           : y,
        pos_z           : z,
        roll            : roll,
        pitch           : pitch,
        yaw             : yaw,
        pos_err         : pos_err,
        ang_err         : ang_err,
        reset_counter   : reset_counter
    };
    WriteBlock(&pkt_visualpos, sizeof(log_VisualPosition));
}

// Write visual velocity sensor data, velocity in NED meters per second
void AP_Logger::Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, float vel_err, uint8_t reset_counter)
{
    const struct log_VisualVelocity pkt_visualvel {
        LOG_PACKET_HEADER_INIT(LOG_VISUALVEL_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        vel_x           : vel.x,
        vel_y           : vel.y,
        vel_z           : vel.z,
        vel_err         : vel_err,
        reset_counter   : reset_counter
    };
    WriteBlock(&pkt_visualvel, sizeof(log_VisualVelocity));
}

// Write AOA and SSA
void AP_Logger::Write_AOA_SSA(AP_AHRS &ahrs)
{
    const struct log_AOA_SSA aoa_ssa{
        LOG_PACKET_HEADER_INIT(LOG_AOA_SSA_MSG),
        time_us         : AP_HAL::micros64(),
        AOA             : ahrs.getAOA(),
        SSA             : ahrs.getSSA()
    };

    WriteBlock(&aoa_ssa, sizeof(aoa_ssa));
}

// Write beacon sensor (position) data
void AP_Logger::Write_Beacon(AP_Beacon &beacon)
{
    if (!beacon.enabled()) {
        return;
    }
    // position
    Vector3f pos;
    float accuracy = 0.0f;
    beacon.get_vehicle_position_ned(pos, accuracy);

    const struct log_Beacon pkt_beacon{
       LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
       time_us         : AP_HAL::micros64(),
       health          : (uint8_t)beacon.healthy(),
       count           : (uint8_t)beacon.count(),
       dist0           : beacon.beacon_distance(0),
       dist1           : beacon.beacon_distance(1),
       dist2           : beacon.beacon_distance(2),
       dist3           : beacon.beacon_distance(3),
       posx            : pos.x,
       posy            : pos.y,
       posz            : pos.z
    };
    WriteBlock(&pkt_beacon, sizeof(pkt_beacon));
}

// Write proximity sensor distances
void AP_Logger::Write_Proximity(AP_Proximity &proximity)
{
    // exit immediately if not enabled
    if (proximity.get_status() == AP_Proximity::Status::NotConnected) {
        return;
    }

    AP_Proximity::Proximity_Distance_Array dist_array {};
    proximity.get_horizontal_distances(dist_array);

    float dist_up;
    if (!proximity.get_upward_distance(dist_up)) {
        dist_up = 0.0f;
    }

    float close_ang = 0.0f, close_dist = 0.0f;
    proximity.get_closest_object(close_ang, close_dist);

    const struct log_Proximity pkt_proximity{
            LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_MSG),
            time_us         : AP_HAL::micros64(),
            health          : (uint8_t)proximity.get_status(),
            dist0           : dist_array.distance[0],
            dist45          : dist_array.distance[1],
            dist90          : dist_array.distance[2],
            dist135         : dist_array.distance[3],
            dist180         : dist_array.distance[4],
            dist225         : dist_array.distance[5],
            dist270         : dist_array.distance[6],
            dist315         : dist_array.distance[7],
            distup          : dist_up,
            closest_angle   : close_ang,
            closest_dist    : close_dist
    };
    WriteBlock(&pkt_proximity, sizeof(pkt_proximity));
}

void AP_Logger::Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& breadcrumb)
{
    const struct log_SRTL pkt_srtl{
        LOG_PACKET_HEADER_INIT(LOG_SRTL_MSG),
        time_us         : AP_HAL::micros64(),
        active          : active,
        num_points      : num_points,
        max_points      : max_points,
        action          : action,
        N               : breadcrumb.x,
        E               : breadcrumb.y,
        D               : breadcrumb.z
    };
    WriteBlock(&pkt_srtl, sizeof(pkt_srtl));
}

void AP_Logger::Write_OABendyRuler(bool active, float target_yaw, bool resist_chg, float margin, const Location &final_dest, const Location &oa_dest)
{
    const struct log_OABendyRuler pkt{
        LOG_PACKET_HEADER_INIT(LOG_OA_BENDYRULER_MSG),
        time_us     : AP_HAL::micros64(),
        active      : active,
        target_yaw  : (uint16_t)wrap_360(target_yaw),
        yaw         : (uint16_t)wrap_360(AP::ahrs().yaw_sensor * 0.01f),
        resist_chg  : resist_chg,
        margin      : margin,
        final_lat   : final_dest.lat,
        final_lng   : final_dest.lng,
        oa_lat      : oa_dest.lat,
        oa_lng      : oa_dest.lng
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_OADijkstra(uint8_t state, uint8_t error_id, uint8_t curr_point, uint8_t tot_points, const Location &final_dest, const Location &oa_dest)
{
    struct log_OADijkstra pkt{
        LOG_PACKET_HEADER_INIT(LOG_OA_DIJKSTRA_MSG),
        time_us     : AP_HAL::micros64(),
        state       : state,
        error_id    : error_id,
        curr_point  : curr_point,
        tot_points  : tot_points,
        final_lat   : final_dest.lat,
        final_lng   : final_dest.lng,
        oa_lat      : oa_dest.lat,
        oa_lng      : oa_dest.lng
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_SimpleAvoidance(uint8_t state, const Vector2f& desired_vel, const Vector2f& modified_vel, bool back_up)
{
    struct log_SimpleAvoid pkt{
        LOG_PACKET_HEADER_INIT(LOG_SIMPLE_AVOID_MSG),
        time_us         : AP_HAL::micros64(),
        state           : state,
        desired_vel_x   : desired_vel.x * 0.01f,
        desired_vel_y   : desired_vel.y * 0.01f,
        modified_vel_x  : modified_vel.x * 0.01f,
        modified_vel_y  : modified_vel.y * 0.01f,
        backing_up      : back_up,
    };
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Winch(bool healthy, bool thread_end, bool moving, bool clutch, uint8_t mode, float desired_length, float length, float desired_rate, uint16_t tension, float voltage, int8_t temp)
{
    struct log_Winch pkt{
        LOG_PACKET_HEADER_INIT(LOG_WINCH_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : healthy,
        thread_end      : thread_end,
        moving          : moving,
        clutch          : clutch,
        mode            : mode,
        desired_length  : desired_length,
        length          : length,
        desired_rate    : desired_rate,
        tension         : tension,
        voltage         : voltage,
        temp            : temp
    };
    WriteBlock(&pkt, sizeof(pkt));
}
