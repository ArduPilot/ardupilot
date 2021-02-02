#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
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
void AP_Logger::Write_GPS(uint8_t i)
{
    const AP_GPS &gps = AP::gps();
    const uint64_t time_us = AP_HAL::micros64();
    const struct Location &loc = gps.location(i);

    float yaw_deg=0, yaw_accuracy_deg=0;
    gps.gps_yaw_deg(i, yaw_deg, yaw_accuracy_deg);

    const struct log_GPS pkt {
        LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
        time_us       : time_us,
        instance      : i,
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
        LOG_PACKET_HEADER_INIT(LOG_GPA_MSG),
        time_us       : time_us,
        instance      : i,
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
    uint16_t values[16] = {};
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

    // don't waste logging bandwidth if we haven't seen non-zero
    // channels 15/16:
    if (!seen_nonzero_rcin15_or_rcin16) {
        if (!values[14] && !values[15]) {
            return;
        }
        seen_nonzero_rcin15_or_rcin16 = true;
    }

    const struct log_RCIN2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_RCIN2_MSG),
        time_us       : AP_HAL::micros64(),
        chan15         : values[14],
        chan16         : values[15]
    };
    WriteBlock(&pkt2, sizeof(pkt2));
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

void AP_Logger::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance)
{
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro(imu_instance);
    const Vector3f &accel = ins.get_accel(imu_instance);
    const struct log_IMU pkt{
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        time_us : time_us,
        instance: imu_instance,
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
    const uint64_t time_us = AP_HAL::micros64();

    const AP_InertialSensor &ins = AP::ins();

    uint8_t n = MAX(ins.get_accel_count(), ins.get_gyro_count());
    for (uint8_t i=0; i<n; i++) {
        Write_IMU_instance(time_us, i);
    }
}

void AP_Logger::Write_Vibration()
{
    const AP_InertialSensor &ins = AP::ins();
    const uint64_t time_us = AP_HAL::micros64();
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        if (!ins.use_accel(i)) {
            continue;
        }

        const Vector3f vibration = ins.get_vibration_levels(i);
        const struct log_Vibe pkt{
            LOG_PACKET_HEADER_INIT(LOG_VIBE_MSG),
            time_us     : time_us,
            imu         : i,
            vibe_x      : vibration.x,
            vibe_y      : vibration.y,
            vibe_z      : vibration.z,
            clipping  : ins.get_accel_clip_count(i)
        };
        WriteBlock(&pkt, sizeof(pkt));
    }
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

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance)
{
    const Compass &compass = AP::compass();

    const Vector3f &mag_field = compass.get_field(mag_instance);
    const Vector3f &mag_offsets = compass.get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(mag_instance);
    const struct log_MAG pkt{
        LOG_PACKET_HEADER_INIT(LOG_MAG_MSG),
        time_us         : time_us,
        instance        : mag_instance,
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
void AP_Logger::Write_Compass()
{
    const uint64_t time_us = AP_HAL::micros64();
    const Compass &compass = AP::compass();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        Write_Compass_instance(time_us, i);
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
void AP_Logger::Write_ESC(uint8_t instance, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t esc_temp, uint16_t current_tot, int16_t motor_temp, float error_rate)
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
        motor_temp  : motor_temp,
        error_rate  : error_rate
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
        Dmod            : info.Dmod,
        limit           : info.limit
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
void AP_Logger::Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float pos_err, float ang_err, uint8_t reset_counter, bool ignored)
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
        reset_counter   : reset_counter,
        ignored         : (uint8_t)ignored
    };
    WriteBlock(&pkt_visualpos, sizeof(log_VisualPosition));
}

// Write visual velocity sensor data, velocity in NED meters per second
void AP_Logger::Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, float vel_err, uint8_t reset_counter, bool ignored)
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
        reset_counter   : reset_counter,
        ignored         : (uint8_t)ignored
    };
    WriteBlock(&pkt_visualvel, sizeof(log_VisualVelocity));
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

void AP_Logger::Write_OABendyRuler(uint8_t type, bool active, float target_yaw, float target_pitch, bool resist_chg, float margin, const Location &final_dest, const Location &oa_dest)
{
    const struct log_OABendyRuler pkt{
        LOG_PACKET_HEADER_INIT(LOG_OA_BENDYRULER_MSG),
        time_us     : AP_HAL::micros64(),
        type        : type,
        active      : active,
        target_yaw  : (uint16_t)wrap_360(target_yaw),
        yaw         : (uint16_t)wrap_360(AP::ahrs().yaw_sensor * 0.01f),
        target_pitch: (uint16_t)target_pitch,
        resist_chg  : resist_chg,
        margin      : margin,
        final_lat   : final_dest.lat,
        final_lng   : final_dest.lng,
        final_alt   : final_dest.alt,
        oa_lat      : oa_dest.lat,
        oa_lng      : oa_dest.lng,
        oa_alt      : oa_dest.alt
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

void AP_Logger::Write_PSC(const Vector3f &pos_target, const Vector3f &position, const Vector3f &vel_target, const Vector3f &velocity, const Vector3f &accel_target, const float &accel_x, const float &accel_y)
{
    struct log_PSC pkt{
        LOG_PACKET_HEADER_INIT(LOG_PSC_MSG),
        time_us         : AP_HAL::micros64(),
        pos_target_x    : pos_target.x * 0.01f,
        pos_target_Y    : pos_target.y * 0.01f,
        position_x      : position.x * 0.01f,
        position_y      : position.y * 0.01f,
        vel_target_x    : vel_target.x * 0.01f,
        vel_target_y    : vel_target.y * 0.01f,
        velocity_x      : velocity.x * 0.01f,
        velocity_y      : velocity.y * 0.01f,
        accel_target_x  : accel_target.x * 0.01f,
        accel_target_y  : accel_target.y * 0.01f,
        accel_x         : accel_x * 0.01f,
        accel_y         : accel_y * 0.01f
    };
    WriteBlock(&pkt, sizeof(pkt));
}
